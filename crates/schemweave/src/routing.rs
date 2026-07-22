use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, HashMap, HashSet, hash_map::Entry},
};

use crate::{
    Edge, EdgeGeometry, EdgeId, Endpoint, LayoutOptions, NetId, NodeGeometry, Point, Port,
    PortSide, validation::IndexedGraph,
};

const MAX_SPARSE_NET_EDGES: usize = 300;
const CROSSING_TRACK_NUDGE: f64 = 1e-4;
const CROSSING_ALIGNMENT_WEIGHT: f64 = 4.0;
const MIN_ROUTE_SEGMENT: f64 = 1e-7;
const FULL_OUTER_LANE_ROUNDS: usize = 16;
const FULL_GAP_LANE_ROUNDS: usize = 32;
const SUPPLEMENTAL_OUTER_LANE_ROUNDS: usize = 4;
const SUPPLEMENTAL_GAP_LANE_ROUNDS: usize = 8;
const MIN_CROSSING_REPAIR_TOTAL: usize = 500;
const MIN_CROSSING_REPAIR_NET: usize = 64;
const MAX_CROSSING_REPAIR_EDGES: usize = 10_000;
const MAX_CROSSING_REPAIR_NODES: usize = 2_000;
const MAX_CROSSING_REPAIR_ROUTE_POINTS: usize = 100_000;
const MAX_CROSSING_REPAIR_LANE_MEMBERSHIPS: usize = 100_000;
const MAX_CROSSING_REPAIR_PATH_STATES: usize = 500_000;
// Small nets keep the historical stable-ID order; at this fanout the many target branches make
// target-proximal channel placement materially more important than the single shared source arm.
const MIN_FANOUT_AWARE_CHANNEL_EDGES: usize = 16;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct RouteQuality {
    pub(crate) crossings: usize,
    pub(crate) bends: usize,
    pub(crate) route_length: f64,
}

pub(crate) struct RoutedEdges {
    pub(crate) primary: Vec<EdgeGeometry>,
    pub(crate) primary_quality: Option<RouteQuality>,
    pub(crate) repair: Option<(RouteQuality, Vec<EdgeGeometry>)>,
    #[cfg(test)]
    pub(crate) feedback_trace: FeedbackCandidateTrace,
}

#[cfg(test)]
#[derive(Debug)]
pub(crate) struct FeedbackCandidateTrace {
    pub(crate) split: bool,
    pub(crate) evaluated: bool,
    pub(crate) selected: bool,
    pub(crate) baseline: Option<(RouteQuality, Vec<EdgeGeometry>)>,
    pub(crate) candidate_quality: Option<RouteQuality>,
}

struct RoutedLaneState {
    routes: Vec<EdgeGeometry>,
    gap_lanes: Vec<BTreeMap<u32, usize>>,
    crossing_paths: Vec<Option<Vec<f64>>>,
}

#[derive(Clone, Copy)]
struct PhysicalSegment {
    net: u32,
    source: Endpoint,
    target: Endpoint,
    horizontal: bool,
    fixed: f64,
    start: f64,
    end: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct FloatKey(f64);

impl Eq for FloatKey {}

impl Ord for FloatKey {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl PartialOrd for FloatKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Clone, Copy)]
struct RouteEdge<'a> {
    edge: &'a Edge,
    participates_in_ranking: bool,
    source_index: usize,
    target_index: usize,
    source_port: &'a Port,
    target_port: &'a Port,
}

pub(crate) struct RoutingPlan<'a> {
    edges: Vec<RouteEdge<'a>>,
    net_edge_counts: BTreeMap<NetId, usize>,
    nodes_by_rank: Vec<Vec<usize>>,
    ranks: Vec<usize>,
    shared_endpoints: HashSet<Endpoint>,
}

impl<'a> RoutingPlan<'a> {
    pub(crate) fn new(graph: &IndexedGraph<'a>, ranks: &[usize]) -> Self {
        let edges = graph
            .edges
            .iter()
            .zip(&graph.rank_edges)
            .map(|(&edge, &participates_in_ranking)| {
                let source_index = graph.node_index[&edge.source.node];
                let target_index = graph.node_index[&edge.target.node];
                RouteEdge {
                    edge,
                    participates_in_ranking,
                    source_index,
                    target_index,
                    source_port: graph.ports[source_index][&edge.source.port],
                    target_port: graph.ports[target_index][&edge.target.port],
                }
            })
            .collect::<Vec<_>>();
        let mut net_edge_counts = BTreeMap::new();
        for resolved in &edges {
            *net_edge_counts.entry(resolved.edge.net).or_insert(0) += 1;
        }
        let mut nodes_by_rank = vec![Vec::new(); ranks.iter().copied().max().unwrap_or(0) + 1];
        for (node, &rank) in ranks.iter().enumerate() {
            nodes_by_rank[rank].push(node);
        }
        Self {
            shared_endpoints: shared_endpoints(edges.iter().map(|edge| edge.edge)),
            edges,
            net_edge_counts,
            nodes_by_rank,
            ranks: ranks.to_vec(),
        }
    }
}

#[cfg(test)]
pub(crate) fn route_edges(
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    options: LayoutOptions,
) -> Vec<EdgeGeometry> {
    let plan = RoutingPlan::new(graph, ranks);
    route_edges_with_lane_rounds(
        &plan,
        nodes,
        options,
        FULL_OUTER_LANE_ROUNDS,
        FULL_GAP_LANE_ROUNDS,
        false,
    )
    .primary
}

/// Route an optional layout candidate with bounded lane-refinement work.
///
/// The same deterministic router and validity-preserving construction are used; only the
/// adjacent-transposition search for a better lane order stops earlier. The canonical full-effort
/// candidate remains available to the exact layout comparator.
#[cfg(test)]
pub(crate) fn route_supplemental_edges(
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    options: LayoutOptions,
) -> Vec<EdgeGeometry> {
    let plan = RoutingPlan::new(graph, ranks);
    route_edges_with_lane_rounds(
        &plan,
        nodes,
        options,
        SUPPLEMENTAL_OUTER_LANE_ROUNDS,
        SUPPLEMENTAL_GAP_LANE_ROUNDS,
        true,
    )
    .primary
}

pub(crate) fn route_planned_candidates(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    supplemental: bool,
) -> RoutedEdges {
    let (outer_rounds, gap_rounds) = if supplemental {
        (SUPPLEMENTAL_OUTER_LANE_ROUNDS, SUPPLEMENTAL_GAP_LANE_ROUNDS)
    } else {
        (FULL_OUTER_LANE_ROUNDS, FULL_GAP_LANE_ROUNDS)
    };
    let mut routed =
        route_edges_with_lane_rounds(plan, nodes, options, outer_rounds, gap_rounds, supplemental);
    if routed.primary_quality.is_none() {
        routed.primary_quality = Some(route_quality_for_plan(plan, &routed.primary));
    }
    routed
}

#[cfg(test)]
pub(crate) fn route_planned_edges(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    supplemental: bool,
) -> Vec<EdgeGeometry> {
    route_planned_candidates(plan, nodes, options, supplemental).primary
}

// Keep one WASM copy of the shared routing loop for full and supplemental effort.
#[inline(never)]
fn route_edges_with_lane_rounds(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    outer_lane_rounds: usize,
    gap_lane_rounds: usize,
    repair_crossings: bool,
) -> RoutedEdges {
    let ranks = &plan.ranks;
    debug_assert_eq!(nodes.len(), ranks.len());
    let top = nodes.iter().map(|node| node.y).fold(0.0, f64::min);
    let bottom = nodes
        .iter()
        .map(|node| node.y + node.height)
        .fold(0.0, f64::max);
    let max_rank = plan.nodes_by_rank.len().saturating_sub(1);
    let mut layer_left = vec![f64::INFINITY; max_rank + 1];
    let mut layer_right = vec![f64::NEG_INFINITY; max_rank + 1];
    for (node, &rank) in nodes.iter().zip(ranks) {
        layer_left[rank] = layer_left[rank].min(node.x);
        layer_right[rank] = layer_right[rank].max(node.x + node.width);
    }

    let free_by_rank: Vec<_> = plan
        .nodes_by_rank
        .iter()
        .map(|indices| {
            let mut layer = indices
                .iter()
                .map(|&index| &nodes[index])
                .collect::<Vec<_>>();
            layer.sort_by(|left, right| left.y.total_cmp(&right.y).then(left.id.cmp(&right.id)));
            free_intervals(&layer, top, bottom)
        })
        .collect();

    let sparse_spans: Vec<_> = plan
        .edges
        .iter()
        .map(|resolved| {
            let edge = resolved.edge;
            let source_index = resolved.source_index;
            let target_index = resolved.target_index;
            let source_port = resolved.source_port;
            let target_port = resolved.target_port;
            let source_rank = ranks[source_index];
            let target_rank = ranks[target_index];
            (source_port.side == PortSide::East
                && target_port.side == PortSide::West
                && source_rank < target_rank
                // Extremely large nets are cheaper as one outer trunk; their sparse tree does
                // not improve quality enough to pay for per-layer corridor construction.
                && plan.net_edge_counts[&edge.net] <= MAX_SPARSE_NET_EDGES
                && (source_rank + 1..target_rank).all(|rank| !free_by_rank[rank].is_empty()))
            .then_some((source_rank, target_rank))
        })
        .collect();

    let mut gap_preferences = vec![BTreeMap::<u32, Vec<f64>>::new(); max_rank];
    let mut crossing_preferences = vec![BTreeMap::<u32, Vec<f64>>::new(); max_rank + 1];
    let mut crossing_pairs = BTreeSet::new();
    let mut outer_nets = BTreeSet::new();
    for (resolved, span) in plan.edges.iter().zip(&sparse_spans) {
        let edge = resolved.edge;
        if let Some((source_rank, target_rank)) = span {
            let source_index = resolved.source_index;
            let target_index = resolved.target_index;
            let source = port_point(&nodes[source_index], resolved.source_port);
            let target = port_point(&nodes[target_index], resolved.target_port);
            let span = (*target_rank - *source_rank) as f64;
            for (gap, preferences) in gap_preferences
                .iter_mut()
                .enumerate()
                .take(*target_rank)
                .skip(*source_rank)
            {
                let progress = (gap - *source_rank) as f64 / span;
                preferences
                    .entry(edge.net)
                    .or_default()
                    .push(source.y + (target.y - source.y) * progress);
            }
            for (rank, preferences) in crossing_preferences
                .iter_mut()
                .enumerate()
                .take(*target_rank)
                .skip(source_rank + 1)
            {
                crossing_pairs.insert((rank, edge.net));
                let progress = (rank - *source_rank) as f64 / span;
                preferences
                    .entry(edge.net)
                    .or_default()
                    .push(source.y + (target.y - source.y) * progress);
            }
        } else {
            outer_nets.insert(edge.net);
        }
    }
    let initial_gap_lanes: Vec<_> = gap_preferences
        .into_iter()
        .map(preferred_lane_indices)
        .collect();
    let crossing_lanes: Vec<_> = crossing_preferences
        .into_iter()
        .map(preferred_lane_indices)
        .collect();
    let crossing_tie_lanes: BTreeMap<_, _> = crossing_pairs
        .into_iter()
        .enumerate()
        .map(|(lane, pair)| (pair, lane))
        .collect();
    let crossing_tie_lane_count = crossing_tie_lanes.len();
    let baseline_outer_lanes = outer_lane_assignments(
        plan,
        nodes,
        ranks,
        &sparse_spans,
        &outer_nets,
        &layer_left,
        &layer_right,
        top,
        bottom,
        options,
        outer_lane_rounds,
        false,
    );
    let RoutedLaneState {
        mut routes,
        gap_lanes,
        crossing_paths,
    } = emit_routes_with_outer_lanes(
        plan,
        nodes,
        &sparse_spans,
        &crossing_lanes,
        &crossing_tie_lanes,
        crossing_tie_lane_count,
        &free_by_rank,
        &layer_left,
        &layer_right,
        &initial_gap_lanes,
        &baseline_outer_lanes,
        top,
        bottom,
        options,
        gap_lane_rounds,
    );
    let mut outer_lanes = baseline_outer_lanes;
    let mut primary_quality = None;
    let node_count = plan
        .nodes_by_rank
        .iter()
        .map(Vec::len)
        .try_fold(0usize, usize::checked_add)
        .unwrap_or(usize::MAX);
    let split_feedback = has_split_feedback_net(plan, &sparse_spans, &outer_lanes);
    let feedback_within_budget = split_feedback
        && crossing_repair_within_budget(
            node_count,
            plan.edges.len(),
            &routes,
            &gap_lanes,
            &sparse_spans,
            &free_by_rank,
        );
    #[cfg(test)]
    let mut feedback_trace = FeedbackCandidateTrace {
        split: split_feedback,
        evaluated: false,
        selected: false,
        baseline: None,
        candidate_quality: None,
    };
    // Only spend on the alternative when the baseline visibly fragments a feedback net. The
    // shared optional-candidate budget keeps the extra exact score bounded on large inputs.
    if feedback_within_budget {
        let coherent_outer_lanes = outer_lane_assignments(
            plan,
            nodes,
            ranks,
            &sparse_spans,
            &outer_nets,
            &layer_left,
            &layer_right,
            top,
            bottom,
            options,
            outer_lane_rounds,
            true,
        );
        let baseline_quality = route_quality_for_plan(plan, &routes);
        // Coherence changes outer side and side-local lane indices, but not the stable per-net
        // channel index. The baseline sparse paths and gap lanes therefore remain valid.
        let candidate_endpoint_tracks = build_endpoint_tracks(
            plan,
            nodes,
            ranks,
            &sparse_spans,
            &layer_left,
            &layer_right,
            &gap_lanes,
            &coherent_outer_lanes,
            options,
        );
        let candidate_routes = emit_routes(
            plan,
            nodes,
            &sparse_spans,
            &crossing_paths,
            &layer_left,
            &layer_right,
            &gap_lanes,
            &candidate_endpoint_tracks,
            &coherent_outer_lanes,
            top,
            bottom,
            options,
        );
        let candidate_quality = route_quality_for_plan(plan, &candidate_routes);
        #[cfg(test)]
        {
            feedback_trace.evaluated = true;
            feedback_trace.baseline = Some((baseline_quality, routes.clone()));
            feedback_trace.candidate_quality = Some(candidate_quality);
        }
        // Preserve the canonical physical-quality ordering; coherence is never accepted merely
        // for looking tidier when it would increase crossings, bends, or route length.
        if route_quality_cmp(candidate_quality, baseline_quality).is_lt() {
            routes = candidate_routes;
            outer_lanes = coherent_outer_lanes;
            primary_quality = Some(candidate_quality);
            #[cfg(test)]
            {
                feedback_trace.selected = true;
            }
        } else {
            primary_quality = Some(baseline_quality);
        }
    }
    let (primary_quality, repair) = if repair_crossings {
        let (quality, repair) = repair_crossing_heavy_net(
            plan,
            nodes,
            &sparse_spans,
            &crossing_lanes,
            &crossing_tie_lanes,
            crossing_tie_lane_count,
            &free_by_rank,
            &layer_left,
            &layer_right,
            &gap_lanes,
            &outer_lanes,
            top,
            bottom,
            options,
            &routes,
        );
        (Some(quality), repair)
    } else {
        (primary_quality, None)
    };
    RoutedEdges {
        primary: routes,
        primary_quality,
        repair,
        #[cfg(test)]
        feedback_trace,
    }
}

fn has_split_feedback_net(
    plan: &RoutingPlan<'_>,
    sparse_spans: &[Option<(usize, usize)>],
    outer_lanes: &BTreeMap<u32, OuterLane>,
) -> bool {
    let feedback_nets = plan
        .edges
        .iter()
        .filter(|resolved| !resolved.participates_in_ranking)
        .map(|resolved| resolved.edge.net)
        .collect::<BTreeSet<_>>();
    let mut sides_by_net = BTreeMap::<NetId, u8>::new();
    plan.edges
        .iter()
        .zip(sparse_spans)
        .filter(|(resolved, span)| span.is_none() && feedback_nets.contains(&resolved.edge.net))
        .any(|(resolved, _)| {
            let side = match outer_lanes[&resolved.edge.id].side {
                OuterSide::Top => 1,
                OuterSide::Bottom => 2,
            };
            let sides = sides_by_net.entry(resolved.edge.net).or_default();
            *sides |= side;
            *sides == 3
        })
}

#[allow(clippy::too_many_arguments)]
fn emit_routes_with_outer_lanes(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_lanes: &[BTreeMap<u32, usize>],
    crossing_tie_lanes: &BTreeMap<(usize, u32), usize>,
    crossing_tie_lane_count: usize,
    free_by_rank: &[Vec<(f64, f64)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    outer_lanes: &BTreeMap<u32, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    gap_lane_rounds: usize,
) -> RoutedLaneState {
    let mut endpoint_tracks = build_endpoint_tracks(
        plan,
        nodes,
        &plan.ranks,
        sparse_spans,
        layer_left,
        layer_right,
        gap_lanes,
        outer_lanes,
        options,
    );
    let crossing_paths = sparse_crossing_paths(
        plan,
        nodes,
        sparse_spans,
        crossing_lanes,
        crossing_tie_lanes,
        crossing_tie_lane_count,
        free_by_rank,
        &endpoint_tracks,
        options.port_stub,
    );
    let gap_lanes = crossing_aware_gap_lanes(
        plan,
        nodes,
        sparse_spans,
        &crossing_paths,
        gap_lanes,
        &endpoint_tracks,
        options.port_stub,
        gap_lane_rounds,
    );
    endpoint_tracks = build_endpoint_tracks(
        plan,
        nodes,
        &plan.ranks,
        sparse_spans,
        layer_left,
        layer_right,
        &gap_lanes,
        outer_lanes,
        options,
    );
    let routes = emit_routes(
        plan,
        nodes,
        sparse_spans,
        &crossing_paths,
        layer_left,
        layer_right,
        &gap_lanes,
        &endpoint_tracks,
        outer_lanes,
        top,
        bottom,
        options,
    );
    RoutedLaneState {
        routes,
        gap_lanes,
        crossing_paths,
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_routes(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_paths: &[Option<Vec<f64>>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    outer_lanes: &BTreeMap<u32, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
) -> Vec<EdgeGeometry> {
    let ranks = &plan.ranks;
    plan.edges
        .iter()
        .zip(sparse_spans)
        .zip(crossing_paths)
        .map(|((resolved, sparse_span), crossing_path)| {
            let edge = resolved.edge;
            let source_index = resolved.source_index;
            let target_index = resolved.target_index;
            let source_node = &nodes[source_index];
            let target_node = &nodes[target_index];
            let source_port = resolved.source_port;
            let target_port = resolved.target_port;
            let source = port_point(source_node, source_port);
            let target = port_point(target_node, target_port);
            if let (Some(&(source_rank, target_rank)), Some(crossing_path)) =
                (sparse_span.as_ref(), crossing_path.as_ref())
            {
                return EdgeGeometry {
                    id: edge.id,
                    points: sparse_channel_route(
                        edge.net,
                        source,
                        target,
                        edge.source,
                        edge.target,
                        source_rank,
                        target_rank,
                        layer_left,
                        layer_right,
                        gap_lanes,
                        crossing_path,
                        endpoint_tracks,
                        options.port_stub,
                    ),
                };
            }

            let lane = outer_lanes[&edge.id];
            let source_stub = stub_point(source, source_port.side, options.port_stub);
            let target_stub = stub_point(target, target_port.side, options.port_stub);
            let source_escape_y = if matches!(source_port.side, PortSide::East | PortSide::West) {
                endpoint_escape_y(source, edge.source, 0, endpoint_tracks, options.port_stub)
            } else {
                source_stub.y
            };
            let target_escape_y = if matches!(target_port.side, PortSide::East | PortSide::West) {
                endpoint_escape_y(target, edge.target, 1, endpoint_tracks, options.port_stub)
            } else {
                target_stub.y
            };
            let source_channel = channel_point(
                source_stub,
                source_node,
                source_port.side,
                ranks[source_index],
                lane.channel_index,
                lane.channel_count,
                layer_left,
                layer_right,
                options,
            );
            let target_channel = channel_point(
                target_stub,
                target_node,
                target_port.side,
                ranks[target_index],
                lane.channel_index,
                lane.channel_count,
                layer_left,
                layer_right,
                options,
            );
            let lane_offset =
                options.port_stub + (lane.side_index + 1) as f64 * options.route_lane_gap;
            let lane_y = match lane.side {
                OuterSide::Top => top - lane_offset,
                OuterSide::Bottom => bottom + lane_offset,
            };
            let mut points = Vec::with_capacity(8);
            push_point(&mut points, source);
            push_point(&mut points, source_stub);
            push_point(
                &mut points,
                Point {
                    x: source_stub.x,
                    y: source_escape_y,
                },
            );
            push_point(
                &mut points,
                Point {
                    x: source_channel.x,
                    y: source_escape_y,
                },
            );
            push_point(
                &mut points,
                Point {
                    x: source_channel.x,
                    y: lane_y,
                },
            );
            push_point(
                &mut points,
                Point {
                    x: target_channel.x,
                    y: lane_y,
                },
            );
            push_point(
                &mut points,
                Point {
                    x: target_channel.x,
                    y: target_escape_y,
                },
            );
            push_point(
                &mut points,
                Point {
                    x: target_stub.x,
                    y: target_escape_y,
                },
            );
            push_point(&mut points, target_stub);
            push_point(&mut points, target);
            EdgeGeometry {
                id: edge.id,
                points,
            }
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
fn repair_crossing_heavy_net(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_lanes: &[BTreeMap<u32, usize>],
    crossing_tie_lanes: &BTreeMap<(usize, u32), usize>,
    crossing_tie_lane_count: usize,
    free_by_rank: &[Vec<(f64, f64)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    outer_lanes: &BTreeMap<u32, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    routes: &[EdgeGeometry],
) -> (RouteQuality, Option<(RouteQuality, Vec<EdgeGeometry>)>) {
    let node_count = plan
        .nodes_by_rank
        .iter()
        .map(Vec::len)
        .try_fold(0usize, usize::checked_add)
        .unwrap_or(usize::MAX);
    if !crossing_repair_within_budget(
        node_count,
        plan.edges.len(),
        routes,
        gap_lanes,
        sparse_spans,
        free_by_rank,
    ) {
        return (route_quality_for_plan(plan, routes), None);
    }
    let (crossing_counts, quality) = horizontal_crossing_counts_by_net(plan, routes);
    let repair = (|| {
        let net = select_crossing_repair_net(quality.crossings, &crossing_counts, gap_lanes)?;
        let candidate_lanes = move_net_to_outer_lane(gap_lanes, net)?;
        let endpoint_tracks = build_endpoint_tracks(
            plan,
            nodes,
            &plan.ranks,
            sparse_spans,
            layer_left,
            layer_right,
            &candidate_lanes,
            outer_lanes,
            options,
        );
        let crossing_paths = sparse_crossing_paths(
            plan,
            nodes,
            sparse_spans,
            crossing_lanes,
            crossing_tie_lanes,
            crossing_tie_lane_count,
            free_by_rank,
            &endpoint_tracks,
            options.port_stub,
        );
        Some(emit_routes(
            plan,
            nodes,
            sparse_spans,
            &crossing_paths,
            layer_left,
            layer_right,
            &candidate_lanes,
            &endpoint_tracks,
            outer_lanes,
            top,
            bottom,
            options,
        ))
    })();
    let repair = repair.map(|routes| (route_quality_for_plan(plan, &routes), routes));
    (quality, repair)
}

fn crossing_repair_within_budget(
    node_count: usize,
    edge_count: usize,
    routes: &[EdgeGeometry],
    gap_lanes: &[BTreeMap<NetId, usize>],
    sparse_spans: &[Option<(usize, usize)>],
    free_by_rank: &[Vec<(f64, f64)>],
) -> bool {
    node_count <= MAX_CROSSING_REPAIR_NODES
        && edge_count <= MAX_CROSSING_REPAIR_EDGES
        && sum_within_limit(
            routes.iter().map(|route| route.points.len()),
            MAX_CROSSING_REPAIR_ROUTE_POINTS,
        )
        && sum_within_limit(
            gap_lanes.iter().map(BTreeMap::len),
            MAX_CROSSING_REPAIR_LANE_MEMBERSHIPS,
        )
        && sum_within_limit(
            sparse_spans
                .iter()
                .filter_map(|span| span.as_ref())
                .flat_map(|&(source_rank, target_rank)| {
                    free_by_rank[source_rank + 1..target_rank]
                        .iter()
                        .map(Vec::len)
                }),
            MAX_CROSSING_REPAIR_PATH_STATES,
        )
}

fn sum_within_limit(mut values: impl Iterator<Item = usize>, limit: usize) -> bool {
    values
        .try_fold(0usize, |total, value| {
            total.checked_add(value).filter(|&sum| sum <= limit)
        })
        .is_some()
}

fn select_crossing_repair_net(
    total_crossings: usize,
    crossing_counts: &BTreeMap<NetId, usize>,
    gap_lanes: &[BTreeMap<NetId, usize>],
) -> Option<NetId> {
    if total_crossings < MIN_CROSSING_REPAIR_TOTAL {
        return None;
    }
    crossing_counts
        .iter()
        .filter(|(net, crossings)| {
            **crossings >= MIN_CROSSING_REPAIR_NET
                && gap_lanes
                    .iter()
                    .any(|lanes| lanes.get(net).is_some_and(|&lane| lane + 1 < lanes.len()))
        })
        .max_by(|left, right| left.1.cmp(right.1).then(right.0.cmp(left.0)))
        .map(|(&net, _)| net)
}

fn move_net_to_outer_lane(
    gap_lanes: &[BTreeMap<NetId, usize>],
    net: NetId,
) -> Option<Vec<BTreeMap<NetId, usize>>> {
    let mut changed = false;
    let result = gap_lanes
        .iter()
        .map(|lanes| {
            let Some(&current) = lanes.get(&net) else {
                return lanes.clone();
            };
            let mut ordered = lanes
                .iter()
                .map(|(&candidate, &lane)| (lane, candidate))
                .collect::<Vec<_>>();
            ordered.sort_unstable();
            let target = lanes.len().saturating_sub(1);
            if current == target {
                return lanes.clone();
            }
            changed = true;
            ordered.retain(|&(_, candidate)| candidate != net);
            ordered.insert(target, (target, net));
            ordered
                .into_iter()
                .enumerate()
                .map(|(lane, (_, candidate))| (candidate, lane))
                .collect()
        })
        .collect();
    changed.then_some(result)
}

pub(crate) fn route_quality(graph: &IndexedGraph<'_>, routes: &[EdgeGeometry]) -> RouteQuality {
    let (segments, bends, route_length) =
        physical_route_segments(graph.edges.iter().copied(), routes);
    let shared_endpoints = shared_endpoints(graph.edges.iter().copied());
    let crossings = physical_crossings(&shared_endpoints, &segments);
    RouteQuality {
        crossings,
        bends,
        route_length,
    }
}

fn route_quality_for_plan(plan: &RoutingPlan<'_>, routes: &[EdgeGeometry]) -> RouteQuality {
    let (segments, bends, route_length) =
        physical_route_segments(plan.edges.iter().map(|edge| edge.edge), routes);
    let crossings = physical_crossings(&plan.shared_endpoints, &segments);
    RouteQuality {
        crossings,
        bends,
        route_length,
    }
}

fn route_quality_cmp(left: RouteQuality, right: RouteQuality) -> Ordering {
    left.crossings
        .cmp(&right.crossings)
        .then(left.bends.cmp(&right.bends))
        .then(left.route_length.total_cmp(&right.route_length))
}

fn physical_route_segments<'a>(
    edges: impl Iterator<Item = &'a Edge>,
    routes: &[EdgeGeometry],
) -> (Vec<PhysicalSegment>, usize, f64) {
    let mut groups =
        BTreeMap::<(u32, bool, FloatKey), Vec<(f64, f64, EdgeId, Endpoint, Endpoint)>>::new();
    let mut bends = BTreeSet::new();
    for (edge, route) in edges.zip(routes) {
        for points in route.points.windows(3) {
            let first_horizontal = points[0].y == points[1].y;
            let second_horizontal = points[1].y == points[2].y;
            if first_horizontal != second_horizontal {
                bends.insert((edge.net, FloatKey(points[1].x), FloatKey(points[1].y)));
            }
        }
        for points in route.points.windows(2) {
            let horizontal = points[0].y == points[1].y;
            let fixed = if horizontal { points[0].y } else { points[0].x };
            let (first, second) = if horizontal {
                (points[0].x, points[1].x)
            } else {
                (points[0].y, points[1].y)
            };
            let start = first.min(second);
            let end = first.max(second);
            if start == end {
                continue;
            }
            groups
                .entry((edge.net, horizontal, FloatKey(fixed)))
                .or_default()
                .push((start, end, edge.id, edge.source, edge.target));
        }
    }

    let mut segments = Vec::<PhysicalSegment>::new();
    for ((net, horizontal, FloatKey(fixed)), intervals) in &mut groups {
        intervals.sort_by(|left, right| {
            left.0
                .total_cmp(&right.0)
                .then(left.1.total_cmp(&right.1))
                .then(left.2.cmp(&right.2))
        });
        for &(start, end, _, source, target) in intervals.iter() {
            if let Some(prior) = segments.last_mut()
                && prior.net == *net
                && prior.horizontal == *horizontal
                && prior.fixed == *fixed
                && start <= prior.end
            {
                prior.end = prior.end.max(end);
            } else {
                segments.push(PhysicalSegment {
                    net: *net,
                    source,
                    target,
                    horizontal: *horizontal,
                    fixed: *fixed,
                    start,
                    end,
                });
            }
        }
    }
    let route_length = segments
        .iter()
        .map(|segment| segment.end - segment.start)
        .sum();
    (segments, bends.len(), route_length)
}

#[derive(Clone, Copy)]
enum CrossingEvent {
    Remove { segment: usize, y: usize },
    Query { segment: usize },
    Add { segment: usize, y: usize },
}

fn shared_endpoints<'a>(edges: impl Iterator<Item = &'a Edge>) -> HashSet<Endpoint> {
    let mut endpoint_nets = HashMap::<Endpoint, NetId>::new();
    let mut shared = HashSet::new();
    for edge in edges {
        for endpoint in [edge.source, edge.target] {
            match endpoint_nets.entry(endpoint) {
                Entry::Vacant(entry) => {
                    entry.insert(edge.net);
                }
                Entry::Occupied(entry) if *entry.get() != edge.net => {
                    shared.insert(endpoint);
                }
                Entry::Occupied(_) => {}
            }
        }
    }
    shared
}

fn physical_crossings(shared_endpoints: &HashSet<Endpoint>, segments: &[PhysicalSegment]) -> usize {
    physical_crossing_sweep(shared_endpoints, segments, false, None)
}

#[inline(never)]
fn physical_crossing_sweep(
    shared_endpoints: &HashSet<Endpoint>,
    segments: &[PhysicalSegment],
    transpose: bool,
    mut contributions: Option<&mut BTreeMap<NetId, usize>>,
) -> usize {
    let horizontal = segments
        .iter()
        .filter(|segment| segment.horizontal != transpose)
        .collect::<Vec<_>>();
    let vertical = segments
        .iter()
        .filter(|segment| segment.horizontal == transpose)
        .collect::<Vec<_>>();
    let mut horizontal_y = horizontal
        .iter()
        .map(|segment| segment.fixed)
        .collect::<Vec<_>>();
    horizontal_y.sort_by(f64::total_cmp);
    horizontal_y.dedup_by(|left, right| left.total_cmp(right).is_eq());

    let mut by_net = HashMap::<NetId, Vec<usize>>::new();
    let mut by_endpoint = HashMap::<Endpoint, Vec<usize>>::new();
    for (index, segment) in horizontal.iter().enumerate() {
        // Same-edge relationships are already covered because one edge has exactly one net.
        by_net.entry(segment.net).or_default().push(index);
        for endpoint in [segment.source, segment.target] {
            if shared_endpoints.contains(&endpoint) {
                by_endpoint.entry(endpoint).or_default().push(index);
            }
        }
    }

    let mut events = Vec::with_capacity(segments.len() * 2);
    for (segment_index, segment) in horizontal.iter().enumerate() {
        let y = horizontal_y
            .binary_search_by(|candidate| candidate.total_cmp(&segment.fixed))
            .expect("horizontal coordinate exists");
        events.push((
            FloatKey(segment.start),
            2u8,
            CrossingEvent::Add {
                segment: segment_index,
                y,
            },
        ));
        events.push((
            FloatKey(segment.end),
            0u8,
            CrossingEvent::Remove {
                segment: segment_index,
                y,
            },
        ));
    }
    for (segment_index, segment) in vertical.iter().enumerate() {
        events.push((
            FloatKey(segment.fixed),
            1u8,
            CrossingEvent::Query {
                segment: segment_index,
            },
        ));
    }
    events.sort_by(|left, right| left.0.cmp(&right.0).then(left.1.cmp(&right.1)));
    let mut active = CrossingFenwick::new(horizontal_y.len());
    let mut active_segments = vec![false; horizontal.len()];
    let mut relation_stamps = vec![0u32; horizontal.len()];
    let mut relation_generation = 0u32;
    let mut crossings = 0usize;
    for (_, _, event) in events {
        match event {
            CrossingEvent::Remove { segment, y } => {
                active_segments[segment] = false;
                active.remove(y);
            }
            CrossingEvent::Query { segment } => {
                let line = vertical[segment];
                let start = horizontal_y.partition_point(|&y| y <= line.start);
                let end = horizontal_y.partition_point(|&y| y < line.end);
                let mut count = active.prefix(end) - active.prefix(start);
                if count == 0 {
                    continue;
                }
                relation_generation = relation_generation.wrapping_add(1);
                if relation_generation == 0 {
                    relation_stamps.fill(0);
                    relation_generation = 1;
                }
                let mut related_count = 0usize;
                let mut visit = |candidates: Option<&Vec<usize>>| {
                    for &candidate in candidates.into_iter().flatten() {
                        if relation_stamps[candidate] == relation_generation {
                            continue;
                        }
                        relation_stamps[candidate] = relation_generation;
                        if active_segments[candidate]
                            && horizontal[candidate].fixed > line.start
                            && horizontal[candidate].fixed < line.end
                        {
                            related_count += 1;
                        }
                    }
                };
                visit(by_net.get(&line.net));
                for endpoint in [line.source, line.target] {
                    visit(by_endpoint.get(&endpoint));
                }
                count -= related_count;
                crossings += count;
                if count != 0
                    && let Some(contributions) = &mut contributions
                {
                    *contributions.entry(line.net).or_default() += count;
                }
            }
            CrossingEvent::Add { segment, y } => {
                active_segments[segment] = true;
                active.add(y);
            }
        }
    }
    crossings
}

/// Attribute each crossing to its original horizontal participant.
///
/// This deliberately uses one sweep: attribution only selects a bounded repair candidate, while
/// complete layouts are accepted using the orientation-independent exact crossing score.
fn horizontal_crossing_counts_by_net(
    plan: &RoutingPlan<'_>,
    routes: &[EdgeGeometry],
) -> (BTreeMap<NetId, usize>, RouteQuality) {
    let (segments, bends, route_length) =
        physical_route_segments(plan.edges.iter().map(|edge| edge.edge), routes);
    let mut counts = BTreeMap::<NetId, usize>::new();
    let crossings =
        physical_crossing_sweep(&plan.shared_endpoints, &segments, true, Some(&mut counts));
    (
        counts,
        RouteQuality {
            crossings,
            bends,
            route_length,
        },
    )
}

struct CrossingFenwick {
    values: Vec<usize>,
}

impl CrossingFenwick {
    fn new(len: usize) -> Self {
        Self {
            values: vec![0; len + 1],
        }
    }

    fn add(&mut self, index: usize) {
        let mut cursor = index + 1;
        while cursor < self.values.len() {
            self.values[cursor] += 1;
            cursor += cursor & cursor.wrapping_neg();
        }
    }

    fn remove(&mut self, index: usize) {
        let mut cursor = index + 1;
        while cursor < self.values.len() {
            self.values[cursor] -= 1;
            cursor += cursor & cursor.wrapping_neg();
        }
    }

    fn prefix(&self, end: usize) -> usize {
        let mut cursor = end;
        let mut total = 0;
        while cursor > 0 {
            total += self.values[cursor];
            cursor &= cursor - 1;
        }
        total
    }
}

fn outer_channel_lane_indices(
    nets: &BTreeSet<NetId>,
    edge_counts: &BTreeMap<NetId, usize>,
) -> BTreeMap<NetId, usize> {
    let mut ordered = nets.iter().copied().collect::<Vec<_>>();
    if ordered
        .iter()
        .any(|net| edge_counts[net] >= MIN_FANOUT_AWARE_CHANNEL_EDGES)
    {
        ordered.sort_unstable_by_key(|net| (edge_counts[net], *net));
    }
    ordered
        .into_iter()
        .enumerate()
        .map(|(index, net)| (net, index))
        .collect()
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OuterSide {
    Top,
    Bottom,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct OuterLane {
    side: OuterSide,
    side_index: usize,
    channel_index: usize,
    channel_count: usize,
}

#[derive(Default)]
struct OuterNetAccess {
    horizontal: Vec<(f64, f64)>,
    vertical_x: Vec<f64>,
}

#[allow(clippy::too_many_arguments)]
fn outer_lane_assignments(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    outer_nets: &BTreeSet<u32>,
    layer_left: &[f64],
    layer_right: &[f64],
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    lane_rounds: usize,
    coherent_feedback: bool,
) -> BTreeMap<u32, OuterLane> {
    let mut top_nets = BTreeSet::new();
    let mut bottom_nets = BTreeSet::new();
    let mut edge_sides = BTreeMap::new();
    let feedback_nets = plan
        .edges
        .iter()
        .filter(|resolved| coherent_feedback && !resolved.participates_in_ranking)
        .map(|resolved| resolved.edge.net)
        .collect::<BTreeSet<_>>();
    let mut feedback_endpoint_costs = BTreeMap::<u32, (f64, f64)>::new();
    let mut counted_feedback_endpoints = HashSet::new();
    for (resolved, span) in plan.edges.iter().zip(sparse_spans) {
        let edge = resolved.edge;
        if span.is_some() {
            continue;
        }
        let mut cost = (0.0, 0.0);
        for (node_index, port) in [
            (resolved.source_index, resolved.source_port),
            (resolved.target_index, resolved.target_port),
        ] {
            let point = port_point(&nodes[node_index], port);
            cost.0 += point.y - top;
            cost.1 += bottom - point.y;
        }
        if feedback_nets.contains(&edge.net) {
            for (endpoint, node_index, port) in [
                (edge.source, resolved.source_index, resolved.source_port),
                (edge.target, resolved.target_index, resolved.target_port),
            ] {
                if counted_feedback_endpoints.insert((edge.net, endpoint)) {
                    let point = port_point(&nodes[node_index], port);
                    let net_cost = feedback_endpoint_costs.entry(edge.net).or_default();
                    net_cost.0 += point.y - top;
                    net_cost.1 += bottom - point.y;
                }
            }
        }
        edge_sides.insert(edge.id, (edge.net, cost));
    }
    let feedback_sides = feedback_endpoint_costs
        .into_iter()
        .map(|(net, cost)| {
            let side = if cost.1 < cost.0 {
                OuterSide::Bottom
            } else {
                OuterSide::Top
            };
            (net, side)
        })
        .collect::<BTreeMap<_, _>>();
    let edge_sides = edge_sides
        .into_iter()
        .map(|(edge, (net, cost))| {
            let side = feedback_sides.get(&net).copied().unwrap_or({
                if cost.1 < cost.0 {
                    OuterSide::Bottom
                } else {
                    OuterSide::Top
                }
            });
            if side == OuterSide::Bottom {
                bottom_nets.insert(net);
            } else {
                top_nets.insert(net);
            }
            (edge, side)
        })
        .collect::<BTreeMap<_, _>>();

    let mut assignments = BTreeMap::new();
    let channel_lanes = outer_channel_lane_indices(outer_nets, &plan.net_edge_counts);
    let channel_count = channel_lanes.len();
    let mut top_access = BTreeMap::<u32, OuterNetAccess>::new();
    let mut bottom_access = BTreeMap::<u32, OuterNetAccess>::new();
    for (resolved, span) in plan.edges.iter().zip(sparse_spans) {
        let edge = resolved.edge;
        if span.is_some() {
            continue;
        }
        let source_index = resolved.source_index;
        let target_index = resolved.target_index;
        let source_node = &nodes[source_index];
        let target_node = &nodes[target_index];
        let source_port = resolved.source_port;
        let target_port = resolved.target_port;
        let source_stub = stub_point(
            port_point(source_node, source_port),
            source_port.side,
            options.port_stub,
        );
        let target_stub = stub_point(
            port_point(target_node, target_port),
            target_port.side,
            options.port_stub,
        );
        let channel_index = channel_lanes[&edge.net];
        let source_x = channel_point(
            source_stub,
            source_node,
            source_port.side,
            ranks[source_index],
            channel_index,
            channel_count,
            layer_left,
            layer_right,
            options,
        )
        .x;
        let target_x = channel_point(
            target_stub,
            target_node,
            target_port.side,
            ranks[target_index],
            channel_index,
            channel_count,
            layer_left,
            layer_right,
            options,
        )
        .x;
        let access_by_net = match edge_sides[&edge.id] {
            OuterSide::Top => &mut top_access,
            OuterSide::Bottom => &mut bottom_access,
        };
        let access = access_by_net.entry(edge.net).or_default();
        access
            .horizontal
            .push((source_x.min(target_x), source_x.max(target_x)));
        access.vertical_x.extend([source_x, target_x]);
    }
    for access in top_access.values_mut().chain(bottom_access.values_mut()) {
        access.vertical_x.sort_by(f64::total_cmp);
    }
    let top_lanes =
        crossing_aware_outer_lane_indices_with_rounds(&top_nets, &top_access, lane_rounds);
    let bottom_lanes =
        crossing_aware_outer_lane_indices_with_rounds(&bottom_nets, &bottom_access, lane_rounds);
    for (resolved, span) in plan.edges.iter().zip(sparse_spans) {
        let edge = resolved.edge;
        if span.is_some() {
            continue;
        }
        let side = edge_sides[&edge.id];
        let side_index = match side {
            OuterSide::Top => top_lanes[&edge.net],
            OuterSide::Bottom => bottom_lanes[&edge.net],
        };
        assignments.insert(
            edge.id,
            OuterLane {
                side,
                side_index,
                channel_index: channel_lanes[&edge.net],
                channel_count,
            },
        );
    }
    assignments
}

#[cfg(test)]
fn crossing_aware_outer_lane_indices(
    nets: &BTreeSet<u32>,
    accesses: &BTreeMap<u32, OuterNetAccess>,
) -> BTreeMap<u32, usize> {
    crossing_aware_outer_lane_indices_with_rounds(nets, accesses, FULL_OUTER_LANE_ROUNDS)
}

fn crossing_aware_outer_lane_indices_with_rounds(
    nets: &BTreeSet<u32>,
    accesses: &BTreeMap<u32, OuterNetAccess>,
    lane_rounds: usize,
) -> BTreeMap<u32, usize> {
    let mut ordered: Vec<_> = nets.iter().copied().collect();
    let mut costs = BTreeMap::new();
    for _ in 0..lane_rounds {
        let mut changed = false;
        for index in 0..ordered.len().saturating_sub(1) {
            let inner = ordered[index];
            let outer = ordered[index + 1];
            let current = *costs
                .entry((inner, outer))
                .or_insert_with(|| outer_pair_crossings(&accesses[&inner], &accesses[&outer]));
            let swapped = *costs
                .entry((outer, inner))
                .or_insert_with(|| outer_pair_crossings(&accesses[&outer], &accesses[&inner]));
            if swapped < current {
                ordered.swap(index, index + 1);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    ordered
        .into_iter()
        .enumerate()
        .map(|(index, net)| (net, index))
        .collect()
}

fn outer_pair_crossings(inner: &OuterNetAccess, outer: &OuterNetAccess) -> usize {
    inner
        .horizontal
        .iter()
        .map(|&(low, high)| {
            let start = outer.vertical_x.partition_point(|&x| x <= low);
            let end = outer.vertical_x.partition_point(|&x| x < high);
            end - start
        })
        .sum()
}

fn preferred_lane_indices(mut preferences: BTreeMap<u32, Vec<f64>>) -> BTreeMap<u32, usize> {
    let mut ordered = Vec::with_capacity(preferences.len());
    for (net, values) in &mut preferences {
        values.sort_by(f64::total_cmp);
        ordered.push((*net, values[values.len() / 2]));
    }
    ordered.sort_by(|(left_net, left), (right_net, right)| {
        left.total_cmp(right).then(left_net.cmp(right_net))
    });
    ordered
        .into_iter()
        .enumerate()
        .map(|(lane, (net, _))| (net, lane))
        .collect()
}

fn endpoint_escape_y(
    point: Point,
    endpoint: crate::Endpoint,
    role: u8,
    tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    spread: f64,
) -> f64 {
    let Some(&(lane, lane_count)) = tracks.get(&(endpoint.node, endpoint.port, role)) else {
        return point.y;
    };
    let fraction = (lane + 1) as f64 / (lane_count + 1) as f64;
    point.y + (fraction - 0.5) * spread
}

#[derive(Clone, Copy)]
struct EndpointAccess {
    endpoint: crate::Endpoint,
    role: u8,
    net: u32,
    y: f64,
    low_x: f64,
    high_x: f64,
}

#[allow(clippy::too_many_arguments)]
fn build_endpoint_tracks(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    outer_lanes: &BTreeMap<u32, OuterLane>,
    options: LayoutOptions,
) -> BTreeMap<(u32, u32, u8), (usize, usize)> {
    let mut accesses = BTreeMap::<(u32, u32, u8), EndpointAccess>::new();
    for (resolved, sparse_span) in plan.edges.iter().zip(sparse_spans) {
        let edge = resolved.edge;
        let source_index = resolved.source_index;
        let target_index = resolved.target_index;
        let source_node = &nodes[source_index];
        let target_node = &nodes[target_index];
        let source_port = resolved.source_port;
        let target_port = resolved.target_port;
        let source = port_point(source_node, source_port);
        let target = port_point(target_node, target_port);
        let source_stub = stub_point(source, source_port.side, options.port_stub);
        let target_stub = stub_point(target, target_port.side, options.port_stub);
        let (source_channel_x, target_channel_x) =
            if let Some((source_rank, target_rank)) = sparse_span {
                (
                    sparse_gap_x(edge.net, *source_rank, layer_left, layer_right, gap_lanes),
                    sparse_gap_x(
                        edge.net,
                        target_rank - 1,
                        layer_left,
                        layer_right,
                        gap_lanes,
                    ),
                )
            } else {
                let lane = outer_lanes[&edge.id];
                (
                    channel_point(
                        source_stub,
                        source_node,
                        source_port.side,
                        ranks[source_index],
                        lane.channel_index,
                        lane.channel_count,
                        layer_left,
                        layer_right,
                        options,
                    )
                    .x,
                    channel_point(
                        target_stub,
                        target_node,
                        target_port.side,
                        ranks[target_index],
                        lane.channel_index,
                        lane.channel_count,
                        layer_left,
                        layer_right,
                        options,
                    )
                    .x,
                )
            };
        for (endpoint, role, port, point, stub, channel_x) in [
            (
                edge.source,
                0,
                source_port,
                source,
                source_stub,
                source_channel_x,
            ),
            (
                edge.target,
                1,
                target_port,
                target,
                target_stub,
                target_channel_x,
            ),
        ] {
            if !matches!(port.side, PortSide::East | PortSide::West) {
                continue;
            }
            let key = (endpoint.node, endpoint.port, role);
            let low_x = stub.x.min(channel_x);
            let high_x = stub.x.max(channel_x);
            accesses
                .entry(key)
                .and_modify(|access| {
                    access.low_x = access.low_x.min(low_x);
                    access.high_x = access.high_x.max(high_x);
                })
                .or_insert(EndpointAccess {
                    endpoint,
                    role,
                    net: edge.net,
                    y: point.y,
                    low_x,
                    high_x,
                });
        }
    }

    let mut accesses_by_y = BTreeMap::<u64, Vec<EndpointAccess>>::new();
    for access in accesses.into_values() {
        accesses_by_y
            .entry(access.y.to_bits())
            .or_default()
            .push(access);
    }
    let mut conflicts_by_y = BTreeMap::<u64, BTreeSet<(u32, u32, u8)>>::new();
    for (y, mut accesses) in accesses_by_y {
        accesses.sort_by(|left, right| {
            left.low_x
                .total_cmp(&right.low_x)
                .then(left.high_x.total_cmp(&right.high_x))
                .then(left.endpoint.node.cmp(&right.endpoint.node))
                .then(left.endpoint.port.cmp(&right.endpoint.port))
                .then(left.role.cmp(&right.role))
        });
        let mut component_start = 0;
        while component_start < accesses.len() {
            let mut component_end = component_start + 1;
            let mut high_x = accesses[component_start].high_x;
            while component_end < accesses.len() && accesses[component_end].low_x <= high_x {
                high_x = high_x.max(accesses[component_end].high_x);
                component_end += 1;
            }
            let component = &accesses[component_start..component_end];
            if component
                .iter()
                .any(|access| access.net != component[0].net)
            {
                let conflicts = conflicts_by_y.entry(y).or_default();
                for access in component {
                    conflicts.insert((access.endpoint.node, access.endpoint.port, access.role));
                }
            }
            component_start = component_end;
        }
    }

    let mut tracks = BTreeMap::new();
    for conflicts in conflicts_by_y.into_values() {
        let lane_count = conflicts.len();
        for (lane, endpoint) in conflicts.into_iter().enumerate() {
            tracks.insert(endpoint, (lane, lane_count));
        }
    }
    tracks
}

fn free_intervals(nodes: &[&NodeGeometry], top: f64, bottom: f64) -> Vec<(f64, f64)> {
    let mut intervals = Vec::with_capacity(nodes.len() + 1);
    let mut cursor = top;
    for node in nodes {
        if node.y > cursor {
            intervals.push((cursor, node.y));
        }
        cursor = cursor.max(node.y + node.height);
    }
    if cursor < bottom {
        intervals.push((cursor, bottom));
    }
    intervals
}

#[allow(clippy::too_many_arguments)]
fn sparse_crossing_paths(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_lanes: &[BTreeMap<u32, usize>],
    crossing_tie_lanes: &BTreeMap<(usize, u32), usize>,
    crossing_tie_lane_count: usize,
    free_by_rank: &[Vec<(f64, f64)>],
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    port_stub: f64,
) -> Vec<Option<Vec<f64>>> {
    // A single-driver net uses one obstacle-safe backbone; each sink route receives the prefix
    // that reaches its rank and branches only in the final gap.
    let mut edges_by_net = HashMap::<u32, Vec<usize>>::new();
    for (edge_index, resolved) in plan.edges.iter().enumerate() {
        if plan.net_edge_counts[&resolved.edge.net] > 1 {
            let edge = resolved.edge;
            edges_by_net.entry(edge.net).or_default().push(edge_index);
        }
    }
    let mut shared_paths = HashMap::<u32, (usize, Vec<f64>)>::new();
    for (net, edge_indices) in edges_by_net {
        if edge_indices.len() < 2
            || edge_indices
                .iter()
                .any(|&edge_index| sparse_spans[edge_index].is_none())
        {
            continue;
        }
        let first = plan.edges[edge_indices[0]];
        let first_edge = first.edge;
        if edge_indices
            .iter()
            .any(|&edge_index| plan.edges[edge_index].edge.source != first_edge.source)
        {
            continue;
        }
        let (source_rank, max_target_rank) = edge_indices
            .iter()
            .map(|&edge_index| sparse_spans[edge_index].expect("all spans are sparse"))
            .fold(
                (usize::MAX, 0),
                |(min_source, max_target), (source, target)| {
                    (min_source.min(source), max_target.max(target))
                },
            );
        if max_target_rank <= source_rank + 1 {
            continue;
        }
        let source = port_point(&nodes[first.source_index], first.source_port);
        let source_y = endpoint_escape_y(source, first_edge.source, 0, endpoint_tracks, port_stub);
        let mut target_ys = edge_indices
            .iter()
            .map(|&edge_index| {
                let resolved = plan.edges[edge_index];
                let edge = resolved.edge;
                let target = port_point(&nodes[resolved.target_index], resolved.target_port);
                endpoint_escape_y(target, edge.target, 1, endpoint_tracks, port_stub)
            })
            .collect::<Vec<_>>();
        target_ys.sort_by(f64::total_cmp);
        let target_y = target_ys[target_ys.len() / 2];
        let path = shortest_crossing_path(
            &free_by_rank[source_rank + 1..max_target_rank],
            source_y,
            target_y,
            &(source_rank + 1..max_target_rank)
                .map(|rank| crossing_lanes[rank][&net])
                .collect::<Vec<_>>(),
            &(source_rank + 1..max_target_rank)
                .map(|rank| crossing_lanes[rank].len())
                .collect::<Vec<_>>(),
            &(source_rank + 1..max_target_rank)
                .map(|rank| crossing_tie_lanes[&(rank, net)])
                .collect::<Vec<_>>(),
            crossing_tie_lane_count,
        );
        shared_paths.insert(net, (source_rank, path));
    }

    plan.edges
        .iter()
        .zip(sparse_spans)
        .map(|(resolved, span)| {
            let edge = resolved.edge;
            let &(source_rank, target_rank) = span.as_ref()?;
            if let Some(&(shared_source_rank, ref shared_path)) = shared_paths.get(&edge.net) {
                debug_assert_eq!(shared_source_rank, source_rank);
                return Some(shared_path[..target_rank - source_rank - 1].to_vec());
            }
            let source = port_point(&nodes[resolved.source_index], resolved.source_port);
            let target = port_point(&nodes[resolved.target_index], resolved.target_port);
            let source_y = endpoint_escape_y(source, edge.source, 0, endpoint_tracks, port_stub);
            let target_y = endpoint_escape_y(target, edge.target, 1, endpoint_tracks, port_stub);
            Some(shortest_crossing_path(
                &free_by_rank[source_rank + 1..target_rank],
                source_y,
                target_y,
                &(source_rank + 1..target_rank)
                    .map(|rank| crossing_lanes[rank][&edge.net])
                    .collect::<Vec<_>>(),
                &(source_rank + 1..target_rank)
                    .map(|rank| crossing_lanes[rank].len())
                    .collect::<Vec<_>>(),
                &(source_rank + 1..target_rank)
                    .map(|rank| crossing_tie_lanes[&(rank, edge.net)])
                    .collect::<Vec<_>>(),
                crossing_tie_lane_count,
            ))
        })
        .collect()
}

#[derive(Default)]
struct GapNetAccess {
    vertical: Vec<(f64, f64)>,
    left_y: Vec<f64>,
    right_y: Vec<f64>,
}

#[allow(clippy::too_many_arguments)]
fn crossing_aware_gap_lanes(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_paths: &[Option<Vec<f64>>],
    current_lanes: &[BTreeMap<u32, usize>],
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    port_stub: f64,
    lane_rounds: usize,
) -> Vec<BTreeMap<u32, usize>> {
    let mut accesses = (0..current_lanes.len())
        .map(|_| BTreeMap::<u32, GapNetAccess>::new())
        .collect::<Vec<_>>();
    for ((resolved, span), path) in plan.edges.iter().zip(sparse_spans).zip(crossing_paths) {
        let edge = resolved.edge;
        let (Some(&(source_rank, target_rank)), Some(path)) = (span.as_ref(), path) else {
            continue;
        };
        let source = port_point(&nodes[resolved.source_index], resolved.source_port);
        let target = port_point(&nodes[resolved.target_index], resolved.target_port);
        let source_y = endpoint_escape_y(source, edge.source, 0, endpoint_tracks, port_stub);
        let target_y = endpoint_escape_y(target, edge.target, 1, endpoint_tracks, port_stub);
        for gap in source_rank..target_rank {
            let before = if gap == source_rank {
                source_y
            } else {
                path[gap - source_rank - 1]
            };
            let after = if gap + 1 == target_rank {
                target_y
            } else {
                path[gap - source_rank]
            };
            let access = accesses[gap].entry(edge.net).or_default();
            access.vertical.push((before.min(after), before.max(after)));
            access.left_y.push(before);
            access.right_y.push(after);
        }
    }
    for by_net in &mut accesses {
        for access in by_net.values_mut() {
            access.left_y.sort_by(f64::total_cmp);
            access.right_y.sort_by(f64::total_cmp);
        }
    }
    current_lanes
        .iter()
        .zip(&accesses)
        .map(|(lanes, access)| {
            crossing_aware_gap_lane_indices_with_rounds(lanes, access, lane_rounds)
        })
        .collect()
}

#[cfg(test)]
fn crossing_aware_gap_lane_indices(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
) -> BTreeMap<u32, usize> {
    crossing_aware_gap_lane_indices_with_rounds(current, accesses, FULL_GAP_LANE_ROUNDS)
}

fn crossing_aware_gap_lane_indices_with_rounds(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
    lane_rounds: usize,
) -> BTreeMap<u32, usize> {
    let mut ordered: Vec<_> = current.iter().map(|(&net, &lane)| (lane, net)).collect();
    ordered.sort_unstable();
    let mut ordered: Vec<_> = ordered.into_iter().map(|(_, net)| net).collect();
    let mut costs = BTreeMap::new();
    for _ in 0..lane_rounds {
        let mut changed = false;
        for index in 0..ordered.len().saturating_sub(1) {
            let left = ordered[index];
            let right = ordered[index + 1];
            let current_cost = *costs
                .entry((left, right))
                .or_insert_with(|| gap_pair_crossings(&accesses[&left], &accesses[&right]));
            let swapped_cost = *costs
                .entry((right, left))
                .or_insert_with(|| gap_pair_crossings(&accesses[&right], &accesses[&left]));
            if swapped_cost < current_cost {
                ordered.swap(index, index + 1);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    ordered
        .into_iter()
        .enumerate()
        .map(|(index, net)| (net, index))
        .collect()
}

fn gap_pair_crossings(left: &GapNetAccess, right: &GapNetAccess) -> usize {
    vertical_horizontal_crossings(&left.vertical, &right.left_y)
        + vertical_horizontal_crossings(&right.vertical, &left.right_y)
}

fn vertical_horizontal_crossings(vertical: &[(f64, f64)], horizontal_y: &[f64]) -> usize {
    vertical
        .iter()
        .map(|&(low, high)| {
            if low >= high {
                return 0;
            }
            let start = horizontal_y.partition_point(|&y| y <= low);
            let end = horizontal_y.partition_point(|&y| y < high);
            end - start
        })
        .sum()
}

#[allow(clippy::too_many_arguments)]
fn sparse_channel_route(
    net: u32,
    source: Point,
    target: Point,
    source_endpoint: crate::Endpoint,
    target_endpoint: crate::Endpoint,
    source_rank: usize,
    target_rank: usize,
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    crossing_path: &[f64],
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    port_stub: f64,
) -> Vec<Point> {
    let source_stub = stub_point(source, PortSide::East, port_stub);
    let target_stub = stub_point(target, PortSide::West, port_stub);
    let source_escape_y = endpoint_escape_y(source, source_endpoint, 0, endpoint_tracks, port_stub);
    let target_escape_y = endpoint_escape_y(target, target_endpoint, 1, endpoint_tracks, port_stub);
    let mut points = Vec::with_capacity(2 * (target_rank - source_rank) + 8);
    push_point(&mut points, source);
    push_point(&mut points, source_stub);
    push_point(
        &mut points,
        Point {
            x: source_stub.x,
            y: source_escape_y,
        },
    );
    let mut x = sparse_gap_x(net, source_rank, layer_left, layer_right, gap_lanes);
    push_point(
        &mut points,
        Point {
            x,
            y: source_escape_y,
        },
    );

    for (rank, &y) in (source_rank + 1..target_rank).zip(crossing_path) {
        push_point(&mut points, Point { x, y });
        x = sparse_gap_x(net, rank, layer_left, layer_right, gap_lanes);
        push_point(&mut points, Point { x, y });
    }

    let current_y = points.last().expect("route has a source channel").y;
    if current_y != target_escape_y && (current_y - target_escape_y).abs() <= MIN_ROUTE_SEGMENT {
        let detour_y = current_y + port_stub;
        push_point(&mut points, Point { x, y: detour_y });
        push_point(
            &mut points,
            Point {
                x: target_stub.x,
                y: detour_y,
            },
        );
    } else {
        push_point(
            &mut points,
            Point {
                x,
                y: target_escape_y,
            },
        );
    }
    push_point(
        &mut points,
        Point {
            x: target_stub.x,
            y: target_escape_y,
        },
    );
    push_point(&mut points, target_stub);
    push_point(&mut points, target);
    points
}

fn sparse_gap_x(
    net: u32,
    gap: usize,
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
) -> f64 {
    let lanes = &gap_lanes[gap];
    let lane_fraction = (lanes[&net] + 1) as f64 / (lanes.len() + 1) as f64;
    let fraction = 0.55 + 0.15 * lane_fraction;
    layer_right[gap] + (layer_left[gap + 1] - layer_right[gap]) * fraction
}

fn crossing_track_y(
    interval: (f64, f64),
    lane: usize,
    lane_count: usize,
    tie_lane: usize,
    tie_lane_count: usize,
) -> f64 {
    let (low, high) = interval;
    let lane_fraction = (lane + 1) as f64 / (lane_count + 1) as f64;
    let y = low + (high - low) * lane_fraction;
    let margin = (y - low).min(high - y);
    let tie_fraction = (tie_lane + 1) as f64 / (tie_lane_count + 1) as f64;
    y + (CROSSING_TRACK_NUDGE * tie_fraction).min(margin / 2.0)
}

fn shortest_crossing_path(
    layers: &[Vec<(f64, f64)>],
    source_y: f64,
    target_y: f64,
    lanes: &[usize],
    lane_counts: &[usize],
    tie_lanes: &[usize],
    tie_lane_count: usize,
) -> Vec<f64> {
    if layers.is_empty() {
        return Vec::new();
    }
    let candidates: Vec<Vec<f64>> = layers
        .iter()
        .zip(lanes)
        .zip(lane_counts)
        .zip(tie_lanes)
        .map(|(((intervals, &lane), &lane_count), &tie_lane)| {
            intervals
                .iter()
                .copied()
                .map(|interval| {
                    crossing_track_y(interval, lane, lane_count, tie_lane, tie_lane_count)
                })
                .collect()
        })
        .collect();
    let mut costs: Vec<f64> = candidates[0]
        .iter()
        .map(|&y| {
            (source_y - y).abs()
                + CROSSING_ALIGNMENT_WEIGHT
                    * (y - preferred_crossing_y(source_y, target_y, 0, candidates.len())).abs()
        })
        .collect();
    let mut predecessors = Vec::with_capacity(candidates.len().saturating_sub(1));
    for (layer_index, layer) in candidates.iter().enumerate().skip(1) {
        let previous = &candidates[predecessors.len()];
        let (mut next_costs, layer_predecessors) = distance_transform(previous, &costs, layer);
        let preferred = preferred_crossing_y(source_y, target_y, layer_index, candidates.len());
        for (cost, &y) in next_costs.iter_mut().zip(layer) {
            *cost += CROSSING_ALIGNMENT_WEIGHT * (y - preferred).abs();
        }
        costs = next_costs;
        predecessors.push(layer_predecessors);
    }
    let last = candidates.last().expect("crossing path has a layer");
    let mut selected = last
        .iter()
        .enumerate()
        .map(|(index, &y)| (index, costs[index] + (y - target_y).abs()))
        .min_by(|(left_index, left), (right_index, right)| {
            left.total_cmp(right).then(left_index.cmp(right_index))
        })
        .map(|(index, _)| index)
        .expect("crossing layers contain free intervals");
    let mut result = vec![0.0; candidates.len()];
    for layer in (0..candidates.len()).rev() {
        result[layer] = candidates[layer][selected];
        if layer > 0 {
            selected = predecessors[layer - 1][selected];
        }
    }
    result
}

fn preferred_crossing_y(source: f64, target: f64, index: usize, count: usize) -> f64 {
    let progress = (index + 1) as f64 / (count + 1) as f64;
    source + (target - source) * progress
}

fn distance_transform(previous: &[f64], costs: &[f64], current: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut prefix = Vec::with_capacity(previous.len());
    let mut best = 0usize;
    for index in 0..previous.len() {
        let candidate = costs[index] - previous[index];
        let current_best = costs[best] - previous[best];
        if candidate.total_cmp(&current_best).is_lt() {
            best = index;
        }
        prefix.push(best);
    }
    let mut suffix = vec![0usize; previous.len()];
    best = previous.len() - 1;
    for index in (0..previous.len()).rev() {
        let candidate = costs[index] + previous[index];
        let current_best = costs[best] + previous[best];
        if candidate.total_cmp(&current_best).is_lt() || (candidate == current_best && index < best)
        {
            best = index;
        }
        suffix[index] = best;
    }

    let mut next_costs = Vec::with_capacity(current.len());
    let mut predecessors = Vec::with_capacity(current.len());
    for &y in current {
        let split = previous.partition_point(|&previous_y| previous_y <= y);
        let left = (split > 0).then(|| {
            let index = prefix[split - 1];
            (index, costs[index] + y - previous[index])
        });
        let right = (split < previous.len()).then(|| {
            let index = suffix[split];
            (index, costs[index] + previous[index] - y)
        });
        let (predecessor, cost) = match (left, right) {
            (Some(left), Some(right)) => {
                if left.1.total_cmp(&right.1).is_lt() || (left.1 == right.1 && left.0 < right.0) {
                    left
                } else {
                    right
                }
            }
            (Some(left), None) => left,
            (None, Some(right)) => right,
            (None, None) => unreachable!("crossing layers contain free intervals"),
        };
        next_costs.push(cost);
        predecessors.push(predecessor);
    }
    (next_costs, predecessors)
}

fn stub_point(point: Point, side: PortSide, distance: f64) -> Point {
    match side {
        PortSide::North => Point {
            x: point.x,
            y: point.y - distance,
        },
        PortSide::East => Point {
            x: point.x + distance,
            y: point.y,
        },
        PortSide::South => Point {
            x: point.x,
            y: point.y + distance,
        },
        PortSide::West => Point {
            x: point.x - distance,
            y: point.y,
        },
    }
}

fn port_point(node: &NodeGeometry, port: &Port) -> Point {
    match port.side {
        PortSide::North => Point {
            x: node.x + port.offset,
            y: node.y,
        },
        PortSide::East => Point {
            x: node.x + node.width,
            y: node.y + port.offset,
        },
        PortSide::South => Point {
            x: node.x + port.offset,
            y: node.y + node.height,
        },
        PortSide::West => Point {
            x: node.x,
            y: node.y + port.offset,
        },
    }
}

#[allow(clippy::too_many_arguments)]
fn channel_point(
    point: Point,
    node: &NodeGeometry,
    side: PortSide,
    rank: usize,
    edge_index: usize,
    edge_count: usize,
    layer_left: &[f64],
    layer_right: &[f64],
    options: LayoutOptions,
) -> Point {
    let lane_fraction = (edge_index + 1) as f64 / (edge_count + 1) as f64;
    let fraction = 0.35 + 0.15 * lane_fraction;
    let west_limit = if rank == 0 {
        layer_left[rank] - options.layer_gap
    } else {
        layer_right[rank - 1]
    };
    let east_limit = if rank + 1 == layer_left.len() {
        layer_right[rank] + options.layer_gap
    } else {
        layer_left[rank + 1]
    };
    let west_x = west_limit + (layer_left[rank] - west_limit) * fraction;
    let east_x = layer_right[rank] + (east_limit - layer_right[rank]) * fraction;
    match side {
        PortSide::East => Point {
            x: east_x,
            y: point.y,
        },
        PortSide::West => Point {
            x: west_x,
            y: point.y,
        },
        PortSide::North => Point {
            x: if point.x <= node.x + node.width / 2.0 {
                west_x
            } else {
                east_x
            },
            y: point.y,
        },
        PortSide::South => Point {
            x: if point.x <= node.x + node.width / 2.0 {
                west_x
            } else {
                east_x
            },
            y: point.y,
        },
    }
}

fn push_point(points: &mut Vec<Point>, point: Point) {
    if points.last() == Some(&point) {
        return;
    }
    if points.len() >= 2 {
        let before = points[points.len() - 2];
        let last = points[points.len() - 1];
        if (before.x == last.x && last.x == point.x) || (before.y == last.y && last.y == point.y) {
            *points.last_mut().expect("point exists") = point;
            return;
        }
    }
    points.push(point);
}

#[cfg(test)]
mod tests {
    use std::collections::{BTreeMap, BTreeSet};

    use crate::{
        Edge, EdgeGeometry, Endpoint, Graph, LayoutOptions, Node, NodeGeometry, Point, Port,
        PortSide, validation::validate_and_index,
    };

    use super::{
        FULL_OUTER_LANE_ROUNDS, GapNetAccess, MAX_CROSSING_REPAIR_NODES,
        MAX_CROSSING_REPAIR_PATH_STATES, MAX_CROSSING_REPAIR_ROUTE_POINTS, MIN_CROSSING_REPAIR_NET,
        MIN_CROSSING_REPAIR_TOTAL, OuterNetAccess, OuterSide, RoutingPlan,
        crossing_aware_gap_lane_indices, crossing_aware_outer_lane_indices,
        crossing_repair_within_budget, crossing_track_y, distance_transform,
        has_split_feedback_net, horizontal_crossing_counts_by_net, move_net_to_outer_lane,
        outer_channel_lane_indices, outer_lane_assignments, port_point, route_edges,
        route_planned_candidates, route_planned_edges, route_quality, route_quality_cmp,
        route_quality_for_plan, route_supplemental_edges, select_crossing_repair_net,
        shortest_crossing_path, sparse_channel_route, sum_within_limit,
        vertical_horizontal_crossings,
    };

    #[test]
    fn zero_length_vertical_access_has_no_crossings() {
        assert_eq!(
            vertical_horizontal_crossings(&[(20.0, 20.0)], &[10.0, 20.0, 30.0]),
            0
        );
    }

    #[test]
    fn outer_channels_put_dominant_fanout_nearest_its_targets() {
        let nets = BTreeSet::from([1, 2, 3]);

        assert_eq!(
            outer_channel_lane_indices(&nets, &BTreeMap::from([(1, 15), (2, 1), (3, 2)])),
            BTreeMap::from([(1, 0), (2, 1), (3, 2)]),
        );
        assert_eq!(
            outer_channel_lane_indices(&nets, &BTreeMap::from([(1, 16), (2, 1), (3, 16)])),
            BTreeMap::from([(1, 1), (2, 0), (3, 2)]),
        );
    }

    #[test]
    fn gap_lane_transpose_uses_predicted_crossing_cost() {
        let current = BTreeMap::from([(1, 0), (2, 1)]);
        let accesses = BTreeMap::from([
            (
                1,
                GapNetAccess {
                    vertical: vec![(0.0, 10.0)],
                    left_y: Vec::new(),
                    right_y: Vec::new(),
                },
            ),
            (
                2,
                GapNetAccess {
                    vertical: vec![(20.0, 30.0)],
                    left_y: vec![5.0],
                    right_y: Vec::new(),
                },
            ),
        ]);

        let lanes = crossing_aware_gap_lane_indices(&current, &accesses);

        assert_eq!(lanes[&2], 0);
        assert_eq!(lanes[&1], 1);
    }

    #[test]
    fn hot_net_move_preserves_lane_permutations_and_uses_the_outer_edge() {
        let current = vec![
            BTreeMap::from([(1, 1), (2, 0), (3, 2)]),
            BTreeMap::from([(1, 0), (2, 2), (3, 1)]),
            BTreeMap::from([(1, 2), (2, 1), (3, 0), (4, 3)]),
        ];

        let moved = move_net_to_outer_lane(&current, 2).unwrap();

        assert_eq!(moved[0], BTreeMap::from([(1, 0), (2, 2), (3, 1)]));
        assert_eq!(moved[1], current[1]);
        assert_eq!(moved[2], BTreeMap::from([(1, 1), (2, 3), (3, 0), (4, 2)]));
        for (before, after) in current.iter().zip(&moved) {
            assert_eq!(
                before.keys().collect::<Vec<_>>(),
                after.keys().collect::<Vec<_>>()
            );
            let mut lanes = after.values().copied().collect::<Vec<_>>();
            lanes.sort_unstable();
            assert_eq!(lanes, (0..after.len()).collect::<Vec<_>>());
        }
        assert!(move_net_to_outer_lane(&moved, 2).is_none());
    }

    #[test]
    fn repair_selector_honors_thresholds_ties_and_movable_runner_up() {
        let lanes = vec![BTreeMap::from([(1, 2), (2, 0), (3, 1)])];
        let counts = BTreeMap::from([
            (1, MIN_CROSSING_REPAIR_NET + 20),
            (2, MIN_CROSSING_REPAIR_NET),
            (3, MIN_CROSSING_REPAIR_NET),
        ]);

        assert_eq!(
            select_crossing_repair_net(MIN_CROSSING_REPAIR_TOTAL, &counts, &lanes),
            Some(2)
        );
        assert_eq!(
            select_crossing_repair_net(MIN_CROSSING_REPAIR_TOTAL - 1, &counts, &lanes),
            None
        );
        assert_eq!(
            select_crossing_repair_net(
                MIN_CROSSING_REPAIR_TOTAL,
                &BTreeMap::from([(2, MIN_CROSSING_REPAIR_NET - 1)]),
                &lanes,
            ),
            None
        );
    }

    #[test]
    fn repair_work_sum_is_inclusive_and_overflow_safe() {
        assert!(sum_within_limit(
            [MAX_CROSSING_REPAIR_ROUTE_POINTS].into_iter(),
            MAX_CROSSING_REPAIR_ROUTE_POINTS,
        ));
        assert!(!sum_within_limit(
            [MAX_CROSSING_REPAIR_ROUTE_POINTS, 1].into_iter(),
            MAX_CROSSING_REPAIR_ROUTE_POINTS,
        ));
        assert!(!sum_within_limit([usize::MAX, 1].into_iter(), usize::MAX,));
    }

    #[test]
    fn repair_budget_enforces_node_and_actual_path_state_boundaries() {
        let empty_routes = Vec::<EdgeGeometry>::new();
        let empty_lanes = Vec::<BTreeMap<u32, usize>>::new();
        let empty_spans = Vec::<Option<(usize, usize)>>::new();
        let empty_free = Vec::<Vec<(f64, f64)>>::new();
        assert!(crossing_repair_within_budget(
            MAX_CROSSING_REPAIR_NODES,
            0,
            &empty_routes,
            &empty_lanes,
            &empty_spans,
            &empty_free,
        ));
        assert!(!crossing_repair_within_budget(
            MAX_CROSSING_REPAIR_NODES + 1,
            0,
            &empty_routes,
            &empty_lanes,
            &empty_spans,
            &empty_free,
        ));

        let spans = vec![Some((0, 2))];
        let mut free_by_rank = vec![
            Vec::new(),
            vec![(0.0, 1.0); MAX_CROSSING_REPAIR_PATH_STATES + 1],
            Vec::new(),
        ];
        assert!(!crossing_repair_within_budget(
            0,
            1,
            &empty_routes,
            &empty_lanes,
            &spans,
            &free_by_rank,
        ));
        free_by_rank[1].pop();
        assert!(crossing_repair_within_budget(
            0,
            1,
            &empty_routes,
            &empty_lanes,
            &spans,
            &free_by_rank,
        ));
    }

    #[test]
    fn supplemental_routing_generates_and_exactly_scores_a_crossing_repair() {
        const SIDE: u32 = 70;
        let graph = Graph {
            nodes: (0..SIDE * 2)
                .map(|id| Node {
                    id,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: if id < SIDE {
                            PortSide::East
                        } else {
                            PortSide::West
                        },
                        offset: 5.0,
                    }],
                })
                .collect(),
            edges: (0..SIDE)
                .map(|id| Edge {
                    id,
                    source: Endpoint { node: id, port: 0 },
                    target: Endpoint {
                        node: SIDE * 2 - 1 - id,
                        port: 0,
                    },
                    net: id,
                    participates_in_ranking: true,
                })
                .collect(),
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let ranks = (0..SIDE * 2)
            .map(|id| usize::from(id >= SIDE))
            .collect::<Vec<_>>();
        let nodes = (0..SIDE * 2)
            .map(|id| NodeGeometry {
                id,
                x: if id < SIDE { 0.0 } else { 200.0 },
                y: f64::from(id % SIDE) * 30.0,
                width: 10.0,
                height: 10.0,
            })
            .collect::<Vec<_>>();
        let plan = RoutingPlan::new(&indexed, &ranks);

        let routed = route_planned_candidates(&plan, &nodes, LayoutOptions::default(), true);
        let primary_quality = routed.primary_quality.unwrap();
        let (repair_quality, repair) = routed.repair.expect("fixture activates repair");
        let exact_primary = route_quality(&indexed, &routed.primary);
        let exact_repair = route_quality(&indexed, &repair);

        assert!(primary_quality.crossings >= MIN_CROSSING_REPAIR_TOTAL);
        assert_eq!(primary_quality.crossings, exact_primary.crossings);
        assert_eq!(primary_quality.bends, exact_primary.bends);
        assert_eq!(primary_quality.route_length, exact_primary.route_length);
        assert_eq!(repair_quality.crossings, exact_repair.crossings);
        assert_eq!(repair_quality.bends, exact_repair.bends);
        assert_eq!(repair_quality.route_length, exact_repair.route_length);
        for routes in [&routed.primary, &repair] {
            assert_eq!(routes.len(), graph.edges.len());
            for (edge, route) in graph.edges.iter().zip(routes) {
                assert_eq!(route.id, edge.id);
                assert_eq!(
                    route.points.first(),
                    Some(&port_point(
                        &nodes[edge.source.node as usize],
                        &graph.nodes[edge.source.node as usize].ports[0]
                    ))
                );
                assert_eq!(
                    route.points.last(),
                    Some(&port_point(
                        &nodes[edge.target.node as usize],
                        &graph.nodes[edge.target.node as usize].ports[0]
                    ))
                );
                assert!(
                    route.points.windows(2).all(|points| {
                        (points[0].x == points[1].x) ^ (points[0].y == points[1].y)
                    })
                );
            }
        }

        let mut permuted_graph = graph.clone();
        permuted_graph.nodes.reverse();
        permuted_graph.edges.reverse();
        let permuted_indexed =
            validate_and_index(&permuted_graph, LayoutOptions::default()).unwrap();
        let permuted_plan = RoutingPlan::new(&permuted_indexed, &ranks);
        let permuted =
            route_planned_candidates(&permuted_plan, &nodes, LayoutOptions::default(), true);
        let (permuted_quality, permuted_repair) =
            permuted.repair.expect("permuted fixture activates repair");

        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted_quality.crossings, repair_quality.crossings);
        assert_eq!(permuted_quality.bends, repair_quality.bends);
        assert_eq!(permuted_quality.route_length, repair_quality.route_length);
        assert_eq!(permuted_repair, repair);
    }

    #[test]
    fn gap_lane_transpose_can_move_a_net_across_more_than_sixteen_lanes() {
        let current = (0..18).map(|net| (net, net as usize)).collect();
        let mut accesses = (0..17)
            .map(|net| {
                (
                    net,
                    GapNetAccess {
                        vertical: vec![(0.0, 10.0)],
                        left_y: Vec::new(),
                        right_y: Vec::new(),
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();
        accesses.insert(
            17,
            GapNetAccess {
                vertical: vec![(20.0, 30.0)],
                left_y: vec![5.0],
                right_y: Vec::new(),
            },
        );

        let lanes = crossing_aware_gap_lane_indices(&current, &accesses);

        assert_eq!(lanes[&17], 0);
    }

    #[test]
    fn outer_lane_transpose_uses_predicted_crossing_cost() {
        let nets = BTreeSet::from([1, 2]);
        let accesses = BTreeMap::from([
            (
                1,
                OuterNetAccess {
                    horizontal: vec![(0.0, 10.0)],
                    vertical_x: vec![20.0],
                },
            ),
            (
                2,
                OuterNetAccess {
                    horizontal: vec![(20.0, 30.0)],
                    vertical_x: vec![5.0],
                },
            ),
        ]);

        let lanes = crossing_aware_outer_lane_indices(&nets, &accesses);

        assert_eq!(lanes[&2], 0);
        assert_eq!(lanes[&1], 1);
    }

    #[test]
    fn multi_terminal_outer_net_can_branch_above_and_below() {
        let node = |id| Node {
            id,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side: if id == 1 {
                    PortSide::East
                } else {
                    PortSide::West
                },
                offset: 10.0,
            }],
        };
        let graph = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
            ],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let geometry = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 50.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 3,
                x: 100.0,
                y: 100.0,
                width: 20.0,
                height: 20.0,
            },
        ];

        let plan = RoutingPlan::new(&indexed, &[0, 1, 1]);
        let lanes = outer_lane_assignments(
            &plan,
            &geometry,
            &[0, 1, 1],
            &[None, None],
            &BTreeSet::from([7]),
            &[0.0, 100.0],
            &[20.0, 120.0],
            0.0,
            120.0,
            LayoutOptions::default(),
            FULL_OUTER_LANE_ROUNDS,
            false,
        );

        assert_eq!(lanes[&10].side, OuterSide::Top);
        assert_eq!(lanes[&11].side, OuterSide::Bottom);
        assert_eq!(lanes[&10].channel_index, lanes[&11].channel_index);
    }

    #[test]
    fn feedback_net_uses_one_coherent_outer_side() {
        let node = |id| Node {
            id,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side: if id == 1 {
                    PortSide::East
                } else {
                    PortSide::West
                },
                offset: 10.0,
            }],
        };
        let graph = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 7,
                    participates_in_ranking: false,
                },
            ],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let geometry = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 50.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 3,
                x: 100.0,
                y: 100.0,
                width: 20.0,
                height: 20.0,
            },
        ];

        let plan = RoutingPlan::new(&indexed, &[0, 1, 1]);
        let split_lanes = outer_lane_assignments(
            &plan,
            &geometry,
            &[0, 1, 1],
            &[None, None],
            &BTreeSet::from([7]),
            &[0.0, 100.0],
            &[20.0, 120.0],
            0.0,
            120.0,
            LayoutOptions::default(),
            FULL_OUTER_LANE_ROUNDS,
            false,
        );
        assert!(has_split_feedback_net(&plan, &[None, None], &split_lanes));
        let lanes = outer_lane_assignments(
            &plan,
            &geometry,
            &[0, 1, 1],
            &[None, None],
            &BTreeSet::from([7]),
            &[0.0, 100.0],
            &[20.0, 120.0],
            0.0,
            120.0,
            LayoutOptions::default(),
            FULL_OUTER_LANE_ROUNDS,
            true,
        );

        assert_eq!(lanes[&10].side, OuterSide::Top);
        assert_eq!(lanes[&11].side, OuterSide::Top);
        assert_eq!(lanes[&10].side_index, lanes[&11].side_index);
        assert!(!has_split_feedback_net(&plan, &[None, None], &lanes));
    }

    fn feedback_candidate_fixture(
        source_a: f64,
        source_b: f64,
        targets: [f64; 4],
        extra_nodes: usize,
    ) -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let make_node = |id, cycle_breaker, source| Node {
            id,
            width: 20.0,
            height: 20.0,
            cycle_breaker,
            ports: if source {
                vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 10.0,
                    },
                ]
            } else {
                vec![Port {
                    id: 0,
                    side: if id == 0 {
                        PortSide::East
                    } else {
                        PortSide::West
                    },
                    offset: 10.0,
                }]
            },
        };
        let mut nodes = vec![
            make_node(0, false, false),
            make_node(1, false, true),
            make_node(2, false, true),
            make_node(3, true, false),
            make_node(4, true, false),
            make_node(5, true, false),
            make_node(6, true, false),
        ];
        nodes.extend((0..extra_nodes).map(|offset| Node {
            id: 7 + offset as u32,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: Vec::new(),
        }));
        let graph = Graph {
            nodes,
            edges: vec![
                Edge {
                    id: 0,
                    source: Endpoint { node: 0, port: 0 },
                    target: Endpoint { node: 1, port: 0 },
                    net: 100,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 1,
                    source: Endpoint { node: 0, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 101,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 4, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 12,
                    source: Endpoint { node: 2, port: 1 },
                    target: Endpoint { node: 5, port: 0 },
                    net: 8,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 13,
                    source: Endpoint { node: 2, port: 1 },
                    target: Endpoint { node: 6, port: 0 },
                    net: 8,
                    participates_in_ranking: true,
                },
            ],
        };
        let mut geometry = vec![
            NodeGeometry {
                id: 0,
                x: 0.0,
                y: 60.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 1,
                x: 100.0,
                y: source_a,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: source_b,
                width: 20.0,
                height: 20.0,
            },
        ];
        geometry.extend(
            targets
                .into_iter()
                .enumerate()
                .map(|(offset, y)| NodeGeometry {
                    id: 3 + offset as u32,
                    x: 0.0,
                    y,
                    width: 20.0,
                    height: 20.0,
                }),
        );
        geometry.extend((0..extra_nodes).map(|offset| NodeGeometry {
            id: 7 + offset as u32,
            x: 200.0,
            y: (offset % 7) as f64 * 20.0,
            width: 20.0,
            height: 20.0,
        }));
        let mut ranks = vec![0, 1, 1, 0, 0, 0, 0];
        ranks.extend(std::iter::repeat_n(2, extra_nodes));
        (graph, geometry, ranks)
    }

    fn route_feedback_fixture(
        graph: &Graph,
        geometry: &[NodeGeometry],
        ranks: &[usize],
    ) -> super::RoutedEdges {
        let indexed = validate_and_index(graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, ranks);
        route_planned_candidates(&plan, geometry, LayoutOptions::default(), false)
    }

    #[test]
    fn production_feedback_candidate_uses_inferred_cycle_cuts_and_is_deterministic() {
        let (graph, geometry, ranks) =
            feedback_candidate_fixture(0.0, 40.0, [0.0, 120.0, 20.0, 100.0], 0);
        assert!(graph.edges.iter().all(|edge| edge.participates_in_ranking));
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        assert!(
            plan.edges
                .iter()
                .filter(|resolved| resolved.edge.id >= 10)
                .all(|resolved| !resolved.participates_in_ranking)
        );

        let routed = route_planned_candidates(&plan, &geometry, LayoutOptions::default(), false);
        let (baseline_quality, baseline) = routed
            .feedback_trace
            .baseline
            .as_ref()
            .expect("split feedback evaluates a bounded alternative");
        let candidate_quality = routed.feedback_trace.candidate_quality.unwrap();
        assert!(routed.feedback_trace.split);
        assert!(routed.feedback_trace.evaluated);
        assert!(routed.feedback_trace.selected);
        assert!(route_quality_cmp(candidate_quality, *baseline_quality).is_lt());
        assert_eq!(routed.primary_quality, Some(candidate_quality));
        assert_ne!(&routed.primary, baseline);

        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let permuted = route_feedback_fixture(&permuted, &geometry, &ranks);
        assert!(permuted.feedback_trace.selected);
        assert_eq!(routed.primary, permuted.primary);
        assert_eq!(routed.primary_quality, permuted.primary_quality);
    }

    #[test]
    fn production_feedback_candidate_retains_exact_baseline_when_not_better() {
        let (graph, geometry, ranks) =
            feedback_candidate_fixture(0.0, 20.0, [0.0, 100.0, 40.0, 120.0], 0);
        let routed = route_feedback_fixture(&graph, &geometry, &ranks);
        let (baseline_quality, baseline) = routed
            .feedback_trace
            .baseline
            .as_ref()
            .expect("split feedback evaluates a bounded alternative");
        let candidate_quality = routed.feedback_trace.candidate_quality.unwrap();

        assert!(routed.feedback_trace.split);
        assert!(routed.feedback_trace.evaluated);
        assert!(!routed.feedback_trace.selected);
        assert!(!route_quality_cmp(candidate_quality, *baseline_quality).is_lt());
        assert_eq!(routed.primary_quality, Some(*baseline_quality));
        assert_eq!(&routed.primary, baseline);
    }

    #[test]
    fn production_feedback_candidate_skips_over_budget_graph() {
        let (graph, geometry, ranks) = feedback_candidate_fixture(
            0.0,
            40.0,
            [0.0, 120.0, 20.0, 100.0],
            MAX_CROSSING_REPAIR_NODES + 1 - 7,
        );
        let routed = route_feedback_fixture(&graph, &geometry, &ranks);

        assert!(routed.feedback_trace.split);
        assert!(!routed.feedback_trace.evaluated);
        assert!(!routed.feedback_trace.selected);
        assert!(routed.feedback_trace.baseline.is_none());
        assert!(routed.feedback_trace.candidate_quality.is_none());
    }

    #[test]
    fn routing_plan_matches_fresh_preparation_and_does_not_retain_candidate_state() {
        let node = |id, side| Node {
            id,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side,
                offset: 10.0,
            }],
        };
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::West),
                node(3, PortSide::West),
            ],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
            ],
        };
        let options = LayoutOptions::default();
        let indexed = validate_and_index(&graph, options).unwrap();
        let ranks = [0, 1, 1];
        let candidate_a = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 50.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 3,
                x: 100.0,
                y: 100.0,
                width: 20.0,
                height: 20.0,
            },
        ];
        let mut candidate_b = candidate_a.clone();
        candidate_b[0].y = 140.0;
        candidate_b[1].y = 120.0;
        candidate_b[2].y = 10.0;
        let plan = RoutingPlan::new(&indexed, &ranks);

        let full_a = route_planned_edges(&plan, &candidate_a, options, false);
        assert_eq!(full_a, route_edges(&indexed, &candidate_a, &ranks, options));
        assert_eq!(
            route_planned_edges(&plan, &candidate_a, options, true),
            route_supplemental_edges(&indexed, &candidate_a, &ranks, options)
        );

        let _full_b = route_planned_edges(&plan, &candidate_b, options, false);
        assert_eq!(
            route_planned_edges(&plan, &candidate_a, options, false),
            full_a
        );
    }

    #[test]
    fn crossing_tie_breaker_separates_coincident_tracks() {
        let first = crossing_track_y((0.0, 4.0), 0, 3, 0, 2);
        let second = crossing_track_y((0.0, 2.0), 1, 3, 1, 2);

        assert_ne!(first, second);
        assert!(first > 0.0 && first < 4.0);
        assert!(second > 0.0 && second < 2.0);
    }

    #[test]
    fn near_equal_endpoint_tracks_use_an_orthogonal_detour() {
        let source = Point {
            x: 20.0,
            y: 840.000_000_000_000_1,
        };
        let target = Point { x: 100.0, y: 840.0 };
        let points = sparse_channel_route(
            7,
            source,
            target,
            Endpoint { node: 1, port: 0 },
            Endpoint { node: 2, port: 0 },
            0,
            1,
            &[0.0, 100.0],
            &[20.0, 120.0],
            &[BTreeMap::from([(7, 0)])],
            &[],
            &BTreeMap::new(),
            10.0,
        );

        assert_eq!(points.first(), Some(&source));
        assert_eq!(points.last(), Some(&target));
        assert!(points.windows(2).all(|pair| {
            (pair[0].x == pair[1].x || pair[0].y == pair[1].y)
                && (pair[0].x != pair[1].x || pair[0].y != pair[1].y)
        }));
    }

    #[test]
    fn shortest_path_keeps_a_consistent_free_corridor() {
        let path = shortest_crossing_path(
            &[
                vec![(0.0, 10.0), (90.0, 100.0)],
                vec![(0.0, 10.0), (90.0, 100.0)],
                vec![(0.0, 10.0), (90.0, 100.0)],
            ],
            5.0,
            5.0,
            &[0, 0, 0],
            &[1, 1, 1],
            &[0, 1, 2],
            3,
        );

        assert!(path.iter().all(|&y| y < 10.0));
    }

    #[test]
    fn linear_distance_transform_matches_exhaustive_costs() {
        let previous = [0.0, 10.0, 30.0];
        let costs = [4.0, 0.0, 2.0];
        let current = [5.0, 20.0, 40.0];
        let (actual_costs, actual_predecessors) = distance_transform(&previous, &costs, &current);

        let expected: Vec<_> = current
            .iter()
            .map(|&y| {
                previous
                    .iter()
                    .enumerate()
                    .map(|(index, &before)| (index, costs[index] + f64::abs(before - y)))
                    .min_by(|left, right| left.1.total_cmp(&right.1).then(left.0.cmp(&right.0)))
                    .unwrap()
            })
            .collect();
        assert_eq!(
            actual_costs,
            expected.iter().map(|item| item.1).collect::<Vec<_>>()
        );
        assert_eq!(
            actual_predecessors,
            expected.iter().map(|item| item.0).collect::<Vec<_>>()
        );
    }

    #[test]
    fn physical_quality_merges_shared_net_geometry_and_excludes_related_crossings() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 1,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 5.0,
                    }],
                },
                Node {
                    id: 2,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 5.0,
                    }],
                },
            ],
            edges: (0..4)
                .map(|id| Edge {
                    id,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: [1, 2, 1, 3][id as usize],
                    participates_in_ranking: true,
                })
                .collect(),
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let routes = vec![
            EdgeGeometry {
                id: 0,
                points: vec![Point { x: 0.0, y: 5.0 }, Point { x: 10.0, y: 5.0 }],
            },
            EdgeGeometry {
                id: 1,
                points: vec![Point { x: 5.0, y: 0.0 }, Point { x: 5.0, y: 10.0 }],
            },
            EdgeGeometry {
                id: 2,
                points: vec![Point { x: 0.0, y: 5.0 }, Point { x: 10.0, y: 5.0 }],
            },
            EdgeGeometry {
                id: 3,
                points: vec![
                    Point { x: 20.0, y: 0.0 },
                    Point { x: 25.0, y: 0.0 },
                    Point { x: 25.0, y: 5.0 },
                ],
            },
        ];

        let quality = route_quality(&indexed, &routes);
        let plan = RoutingPlan::new(&indexed, &[0, 1]);
        let (counts, attributed) = horizontal_crossing_counts_by_net(&plan, &routes);
        let planned = route_quality_for_plan(&plan, &routes);

        assert_eq!(quality.crossings, 0);
        assert_eq!(quality.bends, 1);
        assert_eq!(quality.route_length, 30.0);
        assert!(counts.is_empty());
        assert_eq!(attributed.crossings, quality.crossings);
        assert_eq!(attributed.bends, quality.bends);
        assert_eq!(attributed.route_length, quality.route_length);
        assert_eq!(planned.crossings, quality.crossings);
        assert_eq!(planned.bends, quality.bends);
        assert_eq!(planned.route_length, quality.route_length);
    }

    #[test]
    fn physical_quality_counts_crossings_between_unrelated_edges() {
        let graph = Graph {
            nodes: (1..=4)
                .map(|id| Node {
                    id,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 5.0,
                    }],
                })
                .collect(),
            edges: vec![
                Edge {
                    id: 0,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 1,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 1,
                    source: Endpoint { node: 3, port: 0 },
                    target: Endpoint { node: 4, port: 0 },
                    net: 2,
                    participates_in_ranking: true,
                },
            ],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let routes = vec![
            EdgeGeometry {
                id: 0,
                points: vec![Point { x: 0.0, y: 5.0 }, Point { x: 10.0, y: 5.0 }],
            },
            EdgeGeometry {
                id: 1,
                points: vec![Point { x: 5.0, y: 0.0 }, Point { x: 5.0, y: 10.0 }],
            },
        ];

        let quality = route_quality(&indexed, &routes);
        let plan = RoutingPlan::new(&indexed, &[0, 1, 0, 1]);
        let (counts, attributed) = horizontal_crossing_counts_by_net(&plan, &routes);
        let planned = route_quality_for_plan(&plan, &routes);

        assert_eq!(quality.crossings, 1);
        assert_eq!(quality.bends, 0);
        assert_eq!(quality.route_length, 20.0);
        assert_eq!(counts.values().sum::<usize>(), quality.crossings);
        assert_eq!(counts, BTreeMap::from([(1, 1)]));
        assert_eq!(attributed.crossings, quality.crossings);
        assert_eq!(attributed.bends, quality.bends);
        assert_eq!(attributed.route_length, quality.route_length);
        assert_eq!(planned.crossings, quality.crossings);
        assert_eq!(planned.bends, quality.bends);
        assert_eq!(planned.route_length, quality.route_length);
    }
}
