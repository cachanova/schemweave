use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, HashMap, HashSet, hash_map::Entry},
};

#[cfg(test)]
use std::cell::Cell;

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
// Supplemental placements reuse the same bounded adjacent descent. Give them the same search
// budgets as the baseline; both searches still stop immediately when a pass is idle.
const SUPPLEMENTAL_OUTER_LANE_ROUNDS: usize = 16;
const SUPPLEMENTAL_GAP_LANE_ROUNDS: usize = 32;
// The global seed is quadratic in the lanes of one gap and emits one complete exact-scored route
// alternative. Bound both dimensions: small gaps only, and enough aggregate predicted crossings
// removed to amortize that second route family on measured large sparse graphs.
const MAX_GLOBAL_GAP_LANES: usize = 32;
const MAX_LARGE_GLOBAL_GAP_LANES: usize = 705;
const MAX_LARGE_GLOBAL_GAP_HOT_NETS: usize = 32;
// Preserve the first Max family exactly, then admit one deeper family. The public comparator
// exact-scores every complete route set, so proxy improvement cannot regress canonical quality.
const MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_NETS: usize = 64;
const MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS: usize = 2;
const MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS: usize = 256;
const MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS: usize = 5;
const MIN_GLOBAL_GAP_ORDER_GAIN: usize = 256;
// Aggregate caps bound the pair table and vertical-access comparisons across every eligible gap;
// both measured 2,000-node winners remain below these limits.
const MAX_GLOBAL_GAP_PAIRS: usize = 32_768;
const MAX_GLOBAL_GAP_ACCESS_WORK: usize = 500_000;
const MAX_LARGE_GLOBAL_GAP_PAIRS: usize = 262_144;
const MAX_LARGE_GLOBAL_GAP_ACCESS_WORK: usize = 2_000_000;
// Admit exactly two maximum-size refined gaps after charging directional precompute plus every
// linear locate/remove/gather/fold/walk/insert pass in every configured round.
const MAX_REFINED_LARGE_GLOBAL_GAP_LANE_WORK: usize = MAX_LARGE_GLOBAL_GAP_LANES
    * (MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS * (2 + MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS * 6)
        + MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_NETS * 6
        + 2)
    * 2;
const MIN_CROSSING_REPAIR_TOTAL: usize = 500;
const MIN_CROSSING_REPAIR_NET: usize = 64;
// Move a bounded hot-net block before the existing single rebuild and exact score. Two captures
// the measured quality knee without adding another complete routing/scoring pass.
const MAX_BATCHED_CROSSING_REPAIR_NETS: usize = 2;
// Max may evaluate one deeper repair family, but only when a third or fourth movable hot net
// exists. Otherwise the candidate is byte-identical to the existing two-net repair and a second
// complete route emission plus exact score would be wasted work.
const MAX_DEEP_CROSSING_REPAIR_NETS: usize = 4;
// Outer arm prediction is only a selector; the complete candidate still passes the exact scorer.
// Require the same visible per-net gain as sparse repair and move at most two whole nets in one
// bounded rebuild.
const MIN_OUTER_SIDE_REPAIR_GAIN: usize = 64;
const MAX_BATCHED_OUTER_SIDE_REPAIRS: usize = 2;
const MAX_CROSSING_REPAIR_EDGES: usize = 10_000;
const MAX_CROSSING_REPAIR_NODES: usize = 2_000;
const MAX_CROSSING_REPAIR_ROUTE_POINTS: usize = 100_000;
const MAX_CROSSING_REPAIR_LANE_MEMBERSHIPS: usize = 100_000;
const MAX_CROSSING_REPAIR_PATH_STATES: usize = 500_000;
// Small nets keep the historical stable-ID order. For a single-source outer fanout, higher channel
// indices usually shorten the many sink escapes at the cost of one shared source arm; the exact
// route scorer below still rejects the heuristic whenever that trade is not beneficial.
const MIN_FANOUT_AWARE_CHANNEL_EDGES: usize = 16;
// A channel-order candidate emits and exactly scores a second complete route set. Require enough
// actual outer sink work to amortize that cost; smaller clusters keep the historical fast path.
const MIN_FANOUT_AWARE_OUTER_BRANCHES: usize = 512;
const MIN_FANOUT_AWARE_NODES: usize = 1_000;
// Emitting and exactly scoring a complete route family is only worthwhile when the obstacle-safe
// crossing paths contain enough removable stair steps. Work stays linear in the already-bounded
// sparse path payload, and the candidate is never admitted without the canonical physical score.
const MIN_STAIRCASE_ALIGNMENT_TRANSITIONS: usize = 32;
const MIN_STAIRCASE_ALIGNMENT_RATIO_DENOMINATOR: usize = 9;

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
    pub(crate) alternatives: Vec<(RouteQuality, Vec<EdgeGeometry>)>,
    #[cfg(test)]
    pub(crate) feedback_trace: FeedbackCandidateTrace,
    #[cfg(test)]
    pub(crate) fanout_trace: FanoutCandidateTrace,
    #[cfg(test)]
    pub(crate) repair_nets: Vec<NetId>,
    #[cfg(test)]
    repair_outer_sides: Vec<(NetId, OuterSide)>,
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

#[cfg(test)]
#[derive(Debug)]
pub(crate) struct FanoutCandidateTrace {
    pub(crate) evaluated: bool,
    pub(crate) selected: bool,
    pub(crate) baseline_quality: Option<RouteQuality>,
    pub(crate) candidate_quality: Option<RouteQuality>,
}

struct RoutedLaneState {
    routes: Vec<EdgeGeometry>,
    route_quality: Option<RouteQuality>,
    gap_spacing: GapTrackSpacing,
    gap_lanes: Vec<BTreeMap<u32, usize>>,
    global_gap_lanes: Option<Vec<BTreeMap<u32, usize>>>,
    preserved_refined_global_gap_lanes: Option<Vec<BTreeMap<u32, usize>>>,
    refined_global_gap_lanes: Option<Vec<BTreeMap<u32, usize>>>,
    endpoint_tracks: BTreeMap<(u32, u32, u8), (usize, usize)>,
    crossing_paths_match_endpoint_tracks: bool,
    crossing_paths: Vec<Option<Vec<f64>>>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum GapTrackSpacing {
    Compact,
    Adaptive,
}

struct RouteFamily {
    primary: Vec<EdgeGeometry>,
    primary_quality: RouteQuality,
    repair: Option<(RouteQuality, Vec<EdgeGeometry>)>,
    deeper_repair: Option<(RouteQuality, Vec<EdgeGeometry>)>,
    #[cfg(test)]
    feedback_trace: FeedbackCandidateTrace,
    #[cfg(test)]
    repair_nets: Vec<NetId>,
    #[cfg(test)]
    repair_outer_sides: Vec<(NetId, OuterSide)>,
}

struct CrossingRepair {
    baseline_quality: RouteQuality,
    candidate: Option<(RouteQuality, Vec<EdgeGeometry>)>,
    #[cfg(test)]
    selected_nets: Vec<NetId>,
    #[cfg(test)]
    selected_outer_sides: Vec<(NetId, OuterSide)>,
    #[cfg(test)]
    candidate_lanes_built: bool,
    #[cfg(test)]
    candidate_emitted: bool,
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
        false,
    )
    .primary
}

/// Route an optional layout candidate with bounded lane-refinement work.
///
/// The same deterministic router and validity-preserving construction are used. Supplemental
/// candidates share the baseline lane-refinement caps and may evaluate bounded, exact-scored
/// repair variants. The canonical candidate remains available to the exact layout comparator.
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
        true,
    )
    .primary
}

#[cfg(test)]
pub(crate) fn route_planned_candidates(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    supplemental: bool,
) -> RoutedEdges {
    route_planned_candidates_with_sparse_global(plan, nodes, options, supplemental, false, false)
}

#[cfg(test)]
pub(crate) fn route_planned_candidates_with_sparse_global(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    supplemental: bool,
    sparse_global: bool,
    large_sparse_global: bool,
) -> RoutedEdges {
    route_planned_candidates_with_refined_sparse_global(
        plan,
        nodes,
        options,
        supplemental,
        sparse_global,
        large_sparse_global,
        false,
    )
}

#[cfg(test)]
pub(crate) fn route_planned_candidates_with_refined_sparse_global(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    supplemental: bool,
    sparse_global: bool,
    large_sparse_global: bool,
    refined_large_sparse_global: bool,
) -> RoutedEdges {
    route_planned_candidates_with_quality_options(
        plan,
        nodes,
        options,
        supplemental,
        sparse_global,
        large_sparse_global,
        refined_large_sparse_global,
        false,
        false,
    )
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn route_planned_candidates_with_quality_options(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    supplemental: bool,
    sparse_global: bool,
    large_sparse_global: bool,
    refined_large_sparse_global: bool,
    adaptive_gap_spacing: bool,
    deeper_crossing_repair: bool,
) -> RoutedEdges {
    let (outer_rounds, gap_rounds) = if supplemental {
        (SUPPLEMENTAL_OUTER_LANE_ROUNDS, SUPPLEMENTAL_GAP_LANE_ROUNDS)
    } else {
        (FULL_OUTER_LANE_ROUNDS, FULL_GAP_LANE_ROUNDS)
    };
    let mut routed = route_edges_with_lane_rounds_and_refined_global(
        plan,
        nodes,
        options,
        outer_rounds,
        gap_rounds,
        supplemental,
        supplemental,
        sparse_global,
        large_sparse_global,
        refined_large_sparse_global,
        adaptive_gap_spacing,
        deeper_crossing_repair,
    );
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
#[cfg(test)]
#[inline(never)]
fn route_edges_with_lane_rounds(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    outer_lane_rounds: usize,
    gap_lane_rounds: usize,
    repair_crossings: bool,
    fanout_candidates: bool,
) -> RoutedEdges {
    route_edges_with_lane_rounds_and_global(
        plan,
        nodes,
        options,
        outer_lane_rounds,
        gap_lane_rounds,
        repair_crossings,
        fanout_candidates,
        false,
        false,
    )
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
#[cfg(test)]
fn route_edges_with_lane_rounds_and_global(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    outer_lane_rounds: usize,
    gap_lane_rounds: usize,
    repair_crossings: bool,
    fanout_candidates: bool,
    sparse_global: bool,
    large_sparse_global: bool,
) -> RoutedEdges {
    route_edges_with_lane_rounds_and_refined_global(
        plan,
        nodes,
        options,
        outer_lane_rounds,
        gap_lane_rounds,
        repair_crossings,
        fanout_candidates,
        sparse_global,
        large_sparse_global,
        false,
        false,
        false,
    )
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn route_edges_with_lane_rounds_and_refined_global(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    options: LayoutOptions,
    outer_lane_rounds: usize,
    gap_lane_rounds: usize,
    repair_crossings: bool,
    fanout_candidates: bool,
    sparse_global: bool,
    large_sparse_global: bool,
    refined_large_sparse_global: bool,
    adaptive_gap_spacing: bool,
    deeper_crossing_repair: bool,
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
    let stable_channel_lanes = lane_indices(&outer_nets);
    let baseline_outer_lanes = outer_lane_assignments(
        plan,
        nodes,
        ranks,
        &sparse_spans,
        &stable_channel_lanes,
        &layer_left,
        &layer_right,
        top,
        bottom,
        options,
        outer_lane_rounds,
        false,
    );
    let node_count = plan
        .nodes_by_rank
        .iter()
        .map(Vec::len)
        .try_fold(0usize, usize::checked_add)
        .unwrap_or(usize::MAX);
    let sparse_global = sparse_global
        && route_family_candidate_shape_within_budget(node_count, plan.edges.len(), &sparse_spans);
    let align_staircases = adaptive_gap_spacing && sparse_global;
    let adaptive_gap_spacing = adaptive_gap_spacing
        && route_family_candidate_shape_within_budget(node_count, plan.edges.len(), &sparse_spans);
    let RoutedLaneState {
        mut routes,
        route_quality: spacing_quality,
        gap_spacing,
        gap_lanes,
        global_gap_lanes,
        preserved_refined_global_gap_lanes,
        refined_global_gap_lanes,
        mut endpoint_tracks,
        mut crossing_paths_match_endpoint_tracks,
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
        sparse_global,
        large_sparse_global,
        refined_large_sparse_global,
        adaptive_gap_spacing,
    );
    let staircase_alternative = align_staircases
        .then(|| {
            build_staircase_alignment_alternative(
                plan,
                nodes,
                &sparse_spans,
                &free_by_rank,
                &layer_left,
                &layer_right,
                &gap_lanes,
                &crossing_paths,
                &baseline_outer_lanes,
                top,
                bottom,
                options,
                gap_spacing,
                &routes,
                spacing_quality,
            )
        })
        .flatten();
    let build_sparse_alternative = |candidate_lanes: Vec<BTreeMap<u32, usize>>| {
        if !route_family_candidate_within_budget(node_count, plan.edges.len(), &routes) {
            return None;
        }
        let candidate_endpoint_tracks = build_endpoint_tracks(
            plan,
            nodes,
            ranks,
            &sparse_spans,
            &layer_left,
            &layer_right,
            &candidate_lanes,
            &baseline_outer_lanes,
            options,
            GapTrackSpacing::Compact,
        );
        let candidate_crossing_paths_match_endpoint_tracks =
            crossing_paths_match_endpoint_tracks && candidate_endpoint_tracks == endpoint_tracks;
        let compact_candidate_routes = emit_routes(
            plan,
            nodes,
            &sparse_spans,
            &crossing_paths,
            &layer_left,
            &layer_right,
            &candidate_lanes,
            &candidate_endpoint_tracks,
            &baseline_outer_lanes,
            top,
            bottom,
            options,
            GapTrackSpacing::Compact,
        );
        let adaptive_candidate_routes = (adaptive_gap_spacing
            && candidate_lanes.iter().any(|lanes| lanes.len() > 1))
        .then(|| {
            emit_routes(
                plan,
                nodes,
                &sparse_spans,
                &crossing_paths,
                &layer_left,
                &layer_right,
                &candidate_lanes,
                &candidate_endpoint_tracks,
                &baseline_outer_lanes,
                top,
                bottom,
                options,
                GapTrackSpacing::Adaptive,
            )
        });
        let (candidate_routes, spacing_quality, candidate_gap_spacing) =
            select_gap_spacing_candidate(plan, compact_candidate_routes, adaptive_candidate_routes);
        if !route_family_candidate_within_budget(node_count, plan.edges.len(), &candidate_routes) {
            return None;
        }
        let large_gap = gap_lanes
            .iter()
            .any(|lanes| lanes.len() > MAX_GLOBAL_GAP_LANES);
        let (candidate_quality, candidate_routes) = if large_gap {
            let mut candidate = finish_route_family(
                plan,
                nodes,
                ranks,
                &sparse_spans,
                &crossing_lanes,
                &crossing_tie_lanes,
                crossing_tie_lane_count,
                &free_by_rank,
                &layer_left,
                &layer_right,
                &candidate_lanes,
                &crossing_paths,
                candidate_endpoint_tracks,
                candidate_crossing_paths_match_endpoint_tracks,
                &stable_channel_lanes,
                baseline_outer_lanes.clone(),
                top,
                bottom,
                options,
                outer_lane_rounds,
                repair_crossings,
                false,
                None,
                candidate_routes,
                candidate_gap_spacing,
            );
            candidate
                .repair
                .take()
                .filter(|(quality, _)| {
                    route_quality_cmp(*quality, candidate.primary_quality).is_lt()
                })
                .unwrap_or((candidate.primary_quality, candidate.primary))
        } else {
            (
                spacing_quality.unwrap_or_else(|| route_quality_for_plan(plan, &candidate_routes)),
                candidate_routes,
            )
        };
        Some((candidate_quality, candidate_routes))
    };
    let sparse_alternative = global_gap_lanes.and_then(&build_sparse_alternative);
    let preserved_refined_sparse_alternative =
        preserved_refined_global_gap_lanes.and_then(&build_sparse_alternative);
    let refined_sparse_alternative = refined_global_gap_lanes.and_then(build_sparse_alternative);
    let fanout_within_budget = fanout_candidates
        && repair_crossings
        && node_count >= MIN_FANOUT_AWARE_NODES
        && route_family_candidate_within_budget(node_count, plan.edges.len(), &routes);
    if fanout_within_budget
        && let Some(adaptive_channel_lanes) =
            fanout_outer_channel_lane_indices(plan, &sparse_spans, &outer_nets)
    {
        let mut routed = finish_fanout_route_families(
            plan,
            nodes,
            ranks,
            &sparse_spans,
            &crossing_lanes,
            &crossing_tie_lanes,
            crossing_tie_lane_count,
            &free_by_rank,
            &layer_left,
            &layer_right,
            &gap_lanes,
            &crossing_paths,
            endpoint_tracks,
            crossing_paths_match_endpoint_tracks,
            &stable_channel_lanes,
            adaptive_channel_lanes,
            baseline_outer_lanes,
            top,
            bottom,
            options,
            outer_lane_rounds,
            repair_crossings,
            deeper_crossing_repair,
            routes,
            sparse_alternative,
            gap_spacing,
        );
        routed
            .alternatives
            .extend(preserved_refined_sparse_alternative);
        routed.alternatives.extend(refined_sparse_alternative);
        routed.alternatives.extend(staircase_alternative);
        return routed;
    }
    let mut outer_lanes = baseline_outer_lanes;
    let mut primary_quality = spacing_quality;
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
    if feedback_within_budget {
        let coherent_outer_lanes = outer_lane_assignments(
            plan,
            nodes,
            ranks,
            &sparse_spans,
            &stable_channel_lanes,
            &layer_left,
            &layer_right,
            top,
            bottom,
            options,
            outer_lane_rounds,
            true,
        );
        let baseline_quality = route_quality_for_plan(plan, &routes);
        let candidate_endpoint_tracks =
            (!outer_lane_channels_match(&coherent_outer_lanes, &outer_lanes)).then(|| {
                build_endpoint_tracks(
                    plan,
                    nodes,
                    ranks,
                    &sparse_spans,
                    &layer_left,
                    &layer_right,
                    &gap_lanes,
                    &coherent_outer_lanes,
                    options,
                    gap_spacing,
                )
            });
        #[cfg(test)]
        if candidate_endpoint_tracks.is_none() {
            update_routing_reuse_counts(|counts| counts.coherent_endpoint_tracks += 1);
        }
        debug_assert!(
            candidate_endpoint_tracks.is_some() || {
                endpoint_tracks
                    == build_endpoint_tracks(
                        plan,
                        nodes,
                        ranks,
                        &sparse_spans,
                        &layer_left,
                        &layer_right,
                        &gap_lanes,
                        &coherent_outer_lanes,
                        options,
                        gap_spacing,
                    )
            }
        );
        let candidate_endpoint_tracks_ref = candidate_endpoint_tracks
            .as_ref()
            .unwrap_or(&endpoint_tracks);
        let candidate_routes = emit_routes(
            plan,
            nodes,
            &sparse_spans,
            &crossing_paths,
            &layer_left,
            &layer_right,
            &gap_lanes,
            candidate_endpoint_tracks_ref,
            &coherent_outer_lanes,
            top,
            bottom,
            options,
            gap_spacing,
        );
        let candidate_quality = route_quality_for_plan(plan, &candidate_routes);
        #[cfg(test)]
        {
            feedback_trace.evaluated = true;
            feedback_trace.baseline = Some((baseline_quality, routes.clone()));
            feedback_trace.candidate_quality = Some(candidate_quality);
        }
        if route_quality_cmp(candidate_quality, baseline_quality).is_lt() {
            routes = candidate_routes;
            if let Some(candidate_endpoint_tracks) = candidate_endpoint_tracks {
                crossing_paths_match_endpoint_tracks &=
                    candidate_endpoint_tracks == endpoint_tracks;
                endpoint_tracks = candidate_endpoint_tracks;
            }
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
    let deeper_repair_within_budget = repair_crossings
        && deeper_crossing_repair
        && crossing_repair_within_budget(
            node_count,
            plan.edges.len(),
            &routes,
            &gap_lanes,
            &sparse_spans,
            &free_by_rank,
        );
    let shared_repair_profile =
        deeper_repair_within_budget.then(|| horizontal_crossing_profile_by_net(plan, &routes));
    let precomputed_repair_profile = shared_repair_profile
        .as_ref()
        .map(|(segments, counts, quality)| (segments.as_slice(), counts, *quality));
    let mut repair = if repair_crossings {
        Some(repair_crossing_heavy_net(
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
            outer_lane_rounds,
            &routes,
            &endpoint_tracks,
            &crossing_paths,
            crossing_paths_match_endpoint_tracks,
            precomputed_repair_profile,
            gap_spacing,
            MAX_BATCHED_CROSSING_REPAIR_NETS,
        ))
    } else {
        None
    };
    let deeper_crossing_repair_candidate = if deeper_repair_within_budget {
        repair_crossing_heavy_net(
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
            outer_lane_rounds,
            &routes,
            &endpoint_tracks,
            &crossing_paths,
            crossing_paths_match_endpoint_tracks,
            precomputed_repair_profile,
            gap_spacing,
            MAX_DEEP_CROSSING_REPAIR_NETS,
        )
        .candidate
        .filter(|candidate| {
            repair.as_ref().and_then(|item| item.candidate.as_ref()) != Some(candidate)
        })
    } else {
        None
    };
    let selected_quality = repair
        .as_ref()
        .map_or(primary_quality, |repair| Some(repair.baseline_quality));
    #[cfg(test)]
    let repair_outer_sides = repair
        .as_ref()
        .map_or_else(Vec::new, |repair| repair.selected_outer_sides.clone());
    RoutedEdges {
        primary: routes,
        primary_quality: selected_quality,
        repair: repair.as_mut().and_then(|repair| repair.candidate.take()),
        alternatives: sparse_alternative
            .into_iter()
            .chain(preserved_refined_sparse_alternative)
            .chain(refined_sparse_alternative)
            .chain(deeper_crossing_repair_candidate)
            .chain(staircase_alternative)
            .collect(),
        #[cfg(test)]
        feedback_trace,
        #[cfg(test)]
        fanout_trace: FanoutCandidateTrace {
            evaluated: false,
            selected: false,
            baseline_quality: None,
            candidate_quality: None,
        },
        #[cfg(test)]
        repair_nets: repair.map_or_else(Vec::new, |repair| repair.selected_nets),
        #[cfg(test)]
        repair_outer_sides,
    }
}

#[allow(clippy::too_many_arguments)]
fn build_staircase_alignment_alternative(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    free_by_rank: &[Vec<(f64, f64)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<NetId, usize>],
    crossing_paths: &[Option<Vec<f64>>],
    outer_lanes: &BTreeMap<EdgeId, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    gap_spacing: GapTrackSpacing,
    baseline_routes: &[EdgeGeometry],
    baseline_quality: Option<RouteQuality>,
) -> Option<(RouteQuality, Vec<EdgeGeometry>)> {
    let (candidate_paths, aligned_transitions) = align_crossing_path_staircases(
        plan,
        sparse_spans,
        free_by_rank,
        crossing_paths,
        layer_left,
        layer_right,
        gap_lanes,
        options,
        gap_spacing,
    )?;
    if aligned_transitions < MIN_STAIRCASE_ALIGNMENT_TRANSITIONS {
        return None;
    }
    let baseline_quality =
        baseline_quality.unwrap_or_else(|| route_quality_for_plan(plan, baseline_routes));
    if aligned_transitions
        < baseline_quality
            .bends
            .div_ceil(MIN_STAIRCASE_ALIGNMENT_RATIO_DENOMINATOR)
    {
        return None;
    }
    let endpoint_tracks = build_endpoint_tracks(
        plan,
        nodes,
        &plan.ranks,
        sparse_spans,
        layer_left,
        layer_right,
        gap_lanes,
        outer_lanes,
        options,
        gap_spacing,
    );
    let candidate_routes = emit_routes(
        plan,
        nodes,
        sparse_spans,
        &candidate_paths,
        layer_left,
        layer_right,
        gap_lanes,
        &endpoint_tracks,
        outer_lanes,
        top,
        bottom,
        options,
        gap_spacing,
    );
    let candidate_quality = route_quality_for_plan(plan, &candidate_routes);
    (candidate_quality.crossings <= baseline_quality.crossings
        && route_quality_cmp(candidate_quality, baseline_quality).is_lt())
    .then_some((candidate_quality, candidate_routes))
}

#[allow(clippy::too_many_arguments)]
fn align_crossing_path_staircases(
    plan: &RoutingPlan<'_>,
    sparse_spans: &[Option<(usize, usize)>],
    free_by_rank: &[Vec<(f64, f64)>],
    crossing_paths: &[Option<Vec<f64>>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<NetId, usize>],
    options: LayoutOptions,
    gap_spacing: GapTrackSpacing,
) -> Option<(Vec<Option<Vec<f64>>>, usize)> {
    let net_ordinals = plan
        .edges
        .iter()
        .map(|resolved| resolved.edge.net)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .enumerate()
        .map(|(ordinal, net)| (net, ordinal))
        .collect::<BTreeMap<_, _>>();
    let net_count = net_ordinals.len();
    let mut paths = crossing_paths.to_vec();
    let mut backbones = BTreeMap::<(NetId, u32, u32), Vec<usize>>::new();
    for (edge_index, ((resolved, span), path)) in plan
        .edges
        .iter()
        .zip(sparse_spans)
        .zip(crossing_paths)
        .enumerate()
    {
        let (Some(&(source_rank, target_rank)), Some(path)) = (span.as_ref(), path.as_ref()) else {
            continue;
        };
        debug_assert_eq!(path.len(), target_rank - source_rank - 1);
        if !path.is_empty() {
            backbones
                .entry((
                    resolved.edge.net,
                    resolved.edge.source.node,
                    resolved.edge.source.port,
                ))
                .or_default()
                .push(edge_index);
        }
    }

    let mut aligned_transitions = 0usize;
    for edge_indices in backbones.values_mut() {
        edge_indices.sort_unstable_by(|&left, &right| {
            crossing_paths[right]
                .as_ref()
                .expect("backbone path exists")
                .len()
                .cmp(
                    &crossing_paths[left]
                        .as_ref()
                        .expect("backbone path exists")
                        .len(),
                )
                .then(plan.edges[left].edge.id.cmp(&plan.edges[right].edge.id))
        });
        let canonical_index = edge_indices[0];
        let canonical_path = crossing_paths[canonical_index]
            .as_ref()
            .expect("backbone path exists");
        if canonical_path.len() < 2 {
            continue;
        }
        let source_rank = sparse_spans[canonical_index]
            .expect("backbone span exists")
            .0;
        let net = plan.edges[canonical_index].edge.net;
        let canonical_aligned = align_one_crossing_path_staircase(
            canonical_path,
            source_rank,
            free_by_rank,
            net_ordinals[&net],
            net_count,
        );
        aligned_transitions += removed_staircase_transitions(canonical_path, &canonical_aligned);
        paths[canonical_index] = Some(canonical_aligned.clone());

        for &edge_index in edge_indices.iter().skip(1) {
            let path = crossing_paths[edge_index]
                .as_ref()
                .expect("backbone path exists");
            if path.len() <= canonical_path.len()
                && path.as_slice() == &canonical_path[..path.len()]
            {
                let aligned = canonical_aligned[..path.len()].to_vec();
                paths[edge_index] = Some(aligned);
            }
        }
    }

    (!crossing_paths_have_unrelated_collinear_tracks(
        plan,
        sparse_spans,
        &paths,
        layer_left,
        layer_right,
        gap_lanes,
        options,
        gap_spacing,
    ))
    .then_some((paths, aligned_transitions))
}

fn align_one_crossing_path_staircase(
    path: &[f64],
    source_rank: usize,
    free_by_rank: &[Vec<(f64, f64)>],
    net_ordinal: usize,
    net_count: usize,
) -> Vec<f64> {
    let intervals = path
        .iter()
        .enumerate()
        .map(|(offset, &y)| {
            free_interval_containing(&free_by_rank[source_rank + offset + 1], y)
                .expect("crossing path ordinate remains in its selected free interval")
        })
        .collect::<Vec<_>>();
    let mut aligned = path.to_vec();
    let mut start = 0usize;
    while start < path.len() {
        let mut end = start + 1;
        let mut low = intervals[start].0;
        let mut high = intervals[start].1;
        while end < path.len() {
            let next_low = low.max(intervals[end].0);
            let next_high = high.min(intervals[end].1);
            if next_low >= next_high {
                break;
            }
            low = next_low;
            high = next_high;
            end += 1;
        }
        if end - start >= 2 {
            let margin = (CROSSING_TRACK_NUDGE * (net_ordinal + 1) as f64).min((high - low) / 4.0);
            let net_offset = if net_count < 2 {
                0.0
            } else {
                (net_ordinal as f64 / (net_count - 1) as f64 - 0.5) * 0.01
            };
            let y = (path[start] + net_offset).clamp(low + margin, high - margin);
            aligned[start..end].fill(y);
        }
        start = end;
    }
    aligned
}

fn free_interval_containing(intervals: &[(f64, f64)], y: f64) -> Option<(f64, f64)> {
    let index = intervals.partition_point(|&(_, high)| high < y);
    intervals
        .get(index)
        .copied()
        .filter(|&(low, high)| low <= y && y <= high)
}

fn removed_staircase_transitions(original: &[f64], aligned: &[f64]) -> usize {
    original
        .windows(2)
        .zip(aligned.windows(2))
        .filter(|(original, aligned)| original[0] != original[1] && aligned[0] == aligned[1])
        .count()
}

#[derive(Clone, Copy)]
struct HorizontalCrossingRun {
    net: NetId,
    start: f64,
    end: f64,
}

#[allow(clippy::too_many_arguments)]
fn crossing_paths_have_unrelated_collinear_tracks(
    plan: &RoutingPlan<'_>,
    sparse_spans: &[Option<(usize, usize)>],
    paths: &[Option<Vec<f64>>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<NetId, usize>],
    options: LayoutOptions,
    gap_spacing: GapTrackSpacing,
) -> bool {
    let mut runs_by_y = BTreeMap::<FloatKey, Vec<HorizontalCrossingRun>>::new();
    for ((resolved, span), path) in plan.edges.iter().zip(sparse_spans).zip(paths) {
        let (Some(&(source_rank, _)), Some(path)) = (span.as_ref(), path.as_ref()) else {
            continue;
        };
        for (offset, &y) in path.iter().enumerate() {
            let rank = source_rank + offset + 1;
            let left = sparse_gap_x(
                resolved.edge.net,
                rank - 1,
                layer_left,
                layer_right,
                gap_lanes,
                options,
                gap_spacing,
            );
            let right = sparse_gap_x(
                resolved.edge.net,
                rank,
                layer_left,
                layer_right,
                gap_lanes,
                options,
                gap_spacing,
            );
            let y = if y == 0.0 { 0.0 } else { y };
            runs_by_y
                .entry(FloatKey(y))
                .or_default()
                .push(HorizontalCrossingRun {
                    net: resolved.edge.net,
                    start: left.min(right),
                    end: left.max(right),
                });
        }
    }

    runs_by_y.into_values().any(|mut runs| {
        runs.sort_unstable_by(|left, right| {
            left.start
                .total_cmp(&right.start)
                .then(left.end.total_cmp(&right.end))
                .then(left.net.cmp(&right.net))
        });
        let mut longest = None;
        let mut second_longest = None;
        for run in runs {
            let unrelated_end = longest
                .filter(|&(net, _)| net != run.net)
                .or_else(|| second_longest.filter(|&(net, _)| net != run.net))
                .map(|(_, end)| end);
            if unrelated_end.is_some_and(|end| end >= run.start) {
                return true;
            }
            update_longest_track_ends(&mut longest, &mut second_longest, run.net, run.end);
        }
        false
    })
}

fn update_longest_track_ends(
    longest: &mut Option<(NetId, f64)>,
    second_longest: &mut Option<(NetId, f64)>,
    net: NetId,
    end: f64,
) {
    if let Some((candidate, current_end)) = longest.as_mut()
        && *candidate == net
    {
        *current_end = current_end.max(end);
        return;
    }
    if let Some((candidate, current_end)) = second_longest.as_mut()
        && *candidate == net
    {
        *current_end = current_end.max(end);
    } else if longest.is_none() {
        *longest = Some((net, end));
    } else if second_longest.is_none_or(|(_, current_end)| end > current_end) {
        *second_longest = Some((net, end));
    }
    if longest.zip(*second_longest).is_some_and(|(first, second)| {
        second.1.total_cmp(&first.1).is_gt() || (second.1 == first.1 && second.0 < first.0)
    }) {
        std::mem::swap(longest, second_longest);
    }
}

#[cold]
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn finish_fanout_route_families(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_lanes: &[BTreeMap<NetId, usize>],
    crossing_tie_lanes: &BTreeMap<(usize, NetId), usize>,
    crossing_tie_lane_count: usize,
    free_by_rank: &[Vec<(f64, f64)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<NetId, usize>],
    crossing_paths: &[Option<Vec<f64>>],
    stable_endpoint_tracks: BTreeMap<(u32, u32, u8), (usize, usize)>,
    stable_crossing_paths_match_endpoint_tracks: bool,
    stable_channel_lanes: &BTreeMap<NetId, usize>,
    adaptive_channel_lanes: BTreeMap<NetId, usize>,
    stable_outer_lanes: BTreeMap<NetId, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    outer_lane_rounds: usize,
    repair_crossings: bool,
    deeper_crossing_repair: bool,
    stable_routes: Vec<EdgeGeometry>,
    sparse_alternative: Option<(RouteQuality, Vec<EdgeGeometry>)>,
    gap_spacing: GapTrackSpacing,
) -> RoutedEdges {
    let adaptive_outer_lanes = outer_lane_assignments(
        plan,
        nodes,
        ranks,
        sparse_spans,
        &adaptive_channel_lanes,
        layer_left,
        layer_right,
        top,
        bottom,
        options,
        outer_lane_rounds,
        false,
    );
    // Channel order changes only outer accesses. Preserve the already-refined sparse corridors
    // and rebuild the endpoint escapes plus complete route geometry affected by those accesses.
    let adaptive_endpoint_tracks =
        if outer_lane_channels_match(&adaptive_outer_lanes, &stable_outer_lanes) {
            debug_assert_eq!(
                stable_endpoint_tracks,
                build_endpoint_tracks(
                    plan,
                    nodes,
                    ranks,
                    sparse_spans,
                    layer_left,
                    layer_right,
                    gap_lanes,
                    &adaptive_outer_lanes,
                    options,
                    gap_spacing,
                ),
            );
            stable_endpoint_tracks.clone()
        } else {
            build_endpoint_tracks(
                plan,
                nodes,
                ranks,
                sparse_spans,
                layer_left,
                layer_right,
                gap_lanes,
                &adaptive_outer_lanes,
                options,
                gap_spacing,
            )
        };
    let adaptive_crossing_paths_match_endpoint_tracks = stable_crossing_paths_match_endpoint_tracks
        && adaptive_endpoint_tracks == stable_endpoint_tracks;
    let adaptive_routes = emit_routes(
        plan,
        nodes,
        sparse_spans,
        crossing_paths,
        layer_left,
        layer_right,
        gap_lanes,
        &adaptive_endpoint_tracks,
        &adaptive_outer_lanes,
        top,
        bottom,
        options,
        gap_spacing,
    );
    let baseline_profile = horizontal_crossing_profile_by_net(plan, &stable_routes);
    let candidate_profile = horizontal_crossing_profile_by_net(plan, &adaptive_routes);
    let baseline_quality = baseline_profile.2;
    let candidate_quality = candidate_profile.2;
    let adaptive_is_better = route_quality_cmp(candidate_quality, baseline_quality).is_lt();

    let (selected, mut alternatives) = if adaptive_is_better {
        let adaptive = finish_route_family(
            plan,
            nodes,
            ranks,
            sparse_spans,
            crossing_lanes,
            crossing_tie_lanes,
            crossing_tie_lane_count,
            free_by_rank,
            layer_left,
            layer_right,
            gap_lanes,
            crossing_paths,
            adaptive_endpoint_tracks,
            adaptive_crossing_paths_match_endpoint_tracks,
            &adaptive_channel_lanes,
            adaptive_outer_lanes,
            top,
            bottom,
            options,
            outer_lane_rounds,
            repair_crossings,
            deeper_crossing_repair,
            Some(candidate_profile),
            adaptive_routes,
            gap_spacing,
        );
        let stable = finish_route_family(
            plan,
            nodes,
            ranks,
            sparse_spans,
            crossing_lanes,
            crossing_tie_lanes,
            crossing_tie_lane_count,
            free_by_rank,
            layer_left,
            layer_right,
            gap_lanes,
            crossing_paths,
            stable_endpoint_tracks,
            stable_crossing_paths_match_endpoint_tracks,
            stable_channel_lanes,
            stable_outer_lanes,
            top,
            bottom,
            options,
            outer_lane_rounds,
            repair_crossings,
            deeper_crossing_repair,
            Some(baseline_profile),
            stable_routes,
            gap_spacing,
        );
        let mut alternatives = vec![(stable.primary_quality, stable.primary)];
        if let Some(repair) = stable.repair {
            alternatives.push(repair);
        }
        if let Some(repair) = stable.deeper_repair {
            alternatives.push(repair);
        }
        (adaptive, alternatives)
    } else {
        (
            finish_route_family(
                plan,
                nodes,
                ranks,
                sparse_spans,
                crossing_lanes,
                crossing_tie_lanes,
                crossing_tie_lane_count,
                free_by_rank,
                layer_left,
                layer_right,
                gap_lanes,
                crossing_paths,
                stable_endpoint_tracks,
                stable_crossing_paths_match_endpoint_tracks,
                stable_channel_lanes,
                stable_outer_lanes,
                top,
                bottom,
                options,
                outer_lane_rounds,
                repair_crossings,
                deeper_crossing_repair,
                Some(baseline_profile),
                stable_routes,
                gap_spacing,
            ),
            Vec::new(),
        )
    };
    alternatives.extend(sparse_alternative);
    alternatives.extend(selected.deeper_repair);
    RoutedEdges {
        primary: selected.primary,
        primary_quality: Some(selected.primary_quality),
        repair: selected.repair,
        alternatives,
        #[cfg(test)]
        feedback_trace: selected.feedback_trace,
        #[cfg(test)]
        fanout_trace: FanoutCandidateTrace {
            evaluated: true,
            selected: adaptive_is_better,
            baseline_quality: Some(baseline_quality),
            candidate_quality: Some(candidate_quality),
        },
        #[cfg(test)]
        repair_nets: selected.repair_nets,
        #[cfg(test)]
        repair_outer_sides: selected.repair_outer_sides,
    }
}

fn route_family_candidate_within_budget(
    node_count: usize,
    edge_count: usize,
    routes: &[EdgeGeometry],
) -> bool {
    node_count <= MAX_CROSSING_REPAIR_NODES
        && edge_count <= MAX_CROSSING_REPAIR_EDGES
        && sum_within_limit(
            routes.iter().map(|route| route.points.len()),
            MAX_CROSSING_REPAIR_ROUTE_POINTS,
        )
}

fn route_family_candidate_shape_within_budget(
    node_count: usize,
    edge_count: usize,
    sparse_spans: &[Option<(usize, usize)>],
) -> bool {
    node_count <= MAX_CROSSING_REPAIR_NODES
        && edge_count <= MAX_CROSSING_REPAIR_EDGES
        && candidate_route_points_within_budget(sparse_spans)
}

#[allow(clippy::too_many_arguments)]
fn finish_route_family(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_lanes: &[BTreeMap<u32, usize>],
    crossing_tie_lanes: &BTreeMap<(usize, u32), usize>,
    crossing_tie_lane_count: usize,
    free_by_rank: &[Vec<(f64, f64)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    crossing_paths: &[Option<Vec<f64>>],
    mut endpoint_tracks: BTreeMap<(u32, u32, u8), (usize, usize)>,
    mut crossing_paths_match_endpoint_tracks: bool,
    channel_lanes: &BTreeMap<NetId, usize>,
    mut outer_lanes: BTreeMap<u32, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    outer_lane_rounds: usize,
    repair_crossings: bool,
    deeper_crossing_repair: bool,
    mut precomputed_profile: Option<(Vec<PhysicalSegment>, BTreeMap<NetId, usize>, RouteQuality)>,
    mut routes: Vec<EdgeGeometry>,
    gap_spacing: GapTrackSpacing,
) -> RouteFamily {
    let node_count = plan
        .nodes_by_rank
        .iter()
        .map(Vec::len)
        .try_fold(0usize, usize::checked_add)
        .unwrap_or(usize::MAX);
    let split_feedback = has_split_feedback_net(plan, sparse_spans, &outer_lanes);
    let feedback_within_budget = split_feedback
        && crossing_repair_within_budget(
            node_count,
            plan.edges.len(),
            &routes,
            gap_lanes,
            sparse_spans,
            free_by_rank,
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
            sparse_spans,
            channel_lanes,
            layer_left,
            layer_right,
            top,
            bottom,
            options,
            outer_lane_rounds,
            true,
        );
        let baseline_profile = precomputed_profile
            .take()
            .unwrap_or_else(|| horizontal_crossing_profile_by_net(plan, &routes));
        let baseline_quality = baseline_profile.2;
        // Coherence changes outer side and side-local lane indices, but not the stable per-net
        // channel index. The baseline sparse paths and gap lanes therefore remain valid.
        let candidate_endpoint_tracks =
            (!outer_lane_channels_match(&coherent_outer_lanes, &outer_lanes)).then(|| {
                build_endpoint_tracks(
                    plan,
                    nodes,
                    ranks,
                    sparse_spans,
                    layer_left,
                    layer_right,
                    gap_lanes,
                    &coherent_outer_lanes,
                    options,
                    gap_spacing,
                )
            });
        #[cfg(test)]
        if candidate_endpoint_tracks.is_none() {
            update_routing_reuse_counts(|counts| counts.coherent_endpoint_tracks += 1);
        }
        debug_assert!(
            candidate_endpoint_tracks.is_some() || {
                endpoint_tracks
                    == build_endpoint_tracks(
                        plan,
                        nodes,
                        ranks,
                        sparse_spans,
                        layer_left,
                        layer_right,
                        gap_lanes,
                        &coherent_outer_lanes,
                        options,
                        gap_spacing,
                    )
            }
        );
        let candidate_endpoint_tracks_ref = candidate_endpoint_tracks
            .as_ref()
            .unwrap_or(&endpoint_tracks);
        let candidate_routes = emit_routes(
            plan,
            nodes,
            sparse_spans,
            crossing_paths,
            layer_left,
            layer_right,
            gap_lanes,
            candidate_endpoint_tracks_ref,
            &coherent_outer_lanes,
            top,
            bottom,
            options,
            gap_spacing,
        );
        let candidate_profile = horizontal_crossing_profile_by_net(plan, &candidate_routes);
        let candidate_quality = candidate_profile.2;
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
            if let Some(candidate_endpoint_tracks) = candidate_endpoint_tracks {
                crossing_paths_match_endpoint_tracks &=
                    candidate_endpoint_tracks == endpoint_tracks;
                endpoint_tracks = candidate_endpoint_tracks;
            }
            outer_lanes = coherent_outer_lanes;
            precomputed_profile = Some(candidate_profile);
            #[cfg(test)]
            {
                feedback_trace.selected = true;
            }
        } else {
            precomputed_profile = Some(baseline_profile);
        }
    }
    let deeper_repair_within_budget = repair_crossings
        && deeper_crossing_repair
        && crossing_repair_within_budget(
            node_count,
            plan.edges.len(),
            &routes,
            gap_lanes,
            sparse_spans,
            free_by_rank,
        );
    if deeper_repair_within_budget && precomputed_profile.is_none() {
        precomputed_profile = Some(horizontal_crossing_profile_by_net(plan, &routes));
    }
    let precomputed_quality = precomputed_profile.as_ref().map(|(_, _, quality)| *quality);
    let precomputed_repair_profile = precomputed_profile
        .as_ref()
        .map(|(segments, counts, quality)| (segments.as_slice(), counts, *quality));
    let mut repair = if repair_crossings {
        Some(repair_crossing_heavy_net(
            plan,
            nodes,
            sparse_spans,
            crossing_lanes,
            crossing_tie_lanes,
            crossing_tie_lane_count,
            free_by_rank,
            layer_left,
            layer_right,
            gap_lanes,
            &outer_lanes,
            top,
            bottom,
            options,
            outer_lane_rounds,
            &routes,
            &endpoint_tracks,
            crossing_paths,
            crossing_paths_match_endpoint_tracks,
            precomputed_repair_profile,
            gap_spacing,
            MAX_BATCHED_CROSSING_REPAIR_NETS,
        ))
    } else {
        None
    };
    let deeper_repair = if deeper_repair_within_budget {
        repair_crossing_heavy_net(
            plan,
            nodes,
            sparse_spans,
            crossing_lanes,
            crossing_tie_lanes,
            crossing_tie_lane_count,
            free_by_rank,
            layer_left,
            layer_right,
            gap_lanes,
            &outer_lanes,
            top,
            bottom,
            options,
            outer_lane_rounds,
            &routes,
            &endpoint_tracks,
            crossing_paths,
            crossing_paths_match_endpoint_tracks,
            precomputed_repair_profile,
            gap_spacing,
            MAX_DEEP_CROSSING_REPAIR_NETS,
        )
        .candidate
        .filter(|candidate| {
            repair.as_ref().and_then(|item| item.candidate.as_ref()) != Some(candidate)
        })
    } else {
        None
    };
    let primary_quality = repair.as_ref().map_or_else(
        || precomputed_quality.unwrap_or_else(|| route_quality_for_plan(plan, &routes)),
        |repair| repair.baseline_quality,
    );
    #[cfg(test)]
    let repair_outer_sides = repair
        .as_ref()
        .map_or_else(Vec::new, |repair| repair.selected_outer_sides.clone());
    RouteFamily {
        primary: routes,
        primary_quality,
        repair: repair.as_mut().and_then(|repair| repair.candidate.take()),
        deeper_repair,
        #[cfg(test)]
        feedback_trace,
        #[cfg(test)]
        repair_nets: repair.map_or_else(Vec::new, |repair| repair.selected_nets),
        #[cfg(test)]
        repair_outer_sides,
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
    initial_gap_lanes: &[BTreeMap<u32, usize>],
    outer_lanes: &BTreeMap<u32, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    gap_lane_rounds: usize,
    sparse_global: bool,
    large_sparse_global: bool,
    refined_large_sparse_global: bool,
    adaptive_gap_spacing: bool,
) -> RoutedLaneState {
    let initial_endpoint_tracks = build_endpoint_tracks(
        plan,
        nodes,
        &plan.ranks,
        sparse_spans,
        layer_left,
        layer_right,
        initial_gap_lanes,
        outer_lanes,
        options,
        GapTrackSpacing::Compact,
    );
    let crossing_paths = sparse_crossing_paths(
        plan,
        nodes,
        sparse_spans,
        crossing_lanes,
        crossing_tie_lanes,
        crossing_tie_lane_count,
        free_by_rank,
        &initial_endpoint_tracks,
        options.port_stub,
    );
    let GapLaneCandidates {
        baseline: gap_lanes,
        global: global_gap_lanes,
        preserved_refined: preserved_refined_global_gap_lanes,
        refined: refined_global_gap_lanes,
    } = crossing_aware_gap_lanes(
        plan,
        nodes,
        sparse_spans,
        &crossing_paths,
        initial_gap_lanes,
        &initial_endpoint_tracks,
        options.port_stub,
        gap_lane_rounds,
        sparse_global && (outer_lanes.is_empty() || large_sparse_global),
        large_sparse_global,
        refined_large_sparse_global,
    );
    let (endpoint_tracks, crossing_paths_match_endpoint_tracks) = if gap_lanes == initial_gap_lanes
    {
        #[cfg(test)]
        update_routing_reuse_counts(|counts| counts.final_endpoint_tracks += 1);
        (initial_endpoint_tracks, true)
    } else {
        let endpoint_tracks = build_endpoint_tracks(
            plan,
            nodes,
            &plan.ranks,
            sparse_spans,
            layer_left,
            layer_right,
            &gap_lanes,
            outer_lanes,
            options,
            GapTrackSpacing::Compact,
        );
        let matches = endpoint_tracks == initial_endpoint_tracks;
        (endpoint_tracks, matches)
    };
    let compact_routes = emit_routes(
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
        GapTrackSpacing::Compact,
    );
    // A non-fallback adaptive track implies port_stub < 30% of the gap, stays in the sparse
    // 55%..(right - stub) band, and preserves lane order; outer tracks stay in 35%..50%.
    // Pairwise endpoint-access overlap components (and therefore endpoint escapes and crossing
    // paths) are consequently identical even though the channel x coordinates move apart. Reuse
    // the compact endpoint work here; the band invariant is covered by a focused regression.
    let adaptive_routes = (adaptive_gap_spacing && gap_lanes.iter().any(|lanes| lanes.len() > 1))
        .then(|| {
            emit_routes(
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
                GapTrackSpacing::Adaptive,
            )
        });
    let (routes, route_quality, gap_spacing) =
        select_gap_spacing_candidate(plan, compact_routes, adaptive_routes);
    RoutedLaneState {
        routes,
        route_quality,
        gap_spacing,
        gap_lanes,
        global_gap_lanes,
        preserved_refined_global_gap_lanes,
        refined_global_gap_lanes,
        endpoint_tracks,
        crossing_paths_match_endpoint_tracks,
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
    gap_spacing: GapTrackSpacing,
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
                        options,
                        gap_spacing,
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
    outer_lane_rounds: usize,
    routes: &[EdgeGeometry],
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    crossing_paths: &[Option<Vec<f64>>],
    crossing_paths_match_endpoint_tracks: bool,
    precomputed: Option<(&[PhysicalSegment], &BTreeMap<NetId, usize>, RouteQuality)>,
    gap_spacing: GapTrackSpacing,
    max_repair_nets: usize,
) -> CrossingRepair {
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
        return CrossingRepair {
            baseline_quality: route_quality_for_plan(plan, routes),
            candidate: None,
            #[cfg(test)]
            selected_nets: Vec::new(),
            #[cfg(test)]
            selected_outer_sides: Vec::new(),
            #[cfg(test)]
            candidate_lanes_built: false,
            #[cfg(test)]
            candidate_emitted: false,
        };
    }
    let computed_profile;
    let (physical_segments, crossing_counts, quality) = match precomputed {
        Some((segments, counts, quality)) => (segments, counts, quality),
        None => {
            computed_profile = horizontal_crossing_profile_by_net(plan, routes);
            (
                computed_profile.0.as_slice(),
                &computed_profile.1,
                computed_profile.2,
            )
        }
    };
    // Sparse-lane attribution and the outer-arm profiles select independent whole-net moves.
    // Combine both bounded repair sets before the one rebuild and exact score so the added
    // selector does not add another complete routing candidate.
    let selected_nets = select_crossing_repair_nets(
        quality.crossings,
        crossing_counts,
        gap_lanes,
        max_repair_nets,
    );
    if !repair_selection_adds_new_nets(max_repair_nets, selected_nets.len()) {
        return CrossingRepair {
            baseline_quality: quality,
            candidate: None,
            #[cfg(test)]
            selected_nets,
            #[cfg(test)]
            selected_outer_sides: Vec::new(),
            #[cfg(test)]
            candidate_lanes_built: false,
            #[cfg(test)]
            candidate_emitted: false,
        };
    }
    let candidate_points_within_budget = candidate_route_points_within_budget(sparse_spans);
    let selected_outer_sides = if candidate_points_within_budget
        && quality.crossings >= MIN_CROSSING_REPAIR_TOTAL
        && !outer_lanes.is_empty()
    {
        select_outer_side_repairs(
            plan,
            nodes,
            sparse_spans,
            layer_left,
            layer_right,
            gap_lanes,
            outer_lanes,
            top,
            bottom,
            options,
            physical_segments,
            gap_spacing,
            endpoint_tracks,
        )
    } else {
        Vec::new()
    };
    #[cfg(test)]
    let mut candidate_lanes_built = false;
    #[cfg(test)]
    let mut candidate_emitted = false;
    let repair = (|| {
        if selected_nets.is_empty() && selected_outer_sides.is_empty() {
            return None;
        }
        if !candidate_points_within_budget {
            return None;
        }
        let candidate_lanes = if selected_nets.is_empty() {
            gap_lanes.to_vec()
        } else {
            move_nets_to_outer_lanes(gap_lanes, &selected_nets)?
        };
        let mut candidate_outer_lanes = outer_lanes.clone();
        if !selected_outer_sides.is_empty() {
            let sides = selected_outer_sides
                .iter()
                .copied()
                .collect::<BTreeMap<_, _>>();
            for (resolved, span) in plan.edges.iter().zip(sparse_spans) {
                if span.is_none()
                    && let Some(&side) = sides.get(&resolved.edge.net)
                {
                    candidate_outer_lanes
                        .get_mut(&resolved.edge.id)
                        .expect("outer edge has an assignment")
                        .side = side;
                }
            }
            reindex_outer_lane_assignments(
                plan,
                nodes,
                &plan.ranks,
                sparse_spans,
                layer_left,
                layer_right,
                options,
                outer_lane_rounds,
                &mut candidate_outer_lanes,
            );
        }
        #[cfg(test)]
        {
            candidate_lanes_built = true;
        }
        let candidate_endpoint_tracks = build_endpoint_tracks(
            plan,
            nodes,
            &plan.ranks,
            sparse_spans,
            layer_left,
            layer_right,
            &candidate_lanes,
            &candidate_outer_lanes,
            options,
            gap_spacing,
        );
        let reuse_crossing_paths =
            crossing_paths_match_endpoint_tracks && candidate_endpoint_tracks == *endpoint_tracks;
        #[cfg(test)]
        update_routing_reuse_counts(|counts| {
            if reuse_crossing_paths {
                counts.repair_crossing_paths += 1;
            } else {
                counts.repair_crossing_paths_recomputed += 1;
            }
        });
        let candidate_crossing_paths = (!reuse_crossing_paths).then(|| {
            sparse_crossing_paths(
                plan,
                nodes,
                sparse_spans,
                crossing_lanes,
                crossing_tie_lanes,
                crossing_tie_lane_count,
                free_by_rank,
                &candidate_endpoint_tracks,
                options.port_stub,
            )
        });
        let candidate_crossing_paths = candidate_crossing_paths
            .as_deref()
            .unwrap_or(crossing_paths);
        let candidate = emit_routes(
            plan,
            nodes,
            sparse_spans,
            candidate_crossing_paths,
            layer_left,
            layer_right,
            &candidate_lanes,
            &candidate_endpoint_tracks,
            &candidate_outer_lanes,
            top,
            bottom,
            options,
            gap_spacing,
        );
        #[cfg(test)]
        {
            candidate_emitted = true;
        }
        Some(candidate)
    })();
    let repair = repair.and_then(|routes| {
        sum_within_limit(
            routes.iter().map(|route| route.points.len()),
            MAX_CROSSING_REPAIR_ROUTE_POINTS,
        )
        .then(|| (route_quality_for_plan(plan, &routes), routes))
    });
    CrossingRepair {
        baseline_quality: quality,
        candidate: repair,
        #[cfg(test)]
        selected_nets,
        #[cfg(test)]
        selected_outer_sides,
        #[cfg(test)]
        candidate_lanes_built,
        #[cfg(test)]
        candidate_emitted,
    }
}

fn repair_selection_adds_new_nets(max_repair_nets: usize, selected_nets: usize) -> bool {
    max_repair_nets <= MAX_BATCHED_CROSSING_REPAIR_NETS
        || selected_nets > MAX_BATCHED_CROSSING_REPAIR_NETS
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

fn candidate_route_points_within_budget(sparse_spans: &[Option<(usize, usize)>]) -> bool {
    sum_within_limit(
        sparse_spans.iter().map(|span| match *span {
            Some((source_rank, target_rank)) => target_rank
                .checked_sub(source_rank)
                .and_then(|rank_span| rank_span.checked_mul(2))
                .and_then(|points| points.checked_add(8))
                .unwrap_or(usize::MAX),
            None => 10,
        }),
        MAX_CROSSING_REPAIR_ROUTE_POINTS,
    )
}

fn sum_within_limit(mut values: impl Iterator<Item = usize>, limit: usize) -> bool {
    values
        .try_fold(0usize, |total, value| {
            total.checked_add(value).filter(|&sum| sum <= limit)
        })
        .is_some()
}

fn select_crossing_repair_nets(
    total_crossings: usize,
    crossing_counts: &BTreeMap<NetId, usize>,
    gap_lanes: &[BTreeMap<NetId, usize>],
    max_repair_nets: usize,
) -> Vec<NetId> {
    if total_crossings < MIN_CROSSING_REPAIR_TOTAL {
        return Vec::new();
    }
    let mut movable = HashSet::new();
    for lanes in gap_lanes {
        for (&net, &lane) in lanes {
            if lane + 1 < lanes.len() {
                movable.insert(net);
            }
        }
    }
    let mut selected = Vec::<(usize, NetId)>::with_capacity(max_repair_nets);
    for (&net, &crossings) in crossing_counts {
        if crossings < MIN_CROSSING_REPAIR_NET || !movable.contains(&net) {
            continue;
        }
        let index = selected.partition_point(|&(selected_crossings, selected_net)| {
            selected_crossings > crossings
                || (selected_crossings == crossings && selected_net < net)
        });
        if index < max_repair_nets {
            selected.insert(index, (crossings, net));
            selected.truncate(max_repair_nets);
        }
    }
    selected.into_iter().map(|(_, net)| net).collect()
}

#[derive(Clone, Copy)]
enum OuterArmSide {
    Current,
    Fixed(OuterSide),
}

#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn select_outer_side_repairs(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<NetId, usize>],
    outer_lanes: &BTreeMap<EdgeId, OuterLane>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    physical_segments: &[PhysicalSegment],
    gap_spacing: GapTrackSpacing,
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
) -> Vec<(NetId, OuterSide)> {
    #[cfg(test)]
    update_routing_reuse_counts(|counts| counts.outer_repair_endpoint_tracks += 1);
    debug_assert_eq!(
        endpoint_tracks,
        &build_endpoint_tracks(
            plan,
            nodes,
            &plan.ranks,
            sparse_spans,
            layer_left,
            layer_right,
            gap_lanes,
            outer_lanes,
            options,
            gap_spacing,
        ),
        "the shared endpoint tracks must describe the baseline repair family",
    );
    let horizontal = physical_segments
        .iter()
        .filter(|segment| segment.horizontal)
        .collect::<Vec<_>>();
    let [current, top_counts, bottom_counts] = outer_arm_crossing_profiles(
        plan,
        nodes,
        sparse_spans,
        layer_left,
        layer_right,
        outer_lanes,
        endpoint_tracks,
        top,
        bottom,
        options,
        &horizontal,
    );
    if !current
        .values()
        .any(|&crossings| crossings >= MIN_OUTER_SIDE_REPAIR_GAIN)
    {
        return Vec::new();
    }
    let outer_nets = plan
        .edges
        .iter()
        .zip(sparse_spans)
        .filter(|(_, span)| span.is_none())
        .map(|(resolved, _)| resolved.edge.net)
        .collect::<BTreeSet<_>>();
    let mut selected =
        Vec::<(usize, NetId, OuterSide)>::with_capacity(MAX_BATCHED_OUTER_SIDE_REPAIRS);
    for net in outer_nets {
        let current_cost = current.get(&net).copied().unwrap_or(0);
        let top_cost = top_counts.get(&net).copied().unwrap_or(0);
        let bottom_cost = bottom_counts.get(&net).copied().unwrap_or(0);
        let (best_cost, side) = match top_cost.cmp(&bottom_cost) {
            Ordering::Less => (top_cost, OuterSide::Top),
            Ordering::Greater => (bottom_cost, OuterSide::Bottom),
            Ordering::Equal => continue,
        };
        let Some(gain) = current_cost.checked_sub(best_cost) else {
            continue;
        };
        if gain < MIN_OUTER_SIDE_REPAIR_GAIN
            || !plan.edges.iter().zip(sparse_spans).any(|(resolved, span)| {
                span.is_none()
                    && resolved.edge.net == net
                    && outer_lanes[&resolved.edge.id].side != side
            })
        {
            continue;
        }
        let index = selected.partition_point(|&(selected_gain, selected_net, _)| {
            selected_gain > gain || (selected_gain == gain && selected_net < net)
        });
        if index < MAX_BATCHED_OUTER_SIDE_REPAIRS {
            selected.insert(index, (gain, net, side));
            selected.truncate(MAX_BATCHED_OUTER_SIDE_REPAIRS);
        }
    }
    selected
        .into_iter()
        .map(|(_, net, side)| (net, side))
        .collect()
}

#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn outer_arm_crossing_profiles(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    layer_left: &[f64],
    layer_right: &[f64],
    outer_lanes: &BTreeMap<EdgeId, OuterLane>,
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    horizontal: &[&PhysicalSegment],
) -> [BTreeMap<NetId, usize>; 3] {
    // Outer trunks remain beyond the layout boundary, so side choice changes which interior
    // vertical arms meet the existing horizontal geometry. Score current, all-top, and
    // all-bottom arm profiles in one sweep; the rebuilt route still decides acceptance exactly.
    let arms = [
        OuterArmSide::Current,
        OuterArmSide::Fixed(OuterSide::Top),
        OuterArmSide::Fixed(OuterSide::Bottom),
    ]
    .map(|side| {
        outer_arm_segments(
            plan,
            nodes,
            sparse_spans,
            layer_left,
            layer_right,
            outer_lanes,
            endpoint_tracks,
            top,
            bottom,
            options,
            side,
            None,
        )
    });
    let mut vertical = Vec::with_capacity(arms.iter().map(Vec::len).sum());
    for (profile, segments) in arms.iter().enumerate() {
        vertical.extend(
            segments
                .iter()
                .map(|segment| (segment, ((profile as u64) << 32) | u64::from(segment.net))),
        );
    }
    let mut counts = std::array::from_fn(|_| BTreeMap::new());
    physical_crossing_sweep_lines(
        &plan.shared_endpoints,
        horizontal,
        &vertical,
        |key, count| {
            let profile = (key >> 32) as usize;
            let net = key as u32;
            *counts[profile].entry(net).or_default() += count;
        },
    );
    counts
}

#[allow(clippy::too_many_arguments)]
#[inline(never)]
fn outer_arm_segments(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    layer_left: &[f64],
    layer_right: &[f64],
    outer_lanes: &BTreeMap<EdgeId, OuterLane>,
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    side: OuterArmSide,
    nets: Option<&BTreeSet<NetId>>,
) -> Vec<PhysicalSegment> {
    let mut groups =
        BTreeMap::<(NetId, FloatKey), Vec<(f64, f64, EdgeId, Endpoint, Endpoint)>>::new();
    for (resolved, span) in plan.edges.iter().zip(sparse_spans) {
        if span.is_some() {
            continue;
        }
        let edge = resolved.edge;
        if nets.is_some_and(|nets| !nets.contains(&edge.net)) {
            continue;
        }
        let lane = outer_lanes[&edge.id];
        let side = match side {
            OuterArmSide::Current => lane.side,
            OuterArmSide::Fixed(side) => side,
        };
        let boundary_y = match side {
            OuterSide::Top => top,
            OuterSide::Bottom => bottom,
        };
        let source_node = &nodes[resolved.source_index];
        let target_node = &nodes[resolved.target_index];
        let source = port_point(source_node, resolved.source_port);
        let target = port_point(target_node, resolved.target_port);
        let source_stub = stub_point(source, resolved.source_port.side, options.port_stub);
        let target_stub = stub_point(target, resolved.target_port.side, options.port_stub);
        let source_y =
            endpoint_escape_y(source, edge.source, 0, endpoint_tracks, options.port_stub);
        let target_y =
            endpoint_escape_y(target, edge.target, 1, endpoint_tracks, options.port_stub);
        let source_x = channel_point(
            source_stub,
            source_node,
            resolved.source_port.side,
            plan.ranks[resolved.source_index],
            lane.channel_index,
            lane.channel_count,
            layer_left,
            layer_right,
            options,
        )
        .x;
        let target_x = channel_point(
            target_stub,
            target_node,
            resolved.target_port.side,
            plan.ranks[resolved.target_index],
            lane.channel_index,
            lane.channel_count,
            layer_left,
            layer_right,
            options,
        )
        .x;
        for (fixed, endpoint_y) in [(source_x, source_y), (target_x, target_y)] {
            let start = endpoint_y.min(boundary_y);
            let end = endpoint_y.max(boundary_y);
            if start != end {
                groups
                    .entry((edge.net, FloatKey(fixed)))
                    .or_default()
                    .push((start, end, edge.id, edge.source, edge.target));
            }
        }
    }
    let mut segments = Vec::<PhysicalSegment>::new();
    for ((net, FloatKey(fixed)), intervals) in &mut groups {
        intervals.sort_by(|left, right| {
            left.0
                .total_cmp(&right.0)
                .then(left.1.total_cmp(&right.1))
                .then(left.2.cmp(&right.2))
        });
        for &(start, end, _, source, target) in intervals.iter() {
            if let Some(prior) = segments.last_mut()
                && prior.net == *net
                && prior.fixed == *fixed
                && start <= prior.end
            {
                prior.end = prior.end.max(end);
            } else {
                segments.push(PhysicalSegment {
                    net: *net,
                    source,
                    target,
                    horizontal: false,
                    fixed: *fixed,
                    start,
                    end,
                });
            }
        }
    }
    segments
}

fn move_nets_to_outer_lanes(
    gap_lanes: &[BTreeMap<NetId, usize>],
    nets: &[NetId],
) -> Option<Vec<BTreeMap<NetId, usize>>> {
    if nets.is_empty() {
        return None;
    }
    let mut changed = false;
    let result = gap_lanes
        .iter()
        .map(|lanes| {
            let selected = nets
                .iter()
                .enumerate()
                .filter_map(|(priority, &net)| lanes.get(&net).map(|&lane| (priority, net, lane)))
                .collect::<Vec<_>>();
            let selected_count = selected.len();
            lanes
                .iter()
                .map(|(&net, &lane)| {
                    // This is equivalent to moving each selected net to the outer edge in
                    // priority order, without sorting and rebuilding every gap lane map.
                    let next = selected
                        .iter()
                        .position(|&(_, selected_net, _)| selected_net == net)
                        .map_or_else(
                            || {
                                lane - selected
                                    .iter()
                                    .filter(|&&(_, _, selected_lane)| selected_lane < lane)
                                    .count()
                            },
                            |position| lanes.len() - selected_count + position,
                        );
                    changed |= next != lane;
                    (net, next)
                })
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

fn select_gap_spacing_candidate(
    plan: &RoutingPlan<'_>,
    compact: Vec<EdgeGeometry>,
    adaptive: Option<Vec<EdgeGeometry>>,
) -> (Vec<EdgeGeometry>, Option<RouteQuality>, GapTrackSpacing) {
    let Some(adaptive) = adaptive else {
        return (compact, None, GapTrackSpacing::Compact);
    };
    let compact_quality = route_quality_for_plan(plan, &compact);
    let adaptive_quality = route_quality_for_plan(plan, &adaptive);
    if route_quality_cmp(adaptive_quality, compact_quality).is_lt() {
        (adaptive, Some(adaptive_quality), GapTrackSpacing::Adaptive)
    } else {
        (compact, Some(compact_quality), GapTrackSpacing::Compact)
    }
}

fn physical_route_segments<'a>(
    edges: impl Iterator<Item = &'a Edge>,
    routes: &[EdgeGeometry],
) -> (Vec<PhysicalSegment>, usize, f64) {
    struct RawSegment {
        net: u32,
        source: Endpoint,
        target: Endpoint,
        horizontal: bool,
        fixed: f64,
        start: f64,
        end: f64,
        edge: EdgeId,
    }

    let mut segments = Vec::<RawSegment>::new();
    let mut bends = Vec::new();
    for (edge, route) in edges.zip(routes) {
        for points in route.points.windows(3) {
            let first_horizontal = points[0].y == points[1].y;
            let second_horizontal = points[1].y == points[2].y;
            if first_horizontal != second_horizontal {
                bends.push((edge.net, FloatKey(points[1].x), FloatKey(points[1].y)));
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
            segments.push(RawSegment {
                net: edge.net,
                source: edge.source,
                target: edge.target,
                horizontal,
                fixed,
                start,
                end,
                edge: edge.id,
            });
        }
    }

    bends.sort_unstable();
    bends.dedup();
    segments.sort_unstable_by(|left, right| {
        left.net
            .cmp(&right.net)
            .then(left.horizontal.cmp(&right.horizontal))
            .then(left.fixed.total_cmp(&right.fixed))
            .then(left.start.total_cmp(&right.start))
            .then(left.end.total_cmp(&right.end))
            .then(left.edge.cmp(&right.edge))
    });
    let mut merged = Vec::<PhysicalSegment>::new();
    for segment in segments {
        if let Some(prior) = merged.last_mut()
            && prior.net == segment.net
            && prior.horizontal == segment.horizontal
            && prior.fixed == segment.fixed
            && segment.start <= prior.end
        {
            prior.end = prior.end.max(segment.end);
        } else {
            merged.push(PhysicalSegment {
                net: segment.net,
                source: segment.source,
                target: segment.target,
                horizontal: segment.horizontal,
                fixed: segment.fixed,
                start: segment.start,
                end: segment.end,
            });
        }
    }
    let route_length = merged
        .iter()
        .map(|segment| segment.end - segment.start)
        .sum();
    (merged, bends.len(), route_length)
}

#[cfg(test)]
fn physical_route_segments_btree_reference<'a>(
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
        .map(|segment| (segment, u64::from(segment.net)))
        .collect::<Vec<_>>();
    physical_crossing_sweep_lines(shared_endpoints, &horizontal, &vertical, |net, count| {
        if let Some(contributions) = &mut contributions {
            *contributions.entry(net as u32).or_default() += count;
        }
    })
}

#[inline(never)]
fn physical_crossing_sweep_lines(
    shared_endpoints: &HashSet<Endpoint>,
    horizontal: &[&PhysicalSegment],
    vertical: &[(&PhysicalSegment, u64)],
    mut record: impl FnMut(u64, usize),
) -> usize {
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

    let mut events = Vec::with_capacity(horizontal.len() * 2 + vertical.len());
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
    for (segment_index, (segment, _)) in vertical.iter().enumerate() {
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
                let (line, key) = vertical[segment];
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
                if count != 0 {
                    record(key, count);
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
#[cfg(test)]
fn horizontal_crossing_counts_by_net(
    plan: &RoutingPlan<'_>,
    routes: &[EdgeGeometry],
) -> (BTreeMap<NetId, usize>, RouteQuality) {
    let (_, counts, quality) = horizontal_crossing_profile_by_net(plan, routes);
    (counts, quality)
}

fn horizontal_crossing_profile_by_net(
    plan: &RoutingPlan<'_>,
    routes: &[EdgeGeometry],
) -> (Vec<PhysicalSegment>, BTreeMap<NetId, usize>, RouteQuality) {
    #[cfg(test)]
    HORIZONTAL_CROSSING_PROFILE_CALLS.with(|calls| calls.set(calls.get() + 1));
    let (segments, bends, route_length) =
        physical_route_segments(plan.edges.iter().map(|edge| edge.edge), routes);
    let mut counts = BTreeMap::<NetId, usize>::new();
    let crossings =
        physical_crossing_sweep(&plan.shared_endpoints, &segments, true, Some(&mut counts));
    (
        segments,
        counts,
        RouteQuality {
            crossings,
            bends,
            route_length,
        },
    )
}

#[cfg(test)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct RoutingReuseCounts {
    final_endpoint_tracks: usize,
    coherent_endpoint_tracks: usize,
    outer_repair_endpoint_tracks: usize,
    repair_crossing_paths: usize,
    repair_crossing_paths_recomputed: usize,
}

#[cfg(test)]
thread_local! {
    static HORIZONTAL_CROSSING_PROFILE_CALLS: Cell<usize> = const { Cell::new(0) };
    static ROUTING_REUSE_COUNTS: Cell<RoutingReuseCounts> = const {
        Cell::new(RoutingReuseCounts {
            final_endpoint_tracks: 0,
            coherent_endpoint_tracks: 0,
            outer_repair_endpoint_tracks: 0,
            repair_crossing_paths: 0,
            repair_crossing_paths_recomputed: 0,
        })
    };
}

#[cfg(test)]
fn take_horizontal_crossing_profile_calls() -> usize {
    HORIZONTAL_CROSSING_PROFILE_CALLS.with(|calls| calls.replace(0))
}

#[cfg(test)]
fn update_routing_reuse_counts(update: impl FnOnce(&mut RoutingReuseCounts)) {
    ROUTING_REUSE_COUNTS.with(|counts| {
        let mut next = counts.get();
        update(&mut next);
        counts.set(next);
    });
}

#[cfg(test)]
fn take_routing_reuse_counts() -> RoutingReuseCounts {
    ROUTING_REUSE_COUNTS.with(|counts| counts.replace(RoutingReuseCounts::default()))
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

fn lane_indices(nets: &BTreeSet<NetId>) -> BTreeMap<NetId, usize> {
    nets.iter()
        .copied()
        .enumerate()
        .map(|(index, net)| (net, index))
        .collect()
}

#[derive(Clone, Copy)]
struct OuterFanout {
    source: Endpoint,
    branches: usize,
    eligible: bool,
}

fn fanout_outer_channel_lane_indices(
    plan: &RoutingPlan<'_>,
    sparse_spans: &[Option<(usize, usize)>],
    outer_nets: &BTreeSet<NetId>,
) -> Option<BTreeMap<NetId, usize>> {
    let mut fanout = BTreeMap::<NetId, OuterFanout>::new();
    for (resolved, span) in plan.edges.iter().zip(sparse_spans) {
        if span.is_some() {
            continue;
        }
        let edge = resolved.edge;
        if let Some(access) = fanout.get_mut(&edge.net) {
            access.branches = access.branches.saturating_add(1);
            access.eligible &= edge.source == access.source;
        } else {
            fanout.insert(
                edge.net,
                OuterFanout {
                    source: edge.source,
                    branches: 1,
                    eligible: true,
                },
            );
        }
    }
    if fanout
        .values()
        .filter(|access| access.eligible && access.branches >= MIN_FANOUT_AWARE_CHANNEL_EDGES)
        .map(|access| access.branches)
        .sum::<usize>()
        < MIN_FANOUT_AWARE_OUTER_BRANCHES
    {
        return None;
    }
    let mut ordered = outer_nets
        .iter()
        .map(|&net| {
            let priority = fanout
                .get(&net)
                .filter(|access| {
                    access.eligible && access.branches >= MIN_FANOUT_AWARE_CHANNEL_EDGES
                })
                .map_or(0, |access| access.branches);
            (priority, net)
        })
        .collect::<Vec<_>>();
    ordered.sort_unstable();
    Some(
        ordered
            .into_iter()
            .enumerate()
            .map(|(index, (_, net))| (net, index))
            .collect(),
    )
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

fn outer_lane_channels_match(
    left: &BTreeMap<EdgeId, OuterLane>,
    right: &BTreeMap<EdgeId, OuterLane>,
) -> bool {
    left.len() == right.len()
        && left.iter().all(|(edge, left_lane)| {
            right.get(edge).is_some_and(|right_lane| {
                left_lane.channel_index == right_lane.channel_index
                    && left_lane.channel_count == right_lane.channel_count
            })
        })
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
    channel_lanes: &BTreeMap<NetId, usize>,
    layer_left: &[f64],
    layer_right: &[f64],
    top: f64,
    bottom: f64,
    options: LayoutOptions,
    lane_rounds: usize,
    coherent_feedback: bool,
) -> BTreeMap<u32, OuterLane> {
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
            (edge, side)
        })
        .collect::<BTreeMap<_, _>>();

    let channel_count = channel_lanes.len();
    let mut assignments = plan
        .edges
        .iter()
        .zip(sparse_spans)
        .filter(|(_, span)| span.is_none())
        .map(|(resolved, _)| {
            let edge = resolved.edge;
            (
                edge.id,
                OuterLane {
                    side: edge_sides[&edge.id],
                    side_index: 0,
                    channel_index: channel_lanes[&edge.net],
                    channel_count,
                },
            )
        })
        .collect::<BTreeMap<_, _>>();
    reindex_outer_lane_assignments(
        plan,
        nodes,
        ranks,
        sparse_spans,
        layer_left,
        layer_right,
        options,
        lane_rounds,
        &mut assignments,
    );
    assignments
}

#[allow(clippy::too_many_arguments)]
fn reindex_outer_lane_assignments(
    plan: &RoutingPlan<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    layer_left: &[f64],
    layer_right: &[f64],
    options: LayoutOptions,
    lane_rounds: usize,
    assignments: &mut BTreeMap<EdgeId, OuterLane>,
) {
    let mut top_nets = BTreeSet::new();
    let mut bottom_nets = BTreeSet::new();
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
        let lane = assignments[&edge.id];
        let channel_index = lane.channel_index;
        let source_x = channel_point(
            source_stub,
            source_node,
            source_port.side,
            ranks[source_index],
            channel_index,
            lane.channel_count,
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
            lane.channel_count,
            layer_left,
            layer_right,
            options,
        )
        .x;
        let access_by_net = match lane.side {
            OuterSide::Top => {
                top_nets.insert(edge.net);
                &mut top_access
            }
            OuterSide::Bottom => {
                bottom_nets.insert(edge.net);
                &mut bottom_access
            }
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
        let side = assignments[&edge.id].side;
        let side_index = match side {
            OuterSide::Top => top_lanes[&edge.net],
            OuterSide::Bottom => bottom_lanes[&edge.net],
        };
        assignments
            .get_mut(&edge.id)
            .expect("outer lane exists")
            .side_index = side_index;
    }
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
    gap_spacing: GapTrackSpacing,
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
                    sparse_gap_x(
                        edge.net,
                        *source_rank,
                        layer_left,
                        layer_right,
                        gap_lanes,
                        options,
                        gap_spacing,
                    ),
                    sparse_gap_x(
                        edge.net,
                        target_rank - 1,
                        layer_left,
                        layer_right,
                        gap_lanes,
                        options,
                        gap_spacing,
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

#[derive(Clone, Default)]
struct GapNetAccess {
    vertical: Vec<(f64, f64)>,
    left_y: Vec<f64>,
    right_y: Vec<f64>,
}

struct GapLaneCandidates {
    baseline: Vec<BTreeMap<u32, usize>>,
    global: Option<Vec<BTreeMap<u32, usize>>>,
    preserved_refined: Option<Vec<BTreeMap<u32, usize>>>,
    refined: Option<Vec<BTreeMap<u32, usize>>>,
}

type GapPairCosts = BTreeMap<(u32, u32), usize>;
type GapLaneOrder = (BTreeMap<u32, usize>, usize);

#[cfg(test)]
static USE_BTREE_GAP_PAIR_COSTS: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

struct DenseGapPairCosts<'a> {
    accesses: Vec<&'a GapNetAccess>,
    values: Vec<Option<usize>>,
}

impl<'a> DenseGapPairCosts<'a> {
    fn new(ordered: &[NetId], accesses: &'a BTreeMap<NetId, GapNetAccess>) -> Self {
        debug_assert!(ordered.len() <= MAX_GLOBAL_GAP_LANES);
        let accesses = ordered.iter().map(|net| &accesses[net]).collect::<Vec<_>>();
        Self {
            values: vec![None; ordered.len() * ordered.len()],
            accesses,
        }
    }

    fn cost(&mut self, left: usize, right: usize) -> usize {
        let index = left * self.accesses.len() + right;
        if let Some(cost) = self.values[index] {
            return cost;
        }
        let cost = gap_pair_crossings(self.accesses[left], self.accesses[right]);
        self.values[index] = Some(cost);
        cost
    }
}

struct GapLaneOrderCandidates {
    global: Option<GapLaneOrder>,
    preserved_refined: Option<GapLaneOrder>,
    refined: Option<GapLaneOrder>,
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
    global_candidates: bool,
    large_global_candidates: bool,
    refined_large_global_candidates: bool,
) -> GapLaneCandidates {
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
    let global_candidates = global_candidates
        && global_gap_candidate_work_within_budget(
            current_lanes,
            &accesses,
            large_global_candidates,
        );
    let mut baseline = Vec::with_capacity(current_lanes.len());
    let mut global = global_candidates.then(|| Vec::with_capacity(current_lanes.len()));
    let refined_candidates = global_candidates
        && large_global_candidates
        && refined_large_global_candidates
        && refined_large_gap_candidate_work_within_budget(current_lanes, &accesses);
    let mut refined = refined_candidates.then(|| Vec::with_capacity(current_lanes.len()));
    let mut preserved_refined = refined_candidates.then(|| Vec::with_capacity(current_lanes.len()));
    let mut changed = false;
    let mut preserved_refined_changed = false;
    let mut refined_changed = false;
    let mut total_gain = 0usize;
    let mut preserved_refined_total_gain = 0usize;
    let mut refined_total_gain = 0usize;
    for (lanes, access) in current_lanes.iter().zip(&accesses) {
        let local = crossing_aware_gap_lane_indices_with_rounds(lanes, access, lane_rounds);
        let fused_large_gap = refined.is_some() && lanes.len() > MAX_GLOBAL_GAP_LANES;
        let GapLaneOrderCandidates {
            global: mut fused_global,
            preserved_refined: mut fused_preserved_refined,
            refined: mut fused_refined,
        } = if fused_large_gap {
            refined_large_gap_hot_insertion_orders(access, &local)
        } else {
            GapLaneOrderCandidates {
                global: None,
                preserved_refined: None,
                refined: None,
            }
        };
        let mut global_lane = local.clone();
        if let Some(global) = &mut global {
            let candidate = if fused_large_gap {
                fused_global.take()
            } else {
                global_gap_lane_indices_with_rounds(
                    lanes,
                    access,
                    lane_rounds,
                    &local,
                    large_global_candidates,
                )
            };
            if let Some((candidate, gain)) = candidate {
                changed = true;
                total_gain = total_gain.saturating_add(gain);
                global_lane = candidate;
            }
            global.push(global_lane.clone());
        }
        let mut preserved_refined_lane = global_lane.clone();
        if let Some(preserved_refined) = &mut preserved_refined {
            let (candidate, gain) = if lanes.len() > MAX_GLOBAL_GAP_LANES {
                fused_preserved_refined
                    .take()
                    .unwrap_or_else(|| (local.clone(), 0))
            } else {
                (global_lane.clone(), 0)
            };
            preserved_refined_changed |= candidate != global_lane;
            preserved_refined_total_gain = preserved_refined_total_gain.saturating_add(gain);
            preserved_refined_lane = candidate;
            preserved_refined.push(preserved_refined_lane.clone());
        }
        if let Some(refined) = &mut refined {
            let (refined_lane, gain) = if lanes.len() > MAX_GLOBAL_GAP_LANES {
                fused_refined.take().unwrap_or_else(|| (local.clone(), 0))
            } else {
                (preserved_refined_lane.clone(), 0)
            };
            refined_changed |= refined_lane != preserved_refined_lane;
            refined_total_gain = refined_total_gain.saturating_add(gain);
            refined.push(refined_lane);
        }
        baseline.push(local);
    }
    let global = global.filter(|_| changed && total_gain >= MIN_GLOBAL_GAP_ORDER_GAIN);
    let preserved_refined = preserved_refined.filter(|candidate| {
        preserved_refined_changed
            && preserved_refined_total_gain >= MIN_GLOBAL_GAP_ORDER_GAIN
            && global.as_ref() != Some(candidate)
    });
    let refined = refined.filter(|candidate| {
        refined_changed
            && refined_total_gain >= MIN_GLOBAL_GAP_ORDER_GAIN
            && global.as_ref() != Some(candidate)
            && preserved_refined.as_ref() != Some(candidate)
    });
    GapLaneCandidates {
        baseline,
        global,
        preserved_refined,
        refined,
    }
}

fn global_gap_candidate_work_within_budget(
    current_lanes: &[BTreeMap<u32, usize>],
    accesses: &[BTreeMap<u32, GapNetAccess>],
    large_global_candidates: bool,
) -> bool {
    let max_lanes = if large_global_candidates {
        MAX_LARGE_GLOBAL_GAP_LANES
    } else {
        MAX_GLOBAL_GAP_LANES
    };
    let pair_budget = if large_global_candidates {
        MAX_LARGE_GLOBAL_GAP_PAIRS
    } else {
        MAX_GLOBAL_GAP_PAIRS
    };
    let access_budget = if large_global_candidates {
        MAX_LARGE_GLOBAL_GAP_ACCESS_WORK
    } else {
        MAX_GLOBAL_GAP_ACCESS_WORK
    };
    let eligible = current_lanes
        .iter()
        .zip(accesses)
        .filter(|(lanes, _)| (2..=max_lanes).contains(&lanes.len()));
    let pairs_within_budget = sum_within_limit(
        eligible.clone().map(|(lanes, _)| {
            if lanes.len() <= MAX_GLOBAL_GAP_LANES {
                lanes
                    .len()
                    .checked_mul(lanes.len() - 1)
                    .map(|ordered| ordered / 2)
                    .unwrap_or(usize::MAX)
            } else {
                lanes
                    .len()
                    .checked_mul(MAX_LARGE_GLOBAL_GAP_HOT_NETS.min(lanes.len()))
                    .and_then(|pairs| pairs.checked_mul(2))
                    .unwrap_or(usize::MAX)
            }
        }),
        pair_budget,
    );
    pairs_within_budget
        && sum_within_limit(
            eligible.map(|(lanes, access)| {
                if lanes.len() <= MAX_GLOBAL_GAP_LANES {
                    let comparisons_per_access = (lanes.len() - 1).saturating_mul(2);
                    access
                        .values()
                        .map(|net| net.vertical.len())
                        .try_fold(0usize, |total, count| {
                            count
                                .checked_mul(comparisons_per_access)
                                .and_then(|work| total.checked_add(work))
                        })
                        .unwrap_or(usize::MAX)
                } else {
                    large_gap_hot_access_work(lanes, access).unwrap_or(usize::MAX)
                }
            }),
            access_budget,
        )
}

fn refined_large_gap_candidate_work_within_budget(
    current_lanes: &[BTreeMap<u32, usize>],
    accesses: &[BTreeMap<u32, GapNetAccess>],
) -> bool {
    let eligible = current_lanes.iter().zip(accesses).filter(|(lanes, _)| {
        (MAX_GLOBAL_GAP_LANES + 1..=MAX_LARGE_GLOBAL_GAP_LANES).contains(&lanes.len())
    });
    sum_within_limit(
        eligible.clone().map(|(lanes, _)| {
            let hot_count = MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS.min(lanes.len());
            lanes
                .len()
                .checked_mul(hot_count)
                // Materialize both directional costs once. Every insertion then performs six
                // linear passes: locate, remove-shift, gather, fold, walk, and insert-shift.
                // Checked arithmetic makes the complete CPU/memory admission explicit.
                .and_then(|pairs| {
                    let precompute = pairs.checked_mul(2)?;
                    let scans = pairs
                        .checked_mul(MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS)?
                        .checked_mul(6)?;
                    precompute.checked_add(scans)
                })
                .unwrap_or(usize::MAX)
        }),
        MAX_REFINED_LARGE_GLOBAL_GAP_LANE_WORK,
    ) && sum_within_limit(
        eligible.map(|(lanes, access)| {
            large_gap_hot_access_work_with_limit(
                lanes,
                access,
                MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS,
            )
            .unwrap_or(usize::MAX)
        }),
        MAX_LARGE_GLOBAL_GAP_ACCESS_WORK,
    )
}

fn large_gap_hot_access_work(
    lanes: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
) -> Option<usize> {
    large_gap_hot_access_work_with_limit(lanes, accesses, MAX_LARGE_GLOBAL_GAP_HOT_NETS)
}

fn large_gap_hot_access_work_with_limit(
    lanes: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
    hot_limit: usize,
) -> Option<usize> {
    let hot = large_gap_hot_nets_with_limit(accesses, lanes, hot_limit);
    let hot_verticals = hot.iter().try_fold(0usize, |total, net| {
        total.checked_add(accesses.get(net).map_or(0, |access| access.vertical.len()))
    })?;
    let total_verticals = lanes.keys().try_fold(0usize, |total, net| {
        total.checked_add(accesses.get(net).map_or(0, |access| access.vertical.len()))
    })?;
    let nonhot_verticals = total_verticals.checked_sub(hot_verticals)?;
    large_gap_hot_access_work_from_counts(lanes.len(), hot.len(), hot_verticals, nonhot_verticals)
}

fn large_gap_hot_access_work_from_counts(
    lane_count: usize,
    hot_count: usize,
    hot_verticals: usize,
    nonhot_verticals: usize,
) -> Option<usize> {
    let hot_to_all = lane_count.checked_sub(1)?.checked_mul(hot_verticals)?;
    let nonhot_to_hot = hot_count.checked_mul(nonhot_verticals)?;
    // Pair vectors deliberately avoid a cross-hot cache. Each hot-hot pair is therefore scored
    // once from each hot net's insertion walk, adding one repeat of both directional costs.
    let repeated_hot_to_hot = hot_count.saturating_sub(1).checked_mul(hot_verticals)?;
    hot_to_all
        .checked_add(nonhot_to_hot)?
        .checked_add(repeated_hot_to_hot)?
        .checked_mul(2)
}

#[cfg(test)]
fn crossing_aware_gap_lane_indices(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
) -> BTreeMap<u32, usize> {
    crossing_aware_gap_lane_indices_with_rounds(current, accesses, FULL_GAP_LANE_ROUNDS)
}

#[cfg(test)]
fn crossing_aware_gap_lane_indices_btree_reference(
    current: &BTreeMap<NetId, usize>,
    accesses: &BTreeMap<NetId, GapNetAccess>,
    lane_rounds: usize,
) -> BTreeMap<NetId, usize> {
    let mut ordered = current
        .iter()
        .map(|(&net, &lane)| (lane, net))
        .collect::<Vec<_>>();
    ordered.sort_unstable();
    let mut ordered = ordered.into_iter().map(|(_, net)| net).collect::<Vec<_>>();
    let mut costs = GapPairCosts::new();
    refine_gap_lane_order(&mut ordered, lane_rounds, |left, right| {
        *costs
            .entry((left, right))
            .or_insert_with(|| gap_pair_crossings(&accesses[&left], &accesses[&right]))
    });
    ordered
        .into_iter()
        .enumerate()
        .map(|(lane, net)| (net, lane))
        .collect()
}

fn crossing_aware_gap_lane_indices_with_rounds(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
    lane_rounds: usize,
) -> BTreeMap<u32, usize> {
    let mut ordered: Vec<_> = current.iter().map(|(&net, &lane)| (lane, net)).collect();
    ordered.sort_unstable();
    let seed: Vec<_> = ordered.into_iter().map(|(_, net)| net).collect();
    let mut ordered = seed;
    let use_dense = ordered.len() <= MAX_GLOBAL_GAP_LANES && {
        #[cfg(test)]
        {
            !USE_BTREE_GAP_PAIR_COSTS.load(std::sync::atomic::Ordering::Relaxed)
        }
        #[cfg(not(test))]
        {
            true
        }
    };
    if use_dense {
        let mut costs = DenseGapPairCosts::new(&ordered, accesses);
        let mut dense_order = (0..ordered.len()).collect::<Vec<_>>();
        refine_gap_lane_order(&mut dense_order, lane_rounds, |left, right| {
            costs.cost(left, right)
        });
        ordered = dense_order
            .into_iter()
            .map(|index| ordered[index])
            .collect();
    } else {
        let mut costs = BTreeMap::new();
        refine_gap_lane_order(&mut ordered, lane_rounds, |left, right| {
            *costs
                .entry((left, right))
                .or_insert_with(|| gap_pair_crossings(&accesses[&left], &accesses[&right]))
        });
    }
    ordered
        .into_iter()
        .enumerate()
        .map(|(index, net)| (net, index))
        .collect()
}

fn global_gap_lane_indices_with_rounds(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
    lane_rounds: usize,
    baseline: &BTreeMap<u32, usize>,
    large_global_candidates: bool,
) -> Option<(BTreeMap<u32, usize>, usize)> {
    if current.len() > MAX_GLOBAL_GAP_LANES {
        return large_global_candidates
            .then(|| large_gap_hot_insertion_order(accesses, baseline))
            .flatten();
    }
    let (mut global, mut costs) = global_gap_order_seed(current, accesses)?;
    refine_gap_lane_order(&mut global, lane_rounds, |left, right| {
        *costs
            .entry((left, right))
            .or_insert_with(|| gap_pair_crossings(&accesses[&left], &accesses[&right]))
    });
    let mut baseline_order: Vec<_> = baseline.iter().map(|(&net, &lane)| (lane, net)).collect();
    baseline_order.sort_unstable();
    let baseline_order: Vec<_> = baseline_order.into_iter().map(|(_, net)| net).collect();
    let global_cost = gap_lane_order_cost(&global, |left, right| {
        *costs
            .entry((left, right))
            .or_insert_with(|| gap_pair_crossings(&accesses[&left], &accesses[&right]))
    });
    let baseline_cost = gap_lane_order_cost(&baseline_order, |left, right| {
        *costs
            .entry((left, right))
            .or_insert_with(|| gap_pair_crossings(&accesses[&left], &accesses[&right]))
    });
    if global_cost >= baseline_cost {
        return None;
    }

    Some((
        global
            .into_iter()
            .enumerate()
            .map(|(index, net)| (net, index))
            .collect(),
        baseline_cost - global_cost,
    ))
}

fn large_gap_hot_insertion_order(
    accesses: &BTreeMap<u32, GapNetAccess>,
    baseline: &BTreeMap<u32, usize>,
) -> Option<(BTreeMap<u32, usize>, usize)> {
    large_gap_hot_insertion_order_with_rounds(accesses, baseline, MAX_LARGE_GLOBAL_GAP_HOT_NETS, 1)
}

fn refined_large_gap_hot_insertion_orders(
    accesses: &BTreeMap<u32, GapNetAccess>,
    baseline: &BTreeMap<u32, usize>,
) -> GapLaneOrderCandidates {
    if !(MAX_GLOBAL_GAP_LANES + 1..=MAX_LARGE_GLOBAL_GAP_LANES).contains(&baseline.len()) {
        return GapLaneOrderCandidates {
            global: None,
            preserved_refined: None,
            refined: None,
        };
    }
    let ordered = initial_large_gap_order(baseline);
    let refined_hot =
        large_gap_hot_nets_with_limit(accesses, baseline, MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS);
    let pair_cost_table = large_gap_pair_cost_table(accesses, &ordered, &refined_hot);
    let row_width = ordered.len();
    let mut refined_order = ordered;
    let mut refined_gain = 0usize;
    let global_count = MAX_LARGE_GLOBAL_GAP_HOT_NETS.min(refined_hot.len());
    let preserved_count = MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_NETS.min(refined_hot.len());
    let mut first_round_changed = run_large_gap_hot_insertion_range(
        accesses,
        &mut refined_order,
        &refined_hot,
        Some(&pair_cost_table),
        row_width,
        0,
        global_count,
        &mut refined_gain,
    );
    let global = gap_lane_order_candidate(&refined_order, refined_gain);
    first_round_changed |= run_large_gap_hot_insertion_range(
        accesses,
        &mut refined_order,
        &refined_hot,
        Some(&pair_cost_table),
        row_width,
        global_count,
        preserved_count,
        &mut refined_gain,
    );
    let mut preserved_order = refined_order.clone();
    let mut preserved_gain = refined_gain;
    for _ in 1..MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS {
        if !run_large_gap_hot_insertion_range(
            accesses,
            &mut preserved_order,
            &refined_hot,
            Some(&pair_cost_table),
            row_width,
            0,
            preserved_count,
            &mut preserved_gain,
        ) {
            break;
        }
    }
    let preserved_refined = gap_lane_order_candidate(&preserved_order, preserved_gain);
    first_round_changed |= run_large_gap_hot_insertion_range(
        accesses,
        &mut refined_order,
        &refined_hot,
        Some(&pair_cost_table),
        row_width,
        preserved_count,
        refined_hot.len(),
        &mut refined_gain,
    );
    if first_round_changed {
        for _ in 1..MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS {
            if !run_large_gap_hot_insertion_range(
                accesses,
                &mut refined_order,
                &refined_hot,
                Some(&pair_cost_table),
                row_width,
                0,
                refined_hot.len(),
                &mut refined_gain,
            ) {
                break;
            }
        }
    }
    let refined = gap_lane_order_candidate(&refined_order, refined_gain);
    let standard_hot =
        large_gap_hot_nets_with_limit(accesses, baseline, MAX_LARGE_GLOBAL_GAP_HOT_NETS);
    debug_assert_eq!(
        standard_hot,
        refined_hot[..MAX_LARGE_GLOBAL_GAP_HOT_NETS.min(refined_hot.len())]
    );
    GapLaneOrderCandidates {
        global,
        preserved_refined,
        refined,
    }
}

fn large_gap_hot_insertion_order_with_rounds(
    accesses: &BTreeMap<u32, GapNetAccess>,
    baseline: &BTreeMap<u32, usize>,
    hot_limit: usize,
    rounds: usize,
) -> Option<(BTreeMap<u32, usize>, usize)> {
    large_gap_hot_insertion_orders_with_rounds(accesses, baseline, hot_limit, rounds, None).1
}

fn large_gap_hot_insertion_orders_with_rounds(
    accesses: &BTreeMap<u32, GapNetAccess>,
    baseline: &BTreeMap<u32, usize>,
    hot_limit: usize,
    rounds: usize,
    snapshot_hot_limit: Option<usize>,
) -> (Option<GapLaneOrder>, Option<GapLaneOrder>) {
    if !(MAX_GLOBAL_GAP_LANES + 1..=MAX_LARGE_GLOBAL_GAP_LANES).contains(&baseline.len()) {
        return (None, None);
    }
    let ordered = initial_large_gap_order(baseline);
    let hot = large_gap_hot_nets_with_limit(accesses, baseline, hot_limit);
    let pair_cost_table = (rounds > 1).then(|| large_gap_pair_cost_table(accesses, &ordered, &hot));
    let row_width = ordered.len();
    run_large_gap_hot_insertion_rounds(
        accesses,
        ordered,
        &hot,
        pair_cost_table.as_deref(),
        row_width,
        rounds,
        snapshot_hot_limit,
    )
}

fn initial_large_gap_order(baseline: &BTreeMap<u32, usize>) -> Vec<(u32, usize)> {
    let mut ordered = baseline
        .iter()
        .map(|(&net, &lane)| (lane, net))
        .collect::<Vec<_>>();
    ordered.sort_unstable();
    ordered
        .into_iter()
        .enumerate()
        .map(|(original_lane, (_, net))| (net, original_lane))
        .collect()
}

fn large_gap_pair_cost_table(
    accesses: &BTreeMap<u32, GapNetAccess>,
    ordered: &[(u32, usize)],
    hot: &[u32],
) -> Vec<(usize, usize)> {
    hot.iter()
        .flat_map(|&hot_net| {
            ordered.iter().map(move |&(other, _)| {
                (
                    gap_pair_crossings(&accesses[&hot_net], &accesses[&other]),
                    gap_pair_crossings(&accesses[&other], &accesses[&hot_net]),
                )
            })
        })
        .collect()
}

fn run_large_gap_hot_insertion_rounds(
    accesses: &BTreeMap<u32, GapNetAccess>,
    mut ordered: Vec<(u32, usize)>,
    hot: &[u32],
    pair_cost_table: Option<&[(usize, usize)]>,
    row_width: usize,
    rounds: usize,
    snapshot_hot_limit: Option<usize>,
) -> (Option<GapLaneOrder>, Option<GapLaneOrder>) {
    let mut total_gain = 0usize;
    let mut snapshot = None;
    for round in 0..rounds {
        let snapshot_index = (round == 0)
            .then_some(snapshot_hot_limit)
            .flatten()
            .unwrap_or(0)
            .min(hot.len());
        let mut changed = run_large_gap_hot_insertion_range(
            accesses,
            &mut ordered,
            hot,
            pair_cost_table,
            row_width,
            0,
            if snapshot_index == 0 {
                hot.len()
            } else {
                snapshot_index
            },
            &mut total_gain,
        );
        if snapshot_index > 0 {
            snapshot = gap_lane_order_candidate(&ordered, total_gain);
            changed |= run_large_gap_hot_insertion_range(
                accesses,
                &mut ordered,
                hot,
                pair_cost_table,
                row_width,
                snapshot_index,
                hot.len(),
                &mut total_gain,
            );
        }
        if !changed {
            break;
        }
    }
    (snapshot, gap_lane_order_candidate(&ordered, total_gain))
}

#[allow(clippy::too_many_arguments)]
fn run_large_gap_hot_insertion_range(
    accesses: &BTreeMap<u32, GapNetAccess>,
    ordered: &mut Vec<(u32, usize)>,
    hot: &[u32],
    pair_cost_table: Option<&[(usize, usize)]>,
    row_width: usize,
    start: usize,
    end: usize,
    total_gain: &mut usize,
) -> bool {
    let mut changed = false;
    for hot_index in start..end {
        let hot_net = hot[hot_index];
        let Some(current_index) = ordered.iter().position(|&(net, _)| net == hot_net) else {
            continue;
        };
        let hot_entry = ordered.remove(current_index);
        // The one-round production candidate materializes costs without a pair table, as in
        // PR #42. The deeper candidate reuses its dense row on the second round.
        let pair_costs = ordered
            .iter()
            .map(|&(other, original_lane)| {
                pair_cost_table.map_or_else(
                    || {
                        (
                            gap_pair_crossings(&accesses[&hot_net], &accesses[&other]),
                            gap_pair_crossings(&accesses[&other], &accesses[&hot_net]),
                        )
                    },
                    |table| table[hot_index * row_width + original_lane],
                )
            })
            .collect::<Vec<_>>();
        let mut insertion_cost = pair_costs
            .iter()
            .map(|&(hot_before, _)| hot_before)
            .fold(0usize, usize::saturating_add);
        let mut best_index = 0usize;
        let mut best_cost = insertion_cost;
        let mut current_cost = if current_index == 0 {
            insertion_cost
        } else {
            0
        };
        for (index, &(hot_before, other_before)) in pair_costs.iter().enumerate() {
            insertion_cost = insertion_cost
                .saturating_sub(hot_before)
                .saturating_add(other_before);
            if index + 1 == current_index {
                current_cost = insertion_cost;
            }
            if insertion_cost < best_cost {
                best_cost = insertion_cost;
                best_index = index + 1;
            }
        }
        if current_cost > best_cost {
            *total_gain = total_gain.saturating_add(current_cost - best_cost);
            ordered.insert(best_index, hot_entry);
            changed = true;
        } else {
            ordered.insert(current_index, hot_entry);
        }
    }
    changed
}

fn gap_lane_order_candidate(ordered: &[(u32, usize)], total_gain: usize) -> Option<GapLaneOrder> {
    (total_gain > 0).then(|| {
        (
            ordered
                .iter()
                .enumerate()
                .map(|(lane, &(net, _))| (net, lane))
                .collect(),
            total_gain,
        )
    })
}

#[cfg(test)]
fn large_gap_hot_insertion_order_btree_reference(
    accesses: &BTreeMap<u32, GapNetAccess>,
    baseline: &BTreeMap<u32, usize>,
    hot_limit: usize,
    rounds: usize,
) -> Option<(BTreeMap<u32, usize>, usize)> {
    if !(MAX_GLOBAL_GAP_LANES + 1..=MAX_LARGE_GLOBAL_GAP_LANES).contains(&baseline.len()) {
        return None;
    }
    let mut ordered = baseline
        .iter()
        .map(|(&net, &lane)| (lane, net))
        .collect::<Vec<_>>();
    ordered.sort_unstable();
    let mut ordered = ordered.into_iter().map(|(_, net)| net).collect::<Vec<_>>();
    let hot = large_gap_hot_nets_with_limit(accesses, baseline, hot_limit);
    let mut costs = GapPairCosts::new();
    let mut total_gain = 0usize;
    for _ in 0..rounds {
        let mut changed = false;
        for &hot_net in &hot {
            let Some(current_index) = ordered.iter().position(|&net| net == hot_net) else {
                continue;
            };
            ordered.remove(current_index);
            let pair_costs = ordered
                .iter()
                .map(|&other| {
                    let hot_before = *costs.entry((hot_net, other)).or_insert_with(|| {
                        gap_pair_crossings(&accesses[&hot_net], &accesses[&other])
                    });
                    let other_before = *costs.entry((other, hot_net)).or_insert_with(|| {
                        gap_pair_crossings(&accesses[&other], &accesses[&hot_net])
                    });
                    (hot_before, other_before)
                })
                .collect::<Vec<_>>();
            let mut insertion_cost = pair_costs
                .iter()
                .map(|&(hot_before, _)| hot_before)
                .fold(0usize, usize::saturating_add);
            let mut best_index = 0usize;
            let mut best_cost = insertion_cost;
            let mut current_cost = if current_index == 0 {
                insertion_cost
            } else {
                0
            };
            for (index, &(hot_before, other_before)) in pair_costs.iter().enumerate() {
                insertion_cost = insertion_cost
                    .saturating_sub(hot_before)
                    .saturating_add(other_before);
                if index + 1 == current_index {
                    current_cost = insertion_cost;
                }
                if insertion_cost < best_cost {
                    best_cost = insertion_cost;
                    best_index = index + 1;
                }
            }
            if current_cost > best_cost {
                total_gain = total_gain.saturating_add(current_cost - best_cost);
                ordered.insert(best_index, hot_net);
                changed = true;
            } else {
                ordered.insert(current_index, hot_net);
            }
        }
        if !changed {
            break;
        }
    }
    (total_gain > 0).then(|| {
        (
            ordered
                .into_iter()
                .enumerate()
                .map(|(lane, net)| (net, lane))
                .collect(),
            total_gain,
        )
    })
}

#[cfg(test)]
fn large_gap_hot_nets(
    accesses: &BTreeMap<u32, GapNetAccess>,
    lanes: &BTreeMap<u32, usize>,
) -> Vec<u32> {
    large_gap_hot_nets_with_limit(accesses, lanes, MAX_LARGE_GLOBAL_GAP_HOT_NETS)
}

fn large_gap_hot_nets_with_limit(
    accesses: &BTreeMap<u32, GapNetAccess>,
    lanes: &BTreeMap<u32, usize>,
    limit: usize,
) -> Vec<u32> {
    let mut hot = accesses
        .iter()
        .filter(|(net, _)| lanes.contains_key(net))
        .map(|(&net, access)| {
            let vertical_span = access
                .vertical
                .iter()
                .map(|&(low, high)| high - low)
                .sum::<f64>();
            (net, access.vertical.len(), vertical_span)
        })
        .collect::<Vec<_>>();
    hot.sort_by(|left, right| {
        right
            .1
            .cmp(&left.1)
            .then_with(|| right.2.total_cmp(&left.2))
            .then(left.0.cmp(&right.0))
    });
    hot.truncate(limit);
    hot.into_iter().map(|(net, _, _)| net).collect()
}

fn global_gap_order_seed(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
) -> Option<(Vec<u32>, GapPairCosts)> {
    let mut seed: Vec<_> = current.iter().map(|(&net, &lane)| (lane, net)).collect();
    seed.sort_unstable();
    let seed: Vec<_> = seed.into_iter().map(|(_, net)| net).collect();
    if !(2..=MAX_GLOBAL_GAP_LANES).contains(&seed.len()) {
        return None;
    }

    // Each ordered pair is a weighted tournament edge. A single saturating signed out-minus-in
    // balance gives every net a total-order key even at numeric limits. The resulting deterministic
    // non-local seed can escape strict adjacent-swap plateaus; the existing bounded adjacent descent
    // then refines it under the same proxy objective.
    let mut costs = BTreeMap::new();
    let mut scores = BTreeMap::<u32, i64>::new();
    for (index, &left) in seed.iter().enumerate() {
        for &right in &seed[index + 1..] {
            let left_before_right = gap_pair_crossings(&accesses[&left], &accesses[&right]);
            let right_before_left = gap_pair_crossings(&accesses[&right], &accesses[&left]);
            costs.insert((left, right), left_before_right);
            costs.insert((right, left), right_before_left);
            let pair_balance = i64::try_from(left_before_right)
                .unwrap_or(i64::MAX)
                .saturating_sub(i64::try_from(right_before_left).unwrap_or(i64::MAX));
            scores
                .entry(left)
                .and_modify(|score| *score = score.saturating_add(pair_balance))
                .or_insert(pair_balance);
            scores
                .entry(right)
                .and_modify(|score| *score = score.saturating_sub(pair_balance))
                .or_insert_with(|| 0i64.saturating_sub(pair_balance));
        }
    }
    let seed_lanes = current;
    let mut global = seed;
    global.sort_by(|left, right| {
        scores[left]
            .cmp(&scores[right])
            .then(seed_lanes[left].cmp(&seed_lanes[right]))
            .then(left.cmp(right))
    });
    Some((global, costs))
}

fn refine_gap_lane_order<T: Copy>(
    ordered: &mut [T],
    lane_rounds: usize,
    mut pair_cost: impl FnMut(T, T) -> usize,
) {
    for _ in 0..lane_rounds {
        let mut changed = false;
        for index in 0..ordered.len().saturating_sub(1) {
            let left = ordered[index];
            let right = ordered[index + 1];
            let current_cost = pair_cost(left, right);
            let swapped_cost = pair_cost(right, left);
            if swapped_cost < current_cost {
                ordered.swap(index, index + 1);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
}

fn gap_lane_order_cost(ordered: &[u32], mut pair_cost: impl FnMut(NetId, NetId) -> usize) -> usize {
    let mut total = 0usize;
    for (index, &left) in ordered.iter().enumerate() {
        for &right in &ordered[index + 1..] {
            total = total.saturating_add(pair_cost(left, right));
        }
    }
    total
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
    options: LayoutOptions,
    gap_spacing: GapTrackSpacing,
) -> Vec<Point> {
    let port_stub = options.port_stub;
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
    let mut x = sparse_gap_x(
        net,
        source_rank,
        layer_left,
        layer_right,
        gap_lanes,
        options,
        gap_spacing,
    );
    push_point(
        &mut points,
        Point {
            x,
            y: source_escape_y,
        },
    );

    for (rank, &y) in (source_rank + 1..target_rank).zip(crossing_path) {
        push_point(&mut points, Point { x, y });
        x = sparse_gap_x(
            net,
            rank,
            layer_left,
            layer_right,
            gap_lanes,
            options,
            gap_spacing,
        );
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
    options: LayoutOptions,
    gap_spacing: GapTrackSpacing,
) -> f64 {
    let lanes = &gap_lanes[gap];
    let lane_fraction = (lanes[&net] + 1) as f64 / (lanes.len() + 1) as f64;
    let gap_left = layer_right[gap];
    let gap_right = layer_left[gap + 1];
    let gap_width = gap_right - gap_left;
    let compact_x = gap_left + gap_width * (0.55 + 0.15 * lane_fraction);
    if gap_spacing == GapTrackSpacing::Compact {
        return compact_x;
    }
    let available_left = gap_left + gap_width * 0.55;
    let available_right = gap_right - options.port_stub;
    let available_width = available_right - available_left;
    if available_width <= gap_width * 0.15 {
        return compact_x;
    }
    let desired_width = options.route_lane_gap * (lanes.len() + 1) as f64;
    let width = desired_width.min(available_width);
    let preferred_left = gap_left + gap_width * 0.625 - width / 2.0;
    let left = preferred_left.clamp(available_left, available_right - width);
    left + width * lane_fraction
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
    use std::collections::{BTreeMap, BTreeSet, HashSet};

    use crate::{
        Edge, EdgeGeometry, Endpoint, Graph, LayoutOptions, Node, NodeGeometry, Point, Port,
        PortSide, validation::validate_and_index,
    };

    use super::{
        FULL_OUTER_LANE_ROUNDS, GapNetAccess, GapTrackSpacing, MAX_CROSSING_REPAIR_EDGES,
        MAX_CROSSING_REPAIR_NODES, MAX_CROSSING_REPAIR_PATH_STATES,
        MAX_CROSSING_REPAIR_ROUTE_POINTS, MIN_CROSSING_REPAIR_NET, MIN_CROSSING_REPAIR_TOTAL,
        OuterLane, OuterNetAccess, OuterSide, PhysicalSegment, RoutingPlan,
        align_crossing_path_staircases, build_endpoint_tracks,
        candidate_route_points_within_budget, crossing_aware_gap_lane_indices,
        crossing_aware_gap_lane_indices_btree_reference, crossing_aware_outer_lane_indices,
        crossing_paths_have_unrelated_collinear_tracks, crossing_repair_within_budget,
        crossing_track_y, distance_transform, fanout_outer_channel_lane_indices,
        free_interval_containing, global_gap_candidate_work_within_budget,
        global_gap_lane_indices_with_rounds, global_gap_order_seed, has_split_feedback_net,
        horizontal_crossing_counts_by_net, lane_indices, large_gap_hot_access_work,
        large_gap_hot_access_work_from_counts, large_gap_hot_insertion_order_btree_reference,
        large_gap_hot_insertion_order_with_rounds, large_gap_hot_nets,
        large_gap_hot_nets_with_limit, move_nets_to_outer_lanes, outer_lane_assignments,
        outer_lane_channels_match, physical_crossing_sweep, physical_crossing_sweep_lines,
        physical_route_segments, physical_route_segments_btree_reference, port_point,
        refined_large_gap_candidate_work_within_budget, refined_large_gap_hot_insertion_orders,
        repair_crossing_heavy_net, repair_selection_adds_new_nets, route_edges,
        route_edges_with_lane_rounds, route_edges_with_lane_rounds_and_global,
        route_planned_candidates, route_planned_candidates_with_quality_options,
        route_planned_candidates_with_sparse_global, route_planned_edges, route_quality,
        route_quality_cmp, route_quality_for_plan, route_supplemental_edges,
        select_crossing_repair_nets, select_outer_side_repairs, shortest_crossing_path,
        sparse_channel_route, sparse_gap_x, sum_within_limit,
        take_horizontal_crossing_profile_calls, take_routing_reuse_counts,
        vertical_horizontal_crossings,
    };

    #[test]
    fn zero_length_vertical_access_has_no_crossings() {
        assert_eq!(
            vertical_horizontal_crossings(&[(20.0, 20.0)], &[10.0, 20.0, 30.0]),
            0
        );
    }

    fn align_crossing_path_staircases_for_test(
        plan: &RoutingPlan<'_>,
        sparse_spans: &[Option<(usize, usize)>],
        free_by_rank: &[Vec<(f64, f64)>],
        crossing_paths: &[Option<Vec<f64>>],
    ) -> Option<(Vec<Option<Vec<f64>>>, usize)> {
        let rank_count = free_by_rank.len();
        let layer_left = (0..rank_count)
            .map(|rank| rank as f64 * 100.0)
            .collect::<Vec<_>>();
        let layer_right = layer_left
            .iter()
            .map(|left| left + 20.0)
            .collect::<Vec<_>>();
        let nets = plan
            .edges
            .iter()
            .map(|resolved| resolved.edge.net)
            .collect::<BTreeSet<_>>();
        let gap_lanes = (1..rank_count)
            .map(|_| {
                nets.iter()
                    .copied()
                    .enumerate()
                    .map(|(lane, net)| (net, lane))
                    .collect::<BTreeMap<_, _>>()
            })
            .collect::<Vec<_>>();
        align_crossing_path_staircases(
            plan,
            sparse_spans,
            free_by_rank,
            crossing_paths,
            &layer_left,
            &layer_right,
            &gap_lanes,
            LayoutOptions::default(),
            GapTrackSpacing::Compact,
        )
    }

    #[test]
    fn staircase_alignment_uses_one_interior_ordinate_across_overlapping_free_spans() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 0,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: 1,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                },
            ],
            edges: vec![Edge {
                id: 0,
                source: Endpoint { node: 0, port: 0 },
                target: Endpoint { node: 1, port: 0 },
                net: 7,
                participates_in_ranking: true,
            }],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 4]);
        let (aligned, transitions) = align_crossing_path_staircases_for_test(
            &plan,
            &[Some((0, 4))],
            &[
                Vec::new(),
                vec![(0.0, 10.0)],
                vec![(2.0, 9.0)],
                vec![(4.0, 8.0)],
                Vec::new(),
            ],
            &[Some(vec![1.0, 5.0, 7.0])],
        )
        .expect("single-net alignment stays separated");

        let aligned = aligned[0].as_ref().unwrap();
        assert_eq!(transitions, 2);
        assert_eq!(aligned[0], aligned[1]);
        assert_eq!(aligned[1], aligned[2]);
        assert!(aligned[0] > 4.0 && aligned[0] < 8.0);
    }

    #[test]
    fn staircase_alignment_preserves_paths_without_a_shared_free_span() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 0,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: 1,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                },
            ],
            edges: vec![Edge {
                id: 0,
                source: Endpoint { node: 0, port: 0 },
                target: Endpoint { node: 1, port: 0 },
                net: 7,
                participates_in_ranking: true,
            }],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 3]);
        let original = vec![1.0, 5.0];
        let (aligned, transitions) = align_crossing_path_staircases_for_test(
            &plan,
            &[Some((0, 3))],
            &[Vec::new(), vec![(0.0, 2.0)], vec![(4.0, 6.0)], Vec::new()],
            &[Some(original.clone())],
        )
        .expect("unaligned single-net path stays separated");

        assert_eq!(transitions, 0);
        assert_eq!(aligned[0], Some(original));
    }

    #[test]
    fn staircase_alignment_propagates_long_backbone_to_unequal_rank_sink_prefix() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 0,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: 1,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: 2,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                },
            ],
            edges: vec![
                Edge {
                    id: 0,
                    source: Endpoint { node: 0, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 1,
                    source: Endpoint { node: 0, port: 0 },
                    target: Endpoint { node: 1, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
            ],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 3, 5]);
        let (aligned, transitions) = align_crossing_path_staircases_for_test(
            &plan,
            &[Some((0, 5)), Some((0, 3))],
            &[
                Vec::new(),
                vec![(0.0, 10.0)],
                vec![(4.0, 10.0)],
                vec![(8.0, 10.0)],
                vec![(0.0, 6.0)],
                Vec::new(),
            ],
            &[Some(vec![1.0, 5.0, 9.0, 3.0]), Some(vec![1.0, 5.0])],
        )
        .expect("shared net has no unrelated track collision");

        let long = aligned[0].as_ref().unwrap();
        let short = aligned[1].as_ref().unwrap();
        assert_eq!(transitions, 2);
        assert_eq!(short, &long[..short.len()]);
        assert_eq!(long[0], long[1]);
        assert_eq!(long[1], long[2]);
        assert!(long[0] > 8.0 && long[0] < 10.0);
    }

    #[test]
    fn staircase_alignment_rejects_narrow_multi_net_track_collision_before_emission() {
        let nodes = (0..8)
            .map(|id| Node {
                id,
                width: 20.0,
                height: 20.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: if id < 4 {
                        PortSide::East
                    } else {
                        PortSide::West
                    },
                    offset: 10.0,
                }],
            })
            .collect();
        let edges = (0..4)
            .map(|net| Edge {
                id: net,
                source: Endpoint { node: net, port: 0 },
                target: Endpoint {
                    node: net + 4,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            })
            .collect();
        let graph = Graph { nodes, edges };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 0, 0, 0, 3, 3, 3, 3]);
        let candidate = align_crossing_path_staircases_for_test(
            &plan,
            &[Some((0, 3)); 4],
            &[
                Vec::new(),
                vec![(0.0, 0.000_001)],
                vec![(0.0, 0.000_001)],
                Vec::new(),
            ],
            &[
                Some(vec![0.000_000_40, 0.000_000_50]),
                Some(vec![0.000_000_45, 0.000_000_55]),
                Some(vec![0.000_000_50, 0.000_000_60]),
                Some(vec![0.000_000_55, 0.000_000_65]),
            ],
        );

        assert!(candidate.is_none());
    }

    fn two_sparse_net_graph() -> Graph {
        Graph {
            nodes: (0..4)
                .map(|id| Node {
                    id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: if id < 2 {
                            PortSide::East
                        } else {
                            PortSide::West
                        },
                        offset: 10.0,
                    }],
                })
                .collect(),
            edges: (0..2)
                .map(|net| Edge {
                    id: net,
                    source: Endpoint { node: net, port: 0 },
                    target: Endpoint {
                        node: net + 2,
                        port: 0,
                    },
                    net,
                    participates_in_ranking: true,
                })
                .collect(),
        }
    }

    #[test]
    fn staircase_alignment_rejects_adjacent_rank_lane_reversal_contact() {
        let graph = two_sparse_net_graph();
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 0, 3, 3]);
        let gap_lanes = [
            BTreeMap::from([(0, 0), (1, 1)]),
            BTreeMap::from([(0, 1), (1, 0)]),
            BTreeMap::from([(0, 0), (1, 1)]),
        ];

        assert!(crossing_paths_have_unrelated_collinear_tracks(
            &plan,
            &[Some((0, 3)); 2],
            &[Some(vec![5.0, 6.0]), Some(vec![7.0, 5.0])],
            &[0.0, 100.0, 200.0, 300.0],
            &[20.0, 120.0, 220.0, 320.0],
            &gap_lanes,
            LayoutOptions::default(),
            GapTrackSpacing::Compact,
        ));
    }

    #[test]
    fn staircase_alignment_treats_signed_zero_tracks_as_collinear() {
        let graph = two_sparse_net_graph();
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 0, 3, 3]);
        let gap_lanes = [
            BTreeMap::from([(0, 0), (1, 1)]),
            BTreeMap::from([(0, 0), (1, 1)]),
            BTreeMap::from([(0, 0), (1, 1)]),
        ];

        assert!(crossing_paths_have_unrelated_collinear_tracks(
            &plan,
            &[Some((0, 3)); 2],
            &[Some(vec![-0.0, 6.0]), Some(vec![0.0, 7.0])],
            &[0.0, 100.0, 200.0, 300.0],
            &[20.0, 120.0, 220.0, 320.0],
            &gap_lanes,
            LayoutOptions::default(),
            GapTrackSpacing::Compact,
        ));
    }

    #[test]
    fn free_interval_lookup_preserves_ordered_boundary_semantics() {
        let intervals = [(0.0, 1.0), (3.0, 4.0), (10.0, 12.0)];
        assert_eq!(free_interval_containing(&intervals, 0.0), Some((0.0, 1.0)));
        assert_eq!(free_interval_containing(&intervals, 1.0), Some((0.0, 1.0)));
        assert_eq!(free_interval_containing(&intervals, 3.0), Some((3.0, 4.0)));
        assert_eq!(
            free_interval_containing(&intervals, 11.0),
            Some((10.0, 12.0))
        );
        assert_eq!(free_interval_containing(&intervals, 2.0), None);
        assert_eq!(free_interval_containing(&intervals, 13.0), None);
        assert_eq!(
            free_interval_containing(&[(0.0, 1.0), (1.0, 2.0)], 1.0),
            Some((0.0, 1.0))
        );
    }

    fn outer_side_route_fixture(
        branches: u32,
        hot_y: f64,
        other_y: f64,
    ) -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let mut nodes = Vec::new();
        let mut geometry = Vec::new();
        let mut ranks = Vec::new();
        let mut add_node = |id, x, y, side| {
            nodes.push(Node {
                id,
                width: 20.0,
                height: 20.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side,
                    offset: 10.0,
                }],
            });
            geometry.push(NodeGeometry {
                id,
                x,
                y,
                width: 20.0,
                height: 20.0,
            });
            ranks.push(0);
        };
        add_node(0, 0.0, hot_y, PortSide::East);
        add_node(1, 200.0, hot_y, PortSide::West);
        for branch in 0..branches {
            add_node(2 + branch * 2, 50.0, other_y, PortSide::East);
            add_node(3 + branch * 2, 150.0, other_y, PortSide::West);
        }
        let mut edges = vec![Edge {
            id: 0,
            source: Endpoint { node: 0, port: 0 },
            target: Endpoint { node: 1, port: 0 },
            net: 1,
            participates_in_ranking: false,
        }];
        edges.extend((0..branches).map(|branch| Edge {
            id: 1 + branch,
            source: Endpoint {
                node: 2 + branch * 2,
                port: 0,
            },
            target: Endpoint {
                node: 3 + branch * 2,
                port: 0,
            },
            net: 100 + branch,
            participates_in_ranking: false,
        }));
        (Graph { nodes, edges }, geometry, ranks)
    }

    #[test]
    fn endpoint_tracks_depend_only_on_outer_channel_coordinates() {
        let baseline = BTreeMap::from([(
            7,
            OuterLane {
                side: OuterSide::Top,
                side_index: 0,
                channel_index: 3,
                channel_count: 5,
            },
        )]);
        let side_only = BTreeMap::from([(
            7,
            OuterLane {
                side: OuterSide::Bottom,
                side_index: 4,
                channel_index: 3,
                channel_count: 5,
            },
        )]);
        let different_channel = BTreeMap::from([(
            7,
            OuterLane {
                side: OuterSide::Bottom,
                side_index: 4,
                channel_index: 2,
                channel_count: 5,
            },
        )]);

        assert!(outer_lane_channels_match(&baseline, &side_only));
        assert!(!outer_lane_channels_match(&baseline, &different_channel));
    }

    #[test]
    fn outer_side_repair_is_exactly_scored_and_deterministic() {
        let (graph, geometry, ranks) = outer_side_route_fixture(256, 70.0, 90.0);
        let options = LayoutOptions::default();
        let indexed = validate_and_index(&graph, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        take_routing_reuse_counts();
        let routed = route_planned_candidates(&plan, &geometry, options, true);
        let reuse_counts = take_routing_reuse_counts();
        assert_eq!(reuse_counts.final_endpoint_tracks, 1);
        assert_eq!(reuse_counts.outer_repair_endpoint_tracks, 1);
        assert_eq!(reuse_counts.repair_crossing_paths, 1);
        assert_eq!(reuse_counts.repair_crossing_paths_recomputed, 0);
        let baseline = routed.primary_quality.unwrap();
        let (candidate, repair) = routed.repair.as_ref().expect("fixture activates repair");

        assert_eq!(
            routed.repair_outer_sides,
            vec![(1, OuterSide::Top), (100, OuterSide::Top)]
        );
        assert_eq!(baseline.crossings, 98_304);
        assert_eq!(candidate.crossings, 97_284);
        assert!(candidate.crossings < baseline.crossings);
        assert!(route_quality_cmp(*candidate, baseline).is_lt());
        assert_eq!(route_quality(&indexed, &routed.primary), baseline);
        assert_eq!(route_quality(&indexed, repair), *candidate);
        for (primary, repaired) in routed.primary.iter().zip(repair) {
            assert_eq!(primary.id, repaired.id);
            assert_eq!(
                primary
                    .points
                    .iter()
                    .map(|point| point.x)
                    .collect::<Vec<_>>(),
                repaired
                    .points
                    .iter()
                    .map(|point| point.x)
                    .collect::<Vec<_>>()
            );
            for route in [primary, repaired] {
                assert!(
                    route.points.windows(2).all(|points| {
                        (points[0].x == points[1].x) ^ (points[0].y == points[1].y)
                    })
                );
            }
        }

        let mut permuted_graph = graph;
        permuted_graph.nodes.reverse();
        permuted_graph.edges.reverse();
        let permuted_indexed = validate_and_index(&permuted_graph, options).unwrap();
        let permuted_plan = RoutingPlan::new(&permuted_indexed, &ranks);
        let permuted = route_planned_candidates(&permuted_plan, &geometry, options, true);
        assert_eq!(permuted.repair_outer_sides, routed.repair_outer_sides);
        assert_eq!(permuted.primary_quality, routed.primary_quality);
        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted.repair, routed.repair);
    }

    #[test]
    fn sparse_and_outer_repairs_share_one_exact_deterministic_candidate() {
        let (mut graph, mut geometry, mut ranks) = outer_side_route_fixture(256, 70.0, 90.0);
        add_crossing_repair_fixture(&mut graph, &mut geometry, &mut ranks);
        let options = LayoutOptions::default();
        let indexed = validate_and_index(&graph, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let routed = route_planned_candidates(&plan, &geometry, options, true);
        let baseline = routed.primary_quality.unwrap();
        let (candidate, repair) = routed.repair.as_ref().expect("combined repair activates");

        assert!(!routed.repair_nets.is_empty());
        assert!(!routed.repair_outer_sides.is_empty());
        assert!(route_quality_cmp(*candidate, baseline).is_lt());
        assert_eq!(route_quality(&indexed, &routed.primary), baseline);
        assert_eq!(route_quality(&indexed, repair), *candidate);

        let mut permuted_graph = graph;
        permuted_graph.nodes.reverse();
        permuted_graph.edges.reverse();
        let permuted_indexed = validate_and_index(&permuted_graph, options).unwrap();
        let permuted_plan = RoutingPlan::new(&permuted_indexed, &ranks);
        let permuted = route_planned_candidates(&permuted_plan, &geometry, options, true);
        assert_eq!(permuted.repair_nets, routed.repair_nets);
        assert_eq!(permuted.repair_outer_sides, routed.repair_outer_sides);
        assert_eq!(permuted.primary_quality, routed.primary_quality);
        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted.repair, routed.repair);
    }

    #[test]
    fn outer_side_repair_gain_gate_is_literal_and_inclusive() {
        assert_eq!(super::MIN_OUTER_SIDE_REPAIR_GAIN, 64);
        assert_eq!(super::MAX_BATCHED_OUTER_SIDE_REPAIRS, 2);
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 0,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: 1,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                },
            ],
            edges: vec![Edge {
                id: 0,
                source: Endpoint { node: 0, port: 0 },
                target: Endpoint { node: 1, port: 0 },
                net: 1,
                participates_in_ranking: false,
            }],
        };
        let nodes = vec![
            NodeGeometry {
                id: 0,
                x: 0.0,
                y: 70.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 1,
                x: 200.0,
                y: 70.0,
                width: 20.0,
                height: 20.0,
            },
        ];
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &[0, 0]);
        let spans = vec![None];
        let outer_lanes = BTreeMap::from([(
            0,
            OuterLane {
                side: OuterSide::Bottom,
                side_index: 0,
                channel_index: 0,
                channel_count: 1,
            },
        )]);
        let endpoint_tracks = build_endpoint_tracks(
            &plan,
            &nodes,
            &[0, 0],
            &spans,
            &[0.0],
            &[220.0],
            &[],
            &outer_lanes,
            LayoutOptions::default(),
            GapTrackSpacing::Compact,
        );
        let select = |crossings: u32| {
            let segments = (0..crossings)
                .map(|net| PhysicalSegment {
                    net: 100 + net,
                    source: Endpoint {
                        node: 1_000 + net * 2,
                        port: 0,
                    },
                    target: Endpoint {
                        node: 1_001 + net * 2,
                        port: 0,
                    },
                    horizontal: true,
                    fixed: 100.0,
                    start: 230.0,
                    end: 270.0,
                })
                .collect::<Vec<_>>();
            select_outer_side_repairs(
                &plan,
                &nodes,
                &spans,
                &[0.0],
                &[220.0],
                &[],
                &outer_lanes,
                0.0,
                110.0,
                LayoutOptions::default(),
                &segments,
                GapTrackSpacing::Compact,
                &endpoint_tracks,
            )
        };

        assert!(select(63).is_empty());
        assert_eq!(select(64), vec![(1, OuterSide::Top)]);
    }

    #[test]
    fn outer_side_repair_caps_equal_gain_nets_and_skips_side_ties() {
        let (graph, nodes, ranks) = outer_side_route_fixture(2, 70.0, 70.0);
        let options = LayoutOptions::default();
        let indexed = validate_and_index(&graph, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let spans = vec![None; graph.edges.len()];
        let outer_lanes = graph
            .edges
            .iter()
            .enumerate()
            .map(|(channel_index, edge)| {
                (
                    edge.id,
                    OuterLane {
                        side: OuterSide::Bottom,
                        side_index: channel_index,
                        channel_index,
                        channel_count: graph.edges.len(),
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();
        let endpoint_tracks = build_endpoint_tracks(
            &plan,
            &nodes,
            &ranks,
            &spans,
            &[0.0],
            &[220.0],
            &[],
            &outer_lanes,
            options,
            GapTrackSpacing::Compact,
        );
        let horizontals = |ys: &[f64]| {
            ys.iter()
                .enumerate()
                .map(|(index, &fixed)| PhysicalSegment {
                    net: 1_000 + index as u32,
                    source: Endpoint {
                        node: 10_000 + index as u32 * 2,
                        port: 0,
                    },
                    target: Endpoint {
                        node: 10_001 + index as u32 * 2,
                        port: 0,
                    },
                    horizontal: true,
                    fixed,
                    start: -1_000.0,
                    end: 1_000.0,
                })
                .collect::<Vec<_>>()
        };
        let select = |segments: &[PhysicalSegment]| {
            select_outer_side_repairs(
                &plan,
                &nodes,
                &spans,
                &[0.0],
                &[220.0],
                &[],
                &outer_lanes,
                0.0,
                110.0,
                options,
                segments,
                GapTrackSpacing::Compact,
                &endpoint_tracks,
            )
        };

        let bottom_only = horizontals(&[100.0; 64]);
        assert_eq!(
            select(&bottom_only),
            vec![(1, OuterSide::Top), (100, OuterSide::Top)]
        );
        let tied = horizontals(&[vec![40.0; 64], vec![100.0; 64]].concat());
        assert!(select(&tied).is_empty());
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
    fn global_gap_order_escapes_an_adjacent_swap_plateau_and_preserves_ties() {
        let current = BTreeMap::from([(0, 0), (1, 1), (2, 2)]);
        let accesses = BTreeMap::from([
            (
                0,
                GapNetAccess {
                    vertical: vec![(0.0, 40.0)],
                    left_y: vec![0.0],
                    right_y: vec![40.0],
                },
            ),
            (
                1,
                GapNetAccess {
                    vertical: vec![(40.0, 80.0)],
                    left_y: vec![40.0],
                    right_y: vec![80.0],
                },
            ),
            (
                2,
                GapNetAccess {
                    vertical: vec![(0.0, 80.0)],
                    left_y: vec![80.0],
                    right_y: vec![0.0],
                },
            ),
        ]);
        let baseline = crossing_aware_gap_lane_indices(&current, &accesses);
        let (global, gain) = global_gap_lane_indices_with_rounds(
            &current,
            &accesses,
            super::FULL_GAP_LANE_ROUNDS,
            &baseline,
            false,
        )
        .expect("global seed escapes the strict adjacent plateau");

        assert_eq!(baseline, current);
        assert_eq!(global, BTreeMap::from([(1, 0), (2, 1), (0, 2)]));
        assert_eq!(gain, 1);

        let tied_order = BTreeMap::from([(0, 1), (1, 2), (2, 0)]);
        let tied_access = GapNetAccess {
            vertical: vec![(0.0, 20.0)],
            left_y: vec![10.0],
            right_y: vec![10.0],
        };
        let tied = BTreeMap::from([
            (0, tied_access.clone()),
            (1, tied_access.clone()),
            (2, tied_access),
        ]);
        let (tied_seed, _) = global_gap_order_seed(&tied_order, &tied).unwrap();
        assert_eq!(tied_seed, vec![2, 0, 1]);
        assert!(
            global_gap_lane_indices_with_rounds(
                &tied_order,
                &tied,
                super::FULL_GAP_LANE_ROUNDS,
                &tied_order,
                false,
            )
            .is_none(),
            "equal nonzero pair costs must retain the non-ID stable lane order"
        );
    }

    fn global_gap_route_fixture() -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let patterns = [(0.0, 40.0), (40.0, 80.0), (80.0, 0.0)];
        let mut nodes = Vec::new();
        let mut geometry = Vec::new();
        let mut ranks = Vec::new();
        let mut edges = Vec::new();
        for (net, &(source_y, target_y)) in patterns.iter().enumerate() {
            for branch in 0..16u32 {
                let source_id = (net as u32 * 16 + branch) * 2;
                let target_id = source_id + 1;
                nodes.push(Node {
                    id: source_id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                });
                geometry.push(NodeGeometry {
                    id: source_id,
                    x: 0.0,
                    y: source_y - 10.0,
                    width: 20.0,
                    height: 20.0,
                });
                ranks.push(0);
                nodes.push(Node {
                    id: target_id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                });
                geometry.push(NodeGeometry {
                    id: target_id,
                    x: 100.0,
                    y: target_y - 10.0,
                    width: 20.0,
                    height: 20.0,
                });
                ranks.push(1);
                edges.push(Edge {
                    id: net as u32 * 16 + branch,
                    source: Endpoint {
                        node: source_id,
                        port: 0,
                    },
                    target: Endpoint {
                        node: target_id,
                        port: 0,
                    },
                    net: net as u32,
                    participates_in_ranking: true,
                });
            }
        }
        (Graph { nodes, edges }, geometry, ranks)
    }

    fn large_global_gap_route_fixture() -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let normal_nets = 64u32;
        let hot_net = normal_nets;
        let hot_branches = 40u32;
        let mut nodes = Vec::new();
        let mut geometry = Vec::new();
        let mut ranks = Vec::new();
        let mut edges = Vec::new();
        let mut edge_id = 0u32;
        let mut add_edge = |net: u32, source_y: f64, target_y: f64| {
            let source_id = edge_id * 2;
            let target_id = source_id + 1;
            for (id, side, x, y, rank) in [
                (source_id, PortSide::East, 0.0, source_y, 0),
                (target_id, PortSide::West, 100.0, target_y, 1),
            ] {
                nodes.push(Node {
                    id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side,
                        offset: 10.0,
                    }],
                });
                geometry.push(NodeGeometry {
                    id,
                    x,
                    y: y - 10.0,
                    width: 20.0,
                    height: 20.0,
                });
                ranks.push(rank);
            }
            edges.push(Edge {
                id: edge_id,
                source: Endpoint {
                    node: source_id,
                    port: 0,
                },
                target: Endpoint {
                    node: target_id,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            });
            edge_id += 1;
        };
        for net in 0..normal_nets {
            add_edge(net, 0.0, 100.0);
        }
        for _ in 0..hot_branches {
            add_edge(hot_net, 50.0, 150.0);
        }
        (Graph { nodes, edges }, geometry, ranks)
    }

    fn global_gap_gain_fixture(gap_count: usize) -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let source_y = [0.0, 20.0, 20.0];
        let target_y = [20.0, 0.0, 20.0];
        let mut nodes = Vec::new();
        let mut geometry = Vec::new();
        let mut ranks = Vec::new();
        let mut edges = Vec::new();
        for gap in 0..gap_count as u32 {
            for (lane, &target_y) in target_y.iter().enumerate() {
                let edge_id = gap * 3 + lane as u32;
                let source_id = edge_id * 2;
                let target_id = source_id + 1;
                nodes.push(Node {
                    id: source_id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                });
                geometry.push(NodeGeometry {
                    id: source_id,
                    x: gap as f64 * 100.0,
                    y: source_y[lane] - 10.0,
                    width: 20.0,
                    height: 20.0,
                });
                ranks.push(gap as usize);
                nodes.push(Node {
                    id: target_id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                });
                geometry.push(NodeGeometry {
                    id: target_id,
                    x: (gap + 1) as f64 * 100.0,
                    y: target_y - 10.0,
                    width: 20.0,
                    height: 20.0,
                });
                ranks.push(gap as usize + 1);
                edges.push(Edge {
                    id: edge_id,
                    source: Endpoint {
                        node: source_id,
                        port: 0,
                    },
                    target: Endpoint {
                        node: target_id,
                        port: 0,
                    },
                    net: edge_id,
                    participates_in_ranking: true,
                });
            }
        }
        (Graph { nodes, edges }, geometry, ranks)
    }

    fn pad_global_gap_fixture_nodes(
        graph: &mut Graph,
        geometry: &mut Vec<NodeGeometry>,
        ranks: &mut Vec<usize>,
        node_count: usize,
    ) {
        while graph.nodes.len() < node_count {
            let id = graph.nodes.len() as u32;
            graph.nodes.push(Node {
                id,
                width: 20.0,
                height: 20.0,
                cycle_breaker: false,
                ports: Vec::new(),
            });
            geometry.push(NodeGeometry {
                id,
                x: 0.0,
                y: id as f64 * 30.0,
                width: 20.0,
                height: 20.0,
            });
            ranks.push(0);
        }
    }

    fn assert_global_gap_route_candidate() {
        let (graph, geometry, ranks) = global_gap_route_fixture();
        let options = LayoutOptions {
            port_stub: 1e-3,
            ..LayoutOptions::default()
        };
        let indexed = validate_and_index(&graph, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let stable = route_planned_candidates(&plan, &geometry, options, false);
        let routed = route_planned_candidates_with_sparse_global(
            &plan, &geometry, options, false, true, false,
        );

        assert_eq!(routed.primary, stable.primary);
        assert_eq!(routed.alternatives.len(), 1);
        let (candidate_quality, candidate) = &routed.alternatives[0];
        assert_ne!(candidate, &routed.primary);
        assert_eq!(route_quality(&indexed, candidate), *candidate_quality);
        let primary_quality = route_quality(&indexed, &routed.primary);
        assert!(candidate_quality.crossings < primary_quality.crossings);
        assert!(route_quality_cmp(*candidate_quality, primary_quality).is_lt());

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let indexed = validate_and_index(&permuted, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let permuted = route_planned_candidates_with_sparse_global(
            &plan, &geometry, options, false, true, false,
        );
        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted.alternatives, routed.alternatives);
    }

    #[test]
    fn global_gap_route_candidate_is_exactly_scored_and_deterministic() {
        assert_global_gap_route_candidate();
    }

    #[test]
    fn large_global_gap_route_candidate_is_finished_exactly_and_deterministically() {
        let (graph, geometry, ranks) = large_global_gap_route_fixture();
        let options = LayoutOptions {
            port_stub: 1e-3,
            ..LayoutOptions::default()
        };
        let indexed = validate_and_index(&graph, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let stable = route_planned_candidates(&plan, &geometry, options, false);
        let routed = route_planned_candidates_with_sparse_global(
            &plan, &geometry, options, false, true, true,
        );

        assert_eq!(routed.primary, stable.primary);
        assert_eq!(routed.alternatives.len(), 1);
        let (candidate_quality, candidate) = &routed.alternatives[0];
        assert_ne!(candidate, &routed.primary);
        assert_eq!(route_quality(&indexed, candidate), *candidate_quality);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let indexed = validate_and_index(&permuted, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let permuted = route_planned_candidates_with_sparse_global(
            &plan, &geometry, options, false, true, true,
        );
        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted.alternatives, routed.alternatives);
    }

    #[test]
    fn global_gap_order_enforces_per_gap_and_aggregate_work_gates() {
        let candidates = |count: u32| {
            let current = (0..count)
                .enumerate()
                .map(|(lane, net)| (net, lane))
                .collect::<BTreeMap<_, _>>();
            let mut accesses = (0..count)
                .map(|net| (net, GapNetAccess::default()))
                .collect::<BTreeMap<_, _>>();
            if count >= 2 {
                accesses.get_mut(&0).unwrap().vertical.push((0.0, 20.0));
                accesses.get_mut(&1).unwrap().left_y.push(10.0);
            }
            global_gap_lane_indices_with_rounds(&current, &accesses, 0, &current, false)
        };
        assert!(candidates(1).is_none());
        assert!(candidates(2).is_some());
        assert!(candidates(32).is_some());
        assert!(candidates(33).is_none());

        assert_eq!(super::MAX_GLOBAL_GAP_PAIRS, 32_768);
        assert_eq!(super::MAX_GLOBAL_GAP_ACCESS_WORK, 500_000);
        let twenty_lanes = (0..20u32)
            .enumerate()
            .map(|(lane, net)| (net, lane))
            .collect::<BTreeMap<_, _>>();
        let twenty_accesses = (0..20u32)
            .map(|net| (net, GapNetAccess::default()))
            .collect::<BTreeMap<_, _>>();
        assert!(global_gap_candidate_work_within_budget(
            &vec![twenty_lanes.clone(); 172],
            &vec![twenty_accesses.clone(); 172],
            false,
        ));
        assert!(!global_gap_candidate_work_within_budget(
            &vec![twenty_lanes; 173],
            &vec![twenty_accesses; 173],
            false,
        ));
        let two_lanes = vec![BTreeMap::from([(0, 0), (1, 1)])];
        let access_budget = |vertical_count| {
            vec![BTreeMap::from([
                (
                    0,
                    GapNetAccess {
                        vertical: vec![(0.0, 1.0); vertical_count],
                        ..GapNetAccess::default()
                    },
                ),
                (1, GapNetAccess::default()),
            ])]
        };
        assert!(global_gap_candidate_work_within_budget(
            &two_lanes,
            &access_budget(250_000),
            false,
        ));
        assert!(!global_gap_candidate_work_within_budget(
            &two_lanes,
            &access_budget(250_001),
            false,
        ));

        assert_eq!(super::MIN_GLOBAL_GAP_ORDER_GAIN, 256);
        assert_eq!(super::MAX_CROSSING_REPAIR_NODES, 2_000);
        let route = |gap_count, node_count| {
            let (mut graph, mut geometry, mut ranks) = global_gap_gain_fixture(gap_count);
            pad_global_gap_fixture_nodes(&mut graph, &mut geometry, &mut ranks, node_count);
            let options = LayoutOptions {
                port_stub: 1e-3,
                ..LayoutOptions::default()
            };
            let indexed = validate_and_index(&graph, options).unwrap();
            let plan = RoutingPlan::new(&indexed, &ranks);
            route_edges_with_lane_rounds_and_global(
                &plan, &geometry, options, 0, 0, false, false, true, false,
            )
            .alternatives
            .len()
        };
        assert_eq!(route(255, 255 * 6), 0);
        assert_eq!(route(256, 256 * 6), 1);
        assert_eq!(route(256, 2_000), 1);
        assert_eq!(route(256, 2_001), 0);
    }

    #[test]
    fn large_gap_work_gate_enforces_lane_pair_access_and_overflow_boundaries() {
        assert_eq!(super::MAX_LARGE_GLOBAL_GAP_HOT_NETS, 32);
        assert_eq!(super::MAX_LARGE_GLOBAL_GAP_LANES, 705);
        assert_eq!(super::MAX_LARGE_GLOBAL_GAP_PAIRS, 262_144);
        assert_eq!(super::MAX_LARGE_GLOBAL_GAP_ACCESS_WORK, 2_000_000);
        assert_eq!(super::MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_NETS, 64);
        assert_eq!(super::MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS, 2);
        assert_eq!(super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS, 256);
        assert_eq!(super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS, 5);
        assert_eq!(super::MAX_REFINED_LARGE_GLOBAL_GAP_LANE_WORK, 12_094_980);
        let lanes = |count: u32| {
            (0..count)
                .enumerate()
                .map(|(lane, net)| (net, lane))
                .collect::<BTreeMap<_, _>>()
        };
        let empty_accesses = |count: u32| {
            (0..count)
                .map(|net| (net, GapNetAccess::default()))
                .collect::<BTreeMap<_, _>>()
        };
        let moving_accesses = |count: u32| {
            (0..count)
                .map(|net| {
                    let access = if net == count - 1 {
                        GapNetAccess {
                            vertical: vec![(0.0, 1.0), (2.0, 3.0)],
                            left_y: vec![100.0],
                            right_y: vec![0.0],
                        }
                    } else {
                        GapNetAccess {
                            vertical: vec![(50.0, 150.0)],
                            ..GapNetAccess::default()
                        }
                    };
                    (net, access)
                })
                .collect::<BTreeMap<_, _>>()
        };

        for count in [33, 705] {
            let current = lanes(count);
            let accesses = moving_accesses(count);
            assert!(
                global_gap_lane_indices_with_rounds(&current, &accesses, 0, &current, true)
                    .is_some()
            );
        }
        let oversized_count = 706;
        let oversized = lanes(oversized_count);
        assert!(
            global_gap_lane_indices_with_rounds(
                &oversized,
                &moving_accesses(oversized_count),
                0,
                &oversized,
                true,
            )
            .is_none()
        );

        let max_lanes = lanes(705);
        let max_accesses = empty_accesses(705);
        assert!(global_gap_candidate_work_within_budget(
            &vec![max_lanes.clone(); 5],
            &vec![max_accesses.clone(); 5],
            true,
        ));
        assert!(!global_gap_candidate_work_within_budget(
            &vec![max_lanes; 6],
            &vec![max_accesses; 6],
            true,
        ));

        let refined_lanes = lanes(705);
        let refined_accesses = empty_accesses(705);
        assert!(refined_large_gap_candidate_work_within_budget(
            &vec![refined_lanes.clone(); 2],
            &vec![refined_accesses.clone(); 2],
        ));
        assert!(!refined_large_gap_candidate_work_within_budget(
            &vec![refined_lanes; 3],
            &vec![refined_accesses; 3],
        ));

        let thirty_three = lanes(33);
        let mut exact_accesses = empty_accesses(33);
        exact_accesses.get_mut(&0).unwrap().vertical = vec![(0.0, 1.0); 15_779];
        for net in 1..32 {
            exact_accesses.get_mut(&net).unwrap().vertical = vec![(0.0, 1.0); 3];
        }
        exact_accesses.get_mut(&32).unwrap().vertical = vec![(0.0, 1.0); 2];
        assert_eq!(
            large_gap_hot_access_work(&thirty_three, &exact_accesses),
            Some(super::MAX_LARGE_GLOBAL_GAP_ACCESS_WORK)
        );
        assert!(global_gap_candidate_work_within_budget(
            std::slice::from_ref(&thirty_three),
            std::slice::from_ref(&exact_accesses),
            true,
        ));
        exact_accesses
            .get_mut(&0)
            .unwrap()
            .vertical
            .push((0.0, 1.0));
        assert!(!global_gap_candidate_work_within_budget(
            std::slice::from_ref(&thirty_three),
            std::slice::from_ref(&exact_accesses),
            true,
        ));
        assert!(
            large_gap_hot_access_work_from_counts(usize::MAX, 32, 2, 0).is_none(),
            "checked arithmetic must reject overflow"
        );
        assert_eq!(large_gap_hot_access_work_from_counts(33, 0, 0, 0), Some(0));
    }

    #[test]
    fn large_gap_hot_insertion_is_bounded_and_deterministic() {
        let count = 40u32;
        let current = (0..count)
            .enumerate()
            .map(|(lane, net)| (net, lane))
            .collect::<BTreeMap<_, _>>();
        let hot = count - 1;
        let accesses = (0..count)
            .map(|net| {
                let access = if net == hot {
                    GapNetAccess {
                        vertical: vec![(0.0, 1.0), (2.0, 3.0)],
                        left_y: vec![100.0],
                        right_y: vec![0.0],
                    }
                } else {
                    GapNetAccess {
                        vertical: vec![(50.0, 150.0)],
                        ..GapNetAccess::default()
                    }
                };
                (net, access)
            })
            .collect::<BTreeMap<_, _>>();

        assert!(
            global_gap_lane_indices_with_rounds(&current, &accesses, 0, &current, false).is_none(),
            "large-gap insertion requires the explicit large candidate gate"
        );

        let (candidate, gain) =
            global_gap_lane_indices_with_rounds(&current, &accesses, 0, &current, true)
                .expect("hot net should move across the large gap");
        assert_eq!(candidate[&hot], 0);
        assert_eq!(gain, (count - 1) as usize);

        let mut lanes = candidate.values().copied().collect::<Vec<_>>();
        lanes.sort_unstable();
        assert_eq!(lanes, (0..count as usize).collect::<Vec<_>>());

        let oversized = (0..=super::MAX_LARGE_GLOBAL_GAP_LANES as u32)
            .enumerate()
            .map(|(lane, net)| (net, lane))
            .collect::<BTreeMap<_, _>>();
        assert!(
            global_gap_lane_indices_with_rounds(&oversized, &accesses, 0, &oversized, true)
                .is_none()
        );

        let tied_accesses = (0..count)
            .rev()
            .map(|net| {
                (
                    net,
                    GapNetAccess {
                        vertical: vec![(0.0, 1.0)],
                        ..GapNetAccess::default()
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();
        assert_eq!(
            large_gap_hot_nets(&tied_accesses, &current),
            (0..super::MAX_LARGE_GLOBAL_GAP_HOT_NETS as u32).collect::<Vec<_>>()
        );
        assert_eq!(
            large_gap_hot_nets_with_limit(&tied_accesses, &current, count as usize),
            (0..count).collect::<Vec<_>>()
        );
        let mut cutoff = tied_accesses;
        cutoff.get_mut(&32).unwrap().vertical.push((2.0, 3.0));
        let selected = large_gap_hot_nets(&cutoff, &current);
        assert_eq!(selected[0], 32);
        assert!(selected.contains(&30));
        assert!(!selected.contains(&31));
    }

    #[test]
    fn refined_large_gap_insertion_extends_the_bounded_hot_set_deterministically() {
        let count = 80u32;
        let current = (0..count)
            .enumerate()
            .map(|(lane, net)| (net, lane))
            .collect::<BTreeMap<_, _>>();
        let accesses = (0..count)
            .map(|net| {
                let access = if net >= 40 {
                    GapNetAccess {
                        vertical: vec![(0.0, 1.0), (2.0, 3.0)],
                        left_y: vec![100.0],
                        right_y: vec![0.0],
                    }
                } else {
                    GapNetAccess {
                        vertical: vec![(50.0, 150.0)],
                        ..GapNetAccess::default()
                    }
                };
                (net, access)
            })
            .collect::<BTreeMap<_, _>>();

        let (baseline, baseline_gain) = large_gap_hot_insertion_order_with_rounds(
            &accesses,
            &current,
            super::MAX_LARGE_GLOBAL_GAP_HOT_NETS,
            1,
        )
        .unwrap();
        let preserved_refined = large_gap_hot_insertion_order_with_rounds(
            &accesses,
            &current,
            super::MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_NETS,
            super::MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS,
        )
        .expect("preserved refined search should improve the standard candidate");
        let refined = large_gap_hot_insertion_order_with_rounds(
            &accesses,
            &current,
            super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS,
            super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS,
        )
        .expect("expanded hot set should move the remaining hot nets");
        let fused = refined_large_gap_hot_insertion_orders(&accesses, &current);
        let expected_global = Some((baseline.clone(), baseline_gain));
        assert_eq!(fused.global, expected_global);
        assert_eq!(
            fused.global,
            large_gap_hot_insertion_order_btree_reference(
                &accesses,
                &current,
                super::MAX_LARGE_GLOBAL_GAP_HOT_NETS,
                1,
            )
        );
        assert_eq!(fused.preserved_refined, Some(preserved_refined.clone()));
        assert_eq!(
            fused.preserved_refined,
            large_gap_hot_insertion_order_btree_reference(
                &accesses,
                &current,
                super::MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_NETS,
                super::MAX_PRESERVED_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS,
            )
        );
        assert_eq!(fused.refined, Some(refined.clone()));
        assert_eq!(
            fused.refined,
            large_gap_hot_insertion_order_btree_reference(
                &accesses,
                &current,
                super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS,
                super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS,
            )
        );
        assert_ne!(refined.0, baseline);
        assert!(refined.1 > baseline_gain);
        assert!(refined.0[&79] < baseline[&79]);
        assert_eq!(
            large_gap_hot_insertion_order_with_rounds(
                &accesses,
                &current,
                super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_NETS,
                super::MAX_REFINED_LARGE_GLOBAL_GAP_HOT_ROUNDS,
            ),
            Some(refined)
        );
    }

    #[test]
    fn global_gap_order_skips_graphs_with_outer_routes() {
        let (mut graph, geometry, ranks) = global_gap_route_fixture();
        graph.edges.push(Edge {
            id: 10_000,
            source: Endpoint { node: 1, port: 0 },
            target: Endpoint { node: 3, port: 0 },
            net: 10_000,
            participates_in_ranking: false,
        });
        let options = LayoutOptions {
            port_stub: 1e-3,
            ..LayoutOptions::default()
        };
        let indexed = validate_and_index(&graph, options).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let routed = route_planned_candidates_with_sparse_global(
            &plan, &geometry, options, false, true, false,
        );

        assert!(routed.alternatives.is_empty());
    }

    #[test]
    fn batched_hot_net_move_preserves_lane_permutations_and_priority_order() {
        let current = vec![
            BTreeMap::from([(1, 1), (2, 0), (3, 2)]),
            BTreeMap::from([(1, 0), (2, 2), (3, 1)]),
            BTreeMap::from([(1, 2), (2, 1), (3, 0), (4, 3)]),
        ];

        let moved = move_nets_to_outer_lanes(&current, &[2, 1]).unwrap();

        assert_eq!(moved[0], BTreeMap::from([(1, 2), (2, 1), (3, 0)]));
        assert_eq!(moved[1], BTreeMap::from([(1, 2), (2, 1), (3, 0)]));
        assert_eq!(moved[2], BTreeMap::from([(1, 3), (2, 2), (3, 0), (4, 1)]));
        for (before, after) in current.iter().zip(&moved) {
            assert_eq!(
                before.keys().collect::<Vec<_>>(),
                after.keys().collect::<Vec<_>>()
            );
            let mut lanes = after.values().copied().collect::<Vec<_>>();
            lanes.sort_unstable();
            assert_eq!(lanes, (0..after.len()).collect::<Vec<_>>());
        }
        assert!(move_nets_to_outer_lanes(&moved, &[2, 1]).is_none());
        assert!(move_nets_to_outer_lanes(&current, &[]).is_none());
        assert_eq!(
            move_nets_to_outer_lanes(&current, &[2]).unwrap()[0],
            BTreeMap::from([(1, 0), (2, 2), (3, 1)])
        );
    }

    #[test]
    fn repair_selector_honors_thresholds_ties_and_movable_runner_up() {
        assert!(repair_selection_adds_new_nets(2, 2));
        assert!(!repair_selection_adds_new_nets(4, 2));
        assert!(repair_selection_adds_new_nets(4, 3));

        let lanes = vec![BTreeMap::from([(1, 2), (2, 0), (3, 1)])];
        let counts = BTreeMap::from([
            (1, MIN_CROSSING_REPAIR_NET + 20),
            (2, MIN_CROSSING_REPAIR_NET),
            (3, MIN_CROSSING_REPAIR_NET),
        ]);

        assert_eq!(
            select_crossing_repair_nets(
                MIN_CROSSING_REPAIR_TOTAL,
                &counts,
                &lanes,
                super::MAX_BATCHED_CROSSING_REPAIR_NETS,
            ),
            vec![2, 3]
        );
        assert_eq!(
            select_crossing_repair_nets(
                MIN_CROSSING_REPAIR_TOTAL - 1,
                &counts,
                &lanes,
                super::MAX_BATCHED_CROSSING_REPAIR_NETS,
            ),
            Vec::<u32>::new()
        );
        assert_eq!(
            select_crossing_repair_nets(
                MIN_CROSSING_REPAIR_TOTAL,
                &BTreeMap::from([(2, MIN_CROSSING_REPAIR_NET - 1)]),
                &lanes,
                super::MAX_BATCHED_CROSSING_REPAIR_NETS,
            ),
            Vec::<u32>::new()
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

        assert!(candidate_route_points_within_budget(&vec![
            None;
            MAX_CROSSING_REPAIR_EDGES
        ]));
        assert!(!candidate_route_points_within_budget(&vec![
            None;
            MAX_CROSSING_REPAIR_EDGES
                + 1
        ]));
        let exact_sparse_span = (MAX_CROSSING_REPAIR_ROUTE_POINTS - 8) / 2;
        assert!(candidate_route_points_within_budget(&[Some((
            0,
            exact_sparse_span,
        ))]));
        assert!(!candidate_route_points_within_budget(&[Some((
            0,
            exact_sparse_span + 1,
        ))]));
        assert!(!candidate_route_points_within_budget(&[Some((
            0,
            usize::MAX,
        ))]));
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
    fn production_repair_batches_two_hot_nets_in_one_exact_candidate() {
        const SIDE: u32 = 100;
        const TARGET_Y_ORDER: [u32; SIDE as usize] = [
            81, 59, 6, 63, 25, 72, 93, 95, 68, 87, 29, 60, 55, 64, 5, 94, 78, 49, 0, 58, 67, 28,
            57, 92, 80, 88, 76, 71, 30, 35, 65, 26, 51, 73, 77, 90, 86, 97, 75, 70, 13, 23, 31, 3,
            98, 37, 16, 69, 56, 85, 46, 66, 82, 42, 33, 47, 44, 24, 50, 20, 21, 48, 89, 11, 74, 12,
            40, 45, 96, 41, 22, 84, 7, 18, 52, 91, 54, 27, 19, 99, 17, 8, 79, 4, 83, 39, 15, 36,
            14, 1, 61, 9, 2, 43, 38, 10, 32, 62, 53, 34,
        ];
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
                        node: SIDE + id,
                        port: 0,
                    },
                    net: id,
                    participates_in_ranking: true,
                })
                .collect(),
        };
        let nodes = (0..SIDE * 2)
            .map(|id| NodeGeometry {
                id,
                x: if id < SIDE { 0.0 } else { 200.0 },
                y: if id < SIDE {
                    f64::from(id) * 30.0
                } else {
                    f64::from(TARGET_Y_ORDER[(id - SIDE) as usize]) * 30.0
                },
                width: 10.0,
                height: 10.0,
            })
            .collect::<Vec<_>>();
        let ranks = (0..SIDE * 2)
            .map(|id| usize::from(id >= SIDE))
            .collect::<Vec<_>>();
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        take_routing_reuse_counts();
        let routed = route_planned_candidates(&plan, &nodes, LayoutOptions::default(), true);
        let reuse_counts = take_routing_reuse_counts();
        assert_eq!(reuse_counts.repair_crossing_paths, 0);
        assert_eq!(reuse_counts.repair_crossing_paths_recomputed, 1);
        let baseline = routed.primary_quality.unwrap();
        let (candidate, repair) = routed.repair.as_ref().expect("fixture activates repair");

        assert_eq!(routed.repair_nets, vec![17, 12]);
        assert_eq!(baseline.crossings, 3_906);
        assert_eq!(candidate.crossings, 3_818);
        assert!(route_quality_cmp(*candidate, baseline).is_lt());
        assert_eq!(route_quality(&indexed, &routed.primary), baseline);
        assert_eq!(route_quality(&indexed, repair), *candidate);

        take_horizontal_crossing_profile_calls();
        let deeper = route_planned_candidates_with_quality_options(
            &plan,
            &nodes,
            LayoutOptions::default(),
            true,
            false,
            false,
            false,
            false,
            true,
        );
        assert_eq!(
            take_horizontal_crossing_profile_calls(),
            1,
            "both repairs must share one baseline attribution profile",
        );
        assert_eq!(deeper.primary, routed.primary);
        assert_eq!(deeper.primary_quality, routed.primary_quality);
        assert_eq!(deeper.repair, routed.repair);
        let (deeper_quality, deeper_routes) = deeper
            .alternatives
            .last()
            .expect("fixture activates the deeper repair");
        assert_eq!(route_quality(&indexed, deeper_routes), *deeper_quality);

        let oversized_spans = vec![Some((0, 500)); SIDE as usize];
        let oversized_free = vec![Vec::new(); 501];
        let synthetic_lanes = vec![
            (0..SIDE)
                .enumerate()
                .map(|(lane, net)| (net, lane))
                .collect::<BTreeMap<_, _>>(),
        ];
        let bounded = repair_crossing_heavy_net(
            &plan,
            &nodes,
            &oversized_spans,
            &[],
            &BTreeMap::new(),
            0,
            &oversized_free,
            &[],
            &[],
            &synthetic_lanes,
            &BTreeMap::new(),
            0.0,
            0.0,
            LayoutOptions::default(),
            FULL_OUTER_LANE_ROUNDS,
            &routed.primary,
            &BTreeMap::new(),
            &[],
            false,
            None,
            GapTrackSpacing::Compact,
            super::MAX_BATCHED_CROSSING_REPAIR_NETS,
        );
        assert_eq!(bounded.selected_nets, vec![17, 12]);
        assert!(bounded.selected_outer_sides.is_empty());
        assert!(bounded.candidate.is_none());
        assert!(!bounded.candidate_lanes_built);
        assert!(!bounded.candidate_emitted);

        let empty_physical_segments = Vec::new();
        let empty_crossing_counts = BTreeMap::new();
        let no_selection = repair_crossing_heavy_net(
            &plan,
            &nodes,
            &oversized_spans,
            &[],
            &BTreeMap::new(),
            0,
            &oversized_free,
            &[],
            &[],
            &synthetic_lanes,
            &BTreeMap::new(),
            0.0,
            0.0,
            LayoutOptions::default(),
            FULL_OUTER_LANE_ROUNDS,
            &routed.primary,
            &BTreeMap::new(),
            &[],
            false,
            Some((&empty_physical_segments, &empty_crossing_counts, baseline)),
            GapTrackSpacing::Compact,
            super::MAX_BATCHED_CROSSING_REPAIR_NETS,
        );
        assert!(no_selection.selected_nets.is_empty());
        assert!(no_selection.selected_outer_sides.is_empty());
        assert!(no_selection.candidate.is_none());
        assert!(!no_selection.candidate_lanes_built);
        assert!(!no_selection.candidate_emitted);

        let mut permuted_graph = graph.clone();
        permuted_graph.nodes.reverse();
        permuted_graph.edges.reverse();
        let permuted_indexed =
            validate_and_index(&permuted_graph, LayoutOptions::default()).unwrap();
        let permuted_plan = RoutingPlan::new(&permuted_indexed, &ranks);
        let permuted =
            route_planned_candidates(&permuted_plan, &nodes, LayoutOptions::default(), true);

        assert_eq!(permuted.repair_nets, routed.repair_nets);
        assert_eq!(permuted.repair_outer_sides, routed.repair_outer_sides);
        assert_eq!(permuted.primary_quality, routed.primary_quality);
        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted.repair, routed.repair);
    }

    fn fanout_candidate_fixture(
        fanout_branches: u32,
        other_nets: u32,
    ) -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let mut nodes = Vec::new();
        let mut geometry = Vec::new();
        let mut ranks = Vec::new();
        let mut add_node =
            |id: u32, x: f64, y: f64, height: f64, rank: usize, side: Option<PortSide>| {
                nodes.push(Node {
                    id,
                    width: 20.0,
                    height,
                    cycle_breaker: false,
                    ports: side
                        .map(|side| {
                            vec![Port {
                                id: 0,
                                side,
                                offset: if matches!(side, PortSide::East | PortSide::West) {
                                    10.0
                                } else {
                                    5.0
                                },
                            }]
                        })
                        .unwrap_or_default(),
                });
                geometry.push(NodeGeometry {
                    id,
                    x,
                    y,
                    width: 20.0,
                    height,
                });
                ranks.push(rank);
            };

        add_node(0, 0.0, 30.0, 20.0, 0, Some(PortSide::East));
        for branch in 0..fanout_branches {
            add_node(
                1 + branch,
                240.0,
                20.0 + f64::from(branch) * 18.0,
                12.0,
                2,
                Some(PortSide::West),
            );
        }
        let blocker = 1 + fanout_branches;
        add_node(
            blocker,
            110.0,
            0.0,
            (f64::from(fanout_branches) * 18.0 + 100.0).max(900.0),
            1,
            None,
        );
        for branch in 0..other_nets {
            add_node(
                blocker + 1 + branch,
                0.0,
                320.0 + f64::from(branch) * 14.0,
                12.0,
                0,
                Some(PortSide::East),
            );
        }
        for branch in 0..other_nets {
            add_node(
                blocker + 1 + other_nets + branch,
                240.0,
                340.0 - f64::from(branch) * 14.0,
                12.0,
                2,
                Some(PortSide::West),
            );
        }

        let mut edges = (0..fanout_branches)
            .map(|branch| Edge {
                id: branch,
                source: Endpoint { node: 0, port: 0 },
                target: Endpoint {
                    node: 1 + branch,
                    port: 0,
                },
                net: 1,
                participates_in_ranking: true,
            })
            .collect::<Vec<_>>();
        edges.extend((0..other_nets).map(|branch| Edge {
            id: fanout_branches + branch,
            source: Endpoint {
                node: blocker + 1 + branch,
                port: 0,
            },
            target: Endpoint {
                node: blocker + 1 + other_nets + branch,
                port: 0,
            },
            net: 100 + branch,
            participates_in_ranking: true,
        }));

        (Graph { nodes, edges }, geometry, ranks)
    }

    fn add_crossing_repair_fixture(
        graph: &mut Graph,
        geometry: &mut Vec<NodeGeometry>,
        ranks: &mut Vec<usize>,
    ) {
        const SIDE: u32 = 70;
        let first = graph.nodes.len() as u32;
        graph.nodes.extend((0..SIDE * 2).map(|offset| Node {
            id: first + offset,
            width: 10.0,
            height: 10.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side: if offset < SIDE {
                    PortSide::East
                } else {
                    PortSide::West
                },
                offset: 5.0,
            }],
        }));
        geometry.extend((0..SIDE * 2).map(|offset| NodeGeometry {
            id: first + offset,
            x: if offset < SIDE { -400.0 } else { -200.0 },
            y: f64::from(offset % SIDE) * 30.0,
            width: 10.0,
            height: 10.0,
        }));
        ranks.extend((0..SIDE * 2).map(|offset| usize::from(offset >= SIDE)));
        let first_edge = graph.edges.len() as u32;
        graph.edges.extend((0..SIDE).map(|offset| Edge {
            id: first_edge + offset,
            source: Endpoint {
                node: first + offset,
                port: 0,
            },
            target: Endpoint {
                node: first + SIDE * 2 - 1 - offset,
                port: 0,
            },
            net: 20_000 + offset,
            participates_in_ranking: true,
        }));
    }

    fn add_feedback_fixture(
        graph: &mut Graph,
        geometry: &mut Vec<NodeGeometry>,
        ranks: &mut Vec<usize>,
    ) {
        let first = graph.nodes.len() as u32;
        let node = |id, cycle_breaker, source| Node {
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
                    side: if id == first {
                        PortSide::East
                    } else {
                        PortSide::West
                    },
                    offset: 10.0,
                }]
            },
        };
        graph.nodes.extend([
            node(first, false, false),
            node(first + 1, false, true),
            node(first + 2, false, true),
            node(first + 3, true, false),
            node(first + 4, true, false),
            node(first + 5, true, false),
            node(first + 6, true, false),
        ]);
        for (offset, (x, y)) in [
            (-800.0, 4_500.0),
            (-700.0, 0.0),
            (-700.0, 3_000.0),
            (-800.0, 0.0),
            (-800.0, 9_000.0),
            (-800.0, 1_500.0),
            (-800.0, 7_500.0),
        ]
        .into_iter()
        .enumerate()
        {
            geometry.push(NodeGeometry {
                id: first + offset as u32,
                x,
                y,
                width: 20.0,
                height: 20.0,
            });
        }
        ranks.extend([0, 1, 1, 0, 0, 0, 0]);
        let edge = graph.edges.len() as u32;
        graph.edges.extend([
            Edge {
                id: edge,
                source: Endpoint {
                    node: first,
                    port: 0,
                },
                target: Endpoint {
                    node: first + 1,
                    port: 0,
                },
                net: 30_100,
                participates_in_ranking: true,
            },
            Edge {
                id: edge + 1,
                source: Endpoint {
                    node: first,
                    port: 0,
                },
                target: Endpoint {
                    node: first + 2,
                    port: 0,
                },
                net: 30_101,
                participates_in_ranking: true,
            },
            Edge {
                id: edge + 2,
                source: Endpoint {
                    node: first + 1,
                    port: 1,
                },
                target: Endpoint {
                    node: first + 3,
                    port: 0,
                },
                net: 30_007,
                participates_in_ranking: true,
            },
            Edge {
                id: edge + 3,
                source: Endpoint {
                    node: first + 1,
                    port: 1,
                },
                target: Endpoint {
                    node: first + 4,
                    port: 0,
                },
                net: 30_007,
                participates_in_ranking: true,
            },
            Edge {
                id: edge + 4,
                source: Endpoint {
                    node: first + 2,
                    port: 1,
                },
                target: Endpoint {
                    node: first + 5,
                    port: 0,
                },
                net: 30_008,
                participates_in_ranking: true,
            },
            Edge {
                id: edge + 5,
                source: Endpoint {
                    node: first + 2,
                    port: 1,
                },
                target: Endpoint {
                    node: first + 6,
                    port: 0,
                },
                net: 30_008,
                participates_in_ranking: true,
            },
        ]);
    }

    fn pad_fixture_to_node_count(
        graph: &mut Graph,
        geometry: &mut Vec<NodeGeometry>,
        ranks: &mut Vec<usize>,
        node_count: usize,
    ) {
        while graph.nodes.len() < node_count {
            let id = graph.nodes.len() as u32;
            graph.nodes.push(Node {
                id,
                width: 20.0,
                height: 20.0,
                cycle_breaker: false,
                ports: Vec::new(),
            });
            geometry.push(NodeGeometry {
                id,
                x: 400.0,
                y: f64::from(id % 20) * 30.0,
                width: 20.0,
                height: 20.0,
            });
            ranks.push(3);
        }
    }

    #[test]
    fn fanout_channel_candidate_counts_only_outer_branches_and_is_total() {
        let (graph, _, ranks) = fanout_candidate_fixture(512, 1);
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let outer_nets = BTreeSet::from([1, 100, 999]);

        assert!(
            fanout_outer_channel_lane_indices(&plan, &vec![Some((0, 2)); 513], &outer_nets)
                .is_none()
        );
        let mut mixed_spans = vec![Some((0, 2)); 513];
        mixed_spans[..511].fill(None);
        assert!(
            fanout_outer_channel_lane_indices(&plan, &mixed_spans, &outer_nets).is_none(),
            "electrical fanout does not substitute for the actual outer-branch threshold"
        );
        mixed_spans[511] = None;
        let lanes = fanout_outer_channel_lane_indices(&plan, &mixed_spans, &outer_nets)
            .expect("enough actual outer branches activate the candidate");
        assert_eq!(lanes, BTreeMap::from([(100, 0), (999, 1), (1, 2)]));
    }

    #[test]
    fn production_fanout_candidate_is_exactly_scored_and_deterministic() {
        let (mut graph, mut geometry, mut ranks) = fanout_candidate_fixture(512, 16);
        add_crossing_repair_fixture(&mut graph, &mut geometry, &mut ranks);
        add_feedback_fixture(&mut graph, &mut geometry, &mut ranks);
        pad_fixture_to_node_count(
            &mut graph,
            &mut geometry,
            &mut ranks,
            super::MIN_FANOUT_AWARE_NODES,
        );
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let routed = route_planned_candidates(&plan, &geometry, LayoutOptions::default(), true);

        assert!(routed.fanout_trace.evaluated);
        assert!(routed.fanout_trace.selected);
        assert!(routed.feedback_trace.selected);
        let baseline = routed.fanout_trace.baseline_quality.unwrap();
        let candidate = routed.fanout_trace.candidate_quality.unwrap();
        assert!(candidate.crossings < baseline.crossings);
        assert!(route_quality_cmp(candidate, baseline).is_lt());
        let selected = routed.primary_quality.unwrap();
        assert!(!route_quality_cmp(selected, candidate).is_gt());
        assert_eq!(route_quality(&indexed, &routed.primary), selected);
        let adaptive_repair = routed
            .repair
            .as_ref()
            .expect("fixture must cover the selected adaptive repair path");
        assert_eq!(
            route_quality(&indexed, &adaptive_repair.1),
            adaptive_repair.0
        );
        let stable = route_edges_with_lane_rounds(
            &plan,
            &geometry,
            LayoutOptions::default(),
            super::SUPPLEMENTAL_OUTER_LANE_ROUNDS,
            super::SUPPLEMENTAL_GAP_LANE_ROUNDS,
            true,
            false,
        );
        assert!(stable.feedback_trace.selected);
        assert!(
            stable.repair.is_some(),
            "fixture must cover the repair family"
        );
        let mut stable_family = vec![(stable.primary_quality.unwrap(), stable.primary)];
        if let Some(repair) = stable.repair {
            stable_family.push(repair);
        }
        assert_eq!(routed.alternatives, stable_family);

        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let indexed = validate_and_index(&permuted, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let permuted = route_planned_candidates(&plan, &geometry, LayoutOptions::default(), true);
        assert!(permuted.fanout_trace.selected);
        assert_eq!(permuted.primary_quality, routed.primary_quality);
        assert_eq!(permuted.primary, routed.primary);
        assert_eq!(permuted.repair, routed.repair);
        assert_eq!(permuted.alternatives, routed.alternatives);
    }

    #[test]
    fn production_fanout_candidate_retains_exact_baseline_when_not_better() {
        let (mut graph, mut geometry, mut ranks) = fanout_candidate_fixture(512, 16);
        pad_fixture_to_node_count(
            &mut graph,
            &mut geometry,
            &mut ranks,
            super::MIN_FANOUT_AWARE_NODES,
        );
        for edge in graph.edges.iter_mut().take(512) {
            edge.net = 1_000;
        }
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let routed = route_planned_candidates(&plan, &geometry, LayoutOptions::default(), true);
        let stable = route_edges_with_lane_rounds(
            &plan,
            &geometry,
            LayoutOptions::default(),
            super::SUPPLEMENTAL_OUTER_LANE_ROUNDS,
            super::SUPPLEMENTAL_GAP_LANE_ROUNDS,
            true,
            false,
        );

        assert!(routed.fanout_trace.evaluated);
        assert!(!routed.fanout_trace.selected);
        assert!(
            !route_quality_cmp(
                routed.fanout_trace.candidate_quality.unwrap(),
                routed.fanout_trace.baseline_quality.unwrap(),
            )
            .is_lt()
        );
        assert_eq!(routed.primary, stable.primary);
        assert_eq!(routed.primary_quality, stable.primary_quality);
    }

    #[test]
    fn production_fanout_candidate_preserves_subthreshold_routes() {
        let (graph, geometry, ranks) = fanout_candidate_fixture(15, 16);
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let supplemental =
            route_planned_candidates(&plan, &geometry, LayoutOptions::default(), true);
        let stable = route_edges_with_lane_rounds(
            &plan,
            &geometry,
            LayoutOptions::default(),
            super::SUPPLEMENTAL_OUTER_LANE_ROUNDS,
            super::SUPPLEMENTAL_GAP_LANE_ROUNDS,
            true,
            false,
        );

        assert!(!supplemental.fanout_trace.evaluated);
        assert!(!supplemental.fanout_trace.selected);
        assert_eq!(supplemental.primary, stable.primary);
    }

    #[test]
    fn production_fanout_candidate_preserves_routes_below_node_threshold() {
        let (mut graph, mut geometry, mut ranks) = fanout_candidate_fixture(512, 16);
        pad_fixture_to_node_count(
            &mut graph,
            &mut geometry,
            &mut ranks,
            super::MIN_FANOUT_AWARE_NODES - 1,
        );
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let supplemental =
            route_planned_candidates(&plan, &geometry, LayoutOptions::default(), true);
        let stable = route_edges_with_lane_rounds(
            &plan,
            &geometry,
            LayoutOptions::default(),
            super::SUPPLEMENTAL_OUTER_LANE_ROUNDS,
            super::SUPPLEMENTAL_GAP_LANE_ROUNDS,
            true,
            false,
        );

        assert_eq!(graph.nodes.len(), super::MIN_FANOUT_AWARE_NODES - 1);
        assert!(!supplemental.fanout_trace.evaluated);
        assert_eq!(supplemental.primary, stable.primary);
        assert_eq!(supplemental.primary_quality, stable.primary_quality);
        assert_eq!(supplemental.repair, stable.repair);
    }

    #[test]
    fn supplemental_gap_lane_transpose_can_reach_the_best_lane() {
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

        let legacy = super::crossing_aware_gap_lane_indices_with_rounds(&current, &accesses, 8);
        let lanes = super::crossing_aware_gap_lane_indices_with_rounds(
            &current,
            &accesses,
            super::SUPPLEMENTAL_GAP_LANE_ROUNDS,
        );

        assert_eq!(legacy[&17], 9);
        assert_eq!(lanes[&17], 0);
        assert_eq!(
            super::SUPPLEMENTAL_GAP_LANE_ROUNDS,
            super::FULL_GAP_LANE_ROUNDS
        );
    }

    #[test]
    fn dense_gap_pair_costs_match_btree_reference_under_randomized_permutations() {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let mut state = 0xd3_45e_u64;
        for case in 0..128u32 {
            let count = 1 + (next(&mut state) % 32) as usize;
            let nets = (0..count)
                .map(|index| 10_000 + case * 10_000 + index as u32 * 97)
                .collect::<Vec<_>>();
            let mut lane_order = nets.clone();
            for index in (1..lane_order.len()).rev() {
                let other = (next(&mut state) % (index + 1) as u64) as usize;
                lane_order.swap(index, other);
            }
            let current = lane_order
                .iter()
                .enumerate()
                .map(|(lane, &net)| (net, lane))
                .collect::<BTreeMap<_, _>>();
            let mut accesses = BTreeMap::new();
            for &net in &nets {
                let mut access = GapNetAccess::default();
                for _ in 0..next(&mut state) % 9 {
                    let first = (next(&mut state) % 101) as f64;
                    let second = (next(&mut state) % 101) as f64;
                    access.vertical.push((first.min(second), first.max(second)));
                }
                for _ in 0..next(&mut state) % 9 {
                    access.left_y.push((next(&mut state) % 101) as f64);
                }
                for _ in 0..next(&mut state) % 9 {
                    access.right_y.push((next(&mut state) % 101) as f64);
                }
                access.left_y.sort_by(f64::total_cmp);
                access.right_y.sort_by(f64::total_cmp);
                accesses.insert(net, access);
            }
            let permuted_accesses = accesses
                .iter()
                .rev()
                .map(|(&net, access)| (net, access.clone()))
                .collect::<BTreeMap<_, _>>();

            for rounds in [0, 1, 4, super::FULL_GAP_LANE_ROUNDS] {
                let expected =
                    crossing_aware_gap_lane_indices_btree_reference(&current, &accesses, rounds);
                let actual =
                    super::crossing_aware_gap_lane_indices_with_rounds(&current, &accesses, rounds);
                let permuted = super::crossing_aware_gap_lane_indices_with_rounds(
                    &current,
                    &permuted_accesses,
                    rounds,
                );
                assert_eq!(actual, expected, "case={case} rounds={rounds}");
                assert_eq!(permuted, expected, "case={case} rounds={rounds}");
            }
        }
    }

    fn supplemental_round_route_fixture() -> (Graph, Vec<NodeGeometry>, Vec<usize>) {
        let mut nodes = Vec::new();
        let mut geometry = Vec::new();
        let mut ranks = Vec::new();
        let mut edges = Vec::new();
        for net in 0..18u32 {
            let source_id = net * 2;
            let target_id = source_id + 1;
            let source_y = if net == 17 { 5.0 } else { 0.0 };
            let target_y = if net == 17 { 30.0 } else { 10.0 };
            nodes.extend([
                Node {
                    id: source_id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: target_id,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 10.0,
                    }],
                },
            ]);
            geometry.extend([
                NodeGeometry {
                    id: source_id,
                    x: 0.0,
                    y: source_y - 10.0,
                    width: 20.0,
                    height: 20.0,
                },
                NodeGeometry {
                    id: target_id,
                    x: 100.0,
                    y: target_y - 10.0,
                    width: 20.0,
                    height: 20.0,
                },
            ]);
            ranks.extend([0, 1]);
            edges.push(Edge {
                id: net,
                source: Endpoint {
                    node: source_id,
                    port: 0,
                },
                target: Endpoint {
                    node: target_id,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            });
        }
        (Graph { nodes, edges }, geometry, ranks)
    }

    #[test]
    fn supplemental_round_budgets_improve_emitted_routes_deterministically() {
        let (graph, geometry, ranks) = supplemental_round_route_fixture();
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let plan = RoutingPlan::new(&indexed, &ranks);
        let options = LayoutOptions {
            port_stub: 1e-3,
            ..LayoutOptions::default()
        };
        let legacy = route_edges_with_lane_rounds(&plan, &geometry, options, 4, 8, false, false);
        let current = route_supplemental_edges(&indexed, &geometry, &ranks, options);
        let legacy_quality = route_quality(&indexed, &legacy.primary);
        let current_quality = route_quality(&indexed, &current);

        assert!(route_quality_cmp(current_quality, legacy_quality).is_lt());
        assert!(current_quality.crossings < legacy_quality.crossings);

        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let indexed = validate_and_index(&permuted, LayoutOptions::default()).unwrap();
        let permuted = route_supplemental_edges(&indexed, &geometry, &ranks, options);
        assert_eq!(permuted, current);
        assert_eq!(route_quality(&indexed, &permuted), current_quality);
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
    fn supplemental_outer_lane_transpose_can_reach_the_best_lane() {
        let nets = (0..10).collect::<BTreeSet<_>>();
        let mut accesses = (0..9)
            .map(|net| {
                (
                    net,
                    OuterNetAccess {
                        horizontal: vec![(0.0, 10.0)],
                        vertical_x: vec![20.0],
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();
        accesses.insert(
            9,
            OuterNetAccess {
                horizontal: vec![(20.0, 30.0)],
                vertical_x: vec![5.0],
            },
        );

        let legacy = super::crossing_aware_outer_lane_indices_with_rounds(&nets, &accesses, 4);
        let lanes = super::crossing_aware_outer_lane_indices_with_rounds(
            &nets,
            &accesses,
            super::SUPPLEMENTAL_OUTER_LANE_ROUNDS,
        );

        assert_eq!(legacy[&9], 5);
        assert_eq!(lanes[&9], 0);
        assert_eq!(
            super::SUPPLEMENTAL_OUTER_LANE_ROUNDS,
            super::FULL_OUTER_LANE_ROUNDS
        );
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
        let channels = lane_indices(&BTreeSet::from([7]));
        let lanes = outer_lane_assignments(
            &plan,
            &geometry,
            &[0, 1, 1],
            &[None, None],
            &channels,
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
        let channels = lane_indices(&BTreeSet::from([7]));
        let split_lanes = outer_lane_assignments(
            &plan,
            &geometry,
            &[0, 1, 1],
            &[None, None],
            &channels,
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
            &channels,
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

        take_routing_reuse_counts();
        let routed = route_planned_candidates(&plan, &geometry, LayoutOptions::default(), false);
        let reuse_counts = take_routing_reuse_counts();
        assert_eq!(reuse_counts.coherent_endpoint_tracks, 1);
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
    fn adaptive_gap_tracks_target_the_lane_gap_and_respect_sparse_band_boundaries() {
        let options = LayoutOptions::default();
        let layer_left = [0.0, 86.0];
        let layer_right = [20.0, 106.0];
        let lanes = [BTreeMap::from([(10, 0), (11, 1), (12, 2)])];
        let compact = (10..=12)
            .map(|net| {
                sparse_gap_x(
                    net,
                    0,
                    &layer_left,
                    &layer_right,
                    &lanes,
                    options,
                    GapTrackSpacing::Compact,
                )
            })
            .collect::<Vec<_>>();
        let adaptive = (10..=12)
            .map(|net| {
                sparse_gap_x(
                    net,
                    0,
                    &layer_left,
                    &layer_right,
                    &lanes,
                    options,
                    GapTrackSpacing::Adaptive,
                )
            })
            .collect::<Vec<_>>();

        assert!((adaptive[1] - adaptive[0] - options.route_lane_gap).abs() < 1e-12);
        assert!((adaptive[2] - adaptive[1] - options.route_lane_gap).abs() < 1e-12);
        assert!(adaptive[1] - adaptive[0] > compact[1] - compact[0]);
        assert!(adaptive[0] > layer_right[0] + (layer_left[1] - layer_right[0]) * 0.55);
        assert!(adaptive[2] < layer_left[1] - options.port_stub);
    }

    #[test]
    fn adaptive_gap_tracks_fall_back_deterministically_when_the_gap_is_too_narrow() {
        let options = LayoutOptions::default();
        let layer_left = [0.0, 50.0];
        let layer_right = [20.0, 70.0];
        let lanes = [BTreeMap::from([(10, 0), (11, 1)])];
        for net in [10, 11] {
            let compact = sparse_gap_x(
                net,
                0,
                &layer_left,
                &layer_right,
                &lanes,
                options,
                GapTrackSpacing::Compact,
            );
            let adaptive = sparse_gap_x(
                net,
                0,
                &layer_left,
                &layer_right,
                &lanes,
                options,
                GapTrackSpacing::Adaptive,
            );
            assert_eq!(adaptive, compact);
        }
    }

    #[test]
    fn adaptive_gap_tracks_preserve_endpoint_access_conflicts_when_intervals_move() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 1,
                    width: 20.0,
                    height: 80.0,
                    cycle_breaker: false,
                    ports: vec![
                        Port {
                            id: 0,
                            side: PortSide::East,
                            offset: 10.0,
                        },
                        Port {
                            id: 1,
                            side: PortSide::East,
                            offset: 60.0,
                        },
                    ],
                },
                Node {
                    id: 2,
                    width: 20.0,
                    height: 80.0,
                    cycle_breaker: false,
                    ports: vec![
                        Port {
                            id: 0,
                            side: PortSide::West,
                            offset: 50.0,
                        },
                        Port {
                            id: 1,
                            side: PortSide::West,
                            offset: 10.0,
                        },
                    ],
                },
            ],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 10,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 2, port: 1 },
                    net: 11,
                    participates_in_ranking: true,
                },
            ],
        };
        let options = LayoutOptions::default();
        let indexed = validate_and_index(&graph, options).unwrap();
        let ranks = [0, 1];
        let plan = RoutingPlan::new(&indexed, &ranks);
        let geometry = [
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 20.0,
                height: 80.0,
            },
            NodeGeometry {
                id: 2,
                x: 86.0,
                y: 0.0,
                width: 20.0,
                height: 80.0,
            },
        ];
        let spans = [Some((0, 1)), Some((0, 1))];
        let layer_left = [0.0, 86.0];
        let layer_right = [20.0, 106.0];
        let outer_lanes = BTreeMap::new();
        let build = |gap_lanes: &[BTreeMap<u32, usize>], spacing| {
            build_endpoint_tracks(
                &plan,
                &geometry,
                &ranks,
                &spans,
                &layer_left,
                &layer_right,
                gap_lanes,
                &outer_lanes,
                options,
                spacing,
            )
        };

        let separated_lanes = [BTreeMap::from([(10, 0), (11, 1)])];
        let compact_source_x = sparse_gap_x(
            10,
            0,
            &layer_left,
            &layer_right,
            &separated_lanes,
            options,
            GapTrackSpacing::Compact,
        );
        let adaptive_source_x = sparse_gap_x(
            10,
            0,
            &layer_left,
            &layer_right,
            &separated_lanes,
            options,
            GapTrackSpacing::Adaptive,
        );
        let compact_target_x = sparse_gap_x(
            11,
            0,
            &layer_left,
            &layer_right,
            &separated_lanes,
            options,
            GapTrackSpacing::Compact,
        );
        let adaptive_target_x = sparse_gap_x(
            11,
            0,
            &layer_left,
            &layer_right,
            &separated_lanes,
            options,
            GapTrackSpacing::Adaptive,
        );
        assert_ne!(adaptive_source_x, compact_source_x);
        assert_ne!(adaptive_target_x, compact_target_x);
        assert!(compact_source_x < compact_target_x);
        assert!(adaptive_source_x < adaptive_target_x);
        assert_eq!(
            build(&separated_lanes, GapTrackSpacing::Compact),
            build(&separated_lanes, GapTrackSpacing::Adaptive)
        );
        assert!(build(&separated_lanes, GapTrackSpacing::Compact).is_empty());

        let overlapping_lanes = [BTreeMap::from([(10, 1), (11, 0)])];
        let compact = build(&overlapping_lanes, GapTrackSpacing::Compact);
        let adaptive = build(&overlapping_lanes, GapTrackSpacing::Adaptive);
        assert_eq!(adaptive, compact);
        assert_eq!(
            compact,
            BTreeMap::from([((1, 0, 0), (0, 2)), ((2, 1, 1), (1, 2))])
        );
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
            LayoutOptions::default(),
            GapTrackSpacing::Compact,
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
    fn shared_crossing_sweep_matches_an_independent_pairwise_oracle() {
        let endpoint = |node| Endpoint { node, port: 0 };
        let shared = endpoint(50);
        let segment = |net, source, target, horizontal, fixed, start, end| PhysicalSegment {
            net,
            source: endpoint(source),
            target: endpoint(target),
            horizontal,
            fixed,
            start,
            end,
        };
        let segments = vec![
            segment(1, 1, 2, true, 5.0, 0.0, 10.0),
            segment(2, 50, 4, true, 7.0, 0.0, 10.0),
            segment(3, 5, 6, true, 9.0, 0.0, 10.0),
            segment(4, 7, 8, true, 5.0, 5.0, 15.0),
            segment(10, 9, 10, false, 5.0, 0.0, 10.0),
            segment(1, 11, 12, false, 6.0, 0.0, 10.0),
            segment(20, 50, 13, false, 7.0, 0.0, 10.0),
            segment(30, 14, 15, false, 10.0, 0.0, 10.0),
            segment(31, 16, 17, false, 8.0, 5.0, 9.0),
        ];
        let shared_endpoints = HashSet::from([shared]);
        let oracle = |horizontal: &[&PhysicalSegment], vertical: &[&PhysicalSegment]| {
            let mut counts = BTreeMap::<u32, usize>::new();
            for line in vertical {
                for crossing in horizontal {
                    let shares_endpoint = [line.source, line.target].into_iter().any(|endpoint| {
                        shared_endpoints.contains(&endpoint)
                            && (crossing.source == endpoint || crossing.target == endpoint)
                    });
                    if line.fixed > crossing.start
                        && line.fixed < crossing.end
                        && crossing.fixed > line.start
                        && crossing.fixed < line.end
                        && line.net != crossing.net
                        && !shares_endpoint
                    {
                        *counts.entry(line.net).or_default() += 1;
                    }
                }
            }
            counts
        };

        for transpose in [false, true] {
            let horizontal = segments
                .iter()
                .filter(|segment| segment.horizontal != transpose)
                .collect::<Vec<_>>();
            let vertical = segments
                .iter()
                .filter(|segment| segment.horizontal == transpose)
                .collect::<Vec<_>>();
            let expected = oracle(&horizontal, &vertical);
            let mut actual = BTreeMap::new();
            let crossings =
                physical_crossing_sweep(&shared_endpoints, &segments, transpose, Some(&mut actual));
            assert_eq!(crossings, expected.values().sum::<usize>());
            assert_eq!(actual, expected);
        }

        let horizontal = segments[..4].iter().collect::<Vec<_>>();
        let vertical = &segments[4..];
        let profile_indices: [&[usize]; 3] = [&[0, 1, 2, 3, 4], &[0, 1, 2, 3], &[0, 3]];
        let tagged = profile_indices
            .iter()
            .enumerate()
            .flat_map(|(profile, indices)| {
                indices.iter().map(move |&index| {
                    let line = &vertical[index];
                    (line, ((profile as u64) << 32) | u64::from(line.net))
                })
            })
            .collect::<Vec<_>>();
        let mut actual_profiles = BTreeMap::new();
        physical_crossing_sweep_lines(&shared_endpoints, &horizontal, &tagged, |key, crossings| {
            *actual_profiles.entry(key).or_default() += crossings;
        });
        let mut expected_profiles = BTreeMap::new();
        for (profile, indices) in profile_indices.iter().enumerate() {
            let lines = indices
                .iter()
                .map(|&index| &vertical[index])
                .collect::<Vec<_>>();
            for (net, crossings) in oracle(&horizontal, &lines) {
                expected_profiles.insert(((profile as u64) << 32) | u64::from(net), crossings);
            }
        }
        assert_eq!(actual_profiles, expected_profiles);
    }

    #[test]
    fn flat_physical_segments_match_btree_reference() {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let mut state = 0x5eed_u64;
        let mut edges = Vec::new();
        let mut routes = Vec::new();
        for id in 0..256u32 {
            let x0 = (next(&mut state) % 32) as f64;
            let y0 = (next(&mut state) % 24) as f64;
            let x1 = (next(&mut state) % 32) as f64;
            let y1 = (next(&mut state) % 24) as f64;
            let x2 = (next(&mut state) % 32) as f64;
            edges.push(Edge {
                id,
                source: Endpoint {
                    node: id * 2,
                    port: 0,
                },
                target: Endpoint {
                    node: id * 2 + 1,
                    port: 0,
                },
                net: (next(&mut state) % 23) as u32,
                participates_in_ranking: true,
            });
            routes.push(EdgeGeometry {
                id,
                points: vec![
                    Point { x: x0, y: y0 },
                    Point { x: x1, y: y0 },
                    Point { x: x1, y: y1 },
                    Point { x: x2, y: y1 },
                ],
            });
        }

        let signature = |segments: &[PhysicalSegment]| {
            segments
                .iter()
                .map(|segment| {
                    (
                        segment.net,
                        segment.source,
                        segment.target,
                        segment.horizontal,
                        segment.fixed.to_bits(),
                        segment.start.to_bits(),
                        segment.end.to_bits(),
                    )
                })
                .collect::<Vec<_>>()
        };
        let actual = physical_route_segments(edges.iter(), &routes);
        let expected = physical_route_segments_btree_reference(edges.iter(), &routes);

        assert_eq!(signature(&actual.0), signature(&expected.0));
        assert_eq!(actual.1, expected.1);
        assert_eq!(actual.2.to_bits(), expected.2.to_bits());
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

    #[test]
    #[ignore = "manual release-mode end-to-end benchmark"]
    fn benchmark_dense_gap_end_to_end() {
        use std::{hint::black_box, sync::atomic::Ordering as AtomicOrdering, time::Instant};

        fn fixture(node_count: u32, layers: u32, width: u32) -> Graph {
            let nodes = (0..node_count)
                .map(|id| Node {
                    id,
                    width: 80.0,
                    height: 60.0,
                    cycle_breaker: false,
                    ports: std::iter::once(Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 30.0,
                    })
                    .chain((0..5).map(|branch| Port {
                        id: branch + 1,
                        side: PortSide::West,
                        offset: 10.0 * f64::from(branch + 1),
                    }))
                    .collect(),
                })
                .collect();
            let mut edges = Vec::new();
            for layer in 0..layers - 1 {
                for source in 0..width {
                    for branch in 0..5 {
                        edges.push(Edge {
                            id: edges.len() as u32,
                            source: Endpoint {
                                node: layer * width + source,
                                port: 0,
                            },
                            target: Endpoint {
                                node: (layer + 1) * width
                                    + (source * 7 + branch * 11 + layer * 13) % width,
                                port: branch + 1,
                            },
                            net: layer * width + source,
                            participates_in_ranking: true,
                        });
                    }
                }
            }
            Graph { nodes, edges }
        }

        fn checksum(bytes: &[u8]) -> u64 {
            bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, &byte| {
                (hash ^ u64::from(byte)).wrapping_mul(0x100_0000_01b3)
            })
        }

        for (node_count, layers) in [(600, 18), (1_000, 31), (2_000, 62)] {
            let graph = fixture(node_count, layers, 32);
            for effort in [
                crate::QualityEffort::Fast,
                crate::QualityEffort::Quality,
                crate::QualityEffort::Max,
            ] {
                super::USE_BTREE_GAP_PAIR_COSTS.store(false, AtomicOrdering::Relaxed);
                let expected = crate::layout_with_quality_effort(
                    black_box(&graph),
                    LayoutOptions::default(),
                    effort,
                )
                .unwrap();
                let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
                let quality = route_quality(&indexed, &expected.edges);
                let bytes = serde_json::to_vec(&expected).unwrap();
                let measure = |use_btree| {
                    super::USE_BTREE_GAP_PAIR_COSTS.store(use_btree, AtomicOrdering::Relaxed);
                    let start = Instant::now();
                    let actual = crate::layout_with_quality_effort(
                        black_box(&graph),
                        LayoutOptions::default(),
                        effort,
                    )
                    .unwrap();
                    let elapsed = start.elapsed().as_micros();
                    assert_eq!(actual, expected);
                    elapsed
                };
                let mut btree_samples = Vec::new();
                let mut dense_samples = Vec::new();
                for iteration in 0..5 {
                    if iteration % 2 == 0 {
                        btree_samples.push(measure(true));
                        dense_samples.push(measure(false));
                    } else {
                        dense_samples.push(measure(false));
                        btree_samples.push(measure(true));
                    }
                }
                super::USE_BTREE_GAP_PAIR_COSTS.store(false, AtomicOrdering::Relaxed);
                let mut permuted = graph.clone();
                permuted.nodes.reverse();
                permuted.edges.reverse();
                assert_eq!(
                    crate::layout_with_quality_effort(&permuted, LayoutOptions::default(), effort,)
                        .unwrap(),
                    expected
                );
                btree_samples.sort_unstable();
                dense_samples.sort_unstable();
                let btree_median = btree_samples[btree_samples.len() / 2];
                let dense_median = dense_samples[dense_samples.len() / 2];
                eprintln!(
                    "nodes={} effort={effort:?} btree_median_us={} btree_tail_us={} dense_median_us={} dense_tail_us={} speedup={:.2}x bytes={} checksum={:016x} quality=({},{},{:016x})",
                    graph.nodes.len(),
                    btree_median,
                    btree_samples[btree_samples.len() - 1],
                    dense_median,
                    dense_samples[dense_samples.len() - 1],
                    btree_median as f64 / dense_median as f64,
                    bytes.len(),
                    checksum(&bytes),
                    quality.crossings,
                    quality.bends,
                    quality.route_length.to_bits(),
                );
            }
        }
        super::USE_BTREE_GAP_PAIR_COSTS.store(false, AtomicOrdering::Relaxed);
    }
}
