use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::{
    EdgeGeometry, LayoutOptions, NodeGeometry, Point, Port, PortSide, validation::IndexedGraph,
};

const MAX_SPARSE_NET_EDGES: usize = 300;
const CROSSING_TRACK_NUDGE: f64 = 1e-4;
const CROSSING_ALIGNMENT_WEIGHT: f64 = 4.0;
const MIN_ROUTE_SEGMENT: f64 = 1e-7;

pub(crate) fn route_edges(
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    options: LayoutOptions,
) -> Vec<EdgeGeometry> {
    let top = nodes.iter().map(|node| node.y).fold(0.0, f64::min);
    let bottom = nodes
        .iter()
        .map(|node| node.y + node.height)
        .fold(0.0, f64::max);
    let max_rank = ranks.iter().copied().max().unwrap_or(0);
    let mut layer_left = vec![f64::INFINITY; max_rank + 1];
    let mut layer_right = vec![f64::NEG_INFINITY; max_rank + 1];
    for (node, &rank) in nodes.iter().zip(ranks) {
        layer_left[rank] = layer_left[rank].min(node.x);
        layer_right[rank] = layer_right[rank].max(node.x + node.width);
    }

    let mut nodes_by_rank = vec![Vec::new(); max_rank + 1];
    for (node, &rank) in nodes.iter().zip(ranks) {
        nodes_by_rank[rank].push(node);
    }
    let free_by_rank: Vec<_> = nodes_by_rank
        .iter_mut()
        .map(|layer| {
            layer.sort_by(|left, right| left.y.total_cmp(&right.y).then(left.id.cmp(&right.id)));
            free_intervals(layer, top, bottom)
        })
        .collect();

    let mut net_edge_counts = BTreeMap::new();
    for edge in &graph.edges {
        *net_edge_counts.entry(edge.net).or_insert(0usize) += 1;
    }

    let sparse_spans: Vec<_> = graph
        .edges
        .iter()
        .map(|edge| {
            let source_index = graph.node_index[&edge.source.node];
            let target_index = graph.node_index[&edge.target.node];
            let source_port = graph.ports[source_index][&edge.source.port];
            let target_port = graph.ports[target_index][&edge.target.port];
            let source_rank = ranks[source_index];
            let target_rank = ranks[target_index];
            (source_port.side == PortSide::East
                && target_port.side == PortSide::West
                && source_rank < target_rank
                // Extremely large nets are cheaper as one outer trunk; their sparse tree does
                // not improve quality enough to pay for per-layer corridor construction.
                && net_edge_counts[&edge.net] <= MAX_SPARSE_NET_EDGES
                && (source_rank + 1..target_rank).all(|rank| !free_by_rank[rank].is_empty()))
            .then_some((source_rank, target_rank))
        })
        .collect();

    let mut gap_preferences = vec![BTreeMap::<u32, Vec<f64>>::new(); max_rank];
    let mut crossing_preferences = vec![BTreeMap::<u32, Vec<f64>>::new(); max_rank + 1];
    let mut crossing_pairs = BTreeSet::new();
    let mut outer_nets = BTreeSet::new();
    for (edge, span) in graph.edges.iter().zip(&sparse_spans) {
        if let Some((source_rank, target_rank)) = span {
            let source_index = graph.node_index[&edge.source.node];
            let target_index = graph.node_index[&edge.target.node];
            let source = port_point(
                &nodes[source_index],
                graph.ports[source_index][&edge.source.port],
            );
            let target = port_point(
                &nodes[target_index],
                graph.ports[target_index][&edge.target.port],
            );
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
    let mut gap_lanes: Vec<_> = gap_preferences
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
    let outer_lanes = outer_lane_assignments(
        graph,
        nodes,
        ranks,
        &sparse_spans,
        &outer_nets,
        &layer_left,
        &layer_right,
        top,
        bottom,
        options,
    );
    let mut endpoint_tracks = build_endpoint_tracks(
        graph,
        nodes,
        ranks,
        &sparse_spans,
        &layer_left,
        &layer_right,
        &gap_lanes,
        &outer_lanes,
        options,
    );
    let crossing_paths = sparse_crossing_paths(
        graph,
        nodes,
        &sparse_spans,
        &net_edge_counts,
        &crossing_lanes,
        &crossing_tie_lanes,
        crossing_tie_lane_count,
        &free_by_rank,
        &endpoint_tracks,
        options.port_stub,
    );
    gap_lanes = crossing_aware_gap_lanes(
        graph,
        nodes,
        &sparse_spans,
        &crossing_paths,
        &gap_lanes,
        &endpoint_tracks,
        options.port_stub,
    );
    endpoint_tracks = build_endpoint_tracks(
        graph,
        nodes,
        ranks,
        &sparse_spans,
        &layer_left,
        &layer_right,
        &gap_lanes,
        &outer_lanes,
        options,
    );
    graph
        .edges
        .iter()
        .zip(sparse_spans)
        .zip(crossing_paths)
        .map(|((edge, sparse_span), crossing_path)| {
            let source_index = graph.node_index[&edge.source.node];
            let target_index = graph.node_index[&edge.target.node];
            let source_node = &nodes[source_index];
            let target_node = &nodes[target_index];
            let source_port = graph.ports[source_index][&edge.source.port];
            let target_port = graph.ports[target_index][&edge.target.port];
            let source = port_point(source_node, source_port);
            let target = port_point(target_node, target_port);
            if let (Some((source_rank, target_rank)), Some(crossing_path)) =
                (sparse_span, crossing_path)
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
                        &layer_left,
                        &layer_right,
                        &gap_lanes,
                        &crossing_path,
                        &endpoint_tracks,
                        options.port_stub,
                    ),
                };
            }

            let lane = outer_lanes[&edge.id];
            let source_stub = stub_point(source, source_port.side, options.port_stub);
            let target_stub = stub_point(target, target_port.side, options.port_stub);
            let source_escape_y = if matches!(source_port.side, PortSide::East | PortSide::West) {
                endpoint_escape_y(source, edge.source, 0, &endpoint_tracks, options.port_stub)
            } else {
                source_stub.y
            };
            let target_escape_y = if matches!(target_port.side, PortSide::East | PortSide::West) {
                endpoint_escape_y(target, edge.target, 1, &endpoint_tracks, options.port_stub)
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
                &layer_left,
                &layer_right,
                options,
            );
            let target_channel = channel_point(
                target_stub,
                target_node,
                target_port.side,
                ranks[target_index],
                lane.channel_index,
                lane.channel_count,
                &layer_left,
                &layer_right,
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

fn lane_indices(nets: &BTreeSet<u32>) -> BTreeMap<u32, usize> {
    nets.iter()
        .copied()
        .enumerate()
        .map(|(index, net)| (net, index))
        .collect()
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum OuterSide {
    Top,
    Bottom,
}

#[derive(Clone, Copy, Debug)]
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
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    outer_nets: &BTreeSet<u32>,
    layer_left: &[f64],
    layer_right: &[f64],
    top: f64,
    bottom: f64,
    options: LayoutOptions,
) -> BTreeMap<u32, OuterLane> {
    let mut top_nets = BTreeSet::new();
    let mut bottom_nets = BTreeSet::new();
    let mut edge_sides = BTreeMap::new();
    for (edge, span) in graph.edges.iter().zip(sparse_spans) {
        if span.is_some() {
            continue;
        }
        let mut cost = (0.0, 0.0);
        for endpoint in [edge.source, edge.target] {
            let node_index = graph.node_index[&endpoint.node];
            let point = port_point(&nodes[node_index], graph.ports[node_index][&endpoint.port]);
            cost.0 += point.y - top;
            cost.1 += bottom - point.y;
        }
        let side = if cost.1 < cost.0 {
            bottom_nets.insert(edge.net);
            OuterSide::Bottom
        } else {
            top_nets.insert(edge.net);
            OuterSide::Top
        };
        edge_sides.insert(edge.id, side);
    }

    let mut assignments = BTreeMap::new();
    let channel_lanes = lane_indices(outer_nets);
    let channel_count = channel_lanes.len();
    let mut top_access = BTreeMap::<u32, OuterNetAccess>::new();
    let mut bottom_access = BTreeMap::<u32, OuterNetAccess>::new();
    for (edge, span) in graph.edges.iter().zip(sparse_spans) {
        if span.is_some() {
            continue;
        }
        let source_index = graph.node_index[&edge.source.node];
        let target_index = graph.node_index[&edge.target.node];
        let source_node = &nodes[source_index];
        let target_node = &nodes[target_index];
        let source_port = graph.ports[source_index][&edge.source.port];
        let target_port = graph.ports[target_index][&edge.target.port];
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
    let top_lanes = crossing_aware_outer_lane_indices(&top_nets, &top_access);
    let bottom_lanes = crossing_aware_outer_lane_indices(&bottom_nets, &bottom_access);
    for (edge, span) in graph.edges.iter().zip(sparse_spans) {
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

fn crossing_aware_outer_lane_indices(
    nets: &BTreeSet<u32>,
    accesses: &BTreeMap<u32, OuterNetAccess>,
) -> BTreeMap<u32, usize> {
    let mut ordered: Vec<_> = nets.iter().copied().collect();
    let mut costs = BTreeMap::new();
    for _ in 0..16 {
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
    graph: &IndexedGraph<'_>,
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
    for (edge, sparse_span) in graph.edges.iter().zip(sparse_spans) {
        let source_index = graph.node_index[&edge.source.node];
        let target_index = graph.node_index[&edge.target.node];
        let source_node = &nodes[source_index];
        let target_node = &nodes[target_index];
        let source_port = graph.ports[source_index][&edge.source.port];
        let target_port = graph.ports[target_index][&edge.target.port];
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
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    net_edge_counts: &BTreeMap<u32, usize>,
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
    for (edge_index, edge) in graph.edges.iter().enumerate() {
        if net_edge_counts[&edge.net] > 1 {
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
        let first_edge = graph.edges[edge_indices[0]];
        if edge_indices
            .iter()
            .any(|&edge_index| graph.edges[edge_index].source != first_edge.source)
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
        let source_index = graph.node_index[&first_edge.source.node];
        let source = port_point(
            &nodes[source_index],
            graph.ports[source_index][&first_edge.source.port],
        );
        let source_y = endpoint_escape_y(source, first_edge.source, 0, endpoint_tracks, port_stub);
        let mut target_ys = edge_indices
            .iter()
            .map(|&edge_index| {
                let edge = graph.edges[edge_index];
                let target_index = graph.node_index[&edge.target.node];
                let target = port_point(
                    &nodes[target_index],
                    graph.ports[target_index][&edge.target.port],
                );
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

    graph
        .edges
        .iter()
        .zip(sparse_spans)
        .map(|(edge, span)| {
            let &(source_rank, target_rank) = span.as_ref()?;
            if let Some(&(shared_source_rank, ref shared_path)) = shared_paths.get(&edge.net) {
                debug_assert_eq!(shared_source_rank, source_rank);
                return Some(shared_path[..target_rank - source_rank - 1].to_vec());
            }
            let source_index = graph.node_index[&edge.source.node];
            let target_index = graph.node_index[&edge.target.node];
            let source = port_point(
                &nodes[source_index],
                graph.ports[source_index][&edge.source.port],
            );
            let target = port_point(
                &nodes[target_index],
                graph.ports[target_index][&edge.target.port],
            );
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
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    sparse_spans: &[Option<(usize, usize)>],
    crossing_paths: &[Option<Vec<f64>>],
    current_lanes: &[BTreeMap<u32, usize>],
    endpoint_tracks: &BTreeMap<(u32, u32, u8), (usize, usize)>,
    port_stub: f64,
) -> Vec<BTreeMap<u32, usize>> {
    let mut accesses = (0..current_lanes.len())
        .map(|_| BTreeMap::<u32, GapNetAccess>::new())
        .collect::<Vec<_>>();
    for ((edge, span), path) in graph.edges.iter().zip(sparse_spans).zip(crossing_paths) {
        let (Some(&(source_rank, target_rank)), Some(path)) = (span.as_ref(), path) else {
            continue;
        };
        let source_index = graph.node_index[&edge.source.node];
        let target_index = graph.node_index[&edge.target.node];
        let source = port_point(
            &nodes[source_index],
            graph.ports[source_index][&edge.source.port],
        );
        let target = port_point(
            &nodes[target_index],
            graph.ports[target_index][&edge.target.port],
        );
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
        .map(|(lanes, access)| crossing_aware_gap_lane_indices(lanes, access))
        .collect()
}

fn crossing_aware_gap_lane_indices(
    current: &BTreeMap<u32, usize>,
    accesses: &BTreeMap<u32, GapNetAccess>,
) -> BTreeMap<u32, usize> {
    let mut ordered: Vec<_> = current.iter().map(|(&net, &lane)| (lane, net)).collect();
    ordered.sort_unstable();
    let mut ordered: Vec<_> = ordered.into_iter().map(|(_, net)| net).collect();
    let mut costs = BTreeMap::new();
    for _ in 0..16 {
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
        Edge, Endpoint, Graph, LayoutOptions, Node, NodeGeometry, Point, Port, PortSide,
        validation::validate_and_index,
    };

    use super::{
        GapNetAccess, OuterNetAccess, OuterSide, crossing_aware_gap_lane_indices,
        crossing_aware_outer_lane_indices, crossing_track_y, distance_transform,
        outer_lane_assignments, shortest_crossing_path, sparse_channel_route,
    };

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

        let lanes = outer_lane_assignments(
            &indexed,
            &geometry,
            &[0, 1, 1],
            &[None, None],
            &BTreeSet::from([7]),
            &[0.0, 100.0],
            &[20.0, 120.0],
            0.0,
            120.0,
            LayoutOptions::default(),
        );

        assert_eq!(lanes[&10].side, OuterSide::Top);
        assert_eq!(lanes[&11].side, OuterSide::Bottom);
        assert_eq!(lanes[&10].channel_index, lanes[&11].channel_index);
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
}
