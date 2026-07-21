use std::collections::{BTreeMap, BTreeSet};

use crate::{
    EdgeGeometry, LayoutOptions, NodeGeometry, Point, Port, PortSide, validation::IndexedGraph,
};

const MAX_SPARSE_NET_EDGES: usize = 32;
const CROSSING_TRACK_NUDGE: f64 = 1e-4;

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
                // Large multi-terminal nets are better represented by the shared outer trunk
                // until sparse routing grows a native tree rather than duplicating point paths.
                && net_edge_counts[&edge.net] <= MAX_SPARSE_NET_EDGES
                && (source_rank + 1..target_rank).all(|rank| !free_by_rank[rank].is_empty()))
            .then_some((source_rank, target_rank))
        })
        .collect();

    let mut gap_nets = vec![BTreeSet::new(); max_rank];
    let mut crossing_pairs = BTreeSet::new();
    let mut outer_nets = BTreeSet::new();
    for (edge, span) in graph.edges.iter().zip(&sparse_spans) {
        if let Some((source_rank, target_rank)) = span {
            for nets in &mut gap_nets[*source_rank..*target_rank] {
                nets.insert(edge.net);
            }
            for rank in source_rank + 1..*target_rank {
                crossing_pairs.insert((rank, edge.net));
            }
        } else {
            outer_nets.insert(edge.net);
        }
    }
    let gap_lanes: Vec<_> = gap_nets.iter().map(lane_indices).collect();
    let crossing_lanes: BTreeMap<_, _> = crossing_pairs
        .iter()
        .copied()
        .enumerate()
        .map(|(index, pair)| (pair, index))
        .collect();
    let crossing_lane_count = crossing_lanes.len();
    let outer_lanes = lane_indices(&outer_nets);
    let outer_lane_count = outer_lanes.len();
    let endpoint_tracks = endpoint_tracks(
        graph,
        nodes,
        ranks,
        &sparse_spans,
        &layer_left,
        &layer_right,
        &gap_lanes,
        &outer_lanes,
        outer_lane_count,
        options,
    );

    graph
        .edges
        .iter()
        .zip(sparse_spans)
        .map(|(edge, sparse_span)| {
            let source_index = graph.node_index[&edge.source.node];
            let target_index = graph.node_index[&edge.target.node];
            let source_node = &nodes[source_index];
            let target_node = &nodes[target_index];
            let source_port = graph.ports[source_index][&edge.source.port];
            let target_port = graph.ports[target_index][&edge.target.port];
            let source = port_point(source_node, source_port);
            let target = port_point(target_node, target_port);
            if let Some((source_rank, target_rank)) = sparse_span {
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
                        &crossing_lanes,
                        crossing_lane_count,
                        &free_by_rank,
                        &endpoint_tracks,
                        options.port_stub,
                    ),
                };
            }

            let lane = outer_lanes[&edge.net];
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
                lane,
                outer_lane_count,
                &layer_left,
                &layer_right,
                options,
            );
            let target_channel = channel_point(
                target_stub,
                target_node,
                target_port.side,
                ranks[target_index],
                lane,
                outer_lane_count,
                &layer_left,
                &layer_right,
                options,
            );
            let lane_y = top - options.port_stub - (lane + 1) as f64 * options.route_lane_gap;
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
fn endpoint_tracks(
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    sparse_spans: &[Option<(usize, usize)>],
    layer_left: &[f64],
    layer_right: &[f64],
    gap_lanes: &[BTreeMap<u32, usize>],
    outer_lanes: &BTreeMap<u32, usize>,
    outer_lane_count: usize,
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
                let lane = outer_lanes[&edge.net];
                (
                    channel_point(
                        source_stub,
                        source_node,
                        source_port.side,
                        ranks[source_index],
                        lane,
                        outer_lane_count,
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
                        lane,
                        outer_lane_count,
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
    crossing_lanes: &BTreeMap<(usize, u32), usize>,
    crossing_lane_count: usize,
    free_by_rank: &[Vec<(f64, f64)>],
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

    for rank in source_rank + 1..target_rank {
        let progress = (rank - source_rank) as f64 / (target_rank - source_rank) as f64;
        let preferred = source.y + (target.y - source.y) * progress;
        let y = choose_crossing_y(
            &free_by_rank[rank],
            preferred,
            crossing_lanes[&(rank, net)],
            crossing_lane_count,
        );
        push_point(&mut points, Point { x, y });
        x = sparse_gap_x(net, rank, layer_left, layer_right, gap_lanes);
        push_point(&mut points, Point { x, y });
    }

    push_point(
        &mut points,
        Point {
            x,
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

fn choose_crossing_y(
    intervals: &[(f64, f64)],
    preferred: f64,
    lane: usize,
    lane_count: usize,
) -> f64 {
    let &(low, high) = intervals
        .iter()
        .min_by(|(left_low, left_high), (right_low, right_high)| {
            interval_distance(preferred, *left_low, *left_high)
                .total_cmp(&interval_distance(preferred, *right_low, *right_high))
                .then(left_low.total_cmp(right_low))
        })
        .expect("sparse routes require a free crossing interval");
    let lane_fraction = (lane + 1) as f64 / (lane_count + 1) as f64;
    let y = low + (high - low) * lane_fraction;
    let margin = (y - low).min(high - y);
    y + (CROSSING_TRACK_NUDGE * lane_fraction).min(margin / 2.0)
}

fn interval_distance(value: f64, low: f64, high: f64) -> f64 {
    if value < low {
        low - value
    } else if value > high {
        value - high
    } else {
        0.0
    }
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
    use super::choose_crossing_y;

    #[test]
    fn crossing_tie_breaker_separates_coincident_tracks() {
        let first = choose_crossing_y(&[(0.0, 4.0)], 1.0, 0, 3);
        let second = choose_crossing_y(&[(0.0, 2.0)], 1.0, 1, 3);

        assert_ne!(first, second);
        assert!(first > 0.0 && first < 4.0);
        assert!(second > 0.0 && second < 2.0);
    }
}
