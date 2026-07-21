use std::collections::BTreeMap;

use crate::{
    EdgeGeometry, LayoutOptions, NodeGeometry, Point, Port, PortSide, validation::IndexedGraph,
};

pub(crate) fn route_edges(
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    ranks: &[usize],
    options: LayoutOptions,
) -> Vec<EdgeGeometry> {
    let top = nodes.iter().map(|node| node.y).fold(0.0, f64::min);
    let max_rank = ranks.iter().copied().max().unwrap_or(0);
    let mut layer_left = vec![f64::INFINITY; max_rank + 1];
    let mut layer_right = vec![f64::NEG_INFINITY; max_rank + 1];
    for (node, &rank) in nodes.iter().zip(ranks) {
        layer_left[rank] = layer_left[rank].min(node.x);
        layer_right[rank] = layer_right[rank].max(node.x + node.width);
    }
    let mut lane_by_net = BTreeMap::new();
    for edge in &graph.edges {
        lane_by_net.insert(edge.net, 0usize);
    }
    for (lane, index) in lane_by_net.values_mut().enumerate() {
        *index = lane;
    }
    let lane_count = lane_by_net.len();

    graph
        .edges
        .iter()
        .map(|edge| {
            let lane = lane_by_net[&edge.net];
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
            let source_channel = channel_point(
                source_stub,
                source_node,
                source_port.side,
                ranks[source_index],
                lane,
                lane_count,
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
                lane_count,
                &layer_left,
                &layer_right,
                options,
            );
            let lane_y = top - options.port_stub - (lane + 1) as f64 * options.route_lane_gap;
            let mut points = Vec::with_capacity(8);
            push_point(&mut points, source);
            push_point(&mut points, source_stub);
            push_point(&mut points, source_channel);
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
            push_point(&mut points, target_channel);
            push_point(&mut points, target_stub);
            push_point(&mut points, target);
            EdgeGeometry {
                id: edge.id,
                points,
            }
        })
        .collect()
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
    let fraction = (edge_index + 1) as f64 / (edge_count + 1) as f64;
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
