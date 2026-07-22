//! Deterministic layered placement and orthogonal routing for circuit graphs.

#![forbid(unsafe_code)]

mod placement;
mod routing;
mod topology;
mod validation;

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub use placement::place;

pub type NodeId = u32;
pub type PortId = u32;
pub type EdgeId = u32;
pub type NetId = u32;

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Node {
    pub id: NodeId,
    pub width: f64,
    pub height: f64,
    #[serde(default)]
    pub cycle_breaker: bool,
    pub ports: Vec<Port>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Port {
    pub id: PortId,
    pub side: PortSide,
    /// Distance from the top for east/west ports or from the left for north/south.
    pub offset: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PortSide {
    North,
    East,
    South,
    West,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
pub struct Endpoint {
    pub node: NodeId,
    pub port: PortId,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Edge {
    pub id: EdgeId,
    pub source: Endpoint,
    pub target: Endpoint,
    /// Routes may share trunks when the caller assigns the same net.
    pub net: NetId,
    #[serde(default = "default_true")]
    pub participates_in_ranking: bool,
}

const fn default_true() -> bool {
    true
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct LayoutOptions {
    pub layer_gap: f64,
    pub node_gap: f64,
    pub port_stub: f64,
    pub route_lane_gap: f64,
    pub ordering_sweeps: usize,
}

impl Default for LayoutOptions {
    fn default() -> Self {
        Self {
            layer_gap: 66.0,
            node_gap: 30.0,
            port_stub: 10.0,
            route_lane_gap: 4.0,
            ordering_sweeps: 4,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct NodeGeometry {
    pub id: NodeId,
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct EdgeGeometry {
    pub id: EdgeId,
    pub points: Vec<Point>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Layout {
    pub nodes: Vec<NodeGeometry>,
    pub edges: Vec<EdgeGeometry>,
    pub width: f64,
    pub height: f64,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum LayoutError {
    #[error("duplicate node id {0}")]
    DuplicateNode(NodeId),
    #[error("node {node} has duplicate port id {port}")]
    DuplicatePort { node: NodeId, port: PortId },
    #[error("duplicate edge id {0}")]
    DuplicateEdge(EdgeId),
    #[error("node {node} has invalid {field} {value}")]
    InvalidNodeDimension {
        node: NodeId,
        field: &'static str,
        value: f64,
    },
    #[error("node {node} port {port} has invalid offset {offset}")]
    InvalidPortOffset {
        node: NodeId,
        port: PortId,
        offset: f64,
    },
    #[error("edge {edge} references unknown {role} node {node}")]
    UnknownEndpointNode {
        edge: EdgeId,
        role: &'static str,
        node: NodeId,
    },
    #[error("edge {edge} references unknown {role} port {node}:{port}")]
    UnknownEndpointPort {
        edge: EdgeId,
        role: &'static str,
        node: NodeId,
        port: PortId,
    },
    #[error("invalid layout option {field}={value}")]
    InvalidOption { field: &'static str, value: f64 },
    #[error("ordering_sweeps must be at most 16, got {0}")]
    TooManyOrderingSweeps(usize),
}

/// Lay out a graph. Output ordering depends only on stable identifiers, not input order.
pub fn layout(graph: &Graph, options: LayoutOptions) -> Result<Layout, LayoutError> {
    let indexed = validation::validate_and_index(graph, options)?;
    let ranks = topology::assign_ranks(&indexed);
    let (forward, reverse, net_representative) =
        topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
    let quality_layers = if reverse.crossings < forward.crossings {
        &reverse.layers
    } else {
        &forward.layers
    };
    let mut best: Option<(routing::RouteQuality, Layout)> = None;
    let mut evaluate = |mut nodes: Vec<NodeGeometry>| {
        let mut edges = routing::route_edges(&indexed, &nodes, &ranks, options);
        let quality = routing::route_quality(&indexed, &edges);
        let candidate = placement::normalize(&mut nodes, &mut edges);
        retain_better_candidate(&mut best, quality, candidate);
    };
    evaluate(placement::place_baseline_nodes(
        &indexed,
        &ranks,
        &forward.layers,
        options,
    ));
    evaluate(placement::place_nodes(
        &indexed,
        &ranks,
        quality_layers,
        options,
    ));
    if let Some(net_representative) = net_representative
        && net_representative.layers != *quality_layers
    {
        evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            &net_representative.layers,
            options,
        ));
    }
    Ok(best.expect("layout has deterministic candidates").1)
}

fn retain_better_candidate(
    best: &mut Option<(routing::RouteQuality, Layout)>,
    quality: routing::RouteQuality,
    candidate: Layout,
) {
    let replace = best.as_ref().is_none_or(|(current_quality, current)| {
        candidate_quality_cmp(quality, &candidate, *current_quality, current).is_lt()
    });
    if replace {
        *best = Some((quality, candidate));
    }
}

fn candidate_quality_cmp(
    left: routing::RouteQuality,
    left_layout: &Layout,
    right: routing::RouteQuality,
    right_layout: &Layout,
) -> std::cmp::Ordering {
    left.crossings
        .cmp(&right.crossings)
        .then(left.bends.cmp(&right.bends))
        .then(left.route_length.total_cmp(&right.route_length))
        .then(
            (left_layout.width * left_layout.height)
                .total_cmp(&(right_layout.width * right_layout.height)),
        )
}

#[cfg(test)]
mod tests {
    use super::{
        Edge, Endpoint, Graph, Layout, LayoutOptions, Node, NodeGeometry, Port, PortSide,
        candidate_quality_cmp, layout, placement, retain_better_candidate, routing,
        routing::RouteQuality, topology, validation,
    };

    fn candidate(
        crossings: usize,
        bends: usize,
        route_length: f64,
        area: f64,
    ) -> (RouteQuality, Layout) {
        (
            RouteQuality {
                crossings,
                bends,
                route_length,
            },
            Layout {
                nodes: Vec::new(),
                edges: Vec::new(),
                width: area,
                height: 1.0,
            },
        )
    }

    fn net_representative_graph() -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..40)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 25.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 25.0,
                    },
                ],
            })
            .collect();
        // One 16-branch net activates the guarded path; fixed sparse cross-connections
        // make its retained ordering distinct without storing a large literal fixture.
        let mut state = 3u64;
        let mut endpoints = Vec::new();
        for target in 8..24 {
            endpoints.push((0, target, 100));
        }
        for source in 1..8 {
            for target in 8..24 {
                if next(&mut state) % 100 < 24 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        for source in 8..24 {
            for target in 24..40 {
                if next(&mut state) % 100 < 20 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, net))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    #[test]
    fn candidate_selection_preserves_baseline_and_uses_quality_priority() {
        let baseline = candidate(1, 4, 30.0, 40.0);
        let mut best = None;
        retain_better_candidate(&mut best, baseline.0, baseline.1.clone());

        let worse_crossings = candidate(2, 0, 1.0, 1.0);
        retain_better_candidate(&mut best, worse_crossings.0, worse_crossings.1);
        assert_eq!(best.as_ref().unwrap().1, baseline.1);

        let fewer_bends = candidate(1, 3, 100.0, 100.0);
        retain_better_candidate(&mut best, fewer_bends.0, fewer_bends.1.clone());
        assert_eq!(best.as_ref().unwrap().1, fewer_bends.1);

        let shorter = candidate(1, 3, 20.0, 100.0);
        retain_better_candidate(&mut best, shorter.0, shorter.1.clone());
        assert_eq!(best.as_ref().unwrap().1, shorter.1);

        let smaller = candidate(1, 3, 20.0, 20.0);
        retain_better_candidate(&mut best, smaller.0, smaller.1.clone());
        assert_eq!(best.as_ref().unwrap().1, smaller.1);

        let mut exact_tie = candidate(1, 3, 20.0, 20.0);
        exact_tie.1.width = 10.0;
        exact_tie.1.height = 2.0;
        assert_ne!(exact_tie.1, smaller.1);
        retain_better_candidate(&mut best, exact_tie.0, exact_tie.1);
        assert_eq!(best.as_ref().unwrap().1, smaller.1);
    }

    #[test]
    fn selects_a_distinct_net_representative_candidate_deterministically() {
        let graph = net_representative_graph();
        let options = LayoutOptions::default();
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        assert!(
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, false)
                .2
                .is_none()
        );
        let (forward, reverse, net_representative) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
        let quality_layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let net_representative = net_representative.unwrap();
        assert_ne!(net_representative.layers, *quality_layers);

        let evaluate = |mut nodes: Vec<NodeGeometry>| {
            let mut edges = routing::route_edges(&indexed, &nodes, &ranks, options);
            let quality = routing::route_quality(&indexed, &edges);
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let baseline = evaluate(placement::place_baseline_nodes(
            &indexed,
            &ranks,
            &forward.layers,
            options,
        ));
        let ordinary = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            quality_layers,
            options,
        ));
        let net_representative = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            &net_representative.layers,
            options,
        ));
        let best_ordinary =
            if candidate_quality_cmp(ordinary.0, &ordinary.1, baseline.0, &baseline.1).is_lt() {
                &ordinary
            } else {
                &baseline
            };
        assert!(
            candidate_quality_cmp(
                net_representative.0,
                &net_representative.1,
                best_ordinary.0,
                &best_ordinary.1,
            )
            .is_lt()
        );

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, net_representative.1);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }
}
