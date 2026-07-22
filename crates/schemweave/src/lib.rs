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
    let [forward, reverse] =
        topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps);
    let quality_layers = if reverse.crossings < forward.crossings {
        &reverse.layers
    } else {
        &forward.layers
    };
    let candidates = [
        placement::place_baseline_nodes(&indexed, &ranks, &forward.layers, options),
        placement::place_nodes(&indexed, &ranks, quality_layers, options),
    ];
    let mut best: Option<(routing::RouteQuality, Layout)> = None;
    for mut nodes in candidates {
        let mut edges = routing::route_edges(&indexed, &nodes, &ranks, options);
        let quality = routing::route_quality(&indexed, &edges);
        let candidate = placement::normalize(&mut nodes, &mut edges);
        retain_better_candidate(&mut best, quality, candidate);
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
    use super::{Layout, retain_better_candidate, routing::RouteQuality};

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

        let exact_tie = candidate(1, 3, 20.0, 20.0);
        retain_better_candidate(&mut best, exact_tie.0, exact_tie.1);
        assert_eq!(best.as_ref().unwrap().1, smaller.1);
    }
}
