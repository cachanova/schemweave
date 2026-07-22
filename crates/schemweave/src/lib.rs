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

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum QualityEffort {
    Fast,
    #[default]
    Quality,
    Max,
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
    layout_with_quality_effort(graph, options, QualityEffort::Quality)
}

/// Lay out a graph with an explicit quality-versus-latency policy.
pub fn layout_with_quality_effort(
    graph: &Graph,
    options: LayoutOptions,
    quality_effort: QualityEffort,
) -> Result<Layout, LayoutError> {
    let indexed = validation::validate_and_index(graph, options)?;
    let (ranks, latest_ranks) = topology::rank_candidates(&indexed);
    let (forward, reverse, net_representative) =
        topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
    let quality_layers = if reverse.crossings < forward.crossings {
        &reverse.layers
    } else {
        &forward.layers
    };
    let baseline_order_crossings = forward.crossings.min(reverse.crossings);
    let routing_plan = routing::RoutingPlan::new(&indexed, &ranks);
    let mut best: Option<(routing::RouteQuality, Layout)> = None;
    let mut retain = |mut nodes: Vec<NodeGeometry>,
                      mut edges: Vec<EdgeGeometry>,
                      quality: Option<routing::RouteQuality>| {
        let quality = quality.unwrap_or_else(|| routing::route_quality(&indexed, &edges));
        let candidate = placement::normalize(&mut nodes, &mut edges);
        retain_better_candidate(&mut best, quality, candidate);
    };
    let mut evaluate = |nodes: Vec<NodeGeometry>,
                        supplemental: bool,
                        sparse_global: bool,
                        large_sparse_global: bool| {
        let routed = routing::route_planned_candidates_with_sparse_global(
            &routing_plan,
            &nodes,
            options,
            supplemental,
            sparse_global,
            large_sparse_global,
        );
        retain(nodes.clone(), routed.primary, routed.primary_quality);
        if let Some((repair_quality, repair)) = routed.repair {
            retain(nodes.clone(), repair, Some(repair_quality));
        }
        for (quality, alternative) in routed.alternatives {
            retain(nodes.clone(), alternative, Some(quality));
        }
    };
    evaluate(
        placement::place_baseline_nodes(&indexed, &ranks, &forward.layers, options),
        false,
        false,
        false,
    );
    let ordinary_nodes = placement::place_nodes(&indexed, &ranks, quality_layers, options);
    let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary_nodes);
    evaluate(ordinary_nodes, false, true, false);
    if placement::preferred_alignment_can_be_significant(ordinary_alignment) {
        let preferred_nodes =
            placement::place_preferred_nodes(&indexed, &ranks, quality_layers, options);
        let preferred_alignment =
            placement::port_alignment_error(&indexed, &ranks, &preferred_nodes);
        if placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment) {
            evaluate(preferred_nodes, true, false, false);
        }
    }
    if let Some(net_representative) = net_representative
        && net_representative.layers != *quality_layers
    {
        let (sparse_global, large_sparse_global) =
            net_representative_sparse_global_flags(graph.nodes.len(), quality_effort);
        if quality_effort == QualityEffort::Max && large_sparse_global {
            let nodes =
                placement::place_nodes(&indexed, &ranks, &net_representative.layers, options);
            let routed = routing::route_planned_candidates_with_refined_sparse_global(
                &routing_plan,
                &nodes,
                options,
                false,
                sparse_global,
                large_sparse_global,
                true,
            );
            retain(nodes.clone(), routed.primary, routed.primary_quality);
            if let Some((repair_quality, repair)) = routed.repair {
                retain(nodes.clone(), repair, Some(repair_quality));
            }
            for (quality, alternative) in routed.alternatives {
                retain(nodes.clone(), alternative, Some(quality));
            }
        } else {
            evaluate(
                placement::place_nodes(&indexed, &ranks, &net_representative.layers, options),
                false,
                sparse_global,
                large_sparse_global,
            );
        }
    }
    if let Some(alternative_ranks) = latest_ranks {
        let (forward, reverse, _) = topology::order_layer_candidates(
            &indexed,
            &alternative_ranks,
            options.ordering_sweeps.min(3),
            false,
        );
        let layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let alternative_crossings = forward.crossings.min(reverse.crossings);
        if alternative_crossings < baseline_order_crossings
            && baseline_order_crossings - alternative_crossings
                >= baseline_order_crossings.div_ceil(100)
        {
            let mut nodes = placement::place_nodes(&indexed, &alternative_ranks, layers, options);
            let alternative_plan = routing::RoutingPlan::new(&indexed, &alternative_ranks);
            let routed =
                routing::route_planned_candidates(&alternative_plan, &nodes, options, false);
            let mut edges = routed.primary;
            let quality = routed
                .primary_quality
                .expect("planned candidates include exact primary quality");
            let candidate = placement::normalize(&mut nodes, &mut edges);
            retain_better_candidate(&mut best, quality, candidate);
        }
    }
    Ok(best.expect("layout has deterministic candidates").1)
}

fn net_representative_sparse_global_flags(
    node_count: usize,
    quality_effort: QualityEffort,
) -> (bool, bool) {
    match quality_effort {
        QualityEffort::Fast => (false, false),
        QualityEffort::Quality | QualityEffort::Max => {
            let admitted = (600..=1_000).contains(&node_count);
            (admitted, admitted)
        }
    }
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
        QualityEffort, candidate_quality_cmp, layout, placement, retain_better_candidate, routing,
        routing::RouteQuality, topology, validation,
    };

    mod active_fanout_fixture {
        use crate as schemweave;

        include!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/support/active_fanout.rs"
        ));
    }

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
    fn net_representative_sparse_global_is_admitted_only_for_medium_large_graphs() {
        assert_eq!(
            super::net_representative_sparse_global_flags(599, QualityEffort::Quality),
            (false, false)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(600, QualityEffort::Quality),
            (true, true)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(1_000, QualityEffort::Quality),
            (true, true)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(1_001, QualityEffort::Quality),
            (false, false)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(855, QualityEffort::Fast),
            (false, false)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(855, QualityEffort::Max),
            (true, true)
        );
    }

    fn sparse_global_layered_graph(
        layer_count: u32,
        source_stride: u32,
        branch_stride: u32,
    ) -> Graph {
        let mut nodes = Vec::new();
        for id in 0..layer_count * 20 {
            nodes.push(Node {
                id,
                width: 76.0,
                height: 84.0,
                cycle_breaker: false,
                ports: std::iter::once(Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 42.0,
                })
                .chain((0..5).map(|branch| Port {
                    id: branch + 1,
                    side: PortSide::West,
                    offset: 14.0 * (branch + 1) as f64,
                }))
                .collect(),
            });
        }
        let mut edges = Vec::new();
        for layer in 0..layer_count - 1 {
            for source in 0..20u32 {
                for branch in 0..5u32 {
                    edges.push(Edge {
                        id: edges.len() as u32,
                        source: Endpoint {
                            node: layer * 20 + source,
                            port: 0,
                        },
                        target: Endpoint {
                            node: (layer + 1) * 20
                                + (source * source_stride + branch * branch_stride) % 20,
                            port: branch + 1,
                        },
                        net: layer * 20 + source,
                        participates_in_ranking: true,
                    });
                }
            }
        }
        Graph { nodes, edges }
    }

    fn sparse_global_ordinary_routes(
        graph: &Graph,
        options: LayoutOptions,
    ) -> (Vec<NodeGeometry>, routing::RoutedEdges) {
        let indexed = validation::validate_and_index(graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let (forward, reverse, _) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, false);
        let layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let ordinary = placement::place_nodes(&indexed, &ranks, layers, options);
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let routed = routing::route_planned_candidates_with_sparse_global(
            &plan, &ordinary, options, false, true, false,
        );
        (ordinary, routed)
    }

    #[test]
    fn public_layout_selects_an_active_sparse_global_candidate() {
        let options = LayoutOptions::default();
        let graph = sparse_global_layered_graph(4, 7, 11);
        let (ordinary, routed) = sparse_global_ordinary_routes(&graph, options);
        assert_eq!(routed.alternatives.len(), 1);
        let (candidate_quality, candidate_edges) = &routed.alternatives[0];
        assert!(candidate_quality.crossings < routed.primary_quality.unwrap().crossings);
        let mut candidate_nodes = ordinary;
        let mut candidate_edges = candidate_edges.clone();
        let candidate_layout = placement::normalize(&mut candidate_nodes, &mut candidate_edges);

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, candidate_layout);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn public_layout_rejects_a_proxy_better_but_exact_worse_sparse_global_candidate() {
        let options = LayoutOptions::default();
        let graph = sparse_global_layered_graph(4, 6, 17);
        let (ordinary, routed) = sparse_global_ordinary_routes(&graph, options);
        assert_eq!(routed.alternatives.len(), 1);
        let (candidate_quality, candidate_edges) = &routed.alternatives[0];
        let primary_quality = routed.primary_quality.unwrap();
        assert!(candidate_quality.crossings > primary_quality.crossings);
        let mut primary_nodes = ordinary.clone();
        let mut primary_edges = routed.primary;
        let primary_layout = placement::normalize(&mut primary_nodes, &mut primary_edges);
        let mut candidate_nodes = ordinary;
        let mut candidate_edges = candidate_edges.clone();
        let candidate_layout = placement::normalize(&mut candidate_nodes, &mut candidate_edges);

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, primary_layout);
        assert_ne!(selected, candidate_layout);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    fn net_representative_graph() -> Graph {
        net_representative_graph_with_padding(40, 3)
    }

    fn net_representative_graph_with_padding(node_count: u32, seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..node_count)
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
        let mut state = seed;
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

    fn reverse_median_graph(seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..140)
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
        let mut state = seed;
        let mut endpoints = Vec::new();
        for target in 10..75 {
            endpoints.push((0, target, 100));
        }
        for target in 15..80 {
            endpoints.push((1, target, 101));
        }
        for source in 2..10 {
            for target in 10..80 {
                if next(&mut state) % 100 < 24 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        for source in 10..80 {
            for target in 80..140 {
                if next(&mut state) % 100 < 12 {
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

    fn slack_rank_graph(seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..51)
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
        let mut endpoints = vec![(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)];
        endpoints.extend((30..51).map(|target| (4, target)));
        let mut state = seed;
        for source in 6..30 {
            for target in 30..51 {
                if next(&mut state) % 100 < 20 {
                    endpoints.push((source, target));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net: id as u32,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    fn preferred_backbone_graph(seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..81)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 4_000.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 0.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 2_000.0,
                    },
                    Port {
                        id: 2,
                        side: PortSide::West,
                        offset: 4_000.0,
                    },
                ],
            })
            .collect();
        let mut endpoints = Vec::<(u32, u32, u32)>::new();
        for lane in 0..20u32 {
            endpoints.push((0, 21 + lane, 0));
            endpoints.push((1 + lane, 21 + lane, 2));
            endpoints.push((21 + lane, 41 + lane, 0));
            endpoints.push((41 + lane, 61 + lane, 0));
        }
        let mut state = seed;
        for source in 21..41u32 {
            for target in 41..61u32 {
                if source - 21 != target - 41 && next(&mut state) % 100 < 8 {
                    endpoints.push((source, target, 0));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, target_port))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: target_port,
                },
                net: id as u32,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    #[test]
    fn selects_a_preferred_backbone_deterministically() {
        let options = LayoutOptions {
            ordering_sweeps: 0,
            ..LayoutOptions::default()
        };
        let graph = preferred_backbone_graph(26);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let layers = topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
        let ordinary = placement::place_nodes(&indexed, &ranks, &layers, options);
        let preferred = placement::place_preferred_nodes(&indexed, &ranks, &layers, options);
        let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary);
        let preferred_alignment = placement::port_alignment_error(&indexed, &ranks, &preferred);
        assert!(
            placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment,),
            "ordinary={ordinary_alignment} preferred={preferred_alignment}"
        );
        let evaluate = |nodes: Vec<NodeGeometry>, supplemental: bool| {
            let mut edges = if supplemental {
                routing::route_supplemental_edges(&indexed, &nodes, &ranks, options)
            } else {
                routing::route_edges(&indexed, &nodes, &ranks, options)
            };
            let quality = routing::route_quality(&indexed, &edges);
            let mut nodes = nodes;
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let ordinary = evaluate(ordinary, false);
        let preferred = evaluate(preferred, true);
        assert!(candidate_quality_cmp(preferred.0, &preferred.1, ordinary.0, &ordinary.1).is_lt());

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, preferred.1);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn public_layout_selects_an_active_fanout_candidate() {
        let options = LayoutOptions {
            ordering_sweeps: 0,
            ..LayoutOptions::default()
        };
        let graph = active_fanout_fixture::graph();
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let layers = topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
        let ordinary = placement::place_nodes(&indexed, &ranks, &layers, options);
        let preferred = placement::place_preferred_nodes(&indexed, &ranks, &layers, options);
        let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary);
        let preferred_alignment = placement::port_alignment_error(&indexed, &ranks, &preferred);
        assert!(
            placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment),
            "ordinary={ordinary_alignment} preferred={preferred_alignment}"
        );
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let routed = routing::route_planned_candidates(&plan, &preferred, options, true);
        assert!(routed.fanout_trace.evaluated, "{:#?}", routed.fanout_trace);
        assert!(routed.fanout_trace.selected, "{:#?}", routed.fanout_trace);
        let mut adaptive_layouts = Vec::new();
        let mut candidate_nodes = preferred.clone();
        let mut candidate_edges = routed.primary;
        adaptive_layouts.push(placement::normalize(
            &mut candidate_nodes,
            &mut candidate_edges,
        ));
        if let Some((_, mut repair)) = routed.repair {
            let mut candidate_nodes = preferred.clone();
            adaptive_layouts.push(placement::normalize(&mut candidate_nodes, &mut repair));
        }
        let selected = layout(&graph, options).unwrap();
        assert!(
            adaptive_layouts.contains(&selected),
            "public layout did not retain the active adaptive family"
        );
    }

    #[test]
    fn selects_a_latest_feasible_rank_candidate_deterministically() {
        let options = LayoutOptions::default();
        let graph = slack_rank_graph(10);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let (ranks, alternative_ranks) = topology::rank_candidates(&indexed);
        let alternative_ranks = alternative_ranks.unwrap();
        let (forward, reverse, _) = topology::order_layer_candidates(&indexed, &ranks, 4, false);
        let ordinary_layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let baseline_crossings = forward.crossings.min(reverse.crossings);
        let (alternative_forward, alternative_reverse, _) =
            topology::order_layer_candidates(&indexed, &alternative_ranks, 3, false);
        let alternative_layers = if alternative_reverse.crossings < alternative_forward.crossings {
            &alternative_reverse.layers
        } else {
            &alternative_forward.layers
        };
        let alternative_crossings = alternative_forward
            .crossings
            .min(alternative_reverse.crossings);
        assert!(baseline_crossings - alternative_crossings >= baseline_crossings.div_ceil(100));

        let evaluate = |ranks: &[usize], layers: &[Vec<usize>], baseline| {
            let mut nodes = if baseline {
                placement::place_baseline_nodes(&indexed, ranks, layers, options)
            } else {
                placement::place_nodes(&indexed, ranks, layers, options)
            };
            let mut edges = routing::route_edges(&indexed, &nodes, ranks, options);
            let quality = routing::route_quality(&indexed, &edges);
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let baseline = evaluate(&ranks, &forward.layers, true);
        let ordinary = evaluate(&ranks, ordinary_layers, false);
        let alternative = evaluate(&alternative_ranks, alternative_layers, false);
        let best_ordinary =
            if candidate_quality_cmp(ordinary.0, &ordinary.1, baseline.0, &baseline.1).is_lt() {
                &ordinary
            } else {
                &baseline
            };
        assert!(
            candidate_quality_cmp(
                alternative.0,
                &alternative.1,
                best_ordinary.0,
                &best_ordinary.1,
            )
            .is_lt()
        );

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, alternative.1);

        let zero_sweeps = LayoutOptions {
            ordering_sweeps: 0,
            ..options
        };
        let zero_sweep_selected = layout(&graph, zero_sweeps).unwrap();
        assert_eq!(layout(&graph, zero_sweeps).unwrap(), zero_sweep_selected);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
        assert_eq!(layout(&permuted, zero_sweeps).unwrap(), zero_sweep_selected);
    }

    #[test]
    fn selects_a_reverse_median_candidate_deterministically() {
        let options = LayoutOptions::default();
        let graph = reverse_median_graph(1);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);

        assert!(
            topology::order_layer_candidates(&indexed, &ranks, 0, true)
                .2
                .is_none()
        );
        let one_sweep = topology::order_layer_candidates(&indexed, &ranks, 1, true)
            .2
            .unwrap()
            .layers;
        assert_eq!(
            topology::order_layer_candidates(&indexed, &ranks, 1, true)
                .2
                .unwrap()
                .layers,
            one_sweep
        );

        let (forward, reverse, alternative) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
        let ordinary_layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let alternative = alternative.unwrap();
        assert!(alternative.crossings < forward.crossings.min(reverse.crossings));

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
            ordinary_layers,
            options,
        ));
        let alternative = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            &alternative.layers,
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
                alternative.0,
                &alternative.1,
                best_ordinary.0,
                &best_ordinary.1,
            )
            .is_lt()
        );

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, alternative.1);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
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

    #[test]
    fn quality_effort_controls_the_admitted_public_layout_deterministically() {
        let graph = net_representative_graph_with_padding(600, 81);
        let options = LayoutOptions::default();
        let fast = super::layout_with_quality_effort(&graph, options, QualityEffort::Fast).unwrap();
        let quality =
            super::layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
        let max = super::layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();
        assert_eq!(layout(&graph, options).unwrap(), quality);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let fast_quality = routing::route_quality(&indexed, &fast.edges);
        let quality_quality = routing::route_quality(&indexed, &quality.edges);
        let max_quality = routing::route_quality(&indexed, &max.edges);
        assert_eq!(fast_quality.crossings, 552);
        assert_eq!(quality_quality.crossings, 405);
        assert!(candidate_quality_cmp(quality_quality, &quality, fast_quality, &fast).is_lt());
        assert!(!candidate_quality_cmp(max_quality, &max, quality_quality, &quality).is_gt());

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Fast).unwrap(),
            fast
        );
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Quality).unwrap(),
            quality
        );
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Max).unwrap(),
            max
        );
    }
}
