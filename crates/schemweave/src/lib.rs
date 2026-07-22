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
    let (ranks, latest_ranks) = topology::rank_candidates(&indexed);
    let (forward, reverse, net_representative) =
        topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
    let quality_layers = if reverse.crossings < forward.crossings {
        &reverse.layers
    } else {
        &forward.layers
    };
    let baseline_order_crossings = forward.crossings.min(reverse.crossings);
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
    let ordinary_nodes = placement::place_nodes(&indexed, &ranks, quality_layers, options);
    let preferred_nodes =
        placement::place_preferred_nodes(&indexed, &ranks, quality_layers, options);
    let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary_nodes);
    let preferred_alignment = placement::port_alignment_error(&indexed, &ranks, &preferred_nodes);
    if placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment) {
        evaluate(preferred_nodes);
    } else {
        evaluate(ordinary_nodes);
    }
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
            let mut edges = routing::route_edges(&indexed, &nodes, &alternative_ranks, options);
            let quality = routing::route_quality(&indexed, &edges);
            let candidate = placement::normalize(&mut nodes, &mut edges);
            retain_better_candidate(&mut best, quality, candidate);
        }
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
                height: 2_000.0,
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
                        offset: 1_000.0,
                    },
                    Port {
                        id: 2,
                        side: PortSide::West,
                        offset: 2_000.0,
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
        assert!(placement::preferred_alignment_is_significant(
            placement::port_alignment_error(&indexed, &ranks, &ordinary),
            placement::port_alignment_error(&indexed, &ranks, &preferred),
        ));
        let evaluate = |nodes: Vec<NodeGeometry>| {
            let mut edges = routing::route_edges(&indexed, &nodes, &ranks, options);
            let quality = routing::route_quality(&indexed, &edges);
            let mut nodes = nodes;
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let ordinary = evaluate(ordinary);
        let preferred = evaluate(preferred);
        assert!(candidate_quality_cmp(preferred.0, &preferred.1, ordinary.0, &ordinary.1).is_lt());

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, preferred.1);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
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
}
