use crate::{
    EdgeGeometry, Layout, LayoutOptions, NodeGeometry, PortSide, validation::IndexedGraph,
};

const PREFERRED_ALIGNMENT_WEIGHT: f64 = 8.0;
const MIN_PREFERRED_ALIGNMENT_IMPROVEMENT: f64 = 100_000.0;

#[derive(Clone, Copy)]
struct AlignmentPolicy {
    rounds: usize,
    stability_weight: f64,
    prefer_backbones: bool,
}

const BASELINE_ALIGNMENT: AlignmentPolicy = AlignmentPolicy {
    rounds: 4,
    stability_weight: 4.0,
    prefer_backbones: false,
};
const QUALITY_ALIGNMENT: AlignmentPolicy = AlignmentPolicy {
    rounds: 16,
    stability_weight: 0.25,
    prefer_backbones: false,
};
const PREFERRED_ALIGNMENT: AlignmentPolicy = AlignmentPolicy {
    prefer_backbones: true,
    ..QUALITY_ALIGNMENT
};

pub(crate) fn place_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<NodeGeometry> {
    place_nodes_with_alignment(graph, ranks, layers, options, QUALITY_ALIGNMENT)
}

pub(crate) fn place_preferred_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<NodeGeometry> {
    place_nodes_with_alignment(graph, ranks, layers, options, PREFERRED_ALIGNMENT)
}

pub(crate) fn place_baseline_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<NodeGeometry> {
    place_nodes_with_alignment(graph, ranks, layers, options, BASELINE_ALIGNMENT)
}

// Keep one WASM copy of the shared placement loop for all alignment policies.
#[inline(never)]
fn place_nodes_with_alignment(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
    policy: AlignmentPolicy,
) -> Vec<NodeGeometry> {
    let widths: Vec<f64> = layers
        .iter()
        .map(|layer| {
            layer
                .iter()
                .map(|&node| graph.nodes[node].width)
                .fold(0.0, f64::max)
        })
        .collect();
    let heights: Vec<f64> = layers
        .iter()
        .map(|layer| {
            layer
                .iter()
                .map(|&node| graph.nodes[node].height)
                .sum::<f64>()
                + options.node_gap * layer.len().saturating_sub(1) as f64
        })
        .collect();
    let canvas_height = heights.iter().copied().fold(0.0, f64::max);
    let mut layer_x = vec![0.0; layers.len()];
    for rank in 1..layers.len() {
        layer_x[rank] = layer_x[rank - 1] + widths[rank - 1] + options.layer_gap;
    }

    let mut positioned = vec![None; graph.nodes.len()];
    for (rank, layer) in layers.iter().enumerate() {
        let mut y = (canvas_height - heights[rank]) / 2.0;
        for &node_index in layer {
            let node = graph.nodes[node_index];
            positioned[node_index] = Some(NodeGeometry {
                id: node.id,
                x: layer_x[rank],
                y,
                width: node.width,
                height: node.height,
            });
            y += node.height + options.node_gap;
        }
    }
    let mut positioned: Vec<_> = positioned.into_iter().map(Option::unwrap).collect();
    align_connected_ports(
        graph,
        ranks,
        layers,
        &mut positioned,
        options.node_gap,
        policy,
    );
    positioned
}

#[derive(Clone, Copy)]
struct Alignment {
    neighbor: usize,
    own_offset: f64,
    neighbor_offset: f64,
    weight: f64,
}

fn preferred_edges(graph: &IndexedGraph<'_>, ranks: &[usize]) -> Vec<bool> {
    let mut qualifying = Vec::new();
    let mut incoming_degree = vec![0usize; graph.nodes.len()];
    let mut outgoing_degree = vec![0usize; graph.nodes.len()];
    for (edge_index, edge) in graph.edges.iter().enumerate() {
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        let source_port = graph.ports[source][&edge.source.port];
        let target_port = graph.ports[target][&edge.target.port];
        if ranks[target] == ranks[source] + 1
            && source_port.side == PortSide::East
            && target_port.side == PortSide::West
        {
            qualifying.push((edge_index, source, target));
            outgoing_degree[source] = outgoing_degree[source].saturating_add(1);
            incoming_degree[target] = incoming_degree[target].saturating_add(1);
        }
    }
    qualifying.sort_unstable_by_key(|&(_, source, target)| {
        (
            outgoing_degree[source].saturating_mul(incoming_degree[target]),
            outgoing_degree[source].saturating_add(incoming_degree[target]),
            graph.nodes[source].id,
            graph.nodes[target].id,
        )
    });
    let mut edges = vec![false; graph.edges.len()];
    let mut matched_outgoing = vec![false; graph.nodes.len()];
    let mut matched_incoming = vec![false; graph.nodes.len()];
    for (edge_index, source, target) in qualifying {
        if !matched_outgoing[source] && !matched_incoming[target] {
            edges[edge_index] = true;
            matched_outgoing[source] = true;
            matched_incoming[target] = true;
        }
    }
    edges
}

// Avoid specializing the alignment rounds into separate baseline and quality copies in WASM.
#[inline(never)]
fn align_connected_ports(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    nodes: &mut [NodeGeometry],
    node_gap: f64,
    policy: AlignmentPolicy,
) {
    let preferred = policy
        .prefer_backbones
        .then(|| preferred_edges(graph, ranks));

    let mut alignments = vec![Vec::<Alignment>::new(); graph.nodes.len()];
    for (edge_index, edge) in graph.edges.iter().enumerate() {
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        let source_port = graph.ports[source][&edge.source.port];
        let target_port = graph.ports[target][&edge.target.port];
        if ranks[source] >= ranks[target]
            || source_port.side != PortSide::East
            || target_port.side != PortSide::West
        {
            continue;
        }
        alignments[source].push(Alignment {
            neighbor: target,
            own_offset: source_port.offset,
            neighbor_offset: target_port.offset,
            weight: if preferred
                .as_ref()
                .is_some_and(|preferred| preferred[edge_index])
            {
                PREFERRED_ALIGNMENT_WEIGHT
            } else {
                1.0
            },
        });
        alignments[target].push(Alignment {
            neighbor: source,
            own_offset: target_port.offset,
            neighbor_offset: source_port.offset,
            weight: if preferred
                .as_ref()
                .is_some_and(|preferred| preferred[edge_index])
            {
                PREFERRED_ALIGNMENT_WEIGHT
            } else {
                1.0
            },
        });
    }

    for _ in 0..policy.rounds {
        for layer in layers.iter().skip(1) {
            align_layer(layer, &alignments, nodes, node_gap, policy.stability_weight);
        }
        for layer in layers.iter().take(layers.len().saturating_sub(1)).rev() {
            align_layer(layer, &alignments, nodes, node_gap, policy.stability_weight);
        }
    }
}

pub(crate) fn port_alignment_error(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    nodes: &[NodeGeometry],
) -> f64 {
    let mut error = 0.0;
    for edge in &graph.edges {
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        let source_port = graph.ports[source][&edge.source.port];
        let target_port = graph.ports[target][&edge.target.port];
        if ranks[target] == ranks[source] + 1
            && source_port.side == PortSide::East
            && target_port.side == PortSide::West
        {
            error +=
                (nodes[source].y + source_port.offset - nodes[target].y - target_port.offset).abs();
        }
    }
    error
}

pub(crate) fn preferred_alignment_is_significant(ordinary: f64, preferred: f64) -> bool {
    ordinary - preferred >= MIN_PREFERRED_ALIGNMENT_IMPROVEMENT
        && preferred * 20.0 <= ordinary * 19.0
}

fn align_layer(
    layer: &[usize],
    alignments: &[Vec<Alignment>],
    nodes: &mut [NodeGeometry],
    node_gap: f64,
    stability_weight: f64,
) {
    if layer.is_empty() {
        return;
    }
    let mut offsets = Vec::with_capacity(layer.len());
    let mut offset = 0.0;
    let mut targets = Vec::with_capacity(layer.len());
    let mut weights = Vec::with_capacity(layer.len());
    for &node in layer {
        offsets.push(offset);
        let mut weighted_y = stability_weight * nodes[node].y;
        let mut weight = stability_weight;
        for alignment in &alignments[node] {
            weighted_y += alignment.weight
                * (nodes[alignment.neighbor].y + alignment.neighbor_offset - alignment.own_offset);
            weight += alignment.weight;
        }
        targets.push(weighted_y / weight - offset);
        weights.push(weight);
        offset += nodes[node].height + node_gap;
    }
    let projected = isotonic_projection(&targets, &weights);
    for ((&node, &base), y) in layer.iter().zip(&offsets).zip(projected) {
        nodes[node].y = base + y;
    }
}

fn isotonic_projection(targets: &[f64], weights: &[f64]) -> Vec<f64> {
    #[derive(Clone, Copy)]
    struct Block {
        start: usize,
        end: usize,
        weight: f64,
        weighted_sum: f64,
    }

    let mut blocks = Vec::<Block>::with_capacity(targets.len());
    for (index, (&target, &weight)) in targets.iter().zip(weights).enumerate() {
        blocks.push(Block {
            start: index,
            end: index + 1,
            weight,
            weighted_sum: target * weight,
        });
        while blocks.len() >= 2 {
            let right = blocks[blocks.len() - 1];
            let left = blocks[blocks.len() - 2];
            if left.weighted_sum / left.weight <= right.weighted_sum / right.weight {
                break;
            }
            blocks.truncate(blocks.len() - 2);
            blocks.push(Block {
                start: left.start,
                end: right.end,
                weight: left.weight + right.weight,
                weighted_sum: left.weighted_sum + right.weighted_sum,
            });
        }
    }
    let mut projected = vec![0.0; targets.len()];
    for block in blocks {
        projected[block.start..block.end].fill(block.weighted_sum / block.weight);
    }
    projected
}

pub(crate) fn normalize(nodes: &mut [NodeGeometry], edges: &mut [EdgeGeometry]) -> Layout {
    let mut min_x = nodes.iter().map(|node| node.x).fold(0.0, f64::min);
    let mut min_y = nodes.iter().map(|node| node.y).fold(0.0, f64::min);
    for point in edges.iter().flat_map(|edge| &edge.points) {
        min_x = min_x.min(point.x);
        min_y = min_y.min(point.y);
    }
    if min_x != 0.0 || min_y != 0.0 {
        for node in nodes.iter_mut() {
            node.x -= min_x;
            node.y -= min_y;
        }
        for point in edges.iter_mut().flat_map(|edge| &mut edge.points) {
            point.x -= min_x;
            point.y -= min_y;
        }
    }
    let width = nodes
        .iter()
        .map(|node| node.x + node.width)
        .chain(
            edges
                .iter()
                .flat_map(|edge| edge.points.iter().map(|point| point.x)),
        )
        .fold(0.0, f64::max);
    let height = nodes
        .iter()
        .map(|node| node.y + node.height)
        .chain(
            edges
                .iter()
                .flat_map(|edge| edge.points.iter().map(|point| point.y)),
        )
        .fold(0.0, f64::max);
    Layout {
        nodes: nodes.to_vec(),
        edges: edges.to_vec(),
        width,
        height,
    }
}

/// Place nodes without routing edges for consumers that provide a custom routing stage.
pub fn place(
    graph: &crate::Graph,
    options: LayoutOptions,
) -> Result<Vec<NodeGeometry>, crate::LayoutError> {
    let indexed = crate::validation::validate_and_index(graph, options)?;
    let ranks = crate::topology::assign_ranks(&indexed);
    let layers = crate::topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
    Ok(place_nodes(&indexed, &ranks, &layers, options))
}

#[cfg(test)]
mod tests {
    use super::{
        Alignment, align_layer, isotonic_projection, preferred_alignment_is_significant,
        preferred_edges,
    };
    use crate::{
        Edge, Endpoint, Graph, LayoutOptions, Node, NodeGeometry, Port, PortSide, place,
        topology::assign_ranks, validation::validate_and_index,
    };

    #[test]
    fn preferred_alignment_gate_requires_absolute_and_relative_value() {
        assert!(!preferred_alignment_is_significant(
            2_000_000.0,
            1_900_001.0
        ));
        assert!(preferred_alignment_is_significant(2_000_000.0, 1_900_000.0));
        assert!(!preferred_alignment_is_significant(1_000_000.0, 900_001.0));
        assert!(preferred_alignment_is_significant(1_000_000.0, 900_000.0));
        assert!(!preferred_alignment_is_significant(100_000.0, 110_000.0));
    }

    #[test]
    fn preferred_edges_form_a_deterministic_low_degree_matching() {
        let node = |id| Node {
            id,
            width: 40.0,
            height: 40.0,
            cycle_breaker: false,
            ports: vec![
                Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 20.0,
                },
                Port {
                    id: 1,
                    side: PortSide::East,
                    offset: 20.0,
                },
            ],
        };
        let edge = |id, source, target| Edge {
            id,
            source: Endpoint {
                node: source,
                port: 1,
            },
            target: Endpoint {
                node: target,
                port: 0,
            },
            net: id,
            participates_in_ranking: true,
        };
        let selected = |graph: &Graph| {
            let indexed = validate_and_index(graph, LayoutOptions::default()).unwrap();
            let ranks = assign_ranks(&indexed);
            preferred_edges(&indexed, &ranks)
                .into_iter()
                .zip(&indexed.edges)
                .filter_map(|(preferred, edge)| preferred.then_some(edge.id))
                .collect::<Vec<_>>()
        };
        let graph = Graph {
            nodes: (0..4).map(node).collect(),
            edges: vec![edge(30, 0, 2), edge(20, 0, 3), edge(10, 1, 2)],
        };
        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        permuted.edges.reverse();

        assert_eq!(selected(&graph), vec![10, 20]);
        assert_eq!(selected(&permuted), vec![10, 20]);
    }

    #[test]
    fn isotonic_projection_merges_only_violating_neighbors() {
        let projected = isotonic_projection(&[3.0, 1.0, 4.0], &[1.0, 1.0, 1.0]);

        assert_eq!(projected, vec![2.0, 2.0, 4.0]);
    }

    #[test]
    fn layer_alignment_preserves_order_and_minimum_gap() {
        let mut nodes: Vec<_> = (0..6)
            .map(|id| NodeGeometry {
                id,
                x: 0.0,
                y: if id < 3 { id as f64 * 30.0 } else { 0.0 },
                width: 20.0,
                height: 20.0,
            })
            .collect();
        nodes[3].y = 100.0;
        nodes[4].y = 50.0;
        nodes[5].y = 0.0;
        let mut alignments = vec![Vec::new(); nodes.len()];
        for (node, neighbor) in [(0, 3), (1, 4), (2, 5)] {
            alignments[node].push(Alignment {
                neighbor,
                own_offset: 10.0,
                neighbor_offset: 10.0,
                weight: 1.0,
            });
        }

        align_layer(&[0, 1, 2], &alignments, &mut nodes, 10.0, 1.0);

        assert!(nodes[1].y >= nodes[0].y + nodes[0].height + 10.0);
        assert!(nodes[2].y >= nodes[1].y + nodes[1].height + 10.0);
    }

    #[test]
    fn placement_converges_connected_ports_to_alignment() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 1,
                    width: 40.0,
                    height: 50.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 5.0,
                    }],
                },
                Node {
                    id: 2,
                    width: 40.0,
                    height: 50.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 1,
                        side: PortSide::West,
                        offset: 45.0,
                    }],
                },
            ],
            edges: vec![Edge {
                id: 1,
                source: Endpoint { node: 1, port: 1 },
                target: Endpoint { node: 2, port: 1 },
                net: 1,
                participates_in_ranking: true,
            }],
        };

        let nodes = place(&graph, LayoutOptions::default()).unwrap();
        let source = nodes.iter().find(|node| node.id == 1).unwrap();
        let target = nodes.iter().find(|node| node.id == 2).unwrap();
        let aligned_port_delta = (source.y + 5.0 - target.y - 45.0).abs();

        assert!(aligned_port_delta < 1.0, "{aligned_port_delta}");
    }
}
