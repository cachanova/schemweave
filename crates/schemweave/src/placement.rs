use crate::{
    EdgeGeometry, Layout, LayoutOptions, NetId, NodeGeometry, PortSide, validation::IndexedGraph,
};
use std::collections::BTreeMap;

const PREFERRED_ALIGNMENT_WEIGHT: f64 = 8.0;
const CHAIN_ALIGNMENT_WEIGHT: f64 = 16.0;
const CHAIN_ALIGNMENT_EPSILON: f64 = 1e-7;
pub(crate) const MAX_CHAIN_HEIGHT_FACTOR: f64 = 1.10;
pub(crate) const MAX_CHAIN_CANDIDATE_NODES: usize = 1_000;
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
    place_nodes_with_alignment(graph, ranks, layers, options, QUALITY_ALIGNMENT, false)
}

pub(crate) fn place_demand_aware_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<NodeGeometry> {
    place_nodes_with_alignment(graph, ranks, layers, options, QUALITY_ALIGNMENT, true)
}

pub(crate) fn demand_aware_spacing_is_relevant(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    rank_count: usize,
    options: LayoutOptions,
) -> bool {
    let options = crate::effective_layout_options(options);
    demand_aware_layer_gaps(graph, ranks, rank_count, options)
        .into_iter()
        .any(|gap| gap >= options.layer_gap * 1.5)
}

pub(crate) fn place_preferred_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<NodeGeometry> {
    place_nodes_with_alignment(graph, ranks, layers, options, PREFERRED_ALIGNMENT, false)
}

pub(crate) fn place_straight_chain_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    ordinary: &[NodeGeometry],
    node_gap: f64,
) -> Option<Vec<NodeGeometry>> {
    let chains = preferred_chain_targets(graph, ranks, ordinary)?;
    let packed_height = layers
        .iter()
        .map(|layer| {
            layer
                .iter()
                .map(|&node| graph.nodes[node].height)
                .sum::<f64>()
                + node_gap * layer.len().saturating_sub(1) as f64
        })
        .fold(0.0, f64::max);
    let mut nodes = ordinary.to_vec();
    for layer in layers {
        let mut offsets = Vec::with_capacity(layer.len());
        let mut projected_targets = Vec::with_capacity(layer.len());
        let mut weights = Vec::with_capacity(layer.len());
        let mut offset = 0.0;
        for &node in layer {
            offsets.push(offset);
            projected_targets.push(chains.targets[node].unwrap_or(nodes[node].y) - offset);
            weights.push(if chains.targets[node].is_some() {
                CHAIN_ALIGNMENT_WEIGHT
            } else {
                1.0
            });
            offset += nodes[node].height + node_gap;
        }
        for ((&node, &base), y) in layer
            .iter()
            .zip(&offsets)
            .zip(isotonic_projection(&projected_targets, &weights))
        {
            nodes[node].y = base + y;
        }
    }
    straight_chain_is_eligible(graph, ordinary, &nodes, &chains.edges, packed_height)
        .then_some(nodes)
}

pub(crate) fn place_baseline_nodes(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<NodeGeometry> {
    place_nodes_with_alignment(graph, ranks, layers, options, BASELINE_ALIGNMENT, false)
}

// Keep one WASM copy of the shared placement loop for all alignment policies.
#[inline(never)]
fn place_nodes_with_alignment(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
    policy: AlignmentPolicy,
    demand_aware_spacing: bool,
) -> Vec<NodeGeometry> {
    let options = crate::effective_layout_options(options);
    let widths: Vec<f64> = layers
        .iter()
        .map(|layer| {
            layer
                .iter()
                .map(|&node| graph.nodes[node].width)
                .fold(0.0, f64::max)
        })
        .collect();
    let node_gaps = clearance_aware_node_gaps(graph, ranks, layers, options);
    let heights: Vec<f64> = layers
        .iter()
        .enumerate()
        .map(|(rank, layer)| {
            layer
                .iter()
                .map(|&node| graph.nodes[node].height)
                .sum::<f64>()
                + node_gaps[rank].iter().sum::<f64>()
        })
        .collect();
    let canvas_height = heights.iter().copied().fold(0.0, f64::max);
    let layer_gaps = if demand_aware_spacing {
        demand_aware_layer_gaps(graph, ranks, layers.len(), options)
    } else {
        vec![options.layer_gap; layers.len().saturating_sub(1)]
    };
    let mut layer_x = vec![0.0; layers.len()];
    for rank in 1..layers.len() {
        layer_x[rank] = layer_x[rank - 1] + widths[rank - 1] + layer_gaps[rank - 1];
    }

    let mut positioned = vec![None; graph.nodes.len()];
    for (rank, layer) in layers.iter().enumerate() {
        let mut y = (canvas_height - heights[rank]) / 2.0;
        for (position, &node_index) in layer.iter().enumerate() {
            let node = graph.nodes[node_index];
            positioned[node_index] = Some(NodeGeometry {
                id: node.id,
                x: layer_x[rank]
                    + if graph.boundary_outputs[node_index] {
                        widths[rank] - node.width
                    } else {
                        0.0
                    },
                y,
                width: node.width,
                height: node.height,
            });
            y += node.height + node_gaps[rank].get(position).copied().unwrap_or(0.0);
        }
    }
    let mut positioned: Vec<_> = positioned.into_iter().map(Option::unwrap).collect();
    align_connected_ports(graph, ranks, layers, &mut positioned, &node_gaps, policy);
    positioned
}

fn clearance_aware_node_gaps(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
) -> Vec<Vec<f64>> {
    let base_gap = options
        .node_gap
        .max(options.edge_node_clearance + options.route_lane_gap);
    let mut gaps = layers
        .iter()
        .map(|layer| vec![base_gap; layer.len().saturating_sub(1)])
        .collect::<Vec<_>>();
    if options.edge_node_clearance == 0.0 {
        return gaps;
    }
    let transit_gap =
        (options.edge_node_clearance * 2.0 + options.route_lane_gap).max(options.node_gap);
    let endpoint_gap =
        (options.edge_node_clearance * 2.0 + options.route_lane_gap).max(options.node_gap);
    let mut north_escape = vec![false; graph.nodes.len()];
    let mut south_escape = vec![false; graph.nodes.len()];
    for edge in &graph.edges {
        for (endpoint, node) in [
            (edge.source, graph.node_index[&edge.source.node]),
            (edge.target, graph.node_index[&edge.target.node]),
        ] {
            match graph.ports[node][&endpoint.port].side {
                PortSide::North => north_escape[node] = true,
                PortSide::South => south_escape[node] = true,
                PortSide::East | PortSide::West => {}
            }
        }
    }
    for (rank, layer) in layers.iter().enumerate() {
        for (position, pair) in layer.windows(2).enumerate() {
            if south_escape[pair[0]] || north_escape[pair[1]] {
                gaps[rank][position] = endpoint_gap;
            }
        }
    }
    let mut layer_position = vec![0usize; graph.nodes.len()];
    for layer in layers {
        for (position, &node) in layer.iter().enumerate() {
            layer_position[node] = position;
        }
    }
    let mut demand = layers
        .iter()
        .map(|layer| vec![0usize; layer.len().saturating_sub(1)])
        .collect::<Vec<_>>();
    for edge in &graph.edges {
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        let source_rank = ranks[source];
        let target_rank = ranks[target];
        if source_rank >= target_rank
            || graph.ports[source][&edge.source.port].side != PortSide::East
            || graph.ports[target][&edge.target.port].side != PortSide::West
        {
            continue;
        }
        for rank in source_rank + 1..target_rank {
            if demand[rank].is_empty() {
                continue;
            }
            let source_fraction =
                (layer_position[source] as f64 + 0.5) / layers[source_rank].len() as f64;
            let target_fraction =
                (layer_position[target] as f64 + 0.5) / layers[target_rank].len() as f64;
            let progress = (rank - source_rank) as f64 / (target_rank - source_rank) as f64;
            let projected = source_fraction + (target_fraction - source_fraction) * progress;
            let boundary = (projected * layers[rank].len() as f64)
                .round()
                .clamp(1.0, layers[rank].len().saturating_sub(1) as f64)
                as usize;
            demand[rank][boundary - 1] = demand[rank][boundary - 1].saturating_add(1);
        }
    }
    for rank in 0..layers.len() {
        for (gap, &count) in demand[rank].iter().enumerate() {
            if count > 0 {
                gaps[rank][gap] = transit_gap;
            }
        }
    }
    gaps
}

fn demand_aware_layer_gaps(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    rank_count: usize,
    options: LayoutOptions,
) -> Vec<f64> {
    let gap_count = rank_count.saturating_sub(1);
    if gap_count == 0 {
        return Vec::new();
    }
    let mut spans_by_net = BTreeMap::<NetId, Vec<(usize, usize)>>::new();
    for edge in &graph.edges {
        if !edge.participates_in_ranking {
            continue;
        }
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        let source_rank = ranks[source];
        let target_rank = ranks[target];
        if source_rank < target_rank
            && graph.ports[source][&edge.source.port].side == PortSide::East
            && graph.ports[target][&edge.target.port].side == PortSide::West
        {
            spans_by_net
                .entry(edge.net)
                .or_default()
                .push((source_rank, target_rank));
        }
    }
    let mut difference = vec![0isize; rank_count];
    for spans in spans_by_net.values_mut() {
        spans.sort_unstable();
        let mut merged = Vec::with_capacity(spans.len());
        for &(start, end) in spans.iter() {
            if let Some((_, merged_end)) = merged.last_mut()
                && start <= *merged_end
            {
                *merged_end = (*merged_end).max(end);
            } else {
                merged.push((start, end));
            }
        }
        for (start, end) in merged {
            difference[start] += 1;
            difference[end] -= 1;
        }
    }
    let mut active = 0isize;
    (0..gap_count)
        .map(|gap| {
            active += difference[gap];
            let lane_count = usize::try_from(active).expect("net span count is nonnegative");
            let boundary = options.port_stub.max(options.edge_node_clearance) * 2.0;
            let demanded =
                boundary + options.route_lane_gap * (lane_count.saturating_add(1) as f64);
            options.layer_gap.max(demanded)
        })
        .collect()
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
        if ranks[source].checked_add(1) == Some(ranks[target])
            && source_port.side == PortSide::East
            && target_port.side == PortSide::West
        {
            qualifying.push((edge_index, source, target));
            outgoing_degree[source] = outgoing_degree[source].saturating_add(1);
            incoming_degree[target] = incoming_degree[target].saturating_add(1);
        }
    }
    qualifying.sort_unstable_by_key(|&(edge_index, source, target)| {
        let edge = graph.edges[edge_index];
        (
            outgoing_degree[source].saturating_mul(incoming_degree[target]),
            outgoing_degree[source].saturating_add(incoming_degree[target]),
            graph.nodes[source].id,
            graph.nodes[target].id,
            edge.id,
            edge.source.port,
            edge.target.port,
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

struct PreferredChains {
    targets: Vec<Option<f64>>,
    edges: Vec<usize>,
}

fn preferred_chain_targets(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    ordinary: &[NodeGeometry],
) -> Option<PreferredChains> {
    let preferred = preferred_edges(graph, ranks);
    let mut incoming = vec![None; graph.nodes.len()];
    let mut outgoing = vec![None; graph.nodes.len()];
    for (edge_index, (&selected, edge)) in preferred.iter().zip(&graph.edges).enumerate() {
        if !selected {
            continue;
        }
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        outgoing[source] = Some(edge_index);
        incoming[target] = Some(edge_index);
    }

    let mut targets = vec![None; graph.nodes.len()];
    let mut chain_edges = Vec::new();
    for start in 0..graph.nodes.len() {
        if incoming[start].is_some() || outgoing[start].is_none() {
            continue;
        }
        let mut chain = vec![(start, 0.0)];
        let mut edges = Vec::new();
        let mut node = start;
        while let Some(edge_index) = outgoing[node] {
            let edge = graph.edges[edge_index];
            let target = graph.node_index[&edge.target.node];
            let source_port = graph.ports[node][&edge.source.port];
            let target_port = graph.ports[target][&edge.target.port];
            let relative_y = chain.last().unwrap().1 + source_port.offset - target_port.offset;
            chain.push((target, relative_y));
            edges.push(edge_index);
            node = target;
        }
        if chain.len() < 3 {
            continue;
        }
        chain_edges.extend(edges);
        let anchor = chain
            .iter()
            .map(|&(node, relative_y)| ordinary[node].y - relative_y)
            .sum::<f64>()
            / chain.len() as f64;
        for (node, relative_y) in chain {
            targets[node] = Some(anchor + relative_y);
        }
    }
    (!chain_edges.is_empty()).then_some(PreferredChains {
        targets,
        edges: chain_edges,
    })
}

fn straight_chain_is_eligible(
    graph: &IndexedGraph<'_>,
    ordinary: &[NodeGeometry],
    candidate: &[NodeGeometry],
    chain_edges: &[usize],
    packed_height: f64,
) -> bool {
    let newly_straight = chain_edges
        .iter()
        .filter(|&&edge_index| {
            let edge = graph.edges[edge_index];
            let source = graph.node_index[&edge.source.node];
            let target = graph.node_index[&edge.target.node];
            let source_offset = graph.ports[source][&edge.source.port].offset;
            let target_offset = graph.ports[target][&edge.target.port].offset;
            let error = |nodes: &[NodeGeometry]| {
                (nodes[source].y + source_offset - nodes[target].y - target_offset).abs()
            };
            error(ordinary) > CHAIN_ALIGNMENT_EPSILON && error(candidate) <= CHAIN_ALIGNMENT_EPSILON
        })
        .count();
    straight_chain_scale_is_eligible(graph.nodes.len(), newly_straight, chain_edges.len())
        && vertical_span(candidate) <= packed_height * MAX_CHAIN_HEIGHT_FACTOR
}

fn straight_chain_scale_is_eligible(
    node_count: usize,
    newly_straight: usize,
    chain_edge_count: usize,
) -> bool {
    newly_straight >= 2
        && node_count <= MAX_CHAIN_CANDIDATE_NODES
        // Above the interactive threshold, require the second exact route family to straighten a
        // substantial share of its matched backbone rather than paying that cost for isolated wins.
        && (node_count <= 600 || newly_straight.saturating_mul(5) >= chain_edge_count)
}

pub(crate) fn vertical_span(nodes: &[NodeGeometry]) -> f64 {
    let top = nodes
        .iter()
        .map(|node| node.y)
        .fold(f64::INFINITY, f64::min);
    let bottom = nodes
        .iter()
        .map(|node| node.y + node.height)
        .fold(f64::NEG_INFINITY, f64::max);
    bottom - top
}

// Avoid specializing the alignment rounds into separate baseline and quality copies in WASM.
#[inline(never)]
fn align_connected_ports(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    layers: &[Vec<usize>],
    nodes: &mut [NodeGeometry],
    node_gaps: &[Vec<f64>],
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
        for (rank, layer) in layers.iter().enumerate().skip(1) {
            align_layer_with_gaps(
                layer,
                &alignments,
                nodes,
                &node_gaps[rank],
                policy.stability_weight,
            );
        }
        for (rank, layer) in layers
            .iter()
            .enumerate()
            .take(layers.len().saturating_sub(1))
            .rev()
        {
            align_layer_with_gaps(
                layer,
                &alignments,
                nodes,
                &node_gaps[rank],
                policy.stability_weight,
            );
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
        if ranks[source].checked_add(1) == Some(ranks[target])
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
        && preferred * 50.0 <= ordinary * 47.0
}

pub(crate) fn preferred_alignment_can_be_significant(ordinary: f64) -> bool {
    // Alignment error is nonnegative, so no candidate can reduce it by more than `ordinary`.
    ordinary >= MIN_PREFERRED_ALIGNMENT_IMPROVEMENT
}

#[cfg(test)]
fn align_layer(
    layer: &[usize],
    alignments: &[Vec<Alignment>],
    nodes: &mut [NodeGeometry],
    node_gap: f64,
    stability_weight: f64,
) {
    let gaps = vec![node_gap; layer.len().saturating_sub(1)];
    align_layer_with_gaps(layer, alignments, nodes, &gaps, stability_weight);
}

fn align_layer_with_gaps(
    layer: &[usize],
    alignments: &[Vec<Alignment>],
    nodes: &mut [NodeGeometry],
    node_gaps: &[f64],
    stability_weight: f64,
) {
    if layer.is_empty() {
        return;
    }
    let mut offsets = Vec::with_capacity(layer.len());
    let mut offset = 0.0;
    let mut targets = Vec::with_capacity(layer.len());
    let mut weights = Vec::with_capacity(layer.len());
    for (position, &node) in layer.iter().enumerate() {
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
        offset += nodes[node].height + node_gaps.get(position).copied().unwrap_or(0.0);
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

#[allow(dead_code)]
pub(crate) fn normalize(nodes: &mut [NodeGeometry], edges: &mut [EdgeGeometry]) -> Layout {
    let (width, height) = normalize_in_place(nodes, edges);
    Layout {
        nodes: nodes.to_vec(),
        edges: edges.to_vec(),
        boundary_bundles: Vec::new(),
        width,
        height,
    }
}

pub(crate) fn normalize_owned(
    mut nodes: Vec<NodeGeometry>,
    mut edges: Vec<EdgeGeometry>,
) -> Layout {
    let (width, height) = normalize_in_place(&mut nodes, &mut edges);
    Layout {
        nodes,
        edges,
        boundary_bundles: Vec::new(),
        width,
        height,
    }
}

fn normalize_in_place(nodes: &mut [NodeGeometry], edges: &mut [EdgeGeometry]) -> (f64, f64) {
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
    (width, height)
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
        Alignment, align_layer, demand_aware_layer_gaps, isotonic_projection, normalize,
        normalize_owned, place_straight_chain_nodes, preferred_alignment_can_be_significant,
        preferred_alignment_is_significant, preferred_chain_targets, preferred_edges,
        straight_chain_scale_is_eligible,
    };
    use crate::{
        Edge, EdgeGeometry, Endpoint, Graph, LayoutOptions, Node, NodeGeometry, Point, Port,
        PortSide, place, topology::assign_ranks, validation::validate_and_index,
    };

    #[test]
    fn demand_aware_layer_gaps_count_distinct_net_spans_deterministically() {
        let nodes = vec![
            Node {
                id: 1,
                width: 40.0,
                height: 40.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 20.0,
                }],
            },
            Node {
                id: 2,
                width: 40.0,
                height: 40.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 20.0,
                }],
            },
        ];
        let mut edges = (0..12)
            .map(|net| Edge {
                id: net,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net,
                participates_in_ranking: true,
            })
            .collect::<Vec<_>>();
        edges.push(Edge {
            id: 99,
            source: Endpoint { node: 1, port: 0 },
            target: Endpoint { node: 2, port: 0 },
            net: 0,
            participates_in_ranking: true,
        });
        let graph = Graph { nodes, edges };
        let options = LayoutOptions::default();
        let indexed = validate_and_index(&graph, options).unwrap();

        assert_eq!(
            demand_aware_layer_gaps(&indexed, &[0, 2], 3, options),
            vec![72.0, 72.0],
        );

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        let indexed = validate_and_index(&permuted, options).unwrap();
        assert_eq!(
            demand_aware_layer_gaps(&indexed, &[0, 2], 3, options),
            vec![72.0, 72.0],
        );
    }

    #[test]
    fn owned_normalization_matches_the_borrowed_api_exactly() {
        let nodes = vec![NodeGeometry {
            id: 7,
            x: -4.0,
            y: 3.0,
            width: 20.0,
            height: 10.0,
        }];
        let edges = vec![EdgeGeometry {
            id: 11,
            points: vec![Point { x: -8.0, y: -6.0 }, Point { x: 5.0, y: -6.0 }],
        }];
        let mut borrowed_nodes = nodes.clone();
        let mut borrowed_edges = edges.clone();

        assert_eq!(
            normalize_owned(nodes, edges),
            normalize(&mut borrowed_nodes, &mut borrowed_edges)
        );
    }

    #[test]
    fn preferred_alignment_gate_requires_absolute_and_relative_value() {
        assert!(!preferred_alignment_can_be_significant(99_999.0));
        assert!(preferred_alignment_can_be_significant(100_000.0));
        assert!(!preferred_alignment_is_significant(1_000_000.0, 900_001.0));
        assert!(preferred_alignment_is_significant(1_000_000.0, 900_000.0));
        assert!(!preferred_alignment_is_significant(
            2_000_000.0,
            1_880_001.0
        ));
        assert!(preferred_alignment_is_significant(2_000_000.0, 1_880_000.0));
        assert!(!preferred_alignment_is_significant(
            3_000_000.0,
            2_900_000.0
        ));
        assert!(!preferred_alignment_is_significant(100_000.0, 110_000.0));

        for ordinary in [0.0, 99_999.0, 100_000.0, 1_000_000.0, 2_000_000.0] {
            for preferred in [0.0, 50_000.0, 90_000.0, 900_000.0, 1_880_000.0] {
                if preferred_alignment_is_significant(ordinary, preferred) {
                    assert!(preferred_alignment_can_be_significant(ordinary));
                }
            }
        }
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
    fn preferred_edge_matching_has_explicit_eligibility_boundaries() {
        let node = |id| Node {
            id,
            width: 40.0,
            height: 40.0,
            cycle_breaker: false,
            ports: vec![
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
                Port {
                    id: 2,
                    side: PortSide::North,
                    offset: 10.0,
                },
                Port {
                    id: 3,
                    side: PortSide::South,
                    offset: 10.0,
                },
            ],
        };
        let edge = |id, source, source_port, target, target_port, participates_in_ranking| Edge {
            id,
            source: Endpoint {
                node: source,
                port: source_port,
            },
            target: Endpoint {
                node: target,
                port: target_port,
            },
            net: id,
            participates_in_ranking,
        };
        let graph = Graph {
            nodes: (0..10).map(node).collect(),
            edges: vec![
                edge(10, 0, 1, 1, 0, true),
                edge(20, 2, 1, 3, 0, true),
                edge(30, 4, 1, 5, 0, true),
                edge(40, 7, 1, 6, 0, false),
                edge(50, 8, 2, 9, 3, true),
                edge(60, 6, 1, 7, 0, false),
            ],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        // Eligible, long-span, same-rank, backward, wrong-side, and non-ranking adjacent.
        let ranks = [0, 1, 0, 2, 0, 0, 0, 1, 0, 1];
        let selected = preferred_edges(&indexed, &ranks)
            .into_iter()
            .zip(&indexed.edges)
            .filter_map(|(preferred, edge)| preferred.then_some(edge.id))
            .collect::<Vec<_>>();

        assert_eq!(selected, vec![10, 60]);
    }

    #[test]
    fn preferred_edge_matching_ties_parallel_edges_by_stable_identity() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 0,
                    width: 40.0,
                    height: 40.0,
                    cycle_breaker: false,
                    ports: vec![
                        Port {
                            id: 1,
                            side: PortSide::East,
                            offset: 10.0,
                        },
                        Port {
                            id: 3,
                            side: PortSide::East,
                            offset: 30.0,
                        },
                    ],
                },
                Node {
                    id: 1,
                    width: 40.0,
                    height: 40.0,
                    cycle_breaker: false,
                    ports: vec![
                        Port {
                            id: 0,
                            side: PortSide::West,
                            offset: 10.0,
                        },
                        Port {
                            id: 2,
                            side: PortSide::West,
                            offset: 30.0,
                        },
                    ],
                },
            ],
            edges: vec![
                Edge {
                    id: 20,
                    source: Endpoint { node: 0, port: 1 },
                    target: Endpoint { node: 1, port: 0 },
                    net: 20,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 10,
                    source: Endpoint { node: 0, port: 3 },
                    target: Endpoint { node: 1, port: 2 },
                    net: 10,
                    participates_in_ranking: true,
                },
            ],
        };
        let selected = |graph: &Graph| {
            let indexed = validate_and_index(graph, LayoutOptions::default()).unwrap();
            preferred_edges(&indexed, &[0, 1])
                .into_iter()
                .zip(&indexed.edges)
                .filter_map(|(preferred, edge)| preferred.then_some(edge.id))
                .collect::<Vec<_>>()
        };
        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        permuted.edges.reverse();

        assert_eq!(selected(&graph), vec![10]);
        assert_eq!(selected(&permuted), vec![10]);
    }

    #[test]
    fn straight_chain_candidate_requires_two_matched_steps_and_is_permutation_invariant() {
        assert!(!straight_chain_scale_is_eligible(600, 1, 2));
        assert!(straight_chain_scale_is_eligible(600, 2, 100));
        assert!(!straight_chain_scale_is_eligible(601, 19, 100));
        assert!(straight_chain_scale_is_eligible(601, 20, 100));
        assert!(!straight_chain_scale_is_eligible(1_001, 1_000, 1_000));

        let graph = Graph {
            nodes: vec![
                Node {
                    id: 0,
                    width: 40.0,
                    height: 40.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 20.0,
                    }],
                },
                Node {
                    id: 1,
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
                },
                Node {
                    id: 2,
                    width: 40.0,
                    height: 40.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 20.0,
                    }],
                },
            ],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 0, port: 1 },
                    target: Endpoint { node: 1, port: 0 },
                    net: 10,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 20,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 20,
                    participates_in_ranking: true,
                },
            ],
        };
        let ordinary = vec![
            NodeGeometry {
                id: 0,
                x: 0.0,
                y: 0.0,
                width: 40.0,
                height: 40.0,
            },
            NodeGeometry {
                id: 1,
                x: 100.0,
                y: 100.0,
                width: 40.0,
                height: 40.0,
            },
            NodeGeometry {
                id: 2,
                x: 200.0,
                y: -50.0,
                width: 40.0,
                height: 40.0,
            },
        ];
        let layers = vec![vec![0], vec![1], vec![2]];
        let place_candidate = |graph: &Graph| {
            let indexed = validate_and_index(graph, LayoutOptions::default()).unwrap();
            place_straight_chain_nodes(&indexed, &[0, 1, 2], &layers, &ordinary, 30.0).unwrap()
        };

        let candidate = place_candidate(&graph);
        assert_eq!(candidate[0].y + 20.0, candidate[1].y + 20.0);
        assert_eq!(candidate[1].y + 20.0, candidate[2].y + 20.0);
        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(place_candidate(&permuted), candidate);

        let one_edge = Graph {
            edges: vec![graph.edges[0].clone()],
            ..graph
        };
        let indexed = validate_and_index(&one_edge, LayoutOptions::default()).unwrap();
        assert!(preferred_chain_targets(&indexed, &[0, 1, 2], &ordinary).is_none());
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
