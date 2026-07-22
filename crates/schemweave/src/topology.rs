use std::{
    cmp::Reverse,
    collections::{BTreeMap, BinaryHeap},
};

use crate::{NetId, validation::IndexedGraph};

#[derive(Clone, Copy)]
enum CrossingScore {
    Edge,
    NetRepresentative,
}

#[derive(Clone, Copy)]
enum NeighborMeasure {
    Mean,
    Median,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AlternativeOrdering {
    NetRepresentative,
    ReverseMedian,
}

#[derive(Clone, Copy, Eq, Ord, PartialEq, PartialOrd)]
struct OrderingArc {
    neighbor: usize,
    net: NetId,
}

pub(crate) fn assign_ranks(graph: &IndexedGraph<'_>) -> Vec<usize> {
    let (component, component_count) = strongly_connected_components(graph);
    let mut component_out = vec![Vec::new(); component_count];
    let mut indegree = vec![0usize; component_count];
    for (source, targets) in graph.outgoing.iter().enumerate() {
        for &target in targets {
            let from = component[source];
            let to = component[target];
            if from != to {
                component_out[from].push(to);
            }
        }
    }
    for targets in &mut component_out {
        targets.sort_unstable();
        targets.dedup();
        for &target in targets.iter() {
            indegree[target] += 1;
        }
    }

    let mut component_key = vec![u32::MAX; component_count];
    for (node, item) in graph.nodes.iter().enumerate() {
        component_key[component[node]] = component_key[component[node]].min(item.id);
    }
    let mut ready = BinaryHeap::new();
    for (id, &degree) in indegree.iter().enumerate() {
        if degree == 0 {
            ready.push(Reverse((component_key[id], id)));
        }
    }
    let mut component_rank = vec![0usize; component_count];
    while let Some(Reverse((_, current))) = ready.pop() {
        for &next in &component_out[current] {
            component_rank[next] = component_rank[next].max(component_rank[current] + 1);
            indegree[next] -= 1;
            if indegree[next] == 0 {
                ready.push(Reverse((component_key[next], next)));
            }
        }
    }
    component.into_iter().map(|id| component_rank[id]).collect()
}

fn strongly_connected_components(graph: &IndexedGraph<'_>) -> (Vec<usize>, usize) {
    let count = graph.nodes.len();
    let mut seen = vec![false; count];
    let mut finish = Vec::with_capacity(count);
    for start in 0..count {
        if seen[start] {
            continue;
        }
        seen[start] = true;
        let mut stack = vec![(start, 0usize)];
        while let Some((node, cursor)) = stack.last_mut() {
            if *cursor < graph.outgoing[*node].len() {
                let next = graph.outgoing[*node][*cursor];
                *cursor += 1;
                if !seen[next] {
                    seen[next] = true;
                    stack.push((next, 0));
                }
            } else {
                finish.push(*node);
                stack.pop();
            }
        }
    }

    let mut component = vec![usize::MAX; count];
    let mut component_count = 0;
    for &start in finish.iter().rev() {
        if component[start] != usize::MAX {
            continue;
        }
        component[start] = component_count;
        let mut stack = vec![start];
        while let Some(node) = stack.pop() {
            for &next in &graph.incoming[node] {
                if component[next] == usize::MAX {
                    component[next] = component_count;
                    stack.push(next);
                }
            }
        }
        component_count += 1;
    }
    (component, component_count)
}

pub(crate) fn order_layers(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    sweeps: usize,
) -> Vec<Vec<usize>> {
    let (forward, reverse, _) = order_layer_candidates(graph, ranks, sweeps, false);
    if reverse.crossings < forward.crossings {
        reverse.layers
    } else {
        forward.layers
    }
}

pub(crate) struct LayerOrdering {
    pub(crate) layers: Vec<Vec<usize>>,
    pub(crate) crossings: usize,
}

struct OptimizedSeed {
    edge_layers: Vec<Vec<usize>>,
    edge_crossings: usize,
    alternative: Option<ScoredOrdering>,
}

struct ScoredOrdering {
    layers: Vec<Vec<usize>>,
    crossings: usize,
    edge_crossings: usize,
}

pub(crate) fn order_layer_candidates(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    sweeps: usize,
    include_alternative: bool,
) -> (LayerOrdering, LayerOrdering, Option<LayerOrdering>) {
    let real_count = graph.nodes.len();
    let alternative = include_alternative
        .then(|| alternative_ordering_candidate(graph))
        .flatten();
    let track_net_representative =
        matches!(alternative, Some(AlternativeOrdering::NetRepresentative));
    let ordering = expanded_ordering_graph(graph, ranks, track_net_representative);
    let forward_score = track_net_representative.then_some(CrossingScore::NetRepresentative);
    let forward = optimize_ordering_seed(
        &ordering,
        sweeps,
        false,
        forward_score,
        NeighborMeasure::Mean,
    );
    let reverse = optimize_ordering_seed(&ordering, sweeps, true, None, NeighborMeasure::Mean);
    let reverse_median =
        matches!(alternative, Some(AlternativeOrdering::ReverseMedian)).then(|| {
            let median = optimize_ordering_seed(
                &ordering,
                sweeps.min(2),
                true,
                None,
                NeighborMeasure::Median,
            );
            ScoredOrdering {
                layers: median.edge_layers,
                crossings: median.edge_crossings,
                edge_crossings: median.edge_crossings,
            }
        });
    let strip_virtual = |mut layers: Vec<Vec<usize>>| {
        for layer in &mut layers {
            layer.retain(|&item| item < real_count);
        }
        layers
    };
    (
        LayerOrdering {
            layers: strip_virtual(forward.edge_layers),
            crossings: forward.edge_crossings,
        },
        LayerOrdering {
            layers: strip_virtual(reverse.edge_layers),
            crossings: reverse.edge_crossings,
        },
        forward
            .alternative
            .or(reverse_median)
            .map(|ordering| LayerOrdering {
                layers: strip_virtual(ordering.layers),
                crossings: ordering.crossings,
            }),
    )
}

fn alternative_ordering_candidate(graph: &IndexedGraph<'_>) -> Option<AlternativeOrdering> {
    let mut fanout = BTreeMap::<NetId, usize>::new();
    for (edge, &participates_in_ranking) in graph.edges.iter().zip(&graph.rank_edges) {
        if participates_in_ranking {
            *fanout.entry(edge.net).or_default() += 1;
        }
    }
    let mut largest_fanout = 0;
    let mut second_largest_fanout = 0;
    for &count in fanout.values() {
        if count > largest_fanout {
            second_largest_fanout = largest_fanout;
            largest_fanout = count;
        } else if count > second_largest_fanout {
            second_largest_fanout = count;
        }
    }
    if (16..=64).contains(&largest_fanout) {
        return Some(AlternativeOrdering::NetRepresentative);
    }
    if second_largest_fanout < 65 {
        return None;
    }
    let mut sinks_by_net = BTreeMap::<NetId, Vec<(u32, u32)>>::new();
    for (edge, &participates_in_ranking) in graph.edges.iter().zip(&graph.rank_edges) {
        if participates_in_ranking && fanout[&edge.net] >= 65 {
            sinks_by_net
                .entry(edge.net)
                .or_default()
                .push((edge.target.node, edge.target.port));
        }
    }
    let mut largest_distinct_fanout = 0;
    let mut second_largest_distinct_fanout = 0;
    for sinks in sinks_by_net.values_mut() {
        sinks.sort_unstable();
        sinks.dedup();
        let distinct_fanout = sinks.len();
        if distinct_fanout > largest_distinct_fanout {
            second_largest_distinct_fanout = largest_distinct_fanout;
            largest_distinct_fanout = distinct_fanout;
        } else if distinct_fanout > second_largest_distinct_fanout {
            second_largest_distinct_fanout = distinct_fanout;
        }
    }
    ((65..=100).contains(&second_largest_distinct_fanout)
        || (graph.nodes.len() >= 1_000 && (101..=300).contains(&second_largest_distinct_fanout)))
    .then_some(AlternativeOrdering::ReverseMedian)
}

fn optimize_ordering_seed(
    ordering: &OrderingGraph,
    sweeps: usize,
    reverse: bool,
    alternative_score: Option<CrossingScore>,
    neighbor_measure: NeighborMeasure,
) -> OptimizedSeed {
    let mut positions = vec![0usize; ordering.stable_keys.len()];
    let mut layers = ordering.layers.clone();
    for layer in &mut layers {
        if reverse {
            layer.sort_unstable_by_key(|&item| Reverse(ordering.stable_keys[item]));
        } else {
            layer.sort_unstable_by_key(|&item| ordering.stable_keys[item]);
        }
    }
    let mut edge_layers = layers.clone();
    let mut edge_crossings = crossing_score(&layers, ordering, &mut positions, CrossingScore::Edge);
    let mut alternative = alternative_score.map(|score| ScoredOrdering {
        layers: layers.clone(),
        crossings: crossing_score(&layers, ordering, &mut positions, score),
        edge_crossings,
    });
    refresh_positions(&layers, &mut positions);
    let mut ordering_scores = if sweeps == 0 {
        Vec::new()
    } else {
        vec![0.0; ordering.stable_keys.len()]
    };
    let mut median_scratch = Vec::new();
    for _ in 0..sweeps {
        for layer in layers.iter_mut().skip(1) {
            sort_layer(
                layer,
                &ordering.stable_keys,
                &positions,
                &ordering.incoming,
                &mut ordering_scores,
                neighbor_measure,
                &mut median_scratch,
            );
            refresh_layer(layer, &mut positions);
        }
        transpose_layers(
            &mut layers,
            &ordering.incoming,
            &ordering.outgoing,
            &mut positions,
        );
        let current_edge_crossings = retain_best_edge(
            &layers,
            ordering,
            &mut positions,
            &mut edge_layers,
            &mut edge_crossings,
        );
        if let (Some(score), Some(best)) = (alternative_score, &mut alternative) {
            retain_best_alternative(
                &layers,
                ordering,
                &mut positions,
                current_edge_crossings,
                score,
                best,
            );
        }
        let reverse_count = layers.len().saturating_sub(1);
        for layer in layers.iter_mut().take(reverse_count).rev() {
            sort_layer(
                layer,
                &ordering.stable_keys,
                &positions,
                &ordering.outgoing,
                &mut ordering_scores,
                neighbor_measure,
                &mut median_scratch,
            );
            refresh_layer(layer, &mut positions);
        }
        transpose_layers(
            &mut layers,
            &ordering.incoming,
            &ordering.outgoing,
            &mut positions,
        );
        let current_edge_crossings = retain_best_edge(
            &layers,
            ordering,
            &mut positions,
            &mut edge_layers,
            &mut edge_crossings,
        );
        if let (Some(score), Some(best)) = (alternative_score, &mut alternative) {
            retain_best_alternative(
                &layers,
                ordering,
                &mut positions,
                current_edge_crossings,
                score,
                best,
            );
        }
    }
    OptimizedSeed {
        edge_layers,
        edge_crossings,
        alternative,
    }
}

fn transpose_layers(
    layers: &mut [Vec<usize>],
    incoming: &[Vec<usize>],
    outgoing: &[Vec<usize>],
    positions: &mut [usize],
) {
    for _ in 0..4 {
        let mut changed = false;
        for layer in layers.iter_mut() {
            for index in 0..layer.len().saturating_sub(1) {
                let left = layer[index];
                let right = layer[index + 1];
                let gain = pair_crossing_gain(&incoming[left], &incoming[right], positions)
                    + pair_crossing_gain(&outgoing[left], &outgoing[right], positions);
                if gain > 0 {
                    layer.swap(index, index + 1);
                    positions[left] = index + 1;
                    positions[right] = index;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
}

fn pair_crossing_gain(left: &[usize], right: &[usize], positions: &[usize]) -> isize {
    let mut gain = 0isize;
    for &left_neighbor in left {
        for &right_neighbor in right {
            gain += match positions[left_neighbor].cmp(&positions[right_neighbor]) {
                std::cmp::Ordering::Greater => 1,
                std::cmp::Ordering::Less => -1,
                std::cmp::Ordering::Equal => 0,
            };
        }
    }
    gain
}

fn sort_layer(
    layer: &mut [usize],
    stable_keys: &[(u8, u32, u32)],
    positions: &[usize],
    neighbors: &[Vec<usize>],
    scores: &mut [f64],
    neighbor_measure: NeighborMeasure,
    median_scratch: &mut Vec<usize>,
) {
    if layer.len() < 2 {
        return;
    }
    for &item in layer.iter() {
        scores[item] = match neighbor_measure {
            NeighborMeasure::Mean => barycenter(item, positions, neighbors),
            NeighborMeasure::Median => median(item, positions, neighbors, median_scratch),
        };
    }
    layer.sort_by(|&left, &right| {
        scores[left]
            .total_cmp(&scores[right])
            .then_with(|| stable_keys[left].cmp(&stable_keys[right]))
    });
}

struct OrderingGraph {
    layers: Vec<Vec<usize>>,
    incoming: Vec<Vec<usize>>,
    outgoing: Vec<Vec<usize>>,
    outgoing_arcs: Option<Vec<Vec<OrderingArc>>>,
    stable_keys: Vec<(u8, u32, u32)>,
}

fn expanded_ordering_graph(
    graph: &IndexedGraph<'_>,
    ranks: &[usize],
    track_net_representative: bool,
) -> OrderingGraph {
    let mut layers = vec![Vec::new(); ranks.iter().copied().max().unwrap_or(0) + 1];
    let mut stable_keys = Vec::with_capacity(graph.nodes.len());
    for (node, &rank) in ranks.iter().enumerate() {
        layers[rank].push(node);
        stable_keys.push((0, graph.nodes[node].id, 0));
    }
    let mut incoming = vec![Vec::new(); graph.nodes.len()];
    let mut outgoing = vec![Vec::new(); graph.nodes.len()];
    let mut outgoing_arcs = track_net_representative.then(|| vec![Vec::new(); graph.nodes.len()]);
    let mut virtual_items = BTreeMap::new();

    for (edge, &rank_edge) in graph.edges.iter().zip(&graph.rank_edges) {
        if !rank_edge {
            continue;
        }
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        if ranks[source] >= ranks[target] {
            continue;
        }
        let mut previous = source;
        for (rank, layer) in layers
            .iter_mut()
            .enumerate()
            .take(ranks[target])
            .skip(ranks[source] + 1)
        {
            let item = if let Some(&item) = virtual_items.get(&(rank, edge.net)) {
                item
            } else {
                let item = stable_keys.len();
                virtual_items.insert((rank, edge.net), item);
                stable_keys.push((1, edge.net, rank as u32));
                incoming.push(Vec::new());
                outgoing.push(Vec::new());
                if let Some(outgoing_arcs) = &mut outgoing_arcs {
                    outgoing_arcs.push(Vec::new());
                }
                layer.push(item);
                item
            };
            outgoing[previous].push(item);
            if let Some(outgoing_arcs) = &mut outgoing_arcs {
                outgoing_arcs[previous].push(OrderingArc {
                    neighbor: item,
                    net: edge.net,
                });
            }
            incoming[item].push(previous);
            previous = item;
        }
        outgoing[previous].push(target);
        if let Some(outgoing_arcs) = &mut outgoing_arcs {
            outgoing_arcs[previous].push(OrderingArc {
                neighbor: target,
                net: edge.net,
            });
        }
        incoming[target].push(previous);
    }
    for neighbors in incoming.iter_mut().chain(&mut outgoing) {
        neighbors.sort_unstable();
        neighbors.dedup();
    }
    if let Some(outgoing_arcs) = &mut outgoing_arcs {
        for arcs in outgoing_arcs {
            arcs.sort_unstable();
            arcs.dedup();
        }
    }
    OrderingGraph {
        layers,
        incoming,
        outgoing,
        outgoing_arcs,
        stable_keys,
    }
}

fn retain_best_edge(
    layers: &[Vec<usize>],
    ordering: &OrderingGraph,
    positions: &mut [usize],
    best_layers: &mut [Vec<usize>],
    best_crossings: &mut usize,
) -> usize {
    let crossings = crossing_score(layers, ordering, positions, CrossingScore::Edge);
    if crossings < *best_crossings {
        *best_crossings = crossings;
        best_layers.clone_from_slice(layers);
    }
    crossings
}

fn retain_best_alternative(
    layers: &[Vec<usize>],
    ordering: &OrderingGraph,
    positions: &mut [usize],
    edge_crossings: usize,
    score: CrossingScore,
    best: &mut ScoredOrdering,
) {
    let score = (
        crossing_score(layers, ordering, positions, score),
        edge_crossings,
    );
    if score < (best.crossings, best.edge_crossings) {
        best.crossings = score.0;
        best.edge_crossings = score.1;
        best.layers.clone_from_slice(layers);
    }
}

fn crossing_score(
    layers: &[Vec<usize>],
    ordering: &OrderingGraph,
    positions: &mut [usize],
    score: CrossingScore,
) -> usize {
    match score {
        CrossingScore::Edge => crossing_count(layers, &ordering.outgoing, positions),
        CrossingScore::NetRepresentative => net_representative_crossing_count(
            layers,
            ordering
                .outgoing_arcs
                .as_deref()
                .expect("net representative score requires net arcs"),
            positions,
        ),
    }
}

fn net_representative_crossing_count(
    layers: &[Vec<usize>],
    outgoing: &[Vec<OrderingArc>],
    positions: &mut [usize],
) -> usize {
    refresh_positions(layers, positions);
    let mut crossings = 0usize;
    for layer in layers.iter().take(layers.len().saturating_sub(1)) {
        let mut by_net = BTreeMap::<NetId, (Vec<usize>, Vec<usize>)>::new();
        for &source in layer {
            for arc in &outgoing[source] {
                let endpoints = by_net.entry(arc.net).or_default();
                endpoints.0.push(positions[source]);
                endpoints.1.push(positions[arc.neighbor]);
            }
        }
        let mut connections = Vec::with_capacity(by_net.len());
        for (net, (mut sources, mut targets)) in by_net {
            sources.sort_unstable();
            sources.dedup();
            targets.sort_unstable();
            targets.dedup();
            connections.push((sources[sources.len() / 2], targets[targets.len() / 2], net));
        }
        connections.sort_unstable();
        let target_count = connections
            .iter()
            .map(|&(_, target, _)| target)
            .max()
            .map_or(0, |target| target + 1);
        let mut tree = Fenwick::new(target_count);
        for (seen, (_, target, _)) in connections.into_iter().enumerate() {
            crossings += seen - tree.prefix(target + 1);
            tree.add(target);
        }
    }
    crossings
}

fn crossing_count(
    layers: &[Vec<usize>],
    outgoing: &[Vec<usize>],
    positions: &mut [usize],
) -> usize {
    refresh_positions(layers, positions);
    let mut crossings = 0usize;
    for layer in layers.iter().take(layers.len().saturating_sub(1)) {
        let mut connections = Vec::new();
        for &source in layer {
            for &target in &outgoing[source] {
                connections.push((positions[source], positions[target]));
            }
        }
        connections.sort_unstable();
        let target_count = connections
            .iter()
            .map(|&(_, target)| target)
            .max()
            .map_or(0, |target| target + 1);
        let mut tree = Fenwick::new(target_count);
        for (seen, (_, target)) in connections.into_iter().enumerate() {
            crossings += seen - tree.prefix(target + 1);
            tree.add(target);
        }
    }
    crossings
}

struct Fenwick {
    values: Vec<usize>,
}

impl Fenwick {
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

fn barycenter(node: usize, positions: &[usize], neighbors: &[Vec<usize>]) -> f64 {
    let adjacent = &neighbors[node];
    if adjacent.is_empty() {
        return positions[node] as f64;
    }
    adjacent
        .iter()
        .map(|&item| positions[item] as f64)
        .sum::<f64>()
        / adjacent.len() as f64
}

fn median(
    node: usize,
    positions: &[usize],
    neighbors: &[Vec<usize>],
    scratch: &mut Vec<usize>,
) -> f64 {
    let adjacent = &neighbors[node];
    if adjacent.is_empty() {
        return positions[node] as f64;
    }
    scratch.clear();
    scratch.extend(adjacent.iter().map(|&item| positions[item]));
    scratch.sort_unstable();
    let middle = scratch.len() / 2;
    if scratch.len().is_multiple_of(2) {
        (scratch[middle - 1] + scratch[middle]) as f64 / 2.0
    } else {
        scratch[middle] as f64
    }
}

fn refresh_positions(layers: &[Vec<usize>], positions: &mut [usize]) {
    for layer in layers {
        refresh_layer(layer, positions);
    }
}

fn refresh_layer(layer: &[usize], positions: &mut [usize]) {
    for (position, &node) in layer.iter().enumerate() {
        positions[node] = position;
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        Edge, Endpoint, Graph, LayoutOptions, Node, Port, PortSide, validation::validate_and_index,
    };

    use super::{
        AlternativeOrdering, NeighborMeasure, alternative_ordering_candidate, assign_ranks,
        crossing_count, expanded_ordering_graph, median, net_representative_crossing_count,
        optimize_ordering_seed, order_layers,
    };

    fn node(id: u32) -> Node {
        Node {
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
        }
    }

    #[test]
    fn long_edges_share_one_virtual_item_per_net_and_rank() {
        let graph = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: vec![
                Edge {
                    id: 1,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 2,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 7,
                    participates_in_ranking: true,
                },
            ],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let ordering = expanded_ordering_graph(&indexed, &[0, 2, 3], false);

        assert_eq!(ordering.stable_keys.len(), 5);
        assert!(ordering.stable_keys.contains(&(1, 7, 1)));
        assert!(ordering.stable_keys.contains(&(1, 7, 2)));
        assert_eq!(ordering.outgoing[0].len(), 1);
        assert!(ordering.outgoing_arcs.is_none());
    }

    #[test]
    fn root_to_cycle_breaker_edge_participates_in_ordering() {
        let mut register = node(2);
        register.cycle_breaker = true;
        let graph = Graph {
            nodes: vec![node(1), register],
            edges: vec![Edge {
                id: 1,
                source: Endpoint { node: 1, port: 1 },
                target: Endpoint { node: 2, port: 0 },
                net: 7,
                participates_in_ranking: true,
            }],
        };
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let ranks = assign_ranks(&indexed);
        let ordering = expanded_ordering_graph(&indexed, &ranks, false);

        assert_eq!(ranks, vec![0, 1]);
        assert_eq!(ordering.outgoing[0], vec![1]);
        assert_eq!(ordering.incoming[1], vec![0]);
    }

    #[test]
    fn alternative_candidate_is_bounded_to_measured_fanout_ranges() {
        let candidate = |primary_edges: u32,
                         primary_sinks: u32,
                         secondary_edges: u32,
                         secondary_sinks: u32,
                         node_count: u32,
                         participates_in_ranking| {
            let primary_sinks = primary_sinks.max(1);
            let secondary_sinks = secondary_sinks.max(1);
            let graph = Graph {
                nodes: (0..node_count.max(primary_sinks.max(secondary_sinks) + 2))
                    .map(node)
                    .collect(),
                edges: (0..primary_edges)
                    .map(|id| Edge {
                        id,
                        source: Endpoint { node: 0, port: 1 },
                        target: Endpoint {
                            node: 2 + id % primary_sinks,
                            port: 0,
                        },
                        net: 7,
                        participates_in_ranking,
                    })
                    .chain((0..secondary_edges).map(|id| Edge {
                        id: primary_edges + id,
                        source: Endpoint { node: 1, port: 1 },
                        target: Endpoint {
                            node: 2 + id % secondary_sinks,
                            port: 0,
                        },
                        net: 8,
                        participates_in_ranking,
                    }))
                    .collect(),
            };
            let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
            alternative_ordering_candidate(&indexed)
        };

        assert_eq!(candidate(0, 1, 0, 1, 2, true), None);
        assert_eq!(candidate(15, 15, 15, 15, 17, true), None);
        assert_eq!(
            candidate(16, 1, 1, 1, 3, true),
            Some(AlternativeOrdering::NetRepresentative)
        );
        assert_eq!(
            candidate(64, 1, 1, 1, 3, true),
            Some(AlternativeOrdering::NetRepresentative)
        );
        assert_eq!(candidate(65, 1, 65, 1, 3, true), None);
        assert_eq!(
            candidate(269, 269, 65, 65, 271, true),
            Some(AlternativeOrdering::ReverseMedian)
        );
        assert_eq!(
            candidate(269, 269, 100, 100, 271, true),
            Some(AlternativeOrdering::ReverseMedian)
        );
        assert_eq!(candidate(269, 269, 101, 101, 271, true), None);
        assert_eq!(
            candidate(301, 301, 101, 101, 1_000, true),
            Some(AlternativeOrdering::ReverseMedian)
        );
        assert_eq!(
            candidate(301, 301, 300, 300, 1_000, true),
            Some(AlternativeOrdering::ReverseMedian)
        );
        assert_eq!(candidate(302, 302, 301, 301, 1_000, true), None);
        assert_eq!(candidate(269, 269, 65, 65, 271, false), None);

        let mixed = Graph {
            nodes: vec![node(0), node(1)],
            edges: (0..16)
                .map(|id| Edge {
                    id,
                    source: Endpoint { node: 0, port: 1 },
                    target: Endpoint { node: 1, port: 0 },
                    net: 7,
                    participates_in_ranking: id < 15,
                })
                .collect(),
        };
        let indexed = validate_and_index(&mixed, LayoutOptions::default()).unwrap();
        assert_eq!(alternative_ordering_candidate(&indexed), None);
    }

    #[test]
    fn median_neighbor_measure_handles_odd_even_and_isolated_items() {
        let positions = [0, 1, 9, 3, 7];
        let mut neighbors = vec![Vec::new(); positions.len()];
        let mut scratch = Vec::new();

        neighbors[0] = vec![1, 2, 3];
        assert_eq!(median(0, &positions, &neighbors, &mut scratch), 3.0);

        neighbors[0].push(4);
        assert_eq!(median(0, &positions, &neighbors, &mut scratch), 5.0);

        neighbors[0].clear();
        assert_eq!(median(0, &positions, &neighbors, &mut scratch), 0.0);
    }

    #[test]
    fn selects_the_better_of_forward_and_reverse_stable_seeds() {
        let endpoints = [
            (0, 4),
            (1, 4),
            (1, 6),
            (1, 7),
            (4, 8),
            (4, 9),
            (4, 11),
            (5, 8),
            (5, 10),
            (6, 9),
        ];
        let graph = Graph {
            nodes: (0..12).map(node).collect(),
            edges: endpoints
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
                .collect(),
        };
        let ranks = [0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2];
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let ordering = expanded_ordering_graph(&indexed, &ranks, false);
        let forward = optimize_ordering_seed(&ordering, 4, false, None, NeighborMeasure::Mean);
        let reverse = optimize_ordering_seed(&ordering, 4, true, None, NeighborMeasure::Mean);

        assert_eq!(forward.edge_crossings, 2);
        assert_eq!(reverse.edge_crossings, 0);
        assert_eq!(order_layers(&indexed, &ranks, 4), reverse.edge_layers);
    }

    #[test]
    fn net_representative_score_does_not_multiply_shared_branch_crossings() {
        let mut graph = Graph {
            nodes: (0..10).map(node).collect(),
            edges: vec![
                (0, 0, 4, 100),
                (1, 0, 5, 100),
                (2, 0, 6, 100),
                (3, 1, 7, 1),
                (4, 2, 8, 2),
                (5, 3, 9, 3),
            ]
            .into_iter()
            .map(|(id, source, target, net)| Edge {
                id,
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
            .collect(),
        };
        graph.edges.extend((6..19).map(|id| Edge {
            id,
            source: Endpoint { node: 0, port: 1 },
            target: Endpoint { node: 4, port: 0 },
            net: 100,
            participates_in_ranking: true,
        }));
        let ranks = [0, 0, 0, 0, 1, 1, 1, 1, 1, 1];
        let indexed = validate_and_index(&graph, LayoutOptions::default()).unwrap();
        let ordering = expanded_ordering_graph(&indexed, &ranks, true);
        let branch_weighted = vec![vec![0, 1, 2, 3], vec![4, 5, 6, 8, 9, 7]];
        let net_representative_better = vec![vec![0, 1, 2, 3], vec![7, 4, 5, 6, 8, 9]];
        let mut positions = vec![0; graph.nodes.len()];

        assert_eq!(
            crossing_count(&branch_weighted, &ordering.outgoing, &mut positions),
            2
        );
        assert_eq!(
            crossing_count(
                &net_representative_better,
                &ordering.outgoing,
                &mut positions
            ),
            3
        );
        assert_eq!(
            net_representative_crossing_count(
                &branch_weighted,
                ordering.outgoing_arcs.as_deref().unwrap(),
                &mut positions
            ),
            2
        );
        assert_eq!(
            net_representative_crossing_count(
                &net_representative_better,
                ordering.outgoing_arcs.as_deref().unwrap(),
                &mut positions
            ),
            1
        );
    }
}
