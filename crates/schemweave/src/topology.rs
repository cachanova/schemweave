use std::{
    cmp::Reverse,
    collections::{BTreeMap, BinaryHeap},
};

use crate::validation::IndexedGraph;

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
    let real_count = graph.nodes.len();
    let mut ordering = expanded_ordering_graph(graph, ranks);
    let mut positions = vec![0usize; ordering.stable_keys.len()];
    for layer in &mut ordering.layers {
        layer.sort_unstable_by_key(|&item| ordering.stable_keys[item]);
    }
    let mut best_layers = ordering.layers.clone();
    let mut best_crossings = crossing_count(&ordering.layers, &ordering.outgoing, &mut positions);
    let mut layers = ordering.layers;
    refresh_positions(&layers, &mut positions);
    for _ in 0..sweeps {
        for layer in layers.iter_mut().skip(1) {
            sort_layer(layer, &ordering.stable_keys, &positions, &ordering.incoming);
            refresh_layer(layer, &mut positions);
        }
        transpose_layers(
            &mut layers,
            &ordering.incoming,
            &ordering.outgoing,
            &mut positions,
        );
        retain_best(
            &layers,
            &ordering.outgoing,
            &mut positions,
            &mut best_layers,
            &mut best_crossings,
        );
        let reverse_count = layers.len().saturating_sub(1);
        for layer in layers.iter_mut().take(reverse_count).rev() {
            sort_layer(layer, &ordering.stable_keys, &positions, &ordering.outgoing);
            refresh_layer(layer, &mut positions);
        }
        transpose_layers(
            &mut layers,
            &ordering.incoming,
            &ordering.outgoing,
            &mut positions,
        );
        retain_best(
            &layers,
            &ordering.outgoing,
            &mut positions,
            &mut best_layers,
            &mut best_crossings,
        );
    }
    for layer in &mut best_layers {
        layer.retain(|&item| item < real_count);
    }
    best_layers
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
) {
    layer.sort_by(|&left, &right| {
        barycenter(left, positions, neighbors)
            .total_cmp(&barycenter(right, positions, neighbors))
            .then_with(|| stable_keys[left].cmp(&stable_keys[right]))
    });
}

struct OrderingGraph {
    layers: Vec<Vec<usize>>,
    incoming: Vec<Vec<usize>>,
    outgoing: Vec<Vec<usize>>,
    stable_keys: Vec<(u8, u32, u32)>,
}

fn expanded_ordering_graph(graph: &IndexedGraph<'_>, ranks: &[usize]) -> OrderingGraph {
    let mut layers = vec![Vec::new(); ranks.iter().copied().max().unwrap_or(0) + 1];
    let mut stable_keys = Vec::with_capacity(graph.nodes.len());
    for (node, &rank) in ranks.iter().enumerate() {
        layers[rank].push(node);
        stable_keys.push((0, graph.nodes[node].id, 0));
    }
    let mut incoming = vec![Vec::new(); graph.nodes.len()];
    let mut outgoing = vec![Vec::new(); graph.nodes.len()];
    let mut virtual_items = BTreeMap::new();

    for edge in &graph.edges {
        if !edge.participates_in_ranking {
            continue;
        }
        let source = graph.node_index[&edge.source.node];
        let target = graph.node_index[&edge.target.node];
        if graph.nodes[target].cycle_breaker || ranks[source] >= ranks[target] {
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
                layer.push(item);
                item
            };
            outgoing[previous].push(item);
            incoming[item].push(previous);
            previous = item;
        }
        outgoing[previous].push(target);
        incoming[target].push(previous);
    }
    for neighbors in incoming.iter_mut().chain(&mut outgoing) {
        neighbors.sort_unstable();
        neighbors.dedup();
    }
    OrderingGraph {
        layers,
        incoming,
        outgoing,
        stable_keys,
    }
}

fn retain_best(
    layers: &[Vec<usize>],
    outgoing: &[Vec<usize>],
    positions: &mut [usize],
    best_layers: &mut [Vec<usize>],
    best_crossings: &mut usize,
) {
    let crossings = crossing_count(layers, outgoing, positions);
    if crossings < *best_crossings {
        *best_crossings = crossings;
        best_layers.clone_from_slice(layers);
    }
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

    use super::expanded_ordering_graph;

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
        let ordering = expanded_ordering_graph(&indexed, &[0, 2, 3]);

        assert_eq!(ordering.stable_keys.len(), 5);
        assert!(ordering.stable_keys.contains(&(1, 7, 1)));
        assert!(ordering.stable_keys.contains(&(1, 7, 2)));
        assert_eq!(ordering.outgoing[0].len(), 1);
    }
}
