use std::{cmp::Reverse, collections::BinaryHeap};

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
    let mut layers = vec![Vec::new(); ranks.iter().copied().max().unwrap_or(0) + 1];
    for (node, &rank) in ranks.iter().enumerate() {
        layers[rank].push(node);
    }
    for layer in &mut layers {
        layer.sort_unstable_by_key(|&node| graph.nodes[node].id);
    }
    let mut positions = vec![0usize; graph.nodes.len()];
    refresh_positions(&layers, &mut positions);
    for _ in 0..sweeps {
        for layer in layers.iter_mut().skip(1) {
            sort_layer(layer, graph, &positions, &graph.incoming);
            refresh_layer(layer, &mut positions);
        }
        let reverse_count = layers.len().saturating_sub(1);
        for layer in layers.iter_mut().take(reverse_count).rev() {
            sort_layer(layer, graph, &positions, &graph.outgoing);
            refresh_layer(layer, &mut positions);
        }
    }
    layers
}

fn sort_layer(
    layer: &mut [usize],
    graph: &IndexedGraph<'_>,
    positions: &[usize],
    neighbors: &[Vec<usize>],
) {
    layer.sort_by(|&left, &right| {
        barycenter(left, positions, neighbors)
            .total_cmp(&barycenter(right, positions, neighbors))
            .then_with(|| graph.nodes[left].id.cmp(&graph.nodes[right].id))
    });
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
