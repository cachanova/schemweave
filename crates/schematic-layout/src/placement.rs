use crate::{EdgeGeometry, Layout, LayoutOptions, NodeGeometry, validation::IndexedGraph};

pub(crate) fn place_nodes(
    graph: &IndexedGraph<'_>,
    _ranks: &[usize],
    layers: &[Vec<usize>],
    options: LayoutOptions,
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
    positioned.into_iter().map(Option::unwrap).collect()
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
