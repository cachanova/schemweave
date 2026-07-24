use schemweave::{
    Graph, GroupExpansion, GroupExpansionError, GroupExpansionOptions, Layout, LayoutConfig,
    LayoutConstraints, expand_group_in_place, expand_group_in_place_with_reference_height,
};
use serde::Deserialize;

#[derive(Deserialize)]
struct CapturedExpansion {
    compact_graph: Graph,
    compact_layout: Layout,
    expanded_graph: Graph,
    expansion: GroupExpansion,
    constraints: LayoutConstraints,
}

fn captured_register_vector_expansion() -> CapturedExpansion {
    serde_json::from_str(include_str!(
        "fixtures/consumer_reg_mux_register_expansion.json"
    ))
    .expect("captured consumer expansion request is valid")
}

fn captured_mux_vector_expansion() -> CapturedExpansion {
    serde_json::from_str(include_str!("fixtures/consumer_reg_mux_mux_expansion.json"))
        .expect("captured consumer expansion request is valid")
}

fn captured_focused_register_vector_expansion() -> CapturedExpansion {
    serde_json::from_str(include_str!(
        "fixtures/consumer_reg_mux_focused_register_expansion.json"
    ))
    .expect("captured focused consumer expansion request is valid")
}

fn expand_without_a_full_relayout(captured: &CapturedExpansion) -> Layout {
    let mut config = LayoutConfig::highest_quality();
    config.constraints = captured.constraints.clone();

    let expanded = expand_group_in_place(
        &captured.compact_graph,
        &captured.compact_layout,
        &captured.expanded_graph,
        &captured.expansion,
        &GroupExpansionOptions {
            layout: config.layout,
            quality_effort: config.quality_effort,
            constraints: config.constraints,
        },
    )
    .expect("a grouped vector should preserve unrelated compact geometry");

    assert_eq!(expanded.nodes.len(), captured.expanded_graph.nodes.len());
    assert_eq!(expanded.edges.len(), captured.expanded_graph.edges.len());
    expanded
}

#[test]
fn captured_register_vector_expands_without_a_full_relayout() {
    let captured = captured_register_vector_expansion();
    let expanded = expand_without_a_full_relayout(&captured);
    let member_nodes = expanded
        .nodes
        .iter()
        .filter(|node| captured.expansion.members.contains(&node.id))
        .collect::<Vec<_>>();
    let frame_top = member_nodes
        .iter()
        .map(|node| node.y)
        .min_by(f64::total_cmp)
        .expect("expanded group contains members");
    let frame_bottom = member_nodes
        .iter()
        .map(|node| node.y + node.height)
        .max_by(f64::total_cmp)
        .expect("expanded group contains members");
    let shared_net_edges = captured
        .expanded_graph
        .edges
        .iter()
        .filter(|edge| {
            edge.net == 24
                && !captured.expansion.members.contains(&edge.source.node)
                && captured.expansion.members.contains(&edge.target.node)
        })
        .map(|edge| edge.id)
        .collect::<Vec<_>>();
    let mut shared_points = expanded
        .edges
        .iter()
        .find(|route| route.id == shared_net_edges[0])
        .expect("shared fanout route exists")
        .points
        .clone();
    for edge in shared_net_edges.iter().skip(1) {
        let route = expanded
            .edges
            .iter()
            .find(|route| route.id == *edge)
            .expect("shared fanout route exists");
        shared_points.retain(|point| route.points.contains(point));
    }
    assert!(
        shared_points
            .iter()
            .any(|point| point.y > frame_top && point.y < frame_bottom),
        "the shared fanout trunk should enter through the expanded frame, not detour around it"
    );
}

#[test]
fn captured_mux_vector_expands_without_a_full_relayout() {
    expand_without_a_full_relayout(&captured_mux_vector_expansion());
}

#[test]
fn captured_focused_register_vector_expands_without_a_full_relayout() {
    let captured = captured_focused_register_vector_expansion();
    let expanded = expand_without_a_full_relayout(&captured);
    let members = expanded
        .nodes
        .iter()
        .filter(|node| captured.expansion.members.contains(&node.id))
        .collect::<Vec<_>>();
    let columns = members
        .iter()
        .map(|node| node.x.to_bits())
        .collect::<std::collections::BTreeSet<_>>();
    let rows = members
        .iter()
        .map(|node| node.y.to_bits())
        .collect::<std::collections::BTreeSet<_>>();
    assert!(columns.len() > 1 && rows.len() > 1);

    let left = members
        .iter()
        .map(|node| node.x)
        .fold(f64::INFINITY, f64::min);
    let top = members
        .iter()
        .map(|node| node.y)
        .fold(f64::INFINITY, f64::min);
    let right = members
        .iter()
        .map(|node| node.x + node.width)
        .fold(f64::NEG_INFINITY, f64::max);
    let bottom = members
        .iter()
        .map(|node| node.y + node.height)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(expanded.nodes.iter().all(|node| {
        captured.expansion.members.contains(&node.id)
            || node.x >= right
            || node.x + node.width <= left
            || node.y >= bottom
            || node.y + node.height <= top
    }));
}

#[test]
fn focused_register_vector_keeps_its_original_grid_shape() {
    let original = captured_register_vector_expansion();
    let focused = captured_focused_register_vector_expansion();
    let original_expanded = expand_without_a_full_relayout(&original);
    let mut config = LayoutConfig::highest_quality();
    config.constraints = focused.constraints.clone();
    let focused_expanded = expand_group_in_place_with_reference_height(
        &focused.compact_graph,
        &focused.compact_layout,
        &focused.expanded_graph,
        &focused.expansion,
        original.compact_layout.height,
        &GroupExpansionOptions {
            layout: config.layout,
            quality_effort: config.quality_effort,
            constraints: config.constraints,
        },
    )
    .expect("focused expansion should retain the original arrangement decision");
    let grid_shape = |layout: &Layout, expansion: &GroupExpansion| {
        let members = layout
            .nodes
            .iter()
            .filter(|node| expansion.members.contains(&node.id))
            .collect::<Vec<_>>();
        let columns = members
            .iter()
            .map(|node| node.x.to_bits())
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        let rows = members
            .iter()
            .map(|node| node.y.to_bits())
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        (columns, rows)
    };

    assert_eq!(
        grid_shape(&focused_expanded, &focused.expansion),
        grid_shape(&original_expanded, &original.expansion),
    );
}

#[test]
fn expansion_rejects_an_invalid_reference_height() {
    let captured = captured_focused_register_vector_expansion();
    let mut config = LayoutConfig::highest_quality();
    config.constraints = captured.constraints.clone();
    let error = expand_group_in_place_with_reference_height(
        &captured.compact_graph,
        &captured.compact_layout,
        &captured.expanded_graph,
        &captured.expansion,
        0.0,
        &GroupExpansionOptions {
            layout: config.layout,
            quality_effort: config.quality_effort,
            constraints: config.constraints,
        },
    )
    .expect_err("a non-positive reference height must be rejected");

    assert_eq!(error, GroupExpansionError::InvalidReferenceHeight(0.0));
}
