use schemweave::{
    BoundaryBundleConstraint, BoundaryBundleMemberConstraint, Graph, GroupCollapseOptions,
    GroupExpansion, GroupExpansionError, GroupExpansionOptions, Layout, LayoutConfig,
    LayoutConstraints, collapse_group_in_place, expand_group_in_place,
    expand_group_in_place_with_reference_height,
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

#[derive(Clone, Deserialize)]
struct CapturedCollapse {
    expanded_graph: Graph,
    expanded_layout: Layout,
    compact_graph: Graph,
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

fn captured_pipe_middle_group_collapse() -> CapturedCollapse {
    serde_json::from_str(include_str!(
        "fixtures/consumer_pipe_middle_group_collapse.json"
    ))
    .expect("captured middle-group collapse request is valid")
}

fn collapse_captured_group(captured: &CapturedCollapse) -> Result<Layout, GroupExpansionError> {
    let mut config = LayoutConfig::highest_quality();
    config.constraints = captured.constraints.clone();
    collapse_group_in_place(
        &captured.expanded_graph,
        &captured.expanded_layout,
        &captured.compact_graph,
        &captured.expansion,
        &GroupCollapseOptions {
            layout: config.layout,
            constraints: config.constraints,
        },
    )
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

fn constraints_from_layout(layout: &Layout, source: &LayoutConstraints) -> LayoutConstraints {
    let inputs = source.inputs.clone();
    let outputs = source.outputs.clone();
    let boundary_bundles = layout
        .boundary_bundles
        .iter()
        .map(|bundle| BoundaryBundleConstraint {
            id: bundle.id,
            endpoint: bundle.endpoint,
            width: bundle.width,
            members: bundle
                .members
                .iter()
                .map(|member| BoundaryBundleMemberConstraint {
                    edge: member.edge,
                    slots: member.slots.clone(),
                })
                .collect(),
        })
        .collect();
    LayoutConstraints {
        inputs,
        outputs,
        boundary_bundles,
    }
}

fn collapse_without_a_full_relayout(captured: &CapturedExpansion, expanded: &Layout) -> Layout {
    let mut config = LayoutConfig::highest_quality();
    config.constraints = constraints_from_layout(&captured.compact_layout, &captured.constraints);
    collapse_group_in_place(
        &captured.expanded_graph,
        expanded,
        &captured.compact_graph,
        &captured.expansion,
        &GroupCollapseOptions {
            layout: config.layout,
            constraints: config.constraints,
        },
    )
    .expect("a grouped vector should collapse without moving unrelated nodes")
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
fn captured_register_vector_collapses_without_moving_unrelated_nodes() {
    let captured = captured_register_vector_expansion();
    let expanded = expand_without_a_full_relayout(&captured);
    let collapsed = collapse_without_a_full_relayout(&captured, &expanded);

    for retained in captured
        .compact_graph
        .nodes
        .iter()
        .filter(|node| node.id != captured.expansion.anchor)
    {
        assert_eq!(
            collapsed.nodes.iter().find(|node| node.id == retained.id),
            expanded.nodes.iter().find(|node| node.id == retained.id),
        );
    }
    assert!(
        collapsed
            .nodes
            .iter()
            .any(|node| node.id == captured.expansion.anchor)
    );
    assert!(
        collapsed
            .nodes
            .iter()
            .all(|node| !captured.expansion.members.contains(&node.id))
    );
    assert_eq!(
        collapsed.boundary_bundles.len(),
        captured.compact_layout.boundary_bundles.len()
    );
}

#[test]
fn captured_mux_vector_expands_without_a_full_relayout() {
    expand_without_a_full_relayout(&captured_mux_vector_expansion());
}

#[test]
fn captured_mux_vector_collapses_without_moving_unrelated_nodes() {
    let captured = captured_mux_vector_expansion();
    let expanded = expand_without_a_full_relayout(&captured);
    let collapsed = collapse_without_a_full_relayout(&captured, &expanded);
    for retained in captured
        .compact_graph
        .nodes
        .iter()
        .filter(|node| node.id != captured.expansion.anchor)
    {
        assert_eq!(
            collapsed.nodes.iter().find(|node| node.id == retained.id),
            expanded.nodes.iter().find(|node| node.id == retained.id),
        );
    }
}

#[test]
fn captured_middle_group_collapses_without_moving_expanded_peers() {
    let captured = captured_pipe_middle_group_collapse();
    let collapsed = collapse_captured_group(&captured)
        .expect("a middle group should collapse without rebuilding its expanded peers");

    for retained in captured
        .compact_graph
        .nodes
        .iter()
        .filter(|node| node.id != captured.expansion.anchor)
    {
        assert_eq!(
            collapsed.nodes.iter().find(|node| node.id == retained.id),
            captured
                .expanded_layout
                .nodes
                .iter()
                .find(|node| node.id == retained.id),
        );
    }
    for retained in captured.compact_graph.edges.iter().filter(|edge| {
        edge.source.node != captured.expansion.anchor
            && edge.target.node != captured.expansion.anchor
    }) {
        assert_eq!(
            collapsed.edges.iter().find(|edge| edge.id == retained.id),
            captured
                .expanded_layout
                .edges
                .iter()
                .find(|edge| edge.id == retained.id),
        );
    }
    assert!(
        captured
            .expansion
            .members
            .iter()
            .all(|member| collapsed.nodes.iter().all(|node| node.id != *member))
    );
    assert!(
        collapsed
            .nodes
            .iter()
            .any(|node| node.id == captured.expansion.anchor)
    );
}

#[test]
fn captured_middle_group_collapse_is_permutation_deterministic() {
    let captured = captured_pipe_middle_group_collapse();
    let expected = collapse_captured_group(&captured).expect("canonical collapse");
    let mut permutations = Vec::<(&str, CapturedCollapse)>::new();
    macro_rules! reversed {
        ($label:literal, $mutation:expr) => {{
            let mut permuted = captured.clone();
            $mutation(&mut permuted);
            permutations.push(($label, permuted));
        }};
    }
    reversed!("expanded graph nodes", |value: &mut CapturedCollapse| {
        value.expanded_graph.nodes.reverse()
    });
    reversed!("expanded graph edges", |value: &mut CapturedCollapse| {
        value.expanded_graph.edges.reverse()
    });
    reversed!("expanded layout nodes", |value: &mut CapturedCollapse| {
        value.expanded_layout.nodes.reverse()
    });
    reversed!("expanded layout edges", |value: &mut CapturedCollapse| {
        value.expanded_layout.edges.reverse()
    });
    reversed!("expanded layout bundles", |value: &mut CapturedCollapse| {
        value.expanded_layout.boundary_bundles.reverse()
    });
    reversed!(
        "expanded layout bundle members",
        |value: &mut CapturedCollapse| {
            for bundle in &mut value.expanded_layout.boundary_bundles {
                bundle.members.reverse();
            }
        }
    );
    reversed!("compact graph nodes", |value: &mut CapturedCollapse| {
        value.compact_graph.nodes.reverse()
    });
    reversed!("compact graph edges", |value: &mut CapturedCollapse| {
        value.compact_graph.edges.reverse()
    });
    reversed!("expansion members", |value: &mut CapturedCollapse| {
        value.expansion.members.reverse()
    });
    reversed!("boundary trunks", |value: &mut CapturedCollapse| {
        value.expansion.boundary_trunks.reverse()
    });
    reversed!("constraint inputs", |value: &mut CapturedCollapse| {
        value.constraints.inputs.reverse()
    });
    reversed!("constraint outputs", |value: &mut CapturedCollapse| {
        value.constraints.outputs.reverse()
    });
    reversed!("constraint bundles", |value: &mut CapturedCollapse| {
        value.constraints.boundary_bundles.reverse()
    });
    reversed!(
        "constraint bundle members",
        |value: &mut CapturedCollapse| {
            for bundle in &mut value.constraints.boundary_bundles {
                bundle.members.reverse();
            }
        }
    );

    for (label, permuted) in permutations {
        let actual =
            collapse_captured_group(&permuted).unwrap_or_else(|error| panic!("{label}: {error:?}"));
        assert_eq!(actual, expected, "{label}");
    }
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
