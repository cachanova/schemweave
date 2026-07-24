//! Guards the bench fixtures: generators must be deterministic and every
//! fixture must lay out successfully at Fast effort.

#[path = "../benches/support/generators.rs"]
mod generators;

use schemweave::{LayoutConfig, QualityEffort, layout_with_config};

#[allow(clippy::field_reassign_with_default)]
fn fast() -> LayoutConfig {
    let mut config = LayoutConfig::default();
    config.quality_effort = QualityEffort::Fast;
    config
}

#[test]
fn generators_are_deterministic() {
    assert_eq!(generators::pipeline(10, 3), generators::pipeline(10, 3));
    assert_eq!(
        generators::layered_dag(6, 8, 7),
        generators::layered_dag(6, 8, 7)
    );
    assert_eq!(generators::bus_chain(3, 8), generators::bus_chain(3, 8));
    assert_eq!(
        generators::protected_horizontal_expansion_pair(8, 8),
        generators::protected_horizontal_expansion_pair(8, 8)
    );
    assert_eq!(
        generators::boundary_fanout_expansion_pair(64),
        generators::boundary_fanout_expansion_pair(64)
    );
    assert_eq!(
        generators::boundary_fanin_expansion_pair(64),
        generators::boundary_fanin_expansion_pair(64)
    );
}

#[test]
fn protected_expansion_fixture_moves_the_peer_corridor() {
    use schemweave::{GroupExpansionOptions, ProtectedGroup, expand_group_in_place};

    let (compact, expanded, expansion, peer_ids) =
        generators::protected_horizontal_expansion_pair(8, 8);
    let config = fast();
    let compact_layout = layout_with_config(&compact, &config).expect("compact protected fixture");
    let layout = expand_group_in_place(
        &compact,
        &compact_layout,
        &expanded,
        &expansion,
        &GroupExpansionOptions {
            layout: config.layout,
            quality_effort: config.quality_effort,
            constraints: config.constraints,
            protected_groups: vec![ProtectedGroup {
                id: 1_000,
                members: peer_ids.clone(),
                frame_padding: 16.0,
            }],
        },
    )
    .expect("protected horizontal expansion");
    let peer_left = |layout: &schemweave::Layout| {
        layout
            .nodes
            .iter()
            .filter(|node| peer_ids.contains(&node.id))
            .map(|node| node.x)
            .min_by(f64::total_cmp)
            .expect("peer frame")
    };
    assert!(peer_left(&layout) > peer_left(&compact_layout));
}

#[test]
fn every_fixture_lays_out_and_layout_is_reproducible() {
    let fixtures = [
        ("pipeline", generators::pipeline(10, 3)),
        ("fanout", generators::fanout(16)),
        ("fanin", generators::fanin(16)),
        ("layered_dag", generators::layered_dag(6, 8, 7)),
        ("bus_chain", generators::bus_chain(3, 8)),
    ];
    for (name, graph) in fixtures {
        let first = layout_with_config(&graph, &fast())
            .unwrap_or_else(|error| panic!("{name} failed to lay out: {error:?}"));
        let second = layout_with_config(&graph, &fast()).expect("second run");
        assert_eq!(first, second, "{name} layout is not reproducible");
    }
}

#[test]
fn expansion_fixture_expands_without_full_relayout() {
    use schemweave::{
        GroupCollapseOptions, GroupExpansionOptions, collapse_group_in_place, expand_group_in_place,
    };

    for (members, bystanders) in [(4, 6), (8, 40)] {
        let (compact, expanded, expansion) = generators::expansion_pair(members, bystanders);
        let config = fast();
        let compact_layout = layout_with_config(&compact, &config)
            .unwrap_or_else(|error| panic!("({members}, {bystanders}) compact layout: {error:?}"));
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: config.layout,
                quality_effort: config.quality_effort,
                constraints: config.constraints.clone(),
                protected_groups: Vec::new(),
            },
        );
        let layout = result.unwrap_or_else(|error| {
            panic!("({members}, {bystanders}) synthetic expansion must succeed in place: {error:?}")
        });
        assert_eq!(
            layout.nodes.len(),
            expanded.nodes.len(),
            "({members}, {bystanders}) node count"
        );
        assert_eq!(
            layout.edges.len(),
            expanded.edges.len(),
            "({members}, {bystanders}) edge count"
        );
        let collapsed = collapse_group_in_place(
            &expanded,
            &layout,
            &compact,
            &expansion,
            &GroupCollapseOptions {
                layout: config.layout,
                constraints: config.constraints,
            },
        )
        .unwrap_or_else(|error| {
            panic!("({members}, {bystanders}) synthetic collapse must succeed in place: {error:?}")
        });
        assert_eq!(
            collapsed.nodes.len(),
            compact.nodes.len(),
            "({members}, {bystanders}) collapsed node count"
        );
        assert_eq!(
            collapsed.edges.len(),
            compact.edges.len(),
            "({members}, {bystanders}) collapsed edge count"
        );
    }
}

#[test]
fn shared_boundary_expansion_fixtures_are_reproducible() {
    use schemweave::{GroupExpansionOptions, expand_group_in_place};

    for (name, fixture) in [
        (
            "fanout",
            generators::boundary_fanout_expansion_pair
                as fn(
                    u32,
                ) -> (
                    schemweave::Graph,
                    schemweave::Graph,
                    schemweave::GroupExpansion,
                ),
        ),
        ("fanin", generators::boundary_fanin_expansion_pair),
    ] {
        for members in [64, 256] {
            let (compact, expanded, expansion) = fixture(members);
            let config = fast();
            let compact_layout =
                layout_with_config(&compact, &config).expect("compact boundary fixture");
            let options = GroupExpansionOptions {
                layout: config.layout,
                quality_effort: config.quality_effort,
                constraints: config.constraints,
                protected_groups: Vec::new(),
            };
            let first =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .unwrap_or_else(|error| {
                        panic!("{name}-{members} shared-boundary expansion failed: {error:?}")
                    });
            let second =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("second shared-boundary expansion");
            assert_eq!(
                first, second,
                "{name}-{members} expansion is not reproducible"
            );
        }
    }
}
