use schemweave::{
    BoundaryBundleConstraint, BoundaryBundleGeometry, BoundaryBundleMemberConstraint,
    BoundaryBundleRole, BoundaryBundleSegment, BoundaryTrunk, Edge, EdgeGeometry, Endpoint, Graph,
    GroupExpansion, GroupExpansionOptions, Layout, LayoutConfig, LayoutConstraints, LayoutOptions,
    Node, NodeGeometry, Point, Port, PortSide, QualityEffort, expand_group_in_place, layout,
    layout_with_config, layout_with_quality_effort, layout_with_quality_effort_and_constraints,
};
use schemweave_eval::{QualityReport, ScoreOptions, ViolationKind, score};

mod active_fanout_fixture {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../schemweave/tests/support/active_fanout.rs"
    ));
}

fn graph() -> Graph {
    Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 1,
                    side: PortSide::East,
                    offset: 25.0,
                }],
            },
            Node {
                id: 2,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 25.0,
                }],
            },
        ],
        edges: vec![Edge {
            id: 10,
            source: Endpoint { node: 1, port: 1 },
            target: Endpoint { node: 2, port: 0 },
            net: 7,
            participates_in_ranking: true,
        }],
    }
}

fn constructed_routes(routes: &[(u32, Vec<Point>)]) -> (Graph, Layout) {
    let mut nodes = Vec::with_capacity(routes.len() * 2);
    let mut node_geometry = Vec::with_capacity(routes.len() * 2);
    let mut edges = Vec::with_capacity(routes.len());
    let mut edge_geometry = Vec::with_capacity(routes.len());
    let mut width: f64 = 0.0;
    let mut height: f64 = 0.0;

    for (index, (net, points)) in routes.iter().enumerate() {
        let source_id = u32::try_from(index * 2).unwrap();
        let target_id = source_id + 1;
        let edge_id = u32::try_from(index).unwrap();
        let source = points[0];
        let target = points[points.len() - 1];
        nodes.extend([
            Node {
                id: source_id,
                width: 10.0,
                height: 10.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 5.0,
                }],
            },
            Node {
                id: target_id,
                width: 10.0,
                height: 10.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 5.0,
                }],
            },
        ]);
        node_geometry.extend([
            NodeGeometry {
                id: source_id,
                x: source.x - 10.0,
                y: source.y - 5.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: target_id,
                x: target.x,
                y: target.y - 5.0,
                width: 10.0,
                height: 10.0,
            },
        ]);
        edges.push(Edge {
            id: edge_id,
            source: Endpoint {
                node: source_id,
                port: 0,
            },
            target: Endpoint {
                node: target_id,
                port: 0,
            },
            net: *net,
            participates_in_ranking: true,
        });
        edge_geometry.push(EdgeGeometry {
            id: edge_id,
            points: points.clone(),
        });
        for point in points {
            width = width.max(point.x + 10.0);
            height = height.max(point.y + 10.0);
        }
    }

    (
        Graph { nodes, edges },
        Layout {
            nodes: node_geometry,
            edges: edge_geometry,
            boundary_bundles: Vec::new(),
            width,
            height,
        },
    )
}

#[test]
fn new_quality_fields_preserve_default_json_compatibility() {
    let options: ScoreOptions = serde_json::from_str("{}").unwrap();
    let report: QualityReport = serde_json::from_str("{}").unwrap();

    assert_eq!(options, ScoreOptions::default());
    assert_eq!(report, QualityReport::default());
}

#[test]
fn reports_bounded_net_aware_edge_node_clearance() {
    let (mut graph, mut layout) = constructed_routes(&[(
        1,
        vec![Point { x: 20.0, y: 20.0 }, Point { x: 180.0, y: 20.0 }],
    )]);
    graph.nodes.push(Node {
        id: 99,
        width: 10.0,
        height: 10.0,
        cycle_breaker: false,
        ports: Vec::new(),
    });
    layout.nodes.push(NodeGeometry {
        id: 99,
        x: 80.0,
        y: 30.0,
        width: 10.0,
        height: 10.0,
    });
    let exact = score(
        &graph,
        &layout,
        ScoreOptions {
            edge_node_clearance_threshold: 10.0,
            max_edge_node_clearance_pair_visits: 3,
            ..ScoreOptions::default()
        },
    );
    assert_eq!(exact.minimum_edge_node_clearance, Some(10.0));
    assert_eq!(exact.edge_node_clearance_violations, 0);
    assert!(!exact.edge_node_clearance_exhausted);

    let inside = score(
        &graph,
        &layout,
        ScoreOptions {
            edge_node_clearance_threshold: 10.0 + 1e-9,
            max_edge_node_clearance_pair_visits: 3,
            ..ScoreOptions::default()
        },
    );
    assert_eq!(inside.edge_node_clearance_violations, 1);

    let exhausted = score(
        &graph,
        &layout,
        ScoreOptions {
            edge_node_clearance_threshold: 10.0,
            max_edge_node_clearance_pair_visits: 2,
            ..ScoreOptions::default()
        },
    );
    assert!(exhausted.edge_node_clearance_exhausted);
    assert_eq!(exhausted.minimum_edge_node_clearance, None);
}

#[test]
fn shared_same_net_prefix_counts_as_one_physical_clearance_violation() {
    let route = vec![Point { x: 20.0, y: 20.0 }, Point { x: 180.0, y: 20.0 }];
    let (mut graph, mut layout) = constructed_routes(&[(7, route.clone()), (7, route)]);
    graph.nodes.push(Node {
        id: 99,
        width: 10.0,
        height: 10.0,
        cycle_breaker: false,
        ports: Vec::new(),
    });
    layout.nodes.push(NodeGeometry {
        id: 99,
        x: 80.0,
        y: 25.0,
        width: 10.0,
        height: 10.0,
    });

    let report = score(
        &graph,
        &layout,
        ScoreOptions {
            edge_node_clearance_threshold: 10.0,
            max_edge_node_clearance_pair_visits: 5,
            ..ScoreOptions::default()
        },
    );

    assert!(!report.edge_node_clearance_exhausted);
    assert_eq!(report.minimum_edge_node_clearance, Some(5.0));
    assert_eq!(report.edge_node_clearance_violations, 1);
}

#[test]
fn incremental_group_expansion_preserves_every_hard_gate() {
    let node = |id| Node {
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
    };
    let edge = |id, source, target, net| Edge {
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
    };
    let compact = Graph {
        nodes: vec![node(1), node(10), node(4)],
        edges: vec![edge(1, 1, 10, 100), edge(2, 10, 4, 200)],
    };
    let options = LayoutOptions::default();
    let compact_layout = layout(&compact, options).unwrap();
    let members = (1_000..1_032).collect::<Vec<_>>();
    let mut nodes = vec![node(1), node(4)];
    nodes.extend(members.iter().copied().map(node));
    let mut edges = Vec::new();
    for (index, &member) in members.iter().enumerate() {
        edges.push(edge(index as u32 * 2 + 10, 1, member, 100));
        edges.push(edge(index as u32 * 2 + 11, member, 4, 200));
    }
    let expanded = Graph { nodes, edges };
    let boundary_trunks = (0..members.len())
        .flat_map(|index| {
            [
                BoundaryTrunk {
                    expanded_edge: index as u32 * 2 + 10,
                    compact_edge: 1,
                },
                BoundaryTrunk {
                    expanded_edge: index as u32 * 2 + 11,
                    compact_edge: 2,
                },
            ]
        })
        .collect();
    let expanded_layout = expand_group_in_place(
        &compact,
        &compact_layout,
        &expanded,
        &GroupExpansion {
            anchor: 10,
            members,
            boundary_trunks,
        },
        &GroupExpansionOptions {
            layout: options,
            quality_effort: QualityEffort::Max,
            ..GroupExpansionOptions::default()
        },
    )
    .unwrap();

    let report = score(&expanded, &expanded_layout, ScoreOptions::default());
    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(report.ranking_direction_violations, 0, "{report:#?}");
}

#[test]
fn measures_straight_route_share_and_maximum_route_bends() {
    let (graph, layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 180.0, y: 20.0 }],
        ),
        (
            2,
            vec![
                Point { x: 20.0, y: 60.0 },
                Point { x: 40.0, y: 60.0 },
                Point { x: 40.0, y: 100.0 },
                Point { x: 80.0, y: 100.0 },
                Point { x: 80.0, y: 60.0 },
                Point { x: 180.0, y: 60.0 },
            ],
        ),
    ]);

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.scored_route_count, 2);
    assert_eq!(report.straight_route_count, 1);
    assert_eq!(report.straight_route_ratio, 0.5);
    assert_eq!(report.max_bends_per_route, 4);
}

#[test]
fn measures_minimum_parallel_separation_only_for_overlapping_spans() {
    let (graph, layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 100.0, y: 20.0 }],
        ),
        (
            2,
            vec![Point { x: 40.0, y: 32.0 }, Point { x: 160.0, y: 32.0 }],
        ),
        (
            3,
            vec![Point { x: 180.0, y: 21.0 }, Point { x: 260.0, y: 21.0 }],
        ),
    ]);

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.minimum_parallel_route_separation, Some(12.0));
}

#[test]
fn parallel_congestion_distinguishes_overlap_sub_lane_gap_and_target_spacing() {
    let (overlap_graph, overlap_layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 120.0, y: 20.0 }],
        ),
        (
            2,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 120.0, y: 20.0 }],
        ),
    ]);
    let overlap = score(&overlap_graph, &overlap_layout, ScoreOptions::default());
    assert_eq!(overlap.parallel_congestion_ratio, 1.0);

    let (graph, layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 120.0, y: 20.0 }],
        ),
        (
            2,
            vec![Point { x: 20.0, y: 20.23 }, Point { x: 120.0, y: 20.23 }],
        ),
        (
            3,
            vec![Point { x: 20.0, y: 24.23 }, Point { x: 120.0, y: 24.23 }],
        ),
    ]);

    let report = score(&graph, &layout, ScoreOptions::default());

    assert!((report.parallel_congestion_ratio - 2.0 / 3.0).abs() < 1e-12);
}

#[test]
fn parallel_congestion_weights_only_the_actual_close_overlap_length() {
    let (weighted_graph, weighted_layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 120.0, y: 20.0 }],
        ),
        (
            2,
            vec![Point { x: 60.0, y: 20.23 }, Point { x: 70.0, y: 20.23 }],
        ),
        (
            3,
            vec![Point { x: 40.0, y: 40.0 }, Point { x: 230.0, y: 40.0 }],
        ),
        (
            4,
            vec![Point { x: 40.0, y: 48.0 }, Point { x: 230.0, y: 48.0 }],
        ),
    ]);
    let weighted = score(&weighted_graph, &weighted_layout, ScoreOptions::default());
    // Only the ten-unit shared span on each of the first two routes is
    // congested. The remaining long route pair is eight units apart.
    assert!((weighted.parallel_congestion_ratio - 20.0 / 490.0).abs() < 1e-12);
}

#[test]
fn parallel_congestion_excludes_same_net_and_disjoint_spans() {
    let (excluded_graph, excluded_layout) = constructed_routes(&[
        (
            7,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 80.0, y: 20.0 }],
        ),
        (
            7,
            vec![Point { x: 20.0, y: 20.23 }, Point { x: 80.0, y: 20.23 }],
        ),
        (
            8,
            vec![Point { x: 80.0, y: 20.1 }, Point { x: 140.0, y: 20.1 }],
        ),
    ]);
    let excluded = score(&excluded_graph, &excluded_layout, ScoreOptions::default());
    assert_eq!(excluded.parallel_congestion_ratio, 0.0);
}

#[test]
fn parallel_congestion_is_permutation_deterministic() {
    let (graph, layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 180.0, y: 20.0 }],
        ),
        (
            2,
            vec![Point { x: 40.0, y: 20.23 }, Point { x: 160.0, y: 20.23 }],
        ),
        (
            3,
            vec![Point { x: 60.0, y: 24.23 }, Point { x: 140.0, y: 24.23 }],
        ),
    ]);
    let expected = score(&graph, &layout, ScoreOptions::default());
    let mut permuted_graph = graph;
    let mut permuted_layout = layout;
    permuted_graph.nodes.reverse();
    permuted_graph.edges.reverse();
    permuted_layout.nodes.reverse();
    permuted_layout.edges.reverse();

    let actual = score(&permuted_graph, &permuted_layout, ScoreOptions::default());

    assert_eq!(
        actual.parallel_congestion_ratio,
        expected.parallel_congestion_ratio
    );
}

#[test]
fn measures_the_largest_crossing_knot_on_one_physical_segment() {
    let (graph, layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 20.0 }, Point { x: 180.0, y: 20.0 }],
        ),
        (
            2,
            vec![Point { x: 20.0, y: 40.0 }, Point { x: 180.0, y: 40.0 }],
        ),
        (
            3,
            vec![Point { x: 20.0, y: 60.0 }, Point { x: 180.0, y: 60.0 }],
        ),
        (
            4,
            vec![
                Point { x: 20.0, y: 80.0 },
                Point { x: 100.0, y: 80.0 },
                Point { x: 100.0, y: 10.0 },
                Point { x: 180.0, y: 10.0 },
            ],
        ),
    ]);

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.crossings, 3);
    assert_eq!(report.max_crossings_on_segment, 3);
}

#[test]
fn attributes_a_crossing_knot_to_the_horizontal_segment() {
    let (graph, layout) = constructed_routes(&[
        (
            1,
            vec![Point { x: 20.0, y: 50.0 }, Point { x: 240.0, y: 50.0 }],
        ),
        (
            2,
            vec![
                Point { x: 20.0, y: 80.0 },
                Point { x: 60.0, y: 80.0 },
                Point { x: 60.0, y: 10.0 },
                Point { x: 180.0, y: 10.0 },
            ],
        ),
        (
            3,
            vec![
                Point { x: 70.0, y: 90.0 },
                Point { x: 100.0, y: 90.0 },
                Point { x: 100.0, y: 20.0 },
                Point { x: 200.0, y: 20.0 },
            ],
        ),
        (
            4,
            vec![
                Point { x: 120.0, y: 100.0 },
                Point { x: 140.0, y: 100.0 },
                Point { x: 140.0, y: 30.0 },
                Point { x: 220.0, y: 30.0 },
            ],
        ),
    ]);

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.crossings, 3);
    assert_eq!(report.max_crossings_on_segment, 3);
}

#[test]
fn measures_physical_route_usage_outside_the_node_envelope() {
    let (graph, layout) = constructed_routes(&[(
        1,
        vec![
            Point { x: 20.0, y: 50.0 },
            Point { x: 40.0, y: 50.0 },
            Point { x: 40.0, y: 10.0 },
            Point { x: 160.0, y: 10.0 },
            Point { x: 160.0, y: 50.0 },
            Point { x: 180.0, y: 50.0 },
        ],
    )]);

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.route_length, 240.0);
    assert_eq!(report.perimeter_route_length, 190.0);
    assert_eq!(report.perimeter_route_ratio, 190.0 / 240.0);
}

#[test]
fn quality_effort_selects_the_exact_scored_adaptive_gap_tracks() {
    let graph = Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 20.0,
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
                width: 20.0,
                height: 40.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 20.0,
                }],
            },
            Node {
                id: 3,
                width: 20.0,
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
                        side: PortSide::West,
                        offset: 30.0,
                    },
                ],
            },
            Node {
                id: 4,
                width: 20.0,
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
                        side: PortSide::West,
                        offset: 30.0,
                    },
                ],
            },
        ],
        edges: vec![
            Edge {
                id: 10,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 3, port: 0 },
                net: 1,
                participates_in_ranking: true,
            },
            Edge {
                id: 11,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 4, port: 0 },
                net: 1,
                participates_in_ranking: true,
            },
            Edge {
                id: 12,
                source: Endpoint { node: 2, port: 0 },
                target: Endpoint { node: 3, port: 1 },
                net: 2,
                participates_in_ranking: true,
            },
            Edge {
                id: 13,
                source: Endpoint { node: 2, port: 0 },
                target: Endpoint { node: 4, port: 1 },
                net: 2,
                participates_in_ranking: true,
            },
        ],
    };
    let options = LayoutOptions::default();
    let fast = layout_with_quality_effort(&graph, options, QualityEffort::Fast).unwrap();
    let quality = layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
    let max = layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();
    let fast_report = score(&graph, &fast, ScoreOptions::default());
    let quality_report = score(&graph, &quality, ScoreOptions::default());

    assert!(fast_report.passes_hard_gates(), "{fast_report:#?}");
    assert!(quality_report.passes_hard_gates(), "{quality_report:#?}");
    assert!(
        quality_report.minimum_parallel_route_separation
            > fast_report.minimum_parallel_route_separation,
        "fast={fast_report:#?}\nquality={quality_report:#?}"
    );
    assert_eq!(quality_report.crossings, fast_report.crossings);
    assert_eq!(quality_report.bends, fast_report.bends);
    assert_eq!(
        quality_report.max_crossings_on_segment,
        fast_report.max_crossings_on_segment
    );
    assert!(quality_report.route_length <= fast_report.route_length);
    assert_eq!(quality_report.area, fast_report.area);
    assert_eq!(
        quality_report.straight_route_ratio,
        fast_report.straight_route_ratio
    );
    assert_eq!(max, quality);

    let mut permuted = graph;
    permuted.nodes.reverse();
    permuted.edges.reverse();
    assert_eq!(
        layout_with_quality_effort(&permuted, options, QualityEffort::Quality).unwrap(),
        quality
    );
}

#[test]
fn quality_spreads_a_dense_small_gap_across_the_full_safe_channel() {
    let port_count = 10u32;
    let ports = |side| {
        (0..port_count)
            .map(|id| Port {
                id,
                side,
                offset: 10.0 + f64::from(id) * 20.0,
            })
            .collect()
    };
    let graph = Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 20.0,
                height: 210.0,
                cycle_breaker: false,
                ports: ports(PortSide::East),
            },
            Node {
                id: 2,
                width: 20.0,
                height: 210.0,
                cycle_breaker: false,
                ports: ports(PortSide::West),
            },
        ],
        edges: (0..port_count)
            .map(|id| Edge {
                id,
                source: Endpoint { node: 1, port: id },
                target: Endpoint {
                    node: 2,
                    port: port_count - id - 1,
                },
                net: id,
                participates_in_ranking: true,
            })
            .collect(),
    };
    let options = LayoutOptions::default();
    let fast = layout_with_quality_effort(&graph, options, QualityEffort::Fast).unwrap();
    let quality = layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
    let fast_report = score(&graph, &fast, ScoreOptions::default());
    let quality_report = score(&graph, &quality, ScoreOptions::default());
    assert!(fast_report.passes_hard_gates(), "{fast_report:#?}");
    assert!(quality_report.passes_hard_gates(), "{quality_report:#?}");
    assert_eq!(quality_report.crossings, fast_report.crossings);
    assert_eq!(quality_report.bends, fast_report.bends);
    assert_eq!(quality_report.route_length, fast_report.route_length);
    assert_eq!(quality_report.area, fast_report.area);
    assert!(
        quality_report.parallel_congestion_ratio < fast_report.parallel_congestion_ratio,
        "fast={fast_report:#?}\nquality={quality_report:#?}"
    );
    assert!(
        quality_report.minimum_parallel_route_separation
            > fast_report.minimum_parallel_route_separation,
        "fast={fast_report:#?}\nquality={quality_report:#?}"
    );

    let mut large_graph = graph.clone();
    large_graph.nodes.extend((3..=401).map(|id| Node {
        id,
        width: 20.0,
        height: 20.0,
        cycle_breaker: false,
        ports: Vec::new(),
    }));
    let bounded =
        layout_with_quality_effort(&large_graph, options, QualityEffort::Quality).unwrap();
    let maximum = layout_with_quality_effort(&large_graph, options, QualityEffort::Max).unwrap();
    let bounded_report = score(&large_graph, &bounded, ScoreOptions::default());
    let maximum_report = score(&large_graph, &maximum, ScoreOptions::default());
    assert!(bounded_report.passes_hard_gates(), "{bounded_report:#?}");
    assert!(maximum_report.passes_hard_gates(), "{maximum_report:#?}");
    assert_eq!(maximum_report.crossings, bounded_report.crossings);
    assert_eq!(maximum_report.bends, bounded_report.bends);
    assert!(
        maximum_report.parallel_congestion_ratio < bounded_report.parallel_congestion_ratio,
        "quality={bounded_report:#?}\nmax={maximum_report:#?}"
    );

    let mut mixed_graph = graph.clone();
    mixed_graph.nodes[0].ports.push(Port {
        id: port_count,
        side: PortSide::West,
        offset: 200.0,
    });
    mixed_graph.nodes[1].ports.push(Port {
        id: port_count,
        side: PortSide::East,
        offset: 200.0,
    });
    mixed_graph.edges.push(Edge {
        id: port_count,
        source: Endpoint {
            node: 2,
            port: port_count,
        },
        target: Endpoint {
            node: 1,
            port: port_count,
        },
        net: port_count,
        participates_in_ranking: false,
    });
    let mixed = layout_with_quality_effort(&mixed_graph, options, QualityEffort::Quality).unwrap();
    let mixed_report = score(&mixed_graph, &mixed, ScoreOptions::default());
    assert!(mixed_report.passes_hard_gates(), "{mixed_report:#?}");
    let mut mixed_permuted = mixed_graph.clone();
    mixed_permuted.nodes.reverse();
    mixed_permuted.edges.reverse();
    assert_eq!(
        layout_with_quality_effort(&mixed_permuted, options, QualityEffort::Quality).unwrap(),
        mixed
    );

    let mut permuted = graph;
    permuted.nodes.reverse();
    permuted.edges.reverse();
    assert_eq!(
        layout_with_quality_effort(&permuted, options, QualityEffort::Quality).unwrap(),
        quality
    );
}

#[test]
fn accepts_the_current_exact_port_baseline() {
    let graph = graph();
    let layout = layout(&graph, LayoutOptions::default()).unwrap();
    let report = score(&graph, &layout, ScoreOptions::default());
    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(report.semantic_violations, 0);
    assert_eq!(report.node_overlaps, 0);
    assert_eq!(report.node_intersections, 0);
    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.forward_edge_count, 1);
    assert_eq!(report.reverse_x_length, 0.0);
    assert_eq!(report.forward_routes_with_reverse_x, 0);
    assert_eq!(report.p95_forward_stretch, 1.0);
    assert_eq!(report.split_feedback_nets, 0);
    assert_eq!(report.feedback_net_count, 0);
    assert_eq!(report.shared_route_ratio, 0.0);
    assert_eq!(
        report.viewport_fit,
        (layout.width / 1_600.0).max(layout.height / 900.0)
    );
    assert!(report.segments > 0);
    assert!(report.route_length > 0.0);
}

#[test]
fn fanout_candidate_layout_preserves_all_hard_gates() {
    let graph = active_fanout_fixture::graph();
    let options = LayoutOptions {
        ordering_sweeps: 0,
        ..LayoutOptions::default()
    };
    let layout = layout(&graph, options).unwrap();
    let report = score(&graph, &layout, ScoreOptions::default());

    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(report.semantic_violations, 0);
    assert_eq!(report.ranking_direction_violations, 0);
}

fn regional_fanout_graph() -> Graph {
    let mut nodes = vec![Node {
        id: 0,
        width: 82.0,
        height: 34.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side: PortSide::East,
            offset: 17.0,
        }],
    }];
    for id in 1..500 {
        let width = if id < 10 {
            82.0
        } else if id < 100 {
            89.0
        } else {
            96.0
        };
        let height = if id >= 492 {
            42.0
        } else if id % 5 == 0 {
            58.0
        } else {
            46.0
        };
        nodes.push(Node {
            id,
            width,
            height,
            cycle_breaker: false,
            ports: std::iter::once(Port {
                id: 0,
                side: PortSide::West,
                offset: if id < 9 { height / 2.0 } else { height / 3.0 },
            })
            .chain((id >= 9).then_some(Port {
                id: 1,
                side: PortSide::West,
                offset: height * 2.0 / 3.0,
            }))
            .chain((id <= 491).then_some(Port {
                id: 2,
                side: PortSide::East,
                offset: height / 2.0,
            }))
            .collect(),
        });
    }
    let mut edges = Vec::new();
    for node in 1..=491 {
        edges.push(Edge {
            id: edges.len() as u32,
            source: Endpoint { node: 0, port: 0 },
            target: Endpoint { node, port: 0 },
            net: 1,
            participates_in_ranking: true,
        });
        edges.push(Edge {
            id: edges.len() as u32,
            source: Endpoint { node, port: 2 },
            target: Endpoint {
                node: node + 8,
                port: 1,
            },
            net: 10_000 + node,
            participates_in_ranking: true,
        });
    }
    for source in 1..=8 {
        edges.push(Edge {
            id: edges.len() as u32,
            source: Endpoint {
                node: source,
                port: 2,
            },
            target: Endpoint {
                node: 491 + source,
                port: 0,
            },
            net: 10_000 + source,
            participates_in_ranking: true,
        });
    }
    Graph { nodes, edges }
}

fn negotiated_corridor_graph() -> Graph {
    const LAYERS: u32 = 6;
    const WIDTH: u32 = 20;
    let nodes = (0..LAYERS * WIDTH)
        .map(|id| Node {
            id,
            width: 76.0,
            height: 84.0,
            cycle_breaker: false,
            ports: std::iter::once(Port {
                id: 0,
                side: PortSide::East,
                offset: 42.0,
            })
            .chain((1..=6).map(|id| Port {
                id,
                side: PortSide::West,
                offset: 12.0 * id as f64,
            }))
            .collect(),
        })
        .collect();
    let mut edges = Vec::new();
    for layer in 0..LAYERS - 1 {
        for source in 0..WIDTH {
            edges.push(Edge {
                id: edges.len() as u32,
                source: Endpoint {
                    node: layer * WIDTH + source,
                    port: 0,
                },
                target: Endpoint {
                    node: (layer + 1) * WIDTH + source,
                    port: 1,
                },
                net: layer * WIDTH + source,
                participates_in_ranking: true,
            });
        }
    }
    for layer in 0..LAYERS - 3 {
        for source in 0..WIDTH {
            for branch in 0..5 {
                edges.push(Edge {
                    id: edges.len() as u32,
                    source: Endpoint {
                        node: layer * WIDTH + source,
                        port: 0,
                    },
                    target: Endpoint {
                        node: (layer + 3) * WIDTH + (source * 7 + branch * 11) % WIDTH,
                        port: branch + 2,
                    },
                    net: layer * WIDTH + source,
                    participates_in_ranking: true,
                });
            }
        }
    }
    Graph { nodes, edges }
}

fn rounded_staircase_fanout_graph() -> Graph {
    const INTERNAL_END: u32 = 17;
    const BRANCHES: u32 = 5;

    let mut nodes = vec![Node {
        id: 0,
        width: 82.0,
        height: 34.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side: PortSide::East,
            offset: 17.0,
        }],
    }];
    for id in 1..=INTERNAL_END {
        let width = if id < 10 { 82.0 } else { 89.0 };
        let height = if id % 5 == 0 { 58.0 } else { 46.0 };
        let mut ports = vec![Port {
            id: 0,
            side: PortSide::East,
            offset: height / 2.0,
        }];
        if id <= BRANCHES {
            ports.push(Port {
                id: 1,
                side: PortSide::West,
                offset: height / 2.0,
            });
        } else {
            ports.extend([
                Port {
                    id: 1,
                    side: PortSide::West,
                    offset: height / 3.0,
                },
                Port {
                    id: 2,
                    side: PortSide::West,
                    offset: height * 2.0 / 3.0,
                },
            ]);
        }
        nodes.push(Node {
            id,
            width,
            height,
            cycle_breaker: false,
            ports,
        });
    }
    for id in INTERNAL_END + 1..=INTERNAL_END + BRANCHES {
        nodes.push(Node {
            id,
            width: 89.0,
            height: 42.0,
            cycle_breaker: false,
            ports: vec![
                Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 14.0,
                },
                Port {
                    id: 1,
                    side: PortSide::West,
                    offset: 28.0,
                },
            ],
        });
    }

    let mut edges = Vec::new();
    for node in 1..=INTERNAL_END {
        edges.push(Edge {
            id: edges.len() as u32,
            source: Endpoint { node: 0, port: 0 },
            target: Endpoint { node, port: 1 },
            net: 0,
            participates_in_ranking: true,
        });
        let target = node + BRANCHES;
        edges.push(Edge {
            id: edges.len() as u32,
            source: Endpoint { node, port: 0 },
            target: Endpoint {
                node: target,
                port: if target <= INTERNAL_END { 2 } else { 1 },
            },
            net: node,
            participates_in_ranking: true,
        });
    }
    for source in 1..=BRANCHES {
        edges.push(Edge {
            id: edges.len() as u32,
            source: Endpoint {
                node: source,
                port: 0,
            },
            target: Endpoint {
                node: INTERNAL_END + source,
                port: 0,
            },
            net: source,
            participates_in_ranking: true,
        });
    }
    Graph { nodes, edges }
}

#[test]
fn rounded_staircase_fanout_highest_quality_is_deterministic_and_hard_safe() {
    let graph = rounded_staircase_fanout_graph();
    let config = LayoutConfig::highest_quality();
    let first = layout_with_config(&graph, &config).unwrap();
    let mut permuted = graph.clone();
    permuted.nodes.reverse();
    for node in &mut permuted.nodes {
        node.ports.reverse();
    }
    permuted.edges.reverse();
    let second = layout_with_config(&permuted, &config).unwrap();
    let report = score(&graph, &first, ScoreOptions::default());

    assert_eq!(
        first, second,
        "the public Max layout must be stable-ID deterministic"
    );
    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(report.semantic_violations, 0);
    assert_eq!(report.node_overlaps, 0);
    assert_eq!(report.node_intersections, 0);
    assert_eq!(report.unrelated_overlaps, 0);
    assert_eq!(report.unrelated_contacts, 0);
    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.reverse_x_length, 0.0);
    assert!(!report.edge_node_clearance_exhausted);
    assert_eq!(report.edge_node_clearance_violations, 0);
    assert!(
        report
            .minimum_edge_node_clearance
            .is_some_and(|clearance| clearance >= 20.0)
    );
}

#[test]
fn negotiated_corridor_max_candidate_preserves_every_hard_gate() {
    let graph = negotiated_corridor_graph();
    let options = LayoutOptions::default();
    let quality = layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
    let max = layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();
    let quality_report = score(&graph, &quality, ScoreOptions::default());
    let report = score(&graph, &max, ScoreOptions::default());

    assert_ne!(
        max, quality,
        "fixture must activate a Max negotiated-corridor candidate"
    );
    assert!(
        report.crossings < quality_report.crossings,
        "negotiated corridors must improve exact crossings",
    );
    assert!(report.passes_hard_gates(), "{report:#?}");
}

#[test]
fn positive_clearance_covers_negotiated_corridor_candidates() {
    let graph = negotiated_corridor_graph();
    let layout = layout_with_quality_effort(
        &graph,
        LayoutOptions {
            edge_node_clearance: 20.0,
            ..LayoutOptions::default()
        },
        QualityEffort::Max,
    )
    .unwrap();
    let report = score(&graph, &layout, ScoreOptions::default());
    assert!(!report.edge_node_clearance_exhausted);
    assert_eq!(report.edge_node_clearance_violations, 0, "{report:#?}");
}

#[test]
fn regional_fanout_max_candidate_preserves_every_hard_gate() {
    let graph = regional_fanout_graph();
    let options = LayoutOptions::default();
    let constraints = LayoutConstraints {
        inputs: vec![0],
        outputs: (492..500).collect(),
        boundary_bundles: Vec::new(),
    };
    let quality = layout_with_quality_effort_and_constraints(
        &graph,
        options,
        QualityEffort::Quality,
        &constraints,
    )
    .unwrap();
    let max = layout_with_quality_effort_and_constraints(
        &graph,
        options,
        QualityEffort::Max,
        &constraints,
    )
    .unwrap();
    let quality_report = score(&graph, &quality, ScoreOptions::default());
    let report = score(&graph, &max, ScoreOptions::default());
    let top = max
        .nodes
        .iter()
        .map(|node| node.y)
        .fold(f64::INFINITY, f64::min);
    let bottom = max
        .nodes
        .iter()
        .map(|node| node.y + node.height)
        .fold(f64::NEG_INFINITY, f64::max);
    let max_hot_routes = graph
        .edges
        .iter()
        .zip(&max.edges)
        .filter(|(edge, _)| edge.net == 1)
        .map(|(_, route)| route);
    let quality_hot_routes = graph
        .edges
        .iter()
        .zip(&quality.edges)
        .filter(|(edge, _)| edge.net == 1)
        .map(|(_, route)| route);

    assert_ne!(
        max, quality,
        "fixture must activate the Max regional candidate"
    );
    assert!(
        report.crossings < quality_report.crossings,
        "regional trunks must improve exact crossings",
    );
    assert!(
        max_hot_routes
            .clone()
            .flat_map(|route| &route.points)
            .all(|point| point.y >= top && point.y <= bottom),
        "regional fanout routes must use interior trunks",
    );
    assert!(
        quality_hot_routes
            .flat_map(|route| &route.points)
            .any(|point| point.y < top || point.y > bottom),
        "the control layout must retain its outer fanout trunk",
    );
    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(report.semantic_violations, 0);
    assert_eq!(report.node_overlaps, 0);
    assert_eq!(report.node_intersections, 0);
    assert_eq!(report.unrelated_overlaps, 0);
    assert_eq!(report.unrelated_contacts, 0);
    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.reverse_x_length, 0.0);
}

#[test]
fn positive_clearance_covers_regional_fanout_and_boundary_constraints() {
    let graph = regional_fanout_graph();
    let constraints = LayoutConstraints {
        inputs: vec![0],
        outputs: (492..500).collect(),
        boundary_bundles: Vec::new(),
    };
    let layout = layout_with_quality_effort_and_constraints(
        &graph,
        LayoutOptions {
            edge_node_clearance: 20.0,
            ..LayoutOptions::default()
        },
        QualityEffort::Max,
        &constraints,
    )
    .unwrap();
    let report = score(&graph, &layout, ScoreOptions::default());
    assert!(!report.edge_node_clearance_exhausted);
    assert_eq!(report.edge_node_clearance_violations, 0, "{report:#?}");
    let input_x = layout.nodes.iter().find(|node| node.id == 0).unwrap().x;
    assert!(
        layout
            .nodes
            .iter()
            .all(|node| node.id == 0 || node.x >= input_x)
    );
    let right = layout
        .nodes
        .iter()
        .filter(|node| constraints.outputs.contains(&node.id))
        .map(|node| node.x + node.width)
        .collect::<Vec<_>>();
    assert!(right.windows(2).all(|pair| pair[0] == pair[1]));
}

#[test]
fn demand_aware_and_pitched_max_candidates_are_selected_safely_and_deterministically() {
    let (graph, constraints): (Graph, LayoutConstraints) =
        serde_json::from_str(include_str!("fixtures/demand_aware_priority.json")).unwrap();
    let options = LayoutOptions {
        route_lane_gap: 6.0,
        edge_node_clearance: 20.0,
        ..LayoutOptions::default()
    };
    let quality = layout_with_quality_effort_and_constraints(
        &graph,
        options,
        QualityEffort::Quality,
        &constraints,
    )
    .unwrap();
    let selected = layout_with_quality_effort_and_constraints(
        &graph,
        options,
        QualityEffort::Max,
        &constraints,
    )
    .unwrap();
    let quality_report = score(&graph, &quality, ScoreOptions::default());
    let selected_report = score(&graph, &selected, ScoreOptions::default());
    let fast = layout_with_quality_effort_and_constraints(
        &graph,
        options,
        QualityEffort::Fast,
        &constraints,
    )
    .unwrap();
    let fast_report = score(&graph, &fast, ScoreOptions::default());

    assert!(selected_report.passes_hard_gates(), "{selected_report:#?}");
    assert_eq!(fast_report.crossings, 860);
    assert_eq!(fast_report.bends, 1_368);
    assert_eq!(fast_report.area, 5_750_000.180_571_682);
    assert_eq!(quality_report.crossings, 860);
    assert_eq!(quality_report.bends, 1_368);
    assert_eq!(quality_report.area, 5_750_000.180_571_682);
    assert_eq!(selected_report.crossings, 816);
    assert_eq!(selected_report.bends, 1_226);
    assert_eq!(selected_report.route_length, 150_047.589_694_203_02);
    assert_eq!(selected_report.area, 5_846_383.068_468_34);
    assert_eq!(
        selected_report.minimum_parallel_route_separation,
        Some(0.153_847_077_633_599_84)
    );
    assert_eq!(
        selected_report.parallel_congestion_ratio,
        0.297_335_049_197_400_45
    );
    assert!(selected_report.area > quality_report.area * 1.01);
    assert!(
        selected_report.parallel_congestion_ratio < quality_report.parallel_congestion_ratio * 0.55
    );
    let node = |id| selected.nodes.iter().find(|node| node.id == id).unwrap();
    let input_x = constraints
        .inputs
        .iter()
        .map(|&id| node(id).x)
        .collect::<Vec<_>>();
    let output_right = constraints
        .outputs
        .iter()
        .map(|&id| node(id).x + node(id).width)
        .collect::<Vec<_>>();
    assert!(input_x.windows(2).all(|pair| pair[0] == pair[1]));
    assert!(output_right.windows(2).all(|pair| pair[0] == pair[1]));

    let mut permuted = graph.clone();
    permuted.nodes.reverse();
    for node in &mut permuted.nodes {
        node.ports.reverse();
    }
    permuted.edges.reverse();
    let mut permuted_constraints = constraints;
    permuted_constraints.inputs.reverse();
    permuted_constraints.outputs.reverse();
    assert_eq!(
        layout_with_quality_effort_and_constraints(
            &permuted,
            options,
            QualityEffort::Max,
            &permuted_constraints,
        )
        .unwrap(),
        selected
    );
}

#[test]
fn viewport_fit_uses_configured_dimensions_and_rejects_invalid_dimensions() {
    let graph = graph();
    let layout = layout(&graph, LayoutOptions::default()).unwrap();
    let options = ScoreOptions {
        viewport_width: layout.width * 2.0,
        viewport_height: layout.height * 4.0,
        ..ScoreOptions::default()
    };

    let report = score(&graph, &layout, options);
    assert_eq!(report.viewport_fit, 0.5);

    for invalid in [
        ScoreOptions {
            viewport_width: 0.0,
            ..ScoreOptions::default()
        },
        ScoreOptions {
            viewport_height: f64::NAN,
            ..ScoreOptions::default()
        },
        ScoreOptions {
            parallel_congestion_threshold: 0.0,
            ..ScoreOptions::default()
        },
    ] {
        let report = score(&graph, &layout, invalid);
        assert_eq!(report.semantic_violations, 1);
        assert_eq!(report.examples[0].kind, ViolationKind::InvalidGeometry);
    }
}

#[test]
fn measures_westward_detours_on_forward_routes() {
    let graph = graph();
    let mut layout = layout(&graph, LayoutOptions::default()).unwrap();
    let route = &mut layout.edges[0];
    let source = route.points[0];
    let target = route.points[route.points.len() - 1];
    route.points = vec![
        source,
        Point {
            x: source.x + 20.0,
            y: source.y,
        },
        Point {
            x: source.x + 20.0,
            y: source.y + 10.0,
        },
        Point {
            x: source.x + 10.0,
            y: source.y + 10.0,
        },
        Point {
            x: source.x + 10.0,
            y: target.y,
        },
        target,
    ];

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.reverse_x_length, 10.0);
    assert_eq!(report.forward_routes_with_reverse_x, 1);
    assert!(report.p95_forward_stretch > 1.0);
}

#[test]
fn p95_forward_stretch_uses_the_nearest_rank_boundary() {
    let mut nodes = Vec::new();
    let mut node_geometry = Vec::new();
    let mut edges = Vec::new();
    let mut edge_geometry = Vec::new();

    for id in 0u32..20 {
        let source_id = id * 2;
        let target_id = source_id + 1;
        let y = f64::from(id) * 50.0;
        nodes.push(Node {
            id: source_id,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side: PortSide::East,
                offset: 10.0,
            }],
        });
        nodes.push(Node {
            id: target_id,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side: PortSide::West,
                offset: 10.0,
            }],
        });
        node_geometry.push(NodeGeometry {
            id: source_id,
            x: 20.0,
            y,
            width: 20.0,
            height: 20.0,
        });
        node_geometry.push(NodeGeometry {
            id: target_id,
            x: 220.0,
            y,
            width: 20.0,
            height: 20.0,
        });
        edges.push(Edge {
            id,
            source: Endpoint {
                node: source_id,
                port: 0,
            },
            target: Endpoint {
                node: target_id,
                port: 0,
            },
            net: id,
            participates_in_ranking: true,
        });
        let source = Point {
            x: 40.0,
            y: y + 10.0,
        };
        let target = Point {
            x: 220.0,
            y: y + 10.0,
        };
        let points = if id == 19 {
            vec![
                source,
                Point {
                    x: 80.0,
                    y: y + 10.0,
                },
                Point {
                    x: 80.0,
                    y: y + 30.0,
                },
                Point {
                    x: 60.0,
                    y: y + 30.0,
                },
                Point {
                    x: 60.0,
                    y: y + 10.0,
                },
                target,
            ]
        } else {
            vec![source, target]
        };
        edge_geometry.push(EdgeGeometry { id, points });
    }

    let graph = Graph { nodes, edges };
    let layout = Layout {
        nodes: node_geometry,
        edges: edge_geometry,
        boundary_bundles: Vec::new(),
        width: 260.0,
        height: 990.0,
    };
    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.forward_edge_count, 20);
    assert_eq!(report.forward_routes_with_reverse_x, 1);
    assert_eq!(report.p95_forward_stretch, 1.0);
}

#[test]
fn ranking_edges_must_advance_to_a_nonoverlapping_x_range() {
    let graph = graph();
    let layout = Layout {
        nodes: vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 80.0,
                height: 50.0,
            },
            NodeGeometry {
                id: 2,
                x: 70.0,
                y: 100.0,
                width: 80.0,
                height: 50.0,
            },
        ],
        edges: vec![EdgeGeometry {
            id: 10,
            points: vec![
                Point { x: 80.0, y: 25.0 },
                Point { x: 160.0, y: 25.0 },
                Point { x: 160.0, y: 170.0 },
                Point { x: 60.0, y: 170.0 },
                Point { x: 60.0, y: 125.0 },
                Point { x: 70.0, y: 125.0 },
            ],
        }],
        boundary_bundles: Vec::new(),
        width: 180.0,
        height: 180.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.node_overlaps, 0);
    assert_eq!(report.node_intersections, 0);
    assert_eq!(report.ranking_direction_violations, 1);
    assert!(!report.passes_hard_gates());
}

#[test]
fn excludes_scc_internal_edges_but_scores_edges_leaving_the_component() {
    let node = |id, ports| Node {
        id,
        width: 20.0,
        height: 20.0,
        cycle_breaker: false,
        ports,
    };
    let graph = Graph {
        nodes: vec![
            node(
                0,
                vec![
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
                ],
            ),
            node(
                1,
                vec![
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
                ],
            ),
            node(
                2,
                vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 10.0,
                }],
            ),
        ],
        edges: vec![
            Edge {
                id: 0,
                source: Endpoint { node: 0, port: 1 },
                target: Endpoint { node: 1, port: 0 },
                net: 0,
                participates_in_ranking: true,
            },
            Edge {
                id: 1,
                source: Endpoint { node: 1, port: 1 },
                target: Endpoint { node: 0, port: 0 },
                net: 1,
                participates_in_ranking: true,
            },
            Edge {
                id: 2,
                source: Endpoint { node: 1, port: 1 },
                target: Endpoint { node: 2, port: 0 },
                net: 2,
                participates_in_ranking: true,
            },
        ],
    };
    let layout = Layout {
        nodes: vec![
            NodeGeometry {
                id: 0,
                x: 20.0,
                y: 0.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 1,
                x: 120.0,
                y: 0.0,
                width: 20.0,
                height: 20.0,
            },
            NodeGeometry {
                id: 2,
                x: 220.0,
                y: 0.0,
                width: 20.0,
                height: 20.0,
            },
        ],
        edges: vec![
            EdgeGeometry {
                id: 0,
                points: vec![Point { x: 40.0, y: 10.0 }, Point { x: 120.0, y: 10.0 }],
            },
            EdgeGeometry {
                id: 1,
                points: vec![
                    Point { x: 140.0, y: 10.0 },
                    Point { x: 150.0, y: 10.0 },
                    Point { x: 150.0, y: 50.0 },
                    Point { x: 10.0, y: 50.0 },
                    Point { x: 10.0, y: 10.0 },
                    Point { x: 20.0, y: 10.0 },
                ],
            },
            EdgeGeometry {
                id: 2,
                points: vec![Point { x: 140.0, y: 10.0 }, Point { x: 220.0, y: 10.0 }],
            },
        ],
        boundary_bundles: Vec::new(),
        width: 260.0,
        height: 60.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.forward_edge_count, 1);
    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.reverse_x_length, 0.0);
    assert_eq!(report.p95_forward_stretch, 1.0);
}

#[test]
fn detects_a_feedback_net_split_across_outer_bands() {
    let node = |id, x, y, cycle_breaker, ports| {
        (
            Node {
                id,
                width: 20.0,
                height: 20.0,
                cycle_breaker,
                ports,
            },
            NodeGeometry {
                id,
                x,
                y,
                width: 20.0,
                height: 20.0,
            },
        )
    };
    let (root, root_geometry) = node(
        0,
        0.0,
        70.0,
        false,
        vec![Port {
            id: 0,
            side: PortSide::East,
            offset: 10.0,
        }],
    );
    let (source, source_geometry) = node(
        1,
        50.0,
        70.0,
        true,
        vec![
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
        ],
    );
    let (top, top_geometry) = node(
        2,
        200.0,
        20.0,
        false,
        vec![
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
        ],
    );
    let (bottom, bottom_geometry) = node(
        3,
        200.0,
        120.0,
        true,
        vec![Port {
            id: 0,
            side: PortSide::West,
            offset: 10.0,
        }],
    );
    let graph = Graph {
        nodes: vec![root, source, top, bottom],
        edges: vec![
            Edge {
                id: 0,
                source: Endpoint { node: 0, port: 0 },
                target: Endpoint { node: 1, port: 0 },
                net: 8,
                participates_in_ranking: true,
            },
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
            Edge {
                id: 3,
                source: Endpoint { node: 2, port: 1 },
                target: Endpoint { node: 1, port: 0 },
                net: 7,
                participates_in_ranking: true,
            },
        ],
    };
    let layout = Layout {
        nodes: vec![
            root_geometry,
            source_geometry,
            top_geometry,
            bottom_geometry,
        ],
        edges: vec![
            EdgeGeometry {
                id: 0,
                points: vec![Point { x: 20.0, y: 80.0 }, Point { x: 50.0, y: 80.0 }],
            },
            EdgeGeometry {
                id: 1,
                points: vec![
                    Point { x: 70.0, y: 80.0 },
                    Point { x: 80.0, y: 80.0 },
                    Point { x: 80.0, y: 0.0 },
                    Point { x: 190.0, y: 0.0 },
                    Point { x: 190.0, y: 30.0 },
                    Point { x: 200.0, y: 30.0 },
                ],
            },
            EdgeGeometry {
                id: 2,
                points: vec![
                    Point { x: 70.0, y: 80.0 },
                    Point { x: 90.0, y: 80.0 },
                    Point { x: 90.0, y: 160.0 },
                    Point { x: 190.0, y: 160.0 },
                    Point { x: 190.0, y: 130.0 },
                    Point { x: 200.0, y: 130.0 },
                ],
            },
            EdgeGeometry {
                id: 3,
                points: vec![
                    Point { x: 220.0, y: 30.0 },
                    Point { x: 230.0, y: 30.0 },
                    Point { x: 230.0, y: 0.0 },
                    Point { x: 40.0, y: 0.0 },
                    Point { x: 40.0, y: 80.0 },
                    Point { x: 50.0, y: 80.0 },
                ],
            },
        ],
        boundary_bundles: Vec::new(),
        width: 240.0,
        height: 160.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.split_feedback_nets, 1);
    assert_eq!(report.feedback_net_count, 1);

    let mut one_sided = layout.clone();
    one_sided.edges[2].points = vec![
        Point { x: 70.0, y: 80.0 },
        Point { x: 90.0, y: 80.0 },
        Point { x: 90.0, y: 0.0 },
        Point { x: 190.0, y: 0.0 },
        Point { x: 190.0, y: 130.0 },
        Point { x: 200.0, y: 130.0 },
    ];
    let one_sided_report = score(&graph, &one_sided, ScoreOptions::default());
    assert_eq!(
        one_sided_report.semantic_violations, 0,
        "{one_sided_report:#?}"
    );
    assert_eq!(one_sided_report.split_feedback_nets, 0);
    assert_eq!(one_sided_report.feedback_net_count, 1);
}

#[test]
fn detects_overlapping_nodes() {
    let graph = graph();
    let mut layout = layout(&graph, LayoutOptions::default()).unwrap();
    layout.nodes[1].x = layout.nodes[0].x;
    layout.nodes[1].y = layout.nodes[0].y;
    let report = score(&graph, &layout, ScoreOptions::default());
    assert_eq!(report.node_overlaps, 1);
    assert!(!report.passes_hard_gates());
}

fn quality_regression_graph(seed: u64) -> Graph {
    fn next(state: &mut u64) -> u64 {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        *state
    }

    let mut state = seed + 1;
    let nodes = (0u32..15)
        .map(|id| {
            let height = 36.0 + (next(&mut state) % 5) as f64 * 7.0;
            Node {
                id,
                width: 64.0 + (id % 3) as f64 * 9.0,
                height,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 6.0 + (next(&mut state) % (height as u64 - 11)) as f64,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 6.0 + (next(&mut state) % (height as u64 - 11)) as f64,
                    },
                ],
            }
        })
        .collect();

    let mut edges = Vec::new();
    for &(lo, hi, target_lo, target_hi, threshold) in &[
        (0u32, 5u32, 5u32, 10u32, 38u64),
        (5, 10, 10, 15, 38),
        (0, 5, 10, 15, 18),
    ] {
        for from in lo..hi {
            for to in target_lo..target_hi {
                if next(&mut state) % 100 < threshold {
                    let id = edges.len() as u32;
                    edges.push(Edge {
                        id,
                        source: Endpoint {
                            node: from,
                            port: 1,
                        },
                        target: Endpoint { node: to, port: 0 },
                        net: id,
                        participates_in_ranking: true,
                    });
                }
            }
        }
    }

    Graph { nodes, edges }
}

#[test]
fn candidate_selection_never_increases_canonical_crossings_over_baseline() {
    let graph = quality_regression_graph(172);
    let selected = layout(&graph, LayoutOptions::default()).unwrap();
    let report = score(&graph, &selected, ScoreOptions::default());

    assert!(report.passes_hard_gates(), "{report:#?}");
    assert!(report.crossings <= 13, "{report:#?}");
}

#[test]
fn rejects_a_diagonal_and_wrong_fixed_endpoint() {
    let graph = graph();
    let mut layout = layout(&graph, LayoutOptions::default()).unwrap();
    layout.edges[0].points[0].x += 3.0;
    layout.edges[0].points[1].y += 2.0;
    let report = score(&graph, &layout, ScoreOptions::default());
    assert!(!report.passes_hard_gates());
    assert!(
        report
            .examples
            .iter()
            .any(|item| item.kind == ViolationKind::WrongEndpoint)
    );
    assert!(
        report
            .examples
            .iter()
            .any(|item| item.kind == ViolationKind::NonOrthogonal)
    );
}

#[test]
fn detects_a_route_through_a_node_interior() {
    let graph = graph();
    let mut layout = layout(&graph, LayoutOptions::default()).unwrap();
    let source = layout.nodes.iter().find(|node| node.id == 1).unwrap();
    let target = layout.nodes.iter().find(|node| node.id == 2).unwrap();
    layout.edges[0].points = vec![
        schemweave::Point {
            x: source.x + source.width,
            y: source.y + 25.0,
        },
        schemweave::Point {
            x: target.x + target.width,
            y: source.y + 25.0,
        },
        schemweave::Point {
            x: target.x,
            y: target.y + 25.0,
        },
    ];
    let report = score(&graph, &layout, ScoreOptions::default());
    assert!(report.node_intersections > 0 || report.semantic_violations > 0);
}

#[test]
fn boundary_rounding_does_not_count_as_a_node_intersection() {
    let graph = Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 0.2,
                height: 1.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 0.5,
                }],
            },
            Node {
                id: 2,
                width: 0.2,
                height: 1.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 0.5,
                }],
            },
        ],
        edges: vec![Edge {
            id: 1,
            source: Endpoint { node: 1, port: 0 },
            target: Endpoint { node: 2, port: 0 },
            net: 1,
            participates_in_ranking: true,
        }],
    };
    let layout = Layout {
        nodes: vec![
            NodeGeometry {
                id: 1,
                x: 0.1,
                y: 0.0,
                width: 0.2,
                height: 1.0,
            },
            NodeGeometry {
                id: 2,
                x: 1.0,
                y: 0.0,
                width: 0.2,
                height: 1.0,
            },
        ],
        edges: vec![EdgeGeometry {
            id: 1,
            points: vec![Point { x: 0.3, y: 0.5 }, Point { x: 1.0, y: 0.5 }],
        }],
        boundary_bundles: Vec::new(),
        width: 1.2,
        height: 1.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());
    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.node_intersections, 0, "{report:#?}");
}

#[test]
fn caps_examples_without_hiding_violation_counts() {
    let graph = graph();
    let mut layout = layout(&graph, LayoutOptions::default()).unwrap();
    layout.width = -1.0;
    let report = score(
        &graph,
        &layout,
        ScoreOptions {
            max_examples: 1,
            ..ScoreOptions::default()
        },
    );
    assert!(report.semantic_violations > report.examples.len());
    assert_eq!(report.examples.len(), 1);
}

#[test]
fn scores_the_full_consumer_bound() {
    let node_count = 2_000u32;
    let graph = Graph {
        nodes: (0..node_count)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 50.0,
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
                        offset: 25.0,
                    },
                ],
            })
            .collect(),
        edges: (0..10_000u32)
            .map(|id| {
                let source = id % node_count;
                Edge {
                    id,
                    source: Endpoint {
                        node: source,
                        port: 1,
                    },
                    target: Endpoint {
                        node: (source + 1 + id / node_count) % node_count,
                        port: 0,
                    },
                    net: source,
                    participates_in_ranking: true,
                }
            })
            .collect(),
    };
    let layout = layout(&graph, LayoutOptions::default()).unwrap();
    let report = score(&graph, &layout, ScoreOptions::default());
    assert_eq!(report.semantic_violations, 0);
    assert_eq!(report.node_overlaps, 0);
    assert_eq!(report.node_intersections, 0);
    assert_eq!(report.unrelated_overlaps, 0);
    assert_eq!(report.unrelated_contacts, 0);
    assert_eq!(report.forward_edge_count, 0);
    assert_eq!(report.ranking_direction_violations, 0);
    assert_eq!(report.reverse_x_length, 0.0);
    assert!(!report.edge_node_clearance_exhausted);
    assert!(report.minimum_edge_node_clearance.is_some());
    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(
        report.segments,
        layout
            .edges
            .iter()
            .map(|edge| edge.points.len() - 1)
            .sum::<usize>()
    );
}

#[test]
fn counts_overlapping_same_net_branches_as_one_physical_crossing() {
    let small_node = |id, side| Node {
        id,
        width: 10.0,
        height: 10.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side,
            offset: 5.0,
        }],
    };
    let graph = Graph {
        nodes: vec![
            small_node(1, PortSide::East),
            small_node(2, PortSide::West),
            small_node(3, PortSide::South),
            small_node(4, PortSide::North),
        ],
        edges: vec![
            Edge {
                id: 1,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 1,
                participates_in_ranking: true,
            },
            Edge {
                id: 2,
                source: Endpoint { node: 3, port: 0 },
                target: Endpoint { node: 4, port: 0 },
                net: 2,
                participates_in_ranking: true,
            },
            Edge {
                id: 3,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 1,
                participates_in_ranking: true,
            },
        ],
    };
    let layout = Layout {
        nodes: vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 40.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 2,
                x: 90.0,
                y: 40.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 3,
                x: 45.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 4,
                x: 45.0,
                y: 90.0,
                width: 10.0,
                height: 10.0,
            },
        ],
        edges: vec![
            EdgeGeometry {
                id: 1,
                points: vec![Point { x: 10.0, y: 45.0 }, Point { x: 90.0, y: 45.0 }],
            },
            EdgeGeometry {
                id: 2,
                points: vec![Point { x: 50.0, y: 10.0 }, Point { x: 50.0, y: 90.0 }],
            },
            EdgeGeometry {
                id: 3,
                points: vec![Point { x: 10.0, y: 45.0 }, Point { x: 90.0, y: 45.0 }],
            },
        ],
        boundary_bundles: Vec::new(),
        width: 100.0,
        height: 100.0,
    };
    let report = score(&graph, &layout, ScoreOptions::default());
    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.crossings, 1);
    assert_eq!(report.route_length, 160.0);
    assert!((report.shared_route_ratio - 1.0 / 3.0).abs() < 1e-12);
    assert_eq!(report.bends, 0);
    assert_eq!(report.unrelated_contacts, 0);
}

#[test]
fn shared_same_net_corners_count_once() {
    let graph = Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 10.0,
                height: 10.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 5.0,
                }],
            },
            Node {
                id: 2,
                width: 10.0,
                height: 10.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 5.0,
                }],
            },
        ],
        edges: vec![
            Edge {
                id: 1,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 7,
                participates_in_ranking: true,
            },
            Edge {
                id: 2,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 7,
                participates_in_ranking: true,
            },
        ],
    };
    let points = vec![
        Point { x: 10.0, y: 5.0 },
        Point { x: 50.0, y: 5.0 },
        Point { x: 50.0, y: 45.0 },
        Point { x: 90.0, y: 45.0 },
    ];
    let layout = Layout {
        nodes: vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 2,
                x: 90.0,
                y: 40.0,
                width: 10.0,
                height: 10.0,
            },
        ],
        edges: vec![
            EdgeGeometry {
                id: 1,
                points: points.clone(),
            },
            EdgeGeometry { id: 2, points },
        ],
        boundary_bundles: Vec::new(),
        width: 100.0,
        height: 50.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.route_length, 120.0);
    assert_eq!(report.shared_route_ratio, 0.5);
    assert_eq!(report.bends, 2);
}

#[test]
fn boundary_bundle_geometry_and_tap_endpoints_participate_in_scoring() {
    let graph = Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 1,
                    side: PortSide::East,
                    offset: 25.0,
                }],
            },
            Node {
                id: 2,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 25.0,
                }],
            },
            Node {
                id: 3,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 25.0,
                }],
            },
        ],
        edges: vec![
            Edge {
                id: 10,
                source: Endpoint { node: 1, port: 1 },
                target: Endpoint { node: 2, port: 0 },
                net: 10,
                participates_in_ranking: true,
            },
            Edge {
                id: 11,
                source: Endpoint { node: 1, port: 1 },
                target: Endpoint { node: 3, port: 0 },
                net: 11,
                participates_in_ranking: true,
            },
        ],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![2, 3],
        boundary_bundles: vec![
            BoundaryBundleConstraint {
                id: 3,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 2,
                members: vec![
                    BoundaryBundleMemberConstraint {
                        edge: 10,
                        slots: vec![0],
                    },
                    BoundaryBundleMemberConstraint {
                        edge: 11,
                        slots: vec![1],
                    },
                ],
            },
            BoundaryBundleConstraint {
                id: 4,
                endpoint: Endpoint { node: 2, port: 0 },
                width: 1,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 10,
                    slots: vec![0],
                }],
            },
        ],
    };
    let layout = layout_with_quality_effort_and_constraints(
        &graph,
        LayoutOptions::default(),
        QualityEffort::Quality,
        &constraints,
    )
    .unwrap();
    let report = score(
        &graph,
        &layout,
        ScoreOptions {
            edge_node_clearance_threshold: 0.0,
            ..ScoreOptions::default()
        },
    );
    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.node_intersections, 0, "{report:#?}");
    assert_eq!(report.edge_node_clearance_violations, 0, "{report:#?}");
    let bus_length = layout
        .boundary_bundles
        .iter()
        .flat_map(|bundle| [bundle.collector, bundle.spine])
        .map(|segment| {
            (segment.end.x - segment.start.x).abs() + (segment.end.y - segment.start.y).abs()
        })
        .sum::<f64>();
    let member_length = layout
        .edges
        .iter()
        .flat_map(|edge| edge.points.windows(2))
        .map(|pair| (pair[1].x - pair[0].x).abs() + (pair[1].y - pair[0].y).abs())
        .sum::<f64>();
    assert_eq!(report.route_length, member_length + bus_length);
}

#[test]
fn boundary_bundle_segments_participate_in_crossing_and_parallel_spacing_metrics() {
    let graph = graph();
    let layout = Layout {
        nodes: vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 80.0,
                height: 50.0,
            },
            NodeGeometry {
                id: 2,
                x: 120.0,
                y: 0.0,
                width: 80.0,
                height: 50.0,
            },
        ],
        edges: vec![EdgeGeometry {
            id: 10,
            points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 120.0, y: 25.0 }],
        }],
        boundary_bundles: vec![BoundaryBundleGeometry {
            id: 99,
            endpoint: Endpoint { node: 1, port: 1 },
            role: BoundaryBundleRole::Input,
            width: 1,
            collector: BoundaryBundleSegment {
                start: Point { x: 100.0, y: 0.0 },
                end: Point { x: 100.0, y: 50.0 },
            },
            spine: BoundaryBundleSegment {
                start: Point { x: 80.0, y: 29.0 },
                end: Point { x: 100.0, y: 29.0 },
            },
            members: Vec::new(),
        }],
        width: 200.0,
        height: 50.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.crossings, 1, "{report:#?}");
    assert_eq!(report.minimum_parallel_route_separation, Some(4.0));
}
