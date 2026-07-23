use schemweave::{
    Edge, EdgeGeometry, Endpoint, Graph, Layout, LayoutOptions, Node, NodeGeometry, Point, Port,
    PortSide, QualityEffort, layout, layout_with_quality_effort,
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
    let (top, top_geometry) = node(
        2,
        200.0,
        20.0,
        true,
        vec![Port {
            id: 0,
            side: PortSide::West,
            offset: 10.0,
        }],
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
        ],
        width: 220.0,
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
        width: 100.0,
        height: 50.0,
    };

    let report = score(&graph, &layout, ScoreOptions::default());

    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.route_length, 120.0);
    assert_eq!(report.shared_route_ratio, 0.5);
    assert_eq!(report.bends, 2);
}
