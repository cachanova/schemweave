use schemweave::{
    Edge, EdgeGeometry, Endpoint, Graph, Layout, LayoutOptions, Node, NodeGeometry, Point, Port,
    PortSide, layout,
};
use schemweave_eval::{QualityReport, ScoreOptions, ViolationKind, score};

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

#[test]
fn new_quality_fields_preserve_default_json_compatibility() {
    let options: ScoreOptions = serde_json::from_str("{}").unwrap();
    let report: QualityReport = serde_json::from_str("{}").unwrap();

    assert_eq!(options, ScoreOptions::default());
    assert_eq!(report, QualityReport::default());
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
    assert_eq!(report.bends, 2);
}
