use schemweave::{
    Edge, EdgeGeometry, Endpoint, Graph, Layout, LayoutOptions, Node, NodeGeometry, Point, Port,
    PortSide, layout,
};
use schemweave_eval::{ScoreOptions, ViolationKind, score};

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
fn accepts_the_current_exact_port_baseline() {
    let graph = graph();
    let layout = layout(&graph, LayoutOptions::default()).unwrap();
    let report = score(&graph, &layout, ScoreOptions::default());
    assert!(report.passes_hard_gates(), "{report:#?}");
    assert_eq!(report.semantic_violations, 0);
    assert_eq!(report.node_overlaps, 0);
    assert_eq!(report.node_intersections, 0);
    assert!(report.segments > 0);
    assert!(report.route_length > 0.0);
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
    assert_eq!(
        report.segments,
        layout.edges.iter().map(|edge| edge.points.len() - 1).sum()
    );
}

#[test]
fn counts_one_unrelated_perpendicular_crossing() {
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
        ],
        width: 100.0,
        height: 100.0,
    };
    let report = score(&graph, &layout, ScoreOptions::default());
    assert_eq!(report.semantic_violations, 0, "{report:#?}");
    assert_eq!(report.crossings, 1);
    assert_eq!(report.unrelated_contacts, 0);
}
