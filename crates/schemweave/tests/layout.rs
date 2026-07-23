use schemweave::{
    BoundaryBundleConstraint, BoundaryBundleMemberConstraint, BoundaryBundleRole,
    ConstrainedLayoutError, Edge, EdgeNodeSegment, Endpoint, Graph, Layout, LayoutConfig,
    LayoutConstraintError, LayoutConstraints, LayoutError, LayoutOptions, NetNodeRelation, Node,
    Port, PortSide, QualityEffort, layout, layout_with_config, layout_with_constraints,
    measure_edge_node_clearance_bounded, place,
};

fn node(id: u32, cycle_breaker: bool) -> Node {
    Node {
        id,
        width: 80.0,
        height: 50.0,
        cycle_breaker,
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
    }
}

fn edge(id: u32, source: u32, target: u32) -> Edge {
    Edge {
        id,
        source: Endpoint {
            node: source,
            port: 1,
        },
        target: Endpoint {
            node: target,
            port: 0,
        },
        net: source,
        participates_in_ranking: true,
    }
}

fn assert_routes_avoid_node_interiors(result: &Layout) {
    for edge in &result.edges {
        for segment in edge.points.windows(2) {
            let (a, b) = (segment[0], segment[1]);
            for node in &result.nodes {
                let crosses = if a.x == b.x {
                    a.x > node.x
                        && a.x < node.x + node.width
                        && a.y.min(b.y) < node.y + node.height
                        && a.y.max(b.y) > node.y
                } else {
                    a.y > node.y
                        && a.y < node.y + node.height
                        && a.x.min(b.x) < node.x + node.width
                        && a.x.max(b.x) > node.x
                };
                assert!(!crosses, "edge {} crosses node {}", edge.id, node.id);
            }
        }
    }
}

fn assert_edge_node_clearance(graph: &Graph, result: &Layout, threshold: f64) {
    let nets = graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge.net))
        .collect::<std::collections::HashMap<_, _>>();
    let segments = result
        .edges
        .iter()
        .flat_map(|edge| {
            edge.points.windows(2).filter_map(|points| {
                let horizontal = points[0].y == points[1].y;
                let (start, end, fixed) = if horizontal {
                    (
                        points[0].x.min(points[1].x),
                        points[0].x.max(points[1].x),
                        points[0].y,
                    )
                } else {
                    (
                        points[0].y.min(points[1].y),
                        points[0].y.max(points[1].y),
                        points[0].x,
                    )
                };
                (start < end).then_some(EdgeNodeSegment {
                    net: nets[&edge.id],
                    horizontal,
                    fixed,
                    start,
                    end,
                })
            })
        })
        .collect::<Vec<_>>();
    let relations = graph
        .edges
        .iter()
        .flat_map(|edge| {
            [
                NetNodeRelation {
                    net: edge.net,
                    node: edge.source.node,
                },
                NetNodeRelation {
                    net: edge.net,
                    node: edge.target.node,
                },
            ]
        })
        .collect::<Vec<_>>();
    let measured = measure_edge_node_clearance_bounded(
        &segments,
        &result.nodes,
        &relations,
        threshold,
        1_000_000,
    )
    .unwrap();
    assert_eq!(measured.violations, 0);
    assert!(
        measured
            .minimum_clearance
            .is_none_or(|minimum| minimum >= threshold),
        "{measured:?}"
    );
}

#[test]
fn canonical_config_exposes_the_highest_quality_profile() {
    let config = LayoutConfig::highest_quality();

    assert_eq!(config.layout.edge_node_clearance, 20.0);
    assert_eq!(config.layout.minimum_parallel_wire_spacing, 0.0);
    assert_eq!(
        config.layout,
        LayoutOptions {
            route_lane_gap: 6.0,
            edge_node_clearance: 20.0,
            max_quality_area_factor: 2.0,
            max_quality_route_length_factor: 1.25,
            ..LayoutOptions::default()
        }
    );
    assert_eq!(config.quality_effort, QualityEffort::Max);
    assert_eq!(config.constraints, LayoutConstraints::default());
}

#[test]
fn public_placement_uses_the_same_effective_positive_clearance_spacing() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false)],
        edges: vec![edge(1, 1, 2)],
    };
    let options = LayoutOptions {
        edge_node_clearance: 40.0,
        ..LayoutOptions::default()
    };
    let placed = place(&graph, options).unwrap();
    let source = placed.iter().find(|node| node.id == 1).unwrap();
    let target = placed.iter().find(|node| node.id == 2).unwrap();

    assert_eq!(target.x - (source.x + source.width), 84.0);
}

#[test]
fn edge_node_clearance_defaults_to_disabled_and_rejects_invalid_values() {
    let options: LayoutOptions = serde_json::from_str("{}").unwrap();
    assert_eq!(options.edge_node_clearance, 0.0);
    let graph = Graph {
        nodes: vec![node(1, false)],
        edges: vec![],
    };
    for value in [
        f64::NAN,
        f64::INFINITY,
        -1.0,
        1_000_000.0 + f64::EPSILON * 1_000_000.0,
        f64::MAX,
    ] {
        let error = layout(
            &graph,
            LayoutOptions {
                edge_node_clearance: value,
                ..LayoutOptions::default()
            },
        )
        .unwrap_err();
        assert!(matches!(
            error,
            LayoutError::InvalidOption {
                field: "edge_node_clearance",
                value: invalid,
            } if (value.is_nan() && invalid.is_nan()) || value == invalid
        ));
    }
    assert!(
        layout(
            &graph,
            LayoutOptions {
                edge_node_clearance: 1_000_000.0,
                ..LayoutOptions::default()
            },
        )
        .is_ok()
    );
}

#[test]
fn parallel_wire_spacing_defaults_to_disabled_and_rejects_invalid_values() {
    let options: LayoutOptions = serde_json::from_str("{}").unwrap();
    assert_eq!(options.minimum_parallel_wire_spacing, 0.0);
    let graph = Graph {
        nodes: vec![node(1, false)],
        edges: vec![],
    };
    for value in [
        f64::NAN,
        f64::INFINITY,
        -1.0,
        1_000_000.0 + f64::EPSILON * 1_000_000.0,
        f64::MAX,
    ] {
        let error = layout(
            &graph,
            LayoutOptions {
                minimum_parallel_wire_spacing: value,
                ..LayoutOptions::default()
            },
        )
        .unwrap_err();
        assert!(matches!(
            error,
            LayoutError::InvalidOption {
                field: "minimum_parallel_wire_spacing",
                value: invalid,
            } if (value.is_nan() && invalid.is_nan()) || value == invalid
        ));
    }
    assert!(
        layout(
            &graph,
            LayoutOptions {
                minimum_parallel_wire_spacing: 1_000_000.0,
                ..LayoutOptions::default()
            },
        )
        .is_ok()
    );
}

#[test]
fn quality_budget_factors_have_stable_defaults_and_validate_their_lower_bound() {
    let options: LayoutOptions = serde_json::from_str("{}").unwrap();
    assert_eq!(options.max_quality_area_factor, 1.2);
    assert_eq!(options.max_quality_route_length_factor, 1.1);
    let graph = Graph {
        nodes: vec![node(1, false)],
        edges: vec![],
    };
    for (field, options) in [
        (
            "max_quality_area_factor",
            LayoutOptions {
                max_quality_area_factor: 0.99,
                ..LayoutOptions::default()
            },
        ),
        (
            "max_quality_route_length_factor",
            LayoutOptions {
                max_quality_route_length_factor: 0.99,
                ..LayoutOptions::default()
            },
        ),
    ] {
        assert!(matches!(
            layout(&graph, options),
            Err(LayoutError::InvalidOption {
                field: invalid,
                value: 0.99,
            }) if invalid == field
        ));
    }
}

#[test]
fn positive_edge_node_clearance_is_exact_and_permutation_deterministic() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(4, false),
            node(5, false),
        ],
        edges: vec![
            edge(10, 1, 2),
            edge(11, 1, 3),
            edge(12, 2, 4),
            edge(13, 3, 4),
            edge(14, 1, 5),
            edge(15, 5, 4),
        ],
    };
    let options = LayoutOptions {
        edge_node_clearance: 20.0,
        ..LayoutOptions::default()
    };
    let selected = layout(&graph, options).unwrap();
    assert_edge_node_clearance(&graph, &selected, 20.0);
    let highest = layout_with_config(&graph, &LayoutConfig::highest_quality()).unwrap();
    assert_edge_node_clearance(&graph, &highest, 20.0);

    let permuted = Graph {
        nodes: graph.nodes.iter().cloned().rev().collect(),
        edges: graph.edges.iter().cloned().rev().collect(),
    };
    assert_eq!(layout(&permuted, options).unwrap(), selected);
}

#[test]
fn positive_clearance_preserves_aligned_input_and_output_boundaries() {
    let mut wide_output = node(4, false);
    wide_output.width = 120.0;
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false), node(3, false), wide_output],
        edges: vec![edge(10, 1, 2), edge(11, 2, 3), edge(12, 2, 4)],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![3, 4],
        boundary_bundles: Vec::new(),
    };
    let options = LayoutOptions {
        edge_node_clearance: 20.0,
        ..LayoutOptions::default()
    };
    let result = layout_with_constraints(&graph, options, &constraints).unwrap();
    assert_edge_node_clearance(&graph, &result, 20.0);
    let input = result.nodes.iter().find(|node| node.id == 1).unwrap();
    assert!(result.nodes.iter().all(|node| node.x >= input.x));
    let output_right = result
        .nodes
        .iter()
        .filter(|node| constraints.outputs.contains(&node.id))
        .map(|node| node.x + node.width)
        .collect::<Vec<_>>();
    assert!(output_right.windows(2).all(|pair| pair[0] == pair[1]));
}

#[test]
fn highest_quality_clears_opposing_north_south_endpoint_escapes() {
    let source = |id| Node {
        id,
        width: 80.0,
        height: 50.0,
        cycle_breaker: false,
        ports: vec![
            Port {
                id: 0,
                side: PortSide::North,
                offset: 20.0,
            },
            Port {
                id: 1,
                side: PortSide::South,
                offset: 60.0,
            },
        ],
    };
    let sink = |id| Node {
        id,
        width: 80.0,
        height: 50.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side: PortSide::West,
            offset: 25.0,
        }],
    };
    let route = |id, from, port, to| Edge {
        id,
        source: Endpoint { node: from, port },
        target: Endpoint { node: to, port: 0 },
        net: id,
        participates_in_ranking: true,
    };
    let graph = Graph {
        nodes: vec![source(1), source(2), sink(3), sink(4), sink(5), sink(6)],
        edges: vec![
            route(10, 1, 0, 3),
            route(11, 1, 1, 4),
            route(12, 2, 0, 5),
            route(13, 2, 1, 6),
        ],
    };
    let result = layout_with_config(&graph, &LayoutConfig::highest_quality()).unwrap();
    assert_edge_node_clearance(&graph, &result, 20.0);
}

#[test]
fn configurable_clearance_expands_ordinary_same_rank_gaps() {
    let source = |id| Node {
        id,
        width: 80.0,
        height: 50.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side: PortSide::East,
            offset: 0.0,
        }],
    };
    let sink = |id| Node {
        id,
        width: 80.0,
        height: 50.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side: PortSide::West,
            offset: 0.0,
        }],
    };
    let graph = Graph {
        nodes: vec![source(1), source(2), sink(3), sink(4)],
        edges: vec![
            Edge {
                id: 10,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 3, port: 0 },
                net: 10,
                participates_in_ranking: true,
            },
            Edge {
                id: 11,
                source: Endpoint { node: 2, port: 0 },
                target: Endpoint { node: 4, port: 0 },
                net: 11,
                participates_in_ranking: true,
            },
        ],
    };
    let options = LayoutOptions {
        edge_node_clearance: 40.0,
        ..LayoutOptions::default()
    };
    let result = layout(&graph, options).unwrap();
    assert_edge_node_clearance(&graph, &result, 40.0);
}

#[test]
fn coincident_distinct_corner_ports_fail_with_typed_contact_error() {
    let isolated = |id| Node {
        id,
        width: 80.0,
        height: 60.0,
        cycle_breaker: false,
        ports: vec![],
    };
    let graph = Graph {
        nodes: vec![
            isolated(10),
            isolated(11),
            isolated(12),
            isolated(13),
            isolated(14),
            Node {
                id: 15,
                width: 80.0,
                height: 60.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 60.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::South,
                        offset: 80.0,
                    },
                ],
            },
            Node {
                id: 16,
                width: 80.0,
                height: 60.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 0.0,
                }],
            },
            Node {
                id: 19,
                width: 80.0,
                height: 60.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::South,
                    offset: 0.0,
                }],
            },
        ],
        edges: vec![
            Edge {
                id: 1,
                source: Endpoint { node: 15, port: 0 },
                target: Endpoint { node: 19, port: 0 },
                net: 1,
                participates_in_ranking: true,
            },
            Edge {
                id: 2,
                source: Endpoint { node: 15, port: 1 },
                target: Endpoint { node: 16, port: 0 },
                net: 2,
                participates_in_ranking: true,
            },
        ],
    };
    let options = LayoutOptions {
        edge_node_clearance: 40.0,
        ..LayoutOptions::default()
    };
    let result = std::panic::catch_unwind(|| {
        schemweave::layout_with_quality_effort(&graph, options, QualityEffort::Max)
    })
    .expect("positive-clearance layout must not panic");
    assert_eq!(result, Err(LayoutError::UnrelatedRouteContactUnsatisfied));
}

#[test]
fn positive_clearance_self_loop_exempts_only_its_endpoint_node() {
    let graph = Graph {
        nodes: vec![
            Node {
                id: 1,
                width: 80.0,
                height: 60.0,
                cycle_breaker: true,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 20.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::West,
                        offset: 40.0,
                    },
                ],
            },
            Node {
                id: 2,
                width: 80.0,
                height: 60.0,
                cycle_breaker: false,
                ports: Vec::new(),
            },
        ],
        edges: vec![Edge {
            id: 1,
            source: Endpoint { node: 1, port: 0 },
            target: Endpoint { node: 1, port: 1 },
            net: 1,
            participates_in_ranking: false,
        }],
    };
    let options = LayoutOptions {
        edge_node_clearance: 20.0,
        ..LayoutOptions::default()
    };
    let result =
        schemweave::layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();

    assert_edge_node_clearance(&graph, &result, 20.0);
    assert_eq!(result.edges.len(), 1);
    assert!(result.edges[0].points.len() >= 4);
}

#[test]
fn mixed_side_corner_port_contacts_fail_with_typed_error_deterministically() {
    fn rng(state: &mut u64) -> u64 {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        *state
    }
    let node = |id| Node {
        id,
        width: 80.0,
        height: 60.0,
        cycle_breaker: false,
        ports: vec![
            Port {
                id: 0,
                side: PortSide::West,
                offset: 0.0,
            },
            Port {
                id: 1,
                side: PortSide::West,
                offset: 60.0,
            },
            Port {
                id: 2,
                side: PortSide::East,
                offset: 0.0,
            },
            Port {
                id: 3,
                side: PortSide::East,
                offset: 60.0,
            },
            Port {
                id: 4,
                side: PortSide::North,
                offset: 0.0,
            },
            Port {
                id: 5,
                side: PortSide::North,
                offset: 80.0,
            },
            Port {
                id: 6,
                side: PortSide::South,
                offset: 0.0,
            },
            Port {
                id: 7,
                side: PortSide::South,
                offset: 80.0,
            },
        ],
    };
    let per = 5u32;
    let ranks = 4u32;
    for seed in [17u64, 25] {
        let nodes = (0..per * ranks).map(|index| node(index + 1)).collect();
        let mut state = seed;
        let mut id = 1u32;
        let mut edges = Vec::new();
        for rank in 0..ranks - 1 {
            for index in 0..per {
                for _ in 0..2 {
                    let jump = 1 + (rng(&mut state) % (ranks - 1 - rank) as u64) as u32;
                    let target = (rank + jump) * per + (rng(&mut state) % per as u64) as u32 + 1;
                    let source_port = (rng(&mut state) % 8) as u32;
                    let target_port = (rng(&mut state) % 8) as u32;
                    edges.push(Edge {
                        id,
                        source: Endpoint {
                            node: rank * per + index + 1,
                            port: source_port,
                        },
                        target: Endpoint {
                            node: target,
                            port: target_port,
                        },
                        net: id,
                        participates_in_ranking: true,
                    });
                    id += 1;
                }
            }
        }
        let graph = Graph { nodes, edges };
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            ..LayoutOptions::default()
        };
        let result = schemweave::layout_with_quality_effort(&graph, options, QualityEffort::Max);
        assert_eq!(result, Err(LayoutError::UnrelatedRouteContactUnsatisfied));
    }
}

#[test]
fn canonical_config_matches_the_explicit_layout_api() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false), node(3, false)],
        edges: vec![edge(10, 1, 2), edge(11, 1, 3), edge(12, 2, 3)],
    };
    let config = LayoutConfig {
        layout: LayoutOptions {
            layer_gap: 72.0,
            node_gap: 36.0,
            ordering_sweeps: 6,
            ..LayoutOptions::default()
        },
        quality_effort: QualityEffort::Max,
        constraints: LayoutConstraints::default(),
    };

    assert_eq!(
        layout_with_config(&graph, &config).unwrap(),
        schemweave::layout_with_quality_effort_and_constraints(
            &graph,
            config.layout,
            config.quality_effort,
            &config.constraints,
        )
        .unwrap()
    );
}

#[test]
fn is_deterministic_across_input_permutations() {
    let forward = Graph {
        nodes: vec![node(1, false), node(2, false), node(3, false)],
        edges: vec![edge(10, 1, 2), edge(11, 1, 3), edge(12, 2, 3)],
    };
    let reversed = Graph {
        nodes: forward.nodes.iter().cloned().rev().collect(),
        edges: forward.edges.iter().cloned().rev().collect(),
    };
    assert_eq!(
        layout(&forward, LayoutOptions::default()).unwrap(),
        layout(&reversed, LayoutOptions::default()).unwrap()
    );
}

#[test]
fn multi_terminal_sparse_net_shares_an_intermediate_backbone() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(4, false),
        ],
        edges: vec![
            Edge {
                net: 10,
                ..edge(10, 1, 2)
            },
            Edge {
                net: 11,
                ..edge(11, 2, 3)
            },
            Edge {
                net: 12,
                ..edge(12, 2, 4)
            },
            Edge {
                net: 7,
                ..edge(20, 1, 3)
            },
            Edge {
                net: 7,
                ..edge(21, 1, 4)
            },
        ],
    };

    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let reversed = Graph {
        nodes: graph.nodes.iter().cloned().rev().collect(),
        edges: graph.edges.iter().cloned().rev().collect(),
    };
    assert_eq!(result, layout(&reversed, LayoutOptions::default()).unwrap());
    let first = &result
        .edges
        .iter()
        .find(|edge| edge.id == 20)
        .unwrap()
        .points;
    let second = &result
        .edges
        .iter()
        .find(|edge| edge.id == 21)
        .unwrap()
        .points;
    let shares_backbone = first.windows(2).any(|left| {
        left[0].y == left[1].y
            && (left[1].x - left[0].x).abs() > LayoutOptions::default().layer_gap
            && second
                .windows(2)
                .any(|right| left[0] == right[0] && left[1] == right[1])
    });

    assert!(shares_backbone);
}

#[test]
fn returns_exact_orthogonal_port_routes_and_nonnegative_bounds() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false)],
        edges: vec![edge(10, 1, 2)],
    };
    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let source = result.nodes.iter().find(|node| node.id == 1).unwrap();
    let target = result.nodes.iter().find(|node| node.id == 2).unwrap();
    let route = &result.edges[0].points;
    assert_eq!(route.first().unwrap().x, source.x + source.width);
    assert_eq!(route.first().unwrap().y, source.y + 25.0);
    assert_eq!(route.last().unwrap().x, target.x);
    assert_eq!(route.last().unwrap().y, target.y + 20.0);
    assert!(
        route
            .windows(2)
            .all(|pair| pair[0].x == pair[1].x || pair[0].y == pair[1].y)
    );
    assert!(
        result
            .nodes
            .iter()
            .all(|node| node.x >= 0.0 && node.y >= 0.0)
    );
    assert!(route.iter().all(|point| point.x >= 0.0 && point.y >= 0.0));
    assert!(result.width >= target.x + target.width);
    assert!(result.height >= target.y + target.height);
}

#[test]
fn handles_all_port_sides_without_diagonal_segments() {
    let mut source = node(1, false);
    source.ports = vec![
        Port {
            id: 0,
            side: PortSide::North,
            offset: 20.0,
        },
        Port {
            id: 1,
            side: PortSide::South,
            offset: 60.0,
        },
    ];
    let mut target = node(2, false);
    target.ports = vec![
        Port {
            id: 0,
            side: PortSide::North,
            offset: 20.0,
        },
        Port {
            id: 1,
            side: PortSide::South,
            offset: 60.0,
        },
    ];
    let graph = Graph {
        nodes: vec![source, target],
        edges: vec![
            Edge {
                id: 1,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 1 },
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
    let result = layout(&graph, LayoutOptions::default()).unwrap();
    assert!(result.edges.iter().all(|edge| {
        edge.points
            .windows(2)
            .all(|pair| pair[0].x == pair[1].x || pair[0].y == pair[1].y)
    }));
    assert_routes_avoid_node_interiors(&result);
}

#[test]
fn outer_lane_baseline_avoids_intermediate_nodes() {
    let graph = Graph {
        nodes: (1..=8).map(|id| node(id, id == 1)).collect(),
        edges: vec![
            edge(1, 1, 2),
            edge(2, 1, 3),
            edge(3, 2, 4),
            edge(4, 3, 5),
            edge(5, 4, 6),
            edge(6, 5, 7),
            edge(7, 6, 8),
            edge(8, 7, 1),
        ],
    };
    let result = layout(&graph, LayoutOptions::default()).unwrap();
    assert_routes_avoid_node_interiors(&result);
}

#[test]
fn adjacent_layers_align_ports_and_route_straight_through_their_channel() {
    let mut source = node(1, false);
    source.ports[1].offset = 10.0;
    let mut target = node(2, false);
    target.ports[0].offset = 40.0;
    let graph = Graph {
        nodes: vec![source, target],
        edges: vec![edge(10, 1, 2)],
    };

    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let source = result.nodes.iter().find(|node| node.id == 1).unwrap();
    let target = result.nodes.iter().find(|node| node.id == 2).unwrap();
    let route = &result.edges[0].points;

    assert_eq!(source.y + 10.0, target.y + 40.0);
    assert_eq!(route.len(), 2);
    assert_eq!(
        result.height,
        result
            .nodes
            .iter()
            .map(|node| node.y + node.height)
            .fold(0.0, f64::max)
    );
    assert_routes_avoid_node_interiors(&result);
}

#[test]
fn uncontended_adjacent_route_stays_straight() {
    let source = node(1, false);
    let mut target = node(2, false);
    target.ports[0].offset = 25.0;
    let graph = Graph {
        nodes: vec![source, target],
        edges: vec![edge(10, 1, 2)],
    };

    let result = layout(&graph, LayoutOptions::default()).unwrap();
    assert_eq!(result.edges[0].points.len(), 2);
}

#[test]
fn large_multi_terminal_nets_keep_a_shared_outer_trunk() {
    let target_count = 301;
    let graph = Graph {
        nodes: (0..=target_count).map(|id| node(id, false)).collect(),
        edges: (1..=target_count)
            .map(|target| Edge {
                id: target,
                source: Endpoint { node: 0, port: 1 },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net: 0,
                participates_in_ranking: true,
            })
            .collect(),
    };

    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let source = result.nodes.iter().find(|node| node.id == 0).unwrap();
    let target_left = result
        .nodes
        .iter()
        .filter(|node| node.id != 0)
        .map(|node| node.x)
        .fold(f64::INFINITY, f64::min);
    let gap_middle = (source.x + source.width + target_left) / 2.0;
    for edge in &result.edges {
        assert!(
            edge.points
                .iter()
                .any(|point| { point.x > source.x + source.width && point.x < gap_middle }),
            "edge {} does not use the shared outer channel: {:?}",
            edge.id,
            edge.points,
        );
    }
    assert_routes_avoid_node_interiors(&result);
}

#[test]
fn long_forward_routes_weave_through_free_layer_space() {
    let mut source = node(1, false);
    source.ports[1].offset = 10.0;
    let middle_a = node(2, false);
    let middle_b = node(3, false);
    let mut target = node(4, false);
    target.ports[0].offset = 40.0;
    let graph = Graph {
        nodes: vec![source, middle_a, middle_b, target],
        edges: vec![
            edge(10, 1, 2),
            edge(11, 1, 3),
            edge(12, 2, 4),
            edge(13, 3, 4),
            edge(99, 1, 4),
        ],
    };

    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let route = &result
        .edges
        .iter()
        .find(|edge| edge.id == 99)
        .unwrap()
        .points;
    let min_node_y = result
        .nodes
        .iter()
        .map(|node| node.y)
        .fold(f64::INFINITY, f64::min);
    let max_node_y = result
        .nodes
        .iter()
        .map(|node| node.y + node.height)
        .fold(f64::NEG_INFINITY, f64::max);

    assert!(
        route
            .iter()
            .all(|point| point.y >= min_node_y && point.y <= max_node_y)
    );
    assert_routes_avoid_node_interiors(&result);
}

#[test]
fn cycle_breakers_keep_feedback_from_flattening_the_dataflow() {
    let graph = Graph {
        nodes: vec![node(1, true), node(2, false), node(3, false)],
        edges: vec![edge(10, 1, 2), edge(11, 2, 3), edge(12, 3, 1)],
    };
    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let x = |id| result.nodes.iter().find(|node| node.id == id).unwrap().x;
    assert!(x(1) < x(2));
    assert!(x(2) < x(3));
}

#[test]
fn root_inputs_rank_cycle_breakers_without_reopening_feedback() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, true), node(3, false)],
        edges: vec![edge(10, 1, 2), edge(11, 2, 3), edge(12, 3, 2)],
    };

    let result = layout(&graph, LayoutOptions::default()).unwrap();
    let x = |id| result.nodes.iter().find(|node| node.id == id).unwrap().x;

    assert!(x(1) < x(2));
    assert!(x(2) < x(3));
}

#[test]
fn pure_cycles_are_supported_without_recursion() {
    let count = 2_000u32;
    let graph = Graph {
        nodes: (0..count).map(|id| node(id, false)).collect(),
        edges: (0..count)
            .map(|id| edge(id, id, (id + 1) % count))
            .collect(),
    };
    let result = layout(&graph, LayoutOptions::default()).unwrap();
    assert_eq!(result.nodes.len(), count as usize);
    assert_eq!(result.edges.len(), count as usize);
}

#[test]
fn handles_the_full_consumer_graph_bound() {
    let node_count = 2_000u32;
    let graph = Graph {
        nodes: (0..node_count).map(|id| node(id, false)).collect(),
        edges: (0..10_000u32)
            .map(|id| {
                let (source, target) = if id == 0 {
                    (0, 1_000)
                } else if id == 1 {
                    (1_000, 1_999)
                } else if id < 2_002 {
                    // Direct edges to the critical-path sink have span two under earliest ranks
                    // and span one under latest ranks. This activates the value-gated large-graph
                    // candidate without relying on the small-graph fanout exception.
                    ((id - 2) % 1_000, 1_999)
                } else {
                    let offset = id - 2_002;
                    (offset % 1_000, 1_000 + (offset * 7 + offset / 1_000) % 999)
                };
                let mut edge = edge(id, source, target);
                if (2_002..2_302).contains(&id) {
                    edge.net = node_count;
                } else if (2_302..2_403).contains(&id) {
                    edge.net = node_count + 1;
                }
                edge
            })
            .collect(),
    };
    let result = layout(&graph, LayoutOptions::default()).unwrap();
    assert_eq!(result.nodes.len(), node_count as usize);
    assert_eq!(result.edges.len(), 10_000);
    assert!(result.edges.iter().all(|edge| edge.points.len() <= 32));
}

#[test]
fn rejects_invalid_graphs_before_layout() {
    let graph = Graph {
        nodes: vec![node(1, false)],
        edges: vec![edge(10, 1, 9)],
    };
    assert_eq!(
        layout(&graph, LayoutOptions::default()),
        Err(LayoutError::UnknownEndpointNode {
            edge: 10,
            role: "target",
            node: 9
        })
    );
}

#[test]
fn layout_errors_have_deterministic_public_classification() {
    fn classify(error: LayoutError) -> &'static str {
        match error {
            LayoutError::DuplicateNode(_)
            | LayoutError::DuplicatePort { .. }
            | LayoutError::DuplicateEdge(_)
            | LayoutError::InvalidNodeDimension { .. }
            | LayoutError::InvalidPortOffset { .. }
            | LayoutError::UnknownEndpointNode { .. }
            | LayoutError::UnknownEndpointPort { .. } => "graph",
            LayoutError::InvalidOption { .. } | LayoutError::TooManyOrderingSweeps(_) => "option",
            LayoutError::EdgeNodeClearanceUnsatisfied { .. }
            | LayoutError::EdgeNodeClearanceWorkLimitExceeded { .. }
            | LayoutError::UnrelatedRouteContactUnsatisfied
            | LayoutError::UnrelatedRouteContactWorkLimitExceeded { .. }
            | LayoutError::UnrelatedRouteContactSegmentLimitExceeded { .. }
            | LayoutError::ParallelWireSpacingUnsatisfied { .. }
            | LayoutError::ParallelWireSpacingWorkLimitExceeded { .. }
            | LayoutError::ParallelWireSpacingSegmentLimitExceeded { .. }
            | LayoutError::BoundaryBundleGeometryUnsatisfied
            | LayoutError::BoundaryBundleGeometryWorkLimitExceeded { .. } => "clearance",
        }
    }

    assert_eq!(classify(LayoutError::TooManyOrderingSweeps(17)), "option");
}

#[test]
fn boundary_constraints_preserve_acyclic_register_dataflow() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(10, false),
            node(20, true),
            node(30, false),
        ],
        edges: vec![
            edge(1, 1, 10),
            edge(2, 10, 20),
            edge(3, 2, 20),
            edge(4, 3, 20),
            edge(5, 20, 30),
        ],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1, 2, 3],
        outputs: vec![30],
        boundary_bundles: Vec::new(),
    };

    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let geometry = |id| result.nodes.iter().find(|node| node.id == id).unwrap();
    assert_eq!(geometry(1).x, geometry(2).x);
    assert_eq!(geometry(2).x, geometry(3).x);
    assert!(geometry(1).x < geometry(10).x);
    assert!(geometry(10).x < geometry(20).x);
    assert!(geometry(20).x < geometry(30).x);
    assert!(
        result
            .nodes
            .iter()
            .filter(|node| node.id != 30)
            .all(|node| node.x + node.width < geometry(30).x)
    );
    assert_routes_avoid_node_interiors(&result);
}

#[test]
fn constrained_outputs_align_right_edges_across_widths_and_depths() {
    let mut shallow_output = node(30, false);
    shallow_output.width = 35.0;
    let mut deep_output = node(40, false);
    deep_output.width = 125.0;
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(10, false),
            node(20, false),
            shallow_output,
            deep_output,
        ],
        edges: vec![
            edge(1, 1, 30),
            edge(2, 1, 10),
            edge(3, 10, 20),
            edge(4, 20, 40),
        ],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![30, 40],
        boundary_bundles: Vec::new(),
    };

    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let geometry = |id| result.nodes.iter().find(|node| node.id == id).unwrap();
    assert_eq!(
        geometry(30).x + geometry(30).width,
        geometry(40).x + geometry(40).width
    );
    let output_left = geometry(30).x.min(geometry(40).x);
    assert!(
        [1, 10, 20]
            .into_iter()
            .all(|id| geometry(id).x + geometry(id).width < output_left)
    );
}

#[test]
fn constrained_layout_is_deterministic_across_node_port_edge_and_role_permutations() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(10, false),
            node(20, false),
            node(30, false),
        ],
        edges: vec![
            edge(1, 1, 10),
            edge(2, 2, 10),
            edge(3, 10, 20),
            edge(4, 10, 30),
        ],
    };
    let mut permuted = graph.clone();
    permuted.nodes.reverse();
    for node in &mut permuted.nodes {
        node.ports.reverse();
    }
    permuted.edges.reverse();
    let constraints = LayoutConstraints {
        inputs: vec![1, 2],
        outputs: vec![20, 30],
        boundary_bundles: Vec::new(),
    };
    let permuted_constraints = LayoutConstraints {
        inputs: vec![2, 1],
        outputs: vec![30, 20],
        boundary_bundles: Vec::new(),
    };

    assert_eq!(
        layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap(),
        layout_with_constraints(&permuted, LayoutOptions::default(), &permuted_constraints)
            .unwrap()
    );
}

#[test]
fn constrained_internal_sources_do_not_share_the_input_rank() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(4, false),
        ],
        edges: vec![edge(1, 1, 3), edge(2, 2, 4)],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![3, 4],
        boundary_bundles: Vec::new(),
    };

    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let geometry = |id| result.nodes.iter().find(|node| node.id == id).unwrap();
    assert!(geometry(2).x > geometry(1).x);
}

#[test]
fn boundary_constraints_handle_empty_and_one_sided_graphs() {
    let empty = layout_with_constraints(
        &Graph {
            nodes: vec![],
            edges: vec![],
        },
        LayoutOptions::default(),
        &LayoutConstraints::default(),
    )
    .unwrap();
    assert_eq!(empty.width, 0.0);
    assert_eq!(empty.height, 0.0);

    let inputs = layout_with_constraints(
        &Graph {
            nodes: vec![node(1, false), node(2, false)],
            edges: vec![],
        },
        LayoutOptions::default(),
        &LayoutConstraints {
            inputs: vec![1, 2],
            outputs: vec![],
            boundary_bundles: Vec::new(),
        },
    )
    .unwrap();
    assert_eq!(inputs.nodes[0].x, inputs.nodes[1].x);

    let mut narrow = node(1, false);
    narrow.width = 35.0;
    let mut wide = node(2, false);
    wide.width = 125.0;
    let outputs = layout_with_constraints(
        &Graph {
            nodes: vec![narrow, wide],
            edges: vec![],
        },
        LayoutOptions::default(),
        &LayoutConstraints {
            inputs: vec![],
            outputs: vec![1, 2],
            boundary_bundles: Vec::new(),
        },
    )
    .unwrap();
    assert_eq!(
        outputs.nodes[0].x + outputs.nodes[0].width,
        outputs.nodes[1].x + outputs.nodes[1].width
    );
}

#[test]
fn invalid_boundary_constraints_are_rejected_deterministically() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false), node(3, false)],
        edges: vec![edge(1, 1, 2), edge(2, 2, 3)],
    };
    let options = LayoutOptions::default();

    assert_eq!(
        layout_with_constraints(
            &graph,
            options,
            &LayoutConstraints {
                inputs: vec![99],
                outputs: vec![],
                boundary_bundles: Vec::new(),
            }
        ),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::UnknownConstraintNode {
                boundary: "input",
                node: 99,
            }
        ))
    );
    assert_eq!(
        layout_with_constraints(
            &graph,
            options,
            &LayoutConstraints {
                inputs: vec![1, 1],
                outputs: vec![],
                boundary_bundles: Vec::new(),
            }
        ),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::DuplicateConstraintNode {
                boundary: "input",
                node: 1,
            }
        ))
    );
    assert_eq!(
        layout_with_constraints(
            &graph,
            options,
            &LayoutConstraints {
                inputs: vec![1, 2],
                outputs: vec![2],
                boundary_bundles: Vec::new(),
            }
        ),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::OverlappingConstraintNode(2)
        ))
    );
    assert_eq!(
        layout_with_constraints(
            &graph,
            options,
            &LayoutConstraints {
                inputs: vec![2],
                outputs: vec![],
                boundary_bundles: Vec::new(),
            }
        ),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::ConstrainedInputHasIncomingEdge(2)
        ))
    );
    assert_eq!(
        layout_with_constraints(
            &graph,
            options,
            &LayoutConstraints {
                inputs: vec![],
                outputs: vec![2],
                boundary_bundles: Vec::new(),
            }
        ),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::ConstrainedOutputHasOutgoingEdge(2)
        ))
    );
}

#[test]
fn empty_constraints_are_byte_identical_to_the_existing_api() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false), node(3, false)],
        edges: vec![edge(1, 1, 2), edge(2, 2, 3)],
    };
    assert_eq!(
        layout(&graph, LayoutOptions::default()).unwrap(),
        layout_with_constraints(
            &graph,
            LayoutOptions::default(),
            &LayoutConstraints::default()
        )
        .unwrap()
    );
    assert_eq!(
        serde_json::to_string(&LayoutConstraints::default()).unwrap(),
        r#"{"inputs":[],"outputs":[]}"#
    );
}

#[test]
fn input_boundary_bundle_emits_a_pitched_collector_and_unique_horizontal_taps() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(4, false),
        ],
        edges: vec![
            Edge {
                net: 10,
                ..edge(10, 1, 2)
            },
            Edge {
                net: 11,
                ..edge(11, 1, 3)
            },
            Edge {
                net: 12,
                ..edge(12, 1, 4)
            },
        ],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![2, 3, 4],
        boundary_bundles: vec![BoundaryBundleConstraint {
            id: 7,
            endpoint: Endpoint { node: 1, port: 1 },
            width: 8,
            members: vec![
                BoundaryBundleMemberConstraint {
                    edge: 12,
                    slots: vec![7],
                },
                BoundaryBundleMemberConstraint {
                    edge: 10,
                    slots: vec![0, 1, 2, 3],
                },
                BoundaryBundleMemberConstraint {
                    edge: 11,
                    slots: vec![4, 5, 6],
                },
            ],
        }],
    };

    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let bundle = &result.boundary_bundles[0];
    assert_eq!(bundle.id, 7);
    assert_eq!(bundle.role, BoundaryBundleRole::Input);
    assert_eq!(bundle.width, 8);
    assert_eq!(
        bundle
            .members
            .iter()
            .map(|member| member.edge)
            .collect::<Vec<_>>(),
        vec![10, 11, 12]
    );
    assert_eq!(bundle.collector.start.x, bundle.collector.end.x);
    assert_eq!(bundle.spine.start.y, bundle.spine.end.y);
    assert_eq!(
        bundle.members[1].tap.y - bundle.members[0].tap.y,
        LayoutOptions::default().route_lane_gap
    );
    for member in &bundle.members {
        let route = result
            .edges
            .iter()
            .find(|route| route.id == member.edge)
            .unwrap();
        assert_eq!(route.points[0], member.tap);
        assert_eq!(route.points[0].y, route.points[1].y);
        assert!(route.points[1].x > route.points[0].x);
    }
    let output_right = result
        .nodes
        .iter()
        .filter(|node| node.id != 1)
        .map(|node| node.x + node.width)
        .collect::<Vec<_>>();
    assert!(output_right.windows(2).all(|pair| pair[0] == pair[1]));

    let mut permuted_graph = graph.clone();
    permuted_graph.nodes.reverse();
    permuted_graph.edges.reverse();
    for node in &mut permuted_graph.nodes {
        node.ports.reverse();
    }
    let mut permuted_constraints = constraints.clone();
    permuted_constraints.outputs.reverse();
    permuted_constraints.boundary_bundles[0].members.reverse();
    for member in &mut permuted_constraints.boundary_bundles[0].members {
        member.slots.reverse();
    }
    assert_eq!(
        result,
        layout_with_constraints(
            &permuted_graph,
            LayoutOptions::default(),
            &permuted_constraints,
        )
        .unwrap()
    );

    let wider_pitch = layout_with_constraints(
        &graph,
        LayoutOptions {
            route_lane_gap: 6.0,
            ..LayoutOptions::default()
        },
        &constraints,
    )
    .unwrap();
    assert_eq!(
        wider_pitch.boundary_bundles[0].members[1].tap.y
            - wider_pitch.boundary_bundles[0].members[0].tap.y,
        6.0
    );
}

#[test]
fn output_boundary_bundle_routes_members_horizontally_into_unique_taps() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(4, false),
        ],
        edges: vec![edge(10, 1, 4), edge(11, 2, 4), edge(12, 3, 4)],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1, 2, 3],
        outputs: vec![4],
        boundary_bundles: vec![BoundaryBundleConstraint {
            id: 9,
            endpoint: Endpoint { node: 4, port: 0 },
            width: 3,
            members: vec![
                BoundaryBundleMemberConstraint {
                    edge: 10,
                    slots: vec![0],
                },
                BoundaryBundleMemberConstraint {
                    edge: 11,
                    slots: vec![1],
                },
                BoundaryBundleMemberConstraint {
                    edge: 12,
                    slots: vec![2],
                },
            ],
        }],
    };

    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let bundle = &result.boundary_bundles[0];
    assert_eq!(bundle.role, BoundaryBundleRole::Output);
    for member in &bundle.members {
        let route = result
            .edges
            .iter()
            .find(|route| route.id == member.edge)
            .unwrap();
        assert_eq!(route.points.last(), Some(&member.tap));
        assert_eq!(
            route.points[route.points.len() - 2].y,
            route.points.last().unwrap().y
        );
        assert!(route.points[route.points.len() - 2].x < member.tap.x);
    }
}

#[test]
fn same_net_fanout_with_identical_slots_shares_one_visible_tap_deterministically() {
    let graph = Graph {
        nodes: vec![
            node(1, false),
            node(2, false),
            node(3, false),
            node(4, false),
        ],
        edges: vec![
            Edge {
                net: 42,
                ..edge(10, 1, 2)
            },
            Edge {
                net: 42,
                ..edge(11, 1, 3)
            },
            Edge {
                net: 43,
                ..edge(12, 1, 4)
            },
        ],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![2, 3, 4],
        boundary_bundles: vec![BoundaryBundleConstraint {
            id: 8,
            endpoint: Endpoint { node: 1, port: 1 },
            width: 3,
            members: vec![
                BoundaryBundleMemberConstraint {
                    edge: 10,
                    slots: vec![0, 1],
                },
                BoundaryBundleMemberConstraint {
                    edge: 11,
                    slots: vec![0, 1],
                },
                BoundaryBundleMemberConstraint {
                    edge: 12,
                    slots: vec![2],
                },
            ],
        }],
    };
    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let bundle = &result.boundary_bundles[0];
    let tap = |edge| {
        bundle
            .members
            .iter()
            .find(|member| member.edge == edge)
            .unwrap()
            .tap
    };
    assert_eq!(tap(10), tap(11));
    assert_ne!(tap(10), tap(12));
    let distinct_taps = bundle
        .members
        .iter()
        .map(|member| (member.tap.x.to_bits(), member.tap.y.to_bits()))
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(distinct_taps.len(), 2);
    for edge in [10, 11] {
        assert_eq!(
            result
                .edges
                .iter()
                .find(|route| route.id == edge)
                .unwrap()
                .points[0],
            tap(edge)
        );
    }

    let mut permuted_graph = graph.clone();
    permuted_graph.nodes.reverse();
    permuted_graph.edges.reverse();
    for node in &mut permuted_graph.nodes {
        node.ports.reverse();
    }
    let mut permuted_constraints = constraints.clone();
    permuted_constraints.outputs.reverse();
    permuted_constraints.boundary_bundles[0].members.reverse();
    for member in &mut permuted_constraints.boundary_bundles[0].members {
        member.slots.reverse();
    }
    assert_eq!(
        result,
        layout_with_constraints(
            &permuted_graph,
            LayoutOptions::default(),
            &permuted_constraints,
        )
        .unwrap()
    );
}

#[test]
fn direct_alias_edge_can_use_input_and_output_bundle_taps_in_any_bundle_order() {
    let graph = Graph {
        nodes: vec![node(1, false), node(2, false)],
        edges: vec![edge(10, 1, 2)],
    };
    let input = BoundaryBundleConstraint {
        id: 3,
        endpoint: Endpoint { node: 1, port: 1 },
        width: 1,
        members: vec![BoundaryBundleMemberConstraint {
            edge: 10,
            slots: vec![0],
        }],
    };
    let output = BoundaryBundleConstraint {
        id: 7,
        endpoint: Endpoint { node: 2, port: 0 },
        width: 1,
        members: vec![BoundaryBundleMemberConstraint {
            edge: 10,
            slots: vec![0],
        }],
    };
    let constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![2],
        boundary_bundles: vec![output.clone(), input.clone()],
    };
    let result = layout_with_constraints(&graph, LayoutOptions::default(), &constraints).unwrap();
    let route = &result.edges[0];
    let input_tap = result
        .boundary_bundles
        .iter()
        .find(|bundle| bundle.role == BoundaryBundleRole::Input)
        .unwrap()
        .members[0]
        .tap;
    let output_tap = result
        .boundary_bundles
        .iter()
        .find(|bundle| bundle.role == BoundaryBundleRole::Output)
        .unwrap()
        .members[0]
        .tap;
    assert_eq!(route.points[0], input_tap);
    assert_eq!(route.points.last(), Some(&output_tap));
    assert!(route.points[1].x > input_tap.x);
    assert!(route.points[route.points.len() - 2].x < output_tap.x);

    let reordered = layout_with_constraints(
        &graph,
        LayoutOptions::default(),
        &LayoutConstraints {
            inputs: vec![1],
            outputs: vec![2],
            boundary_bundles: vec![input.clone(), output.clone()],
        },
    )
    .unwrap();
    assert_eq!(result, reordered);

    let mut swapped_input = input;
    swapped_input.id = 7;
    let mut swapped_output = output;
    swapped_output.id = 3;
    let swapped = layout_with_constraints(
        &graph,
        LayoutOptions::default(),
        &LayoutConstraints {
            inputs: vec![1],
            outputs: vec![2],
            boundary_bundles: vec![swapped_input, swapped_output],
        },
    )
    .unwrap();
    assert_eq!(result.nodes, swapped.nodes);
    assert_eq!(result.edges, swapped.edges);
    assert_eq!(result.width, swapped.width);
    assert_eq!(result.height, swapped.height);
}

#[test]
fn invalid_boundary_bundle_contracts_return_typed_deterministic_errors() {
    let mut graph = Graph {
        nodes: vec![node(1, false), node(2, false), node(3, false)],
        edges: vec![
            Edge {
                net: 10,
                ..edge(10, 1, 2)
            },
            Edge {
                net: 11,
                ..edge(11, 1, 3)
            },
        ],
    };
    let mut constraints = LayoutConstraints {
        inputs: vec![1],
        outputs: vec![2, 3],
        boundary_bundles: vec![BoundaryBundleConstraint {
            id: 4,
            endpoint: Endpoint { node: 1, port: 1 },
            width: 2,
            members: vec![
                BoundaryBundleMemberConstraint {
                    edge: 10,
                    slots: vec![0],
                },
                BoundaryBundleMemberConstraint {
                    edge: 11,
                    slots: vec![0],
                },
            ],
        }],
    };
    assert_eq!(
        layout_with_constraints(&graph, LayoutOptions::default(), &constraints),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::BoundaryBundleSlotConflict {
                bundle: 4,
                first_edge: 10,
                second_edge: 11,
                slot: 0,
            }
        ))
    );

    graph.edges[0].net = 42;
    graph.edges[1].net = 42;
    constraints.boundary_bundles[0].width = 3;
    constraints.boundary_bundles[0].members[0].slots = vec![0, 1];
    constraints.boundary_bundles[0].members[1].slots = vec![1, 2];
    assert_eq!(
        layout_with_constraints(&graph, LayoutOptions::default(), &constraints),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::BoundaryBundleSlotConflict {
                bundle: 4,
                first_edge: 10,
                second_edge: 11,
                slot: 1,
            }
        ))
    );

    constraints.boundary_bundles[0].width = 2;
    constraints.boundary_bundles[0].members[0].slots = vec![0];
    constraints.boundary_bundles[0].members[1].slots = vec![2];
    assert_eq!(
        layout_with_constraints(&graph, LayoutOptions::default(), &constraints),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::BoundaryBundleSlotOutOfRange {
                bundle: 4,
                edge: 11,
                slot: 2,
                width: 2,
            }
        ))
    );

    constraints.boundary_bundles[0].width = 1_000_001;
    assert_eq!(
        layout_with_constraints(&graph, LayoutOptions::default(), &constraints),
        Err(ConstrainedLayoutError::Constraint(
            LayoutConstraintError::InvalidBoundaryBundleWidth {
                bundle: 4,
                width: 1_000_001,
            }
        ))
    );
}

#[test]
fn serde_defaults_ranking_participation_to_true() {
    let edge: Edge = serde_json::from_str(
        r#"{"id":1,"source":{"node":1,"port":1},"target":{"node":2,"port":0},"net":7}"#,
    )
    .unwrap();
    assert!(edge.participates_in_ranking);
}
