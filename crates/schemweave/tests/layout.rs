use schemweave::{
    ConstrainedLayoutError, Edge, Endpoint, Graph, Layout, LayoutConstraintError,
    LayoutConstraints, LayoutError, LayoutOptions, Node, Port, PortSide, layout,
    layout_with_constraints,
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
fn unconstrained_layout_error_remains_exhaustively_source_compatible() {
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
    };
    let permuted_constraints = LayoutConstraints {
        inputs: vec![2, 1],
        outputs: vec![30, 20],
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
}

#[test]
fn serde_defaults_ranking_participation_to_true() {
    let edge: Edge = serde_json::from_str(
        r#"{"id":1,"source":{"node":1,"port":1},"target":{"node":2,"port":0},"net":7}"#,
    )
    .unwrap();
    assert!(edge.participates_in_ranking);
}
