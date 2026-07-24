#![forbid(unsafe_code)]

use schemweave::{
    Graph, GroupExpansion, GroupExpansionError, GroupExpansionOptions, Layout, LayoutConfig,
};
use serde::Serialize;
use wasm_bindgen::prelude::*;

/// Lay out a graph through a compact JSON boundary suitable for Web Workers.
#[wasm_bindgen]
pub fn layout_json(graph_json: &str, options_json: &str) -> Result<String, JsValue> {
    layout_serialized(graph_json, options_json).map_err(js_error)
}

/// Execute the canonical JSON boundary without converting errors to JavaScript values.
pub fn layout_serialized(graph_json: &str, options_json: &str) -> Result<String, String> {
    let graph: Graph =
        serde_json::from_str(graph_json).map_err(|error| format!("invalid graph JSON: {error}"))?;
    let options = if options_json.trim().is_empty() {
        LayoutConfig::default()
    } else {
        serde_json::from_str(options_json)
            .map_err(|error| format!("invalid options JSON: {error}"))?
    };
    let result =
        schemweave::layout_with_config(&graph, &options).map_err(|error| error.to_string())?;
    serde_json::to_string(&result).map_err(|error| format!("failed to encode layout: {error}"))
}

/// Expand one compact group through the same JSON boundary used by Web Workers.
#[wasm_bindgen]
pub fn expand_group_json(
    compact_graph_json: &str,
    compact_layout_json: &str,
    expanded_graph_json: &str,
    expansion_json: &str,
    options_json: &str,
) -> Result<String, JsValue> {
    expand_group_serialized(
        compact_graph_json,
        compact_layout_json,
        expanded_graph_json,
        expansion_json,
        options_json,
    )
    .map_err(js_error)
}

/// Execute incremental group expansion without converting errors to JavaScript values.
pub fn expand_group_serialized(
    compact_graph_json: &str,
    compact_layout_json: &str,
    expanded_graph_json: &str,
    expansion_json: &str,
    options_json: &str,
) -> Result<String, String> {
    let compact_graph: Graph = serde_json::from_str(compact_graph_json)
        .map_err(|error| format!("invalid compact graph JSON: {error}"))?;
    let compact_layout: Layout = serde_json::from_str(compact_layout_json)
        .map_err(|error| format!("invalid compact layout JSON: {error}"))?;
    let expanded_graph: Graph = serde_json::from_str(expanded_graph_json)
        .map_err(|error| format!("invalid expanded graph JSON: {error}"))?;
    let expansion: GroupExpansion = serde_json::from_str(expansion_json)
        .map_err(|error| format!("invalid group expansion JSON: {error}"))?;
    let options = if options_json.trim().is_empty() {
        LayoutConfig::default()
    } else {
        serde_json::from_str(options_json)
            .map_err(|error| format!("invalid options JSON: {error}"))?
    };
    let result = match schemweave::expand_group_in_place(
        &compact_graph,
        &compact_layout,
        &expanded_graph,
        &expansion,
        &GroupExpansionOptions {
            layout: options.layout,
            quality_effort: options.quality_effort,
            constraints: options.constraints,
        },
    ) {
        Ok(layout) => SerializedGroupExpansionResult::Layout { layout },
        Err(GroupExpansionError::NeedsFullRelayout) => {
            SerializedGroupExpansionResult::NeedsFullRelayout {
                reason: FullRelayoutReason::Geometry,
            }
        }
        Err(GroupExpansionError::ExpansionWorkLimitExceeded { .. }) => {
            SerializedGroupExpansionResult::NeedsFullRelayout {
                reason: FullRelayoutReason::WorkLimit,
            }
        }
        Err(GroupExpansionError::PreservedGeometryTooLarge { .. }) => {
            SerializedGroupExpansionResult::NeedsFullRelayout {
                reason: FullRelayoutReason::PreservedGeometryTooLarge,
            }
        }
        Err(error) => return Err(error.to_string()),
    };
    serde_json::to_string(&result)
        .map_err(|error| format!("failed to encode expanded layout: {error}"))
}

#[derive(Serialize)]
#[serde(tag = "status", rename_all = "snake_case")]
enum SerializedGroupExpansionResult {
    Layout { layout: Layout },
    NeedsFullRelayout { reason: FullRelayoutReason },
}

#[derive(Serialize)]
#[serde(rename_all = "snake_case")]
enum FullRelayoutReason {
    Geometry,
    WorkLimit,
    PreservedGeometryTooLarge,
}

fn js_error(message: impl AsRef<str>) -> JsValue {
    JsValue::from_str(message.as_ref())
}

#[cfg(test)]
mod tests {
    use super::{expand_group_serialized, layout_json, layout_serialized};
    use schemweave::{
        BoundaryTrunk, Edge, Endpoint, Graph, GroupExpansion, Layout, LayoutOptions, Node, Port,
        PortSide, layout, layout_with_constraints,
    };

    fn decode_expanded_layout(response: String) -> Layout {
        let value: serde_json::Value = serde_json::from_str(&response).unwrap();
        assert_eq!(value["status"], "layout");
        serde_json::from_value(value["layout"].clone()).unwrap()
    }

    fn activating_graph_json() -> String {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..600)
            .map(|id| Node {
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
            })
            .collect();
        let mut state = 81;
        let mut endpoints = (8..24).map(|target| (0, target, 100)).collect::<Vec<_>>();
        for source in 1..8 {
            for target in 8..24 {
                if next(&mut state) % 100 < 24 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        for source in 8..24 {
            for target in 24..40 {
                if next(&mut state) % 100 < 20 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, net))| Edge {
                id: id as u32,
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
            })
            .collect();
        serde_json::to_string(&Graph { nodes, edges }).unwrap()
    }

    fn max_activating_graph_json() -> String {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..600)
            .map(|id| Node {
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
            })
            .collect();
        let mut state = 10;
        let mut endpoints = Vec::new();
        for layer in 0..3u32 {
            let source_start = layer * 50;
            let target_start = (layer + 1) * 50;
            for source in source_start..source_start + 50 {
                for target in target_start..target_start + 50 {
                    if next(&mut state) % 100 < 16 {
                        endpoints.push((source, target, source));
                    }
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, net))| Edge {
                id: id as u32,
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
            })
            .collect();
        serde_json::to_string(&Graph { nodes, edges }).unwrap()
    }

    fn node(id: u32) -> Node {
        Node {
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
        }
    }

    fn edge(id: u32, source: u32, target: u32, net: u32) -> Edge {
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
            net,
            participates_in_ranking: true,
        }
    }

    #[test]
    fn uses_default_options_for_an_empty_options_object() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        let result = layout_json(graph, "{}").unwrap();
        assert_eq!(
            result,
            r#"{"nodes":[],"edges":[],"width":0.0,"height":0.0}"#
        );
    }

    #[test]
    fn expands_a_group_over_the_json_boundary_with_max_effort() {
        let mut anchor = node(10);
        anchor.width = 260.0;
        let compact = Graph {
            nodes: vec![node(1), anchor, node(4)],
            edges: vec![edge(1, 1, 10, 100), edge(2, 10, 4, 200)],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(4)],
            edges: vec![
                edge(11, 1, 2, 100),
                edge(12, 2, 3, 150),
                edge(13, 3, 4, 200),
            ],
        };
        let result = decode_expanded_layout(
            expand_group_serialized(
                &serde_json::to_string(&compact).unwrap(),
                &serde_json::to_string(&compact_layout).unwrap(),
                &serde_json::to_string(&expanded).unwrap(),
                &serde_json::to_string(&GroupExpansion {
                    anchor: 10,
                    members: vec![2, 3],
                    boundary_trunks: vec![
                        BoundaryTrunk {
                            expanded_edge: 11,
                            compact_edge: 1,
                        },
                        BoundaryTrunk {
                            expanded_edge: 13,
                            compact_edge: 2,
                        },
                    ],
                })
                .unwrap(),
                r#"{"quality_effort":"max"}"#,
            )
            .unwrap(),
        );
        assert_eq!(
            result.nodes.iter().map(|node| node.id).collect::<Vec<_>>(),
            vec![1, 2, 3, 4]
        );
        assert_eq!(
            result.edges.iter().map(|edge| edge.id).collect::<Vec<_>>(),
            vec![11, 12, 13]
        );
    }

    #[test]
    fn expansion_honors_boundary_constraints_over_the_json_boundary() {
        let compact = Graph {
            nodes: vec![node(1), node(10), node(9)],
            edges: vec![edge(1, 1, 10, 100), edge(3, 1, 9, 300)],
        };
        let compact_layout = layout_with_constraints(
            &compact,
            LayoutOptions::default(),
            &schemweave::LayoutConstraints {
                inputs: vec![1],
                outputs: vec![9, 10],
                boundary_bundles: Vec::new(),
            },
        )
        .unwrap();
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(9)],
            edges: vec![edge(11, 1, 2, 101), edge(12, 1, 3, 102), edge(3, 1, 9, 300)],
        };
        let expansion = GroupExpansion {
            anchor: 10,
            members: vec![2, 3],
            boundary_trunks: vec![
                BoundaryTrunk {
                    expanded_edge: 11,
                    compact_edge: 1,
                },
                BoundaryTrunk {
                    expanded_edge: 12,
                    compact_edge: 1,
                },
            ],
        };
        let result = decode_expanded_layout(
            expand_group_serialized(
                &serde_json::to_string(&compact).unwrap(),
                &serde_json::to_string(&compact_layout).unwrap(),
                &serde_json::to_string(&expanded).unwrap(),
                &serde_json::to_string(&expansion).unwrap(),
                r#"{"quality_effort":"max","constraints":{"inputs":[1],"outputs":[2,3,9]}}"#,
            )
            .unwrap(),
        );
        let geometry = |id| result.nodes.iter().find(|node| node.id == id).unwrap();
        let right = |id| geometry(id).x + geometry(id).width;

        assert_eq!(right(2), right(3));
        assert_eq!(right(2), right(9));
        assert!(geometry(1).x + geometry(1).width < geometry(2).x);
    }

    #[test]
    fn compact_height_selects_stack_or_grid_over_the_json_boundary() {
        let compact = Graph {
            nodes: vec![node(10)],
            edges: Vec::new(),
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: Vec::new(),
        };
        let expansion = GroupExpansion {
            anchor: 10,
            members: vec![1, 2, 3],
            boundary_trunks: Vec::new(),
        };
        let expand = |layout: &Layout| {
            decode_expanded_layout(
                expand_group_serialized(
                    &serde_json::to_string(&compact).unwrap(),
                    &serde_json::to_string(layout).unwrap(),
                    &serde_json::to_string(&expanded).unwrap(),
                    &serde_json::to_string(&expansion).unwrap(),
                    "{}",
                )
                .unwrap(),
            )
        };

        let grid = expand(&compact_layout);
        let mut tall_compact_layout = compact_layout;
        tall_compact_layout.height = 200.0;
        let stack = expand(&tall_compact_layout);
        let distinct_x_positions = |layout: &Layout| {
            let mut positions = layout.nodes.iter().map(|node| node.x).collect::<Vec<_>>();
            positions.sort_unstable_by(f64::total_cmp);
            positions.dedup();
            positions.len()
        };

        assert!(distinct_x_positions(&grid) > 1);
        assert_eq!(distinct_x_positions(&stack), 1);
    }

    #[test]
    fn wider_expansion_returns_a_layout_and_contract_errors_remain_stable() {
        let compact = Graph {
            nodes: vec![node(1), node(10), node(4)],
            edges: vec![edge(1, 1, 10, 100), edge(2, 10, 4, 200)],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(4)],
            edges: vec![
                edge(11, 1, 2, 100),
                edge(12, 2, 3, 150),
                edge(13, 3, 4, 200),
            ],
        };
        let expansion = GroupExpansion {
            anchor: 10,
            members: vec![2, 3],
            boundary_trunks: vec![
                BoundaryTrunk {
                    expanded_edge: 11,
                    compact_edge: 1,
                },
                BoundaryTrunk {
                    expanded_edge: 13,
                    compact_edge: 2,
                },
            ],
        };
        let response: serde_json::Value = serde_json::from_str(
            &expand_group_serialized(
                &serde_json::to_string(&compact).unwrap(),
                &serde_json::to_string(&compact_layout).unwrap(),
                &serde_json::to_string(&expanded).unwrap(),
                &serde_json::to_string(&expansion).unwrap(),
                r#"{"quality_effort":"max"}"#,
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(response["status"], "layout");
        let layout: Layout = serde_json::from_value(response["layout"].clone()).unwrap();
        let first_member = layout.nodes.iter().find(|node| node.id == 2).unwrap();
        let second_member = layout.nodes.iter().find(|node| node.id == 3).unwrap();
        let output = layout.nodes.iter().find(|node| node.id == 4).unwrap();
        assert!(first_member.x < second_member.x);
        assert!(second_member.x < output.x);

        let mut invalid = expansion;
        invalid.boundary_trunks.clear();
        let error = expand_group_serialized(
            &serde_json::to_string(&compact).unwrap(),
            &serde_json::to_string(&compact_layout).unwrap(),
            &serde_json::to_string(&expanded).unwrap(),
            &serde_json::to_string(&invalid).unwrap(),
            r#"{"quality_effort":"max"}"#,
        )
        .unwrap_err();
        assert!(error.contains("has no compact trunk mapping"), "{error}");
    }

    #[test]
    fn expansion_work_limit_is_a_stable_full_relayout_status() {
        let compact = Graph {
            nodes: vec![node(1), node(10)],
            edges: vec![edge(1, 1, 10, 100)],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let members = (1_000..2_000).collect::<Vec<_>>();
        let edges = (0..4_000)
            .map(|index| edge(index + 100, 1, members[index as usize % members.len()], 100))
            .collect::<Vec<_>>();
        let expansion = GroupExpansion {
            anchor: 10,
            members: members.clone(),
            boundary_trunks: edges
                .iter()
                .map(|edge| BoundaryTrunk {
                    expanded_edge: edge.id,
                    compact_edge: 1,
                })
                .collect(),
        };
        let mut nodes = vec![node(1)];
        nodes.extend(members.into_iter().map(node));
        let expanded = Graph { nodes, edges };
        let response: serde_json::Value = serde_json::from_str(
            &expand_group_serialized(
                &serde_json::to_string(&compact).unwrap(),
                &serde_json::to_string(&compact_layout).unwrap(),
                &serde_json::to_string(&expanded).unwrap(),
                &serde_json::to_string(&expansion).unwrap(),
                r#"{"quality_effort":"fast"}"#,
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(
            response,
            serde_json::json!({
                "status":"needs_full_relayout",
                "reason":"work_limit"
            })
        );
    }

    #[test]
    fn accepts_each_quality_effort_over_the_json_boundary() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        for effort in ["fast", "quality", "max"] {
            let options = format!(r#"{{"quality_effort":"{effort}"}}"#);
            assert_eq!(
                layout_serialized(graph, &options).unwrap(),
                r#"{"nodes":[],"edges":[],"width":0.0,"height":0.0}"#
            );
        }
    }

    #[test]
    fn prior_nonempty_options_payloads_do_not_require_constraints() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        let options = r#"{"layer_gap":70.0,"node_gap":30.0,"port_stub":10.0,"route_lane_gap":4.0,"ordering_sweeps":2,"quality_effort":"fast"}"#;
        assert_eq!(
            layout_serialized(graph, options).unwrap(),
            r#"{"nodes":[],"edges":[],"width":0.0,"height":0.0}"#
        );
    }

    #[test]
    fn accepts_boundary_constraints_over_the_json_boundary() {
        let graph = Graph {
            nodes: (1..=3)
                .map(|id| Node {
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
                })
                .collect(),
            edges: vec![
                Edge {
                    id: 1,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 1,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 2,
                    source: Endpoint { node: 2, port: 1 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 2,
                    participates_in_ranking: true,
                },
            ],
        };
        let result: Layout = serde_json::from_str(
            &layout_serialized(
                &serde_json::to_string(&graph).unwrap(),
                r#"{"constraints":{"inputs":[1],"outputs":[3]}}"#,
            )
            .unwrap(),
        )
        .unwrap();
        let x = |id| result.nodes.iter().find(|node| node.id == id).unwrap().x;
        assert!(x(1) < x(2));
        assert!(x(2) < x(3));
    }

    #[test]
    fn accepts_boundary_bundle_constraints_and_emits_geometry_over_json() {
        let graph = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: vec![edge(10, 1, 2, 10), edge(11, 1, 3, 10)],
        };
        let options = r#"{
            "constraints": {
                "inputs": [1],
                "outputs": [2, 3],
                "boundary_bundles": [{
                    "id": 5,
                    "endpoint": {"node": 1, "port": 1},
                    "width": 1,
                    "members": [
                        {"edge": 10, "slots": [0]},
                        {"edge": 11, "slots": [0]}
                    ]
                }, {
                    "id": 6,
                    "endpoint": {"node": 2, "port": 0},
                    "width": 1,
                    "members": [
                        {"edge": 10, "slots": [0]}
                    ]
                }]
            }
        }"#;
        let value: serde_json::Value = serde_json::from_str(
            &layout_serialized(&serde_json::to_string(&graph).unwrap(), options).unwrap(),
        )
        .unwrap();
        assert_eq!(value["boundary_bundles"][0]["id"], 5);
        assert_eq!(value["boundary_bundles"][0]["role"], "input");
        assert_eq!(value["boundary_bundles"][0]["members"][0]["edge"], 10);
        assert_eq!(
            value["boundary_bundles"][0]["members"][0]["tap"],
            value["boundary_bundles"][0]["members"][1]["tap"]
        );
        assert_eq!(
            value["edges"][0]["points"][0],
            value["boundary_bundles"][0]["members"][0]["tap"]
        );
        assert_eq!(
            value["edges"][0]["points"]
                .as_array()
                .unwrap()
                .last()
                .unwrap(),
            &value["boundary_bundles"][1]["members"][0]["tap"]
        );
    }

    #[test]
    fn omitted_effort_uses_quality_on_an_activating_graph() {
        let graph = activating_graph_json();
        let omitted = layout_serialized(&graph, "{}").unwrap();
        let quality = layout_serialized(&graph, r#"{"quality_effort":"quality"}"#).unwrap();
        let fast = layout_serialized(&graph, r#"{"quality_effort":"fast"}"#).unwrap();
        assert_eq!(omitted, quality);
        assert_ne!(omitted, fast);
    }

    #[test]
    fn max_effort_changes_rust_selected_output_over_the_json_boundary() {
        let graph = max_activating_graph_json();
        let quality = layout_serialized(&graph, r#"{"quality_effort":"quality"}"#).unwrap();
        let max = layout_serialized(&graph, r#"{"quality_effort":"max"}"#).unwrap();
        assert_ne!(quality, max);
    }

    #[test]
    fn exposes_the_same_boundary_without_javascript_error_conversion() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        assert_eq!(
            layout_serialized(graph, "{}").unwrap(),
            layout_json(graph, "{}").unwrap()
        );
    }
}
