#![forbid(unsafe_code)]

use schemweave::{Graph, LayoutConstraints, LayoutOptions, QualityEffort};
use serde::Deserialize;
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
        SerializedLayoutOptions::default()
    } else {
        serde_json::from_str(options_json)
            .map_err(|error| format!("invalid options JSON: {error}"))?
    };
    let result = schemweave::layout_with_quality_effort_and_constraints(
        &graph,
        options.layout,
        options.quality_effort,
        &options.constraints,
    )
    .map_err(|error| error.to_string())?;
    serde_json::to_string(&result).map_err(|error| format!("failed to encode layout: {error}"))
}

#[derive(Deserialize)]
#[serde(default)]
struct SerializedLayoutOptions {
    #[serde(flatten)]
    layout: LayoutOptions,
    quality_effort: QualityEffort,
    #[serde(default)]
    constraints: LayoutConstraints,
}

impl Default for SerializedLayoutOptions {
    fn default() -> Self {
        Self {
            layout: LayoutOptions::default(),
            quality_effort: QualityEffort::Quality,
            constraints: LayoutConstraints::default(),
        }
    }
}

fn js_error(message: impl AsRef<str>) -> JsValue {
    JsValue::from_str(message.as_ref())
}

#[cfg(test)]
mod tests {
    use super::{layout_json, layout_serialized};
    use schemweave::{Edge, Endpoint, Graph, Layout, Node, Port, PortSide};

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
