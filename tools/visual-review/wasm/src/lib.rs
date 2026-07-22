#![forbid(unsafe_code)]

use schemweave::{Graph, Layout};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn review_layout_json(graph_json: &str, options_json: &str) -> Result<String, JsValue> {
    review_layout(graph_json, options_json).map_err(js_error)
}

#[wasm_bindgen]
pub fn score_json(graph_json: &str, layout_json: &str) -> Result<String, JsValue> {
    score_layout(graph_json, layout_json).map_err(js_error)
}

fn score_layout(graph_json: &str, layout_json: &str) -> Result<String, String> {
    let graph: Graph = decode(graph_json, "graph")?;
    let layout: Layout = decode(layout_json, "layout")?;
    serde_json::to_string(&schemweave_eval::score(
        &graph,
        &layout,
        schemweave_eval::ScoreOptions::default(),
    ))
    .map_err(|error| format!("failed to encode quality report: {error}"))
}

fn review_layout(graph_json: &str, options_json: &str) -> Result<String, String> {
    schemweave_wasm::layout_serialized(graph_json, options_json)
}

fn decode<T: serde::de::DeserializeOwned>(json: &str, subject: &str) -> Result<T, String> {
    serde_json::from_str(json).map_err(|error| format!("invalid {subject} JSON: {error}"))
}

fn js_error(message: impl AsRef<str>) -> JsValue {
    JsValue::from_str(message.as_ref())
}

#[cfg(test)]
mod tests {
    use schemweave::Layout;

    use super::{review_layout, score_layout};

    #[test]
    fn scores_an_empty_graph_and_layout() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        let quality =
            score_layout(graph, r#"{"nodes":[],"edges":[],"width":0.0,"height":0.0}"#).unwrap();
        assert!(quality.contains(r#""semantic_violations":0"#));
    }

    #[test]
    fn delegates_a_nonempty_layout_and_scores_the_same_identifiers() {
        let graph = r#"{
            "nodes":[
                {"id":7,"width":20.0,"height":20.0,"ports":[{"id":0,"side":"east","offset":10.0}]},
                {"id":9,"width":20.0,"height":20.0,"ports":[{"id":0,"side":"west","offset":10.0}]}
            ],
            "edges":[{
                "id":11,
                "source":{"node":7,"port":0},
                "target":{"node":9,"port":0},
                "net":3,
                "participates_in_ranking":true
            }]
        }"#;
        let layout_json = review_layout(graph, "{}").unwrap();
        let layout: Layout = serde_json::from_str(&layout_json).unwrap();
        assert_eq!(
            layout.nodes.iter().map(|node| node.id).collect::<Vec<_>>(),
            [7, 9]
        );
        assert_eq!(layout.edges[0].id, 11);

        let quality: schemweave_eval::QualityReport =
            serde_json::from_str(&score_layout(graph, &layout_json).unwrap()).unwrap();
        assert_eq!(quality.semantic_violations, 0);
        assert_eq!(quality.forward_edge_count, 1);
    }
}
