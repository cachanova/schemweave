#![forbid(unsafe_code)]

use schemweave::{Graph, Layout};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub fn review_layout_json(graph_json: &str, options_json: &str) -> Result<String, JsValue> {
    schemweave_wasm::layout_serialized(graph_json, options_json).map_err(js_error)
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

fn decode<T: serde::de::DeserializeOwned>(json: &str, subject: &str) -> Result<T, String> {
    serde_json::from_str(json).map_err(|error| format!("invalid {subject} JSON: {error}"))
}

fn js_error(message: impl AsRef<str>) -> JsValue {
    JsValue::from_str(message.as_ref())
}

#[cfg(test)]
mod tests {
    use super::score_layout;

    #[test]
    fn scores_an_empty_graph_and_layout() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        let quality =
            score_layout(graph, r#"{"nodes":[],"edges":[],"width":0.0,"height":0.0}"#).unwrap();
        assert!(quality.contains(r#""semantic_violations":0"#));
    }
}
