#![forbid(unsafe_code)]

use schemweave::{Graph, LayoutOptions};
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
        LayoutOptions::default()
    } else {
        serde_json::from_str(options_json)
            .map_err(|error| format!("invalid options JSON: {error}"))?
    };
    let result = schemweave::layout(&graph, options).map_err(|error| error.to_string())?;
    serde_json::to_string(&result).map_err(|error| format!("failed to encode layout: {error}"))
}

fn js_error(message: impl AsRef<str>) -> JsValue {
    JsValue::from_str(message.as_ref())
}

#[cfg(test)]
mod tests {
    use super::{layout_json, layout_serialized};

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
    fn exposes_the_same_boundary_without_javascript_error_conversion() {
        let graph = r#"{"nodes":[],"edges":[]}"#;
        assert_eq!(
            layout_serialized(graph, "{}").unwrap(),
            layout_json(graph, "{}").unwrap()
        );
    }
}
