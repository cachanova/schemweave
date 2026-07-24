# Usage

Native Rust and WebAssembly callers submit the same serializable graph contract
and receive the same geometry.

## Graph contract

| Type | Meaning |
| --- | --- |
| `Node { id, width, height, ports }` | A sized rectangle. |
| `Port { id, side, offset }` | A fixed boundary point; east/west offsets are from the top, north/south offsets from the left. |
| `Edge { id, source, target, net }` | A directed port-to-port connection. Same-net edges may share a physical trunk. |

All IDs must be stable and unique in their scope. Input order is ignored for
layout ordering and ties. Set `participates_in_ranking: false` for a routed edge
that must not constrain left-to-right flow. Mark an intended feedback boundary,
such as a register, with `cycle_breaker: true`; only actual feedback edges lose
their ranking constraint.

## Rust

The core crate is not currently published as a packaged Cargo release. Consume
the workspace directly or pin a reviewed Git revision:

```toml
[dependencies]
schemweave = { git = "https://github.com/cachanova/schemweave", rev = "<commit>" }
```

```rust
use schemweave::{
    Edge, Endpoint, Graph, LayoutConfig, Node, Port, PortSide, layout_with_config,
};

fn main() -> Result<(), schemweave::ConstrainedLayoutError> {
    let block = |id| Node {
        id,
        width: 80.0,
        height: 48.0,
        cycle_breaker: false,
        ports: vec![
            Port { id: 0, side: PortSide::West, offset: 24.0 },
            Port { id: 1, side: PortSide::East, offset: 24.0 },
        ],
    };

    let graph = Graph {
        nodes: vec![block(1), block(2)],
        edges: vec![Edge {
            id: 1,
            source: Endpoint { node: 1, port: 1 },
            target: Endpoint { node: 2, port: 0 },
            net: 1,
            participates_in_ranking: true,
        }],
    };

    let layout = layout_with_config(&graph, &LayoutConfig::default())?;
    assert_eq!(layout.edges.len(), 1);
    Ok(())
}
```

`Layout` contains node rectangles, edge point lists, optional boundary-bundle
geometry, and total width/height in the caller's layout units. A node's `x/y` is
its top-left corner; edge points are absolute ordered polyline coordinates. Port
positions are north `(x + offset, y)`, east `(x + width, y + offset)`, south
`(x + offset, y + height)`, and west `(x, y + offset)`.

## Configure a request

`LayoutConfig::default()` uses `Quality` effort;
`LayoutConfig::highest_quality()` enables the larger bounded refinement set.

| Field | Purpose |
| --- | --- |
| `quality_effort` | `Fast`, `Quality` (default), or `Max` bounded work. |
| `layer_gap`, `node_gap`, `port_stub`, `route_lane_gap` | Construction spacing. |
| `edge_node_clearance` | Hard route-to-unrelated-node clearance. |
| `minimum_parallel_wire_spacing` | Hard different-net parallel-route separation. |
| `max_quality_area_factor`, `max_quality_route_length_factor` | Refinement budgets. |
| `ordering_sweeps` | Bounded layer-ordering work. |

Positive clearance or wire spacing is never silently weakened: an unsatisfied
request returns an error. `LayoutConstraints` can pin input nodes left, output
nodes right, and define boundary bundles. Constraint failures are returned as
`ConstrainedLayoutError::Constraint`; graph or geometry failures as
`ConstrainedLayoutError::Layout`.

## WASM

`schemweave-wasm::layout_json(graph_json, options_json)` converts JSON `Graph`
and `LayoutConfig` into JSON `Layout`; an empty options string selects defaults.
Build the `cdylib` for `wasm32-unknown-unknown` and run `wasm-bindgen` to produce
the browser module; `tools/visual-review/scripts/build-wasm.mjs` is the working
reference. Call it from a reusable Web Worker. The consumer owns cancellation,
caching, request supersession, and rendering.

`expand_group_json` accepts compact graph/layout JSON, expanded graph JSON, and
expansion metadata. A successful call returns either `status: "layout"` or
`status: "needs_full_relayout"` with `geometry`, `work_limit`, or
`preserved_geometry_too_large` as its reason; request a normal layout for the
latter. Malformed input and other contract failures reject with an error.

The model is optimized for directed, port-based data-flow. See
[Architecture](ARCHITECTURE.md) and [Evaluation](EVALUATION.md).
