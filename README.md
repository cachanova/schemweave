# SchemWeave

SchemWeave is a deterministic Rust layout engine for directed circuit and
data-flow diagrams. Given sized nodes, fixed boundary ports, and directed
electrical nets, it produces left-to-right node geometry and orthogonal edge
routes. Native and WebAssembly consumers execute the same core implementation.

It is built for RTL and circuit schematics, port-based hardware block diagrams,
and other directed data-flow views. It is not a general-purpose replacement for
every ELK diagram family: arbitrary graph, UML, sequence, timing, and physical
floorplan layouts have different semantic and geometric requirements.

SchemWeave is developed and evaluated for
[SynthExplorer](https://github.com/cachanova/synth-explorer). Production
adoption remains subject to that application's visual and runtime gates.

The public API is `Graph` + `LayoutConfig` → `Layout`. The result contains node
rectangles, orthogonal edge polylines, optional boundary-bundle geometry, and
total size in the caller's layout units.

| Path | Purpose |
| --- | --- |
| `crates/schemweave` | Canonical layout and routing engine. |
| `crates/schemweave-wasm` | JSON WebAssembly boundary over the core. |
| `crates/schemweave-eval` | Development-only quality scorer. |
| `tools/visual-review` | Browser comparison harness. |

- [Usage](docs/USAGE.md): graph contract, Rust/WASM calls, and constraints.
- [Architecture](docs/ARCHITECTURE.md): pipeline and invariants.
- [Evaluation](docs/EVALUATION.md): quality gates and visual review.
- [Contributing](CONTRIBUTING.md): setup and checks.

Licensed under Apache-2.0.
