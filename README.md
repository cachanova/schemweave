# SchemWeave

SchemWeave is a deterministic Rust layout engine for directed circuit and
data-flow diagrams. It targets native applications and WebAssembly, with fixed
ports, orthogonal routes, and responsive layouts for large graphs as first-class
requirements.

The engine is under active development. Its first production consumer will be
[Synth Explorer](https://github.com/cachanova/synth-explorer). Until the engine
meets that application's routing-quality gates, Synth Explorer continues to use
its existing layout engine.

The repository contains the native `schemweave` core and the
`schemweave-wasm` browser binding. See [the architecture](docs/ARCHITECTURE.md)
for the current pipeline and production-routing direction.

Licensed under Apache-2.0.
