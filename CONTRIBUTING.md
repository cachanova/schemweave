# Contributing

SchemWeave is a Rust workspace with a TypeScript browser-review harness. Keep
the core independent of application, DOM, HDL, and synthesis-tool models.

## Prerequisites

- Rust 1.97.1 (pinned by `rust-toolchain.toml`)
- the `wasm32-unknown-unknown` Rust target
- `wasm-bindgen-cli` 0.2.122
- Node 24.11.1 and npm 11.6.2
- Chromium for browser verification

Install the Rust WebAssembly prerequisites once:

```bash
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.122 --locked
```

Install browser-review dependencies from its directory:

```bash
cd tools/visual-review
npm ci
```

## Verify a change

Run focused tests while iterating. Before opening a pull request, run the full
set applicable to the workspace:

```bash
cargo fmt --all -- --check
cargo test --workspace --locked
cargo clippy --workspace --locked --all-targets -- -D warnings
cargo check --locked --package schemweave-wasm --target wasm32-unknown-unknown
cargo build --locked --release --package schemweave-wasm --target wasm32-unknown-unknown

cd tools/visual-review
npm test
npm run build
npm run test:browser
```

For a documentation-only change, also inspect rendered Markdown and verify all
local links. Run `cargo doc --workspace --no-deps` when editing Rustdoc or
public API documentation.

## Documentation changes

Update the relevant guide when a public graph/configuration field, response,
error, quality metric, or WASM operation changes. Compile examples before
including them. Preserve deterministic IDs, fixed ports, axis-aligned routes,
explicit net identity, and hard clearance/separation contracts.
