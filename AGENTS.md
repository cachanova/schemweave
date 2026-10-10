# SchemWeave

Shared rules live in `~/agents`. This file holds only SchemWeave facts.

## Product and contracts

SchemWeave is a deterministic Rust layout engine for directed circuit and
data-flow schematics. It targets native and WebAssembly consumers with fixed
ports, orthogonal routes, left-to-right logical flow, and responsive large-graph
layout as first-class requirements.

- `crates/schemweave/`: canonical layout and routing implementation.
- `crates/schemweave-wasm/`: browser binding for the same Rust core.
- `crates/schemweave-eval/`: development-only geometry and quality scorer.
- `tools/visual-review/`: corpus comparison, browser integration, and rendering
  harness.
- `docs/ARCHITECTURE.md`: current architecture and consumer boundary.

Keep the engine independent of Synth Explorer, DOM, JavaScript, HDL, and Yosys
models. Native and WebAssembly builds must execute the same core implementation.
Synth Explorer remains on ELK until the user completes a visual comparison and
explicitly approves adoption of SchemWeave.

Preserve stable-ID determinism, fixed boundary ports, axis-aligned routes,
obstacle avoidance, and explicit electrical-net identity. Proxy costs may select
bounded candidates, but a complete candidate must pass exact geometry scoring
before it replaces the current best layout.

## Quality and performance

- Preserve left-to-right data flow unless a feedback dependency requires a
  clearly separated return path.
- Treat readability as more than crossing count. Measure route separation,
  straight logical chains, local crossing density, bend concentration, wire
  stretch, area, and perimeter detours when evaluating routing changes.
- Keep search bounded by deterministic work counts rather than wall time.
- Avoid graph cloning, dense all-pairs visibility structures, and repeated full
  scoring when an exact incremental formulation is available.
- Use compact numeric indices and contiguous storage on hot paths.
- Keep large-graph interaction responsive in the browser worker. Cancellation,
  request supersession, and rendering remain consumer responsibilities unless a
  reviewed public engine contract explicitly changes that boundary.
- Store durable architecture and usage documentation in the repository. Keep
  transient benchmark histories and experiment logs out of tracked docs.

## Local development

Required toolchains are Rust 1.97.1, Node 24.11.1, npm 11.6.2, the
`wasm32-unknown-unknown` target, and `wasm-bindgen-cli` 0.2.122. Chromium is
required for browser verification.

```bash
cargo fmt --all -- --check
cargo test --workspace --locked
cargo clippy --workspace --locked --all-targets -- -D warnings
cargo check --locked --package schemweave-wasm --target wasm32-unknown-unknown
cargo build --locked --release --package schemweave-wasm --target wasm32-unknown-unknown

cd tools/visual-review
npm ci
npm test
npm run build
npm run test:browser
```

Run focused checks while iterating, then the complete relevant set before a PR.
For performance changes, compare identical fixtures and effort levels, report
median and tail latency, and prove output equivalence or explicitly quantify the
quality tradeoff. For quality changes, run exact hard gates and retain visual
comparison artifacts outside tracked documentation.

## PR workflow

- After implementation is complete and verified, commit, push, and open a PR
  without waiting for another request.
- Review correctness, performance and memory behavior, deterministic output,
  and test coverage before calling the PR ready.
- Do not merge a SchemWeave adoption change into Synth Explorer without the
  user's explicit visual approval, even when the engine PR itself is clean.
