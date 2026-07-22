# SchemWeave Repository Rules

These workspace-level rules apply to every SchemWeave worktree. Start agent
sessions from `/home/leela/code/schemweave`. A worktree-local `AGENTS.md` is
only a bootstrap back to this parent workspace policy.

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

## Workspace

The workspace root is a container, not a checkout:

```text
/home/leela/code/schemweave/
  repo.git/       canonical bare repository
  main/           main branch and persistent .agents checkout
  <worktree>/     feature branches
```

Shared agent policy is pinned as the `main/.agents` submodule. Project-specific
policy is tracked separately at `main/.agent/Repo.md`. The parent `AGENTS.md`,
`CLAUDE.md`, and shared `.agent/` entries are symlinks into `main`; the parent
`.agent/Repo.md` symlink targets the project file instead of the submodule.

- Only `main` maintains a persistent `.agents` checkout. Leave it uninitialized
  in feature worktrees, except temporarily in a dedicated policy-update
  worktree.
- After a policy gitlink changes on `main`, run
  `git -C /home/leela/code/schemweave/main submodule update --init --checkout .agents`.
- For `claude_start`, use `main/.agents` as `policy_root`, the workspace parent
  as `workspace_root`, and `main/.agent/Repo.md` as `repo_policy_file`.
- Run bare-repository commands with
  `git --git-dir=/home/leela/code/schemweave/repo.git ...`.
- Run normal git commands with `git -C <worktree> ...`.
- Keep `main/` on `main`. Never use it for feature work.
- Fetch with prune and fast-forward `main` before creating new worktrees.
- Continue an existing task branch when it has a worktree; otherwise create a
  branch and same-named worktree from updated `main`.
- Merge pull requests with squash and remove merged feature worktrees after
  completion is verified.

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
