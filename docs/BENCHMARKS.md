# Benchmarks

`crates/schemweave/benches/layout.rs` measures `layout_with_config` and
`expand_group_in_place` over deterministic synthetic fixtures
(`benches/support/generators.rs`), across shapes (pipeline, fanout, fanin,
layered DAG, bus chain), sizes, and all three `QualityEffort` levels.
`tests/bench_fixtures.rs` guards generator determinism and fixture validity
in the normal test suite.

## Run

```bash
cargo bench --locked -p schemweave                              # full matrix
cargo bench --locked -p schemweave --bench layout -- 'layout/dag' # one group
cargo bench --locked -p schemweave --bench layout -- --test     # smoke: each bench once
```

Any run that passes flags or filters through to criterion must target
`--bench layout`: the library and integration-test targets use the default
libtest harness, which rejects criterion flags (e.g. `--save-baseline`), so a
bare `cargo bench -p schemweave -- <flag>` fails. The flag-free full-matrix run
above needs no `--bench layout`.

## Compare against a baseline

Performance changes must compare identical fixtures and effort levels and
report median and tail latency (see `.agent/Repo.md`).

```bash
# On the reference commit:
cargo bench --locked -p schemweave --bench layout -- --save-baseline before
# After the change:
cargo bench --locked -p schemweave --bench layout -- --baseline before
```

Criterion prints median estimates and change ratios. For tail latency,
derive per-iteration percentiles from the saved samples:

```bash
jq -r '[.times as $t | .iters as $i | range(0; $t | length) | $t[.] / $i[.]]
  | sort
  | "p50 \(.[length / 2 | floor])  p95 \(.[length * 19 / 20 | floor])  p99 \(.[length * 99 / 100 | floor])  (ns)"' \
  target/criterion/layout_dag/large-1500n/max/new/sample.json
```

(Adjust the path to the group/bench/parameter under `target/criterion/`;
criterion sanitizes `/` in group names to `_`.)

Baselines and reports live in `target/criterion` and are never committed.
Keep benchmark histories out of tracked docs; cite numbers in PR
descriptions instead.
