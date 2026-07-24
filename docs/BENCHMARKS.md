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
derive per-sample percentiles from the saved samples. Each criterion sample
is a batch mean (`times[i] / iters[i]`), not a single iteration, so these are
per-sample rather than per-iteration figures:

```bash
jq -r '[.times as $t | .iters as $i | range(0; $t | length) | $t[.] / $i[.]]
  | sort
  | "p50 \(.[length / 2 | floor])  p95 \(.[length * 19 / 20 | floor])  p99 \(.[length * 99 / 100 | floor])  (ns)"' \
  target/criterion/layout_dag/large-1500n/max/new/sample.json
```

(Adjust the path to the group/bench/parameter under `target/criterion/`;
criterion sanitizes `/` in group names to `_`.)

Tail resolution is bounded by the sample count: at the configured
`sample_size(20)` the p95 and p99 indices both floor to the largest sample, so
p95 == p99 — raise the sample size for finer tail resolution.

Baselines and reports live in `target/criterion` and are never committed.
Keep benchmark histories out of tracked docs; cite numbers in PR
descriptions instead.
