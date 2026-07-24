mod support;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use schemweave::{Graph, LayoutConfig, QualityEffort, layout_with_config};
use support::generators;

#[allow(clippy::field_reassign_with_default)]
fn config(effort: QualityEffort) -> LayoutConfig {
    let mut config = LayoutConfig::default();
    config.quality_effort = effort;
    config
}

const EFFORTS: [(&str, QualityEffort); 3] = [
    ("fast", QualityEffort::Fast),
    ("quality", QualityEffort::Quality),
    ("max", QualityEffort::Max),
];

fn assert_reproducible(graph: &Graph, config: &LayoutConfig) {
    let first = layout_with_config(graph, config).expect("fixture must lay out");
    let second = layout_with_config(graph, config).expect("fixture must lay out");
    assert_eq!(
        first, second,
        "layout must be deterministic before measuring"
    );
}

fn bench_shape(c: &mut Criterion, shape: &str, fixtures: &[(&str, Graph)]) {
    let mut group = c.benchmark_group(format!("layout/{shape}"));
    for (label, graph) in fixtures {
        for (effort_label, effort) in EFFORTS {
            let config = config(effort);
            assert_reproducible(graph, &config);
            group.throughput(Throughput::Elements(graph.edges.len() as u64));
            group.bench_with_input(BenchmarkId::new(*label, effort_label), graph, |b, graph| {
                b.iter(|| layout_with_config(graph, &config).expect("layout"))
            });
        }
    }
    group.finish();
}

fn layout_benches(c: &mut Criterion) {
    bench_shape(
        c,
        "pipeline",
        &[
            ("small-30n", generators::pipeline(10, 3)),
            ("medium-120n", generators::pipeline(30, 4)),
        ],
    );
    bench_shape(
        c,
        "fanout",
        &[
            ("8", generators::fanout(8)),
            ("64", generators::fanout(64)),
            ("240", generators::fanout(240)),
        ],
    );
    bench_shape(c, "fanin", &[("64", generators::fanin(64))]);
    bench_shape(
        c,
        "dag",
        &[
            ("medium-300n", generators::layered_dag(12, 25, 7)),
            ("large-1500n", generators::layered_dag(25, 60, 7)),
        ],
    );
    bench_shape(
        c,
        "bus",
        &[
            ("4x8", generators::bus_chain(4, 8)),
            ("4x32", generators::bus_chain(4, 32)),
        ],
    );
}

use schemweave::{GroupExpansionOptions, expand_group_in_place};

fn expand_benches(c: &mut Criterion) {
    let mut group = c.benchmark_group("expand/chain");
    for (label, members, bystanders) in [("4m-6b", 4, 6), ("8m-40b", 8, 40)] {
        let (compact, expanded, expansion) = generators::expansion_pair(members, bystanders);
        for (effort_label, effort) in EFFORTS {
            let config = config(effort);
            assert_reproducible(&compact, &config);
            let compact_layout =
                layout_with_config(&compact, &config).expect("compact fixture must lay out");
            let options = GroupExpansionOptions {
                layout: config.layout,
                quality_effort: config.quality_effort,
                constraints: config.constraints.clone(),
            };
            let first =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("expansion");
            let second =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("expansion");
            assert_eq!(
                first, second,
                "expansion must be deterministic before measuring"
            );
            group.bench_with_input(
                BenchmarkId::new(label, effort_label),
                &(&compact, &compact_layout, &expanded, &expansion),
                |b, (compact, compact_layout, expanded, expansion)| {
                    b.iter(|| {
                        expand_group_in_place(
                            compact,
                            compact_layout,
                            expanded,
                            expansion,
                            &options,
                        )
                        .expect("expansion")
                    })
                },
            );
        }
    }
    group.finish();
}

fn tuned() -> Criterion {
    Criterion::default().sample_size(20)
}

criterion_group! {
    name = benches;
    config = tuned();
    targets = layout_benches, expand_benches
}
criterion_main!(benches);
