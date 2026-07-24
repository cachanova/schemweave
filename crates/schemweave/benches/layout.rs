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

use schemweave::{
    GroupCollapseOptions, GroupExpansionOptions, ProtectedGroup, collapse_group_in_place,
    expand_group_in_place,
};

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
                protected_groups: Vec::new(),
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

    let mut group = c.benchmark_group("expand/protected-peers");
    for (label, peer_count, peer_members) in [("1x8", 1, 8), ("8x1", 8, 1)] {
        let bystanders = peer_count * peer_members;
        let (mut compact, expanded, expansion) = generators::expansion_pair(8, bystanders);
        compact
            .nodes
            .iter_mut()
            .find(|node| node.id == expansion.anchor)
            .expect("compact anchor")
            .width = 2_400.0;
        for (effort_label, effort) in EFFORTS {
            let config = config(effort);
            let compact_layout =
                layout_with_config(&compact, &config).expect("compact fixture must lay out");
            let protected_groups = (0..peer_count)
                .map(|group| ProtectedGroup {
                    id: 1_000 + group,
                    members: (0..peer_members)
                        .map(|member| 3 + group * peer_members + member)
                        .collect(),
                    frame_padding: 16.0,
                })
                .collect();
            let options = GroupExpansionOptions {
                layout: config.layout,
                quality_effort: config.quality_effort,
                constraints: config.constraints.clone(),
                protected_groups,
            };
            let first =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("protected expansion");
            let second =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("protected expansion");
            assert_eq!(
                first, second,
                "protected expansion must be deterministic before measuring"
            );
            group.throughput(Throughput::Elements(expanded.edges.len() as u64));
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
                        .expect("protected expansion")
                    })
                },
            );
        }
    }
    group.finish();
}

fn collapse_benches(c: &mut Criterion) {
    let mut group = c.benchmark_group("collapse/chain");
    for (label, members, bystanders) in [("4m-6b", 4, 6), ("8m-40b", 8, 40)] {
        let (compact, expanded, expansion) = generators::expansion_pair(members, bystanders);
        for (effort_label, effort) in EFFORTS {
            let config = config(effort);
            let compact_layout =
                layout_with_config(&compact, &config).expect("compact fixture must lay out");
            let expansion_options = GroupExpansionOptions {
                layout: config.layout,
                quality_effort: config.quality_effort,
                constraints: config.constraints.clone(),
                protected_groups: Vec::new(),
            };
            let expanded_layout = expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &expansion_options,
            )
            .expect("expansion");
            let collapse_options = GroupCollapseOptions {
                layout: config.layout,
                constraints: config.constraints.clone(),
            };
            let first = collapse_group_in_place(
                &expanded,
                &expanded_layout,
                &compact,
                &expansion,
                &collapse_options,
            )
            .expect("collapse");
            let second = collapse_group_in_place(
                &expanded,
                &expanded_layout,
                &compact,
                &expansion,
                &collapse_options,
            )
            .expect("collapse");
            assert_eq!(
                first, second,
                "collapse must be deterministic before measuring"
            );
            group.bench_with_input(
                BenchmarkId::new(label, effort_label),
                &(&expanded, &expanded_layout, &compact, &expansion),
                |b, (expanded, expanded_layout, compact, expansion)| {
                    b.iter(|| {
                        collapse_group_in_place(
                            expanded,
                            expanded_layout,
                            compact,
                            expansion,
                            &collapse_options,
                        )
                        .expect("collapse")
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
    targets = layout_benches, expand_benches, collapse_benches
}
criterion_main!(benches);
