mod support;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use schemweave::{
    BoundaryBundleConstraint, BoundaryBundleMemberConstraint, Endpoint, Graph, LayoutConfig,
    QualityEffort, layout_with_config,
};
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

fn boundary_config(
    graph: &Graph,
    effort: QualityEffort,
    input: bool,
    boundary_nodes: &[u32],
    width: u32,
) -> LayoutConfig {
    let mut config = config(effort);
    if input {
        config.constraints.inputs = boundary_nodes.to_vec();
    } else {
        config.constraints.outputs = boundary_nodes.to_vec();
    }
    config.constraints.boundary_bundles = boundary_nodes
        .iter()
        .enumerate()
        .map(|(bundle, &node)| {
            let mut edges = graph
                .edges
                .iter()
                .filter(|edge| {
                    if input {
                        edge.source.node == node
                    } else {
                        edge.target.node == node
                    }
                })
                .collect::<Vec<_>>();
            edges.sort_unstable_by_key(|edge| edge.id);
            let endpoint = if input {
                edges[0].source
            } else {
                edges[0].target
            };
            let members = if edges.len() == 1 {
                vec![BoundaryBundleMemberConstraint {
                    edge: edges[0].id,
                    slots: (0..width).collect(),
                }]
            } else {
                edges
                    .into_iter()
                    .enumerate()
                    .map(|(slot, edge)| BoundaryBundleMemberConstraint {
                        edge: edge.id,
                        slots: vec![slot as u32],
                    })
                    .collect()
            };
            BoundaryBundleConstraint {
                id: bundle as u32 + 1,
                endpoint: Endpoint {
                    node: endpoint.node,
                    port: endpoint.port,
                },
                width,
                members,
            }
        })
        .collect();
    config
}

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

    let mut group = c.benchmark_group("expand/shared-boundary");
    for (role, fixture) in [
        (
            "fanout",
            generators::boundary_fanout_expansion_pair
                as fn(u32) -> (Graph, Graph, schemweave::GroupExpansion),
        ),
        ("fanin", generators::boundary_fanin_expansion_pair),
    ] {
        for members in [64, 256] {
            let (compact, expanded, expansion) = fixture(members);
            for (effort_label, effort) in EFFORTS {
                let config = config(effort);
                let compact_layout =
                    layout_with_config(&compact, &config).expect("compact fixture must lay out");
                let options = GroupExpansionOptions {
                    layout: config.layout,
                    quality_effort: config.quality_effort,
                    constraints: config.constraints.clone(),
                    protected_groups: Vec::new(),
                };
                let first = expand_group_in_place(
                    &compact,
                    &compact_layout,
                    &expanded,
                    &expansion,
                    &options,
                )
                .expect("shared-boundary expansion");
                let second = expand_group_in_place(
                    &compact,
                    &compact_layout,
                    &expanded,
                    &expansion,
                    &options,
                )
                .expect("shared-boundary expansion");
                assert_eq!(
                    first, second,
                    "shared-boundary expansion must be deterministic before measuring"
                );
                group.throughput(Throughput::Elements(expanded.edges.len() as u64));
                group.bench_with_input(
                    BenchmarkId::new(format!("{role}-{members}"), effort_label),
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
                            .expect("shared-boundary expansion")
                        })
                    },
                );
            }
        }
    }
    group.finish();

    let mut group = c.benchmark_group("expand/affected-boundary");
    for (role, fixture) in [
        (
            "fanout",
            generators::boundary_fanout_expansion_pair
                as fn(u32) -> (Graph, Graph, schemweave::GroupExpansion),
        ),
        ("fanin", generators::boundary_fanin_expansion_pair),
    ] {
        let members = 16;
        let (compact, expanded, expansion) = fixture(members);
        for (effort_label, effort) in EFFORTS {
            let input = role == "fanout";
            let boundary_nodes = if input { vec![0] } else { vec![2] };
            let compact_config = boundary_config(&compact, effort, input, &boundary_nodes, members);
            let config = boundary_config(&expanded, effort, input, &boundary_nodes, members);
            let compact_layout = layout_with_config(&compact, &compact_config)
                .expect("compact fixture must lay out");
            let options = GroupExpansionOptions {
                layout: config.layout,
                quality_effort: config.quality_effort,
                constraints: config.constraints.clone(),
                protected_groups: Vec::new(),
            };
            let first =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("affected-boundary expansion");
            let second =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("affected-boundary expansion");
            assert_eq!(
                first, second,
                "affected-boundary expansion must be deterministic before measuring"
            );
            group.throughput(Throughput::Elements(expanded.edges.len() as u64));
            group.bench_with_input(
                BenchmarkId::new(role, effort_label),
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
                        .expect("affected-boundary expansion")
                    })
                },
            );
        }
    }
    group.finish();

    let mut group = c.benchmark_group("expand/shared-boundary-groups");
    for (role, fixture) in [
        (
            "fanout-8x64",
            generators::grouped_boundary_fanout_expansion_pair
                as fn(u32, u32) -> (Graph, Graph, schemweave::GroupExpansion),
        ),
        (
            "fanin-8x64",
            generators::grouped_boundary_fanin_expansion_pair,
        ),
    ] {
        let (compact, expanded, expansion) = fixture(8, 64);
        for (effort_label, effort) in EFFORTS {
            let config = config(effort);
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
                    .expect("grouped shared-boundary expansion");
            let second =
                expand_group_in_place(&compact, &compact_layout, &expanded, &expansion, &options)
                    .expect("grouped shared-boundary expansion");
            assert_eq!(
                first, second,
                "grouped shared-boundary expansion must be deterministic before measuring"
            );
            group.throughput(Throughput::Elements(expanded.edges.len() as u64));
            group.bench_with_input(
                BenchmarkId::new(role, effort_label),
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
                        .expect("grouped shared-boundary expansion")
                    })
                },
            );
        }
    }
    group.finish();

    let mut group = c.benchmark_group("expand/protected-peers");
    for (label, peer_count, peer_members) in [("1x8", 1, 8), ("8x1", 8, 1)] {
        let (compact, expanded, expansion, peer_ids) =
            generators::protected_horizontal_expansion_pair(8, peer_count * peer_members);
        for (effort_label, effort) in EFFORTS {
            let config = config(effort);
            let compact_layout =
                layout_with_config(&compact, &config).expect("compact fixture must lay out");
            let protected_groups = (0..peer_count)
                .map(|group| ProtectedGroup {
                    id: 1_000 + group,
                    members: peer_ids
                        [(group * peer_members) as usize..((group + 1) * peer_members) as usize]
                        .to_vec(),
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
            let compact_peer_left = compact_layout
                .nodes
                .iter()
                .filter(|node| peer_ids.contains(&node.id))
                .map(|node| node.x)
                .min_by(f64::total_cmp)
                .expect("compact peer frame");
            let expanded_peer_left = first
                .nodes
                .iter()
                .filter(|node| peer_ids.contains(&node.id))
                .map(|node| node.x)
                .min_by(f64::total_cmp)
                .expect("expanded peer frame");
            assert!(
                expanded_peer_left > compact_peer_left,
                "protected benchmark must exercise horizontal corridor movement"
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
