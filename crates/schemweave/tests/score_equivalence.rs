//! Locks byte-identical full-layout output across scoring-path optimizations.
//! Digests were captured from SchemWeave main at d02c032, before this branch.

#[path = "support/digest.rs"]
mod digest;
#[path = "../benches/support/generators.rs"]
#[allow(dead_code)]
mod generators;

use digest::layout_digest;
use schemweave::{Edge, Endpoint, Graph, LayoutConfig, QualityEffort, layout_with_config};

#[allow(clippy::field_reassign_with_default)]
fn config(effort: QualityEffort) -> LayoutConfig {
    let mut config = LayoutConfig::default();
    config.quality_effort = effort;
    config
}

fn mixed_sparse_outer_fixture() -> Graph {
    const LAYERS: u32 = 12;
    const PER_LAYER: u32 = 25;
    let node = |layer: u32, slot: u32| layer * PER_LAYER + slot;
    let mut graph = generators::layered_dag(LAYERS, PER_LAYER, 7);
    let mut next_edge_id = graph.edges.iter().map(|edge| edge.id).max().unwrap_or(0) + 1;

    // Pin a full-depth ranking backbone, then add reverse rank-skipping feedback edges.
    // The ordinary DAG retains multi-lane sparse gaps while the reverse edges require outer lanes.
    for layer in 0..LAYERS - 1 {
        graph.edges.push(Edge {
            id: next_edge_id,
            source: Endpoint {
                node: node(layer, 0),
                port: 2,
            },
            target: Endpoint {
                node: node(layer + 1, 0),
                port: 0,
            },
            net: next_edge_id,
            participates_in_ranking: true,
        });
        next_edge_id += 1;
    }
    for (source_layer, target_layer, slot) in [(11, 0, 1), (10, 1, 6), (9, 2, 11), (8, 3, 16)] {
        graph.edges.push(Edge {
            id: next_edge_id,
            source: Endpoint {
                node: node(source_layer, slot),
                port: 3,
            },
            target: Endpoint {
                node: node(target_layer, (slot + 7) % PER_LAYER),
                port: 1,
            },
            net: next_edge_id,
            participates_in_ranking: false,
        });
        next_edge_id += 1;
    }
    graph
}

#[test]
fn mixed_fixture_produces_outer_lane_routes() {
    // Guards the fixture's purpose: the spacing score-reuse eligibility gate
    // must be exercised by a graph that actually routes on outer lanes. If an
    // engine change ever reroutes these feedback edges off the outer lanes,
    // this fixture would silently stop covering the mixed configuration.
    let layout = layout_with_config(
        &mixed_sparse_outer_fixture(),
        &config(QualityEffort::Quality),
    )
    .expect("layout");
    let top = layout
        .nodes
        .iter()
        .map(|node| node.y)
        .min_by(f64::total_cmp)
        .expect("nodes");
    let bottom = layout
        .nodes
        .iter()
        .map(|node| node.y + node.height)
        .max_by(f64::total_cmp)
        .expect("nodes");
    let outer_points = layout
        .edges
        .iter()
        .flat_map(|edge| &edge.points)
        .filter(|point| point.y < top - 1.0 || point.y > bottom + 1.0)
        .count();
    assert!(
        outer_points > 0,
        "expected outer-lane routes outside node span (top={top}, bottom={bottom})"
    );
}

#[test]
fn quality_and_max_layouts_are_reproducible_and_match_preoptimization_digests() {
    let fixtures = [
        (
            "pipeline",
            generators::pipeline(30, 4),
            [
                (QualityEffort::Quality, 15_100_337_167_565_156_021),
                (QualityEffort::Max, 15_100_337_167_565_156_021),
            ],
        ),
        (
            "fanout8_ordinary",
            generators::fanout(8),
            [
                (QualityEffort::Quality, 5_780_370_522_801_744_726),
                (QualityEffort::Max, 5_780_370_522_801_744_726),
            ],
        ),
        (
            "fanout301_regional_min",
            generators::fanout(301),
            [
                (QualityEffort::Quality, 4_047_175_013_081_998_229),
                (QualityEffort::Max, 4_047_175_013_081_998_229),
            ],
        ),
        (
            "fanout512_regional_max",
            generators::fanout(512),
            [
                (QualityEffort::Quality, 2_490_547_472_294_486_322),
                (QualityEffort::Max, 2_490_547_472_294_486_322),
            ],
        ),
        (
            "fanin64",
            generators::fanin(64),
            [
                (QualityEffort::Quality, 18_167_339_820_869_073_029),
                (QualityEffort::Max, 18_167_339_820_869_073_029),
            ],
        ),
        (
            "dag_medium",
            generators::layered_dag(12, 25, 7),
            [
                (QualityEffort::Quality, 10_140_361_843_858_544_721),
                (QualityEffort::Max, 3_609_306_717_082_297_601),
            ],
        ),
        (
            "dag_large",
            generators::layered_dag(25, 60, 7),
            [
                (QualityEffort::Quality, 6_961_204_979_231_150_895),
                (QualityEffort::Max, 6_961_204_979_231_150_895),
            ],
        ),
        (
            "bus",
            generators::bus_chain(4, 32),
            [
                (QualityEffort::Quality, 3_874_013_447_867_824_340),
                (QualityEffort::Max, 3_874_013_447_867_824_340),
            ],
        ),
        (
            "mixed_sparse_outer",
            mixed_sparse_outer_fixture(),
            [
                (QualityEffort::Quality, 11_407_819_457_709_047_713),
                (QualityEffort::Max, 13_280_536_908_894_956_178),
            ],
        ),
    ];
    for (name, graph, expected) in fixtures {
        for (effort, expected_digest) in expected {
            let layout = layout_with_config(&graph, &config(effort)).expect("layout");
            let repeated = layout_with_config(&graph, &config(effort)).expect("repeated layout");
            assert_eq!(repeated, layout, "{name}/{effort:?} is not reproducible");
            assert_eq!(
                layout_digest(&layout),
                expected_digest,
                "{name}/{effort:?} changed"
            );
        }
    }
}

#[test]
#[ignore]
fn print_fixture_digests() {
    let fixtures = [
        ("pipeline", generators::pipeline(30, 4)),
        ("fanout8_ordinary", generators::fanout(8)),
        ("fanout301_regional_min", generators::fanout(301)),
        ("fanout512_regional_max", generators::fanout(512)),
        ("fanin64", generators::fanin(64)),
        ("dag_medium", generators::layered_dag(12, 25, 7)),
        ("dag_large", generators::layered_dag(25, 60, 7)),
        ("bus", generators::bus_chain(4, 32)),
        ("mixed_sparse_outer", mixed_sparse_outer_fixture()),
    ];
    for (name, graph) in fixtures {
        for effort in [
            QualityEffort::Fast,
            QualityEffort::Quality,
            QualityEffort::Max,
        ] {
            let layout = layout_with_config(&graph, &config(effort)).expect("layout");
            println!("{name}/{effort:?} {}", layout_digest(&layout));
        }
    }
}
