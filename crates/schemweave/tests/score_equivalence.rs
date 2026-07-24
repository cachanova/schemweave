//! Locks output equivalence across scoring-path optimizations: full layouts
//! of every bench fixture must be byte-identical to committed expectations
//! established before the optimization (asserted by double-run + cross-effort
//! stability, since golden files are not tracked).

#[path = "support/digest.rs"]
mod digest;
#[path = "../benches/support/generators.rs"]
#[allow(dead_code)]
mod generators;

use digest::layout_digest;
use schemweave::{LayoutConfig, QualityEffort, layout_with_config};

#[allow(clippy::field_reassign_with_default)]
fn config(effort: QualityEffort) -> LayoutConfig {
    let mut config = LayoutConfig::default();
    config.quality_effort = effort;
    config
}

#[test]
fn layouts_are_stable_across_efforts_and_runs() {
    let fixtures = [
        ("pipeline", generators::pipeline(30, 4)),
        ("fanout240", generators::fanout(240)),
        ("fanin64", generators::fanin(64)),
        ("dag_medium", generators::layered_dag(12, 25, 7)),
        ("dag_large", generators::layered_dag(25, 60, 7)),
        ("bus", generators::bus_chain(4, 32)),
    ];
    for (name, graph) in fixtures {
        for effort in [
            QualityEffort::Fast,
            QualityEffort::Quality,
            QualityEffort::Max,
        ] {
            let first = layout_with_config(&graph, &config(effort)).expect("layout");
            let second = layout_with_config(&graph, &config(effort)).expect("layout");
            assert_eq!(first, second, "{name} {effort:?} not reproducible");
        }
    }
}

#[test]
fn quality_and_max_layout_digests_match_spacing_reuse_baseline() {
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
            "fanout240",
            generators::fanout(240),
            [
                (QualityEffort::Quality, 1_454_913_959_470_172_659),
                (QualityEffort::Max, 1_454_913_959_470_172_659),
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
    ];
    for (name, graph, expected) in fixtures {
        for (effort, expected_digest) in expected {
            let layout = layout_with_config(&graph, &config(effort)).expect("layout");
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
        ("fanout240", generators::fanout(240)),
        ("fanin64", generators::fanin(64)),
        ("dag_medium", generators::layered_dag(12, 25, 7)),
        ("dag_large", generators::layered_dag(25, 60, 7)),
        ("bus", generators::bus_chain(4, 32)),
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
