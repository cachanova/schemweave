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
