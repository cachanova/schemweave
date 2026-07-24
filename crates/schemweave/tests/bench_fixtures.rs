//! Guards the bench fixtures: generators must be deterministic and every
//! fixture must lay out successfully at Fast effort.

#[path = "../benches/support/generators.rs"]
mod generators;

use schemweave::{LayoutConfig, QualityEffort, layout_with_config};

#[allow(clippy::field_reassign_with_default)]
fn fast() -> LayoutConfig {
    let mut config = LayoutConfig::default();
    config.quality_effort = QualityEffort::Fast;
    config
}

#[test]
fn generators_are_deterministic() {
    assert_eq!(generators::pipeline(10, 3), generators::pipeline(10, 3));
    assert_eq!(
        generators::layered_dag(6, 8, 7),
        generators::layered_dag(6, 8, 7)
    );
    assert_eq!(generators::bus_chain(3, 8), generators::bus_chain(3, 8));
}

#[test]
fn every_fixture_lays_out_and_layout_is_reproducible() {
    let fixtures = [
        ("pipeline", generators::pipeline(10, 3)),
        ("fanout", generators::fanout(16)),
        ("fanin", generators::fanin(16)),
        ("layered_dag", generators::layered_dag(6, 8, 7)),
        ("bus_chain", generators::bus_chain(3, 8)),
    ];
    for (name, graph) in fixtures {
        let first = layout_with_config(&graph, &fast())
            .unwrap_or_else(|error| panic!("{name} failed to lay out: {error:?}"));
        let second = layout_with_config(&graph, &fast()).expect("second run");
        assert_eq!(first, second, "{name} layout is not reproducible");
    }
}
