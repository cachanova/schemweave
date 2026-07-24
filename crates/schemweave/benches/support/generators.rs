//! Deterministic synthetic graph generators shared by the criterion benches
//! (`benches/layout.rs`) and the guarding test (`tests/bench_fixtures.rs`).

use schemweave::{Edge, Endpoint, Graph, Node, Port, PortSide};

/// Platform-independent LCG so `layered_dag` needs no rand dependency.
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed | 1)
    }

    fn next_below(&mut self, bound: u32) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 33) as u32) % bound.max(1)
    }
}

/// A block with `inputs` west ports (ids `0..inputs`) and `outputs` east
/// ports (ids `inputs..inputs + outputs`), evenly spaced.
fn block(id: u32, inputs: u32, outputs: u32) -> Node {
    let rows = inputs.max(outputs).max(1);
    let height = 24.0 * rows as f64;
    let mut ports = Vec::with_capacity((inputs + outputs) as usize);
    for input in 0..inputs {
        ports.push(Port {
            id: input,
            side: PortSide::West,
            offset: (input as f64 + 0.5) * height / inputs as f64,
        });
    }
    for output in 0..outputs {
        ports.push(Port {
            id: inputs + output,
            side: PortSide::East,
            offset: (output as f64 + 0.5) * height / outputs as f64,
        });
    }
    Node {
        id,
        width: 80.0,
        height,
        cycle_breaker: false,
        ports,
    }
}

fn edge(id: u32, source: (u32, u32), target: (u32, u32), net: u32) -> Edge {
    Edge {
        id,
        source: Endpoint {
            node: source.0,
            port: source.1,
        },
        target: Endpoint {
            node: target.0,
            port: target.1,
        },
        net,
        participates_in_ranking: true,
    }
}

/// `lanes` parallel chains of `depth` blocks with one cross-link per stage:
/// the straight-chain / traceability shape.
pub fn pipeline(depth: u32, lanes: u32) -> Graph {
    let id = |layer: u32, lane: u32| layer * lanes + lane;
    let nodes = (0..depth)
        .flat_map(|layer| (0..lanes).map(move |lane| block(id(layer, lane), 2, 1)))
        .collect();
    let mut edges = Vec::new();
    for layer in 0..depth.saturating_sub(1) {
        for lane in 0..lanes {
            let straight = edges.len() as u32;
            edges.push(edge(
                straight,
                (id(layer, lane), 2),
                (id(layer + 1, lane), 0),
                straight,
            ));
            let cross = edges.len() as u32;
            edges.push(edge(
                cross,
                (id(layer, lane), 2),
                (id(layer + 1, (lane + 1) % lanes), 1),
                straight,
            ));
        }
    }
    Graph { nodes, edges }
}

/// One driver, `sinks` consumers, one shared net: the trunk-sharing shape.
/// Keep `sinks <= 300` to stay on the sparse-channel path; larger values
/// exercise the outer-lane fallback.
pub fn fanout(sinks: u32) -> Graph {
    let mut nodes = vec![block(0, 0, 1)];
    nodes.extend((1..=sinks).map(|id| block(id, 1, 0)));
    let edges = (1..=sinks)
        .map(|sink| edge(sink - 1, (0, 0), (sink, 0), 0))
        .collect();
    Graph { nodes, edges }
}

/// Mirror of `fanout`: `sources` drivers converging on one consumer.
pub fn fanin(sources: u32) -> Graph {
    let mut nodes: Vec<Node> = (0..sources).map(|id| block(id, 0, 1)).collect();
    nodes.push(block(sources, sources, 0));
    let edges = (0..sources)
        .map(|source| edge(source, (source, 0), (sources, source), source))
        .collect();
    Graph { nodes, edges }
}

/// Seeded random-ish layered DAG: each node drives one or two nodes in the
/// next layer, every edge on its own net. The general large-graph shape.
pub fn layered_dag(layers: u32, per_layer: u32, seed: u64) -> Graph {
    let mut random = Lcg::new(seed);
    let id = |layer: u32, slot: u32| layer * per_layer + slot;
    let nodes = (0..layers)
        .flat_map(|layer| (0..per_layer).map(move |slot| block(id(layer, slot), 2, 2)))
        .collect();
    let mut edges = Vec::new();
    for layer in 0..layers.saturating_sub(1) {
        for slot in 0..per_layer {
            let targets = 1 + random.next_below(2);
            for output in 0..targets {
                let target_slot = random.next_below(per_layer);
                let edge_id = edges.len() as u32;
                edges.push(edge(
                    edge_id,
                    (id(layer, slot), 2 + output),
                    (id(layer + 1, target_slot), output),
                    edge_id,
                ));
            }
        }
    }
    Graph { nodes, edges }
}

/// `stages + 1` columns joined by `width` parallel single-edge nets: the
/// grouped-vector / bus shape (dense parallel wires between block pairs).
pub fn bus_chain(stages: u32, width: u32) -> Graph {
    let nodes = (0..=stages).map(|id| block(id, width, width)).collect();
    let mut edges = Vec::new();
    for stage in 0..stages {
        for bit in 0..width {
            let edge_id = edges.len() as u32;
            edges.push(edge(
                edge_id,
                (stage, width + bit),
                (stage + 1, bit),
                edge_id,
            ));
        }
    }
    Graph { nodes, edges }
}
