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

use schemweave::{BoundaryTrunk, GroupExpansion};

/// A compact graph whose anchor node hides a `members`-long chain, plus
/// `bystanders` retained chain nodes that must stay byte-identical after
/// expansion. Node ids: 0 = driver, 1 = anchor, 2 = consumer,
/// 3..3 + bystanders = retained chain, 200.. = members (expanded only).
/// Edge ids: 1/2 compact boundary trunks, 10/11 expanded boundary edges,
/// 100.. bystander edges, 300.. internal member edges.
pub fn expansion_pair(members: u32, bystanders: u32) -> (Graph, Graph, GroupExpansion) {
    assert!(members >= 1);
    let driver = block(0, 0, 1);
    let anchor = block(1, 1, 1);
    let consumer = block(2, 1, 0);

    // Retained bystander chain, identical in both graphs.
    let bystander_nodes: Vec<Node> = (0..bystanders)
        .map(|index| {
            let (inputs, outputs) = if index == 0 {
                (0, 1)
            } else if index + 1 == bystanders {
                (1, 0)
            } else {
                (1, 1)
            };
            block(3 + index, inputs, outputs)
        })
        .collect();
    let bystander_edges: Vec<Edge> = (0..bystanders.saturating_sub(1))
        .map(|index| {
            let source_out = if index == 0 { 0 } else { 1 };
            edge(
                100 + index,
                (3 + index, source_out),
                (3 + index + 1, 0),
                100 + index,
            )
        })
        .collect();

    // Compact: driver -> anchor (edge 1, net 1), anchor -> consumer (edge 2, net 2).
    let mut compact_nodes = vec![driver.clone(), anchor, consumer.clone()];
    compact_nodes.extend(bystander_nodes.iter().cloned());
    let mut compact_edges = vec![edge(1, (0, 0), (1, 0), 1), edge(2, (1, 1), (2, 0), 2)];
    compact_edges.extend(bystander_edges.iter().cloned());
    let compact = Graph {
        nodes: compact_nodes,
        edges: compact_edges,
    };

    // Expanded: anchor replaced by member chain 200..200 + members.
    let member_ids: Vec<u32> = (0..members).map(|index| 200 + index).collect();
    let member_nodes: Vec<Node> = member_ids.iter().map(|&id| block(id, 1, 1)).collect();
    let mut expanded_nodes = vec![driver, consumer];
    expanded_nodes.extend(bystander_nodes);
    expanded_nodes.extend(member_nodes);
    // Boundary edges keep the compact nets; internal member edges get fresh
    // ids/nets from 300 + position (the first is 301).
    let mut expanded_edges = vec![edge(10, (0, 0), (member_ids[0], 0), 1)];
    for window in member_ids.windows(2) {
        let id = 300 + expanded_edges.len() as u32;
        expanded_edges.push(edge(id, (window[0], 1), (window[1], 0), id));
    }
    expanded_edges.push(edge(
        11,
        (*member_ids.last().expect("members >= 1"), 1),
        (2, 0),
        2,
    ));
    expanded_edges.extend(bystander_edges);
    let expanded = Graph {
        nodes: expanded_nodes,
        edges: expanded_edges,
    };

    let expansion = GroupExpansion {
        anchor: 1,
        members: member_ids,
        boundary_trunks: vec![
            BoundaryTrunk {
                expanded_edge: 10,
                compact_edge: 1,
            },
            BoundaryTrunk {
                expanded_edge: 11,
                compact_edge: 2,
            },
        ],
    };
    (compact, expanded, expansion)
}
