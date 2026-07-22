use schemweave::{Edge, Endpoint, Graph, Node, Port, PortSide};

pub fn graph() -> Graph {
    let mut graph = Graph {
        nodes: (0..17)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 100_000.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 0.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 50_000.0,
                    },
                    Port {
                        id: 2,
                        side: PortSide::West,
                        offset: 100_000.0,
                    },
                ],
            })
            .collect(),
        edges: Vec::new(),
    };
    for lane in 0..4u32 {
        for (source, target, target_port) in [
            (0, 5 + lane, 0),
            (1 + lane, 5 + lane, 2),
            (5 + lane, 9 + lane, 0),
            (9 + lane, 13 + lane, 0),
        ] {
            graph.edges.push(Edge {
                id: graph.edges.len() as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: target_port,
                },
                net: graph.edges.len() as u32,
                participates_in_ranking: true,
            });
        }
    }

    let first = graph.nodes.len() as u32;
    graph.nodes.push(Node {
        id: first,
        width: 20.0,
        height: 20.0,
        cycle_breaker: false,
        ports: vec![Port {
            id: 0,
            side: PortSide::East,
            offset: 10.0,
        }],
    });
    for branch in 0..512u32 {
        graph.nodes.push(Node {
            id: first + 1 + branch,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side: PortSide::North,
                offset: 10.0,
            }],
        });
        graph.edges.push(Edge {
            id: graph.edges.len() as u32,
            source: Endpoint {
                node: first,
                port: 0,
            },
            target: Endpoint {
                node: first + 1 + branch,
                port: 0,
            },
            net: 1_000,
            participates_in_ranking: true,
        });
    }
    for branch in 0..16u32 {
        let source = first + 513 + branch;
        let target = first + 529 + (15 - branch);
        graph.nodes.extend([
            Node {
                id: source,
                width: 20.0,
                height: 20.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 10.0,
                }],
            },
            Node {
                id: target,
                width: 20.0,
                height: 20.0,
                cycle_breaker: false,
                ports: vec![Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 10.0,
                }],
            },
        ]);
        graph.edges.push(Edge {
            id: graph.edges.len() as u32,
            source: Endpoint {
                node: source,
                port: 0,
            },
            target: Endpoint {
                node: target,
                port: 0,
            },
            net: 10_000 + branch,
            participates_in_ranking: true,
        });
    }
    graph
}
