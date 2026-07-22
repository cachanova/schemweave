use std::collections::{HashMap, HashSet};

use crate::{
    Edge, Endpoint, Graph, LayoutError, LayoutOptions, Node, NodeId, Port, PortId, PortSide,
};

pub(crate) struct IndexedGraph<'a> {
    pub(crate) nodes: Vec<&'a Node>,
    pub(crate) edges: Vec<&'a Edge>,
    pub(crate) rank_edges: Vec<bool>,
    pub(crate) node_index: HashMap<NodeId, usize>,
    pub(crate) ports: Vec<HashMap<PortId, &'a Port>>,
    pub(crate) outgoing: Vec<Vec<usize>>,
    pub(crate) incoming: Vec<Vec<usize>>,
}

pub(crate) fn validate_and_index(
    graph: &Graph,
    options: LayoutOptions,
) -> Result<IndexedGraph<'_>, LayoutError> {
    validate_options(options)?;
    let mut nodes: Vec<&Node> = graph.nodes.iter().collect();
    nodes.sort_unstable_by_key(|node| node.id);
    let mut node_index = HashMap::with_capacity(nodes.len());
    let mut ports = Vec::with_capacity(nodes.len());
    for (index, node) in nodes.iter().enumerate() {
        if node_index.insert(node.id, index).is_some() {
            return Err(LayoutError::DuplicateNode(node.id));
        }
        validate_dimension(node.id, "width", node.width)?;
        validate_dimension(node.id, "height", node.height)?;
        let mut by_id = HashMap::with_capacity(node.ports.len());
        for port in &node.ports {
            if by_id.insert(port.id, port).is_some() {
                return Err(LayoutError::DuplicatePort {
                    node: node.id,
                    port: port.id,
                });
            }
            let limit = match port.side {
                PortSide::East | PortSide::West => node.height,
                PortSide::North | PortSide::South => node.width,
            };
            if !port.offset.is_finite() || port.offset < 0.0 || port.offset > limit {
                return Err(LayoutError::InvalidPortOffset {
                    node: node.id,
                    port: port.id,
                    offset: port.offset,
                });
            }
        }
        ports.push(by_id);
    }

    let mut edges: Vec<&Edge> = graph.edges.iter().collect();
    edges.sort_unstable_by_key(|edge| edge.id);
    let mut edge_ids = HashSet::with_capacity(edges.len());
    for edge in &edges {
        if !edge_ids.insert(edge.id) {
            return Err(LayoutError::DuplicateEdge(edge.id));
        }
        validate_endpoint(edge.id, "source", edge.source, &node_index, &ports)?;
        validate_endpoint(edge.id, "target", edge.target, &node_index, &ports)?;
    }

    let mut raw_incoming = vec![0usize; nodes.len()];
    for edge in &edges {
        if edge.participates_in_ranking {
            raw_incoming[node_index[&edge.target.node]] += 1;
        }
    }
    let mut outgoing = vec![Vec::new(); nodes.len()];
    let mut incoming = vec![Vec::new(); nodes.len()];
    let mut rank_edges = Vec::with_capacity(edges.len());
    for edge in &edges {
        if !edge.participates_in_ranking {
            rank_edges.push(false);
            continue;
        }
        let source = node_index[&edge.source.node];
        let target = node_index[&edge.target.node];
        // A root source cannot close a cycle, so its constraint can place a register after
        // primary inputs. Other incoming register edges remain cut as feedback boundaries.
        if nodes[target].cycle_breaker && raw_incoming[source] != 0 {
            rank_edges.push(false);
            continue;
        }
        rank_edges.push(true);
        outgoing[source].push(target);
        incoming[target].push(source);
    }
    Ok(IndexedGraph {
        nodes,
        edges,
        rank_edges,
        node_index,
        ports,
        outgoing,
        incoming,
    })
}

fn validate_dimension(node: NodeId, field: &'static str, value: f64) -> Result<(), LayoutError> {
    if value.is_finite() && value > 0.0 && value <= 1_000_000_000.0 {
        Ok(())
    } else {
        Err(LayoutError::InvalidNodeDimension { node, field, value })
    }
}

fn validate_endpoint(
    edge: u32,
    role: &'static str,
    endpoint: Endpoint,
    node_index: &HashMap<NodeId, usize>,
    ports: &[HashMap<PortId, &Port>],
) -> Result<(), LayoutError> {
    let Some(&node) = node_index.get(&endpoint.node) else {
        return Err(LayoutError::UnknownEndpointNode {
            edge,
            role,
            node: endpoint.node,
        });
    };
    if !ports[node].contains_key(&endpoint.port) {
        return Err(LayoutError::UnknownEndpointPort {
            edge,
            role,
            node: endpoint.node,
            port: endpoint.port,
        });
    }
    Ok(())
}

fn validate_options(options: LayoutOptions) -> Result<(), LayoutError> {
    for (field, value) in [
        ("layer_gap", options.layer_gap),
        ("node_gap", options.node_gap),
        ("port_stub", options.port_stub),
        ("route_lane_gap", options.route_lane_gap),
    ] {
        if !value.is_finite() || value <= 0.0 || value > 1_000_000.0 {
            return Err(LayoutError::InvalidOption { field, value });
        }
    }
    if options.node_gap < options.port_stub * 2.0 {
        return Err(LayoutError::InvalidOption {
            field: "node_gap",
            value: options.node_gap,
        });
    }
    if options.ordering_sweeps > 16 {
        return Err(LayoutError::TooManyOrderingSweeps(options.ordering_sweeps));
    }
    Ok(())
}
