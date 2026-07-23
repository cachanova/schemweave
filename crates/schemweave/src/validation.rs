use std::collections::{HashMap, HashSet};

use crate::{
    ConstrainedLayoutError, Edge, Endpoint, Graph, LayoutConstraintError, LayoutConstraints,
    LayoutError, LayoutOptions, Node, NodeId, Port, PortId, PortSide,
};

pub(crate) struct IndexedGraph<'a> {
    pub(crate) nodes: Vec<&'a Node>,
    pub(crate) edges: Vec<&'a Edge>,
    pub(crate) rank_edges: Vec<bool>,
    pub(crate) node_index: HashMap<NodeId, usize>,
    pub(crate) ports: Vec<HashMap<PortId, &'a Port>>,
    pub(crate) outgoing: Vec<Vec<usize>>,
    pub(crate) incoming: Vec<Vec<usize>>,
    pub(crate) boundary_inputs: Vec<bool>,
    pub(crate) boundary_outputs: Vec<bool>,
}

pub(crate) fn validate_and_index(
    graph: &Graph,
    options: LayoutOptions,
) -> Result<IndexedGraph<'_>, LayoutError> {
    match validate_and_index_with_constraints(graph, options, &LayoutConstraints::default()) {
        Ok(indexed) => Ok(indexed),
        Err(ConstrainedLayoutError::Layout(error)) => Err(error),
        Err(ConstrainedLayoutError::Constraint(_)) => {
            unreachable!("empty boundary constraints cannot fail validation")
        }
    }
}

pub(crate) fn validate_and_index_with_constraints<'a>(
    graph: &'a Graph,
    options: LayoutOptions,
    constraints: &LayoutConstraints,
) -> Result<IndexedGraph<'a>, ConstrainedLayoutError> {
    validate_options(options)?;
    let mut nodes: Vec<&Node> = graph.nodes.iter().collect();
    nodes.sort_unstable_by_key(|node| node.id);
    let mut node_index = HashMap::with_capacity(nodes.len());
    let mut ports = Vec::with_capacity(nodes.len());
    for (index, node) in nodes.iter().enumerate() {
        if node_index.insert(node.id, index).is_some() {
            return Err(LayoutError::DuplicateNode(node.id).into());
        }
        validate_dimension(node.id, "width", node.width)?;
        validate_dimension(node.id, "height", node.height)?;
        let mut by_id = HashMap::with_capacity(node.ports.len());
        for port in &node.ports {
            if by_id.insert(port.id, port).is_some() {
                return Err(LayoutError::DuplicatePort {
                    node: node.id,
                    port: port.id,
                }
                .into());
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
                }
                .into());
            }
        }
        ports.push(by_id);
    }

    let mut edges: Vec<&Edge> = graph.edges.iter().collect();
    edges.sort_unstable_by_key(|edge| edge.id);
    let mut edge_ids = HashSet::with_capacity(edges.len());
    for edge in &edges {
        if !edge_ids.insert(edge.id) {
            return Err(LayoutError::DuplicateEdge(edge.id).into());
        }
        validate_endpoint(edge.id, "source", edge.source, &node_index, &ports)?;
        validate_endpoint(edge.id, "target", edge.target, &node_index, &ports)?;
    }

    let (boundary_inputs, boundary_outputs) = validate_constraints(constraints, &node_index)?;
    let mut raw_outgoing = vec![Vec::new(); nodes.len()];
    let mut raw_incoming = vec![Vec::new(); nodes.len()];
    for edge in &edges {
        if !edge.participates_in_ranking {
            continue;
        }
        let source = node_index[&edge.source.node];
        let target = node_index[&edge.target.node];
        raw_outgoing[source].push(target);
        raw_incoming[target].push(source);
    }
    validate_constraint_structure(
        &nodes,
        &boundary_inputs,
        &boundary_outputs,
        &raw_outgoing,
        &raw_incoming,
    )?;
    let (rank_edges, outgoing, incoming) =
        runtime_ranking_graph(&nodes, &edges, &node_index, raw_outgoing, raw_incoming);
    Ok(IndexedGraph {
        nodes,
        edges,
        rank_edges,
        node_index,
        ports,
        outgoing,
        incoming,
        boundary_inputs,
        boundary_outputs,
    })
}

pub(crate) fn runtime_ranking_graph(
    nodes: &[&Node],
    edges: &[&Edge],
    node_index: &HashMap<NodeId, usize>,
    raw_outgoing: Vec<Vec<usize>>,
    raw_incoming: Vec<Vec<usize>>,
) -> (Vec<bool>, Vec<Vec<usize>>, Vec<Vec<usize>>) {
    if !nodes.iter().any(|node| node.cycle_breaker) {
        let rank_edges = edges
            .iter()
            .map(|edge| edge.participates_in_ranking)
            .collect();
        return (rank_edges, raw_outgoing, raw_incoming);
    }
    let (raw_component, _) =
        crate::topology::strongly_connected_components(&raw_outgoing, &raw_incoming);
    let mut outgoing = vec![Vec::new(); nodes.len()];
    let mut incoming = vec![Vec::new(); nodes.len()];
    let mut rank_edges = Vec::with_capacity(edges.len());
    for edge in edges {
        if !edge.participates_in_ranking {
            rank_edges.push(false);
            continue;
        }
        let source = node_index[&edge.source.node];
        let target = node_index[&edge.target.node];
        // Only an edge that participates in an actual cycle is a feedback boundary.
        if nodes[target].cycle_breaker && raw_component[source] == raw_component[target] {
            rank_edges.push(false);
            continue;
        }
        rank_edges.push(true);
        outgoing[source].push(target);
        incoming[target].push(source);
    }
    (rank_edges, outgoing, incoming)
}

fn validate_constraints(
    constraints: &LayoutConstraints,
    node_index: &HashMap<NodeId, usize>,
) -> Result<(Vec<bool>, Vec<bool>), LayoutConstraintError> {
    let mut inputs = constraints.inputs.clone();
    let mut outputs = constraints.outputs.clone();
    inputs.sort_unstable();
    outputs.sort_unstable();
    validate_constraint_ids("input", &inputs, node_index)?;
    validate_constraint_ids("output", &outputs, node_index)?;
    if let Some(node) = inputs
        .iter()
        .copied()
        .find(|node| outputs.binary_search(node).is_ok())
    {
        return Err(LayoutConstraintError::OverlappingConstraintNode(node));
    }
    let mut boundary_inputs = vec![false; node_index.len()];
    let mut boundary_outputs = vec![false; node_index.len()];
    for node in inputs {
        boundary_inputs[node_index[&node]] = true;
    }
    for node in outputs {
        boundary_outputs[node_index[&node]] = true;
    }
    Ok((boundary_inputs, boundary_outputs))
}

fn validate_constraint_ids(
    boundary: &'static str,
    nodes: &[NodeId],
    node_index: &HashMap<NodeId, usize>,
) -> Result<(), LayoutConstraintError> {
    if let Some(node) = nodes
        .windows(2)
        .find_map(|pair| (pair[0] == pair[1]).then_some(pair[0]))
    {
        return Err(LayoutConstraintError::DuplicateConstraintNode { boundary, node });
    }
    if let Some(node) = nodes
        .iter()
        .copied()
        .find(|node| !node_index.contains_key(node))
    {
        return Err(LayoutConstraintError::UnknownConstraintNode { boundary, node });
    }
    Ok(())
}

fn validate_constraint_structure(
    nodes: &[&Node],
    boundary_inputs: &[bool],
    boundary_outputs: &[bool],
    raw_outgoing: &[Vec<usize>],
    raw_incoming: &[Vec<usize>],
) -> Result<(), LayoutConstraintError> {
    for index in 0..nodes.len() {
        if boundary_inputs[index] && !raw_incoming[index].is_empty() {
            return Err(LayoutConstraintError::ConstrainedInputHasIncomingEdge(
                nodes[index].id,
            ));
        }
        if boundary_outputs[index] && !raw_outgoing[index].is_empty() {
            return Err(LayoutConstraintError::ConstrainedOutputHasOutgoingEdge(
                nodes[index].id,
            ));
        }
    }
    Ok(())
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
        ("max_quality_area_factor", options.max_quality_area_factor),
        (
            "max_quality_route_length_factor",
            options.max_quality_route_length_factor,
        ),
    ] {
        if !value.is_finite() || value <= 0.0 || value > 1_000_000.0 {
            return Err(LayoutError::InvalidOption { field, value });
        }
    }
    if options.max_quality_area_factor < 1.0 {
        return Err(LayoutError::InvalidOption {
            field: "max_quality_area_factor",
            value: options.max_quality_area_factor,
        });
    }
    if options.max_quality_route_length_factor < 1.0 {
        return Err(LayoutError::InvalidOption {
            field: "max_quality_route_length_factor",
            value: options.max_quality_route_length_factor,
        });
    }
    if !options.edge_node_clearance.is_finite()
        || options.edge_node_clearance < 0.0
        || options.edge_node_clearance > 1_000_000.0
    {
        return Err(LayoutError::InvalidOption {
            field: "edge_node_clearance",
            value: options.edge_node_clearance,
        });
    }
    if !options.minimum_parallel_wire_spacing.is_finite()
        || options.minimum_parallel_wire_spacing < 0.0
        || options.minimum_parallel_wire_spacing > 1_000_000.0
    {
        return Err(LayoutError::InvalidOption {
            field: "minimum_parallel_wire_spacing",
            value: options.minimum_parallel_wire_spacing,
        });
    }
    if !options
        .edge_node_clearance
        .mul_add(2.0, options.route_lane_gap)
        .is_finite()
    {
        return Err(LayoutError::InvalidOption {
            field: "edge_node_clearance",
            value: options.edge_node_clearance,
        });
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
