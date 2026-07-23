//! Deterministic layered placement and orthogonal routing for circuit graphs.

#![forbid(unsafe_code)]

mod boundary_bundles;
mod incremental;
mod placement;
mod readability;
mod routing;
mod topology;
mod validation;

const MIN_DEMAND_AWARE_SPACING_NODES: usize = 150;
const MAX_DEMAND_AWARE_SPACING_NODES: usize = 400;
const MIN_DEMAND_AWARE_SPACING_EDGES: usize = 250;
const MAX_DEMAND_AWARE_SPACING_EDGES: usize = 400;
const MAX_LAYOUT_CLEARANCE_PAIR_VISITS: usize = 20_000_000;
const MAX_LAYOUT_PARALLEL_WIRE_SPACING_SEGMENTS: usize = 100_000;
const MAX_LAYOUT_PARALLEL_WIRE_SPACING_VISITS: usize = 20_000_000;
const MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS: usize = 100_000;
const MAX_LAYOUT_ROUTE_CONTACT_VISITS: usize = 20_000_000;

use std::collections::{BTreeSet, HashMap};

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub use incremental::{
    BoundaryTrunk, GroupExpansion, GroupExpansionError, GroupExpansionOptions,
    expand_group_in_place,
};
pub use placement::place;
pub(crate) use readability::measure_parallel_congestion_profile_bounded;
#[doc(hidden)]
pub use readability::{
    EdgeNodeClearance, EdgeNodeClearanceError, EdgeNodeSegment, NetNodeRelation,
    ParallelCongestion, ParallelSegment, ParallelSeparation, ParallelSeparationError,
    measure_edge_node_clearance_bounded, measure_parallel_congestion,
    measure_parallel_separation_bounded,
};

pub type NodeId = u32;
pub type PortId = u32;
pub type EdgeId = u32;
pub type NetId = u32;
pub type BoundaryBundleId = u32;

/// Classify the edges for which strict left-to-right placement is meaningful.
///
/// This is public only so the development scorer can share the runtime
/// classification exactly.
#[doc(hidden)]
pub fn effective_ranking_edges(graph: &Graph) -> BTreeSet<EdgeId> {
    let nodes = graph.nodes.iter().collect::<Vec<_>>();
    let node_index = nodes
        .iter()
        .enumerate()
        .map(|(index, node)| (node.id, index))
        .collect::<HashMap<_, _>>();
    let edges = graph
        .edges
        .iter()
        .filter(|edge| {
            node_index.contains_key(&edge.source.node) && node_index.contains_key(&edge.target.node)
        })
        .collect::<Vec<_>>();
    let mut outgoing = vec![Vec::new(); graph.nodes.len()];
    let mut incoming = vec![Vec::new(); graph.nodes.len()];
    for edge in &edges {
        if !edge.participates_in_ranking {
            continue;
        }
        let source = node_index[&edge.source.node];
        let target = node_index[&edge.target.node];
        outgoing[source].push(target);
        incoming[target].push(source);
    }
    let (runtime_mask, runtime_outgoing, runtime_incoming) =
        validation::runtime_ranking_graph(&nodes, &edges, &node_index, outgoing, incoming);
    let (components, _) =
        topology::strongly_connected_components(&runtime_outgoing, &runtime_incoming);
    edges
        .into_iter()
        .zip(runtime_mask)
        .filter(|(edge, ranking)| {
            *ranking
                && components[node_index[&edge.source.node]]
                    != components[node_index[&edge.target.node]]
        })
        .map(|(edge, _)| edge.id)
        .collect()
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Graph {
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Node {
    pub id: NodeId,
    pub width: f64,
    pub height: f64,
    #[serde(default)]
    pub cycle_breaker: bool,
    pub ports: Vec<Port>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Port {
    pub id: PortId,
    pub side: PortSide,
    /// Distance from the top for east/west ports or from the left for north/south.
    pub offset: f64,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PortSide {
    North,
    East,
    South,
    West,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
pub struct Endpoint {
    pub node: NodeId,
    pub port: PortId,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Edge {
    pub id: EdgeId,
    pub source: Endpoint,
    pub target: Endpoint,
    /// Routes may share trunks when the caller assigns the same net.
    pub net: NetId,
    #[serde(default = "default_true")]
    pub participates_in_ranking: bool,
}

const fn default_true() -> bool {
    true
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct LayoutOptions {
    pub layer_gap: f64,
    pub node_gap: f64,
    pub port_stub: f64,
    /// Preferred construction pitch for route lanes; not a hard guarantee.
    pub route_lane_gap: f64,
    /// Minimum axis-aligned distance between a route and every unrelated node.
    pub edge_node_clearance: f64,
    /// Minimum perpendicular distance between longitudinally overlapping
    /// parallel route segments that belong to different nets, except their
    /// mandatory shared escape from one fixed endpoint.
    pub minimum_parallel_wire_spacing: f64,
    /// Maximum area multiple a quality refinement may consume relative to its baseline.
    pub max_quality_area_factor: f64,
    /// Maximum route-length multiple a quality refinement may consume relative to its baseline.
    pub max_quality_route_length_factor: f64,
    pub ordering_sweeps: usize,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum QualityEffort {
    Fast,
    #[default]
    Quality,
    Max,
}

/// Optional semantic constraints for primary boundary nodes.
///
/// Constrained inputs occupy the leftmost rank. Constrained outputs occupy one
/// shared rank to the right of every non-output node.
#[derive(Clone, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
pub struct LayoutConstraints {
    #[serde(default)]
    pub inputs: Vec<NodeId>,
    #[serde(default)]
    pub outputs: Vec<NodeId>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub boundary_bundles: Vec<BoundaryBundleConstraint>,
}

/// One graphical bus collector anchored at a constrained boundary endpoint.
///
/// Members retain independent edge identity. `slots` name occupied positions
/// within the graphical bus. Same-net members with identical slot sets form a
/// fanout cohort and share one physical tap; every other overlap is invalid.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BoundaryBundleConstraint {
    pub id: BoundaryBundleId,
    pub endpoint: Endpoint,
    pub width: u32,
    pub members: Vec<BoundaryBundleMemberConstraint>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BoundaryBundleMemberConstraint {
    pub edge: EdgeId,
    pub slots: Vec<u32>,
}

/// Complete, serializable policy for one layout request.
///
/// The quality effort acts as the coarse quality-versus-latency control while
/// [`LayoutOptions`] remains available for applications that need explicit
/// spacing and ordering overrides.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct LayoutConfig {
    #[serde(flatten)]
    pub layout: LayoutOptions,
    pub quality_effort: QualityEffort,
    pub constraints: LayoutConstraints,
}

impl LayoutConfig {
    /// Use every bounded quality refinement enabled by the engine.
    pub fn highest_quality() -> Self {
        Self {
            layout: LayoutOptions {
                route_lane_gap: 6.0,
                edge_node_clearance: 20.0,
                max_quality_area_factor: 2.0,
                max_quality_route_length_factor: 1.25,
                ..LayoutOptions::default()
            },
            quality_effort: QualityEffort::Max,
            ..Self::default()
        }
    }
}

impl Default for LayoutConfig {
    fn default() -> Self {
        Self {
            layout: LayoutOptions::default(),
            quality_effort: QualityEffort::Quality,
            constraints: LayoutConstraints::default(),
        }
    }
}

impl Default for LayoutOptions {
    fn default() -> Self {
        Self {
            layer_gap: 66.0,
            node_gap: 30.0,
            port_stub: 10.0,
            route_lane_gap: 4.0,
            edge_node_clearance: 0.0,
            minimum_parallel_wire_spacing: 0.0,
            max_quality_area_factor: 1.2,
            max_quality_route_length_factor: 1.1,
            ordering_sweeps: 4,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct NodeGeometry {
    pub id: NodeId,
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct EdgeGeometry {
    pub id: EdgeId,
    pub points: Vec<Point>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum BoundaryBundleRole {
    Input,
    Output,
}

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct BoundaryBundleSegment {
    pub start: Point,
    pub end: Point,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BoundaryBundleMemberGeometry {
    pub edge: EdgeId,
    pub slots: Vec<u32>,
    pub tap: Point,
}

/// Output-only geometry for one graphical boundary bus.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct BoundaryBundleGeometry {
    pub id: BoundaryBundleId,
    pub endpoint: Endpoint,
    pub role: BoundaryBundleRole,
    pub width: u32,
    pub collector: BoundaryBundleSegment,
    pub spine: BoundaryBundleSegment,
    pub members: Vec<BoundaryBundleMemberGeometry>,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Layout {
    pub nodes: Vec<NodeGeometry>,
    pub edges: Vec<EdgeGeometry>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub boundary_bundles: Vec<BoundaryBundleGeometry>,
    pub width: f64,
    pub height: f64,
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum LayoutError {
    #[error("duplicate node id {0}")]
    DuplicateNode(NodeId),
    #[error("node {node} has duplicate port id {port}")]
    DuplicatePort { node: NodeId, port: PortId },
    #[error("duplicate edge id {0}")]
    DuplicateEdge(EdgeId),
    #[error("node {node} has invalid {field} {value}")]
    InvalidNodeDimension {
        node: NodeId,
        field: &'static str,
        value: f64,
    },
    #[error("node {node} port {port} has invalid offset {offset}")]
    InvalidPortOffset {
        node: NodeId,
        port: PortId,
        offset: f64,
    },
    #[error("edge {edge} references unknown {role} node {node}")]
    UnknownEndpointNode {
        edge: EdgeId,
        role: &'static str,
        node: NodeId,
    },
    #[error("edge {edge} references unknown {role} port {node}:{port}")]
    UnknownEndpointPort {
        edge: EdgeId,
        role: &'static str,
        node: NodeId,
        port: PortId,
    },
    #[error("invalid layout option {field}={value}")]
    InvalidOption { field: &'static str, value: f64 },
    #[error("ordering_sweeps must be at most 16, got {0}")]
    TooManyOrderingSweeps(usize),
    #[error(
        "no complete layout satisfies edge_node_clearance={clearance}; every candidate intersects an unrelated node"
    )]
    EdgeNodeClearanceUnsatisfied { clearance: f64 },
    #[error(
        "edge-to-node clearance verification exceeded its deterministic work limit of {maximum} candidate visits"
    )]
    EdgeNodeClearanceWorkLimitExceeded { maximum: usize },
    #[error(
        "no complete positive-clearance layout avoids overlap or contact between unrelated routes"
    )]
    UnrelatedRouteContactUnsatisfied,
    #[error(
        "unrelated-route contact verification exceeded its deterministic work limit of {maximum} candidate visits"
    )]
    UnrelatedRouteContactWorkLimitExceeded { maximum: usize },
    #[error(
        "unrelated-route contact verification exceeded its deterministic route-segment limit of {maximum}"
    )]
    UnrelatedRouteContactSegmentLimitExceeded { maximum: usize },
    #[error("no complete layout satisfies minimum_parallel_wire_spacing={spacing}")]
    ParallelWireSpacingUnsatisfied { spacing: f64 },
    #[error(
        "parallel-wire spacing verification exceeded its deterministic work limit of {maximum} tree-node visits"
    )]
    ParallelWireSpacingWorkLimitExceeded { maximum: usize },
    #[error(
        "parallel-wire spacing verification exceeded its deterministic route-segment limit of {maximum}"
    )]
    ParallelWireSpacingSegmentLimitExceeded { maximum: usize },
    #[error("boundary bundle geometry does not satisfy the hard readability contract")]
    BoundaryBundleGeometryUnsatisfied,
    #[error(
        "boundary bundle geometry verification exceeded its deterministic work limit of {maximum} segment visits"
    )]
    BoundaryBundleGeometryWorkLimitExceeded { maximum: usize },
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum LayoutConstraintError {
    #[error("unknown constrained {boundary} node {node}")]
    UnknownConstraintNode {
        boundary: &'static str,
        node: NodeId,
    },
    #[error("duplicate constrained {boundary} node {node}")]
    DuplicateConstraintNode {
        boundary: &'static str,
        node: NodeId,
    },
    #[error("node {0} cannot be both a constrained input and output")]
    OverlappingConstraintNode(NodeId),
    #[error("constrained input node {0} has a participating incoming edge")]
    ConstrainedInputHasIncomingEdge(NodeId),
    #[error("constrained output node {0} has a participating outgoing edge")]
    ConstrainedOutputHasOutgoingEdge(NodeId),
    #[error("duplicate boundary bundle id {0}")]
    DuplicateBoundaryBundle(BoundaryBundleId),
    #[error("boundary bundle {bundle} has invalid width {width}")]
    InvalidBoundaryBundleWidth {
        bundle: BoundaryBundleId,
        width: u32,
    },
    #[error("boundary bundle {0} has no members")]
    EmptyBoundaryBundle(BoundaryBundleId),
    #[error("boundary bundle {bundle} references unknown endpoint node {node}")]
    UnknownBoundaryBundleEndpointNode {
        bundle: BoundaryBundleId,
        node: NodeId,
    },
    #[error("boundary bundle {bundle} references unknown endpoint port {node}:{port}")]
    UnknownBoundaryBundleEndpointPort {
        bundle: BoundaryBundleId,
        node: NodeId,
        port: PortId,
    },
    #[error("boundary bundle {bundle} endpoint node {node} is not a constrained boundary node")]
    UnconstrainedBoundaryBundleEndpoint {
        bundle: BoundaryBundleId,
        node: NodeId,
    },
    #[error(
        "boundary bundle {bundle} endpoint {node}:{port} must use an east input port or west output port"
    )]
    InvalidBoundaryBundleEndpointSide {
        bundle: BoundaryBundleId,
        node: NodeId,
        port: PortId,
    },
    #[error("boundary bundle {bundle} repeats member edge {edge}")]
    DuplicateBoundaryBundleMember {
        bundle: BoundaryBundleId,
        edge: EdgeId,
    },
    #[error("edge {edge} belongs to more than one boundary bundle")]
    BoundaryBundleMemberInMultipleBundles { edge: EdgeId },
    #[error("boundary bundle {bundle} references unknown member edge {edge}")]
    UnknownBoundaryBundleMember {
        bundle: BoundaryBundleId,
        edge: EdgeId,
    },
    #[error("boundary bundle {bundle} member edge {edge} does not use the bundle endpoint")]
    BoundaryBundleMemberEndpointMismatch {
        bundle: BoundaryBundleId,
        edge: EdgeId,
    },
    #[error("boundary bundle {bundle} member edge {edge} has no slots")]
    EmptyBoundaryBundleMemberSlots {
        bundle: BoundaryBundleId,
        edge: EdgeId,
    },
    #[error("boundary bundle {bundle} member edge {edge} uses slot {slot} outside width {width}")]
    BoundaryBundleSlotOutOfRange {
        bundle: BoundaryBundleId,
        edge: EdgeId,
        slot: u32,
        width: u32,
    },
    #[error("boundary bundle {bundle} repeats slot {slot} within one member")]
    DuplicateBoundaryBundleSlot { bundle: BoundaryBundleId, slot: u32 },
    #[error(
        "boundary bundle {bundle} member edges {first_edge} and {second_edge} conflict at slot {slot}"
    )]
    BoundaryBundleSlotConflict {
        bundle: BoundaryBundleId,
        first_edge: EdgeId,
        second_edge: EdgeId,
        slot: u32,
    },
    #[error(
        "boundary bundle validation exceeded its deterministic work limit of {maximum} members and slots"
    )]
    BoundaryBundleWorkLimitExceeded { maximum: usize },
}

#[derive(Clone, Debug, Error, PartialEq)]
pub enum ConstrainedLayoutError {
    #[error(transparent)]
    Layout(#[from] LayoutError),
    #[error(transparent)]
    Constraint(#[from] LayoutConstraintError),
}

/// Lay out a graph. Output ordering depends only on stable identifiers, not input order.
pub fn layout(graph: &Graph, options: LayoutOptions) -> Result<Layout, LayoutError> {
    layout_with_quality_effort(graph, options, QualityEffort::Quality)
}

/// Lay out a graph with explicit primary boundary constraints.
pub fn layout_with_constraints(
    graph: &Graph,
    options: LayoutOptions,
    constraints: &LayoutConstraints,
) -> Result<Layout, ConstrainedLayoutError> {
    layout_with_quality_effort_and_constraints(graph, options, QualityEffort::Quality, constraints)
}

/// Lay out a graph with an explicit quality-versus-latency policy.
pub fn layout_with_quality_effort(
    graph: &Graph,
    options: LayoutOptions,
    quality_effort: QualityEffort,
) -> Result<Layout, LayoutError> {
    let indexed = validation::validate_and_index(graph, options)?;
    layout_indexed(graph, options, quality_effort, indexed)
}

/// Lay out a graph with explicit boundary constraints and quality policy.
pub fn layout_with_quality_effort_and_constraints(
    graph: &Graph,
    options: LayoutOptions,
    quality_effort: QualityEffort,
    constraints: &LayoutConstraints,
) -> Result<Layout, ConstrainedLayoutError> {
    let indexed = validation::validate_and_index_with_constraints(graph, options, constraints)?;
    Ok(layout_indexed(graph, options, quality_effort, indexed)?)
}

/// Lay out a graph using one canonical request configuration.
pub fn layout_with_config(
    graph: &Graph,
    config: &LayoutConfig,
) -> Result<Layout, ConstrainedLayoutError> {
    layout_with_quality_effort_and_constraints(
        graph,
        config.layout,
        config.quality_effort,
        &config.constraints,
    )
}

fn layout_indexed(
    graph: &Graph,
    options: LayoutOptions,
    quality_effort: QualityEffort,
    indexed: validation::IndexedGraph<'_>,
) -> Result<Layout, LayoutError> {
    let options = effective_layout_options(options);
    let (ranks, latest_ranks) = topology::rank_candidates(&indexed);
    let (forward, reverse, net_representative, max_sifted) = if quality_effort == QualityEffort::Max
    {
        topology::order_layer_candidates_with_max_sifting(
            &indexed,
            &ranks,
            options.ordering_sweeps,
            true,
        )
    } else {
        let (forward, reverse, alternative) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
        (forward, reverse, alternative, None)
    };
    let quality_ordering = if reverse.crossings < forward.crossings {
        &reverse
    } else {
        &forward
    };
    let quality_layers = &quality_ordering.layers;
    let baseline_order_crossings = forward.crossings.min(reverse.crossings);
    let routing_plan = routing::RoutingPlan::new(&indexed, &ranks);
    let adaptive_gap_spacing = quality_effort != QualityEffort::Fast;
    let mut best: Option<AdmittedCandidate> = None;
    let mut admission_state = CandidateAdmissionState::default();
    let mut best_uses_primary_ranks = true;
    let candidate_routing = CandidateRouting {
        adaptive_gap_spacing,
        deeper_crossing_repair: quality_effort == QualityEffort::Max,
        ..CandidateRouting::default()
    };
    evaluate_candidate(
        &indexed,
        &routing_plan,
        &mut best,
        placement::place_baseline_nodes(&indexed, &ranks, &forward.layers, options),
        options,
        candidate_routing,
        &mut admission_state,
    );
    let ordinary_nodes = placement::place_nodes(&indexed, &ranks, quality_layers, options);
    let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary_nodes);
    let straight_chain = (quality_effort != QualityEffort::Fast
        && graph.nodes.len() <= placement::MAX_CHAIN_CANDIDATE_NODES)
        .then(|| {
            placement::place_straight_chain_nodes(
                &indexed,
                &ranks,
                quality_layers,
                &ordinary_nodes,
                options,
            )
        })
        .flatten();
    evaluate_candidate(
        &indexed,
        &routing_plan,
        &mut best,
        ordinary_nodes,
        options,
        CandidateRouting {
            sparse_global: true,
            ..candidate_routing
        },
        &mut admission_state,
    );
    if placement::preferred_alignment_can_be_significant(ordinary_alignment) {
        let preferred_nodes =
            placement::place_preferred_nodes(&indexed, &ranks, quality_layers, options);
        let preferred_alignment =
            placement::port_alignment_error(&indexed, &ranks, &preferred_nodes);
        if placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment) {
            evaluate_candidate(
                &indexed,
                &routing_plan,
                &mut best,
                preferred_nodes,
                options,
                CandidateRouting {
                    supplemental: true,
                    ..candidate_routing
                },
                &mut admission_state,
            );
        }
    }
    if let Some(sifted) = max_sifted
        && sifted.layers != *quality_layers
    {
        let sparse_global = graph.nodes.len() >= 1_000;
        evaluate_candidate(
            &indexed,
            &routing_plan,
            &mut best,
            placement::place_nodes(&indexed, &ranks, &sifted.layers, options),
            options,
            CandidateRouting {
                sparse_global,
                ..candidate_routing
            },
            &mut admission_state,
        );
    }
    if let Some(net_representative) = net_representative
        && net_representative.layers != *quality_layers
    {
        let (sparse_global, large_sparse_global) =
            net_representative_sparse_global_flags(graph.nodes.len(), quality_effort);
        if quality_effort == QualityEffort::Max && large_sparse_global {
            let nodes =
                placement::place_nodes(&indexed, &ranks, &net_representative.layers, options);
            let routed = routing::route_planned_candidates_with_quality_options(
                &routing_plan,
                &nodes,
                options,
                false,
                sparse_global,
                large_sparse_global,
                true,
                adaptive_gap_spacing,
                true,
            );
            retain_routed_candidates(
                &indexed,
                &mut best,
                nodes,
                routed,
                options,
                &mut admission_state,
            );
        } else {
            evaluate_candidate(
                &indexed,
                &routing_plan,
                &mut best,
                placement::place_nodes(&indexed, &ranks, &net_representative.layers, options),
                options,
                CandidateRouting {
                    sparse_global,
                    large_sparse_global,
                    ..candidate_routing
                },
                &mut admission_state,
            );
        }
    }
    if let Some(alternative_ranks) = latest_ranks {
        let (forward, reverse, _) = topology::order_layer_candidates(
            &indexed,
            &alternative_ranks,
            options.ordering_sweeps.min(3),
            false,
        );
        let layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let alternative_crossings = forward.crossings.min(reverse.crossings);
        if alternative_crossings < baseline_order_crossings
            && baseline_order_crossings - alternative_crossings
                >= baseline_order_crossings.div_ceil(100)
        {
            let nodes = placement::place_nodes(&indexed, &alternative_ranks, layers, options);
            let alternative_plan = routing::RoutingPlan::new(&indexed, &alternative_ranks);
            let routed = routing::route_planned_candidates_with_quality_options(
                &alternative_plan,
                &nodes,
                options,
                false,
                false,
                false,
                false,
                adaptive_gap_spacing,
                false,
            );
            if options.edge_node_clearance == 0.0 && options.minimum_parallel_wire_spacing == 0.0 {
                let quality = routed
                    .primary_quality
                    .expect("planned candidates include exact primary quality");
                if retain_owned_candidate(
                    &indexed,
                    &mut best,
                    quality,
                    nodes,
                    routed.primary,
                    options,
                    &mut admission_state,
                ) {
                    best_uses_primary_ranks = false;
                }
            } else {
                let mut alternative_best = None;
                retain_routed_candidates(
                    &indexed,
                    &mut alternative_best,
                    nodes,
                    routed,
                    options,
                    &mut admission_state,
                );
                if let Some(candidate) = alternative_best
                    && retain_better_admitted_candidate(&mut best, candidate)
                {
                    best_uses_primary_ranks = false;
                }
            }
        }
    }
    if let Some(straight_chain) = straight_chain
        && best.as_ref().is_none_or(|candidate| {
            placement::vertical_span(&straight_chain.nodes)
                <= candidate.raw_layout.height * placement::MAX_CHAIN_HEIGHT_FACTOR
        })
    {
        let mut straight_chain_best = None;
        evaluate_candidate(
            &indexed,
            &routing_plan,
            &mut straight_chain_best,
            straight_chain.nodes,
            options,
            CandidateRouting {
                supplemental: true,
                ..candidate_routing
            },
            &mut admission_state,
        );
        if let Some(candidate) = straight_chain_best
            && best.as_ref().is_none_or(|current| {
                straight_chain_cost_is_bounded(
                    &candidate,
                    current,
                    &straight_chain.edge_ids,
                    graph.nodes.len(),
                )
            })
            && retain_better_admitted_candidate(&mut best, candidate)
        {
            best_uses_primary_ranks = true;
        }
    }
    if options.edge_node_clearance > 0.0
        && best.is_none()
        && placement::demand_aware_spacing_is_relevant(
            &indexed,
            &ranks,
            quality_layers.len(),
            options,
        )
    {
        evaluate_candidate(
            &indexed,
            &routing_plan,
            &mut best,
            placement::place_demand_aware_nodes(&indexed, &ranks, quality_layers, options),
            options,
            CandidateRouting {
                sparse_global: true,
                ..candidate_routing
            },
            &mut admission_state,
        );
    }
    let selected = best.ok_or_else(|| hard_geometry_failure(options, &admission_state))?;
    let mut selection_quality = selected.selection_quality;
    let mut quality = selected.quality;
    let mut raw_layout = selected.raw_layout;
    let mut layout = selected.layout;
    let mut horizontal_pitch_applied = None;
    if quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && demand_aware_scale_is_eligible(graph.nodes.len(), graph.edges.len())
        && placement::demand_aware_spacing_is_relevant(
            &indexed,
            &ranks,
            quality_layers.len(),
            options,
        )
        && routing::route_parallel_congestion(&routing_plan, &raw_layout.edges)
            .is_some_and(|congestion| congestion >= 0.30)
    {
        let mut demand_aware_best = None;
        evaluate_candidate(
            &indexed,
            &routing_plan,
            &mut demand_aware_best,
            placement::place_demand_aware_nodes(&indexed, &ranks, quality_layers, options),
            options,
            CandidateRouting {
                sparse_global: true,
                ..candidate_routing
            },
            &mut admission_state,
        );
        if let Some(demand_candidate) = demand_aware_best
            && demand_aware_readability_is_better(
                &routing_plan,
                routing::route_quality(&indexed, &raw_layout.edges),
                &raw_layout,
                demand_candidate.selection_quality,
                &demand_candidate.raw_layout,
            )
        {
            selection_quality = demand_candidate.selection_quality;
            quality = demand_candidate.quality;
            raw_layout = demand_candidate.raw_layout;
            layout = demand_candidate.layout;
        }
    }
    if quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && let Some((candidate_quality, edges)) = routing::regional_fanout_candidate(
            &routing_plan,
            &raw_layout.nodes,
            &raw_layout.edges,
            routing::route_quality(&indexed, &raw_layout.edges),
            options,
        )
        && let Some(candidate) = prepare_owned_candidate(
            &indexed,
            candidate_quality,
            raw_layout.nodes.clone(),
            edges,
            options,
            &mut admission_state,
        )
        && candidate.layout.width * candidate.layout.height <= layout.width * layout.height * 1.05
        && route_quality_cmp(candidate.selection_quality, selection_quality).is_lt()
    {
        selection_quality = candidate.selection_quality;
        quality = candidate.quality;
        raw_layout = candidate.raw_layout;
        layout = candidate.layout;
    }
    let full_family_pitch = full_family_pitched_spacing_enabled(options);
    let pitched_baseline = (quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && options.edge_node_clearance > 0.0)
        .then(|| AdmittedCandidate {
            selection_quality,
            quality: exact_layout_route_quality(&indexed, &layout),
            raw_layout: raw_layout.clone(),
            layout: layout.clone(),
        });
    let mut pitched_refinement_applied = false;
    if quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && options.edge_node_clearance > 0.0
        && full_family_pitch
        && let Some((candidate_quality, nodes, edges, pitch)) =
            routing::selected_layout_horizontal_pitch_candidate(
                &routing_plan,
                &raw_layout.nodes,
                &raw_layout.edges,
                options,
            )
        && let Some(candidate) = prepare_owned_candidate(
            &indexed,
            candidate_quality,
            nodes,
            edges,
            options,
            &mut admission_state,
        )
        && candidate.layout.width * candidate.layout.height
            <= layout.width * layout.height * options.max_quality_area_factor
    {
        selection_quality = candidate.selection_quality;
        quality = candidate.quality;
        raw_layout = candidate.raw_layout;
        layout = candidate.layout;
        horizontal_pitch_applied = Some(pitch);
        pitched_refinement_applied = true;
    }
    if quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && options.edge_node_clearance > 0.0
        && let Some((candidate_quality, nodes, edges)) =
            routing::selected_layout_pitched_gap_candidate(
                &routing_plan,
                &raw_layout.nodes,
                &raw_layout.edges,
                options,
            )
        && let Some(candidate) = prepare_owned_candidate(
            &indexed,
            candidate_quality,
            nodes,
            edges,
            options,
            &mut admission_state,
        )
        && candidate.layout.width * candidate.layout.height
            <= layout.width * layout.height * options.max_quality_area_factor
        && (!full_family_pitch
            || routing::layout_vertical_gap_pitch_is_satisfied(
                &routing_plan,
                &candidate.raw_layout.nodes,
                &candidate.raw_layout.edges,
                options.route_lane_gap,
            ))
        && horizontal_pitch_applied.is_none_or(|pitch| {
            routing::layout_horizontal_crossing_pitch_is_satisfied(
                &routing_plan,
                &candidate.raw_layout.nodes,
                &candidate.raw_layout.edges,
                options,
                pitch,
            )
        })
    {
        selection_quality = candidate.selection_quality;
        quality = candidate.quality;
        raw_layout = candidate.raw_layout;
        layout = candidate.layout;
        pitched_refinement_applied = true;
    }
    if pitched_refinement_applied
        && let Some(baseline) = pitched_baseline
        && !composite_pitched_quality_is_admissible(
            baseline.quality,
            baseline.layout.width * baseline.layout.height,
            exact_layout_route_quality(&indexed, &layout),
            layout.width * layout.height,
            options,
        )
    {
        selection_quality = baseline.selection_quality;
        quality = baseline.quality;
        raw_layout = baseline.raw_layout;
        layout = baseline.layout;
    }
    let selected = AdmittedCandidate {
        selection_quality,
        quality,
        raw_layout,
        layout,
    };
    if cfg!(debug_assertions) {
        let exact = exact_layout_route_quality(&indexed, &selected.layout);
        assert_eq!(selected.quality.crossings, exact.crossings);
        assert_eq!(selected.quality.bends, exact.bends);
        let tolerance = quality
            .route_length
            .abs()
            .max(exact.route_length.abs())
            .max(1.0)
            * f64::EPSILON
            * 8.0;
        assert!(
            (selected.quality.route_length - exact.route_length).abs() <= tolerance,
            "retained quality must describe the returned layout"
        );
    }
    Ok(selected.layout)
}

fn composite_pitched_quality_is_admissible(
    baseline: routing::RouteQuality,
    baseline_area: f64,
    candidate: routing::RouteQuality,
    candidate_area: f64,
    options: LayoutOptions,
) -> bool {
    let crossing_allowance = baseline.crossings / 100;
    let bend_allowance = (baseline.bends / 100).max(2);
    candidate.crossings <= baseline.crossings.saturating_add(crossing_allowance)
        && candidate.bends <= baseline.bends.saturating_add(bend_allowance)
        && candidate.route_length <= baseline.route_length * options.max_quality_route_length_factor
        && candidate_area <= baseline_area * options.max_quality_area_factor
}

fn demand_aware_readability_is_better(
    plan: &routing::RoutingPlan<'_>,
    baseline_quality: routing::RouteQuality,
    baseline: &Layout,
    candidate_quality: routing::RouteQuality,
    candidate: &Layout,
) -> bool {
    routing::route_parallel_congestion(plan, &baseline.edges)
        .zip(routing::route_parallel_congestion(plan, &candidate.edges))
        .is_some_and(|(baseline_congestion, candidate_congestion)| {
            demand_aware_quality_is_better(
                baseline_quality,
                baseline.width * baseline.height,
                baseline_congestion,
                candidate_quality,
                candidate.width * candidate.height,
                candidate_congestion,
            )
        })
}

fn demand_aware_scale_is_eligible(nodes: usize, edges: usize) -> bool {
    (MIN_DEMAND_AWARE_SPACING_NODES..=MAX_DEMAND_AWARE_SPACING_NODES).contains(&nodes)
        && (MIN_DEMAND_AWARE_SPACING_EDGES..=MAX_DEMAND_AWARE_SPACING_EDGES).contains(&edges)
}

fn demand_aware_quality_is_better(
    baseline: routing::RouteQuality,
    baseline_area: f64,
    baseline_congestion: f64,
    candidate: routing::RouteQuality,
    candidate_area: f64,
    candidate_congestion: f64,
) -> bool {
    let allowed_crossings = baseline.crossings.saturating_add(baseline.crossings / 20);
    candidate.crossings <= allowed_crossings
        && candidate.bends as f64 <= baseline.bends as f64 * 1.05
        && candidate.route_length <= baseline.route_length * 1.65
        && candidate_area <= baseline_area * 1.85
        && baseline_congestion >= 0.30
        && candidate_congestion <= baseline_congestion * 0.35
}

fn straight_chain_cost_is_bounded(
    candidate: &AdmittedCandidate,
    current: &AdmittedCandidate,
    chain_edge_ids: &[EdgeId],
    node_count: usize,
) -> bool {
    let (newly_straight, lost_straight) =
        straight_chain_route_gain(&candidate.layout, &current.layout, chain_edge_ids);
    straight_chain_geometry_cost_is_bounded(
        candidate.selection_quality,
        &candidate.raw_layout,
        current.selection_quality,
        &current.raw_layout,
    ) && straight_chain_geometry_cost_is_bounded(
        candidate.quality,
        &candidate.layout,
        current.quality,
        &current.layout,
    ) && newly_straight >= 2
        && newly_straight > lost_straight
        && (node_count <= 600
            || straight_chain_large_gain_is_significant(newly_straight, chain_edge_ids.len()))
}

fn straight_chain_large_gain_is_significant(newly_straight: usize, chain_edges: usize) -> bool {
    newly_straight.saturating_mul(5).saturating_add(1) >= chain_edges
}

fn straight_chain_geometry_cost_is_bounded(
    quality: routing::RouteQuality,
    layout: &Layout,
    current_quality: routing::RouteQuality,
    current: &Layout,
) -> bool {
    quality.route_length <= current_quality.route_length * 1.05
        && quality.bends
            <= current_quality
                .bends
                .saturating_add(current_quality.bends / 20)
        && layout.width * layout.height <= current.width * current.height * 1.10
}

fn straight_chain_route_gain(
    candidate: &Layout,
    current: &Layout,
    chain_edge_ids: &[EdgeId],
) -> (usize, usize) {
    let straight_ids = |layout: &Layout| {
        layout
            .edges
            .iter()
            .filter(|edge| {
                chain_edge_ids.binary_search(&edge.id).is_ok()
                    && edge
                        .points
                        .first()
                        .is_some_and(|first| edge.points.iter().all(|point| point.y == first.y))
            })
            .map(|edge| edge.id)
            .collect::<BTreeSet<_>>()
    };
    let candidate = straight_ids(candidate);
    let current = straight_ids(current);
    (
        candidate.difference(&current).count(),
        current.difference(&candidate).count(),
    )
}

#[derive(Clone, Copy, Default)]
struct CandidateRouting {
    supplemental: bool,
    sparse_global: bool,
    large_sparse_global: bool,
    adaptive_gap_spacing: bool,
    deeper_crossing_repair: bool,
}

#[derive(Default)]
struct CandidateAdmissionState {
    boundary_bundle_rejected: bool,
    boundary_bundle_work_exhausted: bool,
    clearance_work_exhausted: bool,
    clearance_satisfied: bool,
    contact_segment_exhausted: bool,
    contact_work_exhausted: bool,
    contact_rejected: bool,
    contact_satisfied: bool,
    parallel_spacing_segment_exhausted: bool,
    parallel_spacing_work_exhausted: bool,
    parallel_spacing_rejected: bool,
    parallel_spacing_satisfied: bool,
}

fn hard_geometry_failure(
    options: LayoutOptions,
    admission_state: &CandidateAdmissionState,
) -> LayoutError {
    if admission_state.boundary_bundle_work_exhausted {
        LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
            maximum: boundary_bundles::MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
        }
    } else if admission_state.boundary_bundle_rejected {
        LayoutError::BoundaryBundleGeometryUnsatisfied
    } else if admission_state.contact_segment_exhausted {
        LayoutError::UnrelatedRouteContactSegmentLimitExceeded {
            maximum: MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
        }
    } else if admission_state.contact_work_exhausted {
        LayoutError::UnrelatedRouteContactWorkLimitExceeded {
            maximum: MAX_LAYOUT_ROUTE_CONTACT_VISITS,
        }
    } else if admission_state.clearance_work_exhausted {
        LayoutError::EdgeNodeClearanceWorkLimitExceeded {
            maximum: MAX_LAYOUT_CLEARANCE_PAIR_VISITS,
        }
    } else if admission_state.parallel_spacing_segment_exhausted {
        LayoutError::ParallelWireSpacingSegmentLimitExceeded {
            maximum: MAX_LAYOUT_PARALLEL_WIRE_SPACING_SEGMENTS,
        }
    } else if admission_state.parallel_spacing_work_exhausted {
        LayoutError::ParallelWireSpacingWorkLimitExceeded {
            maximum: MAX_LAYOUT_PARALLEL_WIRE_SPACING_VISITS,
        }
    } else if admission_state.contact_rejected && !admission_state.contact_satisfied {
        LayoutError::UnrelatedRouteContactUnsatisfied
    } else if admission_state.clearance_satisfied
        && admission_state.parallel_spacing_rejected
        && !admission_state.parallel_spacing_satisfied
    {
        LayoutError::ParallelWireSpacingUnsatisfied {
            spacing: options.minimum_parallel_wire_spacing,
        }
    } else {
        LayoutError::EdgeNodeClearanceUnsatisfied {
            clearance: options.edge_node_clearance,
        }
    }
}

fn evaluate_candidate(
    indexed: &validation::IndexedGraph<'_>,
    routing_plan: &routing::RoutingPlan<'_>,
    best: &mut Option<AdmittedCandidate>,
    nodes: Vec<NodeGeometry>,
    options: LayoutOptions,
    routing: CandidateRouting,
    admission_state: &mut CandidateAdmissionState,
) {
    let routed = routing::route_planned_candidates_with_quality_options(
        routing_plan,
        &nodes,
        options,
        routing.supplemental,
        routing.sparse_global,
        routing.large_sparse_global,
        false,
        routing.adaptive_gap_spacing,
        routing.deeper_crossing_repair,
    );
    retain_routed_candidates(indexed, best, nodes, routed, options, admission_state);
}

fn retain_routed_candidates(
    indexed: &validation::IndexedGraph<'_>,
    best: &mut Option<AdmittedCandidate>,
    nodes: Vec<NodeGeometry>,
    routed: routing::RoutedEdges,
    options: LayoutOptions,
    admission_state: &mut CandidateAdmissionState,
) {
    let quality = routed
        .primary_quality
        .unwrap_or_else(|| routing::route_quality(indexed, &routed.primary));
    retain_owned_candidate(
        indexed,
        best,
        quality,
        nodes.clone(),
        routed.primary,
        options,
        admission_state,
    );
    if let Some((quality, edges)) = routed.repair {
        retain_owned_candidate(
            indexed,
            best,
            quality,
            nodes.clone(),
            edges,
            options,
            admission_state,
        );
    }
    for (quality, edges) in routed.alternatives {
        retain_owned_candidate(
            indexed,
            best,
            quality,
            nodes.clone(),
            edges,
            options,
            admission_state,
        );
    }
}

fn net_representative_sparse_global_flags(
    node_count: usize,
    quality_effort: QualityEffort,
) -> (bool, bool) {
    match quality_effort {
        QualityEffort::Fast => (false, false),
        QualityEffort::Quality | QualityEffort::Max => {
            let admitted = (600..=1_000).contains(&node_count);
            (admitted, admitted)
        }
    }
}

#[cfg(test)]
fn retain_better_candidate(
    best: &mut Option<(routing::RouteQuality, Layout)>,
    quality: routing::RouteQuality,
    candidate: Layout,
) -> bool {
    let replace = best.as_ref().is_none_or(|(current_quality, current)| {
        candidate_quality_cmp(quality, &candidate, *current_quality, current).is_lt()
    });
    if replace {
        *best = Some((quality, candidate));
    }
    replace
}

#[derive(Clone, Debug)]
struct AdmittedCandidate {
    selection_quality: routing::RouteQuality,
    quality: routing::RouteQuality,
    raw_layout: Layout,
    layout: Layout,
}

fn retain_better_admitted_candidate(
    best: &mut Option<AdmittedCandidate>,
    candidate: AdmittedCandidate,
) -> bool {
    let replace = best.as_ref().is_none_or(|current| {
        candidate_quality_cmp(
            candidate.selection_quality,
            &candidate.raw_layout,
            current.selection_quality,
            &current.raw_layout,
        )
        .is_lt()
    });
    if replace {
        *best = Some(candidate);
    }
    replace
}

fn retain_owned_candidate(
    indexed: &validation::IndexedGraph<'_>,
    best: &mut Option<AdmittedCandidate>,
    quality: routing::RouteQuality,
    nodes: Vec<NodeGeometry>,
    edges: Vec<EdgeGeometry>,
    options: LayoutOptions,
    admission_state: &mut CandidateAdmissionState,
) -> bool {
    if best
        .as_ref()
        .is_some_and(|current| route_quality_cmp(quality, current.selection_quality).is_gt())
    {
        return false;
    }
    let Some(candidate) =
        prepare_owned_candidate(indexed, quality, nodes, edges, options, admission_state)
    else {
        return false;
    };
    retain_better_admitted_candidate(best, candidate)
}

fn prepare_owned_candidate(
    indexed: &validation::IndexedGraph<'_>,
    quality: routing::RouteQuality,
    nodes: Vec<NodeGeometry>,
    edges: Vec<EdgeGeometry>,
    options: LayoutOptions,
    admission_state: &mut CandidateAdmissionState,
) -> Option<AdmittedCandidate> {
    let raw_layout = placement::normalize_owned(nodes, edges);
    let mut candidate = raw_layout.clone();
    if !indexed.boundary_bundles.is_empty() {
        candidate = match boundary_bundles::apply_and_normalize(indexed, candidate, options) {
            Ok(candidate) => candidate,
            Err(LayoutError::BoundaryBundleGeometryWorkLimitExceeded { .. }) => {
                admission_state.boundary_bundle_work_exhausted = true;
                return None;
            }
            Err(_) => {
                admission_state.boundary_bundle_rejected = true;
                return None;
            }
        };
    }
    if !candidate_satisfies_hard_geometry_contract(indexed, &candidate, options, admission_state) {
        return None;
    }
    let exact_quality = if indexed.boundary_bundles.is_empty() {
        quality
    } else {
        boundary_bundles::route_quality(indexed, &candidate)
    };
    Some(AdmittedCandidate {
        selection_quality: quality,
        quality: exact_quality,
        raw_layout,
        layout: candidate,
    })
}

fn exact_layout_route_quality(
    indexed: &validation::IndexedGraph<'_>,
    layout: &Layout,
) -> routing::RouteQuality {
    if indexed.boundary_bundles.is_empty() {
        routing::route_quality(indexed, &layout.edges)
    } else {
        boundary_bundles::route_quality(indexed, layout)
    }
}

#[cfg(test)]
fn retain_owned_candidate_unchecked(
    best: &mut Option<(routing::RouteQuality, Layout)>,
    quality: routing::RouteQuality,
    nodes: Vec<NodeGeometry>,
    edges: Vec<EdgeGeometry>,
) -> bool {
    if best
        .as_ref()
        .is_some_and(|(current_quality, _)| route_quality_cmp(quality, *current_quality).is_gt())
    {
        return false;
    }
    retain_better_candidate(best, quality, placement::normalize_owned(nodes, edges))
}

pub(crate) fn effective_layout_options(mut options: LayoutOptions) -> LayoutOptions {
    options.route_lane_gap = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    if options.edge_node_clearance > 0.0 {
        options.layer_gap = options
            .layer_gap
            .max(options.edge_node_clearance * 2.0 + options.route_lane_gap);
    }
    options
}

pub(crate) fn full_family_pitched_spacing_enabled(options: LayoutOptions) -> bool {
    let defaults = LayoutOptions::default();
    options.max_quality_area_factor > defaults.max_quality_area_factor
        && options.max_quality_route_length_factor > defaults.max_quality_route_length_factor
}

/// Distance used only where a route must move outward beyond endpoint-node obstacles.
///
/// Track planning continues to use the requested `port_stub`; widening every planning margin
/// for semantic clearance needlessly concentrates dense routing. The exact final admission gate
/// remains authoritative.
pub(crate) fn outward_obstacle_clearance_stub(options: LayoutOptions) -> f64 {
    options.port_stub.max(options.edge_node_clearance)
}

/// Horizontal depth of a graphical boundary-bundle spine.
///
/// Positive-clearance sparse channels begin immediately beyond the clearance envelope. Extending
/// the spine by the endpoint stub or another lane pitch can therefore intrude into the first legal
/// channel. Zero-clearance layouts retain the original visible stub plus one lane gap.
pub(crate) fn boundary_bundle_rail_depth(options: LayoutOptions) -> f64 {
    if options.edge_node_clearance > 0.0 {
        options.edge_node_clearance
    } else {
        options.port_stub + options.route_lane_gap
    }
}

fn candidate_satisfies_hard_geometry_contract(
    indexed: &validation::IndexedGraph<'_>,
    candidate: &Layout,
    options: LayoutOptions,
    admission_state: &mut CandidateAdmissionState,
) -> bool {
    if options.edge_node_clearance > 0.0 {
        match routing::route_family_has_unrelated_contact_bounded(
            indexed,
            &candidate.edges,
            MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
            MAX_LAYOUT_ROUTE_CONTACT_VISITS,
        ) {
            Ok(false) => admission_state.contact_satisfied = true,
            Ok(true) | Err(routing::RouteContactError::InvalidInput) => {
                admission_state.contact_rejected = true;
                return false;
            }
            Err(routing::RouteContactError::WorkLimitExceeded) => {
                admission_state.contact_work_exhausted = true;
                return false;
            }
            Err(routing::RouteContactError::SegmentLimitExceeded) => {
                admission_state.contact_segment_exhausted = true;
                return false;
            }
        }
        if !candidate_satisfies_edge_node_clearance(
            indexed,
            candidate,
            options,
            &mut admission_state.clearance_work_exhausted,
        ) {
            return false;
        }
        admission_state.clearance_satisfied = true;
    } else {
        admission_state.clearance_satisfied = true;
    }
    if options.minimum_parallel_wire_spacing > 0.0 {
        match routing::route_family_satisfies_parallel_spacing_bounded(
            indexed,
            &candidate.edges,
            options.minimum_parallel_wire_spacing,
            outward_obstacle_clearance_stub(options),
            MAX_LAYOUT_PARALLEL_WIRE_SPACING_SEGMENTS,
            MAX_LAYOUT_PARALLEL_WIRE_SPACING_VISITS,
        ) {
            Ok(true) => admission_state.parallel_spacing_satisfied = true,
            Ok(false) | Err(routing::ParallelWireSpacingError::InvalidInput) => {
                admission_state.parallel_spacing_rejected = true;
                return false;
            }
            Err(routing::ParallelWireSpacingError::WorkLimitExceeded) => {
                admission_state.parallel_spacing_work_exhausted = true;
                return false;
            }
            Err(routing::ParallelWireSpacingError::SegmentLimitExceeded) => {
                admission_state.parallel_spacing_segment_exhausted = true;
                return false;
            }
        }
    }
    true
}

fn candidate_satisfies_edge_node_clearance(
    indexed: &validation::IndexedGraph<'_>,
    candidate: &Layout,
    options: LayoutOptions,
    clearance_work_exhausted: &mut bool,
) -> bool {
    candidate_satisfies_edge_node_clearance_bounded(
        indexed,
        candidate,
        options,
        MAX_LAYOUT_CLEARANCE_PAIR_VISITS,
        clearance_work_exhausted,
    )
}

fn candidate_satisfies_edge_node_clearance_bounded(
    indexed: &validation::IndexedGraph<'_>,
    candidate: &Layout,
    options: LayoutOptions,
    max_pair_visits: usize,
    clearance_work_exhausted: &mut bool,
) -> bool {
    if options.edge_node_clearance == 0.0 {
        return true;
    }
    match routing::route_edge_node_clearance(
        indexed,
        &candidate.nodes,
        &candidate.edges,
        options.edge_node_clearance,
        max_pair_visits,
    ) {
        Ok(clearance) => clearance.violations == 0,
        Err(EdgeNodeClearanceError::WorkLimitExceeded) => {
            *clearance_work_exhausted = true;
            false
        }
        Err(EdgeNodeClearanceError::InvalidInput) => false,
    }
}

fn route_quality_cmp(
    left: routing::RouteQuality,
    right: routing::RouteQuality,
) -> std::cmp::Ordering {
    left.crossings
        .cmp(&right.crossings)
        .then(left.bends.cmp(&right.bends))
        .then(left.route_length.total_cmp(&right.route_length))
}

fn candidate_quality_cmp(
    left: routing::RouteQuality,
    left_layout: &Layout,
    right: routing::RouteQuality,
    right_layout: &Layout,
) -> std::cmp::Ordering {
    route_quality_cmp(left, right).then(
        (left_layout.width * left_layout.height)
            .total_cmp(&(right_layout.width * right_layout.height)),
    )
}

#[cfg(test)]
mod tests {
    use super::{
        AdmittedCandidate, BoundaryBundleConstraint, BoundaryBundleMemberConstraint,
        CandidateAdmissionState, CandidateRouting, Edge, EdgeGeometry, EdgeId, Endpoint, Graph,
        Layout, LayoutConstraints, LayoutError, LayoutOptions, MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
        Node, NodeGeometry, Point, Port, PortSide, QualityEffort, boundary_bundle_rail_depth,
        candidate_quality_cmp, candidate_satisfies_edge_node_clearance_bounded,
        composite_pitched_quality_is_admissible, demand_aware_quality_is_better,
        demand_aware_scale_is_eligible, effective_layout_options, effective_ranking_edges,
        evaluate_candidate, full_family_pitched_spacing_enabled, hard_geometry_failure, layout,
        outward_obstacle_clearance_stub, placement, retain_better_admitted_candidate,
        retain_better_candidate, retain_owned_candidate, retain_owned_candidate_unchecked, routing,
        routing::RouteQuality, straight_chain_cost_is_bounded,
        straight_chain_large_gain_is_significant, topology, validation,
    };

    mod active_fanout_fixture {
        use crate as schemweave;

        include!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/support/active_fanout.rs"
        ));
    }

    fn candidate(
        crossings: usize,
        bends: usize,
        route_length: f64,
        area: f64,
    ) -> (RouteQuality, Layout) {
        (
            RouteQuality {
                crossings,
                bends,
                route_length,
            },
            Layout {
                nodes: Vec::new(),
                edges: Vec::new(),
                boundary_bundles: Vec::new(),
                width: area,
                height: 1.0,
            },
        )
    }

    #[test]
    fn admitted_initial_candidate_selection_uses_raw_quality_and_area() {
        let selection_quality = RouteQuality {
            crossings: 1,
            bends: 2,
            route_length: 3.0,
        };
        let admitted = |raw_area, rendered_area, rendered_crossings| AdmittedCandidate {
            selection_quality,
            quality: RouteQuality {
                crossings: rendered_crossings,
                ..selection_quality
            },
            raw_layout: candidate(0, 0, 0.0, raw_area).1,
            layout: candidate(0, 0, 0.0, rendered_area).1,
        };
        let mut best = None;
        assert!(retain_better_admitted_candidate(
            &mut best,
            admitted(10.0, 100.0, 9),
        ));
        assert!(!retain_better_admitted_candidate(
            &mut best,
            admitted(20.0, 1.0, 0),
        ));
        let selected = best.unwrap();
        assert_eq!(selected.raw_layout.width, 10.0);
        assert_eq!(selected.layout.width, 100.0);
        assert_eq!(selected.quality.crossings, 9);
    }

    #[test]
    fn effective_spacing_options_are_canonical_idempotent_and_zero_identity() {
        let defaults = LayoutOptions::default();
        assert_eq!(effective_layout_options(defaults), defaults);
        assert_eq!(
            boundary_bundle_rail_depth(defaults),
            defaults.port_stub + defaults.route_lane_gap
        );
        for clearance in [f64::EPSILON, 1.0, 9.0, 10.0, 11.0, 29.0, 30.0, 31.0] {
            assert_eq!(
                boundary_bundle_rail_depth(LayoutOptions {
                    edge_node_clearance: clearance,
                    ..defaults
                }),
                clearance
            );
        }

        let requested = LayoutOptions {
            edge_node_clearance: 40.0,
            ..defaults
        };
        let effective = effective_layout_options(requested);
        assert_eq!(effective.port_stub, defaults.port_stub);
        assert_eq!(effective.layer_gap, 84.0);
        assert_eq!(effective_layout_options(effective), effective);
        assert_eq!(outward_obstacle_clearance_stub(effective), 40.0);

        let parallel_spacing = effective_layout_options(LayoutOptions {
            minimum_parallel_wire_spacing: 6.0,
            ..defaults
        });
        assert_eq!(parallel_spacing.route_lane_gap, 6.0);
        assert_eq!(parallel_spacing.minimum_parallel_wire_spacing, 6.0);
        assert_eq!(effective_layout_options(parallel_spacing), parallel_spacing);
    }

    #[test]
    fn bundle_aware_candidate_admission_rejects_a_raw_winner_and_keeps_its_clean_sibling() {
        let endpoint_node = |id, side| Node {
            id,
            width: 10.0,
            height: 10.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side,
                offset: 5.0,
            }],
        };
        let graph = Graph {
            nodes: vec![
                endpoint_node(1, PortSide::East),
                endpoint_node(2, PortSide::West),
                Node {
                    id: 3,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: Vec::new(),
                },
            ],
            edges: vec![Edge {
                id: 10,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 10,
                participates_in_ranking: true,
            }],
        };
        let options = LayoutOptions::default();
        let constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![2],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 1,
                endpoint: Endpoint { node: 1, port: 0 },
                width: 1,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 10,
                    slots: vec![0],
                }],
            }],
        };
        let indexed =
            validation::validate_and_index_with_constraints(&graph, options, &constraints).unwrap();
        let nodes = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 3,
                x: 30.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
        ];
        let raw_winner = RouteQuality {
            crossings: 0,
            bends: 2,
            route_length: 120.0,
        };
        let clean_sibling = RouteQuality {
            crossings: 1,
            bends: 4,
            route_length: 130.0,
        };
        let mut best = None;
        let mut admission_state = CandidateAdmissionState::default();
        assert!(!retain_owned_candidate(
            &indexed,
            &mut best,
            raw_winner,
            nodes.clone(),
            vec![EdgeGeometry {
                id: 10,
                points: vec![Point { x: 10.0, y: 5.0 }, Point { x: 100.0, y: 5.0 },],
            }],
            options,
            &mut admission_state,
        ));
        assert!(best.is_none());
        assert!(retain_owned_candidate(
            &indexed,
            &mut best,
            clean_sibling,
            nodes,
            vec![EdgeGeometry {
                id: 10,
                points: vec![
                    Point { x: 10.0, y: 5.0 },
                    Point { x: 10.0, y: 20.0 },
                    Point { x: 25.0, y: 20.0 },
                    Point { x: 100.0, y: 20.0 },
                    Point { x: 100.0, y: 5.0 },
                ],
            }],
            options,
            &mut admission_state,
        ));
        let selected = best.unwrap().layout;
        assert_eq!(selected.boundary_bundles.len(), 1);
        assert_eq!(
            selected.edges[0].points[0],
            selected.boundary_bundles[0].members[0].tap
        );
        assert!(admission_state.boundary_bundle_rejected);
    }

    #[test]
    fn full_family_pitched_spacing_requires_both_quality_budgets() {
        let defaults = LayoutOptions::default();
        assert!(!full_family_pitched_spacing_enabled(defaults));
        assert!(!full_family_pitched_spacing_enabled(LayoutOptions {
            max_quality_area_factor: 2.0,
            ..defaults
        }));
        assert!(!full_family_pitched_spacing_enabled(LayoutOptions {
            max_quality_route_length_factor: 1.25,
            ..defaults
        }));
        assert!(full_family_pitched_spacing_enabled(LayoutOptions {
            max_quality_area_factor: 2.0,
            max_quality_route_length_factor: 1.25,
            ..defaults
        }));
    }

    #[test]
    fn demand_aware_spacing_requires_material_congestion_relief_with_bounded_cost() {
        for (nodes, edges, eligible) in [
            (149, 250, false),
            (150, 249, false),
            (150, 250, true),
            (400, 400, true),
            (401, 400, false),
            (400, 401, false),
        ] {
            assert_eq!(
                demand_aware_scale_is_eligible(nodes, edges),
                eligible,
                "nodes={nodes}, edges={edges}",
            );
        }
        let baseline = RouteQuality {
            crossings: 800,
            bends: 1_200,
            route_length: 140_000.0,
        };
        let candidate = RouteQuality {
            crossings: 840,
            bends: 1_260,
            route_length: 231_000.0,
        };
        assert!(demand_aware_quality_is_better(
            baseline,
            5_600_000.0,
            0.43,
            candidate,
            10_360_000.0,
            0.15,
        ));
        for rejected in [
            RouteQuality {
                crossings: 841,
                ..candidate
            },
            RouteQuality {
                bends: 1_261,
                ..candidate
            },
            RouteQuality {
                route_length: 231_000.01,
                ..candidate
            },
        ] {
            assert!(!demand_aware_quality_is_better(
                baseline,
                5_600_000.0,
                0.43,
                rejected,
                10_360_000.0,
                0.15,
            ));
        }
        for crossings in [0, 1, 19] {
            let baseline = RouteQuality {
                crossings,
                ..baseline
            };
            let candidate = RouteQuality {
                crossings: crossings + 1,
                ..candidate
            };
            assert!(!demand_aware_quality_is_better(
                baseline,
                5_600_000.0,
                0.43,
                candidate,
                10_360_000.0,
                0.15,
            ));
        }
        assert!(!demand_aware_quality_is_better(
            baseline,
            5_600_000.0,
            0.29,
            candidate,
            10_360_000.0,
            0.10,
        ));
        assert!(!demand_aware_quality_is_better(
            baseline,
            5_600_000.0,
            0.43,
            candidate,
            10_360_000.01,
            0.15,
        ));
        assert!(!demand_aware_quality_is_better(
            baseline,
            5_600_000.0,
            0.43,
            candidate,
            10_360_000.0,
            0.150_500_000_1,
        ));
    }

    #[test]
    fn effective_ranking_keeps_acyclic_data_into_a_cycle_breaker() {
        let node = |id, cycle_breaker| Node {
            id,
            width: 80.0,
            height: 50.0,
            cycle_breaker,
            ports: vec![
                Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 25.0,
                },
                Port {
                    id: 1,
                    side: PortSide::East,
                    offset: 25.0,
                },
            ],
        };
        let graph = Graph {
            nodes: vec![node(1, false), node(2, false), node(3, true)],
            edges: vec![
                Edge {
                    id: 1,
                    source: Endpoint { node: 1, port: 1 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 1,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 2,
                    source: Endpoint { node: 2, port: 1 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 2,
                    participates_in_ranking: true,
                },
            ],
        };

        assert_eq!(
            effective_ranking_edges(&graph),
            [1, 2].into_iter().collect()
        );
    }

    fn browser_max_effort_graph() -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..600)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 25.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 25.0,
                    },
                ],
            })
            .collect();
        let mut state = 10;
        let mut endpoints = Vec::new();
        for layer in 0..3u32 {
            let source_start = layer * 50;
            let target_start = (layer + 1) * 50;
            for source in source_start..source_start + 50 {
                for target in target_start..target_start + 50 {
                    if next(&mut state) % 100 < 16 {
                        endpoints.push((source, target, source));
                    }
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, net))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    #[test]
    fn net_representative_sparse_global_is_admitted_only_for_medium_large_graphs() {
        assert_eq!(
            super::net_representative_sparse_global_flags(599, QualityEffort::Quality),
            (false, false)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(600, QualityEffort::Quality),
            (true, true)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(1_000, QualityEffort::Quality),
            (true, true)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(1_001, QualityEffort::Quality),
            (false, false)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(855, QualityEffort::Fast),
            (false, false)
        );
        assert_eq!(
            super::net_representative_sparse_global_flags(855, QualityEffort::Max),
            (true, true)
        );
    }

    #[test]
    fn clearance_admission_fails_closed_at_the_exact_work_limit() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 1,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 10.0,
                    }],
                },
                Node {
                    id: 2,
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
                    id: 3,
                    width: 20.0,
                    height: 20.0,
                    cycle_breaker: false,
                    ports: vec![],
                },
            ],
            edges: vec![Edge {
                id: 1,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 1,
                participates_in_ranking: true,
            }],
        };
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            ..LayoutOptions::default()
        };
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let candidate = Layout {
            nodes: vec![
                NodeGeometry {
                    id: 1,
                    x: 0.0,
                    y: 0.0,
                    width: 20.0,
                    height: 20.0,
                },
                NodeGeometry {
                    id: 2,
                    x: 100.0,
                    y: 0.0,
                    width: 20.0,
                    height: 20.0,
                },
                NodeGeometry {
                    id: 3,
                    x: 50.0,
                    y: 20.0,
                    width: 20.0,
                    height: 20.0,
                },
            ],
            edges: vec![EdgeGeometry {
                id: 1,
                points: vec![Point { x: 20.0, y: 10.0 }, Point { x: 100.0, y: 10.0 }],
            }],
            boundary_bundles: Vec::new(),
            width: 120.0,
            height: 40.0,
        };
        let mut exhausted = false;
        assert!(!candidate_satisfies_edge_node_clearance_bounded(
            &indexed,
            &candidate,
            options,
            0,
            &mut exhausted,
        ));
        assert!(exhausted);
    }

    #[test]
    fn exact_admission_keeps_a_safe_route_family_after_rejecting_the_quality_winner() {
        let graph = Graph {
            nodes: vec![
                Node {
                    id: 1,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::East,
                        offset: 5.0,
                    }],
                },
                Node {
                    id: 2,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: vec![Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 5.0,
                    }],
                },
                Node {
                    id: 3,
                    width: 10.0,
                    height: 10.0,
                    cycle_breaker: false,
                    ports: Vec::new(),
                },
            ],
            edges: vec![Edge {
                id: 1,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 1,
                participates_in_ranking: true,
            }],
        };
        let options = LayoutOptions {
            edge_node_clearance: 10.0,
            ..LayoutOptions::default()
        };
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let nodes = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 3,
                x: 45.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
        ];
        let winning_quality = RouteQuality {
            crossings: 0,
            bends: 0,
            route_length: 90.0,
        };
        let safe_quality = RouteQuality {
            crossings: 0,
            bends: 2,
            route_length: 140.0,
        };
        let mut best = None;
        let mut admission_state = CandidateAdmissionState::default();
        assert!(!retain_owned_candidate(
            &indexed,
            &mut best,
            winning_quality,
            nodes.clone(),
            vec![EdgeGeometry {
                id: 1,
                points: vec![Point { x: 10.0, y: 5.0 }, Point { x: 100.0, y: 5.0 }],
            }],
            options,
            &mut admission_state,
        ));
        assert!(best.is_none());
        assert!(retain_owned_candidate(
            &indexed,
            &mut best,
            safe_quality,
            nodes,
            vec![EdgeGeometry {
                id: 1,
                points: vec![
                    Point { x: 10.0, y: 5.0 },
                    Point { x: 10.0, y: 30.0 },
                    Point { x: 100.0, y: 30.0 },
                    Point { x: 100.0, y: 5.0 },
                ],
            }],
            options,
            &mut admission_state,
        ));
        assert_eq!(best.unwrap().selection_quality, safe_quality);
        assert!(!admission_state.clearance_work_exhausted);
        assert!(!admission_state.contact_work_exhausted);
    }

    #[test]
    fn positive_clearance_admission_rejects_an_overlapping_quality_winner() {
        let endpoint_node = |id, side| Node {
            id,
            width: 10.0,
            height: 10.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side,
                offset: 5.0,
            }],
        };
        let graph = Graph {
            nodes: vec![
                endpoint_node(1, PortSide::East),
                endpoint_node(2, PortSide::West),
                endpoint_node(3, PortSide::East),
                endpoint_node(4, PortSide::West),
            ],
            edges: vec![
                Edge {
                    id: 1,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 1,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 2,
                    source: Endpoint { node: 3, port: 0 },
                    target: Endpoint { node: 4, port: 0 },
                    net: 2,
                    participates_in_ranking: true,
                },
            ],
        };
        let options = LayoutOptions {
            edge_node_clearance: 10.0,
            ..LayoutOptions::default()
        };
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let nodes = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 3,
                x: 0.0,
                y: 100.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 4,
                x: 100.0,
                y: 100.0,
                width: 10.0,
                height: 10.0,
            },
        ];
        let overlapping = vec![
            EdgeGeometry {
                id: 1,
                points: vec![
                    Point { x: 10.0, y: 5.0 },
                    Point { x: 20.0, y: 5.0 },
                    Point { x: 20.0, y: 50.0 },
                    Point { x: 100.0, y: 50.0 },
                    Point { x: 100.0, y: 5.0 },
                ],
            },
            EdgeGeometry {
                id: 2,
                points: vec![
                    Point { x: 10.0, y: 105.0 },
                    Point { x: 20.0, y: 105.0 },
                    Point { x: 20.0, y: 50.0 },
                    Point { x: 100.0, y: 50.0 },
                    Point { x: 100.0, y: 105.0 },
                ],
            },
        ];
        let clean = vec![
            overlapping[0].clone(),
            EdgeGeometry {
                id: 2,
                points: vec![
                    Point { x: 10.0, y: 105.0 },
                    Point { x: 20.0, y: 105.0 },
                    Point { x: 20.0, y: 70.0 },
                    Point { x: 100.0, y: 70.0 },
                    Point { x: 100.0, y: 105.0 },
                ],
            },
        ];
        let quality_winner = RouteQuality {
            crossings: 0,
            bends: 6,
            route_length: 300.0,
        };
        let safe_sibling = RouteQuality {
            crossings: 1,
            bends: 6,
            route_length: 260.0,
        };
        let mut best = None;
        let mut admission_state = CandidateAdmissionState::default();

        assert!(!retain_owned_candidate(
            &indexed,
            &mut best,
            quality_winner,
            nodes.clone(),
            overlapping,
            options,
            &mut admission_state,
        ));
        assert!(retain_owned_candidate(
            &indexed,
            &mut best,
            safe_sibling,
            nodes,
            clean,
            options,
            &mut admission_state,
        ));
        assert_eq!(best.unwrap().selection_quality, safe_sibling);
        assert!(!admission_state.clearance_work_exhausted);
        assert!(!admission_state.contact_work_exhausted);
        assert!(admission_state.contact_rejected);
        assert!(admission_state.contact_satisfied);
    }

    #[test]
    fn parallel_spacing_admission_rejects_a_close_quality_winner_and_keeps_a_safe_sibling() {
        let endpoint_node = |id, side| Node {
            id,
            width: 10.0,
            height: 10.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side,
                offset: 5.0,
            }],
        };
        let graph = Graph {
            nodes: vec![
                endpoint_node(1, PortSide::East),
                endpoint_node(2, PortSide::West),
                endpoint_node(3, PortSide::East),
                endpoint_node(4, PortSide::West),
            ],
            edges: vec![
                Edge {
                    id: 1,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 1,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 2,
                    source: Endpoint { node: 3, port: 0 },
                    target: Endpoint { node: 4, port: 0 },
                    net: 2,
                    participates_in_ranking: true,
                },
            ],
        };
        let options = LayoutOptions {
            minimum_parallel_wire_spacing: 6.0,
            ..LayoutOptions::default()
        };
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let nodes = vec![
            NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 2,
                x: 100.0,
                y: 0.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 3,
                x: 0.0,
                y: 100.0,
                width: 10.0,
                height: 10.0,
            },
            NodeGeometry {
                id: 4,
                x: 100.0,
                y: 100.0,
                width: 10.0,
                height: 10.0,
            },
        ];
        let routes_at = |second_track| {
            vec![
                EdgeGeometry {
                    id: 1,
                    points: vec![
                        Point { x: 10.0, y: 5.0 },
                        Point { x: 20.0, y: 5.0 },
                        Point { x: 20.0, y: 50.0 },
                        Point { x: 100.0, y: 50.0 },
                        Point { x: 100.0, y: 5.0 },
                    ],
                },
                EdgeGeometry {
                    id: 2,
                    points: vec![
                        Point { x: 10.0, y: 105.0 },
                        Point { x: 20.0, y: 105.0 },
                        Point {
                            x: 20.0,
                            y: second_track,
                        },
                        Point {
                            x: 100.0,
                            y: second_track,
                        },
                        Point { x: 100.0, y: 105.0 },
                    ],
                },
            ]
        };
        let close_quality = RouteQuality {
            crossings: 0,
            bends: 6,
            route_length: 300.0,
        };
        let safe_quality = RouteQuality {
            crossings: 1,
            bends: 6,
            route_length: 302.0,
        };
        let mut best = None;
        let mut admission_state = CandidateAdmissionState::default();

        assert!(!retain_owned_candidate(
            &indexed,
            &mut best,
            close_quality,
            nodes.clone(),
            routes_at(55.999),
            options,
            &mut admission_state,
        ));
        assert_eq!(
            hard_geometry_failure(options, &admission_state),
            LayoutError::ParallelWireSpacingUnsatisfied { spacing: 6.0 },
        );
        assert!(retain_owned_candidate(
            &indexed,
            &mut best,
            safe_quality,
            nodes,
            routes_at(56.0),
            options,
            &mut admission_state,
        ));
        assert_eq!(best.unwrap().selection_quality, safe_quality);
        assert!(admission_state.parallel_spacing_rejected);
        assert!(admission_state.parallel_spacing_satisfied);
    }

    #[test]
    fn contact_clean_candidate_preserves_clearance_failure_classification() {
        let state = CandidateAdmissionState {
            contact_rejected: true,
            contact_satisfied: true,
            ..CandidateAdmissionState::default()
        };
        assert_eq!(
            hard_geometry_failure(
                LayoutOptions {
                    edge_node_clearance: 20.0,
                    ..LayoutOptions::default()
                },
                &state,
            ),
            LayoutError::EdgeNodeClearanceUnsatisfied { clearance: 20.0 },
        );
        assert_eq!(
            hard_geometry_failure(
                LayoutOptions::default(),
                &CandidateAdmissionState {
                    contact_segment_exhausted: true,
                    ..CandidateAdmissionState::default()
                },
            ),
            LayoutError::UnrelatedRouteContactSegmentLimitExceeded {
                maximum: MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
            },
        );
    }

    fn sparse_global_layered_graph(
        layer_count: u32,
        source_stride: u32,
        branch_stride: u32,
    ) -> Graph {
        let mut nodes = Vec::new();
        for id in 0..layer_count * 20 {
            nodes.push(Node {
                id,
                width: 76.0,
                height: 84.0,
                cycle_breaker: false,
                ports: std::iter::once(Port {
                    id: 0,
                    side: PortSide::East,
                    offset: 42.0,
                })
                .chain((0..5).map(|branch| Port {
                    id: branch + 1,
                    side: PortSide::West,
                    offset: 14.0 * (branch + 1) as f64,
                }))
                .collect(),
            });
        }
        let mut edges = Vec::new();
        for layer in 0..layer_count - 1 {
            for source in 0..20u32 {
                for branch in 0..5u32 {
                    edges.push(Edge {
                        id: edges.len() as u32,
                        source: Endpoint {
                            node: layer * 20 + source,
                            port: 0,
                        },
                        target: Endpoint {
                            node: (layer + 1) * 20
                                + (source * source_stride + branch * branch_stride) % 20,
                            port: branch + 1,
                        },
                        net: layer * 20 + source,
                        participates_in_ranking: true,
                    });
                }
            }
        }
        Graph { nodes, edges }
    }

    fn sparse_global_ordinary_routes(
        graph: &Graph,
        options: LayoutOptions,
    ) -> (Vec<NodeGeometry>, routing::RoutedEdges) {
        let indexed = validation::validate_and_index(graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let (forward, reverse, _) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, false);
        let layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let ordinary = placement::place_nodes(&indexed, &ranks, layers, options);
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let routed = routing::route_planned_candidates_with_quality_options(
            &plan, &ordinary, options, false, true, false, false, true, false,
        );
        (ordinary, routed)
    }

    #[test]
    fn public_layout_selects_an_active_sparse_global_candidate() {
        let options = LayoutOptions::default();
        let graph = sparse_global_layered_graph(4, 7, 11);
        let (ordinary, routed) = sparse_global_ordinary_routes(&graph, options);
        assert_eq!(routed.alternatives.len(), 1);
        let (candidate_quality, candidate_edges) = &routed.alternatives[0];
        assert!(candidate_quality.crossings < routed.primary_quality.unwrap().crossings);
        let mut candidate_nodes = ordinary;
        let mut candidate_edges = candidate_edges.clone();
        let candidate_layout = placement::normalize(&mut candidate_nodes, &mut candidate_edges);

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, candidate_layout);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn public_layout_rejects_a_proxy_better_but_exact_worse_sparse_global_candidate() {
        let options = LayoutOptions::default();
        let graph = sparse_global_layered_graph(4, 6, 17);
        let (ordinary, routed) = sparse_global_ordinary_routes(&graph, options);
        assert_eq!(routed.alternatives.len(), 1);
        let (candidate_quality, candidate_edges) = &routed.alternatives[0];
        let primary_quality = routed.primary_quality.unwrap();
        assert!(candidate_quality.crossings > primary_quality.crossings);
        let mut primary_nodes = ordinary.clone();
        let mut primary_edges = routed.primary;
        let primary_layout = placement::normalize(&mut primary_nodes, &mut primary_edges);
        let mut candidate_nodes = ordinary;
        let mut candidate_edges = candidate_edges.clone();
        let candidate_layout = placement::normalize(&mut candidate_nodes, &mut candidate_edges);

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, primary_layout);
        assert_ne!(selected, candidate_layout);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    fn net_representative_graph() -> Graph {
        net_representative_graph_with_padding(40, 3)
    }

    fn net_representative_graph_with_padding(node_count: u32, seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..node_count)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 25.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 25.0,
                    },
                ],
            })
            .collect();
        // One 16-branch net activates the guarded path; fixed sparse cross-connections
        // make its retained ordering distinct without storing a large literal fixture.
        let mut state = seed;
        let mut endpoints = Vec::new();
        for target in 8..24 {
            endpoints.push((0, target, 100));
        }
        for source in 1..8 {
            for target in 8..24 {
                if next(&mut state) % 100 < 24 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        for source in 8..24 {
            for target in 24..40 {
                if next(&mut state) % 100 < 20 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, net))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    fn reverse_median_graph(seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..140)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 25.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 25.0,
                    },
                ],
            })
            .collect();
        let mut state = seed;
        let mut endpoints = Vec::new();
        for target in 10..75 {
            endpoints.push((0, target, 100));
        }
        for target in 15..80 {
            endpoints.push((1, target, 101));
        }
        for source in 2..10 {
            for target in 10..80 {
                if next(&mut state) % 100 < 24 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        for source in 10..80 {
            for target in 80..140 {
                if next(&mut state) % 100 < 12 {
                    endpoints.push((source, target, 1_000 + endpoints.len() as u32));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, net))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    fn slack_rank_graph(seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..51)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 50.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 25.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 25.0,
                    },
                ],
            })
            .collect();
        let mut endpoints = vec![(0, 1), (1, 2), (2, 3), (3, 4), (4, 5)];
        endpoints.extend((30..51).map(|target| (4, target)));
        let mut state = seed;
        for source in 6..30 {
            for target in 30..51 {
                if next(&mut state) % 100 < 20 {
                    endpoints.push((source, target));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: 0,
                },
                net: id as u32,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    fn preferred_backbone_graph(seed: u64) -> Graph {
        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let nodes = (0..81)
            .map(|id| Node {
                id,
                width: 80.0,
                height: 4_000.0,
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
                        offset: 2_000.0,
                    },
                    Port {
                        id: 2,
                        side: PortSide::West,
                        offset: 4_000.0,
                    },
                ],
            })
            .collect();
        let mut endpoints = Vec::<(u32, u32, u32)>::new();
        for lane in 0..20u32 {
            endpoints.push((0, 21 + lane, 0));
            endpoints.push((1 + lane, 21 + lane, 2));
            endpoints.push((21 + lane, 41 + lane, 0));
            endpoints.push((41 + lane, 61 + lane, 0));
        }
        let mut state = seed;
        for source in 21..41u32 {
            for target in 41..61u32 {
                if source - 21 != target - 41 && next(&mut state) % 100 < 8 {
                    endpoints.push((source, target, 0));
                }
            }
        }
        let edges = endpoints
            .into_iter()
            .enumerate()
            .map(|(id, (source, target, target_port))| Edge {
                id: id as u32,
                source: Endpoint {
                    node: source,
                    port: 1,
                },
                target: Endpoint {
                    node: target,
                    port: target_port,
                },
                net: id as u32,
                participates_in_ranking: true,
            })
            .collect();
        Graph { nodes, edges }
    }

    #[test]
    fn selects_a_preferred_backbone_deterministically() {
        let options = LayoutOptions {
            ordering_sweeps: 0,
            ..LayoutOptions::default()
        };
        let graph = preferred_backbone_graph(26);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let layers = topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
        let ordinary = placement::place_nodes(&indexed, &ranks, &layers, options);
        let preferred = placement::place_preferred_nodes(&indexed, &ranks, &layers, options);
        let straight_chain =
            placement::place_straight_chain_nodes(&indexed, &ranks, &layers, &ordinary, options)
                .expect("fixture has matched paths spanning at least two ranks");
        let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary);
        let preferred_alignment = placement::port_alignment_error(&indexed, &ranks, &preferred);
        assert!(
            placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment,),
            "ordinary={ordinary_alignment} preferred={preferred_alignment}"
        );
        let evaluate = |nodes: Vec<NodeGeometry>, supplemental: bool| {
            let mut edges = if supplemental {
                routing::route_supplemental_edges(&indexed, &nodes, &ranks, options)
            } else {
                routing::route_edges(&indexed, &nodes, &ranks, options)
            };
            let quality = routing::route_quality(&indexed, &edges);
            let mut nodes = nodes;
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let ordinary = evaluate(ordinary, false);
        let preferred = evaluate(preferred, true);
        let straight_chain = evaluate(straight_chain.nodes, true);
        let straight_routes = |layout: &Layout| {
            layout
                .edges
                .iter()
                .filter(|edge| {
                    edge.points
                        .first()
                        .is_some_and(|first| edge.points.iter().all(|point| point.y == first.y))
                })
                .count()
        };
        assert!(candidate_quality_cmp(preferred.0, &preferred.1, ordinary.0, &ordinary.1).is_lt());
        assert!(
            candidate_quality_cmp(
                straight_chain.0,
                &straight_chain.1,
                preferred.0,
                &preferred.1,
            )
            .is_lt()
        );
        assert!(straight_routes(&straight_chain.1) > straight_routes(&preferred.1));

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, straight_chain.1);
        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn positive_clearance_considers_a_safe_straight_chain_candidate() {
        let requested = LayoutOptions {
            edge_node_clearance: 20.0,
            ordering_sweeps: 0,
            ..LayoutOptions::default()
        };
        let graph = preferred_backbone_graph(26);
        let indexed = validation::validate_and_index(&graph, requested).unwrap();
        let options = effective_layout_options(requested);
        let ranks = topology::assign_ranks(&indexed);
        let layers = topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
        let ordinary = placement::place_nodes(&indexed, &ranks, &layers, options);
        let straight_chain =
            placement::place_straight_chain_nodes(&indexed, &ranks, &layers, &ordinary, options)
                .expect("fixture has a safe straight-chain placement");
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let mut straight_chain_best = None;
        let mut admission = CandidateAdmissionState::default();
        evaluate_candidate(
            &indexed,
            &plan,
            &mut straight_chain_best,
            straight_chain.nodes,
            options,
            CandidateRouting {
                supplemental: true,
                adaptive_gap_spacing: true,
                ..CandidateRouting::default()
            },
            &mut admission,
        );
        let expected = straight_chain_best
            .expect("at least one straight-chain route satisfies clearance")
            .layout;

        assert!(admission.contact_satisfied);
        assert!(admission.clearance_satisfied);
        assert_eq!(layout(&graph, requested).unwrap(), expected);
        let mut permuted = graph;
        permuted.nodes.reverse();
        for node in &mut permuted.nodes {
            node.ports.reverse();
        }
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, requested).unwrap(), expected);
    }

    #[test]
    fn straight_chain_admission_requires_bounded_cost_and_a_routed_straight_gain() {
        let route = |id: EdgeId, straight: bool| EdgeGeometry {
            id,
            points: if straight {
                vec![Point { x: 0.0, y: 0.0 }, Point { x: 10.0, y: 0.0 }]
            } else {
                vec![
                    Point { x: 0.0, y: 0.0 },
                    Point { x: 5.0, y: 0.0 },
                    Point { x: 5.0, y: 5.0 },
                    Point { x: 10.0, y: 5.0 },
                ]
            },
        };
        let layout = |area: f64, routes: &[(EdgeId, bool)]| Layout {
            nodes: Vec::new(),
            edges: routes
                .iter()
                .map(|&(id, straight)| route(id, straight))
                .collect(),
            boundary_bundles: Vec::new(),
            width: area,
            height: 1.0,
        };
        let baseline = RouteQuality {
            crossings: 10,
            bends: 100,
            route_length: 100.0,
        };
        let admitted = |quality: RouteQuality, layout: Layout| AdmittedCandidate {
            selection_quality: quality,
            quality,
            raw_layout: layout.clone(),
            layout,
        };
        let chain_edges = [10, 20];
        let current = admitted(
            baseline,
            layout(100.0, &[(10, false), (20, false), (99, false)]),
        );
        let boundary = RouteQuality {
            crossings: 9,
            bends: 105,
            route_length: 105.0,
        };

        assert!(straight_chain_cost_is_bounded(
            &admitted(
                boundary,
                layout(110.0, &[(10, true), (20, true), (99, false)]),
            ),
            &current,
            &chain_edges,
            600,
        ));
        for (quality, candidate) in [
            (
                RouteQuality {
                    route_length: 105.000_001,
                    ..boundary
                },
                layout(110.0, &[(10, true), (20, true), (99, false)]),
            ),
            (
                RouteQuality {
                    bends: 106,
                    ..boundary
                },
                layout(110.0, &[(10, true), (20, true), (99, false)]),
            ),
            (
                boundary,
                layout(110.000_001, &[(10, true), (20, true), (99, false)]),
            ),
            (
                boundary,
                layout(110.0, &[(10, false), (20, false), (99, true)]),
            ),
        ] {
            assert!(!straight_chain_cost_is_bounded(
                &admitted(quality, candidate),
                &current,
                &chain_edges,
                600,
            ));
        }

        let balanced_loss_current =
            layout(100.0, &[(10, false), (20, false), (30, true), (40, true)]);
        let balanced_loss_candidate =
            layout(100.0, &[(10, true), (20, true), (30, false), (40, false)]);
        assert!(!straight_chain_cost_is_bounded(
            &admitted(boundary, balanced_loss_candidate),
            &admitted(baseline, balanced_loss_current),
            &[10, 20, 30, 40],
            600,
        ));

        let large_candidate = layout(100.0, &[(10, true), (20, true)]);
        assert!(straight_chain_cost_is_bounded(
            &admitted(boundary, large_candidate.clone()),
            &current,
            &[10, 20, 30, 40, 50, 60, 70, 80, 90, 100],
            601,
        ));
        assert!(straight_chain_cost_is_bounded(
            &admitted(boundary, large_candidate.clone()),
            &current,
            &[10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110],
            601,
        ));
        assert!(!straight_chain_cost_is_bounded(
            &admitted(boundary, large_candidate.clone()),
            &current,
            &[10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120],
            601,
        ));
        assert!(straight_chain_large_gain_is_significant(46, 231));
        assert!(!straight_chain_large_gain_is_significant(45, 231));

        let over_length = RouteQuality {
            route_length: 105.000_001,
            ..boundary
        };
        let raw_over_budget = AdmittedCandidate {
            selection_quality: over_length,
            raw_layout: large_candidate.clone(),
            quality: boundary,
            layout: large_candidate.clone(),
        };
        assert!(!straight_chain_cost_is_bounded(
            &raw_over_budget,
            &current,
            &chain_edges,
            600,
        ));

        let rendered_over_budget = AdmittedCandidate {
            selection_quality: boundary,
            raw_layout: large_candidate.clone(),
            quality: over_length,
            layout: large_candidate,
        };
        assert!(!straight_chain_cost_is_bounded(
            &rendered_over_budget,
            &current,
            &chain_edges,
            600,
        ));

        let rendered_without_gain = AdmittedCandidate {
            selection_quality: boundary,
            raw_layout: layout(100.0, &[(10, true), (20, true), (99, false)]),
            quality: boundary,
            layout: current.layout.clone(),
        };
        assert!(!straight_chain_cost_is_bounded(
            &rendered_without_gain,
            &current,
            &chain_edges,
            600,
        ));
    }

    #[test]
    fn exact_scoring_rejects_a_straighter_candidate_with_more_crossings() {
        let options = LayoutOptions {
            ordering_sweeps: 0,
            ..LayoutOptions::default()
        };
        let graph = preferred_backbone_graph(3);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let layers = topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
        let ordinary = placement::place_nodes(&indexed, &ranks, &layers, options);
        let chain =
            placement::place_straight_chain_nodes(&indexed, &ranks, &layers, &ordinary, options)
                .unwrap()
                .nodes;
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let routed = routing::route_planned_candidates(&plan, &chain, options, true);
        let mut best_chain = None;
        retain_owned_candidate_unchecked(
            &mut best_chain,
            routed.primary_quality.unwrap(),
            chain.clone(),
            routed.primary,
        );
        if let Some((quality, edges)) = routed.repair {
            retain_owned_candidate_unchecked(&mut best_chain, quality, chain.clone(), edges);
        }
        for (quality, edges) in routed.alternatives {
            retain_owned_candidate_unchecked(&mut best_chain, quality, chain.clone(), edges);
        }
        let best_chain = best_chain.unwrap();
        let selected = layout(&graph, options).unwrap();
        let selected_quality = routing::route_quality(&indexed, &selected.edges);

        assert!(
            candidate_quality_cmp(best_chain.0, &best_chain.1, selected_quality, &selected).is_gt()
        );
        assert_ne!(selected, best_chain.1);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn public_layout_selects_an_active_fanout_candidate() {
        let options = LayoutOptions {
            ordering_sweeps: 0,
            ..LayoutOptions::default()
        };
        let graph = active_fanout_fixture::graph();
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        let layers = topology::order_layers(&indexed, &ranks, options.ordering_sweeps);
        let ordinary = placement::place_nodes(&indexed, &ranks, &layers, options);
        let preferred = placement::place_preferred_nodes(&indexed, &ranks, &layers, options);
        let ordinary_alignment = placement::port_alignment_error(&indexed, &ranks, &ordinary);
        let preferred_alignment = placement::port_alignment_error(&indexed, &ranks, &preferred);
        assert!(
            placement::preferred_alignment_is_significant(ordinary_alignment, preferred_alignment),
            "ordinary={ordinary_alignment} preferred={preferred_alignment}"
        );
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let routed = routing::route_planned_candidates(&plan, &preferred, options, true);
        assert!(routed.fanout_trace.evaluated, "{:#?}", routed.fanout_trace);
        assert!(routed.fanout_trace.selected, "{:#?}", routed.fanout_trace);
        let mut adaptive_layouts = Vec::new();
        let mut candidate_nodes = preferred.clone();
        let mut candidate_edges = routed.primary;
        adaptive_layouts.push(placement::normalize(
            &mut candidate_nodes,
            &mut candidate_edges,
        ));
        if let Some((_, mut repair)) = routed.repair {
            let mut candidate_nodes = preferred.clone();
            adaptive_layouts.push(placement::normalize(&mut candidate_nodes, &mut repair));
        }
        let selected = layout(&graph, options).unwrap();
        assert!(
            adaptive_layouts.contains(&selected),
            "public layout did not retain the active adaptive family"
        );
    }

    #[test]
    fn selects_a_latest_feasible_rank_candidate_deterministically() {
        let options = LayoutOptions::default();
        let graph = slack_rank_graph(10);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let (ranks, alternative_ranks) = topology::rank_candidates(&indexed);
        let alternative_ranks = alternative_ranks.unwrap();
        let (forward, reverse, _) = topology::order_layer_candidates(&indexed, &ranks, 4, false);
        let ordinary_layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let baseline_crossings = forward.crossings.min(reverse.crossings);
        let (alternative_forward, alternative_reverse, _) =
            topology::order_layer_candidates(&indexed, &alternative_ranks, 3, false);
        let alternative_layers = if alternative_reverse.crossings < alternative_forward.crossings {
            &alternative_reverse.layers
        } else {
            &alternative_forward.layers
        };
        let alternative_crossings = alternative_forward
            .crossings
            .min(alternative_reverse.crossings);
        assert!(baseline_crossings - alternative_crossings >= baseline_crossings.div_ceil(100));

        let evaluate = |ranks: &[usize], layers: &[Vec<usize>], baseline| {
            let mut nodes = if baseline {
                placement::place_baseline_nodes(&indexed, ranks, layers, options)
            } else {
                placement::place_nodes(&indexed, ranks, layers, options)
            };
            let plan = routing::RoutingPlan::new(&indexed, ranks);
            let mut edges = routing::route_planned_candidates_with_quality_options(
                &plan, &nodes, options, false, false, false, false, true, false,
            )
            .primary;
            let quality = routing::route_quality(&indexed, &edges);
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let baseline = evaluate(&ranks, &forward.layers, true);
        let ordinary = evaluate(&ranks, ordinary_layers, false);
        let alternative = evaluate(&alternative_ranks, alternative_layers, false);
        let best_ordinary =
            if candidate_quality_cmp(ordinary.0, &ordinary.1, baseline.0, &baseline.1).is_lt() {
                &ordinary
            } else {
                &baseline
            };
        assert!(
            candidate_quality_cmp(
                alternative.0,
                &alternative.1,
                best_ordinary.0,
                &best_ordinary.1,
            )
            .is_lt()
        );

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, alternative.1);
        let max_selected =
            super::layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();
        assert_eq!(
            max_selected, alternative.1,
            "Max must retain the winning alternative-rank plan without a primary-rank post-pass",
        );

        let zero_sweeps = LayoutOptions {
            ordering_sweeps: 0,
            ..options
        };
        let zero_sweep_selected = layout(&graph, zero_sweeps).unwrap();
        assert_eq!(layout(&graph, zero_sweeps).unwrap(), zero_sweep_selected);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
        assert_eq!(layout(&permuted, zero_sweeps).unwrap(), zero_sweep_selected);
    }

    #[test]
    fn selects_a_reverse_median_candidate_deterministically() {
        let options = LayoutOptions::default();
        let graph = reverse_median_graph(1);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);

        assert!(
            topology::order_layer_candidates(&indexed, &ranks, 0, true)
                .2
                .is_none()
        );
        let one_sweep = topology::order_layer_candidates(&indexed, &ranks, 1, true)
            .2
            .unwrap()
            .layers;
        assert_eq!(
            topology::order_layer_candidates(&indexed, &ranks, 1, true)
                .2
                .unwrap()
                .layers,
            one_sweep
        );

        let (forward, reverse, alternative) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
        let ordinary_layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let alternative = alternative.unwrap();
        assert!(alternative.crossings < forward.crossings.min(reverse.crossings));

        let evaluate = |mut nodes: Vec<NodeGeometry>| {
            let plan = routing::RoutingPlan::new(&indexed, &ranks);
            let mut edges = routing::route_planned_candidates_with_quality_options(
                &plan, &nodes, options, false, false, false, false, true, false,
            )
            .primary;
            let quality = routing::route_quality(&indexed, &edges);
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let baseline = evaluate(placement::place_baseline_nodes(
            &indexed,
            &ranks,
            &forward.layers,
            options,
        ));
        let ordinary = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            ordinary_layers,
            options,
        ));
        let alternative = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            &alternative.layers,
            options,
        ));
        let best_ordinary =
            if candidate_quality_cmp(ordinary.0, &ordinary.1, baseline.0, &baseline.1).is_lt() {
                &ordinary
            } else {
                &baseline
            };
        assert!(
            candidate_quality_cmp(
                alternative.0,
                &alternative.1,
                best_ordinary.0,
                &best_ordinary.1,
            )
            .is_lt()
        );

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, alternative.1);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn composite_pitched_quality_budget_has_exact_inclusive_boundaries() {
        let baseline = RouteQuality {
            crossings: 100,
            bends: 100,
            route_length: 100.0,
        };
        let options = LayoutOptions {
            max_quality_area_factor: 2.0,
            max_quality_route_length_factor: 1.25,
            ..LayoutOptions::default()
        };
        let boundary = RouteQuality {
            crossings: 101,
            bends: 102,
            route_length: 125.0,
        };
        assert!(composite_pitched_quality_is_admissible(
            baseline, 100.0, boundary, 200.0, options,
        ));
        for (candidate, area) in [
            (
                RouteQuality {
                    crossings: 102,
                    ..boundary
                },
                200.0,
            ),
            (
                RouteQuality {
                    bends: 103,
                    ..boundary
                },
                200.0,
            ),
            (
                RouteQuality {
                    route_length: 125.000_001,
                    ..boundary
                },
                200.0,
            ),
            (boundary, 200.000_001),
        ] {
            assert!(!composite_pitched_quality_is_admissible(
                baseline, 100.0, candidate, area, options,
            ));
        }
    }

    #[test]
    fn candidate_selection_preserves_baseline_and_uses_quality_priority() {
        let baseline = candidate(1, 4, 30.0, 40.0);
        let mut best = None;
        retain_better_candidate(&mut best, baseline.0, baseline.1.clone());

        let worse_crossings = candidate(2, 0, 1.0, 1.0);
        retain_better_candidate(&mut best, worse_crossings.0, worse_crossings.1);
        assert_eq!(best.as_ref().unwrap().1, baseline.1);

        let fewer_bends = candidate(1, 3, 100.0, 100.0);
        retain_better_candidate(&mut best, fewer_bends.0, fewer_bends.1.clone());
        assert_eq!(best.as_ref().unwrap().1, fewer_bends.1);

        let shorter = candidate(1, 3, 20.0, 100.0);
        retain_better_candidate(&mut best, shorter.0, shorter.1.clone());
        assert_eq!(best.as_ref().unwrap().1, shorter.1);

        let smaller = candidate(1, 3, 20.0, 20.0);
        retain_better_candidate(&mut best, smaller.0, smaller.1.clone());
        assert_eq!(best.as_ref().unwrap().1, smaller.1);

        let mut exact_tie = candidate(1, 3, 20.0, 20.0);
        exact_tie.1.width = 10.0;
        exact_tie.1.height = 2.0;
        assert_ne!(exact_tie.1, smaller.1);
        retain_better_candidate(&mut best, exact_tie.0, exact_tie.1);
        assert_eq!(best.as_ref().unwrap().1, smaller.1);
    }

    #[test]
    fn owned_candidate_selection_keeps_the_exact_quality_and_area_ordering() {
        let baseline = candidate(1, 3, 20.0, 100.0);
        let mut best = Some(baseline.clone());
        retain_owned_candidate_unchecked(
            &mut best,
            candidate(2, 0, 1.0, 1.0).0,
            vec![NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 1.0,
                height: 1.0,
            }],
            Vec::new(),
        );
        assert_eq!(best, Some(baseline.clone()));

        retain_owned_candidate_unchecked(
            &mut best,
            baseline.0,
            vec![NodeGeometry {
                id: 1,
                x: 0.0,
                y: 0.0,
                width: 5.0,
                height: 4.0,
            }],
            Vec::new(),
        );
        assert_eq!(
            best.as_ref().unwrap().1.width * best.as_ref().unwrap().1.height,
            20.0
        );
    }

    #[test]
    fn selects_a_distinct_net_representative_candidate_deterministically() {
        let graph = net_representative_graph();
        let options = LayoutOptions::default();
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let ranks = topology::assign_ranks(&indexed);
        assert!(
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, false)
                .2
                .is_none()
        );
        let (forward, reverse, net_representative) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
        let quality_layers = if reverse.crossings < forward.crossings {
            &reverse.layers
        } else {
            &forward.layers
        };
        let net_representative = net_representative.unwrap();
        assert_ne!(net_representative.layers, *quality_layers);

        let evaluate = |mut nodes: Vec<NodeGeometry>| {
            let plan = routing::RoutingPlan::new(&indexed, &ranks);
            let mut edges = routing::route_planned_candidates_with_quality_options(
                &plan, &nodes, options, false, false, false, false, true, false,
            )
            .primary;
            let quality = routing::route_quality(&indexed, &edges);
            let layout = placement::normalize(&mut nodes, &mut edges);
            (quality, layout)
        };
        let baseline = evaluate(placement::place_baseline_nodes(
            &indexed,
            &ranks,
            &forward.layers,
            options,
        ));
        let ordinary = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            quality_layers,
            options,
        ));
        let net_representative = evaluate(placement::place_nodes(
            &indexed,
            &ranks,
            &net_representative.layers,
            options,
        ));
        let best_ordinary =
            if candidate_quality_cmp(ordinary.0, &ordinary.1, baseline.0, &baseline.1).is_lt() {
                &ordinary
            } else {
                &baseline
            };
        assert!(
            candidate_quality_cmp(
                net_representative.0,
                &net_representative.1,
                best_ordinary.0,
                &best_ordinary.1,
            )
            .is_lt()
        );

        let selected = layout(&graph, options).unwrap();
        assert_eq!(selected, net_representative.1);

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(layout(&permuted, options).unwrap(), selected);
    }

    #[test]
    fn quality_effort_controls_the_admitted_public_layout_deterministically() {
        let graph = net_representative_graph_with_padding(600, 81);
        let options = LayoutOptions::default();
        let fast = super::layout_with_quality_effort(&graph, options, QualityEffort::Fast).unwrap();
        let quality =
            super::layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
        let max = super::layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();
        assert_eq!(layout(&graph, options).unwrap(), quality);
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let fast_quality = routing::route_quality(&indexed, &fast.edges);
        let quality_quality = routing::route_quality(&indexed, &quality.edges);
        let max_quality = routing::route_quality(&indexed, &max.edges);
        assert_eq!(fast_quality.crossings, 552);
        assert_eq!(quality_quality.crossings, 405);
        assert!(candidate_quality_cmp(quality_quality, &quality, fast_quality, &fast).is_lt());
        assert!(!candidate_quality_cmp(max_quality, &max, quality_quality, &quality).is_gt());

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Fast).unwrap(),
            fast
        );
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Quality).unwrap(),
            quality
        );
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Max).unwrap(),
            max
        );
    }

    #[test]
    fn max_preserves_the_browser_corpus_refined_fallback() {
        let graph = browser_max_effort_graph();
        let options = LayoutOptions::default();
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let (ranks, _) = topology::rank_candidates(&indexed);
        let (_, _, representative) =
            topology::order_layer_candidates(&indexed, &ranks, options.ordering_sweeps, true);
        let nodes = placement::place_nodes(
            &indexed,
            &ranks,
            &representative
                .expect("browser fixture has a representative candidate")
                .layers,
            options,
        );
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let routed = routing::route_planned_candidates_with_refined_sparse_global(
            &plan, &nodes, options, false, true, true, true,
        );
        assert_eq!(routed.primary_quality.unwrap().crossings, 22_065);
        assert_eq!(
            routed
                .alternatives
                .iter()
                .map(|item| item.0.crossings)
                .collect::<Vec<_>>(),
            [22_315, 21_959, 22_044]
        );
        let quality =
            super::layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
        let max = super::layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();

        assert_eq!(
            routing::route_quality(&indexed, &quality.edges).crossings,
            22_065
        );
        assert_eq!(
            routing::route_quality(&indexed, &max.edges).crossings,
            21_959
        );

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Max).unwrap(),
            max
        );
    }

    #[test]
    fn max_sifting_is_exactly_scored_and_deterministic() {
        let mut state = 16u64;
        let mut next = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            state
        };
        let nodes = (0..40)
            .map(|id| Node {
                id,
                width: 40.0,
                height: 30.0,
                cycle_breaker: false,
                ports: vec![
                    Port {
                        id: 0,
                        side: PortSide::West,
                        offset: 15.0,
                    },
                    Port {
                        id: 1,
                        side: PortSide::East,
                        offset: 15.0,
                    },
                ],
            })
            .collect();
        let mut edges = Vec::new();
        for layer in 0..3 {
            for source in 0..10 {
                for target in 0..10 {
                    if next() % 100 < 32 {
                        let id = edges.len() as u32;
                        edges.push(Edge {
                            id,
                            source: Endpoint {
                                node: layer * 10 + source,
                                port: 1,
                            },
                            target: Endpoint {
                                node: (layer + 1) * 10 + target,
                                port: 0,
                            },
                            net: id,
                            participates_in_ranking: true,
                        });
                    }
                }
            }
        }
        let graph = Graph { nodes, edges };
        let options = LayoutOptions::default();
        let quality =
            super::layout_with_quality_effort(&graph, options, QualityEffort::Quality).unwrap();
        let max = super::layout_with_quality_effort(&graph, options, QualityEffort::Max).unwrap();
        let indexed = validation::validate_and_index(&graph, options).unwrap();
        let quality_score = routing::route_quality(&indexed, &quality.edges);
        let max_score = routing::route_quality(&indexed, &max.edges);

        assert_eq!(quality_score.crossings, 434);
        assert_eq!(max_score.crossings, 426);
        assert!(candidate_quality_cmp(max_score, &max, quality_score, &quality).is_lt());

        let mut permuted = graph;
        permuted.nodes.reverse();
        permuted.edges.reverse();
        assert_eq!(
            super::layout_with_quality_effort(&permuted, options, QualityEffort::Max).unwrap(),
            max
        );
    }

    #[test]
    fn max_horizontal_pitch_refinement_remains_active_with_a_boundary_bundle() {
        let endpoint = |id, side| Node {
            id,
            width: 20.0,
            height: 100.0,
            cycle_breaker: false,
            ports: (0..4)
                .map(|port| Port {
                    id: port,
                    side,
                    offset: 25.0 + port as f64,
                })
                .collect(),
        };
        let blocker = |id| Node {
            id,
            width: 20.0,
            height: 20.0,
            cycle_breaker: false,
            ports: Vec::new(),
        };
        let graph = Graph {
            nodes: vec![
                endpoint(0, PortSide::East),
                blocker(1),
                blocker(2),
                endpoint(3, PortSide::West),
            ],
            edges: (0..4)
                .map(|edge| Edge {
                    id: 100 + edge,
                    source: Endpoint {
                        node: 0,
                        port: edge,
                    },
                    target: Endpoint {
                        node: 3,
                        port: edge,
                    },
                    net: [7, 7, 8, 9][edge as usize],
                    participates_in_ranking: true,
                })
                .collect(),
        };
        let options = LayoutOptions {
            route_lane_gap: 6.0,
            edge_node_clearance: 20.0,
            max_quality_area_factor: 2.0,
            max_quality_route_length_factor: 1.25,
            ..LayoutOptions::default()
        };
        let constraints = LayoutConstraints {
            inputs: vec![0],
            outputs: vec![3],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 1,
                endpoint: Endpoint { node: 0, port: 0 },
                width: 1,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 100,
                    slots: vec![0],
                }],
            }],
        };
        let quality = super::layout_with_quality_effort_and_constraints(
            &graph,
            options,
            QualityEffort::Quality,
            &constraints,
        )
        .unwrap();
        let max = super::layout_with_quality_effort_and_constraints(
            &graph,
            options,
            QualityEffort::Max,
            &constraints,
        )
        .unwrap();
        assert_ne!(max, quality, "the Max refinement fixture must activate");
        let indexed =
            validation::validate_and_index_with_constraints(&graph, options, &constraints).unwrap();
        let (ranks, _) = topology::rank_candidates(&indexed);
        let plan = routing::RoutingPlan::new(&indexed, &ranks);
        let quality_congestion = routing::route_parallel_congestion(&plan, &quality.edges).unwrap();
        let max_congestion = routing::route_parallel_congestion(&plan, &max.edges).unwrap();
        assert!(max_congestion < quality_congestion * 0.5);
        assert!(routing::layout_horizontal_crossing_pitch_is_satisfied(
            &plan,
            &max.nodes,
            &max.edges,
            options,
            options.route_lane_gap,
        ));
        assert_eq!(max.boundary_bundles.len(), 1);
        assert_eq!(
            max.edges[0].points[0],
            max.boundary_bundles[0].members[0].tap
        );
    }
}
