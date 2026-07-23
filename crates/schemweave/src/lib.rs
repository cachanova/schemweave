//! Deterministic layered placement and orthogonal routing for circuit graphs.

#![forbid(unsafe_code)]

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
pub(crate) use readability::measure_parallel_congestion_bounded;
#[doc(hidden)]
pub use readability::{
    EdgeNodeClearance, EdgeNodeClearanceError, EdgeNodeSegment, NetNodeRelation,
    ParallelCongestion, ParallelSegment, measure_edge_node_clearance_bounded,
    measure_parallel_congestion,
};

pub type NodeId = u32;
pub type PortId = u32;
pub type EdgeId = u32;
pub type NetId = u32;

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
    pub route_lane_gap: f64,
    /// Minimum axis-aligned distance between a route and every unrelated node.
    pub edge_node_clearance: f64,
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
                edge_node_clearance: 20.0,
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

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Layout {
    pub nodes: Vec<NodeGeometry>,
    pub edges: Vec<EdgeGeometry>,
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
    let mut best: Option<(routing::RouteQuality, Layout)> = None;
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
    let straight_chain_nodes = (options.edge_node_clearance == 0.0
        && quality_effort != QualityEffort::Fast
        && graph.nodes.len() <= placement::MAX_CHAIN_CANDIDATE_NODES)
        .then(|| {
            placement::place_straight_chain_nodes(
                &indexed,
                &ranks,
                quality_layers,
                &ordinary_nodes,
                options.node_gap,
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
            if options.edge_node_clearance == 0.0 {
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
                if let Some((quality, layout)) = alternative_best
                    && retain_better_candidate(&mut best, quality, layout)
                {
                    best_uses_primary_ranks = false;
                }
            }
        }
    }
    if let Some(straight_chain_nodes) = straight_chain_nodes
        && best.as_ref().is_some_and(|(_, layout)| {
            placement::vertical_span(&straight_chain_nodes)
                <= layout.height * placement::MAX_CHAIN_HEIGHT_FACTOR
        })
    {
        let mut straight_chain_best = None;
        evaluate_candidate(
            &indexed,
            &routing_plan,
            &mut straight_chain_best,
            straight_chain_nodes,
            options,
            CandidateRouting {
                supplemental: true,
                ..candidate_routing
            },
            &mut admission_state,
        );
        let (quality, layout) = straight_chain_best.expect("candidate routing produces a layout");
        if best.as_ref().is_some_and(|(current_quality, current)| {
            straight_chain_cost_is_bounded(quality, &layout, *current_quality, current)
        }) && retain_better_candidate(&mut best, quality, layout)
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
    let (mut quality, mut layout) =
        best.ok_or_else(|| positive_clearance_failure(options, &admission_state))?;
    if quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && demand_aware_scale_is_eligible(graph.nodes.len(), graph.edges.len())
        && placement::demand_aware_spacing_is_relevant(
            &indexed,
            &ranks,
            quality_layers.len(),
            options,
        )
        && routing::route_parallel_congestion(&routing_plan, &layout.edges)
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
        if let Some((demand_quality, demand_layout)) = demand_aware_best
            && demand_aware_readability_is_better(
                &routing_plan,
                quality,
                &layout,
                demand_quality,
                &demand_layout,
            )
        {
            quality = demand_quality;
            layout = demand_layout;
        }
    }
    if quality_effort == QualityEffort::Max
        && best_uses_primary_ranks
        && let Some((candidate_quality, edges)) = routing::regional_fanout_candidate(
            &routing_plan,
            &layout.nodes,
            &layout.edges,
            quality,
            options,
        )
    {
        let candidate = placement::normalize_owned(layout.nodes.clone(), edges);
        if candidate.width * candidate.height <= layout.width * layout.height * 1.05
            && candidate_satisfies_positive_clearance_contract(
                &indexed,
                &candidate,
                options,
                &mut admission_state,
            )
        {
            debug_assert!(route_quality_cmp(candidate_quality, quality).is_lt());
            return Ok(candidate);
        }
    }
    Ok(layout)
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
    quality: routing::RouteQuality,
    layout: &Layout,
    current_quality: routing::RouteQuality,
    current: &Layout,
) -> bool {
    quality.route_length <= current_quality.route_length * 1.05
        && layout.width * layout.height <= current.width * current.height * 1.10
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
    clearance_work_exhausted: bool,
    contact_segment_exhausted: bool,
    contact_work_exhausted: bool,
    contact_rejected: bool,
    contact_satisfied: bool,
}

fn positive_clearance_failure(
    options: LayoutOptions,
    admission_state: &CandidateAdmissionState,
) -> LayoutError {
    if admission_state.contact_segment_exhausted {
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
    } else if admission_state.contact_rejected && !admission_state.contact_satisfied {
        LayoutError::UnrelatedRouteContactUnsatisfied
    } else {
        LayoutError::EdgeNodeClearanceUnsatisfied {
            clearance: options.edge_node_clearance,
        }
    }
}

fn evaluate_candidate(
    indexed: &validation::IndexedGraph<'_>,
    routing_plan: &routing::RoutingPlan<'_>,
    best: &mut Option<(routing::RouteQuality, Layout)>,
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
    best: &mut Option<(routing::RouteQuality, Layout)>,
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

fn retain_owned_candidate(
    indexed: &validation::IndexedGraph<'_>,
    best: &mut Option<(routing::RouteQuality, Layout)>,
    quality: routing::RouteQuality,
    nodes: Vec<NodeGeometry>,
    edges: Vec<EdgeGeometry>,
    options: LayoutOptions,
    admission_state: &mut CandidateAdmissionState,
) -> bool {
    if best
        .as_ref()
        .is_some_and(|(current_quality, _)| route_quality_cmp(quality, *current_quality).is_gt())
    {
        return false;
    }
    let candidate = placement::normalize_owned(nodes, edges);
    if !candidate_satisfies_positive_clearance_contract(
        indexed,
        &candidate,
        options,
        admission_state,
    ) {
        return false;
    }
    retain_better_candidate(best, quality, candidate)
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
    if options.edge_node_clearance > 0.0 {
        options.layer_gap = options
            .layer_gap
            .max(options.edge_node_clearance * 2.0 + options.route_lane_gap);
    }
    options
}

/// Distance used only where a route must move outward beyond endpoint-node obstacles.
///
/// Track planning continues to use the requested `port_stub`; widening every planning margin
/// for semantic clearance needlessly concentrates dense routing. The exact final admission gate
/// remains authoritative.
pub(crate) fn outward_obstacle_clearance_stub(options: LayoutOptions) -> f64 {
    options.port_stub.max(options.edge_node_clearance)
}

fn candidate_satisfies_positive_clearance_contract(
    indexed: &validation::IndexedGraph<'_>,
    candidate: &Layout,
    options: LayoutOptions,
    admission_state: &mut CandidateAdmissionState,
) -> bool {
    if options.edge_node_clearance == 0.0 {
        return true;
    }
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
    candidate_satisfies_edge_node_clearance(
        indexed,
        candidate,
        options,
        &mut admission_state.clearance_work_exhausted,
    )
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
        CandidateAdmissionState, Edge, EdgeGeometry, Endpoint, Graph, Layout, LayoutError,
        LayoutOptions, MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS, Node, NodeGeometry, Point, Port,
        PortSide, QualityEffort, candidate_quality_cmp,
        candidate_satisfies_edge_node_clearance_bounded, demand_aware_quality_is_better,
        demand_aware_scale_is_eligible, effective_layout_options, effective_ranking_edges, layout,
        outward_obstacle_clearance_stub, placement, positive_clearance_failure,
        retain_better_candidate, retain_owned_candidate, retain_owned_candidate_unchecked, routing,
        routing::RouteQuality, topology, validation,
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
                width: area,
                height: 1.0,
            },
        )
    }

    #[test]
    fn effective_spacing_options_are_canonical_idempotent_and_zero_identity() {
        let defaults = LayoutOptions::default();
        assert_eq!(effective_layout_options(defaults), defaults);

        let requested = LayoutOptions {
            edge_node_clearance: 40.0,
            ..defaults
        };
        let effective = effective_layout_options(requested);
        assert_eq!(effective.port_stub, defaults.port_stub);
        assert_eq!(effective.layer_gap, 84.0);
        assert_eq!(effective_layout_options(effective), effective);
        assert_eq!(outward_obstacle_clearance_stub(effective), 40.0);
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
        assert_eq!(best.unwrap().0, safe_quality);
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
        assert_eq!(best.unwrap().0, safe_sibling);
        assert!(!admission_state.clearance_work_exhausted);
        assert!(!admission_state.contact_work_exhausted);
        assert!(admission_state.contact_rejected);
        assert!(admission_state.contact_satisfied);
    }

    #[test]
    fn contact_clean_candidate_preserves_clearance_failure_classification() {
        let state = CandidateAdmissionState {
            contact_rejected: true,
            contact_satisfied: true,
            ..CandidateAdmissionState::default()
        };
        assert_eq!(
            positive_clearance_failure(
                LayoutOptions {
                    edge_node_clearance: 20.0,
                    ..LayoutOptions::default()
                },
                &state,
            ),
            LayoutError::EdgeNodeClearanceUnsatisfied { clearance: 20.0 },
        );
        assert_eq!(
            positive_clearance_failure(
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
        let straight_chain = placement::place_straight_chain_nodes(
            &indexed,
            &ranks,
            &layers,
            &ordinary,
            options.node_gap,
        )
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
        let straight_chain = evaluate(straight_chain, true);
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
        let chain = placement::place_straight_chain_nodes(
            &indexed,
            &ranks,
            &layers,
            &ordinary,
            options.node_gap,
        )
        .unwrap();
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
}
