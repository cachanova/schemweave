use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    ConstrainedLayoutError, Edge, EdgeGeometry, EdgeId, Endpoint, Graph, Layout, LayoutConstraints,
    LayoutError, LayoutOptions, NetId, Node, NodeGeometry, NodeId, Point, Port, PortSide,
    QualityEffort, layout_with_quality_effort_and_constraints, routing, validation,
};

const MAX_EXPANSION_MEMBERS: usize = 4_096;
const MAX_EXPANSION_EDGES: usize = 10_000;
const MAX_LAYOUT_SEGMENTS: usize = 100_000;
const HARD_GATE_EPSILON: f64 = 1e-7;
const FAST_CANDIDATE_WORK: usize = 10_000_000;
const QUALITY_CANDIDATE_WORK: usize = 30_000_000;
const MAX_CANDIDATE_WORK: usize = 120_000_000;
const SAFETY_CANDIDATES: usize = 2;

/// One expanded boundary edge and the compact route trunk it replaces.
///
/// Several expanded edges may intentionally reuse one compact trunk. The
/// mapping is explicit so electrically distinct named pins are never inferred
/// from net identity alone.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct BoundaryTrunk {
    pub expanded_edge: EdgeId,
    pub compact_edge: EdgeId,
}

/// One compact node replaced by its concrete member nodes.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct GroupExpansion {
    pub anchor: NodeId,
    pub members: Vec<NodeId>,
    pub boundary_trunks: Vec<BoundaryTrunk>,
}

/// Layout policy for an in-place group expansion.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct GroupExpansionOptions {
    #[serde(flatten)]
    pub layout: LayoutOptions,
    pub quality_effort: QualityEffort,
    pub constraints: LayoutConstraints,
}

/// Invalid compact-to-expanded graph or geometry contract.
#[derive(Clone, Debug, Error, PartialEq)]
#[non_exhaustive]
pub enum GroupExpansionError {
    #[error("invalid compact graph: {0}")]
    InvalidCompactGraph(LayoutError),
    #[error("invalid expanded graph or constraints: {0}")]
    InvalidExpandedGraph(ConstrainedLayoutError),
    #[error("group expansion has no members")]
    EmptyMembers,
    #[error("group expansion has {actual} members, maximum is {maximum}")]
    TooManyMembers { actual: usize, maximum: usize },
    #[error("expanded graph has {actual} edges, maximum is {maximum}")]
    TooManyEdges { actual: usize, maximum: usize },
    #[error("group expansion repeats member node {0}")]
    DuplicateMember(NodeId),
    #[error("compact graph does not contain anchor node {0}")]
    MissingAnchor(NodeId),
    #[error("expanded graph still contains anchor node {0}")]
    RetainedAnchor(NodeId),
    #[error("expanded graph does not contain member node {0}")]
    MissingMember(NodeId),
    #[error("compact graph already contains member node {0}")]
    ExistingMember(NodeId),
    #[error("expanded graph contains unexpected node {0}")]
    UnexpectedNode(NodeId),
    #[error("expanded graph is missing retained node {0}")]
    MissingRetainedNode(NodeId),
    #[error("expanded graph changed retained node {0}")]
    ChangedRetainedNode(NodeId),
    #[error("compact layout repeats node geometry {0}")]
    DuplicateNodeGeometry(NodeId),
    #[error("compact layout is missing node geometry {0}")]
    MissingNodeGeometry(NodeId),
    #[error("compact layout contains unknown node geometry {0}")]
    UnknownNodeGeometry(NodeId),
    #[error("compact layout has invalid node geometry {0}")]
    InvalidNodeGeometry(NodeId),
    #[error("compact layout repeats edge geometry {0}")]
    DuplicateEdgeGeometry(EdgeId),
    #[error("compact layout is missing edge geometry {0}")]
    MissingEdgeGeometry(EdgeId),
    #[error("compact layout contains unknown edge geometry {0}")]
    UnknownEdgeGeometry(EdgeId),
    #[error("compact layout has invalid edge geometry {0}")]
    InvalidEdgeGeometry(EdgeId),
    #[error("compact layout has invalid bounds")]
    InvalidLayoutBounds,
    #[error("compact layout has {actual} route segments, maximum is {maximum}")]
    TooManyCompactRouteSegments { actual: usize, maximum: usize },
    #[error("compact layout violates a hard geometry invariant")]
    InvalidCompactHardGeometry,
    #[error("expanded graph is missing retained edge {0}")]
    MissingRetainedEdge(EdgeId),
    #[error("expanded graph changed retained edge {0}")]
    ChangedRetainedEdge(EdgeId),
    #[error("expanded boundary edge {0} has no compact trunk mapping")]
    MissingBoundaryTrunk(EdgeId),
    #[error("group expansion repeats compact trunk mapping for expanded boundary edge {0}")]
    DuplicateBoundaryTrunk(EdgeId),
    #[error("group expansion maps unknown or non-boundary expanded edge {0}")]
    InvalidBoundaryEdge(EdgeId),
    #[error("group expansion maps unknown or non-anchor compact edge {0}")]
    InvalidCompactTrunk(EdgeId),
    #[error(
        "expanded boundary edge {expanded_edge} is incompatible with compact trunk {compact_edge}"
    )]
    IncompatibleBoundaryTrunk {
        expanded_edge: EdgeId,
        compact_edge: EdgeId,
    },
    #[error("compact anchor trunk {0} is not represented by an expanded boundary edge")]
    UnusedCompactTrunk(EdgeId),
    #[error("expanded boundary edge {0} matches an empty compact trunk")]
    EmptyBoundaryTrunk(EdgeId),
    #[error("expanded boundary edge {0} has no obstacle-safe bridge")]
    NoSafeBoundaryBridge(EdgeId),
    #[error("failed to lay out expanded members: {0}")]
    MemberLayout(ConstrainedLayoutError),
    #[error("expansion candidate work {required} exceeds deterministic budget {maximum}")]
    ExpansionWorkLimitExceeded { required: usize, maximum: usize },
    #[error("preserved expansion would contain {actual} route segments, maximum is {maximum}")]
    PreservedGeometryTooLarge { actual: usize, maximum: usize },
    #[error("no in-place expansion preserves every hard geometry and left-to-right invariant")]
    NeedsFullRelayout,
}

struct ExpansionContract<'a> {
    anchor_geometry: &'a NodeGeometry,
    members: BTreeSet<NodeId>,
    expanded_nodes: BTreeMap<NodeId, &'a Node>,
    compact_node_geometry: BTreeMap<NodeId, &'a NodeGeometry>,
    compact_edge_geometry: BTreeMap<EdgeId, &'a EdgeGeometry>,
    boundary_trunks: BTreeMap<EdgeId, EdgeId>,
}

#[derive(Clone, Copy)]
struct ExpansionWork {
    nodes: usize,
    edges: usize,
    boundary_edges: usize,
    projected_segments: usize,
}

struct WorkBudget {
    used: usize,
    maximum: usize,
}

impl WorkBudget {
    fn new(maximum: usize) -> Self {
        Self { used: 0, maximum }
    }

    fn take(&mut self, amount: usize) -> Result<(), usize> {
        let required = self.used.saturating_add(amount);
        if required > self.maximum {
            Err(required)
        } else {
            self.used = required;
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct CandidateScore {
    quality: routing::RouteQuality,
    displacement: f64,
    area: f64,
    x: f64,
    y: f64,
}

impl CandidateScore {
    fn cmp(self, other: Self) -> Ordering {
        self.quality
            .crossings
            .cmp(&other.quality.crossings)
            .then(self.quality.bends.cmp(&other.quality.bends))
            .then(
                self.quality
                    .route_length
                    .total_cmp(&other.quality.route_length),
            )
            .then(self.displacement.total_cmp(&other.displacement))
            .then(self.area.total_cmp(&other.area))
            .then(self.x.total_cmp(&other.x))
            .then(self.y.total_cmp(&other.y))
    }
}

/// Expand one quotient node without moving retained nodes or retained routes.
///
/// The compact and expanded graphs must use stable identifiers. Every node and
/// edge unrelated to the anchor must be byte-for-byte equivalent. Boundary
/// member edges reuse explicitly mapped compact trunks, while member placement
/// and internal routing use the canonical SchemWeave engine. A result is
/// returned only when every hard geometry and left-to-right invariant survives;
/// callers must perform a full layout after `NeedsFullRelayout`.
pub fn expand_group_in_place(
    compact_graph: &Graph,
    compact_layout: &Layout,
    expanded_graph: &Graph,
    expansion: &GroupExpansion,
    options: &GroupExpansionOptions,
) -> Result<Layout, GroupExpansionError> {
    validation::validate_and_index(compact_graph, options.layout)
        .map_err(GroupExpansionError::InvalidCompactGraph)?;
    let expanded_indexed = validation::validate_and_index_with_constraints(
        expanded_graph,
        options.layout,
        &options.constraints,
    )
    .map_err(GroupExpansionError::InvalidExpandedGraph)?;
    if expanded_graph.edges.len() > MAX_EXPANSION_EDGES {
        return Err(GroupExpansionError::TooManyEdges {
            actual: expanded_graph.edges.len(),
            maximum: MAX_EXPANSION_EDGES,
        });
    }
    let contract = validate_contract(compact_graph, compact_layout, expanded_graph, expansion)?;

    let member_graph = member_graph(expanded_graph, &contract.members);
    let member_constraints = LayoutConstraints {
        inputs: options
            .constraints
            .inputs
            .iter()
            .copied()
            .filter(|node| contract.members.contains(node))
            .collect(),
        outputs: options
            .constraints
            .outputs
            .iter()
            .copied()
            .filter(|node| contract.members.contains(node))
            .collect(),
    };
    let member_layout = layout_with_quality_effort_and_constraints(
        &member_graph,
        options.layout,
        options.quality_effort,
        &member_constraints,
    )
    .map_err(GroupExpansionError::MemberLayout)?;
    let member_layout =
        if member_constraints.inputs.is_empty() && member_constraints.outputs.is_empty() {
            pack_disconnected_components(
                &member_graph,
                &member_layout,
                options.layout.node_gap,
                boundary_horizontal_span(expanded_graph, &contract, options.layout.node_gap),
            )
        } else {
            member_layout
        };
    let boundary_edges = expanded_graph
        .edges
        .iter()
        .filter(|edge| {
            contract.members.contains(&edge.source.node)
                ^ contract.members.contains(&edge.target.node)
        })
        .count();
    let projected_segments = projected_segment_count(expanded_graph, &contract, &member_layout);
    if projected_segments > MAX_LAYOUT_SEGMENTS {
        return Err(GroupExpansionError::PreservedGeometryTooLarge {
            actual: projected_segments,
            maximum: MAX_LAYOUT_SEGMENTS,
        });
    }
    let mut positions = candidate_positions(
        compact_layout,
        contract.anchor_geometry,
        &member_layout,
        ExpansionWork {
            nodes: expanded_graph.nodes.len(),
            edges: expanded_graph.edges.len(),
            boundary_edges,
            projected_segments,
        },
        options.layout,
        options.quality_effort,
    )?;
    prioritize_constraint_positions(
        &mut positions,
        constraint_x_candidates(
            compact_layout,
            &member_layout,
            &contract.members,
            &options.constraints,
        ),
    );

    let anchor_center = center(contract.anchor_geometry);
    let mut best: Option<(CandidateScore, Layout)> = None;
    let hard_budget_maximum = candidate_work_budget(options.quality_effort);
    let mut hard_budget = WorkBudget::new(hard_budget_maximum);
    for (x, y) in positions {
        let frame = Rect {
            left: x,
            top: y,
            right: x + member_layout.width,
            bottom: y + member_layout.height,
        };
        if retained_node_overlap_area(compact_layout, expansion.anchor, frame, 0.0) > 0.0 {
            continue;
        }
        let candidate = match compose_candidate(
            compact_layout,
            expanded_graph,
            &contract,
            &member_layout,
            x,
            y,
            options.layout,
        ) {
            Ok(candidate) => candidate,
            Err(GroupExpansionError::NoSafeBoundaryBridge(_)) => continue,
            Err(error) => return Err(error),
        };
        hard_budget
            .take(expanded_graph.edges.len())
            .map_err(|required| GroupExpansionError::ExpansionWorkLimitExceeded {
                required,
                maximum: hard_budget_maximum,
            })?;
        let geometry_is_clean =
            hard_geometry_is_clean_bounded(expanded_graph, &candidate, &mut hard_budget).map_err(
                |required| GroupExpansionError::ExpansionWorkLimitExceeded {
                    required,
                    maximum: hard_budget_maximum,
                },
            )?;
        if ranking_direction_violations(expanded_graph, &candidate) != 0
            || !constraints_are_satisfied(&candidate, &options.constraints)
            || !geometry_is_clean
        {
            continue;
        }
        let score = CandidateScore {
            quality: routing::route_quality(&expanded_indexed, &candidate.edges),
            displacement: squared_distance(
                Point {
                    x: x + member_layout.width / 2.0,
                    y: y + member_layout.height / 2.0,
                },
                anchor_center,
            ),
            area: candidate.width * candidate.height,
            x,
            y,
        };
        if best
            .as_ref()
            .is_none_or(|(current, _)| score.cmp(*current).is_lt())
        {
            best = Some((score, candidate));
        }
    }
    best.map(|(_, layout)| layout)
        .ok_or(GroupExpansionError::NeedsFullRelayout)
}

fn constraint_x_candidates(
    compact_layout: &Layout,
    member_layout: &Layout,
    members: &BTreeSet<NodeId>,
    constraints: &LayoutConstraints,
) -> Vec<f64> {
    let compact_nodes = compact_layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let member_nodes = member_layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let mut candidates = Vec::new();
    let retained_input = constraints
        .inputs
        .iter()
        .find(|node| !members.contains(node))
        .and_then(|node| compact_nodes.get(node).copied());
    let member_input = constraints
        .inputs
        .iter()
        .find(|node| members.contains(node))
        .and_then(|node| member_nodes.get(node).copied());
    if let (Some(retained), Some(member)) = (retained_input, member_input) {
        let offset = retained.x - member.x;
        if offset >= 0.0 {
            candidates.push(offset);
        }
    }
    let retained_output = constraints
        .outputs
        .iter()
        .find(|node| !members.contains(node))
        .and_then(|node| compact_nodes.get(node).copied());
    let member_output = constraints
        .outputs
        .iter()
        .find(|node| members.contains(node))
        .and_then(|node| member_nodes.get(node).copied());
    if let (Some(retained), Some(member)) = (retained_output, member_output) {
        let offset = retained.x + retained.width - member.x - member.width;
        if offset >= 0.0 {
            candidates.push(offset);
        }
    }
    let mut seen = BTreeSet::new();
    candidates.retain(|x| seen.insert(x.to_bits()));
    candidates
}

fn prioritize_constraint_positions(positions: &mut Vec<(f64, f64)>, preferred_x: Vec<f64>) {
    if preferred_x.is_empty() || positions.len() < SAFETY_CANDIDATES {
        return;
    }
    let offset_count = positions.len() - SAFETY_CANDIDATES;
    let safety = positions.split_off(offset_count);
    let mut y_candidates = Vec::new();
    let mut seen_y = BTreeSet::new();
    for &(_, y) in positions.iter().chain(&safety) {
        if seen_y.insert(y.to_bits()) {
            y_candidates.push(y);
        }
    }
    let original = std::mem::take(positions);
    for x in preferred_x {
        for &y in &y_candidates {
            positions.push((x, y));
        }
    }
    positions.extend(original);
    let mut seen = BTreeSet::new();
    positions.retain(|&(x, y)| seen.insert((x.to_bits(), y.to_bits())));
    positions.truncate(offset_count);
    positions.extend(safety);
    let mut seen = BTreeSet::new();
    positions.retain(|&(x, y)| seen.insert((x.to_bits(), y.to_bits())));
}

fn validate_contract<'a>(
    compact_graph: &'a Graph,
    compact_layout: &'a Layout,
    expanded_graph: &'a Graph,
    expansion: &GroupExpansion,
) -> Result<ExpansionContract<'a>, GroupExpansionError> {
    if expansion.members.is_empty() {
        return Err(GroupExpansionError::EmptyMembers);
    }
    if expansion.members.len() > MAX_EXPANSION_MEMBERS {
        return Err(GroupExpansionError::TooManyMembers {
            actual: expansion.members.len(),
            maximum: MAX_EXPANSION_MEMBERS,
        });
    }
    let members = expansion.members.iter().copied().collect::<BTreeSet<_>>();
    if members.len() != expansion.members.len() {
        let mut sorted = expansion.members.clone();
        sorted.sort_unstable();
        let duplicate = sorted
            .windows(2)
            .find_map(|pair| (pair[0] == pair[1]).then_some(pair[0]))
            .expect("member count established a duplicate");
        return Err(GroupExpansionError::DuplicateMember(duplicate));
    }

    let compact_nodes = compact_graph
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let expanded_nodes = expanded_graph
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    if !compact_nodes.contains_key(&expansion.anchor) {
        return Err(GroupExpansionError::MissingAnchor(expansion.anchor));
    }
    if expanded_nodes.contains_key(&expansion.anchor) {
        return Err(GroupExpansionError::RetainedAnchor(expansion.anchor));
    }
    for &member in &members {
        if !expanded_nodes.contains_key(&member) {
            return Err(GroupExpansionError::MissingMember(member));
        }
        if compact_nodes.contains_key(&member) {
            return Err(GroupExpansionError::ExistingMember(member));
        }
    }
    for (&id, compact) in &compact_nodes {
        if id == expansion.anchor {
            continue;
        }
        let Some(expanded) = expanded_nodes.get(&id) else {
            return Err(GroupExpansionError::MissingRetainedNode(id));
        };
        if *expanded != *compact {
            return Err(GroupExpansionError::ChangedRetainedNode(id));
        }
    }
    for &id in expanded_nodes.keys() {
        if !members.contains(&id) && !compact_nodes.contains_key(&id) {
            return Err(GroupExpansionError::UnexpectedNode(id));
        }
    }

    let compact_node_geometry = index_node_geometry(compact_graph, compact_layout)?;
    let compact_edge_geometry = index_edge_geometry(compact_graph, compact_layout)?;
    validate_layout_bounds(compact_layout)?;
    let mut hard_budget = WorkBudget::new(MAX_CANDIDATE_WORK);
    match hard_geometry_is_clean_bounded(compact_graph, compact_layout, &mut hard_budget) {
        Ok(true) => {}
        Ok(false) => return Err(GroupExpansionError::InvalidCompactHardGeometry),
        Err(required) => {
            return Err(GroupExpansionError::ExpansionWorkLimitExceeded {
                required,
                maximum: MAX_CANDIDATE_WORK,
            });
        }
    }
    let anchor_geometry = compact_node_geometry[&expansion.anchor];
    let compact_edges = compact_graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let expanded_edges = expanded_graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    for (&id, compact) in &compact_edges {
        if compact.source.node == expansion.anchor || compact.target.node == expansion.anchor {
            continue;
        }
        let Some(expanded) = expanded_edges.get(&id) else {
            return Err(GroupExpansionError::MissingRetainedEdge(id));
        };
        if *expanded != *compact {
            return Err(GroupExpansionError::ChangedRetainedEdge(id));
        }
    }
    for (&id, expanded) in &expanded_edges {
        let source_member = members.contains(&expanded.source.node);
        let target_member = members.contains(&expanded.target.node);
        if source_member || target_member {
            continue;
        }
        if compact_edges.get(&id).copied() != Some(*expanded) {
            return Err(GroupExpansionError::ChangedRetainedEdge(id));
        }
    }
    let boundary_edges = expanded_edges
        .iter()
        .filter_map(|(&id, edge)| {
            (members.contains(&edge.source.node) ^ members.contains(&edge.target.node))
                .then_some((id, *edge))
        })
        .collect::<BTreeMap<_, _>>();
    let compact_trunks = compact_edges
        .iter()
        .filter_map(|(&id, edge)| {
            (edge.source.node == expansion.anchor || edge.target.node == expansion.anchor)
                .then_some((id, *edge))
        })
        .collect::<BTreeMap<_, _>>();
    let mut boundary_trunks = BTreeMap::new();
    let mut used_compact_trunks = BTreeSet::new();
    for mapping in &expansion.boundary_trunks {
        let Some(expanded) = boundary_edges.get(&mapping.expanded_edge).copied() else {
            return Err(GroupExpansionError::InvalidBoundaryEdge(
                mapping.expanded_edge,
            ));
        };
        let Some(compact) = compact_trunks.get(&mapping.compact_edge).copied() else {
            return Err(GroupExpansionError::InvalidCompactTrunk(
                mapping.compact_edge,
            ));
        };
        if boundary_trunks
            .insert(mapping.expanded_edge, mapping.compact_edge)
            .is_some()
        {
            return Err(GroupExpansionError::DuplicateBoundaryTrunk(
                mapping.expanded_edge,
            ));
        }
        let source_member = members.contains(&expanded.source.node);
        let compatible = if source_member {
            compact.source.node == expansion.anchor && compact.target == expanded.target
        } else {
            compact.target.node == expansion.anchor && compact.source == expanded.source
        };
        if !compatible {
            return Err(GroupExpansionError::IncompatibleBoundaryTrunk {
                expanded_edge: mapping.expanded_edge,
                compact_edge: mapping.compact_edge,
            });
        }
        used_compact_trunks.insert(mapping.compact_edge);
    }
    if let Some(missing) = boundary_edges
        .keys()
        .copied()
        .find(|edge| !boundary_trunks.contains_key(edge))
    {
        return Err(GroupExpansionError::MissingBoundaryTrunk(missing));
    }
    if let Some(unused) = compact_trunks
        .keys()
        .copied()
        .find(|edge| !used_compact_trunks.contains(edge))
    {
        return Err(GroupExpansionError::UnusedCompactTrunk(unused));
    }

    Ok(ExpansionContract {
        anchor_geometry,
        members,
        expanded_nodes,
        compact_node_geometry,
        compact_edge_geometry,
        boundary_trunks,
    })
}

fn index_node_geometry<'a>(
    graph: &Graph,
    layout: &'a Layout,
) -> Result<BTreeMap<NodeId, &'a NodeGeometry>, GroupExpansionError> {
    let nodes = graph
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let mut geometry = BTreeMap::new();
    for laid_out in &layout.nodes {
        if geometry.insert(laid_out.id, laid_out).is_some() {
            return Err(GroupExpansionError::DuplicateNodeGeometry(laid_out.id));
        }
        let Some(node) = nodes.get(&laid_out.id) else {
            return Err(GroupExpansionError::UnknownNodeGeometry(laid_out.id));
        };
        if !laid_out.x.is_finite()
            || !laid_out.y.is_finite()
            || laid_out.x < 0.0
            || laid_out.y < 0.0
            || laid_out.width != node.width
            || laid_out.height != node.height
        {
            return Err(GroupExpansionError::InvalidNodeGeometry(laid_out.id));
        }
    }
    for &id in nodes.keys() {
        if !geometry.contains_key(&id) {
            return Err(GroupExpansionError::MissingNodeGeometry(id));
        }
    }
    Ok(geometry)
}

fn index_edge_geometry<'a>(
    graph: &Graph,
    layout: &'a Layout,
) -> Result<BTreeMap<EdgeId, &'a EdgeGeometry>, GroupExpansionError> {
    let route_segments = layout
        .edges
        .iter()
        .map(|route| route.points.len().saturating_sub(1))
        .fold(0usize, usize::saturating_add);
    if route_segments > MAX_LAYOUT_SEGMENTS {
        return Err(GroupExpansionError::TooManyCompactRouteSegments {
            actual: route_segments,
            maximum: MAX_LAYOUT_SEGMENTS,
        });
    }
    let edges = graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let nodes = graph
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let node_geometry = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let mut geometry = BTreeMap::new();
    for route in &layout.edges {
        if geometry.insert(route.id, route).is_some() {
            return Err(GroupExpansionError::DuplicateEdgeGeometry(route.id));
        }
        let Some(edge) = edges.get(&route.id) else {
            return Err(GroupExpansionError::UnknownEdgeGeometry(route.id));
        };
        let valid_points = route.points.len() >= 2
            && route
                .points
                .iter()
                .all(|point| point.x.is_finite() && point.y.is_finite())
            && route.points.windows(2).all(|points| {
                (points[0].x - points[1].x).abs() + (points[0].y - points[1].y).abs()
                    > HARD_GATE_EPSILON
                    && (points[0].x == points[1].x || points[0].y == points[1].y)
            });
        let source = endpoint_point(
            node_geometry[&edge.source.node],
            nodes[&edge.source.node],
            edge.source,
        );
        let target = endpoint_point(
            node_geometry[&edge.target.node],
            nodes[&edge.target.node],
            edge.target,
        );
        if !valid_points
            || route.points.first().copied() != Some(source)
            || route.points.last().copied() != Some(target)
            || !correct_direction(
                source,
                route.points[1],
                port(nodes[&edge.source.node], edge.source).side,
            )
            || !correct_direction(
                target,
                route.points[route.points.len() - 2],
                port(nodes[&edge.target.node], edge.target).side,
            )
        {
            return Err(GroupExpansionError::InvalidEdgeGeometry(route.id));
        }
    }
    for &id in edges.keys() {
        if !geometry.contains_key(&id) {
            return Err(GroupExpansionError::MissingEdgeGeometry(id));
        }
    }
    Ok(geometry)
}

fn validate_layout_bounds(layout: &Layout) -> Result<(), GroupExpansionError> {
    if !layout.width.is_finite()
        || !layout.height.is_finite()
        || layout.width < 0.0
        || layout.height < 0.0
    {
        return Err(GroupExpansionError::InvalidLayoutBounds);
    }
    let bounds_contain_nodes = layout.nodes.iter().all(|node| {
        node.x >= 0.0
            && node.y >= 0.0
            && node.x + node.width <= layout.width
            && node.y + node.height <= layout.height
    });
    let bounds_contain_edges = layout
        .edges
        .iter()
        .flat_map(|edge| &edge.points)
        .all(|point| {
            point.x >= 0.0 && point.y >= 0.0 && point.x <= layout.width && point.y <= layout.height
        });
    if bounds_contain_nodes && bounds_contain_edges {
        Ok(())
    } else {
        Err(GroupExpansionError::InvalidLayoutBounds)
    }
}

fn member_graph(expanded_graph: &Graph, members: &BTreeSet<NodeId>) -> Graph {
    let mut nodes = expanded_graph
        .nodes
        .iter()
        .filter(|node| members.contains(&node.id))
        .cloned()
        .collect::<Vec<_>>();
    let mut edges = expanded_graph
        .edges
        .iter()
        .filter(|edge| members.contains(&edge.source.node) && members.contains(&edge.target.node))
        .cloned()
        .collect::<Vec<_>>();
    nodes.sort_unstable_by_key(|node| node.id);
    edges.sort_unstable_by_key(|edge| edge.id);
    Graph { nodes, edges }
}

fn pack_disconnected_components(
    graph: &Graph,
    layout: &Layout,
    gap: f64,
    maximum_width: Option<f64>,
) -> Layout {
    let mut adjacency = graph
        .nodes
        .iter()
        .map(|node| (node.id, Vec::new()))
        .collect::<BTreeMap<_, _>>();
    for edge in &graph.edges {
        adjacency
            .get_mut(&edge.source.node)
            .expect("validated graph contains edge source")
            .push(edge.target.node);
        adjacency
            .get_mut(&edge.target.node)
            .expect("validated graph contains edge target")
            .push(edge.source.node);
    }
    for neighbors in adjacency.values_mut() {
        neighbors.sort_unstable();
        neighbors.dedup();
    }
    let mut unvisited = adjacency.keys().copied().collect::<BTreeSet<_>>();
    let mut components = Vec::<Vec<NodeId>>::new();
    while let Some(root) = unvisited.pop_first() {
        let mut stack = vec![root];
        let mut component = Vec::new();
        while let Some(node) = stack.pop() {
            component.push(node);
            for &neighbor in &adjacency[&node] {
                if unvisited.remove(&neighbor) {
                    stack.push(neighbor);
                }
            }
        }
        component.sort_unstable();
        components.push(component);
    }
    if components.len() <= 1 {
        return layout.clone();
    }

    let nodes = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let edges = layout
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let graph_edges = graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let mut component_by_node = BTreeMap::new();
    for (index, component) in components.iter().enumerate() {
        for &node in component {
            component_by_node.insert(node, index);
        }
    }
    let mut component_edges = vec![Vec::<EdgeId>::new(); components.len()];
    for edge in graph_edges.values() {
        let component = component_by_node[&edge.source.node];
        debug_assert_eq!(component, component_by_node[&edge.target.node]);
        component_edges[component].push(edge.id);
    }
    let bounds = components
        .iter()
        .enumerate()
        .map(|(index, component)| {
            component_bounds(
                component.iter().map(|id| nodes[id]),
                component_edges[index].iter().map(|id| edges[id]),
            )
        })
        .collect::<Vec<_>>();
    let max_width = bounds
        .iter()
        .map(|bounds| bounds.width())
        .fold(0.0, f64::max);
    let max_height = bounds
        .iter()
        .map(|bounds| bounds.height())
        .fold(0.0, f64::max);
    let ideal_columns =
        (((components.len() as f64 * (max_height + gap) / (max_width + gap)).sqrt()).ceil()
            as usize)
            .clamp(1, components.len());
    let maximum_columns = maximum_width.map_or(components.len(), |width| {
        (((width + gap) / (max_width + gap)).floor() as usize).max(1)
    });
    let columns = ideal_columns.min(maximum_columns);
    let rows = components.len().div_ceil(columns);
    let mut column_widths = vec![0.0_f64; columns];
    let mut row_heights = vec![0.0_f64; rows];
    for (index, bounds) in bounds.iter().enumerate() {
        column_widths[index % columns] = column_widths[index % columns].max(bounds.width());
        row_heights[index / columns] = row_heights[index / columns].max(bounds.height());
    }
    let mut column_x = vec![0.0; columns];
    for index in 1..columns {
        column_x[index] = column_x[index - 1] + column_widths[index - 1] + gap;
    }
    let mut row_y = vec![0.0; rows];
    for index in 1..rows {
        row_y[index] = row_y[index - 1] + row_heights[index - 1] + gap;
    }

    let mut translated_nodes = Vec::with_capacity(layout.nodes.len());
    let mut translated_edges = Vec::with_capacity(layout.edges.len());
    for (index, component) in components.iter().enumerate() {
        let column = index % columns;
        let row = index / columns;
        let bounds = bounds[index];
        let x = column_x[column] + (column_widths[column] - bounds.width()) / 2.0 - bounds.left;
        let y = row_y[row] + (row_heights[row] - bounds.height()) / 2.0 - bounds.top;
        for &id in component {
            let node = nodes[&id];
            translated_nodes.push(NodeGeometry {
                id,
                x: node.x + x,
                y: node.y + y,
                width: node.width,
                height: node.height,
            });
        }
        for &id in &component_edges[index] {
            translated_edges.push(EdgeGeometry {
                id,
                points: edges[&id]
                    .points
                    .iter()
                    .map(|point| Point {
                        x: point.x + x,
                        y: point.y + y,
                    })
                    .collect(),
            });
        }
    }
    translated_nodes.sort_unstable_by_key(|node| node.id);
    translated_edges.sort_unstable_by_key(|edge| edge.id);
    Layout {
        nodes: translated_nodes,
        edges: translated_edges,
        width: column_widths.iter().sum::<f64>() + gap * columns.saturating_sub(1) as f64,
        height: row_heights.iter().sum::<f64>() + gap * rows.saturating_sub(1) as f64,
    }
}

fn boundary_horizontal_span(
    expanded_graph: &Graph,
    contract: &ExpansionContract<'_>,
    gap: f64,
) -> Option<f64> {
    let mut incoming_right = None::<f64>;
    let mut outgoing_left = None::<f64>;
    for edge in expanded_graph
        .edges
        .iter()
        .filter(|edge| edge.participates_in_ranking)
    {
        let source_member = contract.members.contains(&edge.source.node);
        let target_member = contract.members.contains(&edge.target.node);
        if !source_member && target_member {
            let source = contract.compact_node_geometry[&edge.source.node];
            incoming_right = Some(incoming_right.map_or(source.x + source.width, |right| {
                right.max(source.x + source.width)
            }));
        } else if source_member && !target_member {
            let target = contract.compact_node_geometry[&edge.target.node];
            outgoing_left = Some(outgoing_left.map_or(target.x, |left| left.min(target.x)));
        }
    }
    incoming_right.zip(outgoing_left).and_then(|(right, left)| {
        let width = left - right - gap * 2.0;
        (width > 0.0).then_some(width)
    })
}

fn ranking_direction_violations(graph: &Graph, layout: &Layout) -> usize {
    let nodes = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let ranking_edges = crate::effective_ranking_edges(graph);
    graph
        .edges
        .iter()
        .filter(|edge| {
            ranking_edges.contains(&edge.id)
                && nodes[&edge.source.node].x + nodes[&edge.source.node].width
                    > nodes[&edge.target.node].x + HARD_GATE_EPSILON
        })
        .count()
}

fn constraints_are_satisfied(layout: &Layout, constraints: &LayoutConstraints) -> bool {
    let nodes = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let inputs = constraints
        .inputs
        .iter()
        .filter_map(|node| nodes.get(node).copied())
        .collect::<Vec<_>>();
    if let Some(first) = inputs.first() {
        if inputs
            .iter()
            .any(|node| (node.x - first.x).abs() > HARD_GATE_EPSILON)
        {
            return false;
        }
        let input_ids = constraints.inputs.iter().copied().collect::<BTreeSet<_>>();
        let input_right = inputs
            .iter()
            .map(|node| node.x + node.width)
            .fold(f64::NEG_INFINITY, f64::max);
        if layout
            .nodes
            .iter()
            .filter(|node| !input_ids.contains(&node.id))
            .any(|node| input_right > node.x + HARD_GATE_EPSILON)
        {
            return false;
        }
    }
    let outputs = constraints
        .outputs
        .iter()
        .filter_map(|node| nodes.get(node).copied())
        .collect::<Vec<_>>();
    if let Some(first) = outputs.first() {
        let right = first.x + first.width;
        if outputs
            .iter()
            .any(|node| (node.x + node.width - right).abs() > HARD_GATE_EPSILON)
        {
            return false;
        }
        let output_ids = constraints.outputs.iter().copied().collect::<BTreeSet<_>>();
        let output_left = outputs
            .iter()
            .map(|node| node.x)
            .fold(f64::INFINITY, f64::min);
        if layout
            .nodes
            .iter()
            .filter(|node| !output_ids.contains(&node.id))
            .any(|node| node.x + node.width > output_left + HARD_GATE_EPSILON)
        {
            return false;
        }
    }
    true
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SegmentOrientation {
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug)]
struct HardSegment {
    edge: EdgeId,
    net: NetId,
    orientation: SegmentOrientation,
    fixed: f64,
    start: f64,
    end: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct FloatKey(f64);

impl Eq for FloatKey {}

impl Ord for FloatKey {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}

impl PartialOrd for FloatKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn hard_geometry_is_clean_bounded(
    graph: &Graph,
    layout: &Layout,
    budget: &mut WorkBudget,
) -> Result<bool, usize> {
    let bounds_work = layout
        .edges
        .iter()
        .map(|route| route.points.len())
        .fold(layout.nodes.len(), usize::saturating_add);
    budget.take(bounds_work)?;
    if validate_layout_bounds(layout).is_err() {
        return Ok(false);
    }
    if nodes_overlap(&layout.nodes, budget)? {
        return Ok(false);
    }
    let graph_nodes = graph
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let laid_out_nodes = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let graph_edges = graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let obstacles = ObstacleIndex::new(layout.nodes.iter().map(Rect::from_node).collect());
    let mut segments = Vec::new();
    for route in &layout.edges {
        budget.take(1)?;
        let Some(edge) = graph_edges.get(&route.id).copied() else {
            return Ok(false);
        };
        if route.points.len() < 2 {
            return Ok(false);
        }
        let source = endpoint_point(
            laid_out_nodes[&edge.source.node],
            graph_nodes[&edge.source.node],
            edge.source,
        );
        let target = endpoint_point(
            laid_out_nodes[&edge.target.node],
            graph_nodes[&edge.target.node],
            edge.target,
        );
        if route.points.first().copied() != Some(source)
            || route.points.last().copied() != Some(target)
            || !correct_direction(
                source,
                route.points[1],
                port(graph_nodes[&edge.source.node], edge.source).side,
            )
            || !correct_direction(
                target,
                route.points[route.points.len() - 2],
                port(graph_nodes[&edge.target.node], edge.target).side,
            )
        {
            return Ok(false);
        }
        for points in route.points.windows(2) {
            budget.take(1)?;
            if (points[0].x - points[1].x).abs() + (points[0].y - points[1].y).abs()
                <= HARD_GATE_EPSILON
                || obstacles.segment_crosses_interior_with_epsilon_bounded(
                    points[0],
                    points[1],
                    HARD_GATE_EPSILON,
                    budget,
                )?
            {
                return Ok(false);
            }
            let (orientation, fixed, start, end) = if points[0].y == points[1].y {
                (
                    SegmentOrientation::Horizontal,
                    points[0].y,
                    points[0].x.min(points[1].x),
                    points[0].x.max(points[1].x),
                )
            } else if points[0].x == points[1].x {
                (
                    SegmentOrientation::Vertical,
                    points[0].x,
                    points[0].y.min(points[1].y),
                    points[0].y.max(points[1].y),
                )
            } else {
                return Ok(false);
            };
            segments.push(HardSegment {
                edge: edge.id,
                net: edge.net,
                orientation,
                fixed,
                start,
                end,
            });
            if segments.len() > MAX_LAYOUT_SEGMENTS {
                return Ok(false);
            }
        }
    }
    if !no_unrelated_collinear_contacts(&segments, &graph_edges, budget)? {
        return Ok(false);
    }
    no_unrelated_perpendicular_contacts(&segments, &graph_edges, budget)
}

#[cfg(test)]
fn hard_geometry_is_clean(graph: &Graph, layout: &Layout) -> bool {
    hard_geometry_is_clean_bounded(graph, layout, &mut WorkBudget::new(usize::MAX)).unwrap_or(false)
}

fn nodes_overlap(nodes: &[NodeGeometry], budget: &mut WorkBudget) -> Result<bool, usize> {
    let mut ordered = nodes.iter().collect::<Vec<_>>();
    ordered.sort_by(|left, right| left.x.total_cmp(&right.x).then(left.id.cmp(&right.id)));
    for (index, left) in ordered.iter().enumerate() {
        let right_edge = left.x + left.width;
        for right in ordered.iter().skip(index + 1) {
            budget.take(1)?;
            if right.x >= right_edge {
                break;
            }
            if right.y < left.y + left.height && right.y + right.height > left.y {
                return Ok(true);
            }
        }
    }
    Ok(false)
}

fn no_unrelated_collinear_contacts(
    segments: &[HardSegment],
    edges: &BTreeMap<EdgeId, &Edge>,
    budget: &mut WorkBudget,
) -> Result<bool, usize> {
    let mut groups = BTreeMap::<(u8, FloatKey), Vec<usize>>::new();
    for (index, segment) in segments.iter().enumerate() {
        groups
            .entry((
                u8::from(segment.orientation == SegmentOrientation::Vertical),
                FloatKey(segment.fixed),
            ))
            .or_default()
            .push(index);
    }
    for group in groups.values_mut() {
        budget.take(group.len())?;
        group.sort_by(|&left, &right| {
            segments[left]
                .start
                .total_cmp(&segments[right].start)
                .then(segments[left].end.total_cmp(&segments[right].end))
                .then(segments[left].edge.cmp(&segments[right].edge))
        });
        let mut active_by_end = BTreeSet::<(FloatKey, usize)>::new();
        let mut relations = RelationCounts::default();
        for &index in group.iter() {
            budget.take(1)?;
            let current = segments[index];
            while active_by_end
                .first()
                .is_some_and(|(end, _)| end.0 < current.start)
            {
                let (_, expired) = active_by_end.pop_first().expect("first item existed");
                relations.remove(segments[expired], edges);
            }
            if relations.total > relations.related(current, edges) {
                return Ok(false);
            }
            relations.add(current, edges);
            active_by_end.insert((FloatKey(current.end), index));
        }
    }
    Ok(true)
}

type EndpointKey = (NodeId, u32);
type EndpointPair = (EndpointKey, EndpointKey);

#[derive(Default)]
struct RelationCounts {
    total: usize,
    by_net: BTreeMap<NetId, usize>,
    by_endpoint: BTreeMap<EndpointKey, usize>,
    by_net_endpoint: BTreeMap<(NetId, EndpointKey), usize>,
    by_pair: BTreeMap<EndpointPair, usize>,
    by_net_pair: BTreeMap<(NetId, EndpointPair), usize>,
}

impl RelationCounts {
    fn add(&mut self, segment: HardSegment, edges: &BTreeMap<EdgeId, &Edge>) {
        let (source, target, pair) = relation_keys(edges[&segment.edge]);
        self.total += 1;
        increment(&mut self.by_net, segment.net);
        increment(&mut self.by_endpoint, source);
        increment(&mut self.by_net_endpoint, (segment.net, source));
        if target != source {
            increment(&mut self.by_endpoint, target);
            increment(&mut self.by_net_endpoint, (segment.net, target));
        }
        increment(&mut self.by_pair, pair);
        increment(&mut self.by_net_pair, (segment.net, pair));
    }

    fn remove(&mut self, segment: HardSegment, edges: &BTreeMap<EdgeId, &Edge>) {
        let (source, target, pair) = relation_keys(edges[&segment.edge]);
        self.total -= 1;
        decrement(&mut self.by_net, segment.net);
        decrement(&mut self.by_endpoint, source);
        decrement(&mut self.by_net_endpoint, (segment.net, source));
        if target != source {
            decrement(&mut self.by_endpoint, target);
            decrement(&mut self.by_net_endpoint, (segment.net, target));
        }
        decrement(&mut self.by_pair, pair);
        decrement(&mut self.by_net_pair, (segment.net, pair));
    }

    fn related(&self, segment: HardSegment, edges: &BTreeMap<EdgeId, &Edge>) -> usize {
        let (source, target, pair) = relation_keys(edges[&segment.edge]);
        let net = self.by_net.get(&segment.net).copied().unwrap_or(0);
        let source_count = self.by_endpoint.get(&source).copied().unwrap_or(0);
        let net_source = self
            .by_net_endpoint
            .get(&(segment.net, source))
            .copied()
            .unwrap_or(0);
        if source == target {
            return net + source_count - net_source;
        }
        let target_count = self.by_endpoint.get(&target).copied().unwrap_or(0);
        let net_target = self
            .by_net_endpoint
            .get(&(segment.net, target))
            .copied()
            .unwrap_or(0);
        let both = self.by_pair.get(&pair).copied().unwrap_or(0);
        let net_both = self
            .by_net_pair
            .get(&(segment.net, pair))
            .copied()
            .unwrap_or(0);
        net + source_count + target_count - net_source - net_target - both + net_both
    }
}

fn relation_keys(edge: &Edge) -> (EndpointKey, EndpointKey, EndpointPair) {
    let source = (edge.source.node, edge.source.port);
    let target = (edge.target.node, edge.target.port);
    let pair = if source <= target {
        (source, target)
    } else {
        (target, source)
    };
    (source, target, pair)
}

fn increment<K: Ord + Copy>(counts: &mut BTreeMap<K, usize>, key: K) {
    *counts.entry(key).or_default() += 1;
}

fn decrement<K: Ord + Copy>(counts: &mut BTreeMap<K, usize>, key: K) {
    let count = counts.get_mut(&key).expect("relation count exists");
    *count -= 1;
    if *count == 0 {
        counts.remove(&key);
    }
}

fn no_unrelated_perpendicular_contacts(
    segments: &[HardSegment],
    edges: &BTreeMap<EdgeId, &Edge>,
    budget: &mut WorkBudget,
) -> Result<bool, usize> {
    let horizontal = segments
        .iter()
        .filter(|segment| segment.orientation == SegmentOrientation::Horizontal)
        .collect::<Vec<_>>();
    let vertical = segments
        .iter()
        .filter(|segment| segment.orientation == SegmentOrientation::Vertical)
        .collect::<Vec<_>>();
    if !no_endpoint_contacts(&horizontal, &vertical, edges, budget)? {
        return Ok(false);
    }
    no_endpoint_contacts(&vertical, &horizontal, edges, budget)
}

#[derive(Default)]
struct ContactEvents {
    add: Vec<usize>,
    query: Vec<usize>,
    remove: Vec<usize>,
}

fn no_endpoint_contacts(
    intervals: &[&HardSegment],
    queries: &[&HardSegment],
    edges: &BTreeMap<EdgeId, &Edge>,
    budget: &mut WorkBudget,
) -> Result<bool, usize> {
    let mut tracks = BTreeMap::<FloatKey, BTreeMap<FloatKey, ContactEvents>>::new();
    for (index, interval) in intervals.iter().enumerate() {
        budget.take(1)?;
        tracks
            .entry(FloatKey(interval.fixed))
            .or_default()
            .entry(FloatKey(interval.start))
            .or_default()
            .add
            .push(index);
        tracks
            .entry(FloatKey(interval.fixed))
            .or_default()
            .entry(FloatKey(interval.end))
            .or_default()
            .remove
            .push(index);
    }
    for (index, query) in queries.iter().enumerate() {
        budget.take(1)?;
        for endpoint in [query.start, query.end] {
            tracks
                .entry(FloatKey(endpoint))
                .or_default()
                .entry(FloatKey(query.fixed))
                .or_default()
                .query
                .push(index);
        }
    }
    for events in tracks.values() {
        let mut relations = RelationCounts::default();
        for event in events.values() {
            budget.take(event.add.len() + event.query.len() + event.remove.len())?;
            for &index in &event.add {
                relations.add(*intervals[index], edges);
            }
            for &index in &event.query {
                if relations.total > relations.related(*queries[index], edges) {
                    return Ok(false);
                }
            }
            for &index in &event.remove {
                relations.remove(*intervals[index], edges);
            }
        }
    }
    Ok(true)
}

fn component_bounds<'a>(
    nodes: impl Iterator<Item = &'a NodeGeometry>,
    edges: impl Iterator<Item = &'a EdgeGeometry>,
) -> Rect {
    let mut left = f64::INFINITY;
    let mut top = f64::INFINITY;
    let mut right = f64::NEG_INFINITY;
    let mut bottom = f64::NEG_INFINITY;
    for node in nodes {
        left = left.min(node.x);
        top = top.min(node.y);
        right = right.max(node.x + node.width);
        bottom = bottom.max(node.y + node.height);
    }
    for point in edges.flat_map(|edge| &edge.points) {
        left = left.min(point.x);
        top = top.min(point.y);
        right = right.max(point.x);
        bottom = bottom.max(point.y);
    }
    Rect {
        left,
        top,
        right,
        bottom,
    }
}

fn projected_segment_count(
    expanded_graph: &Graph,
    contract: &ExpansionContract<'_>,
    member_layout: &Layout,
) -> usize {
    let member_routes = member_layout
        .edges
        .iter()
        .map(|route| (route.id, route.points.len().saturating_sub(1)))
        .collect::<BTreeMap<_, _>>();
    expanded_graph.edges.iter().fold(0usize, |total, edge| {
        let source_member = contract.members.contains(&edge.source.node);
        let target_member = contract.members.contains(&edge.target.node);
        let segments = match (source_member, target_member) {
            (false, false) => contract.compact_edge_geometry[&edge.id]
                .points
                .len()
                .saturating_sub(1),
            (true, true) => member_routes[&edge.id],
            _ => {
                let trunk = contract.boundary_trunks[&edge.id];
                contract.compact_edge_geometry[&trunk]
                    .points
                    .len()
                    .saturating_sub(1)
                    .saturating_add(5)
            }
        };
        total.saturating_add(segments)
    })
}

fn candidate_positions(
    compact_layout: &Layout,
    anchor: &NodeGeometry,
    member_layout: &Layout,
    work: ExpansionWork,
    options: LayoutOptions,
    effort: QualityEffort,
) -> Result<Vec<(f64, f64)>, GroupExpansionError> {
    let radius: i32 = match effort {
        QualityEffort::Fast => 0,
        QualityEffort::Quality => 1,
        QualityEffort::Max => 3,
    };
    let budget = candidate_work_budget(effort);
    let bridge_work = work.boundary_edges.saturating_mul(work.nodes);
    let work_per_candidate = work
        .nodes
        .saturating_add(work.edges)
        .saturating_add(work.projected_segments)
        .saturating_add(bridge_work)
        .max(1);
    let minimum_work = work_per_candidate.saturating_mul(SAFETY_CANDIDATES + 1);
    if minimum_work > budget {
        return Err(GroupExpansionError::ExpansionWorkLimitExceeded {
            required: minimum_work,
            maximum: budget,
        });
    }
    let candidate_limit = (budget / work_per_candidate).saturating_sub(SAFETY_CANDIDATES);
    let mut offsets = Vec::new();
    for distance in 0..=radius * 2 {
        for dx in -radius..=radius {
            for dy in -radius..=radius {
                if dx.abs() + dy.abs() == distance {
                    offsets.push((dx, dy));
                }
            }
        }
    }
    offsets.truncate(candidate_limit);

    let centered_x = anchor.x + anchor.width / 2.0 - member_layout.width / 2.0;
    let centered_y = anchor.y + anchor.height / 2.0 - member_layout.height / 2.0;
    let step_x = member_layout.width + options.node_gap * 2.0;
    let step_y = member_layout.height + options.node_gap * 2.0;
    let margin = options.node_gap / 2.0;
    let mut positions = offsets
        .into_iter()
        .map(|(dx, dy)| {
            (
                (centered_x + f64::from(dx) * step_x).max(margin),
                (centered_y + f64::from(dy) * step_y).max(margin),
            )
        })
        .collect::<Vec<_>>();
    positions.push((
        compact_layout.width + options.node_gap,
        centered_y.max(margin),
    ));
    positions.push((
        centered_x.max(margin),
        compact_layout.height + options.node_gap,
    ));
    let mut seen = BTreeSet::new();
    positions.retain(|&(x, y)| seen.insert((x.to_bits(), y.to_bits())));
    debug_assert!(positions.len().saturating_mul(work_per_candidate) <= budget);
    Ok(positions)
}

fn candidate_work_budget(effort: QualityEffort) -> usize {
    match effort {
        QualityEffort::Fast => FAST_CANDIDATE_WORK,
        QualityEffort::Quality => QUALITY_CANDIDATE_WORK,
        QualityEffort::Max => MAX_CANDIDATE_WORK,
    }
}

#[allow(clippy::too_many_arguments)]
fn compose_candidate(
    compact_layout: &Layout,
    expanded_graph: &Graph,
    contract: &ExpansionContract<'_>,
    member_layout: &Layout,
    offset_x: f64,
    offset_y: f64,
    options: LayoutOptions,
) -> Result<Layout, GroupExpansionError> {
    let local_nodes = member_layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let mut nodes = contract
        .expanded_nodes
        .values()
        .map(|node| {
            if contract.members.contains(&node.id) {
                let local = local_nodes[&node.id];
                NodeGeometry {
                    id: node.id,
                    x: local.x + offset_x,
                    y: local.y + offset_y,
                    width: local.width,
                    height: local.height,
                }
            } else {
                contract.compact_node_geometry[&node.id].clone()
            }
        })
        .collect::<Vec<_>>();
    nodes.sort_unstable_by_key(|node| node.id);
    let node_geometry = nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let obstacles = ObstacleIndex::new(nodes.iter().map(Rect::from_node).collect());
    let local_edges = member_layout
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let mut edges = Vec::with_capacity(expanded_graph.edges.len());
    let mut expanded_edges = expanded_graph.edges.iter().collect::<Vec<_>>();
    expanded_edges.sort_unstable_by_key(|edge| edge.id);
    for edge in expanded_edges {
        let source_member = contract.members.contains(&edge.source.node);
        let target_member = contract.members.contains(&edge.target.node);
        let route = match (source_member, target_member) {
            (false, false) => contract.compact_edge_geometry[&edge.id].clone(),
            (true, true) => {
                let local = local_edges[&edge.id];
                EdgeGeometry {
                    id: edge.id,
                    points: local
                        .points
                        .iter()
                        .map(|point| Point {
                            x: point.x + offset_x,
                            y: point.y + offset_y,
                        })
                        .collect(),
                }
            }
            _ => boundary_route(edge, contract, &node_geometry, &obstacles, options)?,
        };
        edges.push(route);
    }

    let mut width = compact_layout.width;
    let mut height = compact_layout.height;
    for node in &nodes {
        width = width.max(node.x + node.width + options.node_gap / 2.0);
        height = height.max(node.y + node.height + options.node_gap / 2.0);
    }
    for point in edges.iter().flat_map(|edge| &edge.points) {
        width = width.max(point.x);
        height = height.max(point.y);
    }
    Ok(Layout {
        nodes,
        edges,
        width,
        height,
    })
}

fn boundary_route(
    edge: &Edge,
    contract: &ExpansionContract<'_>,
    node_geometry: &BTreeMap<NodeId, &NodeGeometry>,
    obstacles: &ObstacleIndex,
    options: LayoutOptions,
) -> Result<EdgeGeometry, GroupExpansionError> {
    let source_member = contract.members.contains(&edge.source.node);
    let target_member = contract.members.contains(&edge.target.node);
    debug_assert_ne!(source_member, target_member);
    let trunk_id = contract.boundary_trunks[&edge.id];
    let trunk_geometry = contract.compact_edge_geometry[&trunk_id];
    let trunk_start = trunk_geometry
        .points
        .first()
        .copied()
        .ok_or(GroupExpansionError::EmptyBoundaryTrunk(edge.id))?;
    let trunk_end = trunk_geometry
        .points
        .last()
        .copied()
        .ok_or(GroupExpansionError::EmptyBoundaryTrunk(edge.id))?;
    let points = if source_member {
        let source_node = contract.expanded_nodes[&edge.source.node];
        let source_port = port(source_node, edge.source);
        let source = endpoint_point(node_geometry[&edge.source.node], source_node, edge.source);
        let source_stub = outward_stub(source, source_port.side, options.port_stub);
        let bridge =
            obstacle_safe_bridge(source_stub, trunk_start, obstacles, options.node_gap / 2.0)
                .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
        let mut points = vec![source, source_stub];
        points.extend(bridge.into_iter().skip(1));
        points.extend(trunk_geometry.points.iter().copied().skip(1));
        points
    } else {
        let target_node = contract.expanded_nodes[&edge.target.node];
        let target_port = port(target_node, edge.target);
        let target = endpoint_point(node_geometry[&edge.target.node], target_node, edge.target);
        let target_stub = outward_stub(target, target_port.side, options.port_stub);
        let bridge =
            obstacle_safe_bridge(trunk_end, target_stub, obstacles, options.node_gap / 2.0)
                .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
        let mut points = trunk_geometry.points.clone();
        points.extend(bridge.into_iter().skip(1));
        points.push(target);
        points
    };
    Ok(EdgeGeometry {
        id: edge.id,
        points: simplify_orthogonal(points),
    })
}

fn obstacle_safe_bridge(
    start: Point,
    end: Point,
    obstacles: &ObstacleIndex,
    gap: f64,
) -> Option<Vec<Point>> {
    if start == end {
        return Some(vec![start]);
    }
    let mut candidates = vec![
        vec![start, end],
        vec![
            start,
            Point {
                x: end.x,
                y: start.y,
            },
            end,
        ],
        vec![
            start,
            Point {
                x: start.x,
                y: end.y,
            },
            end,
        ],
    ];
    let right = obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.right)
        .fold(start.x.max(end.x), f64::max)
        + gap;
    let bottom = obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.bottom)
        .fold(start.y.max(end.y), f64::max)
        + gap;
    let left = (obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.left)
        .fold(start.x.min(end.x), f64::min)
        - gap)
        .max(0.0);
    let top = (obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.top)
        .fold(start.y.min(end.y), f64::min)
        - gap)
        .max(0.0);
    for x in [(start.x + end.x) / 2.0, left, right] {
        candidates.push(vec![
            start,
            Point { x, y: start.y },
            Point { x, y: end.y },
            end,
        ]);
    }
    for y in [(start.y + end.y) / 2.0, top, bottom] {
        candidates.push(vec![
            start,
            Point { x: start.x, y },
            Point { x: end.x, y },
            end,
        ]);
    }
    for (x, y) in [(left, top), (left, bottom), (right, top), (right, bottom)] {
        candidates.push(vec![
            start,
            Point { x: start.x, y },
            Point { x, y },
            Point { x, y: end.y },
            end,
        ]);
        candidates.push(vec![
            start,
            Point { x, y: start.y },
            Point { x, y },
            Point { x: end.x, y },
            end,
        ]);
    }

    candidates
        .into_iter()
        .map(simplify_orthogonal)
        .filter(|points| path_is_clear(points, obstacles))
        .min_by(|left, right| bridge_path_cmp(left, right))
}

fn bridge_path_cmp(left: &[Point], right: &[Point]) -> Ordering {
    let bends = |points: &[Point]| points.len().saturating_sub(2);
    bends(left)
        .cmp(&bends(right))
        .then(path_length(left).total_cmp(&path_length(right)))
        .then_with(|| point_path_cmp(left, right))
}

fn point_path_cmp(left: &[Point], right: &[Point]) -> Ordering {
    for (left, right) in left.iter().zip(right) {
        let ordering = left.x.total_cmp(&right.x).then(left.y.total_cmp(&right.y));
        if !ordering.is_eq() {
            return ordering;
        }
    }
    left.len().cmp(&right.len())
}

fn path_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|points| (points[0].x - points[1].x).abs() + (points[0].y - points[1].y).abs())
        .sum()
}

fn path_is_clear(points: &[Point], obstacles: &ObstacleIndex) -> bool {
    points.windows(2).all(|points| {
        (points[0].x == points[1].x || points[0].y == points[1].y)
            && !obstacles.segment_crosses_interior(points[0], points[1])
    })
}

fn simplify_orthogonal(points: Vec<Point>) -> Vec<Point> {
    let mut simplified = Vec::<Point>::with_capacity(points.len());
    for point in points {
        if simplified.last().copied() == Some(point) {
            continue;
        }
        if simplified.len() >= 2 {
            let before = simplified[simplified.len() - 2];
            let prior = simplified[simplified.len() - 1];
            let monotone_vertical = before.x == prior.x
                && prior.x == point.x
                && prior.y >= before.y.min(point.y)
                && prior.y <= before.y.max(point.y);
            let monotone_horizontal = before.y == prior.y
                && prior.y == point.y
                && prior.x >= before.x.min(point.x)
                && prior.x <= before.x.max(point.x);
            if monotone_vertical || monotone_horizontal {
                simplified.pop();
            }
        }
        simplified.push(point);
    }
    simplified
}

fn retained_node_overlap_area(
    compact_layout: &Layout,
    anchor: NodeId,
    frame: Rect,
    gap: f64,
) -> f64 {
    compact_layout
        .nodes
        .iter()
        .filter(|node| node.id != anchor)
        .map(|node| frame.overlap_area(Rect::from_node(node).expanded(gap)))
        .sum()
}

fn endpoint_point(geometry: &NodeGeometry, node: &Node, endpoint: Endpoint) -> Point {
    let port = port(node, endpoint);
    match port.side {
        PortSide::North => Point {
            x: geometry.x + port.offset,
            y: geometry.y,
        },
        PortSide::East => Point {
            x: geometry.x + geometry.width,
            y: geometry.y + port.offset,
        },
        PortSide::South => Point {
            x: geometry.x + port.offset,
            y: geometry.y + geometry.height,
        },
        PortSide::West => Point {
            x: geometry.x,
            y: geometry.y + port.offset,
        },
    }
}

fn port(node: &Node, endpoint: Endpoint) -> &Port {
    node.ports
        .iter()
        .find(|port| port.id == endpoint.port)
        .expect("validated endpoint has a port")
}

fn outward_stub(point: Point, side: PortSide, distance: f64) -> Point {
    match side {
        PortSide::North => Point {
            x: point.x,
            y: point.y - distance,
        },
        PortSide::East => Point {
            x: point.x + distance,
            y: point.y,
        },
        PortSide::South => Point {
            x: point.x,
            y: point.y + distance,
        },
        PortSide::West => Point {
            x: point.x - distance,
            y: point.y,
        },
    }
}

fn correct_direction(endpoint: Point, adjacent: Point, side: PortSide) -> bool {
    let dx = adjacent.x - endpoint.x;
    let dy = adjacent.y - endpoint.y;
    match side {
        PortSide::North => dy < -HARD_GATE_EPSILON && dx.abs() <= HARD_GATE_EPSILON,
        PortSide::East => dx > HARD_GATE_EPSILON && dy.abs() <= HARD_GATE_EPSILON,
        PortSide::South => dy > HARD_GATE_EPSILON && dx.abs() <= HARD_GATE_EPSILON,
        PortSide::West => dx < -HARD_GATE_EPSILON && dy.abs() <= HARD_GATE_EPSILON,
    }
}

fn center(node: &NodeGeometry) -> Point {
    Point {
        x: node.x + node.width / 2.0,
        y: node.y + node.height / 2.0,
    }
}

fn squared_distance(left: Point, right: Point) -> f64 {
    (left.x - right.x).powi(2) + (left.y - right.y).powi(2)
}

#[derive(Clone, Copy, Debug)]
struct Rect {
    left: f64,
    top: f64,
    right: f64,
    bottom: f64,
}

struct ObstacleIndex {
    rects: Vec<Rect>,
    horizontal: Option<Box<IntervalTree>>,
    vertical: Option<Box<IntervalTree>>,
}

impl ObstacleIndex {
    fn new(rects: Vec<Rect>) -> Self {
        let indices = (0..rects.len()).collect::<Vec<_>>();
        Self {
            horizontal: IntervalTree::build(&rects, &indices, Axis::Y),
            vertical: IntervalTree::build(&rects, &indices, Axis::X),
            rects,
        }
    }

    fn segment_crosses_interior(&self, start: Point, end: Point) -> bool {
        self.segment_crosses_interior_with_epsilon(start, end, 0.0)
    }

    fn segment_crosses_interior_with_epsilon(
        &self,
        start: Point,
        end: Point,
        epsilon: f64,
    ) -> bool {
        self.segment_crosses_interior_with_epsilon_bounded(
            start,
            end,
            epsilon,
            &mut WorkBudget::new(usize::MAX),
        )
        .unwrap_or(true)
    }

    fn segment_crosses_interior_with_epsilon_bounded(
        &self,
        start: Point,
        end: Point,
        epsilon: f64,
        budget: &mut WorkBudget,
    ) -> Result<bool, usize> {
        if start.y == end.y {
            let low = start.x.min(end.x);
            let high = start.x.max(end.x);
            self.horizontal.as_ref().map_or(Ok(false), |tree| {
                tree.any_at_bounded(&self.rects, start.y, Axis::Y, budget, |rect| {
                    start.y > rect.top + epsilon
                        && start.y < rect.bottom - epsilon
                        && high > rect.left + epsilon
                        && low < rect.right - epsilon
                })
            })
        } else if start.x == end.x {
            let low = start.y.min(end.y);
            let high = start.y.max(end.y);
            self.vertical.as_ref().map_or(Ok(false), |tree| {
                tree.any_at_bounded(&self.rects, start.x, Axis::X, budget, |rect| {
                    start.x > rect.left + epsilon
                        && start.x < rect.right - epsilon
                        && high > rect.top + epsilon
                        && low < rect.bottom - epsilon
                })
            })
        } else {
            Ok(true)
        }
    }
}

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
}

impl Axis {
    fn start(self, rect: Rect) -> f64 {
        match self {
            Self::X => rect.left,
            Self::Y => rect.top,
        }
    }

    fn end(self, rect: Rect) -> f64 {
        match self {
            Self::X => rect.right,
            Self::Y => rect.bottom,
        }
    }
}

struct IntervalTree {
    center: f64,
    overlapping_by_start: Vec<usize>,
    overlapping_by_end: Vec<usize>,
    left: Option<Box<Self>>,
    right: Option<Box<Self>>,
}

impl IntervalTree {
    fn build(rects: &[Rect], indices: &[usize], axis: Axis) -> Option<Box<Self>> {
        if indices.is_empty() {
            return None;
        }
        let mut midpoints = indices
            .iter()
            .map(|&index| (axis.start(rects[index]) + axis.end(rects[index])) / 2.0)
            .collect::<Vec<_>>();
        midpoints.sort_by(f64::total_cmp);
        let center = midpoints[midpoints.len() / 2];
        let mut left = Vec::new();
        let mut right = Vec::new();
        let mut overlapping = Vec::new();
        for &index in indices {
            let rect = rects[index];
            if axis.end(rect) <= center {
                left.push(index);
            } else if axis.start(rect) >= center {
                right.push(index);
            } else {
                overlapping.push(index);
            }
        }
        let mut overlapping_by_start = overlapping.clone();
        overlapping_by_start.sort_by(|&left, &right| {
            axis.start(rects[left])
                .total_cmp(&axis.start(rects[right]))
                .then(left.cmp(&right))
        });
        let mut overlapping_by_end = overlapping;
        overlapping_by_end.sort_by(|&left, &right| {
            axis.end(rects[right])
                .total_cmp(&axis.end(rects[left]))
                .then(left.cmp(&right))
        });
        Some(Box::new(Self {
            center,
            overlapping_by_start,
            overlapping_by_end,
            left: Self::build(rects, &left, axis),
            right: Self::build(rects, &right, axis),
        }))
    }

    fn any_at_bounded(
        &self,
        rects: &[Rect],
        coordinate: f64,
        axis: Axis,
        budget: &mut WorkBudget,
        mut crosses_other_axis: impl FnMut(Rect) -> bool + Copy,
    ) -> Result<bool, usize> {
        budget.take(1)?;
        if coordinate < self.center {
            for &index in &self.overlapping_by_start {
                budget.take(1)?;
                let rect = rects[index];
                if axis.start(rect) >= coordinate {
                    break;
                }
                if crosses_other_axis(rect) {
                    return Ok(true);
                }
            }
            self.left.as_ref().map_or(Ok(false), |tree| {
                tree.any_at_bounded(rects, coordinate, axis, budget, crosses_other_axis)
            })
        } else if coordinate > self.center {
            for &index in &self.overlapping_by_end {
                budget.take(1)?;
                let rect = rects[index];
                if axis.end(rect) <= coordinate {
                    break;
                }
                if crosses_other_axis(rect) {
                    return Ok(true);
                }
            }
            self.right.as_ref().map_or(Ok(false), |tree| {
                tree.any_at_bounded(rects, coordinate, axis, budget, crosses_other_axis)
            })
        } else {
            for &index in &self.overlapping_by_start {
                budget.take(1)?;
                if crosses_other_axis(rects[index]) {
                    return Ok(true);
                }
            }
            Ok(false)
        }
    }
}

impl Rect {
    fn from_node(node: &NodeGeometry) -> Self {
        Self {
            left: node.x,
            top: node.y,
            right: node.x + node.width,
            bottom: node.y + node.height,
        }
    }

    fn expanded(self, gap: f64) -> Self {
        Self {
            left: self.left - gap,
            top: self.top - gap,
            right: self.right + gap,
            bottom: self.bottom + gap,
        }
    }

    fn width(self) -> f64 {
        self.right - self.left
    }

    fn height(self) -> f64 {
        self.bottom - self.top
    }

    fn overlap_area(self, other: Self) -> f64 {
        (self.right.min(other.right) - self.left.max(other.left)).max(0.0)
            * (self.bottom.min(other.bottom) - self.top.max(other.top)).max(0.0)
    }

    #[cfg(test)]
    fn segment_crosses_interior(self, start: Point, end: Point) -> bool {
        if start.x == end.x {
            start.x > self.left
                && start.x < self.right
                && start.y.max(end.y) > self.top
                && start.y.min(end.y) < self.bottom
        } else if start.y == end.y {
            start.y > self.top
                && start.y < self.bottom
                && start.x.max(end.x) > self.left
                && start.x.min(end.x) < self.right
        } else {
            true
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use super::{
        BoundaryTrunk, ExpansionWork, GroupExpansion, GroupExpansionError, GroupExpansionOptions,
        ObstacleIndex, Rect, candidate_positions, expand_group_in_place, obstacle_safe_bridge,
        pack_disconnected_components, path_is_clear,
    };
    use crate::{
        Edge, EdgeGeometry, Endpoint, Graph, Layout, LayoutOptions, Node, NodeGeometry, Point,
        Port, PortSide, QualityEffort, layout, layout_with_constraints,
    };

    fn node(id: u32) -> Node {
        Node {
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
        }
    }

    fn edge(id: u32, source: u32, target: u32, net: u32) -> Edge {
        Edge {
            id,
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
        }
    }

    fn fixture() -> (Graph, Graph, GroupExpansion) {
        let mut anchor = node(10);
        anchor.width = 260.0;
        let compact = Graph {
            nodes: vec![node(1), anchor, node(4), node(5)],
            edges: vec![edge(1, 1, 10, 100), edge(2, 10, 4, 200), edge(3, 4, 5, 300)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(4), node(5)],
            edges: vec![
                edge(11, 1, 2, 100),
                edge(12, 2, 3, 150),
                edge(13, 3, 4, 200),
                edge(3, 4, 5, 300),
            ],
        };
        (
            compact,
            expanded,
            GroupExpansion {
                anchor: 10,
                members: vec![2, 3],
                boundary_trunks: vec![
                    BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    },
                    BoundaryTrunk {
                        expanded_edge: 13,
                        compact_edge: 2,
                    },
                ],
            },
        )
    }

    #[test]
    fn expansion_preserves_retained_geometry_and_reuses_trunks() {
        let (compact, expanded, expansion) = fixture();
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();

        for retained in [1, 4, 5] {
            assert_eq!(
                result.nodes.iter().find(|node| node.id == retained),
                compact_layout.nodes.iter().find(|node| node.id == retained)
            );
        }
        assert_eq!(
            result.edges.iter().find(|edge| edge.id == 3),
            compact_layout.edges.iter().find(|edge| edge.id == 3)
        );
        let compact_out = compact_layout
            .edges
            .iter()
            .find(|edge| edge.id == 2)
            .unwrap();
        let expanded_out = result.edges.iter().find(|edge| edge.id == 13).unwrap();
        assert!(expanded_out.points.ends_with(&compact_out.points[1..]));
        assert!(expanded_out.points[1].x > expanded_out.points[0].x);
        assert!(result.nodes.iter().all(|node| node.id != 10));
        assert!(result.nodes.iter().any(|node| node.id == 2));
        assert!(result.nodes.iter().any(|node| node.id == 3));
        let geometry = result
            .nodes
            .iter()
            .map(|node| (node.id, node))
            .collect::<BTreeMap<_, _>>();
        let direction_violations = expanded
            .edges
            .iter()
            .filter(|edge| {
                geometry[&edge.source.node].x + geometry[&edge.source.node].width
                    > geometry[&edge.target.node].x
            })
            .count();
        assert_eq!(direction_violations, 0);
        assert!(
            geometry[&2].x + geometry[&2].width <= geometry[&3].x,
            "internal member logic must retain left-to-right flow"
        );
        assert!(result.edges.iter().all(|edge| {
            edge.points
                .windows(2)
                .all(|points| points[0].x == points[1].x || points[0].y == points[1].y)
        }));
        for (index, left) in result.nodes.iter().enumerate() {
            for right in result.nodes.iter().skip(index + 1) {
                assert_eq!(
                    Rect::from_node(left).overlap_area(Rect::from_node(right)),
                    0.0
                );
            }
        }
    }

    #[test]
    fn expansion_is_deterministic_across_graph_and_member_permutations() {
        let (compact, expanded, expansion) = fixture();
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expected = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        let mut compact_permuted = compact.clone();
        compact_permuted.nodes.reverse();
        compact_permuted.edges.reverse();
        let mut expanded_permuted = expanded.clone();
        expanded_permuted.nodes.rotate_left(2);
        expanded_permuted.edges.reverse();
        let actual = expand_group_in_place(
            &compact_permuted,
            &compact_layout,
            &expanded_permuted,
            &GroupExpansion {
                anchor: 10,
                members: vec![3, 2],
                boundary_trunks: vec![
                    BoundaryTrunk {
                        expanded_edge: 13,
                        compact_edge: 2,
                    },
                    BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    },
                ],
            },
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        assert_eq!(actual, expected);
    }

    #[test]
    fn expansion_rejects_changed_retained_semantics() {
        let (compact, mut expanded, expansion) = fixture();
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        expanded
            .nodes
            .iter_mut()
            .find(|node| node.id == 4)
            .unwrap()
            .width += 1.0;
        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &GroupExpansionOptions::default(),
            ),
            Err(GroupExpansionError::ChangedRetainedNode(4))
        );
    }

    #[test]
    fn explicit_boundary_mapping_preserves_distinct_named_trunks() {
        let mut anchor = node(10);
        anchor.width = 260.0;
        anchor.ports.push(Port {
            id: 2,
            side: PortSide::West,
            offset: 10.0,
        });
        anchor.ports.push(Port {
            id: 3,
            side: PortSide::West,
            offset: 40.0,
        });
        let incoming = |id, target_port| Edge {
            id,
            source: Endpoint { node: 1, port: 1 },
            target: Endpoint {
                node: 10,
                port: target_port,
            },
            net: 100,
            participates_in_ranking: true,
        };
        let compact = Graph {
            nodes: vec![node(1), anchor],
            edges: vec![incoming(1, 2), incoming(2, 3)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: vec![edge(11, 1, 2, 100), edge(12, 1, 3, 100)],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expansion = GroupExpansion {
            anchor: 10,
            members: vec![2, 3],
            boundary_trunks: vec![
                BoundaryTrunk {
                    expanded_edge: 11,
                    compact_edge: 2,
                },
                BoundaryTrunk {
                    expanded_edge: 12,
                    compact_edge: 1,
                },
            ],
        };

        let contract =
            super::validate_contract(&compact, &compact_layout, &expanded, &expansion).unwrap();
        assert_eq!(contract.boundary_trunks[&11], 2);
        assert_eq!(contract.boundary_trunks[&12], 1);
    }

    #[test]
    fn every_compact_anchor_trunk_requires_reverse_coverage() {
        let (compact, mut expanded, mut expansion) = fixture();
        expanded.edges.retain(|edge| edge.id != 13);
        expansion
            .boundary_trunks
            .retain(|mapping| mapping.expanded_edge != 13);
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &GroupExpansionOptions::default(),
            ),
            Err(GroupExpansionError::UnusedCompactTrunk(2))
        );
    }

    #[test]
    fn impossible_left_to_right_preservation_requires_full_relayout() {
        let compact = Graph {
            nodes: vec![node(1), node(10), node(4)],
            edges: vec![edge(1, 1, 10, 100), edge(2, 10, 4, 200)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(4)],
            edges: vec![
                edge(11, 1, 2, 100),
                edge(12, 2, 3, 150),
                edge(13, 3, 4, 200),
            ],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &GroupExpansion {
                    anchor: 10,
                    members: vec![2, 3],
                    boundary_trunks: vec![
                        BoundaryTrunk {
                            expanded_edge: 11,
                            compact_edge: 1,
                        },
                        BoundaryTrunk {
                            expanded_edge: 13,
                            compact_edge: 2,
                        },
                    ],
                },
                &GroupExpansionOptions {
                    quality_effort: QualityEffort::Max,
                    ..GroupExpansionOptions::default()
                },
            ),
            Err(GroupExpansionError::NeedsFullRelayout)
        );
    }

    #[test]
    fn member_and_retained_boundary_outputs_remain_globally_aligned() {
        let compact = Graph {
            nodes: vec![node(1), node(10), node(9)],
            edges: vec![edge(1, 1, 10, 100), edge(3, 1, 9, 300)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(9)],
            edges: vec![edge(11, 1, 2, 101), edge(12, 1, 3, 102), edge(3, 1, 9, 300)],
        };
        let compact_layout = layout_with_constraints(
            &compact,
            LayoutOptions::default(),
            &crate::LayoutConstraints {
                inputs: vec![1],
                outputs: vec![9, 10],
            },
        )
        .unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &GroupExpansion {
                anchor: 10,
                members: vec![2, 3],
                boundary_trunks: vec![
                    BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    },
                    BoundaryTrunk {
                        expanded_edge: 12,
                        compact_edge: 1,
                    },
                ],
            },
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                constraints: crate::LayoutConstraints {
                    inputs: vec![1],
                    outputs: vec![2, 3, 9],
                },
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        let geometry = result
            .nodes
            .iter()
            .map(|node| (node.id, node))
            .collect::<BTreeMap<_, _>>();

        let right = |id| geometry[&id].x + geometry[&id].width;
        assert_eq!(right(2), right(3));
        assert_eq!(right(2), right(9));
        assert!(geometry[&1].x + geometry[&1].width < geometry[&2].x);
    }

    #[test]
    fn member_and_retained_boundary_inputs_remain_globally_aligned() {
        let compact = Graph {
            nodes: vec![node(1), node(10), node(9)],
            edges: vec![edge(1, 10, 9, 100), edge(3, 1, 9, 300)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(9)],
            edges: vec![edge(11, 2, 9, 101), edge(12, 3, 9, 102), edge(3, 1, 9, 300)],
        };
        let compact_layout = layout_with_constraints(
            &compact,
            LayoutOptions::default(),
            &crate::LayoutConstraints {
                inputs: vec![1, 10],
                outputs: vec![9],
            },
        )
        .unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &GroupExpansion {
                anchor: 10,
                members: vec![2, 3],
                boundary_trunks: vec![
                    BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    },
                    BoundaryTrunk {
                        expanded_edge: 12,
                        compact_edge: 1,
                    },
                ],
            },
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                constraints: crate::LayoutConstraints {
                    inputs: vec![1, 2, 3],
                    outputs: vec![9],
                },
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        let geometry = result
            .nodes
            .iter()
            .map(|node| (node.id, node))
            .collect::<BTreeMap<_, _>>();

        assert_eq!(geometry[&1].x, geometry[&2].x);
        assert_eq!(geometry[&1].x, geometry[&3].x);
        assert!(geometry[&2].x + geometry[&2].width < geometry[&9].x);
    }

    #[test]
    fn globally_incompatible_boundary_constraints_require_full_relayout() {
        let compact = Graph {
            nodes: vec![node(1), node(10), node(8), node(9), node(7)],
            edges: vec![edge(1, 1, 10, 100), edge(2, 1, 8, 200), edge(3, 8, 9, 300)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(8), node(9), node(7)],
            edges: vec![
                edge(11, 1, 2, 101),
                edge(12, 1, 3, 102),
                edge(2, 1, 8, 200),
                edge(3, 8, 9, 300),
            ],
        };
        let mut compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let retained_x = compact_layout.width + 100.0;
        let retained_non_output = compact_layout
            .nodes
            .iter_mut()
            .find(|node| node.id == 7)
            .unwrap();
        retained_non_output.x = retained_x;
        compact_layout.width = retained_non_output.x + retained_non_output.width;

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &GroupExpansion {
                    anchor: 10,
                    members: vec![2, 3],
                    boundary_trunks: vec![
                        BoundaryTrunk {
                            expanded_edge: 11,
                            compact_edge: 1,
                        },
                        BoundaryTrunk {
                            expanded_edge: 12,
                            compact_edge: 1,
                        },
                    ],
                },
                &GroupExpansionOptions {
                    quality_effort: QualityEffort::Max,
                    constraints: crate::LayoutConstraints {
                        inputs: vec![1],
                        outputs: vec![2, 3, 9],
                    },
                    ..GroupExpansionOptions::default()
                },
            ),
            Err(GroupExpansionError::NeedsFullRelayout)
        );
    }

    #[test]
    fn many_to_one_trunk_replication_is_rejected_before_candidate_allocation() {
        let compact = Graph {
            nodes: vec![node(1), node(10)],
            edges: vec![edge(1, 1, 10, 100)],
        };
        let compact_layout = Layout {
            nodes: vec![
                NodeGeometry {
                    id: 1,
                    x: 0.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                },
                NodeGeometry {
                    id: 10,
                    x: 400.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                },
            ],
            edges: vec![EdgeGeometry {
                id: 1,
                points: vec![
                    Point { x: 80.0, y: 25.0 },
                    Point { x: 90.0, y: 25.0 },
                    Point { x: 90.0, y: 100.0 },
                    Point { x: 120.0, y: 100.0 },
                    Point { x: 120.0, y: 150.0 },
                    Point { x: 150.0, y: 150.0 },
                    Point { x: 150.0, y: 100.0 },
                    Point { x: 180.0, y: 100.0 },
                    Point { x: 180.0, y: 150.0 },
                    Point { x: 210.0, y: 150.0 },
                    Point { x: 210.0, y: 100.0 },
                    Point { x: 240.0, y: 100.0 },
                    Point { x: 240.0, y: 25.0 },
                    Point { x: 400.0, y: 25.0 },
                ],
            }],
            width: 480.0,
            height: 200.0,
        };
        let edges = (0..super::MAX_EXPANSION_EDGES)
            .map(|index| edge(index as u32 + 100, 1, 2, 100))
            .collect::<Vec<_>>();
        let boundary_trunks = edges
            .iter()
            .map(|edge| BoundaryTrunk {
                expanded_edge: edge.id,
                compact_edge: 1,
            })
            .collect();
        let expanded = Graph {
            nodes: vec![node(1), node(2)],
            edges,
        };

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &GroupExpansion {
                    anchor: 10,
                    members: vec![2],
                    boundary_trunks,
                },
                &GroupExpansionOptions {
                    quality_effort: QualityEffort::Max,
                    ..GroupExpansionOptions::default()
                },
            ),
            Err(GroupExpansionError::PreservedGeometryTooLarge {
                actual: 180_000,
                maximum: super::MAX_LAYOUT_SEGMENTS,
            })
        );
    }

    #[test]
    fn invalid_expansion_constraints_surface_before_layout() {
        let (compact, expanded, expansion) = fixture();
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &GroupExpansionOptions {
                    constraints: crate::LayoutConstraints {
                        inputs: vec![999],
                        outputs: Vec::new(),
                    },
                    ..GroupExpansionOptions::default()
                },
            ),
            Err(GroupExpansionError::InvalidExpandedGraph(
                crate::ConstrainedLayoutError::Constraint(
                    crate::LayoutConstraintError::UnknownConstraintNode {
                        boundary: "input",
                        node: 999,
                    }
                )
            ))
        );
    }

    #[test]
    fn compact_routes_must_leave_and_enter_ports_outward() {
        let (compact, expanded, expansion) = fixture();
        let mut compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let route = compact_layout
            .edges
            .iter_mut()
            .find(|route| route.id == 1)
            .unwrap();
        route.points[1].x = route.points[0].x - 10.0;

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &GroupExpansionOptions::default(),
            ),
            Err(GroupExpansionError::InvalidEdgeGeometry(1))
        );
    }

    #[test]
    fn hard_admission_rejects_unrelated_wire_overlap_but_allows_shared_nets() {
        let graph = |second_net| Graph {
            nodes: vec![node(1), node(2), node(3), node(4)],
            edges: vec![edge(1, 1, 2, 100), edge(2, 3, 4, second_net)],
        };
        let layout = Layout {
            nodes: vec![
                NodeGeometry {
                    id: 1,
                    x: 0.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                },
                NodeGeometry {
                    id: 2,
                    x: 400.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                },
                NodeGeometry {
                    id: 3,
                    x: 0.0,
                    y: 200.0,
                    width: 80.0,
                    height: 50.0,
                },
                NodeGeometry {
                    id: 4,
                    x: 400.0,
                    y: 200.0,
                    width: 80.0,
                    height: 50.0,
                },
            ],
            edges: vec![
                EdgeGeometry {
                    id: 1,
                    points: vec![
                        Point { x: 80.0, y: 25.0 },
                        Point { x: 100.0, y: 25.0 },
                        Point { x: 100.0, y: 120.0 },
                        Point { x: 300.0, y: 120.0 },
                        Point { x: 300.0, y: 25.0 },
                        Point { x: 400.0, y: 25.0 },
                    ],
                },
                EdgeGeometry {
                    id: 2,
                    points: vec![
                        Point { x: 80.0, y: 225.0 },
                        Point { x: 120.0, y: 225.0 },
                        Point { x: 120.0, y: 120.0 },
                        Point { x: 320.0, y: 120.0 },
                        Point { x: 320.0, y: 225.0 },
                        Point { x: 400.0, y: 225.0 },
                    ],
                },
            ],
            width: 480.0,
            height: 250.0,
        };

        assert!(!super::hard_geometry_is_clean(&graph(200), &layout));
        assert!(super::hard_geometry_is_clean(&graph(100), &layout));

        let mut out_of_bounds = layout.clone();
        out_of_bounds.edges[0].points[2].y = -10.0;
        out_of_bounds.edges[0].points[3].y = -10.0;
        assert!(!super::hard_geometry_is_clean(&graph(100), &out_of_bounds));
    }

    #[test]
    fn bridge_search_routes_around_blocking_nodes() {
        let obstacle = super::Rect::from_node(&NodeGeometry {
            id: 1,
            x: 40.0,
            y: 30.0,
            width: 40.0,
            height: 40.0,
        });
        let obstacles = super::ObstacleIndex::new(vec![obstacle]);
        let path = obstacle_safe_bridge(
            Point { x: 20.0, y: 50.0 },
            Point { x: 100.0, y: 50.0 },
            &obstacles,
            10.0,
        )
        .unwrap();
        assert!(path.len() >= 4);
        assert!(path_is_clear(&path, &obstacles));
    }

    #[test]
    fn disconnected_members_pack_into_a_balanced_deterministic_grid() {
        let graph = Graph {
            nodes: (1..=64).map(node).collect(),
            edges: Vec::new(),
        };
        let ordinary = layout(&graph, LayoutOptions::default()).unwrap();
        let packed = pack_disconnected_components(
            &graph,
            &ordinary,
            LayoutOptions::default().node_gap,
            None,
        );
        assert!(packed.width < ordinary.width * 10.0);
        assert!(packed.height < ordinary.height / 4.0);
        assert!(packed.width / packed.height < 2.0);
        assert!(packed.height / packed.width < 2.0);
        for (index, left) in packed.nodes.iter().enumerate() {
            for right in packed.nodes.iter().skip(index + 1) {
                assert_eq!(
                    Rect::from_node(left).overlap_area(Rect::from_node(right)),
                    0.0
                );
            }
        }

        let mut permuted = graph.clone();
        permuted.nodes.reverse();
        let permuted_layout = layout(&permuted, LayoutOptions::default()).unwrap();
        assert_eq!(
            pack_disconnected_components(
                &permuted,
                &permuted_layout,
                LayoutOptions::default().node_gap,
                None,
            ),
            packed
        );
    }

    #[test]
    fn expansion_candidate_search_has_effort_and_total_work_bounds() {
        let compact = Layout {
            nodes: Vec::new(),
            edges: Vec::new(),
            width: 10_000.0,
            height: 8_000.0,
        };
        let anchor = NodeGeometry {
            id: 1,
            x: 4_000.0,
            y: 3_000.0,
            width: 80.0,
            height: 50.0,
        };
        let members = Layout {
            nodes: Vec::new(),
            edges: Vec::new(),
            width: 300.0,
            height: 200.0,
        };
        let options = LayoutOptions::default();
        let small = ExpansionWork {
            nodes: 100,
            edges: 200,
            boundary_edges: 20,
            projected_segments: 1_600,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                small,
                options,
                QualityEffort::Fast,
            )
            .unwrap()
            .len(),
            3
        );
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                small,
                options,
                QualityEffort::Quality,
            )
            .unwrap()
            .len(),
            11
        );
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                small,
                options,
                QualityEffort::Max,
            )
            .unwrap()
            .len(),
            51
        );
        let maximum = ExpansionWork {
            nodes: 4_098,
            edges: 8_192,
            boundary_edges: 8_192,
            projected_segments: 65_536,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                maximum,
                options,
                QualityEffort::Max,
            )
            .unwrap()
            .len(),
            3
        );
        let over_budget = ExpansionWork {
            nodes: 4_098,
            edges: 10_000,
            boundary_edges: 10_000,
            projected_segments: 80_000,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                over_budget,
                options,
                QualityEffort::Max,
            ),
            Err(GroupExpansionError::ExpansionWorkLimitExceeded {
                required: 123_222_294,
                maximum: super::MAX_CANDIDATE_WORK,
            })
        );
    }

    #[test]
    fn obstacle_index_matches_direct_rectangle_queries() {
        let rects = vec![
            Rect {
                left: 10.0,
                top: 10.0,
                right: 30.0,
                bottom: 30.0,
            },
            Rect {
                left: 50.0,
                top: 20.0,
                right: 80.0,
                bottom: 60.0,
            },
            Rect {
                left: 25.0,
                top: 70.0,
                right: 45.0,
                bottom: 90.0,
            },
        ];
        let index = ObstacleIndex::new(rects.clone());
        for fixed in [0.0, 10.0, 20.0, 29.0, 30.0, 50.0, 75.0, 100.0] {
            for (start, end) in [(0.0, 100.0), (15.0, 55.0), (31.0, 49.0)] {
                let horizontal = (Point { x: start, y: fixed }, Point { x: end, y: fixed });
                let vertical = (Point { x: fixed, y: start }, Point { x: fixed, y: end });
                assert_eq!(
                    index.segment_crosses_interior(horizontal.0, horizontal.1),
                    rects
                        .iter()
                        .any(|rect| rect.segment_crosses_interior(horizontal.0, horizontal.1))
                );
                assert_eq!(
                    index.segment_crosses_interior(vertical.0, vertical.1),
                    rects
                        .iter()
                        .any(|rect| rect.segment_crosses_interior(vertical.0, vertical.1))
                );
            }
        }
    }
}
