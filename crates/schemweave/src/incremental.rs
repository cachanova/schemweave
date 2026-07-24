use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    BoundaryBundleConstraint, BoundaryBundleGeometry, BoundaryBundleId,
    BoundaryBundleMemberConstraint, BoundaryBundleMemberGeometry, BoundaryBundleRole,
    BoundaryBundleSegment, ConstrainedLayoutError, Edge, EdgeGeometry, EdgeId, Endpoint, Graph,
    Layout, LayoutConstraints, LayoutError, LayoutOptions, NetId, Node, NodeGeometry, NodeId,
    Point, Port, PortSide, QualityEffort, boundary_bundles,
    layout_with_quality_effort_and_constraints, routing, validation,
};

const MAX_EXPANSION_MEMBERS: usize = 4_096;
const MAX_EXPANSION_EDGES: usize = 10_000;
const MAX_LAYOUT_SEGMENTS: usize = 100_000;
const HARD_GATE_EPSILON: f64 = 1e-7;
const FAST_CANDIDATE_WORK: usize = 10_000_000;
const QUALITY_CANDIDATE_WORK: usize = 30_000_000;
const MAX_CANDIDATE_WORK: usize = 120_000_000;
const SAFETY_CANDIDATES: usize = 2;
const LOCAL_REFLOW_CANDIDATES: usize = 2;
const EXPANSION_COMPONENT_GAP: f64 = 18.0;
const EXPANSION_STACK_HEIGHT_FACTOR: f64 = 1.5;

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

/// Layout policy for replacing expanded members with their compact anchor.
///
/// Collapse is stability-first: unrelated geometry remains at its current
/// coordinates and the vacated member frame is intentionally left available.
/// A later explicit compaction can close that space without coupling a
/// responsive visibility toggle to unrelated movement.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct GroupCollapseOptions {
    #[serde(flatten)]
    pub layout: LayoutOptions,
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
    #[error("group expansion reference height must be finite and positive, got {0}")]
    InvalidReferenceHeight(f64),
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
    compact_boundary_bundle_offsets: BTreeMap<BoundaryBundleId, f64>,
    boundary_trunks: BTreeMap<EdgeId, EdgeId>,
}

struct GraphExpansionContract<'a> {
    members: BTreeSet<NodeId>,
    expanded_nodes: BTreeMap<NodeId, &'a Node>,
    boundary_trunks: BTreeMap<EdgeId, EdgeId>,
}

#[derive(Clone, Copy, Debug, Default)]
struct BundledRouteEndpoints {
    source: Option<Point>,
    target: Option<Point>,
}

struct RemappedBoundaryBundles {
    geometry: Vec<BoundaryBundleGeometry>,
    retaps: BTreeMap<EdgeId, BoundaryBundleRetap>,
}

#[derive(Clone, Copy)]
struct BoundaryBundleRetap {
    role: BoundaryBundleRole,
    tap: Point,
    tap_lane: usize,
    corridor_offset: f64,
}

#[derive(Clone, Copy)]
struct ExpansionWork {
    nodes: usize,
    edges: usize,
    boundary_edges: usize,
    projected_segments: usize,
    boundary_bundles: usize,
    boundary_bundle_members: usize,
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

/// Expand one quotient node while preserving distant retained geometry.
///
/// The compact and expanded graphs must use stable identifiers. Every node and
/// edge unrelated to the anchor must remain semantically equivalent. Boundary
/// member edges reuse explicitly mapped compact trunks, while member placement
/// and internal routing use the canonical SchemWeave engine. Wider groups open
/// a horizontal corridor. Taller groups may move the connected obstructing
/// vertical slab and deform only routes crossing that slab; the non-reflow
/// candidate remains available as a fallback. A result is returned
/// only when every hard geometry and left-to-right invariant survives; callers
/// must perform a full layout after `NeedsFullRelayout`.
pub fn expand_group_in_place(
    compact_graph: &Graph,
    compact_layout: &Layout,
    expanded_graph: &Graph,
    expansion: &GroupExpansion,
    options: &GroupExpansionOptions,
) -> Result<Layout, GroupExpansionError> {
    expand_group_in_place_with_reference_height(
        compact_graph,
        compact_layout,
        expanded_graph,
        expansion,
        compact_layout.height,
        options,
    )
}

/// Collapse one expanded group while preserving every unrelated geometry.
///
/// The graph pair and [`GroupExpansion`] use the same stable-ID contract as
/// [`expand_group_in_place`], with their direction reversed. Member nodes and
/// internal routes are removed, the compact anchor is restored at the member
/// frame origin, and each compact boundary trunk is reverse-spliced onto one
/// mapped expanded route. The result is admitted only after the complete hard
/// geometry, constraint, bundle, spacing, and left-to-right gates pass.
pub fn collapse_group_in_place(
    expanded_graph: &Graph,
    expanded_layout: &Layout,
    compact_graph: &Graph,
    expansion: &GroupExpansion,
    options: &GroupCollapseOptions,
) -> Result<Layout, GroupExpansionError> {
    validation::validate_and_index(expanded_graph, options.layout)
        .map_err(GroupExpansionError::InvalidCompactGraph)?;
    let compact_indexed = validation::validate_and_index_with_constraints(
        compact_graph,
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
    let contract = validate_graph_contract(compact_graph, expanded_graph, expansion)?;
    validate_compact_boundary_bundles(expanded_graph, expanded_layout, options.layout)?;
    let expanded_node_geometry = index_node_geometry(expanded_graph, expanded_layout)?;
    let expanded_edge_geometry = index_edge_geometry(expanded_graph, expanded_layout)?;
    validate_layout_bounds(expanded_layout)?;
    if route_segment_count(expanded_layout) > MAX_LAYOUT_SEGMENTS {
        return Err(GroupExpansionError::TooManyCompactRouteSegments {
            actual: route_segment_count(expanded_layout),
            maximum: MAX_LAYOUT_SEGMENTS,
        });
    }
    let hard_budget_maximum = MAX_CANDIDATE_WORK;
    let mut hard_budget = WorkBudget::new(hard_budget_maximum);
    match hard_geometry_is_clean_bounded(expanded_graph, expanded_layout, &mut hard_budget) {
        Ok(true) => {}
        Ok(false) => return Err(GroupExpansionError::InvalidCompactHardGeometry),
        Err(required) => {
            return Err(GroupExpansionError::ExpansionWorkLimitExceeded {
                required,
                maximum: hard_budget_maximum,
            });
        }
    }

    let candidate = compose_collapse_candidate(
        expanded_graph,
        expanded_layout,
        compact_graph,
        expansion,
        &contract,
        &expanded_node_geometry,
        &expanded_edge_geometry,
        options.layout,
    )?;
    let candidate =
        boundary_bundles::apply_and_normalize(&compact_indexed, candidate, options.layout)
            .map_err(|_| GroupExpansionError::NeedsFullRelayout)?;
    if route_segment_count(&candidate) > MAX_LAYOUT_SEGMENTS {
        return Err(GroupExpansionError::PreservedGeometryTooLarge {
            actual: route_segment_count(&candidate),
            maximum: MAX_LAYOUT_SEGMENTS,
        });
    }
    hard_budget
        .take(compact_graph.edges.len())
        .map_err(|required| GroupExpansionError::ExpansionWorkLimitExceeded {
            required,
            maximum: hard_budget_maximum,
        })?;
    let geometry_is_clean =
        hard_geometry_is_clean_bounded(compact_graph, &candidate, &mut hard_budget).map_err(
            |required| GroupExpansionError::ExpansionWorkLimitExceeded {
                required,
                maximum: hard_budget_maximum,
            },
        )?;
    charge_expansion_work(
        &mut hard_budget,
        clearance_work_upper_bound(&candidate, options.layout),
        hard_budget_maximum,
    )?;
    let mut clearance_work_exhausted = false;
    let clearance_is_clean = crate::candidate_satisfies_edge_node_clearance_bounded(
        &compact_indexed,
        &candidate,
        options.layout,
        crate::MAX_LAYOUT_CLEARANCE_PAIR_VISITS,
        &mut clearance_work_exhausted,
    );
    charge_expansion_work(
        &mut hard_budget,
        parallel_spacing_work_upper_bound(&candidate, options.layout),
        hard_budget_maximum,
    )?;
    let parallel_spacing_is_clean = if options.layout.minimum_parallel_wire_spacing > 0.0 {
        matches!(
            routing::route_family_satisfies_parallel_spacing_bounded(
                &compact_indexed,
                &candidate.edges,
                options.layout.minimum_parallel_wire_spacing,
                crate::outward_obstacle_clearance_stub(options.layout),
                crate::MAX_LAYOUT_PARALLEL_WIRE_SPACING_SEGMENTS,
                crate::MAX_LAYOUT_PARALLEL_WIRE_SPACING_VISITS,
            ),
            Ok(true)
        )
    } else {
        true
    };
    charge_expansion_work(
        &mut hard_budget,
        boundary_bundle_work_upper_bound(&compact_indexed, &candidate),
        hard_budget_maximum,
    )?;
    let bundles_are_clean =
        boundary_bundles::verify_geometry(&compact_indexed, &candidate, options.layout).is_ok();
    if ranking_direction_violations(compact_graph, &candidate) != 0
        || !constraints_are_satisfied(&candidate, &options.constraints)
        || !geometry_is_clean
        || !clearance_is_clean
        || clearance_work_exhausted
        || !parallel_spacing_is_clean
        || !bundles_are_clean
    {
        return Err(GroupExpansionError::NeedsFullRelayout);
    }
    let collapsed_nodes = candidate
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    if compact_graph
        .nodes
        .iter()
        .filter(|node| node.id != expansion.anchor)
        .any(|node| {
            collapsed_nodes.get(&node.id).copied() != expanded_node_geometry.get(&node.id).copied()
        })
    {
        return Err(GroupExpansionError::NeedsFullRelayout);
    }
    let collapsed_edges = candidate
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    if compact_graph
        .edges
        .iter()
        .filter(|edge| edge.source.node != expansion.anchor && edge.target.node != expansion.anchor)
        .any(|edge| {
            collapsed_edges.get(&edge.id).copied() != expanded_edge_geometry.get(&edge.id).copied()
        })
    {
        return Err(GroupExpansionError::NeedsFullRelayout);
    }
    Ok(candidate)
}

/// Expand one quotient node using a stable schematic height to choose the
/// expanded member arrangement.
///
/// This variant preserves the vertical-stack versus grid decision when the
/// compact layout is a focused projection of a larger schematic. The reference
/// height should be the height of the layout in which the group was originally
/// expanded.
pub fn expand_group_in_place_with_reference_height(
    compact_graph: &Graph,
    compact_layout: &Layout,
    expanded_graph: &Graph,
    expansion: &GroupExpansion,
    reference_height: f64,
    options: &GroupExpansionOptions,
) -> Result<Layout, GroupExpansionError> {
    validation::validate_and_index(compact_graph, options.layout)
        .map_err(GroupExpansionError::InvalidCompactGraph)?;
    if !reference_height.is_finite() || reference_height <= 0.0 {
        return Err(GroupExpansionError::InvalidReferenceHeight(
            reference_height,
        ));
    }
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
    let contract = validate_contract(
        compact_graph,
        compact_layout,
        expanded_graph,
        expansion,
        options.layout,
    )?;

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
        boundary_bundles: Vec::new(),
    };
    let member_layout = layout_with_quality_effort_and_constraints(
        &member_graph,
        options.layout,
        options.quality_effort,
        &member_constraints,
    )
    .map_err(GroupExpansionError::MemberLayout)?;
    let mut member_layout = if member_constraints.inputs.is_empty()
        && member_constraints.outputs.is_empty()
    {
        let bridge_pitch = options
            .layout
            .route_lane_gap
            .max(options.layout.minimum_parallel_wire_spacing);
        let shared_boundary_lanes =
            shared_boundary_group_count(expanded_graph, &contract, &options.constraints);
        let component_gap = EXPANSION_COMPONENT_GAP.max(
            crate::outward_obstacle_clearance_stub(options.layout) * 2.0
                + bridge_pitch * shared_boundary_lanes.max(1) as f64,
        );
        let minimum_horizontal_gap = EXPANSION_COMPONENT_GAP
            .max(crate::outward_obstacle_clearance_stub(options.layout) * 2.0 + bridge_pitch);
        let mut horizontal_gap = minimum_horizontal_gap;
        let mut arranged = arrange_member_components(
            &member_graph,
            &member_layout,
            component_gap,
            horizontal_gap,
            reference_height,
            None,
        );
        // Changing the gap can change the aspect-ratio-derived column
        // count. Recompute a bounded number of times so every adjacent
        // column has disjoint outgoing and incoming boundary-lane banks.
        for _ in 0..2 {
            let required =
                expansion_grid_horizontal_gap(expanded_graph, &contract, &arranged, options.layout)
                    .max(minimum_horizontal_gap);
            if required <= horizontal_gap + HARD_GATE_EPSILON {
                break;
            }
            horizontal_gap = required;
            arranged = arrange_member_components(
                &member_graph,
                &member_layout,
                component_gap,
                horizontal_gap,
                reference_height,
                None,
            );
        }
        arranged
    } else {
        member_layout
    };
    let input_padding = expansion_input_corridor_padding(
        compact_layout,
        expanded_graph,
        &expanded_indexed,
        &contract,
        &member_layout,
        options.layout,
    );
    if input_padding > 0.0 {
        member_layout = add_horizontal_member_padding(member_layout, input_padding, 0.0);
    }
    let corridor_layout =
        insert_horizontal_expansion_corridor(compact_layout, expansion.anchor, member_layout.width);
    let horizontal_layout = corridor_layout.as_ref().unwrap_or(compact_layout);
    let vertical_layout = insert_local_vertical_expansion_corridor(
        compact_graph,
        horizontal_layout,
        expansion.anchor,
        member_layout.width,
        member_layout.height,
        options.layout,
    );
    let contract = validate_contract(
        compact_graph,
        horizontal_layout,
        expanded_graph,
        expansion,
        options.layout,
    )?;
    let boundary_bundles = remap_boundary_bundles(
        horizontal_layout,
        expanded_graph,
        &expanded_indexed,
        &contract,
        options.layout,
    )?;
    let boundary_edges = expanded_graph
        .edges
        .iter()
        .filter(|edge| {
            contract.members.contains(&edge.source.node)
                ^ contract.members.contains(&edge.target.node)
        })
        .count();
    let baseline_projected_segments =
        projected_segment_count(expanded_graph, &contract, &member_layout);
    if baseline_projected_segments > MAX_LAYOUT_SEGMENTS {
        return Err(GroupExpansionError::PreservedGeometryTooLarge {
            actual: baseline_projected_segments,
            maximum: MAX_LAYOUT_SEGMENTS,
        });
    }
    let mut projected_segments = baseline_projected_segments;
    let vertical_contract = vertical_layout.as_ref().and_then(|vertical_layout| {
        validate_contract(
            compact_graph,
            vertical_layout,
            expanded_graph,
            expansion,
            options.layout,
        )
        .ok()
    });
    let vertical_bundles = vertical_layout
        .as_ref()
        .zip(vertical_contract.as_ref())
        .and_then(|(vertical_layout, vertical_contract)| {
            remap_boundary_bundles(
                vertical_layout,
                expanded_graph,
                &expanded_indexed,
                vertical_contract,
                options.layout,
            )
            .ok()
        });
    let mut vertical_layout_is_usable = vertical_contract
        .as_ref()
        .zip(vertical_bundles.as_ref())
        .is_some_and(|(vertical_contract, _)| {
            let vertical_segments =
                projected_segment_count(expanded_graph, vertical_contract, &member_layout);
            if vertical_segments <= MAX_LAYOUT_SEGMENTS {
                projected_segments = projected_segments.max(vertical_segments);
                true
            } else {
                false
            }
        });
    let mut reserved_candidates = if vertical_layout_is_usable {
        LOCAL_REFLOW_CANDIDATES
    } else {
        0
    };
    let expansion_work = |projected_segments| ExpansionWork {
        nodes: expanded_graph.nodes.len(),
        edges: expanded_graph.edges.len(),
        boundary_edges,
        projected_segments,
        boundary_bundles: expanded_indexed.boundary_bundles.len(),
        boundary_bundle_members: expanded_indexed
            .boundary_bundles
            .iter()
            .map(|bundle| bundle.members.len())
            .sum(),
    };
    let mut positions = match candidate_positions(
        horizontal_layout,
        contract.anchor_geometry,
        &member_layout,
        expansion_work(projected_segments),
        options.layout,
        options.quality_effort,
        reserved_candidates,
    ) {
        Ok(positions) => positions,
        Err(_) if vertical_layout_is_usable => {
            vertical_layout_is_usable = false;
            reserved_candidates = 0;
            candidate_positions(
                horizontal_layout,
                contract.anchor_geometry,
                &member_layout,
                expansion_work(baseline_projected_segments),
                options.layout,
                options.quality_effort,
                0,
            )?
        }
        Err(error) => return Err(error),
    };
    prioritize_constraint_positions(
        &mut positions,
        constraint_x_candidates(
            compact_layout,
            &member_layout,
            &contract.members,
            &options.constraints,
        ),
    );

    let anchor_origin = Point {
        x: contract.anchor_geometry.x,
        y: contract.anchor_geometry.y,
    };
    let mut best: Option<(CandidateScore, Layout)> = None;
    let hard_budget_maximum = candidate_work_budget(options.quality_effort);
    let mut hard_budget = WorkBudget::new(hard_budget_maximum);
    let mut candidate_attempts_remaining = positions.len() + reserved_candidates;

    if vertical_layout_is_usable {
        let vertical_layout = vertical_layout
            .as_ref()
            .expect("usable vertical layout was constructed");
        let vertical_contract = vertical_contract
            .as_ref()
            .expect("usable vertical layout has a validated contract");
        let vertical_bundles = vertical_bundles
            .as_ref()
            .expect("usable vertical layout has remapped boundary bundles");
        let frame = Rect {
            left: anchor_origin.x,
            top: anchor_origin.y,
            right: anchor_origin.x + member_layout.width,
            bottom: anchor_origin.y + member_layout.height,
        };
        if retained_node_overlap_area(vertical_layout, expansion.anchor, frame, 0.0) == 0.0 {
            for prefer_direct_boundary_routes in [true, false] {
                if candidate_attempts_remaining == 0 {
                    break;
                }
                candidate_attempts_remaining -= 1;
                if let Some(candidate) = evaluate_expansion_candidate(
                    vertical_layout,
                    expanded_graph,
                    &expanded_indexed,
                    vertical_contract,
                    &member_layout,
                    vertical_bundles,
                    anchor_origin.x,
                    anchor_origin.y,
                    options,
                    prefer_direct_boundary_routes,
                    anchor_origin,
                    &mut hard_budget,
                    hard_budget_maximum,
                )? {
                    if best
                        .as_ref()
                        .is_none_or(|(current, _)| candidate.0.cmp(*current).is_lt())
                    {
                        best = Some(candidate);
                    }
                    if prefer_direct_boundary_routes {
                        break;
                    }
                }
            }
        }
    }

    for (x, y) in positions {
        let frame = Rect {
            left: x,
            top: y,
            right: x + member_layout.width,
            bottom: y + member_layout.height,
        };
        if retained_node_overlap_area(horizontal_layout, expansion.anchor, frame, 0.0) > 0.0 {
            continue;
        }
        for prefer_direct_boundary_routes in [true, false] {
            if candidate_attempts_remaining == 0 {
                break;
            }
            candidate_attempts_remaining -= 1;
            let Some(candidate) = evaluate_expansion_candidate(
                horizontal_layout,
                expanded_graph,
                &expanded_indexed,
                &contract,
                &member_layout,
                &boundary_bundles,
                x,
                y,
                options,
                prefer_direct_boundary_routes,
                anchor_origin,
                &mut hard_budget,
                hard_budget_maximum,
            )?
            else {
                continue;
            };
            if best
                .as_ref()
                .is_none_or(|(current, _)| candidate.0.cmp(*current).is_lt())
            {
                best = Some(candidate);
            }
            if prefer_direct_boundary_routes {
                break;
            }
        }
        if candidate_attempts_remaining == 0 {
            break;
        }
    }
    best.map(|(_, layout)| layout)
        .ok_or(GroupExpansionError::NeedsFullRelayout)
}

#[allow(clippy::too_many_arguments)]
fn evaluate_expansion_candidate(
    working_layout: &Layout,
    expanded_graph: &Graph,
    expanded_indexed: &validation::IndexedGraph<'_>,
    contract: &ExpansionContract<'_>,
    member_layout: &Layout,
    remapped_bundles: &RemappedBoundaryBundles,
    x: f64,
    y: f64,
    options: &GroupExpansionOptions,
    prefer_direct_boundary_routes: bool,
    anchor_origin: Point,
    hard_budget: &mut WorkBudget,
    hard_budget_maximum: usize,
) -> Result<Option<(CandidateScore, Layout)>, GroupExpansionError> {
    let candidate = match compose_candidate(
        working_layout,
        expanded_graph,
        contract,
        member_layout,
        remapped_bundles,
        x,
        y,
        options.layout,
        prefer_direct_boundary_routes,
    ) {
        Ok(candidate) => candidate,
        Err(GroupExpansionError::NoSafeBoundaryBridge(_)) => return Ok(None),
        Err(error) => return Err(error),
    };
    hard_budget
        .take(expanded_graph.edges.len())
        .map_err(|required| GroupExpansionError::ExpansionWorkLimitExceeded {
            required,
            maximum: hard_budget_maximum,
        })?;
    let geometry_is_clean = hard_geometry_is_clean_bounded(expanded_graph, &candidate, hard_budget)
        .map_err(|required| GroupExpansionError::ExpansionWorkLimitExceeded {
            required,
            maximum: hard_budget_maximum,
        })?;
    charge_expansion_work(
        hard_budget,
        clearance_work_upper_bound(&candidate, options.layout),
        hard_budget_maximum,
    )?;
    let mut clearance_work_exhausted = false;
    let clearance_is_clean = crate::candidate_satisfies_edge_node_clearance_bounded(
        expanded_indexed,
        &candidate,
        options.layout,
        crate::MAX_LAYOUT_CLEARANCE_PAIR_VISITS,
        &mut clearance_work_exhausted,
    );
    charge_expansion_work(
        hard_budget,
        parallel_spacing_work_upper_bound(&candidate, options.layout),
        hard_budget_maximum,
    )?;
    let parallel_spacing_is_clean = if options.layout.minimum_parallel_wire_spacing > 0.0 {
        matches!(
            routing::route_family_satisfies_parallel_spacing_bounded(
                expanded_indexed,
                &candidate.edges,
                options.layout.minimum_parallel_wire_spacing,
                crate::outward_obstacle_clearance_stub(options.layout),
                crate::MAX_LAYOUT_PARALLEL_WIRE_SPACING_SEGMENTS,
                crate::MAX_LAYOUT_PARALLEL_WIRE_SPACING_VISITS,
            ),
            Ok(true)
        )
    } else {
        true
    };
    charge_expansion_work(
        hard_budget,
        boundary_bundle_work_upper_bound(expanded_indexed, &candidate),
        hard_budget_maximum,
    )?;
    let boundary_bundle_geometry_is_clean =
        boundary_bundles::verify_geometry(expanded_indexed, &candidate, options.layout).is_ok();
    if ranking_direction_violations(expanded_graph, &candidate) != 0
        || !constraints_are_satisfied(&candidate, &options.constraints)
        || !geometry_is_clean
        || !clearance_is_clean
        || clearance_work_exhausted
        || !parallel_spacing_is_clean
        || !boundary_bundle_geometry_is_clean
    {
        return Ok(None);
    }
    let score = CandidateScore {
        quality: routing::route_quality(expanded_indexed, &candidate.edges),
        displacement: squared_distance(Point { x, y }, anchor_origin),
        area: candidate.width * candidate.height,
        x,
        y,
    };
    Ok(Some((score, candidate)))
}

fn charge_expansion_work(
    budget: &mut WorkBudget,
    amount: usize,
    maximum: usize,
) -> Result<(), GroupExpansionError> {
    budget
        .take(amount)
        .map_err(|required| GroupExpansionError::ExpansionWorkLimitExceeded { required, maximum })
}

fn route_segment_count(layout: &Layout) -> usize {
    layout.edges.iter().fold(0usize, |total, route| {
        total.saturating_add(route.points.len().saturating_sub(1))
    })
}

fn clearance_work_upper_bound(layout: &Layout, options: LayoutOptions) -> usize {
    if options.edge_node_clearance == 0.0 {
        return 0;
    }
    let segments = route_segment_count(layout);
    segments
        .saturating_mul(layout.nodes.len())
        .saturating_add(segments)
        .saturating_add(layout.nodes.len())
}

fn parallel_spacing_work_upper_bound(layout: &Layout, options: LayoutOptions) -> usize {
    if options.minimum_parallel_wire_spacing == 0.0 {
        return 0;
    }
    let segments = route_segment_count(layout);
    segments
        .saturating_mul(segments)
        .saturating_mul(8)
        .saturating_add(segments.saturating_mul(8))
}

fn boundary_bundle_work_upper_bound(
    indexed: &validation::IndexedGraph<'_>,
    layout: &Layout,
) -> usize {
    if indexed.boundary_bundles.is_empty() {
        return 0;
    }
    let all_segments = route_segment_count(layout);
    let bundle_segments = indexed.boundary_bundles.len().saturating_mul(2);
    let member_edges = indexed
        .boundary_bundles
        .iter()
        .flat_map(|bundle| bundle.members.iter().map(|member| member.edge))
        .collect::<BTreeSet<_>>();
    let member_segments = layout
        .edges
        .iter()
        .filter(|route| member_edges.contains(&route.id))
        .fold(0usize, |total, route| {
            total.saturating_add(route.points.len().saturating_sub(1))
        });
    let structure = layout
        .nodes
        .len()
        .saturating_add(layout.edges.len())
        .saturating_add(indexed.boundary_bundles.len())
        .saturating_add(
            indexed
                .boundary_bundles
                .iter()
                .map(|bundle| bundle.members.len())
                .sum(),
        );
    structure
        .saturating_add(bundle_segments.saturating_mul(layout.nodes.len()))
        .saturating_add(member_segments.saturating_mul(layout.nodes.len()))
        .saturating_add(bundle_segments.saturating_mul(all_segments))
        .saturating_add(bundle_segments.saturating_mul(bundle_segments))
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
    options: LayoutOptions,
) -> Result<ExpansionContract<'a>, GroupExpansionError> {
    let GraphExpansionContract {
        members,
        expanded_nodes,
        boundary_trunks,
    } = validate_graph_contract(compact_graph, expanded_graph, expansion)?;
    let compact_boundary_bundle_offsets =
        validate_compact_boundary_bundles(compact_graph, compact_layout, options)?;
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

    Ok(ExpansionContract {
        anchor_geometry,
        members,
        expanded_nodes,
        compact_node_geometry,
        compact_edge_geometry,
        compact_boundary_bundle_offsets,
        boundary_trunks,
    })
}

fn validate_graph_contract<'a>(
    compact_graph: &'a Graph,
    expanded_graph: &'a Graph,
    expansion: &GroupExpansion,
) -> Result<GraphExpansionContract<'a>, GroupExpansionError> {
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

    Ok(GraphExpansionContract {
        members,
        expanded_nodes,
        boundary_trunks,
    })
}

fn validate_compact_boundary_bundles(
    compact_graph: &Graph,
    compact_layout: &Layout,
    options: LayoutOptions,
) -> Result<BTreeMap<BoundaryBundleId, f64>, GroupExpansionError> {
    if compact_layout.boundary_bundles.is_empty() {
        return Ok(BTreeMap::new());
    }
    let mut inputs = BTreeSet::new();
    let mut outputs = BTreeSet::new();
    let boundary_bundles = compact_layout
        .boundary_bundles
        .iter()
        .map(|geometry| {
            match geometry.role {
                BoundaryBundleRole::Input => {
                    inputs.insert(geometry.endpoint.node);
                }
                BoundaryBundleRole::Output => {
                    outputs.insert(geometry.endpoint.node);
                }
            }
            BoundaryBundleConstraint {
                id: geometry.id,
                endpoint: geometry.endpoint,
                width: geometry.width,
                members: geometry
                    .members
                    .iter()
                    .map(|member| BoundaryBundleMemberConstraint {
                        edge: member.edge,
                        slots: member.slots.clone(),
                    })
                    .collect(),
            }
        })
        .collect();
    let constraints = LayoutConstraints {
        inputs: inputs.into_iter().collect(),
        outputs: outputs.into_iter().collect(),
        boundary_bundles,
    };
    let indexed =
        validation::validate_and_index_with_constraints(compact_graph, options, &constraints)
            .map_err(|_| GroupExpansionError::NeedsFullRelayout)?;
    boundary_bundles::verify_preserved_geometry_structure(&indexed, compact_layout, options)
        .map_err(|_| GroupExpansionError::NeedsFullRelayout)?;
    Ok(indexed
        .boundary_bundles
        .iter()
        .zip(boundary_bundles::corridor_depth_offsets(&indexed, options))
        .map(|(bundle, offset)| (bundle.id, offset))
        .collect())
}

fn remap_boundary_bundles(
    compact_layout: &Layout,
    expanded_graph: &Graph,
    expanded_indexed: &validation::IndexedGraph<'_>,
    contract: &ExpansionContract<'_>,
    options: LayoutOptions,
) -> Result<RemappedBoundaryBundles, GroupExpansionError> {
    if compact_layout.boundary_bundles.is_empty() && expanded_indexed.boundary_bundles.is_empty() {
        return Ok(RemappedBoundaryBundles {
            geometry: Vec::new(),
            retaps: BTreeMap::new(),
        });
    }
    if compact_layout.boundary_bundles.len() != expanded_indexed.boundary_bundles.len() {
        return Err(GroupExpansionError::NeedsFullRelayout);
    }

    let mut expanded_bundles = BTreeMap::new();
    for bundle in &expanded_indexed.boundary_bundles {
        if expanded_bundles.insert(bundle.id, bundle).is_some()
            || contract.members.contains(&bundle.endpoint.node)
        {
            return Err(GroupExpansionError::NeedsFullRelayout);
        }
    }
    let expanded_corridor_offsets = expanded_indexed
        .boundary_bundles
        .iter()
        .zip(boundary_bundles::corridor_depth_offsets(
            expanded_indexed,
            options,
        ))
        .map(|(bundle, offset)| (bundle.id, offset))
        .collect::<BTreeMap<_, _>>();
    let expanded_edges = expanded_graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let mut seen_compact_bundles = BTreeSet::new();
    let mut remapped = Vec::with_capacity(compact_layout.boundary_bundles.len());
    let mut retaps = BTreeMap::new();
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    for compact_bundle in &compact_layout.boundary_bundles {
        if !seen_compact_bundles.insert(compact_bundle.id) {
            return Err(GroupExpansionError::NeedsFullRelayout);
        }
        let Some(expanded_bundle) = expanded_bundles.get(&compact_bundle.id).copied() else {
            return Err(GroupExpansionError::NeedsFullRelayout);
        };
        if compact_bundle.endpoint != expanded_bundle.endpoint
            || compact_bundle.role != expanded_bundle.role
            || compact_bundle.width != expanded_bundle.width
        {
            return Err(GroupExpansionError::NeedsFullRelayout);
        }
        let Some(&compact_corridor_offset) = contract
            .compact_boundary_bundle_offsets
            .get(&compact_bundle.id)
        else {
            return Err(GroupExpansionError::NeedsFullRelayout);
        };
        let Some(&expanded_corridor_offset) = expanded_corridor_offsets.get(&expanded_bundle.id)
        else {
            return Err(GroupExpansionError::NeedsFullRelayout);
        };

        let mut compact_members = BTreeMap::new();
        for member in &compact_bundle.members {
            if compact_members.insert(member.edge, member).is_some() {
                return Err(GroupExpansionError::NeedsFullRelayout);
            }
        }
        let mut used_compact_members = BTreeSet::new();
        let mut mapped_members = Vec::with_capacity(expanded_bundle.members.len());
        let mut expanded_slots_by_compact_edge = BTreeMap::<EdgeId, BTreeSet<u32>>::new();
        let mut affected = false;
        for expanded_member in &expanded_bundle.members {
            let edge = expanded_edges[&expanded_member.edge];
            let source_member = contract.members.contains(&edge.source.node);
            let target_member = contract.members.contains(&edge.target.node);
            let compact_edge = match (source_member, target_member) {
                (false, false) => expanded_member.edge,
                (true, false) | (false, true) => contract.boundary_trunks[&expanded_member.edge],
                (true, true) => return Err(GroupExpansionError::NeedsFullRelayout),
            };
            let Some(compact_member) = compact_members.get(&compact_edge).copied() else {
                return Err(GroupExpansionError::NeedsFullRelayout);
            };
            let union = expanded_slots_by_compact_edge
                .entry(compact_edge)
                .or_default();
            for &slot in &expanded_member.slots {
                union.insert(slot);
            }
            used_compact_members.insert(compact_edge);
            affected |= compact_edge != expanded_member.edge;
            mapped_members.push((
                expanded_member,
                compact_edge,
                compact_member,
                source_member ^ target_member,
            ));
        }
        if used_compact_members.len() != compact_members.len() {
            return Err(GroupExpansionError::NeedsFullRelayout);
        }
        for (&compact_edge, slots) in &expanded_slots_by_compact_edge {
            if compact_members[&compact_edge]
                .slots
                .iter()
                .copied()
                .collect::<BTreeSet<_>>()
                != *slots
            {
                return Err(GroupExpansionError::NeedsFullRelayout);
            }
        }

        let lane_count = expanded_bundle
            .members
            .last()
            .map_or(0, |member| member.tap_lane + 1);
        let center = lane_count.saturating_sub(1) as f64 / 2.0;
        let mut members = Vec::with_capacity(mapped_members.len());
        for (expanded_member, compact_edge, compact_member, affected_member) in mapped_members {
            let tap = Point {
                x: compact_bundle.spine.end.x,
                y: compact_bundle.spine.end.y + (expanded_member.tap_lane as f64 - center) * pitch,
            };
            let retained_member = !affected_member && expanded_member.edge == compact_edge;
            let corridor_changed = expanded_corridor_offset != compact_corridor_offset;
            let tap_changed = !boundary_bundles::preserved_point_matches(tap, compact_member.tap);
            if retained_member && (tap_changed || corridor_changed) {
                return Err(GroupExpansionError::NeedsFullRelayout);
            }
            if affected_member
                && retaps
                    .insert(
                        expanded_member.edge,
                        BoundaryBundleRetap {
                            role: compact_bundle.role,
                            tap,
                            tap_lane: expanded_member.tap_lane,
                            corridor_offset: expanded_corridor_offset,
                        },
                    )
                    .is_some()
            {
                return Err(GroupExpansionError::NeedsFullRelayout);
            }
            members.push(BoundaryBundleMemberGeometry {
                edge: expanded_member.edge,
                slots: expanded_member.slots.clone(),
                tap,
            });
        }

        if !affected
            && compact_bundle.members.len() == members.len()
            && compact_bundle
                .members
                .iter()
                .zip(&members)
                .all(|(left, right)| {
                    left.edge == right.edge
                        && left.slots == right.slots
                        && boundary_bundles::preserved_point_matches(left.tap, right.tap)
                })
        {
            remapped.push(compact_bundle.clone());
        } else {
            let mut geometry = compact_bundle.clone();
            geometry.collector = crate::BoundaryBundleSegment {
                start: Point {
                    x: geometry.spine.end.x,
                    y: members.first().map_or(geometry.spine.end.y, |member| {
                        member.tap.y.min(geometry.spine.end.y)
                    }),
                },
                end: Point {
                    x: geometry.spine.end.x,
                    y: members.last().map_or(geometry.spine.end.y, |member| {
                        member.tap.y.max(geometry.spine.end.y)
                    }),
                },
            };
            geometry.members = members;
            remapped.push(geometry);
        }
    }
    if seen_compact_bundles.len() != expanded_bundles.len() {
        return Err(GroupExpansionError::NeedsFullRelayout);
    }
    Ok(RemappedBoundaryBundles {
        geometry: remapped,
        retaps,
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
    let bundled_endpoints = bundled_route_endpoints(layout)?;
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
        let graph_source = endpoint_point(
            node_geometry[&edge.source.node],
            nodes[&edge.source.node],
            edge.source,
        );
        let graph_target = endpoint_point(
            node_geometry[&edge.target.node],
            nodes[&edge.target.node],
            edge.target,
        );
        let endpoints = bundled_endpoints.get(&edge.id).copied().unwrap_or_default();
        let source = endpoints.source.unwrap_or(graph_source);
        let target = endpoints.target.unwrap_or(graph_target);
        let source_side = if endpoints.source.is_some() {
            PortSide::East
        } else {
            port(nodes[&edge.source.node], edge.source).side
        };
        let target_side = if endpoints.target.is_some() {
            PortSide::West
        } else {
            port(nodes[&edge.target.node], edge.target).side
        };
        if !valid_points
            || !boundary_bundles::preserved_point_matches(route.points[0], source)
            || !boundary_bundles::preserved_point_matches(
                route.points[route.points.len() - 1],
                target,
            )
            || !correct_direction(source, route.points[1], source_side)
            || !correct_direction(target, route.points[route.points.len() - 2], target_side)
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

fn bundled_route_endpoints(
    layout: &Layout,
) -> Result<BTreeMap<EdgeId, BundledRouteEndpoints>, GroupExpansionError> {
    let mut endpoints = BTreeMap::<EdgeId, BundledRouteEndpoints>::new();
    for bundle in &layout.boundary_bundles {
        for member in &bundle.members {
            let entry = endpoints.entry(member.edge).or_default();
            let endpoint = match bundle.role {
                BoundaryBundleRole::Input => &mut entry.source,
                BoundaryBundleRole::Output => &mut entry.target,
            };
            if endpoint.replace(member.tap).is_some() {
                return Err(GroupExpansionError::NeedsFullRelayout);
            }
        }
    }
    Ok(endpoints)
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
    let bounds_contain_bundles = layout.boundary_bundles.iter().all(|bundle| {
        [
            bundle.collector.start,
            bundle.collector.end,
            bundle.spine.start,
            bundle.spine.end,
        ]
        .into_iter()
        .chain(bundle.members.iter().map(|member| member.tap))
        .all(|point| {
            point.x.is_finite()
                && point.y.is_finite()
                && point.x >= 0.0
                && point.y >= 0.0
                && point.x <= layout.width
                && point.y <= layout.height
        })
    });
    if bounds_contain_nodes && bounds_contain_edges && bounds_contain_bundles {
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

fn shared_boundary_group_count(
    graph: &Graph,
    contract: &ExpansionContract<'_>,
    constraints: &LayoutConstraints,
) -> usize {
    let bundled = constraints
        .boundary_bundles
        .iter()
        .flat_map(|bundle| bundle.members.iter().map(|member| member.edge))
        .collect::<BTreeSet<_>>();
    let mut incoming = BTreeMap::<(EndpointKey, NetId), usize>::new();
    let mut outgoing = BTreeMap::<(EndpointKey, NetId), usize>::new();
    for edge in &graph.edges {
        if bundled.contains(&edge.id) {
            continue;
        }
        let source_member = contract.members.contains(&edge.source.node);
        let target_member = contract.members.contains(&edge.target.node);
        if !source_member && target_member {
            *incoming
                .entry(((edge.source.node, edge.source.port), edge.net))
                .or_default() += 1;
        } else if source_member && !target_member {
            *outgoing
                .entry(((edge.target.node, edge.target.port), edge.net))
                .or_default() += 1;
        }
    }
    incoming
        .into_values()
        .chain(outgoing.into_values())
        .filter(|&members| members > 1)
        .count()
}

fn insert_horizontal_expansion_corridor(
    compact_layout: &Layout,
    anchor: NodeId,
    member_width: f64,
) -> Option<Layout> {
    let anchor_geometry = compact_layout.nodes.iter().find(|node| node.id == anchor)?;
    let additional_width = member_width - anchor_geometry.width;
    if additional_width <= HARD_GATE_EPSILON {
        return None;
    }

    let anchor_right = anchor_geometry.x + anchor_geometry.width;
    let next_node_x = compact_layout
        .nodes
        .iter()
        .filter(|node| node.id != anchor && node.x + HARD_GATE_EPSILON >= anchor_right)
        .map(|node| node.x)
        .min_by(f64::total_cmp);
    let cut = next_node_x.map_or(anchor_right, |next| (anchor_right + next) / 2.0);
    let cut_crosses_retained_node = compact_layout.nodes.iter().any(|node| {
        node.id != anchor
            && node.x < cut - HARD_GATE_EPSILON
            && node.x + node.width > cut + HARD_GATE_EPSILON
    });
    if cut_crosses_retained_node {
        return None;
    }

    let shift_point = |point: &mut Point| {
        if point.x + HARD_GATE_EPSILON >= cut {
            point.x += additional_width;
        }
    };
    let mut expanded = compact_layout.clone();
    for node in &mut expanded.nodes {
        if node.id != anchor && node.x + HARD_GATE_EPSILON >= cut {
            node.x += additional_width;
        }
    }
    for route in &mut expanded.edges {
        for point in &mut route.points {
            shift_point(point);
        }
    }
    for bundle in &mut expanded.boundary_bundles {
        shift_point(&mut bundle.collector.start);
        shift_point(&mut bundle.collector.end);
        shift_point(&mut bundle.spine.start);
        shift_point(&mut bundle.spine.end);
        for member in &mut bundle.members {
            shift_point(&mut member.tap);
        }
    }
    expanded.width += additional_width;
    Some(expanded)
}

fn insert_local_vertical_expansion_corridor(
    compact_graph: &Graph,
    compact_layout: &Layout,
    anchor: NodeId,
    member_width: f64,
    member_height: f64,
    options: LayoutOptions,
) -> Option<Layout> {
    let anchor_geometry = compact_layout.nodes.iter().find(|node| node.id == anchor)?;
    if member_height <= anchor_geometry.height + HARD_GATE_EPSILON {
        return None;
    }

    let cut = anchor_geometry.y + anchor_geometry.height;
    let margin = options.node_gap / 2.0;
    let frame = Rect {
        left: anchor_geometry.x,
        top: anchor_geometry.y,
        right: anchor_geometry.x + member_width,
        bottom: anchor_geometry.y + member_height,
    };
    let blocker_top = compact_layout
        .nodes
        .iter()
        .filter(|node| {
            node.id != anchor
                && node.y + HARD_GATE_EPSILON >= cut
                && Rect::from_node(node).overlap_area(frame.expanded(margin)) > HARD_GATE_EPSILON
        })
        .map(|node| node.y)
        .min_by(f64::total_cmp)?;
    let delta = frame.bottom + margin - blocker_top;
    if delta <= HARD_GATE_EPSILON {
        return None;
    }

    // Merge x intervals once to find the connected slab containing the
    // expanded frame. This keeps nodes atomic without an all-pairs closure.
    let seed_left = (frame.left - margin).max(0.0);
    let seed_right = frame.right + margin;
    let mut intervals = compact_layout
        .nodes
        .iter()
        .filter(|node| node.id != anchor && node.y + HARD_GATE_EPSILON >= cut)
        .map(|node| {
            (
                (node.x - margin).max(0.0),
                node.x + node.width + margin,
                false,
            )
        })
        .collect::<Vec<_>>();
    intervals.push((seed_left, seed_right, true));
    intervals.sort_by(|left, right| {
        left.0
            .total_cmp(&right.0)
            .then(left.1.total_cmp(&right.1))
            .then(left.2.cmp(&right.2))
    });
    let mut components = Vec::<(f64, f64, bool)>::new();
    for (left, right, contains_seed) in intervals {
        if let Some(component) = components.last_mut()
            && left <= component.1 + HARD_GATE_EPSILON
        {
            component.1 = component.1.max(right);
            component.2 |= contains_seed;
        } else {
            components.push((left, right, contains_seed));
        }
    }
    let (left, right, _) = components
        .into_iter()
        .find(|component| component.2)
        .expect("the frame seed belongs to one merged interval");
    let slab = Rect {
        left,
        top: cut,
        right,
        bottom: f64::INFINITY,
    };
    if compact_layout.nodes.iter().any(|node| {
        node.id != anchor
            && node.x < slab.right - HARD_GATE_EPSILON
            && node.x + node.width > slab.left + HARD_GATE_EPSILON
            && node.y < cut - HARD_GATE_EPSILON
            && node.y + node.height > cut + HARD_GATE_EPSILON
    }) {
        return None;
    }

    let mut expanded = compact_layout.clone();
    let anchor_edges = compact_graph
        .edges
        .iter()
        .filter(|edge| edge.source.node == anchor || edge.target.node == anchor)
        .map(|edge| edge.id)
        .collect::<BTreeSet<_>>();
    let protected_bundle_edges = compact_layout
        .boundary_bundles
        .iter()
        .flat_map(|bundle| bundle.members.iter().map(|member| member.edge))
        .filter(|edge| !anchor_edges.contains(edge))
        .collect::<BTreeSet<_>>();
    let mut moved = false;
    for node in &mut expanded.nodes {
        if node.id != anchor
            && node.y + HARD_GATE_EPSILON >= cut
            && node.x >= slab.left - HARD_GATE_EPSILON
            && node.x + node.width <= slab.right + HARD_GATE_EPSILON
        {
            node.y += delta;
            moved = true;
        }
    }
    if !moved {
        return None;
    }
    for route in &mut expanded.edges {
        let warped = warp_vertical_slab_route(&route.points, slab.left, slab.right, cut, delta);
        if protected_bundle_edges.contains(&route.id) && warped != route.points {
            return None;
        }
        route.points = warped;
    }
    if route_segment_count(&expanded) > MAX_LAYOUT_SEGMENTS {
        return None;
    }
    for bundle in &mut expanded.boundary_bundles {
        bundle.collector =
            warp_vertical_slab_segment(bundle.collector, slab.left, slab.right, cut, delta)?;
        bundle.spine = warp_vertical_slab_segment(bundle.spine, slab.left, slab.right, cut, delta)?;
        for member in &mut bundle.members {
            member.tap = warp_vertical_slab_point(member.tap, slab.left, slab.right, cut, delta);
        }
    }
    expanded.height = layout_bottom(&expanded).max(compact_layout.height);
    Some(expanded)
}

fn warp_vertical_slab_point(point: Point, left: f64, right: f64, cut: f64, delta: f64) -> Point {
    if point.x > left + HARD_GATE_EPSILON
        && point.x < right - HARD_GATE_EPSILON
        && point.y + HARD_GATE_EPSILON >= cut
    {
        Point {
            y: point.y + delta,
            ..point
        }
    } else {
        point
    }
}

fn warp_vertical_slab_route(
    points: &[Point],
    left: f64,
    right: f64,
    cut: f64,
    delta: f64,
) -> Vec<Point> {
    let Some(&first) = points.first() else {
        return Vec::new();
    };
    let mut warped = vec![warp_vertical_slab_point(first, left, right, cut, delta)];
    for segment in points.windows(2) {
        let start = segment[0];
        let end = segment[1];
        if start.x == end.x || start.y < cut - HARD_GATE_EPSILON {
            warped.push(warp_vertical_slab_point(end, left, right, cut, delta));
            continue;
        }
        debug_assert_eq!(start.y, end.y);
        let (minimum, maximum) = if start.x <= end.x {
            (start.x, end.x)
        } else {
            (end.x, start.x)
        };
        let mut breaks = [left, right]
            .into_iter()
            .filter(|x| *x > minimum + HARD_GATE_EPSILON && *x < maximum - HARD_GATE_EPSILON)
            .collect::<Vec<_>>();
        breaks.sort_by(f64::total_cmp);
        if start.x > end.x {
            breaks.reverse();
        }
        breaks.push(end.x);
        let mut current_x = start.x;
        for next_x in breaks {
            let midpoint = (current_x + next_x) / 2.0;
            let interval_y =
                if midpoint > left + HARD_GATE_EPSILON && midpoint < right - HARD_GATE_EPSILON {
                    start.y + delta
                } else {
                    start.y
                };
            let current = *warped.last().expect("warped route has a start");
            if current.y != interval_y {
                warped.push(Point {
                    x: current_x,
                    y: interval_y,
                });
            }
            warped.push(Point {
                x: next_x,
                y: interval_y,
            });
            current_x = next_x;
        }
        let transformed_end = warp_vertical_slab_point(end, left, right, cut, delta);
        if warped.last().copied() != Some(transformed_end) {
            warped.push(transformed_end);
        }
    }
    simplify_orthogonal(warped)
}

fn warp_vertical_slab_segment(
    segment: BoundaryBundleSegment,
    left: f64,
    right: f64,
    cut: f64,
    delta: f64,
) -> Option<BoundaryBundleSegment> {
    let points = warp_vertical_slab_route(&[segment.start, segment.end], left, right, cut, delta);
    match points.as_slice() {
        [point] => Some(BoundaryBundleSegment {
            start: *point,
            end: *point,
        }),
        [start, end] => Some(BoundaryBundleSegment {
            start: *start,
            end: *end,
        }),
        _ => None,
    }
}

fn layout_bottom(layout: &Layout) -> f64 {
    layout
        .nodes
        .iter()
        .map(|node| node.y + node.height)
        .chain(
            layout
                .edges
                .iter()
                .flat_map(|edge| edge.points.iter().map(|point| point.y)),
        )
        .chain(layout.boundary_bundles.iter().flat_map(|bundle| {
            [
                bundle.collector.start.y,
                bundle.collector.end.y,
                bundle.spine.start.y,
                bundle.spine.end.y,
            ]
            .into_iter()
            .chain(bundle.members.iter().map(|member| member.tap.y))
        }))
        .fold(0.0, f64::max)
}

fn arrange_member_components(
    graph: &Graph,
    layout: &Layout,
    vertical_gap: f64,
    horizontal_gap: f64,
    compact_layout_height: f64,
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
    let stacked_height = bounds.iter().map(|bounds| bounds.height()).sum::<f64>()
        + vertical_gap * components.len().saturating_sub(1) as f64;
    let height_limit = compact_layout_height * EXPANSION_STACK_HEIGHT_FACTOR;
    let columns = if stacked_height <= height_limit + HARD_GATE_EPSILON {
        1
    } else {
        let ideal = (((components.len() as f64 * (max_height + vertical_gap)
            / (max_width + horizontal_gap))
            .sqrt())
        .ceil() as usize)
            .clamp(2, components.len());
        let maximum = maximum_width.map_or(components.len(), |width| {
            (((width + horizontal_gap) / (max_width + horizontal_gap)).floor() as usize)
                .clamp(1, components.len())
        });
        ideal.min(maximum).max(2.min(maximum))
    };
    let rows = components.len().div_ceil(columns);
    let mut column_widths = vec![0.0_f64; columns];
    let mut row_heights = vec![0.0_f64; rows];
    for (index, bounds) in bounds.iter().enumerate() {
        column_widths[index % columns] = column_widths[index % columns].max(bounds.width());
        row_heights[index / columns] = row_heights[index / columns].max(bounds.height());
    }
    let mut column_x = vec![0.0; columns];
    for index in 1..columns {
        column_x[index] = column_x[index - 1] + column_widths[index - 1] + horizontal_gap;
    }
    let mut row_y = vec![0.0; rows];
    for index in 1..rows {
        row_y[index] = row_y[index - 1] + row_heights[index - 1] + vertical_gap;
    }

    let mut translated_nodes = Vec::with_capacity(layout.nodes.len());
    let mut translated_edges = Vec::with_capacity(layout.edges.len());
    for (index, component) in components.iter().enumerate() {
        let column = index % columns;
        let row = index / columns;
        let bounds = bounds[index];
        let x = column_x[column] - bounds.left;
        let y = row_y[row] - bounds.top;
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
        boundary_bundles: Vec::new(),
        width: column_widths.iter().sum::<f64>()
            + horizontal_gap * columns.saturating_sub(1) as f64,
        height: row_heights.iter().sum::<f64>() + vertical_gap * rows.saturating_sub(1) as f64,
    }
}

fn expansion_grid_horizontal_gap(
    expanded_graph: &Graph,
    contract: &ExpansionContract<'_>,
    arranged: &Layout,
    options: LayoutOptions,
) -> f64 {
    let geometry = arranged
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let mut columns = contract
        .members
        .iter()
        .map(|member| FloatKey(geometry[member].x))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    columns.sort();
    if columns.len() <= 1 {
        return 0.0;
    }

    let clearance = crate::outward_obstacle_clearance_stub(options);
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let mut required = 0.0_f64;
    for adjacent in columns.windows(2) {
        let outgoing = expanded_graph
            .edges
            .iter()
            .filter(|edge| {
                contract.members.contains(&edge.source.node)
                    && !contract.members.contains(&edge.target.node)
                    && FloatKey(geometry[&edge.source.node].x) == adjacent[0]
            })
            .map(|edge| edge.net)
            .collect::<BTreeSet<_>>()
            .len();
        let incoming = expanded_graph
            .edges
            .iter()
            .filter(|edge| {
                !contract.members.contains(&edge.source.node)
                    && contract.members.contains(&edge.target.node)
                    && FloatKey(geometry[&edge.target.node].x) == adjacent[1]
            })
            .map(|edge| edge.net)
            .collect::<BTreeSet<_>>()
            .len();
        let occupied_lanes = outgoing.saturating_add(incoming);
        if occupied_lanes != 0 {
            required = required.max(
                clearance * 2.0 + pitch * occupied_lanes.saturating_sub(1) as f64 + pitch * 2.0,
            );
        }
    }
    required
}

fn expansion_input_corridor_padding(
    compact_layout: &Layout,
    expanded_graph: &Graph,
    expanded_indexed: &validation::IndexedGraph<'_>,
    contract: &ExpansionContract<'_>,
    arranged: &Layout,
    options: LayoutOptions,
) -> f64 {
    let geometry = arranged
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let Some(first_column) = contract
        .members
        .iter()
        .map(|member| geometry[member].x)
        .min_by(f64::total_cmp)
    else {
        return 0.0;
    };
    let incoming = expanded_graph
        .edges
        .iter()
        .filter(|edge| {
            !contract.members.contains(&edge.source.node)
                && contract.members.contains(&edge.target.node)
                && (geometry[&edge.target.node].x - first_column).abs() <= HARD_GATE_EPSILON
        })
        .collect::<Vec<_>>();
    if incoming.is_empty() {
        return 0.0;
    }

    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let bundle_geometry = compact_layout
        .boundary_bundles
        .iter()
        .map(|bundle| (bundle.id, bundle))
        .collect::<BTreeMap<_, _>>();
    let bundle_offsets =
        expanded_indexed
            .boundary_bundles
            .iter()
            .zip(boundary_bundles::corridor_depth_offsets(
                expanded_indexed,
                options,
            ));
    let mut bundle_corridor_by_edge = BTreeMap::new();
    for (bundle, offset) in bundle_offsets {
        if bundle.role != BoundaryBundleRole::Input {
            continue;
        }
        let Some(geometry) = bundle_geometry.get(&bundle.id).copied() else {
            continue;
        };
        for member in &bundle.members {
            bundle_corridor_by_edge.insert(
                member.edge,
                geometry.spine.end.x + offset + (member.tap_lane + 1) as f64 * pitch,
            );
        }
    }

    let furthest_corridor = incoming
        .iter()
        .filter_map(|edge| {
            bundle_corridor_by_edge.get(&edge.id).copied().or_else(|| {
                let compact_edge = contract.boundary_trunks[&edge.id];
                contract.compact_edge_geometry[&compact_edge]
                    .points
                    .iter()
                    .filter(|point| point.x < contract.anchor_geometry.x - HARD_GATE_EPSILON)
                    .map(|point| point.x)
                    .max_by(f64::total_cmp)
            })
        })
        .max_by(f64::total_cmp)
        .unwrap_or(contract.anchor_geometry.x);
    let approach_lanes = incoming
        .iter()
        .map(|edge| edge.net)
        .collect::<BTreeSet<_>>()
        .len();
    let approach_depth = crate::outward_obstacle_clearance_stub(options)
        + approach_lanes.saturating_sub(1) as f64 * pitch
        // One lane separates the two banks; the second keeps a non-bundled
        // boundary net from landing on the final bundle-member corridor.
        + pitch * 2.0;
    let first_node_x = contract.anchor_geometry.x + first_column;
    (furthest_corridor + approach_depth - first_node_x).max(0.0)
}

fn add_horizontal_member_padding(mut layout: Layout, left: f64, right: f64) -> Layout {
    for node in &mut layout.nodes {
        node.x += left;
    }
    for point in layout.edges.iter_mut().flat_map(|edge| &mut edge.points) {
        point.x += left;
    }
    layout.width += left + right;
    layout
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
    let Ok(bundled_endpoints) = bundled_route_endpoints(layout) else {
        return Ok(false);
    };
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
        let graph_source = endpoint_point(
            laid_out_nodes[&edge.source.node],
            graph_nodes[&edge.source.node],
            edge.source,
        );
        let graph_target = endpoint_point(
            laid_out_nodes[&edge.target.node],
            graph_nodes[&edge.target.node],
            edge.target,
        );
        let endpoints = bundled_endpoints.get(&edge.id).copied().unwrap_or_default();
        let source = endpoints.source.unwrap_or(graph_source);
        let target = endpoints.target.unwrap_or(graph_target);
        let source_side = if endpoints.source.is_some() {
            PortSide::East
        } else {
            port(graph_nodes[&edge.source.node], edge.source).side
        };
        let target_side = if endpoints.target.is_some() {
            PortSide::West
        } else {
            port(graph_nodes[&edge.target.node], edge.target).side
        };
        if !boundary_bundles::preserved_point_matches(route.points[0], source)
            || !boundary_bundles::preserved_point_matches(
                route.points[route.points.len() - 1],
                target,
            )
            || !correct_direction(source, route.points[1], source_side)
            || !correct_direction(target, route.points[route.points.len() - 2], target_side)
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
    reserved_candidates: usize,
) -> Result<Vec<(f64, f64)>, GroupExpansionError> {
    let radius: i32 = match effort {
        QualityEffort::Fast => 0,
        QualityEffort::Quality => 1,
        QualityEffort::Max => 3,
    };
    let budget = candidate_work_budget(effort);
    let bridge_work = work.boundary_edges.saturating_mul(work.nodes);
    let projected_clearance_work = if options.edge_node_clearance > 0.0 {
        work.projected_segments
            .saturating_mul(work.nodes)
            .saturating_add(work.projected_segments)
            .saturating_add(work.nodes)
    } else {
        0
    };
    let projected_parallel_work = if options.minimum_parallel_wire_spacing > 0.0 {
        work.projected_segments
            .saturating_mul(work.projected_segments)
            .saturating_mul(8)
            .saturating_add(work.projected_segments.saturating_mul(8))
    } else {
        0
    };
    let projected_bundle_work = if work.boundary_bundles > 0 {
        let bundle_segments = work.boundary_bundles.saturating_mul(2);
        work.nodes
            .saturating_add(work.edges)
            .saturating_add(work.boundary_bundles)
            .saturating_add(work.boundary_bundle_members)
            .saturating_add(bundle_segments.saturating_mul(work.nodes))
            .saturating_add(work.projected_segments.saturating_mul(work.nodes))
            .saturating_add(bundle_segments.saturating_mul(work.projected_segments))
            .saturating_add(bundle_segments.saturating_mul(bundle_segments))
    } else {
        0
    };
    let work_per_candidate = work
        .nodes
        .saturating_add(work.edges)
        .saturating_add(work.projected_segments)
        .saturating_add(bridge_work)
        .saturating_add(projected_clearance_work)
        .saturating_add(projected_parallel_work)
        .saturating_add(projected_bundle_work)
        .max(1);
    let reserved_candidates = reserved_candidates.saturating_add(SAFETY_CANDIDATES);
    let minimum_work = work_per_candidate.saturating_mul(reserved_candidates.saturating_add(1));
    if minimum_work > budget {
        return Err(GroupExpansionError::ExpansionWorkLimitExceeded {
            required: minimum_work,
            maximum: budget,
        });
    }
    let candidate_limit = (budget / work_per_candidate).saturating_sub(reserved_candidates);
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
    let regular_limit = positions.len();
    positions.insert(0, (anchor.x, anchor.y));
    let mut seen = BTreeSet::new();
    positions.retain(|&(x, y)| seen.insert((x.to_bits(), y.to_bits())));
    positions.truncate(regular_limit);
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
    debug_assert!(
        positions
            .len()
            .saturating_add(reserved_candidates.saturating_sub(SAFETY_CANDIDATES))
            .saturating_mul(work_per_candidate)
            <= budget
    );
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
fn compose_collapse_candidate(
    expanded_graph: &Graph,
    expanded_layout: &Layout,
    compact_graph: &Graph,
    expansion: &GroupExpansion,
    contract: &GraphExpansionContract<'_>,
    expanded_node_geometry: &BTreeMap<NodeId, &NodeGeometry>,
    expanded_edge_geometry: &BTreeMap<EdgeId, &EdgeGeometry>,
    options: LayoutOptions,
) -> Result<Layout, GroupExpansionError> {
    let member_frame = contract
        .members
        .iter()
        .map(|member| Rect::from_node(expanded_node_geometry[member]))
        .reduce(|left, right| Rect {
            left: left.left.min(right.left),
            top: left.top.min(right.top),
            right: left.right.max(right.right),
            bottom: left.bottom.max(right.bottom),
        })
        .expect("validated collapse contains members");
    let anchor = compact_graph
        .nodes
        .iter()
        .find(|node| node.id == expansion.anchor)
        .expect("validated collapse contains its compact anchor");
    let anchor_geometry = NodeGeometry {
        id: anchor.id,
        x: member_frame.left,
        y: member_frame.top,
        width: anchor.width,
        height: anchor.height,
    };
    let mut nodes = expanded_layout
        .nodes
        .iter()
        .filter(|node| !contract.members.contains(&node.id))
        .cloned()
        .collect::<Vec<_>>();
    nodes.push(anchor_geometry);
    nodes.sort_unstable_by_key(|node| node.id);
    let node_geometry = nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let obstacles = ObstacleIndex::new(nodes.iter().map(Rect::from_node).collect());
    let expanded_edges = expanded_graph
        .edges
        .iter()
        .map(|edge| (edge.id, edge))
        .collect::<BTreeMap<_, _>>();
    let mut expanded_by_compact_trunk = BTreeMap::<EdgeId, Vec<EdgeId>>::new();
    for (&expanded, &compact) in &contract.boundary_trunks {
        expanded_by_compact_trunk
            .entry(compact)
            .or_default()
            .push(expanded);
    }
    for mapped in expanded_by_compact_trunk.values_mut() {
        mapped.sort_unstable();
    }
    let mut compact_edges = compact_graph.edges.iter().collect::<Vec<_>>();
    compact_edges.sort_unstable_by_key(|edge| edge.id);
    let mut edges = Vec::with_capacity(compact_edges.len());
    for edge in compact_edges {
        let touches_anchor =
            edge.source.node == expansion.anchor || edge.target.node == expansion.anchor;
        if !touches_anchor {
            edges.push(expanded_edge_geometry[&edge.id].clone());
            continue;
        }
        let mapped = expanded_by_compact_trunk
            .get(&edge.id)
            .and_then(|mapped| mapped.first())
            .copied()
            .ok_or(GroupExpansionError::UnusedCompactTrunk(edge.id))?;
        edges.push(collapse_boundary_route(
            edge,
            expanded_edges[&mapped],
            expanded_edge_geometry[&mapped],
            anchor,
            &node_geometry,
            &contract.expanded_nodes,
            member_frame,
            &obstacles,
            options,
        )?);
    }
    Ok(Layout {
        nodes,
        edges,
        boundary_bundles: Vec::new(),
        width: expanded_layout.width,
        height: expanded_layout.height,
    })
}

#[allow(clippy::too_many_arguments)]
fn collapse_boundary_route(
    compact_edge: &Edge,
    expanded_edge: &Edge,
    expanded_route: &EdgeGeometry,
    anchor: &Node,
    node_geometry: &BTreeMap<NodeId, &NodeGeometry>,
    expanded_nodes: &BTreeMap<NodeId, &Node>,
    member_frame: Rect,
    obstacles: &ObstacleIndex,
    options: LayoutOptions,
) -> Result<EdgeGeometry, GroupExpansionError> {
    if expanded_route.points.len() < 2 {
        return Err(GroupExpansionError::EmptyBoundaryTrunk(expanded_edge.id));
    }
    let clearance = crate::outward_obstacle_clearance_stub(options);
    let gap = (options.node_gap / 2.0).max(options.edge_node_clearance);
    let anchor_is_source = compact_edge.source.node == anchor.id;
    let points = if anchor_is_source {
        let source_port = port(anchor, compact_edge.source);
        let source = endpoint_point(node_geometry[&anchor.id], anchor, compact_edge.source);
        let source_stub = outward_stub(source, source_port.side, clearance);
        let target_node = expanded_nodes[&compact_edge.target.node];
        let target_port = port(target_node, compact_edge.target);
        let target = endpoint_point(
            node_geometry[&compact_edge.target.node],
            target_node,
            compact_edge.target,
        );
        let target_stub = outward_stub(target, target_port.side, clearance);
        let splice_index = expanded_route
            .points
            .iter()
            .position(|point| point.x > member_frame.right + HARD_GATE_EPSILON);
        let preserved_suffix = splice_index.map(|index| &expanded_route.points[index..]);
        let suffix_is_usable = preserved_suffix.is_some_and(|suffix| {
            suffix.len() >= 2
                && boundary_bundles::preserved_point_matches(
                    *suffix.last().expect("non-empty suffix"),
                    target,
                )
                && correct_direction(target, suffix[suffix.len() - 2], target_port.side)
        });
        let splice = if suffix_is_usable {
            preserved_suffix.expect("usable suffix exists")[0]
        } else {
            target_stub
        };
        let bridge = obstacle_safe_bridge(source_stub, splice, obstacles, gap, 0.0)
            .ok_or(GroupExpansionError::NoSafeBoundaryBridge(expanded_edge.id))?;
        let mut points = vec![source, source_stub];
        points.extend(bridge.into_iter().skip(1));
        if suffix_is_usable {
            points.extend(
                preserved_suffix
                    .expect("usable suffix exists")
                    .iter()
                    .copied()
                    .skip(1),
            );
        } else {
            points.push(target);
        }
        points
    } else {
        let source_node = expanded_nodes[&compact_edge.source.node];
        let source_port = port(source_node, compact_edge.source);
        let source = endpoint_point(
            node_geometry[&compact_edge.source.node],
            source_node,
            compact_edge.source,
        );
        let source_stub = outward_stub(source, source_port.side, clearance);
        let target_port = port(anchor, compact_edge.target);
        let target = endpoint_point(node_geometry[&anchor.id], anchor, compact_edge.target);
        let target_stub = outward_stub(target, target_port.side, clearance);
        let splice_index = expanded_route
            .points
            .iter()
            .rposition(|point| point.x < member_frame.left - HARD_GATE_EPSILON);
        let preserved_prefix = splice_index.map(|index| &expanded_route.points[..=index]);
        let prefix_is_usable = preserved_prefix.is_some_and(|prefix| {
            prefix.len() >= 2
                && boundary_bundles::preserved_point_matches(prefix[0], source)
                && correct_direction(source, prefix[1], source_port.side)
        });
        let mut points = if prefix_is_usable {
            preserved_prefix.expect("usable prefix exists").to_vec()
        } else {
            vec![source, source_stub]
        };
        let splice = *points.last().expect("collapse route has a source");
        let bridge = obstacle_safe_bridge(splice, target_stub, obstacles, gap, 0.0)
            .ok_or(GroupExpansionError::NoSafeBoundaryBridge(expanded_edge.id))?;
        points.extend(bridge.into_iter().skip(1));
        points.push(target);
        points
    };
    let points = simplify_orthogonal(points);
    path_is_clear(&points, obstacles)
        .then_some(EdgeGeometry {
            id: compact_edge.id,
            points,
        })
        .ok_or(GroupExpansionError::NoSafeBoundaryBridge(expanded_edge.id))
}

#[allow(clippy::too_many_arguments)]
fn compose_candidate(
    compact_layout: &Layout,
    expanded_graph: &Graph,
    contract: &ExpansionContract<'_>,
    member_layout: &Layout,
    boundary_bundles: &RemappedBoundaryBundles,
    offset_x: f64,
    offset_y: f64,
    options: LayoutOptions,
    prefer_direct_boundary_routes: bool,
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
    let expanded_edge_by_id = expanded_edges
        .iter()
        .map(|edge| (edge.id, *edge))
        .collect::<BTreeMap<_, _>>();
    let mut incoming_groups = BTreeMap::<(EndpointKey, NetId), Vec<EdgeId>>::new();
    let mut outgoing_groups = BTreeMap::<(EndpointKey, NetId), Vec<EdgeId>>::new();
    for edge in expanded_edges.iter().copied() {
        let source_member = contract.members.contains(&edge.source.node);
        let target_member = contract.members.contains(&edge.target.node);
        if !source_member && target_member && !boundary_bundles.retaps.contains_key(&edge.id) {
            incoming_groups
                .entry(((edge.source.node, edge.source.port), edge.net))
                .or_default()
                .push(edge.id);
        } else if source_member && !target_member && !boundary_bundles.retaps.contains_key(&edge.id)
        {
            outgoing_groups
                .entry(((edge.target.node, edge.target.port), edge.net))
                .or_default()
                .push(edge.id);
        }
    }
    let incoming_groups = incoming_groups
        .into_values()
        .filter(|group| group.len() > 1)
        .collect::<Vec<_>>();
    let outgoing_groups = outgoing_groups
        .into_values()
        .filter(|group| group.len() > 1)
        .collect::<Vec<_>>();
    let shared_lane_count = incoming_groups.len() + outgoing_groups.len();
    let mut shared_incoming_by_edge = BTreeMap::<EdgeId, (Vec<EdgeId>, usize, usize)>::new();
    for (lane, group) in incoming_groups.into_iter().enumerate() {
        for &edge in &group {
            shared_incoming_by_edge.insert(edge, (group.clone(), lane, shared_lane_count));
        }
    }
    let mut shared_outgoing_by_edge = BTreeMap::<EdgeId, (Vec<EdgeId>, usize, usize)>::new();
    for (offset, group) in outgoing_groups.into_iter().enumerate() {
        let lane = shared_incoming_by_edge
            .values()
            .map(|(_, lane, _)| *lane)
            .max()
            .map_or(0, |lane| lane + 1)
            + offset;
        for &edge in &group {
            shared_outgoing_by_edge.insert(edge, (group.clone(), lane, shared_lane_count));
        }
    }
    let bridge_pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let mut outgoing_nets = BTreeMap::<FloatKey, BTreeSet<NetId>>::new();
    let mut incoming_nets = BTreeMap::<FloatKey, BTreeSet<NetId>>::new();
    for edge in expanded_edges.iter().filter(|edge| {
        contract.members.contains(&edge.source.node) ^ contract.members.contains(&edge.target.node)
    }) {
        if contract.members.contains(&edge.source.node) {
            let right = node_geometry[&edge.source.node].x + node_geometry[&edge.source.node].width;
            outgoing_nets
                .entry(FloatKey(right))
                .or_default()
                .insert(edge.net);
        } else {
            let left = node_geometry[&edge.target.node].x;
            incoming_nets
                .entry(FloatKey(left))
                .or_default()
                .insert(edge.net);
        }
    }
    let outgoing_bridge_lane_by_net = outgoing_nets
        .values()
        .flatten()
        .copied()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .enumerate()
        .map(|(lane, net)| (net, lane as f64 * bridge_pitch))
        .collect::<BTreeMap<_, _>>();
    let incoming_bridge_lane_by_net = incoming_nets
        .values()
        .flatten()
        .copied()
        .collect::<BTreeSet<_>>()
        .into_iter()
        .enumerate()
        .map(|(lane, net)| (net, lane as f64 * bridge_pitch))
        .collect::<BTreeMap<_, _>>();
    let outgoing_lane_by_column_net = outgoing_nets
        .into_iter()
        .flat_map(|(column, nets)| {
            nets.into_iter()
                .enumerate()
                .map(move |(lane, net)| ((column, net), lane as f64 * bridge_pitch))
        })
        .collect::<BTreeMap<_, _>>();
    let incoming_lane_by_column_net = incoming_nets
        .into_iter()
        .flat_map(|(column, nets)| {
            nets.into_iter()
                .enumerate()
                .map(move |(lane, net)| ((column, net), lane as f64 * bridge_pitch))
        })
        .collect::<BTreeMap<_, _>>();
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
            _ => {
                let bridge_lane_offset = if source_member {
                    outgoing_bridge_lane_by_net[&edge.net]
                } else {
                    incoming_bridge_lane_by_net[&edge.net]
                };
                let approach_lane_offset = if source_member {
                    let source = node_geometry[&edge.source.node];
                    outgoing_lane_by_column_net[&(FloatKey(source.x + source.width), edge.net)]
                } else {
                    incoming_lane_by_column_net
                        [&(FloatKey(node_geometry[&edge.target.node].x), edge.net)]
                };
                if let Some((group, trunk_lane, trunk_lanes)) =
                    shared_incoming_by_edge.get(&edge.id)
                {
                    shared_boundary_trunk_route(
                        edge,
                        group,
                        &expanded_edge_by_id,
                        contract,
                        &node_geometry,
                        &obstacles,
                        options,
                        BoundaryBundleRole::Input,
                        approach_lane_offset,
                        &incoming_lane_by_column_net,
                        *trunk_lane,
                        *trunk_lanes,
                    )?
                } else if let Some((group, trunk_lane, trunk_lanes)) =
                    shared_outgoing_by_edge.get(&edge.id)
                {
                    shared_boundary_trunk_route(
                        edge,
                        group,
                        &expanded_edge_by_id,
                        contract,
                        &node_geometry,
                        &obstacles,
                        options,
                        BoundaryBundleRole::Output,
                        approach_lane_offset,
                        &outgoing_lane_by_column_net,
                        *trunk_lane,
                        *trunk_lanes,
                    )?
                } else if let Some(bundle) = boundary_bundles.retaps.get(&edge.id).copied() {
                    bundled_boundary_route(
                        edge,
                        contract,
                        &node_geometry,
                        &obstacles,
                        options,
                        bundle,
                        bridge_lane_offset,
                        approach_lane_offset,
                        compact_layout.height,
                        prefer_direct_boundary_routes,
                    )?
                } else {
                    boundary_route(
                        edge,
                        contract,
                        &node_geometry,
                        &obstacles,
                        options,
                        bridge_lane_offset,
                        approach_lane_offset,
                        compact_layout.height,
                        prefer_direct_boundary_routes,
                    )?
                }
            }
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
        boundary_bundles: boundary_bundles.geometry.clone(),
        width,
        height,
    })
}

#[allow(clippy::too_many_arguments)]
fn shared_boundary_trunk_route(
    edge: &Edge,
    group: &[EdgeId],
    edges: &BTreeMap<EdgeId, &Edge>,
    contract: &ExpansionContract<'_>,
    node_geometry: &BTreeMap<NodeId, &NodeGeometry>,
    obstacles: &ObstacleIndex,
    options: LayoutOptions,
    role: BoundaryBundleRole,
    approach_lane_offset: f64,
    approach_lane_by_column_net: &BTreeMap<(FloatKey, NetId), f64>,
    trunk_lane: usize,
    trunk_lanes: usize,
) -> Result<EdgeGeometry, GroupExpansionError> {
    let clearance = crate::outward_obstacle_clearance_stub(options);
    let member_nodes = contract
        .members
        .iter()
        .map(|member| node_geometry[member])
        .collect::<Vec<_>>();
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let Some(trunk_y) =
        central_member_corridor_y(&member_nodes, clearance, pitch, trunk_lane, trunk_lanes)
    else {
        return Err(GroupExpansionError::NoSafeBoundaryBridge(edge.id));
    };
    let gap = (options.node_gap / 2.0).max(options.edge_node_clearance);

    let points = match role {
        BoundaryBundleRole::Input => {
            let target_node = contract.expanded_nodes[&edge.target.node];
            let target_port = port(target_node, edge.target);
            if target_port.side != PortSide::West {
                return Err(GroupExpansionError::NoSafeBoundaryBridge(edge.id));
            }
            let target = endpoint_point(node_geometry[&edge.target.node], target_node, edge.target);
            let target_stub =
                outward_stub(target, target_port.side, clearance + approach_lane_offset);
            let entry_x = group
                .iter()
                .map(|id| {
                    let grouped = edges[id];
                    let node = contract.expanded_nodes[&grouped.target.node];
                    let endpoint =
                        endpoint_point(node_geometry[&grouped.target.node], node, grouped.target);
                    let offset = approach_lane_by_column_net
                        [&(FloatKey(node_geometry[&grouped.target.node].x), grouped.net)];
                    outward_stub(
                        endpoint,
                        port(node, grouped.target).side,
                        clearance + offset,
                    )
                    .x
                })
                .min_by(f64::total_cmp)
                .expect("shared boundary group is non-empty");
            let source_node = contract.expanded_nodes[&edge.source.node];
            let source_port = port(source_node, edge.source);
            let source = endpoint_point(node_geometry[&edge.source.node], source_node, edge.source);
            let source_stub = outward_stub(source, source_port.side, clearance);
            let entry = Point {
                x: entry_x,
                y: trunk_y,
            };
            let bridge = simplify_orthogonal(vec![
                source_stub,
                Point {
                    x: entry_x,
                    y: source_stub.y,
                },
                entry,
            ]);
            if !path_is_clear(&bridge, obstacles) {
                return Err(GroupExpansionError::NoSafeBoundaryBridge(edge.id));
            }
            let mut points = vec![source, source_stub];
            points.extend(bridge.into_iter().skip(1));
            points.push(Point {
                x: target_stub.x,
                y: trunk_y,
            });
            points.push(target_stub);
            points.push(target);
            points
        }
        BoundaryBundleRole::Output => {
            let source_node = contract.expanded_nodes[&edge.source.node];
            let source_port = port(source_node, edge.source);
            if source_port.side != PortSide::East {
                return Err(GroupExpansionError::NoSafeBoundaryBridge(edge.id));
            }
            let source = endpoint_point(node_geometry[&edge.source.node], source_node, edge.source);
            let source_stub =
                outward_stub(source, source_port.side, clearance + approach_lane_offset);
            let exit_x = group
                .iter()
                .map(|id| {
                    let grouped = edges[id];
                    let node = contract.expanded_nodes[&grouped.source.node];
                    let endpoint =
                        endpoint_point(node_geometry[&grouped.source.node], node, grouped.source);
                    let source = node_geometry[&grouped.source.node];
                    let offset = approach_lane_by_column_net
                        [&(FloatKey(source.x + source.width), grouped.net)];
                    outward_stub(
                        endpoint,
                        port(node, grouped.source).side,
                        clearance + offset,
                    )
                    .x
                })
                .max_by(f64::total_cmp)
                .expect("shared boundary group is non-empty");
            let target_node = contract.expanded_nodes[&edge.target.node];
            let target_port = port(target_node, edge.target);
            let target = endpoint_point(node_geometry[&edge.target.node], target_node, edge.target);
            let target_stub = outward_stub(target, target_port.side, clearance);
            let exit = Point {
                x: exit_x,
                y: trunk_y,
            };
            let bridge = obstacle_safe_bridge(exit, target_stub, obstacles, gap, 0.0)
                .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            let mut points = vec![source, source_stub];
            points.push(Point {
                x: source_stub.x,
                y: trunk_y,
            });
            points.push(exit);
            points.extend(bridge.into_iter().skip(1));
            points.push(target);
            points
        }
    };
    let points = simplify_orthogonal(points);
    path_is_clear(&points, obstacles)
        .then_some(EdgeGeometry {
            id: edge.id,
            points,
        })
        .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))
}

fn central_member_corridor_y(
    nodes: &[&NodeGeometry],
    clearance: f64,
    pitch: f64,
    lane: usize,
    lanes: usize,
) -> Option<f64> {
    let mut bands = nodes
        .iter()
        .map(|node| (node.y, node.y + node.height))
        .collect::<Vec<_>>();
    bands.sort_by(|left, right| left.0.total_cmp(&right.0).then(left.1.total_cmp(&right.1)));
    let mut merged = Vec::<(f64, f64)>::new();
    for band in bands {
        if let Some(last) = merged.last_mut()
            && band.0 <= last.1 + HARD_GATE_EPSILON
        {
            last.1 = last.1.max(band.1);
        } else {
            merged.push(band);
        }
    }
    let frame_center = merged
        .first()
        .zip(merged.last())
        .map(|(first, last)| (first.0 + last.1) / 2.0)?;
    merged
        .windows(2)
        .filter_map(|bands| {
            let gap = bands[1].0 - bands[0].1;
            let required = clearance * 2.0 + pitch * lanes.saturating_sub(1) as f64;
            (gap + HARD_GATE_EPSILON >= required).then_some(
                (bands[0].1 + bands[1].0) / 2.0
                    + (lane as f64 - lanes.saturating_sub(1) as f64 / 2.0) * pitch,
            )
        })
        .min_by(|left, right| {
            (left - frame_center)
                .abs()
                .total_cmp(&(right - frame_center).abs())
                .then(left.total_cmp(right))
        })
}

#[allow(clippy::too_many_arguments)]
fn bundled_boundary_route(
    edge: &Edge,
    contract: &ExpansionContract<'_>,
    node_geometry: &BTreeMap<NodeId, &NodeGeometry>,
    obstacles: &ObstacleIndex,
    options: LayoutOptions,
    bundle: BoundaryBundleRetap,
    bridge_lane_offset: f64,
    approach_lane_offset: f64,
    preserved_bottom: f64,
    prefer_direct: bool,
) -> Result<EdgeGeometry, GroupExpansionError> {
    let source_member = contract.members.contains(&edge.source.node);
    let target_member = contract.members.contains(&edge.target.node);
    debug_assert_ne!(source_member, target_member);
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let corridor_depth = bundle.corridor_offset + (bundle.tap_lane + 1) as f64 * pitch;
    let gap = (options.node_gap / 2.0).max(options.edge_node_clearance);

    let points = match bundle.role {
        BoundaryBundleRole::Input => {
            debug_assert!(target_member);
            let target_node = contract.expanded_nodes[&edge.target.node];
            let target_port = port(target_node, edge.target);
            let target = endpoint_point(node_geometry[&edge.target.node], target_node, edge.target);
            let target_stub = outward_stub(
                target,
                target_port.side,
                crate::outward_obstacle_clearance_stub(options) + approach_lane_offset,
            );
            let corridor = Point {
                x: bundle.tap.x + corridor_depth,
                y: bundle.tap.y,
            };
            let bridge = if target_port.side == PortSide::West {
                boundary_bridge(
                    corridor,
                    target_stub,
                    obstacles,
                    gap,
                    bridge_lane_offset,
                    false,
                    preserved_bottom,
                    prefer_direct,
                )
            } else {
                obstacle_safe_bridge(corridor, target_stub, obstacles, gap, bridge_lane_offset)
            }
            .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            let mut points = vec![bundle.tap, corridor];
            points.extend(bridge.into_iter().skip(1));
            points.push(target);
            points
        }
        BoundaryBundleRole::Output => {
            debug_assert!(source_member);
            let source_node = contract.expanded_nodes[&edge.source.node];
            let source_port = port(source_node, edge.source);
            let source = endpoint_point(node_geometry[&edge.source.node], source_node, edge.source);
            let source_stub = outward_stub(
                source,
                source_port.side,
                crate::outward_obstacle_clearance_stub(options) + approach_lane_offset,
            );
            let corridor = Point {
                x: bundle.tap.x - corridor_depth,
                y: bundle.tap.y,
            };
            let bridge = if source_port.side == PortSide::East {
                boundary_bridge(
                    source_stub,
                    corridor,
                    obstacles,
                    gap,
                    bridge_lane_offset,
                    true,
                    preserved_bottom,
                    prefer_direct,
                )
            } else {
                obstacle_safe_bridge(source_stub, corridor, obstacles, gap, bridge_lane_offset)
            }
            .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            let mut points = vec![source, source_stub];
            points.extend(bridge.into_iter().skip(1));
            points.push(bundle.tap);
            points
        }
    };
    Ok(EdgeGeometry {
        id: edge.id,
        points: simplify_orthogonal(points),
    })
}

#[allow(clippy::too_many_arguments)]
fn boundary_route(
    edge: &Edge,
    contract: &ExpansionContract<'_>,
    node_geometry: &BTreeMap<NodeId, &NodeGeometry>,
    obstacles: &ObstacleIndex,
    options: LayoutOptions,
    bridge_lane_offset: f64,
    approach_lane_offset: f64,
    preserved_bottom: f64,
    prefer_direct: bool,
) -> Result<EdgeGeometry, GroupExpansionError> {
    let source_member = contract.members.contains(&edge.source.node);
    let target_member = contract.members.contains(&edge.target.node);
    debug_assert_ne!(source_member, target_member);
    let trunk_id = contract.boundary_trunks[&edge.id];
    let trunk_geometry = contract.compact_edge_geometry[&trunk_id];
    if trunk_geometry.points.is_empty() {
        return Err(GroupExpansionError::EmptyBoundaryTrunk(edge.id));
    }
    let trunk_start = trunk_geometry.points[0];
    let trunk_end = trunk_geometry.points[trunk_geometry.points.len() - 1];
    let member_frame = contract
        .members
        .iter()
        .map(|member| Rect::from_node(node_geometry[member]))
        .reduce(|left, right| Rect {
            left: left.left.min(right.left),
            top: left.top.min(right.top),
            right: left.right.max(right.right),
            bottom: left.bottom.max(right.bottom),
        })
        .expect("validated expansion contains members");
    let points = if source_member {
        let source_node = contract.expanded_nodes[&edge.source.node];
        let source_port = port(source_node, edge.source);
        let source = endpoint_point(node_geometry[&edge.source.node], source_node, edge.source);
        if boundary_bundles::preserved_point_matches(source, trunk_start)
            && path_is_clear(&trunk_geometry.points, obstacles)
        {
            let mut route = trunk_geometry.clone();
            route.id = edge.id;
            return Ok(route);
        }
        let source_stub = outward_stub(
            source,
            source_port.side,
            crate::outward_obstacle_clearance_stub(options) + approach_lane_offset,
        );
        let target_node = contract.expanded_nodes[&edge.target.node];
        let target_port = port(target_node, edge.target);
        let graph_target =
            endpoint_point(node_geometry[&edge.target.node], target_node, edge.target);
        let (target, target_side) =
            if boundary_bundles::preserved_point_matches(graph_target, trunk_end) {
                (graph_target, target_port.side)
            } else {
                (trunk_end, PortSide::West)
            };
        let target_stub = outward_stub(
            target,
            target_side,
            crate::outward_obstacle_clearance_stub(options),
        );
        let splice_index = if source_port.side == PortSide::East {
            trunk_geometry
                .points
                .iter()
                .position(|point| point.x > member_frame.right + HARD_GATE_EPSILON)
                .unwrap_or(0)
        } else {
            0
        };
        let preserved_suffix = &trunk_geometry.points[splice_index..];
        let suffix_preserves_target_departure = preserved_suffix.len() >= 2
            && boundary_bundles::preserved_point_matches(
                preserved_suffix[preserved_suffix.len() - 1],
                target,
            )
            && correct_direction(
                target,
                preserved_suffix[preserved_suffix.len() - 2],
                target_side,
            );
        let splice = if suffix_preserves_target_departure {
            preserved_suffix[0]
        } else {
            target_stub
        };
        let mut points = vec![source, source_stub];
        let gap = (options.node_gap / 2.0).max(options.edge_node_clearance);
        if source_port.side == PortSide::East {
            let splice_bridge = boundary_bridge(
                source_stub,
                splice,
                obstacles,
                gap,
                bridge_lane_offset,
                true,
                preserved_bottom,
                prefer_direct,
            )
            .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            points.extend(splice_bridge.into_iter().skip(1));
        } else {
            let bridge =
                obstacle_safe_bridge(source_stub, splice, obstacles, gap, bridge_lane_offset)
                    .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            points.extend(bridge.into_iter().skip(1));
        }
        if suffix_preserves_target_departure {
            points.extend(preserved_suffix.iter().copied().skip(1));
        } else {
            points.push(target);
        }
        points
    } else {
        let target_node = contract.expanded_nodes[&edge.target.node];
        let target_port = port(target_node, edge.target);
        let target = endpoint_point(node_geometry[&edge.target.node], target_node, edge.target);
        if boundary_bundles::preserved_point_matches(target, trunk_end)
            && path_is_clear(&trunk_geometry.points, obstacles)
        {
            let mut route = trunk_geometry.clone();
            route.id = edge.id;
            return Ok(route);
        }
        let target_stub = outward_stub(
            target,
            target_port.side,
            crate::outward_obstacle_clearance_stub(options) + approach_lane_offset,
        );
        let splice_index = if target_port.side == PortSide::West {
            trunk_geometry
                .points
                .iter()
                .rposition(|point| point.x < member_frame.left - HARD_GATE_EPSILON)
                .unwrap_or(trunk_geometry.points.len() - 1)
        } else {
            trunk_geometry.points.len() - 1
        };
        let source_node = contract.expanded_nodes[&edge.source.node];
        let source_port = port(source_node, edge.source);
        let graph_source =
            endpoint_point(node_geometry[&edge.source.node], source_node, edge.source);
        let (source, source_side) =
            if boundary_bundles::preserved_point_matches(graph_source, trunk_start) {
                (graph_source, source_port.side)
            } else {
                (trunk_start, PortSide::East)
            };
        let source_stub = outward_stub(
            source,
            source_side,
            crate::outward_obstacle_clearance_stub(options),
        );
        let preserved_prefix = &trunk_geometry.points[..=splice_index];
        let prefix_preserves_source_departure = preserved_prefix.len() >= 2
            && boundary_bundles::preserved_point_matches(preserved_prefix[0], source)
            && correct_direction(source, preserved_prefix[1], source_side);
        let mut points = if prefix_preserves_source_departure {
            preserved_prefix.to_vec()
        } else {
            vec![source, source_stub]
        };
        let splice = *points.last().expect("boundary route has a source");
        let gap = (options.node_gap / 2.0).max(options.edge_node_clearance);
        if target_port.side == PortSide::West {
            let escape_bridge = boundary_bridge(
                splice,
                target_stub,
                obstacles,
                gap,
                bridge_lane_offset,
                false,
                preserved_bottom,
                prefer_direct,
            )
            .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            points.extend(escape_bridge.into_iter().skip(1));
        } else {
            let bridge =
                obstacle_safe_bridge(splice, target_stub, obstacles, gap, bridge_lane_offset)
                    .ok_or(GroupExpansionError::NoSafeBoundaryBridge(edge.id))?;
            points.extend(bridge.into_iter().skip(1));
        }
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
    lane_offset: f64,
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
        + gap
        + lane_offset;
    let bottom = obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.bottom)
        .fold(start.y.max(end.y), f64::max)
        + gap
        + lane_offset;
    let left = (obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.left)
        .fold(start.x.min(end.x), f64::min)
        - gap
        - lane_offset)
        .max(0.0);
    let top = (obstacles
        .rects
        .iter()
        .map(|obstacle| obstacle.top)
        .fold(start.y.min(end.y), f64::min)
        - gap
        - lane_offset)
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

fn outer_lane_bridge(
    start: Point,
    end: Point,
    obstacles: &ObstacleIndex,
    gap: f64,
    lane_offset: f64,
    below: bool,
    preserved_bottom: f64,
) -> Option<Vec<Point>> {
    let lane_y = if below {
        obstacles
            .rects
            .iter()
            .map(|obstacle| obstacle.bottom)
            .fold(start.y.max(end.y), f64::max)
            .max(preserved_bottom)
            + gap
            + lane_offset
    } else {
        let top_lane = obstacles
            .rects
            .iter()
            .map(|obstacle| obstacle.top)
            .fold(start.y.min(end.y), f64::min)
            - gap
            - lane_offset;
        if top_lane >= 0.0 {
            top_lane
        } else {
            obstacles
                .rects
                .iter()
                .map(|obstacle| obstacle.bottom)
                .fold(start.y.max(end.y), f64::max)
                .max(preserved_bottom)
                + gap * 2.0
                + lane_offset
        }
    };
    let points = simplify_orthogonal(vec![
        start,
        Point {
            x: start.x,
            y: lane_y,
        },
        Point {
            x: end.x,
            y: lane_y,
        },
        end,
    ]);
    path_is_clear(&points, obstacles).then_some(points)
}

#[allow(clippy::too_many_arguments)]
fn boundary_bridge(
    start: Point,
    end: Point,
    obstacles: &ObstacleIndex,
    gap: f64,
    lane_offset: f64,
    below: bool,
    preserved_bottom: f64,
    prefer_direct: bool,
) -> Option<Vec<Point>> {
    if prefer_direct {
        obstacle_safe_bridge(start, end, obstacles, gap, lane_offset)
    } else {
        outer_lane_bridge(
            start,
            end,
            obstacles,
            gap,
            lane_offset,
            below,
            preserved_bottom,
        )
    }
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
        BoundaryTrunk, ExpansionWork, GroupCollapseOptions, GroupExpansion, GroupExpansionError,
        GroupExpansionOptions, ObstacleIndex, Rect, arrange_member_components, candidate_positions,
        collapse_group_in_place, expand_group_in_place, obstacle_safe_bridge, path_is_clear,
    };
    use crate::{
        BoundaryBundleConstraint, BoundaryBundleMemberConstraint, Edge, EdgeGeometry, Endpoint,
        Graph, Layout, LayoutConstraints, LayoutOptions, Node, NodeGeometry, Point, Port, PortSide,
        QualityEffort, layout, layout_with_constraints,
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
    fn collapse_restores_anchor_without_moving_retained_geometry() {
        let (compact, expanded, expansion) = fixture();
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expanded_layout = expand_group_in_place(
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

        let collapsed = collapse_group_in_place(
            &expanded,
            &expanded_layout,
            &compact,
            &expansion,
            &GroupCollapseOptions::default(),
        )
        .unwrap();

        for retained in [1, 4, 5] {
            assert_eq!(
                collapsed.nodes.iter().find(|node| node.id == retained),
                expanded_layout
                    .nodes
                    .iter()
                    .find(|node| node.id == retained)
            );
        }
        assert_eq!(
            collapsed.edges.iter().find(|edge| edge.id == 3),
            expanded_layout.edges.iter().find(|edge| edge.id == 3)
        );
        assert!(collapsed.nodes.iter().any(|node| node.id == 10));
        assert!(
            collapsed
                .nodes
                .iter()
                .all(|node| ![2, 3].contains(&node.id))
        );
        assert_eq!(collapsed.nodes.len(), compact.nodes.len());
        assert_eq!(collapsed.edges.len(), compact.edges.len());
        assert!(super::hard_geometry_is_clean(&compact, &collapsed));
        assert_eq!(super::ranking_direction_violations(&compact, &collapsed), 0);
    }

    #[test]
    fn collapse_is_deterministic_across_graph_and_contract_permutations() {
        let (compact, expanded, expansion) = fixture();
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let expanded_layout = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions::default(),
        )
        .unwrap();
        let expected = collapse_group_in_place(
            &expanded,
            &expanded_layout,
            &compact,
            &expansion,
            &GroupCollapseOptions::default(),
        )
        .unwrap();

        let mut permuted_compact = compact;
        permuted_compact.nodes.reverse();
        permuted_compact.edges.reverse();
        let mut permuted_expanded = expanded;
        permuted_expanded.nodes.reverse();
        permuted_expanded.edges.reverse();
        let mut permuted_layout = expanded_layout;
        permuted_layout.nodes.reverse();
        permuted_layout.edges.reverse();
        let mut permuted_expansion = expansion;
        permuted_expansion.members.reverse();
        permuted_expansion.boundary_trunks.reverse();
        let actual = collapse_group_in_place(
            &permuted_expanded,
            &permuted_layout,
            &permuted_compact,
            &permuted_expansion,
            &GroupCollapseOptions::default(),
        )
        .unwrap();

        assert_eq!(actual, expected);
    }

    #[test]
    fn collapsing_one_group_preserves_another_expanded_group() {
        let mut anchor_a = node(10);
        anchor_a.width = 260.0;
        let mut anchor_b = node(30);
        anchor_b.width = 260.0;
        let base = Graph {
            nodes: vec![
                node(1),
                anchor_a.clone(),
                node(20),
                anchor_b.clone(),
                node(40),
            ],
            edges: vec![
                edge(1, 1, 10, 100),
                edge(2, 10, 20, 200),
                edge(3, 20, 30, 300),
                edge(4, 30, 40, 400),
            ],
        };
        let group_a = GroupExpansion {
            anchor: 10,
            members: vec![11, 12],
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
        };
        let group_b = GroupExpansion {
            anchor: 30,
            members: vec![31, 32],
            boundary_trunks: vec![
                BoundaryTrunk {
                    expanded_edge: 31,
                    compact_edge: 3,
                },
                BoundaryTrunk {
                    expanded_edge: 33,
                    compact_edge: 4,
                },
            ],
        };
        let expanded_a = Graph {
            nodes: vec![node(1), node(11), node(12), node(20), anchor_b, node(40)],
            edges: vec![
                edge(11, 1, 11, 100),
                edge(12, 11, 12, 150),
                edge(13, 12, 20, 200),
                edge(3, 20, 30, 300),
                edge(4, 30, 40, 400),
            ],
        };
        let expanded_both = Graph {
            nodes: vec![
                node(1),
                node(11),
                node(12),
                node(20),
                node(31),
                node(32),
                node(40),
            ],
            edges: vec![
                edge(11, 1, 11, 100),
                edge(12, 11, 12, 150),
                edge(13, 12, 20, 200),
                edge(31, 20, 31, 300),
                edge(32, 31, 32, 350),
                edge(33, 32, 40, 400),
            ],
        };
        let only_b = Graph {
            nodes: vec![node(1), anchor_a, node(20), node(31), node(32), node(40)],
            edges: vec![
                edge(1, 1, 10, 100),
                edge(2, 10, 20, 200),
                edge(31, 20, 31, 300),
                edge(32, 31, 32, 350),
                edge(33, 32, 40, 400),
            ],
        };
        let options = GroupExpansionOptions {
            quality_effort: QualityEffort::Max,
            ..GroupExpansionOptions::default()
        };
        let base_layout = layout(&base, LayoutOptions::default()).unwrap();
        let layout_a =
            expand_group_in_place(&base, &base_layout, &expanded_a, &group_a, &options).unwrap();
        let layout_both =
            expand_group_in_place(&expanded_a, &layout_a, &expanded_both, &group_b, &options)
                .unwrap();
        let collapsed_a = collapse_group_in_place(
            &expanded_both,
            &layout_both,
            &only_b,
            &group_a,
            &GroupCollapseOptions::default(),
        )
        .unwrap();

        for retained in [20, 31, 32, 40] {
            assert_eq!(
                collapsed_a.nodes.iter().find(|node| node.id == retained),
                layout_both.nodes.iter().find(|node| node.id == retained),
            );
        }
        assert_eq!(
            collapsed_a.edges.iter().find(|edge| edge.id == 32),
            layout_both.edges.iter().find(|edge| edge.id == 32),
        );
        assert!(collapsed_a.nodes.iter().any(|node| node.id == 10));
        assert!(
            collapsed_a
                .nodes
                .iter()
                .all(|node| ![11, 12].contains(&node.id))
        );
        assert!(collapsed_a.nodes.iter().any(|node| node.id == 31));
        assert!(collapsed_a.nodes.iter().any(|node| node.id == 32));
    }

    #[test]
    fn wider_expansion_opens_a_corridor_and_splits_collapsed_boundary_cohorts() {
        let mut anchor = node(10);
        anchor.width = 80.0;
        let compact = Graph {
            nodes: vec![node(1), anchor, node(4), node(5)],
            edges: vec![
                edge(1, 1, 10, 100),
                edge(2, 10, 4, 200),
                edge(3, 10, 5, 201),
            ],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(4), node(5)],
            edges: vec![
                edge(11, 1, 2, 100),
                edge(12, 2, 3, 150),
                edge(13, 2, 4, 200),
                edge(14, 3, 5, 201),
            ],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let compact_anchor = compact_layout
            .nodes
            .iter()
            .find(|node| node.id == 10)
            .unwrap();
        let compact_outputs = [4, 5].map(|id| {
            compact_layout
                .nodes
                .iter()
                .find(|node| node.id == id)
                .unwrap()
                .clone()
        });
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
                        expanded_edge: 13,
                        compact_edge: 2,
                    },
                    BoundaryTrunk {
                        expanded_edge: 14,
                        compact_edge: 3,
                    },
                ],
            },
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();

        let first_member = result.nodes.iter().find(|node| node.id == 2).unwrap();
        assert_eq!(first_member.x, compact_anchor.x);
        assert_eq!(first_member.y, compact_anchor.y);
        for compact_output in compact_outputs {
            let expanded_output = result
                .nodes
                .iter()
                .find(|node| node.id == compact_output.id)
                .unwrap();
            assert!(expanded_output.x > compact_output.x);
            assert_eq!(expanded_output.y, compact_output.y);
        }
        assert!(result.width > compact_layout.width);
        assert!(super::hard_geometry_is_clean(&expanded, &result));
        assert_eq!(super::ranking_direction_violations(&expanded, &result), 0);
    }

    #[test]
    fn taller_expansion_reflows_only_the_obstructing_local_row() {
        let compact = Graph {
            nodes: vec![node(1), node(2), node(10), node(20), node(30), node(40)],
            edges: vec![
                edge(1, 1, 10, 100),
                edge(2, 10, 20, 200),
                edge(3, 2, 30, 300),
                edge(4, 30, 40, 400),
            ],
        };
        let expanded = Graph {
            nodes: vec![
                node(1),
                node(2),
                node(11),
                node(12),
                node(13),
                node(14),
                node(20),
                node(30),
                node(40),
            ],
            edges: vec![
                edge(11, 1, 11, 100),
                edge(12, 11, 20, 200),
                edge(3, 2, 30, 300),
                edge(4, 30, 40, 400),
            ],
        };
        let compact_layout = layout(&compact, LayoutOptions::default()).unwrap();
        let compact_nodes = compact_layout
            .nodes
            .iter()
            .map(|node| (node.id, node.clone()))
            .collect::<BTreeMap<_, _>>();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &GroupExpansion {
                anchor: 10,
                members: vec![11, 12, 13, 14],
                boundary_trunks: vec![
                    BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    },
                    BoundaryTrunk {
                        expanded_edge: 12,
                        compact_edge: 2,
                    },
                ],
            },
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        let result_nodes = result
            .nodes
            .iter()
            .map(|node| (node.id, node))
            .collect::<BTreeMap<_, _>>();
        let member_left = [11, 12, 13, 14]
            .into_iter()
            .map(|id| result_nodes[&id].x)
            .fold(f64::INFINITY, f64::min);
        let member_top = [11, 12, 13, 14]
            .into_iter()
            .map(|id| result_nodes[&id].y)
            .fold(f64::INFINITY, f64::min);

        assert_eq!(member_left, compact_nodes[&10].x);
        assert_eq!(member_top, compact_nodes[&10].y);
        assert!(result_nodes[&30].y > compact_nodes[&30].y);
        for retained in [1, 2] {
            assert_eq!(result_nodes[&retained], &compact_nodes[&retained]);
        }
        for retained in [20, 40] {
            assert_eq!(result_nodes[&retained].y, compact_nodes[&retained].y);
            assert!(result_nodes[&retained].x > compact_nodes[&retained].x);
        }
        assert_ne!(
            result.edges.iter().find(|edge| edge.id == 3),
            compact_layout.edges.iter().find(|edge| edge.id == 3),
        );
        assert_ne!(
            result.edges.iter().find(|edge| edge.id == 4),
            compact_layout.edges.iter().find(|edge| edge.id == 4),
        );
        assert!(super::hard_geometry_is_clean(&expanded, &result));
        assert_eq!(super::ranking_direction_violations(&expanded, &result), 0);

        let mut permuted_compact = compact;
        permuted_compact.nodes.reverse();
        permuted_compact.edges.reverse();
        let mut permuted_expanded = expanded;
        permuted_expanded.nodes.reverse();
        permuted_expanded.edges.reverse();
        assert_eq!(
            expand_group_in_place(
                &permuted_compact,
                &compact_layout,
                &permuted_expanded,
                &GroupExpansion {
                    anchor: 10,
                    members: vec![14, 13, 12, 11],
                    boundary_trunks: vec![
                        BoundaryTrunk {
                            expanded_edge: 12,
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
            .unwrap(),
            result,
        );
    }

    #[test]
    fn positive_clearance_preserves_an_in_place_group_expansion() {
        let (compact, expanded, expansion) = fixture();
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            ..LayoutOptions::default()
        };
        let compact_layout = layout(&compact, options).unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: options,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        assert!(crate::candidate_satisfies_edge_node_clearance(
            &crate::validation::validate_and_index(&expanded, options).unwrap(),
            &result,
            options,
            &mut false,
        ));
    }

    #[test]
    fn positive_parallel_wire_spacing_preserves_an_in_place_group_expansion() {
        let (compact, expanded, expansion) = fixture();
        let options = LayoutOptions {
            minimum_parallel_wire_spacing: 6.0,
            ..LayoutOptions::default()
        };
        let compact_layout = layout(&compact, options).unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: options,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        assert!(
            crate::routing::route_family_satisfies_parallel_spacing_bounded(
                &crate::validation::validate_and_index(&expanded, options).unwrap(),
                &result.edges,
                options.minimum_parallel_wire_spacing,
                crate::outward_obstacle_clearance_stub(options),
                crate::MAX_LAYOUT_PARALLEL_WIRE_SPACING_SEGMENTS,
                crate::MAX_LAYOUT_PARALLEL_WIRE_SPACING_VISITS,
            )
            .unwrap()
        );
    }

    #[test]
    fn boundary_bundle_geometry_and_taps_survive_an_in_place_group_expansion() {
        let (compact, expanded, expansion) = fixture();
        let compact_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: Vec::new(),
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 8,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 1,
                    slots: (0..8).collect(),
                }],
            }],
        };
        let expanded_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: Vec::new(),
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 8,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 11,
                    slots: (0..8).collect(),
                }],
            }],
        };
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            minimum_parallel_wire_spacing: 6.0,
            ..LayoutOptions::default()
        };
        let compact_layout =
            layout_with_constraints(&compact, options, &compact_constraints).unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: options,
                quality_effort: QualityEffort::Max,
                constraints: expanded_constraints,
            },
        )
        .unwrap();

        assert_eq!(result.boundary_bundles.len(), 1);
        let compact_bundle = &compact_layout.boundary_bundles[0];
        let expanded_bundle = &result.boundary_bundles[0];
        assert_eq!(expanded_bundle.id, compact_bundle.id);
        assert_eq!(expanded_bundle.endpoint, compact_bundle.endpoint);
        assert_eq!(expanded_bundle.collector, compact_bundle.collector);
        assert_eq!(expanded_bundle.spine, compact_bundle.spine);
        assert_eq!(expanded_bundle.members.len(), 1);
        assert_eq!(expanded_bundle.members[0].edge, 11);
        assert_eq!(
            expanded_bundle.members[0].tap,
            compact_bundle.members[0].tap
        );
        assert_eq!(
            result
                .edges
                .iter()
                .find(|route| route.id == 11)
                .unwrap()
                .points[0],
            compact_bundle.members[0].tap
        );
    }

    #[test]
    fn malformed_disconnected_compact_bundle_tap_fails_closed() {
        let (compact, expanded, expansion) = fixture();
        let compact_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: Vec::new(),
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 8,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 1,
                    slots: (0..8).collect(),
                }],
            }],
        };
        let expanded_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: Vec::new(),
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 8,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 11,
                    slots: (0..8).collect(),
                }],
            }],
        };
        let options = LayoutOptions::default();
        let mut compact_layout =
            layout_with_constraints(&compact, options, &compact_constraints).unwrap();
        compact_layout.boundary_bundles[0].members[0].tap.x += 1.0;
        compact_layout
            .edges
            .iter_mut()
            .find(|route| route.id == 1)
            .unwrap()
            .points[0]
            .x += 1.0;

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &GroupExpansionOptions {
                    layout: options,
                    constraints: expanded_constraints,
                    ..GroupExpansionOptions::default()
                },
            ),
            Err(GroupExpansionError::NeedsFullRelayout)
        );
    }

    #[test]
    fn four_lane_bundle_split_allocates_deterministic_local_taps() {
        let mut anchor = node(10);
        anchor.width = 260.0;
        let compact = Graph {
            nodes: vec![node(1), anchor],
            edges: vec![edge(1, 1, 10, 100)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3)],
            edges: vec![edge(11, 1, 2, 101), edge(12, 1, 3, 102)],
        };
        let compact_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: Vec::new(),
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 4,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 1,
                    slots: vec![0, 1, 2, 3],
                }],
            }],
        };
        let expanded_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: Vec::new(),
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 1, port: 1 },
                width: 4,
                members: vec![
                    BoundaryBundleMemberConstraint {
                        edge: 11,
                        slots: vec![0, 1],
                    },
                    BoundaryBundleMemberConstraint {
                        edge: 12,
                        slots: vec![2, 3],
                    },
                ],
            }],
        };
        let options = LayoutOptions::default();
        let compact_layout =
            layout_with_constraints(&compact, options, &compact_constraints).unwrap();
        let expansion = GroupExpansion {
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
        };
        let expected = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: options,
                quality_effort: QualityEffort::Max,
                constraints: expanded_constraints.clone(),
            },
        )
        .unwrap();
        let mut expanded_permuted = expanded.clone();
        expanded_permuted.nodes.reverse();
        expanded_permuted.edges.reverse();
        let mut constraints_permuted = expanded_constraints;
        constraints_permuted.boundary_bundles[0].members.reverse();
        let actual = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded_permuted,
            &GroupExpansion {
                anchor: 10,
                members: vec![3, 2],
                boundary_trunks: vec![
                    BoundaryTrunk {
                        expanded_edge: 12,
                        compact_edge: 1,
                    },
                    BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    },
                ],
            },
            &GroupExpansionOptions {
                layout: options,
                quality_effort: QualityEffort::Max,
                constraints: constraints_permuted,
            },
        )
        .unwrap();

        assert_eq!(actual, expected);
        let bundle = &expected.boundary_bundles[0];
        assert_eq!(bundle.members.len(), 2);
        assert_ne!(bundle.members[0].tap, bundle.members[1].tap);
        assert_eq!(
            bundle.members[1].tap.y - bundle.members[0].tap.y,
            options.route_lane_gap
        );
        for member in &bundle.members {
            assert_eq!(
                expected
                    .edges
                    .iter()
                    .find(|route| route.id == member.edge)
                    .unwrap()
                    .points
                    .first(),
                Some(&member.tap)
            );
        }
    }

    #[test]
    fn output_boundary_bundle_tap_survives_an_in_place_group_expansion() {
        let mut anchor = node(10);
        anchor.width = 260.0;
        let mut output = node(4);
        output.ports[0].offset = 50.0 / 3.0;
        let compact = Graph {
            nodes: vec![node(1), anchor, output.clone()],
            edges: vec![edge(1, 1, 10, 100), edge(2, 10, 4, 200)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), output],
            edges: vec![
                edge(11, 1, 2, 100),
                edge(12, 2, 3, 150),
                edge(13, 3, 4, 200),
            ],
        };
        let compact_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![4],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 4, port: 0 },
                width: 8,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 2,
                    slots: (0..8).collect(),
                }],
            }],
        };
        let expanded_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![4],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 7,
                endpoint: Endpoint { node: 4, port: 0 },
                width: 8,
                members: vec![BoundaryBundleMemberConstraint {
                    edge: 13,
                    slots: (0..8).collect(),
                }],
            }],
        };
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            minimum_parallel_wire_spacing: 6.0,
            ..LayoutOptions::default()
        };
        let compact_layout =
            layout_with_constraints(&compact, options, &compact_constraints).unwrap();
        let compact_layout: Layout =
            serde_json::from_str(&serde_json::to_string(&compact_layout).unwrap()).unwrap();
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
                        expanded_edge: 13,
                        compact_edge: 2,
                    },
                ],
            },
            &GroupExpansionOptions {
                layout: options,
                quality_effort: QualityEffort::Max,
                constraints: expanded_constraints,
            },
        )
        .unwrap();

        let compact_bundle = &compact_layout.boundary_bundles[0];
        let expanded_bundle = &result.boundary_bundles[0];
        assert_eq!(expanded_bundle.collector, compact_bundle.collector);
        assert_eq!(expanded_bundle.spine, compact_bundle.spine);
        assert_eq!(expanded_bundle.members[0].edge, 13);
        assert_eq!(
            expanded_bundle.members[0].tap,
            compact_bundle.members[0].tap
        );
        assert_eq!(
            result
                .edges
                .iter()
                .find(|route| route.id == 13)
                .unwrap()
                .points
                .last(),
            Some(&compact_bundle.members[0].tap)
        );
    }

    #[test]
    fn unrelated_boundary_bundle_geometry_and_route_remain_byte_for_byte_unchanged() {
        let (compact, expanded, expansion) = fixture();
        let compact_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![5],
            boundary_bundles: vec![
                BoundaryBundleConstraint {
                    id: 7,
                    endpoint: Endpoint { node: 1, port: 1 },
                    width: 8,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 1,
                        slots: (0..8).collect(),
                    }],
                },
                BoundaryBundleConstraint {
                    id: 8,
                    endpoint: Endpoint { node: 5, port: 0 },
                    width: 4,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 3,
                        slots: (0..4).collect(),
                    }],
                },
            ],
        };
        let expanded_constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![5],
            boundary_bundles: vec![
                BoundaryBundleConstraint {
                    id: 7,
                    endpoint: Endpoint { node: 1, port: 1 },
                    width: 8,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 11,
                        slots: (0..8).collect(),
                    }],
                },
                BoundaryBundleConstraint {
                    id: 8,
                    endpoint: Endpoint { node: 5, port: 0 },
                    width: 4,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 3,
                        slots: (0..4).collect(),
                    }],
                },
            ],
        };
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            minimum_parallel_wire_spacing: 6.0,
            ..LayoutOptions::default()
        };
        let compact_layout =
            layout_with_constraints(&compact, options, &compact_constraints).unwrap();
        let result = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: options,
                quality_effort: QualityEffort::Max,
                constraints: expanded_constraints,
            },
        )
        .unwrap();

        assert_eq!(
            result.boundary_bundles.iter().find(|bundle| bundle.id == 8),
            compact_layout
                .boundary_bundles
                .iter()
                .find(|bundle| bundle.id == 8)
        );
        assert_eq!(
            result.edges.iter().find(|edge| edge.id == 3),
            compact_layout.edges.iter().find(|edge| edge.id == 3)
        );
    }

    #[test]
    fn changed_bundle_lane_count_fails_closed_before_moving_a_later_same_role_bundle() {
        let mut anchor = node(10);
        anchor.width = 260.0;
        let compact = Graph {
            nodes: vec![node(1), node(6), anchor, node(7)],
            edges: vec![edge(1, 1, 10, 100), edge(2, 6, 7, 200)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2), node(3), node(6), node(7)],
            edges: vec![edge(11, 1, 2, 100), edge(12, 1, 3, 100), edge(2, 6, 7, 200)],
        };
        let compact_constraints = LayoutConstraints {
            inputs: vec![1, 6],
            outputs: Vec::new(),
            boundary_bundles: vec![
                BoundaryBundleConstraint {
                    id: 7,
                    endpoint: Endpoint { node: 1, port: 1 },
                    width: 4,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 1,
                        slots: vec![0, 1, 2, 3],
                    }],
                },
                BoundaryBundleConstraint {
                    id: 8,
                    endpoint: Endpoint { node: 6, port: 1 },
                    width: 1,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 2,
                        slots: vec![0],
                    }],
                },
            ],
        };
        let unchanged_lane_constraints = LayoutConstraints {
            inputs: vec![1, 6],
            outputs: Vec::new(),
            boundary_bundles: vec![
                BoundaryBundleConstraint {
                    id: 7,
                    endpoint: Endpoint { node: 1, port: 1 },
                    width: 4,
                    members: vec![
                        BoundaryBundleMemberConstraint {
                            edge: 11,
                            slots: vec![0, 1, 2, 3],
                        },
                        BoundaryBundleMemberConstraint {
                            edge: 12,
                            slots: vec![0, 1, 2, 3],
                        },
                    ],
                },
                compact_constraints.boundary_bundles[1].clone(),
            ],
        };
        let split_lane_constraints = LayoutConstraints {
            inputs: vec![1, 6],
            outputs: Vec::new(),
            boundary_bundles: vec![
                BoundaryBundleConstraint {
                    id: 7,
                    endpoint: Endpoint { node: 1, port: 1 },
                    width: 4,
                    members: vec![
                        BoundaryBundleMemberConstraint {
                            edge: 11,
                            slots: vec![0, 1],
                        },
                        BoundaryBundleMemberConstraint {
                            edge: 12,
                            slots: vec![2, 3],
                        },
                    ],
                },
                compact_constraints.boundary_bundles[1].clone(),
            ],
        };
        let options = LayoutOptions::default();
        let compact_layout =
            layout_with_constraints(&compact, options, &compact_constraints).unwrap();
        let expansion = GroupExpansion {
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
        };

        let unchanged = expand_group_in_place(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            &GroupExpansionOptions {
                layout: options,
                quality_effort: QualityEffort::Max,
                constraints: unchanged_lane_constraints,
            },
        )
        .unwrap();
        assert_eq!(
            unchanged
                .boundary_bundles
                .iter()
                .find(|bundle| bundle.id == 8),
            compact_layout
                .boundary_bundles
                .iter()
                .find(|bundle| bundle.id == 8)
        );
        assert_eq!(
            unchanged.edges.iter().find(|route| route.id == 2),
            compact_layout.edges.iter().find(|route| route.id == 2)
        );

        let expected_error = Err(GroupExpansionError::NeedsFullRelayout);
        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &expansion,
                &GroupExpansionOptions {
                    layout: options,
                    quality_effort: QualityEffort::Max,
                    constraints: split_lane_constraints.clone(),
                },
            ),
            expected_error
        );
        let mut permuted_graph = expanded.clone();
        permuted_graph.nodes.reverse();
        permuted_graph.edges.reverse();
        let mut permuted_constraints = split_lane_constraints;
        permuted_constraints.inputs.reverse();
        permuted_constraints.boundary_bundles.reverse();
        for bundle in &mut permuted_constraints.boundary_bundles {
            bundle.members.reverse();
            for member in &mut bundle.members {
                member.slots.reverse();
            }
        }
        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &permuted_graph,
                &GroupExpansion {
                    anchor: 10,
                    members: vec![3, 2],
                    boundary_trunks: vec![
                        BoundaryTrunk {
                            expanded_edge: 12,
                            compact_edge: 1,
                        },
                        BoundaryTrunk {
                            expanded_edge: 11,
                            compact_edge: 1,
                        },
                    ],
                },
                &GroupExpansionOptions {
                    layout: options,
                    quality_effort: QualityEffort::Max,
                    constraints: permuted_constraints,
                },
            ),
            Err(GroupExpansionError::NeedsFullRelayout)
        );
    }

    #[test]
    fn boundary_bundle_endpoint_change_requires_full_relayout() {
        let mut anchor = node(10);
        anchor.width = 260.0;
        let compact = Graph {
            nodes: vec![node(1), anchor],
            edges: vec![edge(1, 1, 10, 100)],
        };
        let expanded = Graph {
            nodes: vec![node(1), node(2)],
            edges: vec![edge(11, 1, 2, 100)],
        };
        let options = LayoutOptions {
            edge_node_clearance: 20.0,
            minimum_parallel_wire_spacing: 6.0,
            ..LayoutOptions::default()
        };
        let compact_layout = layout_with_constraints(
            &compact,
            options,
            &LayoutConstraints {
                inputs: vec![1],
                outputs: vec![10],
                boundary_bundles: vec![BoundaryBundleConstraint {
                    id: 7,
                    endpoint: Endpoint { node: 10, port: 0 },
                    width: 8,
                    members: vec![BoundaryBundleMemberConstraint {
                        edge: 1,
                        slots: (0..8).collect(),
                    }],
                }],
            },
        )
        .unwrap();

        assert_eq!(
            expand_group_in_place(
                &compact,
                &compact_layout,
                &expanded,
                &GroupExpansion {
                    anchor: 10,
                    members: vec![2],
                    boundary_trunks: vec![BoundaryTrunk {
                        expanded_edge: 11,
                        compact_edge: 1,
                    }],
                },
                &GroupExpansionOptions {
                    layout: options,
                    quality_effort: QualityEffort::Max,
                    constraints: LayoutConstraints {
                        inputs: vec![1],
                        outputs: vec![2],
                        boundary_bundles: vec![BoundaryBundleConstraint {
                            id: 7,
                            endpoint: Endpoint { node: 2, port: 0 },
                            width: 8,
                            members: vec![BoundaryBundleMemberConstraint {
                                edge: 11,
                                slots: (0..8).collect(),
                            }],
                        }],
                    },
                },
            ),
            Err(GroupExpansionError::NeedsFullRelayout)
        );
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

        let contract = super::validate_contract(
            &compact,
            &compact_layout,
            &expanded,
            &expansion,
            LayoutOptions::default(),
        )
        .unwrap();
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
    fn wider_left_to_right_expansion_opens_a_horizontal_corridor() {
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
        let compact_output = compact_layout
            .nodes
            .iter()
            .find(|node| node.id == 4)
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
                        expanded_edge: 13,
                        compact_edge: 2,
                    },
                ],
            },
            &GroupExpansionOptions {
                quality_effort: QualityEffort::Max,
                ..GroupExpansionOptions::default()
            },
        )
        .unwrap();
        let first_member = result.nodes.iter().find(|node| node.id == 2).unwrap();
        let second_member = result.nodes.iter().find(|node| node.id == 3).unwrap();
        let expanded_output = result.nodes.iter().find(|node| node.id == 4).unwrap();
        assert!(first_member.x < second_member.x);
        assert!(second_member.x < expanded_output.x);
        assert!(expanded_output.x > compact_output.x);
        assert!(result.width > compact_layout.width);
        assert!(super::hard_geometry_is_clean(&expanded, &result));
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
                boundary_bundles: Vec::new(),
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
                    boundary_bundles: Vec::new(),
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
                boundary_bundles: Vec::new(),
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
                    boundary_bundles: Vec::new(),
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
                        boundary_bundles: Vec::new(),
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
            boundary_bundles: Vec::new(),
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
                        boundary_bundles: Vec::new(),
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
            boundary_bundles: Vec::new(),
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
            0.0,
        )
        .unwrap();
        assert!(path.len() >= 4);
        assert!(path_is_clear(&path, &obstacles));
    }

    #[test]
    fn disconnected_members_use_a_deterministic_grid_above_the_height_limit() {
        let graph = Graph {
            nodes: (1..=64).map(node).collect(),
            edges: Vec::new(),
        };
        let ordinary = layout(&graph, LayoutOptions::default()).unwrap();
        let packed = arrange_member_components(
            &graph,
            &ordinary,
            super::EXPANSION_COMPONENT_GAP,
            super::EXPANSION_COMPONENT_GAP,
            500.0,
            None,
        );
        let mut columns = packed.nodes.iter().map(|node| node.x).collect::<Vec<_>>();
        columns.sort_unstable_by(f64::total_cmp);
        columns.dedup();
        assert_eq!(columns.len(), 7);
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
            arrange_member_components(
                &permuted,
                &permuted_layout,
                super::EXPANSION_COMPONENT_GAP,
                super::EXPANSION_COMPONENT_GAP,
                500.0,
                None,
            ),
            packed
        );
    }

    #[test]
    fn disconnected_members_stack_through_the_exact_one_point_five_x_limit() {
        let graph = Graph {
            nodes: (1..=3).map(node).collect(),
            edges: Vec::new(),
        };
        let member_layout = Layout {
            nodes: (0..3)
                .map(|index| NodeGeometry {
                    id: index + 1,
                    x: index as f64 * 100.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                })
                .collect(),
            edges: Vec::new(),
            boundary_bundles: Vec::new(),
            width: 280.0,
            height: 50.0,
        };
        let gap = 10.0;
        let stacked_height = 170.0;
        let stacked = arrange_member_components(
            &graph,
            &member_layout,
            gap,
            gap,
            stacked_height / super::EXPANSION_STACK_HEIGHT_FACTOR,
            None,
        );
        assert_eq!(stacked.width, 80.0);
        assert_eq!(stacked.height, stacked_height);
        assert!(stacked.nodes.iter().all(|node| node.x == 0.0));
        assert_eq!(
            stacked.nodes.iter().map(|node| node.y).collect::<Vec<_>>(),
            vec![0.0, 60.0, 120.0]
        );

        let grid = arrange_member_components(
            &graph,
            &member_layout,
            gap,
            gap,
            (stacked_height - 1.0) / super::EXPANSION_STACK_HEIGHT_FACTOR,
            None,
        );
        assert_eq!(grid.width, 170.0);
        assert_eq!(grid.height, 110.0);
        assert_eq!(
            grid.nodes
                .iter()
                .map(|node| (node.x, node.y))
                .collect::<Vec<_>>(),
            vec![(0.0, 0.0), (90.0, 0.0), (0.0, 60.0)]
        );
    }

    #[test]
    fn fixed_boundary_corridor_prevents_an_unroutable_multi_column_grid() {
        let graph = Graph {
            nodes: (1..=3).map(node).collect(),
            edges: Vec::new(),
        };
        let member_layout = Layout {
            nodes: (0..3)
                .map(|index| NodeGeometry {
                    id: index + 1,
                    x: index as f64 * 100.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                })
                .collect(),
            edges: Vec::new(),
            boundary_bundles: Vec::new(),
            width: 280.0,
            height: 50.0,
        };

        let arranged =
            arrange_member_components(&graph, &member_layout, 18.0, 18.0, 50.0, Some(175.0));
        assert!(arranged.nodes.iter().all(|node| node.x == 0.0));
        assert_eq!(arranged.width, 80.0);
        assert_eq!(arranged.height, 186.0);
    }

    #[test]
    fn component_arrangement_preserves_connected_left_to_right_geometry() {
        let graph = Graph {
            nodes: (1..=3).map(node).collect(),
            edges: vec![edge(1, 1, 2, 1)],
        };
        let member_layout = Layout {
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
                    x: 100.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                },
                NodeGeometry {
                    id: 3,
                    x: 220.0,
                    y: 0.0,
                    width: 80.0,
                    height: 50.0,
                },
            ],
            edges: vec![EdgeGeometry {
                id: 1,
                points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 100.0, y: 25.0 }],
            }],
            boundary_bundles: Vec::new(),
            width: 300.0,
            height: 50.0,
        };

        let arranged = arrange_member_components(&graph, &member_layout, 18.0, 18.0, 200.0, None);
        let nodes = arranged
            .nodes
            .iter()
            .map(|node| (node.id, node))
            .collect::<BTreeMap<_, _>>();
        assert_eq!(nodes[&2].x - nodes[&1].x, 100.0);
        assert_eq!(nodes[&1].y, nodes[&2].y);
        assert_eq!(nodes[&3].x, 0.0);
        assert!(nodes[&3].y > nodes[&1].y);
        assert_eq!(
            arranged.edges[0].points,
            vec![Point { x: 80.0, y: 25.0 }, Point { x: 100.0, y: 25.0 }]
        );
    }

    #[test]
    fn expansion_candidate_search_has_effort_and_total_work_bounds() {
        let compact = Layout {
            nodes: Vec::new(),
            edges: Vec::new(),
            boundary_bundles: Vec::new(),
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
            boundary_bundles: Vec::new(),
            width: 300.0,
            height: 200.0,
        };
        let options = LayoutOptions::default();
        let small = ExpansionWork {
            nodes: 100,
            edges: 200,
            boundary_edges: 20,
            projected_segments: 1_600,
            boundary_bundles: 0,
            boundary_bundle_members: 0,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                small,
                options,
                QualityEffort::Fast,
                0,
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
                0,
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
                0,
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
            boundary_bundles: 0,
            boundary_bundle_members: 0,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                maximum,
                options,
                QualityEffort::Max,
                0,
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
            boundary_bundles: 0,
            boundary_bundle_members: 0,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                over_budget,
                options,
                QualityEffort::Max,
                0,
            ),
            Err(GroupExpansionError::ExpansionWorkLimitExceeded {
                required: 123_222_294,
                maximum: super::MAX_CANDIDATE_WORK,
            })
        );

        let parallel_over_budget = ExpansionWork {
            nodes: 10,
            edges: 10,
            boundary_edges: 0,
            projected_segments: 4_000,
            boundary_bundles: 0,
            boundary_bundle_members: 0,
        };
        assert_eq!(
            candidate_positions(
                &compact,
                &anchor,
                &members,
                parallel_over_budget,
                LayoutOptions {
                    minimum_parallel_wire_spacing: 6.0,
                    ..options
                },
                QualityEffort::Max,
                0,
            ),
            Err(GroupExpansionError::ExpansionWorkLimitExceeded {
                required: 384_108_060,
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
