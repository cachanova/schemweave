use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, HashSet},
};

use crate::{NetId, NodeGeometry, NodeId};

/// One finite, positive-length orthogonal route segment for clearance scoring.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdgeNodeSegment {
    pub net: NetId,
    pub horizontal: bool,
    pub fixed: f64,
    pub start: f64,
    pub end: f64,
}

/// Declares that a node is electrically related to a net and is therefore
/// exempt from that net's route-to-node clearance measurement.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct NetNodeRelation {
    pub net: NetId,
    pub node: NodeId,
}

/// Exact route-to-unrelated-node clearance statistics.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EdgeNodeClearance {
    /// Minimum axis-aligned clearance across every unrelated segment-node pair.
    pub minimum_clearance: Option<f64>,
    /// Unrelated pairs whose distance is strictly below the requested threshold.
    pub violations: usize,
}

/// Why bounded edge-to-node clearance measurement could not complete.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum EdgeNodeClearanceError {
    InvalidInput,
    WorkLimitExceeded,
}

/// Measure edge-to-unrelated-node clearance with an explicit pair-visit cap.
///
/// Exact contact at `threshold` is accepted. Returns
/// [`EdgeNodeClearanceError::WorkLimitExceeded`] before visiting more than
/// `max_pair_visits` segment-node candidates, including related pairs that are
/// subsequently exempted. Malformed geometry returns
/// [`EdgeNodeClearanceError::InvalidInput`]. Callers must
/// provide finite node rectangles and finite, positive-length orthogonal
/// segments with `start < end`, plus a finite nonnegative threshold.
#[doc(hidden)]
pub fn measure_edge_node_clearance_bounded(
    segments: &[EdgeNodeSegment],
    nodes: &[NodeGeometry],
    relations: &[NetNodeRelation],
    threshold: f64,
    max_pair_visits: usize,
) -> Result<EdgeNodeClearance, EdgeNodeClearanceError> {
    if !threshold.is_finite()
        || threshold < 0.0
        || segments.iter().any(|segment| {
            !segment.fixed.is_finite()
                || !segment.start.is_finite()
                || !segment.end.is_finite()
                || segment.start >= segment.end
        })
        || nodes.iter().any(|node| {
            !node.x.is_finite()
                || !node.y.is_finite()
                || !node.width.is_finite()
                || !node.height.is_finite()
                || node.width <= 0.0
                || node.height <= 0.0
                || !(node.x + node.width).is_finite()
                || !(node.y + node.height).is_finite()
        })
    {
        return Err(EdgeNodeClearanceError::InvalidInput);
    }
    if segments.is_empty() || nodes.is_empty() {
        return Ok(EdgeNodeClearance::default());
    }
    let related = relations
        .iter()
        .map(|relation| (relation.net, relation.node))
        .collect::<HashSet<_>>();
    let index = ClearanceIndex::new(nodes);
    let mut ordered_segments = segments.to_vec();
    ordered_segments.sort_by(compare_edge_node_segments);
    let mut remaining_pair_visits = max_pair_visits;
    let mut stack = Vec::with_capacity(index.height);
    let mut minimum = None::<f64>;
    let mut violations = 0usize;
    for segment in ordered_segments {
        index.visit_segment_candidates(
            segment,
            threshold,
            &related,
            &mut remaining_pair_visits,
            &mut stack,
            &mut minimum,
            &mut violations,
        )?;
    }
    Ok(EdgeNodeClearance {
        minimum_clearance: minimum,
        violations,
    })
}

fn compare_edge_node_segments(left: &EdgeNodeSegment, right: &EdgeNodeSegment) -> Ordering {
    left.net
        .cmp(&right.net)
        .then(left.horizontal.cmp(&right.horizontal))
        .then(left.fixed.total_cmp(&right.fixed))
        .then(left.start.total_cmp(&right.start))
        .then(left.end.total_cmp(&right.end))
}

#[derive(Clone, Copy)]
struct Aabb {
    left: f64,
    top: f64,
    right: f64,
    bottom: f64,
}

impl Aabb {
    fn from_node(node: &NodeGeometry) -> Self {
        Self {
            left: node.x,
            top: node.y,
            right: node.x + node.width,
            bottom: node.y + node.height,
        }
    }

    fn from_segment(segment: EdgeNodeSegment) -> Self {
        if segment.horizontal {
            Self {
                left: segment.start,
                top: segment.fixed,
                right: segment.end,
                bottom: segment.fixed,
            }
        } else {
            Self {
                left: segment.fixed,
                top: segment.start,
                right: segment.fixed,
                bottom: segment.end,
            }
        }
    }

    fn union(self, other: Self) -> Self {
        Self {
            left: self.left.min(other.left),
            top: self.top.min(other.top),
            right: self.right.max(other.right),
            bottom: self.bottom.max(other.bottom),
        }
    }

    fn distance(self, other: Self) -> f64 {
        let dx = if self.right < other.left {
            other.left - self.right
        } else if self.left > other.right {
            self.left - other.right
        } else {
            0.0
        };
        let dy = if self.bottom < other.top {
            other.top - self.bottom
        } else if self.top > other.bottom {
            self.top - other.bottom
        } else {
            0.0
        };
        dx.max(dy)
    }
}

#[derive(Clone, Copy)]
struct IndexedNode {
    id: NodeId,
    bounds: Aabb,
}

#[derive(Clone, Copy)]
enum ClearanceBvhNode {
    Leaf {
        bounds: Aabb,
        node: usize,
    },
    Branch {
        bounds: Aabb,
        left: usize,
        right: usize,
    },
}

impl ClearanceBvhNode {
    fn bounds(self) -> Aabb {
        match self {
            Self::Leaf { bounds, .. } | Self::Branch { bounds, .. } => bounds,
        }
    }
}

struct ClearanceIndex {
    nodes: Vec<IndexedNode>,
    tree: Vec<ClearanceBvhNode>,
    root: usize,
    height: usize,
}

impl ClearanceIndex {
    fn new(nodes: &[NodeGeometry]) -> Self {
        let mut indexed = nodes
            .iter()
            .map(|node| IndexedNode {
                id: node.id,
                bounds: Aabb::from_node(node),
            })
            .collect::<Vec<_>>();
        let mut tree = Vec::with_capacity(indexed.len().saturating_mul(2).saturating_sub(1));
        let (root, height) = build_clearance_bvh(&mut indexed, 0, &mut tree);
        Self {
            nodes: indexed,
            tree,
            root,
            height,
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn visit_segment_candidates(
        &self,
        segment: EdgeNodeSegment,
        threshold: f64,
        related: &HashSet<(NetId, NodeId)>,
        remaining_pair_visits: &mut usize,
        stack: &mut Vec<usize>,
        minimum: &mut Option<f64>,
        violations: &mut usize,
    ) -> Result<(), EdgeNodeClearanceError> {
        let segment_bounds = Aabb::from_segment(segment);
        stack.clear();
        stack.push(self.root);
        while let Some(tree_index) = stack.pop() {
            let tree_node = self.tree[tree_index];
            let lower_bound = segment_bounds.distance(tree_node.bounds());
            if !clearance_subtree_can_contribute(lower_bound, threshold, *minimum) {
                continue;
            }
            match tree_node {
                ClearanceBvhNode::Leaf { node, .. } => {
                    *remaining_pair_visits = remaining_pair_visits
                        .checked_sub(1)
                        .ok_or(EdgeNodeClearanceError::WorkLimitExceeded)?;
                    let candidate = self.nodes[node];
                    if related.contains(&(segment.net, candidate.id)) {
                        continue;
                    }
                    let distance = segment_bounds.distance(candidate.bounds);
                    *minimum = Some(minimum.map_or(distance, |current| current.min(distance)));
                    if distance < threshold {
                        *violations = violations.saturating_add(1);
                    }
                }
                ClearanceBvhNode::Branch { left, right, .. } => {
                    let left_distance = segment_bounds.distance(self.tree[left].bounds());
                    let right_distance = segment_bounds.distance(self.tree[right].bounds());
                    let left_key = (FloatKey(left_distance), left);
                    let right_key = (FloatKey(right_distance), right);
                    if left_key <= right_key {
                        stack.push(right);
                        stack.push(left);
                    } else {
                        stack.push(left);
                        stack.push(right);
                    }
                }
            }
        }
        Ok(())
    }
}

fn clearance_subtree_can_contribute(
    lower_bound: f64,
    threshold: f64,
    minimum: Option<f64>,
) -> bool {
    lower_bound < threshold || minimum.is_none_or(|current| lower_bound < current)
}

fn build_clearance_bvh(
    nodes: &mut [IndexedNode],
    offset: usize,
    tree: &mut Vec<ClearanceBvhNode>,
) -> (usize, usize) {
    debug_assert!(!nodes.is_empty());
    let bounds = nodes
        .iter()
        .skip(1)
        .fold(nodes[0].bounds, |bounds, node| bounds.union(node.bounds));
    if nodes.len() == 1 {
        let index = tree.len();
        tree.push(ClearanceBvhNode::Leaf {
            bounds,
            node: offset,
        });
        return (index, 1);
    }

    let horizontal_span = bounds.right - bounds.left;
    let vertical_span = bounds.bottom - bounds.top;
    if horizontal_span.total_cmp(&vertical_span) != Ordering::Less {
        nodes.sort_by(|left, right| compare_indexed_nodes(left, right, true));
    } else {
        nodes.sort_by(|left, right| compare_indexed_nodes(left, right, false));
    }
    let middle = nodes.len() / 2;
    let (left_nodes, right_nodes) = nodes.split_at_mut(middle);
    let (left, left_height) = build_clearance_bvh(left_nodes, offset, tree);
    let (right, right_height) = build_clearance_bvh(right_nodes, offset + middle, tree);
    let index = tree.len();
    tree.push(ClearanceBvhNode::Branch {
        bounds,
        left,
        right,
    });
    (index, left_height.max(right_height) + 1)
}

fn compare_indexed_nodes(left: &IndexedNode, right: &IndexedNode, horizontal: bool) -> Ordering {
    let left_bounds = left.bounds;
    let right_bounds = right.bounds;
    let spatial = if horizontal {
        left_bounds
            .left
            .total_cmp(&right_bounds.left)
            .then(left_bounds.right.total_cmp(&right_bounds.right))
            .then(left_bounds.top.total_cmp(&right_bounds.top))
            .then(left_bounds.bottom.total_cmp(&right_bounds.bottom))
    } else {
        left_bounds
            .top
            .total_cmp(&right_bounds.top)
            .then(left_bounds.bottom.total_cmp(&right_bounds.bottom))
            .then(left_bounds.left.total_cmp(&right_bounds.left))
            .then(left_bounds.right.total_cmp(&right_bounds.right))
    };
    spatial.then(left.id.cmp(&right.id))
}

/// A same-net-unioned, axis-aligned physical wire segment.
///
/// This low-level type is public so the development-only evaluation crate can
/// use the exact same congestion sweep as the production candidate selector.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParallelSegment {
    pub net: NetId,
    pub horizontal: bool,
    pub fixed: f64,
    pub start: f64,
    pub end: f64,
}

/// Exact minimum separation between longitudinally overlapping parallel routes.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ParallelSeparation {
    /// Minimum separation, including collinear different-net overlap at zero.
    pub minimum: Option<f64>,
    /// Minimum strictly positive separation, preserving the development scorer metric.
    pub minimum_positive: Option<f64>,
}

/// Why bounded parallel-route separation measurement could not complete.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[non_exhaustive]
pub enum ParallelSeparationError {
    InvalidInput,
    WorkLimitExceeded,
}

/// Measure exact different-net parallel-route separation with deterministic work.
///
/// Only same-orientation segments with positive longitudinal overlap are comparable.
/// Perpendicular crossings and longitudinal endpoint-only contact are ignored. Same-net
/// segments are electrically related and therefore exempt. Callers must provide finite,
/// positive-length segments.
#[doc(hidden)]
pub fn measure_parallel_separation_bounded(
    segments: &[ParallelSegment],
    max_tree_visits: usize,
) -> Result<ParallelSeparation, ParallelSeparationError> {
    if segments.iter().any(|segment| {
        !segment.fixed.is_finite()
            || !segment.start.is_finite()
            || !segment.end.is_finite()
            || segment.start >= segment.end
    }) {
        return Err(ParallelSeparationError::InvalidInput);
    }

    let mut remaining_tree_visits = max_tree_visits;
    let mut result = ParallelSeparation::default();
    for horizontal in [true, false] {
        let mut oriented = segments
            .iter()
            .filter(|segment| segment.horizontal == horizontal)
            .map(|segment| ParallelSegment {
                fixed: canonical_zero(segment.fixed),
                start: canonical_zero(segment.start),
                end: canonical_zero(segment.end),
                ..*segment
            })
            .collect::<Vec<_>>();
        oriented.sort_unstable_by(|left, right| {
            left.fixed
                .total_cmp(&right.fixed)
                .then(left.start.total_cmp(&right.start))
                .then(left.end.total_cmp(&right.end))
                .then(left.net.cmp(&right.net))
        });
        if oriented.len() < 2 {
            continue;
        }
        if oriented.len() > remaining_tree_visits {
            return Err(ParallelSeparationError::WorkLimitExceeded);
        }
        let measured = parallel_separation_for_orientation(&oriented, &mut remaining_tree_visits)?;
        result.minimum = minimum_option(result.minimum, measured.minimum);
        result.minimum_positive =
            minimum_option(result.minimum_positive, measured.minimum_positive);
    }
    Ok(result)
}

fn parallel_separation_for_orientation(
    segments: &[ParallelSegment],
    remaining_tree_visits: &mut usize,
) -> Result<ParallelSeparation, ParallelSeparationError> {
    let mut fixed_values = segments
        .iter()
        .map(|segment| FloatKey(segment.fixed))
        .collect::<Vec<_>>();
    fixed_values.sort_unstable();
    fixed_values.dedup();
    let fixed_indices = segments
        .iter()
        .map(|segment| {
            fixed_values
                .binary_search(&FloatKey(segment.fixed))
                .expect("parallel coordinate exists")
        })
        .collect::<Vec<_>>();
    let mut events = Vec::with_capacity(segments.len() * 2);
    for (index, segment) in segments.iter().enumerate() {
        // End events precede starts so longitudinal endpoint-only contact is ignored.
        events.push((FloatKey(segment.end), 0u8, index));
        events.push((FloatKey(segment.start), 1u8, index));
    }
    events.sort_unstable();

    let mut active = vec![BTreeMap::<NetId, usize>::new(); fixed_values.len()];
    let mut tree = ActiveNetTree::new(fixed_values.len())?;
    let mut result = ParallelSeparation::default();
    for (_, event_kind, segment_index) in events {
        let segment = segments[segment_index];
        let fixed_index = fixed_indices[segment_index];
        if event_kind == 0 {
            let remove_net = {
                let count = active[fixed_index]
                    .get_mut(&segment.net)
                    .expect("ending parallel segment is active");
                *count -= 1;
                *count == 0
            };
            if remove_net {
                active[fixed_index].remove(&segment.net);
            }
            tree.set(
                fixed_index,
                NetSummary::from_nets(active[fixed_index].keys().copied()),
                remaining_tree_visits,
            )?;
            continue;
        }

        if tree.has_other_at(fixed_index, segment.net, remaining_tree_visits)? {
            result.minimum = Some(0.0);
        }
        for candidate in [
            tree.rightmost_other(fixed_index, segment.net, remaining_tree_visits)?,
            tree.leftmost_other(fixed_index + 1, segment.net, remaining_tree_visits)?,
        ]
        .into_iter()
        .flatten()
        {
            let distance = (fixed_values[candidate].0 - segment.fixed).abs();
            debug_assert!(distance > 0.0);
            result.minimum = minimum_option(result.minimum, Some(distance));
            result.minimum_positive = minimum_option(result.minimum_positive, Some(distance));
        }
        *active[fixed_index].entry(segment.net).or_default() += 1;
        tree.set(
            fixed_index,
            NetSummary::from_nets(active[fixed_index].keys().copied()),
            remaining_tree_visits,
        )?;
    }
    Ok(result)
}

fn minimum_option(left: Option<f64>, right: Option<f64>) -> Option<f64> {
    match (left, right) {
        (Some(left), Some(right)) => Some(left.min(right)),
        (Some(value), None) | (None, Some(value)) => Some(value),
        (None, None) => None,
    }
}

fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}

#[derive(Clone, Copy, Default)]
struct NetSummary {
    first: Option<NetId>,
    second: Option<NetId>,
}

impl NetSummary {
    fn from_nets(nets: impl Iterator<Item = NetId>) -> Self {
        let mut summary = Self::default();
        for net in nets {
            summary.insert(net);
            if summary.second.is_some() {
                break;
            }
        }
        summary
    }

    fn merged(left: Self, right: Self) -> Self {
        Self::from_nets(
            [left.first, left.second, right.first, right.second]
                .into_iter()
                .flatten(),
        )
    }

    fn insert(&mut self, net: NetId) {
        if self.first == Some(net) || self.second == Some(net) {
            return;
        }
        if self.first.is_none() {
            self.first = Some(net);
        } else if self.second.is_none() {
            self.second = Some(net);
        }
    }

    fn has_other(self, excluded: NetId) -> bool {
        self.first.is_some_and(|net| net != excluded)
            || self.second.is_some_and(|net| net != excluded)
    }
}

struct ActiveNetTree {
    leaf_count: usize,
    summaries: Vec<NetSummary>,
}

impl ActiveNetTree {
    fn new(coordinate_count: usize) -> Result<Self, ParallelSeparationError> {
        let leaf_count = coordinate_count
            .checked_next_power_of_two()
            .ok_or(ParallelSeparationError::WorkLimitExceeded)?;
        let tree_size = leaf_count
            .checked_mul(2)
            .ok_or(ParallelSeparationError::WorkLimitExceeded)?;
        Ok(Self {
            leaf_count,
            summaries: vec![NetSummary::default(); tree_size],
        })
    }

    fn set(
        &mut self,
        index: usize,
        summary: NetSummary,
        remaining_tree_visits: &mut usize,
    ) -> Result<(), ParallelSeparationError> {
        let mut cursor = self.leaf_count + index;
        charge_tree_visit(remaining_tree_visits)?;
        self.summaries[cursor] = summary;
        cursor /= 2;
        while cursor != 0 {
            charge_tree_visit(remaining_tree_visits)?;
            self.summaries[cursor] =
                NetSummary::merged(self.summaries[cursor * 2], self.summaries[cursor * 2 + 1]);
            cursor /= 2;
        }
        Ok(())
    }

    fn has_other_at(
        &self,
        index: usize,
        excluded: NetId,
        remaining_tree_visits: &mut usize,
    ) -> Result<bool, ParallelSeparationError> {
        charge_tree_visit(remaining_tree_visits)?;
        Ok(self.summaries[self.leaf_count + index].has_other(excluded))
    }

    fn rightmost_other(
        &self,
        end: usize,
        excluded: NetId,
        remaining_tree_visits: &mut usize,
    ) -> Result<Option<usize>, ParallelSeparationError> {
        self.search_rightmost(1, 0, self.leaf_count, end, excluded, remaining_tree_visits)
    }

    fn search_rightmost(
        &self,
        node: usize,
        start: usize,
        end: usize,
        query_end: usize,
        excluded: NetId,
        remaining_tree_visits: &mut usize,
    ) -> Result<Option<usize>, ParallelSeparationError> {
        charge_tree_visit(remaining_tree_visits)?;
        if start >= query_end || !self.summaries[node].has_other(excluded) {
            return Ok(None);
        }
        if end - start == 1 {
            return Ok(Some(start));
        }
        let middle = (start + end) / 2;
        if let Some(found) = self.search_rightmost(
            node * 2 + 1,
            middle,
            end,
            query_end,
            excluded,
            remaining_tree_visits,
        )? {
            return Ok(Some(found));
        }
        self.search_rightmost(
            node * 2,
            start,
            middle,
            query_end,
            excluded,
            remaining_tree_visits,
        )
    }

    fn leftmost_other(
        &self,
        start: usize,
        excluded: NetId,
        remaining_tree_visits: &mut usize,
    ) -> Result<Option<usize>, ParallelSeparationError> {
        self.search_leftmost(
            1,
            0,
            self.leaf_count,
            start,
            excluded,
            remaining_tree_visits,
        )
    }

    fn search_leftmost(
        &self,
        node: usize,
        start: usize,
        end: usize,
        query_start: usize,
        excluded: NetId,
        remaining_tree_visits: &mut usize,
    ) -> Result<Option<usize>, ParallelSeparationError> {
        charge_tree_visit(remaining_tree_visits)?;
        if end <= query_start || !self.summaries[node].has_other(excluded) {
            return Ok(None);
        }
        if end - start == 1 {
            return Ok(Some(start));
        }
        let middle = (start + end) / 2;
        if let Some(found) = self.search_leftmost(
            node * 2,
            start,
            middle,
            query_start,
            excluded,
            remaining_tree_visits,
        )? {
            return Ok(Some(found));
        }
        self.search_leftmost(
            node * 2 + 1,
            middle,
            end,
            query_start,
            excluded,
            remaining_tree_visits,
        )
    }
}

fn charge_tree_visit(remaining_tree_visits: &mut usize) -> Result<(), ParallelSeparationError> {
    *remaining_tree_visits = remaining_tree_visits
        .checked_sub(1)
        .ok_or(ParallelSeparationError::WorkLimitExceeded)?;
    Ok(())
}

/// Exact physical wire-length accounting for nearby parallel routes.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ParallelCongestion {
    pub total_length: f64,
    pub congested_length: f64,
}

impl ParallelCongestion {
    pub fn ratio(self) -> f64 {
        if self.total_length > 0.0 {
            (self.congested_length / self.total_length).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }
}

/// Measure the exact wire length that overlaps longitudinally with a
/// different-net parallel segment closer than `cutoff`.
///
/// Callers must provide finite, positive-length segments whose overlapping
/// collinear geometry has already been unioned per electrical net.
#[doc(hidden)]
pub fn measure_parallel_congestion(
    segments: &[ParallelSegment],
    cutoff: f64,
) -> ParallelCongestion {
    measure_parallel_congestion_bounded(segments, cutoff, usize::MAX)
        .expect("an unbounded congestion sweep cannot exhaust its work budget")
}

/// Measure parallel congestion while bounding active-neighbor enumeration.
///
/// Returns `None` before the sweep performs more than
/// `max_active_visits` active-range item visits.
pub(crate) fn measure_parallel_congestion_bounded(
    segments: &[ParallelSegment],
    cutoff: f64,
    max_active_visits: usize,
) -> Option<ParallelCongestion> {
    measure_parallel_congestion_profile_bounded(segments, cutoff, max_active_visits)
        .map(|(congestion, _)| congestion)
}

pub(crate) fn measure_parallel_congestion_profile_bounded(
    segments: &[ParallelSegment],
    cutoff: f64,
    max_active_visits: usize,
) -> Option<(ParallelCongestion, Option<f64>)> {
    let total_length = segments
        .iter()
        .map(|segment| segment.end - segment.start)
        .sum::<f64>();
    if total_length <= 0.0 || cutoff <= 0.0 {
        return Some((
            ParallelCongestion {
                total_length,
                congested_length: 0.0,
            },
            None,
        ));
    }

    let mut congested_length = 0.0;
    let mut minimum_positive_separation = None::<f64>;
    let mut remaining_active_visits = max_active_visits;
    for horizontal in [true, false] {
        let oriented = segments
            .iter()
            .filter(|segment| segment.horizontal == horizontal)
            .collect::<Vec<_>>();
        if oriented.len() >= 2 {
            let (oriented_length, oriented_minimum) =
                congested_length_for_orientation(&oriented, cutoff, &mut remaining_active_visits)?;
            congested_length += oriented_length;
            if let Some(oriented_minimum) = oriented_minimum {
                minimum_positive_separation = Some(
                    minimum_positive_separation
                        .map_or(oriented_minimum, |current| current.min(oriented_minimum)),
                );
            }
        }
    }
    Some((
        ParallelCongestion {
            total_length,
            congested_length,
        },
        minimum_positive_separation,
    ))
}

fn congested_length_for_orientation(
    segments: &[&ParallelSegment],
    cutoff: f64,
    remaining_active_visits: &mut usize,
) -> Option<(f64, Option<f64>)> {
    let mut events = Vec::with_capacity(segments.len() * 2);
    for (index, segment) in segments.iter().enumerate() {
        // End events precede starts so longitudinal endpoint contact has zero
        // weight and never becomes an active close pair.
        events.push((FloatKey(segment.end), 0u8, index));
        events.push((FloatKey(segment.start), 1u8, index));
    }
    events.sort_unstable();

    let mut active = BTreeMap::<FloatKey, BTreeSet<usize>>::new();
    let mut close_neighbor_counts = vec![0usize; segments.len()];
    let mut congested_active = 0usize;
    let mut congested_length = 0.0;
    let mut minimum_positive_separation = None::<f64>;
    let mut neighbors = Vec::new();
    let mut previous = events
        .first()
        .map_or(0.0, |(coordinate, _, _)| coordinate.0);

    for (coordinate, event_kind, segment_index) in events {
        congested_length += congested_active as f64 * (coordinate.0 - previous);
        previous = coordinate.0;

        let segment = segments[segment_index];
        if !collect_close_active_other_nets(
            segment,
            cutoff,
            segments,
            &active,
            &mut neighbors,
            remaining_active_visits,
        ) {
            return None;
        }
        for &neighbor in &neighbors {
            let separation = (segments[neighbor].fixed - segment.fixed).abs();
            if separation > 0.0 {
                minimum_positive_separation = Some(
                    minimum_positive_separation
                        .map_or(separation, |current| current.min(separation)),
                );
            }
        }
        if event_kind == 0 {
            if close_neighbor_counts[segment_index] != 0 {
                congested_active -= 1;
            }
            for &neighbor in &neighbors {
                let count = &mut close_neighbor_counts[neighbor];
                *count -= 1;
                if *count == 0 {
                    congested_active -= 1;
                }
            }
            close_neighbor_counts[segment_index] = 0;
            let coordinate = FloatKey(segment.fixed);
            let remove_coordinate = {
                let members = active
                    .get_mut(&coordinate)
                    .expect("ending parallel segment is active");
                assert!(members.remove(&segment_index));
                members.is_empty()
            };
            if remove_coordinate {
                active.remove(&coordinate);
            }
        } else {
            close_neighbor_counts[segment_index] = neighbors.len();
            if !neighbors.is_empty() {
                congested_active += 1;
            }
            for &neighbor in &neighbors {
                let count = &mut close_neighbor_counts[neighbor];
                if *count == 0 {
                    congested_active += 1;
                }
                *count += 1;
            }
            active
                .entry(FloatKey(segment.fixed))
                .or_default()
                .insert(segment_index);
        }
    }
    debug_assert!(active.is_empty());
    debug_assert_eq!(congested_active, 0);
    Some((congested_length, minimum_positive_separation))
}

fn collect_close_active_other_nets(
    query: &ParallelSegment,
    cutoff: f64,
    segments: &[&ParallelSegment],
    active: &BTreeMap<FloatKey, BTreeSet<usize>>,
    neighbors: &mut Vec<usize>,
    remaining_active_visits: &mut usize,
) -> bool {
    use std::ops::Bound::Excluded;

    neighbors.clear();
    for index in active
        .range((
            Excluded(FloatKey(query.fixed - cutoff)),
            Excluded(FloatKey(query.fixed + cutoff)),
        ))
        .flat_map(|(_, members)| members.iter().copied())
    {
        let Some(remaining) = remaining_active_visits.checked_sub(1) else {
            return false;
        };
        *remaining_active_visits = remaining;
        if segments[index].net != query.net {
            neighbors.push(index);
        }
    }
    true
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

#[cfg(test)]
mod tests {
    use super::{
        EdgeNodeClearanceError, EdgeNodeSegment, NetNodeRelation, ParallelSegment,
        ParallelSeparation, ParallelSeparationError, measure_edge_node_clearance_bounded,
        measure_parallel_congestion, measure_parallel_congestion_bounded,
        measure_parallel_congestion_profile_bounded, measure_parallel_separation_bounded,
    };
    use crate::NodeGeometry;

    fn placed_node(id: u32, x: f64, y: f64) -> NodeGeometry {
        NodeGeometry {
            id,
            x,
            y,
            width: 10.0,
            height: 10.0,
        }
    }

    #[test]
    fn edge_node_clearance_measures_horizontal_and_vertical_pairs() {
        let nodes = [placed_node(1, 20.0, 20.0), placed_node(2, 60.0, 60.0)];
        let segments = [
            EdgeNodeSegment {
                net: 1,
                horizontal: true,
                fixed: 15.0,
                start: 20.0,
                end: 30.0,
            },
            EdgeNodeSegment {
                net: 2,
                horizontal: false,
                fixed: 55.0,
                start: 60.0,
                end: 70.0,
            },
        ];

        let measured = measure_edge_node_clearance_bounded(&segments, &nodes, &[], 5.0, 4).unwrap();
        assert_eq!(measured.minimum_clearance, Some(5.0));
        assert_eq!(measured.violations, 0);
        let inside =
            measure_edge_node_clearance_bounded(&segments, &nodes, &[], 5.0 + 1e-9, 4).unwrap();
        assert_eq!(inside.violations, 2);
    }

    #[test]
    fn edge_node_clearance_accepts_exact_expanded_edges_and_corners() {
        let node = placed_node(1, 15.0, 5.0);
        let edge = EdgeNodeSegment {
            net: 1,
            horizontal: true,
            fixed: 0.0,
            start: 15.0,
            end: 25.0,
        };
        let corner = EdgeNodeSegment {
            net: 2,
            horizontal: true,
            fixed: 0.0,
            start: 0.0,
            end: 10.0,
        };

        let exact = measure_edge_node_clearance_bounded(
            &[edge, corner],
            std::slice::from_ref(&node),
            &[],
            5.0,
            2,
        )
        .unwrap();
        assert_eq!(exact.minimum_clearance, Some(5.0));
        assert_eq!(exact.violations, 0);
        let inside =
            measure_edge_node_clearance_bounded(&[edge, corner], &[node], &[], 5.0 + 1e-9, 2)
                .unwrap();
        assert_eq!(inside.violations, 2);
    }

    #[test]
    fn edge_node_clearance_exempts_every_endpoint_node_on_a_net() {
        let nodes = [
            placed_node(1, 0.0, 0.0),
            placed_node(2, 20.0, 0.0),
            placed_node(3, 40.0, 0.0),
            placed_node(4, 20.0, 10.0),
        ];
        let relations = [
            NetNodeRelation { net: 7, node: 1 },
            NetNodeRelation { net: 7, node: 2 },
            NetNodeRelation { net: 7, node: 3 },
        ];
        let segments = [
            EdgeNodeSegment {
                net: 7,
                horizontal: true,
                fixed: 5.0,
                start: 0.0,
                end: 50.0,
            },
            EdgeNodeSegment {
                net: 7,
                horizontal: false,
                fixed: 5.0,
                start: -5.0,
                end: 15.0,
            },
            EdgeNodeSegment {
                net: 7,
                horizontal: false,
                fixed: 45.0,
                start: -5.0,
                end: 15.0,
            },
            EdgeNodeSegment {
                net: 7,
                horizontal: true,
                fixed: 5.0,
                start: -5.0,
                end: 15.0,
            },
        ];

        let measured =
            measure_edge_node_clearance_bounded(&segments, &nodes, &relations, 11.0, 16).unwrap();
        assert_eq!(measured.minimum_clearance, Some(5.0));
        assert_eq!(measured.violations, 2);
    }

    #[test]
    fn edge_node_clearance_exempts_all_four_endpoint_approach_sides() {
        let nodes = [placed_node(1, 20.0, 20.0), placed_node(2, 50.0, 50.0)];
        let segments = [
            EdgeNodeSegment {
                net: 7,
                horizontal: true,
                fixed: 25.0,
                start: 10.0,
                end: 20.0,
            },
            EdgeNodeSegment {
                net: 7,
                horizontal: true,
                fixed: 25.0,
                start: 30.0,
                end: 40.0,
            },
            EdgeNodeSegment {
                net: 7,
                horizontal: false,
                fixed: 25.0,
                start: 10.0,
                end: 20.0,
            },
            EdgeNodeSegment {
                net: 7,
                horizontal: false,
                fixed: 25.0,
                start: 30.0,
                end: 40.0,
            },
        ];

        let measured = measure_edge_node_clearance_bounded(
            &segments,
            &nodes,
            &[NetNodeRelation { net: 7, node: 1 }],
            20.0,
            8,
        )
        .unwrap();
        assert_eq!(measured.minimum_clearance, Some(25.0));
        assert_eq!(measured.violations, 0);
    }

    #[test]
    fn edge_node_clearance_handles_empty_pairs_permutations_and_work_caps() {
        let segment = EdgeNodeSegment {
            net: 1,
            horizontal: true,
            fixed: 0.0,
            start: 0.0,
            end: 10.0,
        };
        assert_eq!(
            measure_edge_node_clearance_bounded(&[], &[], &[], 5.0, 0)
                .unwrap()
                .minimum_clearance,
            None
        );
        assert_eq!(
            measure_edge_node_clearance_bounded(
                &[segment],
                &[placed_node(1, 0.0, 0.0)],
                &[NetNodeRelation { net: 1, node: 1 }],
                5.0,
                1,
            )
            .unwrap()
            .minimum_clearance,
            None
        );
        let mut nodes = vec![placed_node(2, 0.0, 10.0), placed_node(3, 20.0, 0.0)];
        let measured =
            measure_edge_node_clearance_bounded(&[segment], &nodes, &[], 5.0, 2).unwrap();
        nodes.reverse();
        assert_eq!(
            measure_edge_node_clearance_bounded(&[segment], &nodes, &[], 5.0, 2),
            Ok(measured)
        );
        assert_eq!(
            measure_edge_node_clearance_bounded(&[segment], &nodes, &[], 11.0, 1),
            Err(EdgeNodeClearanceError::WorkLimitExceeded)
        );
        assert!(measure_edge_node_clearance_bounded(&[segment], &nodes, &[], 11.0, 2).is_ok());
    }

    #[test]
    fn edge_node_clearance_rejects_malformed_geometry() {
        let node = placed_node(1, 0.0, 0.0);
        let malformed = EdgeNodeSegment {
            net: 1,
            horizontal: true,
            fixed: 0.0,
            start: 10.0,
            end: 0.0,
        };

        assert_eq!(
            measure_edge_node_clearance_bounded(
                &[malformed],
                std::slice::from_ref(&node),
                &[],
                5.0,
                1,
            ),
            Err(EdgeNodeClearanceError::InvalidInput)
        );
        assert_eq!(
            measure_edge_node_clearance_bounded(&[], std::slice::from_ref(&node), &[], f64::NAN, 0,),
            Err(EdgeNodeClearanceError::InvalidInput)
        );
    }

    #[test]
    fn edge_node_clearance_matches_a_brute_force_oracle() {
        fn oracle(
            segments: &[EdgeNodeSegment],
            nodes: &[NodeGeometry],
            relations: &[NetNodeRelation],
            threshold: f64,
        ) -> super::EdgeNodeClearance {
            let related = relations
                .iter()
                .map(|relation| (relation.net, relation.node))
                .collect::<std::collections::BTreeSet<_>>();
            let mut result = super::EdgeNodeClearance::default();
            for segment in segments {
                for node in nodes {
                    if related.contains(&(segment.net, node.id)) {
                        continue;
                    }
                    let (low_x, high_x, low_y, high_y) = if segment.horizontal {
                        (segment.start, segment.end, segment.fixed, segment.fixed)
                    } else {
                        (segment.fixed, segment.fixed, segment.start, segment.end)
                    };
                    let dx = if high_x < node.x {
                        node.x - high_x
                    } else if low_x > node.x + node.width {
                        low_x - node.x - node.width
                    } else {
                        0.0
                    };
                    let dy = if high_y < node.y {
                        node.y - high_y
                    } else if low_y > node.y + node.height {
                        low_y - node.y - node.height
                    } else {
                        0.0
                    };
                    let distance = dx.max(dy);
                    result.minimum_clearance = Some(
                        result
                            .minimum_clearance
                            .map_or(distance, |current| current.min(distance)),
                    );
                    result.violations += usize::from(distance < threshold);
                }
            }
            result
        }

        let nodes = (0..37)
            .map(|id| {
                let mut node =
                    placed_node(id, f64::from((id * 17) % 83), f64::from((id * 29) % 97));
                node.width = f64::from(3 + id % 19);
                node.height = f64::from(4 + (id * 7) % 23);
                node
            })
            .collect::<Vec<_>>();
        let segments = (0..61)
            .map(|id| {
                let horizontal = id % 2 == 0;
                EdgeNodeSegment {
                    net: id % 11,
                    horizontal,
                    fixed: f64::from((id * 13) % 101),
                    start: f64::from((id * 7) % 43),
                    end: f64::from((id * 7) % 43 + 5 + id % 53),
                }
            })
            .collect::<Vec<_>>();
        let relations = (0..29)
            .map(|id| NetNodeRelation {
                net: id % 11,
                node: (id * 3) % 37,
            })
            .collect::<Vec<_>>();

        for threshold in [0.0, 1.0, 7.5, 20.0] {
            let expected = oracle(&segments, &nodes, &relations, threshold);
            assert_eq!(
                measure_edge_node_clearance_bounded(
                    &segments,
                    &nodes,
                    &relations,
                    threshold,
                    segments.len() * nodes.len(),
                ),
                Ok(expected)
            );

            let mut permuted_segments = segments.clone();
            let mut permuted_nodes = nodes.clone();
            let mut permuted_relations = relations.clone();
            permuted_segments.reverse();
            permuted_nodes.rotate_left(13);
            permuted_relations.reverse();
            assert_eq!(
                measure_edge_node_clearance_bounded(
                    &permuted_segments,
                    &permuted_nodes,
                    &permuted_relations,
                    threshold,
                    segments.len() * nodes.len(),
                ),
                Ok(expected)
            );

            let first_success = (0..=segments.len() * nodes.len())
                .find(|&budget| {
                    measure_edge_node_clearance_bounded(
                        &segments, &nodes, &relations, threshold, budget,
                    )
                    .is_ok()
                })
                .unwrap();
            let permuted_first_success = (0..=segments.len() * nodes.len())
                .find(|&budget| {
                    measure_edge_node_clearance_bounded(
                        &permuted_segments,
                        &permuted_nodes,
                        &permuted_relations,
                        threshold,
                        budget,
                    )
                    .is_ok()
                })
                .unwrap();
            assert_eq!(permuted_first_success, first_success);
            assert_eq!(
                measure_edge_node_clearance_bounded(
                    &segments,
                    &nodes,
                    &relations,
                    threshold,
                    first_success - 1,
                ),
                Err(EdgeNodeClearanceError::WorkLimitExceeded)
            );
        }
    }

    #[test]
    fn edge_node_clearance_charges_related_candidates_before_exemption() {
        let nodes = (0..32)
            .map(|id| placed_node(id, f64::from(id * 20), 0.0))
            .collect::<Vec<_>>();
        let segments = [EdgeNodeSegment {
            net: 7,
            horizontal: true,
            fixed: 5.0,
            start: -10.0,
            end: 640.0,
        }];
        let relations = nodes
            .iter()
            .map(|node| NetNodeRelation {
                net: 7,
                node: node.id,
            })
            .collect::<Vec<_>>();

        assert_eq!(
            measure_edge_node_clearance_bounded(
                &segments,
                &nodes,
                &relations,
                20.0,
                nodes.len() - 1,
            ),
            Err(EdgeNodeClearanceError::WorkLimitExceeded)
        );
        assert_eq!(
            measure_edge_node_clearance_bounded(&segments, &nodes, &relations, 20.0, nodes.len(),),
            Ok(super::EdgeNodeClearance::default())
        );
    }

    fn oracle(segments: &[ParallelSegment], cutoff: f64) -> f64 {
        let mut breakpoints = segments
            .iter()
            .flat_map(|segment| [segment.start, segment.end])
            .collect::<Vec<_>>();
        breakpoints.sort_by(f64::total_cmp);
        breakpoints.dedup_by(|left, right| left.total_cmp(right).is_eq());
        let mut congested_length = 0.0;
        for (index, segment) in segments.iter().enumerate() {
            for window in breakpoints.windows(2) {
                let start = window[0].max(segment.start);
                let end = window[1].min(segment.end);
                if end <= start {
                    continue;
                }
                if segments
                    .iter()
                    .enumerate()
                    .any(|(candidate_index, candidate)| {
                        candidate_index != index
                            && candidate.horizontal == segment.horizontal
                            && candidate.net != segment.net
                            && candidate.start < end
                            && candidate.end > start
                            && (candidate.fixed - segment.fixed).abs() < cutoff
                    })
                {
                    congested_length += end - start;
                }
            }
        }
        congested_length
    }

    #[test]
    fn exact_congestion_matches_quadratic_oracle_and_input_permutations() {
        let mut state = 0x6a09_e667_f3bc_c909u64;
        let mut next = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            state
        };
        let mut segments = (0..96u32)
            .map(|_| {
                let start = (next() % 80) as f64;
                ParallelSegment {
                    net: (next() % 11) as u32,
                    horizontal: next() & 1 == 0,
                    fixed: (next() % 37) as f64 * 0.23,
                    start,
                    end: start + (next() % 30 + 1) as f64,
                }
            })
            .collect::<Vec<_>>();
        let expected = oracle(&segments, 4.0);
        assert_eq!(
            measure_parallel_congestion(&segments, 4.0).congested_length,
            expected
        );

        segments.reverse();
        assert_eq!(
            measure_parallel_congestion(&segments, 4.0).congested_length,
            expected
        );
    }

    #[test]
    fn bounded_congestion_stops_at_the_exact_active_visit_budget() {
        let segments = [
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: 0.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 2,
                horizontal: true,
                fixed: 1.0,
                start: 0.0,
                end: 10.0,
            },
        ];

        assert!(measure_parallel_congestion_bounded(&segments, 4.0, 3).is_none());
        assert_eq!(
            measure_parallel_congestion_bounded(&segments, 4.0, 4),
            Some(measure_parallel_congestion(&segments, 4.0))
        );
        assert_eq!(
            measure_parallel_congestion_profile_bounded(&segments, 4.0, 4),
            Some((measure_parallel_congestion(&segments, 4.0), Some(1.0)))
        );
    }

    #[test]
    fn congestion_profile_minimum_is_positive_strict_and_bounded() {
        let segments = [
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: 0.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 2,
                horizontal: true,
                fixed: 0.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 3,
                horizontal: true,
                fixed: 4.0,
                start: 0.0,
                end: 10.0,
            },
        ];

        assert_eq!(
            measure_parallel_congestion_profile_bounded(&segments, 4.0, 5),
            Some((measure_parallel_congestion(&segments, 4.0), None))
        );
        assert!(measure_parallel_congestion_profile_bounded(&segments, 4.0, 4).is_none());
    }

    #[test]
    fn bounded_congestion_counts_same_net_active_visits() {
        let segments = (0..64)
            .map(|index| ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: f64::from(index) * 0.01,
                start: 0.0,
                end: 10.0,
            })
            .collect::<Vec<_>>();

        assert!(measure_parallel_congestion_bounded(&segments, 4.0, 4_095).is_none());
        assert_eq!(
            measure_parallel_congestion_bounded(&segments, 4.0, 4_096),
            Some(measure_parallel_congestion(&segments, 4.0))
        );
    }

    #[test]
    fn parallel_separation_reports_zero_and_nearest_positive_distance() {
        let segments = [
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: 0.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 2,
                horizontal: true,
                fixed: -0.0,
                start: 2.0,
                end: 8.0,
            },
            ParallelSegment {
                net: 3,
                horizontal: true,
                fixed: 5.0,
                start: 4.0,
                end: 6.0,
            },
            ParallelSegment {
                net: 4,
                horizontal: false,
                fixed: 20.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 5,
                horizontal: false,
                fixed: 27.0,
                start: 5.0,
                end: 15.0,
            },
        ];

        assert_eq!(
            measure_parallel_separation_bounded(&segments, usize::MAX),
            Ok(ParallelSeparation {
                minimum: Some(0.0),
                minimum_positive: Some(5.0),
            })
        );
    }

    #[test]
    fn parallel_separation_ignores_same_net_perpendicular_and_endpoint_only_pairs() {
        let segments = [
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: 0.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: 1.0,
                start: 0.0,
                end: 10.0,
            },
            ParallelSegment {
                net: 2,
                horizontal: true,
                fixed: 2.0,
                start: 10.0,
                end: 20.0,
            },
            ParallelSegment {
                net: 3,
                horizontal: false,
                fixed: 5.0,
                start: -5.0,
                end: 5.0,
            },
        ];

        assert_eq!(
            measure_parallel_separation_bounded(&segments, usize::MAX),
            Ok(ParallelSeparation::default())
        );
    }

    #[test]
    fn parallel_separation_is_permutation_deterministic_and_fails_closed_on_work() {
        let mut segments = [
            ParallelSegment {
                net: 9,
                horizontal: true,
                fixed: 12.0,
                start: 0.0,
                end: 30.0,
            },
            ParallelSegment {
                net: 7,
                horizontal: true,
                fixed: 3.0,
                start: 4.0,
                end: 20.0,
            },
            ParallelSegment {
                net: 8,
                horizontal: true,
                fixed: 7.0,
                start: 2.0,
                end: 24.0,
            },
        ];
        let expected = measure_parallel_separation_bounded(&segments, usize::MAX).unwrap();
        assert_eq!(expected.minimum, Some(4.0));
        assert_eq!(expected.minimum_positive, Some(4.0));
        assert_eq!(
            measure_parallel_separation_bounded(&segments, 1),
            Err(ParallelSeparationError::WorkLimitExceeded)
        );

        segments.reverse();
        assert_eq!(
            measure_parallel_separation_bounded(&segments, usize::MAX),
            Ok(expected)
        );
        assert_eq!(
            measure_parallel_separation_bounded(&segments, 1),
            Err(ParallelSeparationError::WorkLimitExceeded)
        );
    }

    #[test]
    fn parallel_separation_rejects_malformed_segments() {
        for segment in [
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: f64::NAN,
                start: 0.0,
                end: 1.0,
            },
            ParallelSegment {
                net: 1,
                horizontal: true,
                fixed: 0.0,
                start: 1.0,
                end: 1.0,
            },
            ParallelSegment {
                net: 1,
                horizontal: false,
                fixed: 0.0,
                start: 2.0,
                end: 1.0,
            },
        ] {
            assert_eq!(
                measure_parallel_separation_bounded(&[segment], usize::MAX),
                Err(ParallelSeparationError::InvalidInput)
            );
        }
    }

    #[test]
    fn parallel_separation_matches_a_brute_force_oracle() {
        fn oracle(segments: &[ParallelSegment]) -> ParallelSeparation {
            let mut result = ParallelSeparation::default();
            for (index, left) in segments.iter().enumerate() {
                for right in &segments[index + 1..] {
                    if left.horizontal != right.horizontal
                        || left.net == right.net
                        || left.start.max(right.start) >= left.end.min(right.end)
                    {
                        continue;
                    }
                    let distance = (left.fixed - right.fixed).abs();
                    result.minimum = super::minimum_option(result.minimum, Some(distance));
                    if distance > 0.0 {
                        result.minimum_positive =
                            super::minimum_option(result.minimum_positive, Some(distance));
                    }
                }
            }
            result
        }

        fn next(state: &mut u64) -> u64 {
            *state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *state
        }

        let mut state = 91_u64;
        let mut segments = (0..256)
            .map(|index| {
                let net = (next(&mut state) % 13) as u32;
                let fixed = (next(&mut state) % 31) as f64 - 15.0;
                let start = (next(&mut state) % 80) as f64 - 40.0;
                let length = (next(&mut state) % 20 + 1) as f64;
                ParallelSegment {
                    net,
                    horizontal: index % 2 == 0,
                    fixed,
                    start,
                    end: start + length,
                }
            })
            .collect::<Vec<_>>();
        let expected = oracle(&segments);

        assert_eq!(
            measure_parallel_separation_bounded(&segments, usize::MAX),
            Ok(expected)
        );
        segments.rotate_left(73);
        assert_eq!(
            measure_parallel_separation_bounded(&segments, usize::MAX),
            Ok(expected)
        );
    }
}
