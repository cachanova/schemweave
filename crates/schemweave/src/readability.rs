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
    let required_visits = segments
        .len()
        .checked_mul(nodes.len())
        .ok_or(EdgeNodeClearanceError::WorkLimitExceeded)?;
    if required_visits > max_pair_visits {
        return Err(EdgeNodeClearanceError::WorkLimitExceeded);
    }
    let related = relations
        .iter()
        .map(|relation| (relation.net, relation.node))
        .collect::<HashSet<_>>();
    let mut minimum = None::<f64>;
    let mut violations = 0usize;
    for segment in segments {
        for node in nodes {
            if related.contains(&(segment.net, node.id)) {
                continue;
            }
            let distance = segment_node_distance(*segment, node);
            minimum = Some(minimum.map_or(distance, |current| current.min(distance)));
            if distance < threshold {
                violations = violations.saturating_add(1);
            }
        }
    }
    Ok(EdgeNodeClearance {
        minimum_clearance: minimum,
        violations,
    })
}

fn segment_node_distance(segment: EdgeNodeSegment, node: &NodeGeometry) -> f64 {
    let (segment_left, segment_top, segment_right, segment_bottom) = if segment.horizontal {
        (segment.start, segment.fixed, segment.end, segment.fixed)
    } else {
        (segment.fixed, segment.start, segment.fixed, segment.end)
    };
    let node_right = node.x + node.width;
    let node_bottom = node.y + node.height;
    let dx = if segment_right < node.x {
        node.x - segment_right
    } else if segment_left > node_right {
        segment_left - node_right
    } else {
        0.0
    };
    let dy = if segment_bottom < node.y {
        node.y - segment_bottom
    } else if segment_top > node_bottom {
        segment_top - node_bottom
    } else {
        0.0
    };
    dx.max(dy)
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
    let total_length = segments
        .iter()
        .map(|segment| segment.end - segment.start)
        .sum::<f64>();
    if total_length <= 0.0 || cutoff <= 0.0 {
        return Some(ParallelCongestion {
            total_length,
            congested_length: 0.0,
        });
    }

    let mut congested_length = 0.0;
    let mut remaining_active_visits = max_active_visits;
    for horizontal in [true, false] {
        let oriented = segments
            .iter()
            .filter(|segment| segment.horizontal == horizontal)
            .collect::<Vec<_>>();
        if oriented.len() >= 2 {
            congested_length +=
                congested_length_for_orientation(&oriented, cutoff, &mut remaining_active_visits)?;
        }
    }
    Some(ParallelCongestion {
        total_length,
        congested_length,
    })
}

fn congested_length_for_orientation(
    segments: &[&ParallelSegment],
    cutoff: f64,
    remaining_active_visits: &mut usize,
) -> Option<f64> {
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
    Some(congested_length)
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
        measure_edge_node_clearance_bounded, measure_parallel_congestion,
        measure_parallel_congestion_bounded,
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
            measure_edge_node_clearance_bounded(&[segment], &nodes, &[], 5.0, 1),
            Err(EdgeNodeClearanceError::WorkLimitExceeded)
        );
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

        let nodes = (0..9)
            .map(|id| placed_node(id, f64::from((id * 17) % 41), f64::from((id * 29) % 47)))
            .collect::<Vec<_>>();
        let segments = (0..17)
            .map(|id| {
                let horizontal = id % 2 == 0;
                EdgeNodeSegment {
                    net: id % 4,
                    horizontal,
                    fixed: f64::from((id * 13) % 53),
                    start: f64::from((id * 7) % 19),
                    end: f64::from((id * 7) % 19 + 5 + id % 11),
                }
            })
            .collect::<Vec<_>>();
        let relations = (0..7)
            .map(|id| NetNodeRelation {
                net: id % 4,
                node: (id * 3) % 9,
            })
            .collect::<Vec<_>>();

        for threshold in [0.0, 1.0, 7.5, 20.0] {
            assert_eq!(
                measure_edge_node_clearance_bounded(
                    &segments,
                    &nodes,
                    &relations,
                    threshold,
                    segments.len() * nodes.len(),
                ),
                Ok(oracle(&segments, &nodes, &relations, threshold))
            );
        }
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
}
