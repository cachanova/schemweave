use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
};

use crate::NetId;

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
        ParallelSegment, measure_parallel_congestion, measure_parallel_congestion_bounded,
    };

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
