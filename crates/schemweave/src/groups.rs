use std::collections::{BTreeMap, BTreeSet};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{
    CandidateAdmissionState, CandidateRouting, ConstrainedLayoutError, Graph, Layout, LayoutConfig,
    NodeGeometry, NodeId, QualityEffort, effective_layout_options, evaluate_candidate,
    hard_geometry_failure, incremental, placement, routing, topology, validation,
};

const MAX_EXPANDED_GROUPS: usize = 4_096;
const MAX_EXPANDED_GROUP_MEMBERS: usize = 4_096;
const FRAME_EPSILON: f64 = 1e-7;

/// One already-expanded visual group in a complete layout request.
///
/// A full layout arranges disconnected members as a vertical stack until that
/// would exceed 1.5 times `reference_height`; larger groups use a deterministic
/// grid. The resulting frame is a keep-out region for every unrelated node.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ExpandedGroupConstraint {
    pub id: NodeId,
    pub members: Vec<NodeId>,
    pub reference_height: f64,
    #[serde(default)]
    pub frame_padding: f64,
}

/// Invalid or unsatisfied full-layout expanded-group contract.
#[derive(Clone, Debug, Error, PartialEq)]
#[non_exhaustive]
pub enum ExpandedGroupLayoutError {
    #[error("full layout has {actual} expanded groups, maximum is {maximum}")]
    TooManyGroups { actual: usize, maximum: usize },
    #[error("full layout has {actual} expanded-group members, maximum is {maximum}")]
    TooManyMembers { actual: usize, maximum: usize },
    #[error("expanded group {0} has no members")]
    EmptyGroup(NodeId),
    #[error("full layout repeats expanded group id {0}")]
    DuplicateGroup(NodeId),
    #[error("expanded group {group} repeats member node {member}")]
    DuplicateMember { group: NodeId, member: NodeId },
    #[error("expanded groups {first} and {second} both contain member node {member}")]
    OverlappingGroups {
        first: NodeId,
        second: NodeId,
        member: NodeId,
    },
    #[error("expanded group {group} references unknown member node {member}")]
    UnknownMember { group: NodeId, member: NodeId },
    #[error("expanded group {group} reference height must be finite and positive, got {height}")]
    InvalidReferenceHeight { group: NodeId, height: f64 },
    #[error("expanded group {group} frame padding must be finite and nonnegative, got {padding}")]
    InvalidFramePadding { group: NodeId, padding: f64 },
    #[error("expanded group {group} keep-out contains unrelated node {node}")]
    KeepOutUnsatisfied { group: NodeId, node: NodeId },
}

#[derive(Clone, Copy)]
struct Bounds {
    left: f64,
    top: f64,
    right: f64,
    bottom: f64,
}

impl Bounds {
    fn from_nodes<'a>(nodes: impl Iterator<Item = &'a NodeGeometry>) -> Option<Self> {
        nodes.fold(None, |bounds, node| {
            Some(match bounds {
                None => Self {
                    left: node.x,
                    top: node.y,
                    right: node.x + node.width,
                    bottom: node.y + node.height,
                },
                Some(bounds) => Self {
                    left: bounds.left.min(node.x),
                    top: bounds.top.min(node.y),
                    right: bounds.right.max(node.x + node.width),
                    bottom: bounds.bottom.max(node.y + node.height),
                },
            })
        })
    }

    fn overlaps(self, other: Self) -> bool {
        self.left < other.right - FRAME_EPSILON
            && self.right > other.left + FRAME_EPSILON
            && self.top < other.bottom - FRAME_EPSILON
            && self.bottom > other.top + FRAME_EPSILON
    }
}

struct GroupPlan {
    id: NodeId,
    members: BTreeSet<NodeId>,
    original: Bounds,
    arranged: Layout,
    frame_padding: f64,
    horizontal_insertion: f64,
}

pub(crate) fn apply_expanded_group_constraints(
    graph: &Graph,
    layout: Layout,
    config: &LayoutConfig,
) -> Result<Layout, ConstrainedLayoutError> {
    if config.expanded_groups.is_empty() {
        return Ok(layout);
    }
    let groups = validate_groups(graph, &config.expanded_groups)?;
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
    let all_members = groups
        .iter()
        .flat_map(|group| group.members.iter().copied())
        .collect::<BTreeSet<_>>();
    let effective_options = effective_layout_options(config.layout);
    let member_gap = incremental::EXPANSION_COMPONENT_GAP.max(
        crate::outward_obstacle_clearance_stub(effective_options) * 2.0
            + effective_options.route_lane_gap,
    );

    let mut plans = Vec::with_capacity(groups.len());
    for group in groups {
        let original = Bounds::from_nodes(group.members.iter().map(|id| nodes[id]))
            .expect("validated groups are non-empty");
        let member_graph = Graph {
            nodes: graph
                .nodes
                .iter()
                .filter(|node| group.members.contains(&node.id))
                .cloned()
                .collect(),
            edges: graph
                .edges
                .iter()
                .filter(|edge| {
                    group.members.contains(&edge.source.node)
                        && group.members.contains(&edge.target.node)
                })
                .cloned()
                .collect(),
        };
        let member_edge_ids = member_graph
            .edges
            .iter()
            .map(|edge| edge.id)
            .collect::<BTreeSet<_>>();
        let member_layout = placement::normalize_owned(
            group
                .members
                .iter()
                .map(|id| (*nodes[id]).clone())
                .collect(),
            member_edge_ids
                .iter()
                .map(|id| (*edges[id]).clone())
                .collect(),
        );
        let arranged = incremental::arrange_member_components(
            &member_graph,
            &member_layout,
            member_gap,
            member_gap,
            group.reference_height,
            None,
        );
        let frame_width = arranged.width + group.frame_padding * 2.0;
        plans.push(GroupPlan {
            id: group.id,
            members: group.members,
            original,
            arranged,
            frame_padding: group.frame_padding,
            horizontal_insertion: (frame_width - (original.right - original.left)).max(0.0),
        });
    }
    plans.sort_by(|left, right| {
        left.original
            .top
            .total_cmp(&right.original.top)
            .then(left.id.cmp(&right.id))
    });

    let mut placed = layout
        .nodes
        .iter()
        .map(|node| {
            let x_shift = plans
                .iter()
                .filter(|plan| node.x >= plan.original.right - FRAME_EPSILON)
                .map(|plan| plan.horizontal_insertion)
                .sum::<f64>();
            (
                node.id,
                NodeGeometry {
                    x: node.x + x_shift,
                    ..node.clone()
                },
            )
        })
        .collect::<BTreeMap<_, _>>();
    for (index, plan) in plans.iter().enumerate() {
        let current = Bounds::from_nodes(plan.members.iter().map(|id| &placed[id]))
            .expect("validated groups are non-empty");
        let left = current.left + plan.frame_padding;
        let top = current.top + plan.frame_padding;
        for node in &plan.arranged.nodes {
            placed.insert(
                node.id,
                NodeGeometry {
                    x: node.x + left,
                    y: node.y + top,
                    ..node.clone()
                },
            );
        }
        let member_bounds = Bounds::from_nodes(plan.members.iter().map(|id| &placed[id]))
            .expect("validated groups are non-empty");
        let frame = Bounds {
            left: member_bounds.left - plan.frame_padding,
            top: member_bounds.top - plan.frame_padding,
            right: member_bounds.right + plan.frame_padding,
            bottom: member_bounds.bottom + plan.frame_padding,
        };
        let mut intruders = placed
            .values()
            .filter(|node| {
                !all_members.contains(&node.id)
                    && frame.overlaps(Bounds {
                        left: node.x,
                        top: node.y,
                        right: node.x + node.width,
                        bottom: node.y + node.height,
                    })
            })
            .map(|node| node.id)
            .collect::<Vec<_>>();
        intruders.sort_unstable_by(|left, right| {
            placed[left]
                .y
                .total_cmp(&placed[right].y)
                .then(left.cmp(right))
        });
        let relocation_height = intruders
            .iter()
            .map(|id| placed[id].height + config.layout.node_gap)
            .sum::<f64>();
        if relocation_height > 0.0 {
            let intruders = intruders.iter().copied().collect::<BTreeSet<_>>();
            for node in placed.values_mut() {
                if !all_members.contains(&node.id)
                    && !intruders.contains(&node.id)
                    && node.y >= frame.bottom - FRAME_EPSILON
                {
                    node.y += relocation_height;
                }
            }
            let mut cursor = frame.bottom + config.layout.node_gap;
            for id in intruders {
                let node = placed
                    .get_mut(&id)
                    .expect("intruder came from placed nodes");
                node.y = cursor;
                cursor += node.height + config.layout.node_gap;
            }
        }
        for future in &plans[index + 1..] {
            let future_bounds = Bounds::from_nodes(future.members.iter().map(|id| &placed[id]))
                .expect("validated groups are non-empty");
            let future_frame = Bounds {
                left: future_bounds.left - future.frame_padding,
                top: future_bounds.top - future.frame_padding,
                right: future_bounds.right + future.frame_padding,
                bottom: future_bounds.bottom + future.frame_padding,
            };
            let shift = if frame.overlaps(future_frame) {
                frame.bottom + relocation_height + config.layout.node_gap - future_frame.top
            } else if future_frame.top >= frame.bottom - FRAME_EPSILON {
                relocation_height
            } else {
                0.0
            };
            if shift > 0.0 {
                for member in &future.members {
                    placed
                        .get_mut(member)
                        .expect("validated member is placed")
                        .y += shift;
                }
            }
        }
    }
    let placed = placed.into_values().collect::<Vec<_>>();

    let options = effective_options;
    let indexed =
        validation::validate_and_index_with_constraints(graph, options, &config.constraints)?;
    let (ranks, _) = topology::rank_candidates(&indexed);
    let routing_plan = routing::RoutingPlan::new(&indexed, &ranks);
    let mut best = None;
    let mut admission_state = CandidateAdmissionState::default();
    let (sparse_global, large_sparse_global) =
        crate::net_representative_sparse_global_flags(graph.nodes.len(), config.quality_effort);
    evaluate_candidate(
        &indexed,
        &routing_plan,
        &mut best,
        placed,
        options,
        CandidateRouting {
            supplemental: true,
            sparse_global,
            large_sparse_global,
            adaptive_gap_spacing: config.quality_effort != QualityEffort::Fast,
            deeper_crossing_repair: config.quality_effort == QualityEffort::Max,
        },
        &mut admission_state,
    );
    let candidate = best
        .ok_or_else(|| hard_geometry_failure(options, &admission_state))?
        .layout;
    verify_keep_outs(&candidate, &plans)?;
    Ok(candidate)
}

fn validate_groups(
    graph: &Graph,
    groups: &[ExpandedGroupConstraint],
) -> Result<Vec<ValidatedGroup>, ExpandedGroupLayoutError> {
    if groups.len() > MAX_EXPANDED_GROUPS {
        return Err(ExpandedGroupLayoutError::TooManyGroups {
            actual: groups.len(),
            maximum: MAX_EXPANDED_GROUPS,
        });
    }
    let total_members = groups
        .iter()
        .try_fold(0usize, |total, group| {
            total.checked_add(group.members.len())
        })
        .unwrap_or(usize::MAX);
    if total_members > MAX_EXPANDED_GROUP_MEMBERS {
        return Err(ExpandedGroupLayoutError::TooManyMembers {
            actual: total_members,
            maximum: MAX_EXPANDED_GROUP_MEMBERS,
        });
    }
    let node_ids = graph
        .nodes
        .iter()
        .map(|node| node.id)
        .collect::<BTreeSet<_>>();
    let mut groups = groups.iter().collect::<Vec<_>>();
    groups.sort_by_key(|group| group.id);
    let mut prior_group = None;
    let mut owner_by_member = BTreeMap::new();
    let mut validated = Vec::with_capacity(groups.len());
    for group in groups {
        if prior_group == Some(group.id) {
            return Err(ExpandedGroupLayoutError::DuplicateGroup(group.id));
        }
        prior_group = Some(group.id);
        if group.members.is_empty() {
            return Err(ExpandedGroupLayoutError::EmptyGroup(group.id));
        }
        if !group.reference_height.is_finite() || group.reference_height <= 0.0 {
            return Err(ExpandedGroupLayoutError::InvalidReferenceHeight {
                group: group.id,
                height: group.reference_height,
            });
        }
        if !group.frame_padding.is_finite() || group.frame_padding < 0.0 {
            return Err(ExpandedGroupLayoutError::InvalidFramePadding {
                group: group.id,
                padding: group.frame_padding,
            });
        }
        let members = group.members.iter().copied().collect::<BTreeSet<_>>();
        if members.len() != group.members.len() {
            let mut sorted = group.members.clone();
            sorted.sort_unstable();
            let member = sorted
                .windows(2)
                .find_map(|pair| (pair[0] == pair[1]).then_some(pair[0]))
                .expect("member count changed");
            return Err(ExpandedGroupLayoutError::DuplicateMember {
                group: group.id,
                member,
            });
        }
        for &member in &members {
            if !node_ids.contains(&member) {
                return Err(ExpandedGroupLayoutError::UnknownMember {
                    group: group.id,
                    member,
                });
            }
            if let Some(&first) = owner_by_member.get(&member) {
                return Err(ExpandedGroupLayoutError::OverlappingGroups {
                    first,
                    second: group.id,
                    member,
                });
            }
            owner_by_member.insert(member, group.id);
        }
        validated.push(ValidatedGroup {
            id: group.id,
            members,
            reference_height: group.reference_height,
            frame_padding: group.frame_padding,
        });
    }
    Ok(validated)
}

struct ValidatedGroup {
    id: NodeId,
    members: BTreeSet<NodeId>,
    reference_height: f64,
    frame_padding: f64,
}

fn verify_keep_outs(layout: &Layout, plans: &[GroupPlan]) -> Result<(), ExpandedGroupLayoutError> {
    let nodes = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    for plan in plans {
        let bounds = Bounds::from_nodes(plan.members.iter().map(|id| nodes[id]))
            .expect("validated groups are non-empty");
        let frame = Bounds {
            left: bounds.left - plan.frame_padding,
            top: bounds.top - plan.frame_padding,
            right: bounds.right + plan.frame_padding,
            bottom: bounds.bottom + plan.frame_padding,
        };
        if let Some(node) = layout.nodes.iter().find(|node| {
            !plan.members.contains(&node.id)
                && frame.overlaps(Bounds {
                    left: node.x,
                    top: node.y,
                    right: node.x + node.width,
                    bottom: node.y + node.height,
                })
        }) {
            return Err(ExpandedGroupLayoutError::KeepOutUnsatisfied {
                group: plan.id,
                node: node.id,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use crate::{
        Edge, Endpoint, ExpandedGroupConstraint, Graph, LayoutConfig, LayoutConstraints, Node,
        NodeGeometry, Port, PortSide, layout_with_config,
    };

    fn boundary_node(id: u32, side: PortSide) -> Node {
        Node {
            id,
            width: 64.0,
            height: 46.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side,
                offset: 23.0,
            }],
        }
    }

    fn gate(id: u32) -> Node {
        Node {
            id,
            width: 76.0,
            height: 52.0,
            cycle_breaker: false,
            ports: vec![
                Port {
                    id: 0,
                    side: PortSide::West,
                    offset: 26.0,
                },
                Port {
                    id: 1,
                    side: PortSide::East,
                    offset: 26.0,
                },
            ],
        }
    }

    fn fixture() -> (Graph, LayoutConfig) {
        let mut nodes = vec![
            boundary_node(0, PortSide::East),
            boundary_node(100, PortSide::West),
        ];
        nodes.extend((1..=12).map(gate));
        let mut edges = Vec::new();
        for node in 1..=12 {
            edges.push(Edge {
                id: node * 2,
                source: Endpoint { node: 0, port: 0 },
                target: Endpoint { node, port: 0 },
                net: 1,
                participates_in_ranking: true,
            });
            edges.push(Edge {
                id: node * 2 + 1,
                source: Endpoint { node, port: 1 },
                target: Endpoint { node: 100, port: 0 },
                net: node + 1,
                participates_in_ranking: true,
            });
        }
        (
            Graph { nodes, edges },
            LayoutConfig {
                constraints: LayoutConstraints {
                    inputs: vec![0],
                    outputs: vec![100],
                    boundary_bundles: Vec::new(),
                },
                expanded_groups: vec![ExpandedGroupConstraint {
                    id: 90,
                    members: vec![1, 3, 5, 7, 9, 11],
                    reference_height: 120.0,
                    frame_padding: 20.0,
                }],
                ..LayoutConfig::default()
            },
        )
    }

    fn bounds<'a>(nodes: impl Iterator<Item = &'a NodeGeometry>) -> super::Bounds {
        super::Bounds::from_nodes(nodes).unwrap()
    }

    #[test]
    fn full_layout_packs_expanded_members_and_keeps_outsiders_out() {
        let (graph, config) = fixture();
        let layout = layout_with_config(&graph, &config).unwrap();
        let members = config.expanded_groups[0]
            .members
            .iter()
            .copied()
            .collect::<BTreeSet<_>>();
        let member_bounds = bounds(
            layout
                .nodes
                .iter()
                .filter(|node| members.contains(&node.id)),
        );
        let frame = super::Bounds {
            left: member_bounds.left - 20.0,
            top: member_bounds.top - 20.0,
            right: member_bounds.right + 20.0,
            bottom: member_bounds.bottom + 20.0,
        };
        let member_columns = layout
            .nodes
            .iter()
            .filter(|node| members.contains(&node.id))
            .map(|node| node.x.to_bits())
            .collect::<BTreeSet<_>>();

        assert!(member_columns.len() > 1);
        assert!(layout.nodes.iter().all(|node| {
            members.contains(&node.id)
                || !frame.overlaps(super::Bounds {
                    left: node.x,
                    top: node.y,
                    right: node.x + node.width,
                    bottom: node.y + node.height,
                })
        }));
    }

    #[test]
    fn full_layout_group_geometry_is_input_order_independent() {
        let (graph, config) = fixture();
        let expected = layout_with_config(&graph, &config).unwrap();
        let mut permuted_graph = graph;
        permuted_graph.nodes.reverse();
        permuted_graph.edges.reverse();
        let mut permuted_config = config;
        permuted_config.expanded_groups[0].members.reverse();

        assert_eq!(
            layout_with_config(&permuted_graph, &permuted_config).unwrap(),
            expected,
        );
    }

    #[test]
    fn full_layout_keeps_multiple_expanded_groups_atomic_and_disjoint() {
        let (graph, mut config) = fixture();
        config.expanded_groups.push(ExpandedGroupConstraint {
            id: 91,
            members: vec![2, 4, 6, 8, 10, 12],
            reference_height: 120.0,
            frame_padding: 20.0,
        });
        let layout = layout_with_config(&graph, &config).unwrap();

        for group in &config.expanded_groups {
            let members = group.members.iter().copied().collect::<BTreeSet<_>>();
            let member_bounds = bounds(
                layout
                    .nodes
                    .iter()
                    .filter(|node| members.contains(&node.id)),
            );
            let frame = super::Bounds {
                left: member_bounds.left - group.frame_padding,
                top: member_bounds.top - group.frame_padding,
                right: member_bounds.right + group.frame_padding,
                bottom: member_bounds.bottom + group.frame_padding,
            };
            assert!(layout.nodes.iter().all(|node| {
                members.contains(&node.id)
                    || !frame.overlaps(super::Bounds {
                        left: node.x,
                        top: node.y,
                        right: node.x + node.width,
                        bottom: node.y + node.height,
                    })
            }));
        }
        for (index, node) in layout.nodes.iter().enumerate() {
            let bounds = super::Bounds {
                left: node.x,
                top: node.y,
                right: node.x + node.width,
                bottom: node.y + node.height,
            };
            assert!(layout.nodes[index + 1..].iter().all(|other| {
                !bounds.overlaps(super::Bounds {
                    left: other.x,
                    top: other.y,
                    right: other.x + other.width,
                    bottom: other.y + other.height,
                })
            }));
        }
    }

    #[test]
    fn full_layout_rejects_overlapping_expanded_groups() {
        let (graph, mut config) = fixture();
        config.expanded_groups.push(ExpandedGroupConstraint {
            id: 91,
            members: vec![2, 3],
            reference_height: 120.0,
            frame_padding: 20.0,
        });

        assert_eq!(
            layout_with_config(&graph, &config).unwrap_err(),
            crate::ConstrainedLayoutError::ExpandedGroup(
                crate::ExpandedGroupLayoutError::OverlappingGroups {
                    first: 90,
                    second: 91,
                    member: 3,
                },
            ),
        );
    }
}
