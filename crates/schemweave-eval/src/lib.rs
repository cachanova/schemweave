#![forbid(unsafe_code)]

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
};

use schemweave::{
    BoundaryBundleRole, BoundaryBundleSegment, Edge, EdgeId, EdgeNodeClearanceError,
    EdgeNodeSegment, Endpoint, Graph, Layout, NetId, NetNodeRelation, NodeGeometry,
    ParallelSegment, Point, PortSide, measure_edge_node_clearance_bounded,
    measure_parallel_congestion, measure_parallel_separation_bounded,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct ScoreOptions {
    pub epsilon: f64,
    pub max_examples: usize,
    pub viewport_width: f64,
    pub viewport_height: f64,
    /// Perpendicular distance below which overlapping different-net parallel
    /// routes count as congested, in layout units.
    pub parallel_congestion_threshold: f64,
    /// Requested minimum distance between route segments and unrelated nodes.
    pub edge_node_clearance_threshold: f64,
    /// Maximum segment-node candidates visited by the exact clearance pass,
    /// including pairs later exempted as electrically related.
    pub max_edge_node_clearance_pair_visits: usize,
}

impl Default for ScoreOptions {
    fn default() -> Self {
        Self {
            epsilon: 1e-7,
            max_examples: 64,
            viewport_width: 1_600.0,
            viewport_height: 900.0,
            parallel_congestion_threshold: 4.0,
            edge_node_clearance_threshold: 20.0,
            max_edge_node_clearance_pair_visits: 5_000_000,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ViolationKind {
    Cardinality,
    DuplicateId,
    UnknownId,
    InvalidGeometry,
    WrongNodeSize,
    WrongEndpoint,
    WrongEndpointDirection,
    NonOrthogonal,
    ZeroLengthSegment,
}

#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct Violation {
    pub kind: ViolationKind,
    pub message: String,
}

#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct QualityReport {
    pub semantic_violations: usize,
    pub node_overlaps: usize,
    pub node_intersections: usize,
    pub unrelated_overlaps: usize,
    pub unrelated_contacts: usize,
    pub crossings: usize,
    /// Ranking edges whose target is not geometrically to the right of their source.
    pub ranking_direction_violations: usize,
    /// Edges between distinct ranking components, used as the directness denominator.
    pub forward_edge_count: usize,
    /// Total westward distance across routes that participate in forward ranking.
    pub reverse_x_length: f64,
    /// 95th-percentile routed length divided by endpoint Manhattan distance for forward edges.
    pub p95_forward_stretch: f64,
    /// Forward routes containing at least one westward segment.
    pub forward_routes_with_reverse_x: usize,
    /// Final non-ranking nets whose outer routes occupy both top and bottom bands.
    pub split_feedback_nets: usize,
    /// Nets excluded from forward ranking, used as the coherence denominator.
    pub feedback_net_count: usize,
    /// Unique direction changes in the physical same-net geometry.
    pub bends: usize,
    /// Unique known routes whose geometry is finite, nonzero, and orthogonal.
    pub scored_route_count: usize,
    /// Scored routes with no direction changes.
    pub straight_route_count: usize,
    /// Straight scored routes divided by `scored_route_count`.
    pub straight_route_ratio: f64,
    /// Largest number of direction changes on one scored route.
    pub max_bends_per_route: usize,
    /// Raw route segments, before overlapping same-net geometry is merged.
    pub segments: usize,
    /// Smallest positive perpendicular distance between overlapping parallel
    /// physical segments from different electrical nets, in layout units.
    pub minimum_parallel_route_separation: Option<f64>,
    /// Fraction of physical route length that runs parallel to a different
    /// electrical net closer than the configured threshold.
    pub parallel_congestion_ratio: f64,
    /// Sum of longitudinal overlap length for every different-net parallel
    /// segment pair closer than the configured threshold.
    pub parallel_pair_overlap_length: f64,
    /// Largest number of simultaneously close different-net parallel
    /// neighbors seen by one segment.
    pub peak_parallel_close_neighbors: usize,
    /// Smallest distance between any physical same-net route segment and unrelated node.
    pub minimum_edge_node_clearance: Option<f64>,
    /// Unique physical segment-node pairs below the configured clearance threshold.
    pub edge_node_clearance_violations: usize,
    /// Whether exact clearance measurement exhausted its configured work cap.
    pub edge_node_clearance_exhausted: bool,
    /// Largest number of unrelated physical crossings on one physical segment.
    pub max_crossings_on_segment: usize,
    /// Union length of the physical same-net geometry.
    pub route_length: f64,
    /// Fraction of raw per-edge route length eliminated by same-net physical sharing.
    pub shared_route_ratio: f64,
    /// Union route length strictly outside the axis-aligned node envelope.
    pub perimeter_route_length: f64,
    /// `perimeter_route_length` divided by physical `route_length`.
    pub perimeter_route_ratio: f64,
    pub area: f64,
    /// Layout spans relative to the configured viewport; lower is easier to fit.
    pub viewport_fit: f64,
    pub examples: Vec<Violation>,
}

impl QualityReport {
    pub fn passes_hard_gates(&self) -> bool {
        self.semantic_violations == 0
            && self.node_overlaps == 0
            && self.node_intersections == 0
            && self.unrelated_overlaps == 0
            && self.unrelated_contacts == 0
            && self.ranking_direction_violations == 0
    }

    fn violation(&mut self, options: ScoreOptions, kind: ViolationKind, message: String) {
        self.semantic_violations += 1;
        if self.examples.len() < options.max_examples {
            self.examples.push(Violation { kind, message });
        }
    }
}

pub fn score(graph: &Graph, layout: &Layout, options: ScoreOptions) -> QualityReport {
    let mut report = QualityReport {
        area: layout.width * layout.height,
        ..QualityReport::default()
    };
    if !options.epsilon.is_finite()
        || options.epsilon < 0.0
        || !options.viewport_width.is_finite()
        || options.viewport_width <= 0.0
        || !options.viewport_height.is_finite()
        || options.viewport_height <= 0.0
        || !options.parallel_congestion_threshold.is_finite()
        || options.parallel_congestion_threshold <= 0.0
        || !options.edge_node_clearance_threshold.is_finite()
        || options.edge_node_clearance_threshold < 0.0
    {
        report.violation(
            options,
            ViolationKind::InvalidGeometry,
            "score epsilon, viewport dimensions, and clearance thresholds must be finite and valid"
                .to_owned(),
        );
        return report;
    }
    if !layout.width.is_finite()
        || !layout.height.is_finite()
        || layout.width < 0.0
        || layout.height < 0.0
    {
        report.violation(
            options,
            ViolationKind::InvalidGeometry,
            "layout bounds must be finite and nonnegative".to_owned(),
        );
    } else {
        report.viewport_fit =
            (layout.width / options.viewport_width).max(layout.height / options.viewport_height);
    }
    let input_nodes: HashMap<_, _> = graph.nodes.iter().map(|node| (node.id, node)).collect();
    let input_edges: HashMap<_, _> = graph.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut bundle_taps = HashMap::<EdgeId, (Option<Point>, Option<Point>)>::new();
    for bundle in &layout.boundary_bundles {
        for member in &bundle.members {
            let taps = bundle_taps.entry(member.edge).or_default();
            let target = match bundle.role {
                BoundaryBundleRole::Input => &mut taps.0,
                BoundaryBundleRole::Output => &mut taps.1,
            };
            if target.replace(member.tap).is_some() {
                report.violation(
                    options,
                    ViolationKind::DuplicateId,
                    format!(
                        "route {} has duplicate {:?} boundary taps",
                        member.edge, bundle.role
                    ),
                );
            }
        }
    }
    let ranking_edges = effective_ranking_edges(graph);
    report.forward_edge_count = ranking_edges.len();
    let mut nodes = HashMap::with_capacity(layout.nodes.len());
    let mut seen_nodes = HashSet::with_capacity(layout.nodes.len());
    for node in &layout.nodes {
        if !seen_nodes.insert(node.id) {
            report.violation(
                options,
                ViolationKind::DuplicateId,
                format!("duplicate layout node {}", node.id),
            );
            continue;
        }
        let Some(input) = input_nodes.get(&node.id) else {
            report.violation(
                options,
                ViolationKind::UnknownId,
                format!("unknown layout node {}", node.id),
            );
            continue;
        };
        if !valid_rect(node, layout, options.epsilon) {
            report.violation(
                options,
                ViolationKind::InvalidGeometry,
                format!("node {} has invalid or out-of-bounds geometry", node.id),
            );
        }
        if !near(node.width, input.width, options.epsilon)
            || !near(node.height, input.height, options.epsilon)
        {
            report.violation(
                options,
                ViolationKind::WrongNodeSize,
                format!("node {} dimensions changed", node.id),
            );
        }
        nodes.insert(node.id, node);
    }
    if layout.nodes.len() != graph.nodes.len() {
        report.violation(
            options,
            ViolationKind::Cardinality,
            format!(
                "expected {} nodes, received {}",
                graph.nodes.len(),
                layout.nodes.len()
            ),
        );
    }

    for edge in graph
        .edges
        .iter()
        .filter(|edge| ranking_edges.contains(&edge.id))
    {
        let (Some(source), Some(target)) =
            (nodes.get(&edge.source.node), nodes.get(&edge.target.node))
        else {
            continue;
        };
        if source.x + source.width > target.x + options.epsilon {
            report.ranking_direction_violations += 1;
        }
    }

    let mut segments = Vec::new();
    let mut bend_points = BTreeSet::new();
    let mut forward_stretches = Vec::new();
    let mut seen_edges = HashSet::with_capacity(layout.edges.len());
    for route in &layout.edges {
        if !seen_edges.insert(route.id) {
            report.violation(
                options,
                ViolationKind::DuplicateId,
                format!("duplicate route {}", route.id),
            );
            continue;
        }
        let Some(edge) = input_edges.get(&route.id) else {
            report.violation(
                options,
                ViolationKind::UnknownId,
                format!("unknown route {}", route.id),
            );
            continue;
        };
        if let Some(route_bends) = validate_route(
            graph,
            edge,
            route.points.as_slice(),
            &nodes,
            bundle_taps.get(&edge.id).copied(),
            layout,
            options,
            &mut report,
            &mut segments,
            &mut bend_points,
        ) {
            report.scored_route_count += 1;
            if route_bends == 0 {
                report.straight_route_count += 1;
            }
            report.max_bends_per_route = report.max_bends_per_route.max(route_bends);
        }
        if ranking_edges.contains(&edge.id) {
            score_forward_route(
                route.points.as_slice(),
                options.epsilon,
                &mut report,
                &mut forward_stretches,
            );
        }
    }
    if layout.edges.len() != graph.edges.len() {
        report.violation(
            options,
            ViolationKind::Cardinality,
            format!(
                "expected {} edges, received {}",
                graph.edges.len(),
                layout.edges.len()
            ),
        );
    }
    if report.scored_route_count != 0 {
        report.straight_route_ratio =
            report.straight_route_count as f64 / report.scored_route_count as f64;
    }
    let boundary_segments = boundary_bundle_scoring_segments(graph, layout);
    report.segments = segments.len() + boundary_segments.len();
    score_node_overlaps(&layout.nodes, &mut report);
    score_node_intersections(&segments, &layout.nodes, options.epsilon, &mut report);
    score_boundary_bundle_node_intersections(layout, options, &mut report);
    let physical_segments = merged_net_segments(&segments);
    let mut clearance_segments = physical_segments
        .iter()
        .map(|segment| EdgeNodeSegment {
            net: segment.net,
            horizontal: segment.orientation == Orientation::Horizontal,
            fixed: segment.fixed,
            start: segment.start,
            end: segment.end,
        })
        .collect::<Vec<_>>();
    let mut clearance_relations = graph
        .edges
        .iter()
        .flat_map(|edge| {
            [
                NetNodeRelation {
                    net: edge.net,
                    node: edge.source.node,
                },
                NetNodeRelation {
                    net: edge.net,
                    node: edge.target.node,
                },
            ]
        })
        .collect::<Vec<_>>();
    append_boundary_bundle_clearance_geometry(
        graph,
        layout,
        &mut clearance_segments,
        &mut clearance_relations,
        options,
        &mut report,
    );
    match measure_edge_node_clearance_bounded(
        &clearance_segments,
        &layout.nodes,
        &clearance_relations,
        options.edge_node_clearance_threshold,
        options.max_edge_node_clearance_pair_visits,
    ) {
        Ok(clearance) => {
            report.minimum_edge_node_clearance = clearance.minimum_clearance;
            report.edge_node_clearance_violations = clearance.violations;
        }
        Err(EdgeNodeClearanceError::WorkLimitExceeded) => {
            report.edge_node_clearance_exhausted = true;
        }
        Err(EdgeNodeClearanceError::InvalidInput) => {}
        Err(_) => {}
    }
    let bundle_length = boundary_bundle_length(layout);
    let raw_route_length: f64 = segments
        .iter()
        .map(|segment| segment.end - segment.start)
        .sum::<f64>()
        + bundle_length;
    report.route_length = physical_segments
        .iter()
        .map(|segment| segment.end - segment.start)
        .sum::<f64>()
        + bundle_length;
    let parallel_segments = physical_segments
        .iter()
        .map(|segment| ParallelSegment {
            net: segment.net,
            horizontal: segment.orientation == Orientation::Horizontal,
            fixed: segment.fixed,
            start: segment.start,
            end: segment.end,
        })
        .chain(
            boundary_segments
                .iter()
                .map(|(_, segment)| ParallelSegment {
                    net: segment.net,
                    horizontal: segment.orientation == Orientation::Horizontal,
                    fixed: segment.fixed,
                    start: segment.start,
                    end: segment.end,
                }),
        )
        .collect::<Vec<_>>();
    report.minimum_parallel_route_separation =
        measure_parallel_separation_bounded(&parallel_segments, usize::MAX)
            .expect("validated physical segments have a finite exact separation sweep")
            .minimum_positive;
    let parallel_congestion = measure_parallel_congestion(
        &parallel_segments,
        options.parallel_congestion_threshold - options.epsilon,
    );
    report.parallel_pair_overlap_length = parallel_congestion.pair_overlap_length;
    report.peak_parallel_close_neighbors = parallel_congestion.peak_close_neighbors;
    report.parallel_congestion_ratio = if report.route_length > options.epsilon {
        parallel_congestion.ratio()
    } else {
        0.0
    };
    report.perimeter_route_length =
        perimeter_route_length(&physical_segments, &layout.nodes, options.epsilon);
    if report.route_length > options.epsilon {
        report.perimeter_route_ratio = report.perimeter_route_length / report.route_length;
    }
    if raw_route_length > options.epsilon {
        report.shared_route_ratio =
            ((raw_route_length - report.route_length) / raw_route_length).clamp(0.0, 1.0);
    }
    forward_stretches.sort_by(f64::total_cmp);
    if !forward_stretches.is_empty() {
        report.p95_forward_stretch =
            forward_stretches[(forward_stretches.len() * 95).div_ceil(100) - 1];
    }
    (report.split_feedback_nets, report.feedback_net_count) =
        split_feedback_nets(graph, layout, &input_edges, &ranking_edges, options.epsilon);
    report.bends = bend_points.len()
        + layout
            .boundary_bundles
            .iter()
            .filter(|bundle| {
                bundle.collector.start != bundle.collector.end
                    && bundle.spine.start != bundle.spine.end
            })
            .count();
    score_segment_relationships(&segments, &physical_segments, &input_edges, &mut report);
    score_boundary_bundle_relationships(
        layout,
        &physical_segments,
        &boundary_segments,
        &mut report,
    );
    report
}

fn effective_ranking_edges(graph: &Graph) -> HashSet<EdgeId> {
    schemweave::effective_ranking_edges(graph)
        .into_iter()
        .collect()
}

fn score_forward_route(
    points: &[Point],
    epsilon: f64,
    report: &mut QualityReport,
    stretches: &mut Vec<f64>,
) {
    if points.len() < 2
        || points
            .iter()
            .any(|point| !point.x.is_finite() || !point.y.is_finite())
    {
        return;
    }
    let mut route_length = 0.0;
    let mut reverse_x = 0.0;
    for pair in points.windows(2) {
        route_length += (pair[1].x - pair[0].x).abs() + (pair[1].y - pair[0].y).abs();
        if pair[1].x < pair[0].x - epsilon {
            reverse_x += pair[0].x - pair[1].x;
        }
    }
    report.reverse_x_length += reverse_x;
    if reverse_x > epsilon {
        report.forward_routes_with_reverse_x += 1;
    }
    let first = points[0];
    let last = points[points.len() - 1];
    let minimum = (last.x - first.x).abs() + (last.y - first.y).abs();
    if minimum > epsilon {
        stretches.push(route_length / minimum);
    }
}

fn split_feedback_nets(
    graph: &Graph,
    layout: &Layout,
    edges: &HashMap<EdgeId, &Edge>,
    ranking_edges: &HashSet<EdgeId>,
    epsilon: f64,
) -> (usize, usize) {
    let Some(top) = layout.nodes.iter().map(|node| node.y).reduce(f64::min) else {
        return (0, 0);
    };
    let Some(bottom) = layout
        .nodes
        .iter()
        .map(|node| node.y + node.height)
        .reduce(f64::max)
    else {
        return (0, 0);
    };
    let feedback_nets = graph
        .edges
        .iter()
        .filter(|edge| !ranking_edges.contains(&edge.id))
        .map(|edge| edge.net)
        .collect::<HashSet<_>>();
    let mut sides_by_net = HashMap::<NetId, u8>::new();
    for route in &layout.edges {
        let Some(edge) = edges.get(&route.id) else {
            continue;
        };
        if !feedback_nets.contains(&edge.net) {
            continue;
        }
        let sides = sides_by_net.entry(edge.net).or_default();
        if route.points.iter().any(|point| point.y < top - epsilon) {
            *sides |= 1;
        }
        if route.points.iter().any(|point| point.y > bottom + epsilon) {
            *sides |= 2;
        }
    }
    (
        sides_by_net.values().filter(|&&sides| sides == 3).count(),
        feedback_nets.len(),
    )
}

fn score_node_overlaps(nodes: &[NodeGeometry], report: &mut QualityReport) {
    for (index, left) in nodes.iter().enumerate() {
        for right in nodes.iter().skip(index + 1) {
            if left.x < right.x + right.width
                && left.x + left.width > right.x
                && left.y < right.y + right.height
                && left.y + left.height > right.y
            {
                report.node_overlaps += 1;
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn validate_route(
    graph: &Graph,
    edge: &Edge,
    points: &[Point],
    nodes: &HashMap<u32, &NodeGeometry>,
    bundle_taps: Option<(Option<Point>, Option<Point>)>,
    layout: &Layout,
    options: ScoreOptions,
    report: &mut QualityReport,
    segments: &mut Vec<Segment>,
    bend_points: &mut BTreeSet<(NetId, FloatKey, FloatKey)>,
) -> Option<usize> {
    if points.len() < 2 {
        report.violation(
            options,
            ViolationKind::InvalidGeometry,
            format!("route {} has fewer than two points", edge.id),
        );
        return None;
    }
    let invalid_point = points.iter().any(|point| {
        !point.x.is_finite()
            || !point.y.is_finite()
            || point.x < -options.epsilon
            || point.y < -options.epsilon
            || point.x > layout.width + options.epsilon
            || point.y > layout.height + options.epsilon
    });
    if invalid_point {
        report.violation(
            options,
            ViolationKind::InvalidGeometry,
            format!("route {} contains invalid or out-of-bounds points", edge.id),
        );
    }
    for (endpoint, point, source) in [
        (edge.source, points[0], true),
        (edge.target, points[points.len() - 1], false),
    ] {
        let expected = match (bundle_taps, source) {
            (Some((Some(tap), _)), true) => Some((tap, PortSide::East)),
            (Some((_, Some(tap))), false) => Some((tap, PortSide::West)),
            _ => endpoint_point(graph, endpoint, nodes),
        };
        let Some(expected) = expected else {
            continue;
        };
        if !near_point(point, expected.0, options.epsilon) {
            report.violation(
                options,
                ViolationKind::WrongEndpoint,
                format!("route {} misses a fixed endpoint", edge.id),
            );
        }
        let adjacent = if source {
            points[1]
        } else {
            points[points.len() - 2]
        };
        if !correct_direction(point, adjacent, expected.1, options.epsilon) {
            report.violation(
                options,
                ViolationKind::WrongEndpointDirection,
                format!("route {} leaves or enters a port incorrectly", edge.id),
            );
        }
    }
    let mut valid_shape = !invalid_point;
    let mut previous = None::<Segment>;
    let mut route_bends = 0;
    for pair in points.windows(2) {
        let dx = (pair[1].x - pair[0].x).abs();
        let dy = (pair[1].y - pair[0].y).abs();
        if dx <= options.epsilon && dy <= options.epsilon {
            report.violation(
                options,
                ViolationKind::ZeroLengthSegment,
                format!("route {} contains a zero-length segment", edge.id),
            );
            valid_shape = false;
            previous = None;
            continue;
        }
        let orientation = if dx <= options.epsilon {
            Orientation::Vertical
        } else if dy <= options.epsilon {
            Orientation::Horizontal
        } else {
            report.violation(
                options,
                ViolationKind::NonOrthogonal,
                format!("route {} contains a diagonal segment", edge.id),
            );
            valid_shape = false;
            previous = None;
            continue;
        };
        let segment = Segment::new(edge.id, edge.net, pair[0], pair[1], orientation);
        if let Some(prior) = previous
            && prior.orientation != segment.orientation
        {
            route_bends += 1;
            let horizontal = if prior.orientation == Orientation::Horizontal {
                prior
            } else {
                segment
            };
            let vertical = if prior.orientation == Orientation::Vertical {
                prior
            } else {
                segment
            };
            bend_points.insert((
                edge.net,
                FloatKey(vertical.fixed),
                FloatKey(horizontal.fixed),
            ));
        }
        segments.push(segment);
        previous = Some(segment);
    }
    if valid_shape { Some(route_bends) } else { None }
}

fn endpoint_point(
    graph: &Graph,
    endpoint: Endpoint,
    nodes: &HashMap<u32, &NodeGeometry>,
) -> Option<(Point, PortSide)> {
    let input = graph.nodes.iter().find(|node| node.id == endpoint.node)?;
    let placed = nodes.get(&endpoint.node)?;
    let port = input.ports.iter().find(|port| port.id == endpoint.port)?;
    let point = match port.side {
        PortSide::North => Point {
            x: placed.x + port.offset,
            y: placed.y,
        },
        PortSide::East => Point {
            x: placed.x + placed.width,
            y: placed.y + port.offset,
        },
        PortSide::South => Point {
            x: placed.x + port.offset,
            y: placed.y + placed.height,
        },
        PortSide::West => Point {
            x: placed.x,
            y: placed.y + port.offset,
        },
    };
    Some((point, port.side))
}

fn correct_direction(endpoint: Point, adjacent: Point, side: PortSide, epsilon: f64) -> bool {
    let (dx, dy) = (adjacent.x - endpoint.x, adjacent.y - endpoint.y);
    match side {
        PortSide::North => dy < -epsilon && dx.abs() <= epsilon,
        PortSide::East => dx > epsilon && dy.abs() <= epsilon,
        PortSide::South => dy > epsilon && dx.abs() <= epsilon,
        PortSide::West => dx < -epsilon && dy.abs() <= epsilon,
    }
}

fn valid_rect(node: &NodeGeometry, layout: &Layout, epsilon: f64) -> bool {
    [node.x, node.y, node.width, node.height]
        .iter()
        .all(|value| value.is_finite())
        && node.x >= -epsilon
        && node.y >= -epsilon
        && node.width > 0.0
        && node.height > 0.0
        && node.x + node.width <= layout.width + epsilon
        && node.y + node.height <= layout.height + epsilon
}

fn near(left: f64, right: f64, epsilon: f64) -> bool {
    (left - right).abs() <= epsilon
}

fn near_point(left: Point, right: Point, epsilon: f64) -> bool {
    near(left.x, right.x, epsilon) && near(left.y, right.y, epsilon)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Orientation {
    Horizontal,
    Vertical,
}

#[derive(Clone, Copy, Debug)]
struct Segment {
    edge: EdgeId,
    net: NetId,
    orientation: Orientation,
    fixed: f64,
    start: f64,
    end: f64,
}

impl Segment {
    fn new(edge: EdgeId, net: NetId, a: Point, b: Point, orientation: Orientation) -> Self {
        match orientation {
            Orientation::Horizontal => Self {
                edge,
                net,
                orientation,
                fixed: a.y,
                start: a.x.min(b.x),
                end: a.x.max(b.x),
            },
            Orientation::Vertical => Self {
                edge,
                net,
                orientation,
                fixed: a.x,
                start: a.y.min(b.y),
                end: a.y.max(b.y),
            },
        }
    }
}

#[derive(Clone, Copy)]
struct NodeEnvelope {
    left: f64,
    right: f64,
    top: f64,
    bottom: f64,
}

fn perimeter_route_length(segments: &[Segment], nodes: &[NodeGeometry], epsilon: f64) -> f64 {
    let mut envelope = None::<NodeEnvelope>;
    for node in nodes.iter().filter(|node| {
        [node.x, node.y, node.width, node.height]
            .iter()
            .all(|value| value.is_finite())
            && node.width > 0.0
            && node.height > 0.0
    }) {
        let right = node.x + node.width;
        let bottom = node.y + node.height;
        envelope = Some(match envelope {
            Some(current) => NodeEnvelope {
                left: current.left.min(node.x),
                right: current.right.max(right),
                top: current.top.min(node.y),
                bottom: current.bottom.max(bottom),
            },
            None => NodeEnvelope {
                left: node.x,
                right,
                top: node.y,
                bottom,
            },
        });
    }
    let Some(envelope) = envelope else {
        return 0.0;
    };

    segments
        .iter()
        .map(|segment| match segment.orientation {
            Orientation::Horizontal
                if segment.fixed < envelope.top - epsilon
                    || segment.fixed > envelope.bottom + epsilon =>
            {
                segment.end - segment.start
            }
            Orientation::Vertical
                if segment.fixed < envelope.left - epsilon
                    || segment.fixed > envelope.right + epsilon =>
            {
                segment.end - segment.start
            }
            Orientation::Horizontal => {
                outside_interval_length(segment.start, segment.end, envelope.left, envelope.right)
            }
            Orientation::Vertical => {
                outside_interval_length(segment.start, segment.end, envelope.top, envelope.bottom)
            }
        })
        .sum()
}

fn outside_interval_length(start: f64, end: f64, inside_start: f64, inside_end: f64) -> f64 {
    let before = (end.min(inside_start) - start).max(0.0);
    let after = (end - start.max(inside_end)).max(0.0);
    before + after
}

fn score_node_intersections(
    segments: &[Segment],
    nodes: &[NodeGeometry],
    epsilon: f64,
    report: &mut QualityReport,
) {
    let mut grid = NodeGrid::new(nodes, 128.0);
    for segment in segments {
        for node_index in grid.candidates(segment) {
            let node = &nodes[node_index];
            let intersects = match segment.orientation {
                Orientation::Horizontal => {
                    segment.fixed > node.y + epsilon
                        && segment.fixed < node.y + node.height - epsilon
                        && segment.start < node.x + node.width - epsilon
                        && segment.end > node.x + epsilon
                }
                Orientation::Vertical => {
                    segment.fixed > node.x + epsilon
                        && segment.fixed < node.x + node.width - epsilon
                        && segment.start < node.y + node.height - epsilon
                        && segment.end > node.y + epsilon
                }
            };
            if intersects {
                report.node_intersections += 1;
            }
        }
    }
}

fn score_boundary_bundle_node_intersections(
    layout: &Layout,
    options: ScoreOptions,
    report: &mut QualityReport,
) {
    let epsilon = options.epsilon;
    for bundle in &layout.boundary_bundles {
        for segment in [bundle.collector, bundle.spine] {
            let segment = match bundle_segment(segment, bundle.id, bundle.id) {
                Ok(Some(segment)) => segment,
                Ok(None) => continue,
                Err(()) => {
                    report.violation(
                        options,
                        ViolationKind::InvalidGeometry,
                        format!("boundary bundle {} contains invalid geometry", bundle.id),
                    );
                    continue;
                }
            };
            for node in layout
                .nodes
                .iter()
                .filter(|node| node.id != bundle.endpoint.node)
            {
                let intersects = match segment.orientation {
                    Orientation::Horizontal => {
                        segment.fixed > node.y + epsilon
                            && segment.fixed < node.y + node.height - epsilon
                            && segment.start < node.x + node.width - epsilon
                            && segment.end > node.x + epsilon
                    }
                    Orientation::Vertical => {
                        segment.fixed > node.x + epsilon
                            && segment.fixed < node.x + node.width - epsilon
                            && segment.start < node.y + node.height - epsilon
                            && segment.end > node.y + epsilon
                    }
                };
                if intersects {
                    report.node_intersections += 1;
                }
            }
        }
    }
}

fn append_boundary_bundle_clearance_geometry(
    graph: &Graph,
    layout: &Layout,
    segments: &mut Vec<EdgeNodeSegment>,
    relations: &mut Vec<NetNodeRelation>,
    options: ScoreOptions,
    report: &mut QualityReport,
) {
    let mut used_nets = graph
        .edges
        .iter()
        .map(|edge| edge.net)
        .collect::<HashSet<_>>();
    let mut candidate = 0u32;
    for bundle in &layout.boundary_bundles {
        while used_nets.contains(&candidate) {
            candidate = candidate.wrapping_add(1);
        }
        let net = candidate;
        used_nets.insert(net);
        candidate = candidate.wrapping_add(1);
        relations.push(NetNodeRelation {
            net,
            node: bundle.endpoint.node,
        });
        for geometry in [bundle.collector, bundle.spine] {
            let segment = match bundle_segment(geometry, bundle.id, net) {
                Ok(Some(segment)) => segment,
                Ok(None) => continue,
                Err(()) => {
                    report.violation(
                        options,
                        ViolationKind::InvalidGeometry,
                        format!("boundary bundle {} contains invalid geometry", bundle.id),
                    );
                    continue;
                }
            };
            segments.push(EdgeNodeSegment {
                net,
                horizontal: segment.orientation == Orientation::Horizontal,
                fixed: segment.fixed,
                start: segment.start,
                end: segment.end,
            });
        }
    }
}

fn boundary_bundle_length(layout: &Layout) -> f64 {
    layout
        .boundary_bundles
        .iter()
        .flat_map(|bundle| [bundle.collector, bundle.spine])
        .map(|segment| {
            (segment.end.x - segment.start.x).abs() + (segment.end.y - segment.start.y).abs()
        })
        .sum()
}

fn bundle_segment(
    geometry: BoundaryBundleSegment,
    edge: EdgeId,
    net: NetId,
) -> Result<Option<Segment>, ()> {
    if ![
        geometry.start.x,
        geometry.start.y,
        geometry.end.x,
        geometry.end.y,
    ]
    .iter()
    .all(|value| value.is_finite())
    {
        return Err(());
    }
    if geometry.start == geometry.end {
        return Ok(None);
    }
    let orientation = if geometry.start.x == geometry.end.x {
        Orientation::Vertical
    } else if geometry.start.y == geometry.end.y {
        Orientation::Horizontal
    } else {
        return Err(());
    };
    Ok(Some(Segment::new(
        edge,
        net,
        geometry.start,
        geometry.end,
        orientation,
    )))
}

fn boundary_bundle_scoring_segments(graph: &Graph, layout: &Layout) -> Vec<(usize, Segment)> {
    let mut used_nets = graph
        .edges
        .iter()
        .map(|edge| edge.net)
        .collect::<HashSet<_>>();
    let mut candidate = 0u32;
    let mut result = Vec::with_capacity(layout.boundary_bundles.len() * 2);
    for (bundle_index, bundle) in layout.boundary_bundles.iter().enumerate() {
        while used_nets.contains(&candidate) {
            candidate = candidate.wrapping_add(1);
        }
        let net = candidate;
        used_nets.insert(net);
        candidate = candidate.wrapping_add(1);
        for geometry in [bundle.collector, bundle.spine] {
            if let Ok(Some(segment)) = bundle_segment(geometry, bundle.id, net) {
                result.push((bundle_index, segment));
            }
        }
    }
    result
}

#[derive(Clone, Copy)]
enum BoundaryRelationship {
    Overlap,
    Contact(Point),
    Crossing(Point),
}

fn score_boundary_bundle_relationships(
    layout: &Layout,
    routes: &[Segment],
    buses: &[(usize, Segment)],
    report: &mut QualityReport,
) {
    let permitted_taps = layout
        .boundary_bundles
        .iter()
        .map(|bundle| {
            bundle
                .members
                .iter()
                .map(|member| (member.edge, member.tap))
                .collect::<HashMap<_, _>>()
        })
        .collect::<Vec<_>>();
    let mut route_crossings = vec![0usize; routes.len()];
    let mut bus_crossings = vec![0usize; buses.len()];
    for (bus_index, &(bundle, bus)) in buses.iter().enumerate() {
        for (route_index, route) in routes.iter().enumerate() {
            let permitted = permitted_taps[bundle].get(&route.edge).copied();
            if let Some(relationship) = boundary_relationship(bus, *route) {
                if relationship_point(relationship).is_some_and(|point| {
                    permitted == Some(point) && segment_has_endpoint(*route, point)
                }) {
                    continue;
                }
                score_boundary_relationship(
                    relationship,
                    route_index,
                    bus_index,
                    &mut route_crossings,
                    &mut bus_crossings,
                    report,
                );
            }
        }
    }
    for left in 0..buses.len() {
        for right in left + 1..buses.len() {
            if buses[left].0 == buses[right].0 {
                continue;
            }
            if let Some(relationship) = boundary_relationship(buses[left].1, buses[right].1) {
                match relationship {
                    BoundaryRelationship::Overlap => report.unrelated_overlaps += 1,
                    BoundaryRelationship::Contact(_) => report.unrelated_contacts += 1,
                    BoundaryRelationship::Crossing(_) => {
                        report.crossings += 1;
                        bus_crossings[left] += 1;
                        bus_crossings[right] += 1;
                    }
                }
            }
        }
    }
    report.max_crossings_on_segment = report
        .max_crossings_on_segment
        .max(route_crossings.into_iter().max().unwrap_or(0))
        .max(bus_crossings.into_iter().max().unwrap_or(0));
}

fn segment_has_endpoint(segment: Segment, point: Point) -> bool {
    match segment.orientation {
        Orientation::Horizontal => {
            point.y == segment.fixed && (point.x == segment.start || point.x == segment.end)
        }
        Orientation::Vertical => {
            point.x == segment.fixed && (point.y == segment.start || point.y == segment.end)
        }
    }
}

fn score_boundary_relationship(
    relationship: BoundaryRelationship,
    route: usize,
    bus: usize,
    route_crossings: &mut [usize],
    bus_crossings: &mut [usize],
    report: &mut QualityReport,
) {
    match relationship {
        BoundaryRelationship::Overlap => report.unrelated_overlaps += 1,
        BoundaryRelationship::Contact(_) => report.unrelated_contacts += 1,
        BoundaryRelationship::Crossing(_) => {
            report.crossings += 1;
            route_crossings[route] += 1;
            bus_crossings[bus] += 1;
        }
    }
}

fn relationship_point(relationship: BoundaryRelationship) -> Option<Point> {
    match relationship {
        BoundaryRelationship::Overlap => None,
        BoundaryRelationship::Contact(point) | BoundaryRelationship::Crossing(point) => Some(point),
    }
}

fn boundary_relationship(left: Segment, right: Segment) -> Option<BoundaryRelationship> {
    if left.orientation == right.orientation {
        if left.fixed != right.fixed || left.start > right.end || right.start > left.end {
            return None;
        }
        let start = left.start.max(right.start);
        let end = left.end.min(right.end);
        if start < end {
            return Some(BoundaryRelationship::Overlap);
        }
        let point = match left.orientation {
            Orientation::Horizontal => Point {
                x: start,
                y: left.fixed,
            },
            Orientation::Vertical => Point {
                x: left.fixed,
                y: start,
            },
        };
        return Some(BoundaryRelationship::Contact(point));
    }
    let (horizontal, vertical) = if left.orientation == Orientation::Horizontal {
        (left, right)
    } else {
        (right, left)
    };
    if vertical.fixed < horizontal.start
        || vertical.fixed > horizontal.end
        || horizontal.fixed < vertical.start
        || horizontal.fixed > vertical.end
    {
        return None;
    }
    let point = Point {
        x: vertical.fixed,
        y: horizontal.fixed,
    };
    let endpoint = vertical.fixed == horizontal.start
        || vertical.fixed == horizontal.end
        || horizontal.fixed == vertical.start
        || horizontal.fixed == vertical.end;
    Some(if endpoint {
        BoundaryRelationship::Contact(point)
    } else {
        BoundaryRelationship::Crossing(point)
    })
}

struct NodeGrid {
    cell: f64,
    cells: HashMap<(i64, i64), Vec<usize>>,
    stamps: Vec<u32>,
    generation: u32,
}

impl NodeGrid {
    fn new(nodes: &[NodeGeometry], cell: f64) -> Self {
        let mut cells: HashMap<(i64, i64), Vec<usize>> = HashMap::new();
        for (index, node) in nodes.iter().enumerate() {
            for x in grid_coord(node.x, cell)..=grid_coord(node.x + node.width, cell) {
                for y in grid_coord(node.y, cell)..=grid_coord(node.y + node.height, cell) {
                    cells.entry((x, y)).or_default().push(index);
                }
            }
        }
        Self {
            cell,
            cells,
            stamps: vec![0; nodes.len()],
            generation: 0,
        }
    }

    fn candidates(&mut self, segment: &Segment) -> Vec<usize> {
        self.generation = self.generation.wrapping_add(1);
        if self.generation == 0 {
            self.stamps.fill(0);
            self.generation = 1;
        }
        let (min_x, max_x, min_y, max_y) = match segment.orientation {
            Orientation::Horizontal => (segment.start, segment.end, segment.fixed, segment.fixed),
            Orientation::Vertical => (segment.fixed, segment.fixed, segment.start, segment.end),
        };
        let mut result = Vec::new();
        for x in grid_coord(min_x, self.cell)..=grid_coord(max_x, self.cell) {
            for y in grid_coord(min_y, self.cell)..=grid_coord(max_y, self.cell) {
                if let Some(items) = self.cells.get(&(x, y)) {
                    for &item in items {
                        if self.stamps[item] != self.generation {
                            self.stamps[item] = self.generation;
                            result.push(item);
                        }
                    }
                }
            }
        }
        result
    }
}

fn grid_coord(value: f64, cell: f64) -> i64 {
    (value / cell).floor() as i64
}

fn score_segment_relationships(
    segments: &[Segment],
    physical_segments: &[Segment],
    edges: &HashMap<EdgeId, &Edge>,
    report: &mut QualityReport,
) {
    let mut collinear: BTreeMap<(u8, FloatKey), Vec<&Segment>> = BTreeMap::new();
    for segment in segments {
        let axis = u8::from(segment.orientation == Orientation::Vertical);
        collinear
            .entry((axis, FloatKey(segment.fixed)))
            .or_default()
            .push(segment);
    }
    for group in collinear.values_mut() {
        group.sort_by(|left, right| left.start.total_cmp(&right.start));
        for (index, left) in group.iter().enumerate() {
            for right in group.iter().skip(index + 1) {
                if right.start > left.end {
                    break;
                }
                if related(left, right, edges) {
                    continue;
                }
                if right.start < left.end.min(right.end) {
                    report.unrelated_overlaps += 1;
                } else {
                    report.unrelated_contacts += 1;
                }
            }
        }
    }

    let horizontal: Vec<_> = physical_segments
        .iter()
        .filter(|segment| segment.orientation == Orientation::Horizontal)
        .collect();
    let vertical: Vec<_> = physical_segments
        .iter()
        .filter(|segment| segment.orientation == Orientation::Vertical)
        .collect();
    let mut events: BTreeMap<FloatKey, SweepEvents> = BTreeMap::new();
    for (index, segment) in horizontal.iter().enumerate() {
        events
            .entry(FloatKey(segment.start))
            .or_default()
            .start
            .push(index);
        events
            .entry(FloatKey(segment.end))
            .or_default()
            .end
            .push(index);
    }
    for (index, segment) in vertical.iter().enumerate() {
        events
            .entry(FloatKey(segment.fixed))
            .or_default()
            .vertical
            .push(index);
    }
    let mut y_values: Vec<FloatKey> = horizontal.iter().map(|line| FloatKey(line.fixed)).collect();
    y_values.sort_unstable();
    y_values.dedup();
    let mut tree = Fenwick::new(y_values.len());
    // A range-add/point-query sweep attributes the same crossings to horizontal segments without
    // enumerating every perpendicular pair. Each segment snapshots the accumulator on entry.
    let mut crossing_accumulator = Fenwick::new(y_values.len());
    let mut crossing_start = vec![0i64; horizontal.len()];
    let mut related_crossings = vec![0i64; horizontal.len()];
    let mut active = vec![false; horizontal.len()];
    let related_horizontal = relation_index(&horizontal, edges);
    for group in events.values() {
        for &index in &group.end {
            let y = y_values
                .binary_search(&FloatKey(horizontal[index].fixed))
                .unwrap();
            let count =
                crossing_accumulator.point(y) - crossing_start[index] - related_crossings[index];
            report.max_crossings_on_segment =
                report.max_crossings_on_segment.max(count.max(0) as usize);
            active[index] = false;
            tree.add(y, -1);
        }
        for &index in &group.vertical {
            let line = vertical[index];
            let low = y_values.partition_point(|value| value.0 <= line.start);
            let high = y_values.partition_point(|value| value.0 < line.end);
            let mut count = tree.range(low, high);
            for candidate in related_candidates(line, &related_horizontal, edges) {
                if active[candidate] {
                    let across = horizontal[candidate];
                    if across.fixed > line.start && across.fixed < line.end {
                        count -= 1;
                        related_crossings[candidate] += 1;
                    }
                }
            }
            let count = count.max(0) as usize;
            report.crossings += count;
            report.max_crossings_on_segment = report.max_crossings_on_segment.max(count);
            if low < high {
                crossing_accumulator.add(low, 1);
                crossing_accumulator.add(high, -1);
            }
        }
        for &index in &group.start {
            active[index] = true;
            let y = y_values
                .binary_search(&FloatKey(horizontal[index].fixed))
                .unwrap();
            crossing_start[index] = crossing_accumulator.point(y);
            tree.add(y, 1);
        }
    }
    let contact_horizontal: Vec<_> = segments
        .iter()
        .filter(|segment| segment.orientation == Orientation::Horizontal)
        .collect();
    let contact_vertical: Vec<_> = segments
        .iter()
        .filter(|segment| segment.orientation == Orientation::Vertical)
        .collect();
    report.unrelated_contacts +=
        perpendicular_contacts(&contact_horizontal, &contact_vertical, edges);
}

fn merged_net_segments(segments: &[Segment]) -> Vec<Segment> {
    let mut groups: BTreeMap<(NetId, u8, FloatKey), Vec<Segment>> = BTreeMap::new();
    for &segment in segments {
        let axis = u8::from(segment.orientation == Orientation::Vertical);
        groups
            .entry((segment.net, axis, FloatKey(segment.fixed)))
            .or_default()
            .push(segment);
    }

    let mut merged = Vec::with_capacity(segments.len());
    for group in groups.values_mut() {
        group.sort_by(|left, right| {
            left.start
                .total_cmp(&right.start)
                .then(left.end.total_cmp(&right.end))
                .then(left.edge.cmp(&right.edge))
        });
        let mut current = group[0];
        for &segment in group.iter().skip(1) {
            if segment.start <= current.end {
                current.end = current.end.max(segment.end);
            } else {
                merged.push(current);
                current = segment;
            }
        }
        merged.push(current);
    }
    merged
}

#[derive(Default)]
struct SweepEvents {
    start: Vec<usize>,
    vertical: Vec<usize>,
    end: Vec<usize>,
}

#[derive(Default)]
struct RelationIndex {
    by_edge: HashMap<EdgeId, Vec<usize>>,
    by_net: HashMap<NetId, Vec<usize>>,
    by_endpoint: HashMap<Endpoint, Vec<usize>>,
}

fn relation_index(segments: &[&Segment], edges: &HashMap<EdgeId, &Edge>) -> RelationIndex {
    let mut index = RelationIndex::default();
    for (position, segment) in segments.iter().enumerate() {
        index
            .by_edge
            .entry(segment.edge)
            .or_default()
            .push(position);
        index.by_net.entry(segment.net).or_default().push(position);
        if let Some(edge) = edges.get(&segment.edge) {
            index
                .by_endpoint
                .entry(edge.source)
                .or_default()
                .push(position);
            index
                .by_endpoint
                .entry(edge.target)
                .or_default()
                .push(position);
        }
    }
    index
}

fn related_candidates(
    segment: &Segment,
    index: &RelationIndex,
    edges: &HashMap<EdgeId, &Edge>,
) -> HashSet<usize> {
    let mut result = HashSet::new();
    if let Some(items) = index.by_edge.get(&segment.edge) {
        result.extend(items);
    }
    if let Some(items) = index.by_net.get(&segment.net) {
        result.extend(items);
    }
    if let Some(edge) = edges.get(&segment.edge) {
        for endpoint in [edge.source, edge.target] {
            if let Some(items) = index.by_endpoint.get(&endpoint) {
                result.extend(items);
            }
        }
    }
    result
}

fn perpendicular_contacts(
    horizontal: &[&Segment],
    vertical: &[&Segment],
    edges: &HashMap<EdgeId, &Edge>,
) -> usize {
    let mut by_y: BTreeMap<FloatKey, Vec<(usize, &Segment)>> = BTreeMap::new();
    let mut by_x: BTreeMap<FloatKey, Vec<(usize, &Segment)>> = BTreeMap::new();
    for (index, line) in horizontal.iter().enumerate() {
        by_y.entry(FloatKey(line.fixed))
            .or_default()
            .push((index, line));
    }
    for (index, line) in vertical.iter().enumerate() {
        by_x.entry(FloatKey(line.fixed))
            .or_default()
            .push((index, line));
    }
    let mut pairs = HashSet::new();
    for (vertical_index, line) in vertical.iter().enumerate() {
        for y in [line.start, line.end] {
            if let Some(items) = by_y.get(&FloatKey(y)) {
                for &(horizontal_index, across) in items {
                    if line.fixed >= across.start
                        && line.fixed <= across.end
                        && !related(across, line, edges)
                    {
                        pairs.insert((horizontal_index, vertical_index));
                    }
                }
            }
        }
    }
    for (horizontal_index, line) in horizontal.iter().enumerate() {
        for x in [line.start, line.end] {
            if let Some(items) = by_x.get(&FloatKey(x)) {
                for &(vertical_index, across) in items {
                    if line.fixed >= across.start
                        && line.fixed <= across.end
                        && !related(line, across, edges)
                    {
                        pairs.insert((horizontal_index, vertical_index));
                    }
                }
            }
        }
    }
    pairs.len()
}

struct Fenwick {
    tree: Vec<i64>,
}

impl Fenwick {
    fn new(size: usize) -> Self {
        Self {
            tree: vec![0; size + 1],
        }
    }

    fn add(&mut self, index: usize, delta: i64) {
        let mut cursor = index + 1;
        while cursor < self.tree.len() {
            self.tree[cursor] += delta;
            cursor += cursor & cursor.wrapping_neg();
        }
    }

    fn prefix(&self, end: usize) -> i64 {
        let mut cursor = end;
        let mut total = 0;
        while cursor > 0 {
            total += self.tree[cursor];
            cursor &= cursor - 1;
        }
        total
    }

    fn range(&self, start: usize, end: usize) -> i64 {
        self.prefix(end) - self.prefix(start)
    }

    fn point(&self, index: usize) -> i64 {
        self.prefix(index + 1)
    }
}

fn related(left: &Segment, right: &Segment, edges: &HashMap<EdgeId, &Edge>) -> bool {
    if left.edge == right.edge || left.net == right.net {
        return true;
    }
    let Some(left_edge) = edges.get(&left.edge) else {
        return false;
    };
    let Some(right_edge) = edges.get(&right.edge) else {
        return false;
    };
    [left_edge.source, left_edge.target]
        .iter()
        .any(|endpoint| *endpoint == right_edge.source || *endpoint == right_edge.target)
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
