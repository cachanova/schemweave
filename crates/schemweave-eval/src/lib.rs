#![forbid(unsafe_code)]

use std::{
    cmp::Ordering,
    collections::{BTreeMap, HashMap, HashSet},
};

use schemweave::{Edge, EdgeId, Endpoint, Graph, Layout, NetId, NodeGeometry, Point, PortSide};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
#[serde(default)]
pub struct ScoreOptions {
    pub epsilon: f64,
    pub max_examples: usize,
}

impl Default for ScoreOptions {
    fn default() -> Self {
        Self {
            epsilon: 1e-7,
            max_examples: 64,
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
pub struct QualityReport {
    pub semantic_violations: usize,
    pub node_overlaps: usize,
    pub node_intersections: usize,
    pub unrelated_overlaps: usize,
    pub unrelated_contacts: usize,
    pub crossings: usize,
    pub bends: usize,
    pub segments: usize,
    pub route_length: f64,
    pub area: f64,
    pub examples: Vec<Violation>,
}

impl QualityReport {
    pub fn passes_hard_gates(&self) -> bool {
        self.semantic_violations == 0
            && self.node_overlaps == 0
            && self.node_intersections == 0
            && self.unrelated_overlaps == 0
            && self.unrelated_contacts == 0
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
    if !options.epsilon.is_finite() || options.epsilon < 0.0 {
        report.violation(
            options,
            ViolationKind::InvalidGeometry,
            "score epsilon must be finite and nonnegative".to_owned(),
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
    }
    let input_nodes: HashMap<_, _> = graph.nodes.iter().map(|node| (node.id, node)).collect();
    let input_edges: HashMap<_, _> = graph.edges.iter().map(|edge| (edge.id, edge)).collect();
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

    let mut segments = Vec::new();
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
        validate_route(
            graph,
            edge,
            route.points.as_slice(),
            &nodes,
            layout,
            options,
            &mut report,
            &mut segments,
        );
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
    report.segments = segments.len();
    score_node_overlaps(&layout.nodes, &mut report);
    score_node_intersections(&segments, &layout.nodes, options.epsilon, &mut report);
    score_segment_relationships(&segments, &input_edges, &mut report);
    report
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
    layout: &Layout,
    options: ScoreOptions,
    report: &mut QualityReport,
    segments: &mut Vec<Segment>,
) {
    if points.len() < 2 {
        report.violation(
            options,
            ViolationKind::InvalidGeometry,
            format!("route {} has fewer than two points", edge.id),
        );
        return;
    }
    if points.iter().any(|point| {
        !point.x.is_finite()
            || !point.y.is_finite()
            || point.x < -options.epsilon
            || point.y < -options.epsilon
            || point.x > layout.width + options.epsilon
            || point.y > layout.height + options.epsilon
    }) {
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
        let Some(expected) = endpoint_point(graph, endpoint, nodes) else {
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
    let before = segments.len();
    for pair in points.windows(2) {
        let dx = (pair[1].x - pair[0].x).abs();
        let dy = (pair[1].y - pair[0].y).abs();
        if dx <= options.epsilon && dy <= options.epsilon {
            report.violation(
                options,
                ViolationKind::ZeroLengthSegment,
                format!("route {} contains a zero-length segment", edge.id),
            );
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
            continue;
        };
        report.route_length += dx + dy;
        segments.push(Segment::new(
            edge.id,
            edge.net,
            pair[0],
            pair[1],
            orientation,
        ));
    }
    report.bends += segments.len().saturating_sub(before + 1);
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

    let crossing_segments = merged_net_segments(segments);
    let horizontal: Vec<_> = crossing_segments
        .iter()
        .filter(|segment| segment.orientation == Orientation::Horizontal)
        .collect();
    let vertical: Vec<_> = crossing_segments
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
    let mut active = vec![false; horizontal.len()];
    let related_horizontal = relation_index(&horizontal, edges);
    for group in events.values() {
        for &index in &group.end {
            active[index] = false;
            let y = y_values
                .binary_search(&FloatKey(horizontal[index].fixed))
                .unwrap();
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
                    }
                }
            }
            report.crossings += count.max(0) as usize;
        }
        for &index in &group.start {
            active[index] = true;
            let y = y_values
                .binary_search(&FloatKey(horizontal[index].fixed))
                .unwrap();
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
