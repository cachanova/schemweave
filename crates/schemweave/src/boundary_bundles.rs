use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::{
    BoundaryBundleGeometry, BoundaryBundleMemberGeometry, BoundaryBundleRole,
    BoundaryBundleSegment, EdgeGeometry, EdgeNodeClearanceError, EdgeNodeSegment, Layout,
    LayoutError, LayoutOptions, NetNodeRelation, NodeGeometry, Point, Port, PortSide,
    measure_edge_node_clearance_bounded,
    validation::{IndexedBoundaryBundle, IndexedGraph},
};

pub(crate) const MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS: usize = 20_000_000;
const MAX_INTERIOR_COLLECTOR_BUNDLES: usize = 32;
const BUNDLE_CLEARANCE_NET_BASE: u32 = 0x8000_0000;
const PRESERVED_GEOMETRY_EPSILON: f64 = 1e-7;

struct BundleGeometryContext<'layout, 'graph> {
    graph: &'layout IndexedGraph<'graph>,
    nodes: &'layout [NodeGeometry],
    node_geometry: &'layout HashMap<u32, usize>,
    route_index: &'layout HashMap<u32, usize>,
    input_fallback_corridors: &'layout HashMap<u32, f64>,
    output_fallback_corridors: &'layout HashMap<u32, f64>,
    pitch: f64,
    rail_depth: f64,
}

pub(crate) fn preserved_point_matches(left: Point, right: Point) -> bool {
    (left.x - right.x).abs() <= PRESERVED_GEOMETRY_EPSILON
        && (left.y - right.y).abs() <= PRESERVED_GEOMETRY_EPSILON
}

fn preserved_segment_matches(left: BoundaryBundleSegment, right: BoundaryBundleSegment) -> bool {
    preserved_point_matches(left.start, right.start) && preserved_point_matches(left.end, right.end)
}

/// Additional horizontal depth assigned to each bundle's member-route corridors.
///
/// Boundary nodes share an aligned outer edge, but may have different widths. Reserving corridor
/// lanes independently per bundle can therefore put unrelated member routes on the same absolute
/// x coordinate, creating an electrical-looking T contact after route rewriting. Bundles are
/// already stable-ID ordered, so allocate disjoint depth ranges per boundary role in one bounded
/// pass. Input and output ranges remain independent because they open from opposite boundaries.
pub(crate) fn corridor_depth_offsets(graph: &IndexedGraph<'_>, options: LayoutOptions) -> Vec<f64> {
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let rail_depth = crate::boundary_bundle_rail_depth(options);
    let mut furthest_input = None::<f64>;
    let mut furthest_output = None::<f64>;
    graph
        .boundary_bundles
        .iter()
        .map(|bundle| {
            let node = graph.nodes[graph.node_index[&bundle.endpoint.node]];
            let boundary_depth = node.width + rail_depth;
            let furthest = match bundle.role {
                BoundaryBundleRole::Input => &mut furthest_input,
                BoundaryBundleRole::Output => &mut furthest_output,
            };
            let first_nominal = boundary_depth + pitch;
            let offset = furthest.map_or(0.0, |used| (used + pitch - first_nominal).max(0.0));
            let lane_count = bundle
                .members
                .last()
                .map_or(0, |member| member.tap_lane + 1);
            let last = boundary_depth + offset + lane_count as f64 * pitch;
            *furthest = Some(furthest.map_or(last, |used| used.max(last)));
            offset
        })
        .collect()
}

pub(crate) fn route_quality(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
) -> crate::routing::RouteQuality {
    let mut quality = crate::routing::route_quality(graph, &layout.edges);
    for bundle in &layout.boundary_bundles {
        let collector_length = segment_length(bundle.collector);
        let spine_length = segment_length(bundle.spine);
        quality.route_length += collector_length + spine_length;
        if collector_length > 0.0 && spine_length > 0.0 {
            quality.bends = quality.bends.saturating_add(1);
        }
    }
    quality
}

fn segment_length(segment: BoundaryBundleSegment) -> f64 {
    (segment.end.x - segment.start.x).abs() + (segment.end.y - segment.start.y).abs()
}

pub(crate) fn apply_and_normalize(
    graph: &IndexedGraph<'_>,
    mut layout: Layout,
    options: LayoutOptions,
) -> Result<Layout, LayoutError> {
    if graph.boundary_bundles.is_empty() {
        return Ok(layout);
    }
    if !layout.boundary_bundles.is_empty() && verify_geometry(graph, &layout, options).is_ok() {
        normalize_layout(&mut layout);
        return Ok(layout);
    }
    let interior_allowed = options.edge_node_clearance == 0.0
        && options.minimum_parallel_wire_spacing == 0.0
        && graph.boundary_bundles.len() <= MAX_INTERIOR_COLLECTOR_BUNDLES;
    apply_bundle_geometry(graph, layout, options, interior_allowed)
}

fn apply_bundle_geometry(
    graph: &IndexedGraph<'_>,
    mut layout: Layout,
    options: LayoutOptions,
    allow_interior_collectors: bool,
) -> Result<Layout, LayoutError> {
    let node_geometry = layout
        .nodes
        .iter()
        .enumerate()
        .map(|(index, node)| (node.id, index))
        .collect::<HashMap<_, _>>();
    let route_index = layout
        .edges
        .iter()
        .enumerate()
        .map(|(index, route)| (route.id, index))
        .collect::<HashMap<_, _>>();
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let rail_depth = crate::boundary_bundle_rail_depth(options);
    let corridor_offsets = corridor_depth_offsets(graph, options);
    let (input_fallback_corridors, output_fallback_corridors) = fallback_corridors_by_edge(
        graph,
        &layout.nodes,
        &node_geometry,
        &corridor_offsets,
        pitch,
        rail_depth,
    );
    let geometry_context = BundleGeometryContext {
        graph,
        nodes: &layout.nodes,
        node_geometry: &node_geometry,
        route_index: &route_index,
        input_fallback_corridors: &input_fallback_corridors,
        output_fallback_corridors: &output_fallback_corridors,
        pitch,
        rail_depth,
    };
    layout.boundary_bundles.clear();
    for (bundle_index, (bundle, corridor_offset)) in graph
        .boundary_bundles
        .iter()
        .zip(corridor_offsets)
        .enumerate()
    {
        let preserved = allow_interior_collectors.then(|| {
            bundle
                .members
                .iter()
                .map(|member| {
                    let index = route_index[&member.edge];
                    (index, layout.edges[index].points.clone())
                })
                .collect::<Vec<_>>()
        });
        let mut geometry = build_geometry(
            &geometry_context,
            bundle,
            &layout.edges,
            allow_interior_collectors,
        );
        rewrite_member_routes(
            bundle,
            &geometry,
            &route_index,
            &mut layout.edges,
            pitch,
            rail_depth,
            corridor_offset,
        )?;
        layout.boundary_bundles.push(geometry);
        if allow_interior_collectors
            && partial_geometry_is_clean(graph, &layout, options, bundle_index).is_err()
        {
            layout.boundary_bundles.pop();
            let Some(preserved) = preserved else {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            };
            for (index, points) in preserved {
                layout.edges[index].points = points;
            }
            geometry = build_geometry(&geometry_context, bundle, &layout.edges, false);
            rewrite_member_routes(
                bundle,
                &geometry,
                &route_index,
                &mut layout.edges,
                pitch,
                rail_depth,
                corridor_offset,
            )?;
            layout.boundary_bundles.push(geometry);
            partial_geometry_is_clean(graph, &layout, options, bundle_index)?;
        }
    }
    normalize_layout(&mut layout);
    verify_geometry(graph, &layout, options)?;
    Ok(layout)
}

fn fallback_corridors_by_edge(
    graph: &IndexedGraph<'_>,
    nodes: &[NodeGeometry],
    node_geometry: &HashMap<u32, usize>,
    corridor_offsets: &[f64],
    pitch: f64,
    rail_depth: f64,
) -> (HashMap<u32, f64>, HashMap<u32, f64>) {
    let mut inputs = HashMap::new();
    let mut outputs = HashMap::new();
    for (bundle, corridor_offset) in graph.boundary_bundles.iter().zip(corridor_offsets) {
        let node = &nodes[node_geometry[&bundle.endpoint.node]];
        let indexed_node = graph.node_index[&bundle.endpoint.node];
        let endpoint = port_point(node, graph.ports[indexed_node][&bundle.endpoint.port]);
        let direction = match bundle.role {
            BoundaryBundleRole::Input => 1.0,
            BoundaryBundleRole::Output => -1.0,
        };
        let tap_x = endpoint.x + direction * rail_depth;
        for member in &bundle.members {
            let corridor_depth = corridor_offset + (member.tap_lane + 1) as f64 * pitch;
            let corridor_x = tap_x + direction * corridor_depth;
            match bundle.role {
                BoundaryBundleRole::Input => {
                    inputs.insert(member.edge, corridor_x);
                }
                BoundaryBundleRole::Output => {
                    outputs.insert(member.edge, corridor_x);
                }
            }
        }
    }
    (inputs, outputs)
}

fn build_geometry(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    routes: &[EdgeGeometry],
    allow_interior_collector: bool,
) -> BoundaryBundleGeometry {
    let node = &context.nodes[context.node_geometry[&bundle.endpoint.node]];
    let indexed_node = context.graph.node_index[&bundle.endpoint.node];
    let port = context.graph.ports[indexed_node][&bundle.endpoint.port];
    let endpoint = port_point(node, port);
    let direction = match bundle.role {
        BoundaryBundleRole::Input => 1.0,
        BoundaryBundleRole::Output => -1.0,
    };
    if allow_interior_collector
        && bundle.width > 1
        && let Some(geometry) = build_interior_geometry(context, bundle, endpoint, routes)
    {
        return geometry;
    }
    let collector_end = Point {
        x: endpoint.x + direction * context.rail_depth,
        y: endpoint.y,
    };
    let lane_count = bundle
        .members
        .last()
        .map_or(0, |member| member.tap_lane + 1);
    let center = lane_count.saturating_sub(1) as f64 / 2.0;
    let members = bundle
        .members
        .iter()
        .map(|member| BoundaryBundleMemberGeometry {
            edge: member.edge,
            slots: member.slots.clone(),
            tap: Point {
                x: collector_end.x,
                y: collector_end.y + (member.tap_lane as f64 - center) * context.pitch,
            },
        })
        .collect::<Vec<_>>();
    let spine_start = Point {
        x: collector_end.x,
        y: members
            .first()
            .map_or(collector_end.y, |member| member.tap.y.min(collector_end.y)),
    };
    let spine_end = Point {
        x: collector_end.x,
        y: members
            .last()
            .map_or(collector_end.y, |member| member.tap.y.max(collector_end.y)),
    };
    BoundaryBundleGeometry {
        id: bundle.id,
        endpoint: bundle.endpoint,
        role: bundle.role,
        width: bundle.width,
        collector: BoundaryBundleSegment {
            start: spine_start,
            end: spine_end,
        },
        spine: BoundaryBundleSegment {
            start: endpoint,
            end: collector_end,
        },
        members,
    }
}

fn build_interior_geometry(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    endpoint: Point,
    routes: &[EdgeGeometry],
) -> Option<BoundaryBundleGeometry> {
    let mut terminal_x = match bundle.role {
        BoundaryBundleRole::Input => f64::INFINITY,
        BoundaryBundleRole::Output => f64::NEG_INFINITY,
    };
    for member in &bundle.members {
        let route = routes.get(*context.route_index.get(&member.edge)?)?;
        let mut member_terminal_x = match bundle.role {
            BoundaryBundleRole::Input => route.points.last()?.x - context.rail_depth,
            BoundaryBundleRole::Output => route.points.first()?.x + context.rail_depth,
        };
        member_terminal_x = match bundle.role {
            BoundaryBundleRole::Input => context
                .output_fallback_corridors
                .get(&member.edge)
                .map_or(member_terminal_x, |corridor| {
                    member_terminal_x.min(*corridor)
                }),
            BoundaryBundleRole::Output => context
                .input_fallback_corridors
                .get(&member.edge)
                .map_or(member_terminal_x, |corridor| {
                    member_terminal_x.max(*corridor)
                }),
        };
        terminal_x = match bundle.role {
            BoundaryBundleRole::Input => terminal_x.min(member_terminal_x),
            BoundaryBundleRole::Output => terminal_x.max(member_terminal_x),
        };
    }
    let direction = match bundle.role {
        BoundaryBundleRole::Input => 1.0,
        BoundaryBundleRole::Output => -1.0,
    };
    let minimum_x = endpoint.x + direction * (context.rail_depth + context.pitch);
    if !terminal_x.is_finite() || direction * (terminal_x - minimum_x) < 0.0 {
        return None;
    }
    let collector_x = terminal_x;
    let mut members = Vec::with_capacity(bundle.members.len());
    let mut collector_low = endpoint.y;
    let mut collector_high = endpoint.y;
    for member in &bundle.members {
        let route = routes.get(*context.route_index.get(&member.edge)?)?;
        let (_, tap) = match bundle.role {
            BoundaryBundleRole::Input => {
                first_vertical_line_intersection(&route.points, collector_x)
            }
            BoundaryBundleRole::Output => {
                last_vertical_line_intersection(&route.points, collector_x)
            }
        }?;
        collector_low = collector_low.min(tap.y);
        collector_high = collector_high.max(tap.y);
        members.push(BoundaryBundleMemberGeometry {
            edge: member.edge,
            slots: member.slots.clone(),
            tap,
        });
    }
    let trunk_end = Point {
        x: collector_x,
        y: endpoint.y,
    };
    Some(BoundaryBundleGeometry {
        id: bundle.id,
        endpoint: bundle.endpoint,
        role: bundle.role,
        width: bundle.width,
        collector: BoundaryBundleSegment {
            start: Point {
                x: collector_x,
                y: collector_low,
            },
            end: Point {
                x: collector_x,
                y: collector_high,
            },
        },
        spine: BoundaryBundleSegment {
            start: endpoint,
            end: trunk_end,
        },
        members,
    })
}

fn rewrite_member_routes(
    bundle: &IndexedBoundaryBundle,
    geometry: &BoundaryBundleGeometry,
    route_index: &HashMap<u32, usize>,
    routes: &mut [EdgeGeometry],
    pitch: f64,
    rail_depth: f64,
    corridor_offset: f64,
) -> Result<(), LayoutError> {
    let interior_collector = geometry.spine.end.x == geometry.collector.start.x
        && geometry.spine.end.x == geometry.collector.end.x
        && geometry
            .members
            .iter()
            .all(|member| member.tap.x == geometry.spine.end.x)
        && (geometry.spine.end.x - geometry.spine.start.x).abs() >= rail_depth + pitch;
    for (member, indexed_member) in geometry.members.iter().zip(&bundle.members) {
        let route = &mut routes[route_index[&member.edge]];
        if route.points.len() < 2 {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
        route.points = if interior_collector {
            match bundle.role {
                BoundaryBundleRole::Input => rewrite_input_shared_trunk(&route.points, member.tap),
                BoundaryBundleRole::Output => {
                    rewrite_output_shared_trunk(&route.points, member.tap)
                }
            }
        } else {
            let corridor_depth = corridor_offset + (indexed_member.tap_lane + 1) as f64 * pitch;
            match bundle.role {
                BoundaryBundleRole::Input => {
                    rewrite_input_route(&route.points, member.tap, corridor_depth)
                }
                BoundaryBundleRole::Output => {
                    rewrite_output_route(&route.points, member.tap, corridor_depth)
                }
            }
        }
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
        if route.points.len() < 2 {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
    }
    Ok(())
}

fn rewrite_input_shared_trunk(points: &[Point], tap: Point) -> Option<Vec<Point>> {
    let (segment, intersection) = first_vertical_line_intersection(points, tap.x)?;
    if !preserved_point_matches(intersection, tap) {
        return None;
    }
    let mut rewritten = Vec::with_capacity(points.len() - segment);
    push_point(&mut rewritten, tap);
    for &point in &points[segment + 1..] {
        push_point(&mut rewritten, point);
    }
    Some(rewritten)
}

fn rewrite_output_shared_trunk(points: &[Point], tap: Point) -> Option<Vec<Point>> {
    let (segment, intersection) = last_vertical_line_intersection(points, tap.x)?;
    if !preserved_point_matches(intersection, tap) {
        return None;
    }
    let mut rewritten = Vec::with_capacity(segment + 2);
    for &point in &points[..=segment] {
        push_point(&mut rewritten, point);
    }
    push_point(&mut rewritten, tap);
    Some(rewritten)
}

fn rewrite_input_route(points: &[Point], tap: Point, pitch: f64) -> Option<Vec<Point>> {
    let corridor_x = tap.x + pitch;
    let (segment, intersection) = first_vertical_line_intersection(points, corridor_x)?;
    let mut rewritten = Vec::with_capacity(points.len() - segment + 3);
    push_point(&mut rewritten, tap);
    push_point(
        &mut rewritten,
        Point {
            x: corridor_x,
            y: tap.y,
        },
    );
    push_point(&mut rewritten, intersection);
    for &point in &points[segment + 1..] {
        push_point(&mut rewritten, point);
    }
    Some(rewritten)
}

fn rewrite_output_route(points: &[Point], tap: Point, pitch: f64) -> Option<Vec<Point>> {
    let corridor_x = tap.x - pitch;
    let (segment, intersection) = last_vertical_line_intersection(points, corridor_x)?;
    let mut rewritten = Vec::with_capacity(segment + 4);
    for &point in &points[..=segment] {
        push_point(&mut rewritten, point);
    }
    push_point(&mut rewritten, intersection);
    push_point(
        &mut rewritten,
        Point {
            x: corridor_x,
            y: tap.y,
        },
    );
    push_point(&mut rewritten, tap);
    Some(rewritten)
}

fn first_vertical_line_intersection(points: &[Point], x: f64) -> Option<(usize, Point)> {
    points.windows(2).enumerate().find_map(|(segment, pair)| {
        segment_vertical_line_intersection(pair[0], pair[1], x, false).map(|point| (segment, point))
    })
}

fn last_vertical_line_intersection(points: &[Point], x: f64) -> Option<(usize, Point)> {
    points
        .windows(2)
        .enumerate()
        .rev()
        .find_map(|(segment, pair)| {
            segment_vertical_line_intersection(pair[0], pair[1], x, true)
                .map(|point| (segment, point))
        })
}

fn segment_vertical_line_intersection(
    start: Point,
    end: Point,
    x: f64,
    prefer_end: bool,
) -> Option<Point> {
    if start.y == end.y && x >= start.x.min(end.x) && x <= start.x.max(end.x) {
        return Some(Point { x, y: start.y });
    }
    if start.x == x && end.x == x {
        return Some(if prefer_end { end } else { start });
    }
    None
}

fn push_point(points: &mut Vec<Point>, point: Point) {
    if points
        .last()
        .is_some_and(|last| preserved_point_matches(*last, point))
    {
        *points.last_mut().expect("point exists") = point;
        return;
    }
    if points.len() >= 2 {
        let before = points[points.len() - 2];
        let last = points[points.len() - 1];
        if (before.x == last.x && last.x == point.x) || (before.y == last.y && last.y == point.y) {
            *points.last_mut().expect("point exists") = point;
            return;
        }
    }
    points.push(point);
}

fn normalize_layout(layout: &mut Layout) {
    let mut min_x = layout.nodes.iter().map(|node| node.x).fold(0.0, f64::min);
    let mut min_y = layout.nodes.iter().map(|node| node.y).fold(0.0, f64::min);
    for point in all_points(layout) {
        min_x = min_x.min(point.x);
        min_y = min_y.min(point.y);
    }
    if min_x != 0.0 || min_y != 0.0 {
        for node in &mut layout.nodes {
            node.x -= min_x;
            node.y -= min_y;
        }
        for point in layout.edges.iter_mut().flat_map(|edge| &mut edge.points) {
            point.x -= min_x;
            point.y -= min_y;
        }
        for bundle in &mut layout.boundary_bundles {
            translate_point(&mut bundle.collector.start, min_x, min_y);
            translate_point(&mut bundle.collector.end, min_x, min_y);
            translate_point(&mut bundle.spine.start, min_x, min_y);
            translate_point(&mut bundle.spine.end, min_x, min_y);
            for member in &mut bundle.members {
                translate_point(&mut member.tap, min_x, min_y);
            }
        }
    }
    layout.width = layout
        .nodes
        .iter()
        .map(|node| node.x + node.width)
        .chain(all_points(layout).map(|point| point.x))
        .fold(0.0, f64::max);
    layout.height = layout
        .nodes
        .iter()
        .map(|node| node.y + node.height)
        .chain(all_points(layout).map(|point| point.y))
        .fold(0.0, f64::max);
}

fn all_points(layout: &Layout) -> impl Iterator<Item = &Point> {
    layout
        .edges
        .iter()
        .flat_map(|edge| edge.points.iter())
        .chain(layout.boundary_bundles.iter().flat_map(|bundle| {
            [
                &bundle.collector.start,
                &bundle.collector.end,
                &bundle.spine.start,
                &bundle.spine.end,
            ]
            .into_iter()
            .chain(bundle.members.iter().map(|member| &member.tap))
        }))
}

fn translate_point(point: &mut Point, min_x: f64, min_y: f64) {
    point.x -= min_x;
    point.y -= min_y;
}

pub(crate) fn verify_geometry(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    options: LayoutOptions,
) -> Result<(), LayoutError> {
    verify_preserved_geometry_structure(graph, layout, options)?;
    verify_bundle_node_clearance(layout, options)?;
    verify_rewritten_route_node_interiors(graph, layout)?;
    verify_bundle_route_contacts(graph, layout, options.minimum_parallel_wire_spacing)
}

fn partial_geometry_is_clean(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    options: LayoutOptions,
    bundle: usize,
) -> Result<(), LayoutError> {
    verify_bundle_node_clearance(layout, options)?;
    verify_one_bundle_route_node_interiors(graph, layout, bundle)?;
    verify_bundle_route_contacts(graph, layout, options.minimum_parallel_wire_spacing)
}

fn verify_bundle_node_clearance(
    layout: &Layout,
    options: LayoutOptions,
) -> Result<(), LayoutError> {
    let mut segments = Vec::with_capacity(layout.boundary_bundles.len() * 2);
    let mut relations = Vec::with_capacity(layout.boundary_bundles.len());
    for (index, bundle) in layout.boundary_bundles.iter().enumerate() {
        let net = BUNDLE_CLEARANCE_NET_BASE.wrapping_add(index as u32);
        push_clearance_segment(&mut segments, net, bundle.collector)?;
        push_clearance_segment(&mut segments, net, bundle.spine)?;
        relations.push(NetNodeRelation {
            net,
            node: bundle.endpoint.node,
        });
    }
    let threshold = options.edge_node_clearance.max(f64::EPSILON);
    match measure_edge_node_clearance_bounded(
        &segments,
        &layout.nodes,
        &relations,
        threshold,
        MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
    ) {
        Ok(clearance) if clearance.violations == 0 => {}
        Ok(_) | Err(EdgeNodeClearanceError::InvalidInput) => {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
        Err(EdgeNodeClearanceError::WorkLimitExceeded) => {
            return Err(LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
                maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
            });
        }
    }
    Ok(())
}

pub(crate) fn verify_preserved_geometry_structure(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    options: LayoutOptions,
) -> Result<(), LayoutError> {
    if graph.boundary_bundles.len() != layout.boundary_bundles.len() {
        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
    }
    let nodes = layout
        .nodes
        .iter()
        .map(|node| (node.id, node))
        .collect::<BTreeMap<_, _>>();
    let routes = layout
        .edges
        .iter()
        .map(|route| (route.id, route))
        .collect::<BTreeMap<_, _>>();
    let mut geometries = BTreeMap::new();
    for geometry in &layout.boundary_bundles {
        if geometries.insert(geometry.id, geometry).is_some() {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
    }
    let pitch = options
        .route_lane_gap
        .max(options.minimum_parallel_wire_spacing);
    let rail_depth = crate::boundary_bundle_rail_depth(options);
    let mut seen_edges = BTreeSet::new();
    for bundle in &graph.boundary_bundles {
        let Some(geometry) = geometries.get(&bundle.id).copied() else {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        };
        if geometry.endpoint != bundle.endpoint
            || geometry.role != bundle.role
            || geometry.width != bundle.width
            || geometry.members.len() != bundle.members.len()
        {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
        let Some(node) = nodes.get(&bundle.endpoint.node).copied() else {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        };
        let indexed_node = graph.node_index[&bundle.endpoint.node];
        let endpoint = port_point(node, graph.ports[indexed_node][&bundle.endpoint.port]);
        let direction = match bundle.role {
            BoundaryBundleRole::Input => 1.0,
            BoundaryBundleRole::Output => -1.0,
        };
        let fallback_collector_center = Point {
            x: endpoint.x + direction * rail_depth,
            y: endpoint.y,
        };
        let interior_collector = preserved_point_matches(geometry.spine.start, endpoint)
            && (geometry.spine.end.y - endpoint.y).abs() <= PRESERVED_GEOMETRY_EPSILON
            && direction * (geometry.spine.end.x - endpoint.x) >= rail_depth + pitch
            && (geometry.collector.start.x - geometry.spine.end.x).abs()
                <= PRESERVED_GEOMETRY_EPSILON
            && (geometry.collector.end.x - geometry.spine.end.x).abs()
                <= PRESERVED_GEOMETRY_EPSILON;
        let collector_center = if interior_collector {
            geometry.spine.end
        } else {
            if !preserved_segment_matches(
                geometry.spine,
                BoundaryBundleSegment {
                    start: endpoint,
                    end: fallback_collector_center,
                },
            ) {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
            fallback_collector_center
        };
        if !collector_center.x.is_finite() || !collector_center.y.is_finite() {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
        let lane_count = bundle
            .members
            .last()
            .map_or(0, |member| member.tap_lane + 1);
        let center = lane_count.saturating_sub(1) as f64 / 2.0;
        let mut declared_members = HashMap::with_capacity(geometry.members.len());
        for declared in &geometry.members {
            if declared_members.insert(declared.edge, declared).is_some() {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
        }
        let mut expected_taps = Vec::with_capacity(bundle.members.len());
        for member in &bundle.members {
            let Some(declared) = declared_members.get(&member.edge).copied() else {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            };
            let expected_tap = if interior_collector {
                if !declared.tap.x.is_finite()
                    || !declared.tap.y.is_finite()
                    || (declared.tap.x - collector_center.x).abs() > PRESERVED_GEOMETRY_EPSILON
                {
                    return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                }
                declared.tap
            } else {
                Point {
                    x: collector_center.x,
                    y: collector_center.y + (member.tap_lane as f64 - center) * pitch,
                }
            };
            if declared.edge != member.edge
                || declared.slots != member.slots
                || !preserved_point_matches(declared.tap, expected_tap)
                || !seen_edges.insert((bundle.role as u8, declared.edge))
            {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
            let Some(route) = routes.get(&declared.edge).copied() else {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            };
            let route_connects = match bundle.role {
                BoundaryBundleRole::Input => route
                    .points
                    .first()
                    .is_some_and(|point| preserved_point_matches(*point, expected_tap)),
                BoundaryBundleRole::Output => route
                    .points
                    .last()
                    .is_some_and(|point| preserved_point_matches(*point, expected_tap)),
            };
            if !route_connects {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
            expected_taps.push(expected_tap);
        }
        let collector_low = expected_taps
            .iter()
            .fold(collector_center.y, |low, tap| low.min(tap.y));
        let collector_high = expected_taps
            .iter()
            .fold(collector_center.y, |high, tap| high.max(tap.y));
        let expected_collector = BoundaryBundleSegment {
            start: Point {
                x: collector_center.x,
                y: collector_low,
            },
            end: Point {
                x: collector_center.x,
                y: collector_high,
            },
        };
        if !preserved_segment_matches(geometry.collector, expected_collector) {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
    }
    Ok(())
}

fn verify_rewritten_route_node_interiors(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
) -> Result<(), LayoutError> {
    let member_edges = graph
        .boundary_bundles
        .iter()
        .flat_map(|bundle| bundle.members.iter().map(|member| member.edge))
        .collect::<std::collections::BTreeSet<_>>();
    verify_member_route_node_interiors(graph, layout, &member_edges)
}

fn verify_one_bundle_route_node_interiors(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    bundle: usize,
) -> Result<(), LayoutError> {
    let member_edges = graph
        .boundary_bundles
        .get(bundle)
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?
        .members
        .iter()
        .map(|member| member.edge)
        .collect::<BTreeSet<_>>();
    verify_member_route_node_interiors(graph, layout, &member_edges)
}

fn verify_member_route_node_interiors(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    member_edges: &BTreeSet<u32>,
) -> Result<(), LayoutError> {
    let graph_edges = graph
        .edges
        .iter()
        .map(|edge| (edge.id, *edge))
        .collect::<HashMap<_, _>>();
    let mut remaining = MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS;
    for route in layout
        .edges
        .iter()
        .filter(|route| member_edges.contains(&route.id))
    {
        let edge = graph_edges[&route.id];
        for pair in route.points.windows(2) {
            let horizontal = if pair[0].y == pair[1].y {
                true
            } else if pair[0].x == pair[1].x {
                false
            } else {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            };
            for node in layout
                .nodes
                .iter()
                .filter(|node| node.id != edge.source.node && node.id != edge.target.node)
            {
                remaining = remaining.checked_sub(1).ok_or(
                    LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
                        maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
                    },
                )?;
                let intersects = if horizontal {
                    pair[0].y > node.y
                        && pair[0].y < node.y + node.height
                        && pair[0].x.min(pair[1].x) < node.x + node.width
                        && pair[0].x.max(pair[1].x) > node.x
                } else {
                    pair[0].x > node.x
                        && pair[0].x < node.x + node.width
                        && pair[0].y.min(pair[1].y) < node.y + node.height
                        && pair[0].y.max(pair[1].y) > node.y
                };
                if intersects {
                    return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                }
            }
        }
    }
    Ok(())
}

fn push_clearance_segment(
    segments: &mut Vec<EdgeNodeSegment>,
    net: u32,
    segment: BoundaryBundleSegment,
) -> Result<(), LayoutError> {
    if segment.start == segment.end {
        return Ok(());
    }
    if segment.start.x == segment.end.x {
        segments.push(EdgeNodeSegment {
            net,
            horizontal: false,
            fixed: segment.start.x,
            start: segment.start.y.min(segment.end.y),
            end: segment.start.y.max(segment.end.y),
        });
        Ok(())
    } else if segment.start.y == segment.end.y {
        segments.push(EdgeNodeSegment {
            net,
            horizontal: true,
            fixed: segment.start.y,
            start: segment.start.x.min(segment.end.x),
            end: segment.start.x.max(segment.end.x),
        });
        Ok(())
    } else {
        Err(LayoutError::BoundaryBundleGeometryUnsatisfied)
    }
}

fn verify_bundle_route_contacts(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    minimum_spacing: f64,
) -> Result<(), LayoutError> {
    let route_index = layout
        .edges
        .iter()
        .map(|route| (route.id, route))
        .collect::<HashMap<_, _>>();
    let mut remaining = MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS;
    for bundle in &layout.boundary_bundles {
        let permitted_taps = bundle
            .members
            .iter()
            .map(|member| (member.edge, member.tap))
            .collect::<HashMap<_, _>>();
        for segment in [bundle.collector, bundle.spine] {
            for route in &layout.edges {
                let permitted_tap = permitted_taps.get(&route.id).copied();
                for pair in route.points.windows(2) {
                    remaining = remaining.checked_sub(1).ok_or(
                        LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
                            maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
                        },
                    )?;
                    if segments_have_disallowed_contact(segment, pair[0], pair[1], permitted_tap) {
                        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                    }
                    if parallel_segments_are_too_close(
                        segment,
                        BoundaryBundleSegment {
                            start: pair[0],
                            end: pair[1],
                        },
                        minimum_spacing,
                    ) {
                        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                    }
                }
            }
        }
    }
    let bundle_segments = layout
        .boundary_bundles
        .iter()
        .enumerate()
        .flat_map(|(bundle, geometry)| [(bundle, geometry.collector), (bundle, geometry.spine)])
        .collect::<Vec<_>>();
    for left in 0..bundle_segments.len() {
        for right in left + 1..bundle_segments.len() {
            if bundle_segments[left].0 == bundle_segments[right].0 {
                continue;
            }
            remaining = remaining.checked_sub(1).ok_or(
                LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
                    maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
                },
            )?;
            if segments_have_disallowed_contact(
                bundle_segments[left].1,
                bundle_segments[right].1.start,
                bundle_segments[right].1.end,
                None,
            ) || parallel_segments_are_too_close(
                bundle_segments[left].1,
                bundle_segments[right].1,
                minimum_spacing,
            ) {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
        }
    }
    debug_assert!(graph.boundary_bundles.iter().all(|bundle| {
        bundle
            .members
            .iter()
            .all(|member| route_index.contains_key(&member.edge))
    }));
    Ok(())
}

fn parallel_segments_are_too_close(
    left: BoundaryBundleSegment,
    right: BoundaryBundleSegment,
    spacing: f64,
) -> bool {
    if spacing <= 0.0 {
        return false;
    }
    let left_horizontal = left.start.y == left.end.y;
    let right_horizontal = right.start.y == right.end.y;
    if left_horizontal != right_horizontal {
        return false;
    }
    if left_horizontal {
        intervals_overlap_positive(left.start.x, left.end.x, right.start.x, right.end.x)
            && (left.start.y - right.start.y).abs() < spacing
    } else {
        intervals_overlap_positive(left.start.y, left.end.y, right.start.y, right.end.y)
            && (left.start.x - right.start.x).abs() < spacing
    }
}

fn intervals_overlap_positive(left_a: f64, left_b: f64, right_a: f64, right_b: f64) -> bool {
    left_a.min(left_b) < right_a.max(right_b) && right_a.min(right_b) < left_a.max(left_b)
}

fn segments_have_disallowed_contact(
    bus: BoundaryBundleSegment,
    route_start: Point,
    route_end: Point,
    permitted_tap: Option<Point>,
) -> bool {
    if route_start == route_end {
        return false;
    }
    let bus_horizontal = bus.start.y == bus.end.y;
    let route_horizontal = route_start.y == route_end.y;
    if !bus_horizontal && bus.start.x != bus.end.x
        || !route_horizontal && route_start.x != route_end.x
    {
        return true;
    }
    let contact = if bus_horizontal == route_horizontal {
        let (bus_fixed, bus_start, bus_end, route_fixed, route_low, route_high) = if bus_horizontal
        {
            (
                bus.start.y,
                bus.start.x.min(bus.end.x),
                bus.start.x.max(bus.end.x),
                route_start.y,
                route_start.x.min(route_end.x),
                route_start.x.max(route_end.x),
            )
        } else {
            (
                bus.start.x,
                bus.start.y.min(bus.end.y),
                bus.start.y.max(bus.end.y),
                route_start.x,
                route_start.y.min(route_end.y),
                route_start.y.max(route_end.y),
            )
        };
        if bus_fixed != route_fixed || bus_start > route_high || route_low > bus_end {
            None
        } else {
            let overlap_start = bus_start.max(route_low);
            let overlap_end = bus_end.min(route_high);
            if overlap_start < overlap_end {
                return true;
            }
            Some(if bus_horizontal {
                Point {
                    x: overlap_start,
                    y: bus_fixed,
                }
            } else {
                Point {
                    x: bus_fixed,
                    y: overlap_start,
                }
            })
        }
    } else {
        let (horizontal_start, horizontal_end, vertical_start, vertical_end) = if bus_horizontal {
            (bus.start, bus.end, route_start, route_end)
        } else {
            (route_start, route_end, bus.start, bus.end)
        };
        let horizontal_low = horizontal_start.x.min(horizontal_end.x);
        let horizontal_high = horizontal_start.x.max(horizontal_end.x);
        let vertical_low = vertical_start.y.min(vertical_end.y);
        let vertical_high = vertical_start.y.max(vertical_end.y);
        (vertical_start.x >= horizontal_low
            && vertical_start.x <= horizontal_high
            && horizontal_start.y >= vertical_low
            && horizontal_start.y <= vertical_high)
            .then_some(Point {
                x: vertical_start.x,
                y: horizontal_start.y,
            })
    };
    contact.is_some_and(|point| {
        !permitted_tap.is_some_and(|tap| preserved_point_matches(tap, point))
            || (!preserved_point_matches(route_start, point)
                && !preserved_point_matches(route_end, point))
    })
}

fn port_point(node: &NodeGeometry, port: &Port) -> Point {
    match port.side {
        PortSide::North => Point {
            x: node.x + port.offset,
            y: node.y,
        },
        PortSide::East => Point {
            x: node.x + node.width,
            y: node.y + port.offset,
        },
        PortSide::South => Point {
            x: node.x + port.offset,
            y: node.y + node.height,
        },
        PortSide::West => Point {
            x: node.x,
            y: node.y + port.offset,
        },
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        BoundaryBundleConstraint, BoundaryBundleMemberConstraint, Edge, Endpoint, Graph,
        LayoutConstraints, Node, Port, PortSide, validation::validate_and_index_with_constraints,
    };

    use super::*;

    #[test]
    fn strict_anchor_rewrite_fails_closed_and_enters_output_taps_horizontally() {
        assert!(
            rewrite_input_route(
                &[Point { x: 0.0, y: 0.0 }, Point { x: 5.0, y: 0.0 }],
                Point { x: 10.0, y: 0.0 },
                1.0,
            )
            .is_none()
        );
        assert!(
            rewrite_output_route(
                &[Point { x: 15.0, y: 0.0 }, Point { x: 20.0, y: 0.0 }],
                Point { x: 10.0, y: 0.0 },
                1.0,
            )
            .is_none()
        );

        let tap = Point { x: 20.0, y: 5.0 };
        let rewritten = rewrite_output_route(
            &[
                Point { x: 0.0, y: 0.0 },
                Point { x: 10.0, y: 0.0 },
                Point { x: 20.0, y: 0.0 },
                Point { x: 20.0, y: 10.0 },
            ],
            tap,
            1.0,
        )
        .unwrap();
        assert_eq!(rewritten.last(), Some(&tap));
        assert!(rewritten[rewritten.len() - 2].x < tap.x);
        assert_eq!(rewritten[rewritten.len() - 2].y, tap.y);
    }

    #[test]
    fn corridor_splice_preserves_obstacle_aware_routes_and_separates_visible_cohorts() {
        let points = [
            Point { x: 125.0, y: 913.0 },
            Point { x: 149.0, y: 913.0 },
            Point { x: 149.0, y: 962.0 },
            Point { x: 295.0, y: 962.0 },
            Point {
                x: 295.0,
                y: 1_010.0,
            },
        ];
        let tap = Point { x: 151.0, y: 905.0 };
        let first = rewrite_input_route(&points, tap, 6.0).unwrap();
        let second = rewrite_input_route(&points, tap, 12.0).unwrap();
        assert_eq!(
            first[..3],
            [
                tap,
                Point { x: 157.0, y: 905.0 },
                Point { x: 157.0, y: 962.0 },
            ]
        );
        assert_eq!(first[3..], points[3..]);
        assert_eq!(second[1].x - first[1].x, 6.0);
        assert_eq!(second[2].x - first[2].x, 6.0);
        assert_eq!(rewrite_input_route(&points, tap, 6.0).unwrap(), first);

        let output_points = [
            Point {
                x: 2_025.0,
                y: 746.0,
            },
            Point {
                x: 2_166.0,
                y: 746.0,
            },
            Point {
                x: 2_166.0,
                y: 679.0,
            },
            Point {
                x: 2_179.0,
                y: 679.0,
            },
        ];
        let output_tap = Point {
            x: 2_165.0,
            y: 713.0,
        };
        let rewritten = rewrite_output_route(&output_points, output_tap, 6.0).unwrap();
        assert_eq!(
            rewritten[rewritten.len() - 3..],
            [
                Point {
                    x: 2_159.0,
                    y: 746.0,
                },
                Point {
                    x: 2_159.0,
                    y: 713.0,
                },
                output_tap,
            ]
        );
    }

    #[test]
    fn collinear_overlap_is_not_exempted_when_it_starts_at_the_tap() {
        let bus = BoundaryBundleSegment {
            start: Point { x: 0.0, y: 0.0 },
            end: Point { x: 10.0, y: 0.0 },
        };
        assert!(segments_have_disallowed_contact(
            bus,
            Point { x: 5.0, y: 0.0 },
            Point { x: 15.0, y: 0.0 },
            Some(Point { x: 5.0, y: 0.0 }),
        ));
        assert!(!segments_have_disallowed_contact(
            bus,
            Point { x: 10.0, y: 0.0 },
            Point { x: 15.0, y: 0.0 },
            Some(Point { x: 10.0, y: 0.0 }),
        ));
    }

    #[test]
    fn permitted_tap_contact_accepts_preserved_geometry_epsilon() {
        let bus = BoundaryBundleSegment {
            start: Point { x: 10.0, y: 0.0 },
            end: Point { x: 10.0, y: 20.0 },
        };
        assert!(!segments_have_disallowed_contact(
            bus,
            Point {
                x: 0.0,
                y: 7.000_000_000_000_004,
            },
            Point {
                x: 10.0,
                y: 7.000_000_000_000_004,
            },
            Some(Point { x: 10.0, y: 7.0 }),
        ));
    }

    #[test]
    fn rewritten_member_routes_cannot_cross_unrelated_node_interiors_at_zero_clearance() {
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::West),
                node(3, PortSide::West),
            ],
            edges: vec![Edge {
                id: 10,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 10,
                participates_in_ranking: true,
            }],
        };
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
            validate_and_index_with_constraints(&graph, LayoutOptions::default(), &constraints)
                .unwrap();
        let layout = Layout {
            nodes: vec![geometry(1, 0.0), geometry(3, 150.0), geometry(2, 300.0)],
            edges: vec![EdgeGeometry {
                id: 10,
                points: vec![Point { x: 94.0, y: 25.0 }, Point { x: 300.0, y: 25.0 }],
            }],
            boundary_bundles: Vec::new(),
            width: 380.0,
            height: 50.0,
        };
        assert_eq!(
            verify_rewritten_route_node_interiors(&indexed, &layout),
            Err(LayoutError::BoundaryBundleGeometryUnsatisfied)
        );
    }

    #[test]
    fn applying_bundle_geometry_twice_is_idempotent_for_post_best_refinements() {
        let graph = Graph {
            nodes: vec![node(1, PortSide::East), node(2, PortSide::West)],
            edges: vec![Edge {
                id: 10,
                source: Endpoint { node: 1, port: 0 },
                target: Endpoint { node: 2, port: 0 },
                net: 10,
                participates_in_ranking: true,
            }],
        };
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
        let options = LayoutOptions::default();
        let indexed = validate_and_index_with_constraints(&graph, options, &constraints).unwrap();
        let layout = Layout {
            nodes: vec![geometry(1, 0.0), geometry(2, 300.0)],
            edges: vec![EdgeGeometry {
                id: 10,
                points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 300.0, y: 25.0 }],
            }],
            boundary_bundles: Vec::new(),
            width: 380.0,
            height: 50.0,
        };
        let once = apply_and_normalize(&indexed, layout, options).unwrap();
        let twice = apply_and_normalize(&indexed, once.clone(), options).unwrap();
        assert_eq!(twice, once);
    }

    #[test]
    fn vector_input_uses_one_interior_collector_and_falls_back_when_it_is_blocked() {
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::West),
                node(3, PortSide::West),
                node(4, PortSide::West),
            ],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 10,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 11,
                    participates_in_ranking: true,
                },
            ],
        };
        let constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![2, 3],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 1,
                endpoint: Endpoint { node: 1, port: 0 },
                width: 2,
                members: vec![
                    BoundaryBundleMemberConstraint {
                        edge: 10,
                        slots: vec![0],
                    },
                    BoundaryBundleMemberConstraint {
                        edge: 11,
                        slots: vec![1],
                    },
                ],
            }],
        };
        let options = LayoutOptions::default();
        let indexed = validate_and_index_with_constraints(&graph, options, &constraints).unwrap();
        let raw = Layout {
            nodes: vec![
                geometry_at(1, 0.0, 0.0),
                geometry_at(2, 300.0, 0.0),
                geometry_at(3, 300.0, 100.0),
                geometry_at(4, 600.0, 0.0),
            ],
            edges: vec![
                EdgeGeometry {
                    id: 10,
                    points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 300.0, y: 25.0 }],
                },
                EdgeGeometry {
                    id: 11,
                    points: vec![
                        Point { x: 80.0, y: 25.0 },
                        Point { x: 120.0, y: 25.0 },
                        Point { x: 120.0, y: 125.0 },
                        Point { x: 300.0, y: 125.0 },
                    ],
                },
            ],
            boundary_bundles: Vec::new(),
            width: 680.0,
            height: 150.0,
        };

        let interior = apply_and_normalize(&indexed, raw.clone(), options).unwrap();
        let bundle = &interior.boundary_bundles[0];
        assert_eq!(
            bundle.spine,
            BoundaryBundleSegment {
                start: Point { x: 80.0, y: 25.0 },
                end: Point { x: 286.0, y: 25.0 },
            }
        );
        assert_eq!(
            bundle.collector,
            BoundaryBundleSegment {
                start: Point { x: 286.0, y: 25.0 },
                end: Point { x: 286.0, y: 125.0 },
            }
        );
        assert_eq!(
            interior.edges,
            vec![
                EdgeGeometry {
                    id: 10,
                    points: vec![Point { x: 286.0, y: 25.0 }, Point { x: 300.0, y: 25.0 }],
                },
                EdgeGeometry {
                    id: 11,
                    points: vec![Point { x: 286.0, y: 125.0 }, Point { x: 300.0, y: 125.0 }],
                },
            ]
        );
        assert_eq!(
            apply_and_normalize(&indexed, interior.clone(), options).unwrap(),
            interior,
        );

        let mut blocked = raw;
        blocked.nodes[3] = geometry_at(4, 260.0, 50.0);
        let fallback = apply_and_normalize(&indexed, blocked, options).unwrap();
        assert_eq!(fallback.boundary_bundles[0].spine.end.x, 94.0);
        assert_ne!(
            fallback.boundary_bundles[0].collector.start,
            fallback.boundary_bundles[0].collector.end,
        );
    }

    #[test]
    fn vector_output_uses_one_interior_collector_after_member_convergence() {
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::East),
                node(3, PortSide::West),
            ],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 10,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 2, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 11,
                    participates_in_ranking: true,
                },
            ],
        };
        let constraints = LayoutConstraints {
            inputs: vec![1, 2],
            outputs: vec![3],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 1,
                endpoint: Endpoint { node: 3, port: 0 },
                width: 2,
                members: vec![
                    BoundaryBundleMemberConstraint {
                        edge: 10,
                        slots: vec![0],
                    },
                    BoundaryBundleMemberConstraint {
                        edge: 11,
                        slots: vec![1],
                    },
                ],
            }],
        };
        let options = LayoutOptions::default();
        let indexed = validate_and_index_with_constraints(&graph, options, &constraints).unwrap();
        let layout = Layout {
            nodes: vec![
                geometry_at(1, 0.0, 0.0),
                geometry_at(2, 0.0, 100.0),
                geometry_at(3, 300.0, 0.0),
            ],
            edges: vec![
                EdgeGeometry {
                    id: 10,
                    points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 300.0, y: 25.0 }],
                },
                EdgeGeometry {
                    id: 11,
                    points: vec![
                        Point { x: 80.0, y: 125.0 },
                        Point { x: 250.0, y: 125.0 },
                        Point { x: 250.0, y: 25.0 },
                        Point { x: 300.0, y: 25.0 },
                    ],
                },
            ],
            boundary_bundles: Vec::new(),
            width: 380.0,
            height: 150.0,
        };

        let interior = apply_and_normalize(&indexed, layout, options).unwrap();
        let bundle = &interior.boundary_bundles[0];
        assert_eq!(
            bundle.spine,
            BoundaryBundleSegment {
                start: Point { x: 300.0, y: 25.0 },
                end: Point { x: 94.0, y: 25.0 },
            }
        );
        assert_eq!(
            bundle.collector,
            BoundaryBundleSegment {
                start: Point { x: 94.0, y: 25.0 },
                end: Point { x: 94.0, y: 125.0 },
            }
        );
        assert_eq!(
            interior.edges,
            vec![
                EdgeGeometry {
                    id: 10,
                    points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 94.0, y: 25.0 }],
                },
                EdgeGeometry {
                    id: 11,
                    points: vec![Point { x: 80.0, y: 125.0 }, Point { x: 94.0, y: 125.0 }],
                },
            ]
        );
    }

    #[test]
    fn preserved_bundle_geometry_is_member_order_invariant_but_rejects_duplicates() {
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::West),
                node(3, PortSide::West),
            ],
            edges: vec![
                Edge {
                    id: 10,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 2, port: 0 },
                    net: 10,
                    participates_in_ranking: true,
                },
                Edge {
                    id: 11,
                    source: Endpoint { node: 1, port: 0 },
                    target: Endpoint { node: 3, port: 0 },
                    net: 11,
                    participates_in_ranking: true,
                },
            ],
        };
        let constraints = LayoutConstraints {
            inputs: vec![1],
            outputs: vec![2, 3],
            boundary_bundles: vec![BoundaryBundleConstraint {
                id: 1,
                endpoint: Endpoint { node: 1, port: 0 },
                width: 2,
                members: vec![
                    BoundaryBundleMemberConstraint {
                        edge: 10,
                        slots: vec![0],
                    },
                    BoundaryBundleMemberConstraint {
                        edge: 11,
                        slots: vec![1],
                    },
                ],
            }],
        };
        let options = LayoutOptions::default();
        let indexed = validate_and_index_with_constraints(&graph, options, &constraints).unwrap();
        let layout = crate::layout_with_constraints(&graph, options, &constraints).unwrap();

        let mut permuted = layout.clone();
        permuted.boundary_bundles[0].members.reverse();
        assert_eq!(
            verify_preserved_geometry_structure(&indexed, &permuted, options),
            Ok(())
        );

        permuted.boundary_bundles[0].members[1].edge = permuted.boundary_bundles[0].members[0].edge;
        assert_eq!(
            verify_preserved_geometry_structure(&indexed, &permuted, options),
            Err(LayoutError::BoundaryBundleGeometryUnsatisfied)
        );
    }

    fn node(id: u32, side: PortSide) -> Node {
        Node {
            id,
            width: 80.0,
            height: 50.0,
            cycle_breaker: false,
            ports: vec![Port {
                id: 0,
                side,
                offset: 25.0,
            }],
        }
    }

    fn geometry(id: u32, x: f64) -> NodeGeometry {
        geometry_at(id, x, 0.0)
    }

    fn geometry_at(id: u32, x: f64, y: f64) -> NodeGeometry {
        NodeGeometry {
            id,
            x,
            y,
            width: 80.0,
            height: 50.0,
        }
    }
}
