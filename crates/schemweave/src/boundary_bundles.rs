use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::{
    BoundaryBundleGeometry, BoundaryBundleMemberGeometry, BoundaryBundleRole,
    BoundaryBundleSegment, EdgeGeometry, EdgeNodeClearanceError, EdgeNodeSegment, Layout,
    LayoutError, LayoutOptions, NetNodeRelation, NodeGeometry, Point, Port, PortSide,
    measure_edge_node_clearance_bounded,
    validation::{IndexedBoundaryBundle, IndexedGraph},
};

pub(crate) const MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS: usize = 20_000_000;
const BUNDLE_CLEARANCE_NET_BASE: u32 = 0x8000_0000;

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
    let mut geometries = Vec::with_capacity(graph.boundary_bundles.len());
    for (bundle, corridor_offset) in graph.boundary_bundles.iter().zip(corridor_offsets) {
        let geometry = build_geometry(
            graph,
            bundle,
            &layout.nodes,
            &node_geometry,
            pitch,
            rail_depth,
        );
        rewrite_member_routes(
            bundle,
            &geometry,
            &route_index,
            &mut layout.edges,
            pitch,
            corridor_offset,
        )?;
        geometries.push(geometry);
    }
    layout.boundary_bundles = geometries;
    normalize_layout(&mut layout);
    verify_geometry(graph, &layout, options)?;
    Ok(layout)
}

fn build_geometry(
    graph: &IndexedGraph<'_>,
    bundle: &IndexedBoundaryBundle,
    nodes: &[NodeGeometry],
    node_geometry: &HashMap<u32, usize>,
    pitch: f64,
    port_stub: f64,
) -> BoundaryBundleGeometry {
    let node = &nodes[node_geometry[&bundle.endpoint.node]];
    let indexed_node = graph.node_index[&bundle.endpoint.node];
    let port = graph.ports[indexed_node][&bundle.endpoint.port];
    let endpoint = port_point(node, port);
    let direction = match bundle.role {
        BoundaryBundleRole::Input => 1.0,
        BoundaryBundleRole::Output => -1.0,
    };
    let collector_end = Point {
        x: endpoint.x + direction * port_stub,
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
                y: collector_end.y + (member.tap_lane as f64 - center) * pitch,
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

fn rewrite_member_routes(
    bundle: &IndexedBoundaryBundle,
    geometry: &BoundaryBundleGeometry,
    route_index: &HashMap<u32, usize>,
    routes: &mut [EdgeGeometry],
    pitch: f64,
    corridor_offset: f64,
) -> Result<(), LayoutError> {
    for (member, indexed_member) in geometry.members.iter().zip(&bundle.members) {
        let route = &mut routes[route_index[&member.edge]];
        if route.points.len() < 2 {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
        let corridor_depth = corridor_offset + (indexed_member.tap_lane + 1) as f64 * pitch;
        route.points = match bundle.role {
            BoundaryBundleRole::Input => {
                rewrite_input_route(&route.points, member.tap, corridor_depth)
            }
            BoundaryBundleRole::Output => {
                rewrite_output_route(&route.points, member.tap, corridor_depth)
            }
        }
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
        if route.points.len() < 2 {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
    }
    Ok(())
}

pub(crate) fn rewrite_preserved_member_route(
    route: &mut EdgeGeometry,
    role: BoundaryBundleRole,
    tap: Point,
    tap_lane: usize,
    pitch: f64,
    corridor_offset: f64,
) -> Result<(), LayoutError> {
    if route.points.len() < 2 {
        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
    }
    let corridor_depth = corridor_offset + (tap_lane + 1) as f64 * pitch;
    route.points = match role {
        BoundaryBundleRole::Input => rewrite_input_route(&route.points, tap, corridor_depth),
        BoundaryBundleRole::Output => rewrite_output_route(&route.points, tap, corridor_depth),
    }
    .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
    if route.points.len() < 2 {
        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
    }
    Ok(())
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
    if points.last() == Some(&point) {
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
    verify_rewritten_route_node_interiors(graph, layout)?;
    verify_bundle_route_contacts(graph, layout, options.minimum_parallel_wire_spacing)
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
        let collector_center = Point {
            x: endpoint.x + direction * rail_depth,
            y: endpoint.y,
        };
        if geometry.spine
            != (BoundaryBundleSegment {
                start: endpoint,
                end: collector_center,
            })
        {
            return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
        }
        let lane_count = bundle
            .members
            .last()
            .map_or(0, |member| member.tap_lane + 1);
        let center = lane_count.saturating_sub(1) as f64 / 2.0;
        let mut expected_taps = Vec::with_capacity(bundle.members.len());
        for (member, declared) in bundle.members.iter().zip(&geometry.members) {
            let expected_tap = Point {
                x: collector_center.x,
                y: collector_center.y + (member.tap_lane as f64 - center) * pitch,
            };
            if declared.edge != member.edge
                || declared.slots != member.slots
                || declared.tap != expected_tap
                || !seen_edges.insert((bundle.role as u8, declared.edge))
            {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
            let Some(route) = routes.get(&declared.edge).copied() else {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            };
            let route_connects = match bundle.role {
                BoundaryBundleRole::Input => route.points.first() == Some(&expected_tap),
                BoundaryBundleRole::Output => route.points.last() == Some(&expected_tap),
            };
            if !route_connects {
                return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
            }
            expected_taps.push(expected_tap);
        }
        let expected_collector = BoundaryBundleSegment {
            start: Point {
                x: collector_center.x,
                y: expected_taps
                    .first()
                    .map_or(collector_center.y, |tap| tap.y.min(collector_center.y)),
            },
            end: Point {
                x: collector_center.x,
                y: expected_taps
                    .last()
                    .map_or(collector_center.y, |tap| tap.y.max(collector_center.y)),
            },
        };
        if geometry.collector != expected_collector {
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
        permitted_tap != Some(point) || (route_start != point && route_end != point)
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
        NodeGeometry {
            id,
            x,
            y: 0.0,
            width: 80.0,
            height: 50.0,
        }
    }
}
