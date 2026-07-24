use std::collections::{BTreeMap, BTreeSet, HashMap};

use crate::{
    BoundaryBundleGeometry, BoundaryBundleMemberGeometry, BoundaryBundleRole,
    BoundaryBundleSegment, EdgeGeometry, EdgeNodeClearanceError, EdgeNodeSegment, Layout,
    LayoutError, LayoutOptions, NetNodeRelation, NodeGeometry, Point, Port, PortSide,
    measure_edge_node_clearance_bounded,
    validation::{IndexedBoundaryBundle, IndexedGraph},
};

pub(crate) const MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS: usize = 20_000_000;
pub(crate) const MAX_INTERIOR_COLLECTOR_BUNDLES: usize = 32;
pub(crate) const MAX_INTERIOR_HORIZONTAL_TAP_VISITS: usize = 2_000_000;
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
    member_endpoint_reserve: f64,
}

#[derive(Clone, Copy)]
struct InteriorCollectorRange {
    role: BoundaryBundleRole,
    minimum_x: f64,
    desired_x: f64,
    maximum_x: f64,
}

struct SameEndpointSharedRouteCandidate {
    geometry: BoundaryBundleGeometry,
    original_routes: Vec<(usize, Vec<Point>)>,
    representative: Vec<Point>,
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
    layout: Layout,
    options: LayoutOptions,
) -> Result<Layout, LayoutError> {
    apply_and_normalize_preserving(graph, layout, options, &BTreeSet::new())
}

pub(crate) fn apply_and_normalize_preserving(
    graph: &IndexedGraph<'_>,
    mut layout: Layout,
    options: LayoutOptions,
    preserved_bundle_ids: &BTreeSet<u32>,
) -> Result<Layout, LayoutError> {
    if graph.boundary_bundles.is_empty() {
        return Ok(layout);
    }
    if !layout.boundary_bundles.is_empty() && verify_geometry(graph, &layout, options).is_ok() {
        normalize_layout(&mut layout);
        return Ok(layout);
    }
    let interior_allowed = options.minimum_parallel_wire_spacing == 0.0
        && graph.boundary_bundles.len() <= MAX_INTERIOR_COLLECTOR_BUNDLES;
    apply_bundle_geometry(
        graph,
        layout,
        options,
        interior_allowed,
        preserved_bundle_ids,
    )
}

fn apply_bundle_geometry(
    graph: &IndexedGraph<'_>,
    mut layout: Layout,
    options: LayoutOptions,
    allow_interior_collectors: bool,
    preserved_bundle_ids: &BTreeSet<u32>,
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
        member_endpoint_reserve: crate::outward_obstacle_clearance_stub(options),
    };
    let interior_collectors = if allow_interior_collectors {
        plan_interior_collectors(&geometry_context, &layout.edges)
    } else {
        vec![None; graph.boundary_bundles.len()]
    };
    let mut processing_order = (0..graph.boundary_bundles.len()).collect::<Vec<_>>();
    processing_order.sort_unstable_by_key(|&bundle| {
        let bundle = &graph.boundary_bundles[bundle];
        (
            match bundle.role {
                BoundaryBundleRole::Input => 0_u8,
                BoundaryBundleRole::Output => 1_u8,
            },
            bundle.endpoint.node,
            bundle.endpoint.port,
            bundle.id,
        )
    });
    let mut partial_remaining = MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS;
    let mut preserved_geometry = layout
        .boundary_bundles
        .iter()
        .filter(|bundle| preserved_bundle_ids.contains(&bundle.id))
        .map(|bundle| (bundle.id, bundle.clone()))
        .collect::<BTreeMap<_, _>>();
    if preserved_geometry.len() != preserved_bundle_ids.len() {
        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
    }
    layout.boundary_bundles.clear();
    for bundle_index in processing_order {
        let bundle = &graph.boundary_bundles[bundle_index];
        if preserved_bundle_ids.contains(&bundle.id) {
            let geometry = preserved_geometry
                .remove(&bundle.id)
                .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
            layout.boundary_bundles.push(geometry);
            continue;
        }
        let corridor_offset = corridor_offsets[bundle_index];
        let planned_collector = interior_collectors[bundle_index];
        let preserved = planned_collector.map(|_| {
            bundle
                .members
                .iter()
                .map(|member| {
                    let index = route_index[&member.edge];
                    (index, layout.edges[index].points.clone())
                })
                .collect::<Vec<_>>()
        });
        let mut geometry =
            build_geometry(&geometry_context, bundle, &layout.edges, planned_collector);
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
        if planned_collector.is_some() {
            match partial_geometry_is_clean(
                graph,
                &layout,
                options,
                bundle_index,
                &mut partial_remaining,
            ) {
                Ok(()) => {}
                Err(LayoutError::BoundaryBundleGeometryUnsatisfied) => {
                    layout.boundary_bundles.pop();
                    let Some(preserved) = preserved else {
                        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                    };
                    for (index, points) in preserved {
                        layout.edges[index].points = points;
                    }
                    geometry = build_geometry(&geometry_context, bundle, &layout.edges, None);
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
                }
                Err(error) => return Err(error),
            }
        }
        let current_geometry = layout
            .boundary_bundles
            .last()
            .cloned()
            .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
        if options.minimum_parallel_wire_spacing == 0.0
            && let Some(shared) = same_endpoint_shared_route_candidate(
                &geometry_context,
                bundle,
                &current_geometry,
                &layout.edges,
            )
        {
            *layout
                .boundary_bundles
                .last_mut()
                .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)? = shared.geometry;
            for member in &bundle.members {
                layout.edges[route_index[&member.edge]].points = shared.representative.clone();
            }
            match partial_geometry_is_clean(
                graph,
                &layout,
                options,
                bundle_index,
                &mut partial_remaining,
            ) {
                Ok(()) => {}
                Err(LayoutError::BoundaryBundleGeometryUnsatisfied) => {
                    *layout
                        .boundary_bundles
                        .last_mut()
                        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)? = current_geometry;
                    for (index, points) in shared.original_routes {
                        layout.edges[index].points = points;
                    }
                }
                Err(error) => return Err(error),
            }
        }
    }
    layout
        .boundary_bundles
        .sort_unstable_by_key(|bundle| bundle.id);
    normalize_layout(&mut layout);
    verify_geometry(graph, &layout, options)?;
    Ok(layout)
}

fn same_endpoint_shared_route_candidate(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    geometry: &BoundaryBundleGeometry,
    routes: &[EdgeGeometry],
) -> Option<SameEndpointSharedRouteCandidate> {
    if bundle.width <= 1
        || context.graph.boundary_bundles.len() > MAX_INTERIOR_COLLECTOR_BUNDLES
        || !bundle_members_share_opposite_endpoint(context.graph, bundle)
        || geometry.members.len() != bundle.members.len()
    {
        return None;
    }
    let (representative_index, representative_tap) = geometry
        .members
        .iter()
        .enumerate()
        .min_by(|(left_index, left), (right_index, right)| {
            (left.tap.y - geometry.spine.start.y)
                .abs()
                .total_cmp(&(right.tap.y - geometry.spine.start.y).abs())
                .then_with(|| {
                    bundle.members[*left_index]
                        .edge
                        .cmp(&bundle.members[*right_index].edge)
                })
        })
        .map(|(index, member)| (index, member.tap))?;
    if !preserved_point_matches(
        Point {
            x: representative_tap.x,
            y: geometry.spine.end.y,
        },
        geometry.spine.end,
    ) {
        return None;
    }
    let representative = routes
        .get(
            *context
                .route_index
                .get(&bundle.members[representative_index].edge)?,
        )?
        .points
        .clone();
    if representative.len() < 2 {
        return None;
    }
    let original_routes = bundle
        .members
        .iter()
        .map(|member| {
            let index = *context.route_index.get(&member.edge)?;
            Some((index, routes.get(index)?.points.clone()))
        })
        .collect::<Option<Vec<_>>>()?;
    if original_routes
        .iter()
        .all(|(_, points)| points == &representative)
        && geometry
            .members
            .iter()
            .all(|member| preserved_point_matches(member.tap, representative_tap))
    {
        return None;
    }
    let existing_segments = routes
        .iter()
        .map(|route| route.points.len().saturating_sub(1))
        .sum::<usize>();
    let replaced_segments = original_routes
        .iter()
        .map(|(_, points)| points.len().saturating_sub(1))
        .sum::<usize>();
    let candidate_segments = representative
        .len()
        .saturating_sub(1)
        .saturating_mul(bundle.members.len());
    if existing_segments
        .saturating_sub(replaced_segments)
        .saturating_add(candidate_segments)
        > crate::MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS
    {
        return None;
    }

    let mut candidate = geometry.clone();
    for member in &mut candidate.members {
        member.tap = representative_tap;
    }
    candidate.collector = BoundaryBundleSegment {
        start: Point {
            x: representative_tap.x,
            y: geometry.spine.end.y.min(representative_tap.y),
        },
        end: Point {
            x: representative_tap.x,
            y: geometry.spine.end.y.max(representative_tap.y),
        },
    };
    Some(SameEndpointSharedRouteCandidate {
        geometry: candidate,
        original_routes,
        representative,
    })
}

fn bundle_members_share_opposite_endpoint(
    graph: &IndexedGraph<'_>,
    bundle: &IndexedBoundaryBundle,
) -> bool {
    if bundle.members.len() < 2 {
        return false;
    }
    let mut opposite_endpoint = None;
    for member in &bundle.members {
        let Ok(edge_index) = graph
            .edges
            .binary_search_by_key(&member.edge, |edge| edge.id)
        else {
            return false;
        };
        let edge = graph.edges[edge_index];
        let endpoint = match bundle.role {
            BoundaryBundleRole::Input => edge.target,
            BoundaryBundleRole::Output => edge.source,
        };
        if opposite_endpoint
            .replace(endpoint)
            .is_some_and(|prior| prior != endpoint)
        {
            return false;
        }
    }
    true
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

fn plan_interior_collectors(
    context: &BundleGeometryContext<'_, '_>,
    routes: &[EdgeGeometry],
) -> Vec<Option<f64>> {
    let mut horizontal_tap_visits = MAX_INTERIOR_HORIZONTAL_TAP_VISITS;
    let ranges = context
        .graph
        .boundary_bundles
        .iter()
        .map(|bundle| interior_collector_range(context, bundle, routes, &mut horizontal_tap_visits))
        .collect::<Vec<_>>();
    let mut input_by_edge = HashMap::new();
    let mut output_by_edge = HashMap::new();
    for (bundle_index, bundle) in context.graph.boundary_bundles.iter().enumerate() {
        if ranges[bundle_index].is_none() {
            continue;
        }
        for member in &bundle.members {
            match bundle.role {
                BoundaryBundleRole::Input => {
                    input_by_edge.insert(member.edge, bundle_index);
                }
                BoundaryBundleRole::Output => {
                    output_by_edge.insert(member.edge, bundle_index);
                }
            }
        }
    }
    let mut adjacent = vec![Vec::new(); ranges.len()];
    for (edge, &input) in &input_by_edge {
        let Some(&output) = output_by_edge.get(edge) else {
            continue;
        };
        adjacent[input].push(output);
        adjacent[output].push(input);
    }
    for neighbors in &mut adjacent {
        neighbors.sort_unstable();
        neighbors.dedup();
    }

    let mut planned = ranges
        .iter()
        .map(|range| range.map(|range| range.desired_x))
        .collect::<Vec<_>>();
    let mut visited = vec![false; ranges.len()];
    for start in 0..ranges.len() {
        if visited[start] || ranges[start].is_none() {
            continue;
        }
        let mut component = Vec::new();
        let mut pending = vec![start];
        visited[start] = true;
        while let Some(bundle) = pending.pop() {
            component.push(bundle);
            for &neighbor in &adjacent[bundle] {
                if !visited[neighbor] {
                    visited[neighbor] = true;
                    pending.push(neighbor);
                }
            }
        }
        let inputs = component
            .iter()
            .filter_map(|&bundle| {
                let range = ranges[bundle]?;
                (range.role == BoundaryBundleRole::Input).then_some(bundle)
            })
            .collect::<Vec<_>>();
        let outputs = component
            .iter()
            .filter_map(|&bundle| {
                let range = ranges[bundle]?;
                (range.role == BoundaryBundleRole::Output).then_some(bundle)
            })
            .collect::<Vec<_>>();
        if inputs.is_empty() || outputs.is_empty() {
            continue;
        }
        let feasible_low = inputs
            .iter()
            .filter_map(|&bundle| ranges[bundle].map(|range| range.minimum_x))
            .fold(f64::NEG_INFINITY, f64::max);
        let latest_output_limit = outputs
            .iter()
            .filter_map(|&bundle| ranges[bundle].map(|range| range.maximum_x))
            .fold(f64::INFINITY, f64::min);
        if feasible_low + context.pitch > latest_output_limit {
            for bundle in component {
                planned[bundle] = None;
            }
            continue;
        }
        let deepest_input = inputs
            .iter()
            .filter_map(|&bundle| ranges[bundle].map(|range| range.desired_x))
            .fold(f64::NEG_INFINITY, f64::max);
        let earliest_output = outputs
            .iter()
            .filter_map(|&bundle| ranges[bundle].map(|range| range.desired_x))
            .fold(f64::INFINITY, f64::min);
        if deepest_input + context.pitch > earliest_output {
            let earliest_separated_output = earliest_output - context.pitch;
            let input_split = (earliest_separated_output
                + (deepest_input - earliest_separated_output) / 2.0)
                .clamp(feasible_low, latest_output_limit - context.pitch);
            for &bundle_index in &inputs {
                let bundle = &context.graph.boundary_bundles[bundle_index];
                planned[bundle_index] = interior_collector_range_bounded(
                    context,
                    bundle,
                    routes,
                    f64::NEG_INFINITY,
                    input_split,
                    &mut horizontal_tap_visits,
                )
                .map(|range| range.desired_x);
            }
        }
        let mut required_output_x = required_output_collector_x(context, routes, &inputs, &planned);
        if required_output_x > latest_output_limit {
            for &bundle_index in &inputs {
                let bundle = &context.graph.boundary_bundles[bundle_index];
                let input_limit = planned[bundle_index]
                    .unwrap_or(latest_output_limit)
                    .min(latest_output_limit - context.pitch);
                planned[bundle_index] = interior_collector_range_bounded(
                    context,
                    bundle,
                    routes,
                    f64::NEG_INFINITY,
                    input_limit,
                    &mut horizontal_tap_visits,
                )
                .map(|range| range.desired_x);
            }
            required_output_x = required_output_collector_x(context, routes, &inputs, &planned);
        }
        for bundle_index in outputs {
            let bundle = &context.graph.boundary_bundles[bundle_index];
            planned[bundle_index] = interior_collector_range_bounded(
                context,
                bundle,
                routes,
                required_output_x,
                f64::INFINITY,
                &mut horizontal_tap_visits,
            )
            .map(|range| range.desired_x);
        }
    }
    planned
}

fn required_output_collector_x(
    context: &BundleGeometryContext<'_, '_>,
    routes: &[EdgeGeometry],
    inputs: &[usize],
    planned: &[Option<f64>],
) -> f64 {
    inputs
        .iter()
        .filter_map(|&bundle_index| {
            let collector_x = planned[bundle_index]?;
            let bundle = &context.graph.boundary_bundles[bundle_index];
            let divergence_x =
                input_shared_trunk_divergence_x(context, bundle, routes, collector_x)?;
            Some((collector_x + context.pitch).max(divergence_x + context.pitch))
        })
        .fold(f64::NEG_INFINITY, f64::max)
}

fn input_shared_trunk_divergence_x(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    routes: &[EdgeGeometry],
    collector_x: f64,
) -> Option<f64> {
    bundle
        .members
        .iter()
        .map(|member| {
            let route = routes.get(*context.route_index.get(&member.edge)?)?;
            let (segment, _) = first_shared_trunk_intersection(
                &route.points,
                collector_x,
                bundle.role,
                context.pitch,
            )?;
            route.points.get(segment + 1).map(|point| point.x)
        })
        .collect::<Option<Vec<_>>>()
        .and_then(|divergences| divergences.into_iter().max_by(f64::total_cmp))
}

fn interior_collector_range(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    routes: &[EdgeGeometry],
    horizontal_tap_visits: &mut usize,
) -> Option<InteriorCollectorRange> {
    interior_collector_range_bounded(
        context,
        bundle,
        routes,
        f64::NEG_INFINITY,
        f64::INFINITY,
        horizontal_tap_visits,
    )
}

fn interior_collector_range_bounded(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    routes: &[EdgeGeometry],
    lower_bound: f64,
    upper_bound: f64,
    horizontal_tap_visits: &mut usize,
) -> Option<InteriorCollectorRange> {
    if bundle.width <= 1 {
        return None;
    }
    let node = &context.nodes[context.node_geometry[&bundle.endpoint.node]];
    let indexed_node = context.graph.node_index[&bundle.endpoint.node];
    let endpoint = port_point(
        node,
        context.graph.ports[indexed_node][&bundle.endpoint.port],
    );
    let mut desired_x = match bundle.role {
        BoundaryBundleRole::Input => f64::INFINITY,
        BoundaryBundleRole::Output => f64::NEG_INFINITY,
    };
    for member in &bundle.members {
        let route = routes.get(*context.route_index.get(&member.edge)?)?;
        let mut member_x = match bundle.role {
            BoundaryBundleRole::Input => route.points.last()?.x - context.member_endpoint_reserve,
            BoundaryBundleRole::Output => route.points.first()?.x + context.member_endpoint_reserve,
        };
        member_x = match bundle.role {
            BoundaryBundleRole::Input => context
                .output_fallback_corridors
                .get(&member.edge)
                .map_or(member_x, |corridor| member_x.min(*corridor)),
            BoundaryBundleRole::Output => context
                .input_fallback_corridors
                .get(&member.edge)
                .map_or(member_x, |corridor| member_x.max(*corridor)),
        };
        desired_x = match bundle.role {
            BoundaryBundleRole::Input => desired_x.min(member_x),
            BoundaryBundleRole::Output => desired_x.max(member_x),
        };
    }
    let (minimum_x, maximum_x) = match bundle.role {
        BoundaryBundleRole::Input => (endpoint.x + context.rail_depth + context.pitch, desired_x),
        BoundaryBundleRole::Output => (desired_x, endpoint.x - context.rail_depth - context.pitch),
    };
    let minimum_x = minimum_x.max(lower_bound);
    let maximum_x = maximum_x.min(upper_bound);
    if !minimum_x.is_finite() || !maximum_x.is_finite() || minimum_x > maximum_x {
        return None;
    }
    let desired_x = desired_x.max(minimum_x).min(maximum_x);
    let desired_x = common_horizontal_collector_x(
        context,
        bundle,
        routes,
        minimum_x,
        desired_x,
        maximum_x,
        horizontal_tap_visits,
    )?;
    Some(InteriorCollectorRange {
        role: bundle.role,
        minimum_x,
        desired_x,
        maximum_x,
    })
}

fn common_horizontal_collector_x(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    routes: &[EdgeGeometry],
    minimum_x: f64,
    desired_x: f64,
    maximum_x: f64,
    remaining: &mut usize,
) -> Option<f64> {
    let mut candidates = vec![desired_x];
    for member in &bundle.members {
        let route = routes.get(*context.route_index.get(&member.edge)?)?;
        for pair in route.points.windows(2) {
            *remaining = remaining.checked_sub(1)?;
            let Some((segment_low, segment_high)) =
                shared_trunk_horizontal_range(pair[0], pair[1], bundle.role, context.pitch)
            else {
                continue;
            };
            let low = segment_low.max(minimum_x);
            let high = segment_high.min(maximum_x);
            if low > high {
                continue;
            }
            candidates.push(match bundle.role {
                BoundaryBundleRole::Input => high.min(desired_x),
                BoundaryBundleRole::Output => low.max(desired_x),
            });
        }
    }
    candidates.sort_by(|left, right| match bundle.role {
        BoundaryBundleRole::Input => right.total_cmp(left),
        BoundaryBundleRole::Output => left.total_cmp(right),
    });
    candidates.dedup_by(|left, right| left.to_bits() == right.to_bits());
    for candidate in candidates {
        let mut common = true;
        for member in &bundle.members {
            let route = routes.get(*context.route_index.get(&member.edge)?)?;
            let mut found = false;
            for pair in route.points.windows(2) {
                *remaining = remaining.checked_sub(1)?;
                if shared_trunk_horizontal_range(pair[0], pair[1], bundle.role, context.pitch)
                    .is_some_and(|(low, high)| candidate >= low && candidate <= high)
                {
                    found = true;
                    break;
                }
            }
            if !found {
                common = false;
                break;
            }
        }
        if common {
            return Some(candidate);
        }
    }
    None
}

fn build_geometry(
    context: &BundleGeometryContext<'_, '_>,
    bundle: &IndexedBoundaryBundle,
    routes: &[EdgeGeometry],
    interior_collector_x: Option<f64>,
) -> BoundaryBundleGeometry {
    let node = &context.nodes[context.node_geometry[&bundle.endpoint.node]];
    let indexed_node = context.graph.node_index[&bundle.endpoint.node];
    let port = context.graph.ports[indexed_node][&bundle.endpoint.port];
    let endpoint = port_point(node, port);
    let direction = match bundle.role {
        BoundaryBundleRole::Input => 1.0,
        BoundaryBundleRole::Output => -1.0,
    };
    if let Some(collector_x) = interior_collector_x
        && let Some(geometry) =
            build_interior_geometry(context, bundle, endpoint, routes, collector_x)
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
    collector_x: f64,
) -> Option<BoundaryBundleGeometry> {
    let mut members = Vec::with_capacity(bundle.members.len());
    let mut collector_low = endpoint.y;
    let mut collector_high = endpoint.y;
    for member in &bundle.members {
        let route = routes.get(*context.route_index.get(&member.edge)?)?;
        let (_, tap) = match bundle.role {
            BoundaryBundleRole::Input => first_shared_trunk_intersection(
                &route.points,
                collector_x,
                bundle.role,
                context.pitch,
            ),
            BoundaryBundleRole::Output => last_shared_trunk_intersection(
                &route.points,
                collector_x,
                bundle.role,
                context.pitch,
            ),
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
                BoundaryBundleRole::Input => {
                    rewrite_input_shared_trunk(&route.points, member.tap, pitch)
                }
                BoundaryBundleRole::Output => {
                    rewrite_output_shared_trunk(&route.points, member.tap, pitch)
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

fn rewrite_input_shared_trunk(
    points: &[Point],
    tap: Point,
    minimum_length: f64,
) -> Option<Vec<Point>> {
    let (segment, intersection) =
        first_shared_trunk_intersection(points, tap.x, BoundaryBundleRole::Input, minimum_length)?;
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

fn rewrite_output_shared_trunk(
    points: &[Point],
    tap: Point,
    minimum_length: f64,
) -> Option<Vec<Point>> {
    let (segment, intersection) =
        last_shared_trunk_intersection(points, tap.x, BoundaryBundleRole::Output, minimum_length)?;
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

fn first_shared_trunk_intersection(
    points: &[Point],
    x: f64,
    role: BoundaryBundleRole,
    minimum_length: f64,
) -> Option<(usize, Point)> {
    points.windows(2).enumerate().find_map(|(segment, pair)| {
        shared_trunk_horizontal_intersection(pair[0], pair[1], x, role, minimum_length)
            .map(|point| (segment, point))
    })
}

fn last_shared_trunk_intersection(
    points: &[Point],
    x: f64,
    role: BoundaryBundleRole,
    minimum_length: f64,
) -> Option<(usize, Point)> {
    points
        .windows(2)
        .enumerate()
        .rev()
        .find_map(|(segment, pair)| {
            shared_trunk_horizontal_intersection(pair[0], pair[1], x, role, minimum_length)
                .map(|point| (segment, point))
        })
}

fn shared_trunk_horizontal_intersection(
    start: Point,
    end: Point,
    x: f64,
    role: BoundaryBundleRole,
    minimum_length: f64,
) -> Option<Point> {
    shared_trunk_horizontal_range(start, end, role, minimum_length)
        .is_some_and(|(low, high)| x >= low && x <= high)
        .then_some(Point { x, y: start.y })
}

fn shared_trunk_horizontal_range(
    start: Point,
    end: Point,
    role: BoundaryBundleRole,
    minimum_length: f64,
) -> Option<(f64, f64)> {
    if start.y != end.y || start.x >= end.x {
        return None;
    }
    let (low, high) = match role {
        BoundaryBundleRole::Input => (start.x, end.x - minimum_length),
        BoundaryBundleRole::Output => (start.x + minimum_length, end.x),
    };
    (low <= high).then_some((low, high))
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
    remaining: &mut usize,
) -> Result<(), LayoutError> {
    let layout_bundle = layout
        .boundary_bundles
        .len()
        .checked_sub(1)
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
    verify_one_bundle_node_clearance(layout, options, layout_bundle, remaining)?;
    verify_one_bundle_route_node_interiors(graph, layout, bundle, remaining)?;
    verify_new_bundle_route_contacts(
        layout,
        layout_bundle,
        graph.boundary_bundles[bundle]
            .members
            .iter()
            .map(|member| member.edge)
            .collect(),
        options.minimum_parallel_wire_spacing,
        remaining,
    )?;
    verify_rewritten_route_contacts(graph, layout, options, remaining)
}

fn verify_one_bundle_node_clearance(
    layout: &Layout,
    options: LayoutOptions,
    bundle: usize,
    remaining: &mut usize,
) -> Result<(), LayoutError> {
    let geometry = layout
        .boundary_bundles
        .get(bundle)
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
    let net = BUNDLE_CLEARANCE_NET_BASE.wrapping_add(bundle as u32);
    let mut segments = Vec::with_capacity(2);
    push_clearance_segment(&mut segments, net, geometry.collector)?;
    push_clearance_segment(&mut segments, net, geometry.spine)?;
    let maximum = segments.len().saturating_mul(layout.nodes.len()).max(1);
    *remaining = remaining.checked_sub(maximum).ok_or(
        LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
            maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
        },
    )?;
    let threshold = options.edge_node_clearance.max(f64::EPSILON);
    match measure_edge_node_clearance_bounded(
        &segments,
        &layout.nodes,
        &[NetNodeRelation {
            net,
            node: geometry.endpoint.node,
        }],
        threshold,
        maximum,
    ) {
        Ok(clearance) if clearance.violations == 0 => Ok(()),
        Ok(_) | Err(EdgeNodeClearanceError::InvalidInput) => {
            Err(LayoutError::BoundaryBundleGeometryUnsatisfied)
        }
        Err(EdgeNodeClearanceError::WorkLimitExceeded) => {
            Err(LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
                maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
            })
        }
    }
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
        let shared_local_collector = !interior_collector
            && bundle_members_share_opposite_endpoint(graph, bundle)
            && geometry.members.first().is_some_and(|first| {
                first.tap.x.is_finite()
                    && first.tap.y.is_finite()
                    && geometry
                        .members
                        .iter()
                        .all(|member| preserved_point_matches(member.tap, first.tap))
                    && (first.tap.x - fallback_collector_center.x).abs()
                        <= PRESERVED_GEOMETRY_EPSILON
            });
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
            let expected_tap = if interior_collector || shared_local_collector {
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
    let mut remaining = MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS;
    verify_member_route_node_interiors(graph, layout, &member_edges, &mut remaining)
}

fn verify_one_bundle_route_node_interiors(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    bundle: usize,
    remaining: &mut usize,
) -> Result<(), LayoutError> {
    let member_edges = graph
        .boundary_bundles
        .get(bundle)
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?
        .members
        .iter()
        .map(|member| member.edge)
        .collect::<BTreeSet<_>>();
    verify_member_route_node_interiors(graph, layout, &member_edges, remaining)
}

fn verify_member_route_node_interiors(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    member_edges: &BTreeSet<u32>,
    remaining: &mut usize,
) -> Result<(), LayoutError> {
    let graph_edges = graph
        .edges
        .iter()
        .map(|edge| (edge.id, *edge))
        .collect::<HashMap<_, _>>();
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
                *remaining = remaining.checked_sub(1).ok_or(
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

fn verify_new_bundle_route_contacts(
    layout: &Layout,
    new_bundle: usize,
    changed_routes: BTreeSet<u32>,
    minimum_spacing: f64,
    remaining: &mut usize,
) -> Result<(), LayoutError> {
    let geometry = layout
        .boundary_bundles
        .get(new_bundle)
        .ok_or(LayoutError::BoundaryBundleGeometryUnsatisfied)?;
    let permitted_taps = geometry
        .members
        .iter()
        .map(|member| (member.edge, member.tap))
        .collect::<HashMap<_, _>>();
    for segment in [geometry.collector, geometry.spine] {
        for route in &layout.edges {
            let permitted_tap = permitted_taps.get(&route.id).copied();
            for pair in route.points.windows(2) {
                consume_geometry_visit(remaining)?;
                if segments_have_disallowed_contact(segment, pair[0], pair[1], permitted_tap)
                    || parallel_segments_are_too_close(
                        segment,
                        BoundaryBundleSegment {
                            start: pair[0],
                            end: pair[1],
                        },
                        minimum_spacing,
                    )
                {
                    return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                }
            }
        }
    }
    for prior in &layout.boundary_bundles[..new_bundle] {
        let prior_taps = prior
            .members
            .iter()
            .map(|member| (member.edge, member.tap))
            .collect::<HashMap<_, _>>();
        for segment in [prior.collector, prior.spine] {
            for route in layout
                .edges
                .iter()
                .filter(|route| changed_routes.contains(&route.id))
            {
                let permitted_tap = prior_taps.get(&route.id).copied();
                for pair in route.points.windows(2) {
                    consume_geometry_visit(remaining)?;
                    if segments_have_disallowed_contact(segment, pair[0], pair[1], permitted_tap)
                        || parallel_segments_are_too_close(
                            segment,
                            BoundaryBundleSegment {
                                start: pair[0],
                                end: pair[1],
                            },
                            minimum_spacing,
                        )
                    {
                        return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                    }
                }
            }
        }
        for new_segment in [geometry.collector, geometry.spine] {
            for prior_segment in [prior.collector, prior.spine] {
                consume_geometry_visit(remaining)?;
                if segments_have_disallowed_contact(
                    new_segment,
                    prior_segment.start,
                    prior_segment.end,
                    None,
                ) || parallel_segments_are_too_close(new_segment, prior_segment, minimum_spacing)
                {
                    return Err(LayoutError::BoundaryBundleGeometryUnsatisfied);
                }
            }
        }
    }
    Ok(())
}

fn consume_geometry_visit(remaining: &mut usize) -> Result<(), LayoutError> {
    *remaining =
        remaining
            .checked_sub(1)
            .ok_or(LayoutError::BoundaryBundleGeometryWorkLimitExceeded {
                maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
            })?;
    Ok(())
}

fn verify_rewritten_route_contacts(
    graph: &IndexedGraph<'_>,
    layout: &Layout,
    options: LayoutOptions,
    remaining: &mut usize,
) -> Result<(), LayoutError> {
    if options.edge_node_clearance <= 0.0 {
        return Ok(());
    }
    let segments = layout
        .edges
        .iter()
        .map(|route| route.points.len().saturating_sub(1))
        .sum::<usize>();
    if segments > crate::MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS {
        return Err(LayoutError::UnrelatedRouteContactSegmentLimitExceeded {
            maximum: crate::MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
        });
    }
    let maximum = segments.saturating_mul(segments).max(1);
    *remaining = remaining.checked_sub(maximum).ok_or(
        LayoutError::UnrelatedRouteContactWorkLimitExceeded {
            maximum: MAX_BOUNDARY_BUNDLE_GEOMETRY_VISITS,
        },
    )?;
    match crate::routing::route_family_has_unrelated_contact_bounded(
        graph,
        &layout.edges,
        crate::MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
        maximum,
    ) {
        Ok(false) => Ok(()),
        Ok(true) | Err(crate::routing::RouteContactError::InvalidInput) => {
            Err(LayoutError::BoundaryBundleGeometryUnsatisfied)
        }
        Err(crate::routing::RouteContactError::WorkLimitExceeded) => {
            Err(LayoutError::UnrelatedRouteContactWorkLimitExceeded { maximum })
        }
        Err(crate::routing::RouteContactError::SegmentLimitExceeded) => {
            Err(LayoutError::UnrelatedRouteContactSegmentLimitExceeded {
                maximum: crate::MAX_LAYOUT_ROUTE_CONTACT_SEGMENTS,
            })
        }
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
    fn common_horizontal_collector_obeys_the_exact_visit_boundary() {
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
        let nodes = [
            geometry(1, 0.0),
            geometry(2, 300.0),
            geometry_at(3, 300.0, 100.0),
        ];
        let node_geometry = HashMap::from([(1, 0), (2, 1), (3, 2)]);
        let route_index = HashMap::from([(10, 0), (11, 1)]);
        let empty_corridors = HashMap::new();
        let context = BundleGeometryContext {
            graph: &indexed,
            nodes: &nodes,
            node_geometry: &node_geometry,
            route_index: &route_index,
            input_fallback_corridors: &empty_corridors,
            output_fallback_corridors: &empty_corridors,
            pitch: 6.0,
            rail_depth: 12.0,
            member_endpoint_reserve: 6.0,
        };
        let routes = [
            EdgeGeometry {
                id: 10,
                points: vec![Point { x: 0.0, y: 10.0 }, Point { x: 100.0, y: 10.0 }],
            },
            EdgeGeometry {
                id: 11,
                points: vec![Point { x: 0.0, y: 20.0 }, Point { x: 100.0, y: 20.0 }],
            },
        ];
        let bundle = &indexed.boundary_bundles[0];

        let mut exact_budget = 4;
        assert_eq!(
            common_horizontal_collector_x(
                &context,
                bundle,
                &routes,
                0.0,
                50.0,
                100.0,
                &mut exact_budget,
            ),
            Some(50.0)
        );
        assert_eq!(exact_budget, 0);

        let mut exhausted_budget = 3;
        assert_eq!(
            common_horizontal_collector_x(
                &context,
                bundle,
                &routes,
                0.0,
                50.0,
                100.0,
                &mut exhausted_budget,
            ),
            None
        );
        assert_eq!(exhausted_budget, 0);
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
                end: Point { x: 290.0, y: 25.0 },
            }
        );
        assert_eq!(
            bundle.collector,
            BoundaryBundleSegment {
                start: Point { x: 290.0, y: 25.0 },
                end: Point { x: 290.0, y: 125.0 },
            }
        );
        assert_eq!(
            interior.edges,
            vec![
                EdgeGeometry {
                    id: 10,
                    points: vec![Point { x: 290.0, y: 25.0 }, Point { x: 300.0, y: 25.0 }],
                },
                EdgeGeometry {
                    id: 11,
                    points: vec![Point { x: 290.0, y: 125.0 }, Point { x: 300.0, y: 125.0 }],
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
    fn interior_collector_falls_back_from_an_unrelated_route_contact() {
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::West),
                node(3, PortSide::West),
                node(4, PortSide::East),
                node(5, PortSide::West),
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
                Edge {
                    id: 12,
                    source: Endpoint { node: 4, port: 0 },
                    target: Endpoint { node: 5, port: 0 },
                    net: 12,
                    participates_in_ranking: true,
                },
            ],
        };
        let constraints = LayoutConstraints {
            inputs: vec![1, 4],
            outputs: vec![2, 3, 5],
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
        let unrelated = EdgeGeometry {
            id: 12,
            points: vec![
                Point { x: 80.0, y: 225.0 },
                Point { x: 250.0, y: 225.0 },
                Point { x: 250.0, y: 75.0 },
                Point { x: 350.0, y: 75.0 },
                Point { x: 350.0, y: 225.0 },
                Point { x: 600.0, y: 225.0 },
            ],
        };
        let layout = Layout {
            nodes: vec![
                geometry_at(1, 0.0, 0.0),
                geometry_at(2, 300.0, 0.0),
                geometry_at(3, 300.0, 100.0),
                geometry_at(4, 0.0, 200.0),
                geometry_at(5, 600.0, 200.0),
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
                unrelated.clone(),
            ],
            boundary_bundles: Vec::new(),
            width: 680.0,
            height: 250.0,
        };

        let result = apply_and_normalize(&indexed, layout, options).unwrap();
        assert_eq!(result.boundary_bundles[0].spine.end.x, 94.0);
        assert_eq!(
            result.edges.iter().find(|route| route.id == 12),
            Some(&unrelated)
        );
    }

    #[test]
    fn vector_output_uses_one_interior_collector_after_member_convergence() {
        let graph = Graph {
            nodes: vec![
                node(1, PortSide::East),
                node(2, PortSide::East),
                node(3, PortSide::West),
                node(4, PortSide::West),
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

        let interior = apply_and_normalize(&indexed, layout.clone(), options).unwrap();
        let bundle = &interior.boundary_bundles[0];
        assert_eq!(
            bundle.spine,
            BoundaryBundleSegment {
                start: Point { x: 300.0, y: 25.0 },
                end: Point { x: 90.0, y: 25.0 },
            }
        );
        assert_eq!(
            bundle.collector,
            BoundaryBundleSegment {
                start: Point { x: 90.0, y: 25.0 },
                end: Point { x: 90.0, y: 125.0 },
            }
        );
        assert_eq!(
            interior.edges,
            vec![
                EdgeGeometry {
                    id: 10,
                    points: vec![Point { x: 80.0, y: 25.0 }, Point { x: 90.0, y: 25.0 }],
                },
                EdgeGeometry {
                    id: 11,
                    points: vec![Point { x: 80.0, y: 125.0 }, Point { x: 90.0, y: 125.0 }],
                },
            ]
        );
        assert_eq!(
            apply_and_normalize(&indexed, interior.clone(), options).unwrap(),
            interior
        );

        let mut blocked = layout;
        blocked.nodes[3] = geometry_at(4, 80.0, 50.0);
        let fallback = apply_and_normalize(&indexed, blocked, options).unwrap();
        assert_eq!(fallback.boundary_bundles[0].spine.end.x, 286.0);
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
