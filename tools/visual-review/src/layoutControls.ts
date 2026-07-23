import type { LayoutOptions } from './types'

export type LayoutPreset = Pick<
  LayoutOptions,
  | 'layer_gap'
  | 'node_gap'
  | 'port_stub'
  | 'route_lane_gap'
  | 'minimum_parallel_wire_spacing'
  | 'edge_node_clearance'
  | 'ordering_sweeps'
  | 'quality_effort'
>

export const layoutPresets = {
  compact: {
    layer_gap: 48,
    node_gap: 20,
    port_stub: 10,
    route_lane_gap: 3,
    minimum_parallel_wire_spacing: 0,
    edge_node_clearance: 0,
    ordering_sweeps: 2,
    quality_effort: 'quality',
  },
  balanced: {
    layer_gap: 66,
    node_gap: 30,
    port_stub: 10,
    route_lane_gap: 4,
    minimum_parallel_wire_spacing: 0,
    edge_node_clearance: 0,
    ordering_sweeps: 4,
    quality_effort: 'quality',
  },
  roomy: {
    layer_gap: 84,
    node_gap: 45,
    port_stub: 10,
    route_lane_gap: 6,
    minimum_parallel_wire_spacing: 0,
    edge_node_clearance: 20,
    ordering_sweeps: 4,
    quality_effort: 'quality',
  },
  debug: {
    layer_gap: 108,
    node_gap: 60,
    port_stub: 10,
    route_lane_gap: 8,
    minimum_parallel_wire_spacing: 0,
    edge_node_clearance: 20,
    ordering_sweeps: 8,
    quality_effort: 'max',
  },
  'highest-quality': {
    layer_gap: 66,
    node_gap: 30,
    port_stub: 10,
    route_lane_gap: 6,
    minimum_parallel_wire_spacing: 0,
    edge_node_clearance: 20,
    ordering_sweeps: 4,
    quality_effort: 'max',
  },
} as const satisfies Record<string, LayoutPreset>

export type PresetName = keyof typeof layoutPresets

export function matchingPreset(options: LayoutOptions): PresetName | null {
  const match = Object.entries(layoutPresets).find(([, preset]) =>
    Object.entries(preset).every(
      ([key, value]) => options[key as keyof LayoutPreset] === value,
    ),
  )
  return (match?.[0] as PresetName | undefined) ?? null
}

export function parallelWireSpacingStatus(options: LayoutOptions): string {
  const spacing = options.minimum_parallel_wire_spacing
  return spacing > 0 ? `hard wire spacing ≥ ${spacing} px` : 'hard wire spacing off'
}
