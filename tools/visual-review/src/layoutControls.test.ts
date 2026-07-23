import { describe, expect, it } from 'vitest'

import {
  layoutPresets,
  matchingPreset,
  parallelWireSpacingStatus,
} from './layoutControls'
import type { LayoutOptions } from './types'

describe('layout controls', () => {
  it('defines the isolated review default as the engine highest-quality profile', () => {
    expect(layoutPresets['highest-quality']).toEqual({
      layer_gap: 66,
      node_gap: 30,
      port_stub: 10,
      route_lane_gap: 6,
      minimum_parallel_wire_spacing: 0,
      edge_node_clearance: 20,
      ordering_sweeps: 4,
      quality_effort: 'max',
    })
  })

  it('keeps the hard wire-spacing guarantee disabled in every preset', () => {
    expect(
      Object.values(layoutPresets).every(
        (preset) => preset.minimum_parallel_wire_spacing === 0,
      ),
    ).toBe(true)
  })

  it('matches presets using quality effort and both hard spacing controls', () => {
    const highestQuality: LayoutOptions = { ...layoutPresets['highest-quality'] }
    expect(matchingPreset(highestQuality)).toBe('highest-quality')
    expect(matchingPreset({ ...highestQuality, quality_effort: 'quality' })).toBeNull()
    expect(matchingPreset({ ...highestQuality, edge_node_clearance: 15 })).toBeNull()
    expect(matchingPreset({ ...highestQuality, minimum_parallel_wire_spacing: 6 })).toBeNull()
    expect(matchingPreset({ ...layoutPresets.balanced })).toBe('balanced')
  })

  it('describes whether the hard parallel-wire guarantee is active', () => {
    const highestQuality: LayoutOptions = { ...layoutPresets['highest-quality'] }
    expect(parallelWireSpacingStatus(highestQuality)).toBe('hard wire spacing off')
    expect(
      parallelWireSpacingStatus({
        ...highestQuality,
        minimum_parallel_wire_spacing: 6,
      }),
    ).toBe('hard wire spacing ≥ 6 px')
  })
})
