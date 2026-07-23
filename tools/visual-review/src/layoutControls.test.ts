import { describe, expect, it } from 'vitest'

import { layoutPresets, matchingPreset } from './layoutControls'
import type { LayoutOptions } from './types'

describe('layout controls', () => {
  it('defines the isolated review default as the engine highest-quality profile', () => {
    expect(layoutPresets['highest-quality']).toEqual({
      layer_gap: 66,
      node_gap: 30,
      port_stub: 10,
      route_lane_gap: 6,
      edge_node_clearance: 20,
      ordering_sweeps: 4,
      quality_effort: 'max',
    })
  })

  it('matches presets using both quality effort and wire-to-gate clearance', () => {
    const highestQuality: LayoutOptions = { ...layoutPresets['highest-quality'] }
    expect(matchingPreset(highestQuality)).toBe('highest-quality')
    expect(matchingPreset({ ...highestQuality, quality_effort: 'quality' })).toBeNull()
    expect(matchingPreset({ ...highestQuality, edge_node_clearance: 15 })).toBeNull()
    expect(matchingPreset({ ...layoutPresets.balanced })).toBe('balanced')
  })
})
