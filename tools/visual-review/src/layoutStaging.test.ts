import { expect, it } from 'vitest'

import { layoutPresets } from './layoutControls'
import { initialLayoutOptions } from './layoutStaging'

it('preserves hard spacing guarantees during staged Fast layout', () => {
  const requested = {
    ...layoutPresets['highest-quality'],
    minimum_parallel_wire_spacing: 6,
  }

  expect(initialLayoutOptions(requested, true)).toEqual({
    ...requested,
    quality_effort: 'fast',
    edge_node_clearance: 20,
    minimum_parallel_wire_spacing: 6,
  })
  expect(initialLayoutOptions(requested, false)).toBe(requested)
})
