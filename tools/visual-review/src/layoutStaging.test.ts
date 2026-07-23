import { expect, it } from 'vitest'

import { layoutPresets } from './layoutControls'
import { initialLayoutOptions } from './layoutStaging'

it('preserves wire-to-gate clearance during staged Fast layout', () => {
  const requested = { ...layoutPresets['highest-quality'] }

  expect(initialLayoutOptions(requested, true)).toEqual({
    ...requested,
    quality_effort: 'fast',
    edge_node_clearance: 20,
  })
  expect(initialLayoutOptions(requested, false)).toBe(requested)
})
