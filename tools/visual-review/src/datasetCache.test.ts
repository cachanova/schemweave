import { expect, it, vi } from 'vitest'

import { DatasetCache } from './datasetCache'

it('never reuses a same-name score across loaded datasets', () => {
  const cache = new DatasetCache<number>()
  const first = vi.fn(() => 11)
  const second = vi.fn(() => 22)

  expect(cache.getOrInsert(1, 'fifo', first)).toBe(11)
  expect(cache.getOrInsert(1, 'fifo', first)).toBe(11)
  expect(cache.getOrInsert(2, 'fifo', second)).toBe(22)
  expect(first).toHaveBeenCalledTimes(1)
  expect(second).toHaveBeenCalledTimes(1)
})
