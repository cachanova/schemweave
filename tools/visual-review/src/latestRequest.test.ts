import { expect, it, vi } from 'vitest'

import { LatestRequest } from './latestRequest'

it('runs one request at a time and retains only the newest pending settings', () => {
  const send = vi.fn()
  const queue = new LatestRequest<string>(send)

  queue.submit('balanced')
  queue.submit('roomy')
  queue.submit('debug')
  expect(send.mock.calls).toEqual([['balanced']])
  expect(queue.busy).toBe(true)
  expect(queue.hasPending).toBe(true)

  queue.complete()
  expect(send.mock.calls).toEqual([['balanced'], ['debug']])
  expect(queue.busy).toBe(true)
  expect(queue.hasPending).toBe(false)

  queue.complete()
  expect(queue.busy).toBe(false)
})

it('rejects stray completions that could corrupt in-flight state', () => {
  const queue = new LatestRequest(() => undefined)
  expect(() => queue.complete()).toThrow('idle request queue')
})
