import { afterEach, beforeEach, expect, it, vi } from 'vitest'

import { REFINE_QUIESCENCE_MS, StagedLayoutRequests } from './latestRequest'
import type {
  LayoutOptions,
  LayoutSubmission,
  QualityReport,
  WorkerLayoutRequest,
  WorkerRequest,
  WorkerResponse,
} from './types'

beforeEach(() => vi.useFakeTimers())
afterEach(() => vi.useRealTimers())

it('returns Fast first, then requests only the selected Quality refinement after quiescence', () => {
  const harness = createHarness()
  harness.requests.submit(submission(600, 'quality'))
  const initial = harness.sent[0] as WorkerLayoutRequest

  harness.requests.receive(result(initial, 'fast', 'fast', false))
  expect(harness.results.map((response) => response.phase)).toEqual(['fast'])
  expect(harness.requests.refining).toBe(true)

  vi.advanceTimersByTime(REFINE_QUIESCENCE_MS - 1)
  expect(harness.sent).toHaveLength(1)
  vi.advanceTimersByTime(1)

  expect(harness.sent).toHaveLength(2)
  expect(harness.sent[1]).toEqual({
    type: 'refine',
    generation: initial.generation,
    id: initial.id,
    datasetId: initial.datasetId,
    fixtureName: initial.fixtureName,
  })
  expect(harness.sent[1]).not.toHaveProperty('graph')

  harness.requests.receive(result(initial, 'final', 'quality', true))
  expect(harness.results.map((response) => response.phase)).toEqual(['fast', 'final'])
  expect(harness.requests.busy).toBe(false)
})

it('refines a large Max request directly to Max without a Quality phase', () => {
  const harness = createHarness()
  harness.requests.submit(submission(600, 'max'))
  const initial = harness.sent[0] as WorkerLayoutRequest
  expect(initial.options.quality_effort).toBe('max')

  harness.requests.receive(result(initial, 'fast', 'fast', false))
  vi.advanceTimersByTime(REFINE_QUIESCENCE_MS)
  expect(harness.sent.map((request) => request.type)).toEqual(['layout', 'refine'])

  harness.requests.receive(result(initial, 'final', 'max', true))
  expect(harness.results.map((response) => response.effort)).toEqual(['fast', 'max'])
})

it('runs the explicitly requested effort directly below 600 nodes', () => {
  const harness = createHarness()
  harness.requests.submit(submission(599, 'max'))
  const initial = harness.sent[0] as WorkerLayoutRequest

  harness.requests.receive(result(initial, 'direct', 'max', true))
  vi.advanceTimersByTime(REFINE_QUIESCENCE_MS)

  expect(harness.sent).toHaveLength(1)
  expect(harness.results.map((response) => response.phase)).toEqual(['direct'])
})

it('restarts an active large worker and rejects its stale generation', () => {
  const harness = createHarness()
  harness.requests.submit(submission(800, 'quality', 1))
  const stale = harness.sent[0] as WorkerLayoutRequest
  harness.requests.submit(submission(20, 'quality', 2))
  const current = harness.sent[1] as WorkerLayoutRequest

  expect(harness.restartWorker).toHaveBeenCalledTimes(1)
  expect(current.generation).toBeGreaterThan(stale.generation)

  harness.requests.receive(result(stale, 'fast', 'fast', false))
  expect(harness.results).toEqual([])
  harness.requests.receive(result(current, 'direct', 'quality', true))
  expect(harness.results).toHaveLength(1)
  expect(harness.results[0].id).toBe(2)
})

it('keeps only the newest request queued behind a small active layout', () => {
  const harness = createHarness()
  harness.requests.submit(submission(20, 'quality', 1))
  const active = harness.sent[0] as WorkerLayoutRequest
  harness.requests.submit(submission(20, 'fast', 2))
  harness.requests.submit(submission(20, 'max', 3))

  expect(harness.sent).toHaveLength(1)
  expect(harness.requests.hasPending).toBe(true)
  harness.requests.receive(result(active, 'direct', 'quality', true))

  expect(harness.sent).toHaveLength(2)
  expect((harness.sent[1] as WorkerLayoutRequest).id).toBe(3)
  expect(harness.restartWorker).not.toHaveBeenCalled()
})

function createHarness() {
  const sent: WorkerRequest[] = []
  const results: Array<Exclude<WorkerResponse, { error: string }>> = []
  const restartWorker = vi.fn()
  const requests = new StagedLayoutRequests(
    {
      send: (request) => sent.push(request),
      restartWorker,
      onStarted: vi.fn(),
      onResult: (response) => results.push(response),
      onError: vi.fn(),
    },
    (callback, delay) => globalThis.setTimeout(callback, delay) as unknown as number,
    (timer) => globalThis.clearTimeout(timer),
  )
  return { requests, sent, results, restartWorker }
}

function submission(
  nodeCount: number,
  effort: LayoutOptions['quality_effort'],
  id = 1,
): LayoutSubmission {
  return {
    id,
    datasetId: 4,
    fixtureName: `fixture-${id}`,
    graph: {
      nodes: Array.from({ length: nodeCount }, (_, nodeId) => ({
        id: nodeId,
        width: 20,
        height: 20,
        cycle_breaker: false,
        ports: [],
      })),
      edges: [],
    },
    elk: { nodes: [], edges: [], width: 0, height: 0 },
    options: {
      layer_gap: 66,
      node_gap: 30,
      port_stub: 10,
      route_lane_gap: 4,
      ordering_sweeps: 4,
      quality_effort: effort,
    },
  }
}

function result(
  request: WorkerLayoutRequest,
  phase: 'direct' | 'fast' | 'final',
  effort: LayoutOptions['quality_effort'],
  final: boolean,
): Exclude<WorkerResponse, { error: string }> {
  return {
    generation: request.generation,
    id: request.id,
    datasetId: request.datasetId,
    fixtureName: request.fixtureName,
    phase,
    effort,
    requestedEffort: request.options.quality_effort,
    final,
    elapsedMs: 1,
    layout: { nodes: [], edges: [], width: 0, height: 0 },
    quality: {} as QualityReport,
    elkQuality: {} as QualityReport,
  }
}
