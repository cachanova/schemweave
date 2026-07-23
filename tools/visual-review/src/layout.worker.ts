/// <reference lib="webworker" />

import init, { review_layout_json, score_json } from './generated/schemweave_review_wasm.js'
import { DatasetCache } from './datasetCache'
import { elkAsLayout } from './graph'
import { LARGE_LAYOUT_NODE_THRESHOLD } from './latestRequest'
import type {
  Layout,
  LayoutOptions,
  LayoutPhase,
  QualityReport,
  WorkerLayoutRequest,
  WorkerRequest,
  WorkerResponse,
} from './types'

const ready = init()
const elkQuality = new DatasetCache<QualityReport>()
const workerScope = self as unknown as DedicatedWorkerGlobalScope
let retained: RetainedRequest | null = null

interface RetainedRequest {
  request: RetainedRequestMetadata
  graphJson: string
  elkQuality: QualityReport
}

type RetainedRequestMetadata = Pick<
  WorkerLayoutRequest,
  'generation' | 'id' | 'datasetId' | 'fixtureName' | 'options'
>

workerScope.onmessage = async (event: MessageEvent<WorkerRequest>) => {
  const request = event.data
  try {
    await ready
    if (request.type === 'refine') {
      if (!retained || !matches(retained.request, request)) return
      const current = retained
      retained = null
      postLayout(current, current.request.options, 'final', true)
      return
    }

    const graphJson = JSON.stringify(request.graph)
    const currentElkQuality = elkQuality.getOrInsert(request.datasetId, request.fixtureName, () =>
      JSON.parse(score_json(graphJson, JSON.stringify(elkAsLayout(request.elk)))),
    )
    const current = {
      request: retainMetadata(request),
      graphJson,
      elkQuality: currentElkQuality,
    }
    const large = request.graph.nodes.length >= LARGE_LAYOUT_NODE_THRESHOLD
    const needsRefinement = large && request.options.quality_effort !== 'fast'
    if (needsRefinement) retained = current
    const options = needsRefinement
      ? { ...request.options, quality_effort: 'fast' as const }
      : request.options
    postLayout(current, options, large ? 'fast' : 'direct', !needsRefinement)
  } catch (error) {
    retained = null
    const response: WorkerResponse = {
      generation: request.generation,
      id: request.id,
      datasetId: request.datasetId,
      fixtureName: request.fixtureName,
      phase: request.type === 'refine' ? 'final' : initialPhase(request),
      error: error instanceof Error ? error.message : String(error),
    }
    workerScope.postMessage(response)
  }
}

function postLayout(
  current: RetainedRequest,
  options: LayoutOptions,
  phase: LayoutPhase,
  final: boolean,
): void {
  const started = performance.now()
  const layoutJson = review_layout_json(current.graphJson, JSON.stringify(options))
  const layout = JSON.parse(layoutJson) as Layout
  const elapsedMs = performance.now() - started
  const quality = JSON.parse(score_json(current.graphJson, layoutJson)) as QualityReport
  const { request } = current
  const response: WorkerResponse = {
    generation: request.generation,
    id: request.id,
    datasetId: request.datasetId,
    fixtureName: request.fixtureName,
    phase,
    effort: options.quality_effort,
    requestedEffort: request.options.quality_effort,
    final,
    elapsedMs,
    layout,
    quality,
    elkQuality: current.elkQuality,
  }
  workerScope.postMessage(response)
}

function matches(
  retainedRequest: RetainedRequestMetadata,
  request: Extract<WorkerRequest, { type: 'refine' }>,
): boolean {
  return (
    retainedRequest.generation === request.generation &&
    retainedRequest.id === request.id &&
    retainedRequest.datasetId === request.datasetId &&
    retainedRequest.fixtureName === request.fixtureName
  )
}

function retainMetadata(request: WorkerLayoutRequest): RetainedRequestMetadata {
  const { generation, id, datasetId, fixtureName, options } = request
  return { generation, id, datasetId, fixtureName, options }
}

function initialPhase(request: WorkerLayoutRequest): LayoutPhase {
  return request.graph.nodes.length >= LARGE_LAYOUT_NODE_THRESHOLD ? 'fast' : 'direct'
}
