/// <reference lib="webworker" />

import init, { review_layout_json, score_json } from './generated/schemweave_review_wasm.js'
import { elkAsLayout } from './graph'
import type { Layout, QualityReport, WorkerRequest, WorkerResponse } from './types'

const ready = init()
const elkQualityByFixture = new Map<string, QualityReport>()
const workerScope = self as unknown as DedicatedWorkerGlobalScope

workerScope.onmessage = async (event: MessageEvent<WorkerRequest>) => {
  const request = event.data
  try {
    await ready
    const graphJson = JSON.stringify(request.graph)
    let elkQuality = elkQualityByFixture.get(request.fixtureName)
    if (!elkQuality) {
      elkQuality = JSON.parse(
        score_json(graphJson, JSON.stringify(elkAsLayout(request.elk))),
      ) as QualityReport
      elkQualityByFixture.set(request.fixtureName, elkQuality)
    }
    const started = performance.now()
    const layoutJson = review_layout_json(graphJson, JSON.stringify(request.options))
    const layout = JSON.parse(layoutJson) as Layout
    const elapsedMs = performance.now() - started
    const quality = JSON.parse(score_json(graphJson, layoutJson)) as QualityReport
    const response: WorkerResponse = {
      id: request.id,
      fixtureName: request.fixtureName,
      elapsedMs,
      layout,
      quality,
      elkQuality,
    }
    workerScope.postMessage(response)
  } catch (error) {
    const response: WorkerResponse = {
      id: request.id,
      fixtureName: request.fixtureName,
      error: error instanceof Error ? error.message : String(error),
    }
    workerScope.postMessage(response)
  }
}
