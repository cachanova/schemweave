import type {
  LayoutSubmission,
  WorkerLayoutRequest,
  WorkerRequest,
  WorkerResponse,
} from './types'

export const LARGE_LAYOUT_NODE_THRESHOLD = 600
export const REFINE_QUIESCENCE_MS = 225

type ActiveStage = 'initial' | 'waiting-to-refine' | 'refining'

interface ActiveRequest {
  request: WorkerLayoutRequest
  large: boolean
  needsRefinement: boolean
  stage: ActiveStage
}

interface StagedRequestHooks {
  send: (request: WorkerRequest) => void
  restartWorker: () => void
  onStarted: (request: WorkerLayoutRequest) => void
  onResult: (response: Exclude<WorkerResponse, { error: string }>) => void
  onError: (error: string) => void
}

export class StagedLayoutRequests {
  private active: ActiveRequest | null = null
  private pending: LayoutSubmission | null = null
  private generation = 0
  private refineTimer: number | null = null

  constructor(
    private readonly hooks: StagedRequestHooks,
    private readonly setTimer: (callback: () => void, delay: number) => number = (callback, delay) =>
      globalThis.setTimeout(callback, delay) as unknown as number,
    private readonly clearTimer: (timer: number) => void = (timer) =>
      globalThis.clearTimeout(timer),
  ) {}

  submit(submission: LayoutSubmission): void {
    if (this.active?.large) {
      this.cancelActiveLargeRequest()
      this.dispatch(submission)
      return
    }
    if (this.active) {
      this.pending = submission
      return
    }
    this.dispatch(submission)
  }

  supersedeActiveLarge(): boolean {
    if (!this.active?.large) return false
    this.cancelActiveLargeRequest()
    return true
  }

  receive(response: WorkerResponse): void {
    const active = this.active
    if (!active || !this.matches(active.request, response)) return

    if ('error' in response) {
      this.finishActive()
      this.hooks.onError(response.error)
      this.dispatchPending()
      return
    }
    if (!this.expectedResponse(active, response)) return

    if (response.phase === 'fast' && !response.final) {
      active.stage = 'waiting-to-refine'
      this.hooks.onResult(response)
      const generation = active.request.generation
      this.refineTimer = this.setTimer(() => this.refine(generation), REFINE_QUIESCENCE_MS)
      return
    }

    this.finishActive()
    this.hooks.onResult(response)
    this.dispatchPending()
  }

  workerFailed(message: string): void {
    if (!this.active) return
    this.finishActive()
    this.hooks.onError(message)
    this.dispatchPending()
  }

  get busy(): boolean {
    return this.active != null
  }

  get hasPending(): boolean {
    return this.pending != null
  }

  get refining(): boolean {
    return this.active?.stage === 'waiting-to-refine' || this.active?.stage === 'refining'
  }

  private dispatch(submission: LayoutSubmission): void {
    const request: WorkerLayoutRequest = {
      ...submission,
      type: 'layout',
      generation: ++this.generation,
    }
    const large = request.graph.nodes.length >= LARGE_LAYOUT_NODE_THRESHOLD
    this.active = {
      request,
      large,
      needsRefinement: large && request.options.quality_effort !== 'fast',
      stage: 'initial',
    }
    this.hooks.onStarted(request)
    this.hooks.send(request)
  }

  private refine(generation: number): void {
    this.refineTimer = null
    const active = this.active
    if (!active || active.request.generation !== generation || active.stage !== 'waiting-to-refine') {
      return
    }
    active.stage = 'refining'
    const { id, datasetId, fixtureName } = active.request
    this.hooks.send({ type: 'refine', generation, id, datasetId, fixtureName })
  }

  private expectedResponse(
    active: ActiveRequest,
    response: Exclude<WorkerResponse, { error: string }>,
  ): boolean {
    if (active.stage === 'refining') return response.phase === 'final' && response.final
    if (active.stage !== 'initial') return false
    if (active.needsRefinement) return response.phase === 'fast' && !response.final
    const expectedPhase = active.large ? 'fast' : 'direct'
    return response.phase === expectedPhase && response.final
  }

  private matches(request: WorkerLayoutRequest, response: WorkerResponse): boolean {
    return (
      response.generation === request.generation &&
      response.id === request.id &&
      response.datasetId === request.datasetId &&
      response.fixtureName === request.fixtureName
    )
  }

  private cancelActiveLargeRequest(): void {
    this.finishActive()
    this.pending = null
    this.hooks.restartWorker()
  }

  private finishActive(): void {
    if (this.refineTimer != null) {
      this.clearTimer(this.refineTimer)
      this.refineTimer = null
    }
    this.active = null
  }

  private dispatchPending(): void {
    if (!this.pending) return
    const pending = this.pending
    this.pending = null
    this.dispatch(pending)
  }
}
