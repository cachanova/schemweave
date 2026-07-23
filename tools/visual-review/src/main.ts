import './style.css'

import { CanvasView } from './canvasView'
import { prepareDataset, type PreparedDataset } from './dataset'
import { elkAsLayout } from './graph'
import { StagedLayoutRequests } from './latestRequest'
import type {
  Corpus,
  ElkRows,
  Fixture,
  LayoutSubmission,
  LayoutOptions,
  QualityReport,
  WorkerResponse,
} from './types'

const presets = {
  compact: { layer_gap: 48, node_gap: 20, route_lane_gap: 3, ordering_sweeps: 2 },
  balanced: { layer_gap: 66, node_gap: 30, route_lane_gap: 4, ordering_sweeps: 4 },
  roomy: { layer_gap: 84, node_gap: 45, route_lane_gap: 6, ordering_sweeps: 4 },
  debug: { layer_gap: 108, node_gap: 60, route_lane_gap: 8, ordering_sweeps: 8 },
} as const

type PresetName = keyof typeof presets

function query<T extends Element>(selector: string): T {
  const value = document.querySelector<T>(selector)
  if (!value) throw new Error(`missing element ${selector}`)
  return value
}

const fixtureSelect = query<HTMLSelectElement>('#fixture')
const presetSelect = query<HTMLSelectElement>('#preset')
const layerGap = query<HTMLInputElement>('#layer-gap')
const nodeGap = query<HTMLInputElement>('#node-gap')
const laneGap = query<HTMLInputElement>('#lane-gap')
const sweeps = query<HTMLInputElement>('#sweeps')
const qualityEffort = query<HTMLInputElement>('#quality-effort')
const status = query<HTMLElement>('#status')
const corpusRevision = query<HTMLElement>('#corpus-revision')
const metrics = query<HTMLElement>('#metrics')
const syncView = query<HTMLInputElement>('#sync')
const elkSize = query<HTMLElement>('#elk-size')
const schemweaveSize = query<HTMLElement>('#schemweave-size')

const elkView = new CanvasView(query<HTMLCanvasElement>('#elk-canvas'))
const schemweaveView = new CanvasView(query<HTMLCanvasElement>('#schemweave-canvas'))
elkView.onViewChanged = () => {
  const view = elkView.normalizedView()
  if (syncView.checked && view) schemweaveView.applyNormalizedView(view)
}
schemweaveView.onViewChanged = () => {
  const view = schemweaveView.normalizedView()
  if (syncView.checked && view) elkView.applyNormalizedView(view)
}

let dataset: PreparedDataset | null = null
let datasetId = 0
let currentIndex = 0
let layoutDirty = false
let requestSequence = 0
let debounce: number | null = null
let lastFixtureName: string | null = null

let workerEpoch = 0
let worker = createWorker()
const requests = new StagedLayoutRequests({
  send: (request) => worker.postMessage(request),
  restartWorker: () => {
    worker.terminate()
    worker = createWorker()
  },
  onStarted: (request) => {
    status.textContent = `Laying out ${request.fixtureName}…`
  },
  onResult: displayLayout,
  onError: (error) => {
    if (!layoutDirty) status.textContent = `Layout failed: ${error}`
  },
})

function createWorker(): Worker {
  const epoch = ++workerEpoch
  const nextWorker = new Worker(new URL('./layout.worker.ts', import.meta.url), { type: 'module' })
  nextWorker.addEventListener('message', (event: MessageEvent<WorkerResponse>) => {
    if (epoch === workerEpoch) requests.receive(event.data)
  })
  nextWorker.addEventListener('error', (event) => {
    if (epoch === workerEpoch) requests.workerFailed(event.message)
  })
  return nextWorker
}

function displayLayout(response: Exclude<WorkerResponse, { error: string }>): void {
  const superseded =
    layoutDirty ||
    requests.hasPending ||
    response.datasetId !== datasetId ||
    response.fixtureName !== currentFixture()?.name
  if (superseded) return
  const fixture = currentFixture()
  const elk = currentElk()
  if (!fixture || !elk) return
  const retainedView = lastFixtureName === fixture.name ? schemweaveView.normalizedView() : null
  schemweaveView.setGeometry(response.layout, fixture)
  if (retainedView) {
    schemweaveView.applyNormalizedView(retainedView)
    if (syncView.checked) elkView.applyNormalizedView(retainedView)
  }
  schemweaveSize.textContent = dimensions(response.layout.width, response.layout.height)
  updateMetrics(response.elkQuality, response.quality)
  const elkInvalid = contractViolations(response.elkQuality)
  const schemweaveInvalid = contractViolations(response.quality)
  const invalid = elkInvalid + schemweaveInvalid
  const refinement = response.final
    ? ''
    : ` · refining to ${capitalize(response.requestedEffort)}…`
  status.textContent = `${invalid > 0 ? `INVALID: ELK ${elkInvalid}, SchemWeave ${schemweaveInvalid} · ` : ''}${fixture.nodeCount.toLocaleString()} nodes · ${fixture.edgeCount.toLocaleString()} edges · SchemWeave ${capitalize(response.effort)} ${response.elapsedMs.toFixed(1)} ms${refinement}`
  lastFixtureName = fixture.name
}

function currentFixture(): Fixture | null {
  return dataset?.fixtures[currentIndex] ?? null
}

function currentElk(): ElkRows['rows'][number] | null {
  const name = currentFixture()?.name
  return name ? (dataset?.elkByName.get(name) ?? null) : null
}

function layoutOptions(): LayoutOptions {
  const effort = ['fast', 'quality', 'max'] as const
  return {
    layer_gap: Number(layerGap.value),
    node_gap: Number(nodeGap.value),
    port_stub: 10,
    route_lane_gap: Number(laneGap.value),
    ordering_sweeps: Number(sweeps.value),
    quality_effort: effort[Number(qualityEffort.value)] ?? 'quality',
  }
}

function scheduleLayout(delay = 100): void {
  layoutDirty = true
  requests.supersedeActiveLarge()
  status.textContent = requests.busy
    ? 'Finishing current layout; newest settings queued…'
    : 'Layout queued…'
  if (debounce != null) window.clearTimeout(debounce)
  debounce = window.setTimeout(() => {
    debounce = null
    dispatchLayout()
  }, delay)
}

function dispatchLayout(): void {
  if (!layoutDirty) return
  const fixture = currentFixture()
  const elk = currentElk()
  const graph = fixture ? dataset?.graphByName.get(fixture.name) : null
  if (!fixture || !elk || !graph) return
  layoutDirty = false
  const request: LayoutSubmission = {
    id: ++requestSequence,
    datasetId,
    fixtureName: fixture.name,
    graph,
    elk: elk.geometry,
    options: layoutOptions(),
  }
  requests.submit(request)
}

function showFixture(index: number): void {
  if (!dataset || dataset.fixtures.length === 0) return
  currentIndex = (index + dataset.fixtures.length) % dataset.fixtures.length
  const fixture = currentFixture()
  const elk = currentElk()
  if (!fixture || !elk) return
  fixtureSelect.value = fixture.name
  history.replaceState(null, '', `#${encodeURIComponent(fixture.name)}`)
  elkView.setGeometry(elkAsLayout(elk.geometry), fixture)
  schemweaveView.clear()
  elkSize.textContent = dimensions(elk.geometry.width, elk.geometry.height)
  schemweaveSize.textContent = 'computing…'
  metrics.innerHTML = metricPlaceholders()
  lastFixtureName = null
  scheduleLayout(0)
}

function initializeData(nextCorpus: Corpus, nextElk: ElkRows): void {
  const prepared = prepareDataset(nextCorpus, nextElk)
  dataset = prepared
  datasetId += 1
  fixtureSelect.replaceChildren(
    ...prepared.fixtures.map(
      (fixture) => new Option(`${fixture.name} (${fixture.nodeCount})`, fixture.name),
    ),
  )
  corpusRevision.textContent = `SynthExplorer corpus ${nextCorpus.exactBaseSha.slice(0, 10)}`
  const requested = decodeURIComponent(location.hash.slice(1))
  const requestedIndex = prepared.fixtures.findIndex((fixture) => fixture.name === requested)
  showFixture(requestedIndex >= 0 ? requestedIndex : 0)
}

async function loadDefaultData(): Promise<void> {
  try {
    const [corpusResponse, elkResponse] = await Promise.all([
      fetch('/review-data/corpus.json'),
      fetch('/review-data/elk.json'),
    ])
    if (!corpusResponse.ok || !elkResponse.ok) throw new Error('review data endpoint unavailable')
    initializeData((await corpusResponse.json()) as Corpus, (await elkResponse.json()) as ElkRows)
  } catch (error) {
    status.textContent =
      error instanceof Error && error.message !== 'review data endpoint unavailable'
        ? error.message
        : 'Select corpus.json and elk.json with “Load data”'
  }
}

query<HTMLInputElement>('#data-files').addEventListener('change', async (event) => {
  const files = [...(event.currentTarget as HTMLInputElement).files ?? []]
  try {
    const corpusFile = files.find((file) => file.name === 'corpus.json')
    const elkFile = files.find((file) => file.name === 'elk.json')
    if (!corpusFile || !elkFile) throw new Error('choose both corpus.json and elk.json')
    initializeData(
      JSON.parse(await corpusFile.text()) as Corpus,
      JSON.parse(await elkFile.text()) as ElkRows,
    )
  } catch (error) {
    status.textContent = error instanceof Error ? error.message : String(error)
  }
})

function applyPreset(name: PresetName): void {
  const preset = presets[name]
  layerGap.value = String(preset.layer_gap)
  nodeGap.value = String(preset.node_gap)
  laneGap.value = String(preset.route_lane_gap)
  sweeps.value = String(preset.ordering_sweeps)
  updateControlLabels()
  scheduleLayout()
}

function updateControlLabels(): void {
  query<HTMLOutputElement>('#layer-gap-value').value = layerGap.value
  query<HTMLOutputElement>('#node-gap-value').value = nodeGap.value
  query<HTMLOutputElement>('#lane-gap-value').value = laneGap.value
  query<HTMLOutputElement>('#sweeps-value').value = sweeps.value
  query<HTMLOutputElement>('#quality-effort-value').value =
    ['Fast', 'Quality', 'Max'][Number(qualityEffort.value)] ?? 'Quality'
  const options = layoutOptions()
  const matching = Object.entries(presets).find(
    ([, preset]) =>
      preset.layer_gap === options.layer_gap &&
      preset.node_gap === options.node_gap &&
      preset.route_lane_gap === options.route_lane_gap &&
      preset.ordering_sweeps === options.ordering_sweeps,
  )
  presetSelect.value = matching?.[0] ?? 'custom'
}

presetSelect.addEventListener('change', () => {
  if (presetSelect.value !== 'custom') applyPreset(presetSelect.value as PresetName)
})
for (const control of [layerGap, nodeGap, laneGap, sweeps, qualityEffort]) {
  control.addEventListener('input', () => {
    updateControlLabels()
    scheduleLayout()
  })
}
fixtureSelect.addEventListener('change', () => {
  showFixture(
    dataset?.fixtures.findIndex((fixture) => fixture.name === fixtureSelect.value) ?? -1,
  )
})
query<HTMLButtonElement>('#previous').addEventListener('click', () => showFixture(currentIndex - 1))
query<HTMLButtonElement>('#next').addEventListener('click', () => showFixture(currentIndex + 1))
query<HTMLButtonElement>('#fit').addEventListener('click', () => {
  elkView.fit()
  schemweaveView.fit()
})
window.addEventListener('keydown', (event) => {
  if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) return
  if (event.key === 'ArrowLeft') showFixture(currentIndex - 1)
  if (event.key === 'ArrowRight') showFixture(currentIndex + 1)
})
window.addEventListener('resize', () => {
  elkView.fit()
  schemweaveView.fit()
})

function dimensions(width: number, height: number): string {
  return `${Math.round(width).toLocaleString()} × ${Math.round(height).toLocaleString()}`
}

interface MetricDefinition {
  label: string
  elk: (quality: QualityReport) => number
  format: (value: number, quality: QualityReport) => string
  higherIsBetter?: boolean
}

function updateMetrics(elkQuality: QualityReport, schemweaveQuality: QualityReport): void {
  const definitions: MetricDefinition[] = [
    { label: 'Crossings', elk: (q) => q.crossings, format: integer },
    {
      label: 'Direction violations',
      elk: (q) => q.ranking_direction_violations,
      format: (value, q) => `${integer(value)} / ${integer(q.forward_edge_count)}`,
    },
    { label: 'Reverse X', elk: (q) => q.reverse_x_length, format: integer },
    { label: 'p95 route stretch', elk: (q) => q.p95_forward_stretch, format: decimal },
    {
      label: 'Split feedback',
      elk: (q) => q.split_feedback_nets,
      format: (value, q) => `${integer(value)} / ${integer(q.feedback_net_count)}`,
    },
    {
      label: 'Shared routing',
      elk: (q) => q.shared_route_ratio,
      format: percentage,
      higherIsBetter: true,
    },
    { label: 'Viewport fit', elk: (q) => q.viewport_fit, format: decimal },
    { label: 'Contract violations', elk: contractViolations, format: integer },
  ]

  metrics.replaceChildren(
    ...definitions.map((definition) => {
      const elkValue = definition.elk(elkQuality)
      const schemweaveValue = definition.elk(schemweaveQuality)
      const delta = relativeDelta(elkValue, schemweaveValue)
      const better = definition.higherIsBetter
        ? schemweaveValue > elkValue
        : schemweaveValue < elkValue
      const card = document.createElement('div')
      card.className = 'metric'
      card.innerHTML = `
        <div class="metric-label" title="${definition.label}">${definition.label}</div>
        <div class="metric-values">
          <span class="elk-value">${definition.format(elkValue, elkQuality)}</span>
          <span class="schemweave-value">${definition.format(schemweaveValue, schemweaveQuality)}</span>
        </div>
        <div class="metric-delta ${delta === 0 ? '' : better ? 'better' : 'worse'}">${formatDelta(delta, elkValue, schemweaveValue)}</div>
      `
      return card
    }),
  )
}

function metricPlaceholders(): string {
  return Array.from(
    { length: 8 },
    () => '<div class="metric"><div class="metric-label">computing</div><div class="metric-values"><span>—</span><span>—</span></div></div>',
  ).join('')
}

function relativeDelta(reference: number, candidate: number): number {
  if (reference === candidate) return 0
  if (reference === 0) return candidate > 0 ? Number.POSITIVE_INFINITY : Number.NEGATIVE_INFINITY
  return ((candidate - reference) / Math.abs(reference)) * 100
}

function formatDelta(delta: number, reference: number, candidate: number): string {
  if (delta === 0) return 'equal'
  if (!Number.isFinite(delta)) return candidate > reference ? 'ELK 0' : 'SchemWeave 0'
  return `${delta > 0 ? '+' : ''}${delta.toFixed(Math.abs(delta) >= 10 ? 0 : 1)}%`
}

function integer(value: number): string {
  return Math.round(value).toLocaleString()
}

function decimal(value: number): string {
  return value.toFixed(3)
}

function percentage(value: number): string {
  return `${(value * 100).toFixed(1)}%`
}

function capitalize(value: string): string {
  return value.charAt(0).toUpperCase() + value.slice(1)
}

function contractViolations(quality: QualityReport): number {
  return (
    quality.semantic_violations +
    quality.node_overlaps +
    quality.node_intersections +
    quality.unrelated_overlaps +
    quality.unrelated_contacts
  )
}

updateControlLabels()
metrics.innerHTML = metricPlaceholders()
void loadDefaultData()
