import './style.css'

import { CanvasView } from './canvasView'
import { prepareDataset, type PreparedDataset } from './dataset'
import { elkAsLayout } from './graph'
import {
  layoutPresets,
  matchingPreset,
  parallelWireSpacingStatus,
  type PresetName,
} from './layoutControls'
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
const minimumParallelWireSpacing = query<HTMLInputElement>('#minimum-parallel-wire-spacing')
const edgeNodeClearance = query<HTMLInputElement>('#edge-node-clearance')
const maxQualityAreaFactor = query<HTMLInputElement>('#max-quality-area-factor')
const maxQualityRouteLengthFactor = query<HTMLInputElement>('#max-quality-route-length-factor')
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
  const options = layoutOptions()
  const clearance = options.edge_node_clearance
  const clearanceStatus = clearance > 0 ? ` · clearance ≥ ${clearance} px` : ' · clearance off'
  const wireSpacingStatus = ` · ${parallelWireSpacingStatus(options)}`
  status.textContent = `${invalid > 0 ? `INVALID: ELK ${elkInvalid}, SchemWeave ${schemweaveInvalid} · ` : ''}${fixture.nodeCount.toLocaleString()} nodes · ${fixture.edgeCount.toLocaleString()} edges · SchemWeave ${capitalize(response.effort)} ${response.elapsedMs.toFixed(1)} ms${clearanceStatus}${wireSpacingStatus}${refinement}`
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
    minimum_parallel_wire_spacing: Number(minimumParallelWireSpacing.value),
    edge_node_clearance: Number(edgeNodeClearance.value),
    max_quality_area_factor: Number(maxQualityAreaFactor.value),
    max_quality_route_length_factor: Number(maxQualityRouteLengthFactor.value),
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
  const preset = layoutPresets[name]
  layerGap.value = String(preset.layer_gap)
  nodeGap.value = String(preset.node_gap)
  laneGap.value = String(preset.route_lane_gap)
  minimumParallelWireSpacing.value = String(preset.minimum_parallel_wire_spacing)
  edgeNodeClearance.value = String(preset.edge_node_clearance)
  maxQualityAreaFactor.value = String(preset.max_quality_area_factor)
  maxQualityRouteLengthFactor.value = String(preset.max_quality_route_length_factor)
  sweeps.value = String(preset.ordering_sweeps)
  qualityEffort.value = String(['fast', 'quality', 'max'].indexOf(preset.quality_effort))
  updateControlLabels()
  scheduleLayout()
}

function updateControlLabels(): void {
  query<HTMLOutputElement>('#layer-gap-value').value = layerGap.value
  query<HTMLOutputElement>('#node-gap-value').value = nodeGap.value
  query<HTMLOutputElement>('#lane-gap-value').value = laneGap.value
  query<HTMLOutputElement>('#minimum-parallel-wire-spacing-value').value =
    `${minimumParallelWireSpacing.value} px`
  query<HTMLOutputElement>('#edge-node-clearance-value').value =
    `${edgeNodeClearance.value} px`
  query<HTMLOutputElement>('#max-quality-area-factor-value').value =
    `${Number(maxQualityAreaFactor.value).toFixed(2)}×`
  query<HTMLOutputElement>('#max-quality-route-length-factor-value').value =
    `${Number(maxQualityRouteLengthFactor.value).toFixed(2)}×`
  query<HTMLOutputElement>('#sweeps-value').value = sweeps.value
  query<HTMLOutputElement>('#quality-effort-value').value =
    ['Fast', 'Quality', 'Max'][Number(qualityEffort.value)] ?? 'Quality'
  const options = layoutOptions()
  presetSelect.value = matchingPreset(options) ?? 'custom'
}

presetSelect.addEventListener('change', () => {
  if (presetSelect.value !== 'custom') applyPreset(presetSelect.value as PresetName)
})
for (const control of [
  layerGap,
  nodeGap,
  laneGap,
  minimumParallelWireSpacing,
  edgeNodeClearance,
  maxQualityAreaFactor,
  maxQualityRouteLengthFactor,
  sweeps,
  qualityEffort,
]) {
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
  title?: string
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
    {
      label: 'Straight routes',
      elk: (q) => q.straight_route_ratio,
      format: percentage,
      higherIsBetter: true,
    },
    { label: 'Max bends / route', elk: (q) => q.max_bends_per_route, format: integer },
    {
      label: 'Min track separation',
      elk: (q) => q.minimum_parallel_route_separation ?? Number.NaN,
      format: nullableDecimal,
      higherIsBetter: true,
    },
    {
      label: 'Parallel congestion',
      elk: (q) => q.parallel_congestion_ratio,
      format: percentage,
    },
    {
      label: 'Pair-overlap length',
      elk: (q) => q.parallel_pair_overlap_length,
      format: integer,
    },
    {
      label: 'Peak close neighbors',
      elk: (q) => q.peak_parallel_close_neighbors,
      format: integer,
    },
    {
      label: 'Edge-node clearance',
      elk: (q) =>
        q.edge_node_clearance_exhausted
          ? Number.NaN
          : (q.minimum_edge_node_clearance ?? Number.NaN),
      format: nullableDecimal,
      higherIsBetter: true,
    },
    {
      label: 'Clearance violations (<20 px)',
      title: 'Fixed 20 px comparison threshold; independent of the active layout clearance setting.',
      elk: (q) =>
        q.edge_node_clearance_exhausted ? Number.NaN : q.edge_node_clearance_violations,
      format: nullableInteger,
    },
    {
      label: 'Max crossing knot',
      elk: (q) => q.max_crossings_on_segment,
      format: integer,
    },
    { label: 'Perimeter routing', elk: (q) => q.perimeter_route_ratio, format: percentage },
    { label: 'Viewport fit', elk: (q) => q.viewport_fit, format: decimal },
    { label: 'Contract violations', elk: contractViolations, format: integer },
  ]

  metrics.replaceChildren(
    ...definitions.map((definition) => {
      const elkValue = definition.elk(elkQuality)
      const schemweaveValue = definition.elk(schemweaveQuality)
      const delta = relativeDelta(elkValue, schemweaveValue)
      const comparable = !Number.isNaN(elkValue) && !Number.isNaN(schemweaveValue)
      const better = definition.higherIsBetter
        ? schemweaveValue > elkValue
        : schemweaveValue < elkValue
      const card = document.createElement('div')
      card.className = 'metric'
      card.innerHTML = `
        <div class="metric-label" title="${definition.title ?? definition.label}">${definition.label}</div>
        <div class="metric-values">
          <span class="elk-value">${definition.format(elkValue, elkQuality)}</span>
          <span class="schemweave-value">${definition.format(schemweaveValue, schemweaveQuality)}</span>
        </div>
        <div class="metric-delta ${!comparable || delta === 0 ? '' : better ? 'better' : 'worse'}">${formatDelta(delta, elkValue, schemweaveValue)}</div>
      `
      return card
    }),
  )
}

function metricPlaceholders(): string {
  return Array.from(
    { length: 16 },
    () => '<div class="metric"><div class="metric-label">computing</div><div class="metric-values"><span>—</span><span>—</span></div></div>',
  ).join('')
}

function relativeDelta(reference: number, candidate: number): number {
  if (reference === candidate) return 0
  if (reference === 0) return candidate > 0 ? Number.POSITIVE_INFINITY : Number.NEGATIVE_INFINITY
  return ((candidate - reference) / Math.abs(reference)) * 100
}

function formatDelta(delta: number, reference: number, candidate: number): string {
  if (Number.isNaN(reference) || Number.isNaN(candidate)) return 'n/a'
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

function nullableDecimal(value: number): string {
  return Number.isNaN(value) ? '—' : decimal(value)
}

function nullableInteger(value: number): string {
  return Number.isNaN(value) ? '—' : integer(value)
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
