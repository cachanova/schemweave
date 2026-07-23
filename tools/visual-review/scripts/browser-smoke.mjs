import { chromium } from '@playwright/test'
import { mkdtemp, rm, writeFile } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import { resolve } from 'node:path'
import { spawn } from 'node:child_process'

const nodeCount = 2_000
const fixtureName = 'browser-chain-2000'
const nodes = Array.from({ length: nodeCount }, (_, id) => ({
  id,
  width: 20,
  height: 20,
  register: false,
}))
const edges = Array.from({ length: nodeCount - 1 }, (_, inputIndex) => ({
  inputIndex,
  from: inputIndex,
  to: inputIndex + 1,
  sourceY: 10,
  targetY: 10,
  control: false,
}))
const effortFixtureName = 'quality-effort-600'
const effortNodeCount = 600
const effortNodes = Array.from({ length: effortNodeCount }, (_, id) => ({
  id,
  width: 80,
  height: 50,
  register: false,
}))
const effortEdges = []
const effortInputEdges = []
const addEffortEdge = (from, to, fromPort) => {
  effortEdges.push({
    inputIndex: effortEdges.length,
    from,
    to,
    sourceY: 25,
    targetY: 25,
    control: false,
  })
  effortInputEdges.push({ fromPort, toPort: 'A', control: false })
}
for (let target = 8; target < 24; target += 1) addEffortEdge(0, target, 'shared_100')
let effortState = 81n
const nextEffortState = () => {
  effortState = BigInt.asUintN(
    64,
    effortState * 6_364_136_223_846_793_005n + 1_442_695_040_888_963_407n,
  )
  return effortState
}
for (let source = 1; source < 8; source += 1) {
  for (let target = 8; target < 24; target += 1) {
    if (nextEffortState() % 100n < 24n) {
      addEffortEdge(source, target, `net_${effortEdges.length}`)
    }
  }
}
for (let source = 8; source < 24; source += 1) {
  for (let target = 24; target < 40; target += 1) {
    if (nextEffortState() % 100n < 20n) {
      addEffortEdge(source, target, `net_${effortEdges.length}`)
    }
  }
}
const maxEffortFixtureName = 'max-effort-600'
const maxEffortEdges = []
const maxEffortInputEdges = []
let maxEffortState = 10n
const nextMaxEffortState = () => {
  maxEffortState = BigInt.asUintN(
    64,
    maxEffortState * 6_364_136_223_846_793_005n + 1_442_695_040_888_963_407n,
  )
  return maxEffortState
}
for (let layer = 0; layer < 3; layer += 1) {
  const sourceStart = layer * 50
  const targetStart = (layer + 1) * 50
  for (let from = sourceStart; from < sourceStart + 50; from += 1) {
    for (let to = targetStart; to < targetStart + 50; to += 1) {
      if (nextMaxEffortState() % 100n < 16n) {
        maxEffortEdges.push({
          inputIndex: maxEffortEdges.length,
          from,
          to,
          sourceY: 25,
          targetY: 25,
          control: false,
        })
        maxEffortInputEdges.push({ fromPort: `net_${from}`, toPort: 'A', control: false })
      }
    }
  }
}
const corpus = {
  exactBaseSha: '0123456789abcdef0123456789abcdef01234567',
  fixtures: [
    {
      name: fixtureName,
      kind: 'browser-smoke',
      nodeCount,
      edgeCount: edges.length,
      layoutInput: {
        edges: edges.map(() => ({ fromPort: 'Y', toPort: 'A', control: false })),
      },
      resolvedInput: { nodes, edges },
    },
    {
      name: effortFixtureName,
      kind: 'quality-effort-smoke',
      nodeCount: effortNodeCount,
      edgeCount: effortEdges.length,
      layoutInput: { edges: effortInputEdges },
      resolvedInput: { nodes: effortNodes, edges: effortEdges },
    },
    {
      name: maxEffortFixtureName,
      kind: 'max-effort-smoke',
      nodeCount: effortNodeCount,
      edgeCount: maxEffortEdges.length,
      layoutInput: { edges: maxEffortInputEdges },
      resolvedInput: { nodes: effortNodes, edges: maxEffortEdges },
    },
  ],
}
const elk = {
  rows: [
    {
      name: fixtureName,
      samplesMs: [],
      geometry: {
        nodes: nodes.map((node) => ({
          id: node.id,
          x: node.id * 60,
          y: 0,
          width: node.width,
          height: node.height,
        })),
        edges: edges.map((edge) => ({
          inputIndex: edge.inputIndex,
          points: [
            { x: edge.from * 60 + 20, y: 10 },
            { x: edge.to * 60, y: 10 },
          ],
        })),
        width: (nodeCount - 1) * 60 + 20,
        height: 20,
      },
    },
    {
      name: effortFixtureName,
      samplesMs: [],
      geometry: {
        nodes: effortNodes.map((node) => ({
          id: node.id,
          x:
            node.id < 8
              ? 0
              : node.id < 24
                ? 180
                : node.id < 40
                  ? 360
                  : ((node.id - 40) % 20) * 100,
          y:
            node.id < 40
              ? (node.id % 16) * 70
              : 1_200 + Math.floor((node.id - 40) / 20) * 70,
          width: node.width,
          height: node.height,
        })),
        edges: effortEdges.map((edge) => {
          const sourceX = edge.from < 8 ? 80 : 260
          const sourceY = (edge.from % 16) * 70 + 25
          const targetX = edge.to < 24 ? 180 : 360
          const targetY = (edge.to % 16) * 70 + 25
          const middleX = (sourceX + targetX) / 2
          return {
            inputIndex: edge.inputIndex,
            points: [
              { x: sourceX, y: sourceY },
              { x: middleX, y: sourceY },
              { x: middleX, y: targetY },
              { x: targetX, y: targetY },
            ],
          }
        }),
        width: 1_980,
        height: 3_160,
      },
    },
    {
      name: maxEffortFixtureName,
      samplesMs: [],
      geometry: {
        nodes: effortNodes.map((node) => ({
          id: node.id,
          x: node.id < 200 ? Math.floor(node.id / 50) * 146 : ((node.id - 200) % 20) * 100,
          y: node.id < 200 ? (node.id % 50) * 80 : 4_100 + Math.floor((node.id - 200) / 20) * 80,
          width: node.width,
          height: node.height,
        })),
        edges: maxEffortEdges.map((edge) => {
          const sourceX = Math.floor(edge.from / 50) * 146 + 80
          const sourceY = (edge.from % 50) * 80 + 25
          const targetX = Math.floor(edge.to / 50) * 146
          const targetY = (edge.to % 50) * 80 + 25
          const middleX = (sourceX + targetX) / 2
          return {
            inputIndex: edge.inputIndex,
            points: [
              { x: sourceX, y: sourceY },
              { x: middleX, y: sourceY },
              { x: middleX, y: targetY },
              { x: targetX, y: targetY },
            ],
          }
        }),
        width: 1_980,
        height: 5_700,
      },
    },
  ],
}

const directory = await mkdtemp(resolve(tmpdir(), 'schemweave-browser-smoke-'))
const corpusPath = resolve(directory, 'corpus.json')
const elkPath = resolve(directory, 'elk.json')
await Promise.all([
  writeFile(corpusPath, JSON.stringify(corpus)),
  writeFile(elkPath, JSON.stringify(elk)),
])

const port = 4187
const server = spawn(
  process.execPath,
  [resolve('node_modules/vite/bin/vite.js'), '--host', '127.0.0.1', '--port', String(port), '--strictPort'],
  {
    env: { ...process.env, SCHEMWEAVE_REVIEW_DATA_DIR: directory },
    stdio: ['ignore', 'pipe', 'inherit'],
  },
)
server.stdout.pipe(process.stdout)

let browser
try {
  await waitForServer(`http://127.0.0.1:${port}/`)
  browser = await chromium.launch({ headless: true })
  const page = await browser.newPage({ viewport: { width: 1_440, height: 900 } })
  await page.addInitScript(() => {
    const requests = []
    Object.defineProperty(window, '__schemweaveWorkerRequests', { value: requests })
    const postMessage = Worker.prototype.postMessage
    Worker.prototype.postMessage = function (...args) {
      requests.push(structuredClone(args[0]))
      return postMessage.apply(this, args)
    }
  })
  const errors = []
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text())
  })
  page.on('pageerror', (error) => errors.push(error.message))
  await page.goto(`http://127.0.0.1:${port}/`)
  await waitForCompletedLayout(page, nodeCount, 'Max')

  const initialRequest = await page.evaluate(() =>
    window.__schemweaveWorkerRequests.find((request) => request.type === 'layout'),
  )
  if (
    initialRequest?.options?.quality_effort !== 'max' ||
    initialRequest?.options?.edge_node_clearance !== 20 ||
    initialRequest?.options?.max_quality_area_factor !== 2 ||
    initialRequest?.options?.max_quality_route_length_factor !== 1.25
  ) {
    throw new Error(`highest-quality request mismatch: ${JSON.stringify(initialRequest?.options)}`)
  }
  if (await page.locator('#preset').inputValue() !== 'highest-quality') {
    throw new Error('highest-quality preset was not the initial isolated review profile')
  }
  if (await page.locator('#edge-node-clearance-value').textContent() !== '20 px') {
    throw new Error('wire-to-gate clearance did not display the highest-quality threshold')
  }
  if (!(await page.locator('#status').textContent())?.includes('clearance ≥ 20 px')) {
    throw new Error('completed layout did not report its active clearance threshold')
  }

  const areaBudgetSlider = page.getByRole('slider', { name: 'Maximum quality area factor' })
  const routeBudgetSlider = page.getByRole('slider', {
    name: 'Maximum quality route length factor',
  })
  if ((await areaBudgetSlider.count()) !== 1 || (await routeBudgetSlider.count()) !== 1) {
    throw new Error('quality budget sliders do not have unique accessible names')
  }
  const requestCountBeforeBudgetChange = await workerRequestCount(page)
  await areaBudgetSlider.fill('1.95')
  await routeBudgetSlider.fill('1.24')
  await page.waitForFunction(
    ({ previousCount }) => {
      const requests = window.__schemweaveWorkerRequests
      const latest = requests.at(-1)
      return (
        requests.length > previousCount &&
        latest?.type === 'layout' &&
        latest.options.max_quality_area_factor === 1.95 &&
        latest.options.max_quality_route_length_factor === 1.24
      )
    },
    { previousCount: requestCountBeforeBudgetChange },
  )
  await waitForCompletedLayout(page, nodeCount, 'Max')
  if (await page.locator('#preset').inputValue() !== 'custom') {
    throw new Error('non-preset quality budgets did not select Custom')
  }
  if (
    (await page.locator('#max-quality-area-factor-value').textContent()) !== '1.95×' ||
    (await page.locator('#max-quality-route-length-factor-value').textContent()) !== '1.24×'
  ) {
    throw new Error('quality budget sliders did not retain their newest values')
  }
  await page.locator('#preset').selectOption('highest-quality')
  await waitForCompletedLayout(page, nodeCount, 'Max')

  const clearanceSlider = page.getByRole('slider', { name: 'Wire-to-gate clearance' })
  if ((await clearanceSlider.count()) !== 1) {
    throw new Error('wire-to-gate clearance slider has no unique accessible name')
  }
  const requestCountBeforeClearanceChange = await workerRequestCount(page)
  await clearanceSlider.fill('5')
  await clearanceSlider.fill('15')
  await page.waitForFunction(
    ({ previousCount }) => {
      const requests = window.__schemweaveWorkerRequests
      const latest = requests.at(-1)
      return (
        requests.length > previousCount &&
        latest?.type === 'layout' &&
        latest.options.edge_node_clearance === 15
      )
    },
    { previousCount: requestCountBeforeClearanceChange },
  )
  await waitForCompletedLayout(page, nodeCount, 'Max')
  if (await page.locator('#preset').inputValue() !== 'custom') {
    throw new Error('non-preset clearance did not select Custom')
  }
  if (await page.locator('#edge-node-clearance-value').textContent() !== '15 px') {
    throw new Error('wire-to-gate clearance did not retain the newest slider value')
  }
  if (!(await page.locator('#status').textContent())?.includes('clearance ≥ 15 px')) {
    throw new Error('completed layout did not report the newest clearance threshold')
  }

  const responsivenessStarted = performance.now()
  await page.locator('#preset').selectOption('debug')
  await page.waitForFunction(() => document.querySelector('#layer-gap')?.value === '108')
  const mainThreadResponseMs = performance.now() - responsivenessStarted
  if (mainThreadResponseMs > 1_000) {
    throw new Error(`main-thread control response took ${mainThreadResponseMs.toFixed(1)} ms`)
  }
  await page.locator('#preset').selectOption('compact')
  await page.locator('#preset').selectOption('roomy')
  await page.locator('#preset').selectOption('balanced')
  const stagedStarted = performance.now()
  await page.locator('#fixture').selectOption(effortFixtureName)
  await waitForRefiningLayout(page, effortNodeCount, 'Quality')
  const fastPhaseMs = performance.now() - stagedStarted
  await waitForCompletedLayout(page, effortNodeCount, 'Quality')
  const finalPhaseMs = performance.now() - stagedStarted
  const qualityCrossings = await schemweaveCrossings(page)
  await page.locator('#fixture').selectOption(maxEffortFixtureName)
  await waitForRefiningLayout(page, effortNodeCount, 'Quality')
  await waitForCompletedLayout(page, effortNodeCount, 'Quality')
  const maxFixtureQualityCrossings = await schemweaveCrossings(page)
  await page.locator('#quality-effort').fill('2')
  await waitForRefiningLayout(page, effortNodeCount, 'Max')
  await waitForCompletedLayout(page, effortNodeCount, 'Max')
  const maxCrossings = await schemweaveCrossings(page)
  if (maxCrossings >= maxFixtureQualityCrossings) {
    throw new Error(
      `Max did not change Rust-selected output: ${maxCrossings} >= ${maxFixtureQualityCrossings}`,
    )
  }
  if (await page.locator('#quality-effort-value').textContent() !== 'Max') {
    throw new Error('layout quality slider did not display Max')
  }
  await page.locator('#quality-effort').fill('1')
  await waitForRefiningLayout(page, effortNodeCount, 'Quality')
  await waitForCompletedLayout(page, effortNodeCount, 'Quality')
  await page.locator('#fixture').selectOption(effortFixtureName)
  await waitForRefiningLayout(page, effortNodeCount, 'Quality')
  await waitForCompletedLayout(page, effortNodeCount, 'Quality')
  if ((await schemweaveCrossings(page)) !== qualityCrossings) {
    throw new Error('Quality output changed after exercising Max')
  }
  await page.locator('#quality-effort').fill('0')
  await waitForCompletedLayout(page, effortNodeCount, 'Fast')
  const fastCrossings = await schemweaveCrossings(page)
  if (fastCrossings <= qualityCrossings) {
    throw new Error(`Fast did not expose its quality tradeoff: ${fastCrossings} <= ${qualityCrossings}`)
  }
  const queuedStatus = await page.evaluate(() => {
    const slider = document.querySelector('#quality-effort')
    const status = document.querySelector('#status')
    if (!(slider instanceof HTMLInputElement) || status == null) {
      throw new Error('layout quality controls are unavailable')
    }
    return new Promise((resolve, reject) => {
      const timeout = window.setTimeout(() => {
        observer.disconnect()
        reject(new Error('Fast request did not enter the worker'))
      }, 10_000)
      const observer = new MutationObserver(() => {
        if (!status.textContent?.startsWith('Laying out')) return
        observer.disconnect()
        window.clearTimeout(timeout)
        slider.value = '1'
        slider.dispatchEvent(new Event('input', { bubbles: true }))
        resolve(status.textContent)
      })
      observer.observe(status, { childList: true, characterData: true, subtree: true })
      slider.value = '0'
      slider.dispatchEvent(new Event('input', { bubbles: true }))
    })
  })
  if (!String(queuedStatus).includes('Layout queued')) {
    throw new Error(`Quality request did not supersede active large Fast: ${queuedStatus}`)
  }
  await waitForRefiningLayout(page, effortNodeCount, 'Quality')
  await waitForCompletedLayout(page, effortNodeCount, 'Quality')
  if ((await schemweaveCrossings(page)) !== qualityCrossings) {
    throw new Error('latest Quality request did not replace the queued Fast request')
  }
  await page.locator('#fixture').selectOption(fixtureName)
  await waitForCompletedLayout(page)

  const status = await page.locator('#status').textContent()
  const labels = await page.locator('.metric-label').allTextContents()
  if (await page.locator('#preset').inputValue() !== 'balanced') {
    throw new Error('rapid preset changes did not retain the newest request')
  }
  if (await page.locator('#quality-effort-value').textContent() !== 'Quality') {
    throw new Error('layout quality slider did not retain the newest request')
  }
  if (errors.length > 0) throw new Error(`browser console errors: ${errors.join('; ')}`)
  if (status?.includes('INVALID')) throw new Error(status)
  if (
    !labels.includes('Min track separation') ||
    !labels.includes('Parallel congestion') ||
    !labels.includes('Edge-node clearance') ||
    !labels.includes('Clearance violations (<20 px)') ||
    !labels.includes('Straight routes') ||
    !labels.includes('Contract violations') ||
    labels.length !== 16
  ) {
    throw new Error(`unexpected metric set: ${labels.join(', ')}`)
  }
  const clearanceMetricTitle = await page
    .locator('.metric-label', { hasText: 'Clearance violations (<20 px)' })
    .getAttribute('title')
  if (
    clearanceMetricTitle !==
    'Fixed 20 px comparison threshold; independent of the active layout clearance setting.'
  ) {
    throw new Error(`unexpected clearance metric tooltip: ${clearanceMetricTitle}`)
  }

  const fallback = await browser.newPage({ viewport: { width: 1_200, height: 800 } })
  await fallback.route(/\/review-data\/(?:corpus|elk)\.json$/, (route) =>
    route.fulfill({ status: 404, body: '' }),
  )
  await fallback.goto(`http://127.0.0.1:${port}/`)
  await fallback.waitForFunction(() =>
    document.querySelector('#status')?.textContent?.includes('Select corpus.json'),
  )
  await fallback.locator('#data-files').setInputFiles([corpusPath, elkPath])
  await waitForCompletedLayout(fallback, nodeCount, 'Max')

  process.stdout.write(
    `PASS: ${nodeCount} nodes, 600-node Quality Fast ${fastPhaseMs.toFixed(1)} ms / final ${finalPhaseMs.toFixed(1)} ms, Max ${maxFixtureQualityCrossings}->${maxCrossings} crossings, latest-only presets, ${mainThreadResponseMs.toFixed(1)} ms control response, synchronized worker scoring, file fallback\n`,
  )
} finally {
  await browser?.close()
  server.kill('SIGTERM')
  await rm(directory, { recursive: true, force: true })
}

async function waitForServer(url) {
  const deadline = Date.now() + 10_000
  while (Date.now() < deadline) {
    try {
      const response = await fetch(url)
      if (response.ok) return
    } catch {}
    await new Promise((resolveWait) => setTimeout(resolveWait, 50))
  }
  throw new Error('Vite did not start within 10 seconds')
}

async function waitForCompletedLayout(page, expectedNodeCount = nodeCount, effort = 'Quality') {
  await page.waitForFunction(
    ({ expectedNodes, expectedEffort }) => {
      const value = document.querySelector('#status')?.textContent ?? ''
      return (
        value.includes(`${expectedNodes.toLocaleString()} nodes`) &&
        value.includes(`SchemWeave ${expectedEffort}`) &&
        !value.includes('refining')
      )
    },
    { expectedNodes: expectedNodeCount, expectedEffort: effort },
    { timeout: 90_000 },
  )
}

async function waitForRefiningLayout(page, expectedNodeCount, effort) {
  await page.waitForFunction(
    ({ expectedNodes, expectedEffort }) => {
      const value = document.querySelector('#status')?.textContent ?? ''
      return (
        value.includes(`${expectedNodes.toLocaleString()} nodes`) &&
        value.includes('SchemWeave Fast') &&
        value.includes(`refining to ${expectedEffort}`)
      )
    },
    { expectedNodes: expectedNodeCount, expectedEffort: effort },
    { timeout: 90_000 },
  )
}

async function schemweaveCrossings(page) {
  const value = await page.locator('.metric').first().locator('.schemweave-value').textContent()
  return Number(value?.replaceAll(',', ''))
}

async function workerRequestCount(page) {
  return page.evaluate(() => window.__schemweaveWorkerRequests.length)
}
