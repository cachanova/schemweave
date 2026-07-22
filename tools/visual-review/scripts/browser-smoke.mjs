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
  const errors = []
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text())
  })
  page.on('pageerror', (error) => errors.push(error.message))
  await page.goto(`http://127.0.0.1:${port}/`)
  await waitForCompletedLayout(page)

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
  await waitForCompletedLayout(page)

  const status = await page.locator('#status').textContent()
  const labels = await page.locator('.metric-label').allTextContents()
  if (await page.locator('#preset').inputValue() !== 'balanced') {
    throw new Error('rapid preset changes did not retain the newest request')
  }
  if (errors.length > 0) throw new Error(`browser console errors: ${errors.join('; ')}`)
  if (status?.includes('INVALID')) throw new Error(status)
  if (!labels.includes('Contract violations') || labels.length !== 8) {
    throw new Error(`unexpected metric set: ${labels.join(', ')}`)
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
  await waitForCompletedLayout(fallback)

  process.stdout.write(
    `PASS: ${nodeCount} nodes, latest-only presets, ${mainThreadResponseMs.toFixed(1)} ms control response, synchronized worker scoring, file fallback\n`,
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

async function waitForCompletedLayout(page) {
  await page.waitForFunction(
    (expectedNodes) => {
      const value = document.querySelector('#status')?.textContent ?? ''
      return value.includes(`${expectedNodes.toLocaleString()} nodes`) && value.includes('SchemWeave')
    },
    nodeCount,
    { timeout: 90_000 },
  )
}
