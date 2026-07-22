import { buildGraph } from './graph'
import type { Corpus, ElkRows, Fixture, SchemWeaveGraph } from './types'

export interface PreparedDataset {
  corpus: Corpus
  fixtures: Fixture[]
  elkByName: Map<string, ElkRows['rows'][number]>
  graphByName: Map<string, SchemWeaveGraph>
}

export function prepareDataset(corpus: Corpus, elk: ElkRows): PreparedDataset {
  if (typeof corpus.exactBaseSha !== 'string' || corpus.exactBaseSha.length < 7) {
    throw new Error('corpus is missing its exact SynthExplorer revision')
  }
  const fixturesByName = uniqueByName(corpus.fixtures, 'corpus fixture')
  const elkByName = uniqueByName(elk.rows, 'ELK row')
  if (fixturesByName.size !== elkByName.size) {
    throw new Error(
      `corpus/ELK fixture count mismatch (${fixturesByName.size} vs ${elkByName.size})`,
    )
  }

  const graphByName = new Map<string, SchemWeaveGraph>()
  for (const fixture of corpus.fixtures) {
    const row = elkByName.get(fixture.name)
    if (!row) throw new Error(`ELK data is missing fixture ${fixture.name}`)
    validateFixture(fixture, row.geometry)
    graphByName.set(fixture.name, buildGraph(fixture))
  }
  for (const name of elkByName.keys()) {
    if (!fixturesByName.has(name)) throw new Error(`ELK data has unknown fixture ${name}`)
  }

  return {
    corpus,
    fixtures: corpus.fixtures,
    elkByName,
    graphByName,
  }
}

function uniqueByName<T extends { name: string }>(rows: T[], subject: string): Map<string, T> {
  const byName = new Map<string, T>()
  for (const row of rows) {
    if (!row.name) throw new Error(`${subject} has an empty name`)
    if (byName.has(row.name)) throw new Error(`duplicate ${subject} ${row.name}`)
    byName.set(row.name, row)
  }
  return byName
}

function validateFixture(fixture: Fixture, geometry: ElkRows['rows'][number]['geometry']): void {
  const nodes = fixture.resolvedInput.nodes
  const edges = fixture.resolvedInput.edges
  if (fixture.nodeCount !== nodes.length || fixture.edgeCount !== edges.length) {
    throw new Error(`fixture ${fixture.name} count metadata does not match its resolved input`)
  }
  if (fixture.layoutInput.edges.length !== edges.length) {
    throw new Error(`fixture ${fixture.name} layout and resolved edge counts differ`)
  }
  if (geometry.nodes.length !== nodes.length || geometry.edges.length !== edges.length) {
    throw new Error(`fixture ${fixture.name} ELK geometry cardinality does not match the corpus`)
  }
  finiteNonnegative(geometry.width, `${fixture.name} ELK width`)
  finiteNonnegative(geometry.height, `${fixture.name} ELK height`)

  const expectedNodes = new Map(nodes.map((node) => [node.id, node]))
  const seenNodes = new Set<number>()
  for (const node of geometry.nodes) {
    const expected = expectedNodes.get(node.id)
    if (!expected || !seenNodes.add(node.id)) {
      throw new Error(`fixture ${fixture.name} ELK node identities do not match the corpus`)
    }
    for (const [value, subject] of [
      [node.x, 'x'],
      [node.y, 'y'],
      [node.width, 'width'],
      [node.height, 'height'],
    ] as const) {
      finiteNonnegative(value, `${fixture.name} ELK node ${node.id} ${subject}`)
    }
    if (!near(node.width, expected.width) || !near(node.height, expected.height)) {
      throw new Error(`fixture ${fixture.name} ELK node ${node.id} dimensions differ from the corpus`)
    }
  }

  const expectedEdges = new Set(edges.map((edge) => edge.inputIndex))
  const seenEdges = new Set<number>()
  for (const edge of geometry.edges) {
    if (!expectedEdges.has(edge.inputIndex) || !seenEdges.add(edge.inputIndex)) {
      throw new Error(`fixture ${fixture.name} ELK edge identities do not match the corpus`)
    }
    if (edge.points.length < 2) {
      throw new Error(`fixture ${fixture.name} ELK edge ${edge.inputIndex} has no routed segment`)
    }
    for (const point of edge.points) {
      finiteNonnegative(point.x, `${fixture.name} ELK edge ${edge.inputIndex} x`)
      finiteNonnegative(point.y, `${fixture.name} ELK edge ${edge.inputIndex} y`)
    }
  }
}

function finiteNonnegative(value: number, subject: string): void {
  if (!Number.isFinite(value) || value < 0) throw new Error(`${subject} must be finite and nonnegative`)
}

function near(left: number, right: number): boolean {
  return Math.abs(left - right) <= 1e-7
}
