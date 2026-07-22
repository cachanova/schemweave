import { buildGraph, elkAsLayout } from './graph'
import type { Fixture } from './types'
import { describe, expect, it } from 'vitest'

const fixture: Fixture = {
  name: 'ports-and-nets',
  kind: 'test',
  nodeCount: 3,
  edgeCount: 3,
  layoutInput: {
    edges: [
      { fromPort: 'Y', toPort: 'A', control: false },
      { fromPort: 'Y', toPort: 'B', control: false },
      { fromPort: 'Q', toPort: 'A', control: false },
    ],
  },
  resolvedInput: {
    nodes: [
      { id: 5, width: 20, height: 30, register: true },
      { id: 7, width: 20, height: 30, register: false },
      { id: 9, width: 20, height: 30, register: false },
    ],
    edges: [
      { inputIndex: 0, from: 5, to: 7, sourceY: 8, targetY: 6, control: false },
      { inputIndex: 1, from: 5, to: 9, sourceY: 8, targetY: 12, control: false },
      { inputIndex: 2, from: 7, to: 9, sourceY: 10, targetY: 18, control: false },
    ],
  },
}

describe('buildGraph', () => {
  it('shares electrical nets by source port and assigns stable boundary ports', () => {
    const graph = buildGraph(fixture)

    expect(graph.nodes[0]).toMatchObject({ id: 5, cycle_breaker: true })
    expect(graph.nodes[0].ports).toEqual([{ id: 0, side: 'east', offset: 8 }])
    expect(graph.nodes[2].ports).toEqual([
      { id: 0, side: 'west', offset: 12 },
      { id: 1, side: 'west', offset: 18 },
    ])
    expect(graph.edges.map((edge) => edge.net)).toEqual([0, 0, 1])
    expect(graph.edges[2].source).toEqual({ node: 7, port: 0 })
    expect(graph.edges[2].target).toEqual({ node: 9, port: 1 })
  })

  it('rejects corpus edges that reference missing nodes', () => {
    const invalid = structuredClone(fixture)
    invalid.resolvedInput.edges[0].to = 99
    expect(() => buildGraph(invalid)).toThrow('unknown node 99')
  })
})

it('maps ELK input indexes to SchemWeave edge ids for canonical scoring', () => {
  expect(
    elkAsLayout({
      nodes: [],
      edges: [{ inputIndex: 7, points: [{ x: 1, y: 2 }] }],
      width: 3,
      height: 4,
    }),
  ).toEqual({
    nodes: [],
    edges: [{ id: 7, points: [{ x: 1, y: 2 }] }],
    width: 3,
    height: 4,
  })
})
