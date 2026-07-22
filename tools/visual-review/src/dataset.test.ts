import { describe, expect, it } from 'vitest'

import { prepareDataset } from './dataset'
import type { Corpus, ElkRows } from './types'

function artifacts(): { corpus: Corpus; elk: ElkRows } {
  return {
    corpus: {
      exactBaseSha: '1234567890abcdef',
      fixtures: [
        {
          name: 'one-edge',
          kind: 'test',
          nodeCount: 2,
          edgeCount: 1,
          layoutInput: {
            edges: [{ fromPort: 'Y', toPort: 'A', control: false }],
          },
          resolvedInput: {
            nodes: [
              { id: 1, width: 20, height: 20, register: false },
              { id: 2, width: 20, height: 20, register: false },
            ],
            edges: [
              {
                inputIndex: 0,
                from: 1,
                to: 2,
                sourceY: 10,
                targetY: 10,
                control: false,
              },
            ],
          },
        },
      ],
    },
    elk: {
      rows: [
        {
          name: 'one-edge',
          samplesMs: [],
          geometry: {
            nodes: [
              { id: 1, x: 0, y: 0, width: 20, height: 20 },
              { id: 2, x: 80, y: 0, width: 20, height: 20 },
            ],
            edges: [
              {
                inputIndex: 0,
                points: [
                  { x: 20, y: 10 },
                  { x: 80, y: 10 },
                ],
              },
            ],
            width: 100,
            height: 20,
          },
        },
      ],
    },
  }
}

describe('prepareDataset', () => {
  it('validates and prepares exact graph and ELK identities together', () => {
    const { corpus, elk } = artifacts()
    const prepared = prepareDataset(corpus, elk)
    expect(prepared.fixtures.map((fixture) => fixture.name)).toEqual(['one-edge'])
    expect(prepared.graphByName.get('one-edge')?.edges[0]).toMatchObject({ id: 0, net: 0 })
    expect(prepared.elkByName.get('one-edge')?.geometry.edges[0].inputIndex).toBe(0)
  })

  it('rejects same-name ELK geometry from a structurally different corpus', () => {
    const { corpus, elk } = artifacts()
    elk.rows[0].geometry.nodes[1].width = 21
    expect(() => prepareDataset(corpus, elk)).toThrow('dimensions differ from the corpus')
  })

  it('rejects incomplete joins instead of silently dropping fixtures', () => {
    const { corpus, elk } = artifacts()
    elk.rows = []
    expect(() => prepareDataset(corpus, elk)).toThrow('fixture count mismatch')
  })

  it('rejects malformed graphs before a worker request can wedge scheduling', () => {
    const { corpus, elk } = artifacts()
    corpus.fixtures[0].resolvedInput.edges[0].to = 99
    expect(() => prepareDataset(corpus, elk)).toThrow('unknown node 99')
  })
})
