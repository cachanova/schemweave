import type { ElkGeometry, Fixture, Layout, SchemWeaveGraph } from './types'

interface PortRecord {
  id: number
  side: 'east' | 'west'
  offset: number
}

function portKey(side: 'east' | 'west', offset: number): string {
  return `${side}:${offset}`
}

export function buildGraph(fixture: Fixture): SchemWeaveGraph {
  const nodeIds = new Set(fixture.resolvedInput.nodes.map((node) => node.id))
  const portsByNode = new Map<number, Map<string, PortRecord>>()
  const ensurePort = (node: number, side: 'east' | 'west', offset: number) => {
    if (!nodeIds.has(node)) throw new Error(`edge references unknown node ${node}`)
    let ports = portsByNode.get(node)
    if (!ports) {
      ports = new Map()
      portsByNode.set(node, ports)
    }
    const key = portKey(side, offset)
    if (!ports.has(key)) ports.set(key, { id: -1, side, offset })
  }

  for (const edge of fixture.resolvedInput.edges) {
    ensurePort(edge.from, 'east', edge.sourceY)
    ensurePort(edge.to, 'west', edge.targetY)
  }

  for (const ports of portsByNode.values()) {
    const ordered = [...ports.values()].sort(
      (left, right) =>
        (left.side === right.side ? 0 : left.side === 'east' ? -1 : 1) ||
        left.offset - right.offset,
    )
    ordered.forEach((port, id) => {
      port.id = id
    })
  }

  const netKeys = new Set<string>()
  for (const edge of fixture.resolvedInput.edges) {
    const identity = fixture.layoutInput.edges[edge.inputIndex]
    if (!identity) throw new Error(`missing layout input edge ${edge.inputIndex}`)
    netKeys.add(`${edge.from}\u0000${identity.fromPort}`)
  }
  const orderedNetKeys = [...netKeys].sort((left, right) => {
    const [leftNode, leftPort] = left.split('\u0000')
    const [rightNode, rightPort] = right.split('\u0000')
    return (
      Number(leftNode) - Number(rightNode) ||
      (leftPort < rightPort ? -1 : leftPort > rightPort ? 1 : 0)
    )
  })
  const netByKey = new Map(orderedNetKeys.map((key, id) => [key, id]))
  const portId = (node: number, side: 'east' | 'west', offset: number) => {
    const port = portsByNode.get(node)?.get(portKey(side, offset))
    if (!port || port.id < 0) throw new Error(`missing ${side} port ${node}:${offset}`)
    return port.id
  }

  return {
    nodes: fixture.resolvedInput.nodes.map((node) => ({
      id: node.id,
      width: node.width,
      height: node.height,
      cycle_breaker: node.register,
      ports: [...(portsByNode.get(node.id)?.values() ?? [])]
        .sort((left, right) => left.id - right.id)
        .map(({ id, side, offset }) => ({ id, side, offset })),
    })),
    edges: fixture.resolvedInput.edges.map((edge) => {
      const identity = fixture.layoutInput.edges[edge.inputIndex]
      const net = netByKey.get(`${edge.from}\u0000${identity.fromPort}`)
      if (net == null) throw new Error(`missing net for edge ${edge.inputIndex}`)
      return {
        id: edge.inputIndex,
        source: {
          node: edge.from,
          port: portId(edge.from, 'east', edge.sourceY),
        },
        target: {
          node: edge.to,
          port: portId(edge.to, 'west', edge.targetY),
        },
        net,
        participates_in_ranking: true,
      }
    }),
  }
}

export function elkAsLayout(geometry: ElkGeometry): Layout {
  return {
    nodes: geometry.nodes,
    edges: geometry.edges.map((edge) => ({ id: edge.inputIndex, points: edge.points })),
    width: geometry.width,
    height: geometry.height,
  }
}
