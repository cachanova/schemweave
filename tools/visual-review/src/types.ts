export type PortSide = 'north' | 'east' | 'south' | 'west'

export interface Point {
  x: number
  y: number
}

export interface SchemWeaveGraph {
  nodes: Array<{
    id: number
    width: number
    height: number
    cycle_breaker: boolean
    ports: Array<{ id: number; side: PortSide; offset: number }>
  }>
  edges: Array<{
    id: number
    source: { node: number; port: number }
    target: { node: number; port: number }
    net: number
    participates_in_ranking: boolean
  }>
}

export interface LayoutOptions {
  layer_gap: number
  node_gap: number
  port_stub: number
  route_lane_gap: number
  ordering_sweeps: number
  quality_effort: 'fast' | 'quality' | 'max'
}

export interface Layout {
  nodes: Array<{
    id: number
    x: number
    y: number
    width: number
    height: number
  }>
  edges: Array<{ id: number; points: Point[] }>
  width: number
  height: number
}

export interface QualityReport {
  semantic_violations: number
  node_overlaps: number
  node_intersections: number
  unrelated_overlaps: number
  unrelated_contacts: number
  crossings: number
  ranking_direction_violations: number
  forward_edge_count: number
  reverse_x_length: number
  p95_forward_stretch: number
  forward_routes_with_reverse_x: number
  split_feedback_nets: number
  feedback_net_count: number
  bends: number
  segments: number
  route_length: number
  shared_route_ratio: number
  area: number
  viewport_fit: number
}

export interface Corpus {
  exactBaseSha: string
  fixtures: Fixture[]
}

export interface Fixture {
  name: string
  kind: string
  nodeCount: number
  edgeCount: number
  subgraph?: {
    nodes: Array<{ id: number; name: string; cell_type?: string; kind?: string }>
  }
  layoutInput: {
    edges: Array<{ fromPort: string; toPort: string; control: boolean }>
  }
  resolvedInput: {
    nodes: Array<{
      id: number
      width: number
      height: number
      register: boolean
    }>
    edges: Array<{
      inputIndex: number
      from: number
      to: number
      sourceY: number
      targetY: number
      control: boolean
    }>
  }
}

export interface ElkRows {
  rows: Array<{
    name: string
    samplesMs: number[]
    geometry: ElkGeometry
  }>
}

export interface ElkGeometry {
  nodes: Layout['nodes']
  edges: Array<{ inputIndex: number; points: Point[] }>
  width: number
  height: number
}

export type LayoutPhase = 'direct' | 'fast' | 'final'

export interface WorkerLayoutRequest {
  type: 'layout'
  generation: number
  id: number
  datasetId: number
  fixtureName: string
  graph: SchemWeaveGraph
  elk: ElkGeometry
  options: LayoutOptions
}

export interface WorkerRefineRequest {
  type: 'refine'
  generation: number
  id: number
  datasetId: number
  fixtureName: string
}

export type WorkerRequest = WorkerLayoutRequest | WorkerRefineRequest
export type LayoutSubmission = Omit<WorkerLayoutRequest, 'type' | 'generation'>

export type WorkerResponse =
  | {
      generation: number
      id: number
      datasetId: number
      fixtureName: string
      phase: LayoutPhase
      effort: LayoutOptions['quality_effort']
      requestedEffort: LayoutOptions['quality_effort']
      final: boolean
      elapsedMs: number
      layout: Layout
      quality: QualityReport
      elkQuality: QualityReport
    }
  | {
      generation: number
      id: number
      datasetId: number
      fixtureName: string
      phase: LayoutPhase
      error: string
    }
