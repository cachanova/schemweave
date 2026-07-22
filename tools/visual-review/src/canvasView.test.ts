import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'

import { CanvasView } from './canvasView'
import type { Fixture, Layout } from './types'

class FakeCanvas {
  width = 800
  height = 600
  readonly classList = { add: vi.fn(), remove: vi.fn() }
  readonly clearRect = vi.fn()
  private readonly listeners = new Map<string, EventListener[]>()
  private readonly context = {
    clearRect: this.clearRect,
    save: vi.fn(),
    restore: vi.fn(),
    translate: vi.fn(),
    scale: vi.fn(),
    beginPath: vi.fn(),
    moveTo: vi.fn(),
    lineTo: vi.fn(),
    stroke: vi.fn(),
    fillRect: vi.fn(),
    strokeRect: vi.fn(),
    rect: vi.fn(),
    clip: vi.fn(),
    fillText: vi.fn(),
  }

  addEventListener(type: string, listener: EventListener): void {
    const listeners = this.listeners.get(type) ?? []
    listeners.push(listener)
    this.listeners.set(type, listeners)
  }

  dispatch(type: string, event: object): void {
    for (const listener of this.listeners.get(type) ?? []) listener(event as Event)
  }

  getBoundingClientRect(): DOMRect {
    return { left: 0, top: 0, width: 800, height: 600 } as DOMRect
  }

  getContext(): CanvasRenderingContext2D {
    return this.context as unknown as CanvasRenderingContext2D
  }

  setPointerCapture(): void {}
}

const fixture: Fixture = {
  name: 'view',
  kind: 'test',
  nodeCount: 1,
  edgeCount: 0,
  layoutInput: { edges: [] },
  resolvedInput: {
    nodes: [{ id: 1, width: 20, height: 20, register: false }],
    edges: [],
  },
}

const geometry: Layout = {
  nodes: [{ id: 1, x: 40, y: 30, width: 20, height: 20 }],
  edges: [],
  width: 100,
  height: 80,
}

let frames: FrameRequestCallback[]

beforeEach(() => {
  frames = []
  vi.stubGlobal('devicePixelRatio', 1)
  vi.stubGlobal('requestAnimationFrame', (callback: FrameRequestCallback) => {
    frames.push(callback)
    return frames.length
  })
})

afterEach(() => vi.unstubAllGlobals())

function flushFrames(): void {
  while (frames.length > 0) frames.shift()?.(0)
}

describe('CanvasView', () => {
  it('synchronizes normalized zoom without feedback and coalesces redraw bursts', () => {
    const sourceCanvas = new FakeCanvas()
    const targetCanvas = new FakeCanvas()
    const source = new CanvasView(sourceCanvas as unknown as HTMLCanvasElement)
    const target = new CanvasView(targetCanvas as unknown as HTMLCanvasElement)
    source.setGeometry(geometry, fixture)
    target.setGeometry({ ...geometry, width: 200, height: 160 }, fixture)
    flushFrames()
    sourceCanvas.clearRect.mockClear()
    targetCanvas.clearRect.mockClear()

    let propagated = 0
    source.onViewChanged = () => {
      propagated += 1
      const view = source.normalizedView()
      if (view) target.applyNormalizedView(view)
    }
    for (let index = 0; index < 20; index += 1) {
      sourceCanvas.dispatch('wheel', {
        clientX: 400,
        clientY: 300,
        deltaY: -2,
        preventDefault: vi.fn(),
      })
    }
    flushFrames()

    expect(propagated).toBe(20)
    expect(sourceCanvas.clearRect).toHaveBeenCalledTimes(1)
    expect(targetCanvas.clearRect).toHaveBeenCalledTimes(1)
    expect(target.normalizedView()).toEqual(source.normalizedView())
  })

  it('preserves a normalized view across recomputed geometry', () => {
    const canvas = new FakeCanvas()
    const view = new CanvasView(canvas as unknown as HTMLCanvasElement)
    view.setGeometry(geometry, fixture)
    flushFrames()
    const retained = view.normalizedView()
    expect(retained).not.toBeNull()

    view.setGeometry({ ...geometry, width: 300, height: 240 }, fixture)
    view.applyNormalizedView(retained!)
    flushFrames()
    const restored = view.normalizedView()
    expect(restored?.centerX).toBeCloseTo(retained!.centerX)
    expect(restored?.centerY).toBeCloseTo(retained!.centerY)
    expect(restored?.relativeScale).toBeCloseTo(retained!.relativeScale)
  })
})
