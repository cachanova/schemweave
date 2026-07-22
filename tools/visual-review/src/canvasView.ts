import type { Fixture, Layout } from './types'

interface ViewState {
  scale: number
  x: number
  y: number
}

interface NormalizedView {
  centerX: number
  centerY: number
  relativeScale: number
}

export class CanvasView {
  private geometry: Layout | null = null
  private fixture: Fixture | null = null
  private view: ViewState = { scale: 1, x: 0, y: 0 }
  private dragging: { x: number; y: number } | null = null
  private fittedScale = 1
  private frame: number | null = null
  private controlEdges = new Set<number>()
  private registers = new Set<number>()
  private labels = new Map<number, string>()
  onViewChanged: (() => void) | null = null

  constructor(private readonly canvas: HTMLCanvasElement) {
    canvas.addEventListener('wheel', (event) => this.zoom(event), { passive: false })
    canvas.addEventListener('pointerdown', (event) => this.startPan(event))
    canvas.addEventListener('pointermove', (event) => this.pan(event))
    canvas.addEventListener('pointerup', () => this.endPan())
    canvas.addEventListener('pointercancel', () => this.endPan())
  }

  setGeometry(geometry: Layout, fixture: Fixture): void {
    this.geometry = geometry
    this.fixture = fixture
    this.controlEdges = new Set(
      fixture.resolvedInput.edges.filter((edge) => edge.control).map((edge) => edge.inputIndex),
    )
    this.registers = new Set(
      fixture.resolvedInput.nodes.filter((node) => node.register).map((node) => node.id),
    )
    this.labels = new Map(
      fixture.subgraph?.nodes.map((node) => [node.id, node.name || node.cell_type || `${node.id}`]),
    )
    this.fit()
  }

  clear(): void {
    this.geometry = null
    this.fixture = null
    this.controlEdges.clear()
    this.registers.clear()
    this.labels.clear()
    this.draw()
  }

  fit(): void {
    if (!this.geometry) return
    this.resize()
    const padding = 24 * devicePixelRatio
    const width = Math.max(this.geometry.width, 1)
    const height = Math.max(this.geometry.height, 1)
    this.fittedScale = Math.max(
      0.00001,
      Math.min(
        (this.canvas.width - padding * 2) / width,
        (this.canvas.height - padding * 2) / height,
      ),
    )
    this.view = {
      scale: this.fittedScale,
      x: (this.canvas.width - width * this.fittedScale) / 2,
      y: (this.canvas.height - height * this.fittedScale) / 2,
    }
    this.scheduleDraw()
  }

  normalizedView(): NormalizedView | null {
    if (!this.geometry) return null
    return {
      centerX:
        (this.canvas.width / 2 - this.view.x) / this.view.scale / Math.max(this.geometry.width, 1),
      centerY:
        (this.canvas.height / 2 - this.view.y) /
        this.view.scale /
        Math.max(this.geometry.height, 1),
      relativeScale: this.view.scale / this.fittedScale,
    }
  }

  applyNormalizedView(view: NormalizedView): void {
    if (!this.geometry) return
    this.resize()
    this.view.scale = this.fittedScale * view.relativeScale
    this.view.x =
      this.canvas.width / 2 - view.centerX * this.geometry.width * this.view.scale
    this.view.y =
      this.canvas.height / 2 - view.centerY * this.geometry.height * this.view.scale
    this.scheduleDraw()
  }

  draw(): void {
    this.resize()
    const context = this.canvas.getContext('2d')
    if (!context) return
    context.clearRect(0, 0, this.canvas.width, this.canvas.height)
    if (!this.geometry || !this.fixture) return

    context.save()
    context.translate(this.view.x, this.view.y)
    context.scale(this.view.scale, this.view.scale)
    context.lineCap = 'square'
    context.lineJoin = 'miter'
    context.globalAlpha = 0.48
    for (const edge of this.geometry.edges) {
      if (edge.points.length < 2) continue
      context.beginPath()
      context.moveTo(edge.points[0].x, edge.points[0].y)
      for (let index = 1; index < edge.points.length; index += 1) {
        context.lineTo(edge.points[index].x, edge.points[index].y)
      }
      context.strokeStyle = this.controlEdges.has(edge.id) ? '#f2a83b' : '#78879a'
      context.lineWidth = Math.max(0.7 / this.view.scale, 1.25)
      context.stroke()
    }

    context.globalAlpha = 0.96
    for (const node of this.geometry.nodes) {
      const register = this.registers.has(node.id)
      context.fillStyle = register ? '#6153d8' : '#243143'
      context.strokeStyle = register ? '#b5aaff' : '#8292a8'
      context.lineWidth = Math.max(0.65 / this.view.scale, 1)
      context.fillRect(node.x, node.y, node.width, node.height)
      context.strokeRect(node.x, node.y, node.width, node.height)
      if (node.width * this.view.scale > 52 * devicePixelRatio) {
        const label = this.labels.get(node.id) ?? `${node.id}`
        context.save()
        context.beginPath()
        context.rect(node.x + 2, node.y + 2, node.width - 4, node.height - 4)
        context.clip()
        context.fillStyle = '#eef4fb'
        context.font = `${Math.max(9 / this.view.scale, 11)}px ui-sans-serif, system-ui`
        context.textBaseline = 'middle'
        context.fillText(label, node.x + 6, node.y + node.height / 2)
        context.restore()
      }
    }
    context.restore()
  }

  private resize(): void {
    const bounds = this.canvas.getBoundingClientRect()
    const width = Math.max(1, Math.round(bounds.width * devicePixelRatio))
    const height = Math.max(1, Math.round(bounds.height * devicePixelRatio))
    if (this.canvas.width !== width) this.canvas.width = width
    if (this.canvas.height !== height) this.canvas.height = height
  }

  private zoom(event: WheelEvent): void {
    if (!this.geometry) return
    event.preventDefault()
    const bounds = this.canvas.getBoundingClientRect()
    const mouseX = (event.clientX - bounds.left) * devicePixelRatio
    const mouseY = (event.clientY - bounds.top) * devicePixelRatio
    const previous = this.view.scale
    const next = Math.min(
      this.fittedScale * 100,
      Math.max(this.fittedScale * 0.15, previous * Math.exp(-event.deltaY * 0.001)),
    )
    this.view.scale = next
    this.view.x = mouseX - (mouseX - this.view.x) * (next / previous)
    this.view.y = mouseY - (mouseY - this.view.y) * (next / previous)
    this.scheduleDraw()
    this.onViewChanged?.()
  }

  private startPan(event: PointerEvent): void {
    this.canvas.setPointerCapture(event.pointerId)
    this.canvas.classList.add('dragging')
    this.dragging = { x: event.clientX, y: event.clientY }
  }

  private pan(event: PointerEvent): void {
    if (!this.dragging) return
    this.view.x += (event.clientX - this.dragging.x) * devicePixelRatio
    this.view.y += (event.clientY - this.dragging.y) * devicePixelRatio
    this.dragging = { x: event.clientX, y: event.clientY }
    this.scheduleDraw()
    this.onViewChanged?.()
  }

  private endPan(): void {
    this.dragging = null
    this.canvas.classList.remove('dragging')
  }

  private scheduleDraw(): void {
    if (this.frame != null) return
    this.frame = requestAnimationFrame(() => {
      this.frame = null
      this.draw()
    })
  }
}
