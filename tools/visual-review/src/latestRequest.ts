export class LatestRequest<T> {
  private pending: T | null = null
  private active = false

  constructor(private readonly send: (request: T) => void) {}

  submit(request: T): void {
    this.pending = request
    this.dispatch()
  }

  complete(): void {
    if (!this.active) throw new Error('cannot complete an idle request queue')
    this.active = false
    this.dispatch()
  }

  get busy(): boolean {
    return this.active
  }

  get hasPending(): boolean {
    return this.pending != null
  }

  private dispatch(): void {
    if (this.active || this.pending == null) return
    const request = this.pending
    this.pending = null
    this.active = true
    this.send(request)
  }
}
