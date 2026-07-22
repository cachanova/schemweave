export class DatasetCache<T> {
  private readonly values = new Map<string, T>()
  private datasetId: number | null = null

  getOrInsert(datasetId: number, fixtureName: string, create: () => T): T {
    if (datasetId !== this.datasetId) {
      this.values.clear()
      this.datasetId = datasetId
    }
    const key = fixtureName
    const existing = this.values.get(key)
    if (existing !== undefined) return existing
    const value = create()
    this.values.set(key, value)
    return value
  }
}
