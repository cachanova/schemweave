import type { LayoutOptions } from './types'

export function initialLayoutOptions(
  requested: LayoutOptions,
  needsRefinement: boolean,
): LayoutOptions {
  return needsRefinement ? { ...requested, quality_effort: 'fast' } : requested
}
