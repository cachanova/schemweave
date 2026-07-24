# Evaluation

`schemweave-eval` is a development-only scorer. The engine uses bounded runtime
admission checks; the scorer supplies corpus and visual-review evidence.

## Hard gates

`QualityReport::passes_hard_gates()` requires zero semantic violations, node
overlaps, route/node intersections, unrelated route overlap/contact, and
left-to-right direction violations for ranking edges. Compare quality metrics
only after the relevant hard gates pass.

The report also measures crossings, bends, directness/stretch, route length,
area, parallel congestion/separation, node clearance, same-net sharing,
perimeter routing, and viewport fit. Same-net geometry is merged only for
physical measurements; hard gates still inspect original edge routes.

## Visual review

`tools/visual-review` converts corpus fixtures into a SchemWeave graph, runs the
real WASM module in a Web Worker, scores SchemWeave and ELK, and renders both
views together.

It requires matching `corpus.json` (fixture inputs and exact source revision)
and `elk.json` (ELK geometry). Load both files in the UI or serve them as
`/review-data/corpus.json` and `/review-data/elk.json`.

```bash
cd tools/visual-review
npm ci
npm run dev
```

For a fair comparison, keep corpus revision, options, and effort fixed; confirm
hard gates first; then weigh readability, directness, congestion, area, and
route length together. Keep transient screenshots and benchmark histories out
of tracked documentation. See [Contributing](../CONTRIBUTING.md) for checks.
