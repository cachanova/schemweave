# Architecture

SchemWeave is a reusable geometry engine. It accepts a directed graph of sized
nodes, fixed ports, net identities, and optional constraints; it returns node
rectangles and orthogonal edge polylines. It has no DOM, HDL, Yosys, or
application-model dependency.

```text
Graph + LayoutConfig
  -> validate and index stable IDs
  -> rank left-to-right components
  -> order layers and place nodes
  -> route bounded candidates
  -> admit exact valid geometry
  -> optionally refine within budget
  -> Layout
```

## Components

| Component | Responsibility |
| --- | --- |
| `schemweave` | Validation, layout, routing, deterministic selection, group expansion. |
| `schemweave-wasm` | JSON/WASM boundary over the same core. |
| `schemweave-eval` | Development-only correctness and readability scoring. |
| `tools/visual-review` | Corpus conversion, worker orchestration, ELK comparison, rendering. |

Consumers own rendering, labels, caching, request supersession, and worker
lifetime. Native and WASM callers execute the same core implementation.

## Layout pipeline

1. **Validate and index.** IDs, dimensions, ports, endpoints, options, and
   constraints are checked. Nodes and edges are sorted by stable ID, which makes
   tie-breaking independent of caller input order.
2. **Rank.** The engine condenses strongly connected components and assigns
   left-to-right ranks. Ranking can exclude explicit non-flow edges or true
   feedback edges entering a cycle breaker; those edges remain routable.
3. **Order and place.** Forward/reverse barycentric ordering, optional
   max-effort alternatives, port alignment, and isotonic projection produce
   bounded placement candidates.
4. **Route.** Forward east-to-west edges use inter-layer channels; eligible
   same-net fanout can share a trunk. Feedback, unsupported directions, and
   very large nets can use deterministic outer lanes.
5. **Admit.** Candidates must meet applicable bundle, node-clearance,
   unrelated-contact, and parallel-wire-spacing checks before comparison by
   crossings, bends, route length, and area; deterministic candidate order
   breaks exact ties.
6. **Refine.** `Fast`, default `Quality`, and `Max` choose bounded work.
   `Max` may add deeper ordering/routing repair, demand-aware spacing, and
   lane-pitch refinements, subject to configured area and route-length budgets.

## Group expansion

Expansion receives a compact graph and layout, an expanded graph, the replaced
anchor/members, and explicit boundary-trunk replacements. It preserves distant
geometry, opens a horizontal corridor for additional width, and can move the
connected local vertical slab obstructing a taller expansion. Routes crossing
that slab receive deterministic orthogonal jogs; the non-reflow retained
candidate remains available when local reflow is unsafe or over budget. The
engine lays out members canonically and accepts either composition only when
all hard geometry and left-to-right invariants hold. Native callers receive
`GroupExpansionError`; the WASM boundary converts selected safe fallbacks into
a tagged full-relayout response.

## Invariants

- Stable IDs define output order and ties.
- Ports are fixed boundary points; routes begin/end at their declared ports.
- Routes are axis-aligned and never cross node interiors.
- Same-net sharing is explicit; unrelated nets never silently merge.
- Hot paths use compact indexes and bounded work counts.

SchemWeave targets port-based schematics and directed data-flow. It can suit
hardware block diagrams and similar pipelines, but is not a general UML,
timing-diagram, arbitrary-graph, or floorplan engine. See [Usage](USAGE.md).
