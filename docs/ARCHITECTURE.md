# Architecture

SchemWeave is a reusable engine, not a Synth Explorer subsystem. The core
crate accepts a directed graph of sized nodes, fixed boundary ports, electrical
net identities, and cycle-break hints. It returns positioned node rectangles and
orthogonal edge polylines. It has no DOM, JavaScript, HDL, Yosys, or application
model dependency.

The current pipeline validates and compacts stable caller identifiers, cuts
feedback dependencies at explicit cycle breakers while retaining acyclic root
constraints, condenses remaining strongly connected components, and assigns
left-to-right ranks. Deterministic barycentric ordering operates over real and
net-scoped virtual items from both forward and reverse stable seeds. The core
compares the conservative baseline against the better-seeded, stronger
port-alignment candidate with a near-linear physical crossing scorer. Placement
uses bounded bidirectional sweeps and weighted isotonic projection, preserving
node order and minimum gaps.

Ordinary forward nets route through sparse inter-layer channels and
obstacle-free vertical corridors. Eligible single-driver multi-terminal nets
share one median-guided corridor backbone and branch only in their final gap.
Inter-layer and outer-trunk lanes use bounded crossing-cost transposition.
Unsupported port directions, feedback, and extremely large nets retain unique
channel tracks and may branch onto deterministic top and bottom outer trunks as
a correctness backstop.

`schemweave-wasm` exposes the same engine through a JSON function designed
to run inside a reusable Web Worker. Cancellation, cache policy, worker lifetime,
and rendering remain responsibilities of the consuming application.

`schemweave-eval` remains the development-only full quality scorer. It validates
the public geometry contract and measures node overlap, route/node intersections,
unrelated segment overlap and contact, physical net crossings, unique physical
net bends, union wire length, and area. Hard gates operate on the original edge
routes; same-net merging affects only physical quality measurements. The scorer
is not a dependency of either runtime crate.

## Invariants

- Stable identifiers determine output order and tie-breaking.
- Ports are fixed points on node boundaries.
- Every route begins and ends at its declared ports and contains only
  axis-aligned segments.
- Routes never pass through node interiors.
- Unrelated nets may cross but only share endpoint escape segments when they
  attach to the same fixed port.
- Native and WebAssembly builds execute the same core implementation.
- The implementation uses compact numeric indices internally and avoids graph
  cloning between phases.

## Routing resources

The router constructs only layer gaps, free vertical intervals, endpoint escape
tracks, and the lanes actually used by the graph. Dynamic programming chooses a
consistent free interval across each forward span. Net-aware ordering and
bounded adjacent transposition minimize predicted crossings without allocating
a dense visibility grid or performing order-dependent global reroute rounds.

Electrical `net` identities are explicit in the API. Shared trunks are legal
within one net; multiple nets may also share the minimum escape segment when
they attach to the same fixed endpoint. Geometric coincidence elsewhere never
silently turns unrelated edges into a shared route.

## Consumer boundary

Synth Explorer will map its compact layout input to this graph contract in its
layout worker and map the returned geometry back to existing renderer types. It
will pin a reviewed crate revision. ELK remains its only production layout path
until the Rust/WASM engine satisfies the application corpus's semantic,
geometric, visual-quality, browser-latency, memory, cancellation, and
determinism gates.
