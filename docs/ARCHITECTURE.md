# Architecture

SchemWeave is a reusable engine, not a Synth Explorer subsystem. The core
crate accepts a directed graph of sized nodes, fixed boundary ports, electrical
net identities, and cycle-break hints. It returns positioned node rectangles and
orthogonal edge polylines. It has no DOM, JavaScript, HDL, Yosys, or application
model dependency.

The current pipeline validates and compacts stable caller identifiers, cuts
ranking dependencies at explicit cycle breakers, condenses remaining strongly
connected components, assigns left-to-right ranks, applies deterministic
barycentric ordering over real and net-scoped virtual items, refines adjacent
crossings, places node layers, and routes ordinary forward nets through sparse
inter-layer channels and obstacle-free vertical corridors. Inter-layer and
outer-trunk lane nesting use bounded crossing-cost transposition. Unsupported port
directions, feedback, and very large multi-terminal nets retain globally unique
channel tracks and may branch onto deterministic top and bottom outer trunks as
a correctness backstop. Negotiated congestion and native multi-terminal trees
will replace that backstop while retaining the public contract and topology
stages.

`schemweave-wasm` exposes the same engine through a JSON function designed
to run inside a reusable Web Worker. Cancellation, cache policy, worker lifetime,
and rendering remain responsibilities of the consuming application.

`schemweave-eval` is a development-only quality scorer. It validates the public
geometry contract and measures node overlap, route/node intersections,
unrelated segment overlap and contact, physical net crossings, edge-level bends
and route length, and area. It is not a dependency of either runtime crate.

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

## Routing direction

The target router uses a sparse rectilinear visibility graph around placed node
obstacles. Initial routes use A* with length, bend, crossing, and congestion
costs. Deterministic Pathfinder-style rip-up and reroute raises historical costs
on contested resources until unrelated segment sharing and node intersections
are eliminated. A final lane-assignment and compaction pass reduces area without
changing connectivity or endpoint direction.

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
