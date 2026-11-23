# RFC-0001 — ferroute architecture

| | |
|---|---|
| **Status** | Accepted (v0.1 vertical slice implemented) |
| **Date** | 2026-11-19 |
| **Scope** | Research synthesis, algorithms, invariants, compute tiers, KiCad 10/11 integration |
| **Supersedes** | — |

ferroute is an open-source (AGPL-3.0-only) PCB auto-router and placement-aware optimiser
for KiCad, written in Rust, that scales from a laptop CPU to a GPU. This RFC records the
research behind it, the decisions taken, the invariants that make its output trustworthy,
and the roadmap. Facts about third-party projects were verified on the date above (sources
at the end); everything marked **[implemented]** exists in this repository with tests.

---

## 1. Problem statement

KiCad users who reach for an autorouter today typically:

1. export Specctra DSN, run Freerouting (Java), import SES, and then fix DRC errors by hand;
2. or route manually with KiCad's interactive push-and-shove router.

On dense boards the batch path stalls at 90–98 % completion, cycles through rip-up passes,
and never touches placement — even when moving one 0603 resistor by half a millimetre
would open a channel. ferroute's goals, in priority order:

1. **Never emit a DRC violation** (exact, independent verification of every result).
2. **Higher completion on dense boards** via negotiated congestion instead of greedy
   sequential rip-up, plus *placement co-optimisation* of explicitly unlocked parts.
3. **Determinism** — every run is a pure function of (board, config, seed), across thread
   counts, accelerators and platforms.
4. **Portable speed** — multi-core everywhere, GPU where available, nothing required.
5. **Clean KiCad integration** — KiCad 10 today (files + live IPC), KiCad 11 headless IPC
   tomorrow, never the deprecated SWIG `pcbnew` bindings.

---

## 2. Prior art

### 2.1 Open-source routers for KiCad

| Project | Approach | Strengths | Why it stalls / limits |
|---|---|---|---|
| **Freerouting** (Java, v1.9.x) | Specctra DSN/SES; gridless shape-based maze routing with push-and-shove, sequential net order, iterative rip-up passes, post-route optimiser | Mature, gridless, handles arbitrary angles, KiCad plugin via PCM, public API in beta | Nets routed one at a time; rip-up has no global *history* cost, so passes oscillate on congested regions; placement immutable; DSN round-trip loses KiCad-specific rules (custom rules, some keep-out semantics); largely single-threaded |
| **OrthoRoute** (Python/CUDA) | GPU-accelerated **PathFinder** on a pre-laid *Manhattan lattice* with blind/buried vias | Demonstrates that negotiated congestion + GPU scales to enormous backplanes and BGA escape | Author explicitly scopes it narrowly (SMD-focused, orthogonal lattice, via-heavy); not a general-purpose router; board-adaptive parameters needed to stop congestion growing per iteration |
| **KiCadRoutingTools** (Python + Rust A*) | Rust A* with PyO3 bindings, orientation-aware A* with Dubins-length heuristic for differential pairs, KiCad 9/10 | Good diff-pair features, fast Rust core | Sequential A* per net; no negotiation; no placement changes |
| **KiCad PNS** (built-in) | Interactive push-and-shove, walk-around, length tuning | Excellent *interactive* quality | Not a batch autorouter |
| **TopoR-style topological routers** (commercial) | Any-angle "rubber-band" sketches (cf. SURF, Dai/Dayan/Staepelaere 1991), topology first, geometry later | Short, smooth, few vias | Topological planning is hard to make exact against arbitrary DRC; closed-source |

**Diagnosis — why existing KiCad routers stall on dense boards**

* **Greedy sequential commitment.** Early nets grab contested resources; later nets fail;
  rip-up of individual nets without a memory of *chronic* congestion produces cycles.
  PathFinder's history term exists precisely to break such cycles.
* **Rigid representations.** Coarse grids miss off-grid pins; gridless push-and-shove is
  inherently serial. ferroute uses an exact *lattice + exact pin stubs* model whose
  correctness is a theorem (§5.3), and keeps gridless refinement as a later, separately
  verified pass.
* **Immutable placement.** Real designers nudge passives. No open router does.
* **Verification outside the loop.** Errors surface after re-import into KiCad. ferroute
  embeds an exact DRC oracle and refuses to emit anything it flags.

### 2.2 VLSI / FPGA routing and placement

| Work | Idea | Adopted? |
|---|---|---|
| **PathFinder** (McMurchie & Ebeling, FPGA'95) | Negotiated congestion: cost = (base + history) × present-congestion; all nets rerouted each iteration with growing present factor | **Yes — core of the router** [implemented] |
| **NCTU-GR 2.0** (Liu et al., TCAD'13) | Congestion-aware global routing with history-based cost scheduling, bounded-length maze routing, parallelism | Cost scheduling ideas; bounded search windows **[implemented: region-bounded A* with margin growth]** |
| **CUGR / CUGR 2.0** (Liu et al., DAC'20 / DAC'23) | 3-D pattern routing + multi-level maze routing, DAG-based pattern routing | Planned: pattern-route first pass on coarse grid (M3) |
| **TritonRoute / TritonRoute-WXL** (Kahng et al., ICCAD'18, TCAD'21) | Detailed routing with search-and-repair, DRC-driven rip-up of *regions*, parallel via region partitioning | Region-based repair → our LNS windows (M2); deterministic region batching **[implemented: batch independence]** |
| **RippleFPGA** (Pui et al., ICCAD'16) | Placement with routability-driven cell inflation & legalisation | Congestion-driven inflation for nudger (M3) |
| **ePlace / RePlAce** (Lu et al., TODAES'15; Cheng et al., TCAD'19) | Electrostatics-based analytic placement: density as charge, Nesterov gradient steps, FFT Poisson solve | Planned analytic nudger for large boards (M4); v0.1 uses annealing (small movable sets) |
| **DREAMPlace** (Lin et al., DAC'19) | Placement as deep-learning training on GPU | GPU placement model for M4 |
| **GAMER** (Lin & Wong, ICCAD'21) | GPU maze routing via alternating horizontal/vertical sweeps, O(log n) per sweep | Planned GPU A* replacement for batches (M3) |
| **FastGR / InstantGR** (TCAD'23 / ICCAD'24) | CPU–GPU task graphs; batches of nets with non-overlapping resources routed simultaneously | **Yes — our deterministic independent batches** [implemented] |
| **RUDY** (Spindler & Johannes, DATE'07) | Rectangular uniform wire density congestion estimate | **Yes — nudger congestion term** [implemented] |

### 2.3 Robotics & motion planning

| Technique | Use in ferroute | Status |
|---|---|---|
| **A\*** with octile heuristic | Lattice search (admissible heuristic to target box) | [implemented] |
| **Theta\*** / Lazy Theta\* (Nash et al., AAAI'07/'10) | Any-angle post-smoothing of lattice paths, verified by the oracle | M2 |
| **Dubins / Reeds–Shepp curves** | Orientation-constrained diff-pair centre-lines (cf. KiCadRoutingTools); 45°-octilinear arc smoothing | M2 |
| **RRT\*** (Karaman & Frazzoli, IJRR'11) | Rejected for main routing (non-deterministic sampling density, poor in narrow passages); possible for escape of awkward pins | Deferred |
| **Signed-distance fields** | GPU/CPU clearance fields that *prune* exact checks (§6) | [implemented] |

### 2.4 Combinatorial optimisation

| Technique | Use | Status |
|---|---|---|
| **Large Neighbourhood Search** (Shaw, CP'98) | Destroy/repair: re-nudge parts around failed nets with up-weighted HPWL, re-route, keep best | [implemented, v0.1 global window] |
| **Simulated annealing**, multi-start | Nudger: seeded chains in parallel, best chain wins deterministically | [implemented] |
| **Parallel tempering** (Swendsen–Wang '86; Earl & Deem '05) | Replica exchange between nudger chains | M3 |
| **Tabu search** (Glover '89) | Forbid recently reverted moves in nudger | M3 |
| **Beam search** | Net-ordering portfolio for multi-start routing rollouts | M3 |

### 2.5 Formal methods (Bend-style law checking)

The Bend language's appeal is *laws checked by the runtime*. ferroute's equivalent is a
catalogue of named invariants (§5), each enforced by (a) exact integer predicates,
(b) property-based tests (`proptest`) and (c) a deterministic DRC gate in CI that also runs
KiCad's own `kicad-cli pcb drc` as an external oracle.

---

## 3. Architecture

```
          ┌────────────── transports ──────────────┐
          │ .kicad_pcb file  │ KiCad IPC (NNG/pb)  │   ferroute-kicad / ferroute-ipc
          └────────┬─────────┴─────────┬───────────┘
                   ▼                   ▼
             ferroute-board  (transport-agnostic model, exact nm geometry)
                   │
      ┌────────────┼──────────────────────────────────┐
      ▼            ▼                                  ▼
 ferroute-place  ferroute-router ──uses──► ferroute-gpu (accelerated fields)
 (nudger, SA)    (PathFinder lattice)
      └────────────┬──────────────────────────────────┘
                   ▼
             ferroute-engine  (pipeline, LNS, synthetic suite, scoring)
                   │
                   ▼
             ferroute-drc  (exact oracle — final gate)  ──► ferroute-render (SVG)
                   │
                   ▼
             ferroute-cli  (`ferroute route|drc|render|gen|bench|ipc|info`)
```

| Crate | Responsibility | Owner tier |
|---|---|---|
| `ferroute-geom` | i64 nm geometry, exact predicates, units, frozen PRNG, deterministic math | Lead |
| `ferroute-board` | Board model, rules, edits + edit invariants, builder | Lead |
| `ferroute-drc` | Exact DRC oracle, connectivity, bucket grid | Lead |
| `ferroute-router` | Legality, reservation plan, pin access, A\*, negotiation, emission, DRC gate | Lead |
| `ferroute-place` | Movability policy, legality, multi-start annealing | Lead |
| `ferroute-engine` | Pipeline + LNS, synthetic generator, scoring/baselines | Lead |
| `ferroute-kicad` | Lossless S-expression reader/writer, KiCad 6–10 dialects, `.kicad_pro` rules | Sub-agent (contract by lead) |
| `ferroute-ipc` | Pure-Rust NNG REQ0 over IPC, hand-written protobufs, capability negotiation, headless server | Sub-agent |
| `ferroute-gpu` | `ClearanceBackend` trait, rayon CPU backend, wgpu WGSL backend with error bounds | Sub-agent |
| `ferroute-render` | Deterministic SVG renderer | Sub-agent |
| `ferroute-cli` | Command-line interface | Lead |

**ADR-1 — Rust.** Memory safety, `rayon` work stealing, zero-cost FFI, single static
binaries (musl), first-class `wgpu`. `unsafe_code = "forbid"` workspace-wide.

**ADR-2 — Integer nanometres.** KiCad's internal unit is the nanometre; using it avoids
conversion error. Decimal strings are parsed exactly (no `f64` for coordinates).

**ADR-3 — Lattice routing with a separation theorem** (instead of gridless shove) for the
global/detailed phase; gridless smoothing later as a separately verified pass.

**ADR-4 — Pure-Rust IPC.** The `nng` crate is unmaintained (last release 2021) and needs a
C toolchain; the SP-over-IPC framing is ~200 lines, verified against real NNG (`pynng`)
and KiCad's official generated protobufs (`kicad-python`/`kipy`) in tests.

**ADR-5 — Hand-written protobuf structs** (`prost` derives) for the subset we use, pinned
by wire-byte tests; avoids vendoring GPL-3.0-or-later `.proto` files and a `protoc`
build dependency. A drift check against upstream protos is part of the KiCad 11 lane.

**ADR-6 — Accelerators propose, the exact oracle disposes** (§6).

**ADR-7 — Own PRNG and deterministic math** (xoshiro256\*\* + SplitMix64; Taylor `exp`) so
seeds replay across dependency upgrades and platforms.

---

## 4. Algorithms (v0.1)

### 4.1 Routing lattice and legality
* Pitch `p` defaults to `w₀ + c₀` (default-class width + clearance), the tightest pitch at
  which adjacent default-class tracks are legal.
* Per net class, every lattice node, every forward edge (E, SE, S, SW) and every via site
  gets a state: `BLOCKED`, `FREE`, or `only(net)` — computed with **exact** predicates
  against fixed obstacles: pads, pre-existing copper, drilled holes (hole clearance and
  hole-to-hole), board edge (edge clearance, containment), keep-outs (containment and
  boundary contact). Policy: no via-in-pad.
* Optional accelerator field (§6) prunes sites provably far from all obstacles.
* **Lattice choice** (`choose_lattice`): the origin is aligned *per axis* so the most pad
  centres share a lattice row/column, which gives straight on-axis approaches. The pitch
  is then grown automatically (+15 % in 0.05 mm steps) whenever that removes break-out
  kinks without losing completion. The pitch never drops below `w₀ + c₀`.

### 4.2 Pin access — on-axis pad entry, ending at the pad centre
Every connection ends exactly at the pad centre and crosses the pad edge on an axis
through the centre:

* **Entry directions:** rectangular, oval and round-rect pads are entered/left
  perpendicular to a side through its midpoint (short or long side, following pad
  rotation); circular pads (incl. through-hole pads) along any 0°/45°/90° line through
  the centre. Non-octilinear pad rotations fall back to the 8 octilinear directions.
* **Stub shape:** centre → break-out point `m` on the entry axis at or beyond the pad edge
  (may be off the lattice) → optional single 0°/45°/90° leg pointing *away* from the pad
  to a lattice node. Only this last approach leaves the lattice.
* **Exactness:** each stub segment is checked against fixed obstacles exactly. The part of
  the stub inside the pad copper (exact erosion test) needs no reservation; the outside
  part reserves lattice nodes within `max(track requirement + pitch/√2, via/hole
  requirement)`. Stub-vs-stub conflicts between nets are precomputed exactly and
  negotiated like any other resource (present cost, history, rip-up).
* **No pad crossing:** lattice nodes inside pad copper are blocked, so routes never cut
  across a pad.
* **No small kinks:** a break-out whose outward leg is shorter than one pitch pays a
  `kink` penalty. The penalty is divided by `pres_fac`, so it shapes the first,
  uncongested iterations and gives way when space really forces a short leg. A\*
  endpoints carry their arrival heading, so turns into and out of a stub (and at vias:
  `via_turn`) are priced like any other bend. Vias therefore sit in line with straight
  runs.
* **Direct links:** nearby same-net pads (≤ 2.54 mm) whose entry axes face each other get
  one straight centre-to-centre track when it is exactly legal. The lattice is not used.

### 4.3 Negotiated congestion (PathFinder)
* Nets are ordered by terminal HPWL (seeded SplitMix64 tie-break).
* Each net is a Steiner tree grown by repeated multi-source/multi-target A\* (sources = tree,
  targets = unconnected islands; pads already joined by existing copper form one island).
* Step cost: `len · (1 + h(v)) · (1 + pres_fac · cong(v)) + bend`, via cost similarly;
  acute turns forbidden. `pres_fac` grows geometrically, history `h` accumulates on
  overused sites.
* Only conflicting or incomplete nets are re-routed in later iterations; failed nets get
  wider search windows (×3) up to the whole board.
* After negotiation, any residual conflict is resolved by dropping the lowest-priority
  conflicting nets (completion < 100 %, never a violation).

### 4.3b Lattice-preserving smoothing
After negotiation, each route (in priority order, two passes) is straightened: a
sub-path between two of its nodes is replaced by a straight or two-segment octilinear
lattice connection (one 45° run + one 0°/90° run) when that connection is legal and
strictly cheaper (length + bend penalties). Endpoints, Steiner junctions and via sites
stay pinned. The smoothed route is re-reserved and kept only if it creates no conflict,
so INV-ROUTE-1 is preserved and every vertex stays on the lattice.

### 4.3c Junctions without acid traps (INV-ROUTE-3)
Same-net copper may only meet at ≥ 90° outside pad/via copper. For a Steiner attach
point, `router::junction` computes the directions of the copper already incident there
(`incident`). From that it derives the departures that would form an acute angle or run
back over existing copper (`forbidden_departures`). The search's start endpoints carry
that mask. Smoothing and stub selection reject candidates for which `has_acute_or_overlap`
holds. A net's own *used* stubs become dynamic keep-outs for the rest of its tree, so a
route never runs back along its own break-out. The oracle check
`ferroute_drc::quality::acute_angles` must be empty on every proptest board.

### 4.4 Deterministic parallelism
Consecutive nets (in priority order) whose *footprint regions* — search window ∪ old route,
inflated by the largest reservation radius — are pairwise disjoint form a batch; members
are searched in parallel against the batch-start snapshot and committed in order. Since
members cannot read or write each other's cells, the result equals sequential routing
(INV-DET-1, tested across 1/3/4-thread pools).

### 4.5 Placement co-optimisation (nudger)
* **Policy:** KiCad-locked footprints are `Locked` (never moved). Others are `Fixed` unless
  they match user globs (default `R*,C*,L*,FB*`) and touch no pre-existing copper; movable
  parts get a displacement radius (default 2 mm) and optional 90° rotations.
* **Cost:** `w_wire·Σ weight·HPWL + w_cong·Σ RUDY-overflow² + w_disp·Σ|Δ|`.
* **Hard constraints (exact):** displacement/rotation bounds; courtyard inside the board,
  not intersecting other courtyards on the same side or footprint keep-outs; moved pads
  clear of all other copper and the edge.
* **Placement grid (INV-PLACE-3):** a translated footprint's anchor always lands on an
  absolute multiple of the placement grid (`--move-grid`, default 0.1 mm).
* **Search:** seeded multi-start annealing (parallel chains, deterministic winner), window
  shrinking with temperature; then routing; then LNS rounds that up-weight failed nets.

### 4.6 Emission and gate
Lattice paths become straight track runs (merged collinear steps), through vias and stubs.
The edit is applied to a copy of the board and the full DRC oracle runs; any net involved
in an introduced violation would be removed (`drc_gate_rips`, expected 0 and asserted 0
in property tests over thousands of random boards).

### 4.7 Electrical rules (INV-RULE-1)
Per-net constraints (`ferroute_board::constraints::NetConstraints`) are stored in exact
integer units (mA, Hz, mΩ, nm). They come from a TOML rules file (`--rules`, net/class
globs, applied in order, unknown keys rejected). The values are converted once, at that
edge.

* **Width:** the required width is the widest of the net-class width, the IPC-2221
  current width (`I = k·ΔT^0.44·A^0.725`, internal-layer `k` on multilayer boards) and the
  IPC-2141 microstrip width for a target impedance over the stack-up. It is rounded *up*
  to 10 µm, computed only with `detmath` (IEEE-portable `exp/ln/pow`). `resolve` moves a
  wider-than-class net into a derived class before routing. Traces are never narrowed.
* **Cost policy (router::search::net_costs):** high-speed nets (≥ 50 MHz or
  impedance-controlled) pay vias ×3, 90° bends ×4 (two 45° bends are cheaper) and
  via-turns ×2. Via-limited nets pay `1 + 4/(1 + max_vias)` more per via.
* **Checks (ferroute_drc::rules):** width vs. requirement, routed length window,
  length-matching skew within a group, and via count. Unmet rules are reported in the
  CLI/JSON report and fail with `--strict-rules`. They are never hidden.
* Next (M2): length tuning (serpentines) and differential pairs, so matching rules are
  *met*, not only checked.

---

## 5. Invariants (the "law book")

| ID | Invariant | Enforcement |
|---|---|---|
| **INV-GEOM-1** | All predicate inputs satisfy \|coord\|, radius ≤ 2³⁰ nm, so i128/u128 arithmetic is exact | `Board::validate`, overflow proptest |
| **INV-DRC-1** | DRC decisions are exact and shapes are convex *supersets* of true copper ⇒ clean report implies clean design | Exact predicates; proptests vs float away from the boundary; bucketed == brute force |
| **INV-DRC-2** | Our oracle agrees with KiCad's | CI lane runs `kicad-cli pcb drc` (KiCad 10.0.8) on every routed fixture |
| **INV-ROUTE-1** | *Lattice separation theorem* (below): no base node of one net in a ring of another ⇒ inter-net clearance holds | `debug_assert` + `routing_never_introduces_violations` proptest (400 dense boards in stress mode) |
| **INV-EDIT-1** | Edits are additive (never delete/modify existing copper) and only move `Movable` footprints within bounds | `BoardEdit::check_invariants`, re-checked by the KiCad writer |
| **INV-PLACE-1** | Locked/Fixed footprints never move; hard constraints always hold | Nudger proptest |
| **INV-PLACE-3** | Translated footprints land on the absolute placement grid | `moves_land_on_the_placement_grid` |
| **INV-ROUTE-2** | Every connection terminates at the pad centre, enters along a pad axis (side midpoint) or 0/45/90° line for circular pads, and every segment is 0°/45°/90° | `tracks_terminate_at_pad_centres_and_stay_octilinear` |
| **INV-CONN-1** | Connectivity is monotone: a run never disconnects pads that were connected | Additive edits + connectivity analysis |
| **INV-COST-1** | PathFinder costs are non-negative; `pres_fac` and history are monotone non-decreasing | Construction (all factors ≥ 1, multipliers > 1) |
| **INV-ACCEL-1** | Accelerators never change results | Legality tables equal with/without accelerator (test); routed edit equal for CPU-field vs exact |
| **INV-DET-1** | Parallel result == sequential result | Thread-pool tests (router, nudger) |
| **INV-DET-2** | PRNG stream frozen per seed | Golden-value test |
| **INV-DET-3** | No host-`libm` transcendental in solver decisions | `detmath` (IEEE-only), lint by review |
| **INV-IO-1** | The writer alters only what the edit specifies; untouched subtrees are byte-identical; same inputs ⇒ byte-identical output | `ferroute-kicad` edit tests |

### 5.3 Lattice separation theorem (INV-ROUTE-1)
Every routed element `e` (track node, the two *side nodes* of a diagonal step, a via site on
every layer) has a base node and reserves the Chebyshev ball of radius `ρ(e)` around it.
**Rule:** no base node of one net may lie inside another net's reserved ring.

*Claim.* For elements `e, f` of different nets, every point of `e`'s centre-line is at
Euclidean distance ≥ `(max(ρ(e), ρ(f)) + 1)·p` from every point of `f`'s centre-line.

*Proof.* The rule gives Chebyshev distance ≥ `max(ρ(e),ρ(f)) + 1` between base nodes of
different nets. Lattice centre-lines are unions of nodes, unit orthogonal segments and unit
diagonals; each lies in a closed (possibly degenerate) unit cell whose corners are all base
nodes of its net (diagonals contribute their side nodes). For two such cells, the minimum
Chebyshev distance between their points equals that between their corners, and Euclidean
distance dominates Chebyshev distance. ∎

The reservation plan (`ferroute_router::plan`) picks the smallest integers ρ with
`(max(ρa, ρb) + 1)·p ≥ D(a, b)` for every element-kind pair (track/track, via/track,
via/via including hole clearance); a separate net-independent *hole channel* guarantees
hole-to-hole spacing even between vias of the same net. Fixed obstacles are handled by
exact legality, stubs by their reservation radius, and the oracle checks everything again.

---

## 6. Compute tiers

| Tier | Backend | Used for | Status |
|---|---|---|---|
| 1 — GPU | `wgpu` 30 (Vulkan / Metal / DX12 / GL), WGSL compute; CUDA via `cudarc` evaluated, deferred (wgpu covers NVIDIA without a vendor SDK) | Obstacle distance fields; later GAMER-style sweeps, multi-start rollouts | Field kernel implemented, validated by `naga`; runs under lavapipe in CI |
| 2 — CPU | `rayon` work stealing, cache-friendly row chunks | Legality, A\* batches, annealing chains, DRC | [implemented] |
| 0 — Exact | Integer predicates | Every decision | [implemented] |

**Rule (ADR-6).** An accelerator returns an approximate field with a *proved* error bound
(`64·E·2⁻²⁴ + 1 nm` for the f32 GPU kernel, `max(cutoff, r)·2⁻²⁴ + 1 nm` for the CPU field).
A site is skipped only if `field > threshold + bound`; otherwise the exact predicate
decides. Therefore results are bit-identical with and without acceleration.

---

## 7. KiCad integration

### 7.1 Facts (verified in KiCad sources, Nov 2026)
* **KiCad 10.0.8** (2026-10-29) writes board file version **20260206**. Since **20251028**
  nets are referenced *by name* (`(net "GND")`) and there is no net table; older files use
  numeric codes. Layer ids changed in KiCad 9 (F.Cu=0, B.Cu=2, In1=4, …).
* Pad `(at x y a)`: position relative to the footprint, unrotated frame; **angle absolute**.
  Text angles absolute. Rules live in `.kicad_pro` (`net_settings`, `design_settings.rules`).
* **IPC API:** KiCad's server is an **NNG REP0** socket on `ipc:///tmp/kicad/api.sock`
  (Windows: `%TEMP%\kicad\api.sock` as a named pipe); messages are `kiapi.common.ApiRequest`
  / `ApiResponse` protobufs with `google.protobuf.Any` payloads; plugins get
  `KICAD_API_SOCKET` / `KICAD_API_TOKEN`.
* **KiCad 9/10:** IPC only with a running GUI; no plotting/export over IPC.
* **KiCad 11 (master / 10.99 nightlies):** `kicad-cli api-server [--socket PATH] [FILE]`
  (headless); `OpenDocument`, `CloseDocument`, `CloseAllDocuments`, `CreateDocument`;
  job export `kiapi.board.jobs.RunBoardJobExport{Svg,Pdf,Gerbers,Drill,…}` returning
  `RunJobResponse`; async tasks (`tasks.proto`).
* SWIG `pcbnew` bindings are deprecated; KiCad developers warn that Python plugins not
  ported to IPC may stop working in KiCad 11. ferroute never uses them.

### 7.2 Design
* **Transport abstraction.** Domain crates see only `Board` and `BoardEdit`. Transports:
  file (`ferroute-kicad`), live IPC (`ferroute-ipc`), headless IPC (same client + spawned
  `kicad-cli api-server`).
* **Capability negotiation.** `GetVersion` → generation (`KiCad9`, `KiCad10`,
  `KiCad11Plus` for ≥ 10.99), then *feature probes* (deliberately invalid `OpenDocument`
  and `RunBoardJobExportSvg`; `AS_UNHANDLED`/`AS_UNIMPLEMENTED` ⇒ absent). Version alone
  is never trusted.
* **KiCad 10 live mode (v0.1):** "file bridge" — `SaveDocument` → route file →
  `RevertDocument` (reload). **Caveat:** the reload discards KiCad's undo history and any
  edit made in the GUI while routing runs. `ferroute ipc route` prints this warning every
  time, and the in-place write keeps a `*.ferroute-bak` backup. **M3:** native
  `BeginCommit` / `CreateItems` / `EndCommit` so the result is a single undoable commit.
* **KiCad 11 mode:** headless server for CI; IPC SVG/Gerber export for visual diffs.
* **Batch / CI:** `.kicad_pcb` in, `.kicad_pcb` out, `kicad-cli pcb drc` as external
  oracle; Specctra DSN/SES interop planned (M3) for Freerouting comparisons.

### 7.3 CI lanes
* **KiCad 10.x (required):** routes every fixture with the static musl binary inside
  `kicad/kicad:10.0.8`, runs `kicad-cli pcb drc` (errors and unconnected items fail),
  exports SVG + Gerbers to prove KiCad loads the output.
* **KiCad 11 preview (non-blocking, nightly):** installs KiCad nightly, starts
  `api-server`, runs `ferroute ipc probe` and `ferroute ipc export-svg`.

---

## 8. Quality gates & benchmarks

Every change is scored on (a) DRC pass rate, (b) completion, (c) wirelength, (d) vias,
(e) displacement, (f) runtime, over the seeded synthetic suite and the fixtures
(`ferroute bench`), and compared with the committed `bench/baseline.json`
(tolerances: completion may not drop; wirelength +3 %; vias +5 % + 2; displacement
+15 %; runtime ×5). SVG renders of every benchmark board are uploaded as CI artefacts.

v0.1 baseline (2-/4-layer synthetic boards up to 55×40 mm, plus fixtures): 100 %
completion and zero violations on every board; 3–210 ms per board on one CI-class core
set. These boards are deliberately modest; the M2/M3 milestones add dense BGA and
4–8-layer stress suites.

### 8.1 Release & build policy
* Releases are cut from `main` by git-cliff only for `feat`, `fix` and `perf` (or
  breaking) commits. `scripts/bump_version.sh` updates the workspace and internal
  dependency versions. Pushes that touch only docs, Markdown or CI files skip the release
  workflow (`paths-ignore`).
* Every release binary is cross-compiled in one Linux container (Docker `cross` stage,
  cargo-zigbuild + zig; Windows via the GNU target) by `scripts/dist.sh`, also available
  as `make dist`. Native macOS/Windows runners only *run* tests.
* `scripts/check_toolchain.sh` keeps `rust-toolchain.toml`, the Docker base image and the
  workspace `rust-version` aligned. It runs in `make lint` and the CI `fmt` job.
* The KiCad 10 lane routes fixtures with `testdata/rules/<board>.toml` (if present) under
  `--strict-rules`.

---

## 9. Roadmap

| Milestone | Content |
|---|---|
| **M1 (this)** | Exact geometry + DRC oracle, lattice PathFinder with separation theorem, deterministic batches, nudger + LNS, KiCad 10 file I/O, IPC probe/live bridge, KiCad 11 headless/export client, accelerated fields, CI lanes, benchmarks |
| **M2** | Gridless post-optimisation (Theta\*-style any-angle shortcutting, 45° smoothing, track hugging) verified by the oracle; per-layer preferred directions; diff pairs (Dubins-aware) and length matching; fine-pitch/BGA escape; region-based LNS windows |
| **M3** | GPU A\* batches (GAMER-style sweeps) and multi-start routing rollouts; pattern-routing pre-pass (CUGR-style); native IPC commits; DSN/SES interop; parallel tempering / tabu in the nudger |
| **M4** | ePlace-style electrostatic analytic nudging for large boards; congestion-driven inflation (RippleFPGA-style); copper pour awareness; blind/buried/micro vias |

---

## 10. Licensing & supply chain

* AGPL-3.0-only with commercial dual licensing (see `COMMERCIAL.md`); contributors grant
  relicensing rights (see `CONTRIBUTING.md`).
* `cargo-deny` allows only AGPL-compatible licences (MIT, Apache-2.0 (± LLVM exception),
  BSD-2/3, ISC, Zlib, MPL-2.0, LGPL, GPL-3.0, AGPL-3.0, plus Unicode-3.0, BSL-1.0, CC0-1.0);
  everything else (GPL-2.0-only, CC-BY-NC, SSPL, BUSL, proprietary) fails CI.
* `cargo-audit` + `cargo-deny` advisories, Dependabot, pinned container digests, committed
  `Cargo.lock`, toolchain pinned via `rust-toolchain.toml` (Rust 1.100.0).

---

## 11. Sub-agent delegation model

The lead owns geometry, model, invariants, DRC, routing, placement, orchestration and
review. Well-scoped, contract-first work is delegated to cheaper models with strict
interfaces and acceptance tests written by the lead; every delegated crate is reviewed and
must pass `fmt`, `clippy -D warnings` and its tests before merge. v0.1 delegations:
DevOps/CI scaffolding, KiCad reader/writer, IPC client, GPU backend, SVG renderer.

---

## References

* L. McMurchie, C. Ebeling. *PathFinder: A Negotiation-Based Performance-Driven Router for FPGAs.* FPGA 1995.
* W.-H. Liu, W.-C. Kao, Y.-L. Li, K.-Y. Chao. *NCTU-GR 2.0: Multithreaded Collision-Aware Global Routing With Bounded-Length Maze Routing.* IEEE TCAD 2013.
* J. Liu, C.-W. Pui, F. Wang, E. F. Y. Young. *CUGR: Detailed-Routability-Driven 3D Global Routing with Probabilistic Resource Model.* DAC 2020.
* A. B. Kahng, L. Wang, B. Xu. *TritonRoute: An Initial Detailed Router for Advanced VLSI Technologies.* ICCAD 2018; *TritonRoute: The Open-Source Detailed Router.* IEEE TCAD 2021.
* C.-W. Pui, G. Chen, W.-K. Chow, et al. *RippleFPGA: A Routability-Driven Placement for Large-Scale Heterogeneous FPGAs.* ICCAD 2016.
* J. Lu, P. Chen, C.-C. Chang, et al. *ePlace: Electrostatics-Based Placement Using Fast Fourier Transform and Nesterov's Method.* ACM TODAES 2015.
* C.-K. Cheng, A. B. Kahng, I. Kang, L. Wang. *RePlAce: Advancing Solution Quality and Routability Validation in Global Placement.* IEEE TCAD 2019.
* Y. Lin, S. Dhar, W. Li, et al. *DREAMPlace: Deep Learning Toolkit-Enabled GPU Acceleration for Modern VLSI Placement.* DAC 2019.
* S. Lin, M. D. F. Wong. *GAMER: GPU Accelerated Maze Routing.* ICCAD 2021 (journal version IEEE TCAD).
* FastGR (IEEE TCAD 2023) and InstantGR (ICCAD 2024): GPU-parallel global routing with resource-disjoint net batches.
* P. Spindler, F. M. Johannes. *Fast and Accurate Routing Demand Estimation for Efficient Routability-driven Placement.* DATE 2007 (RUDY).
* A. Nash, K. Daniel, S. Koenig, A. Felner. *Theta\*: Any-Angle Path Planning on Grids.* AAAI 2007.
* S. Karaman, E. Frazzoli. *Sampling-based Algorithms for Optimal Motion Planning.* IJRR 2011.
* L. E. Dubins (1957); J. A. Reeds, L. A. Shepp (1990) — shortest curvature-bounded paths.
* P. Shaw. *Using Constraint Programming and Local Search Methods to Solve Vehicle Routing Problems.* CP 1998 (LNS).
* R. H. Swendsen, J.-S. Wang (1986); D. J. Earl, M. W. Deem (2005) — parallel tempering.
* F. Glover. *Tabu Search — Part I.* ORSA J. Computing 1989.
* W. W.-M. Dai, T. Dayan, D. Staepelaere. *Topological Routing in SURF.* DAC 1991.
* D. Blackman, S. Vigna. *Scrambled Linear Pseudorandom Number Generators.* ACM TOMS 2021 (xoshiro256\*\*).
* KiCad IPC API developer docs — <https://dev-docs.kicad.org/en/apis-and-binding/ipc-api/>
* KiCad CLI (`api-server`) docs — <https://docs.kicad.org/master/en/cli/cli.html>
* KiCad sources: `api/proto/**`, `common/api/api_server.cpp`, `kicad/cli/command_api_server.cpp`, `pcbnew/pcb_io/kicad_sexpr/*` — <https://gitlab.com/kicad/code/kicad>
* J. Evans. *Bringing a new API to KiCad.* FOSDEM 2025.
* Freerouting — <https://github.com/freerouting/freerouting>
* OrthoRoute — <https://github.com/bbenchoff/OrthoRoute>, <https://bbenchoff.github.io/pages/OrthoRoute.html>
* KiCadRoutingTools — <https://github.com/atomic14/KiCadRoutingTools>
* GAMER — <https://ieeexplore.ieee.org/document/9643563/>; InstantGR — <https://qqq1357962.github.io/pdf/instantgr.pdf>; FastGR — <https://www.cse.cuhk.edu.hk/~byu/papers/J86-TCAD2023-FastGR.pdf>
