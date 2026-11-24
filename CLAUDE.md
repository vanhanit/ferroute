# CLAUDE.md — ferroute

Operating manual for every Claude session (lead or sub-agent) working on this repository.
Read it fully before changing code. The design rationale lives in
[`docs/ARCHITECTURE_RFC.md`](docs/ARCHITECTURE_RFC.md) — keep both in sync.

## Mission

ferroute is an open-source (AGPL-3.0-only, commercial dual licence), GPU- and multi-core
accelerated **PCB auto-router and placement-aware optimiser for KiCad 10 and 11**,
written in Rust. Priorities, in order:

1. **Never emit a DRC violation** — the exact oracle (`ferroute-drc`) gates every result.
2. Higher completion on dense boards: PathFinder negotiated congestion + nudging of
   explicitly *unlocked* parts. **Locked parts never move.**
3. **Determinism** — output is a pure function of (board, config, `--seed`).
4. Portable speed: CPU (`rayon`) everywhere, GPU (`wgpu`) when present, never required.
5. Clean KiCad integration via files and the IPC API. **Never use the SWIG `pcbnew`
   bindings.**

**North star:** route a board as well as — or better than — an expert human, fast and
provably correct. Every design or review decision is judged against "would an expert
layout engineer accept this result without touching it?"

## Operating model (tiered sub-agents)

* **Lead (Opus-class):** research synthesis, maths, core data structures, invariants, DRC,
  routing/placement algorithms, GPU/parallel architecture, review, architectural decisions.
* **Workers (Sonnet/Haiku-class):** well-scoped deterministic tasks — parsers, IPC glue,
  CLI plumbing, test suites, generators, Docker/Makefile/CI, docs.
* **Default to delegation:** implementation goes to the cheapest capable worker model
  (Haiku for mechanical edits, Sonnet for crate-level features). The lead keeps maths,
  invariants and core data structures, writes the contract and acceptance tests, and
  *verifies the output* (diff review, tests, oracle/KiCad DRC, rendered result) instead of
  writing the code itself. Workers on disjoint crates run in parallel.
* **Token economy (always):** pick the cheapest option that will do the job right.
  1. The lead decides *what* to do and *how* (design, contract, acceptance criteria) —
     briefly; no exploratory reading beyond what the decision needs.
  2. Then decides *whether* to delegate and *to whom*: trivial or mechanical edits the lead
     does inline (cheaper than briefing anyone); scoped work goes to the cheapest capable
     worker (Haiku → Sonnet); only maths/invariants/architecture stay with the lead.
  3. Prompts are short and self-contained: files allowed, API contract, acceptance
     criteria, exact finish commands. Workers report concisely (files, tests, results) —
     no file dumps.
  4. Use targeted reads (`grep -n`, line ranges) instead of whole files; filter command
     output (`| grep`, `| tail`); run disjoint workers in parallel (worktrees when they
     share a crate); don't re-run green checks without a change in between.
  5. The lead always verifies the result against what was ordered (diff review, tests,
     oracle/KiCad DRC, rendered output) and fixes or re-delegates gaps before committing.
* **Autonomy:** the lead works continuously through the task board in priority order —
  decide, delegate, verify, merge, push — without waiting for approval between steps.
  Ask the user only for genuinely blocking decisions (credentials, irreversible external
  actions, conflicting requirements).
* **Merging worker results:** apply the worktree diff with `git apply --3way`, resolve conflicts, verify, then commit each feature with an explicit pathspec (`git commit -m … -- <paths>`) so nothing else that is staged slips into the wrong commit.
* **Delegation rules:** the lead writes the interface contract (types + signatures), the
  acceptance tests or criteria, and the list of files the worker may touch. Workers never
  edit crates they do not own, never edit the root `Cargo.toml` (add crate-local deps and
  report them), never commit, and must finish with `cargo fmt -p <crate> -- --check`,
  `cargo clippy -p <crate> --all-targets -- -D warnings`, and `cargo test -p <crate>`
  green. The lead reviews every delegated change before it is committed.

## Repository map

| Path | What |
|---|---|
| `crates/ferroute-geom` | i64-nanometre geometry, **exact** i128 predicates, `units` (exact mm↔nm), `rng` (frozen xoshiro256\*\*), `detmath` (IEEE-only exp/hypot) |
| `crates/ferroute-board` | Board model (`Board`, `BoardEdit`, `Mobility`, rules), `constraints` (per-net electrical rules → widths), `netrules` (rules file), `text::glob`, `builder` for tests/synthetic boards |
| `crates/ferroute-drc` | DRC oracle: clearance/short, edge, holes, keep-outs, courtyards, sizes, connectivity; `quality` (acid traps), `rules` (width/length/skew/via constraints) |
| `crates/ferroute-router` | Lattice PathFinder: `legality`, `plan` (separation theorem), `access`, `search` (A\*), `negotiate`, `emit`; `meander` + `tune` (length tuning) |
| `crates/ferroute-place` | Movability policy + multi-start annealing nudger |
| `crates/ferroute-engine` | Pipeline (nudge → route → LNS → gate), `synth` boards, `score` baselines, criterion bench |
| `crates/ferroute-kicad` | Lossless `.kicad_pcb` S-expr reader/writer (KiCad 6–10 dialects), `.kicad_pro` rules |
| `crates/ferroute-dsn` | Specctra DSN export / Freerouting SES import (transport crate) |
| `crates/ferroute-ipc` | Pure-Rust NNG REQ0 over IPC, hand-written KiCad protobufs, capability negotiation, headless server |
| `crates/ferroute-gpu` | `ClearanceBackend`: rayon CPU + wgpu WGSL GPU (feature `gpu`) with proven error bounds |
| `crates/ferroute-render` | Deterministic SVG renderer (visual diffs) |
| `crates/ferroute-cli` | `ferroute` binary: `route`, `drc`, `render`, `gen`, `bench`, `ipc {probe,route,export-svg}`, `info` |
| `testdata/boards` | KiCad 10 and KiCad 9 fixtures (+ `.kicad_pro`) |
| `testdata/rules` | Per-net electrical rules for fixtures (`<board>.toml`, used by e2e/CI with `--strict-rules`) |
| `bench/baseline.json` | Committed benchmark baseline (regression gate) |
| `docs/` | RFC, images |

## Build & test

Host needs only `make` + `docker` (everything runs in the dev container):

```bash
make help          # list targets
make build         # cargo build (workspace)
make test          # nextest + doctests
make lint          # rustfmt --check + clippy -D warnings
make audit         # cargo-deny (licences/bans/sources/advisories) + cargo-audit
make e2e           # route testdata boards, run our DRC
make kicad10-drc   # run KiCad 10.0.8 kicad-cli DRC on routed outputs
make drc-bench     # benchmark suite vs bench/baseline.json
make test-gpu      # GPU path on lavapipe (Vulkan software adapter)
make sonar         # local SonarQube scan (docker compose profile "sonar")
```

Native shortcuts (toolchain pinned by `rust-toolchain.toml`, Rust 1.100.0):
`cargo test --workspace --release`, `cargo clippy --workspace --all-targets -- -D warnings`,
`cargo run --release -p ferroute-cli -- route testdata/boards/demo_2layer.kicad_pcb -o out/demo.kicad_pcb --allow-move --svg out/demo.svg`.
Update the benchmark baseline only on purpose:
`ferroute bench --suite ci --baseline bench/baseline.json --update-baseline` (explain why in the commit).

## Invariant checklist (must hold for every change)

- [ ] **INV-GEOM-1** coordinates/radii ≤ 2³⁰ nm; legality uses only exact predicates (no float decisions).
- [ ] **INV-DRC-1** shapes are convex *supersets* of true copper; oracle is exact and deterministic.
- [ ] **INV-DRC-2** routed fixtures pass KiCad 10 `kicad-cli pcb drc` (CI lane).
- [ ] **INV-ROUTE-1** no base node of one net inside another net's ring; `routing_never_introduces_violations` proptest green; `drc_gate_rips == 0`.
- [ ] **INV-EDIT-1** edits are additive; only `Movable` footprints move, within bounds (`BoardEdit::check_invariants`).
- [ ] **INV-PLACE-1** locked/fixed parts never move; nudged parts satisfy every hard constraint.
- [ ] **INV-PLACE-3** translated footprints land on the absolute placement grid (`PlaceConfig::grid`, `--move-grid`).
- [ ] **INV-ROUTE-2** every routed connection ends exactly at the pad centre, crosses the pad edge on an axis through the centre (side midpoint for rectangular pads, any 0/45/90° line for circular pads), and every segment is 0°/45°/90°.
- [ ] **INV-ROUTE-3** no acid traps: same-net copper never meets at < 90° outside pad/via copper (`ferroute_drc::quality::acute_angles` is empty), and no overlapping same-net copper.
- [ ] **INV-RULE-1** traces are never narrower than their net's rule width (net class, current-derived or impedance-derived, whichever is widest); per-net clearance, via size and via limits are always honoured.
- [ ] **INV-CONN-1** connectivity never decreases.
- [ ] **INV-COST-1** PathFinder costs non-negative; present factor and history monotone.
- [ ] **INV-ACCEL-1** accelerators never change results (legality tables equal with/without).
- [ ] **INV-DET-1/2/3** parallel == sequential; PRNG stream frozen; no host-libm transcendental in decisions (use `ferroute_geom::detmath`).
- [ ] **INV-IO-1** KiCad writer touches only edited nodes; same input ⇒ byte-identical output.

## Routing quality standard ("as if a professional spent hours on it")

Completion and DRC come first; within that, every result must look hand-tuned:

* **Straight traces, 0°/45°/90° only.** Prefer long straight runs; avoid stair-steps and
  small kinks (outward break-out legs shorter than one lattice pitch are penalised and
  used only where space forces them).
* **Pads:** enter and leave along an axis through the pad centre — perpendicular to a side
  through its midpoint for rectangular/oval pads, any 0/45/90° line for circular pads,
  vias and holes — and end exactly at the centre. Only this final approach may leave the
  lattice. Nearby same-net pads facing each other get one straight direct link.
* **Vias** sit in line with straight traces (no bend right at a via); minimise via count.
* **Meanders** (length tuning) always have **rounded corners** (true KiCad arcs), uniform
  amplitude and constant leg spacing — never square or mitred bumps.
* **No acid traps** (acute same-net junctions) and no overlapping copper — branches leave
  existing copper at ≥ 90°.
* **Never narrow a trace** or change any per-net rule ad hoc. Widths/clearances/vias come
  from the net's rules only.
* **Electrical best practice:** honour per-net constraints — maximum current (derive the
  minimum width; IPC-2221 today, IPC-2152 curve fits planned — task #20), frequency / high-speed (fewer vias, 45° not 90° bends, short
  stubs, continuous reference plane), controlled impedance (width from the stack-up),
  length limits and length/skew matching groups (diff pairs: equal length, constant gap,
  symmetric routing), maximum via count. Where a rule cannot be met, report it — never
  silently violate it.
* Lock semantics: locked footprints never move; nudged parts land on the placement grid.

## Engineering practices (mandatory)

* **Test first.** For every feature or bug fix write the failing unit/property test
  first (red), implement the minimum to pass (green), then refactor. Bugs found by
  property tests keep their seed in the committed `proptest-regressions/` files.
* **Coverage:** aim for ≥ 85 % line coverage per crate (`make coverage`,
  `cargo llvm-cov --workspace --summary-only`); every public function and every
  invariant has a behaviour-focused test. Stress the router proptests before pushing
  router changes: `FERROUTE_PROP_CASES=1000 cargo test -p ferroute-router --release routing_never`.
* **Dependency injection:** inject I/O sinks (`&mut dyn Write`), transports (`Transport`),
  accelerators (`ClearanceBackend`), seeds/RNGs and environment lookups instead of
  reaching for globals, stdout, the filesystem or the clock inside logic. Keep side
  effects at the edges (CLI `main`, transports).
* **DRY & cohesion:** shared math lives in `ferroute-geom` (e.g. `Dir8::from_vector`,
  `Point::is_octilinear`), board semantics in `ferroute-board`; never copy a helper —
  move it to the right crate. Keep related code together (one module per concept, e.g.
  `router::junction`, `router::direct`).
* **Separation of concerns:** geometry facts (e.g. a stub's shape) are computed where the
  geometry lives (`router::access`); cost policy lives in `router::search`; negotiation
  orchestrates. Domain crates never depend on transports.
* Small, pure, deterministic functions; explicit types over tuples (clippy
  `type_complexity` is an error); no hidden global state.
* **Verify before pushing:** `cargo fmt --all -- --check`, clippy `-D warnings`, the
  workspace tests, `ferroute bench --suite ci --baseline bench/baseline.json` (fixtures run
  in nudging *and* fixed-placement mode), and KiCad 10 DRC on routed fixtures
  (`make e2e kicad10-drc`). Never claim CI is green without reading the check results.

## Release & build policy

* Only `feat`, `fix`, `perf` (or breaking) commits cut a release (SemVer via
  `scripts/bump_version.sh` + git-cliff). `docs`, `ci`, `test`, `build`, `chore`,
  `refactor`, `style` never version anything, and pushes that only touch docs, Markdown
  or CI files do not run the release workflow at all.
* **Cross-compile in a Linux container:** all release binaries are built by
  `scripts/dist.sh` inside the Docker `cross` stage (`make dist`, cargo-zigbuild; Windows
  via the GNU target). Native macOS/Windows runners are used only to *run* tests. Locally,
  check other targets with `cargo check --workspace --lib --bins --target <triple>`.

## Conventions

* **Commits:** atomic Conventional Commits (`feat(router): …`, `fix(drc): …`, `perf`, `refactor`, `test`, `docs`, `ci`, `build`, `chore`), enforced by commitlint; releases are cut automatically from `main` by git-cliff (SemVer).
* Every source file starts with `// SPDX-License-Identifier: AGPL-3.0-only`.
* `unsafe_code` is forbidden workspace-wide. Clippy runs with `-D warnings`.
* Units: nanometres (`i64`) internally; parse/format KiCad mm strings only via `ferroute_geom::units`.
* Stochastic code takes an explicit seed and uses `ferroute_geom::rng::Rng`; never `rand`/`thread_rng`, never iterate hash maps for decisions.
* New dependencies: latest stable from crates.io, AGPL-compatible licence (`cargo deny check licenses`), justify in the commit body.
* Domain crates (`board`, `drc`, `router`, `place`, `engine`) must not depend on transports (`kicad`, `ipc`).

## Task board (next steps)

| # | Task | Owner | Notes |
|---|---|---|---|
| 1 | Gridless post-optimiser: extend `router::straighten` (octilinear 1–2-segment chain shortcuts exist) to fine-pitch fan-outs (U1-style jogs) and via repositioning | Lead + worker | M2 |
| 2 | Per-layer preferred directions + via minimisation pass | Lead | M2 |
| 3 | Differential pairs: ✅ coupled routing (`router::diffpair`, `router::pairgeom`; constant gap, symmetric fans, `[[pair]]`/zdiff/`.kicad_dru` gaps, PairGap/PairUncoupled checks, fallback reported). Next: pairs across layers (via pairs), polarity swap, multi-pad ends, length equalisation inside the pair router, fan-out without the short horizontal step between the two 45° segments at the connector | Lead + worker | M2 |
| 4 | Region-window LNS (rip-up + nudge around failures) | Lead | M2 |
| 5 | Specctra DSN/SES: ✅ `ferroute-dsn` (`write_dsn`, `read_ses` → additive `BoardEdit`, exact units, `ferroute dsn export` / `import-ses`). Next: validate against real Freerouting (2.5.0 on Maven Central, needs Java 21), pad holes, zones | Worker | |
| 6 | Native IPC commits (`BeginCommit`/`CreateItems`/`EndCommit`) for live mode | Worker | needs KiCad 10 GUI test rig |
| 7 | GPU A\* batches (GAMER-style sweeps) behind `ClearanceBackend`-like trait | Lead + worker | M3 |
| 8 | ✅ Dense suite (`ferroute bench --suite dense`: bga_256_4l, bga_484_6l, qfn_dense_2l, mixed_8l). Findings drive #27–#29 | Worker | done |
| 9 | ✅ Proto drift check (`scripts/check_proto_drift.sh`, `crates/ferroute-ipc/proto.lock`, nightly `proto-drift` job; KiCad 11 files pinned to `master`, refresh with `--update` after review) | Worker | done |
| 10 | Copper pour awareness, blind/buried/micro vias | Lead | M4 |
| 11 | ✅ Stack-up read from `.kicad_pcb` (per-dielectric thickness/εr, sub-layers, board thickness) | Worker | done |
| 12 | Fine-pitch escape: congestion-aware pin-access ordering for QFN/QFP/BGA | Lead | M2 |
| 15 | `.kicad_dru` custom rules: parse the KiCad DRC rule language (clearance, track_width, via_diameter, hole_size, hole_clearance, edge_clearance, annular_width, diff_pair_gap, length, skew; NetClass/NetName/Type/layer conditions); unsupported conditions apply conservatively and are reported, never ignored | Worker | P1 — correctness gap |
| 15b | `.kicad_dru` follow-ups: pair-specific clearance in the router plan (today a pair rule raises both nets' clearance against everything — conservative, may cost completion on dense boards); enforce `max`/`opt` limits; pad annular rings; `hole_to_hole`, `diff_pair_gap`, `skew` and time-unit length rules; non-error severities | Lead + worker | from #15 |
| 16 | Copper pours: ✅ phase A (zones, fill connectivity, exact obstacles) and ✅ phase B (refill after routing via `ferroute_drc::refill`, `i_overlay` booleans, circumscribed obstacles ⇒ fill ⊆ legal fill; KiCad-clean with and without KiCad's own refill; INV-CONN-1 guard falls back to obstacle mode if a refill would split a pour net). Next: routes may end on same-net fill (via-to-plane fan-out); high-speed nets avoid crossing reference-plane splits; footprint-embedded zones; arcs in fills flattened conservatively; refill may grow fills back for moved same-net pads | Lead + worker | P1 |
| 17 | ✅ `router::lns` (MAPF-LNS-style neighbourhoods, Pareto acceptance). Next: larger/second-order neighbourhoods for coupled cases like the demo U1 V-kink (#26) | Lead + worker | P2 |
| 18 | ✅ Via-in-pad opt-in (`--allow-via-in-pad`, `[routing] via_in_pad`): pad-centre via terminals, relocation, KiCad 10 `(capping yes) (filling yes)`, cost warning | Worker | done |
| 19 | ✅ delay matching (`[[match]] by = "delay"`, `skew_ps`, via barrels). Open: infeasible length+delay mixes ping-pong before being reverted — detect early | Lead | P2 |
| 20 | Electrical models: IPC-2152 current curve fits, Hammerstad–Jensen microstrip (+ solder mask), stripline offset, edge-coupled differential and coplanar impedance; validate against published tables | Lead | P2 |
| 20b | Electrical-model follow-ups: verified Kirschning–Jansen (or Wadell) coupled microstrip (today IPC-2141A coupling approximation), narrow-strip stripline (w/(b−t) < 0.35 returns None → reported as unattainable), offset stripline, solder-mask correction, thickness for coupled stripline | Lead | from #20 |
| 21 | Crosstalk rules: parallel-run-length limits and 3W spacing for sensitive/high-speed nets, reported and enforced by the router cost policy | Lead + worker | P3 |
| 22 | ✅ Teardrops (opt-in, KiCad-native) and DFM checks (annular ring, aspect ratio, min drill, NPTH clearance). Next: solder-mask slivers, copper balance | Worker | done |
| 23 | Exact window optimisation: small congested regions solved exactly (ILP via HiGHS / `good_lp`, MIT) and spliced back under oracle verification | Lead | P3 |
| 24 | Push-and-shove local moves (KiCad PNS-style walkaround/shove) as LNS operator | Lead + worker | P3 |
| 25 | Next-generation core: topological (rubber-band) routing on an octilinear visibility/escape graph — topology first, then tight geometry; removes lattice artefacts at the source | Lead | M3 |
| 14 | Routing-quality residuals: demo has ≤ 1 sliver and ≤ 1 right angle per mode (diagonal–diagonal corner next to a T junction near (108.5, 126.5) with nudging); ci-s2-medium has 3 right angles and ci-s4-medium 2 slivers, where chamfers are illegal and need a local re-route; near-pad bends > 0 on synth boards; tighten the `fixture_quality` bounds to 0 | Lead + worker | commits a8e4162/5244ce8 landed with swapped scopes (a8e4162 also holds the router chamfer, stub and straighten work) |
| 26 | Via-jog residuals after `router::vias`: layer changes whose far anchors are fixed on both sides; demo U1 right side (V-kink before a long diagonal whose parallel neighbour blocks the shift — needs coordinated multi-chain moves, see #24) and a via beside the U1 top-right pad that cannot sit on the pad axis without crowding its own pad | Lead + worker | bounds in `fixture_quality.rs` |
| 27 | ✅ Via-to-plane fan-out (`router::plane`, dog-bone vias inside the net's own fill; bga_256_4l fixed 60.4 % → 75.5 %, VCC complete; GND deep balls need #29) | Lead + worker | done |
| 28 | Speed: ✅ round 1 (byte-identical, ~2×: A* scratch reuse + 4-ary heap + memoised congestion, shared legality across order/LNS rounds, parallel LNS orders, shared post-pass indices, nudger prefilter + incremental cost). Next: indexed decrease-key heap (32 % stale pops), cache-friendly legality/occupancy layout, incremental legality across LNS neighbourhoods; dense negotiation is serial | Lead + worker | P1 |
| 29 | BGA escape: parked on branch `wip/bga-escape` (planner + pour-net rip order). Findings on bga_256_4l: the 75.5 → 88.7 % gain came from ripping pour nets last, not from escapes; escapes add 8 right angles at their end nodes (router arrives at 90°) and +5 slivers; rip order alone regresses bga_484_6l via the pour fallback (79.4 → 70.5 %), escapes rescue it (90.1 %). Next: escape ends with a diagonal departure stub so arrivals are never 90°, stagger ends, let escapes pick micro/blind vias (#30), make rip order pour-continuity-aware (#31); merge only when right angles = 0 and slivers ≤ no-escape run | Lead + worker | P1 |
| 30 | ✅ Blind / buried / micro vias (opt-in `--via-types`, `[routing] via_types`; span-specific legality/reservation, KiCad 10 via kinds, microvia sizes, per-span DRC/DFM, HDI cost note; bga_256_4l 75.5 % → 97.8 %, KiCad DRC clean). Next: `micro_in_pad` (parsed, not yet used by the router); span search ~2.9× slower than through-only; let #29 escapes choose via kinds | Lead + worker | done |
| 31 | Pour continuity on dense multilayer boards: bga_484_6l falls back to obstacle mode because routed copper would split a plane — route/escape vias with pour-continuity gaps, prefer keeping planes contiguous (cost term), refill may then proceed | Lead + worker | from #29 findings |
| 13 | Coverage ≥ 85 % per crate: CLI output via injected `&mut dyn Write`, ipc/render/kicad gaps | Worker | `make coverage` |

## Environment notes for cloud sessions

* GitHub web downloads, Debian mirrors and Launchpad may be blocked from sandboxes; crates.io,
  npm, PyPI, Docker Hub (rate-limited) and gitlab.com usually work. KiCad sources/protos:
  `https://gitlab.com/api/v4/projects/kicad%2Fcode%2Fkicad/repository/files/<path>/raw?ref=10.0`.
* The KiCad 10 container `kicad/kicad:10.0.8` provides `kicad-cli` for DRC/export checks.
* KiCad 10 live mode is a *file bridge* (save → route → reload): it loses undo history and
  GUI edits made during routing. Keep the warning (`FILE_BRIDGE_WARNING`) until task #6 lands.
* Toolchain pins must agree (`scripts/check_toolchain.sh`): bump `rust-toolchain.toml`, the
  Dockerfile `FROM rust:` tag (with digest) and `rust-version` together.
