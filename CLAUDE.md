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
  minimum width, IPC-2152), frequency / high-speed (fewer vias, 45° not 90° bends, short
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
| 1 | Gridless post-optimiser: any-angle shortcutting (Theta\*-style) + 45° smoothing, verified by oracle | Lead | M2 |
| 2 | Per-layer preferred directions + via minimisation pass | Lead | M2 |
| 3 | Differential pairs (Dubins-aware): coupled routing with constant gap, symmetric tuning | Lead | M2; length tuning exists (`router::tune`, `router::meander`) |
| 4 | Region-window LNS (rip-up + nudge around failures) | Lead | M2 |
| 5 | Specctra DSN/SES import/export (Freerouting comparisons) | Worker | contract: `Board` ↔ DSN |
| 6 | Native IPC commits (`BeginCommit`/`CreateItems`/`EndCommit`) for live mode | Worker | needs KiCad 10 GUI test rig |
| 7 | GPU A\* batches (GAMER-style sweeps) behind `ClearanceBackend`-like trait | Lead + worker | M3 |
| 8 | Dense BGA / 4–8-layer benchmark suites | Worker | extend `synth` |
| 9 | Proto drift check against upstream KiCad `api/proto` in nightly lane | Worker | |
| 10 | Copper pour awareness, blind/buried/micro vias | Lead | M4 |
| 11 | Stack-up from `.kicad_pcb` `(setup (stackup …))` → `Board::stackup` (today: defaults or `--rules [stackup]`) | Worker | contract: `ferroute_board::constraints::Stackup` |
| 12 | Fine-pitch escape: congestion-aware pin-access ordering for QFN/QFP/BGA | Lead | M2 |
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
