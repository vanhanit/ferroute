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
| `crates/ferroute-board` | Board model (`Board`, `BoardEdit`, `Mobility`, rules), `builder` for tests/synthetic boards |
| `crates/ferroute-drc` | DRC oracle: clearance/short, edge, holes, keep-outs, courtyards, sizes, connectivity |
| `crates/ferroute-router` | Lattice PathFinder: `legality`, `plan` (separation theorem), `access`, `search` (A\*), `negotiate`, `emit` |
| `crates/ferroute-place` | Movability policy + multi-start annealing nudger |
| `crates/ferroute-engine` | Pipeline (nudge → route → LNS → gate), `synth` boards, `score` baselines, criterion bench |
| `crates/ferroute-kicad` | Lossless `.kicad_pcb` S-expr reader/writer (KiCad 6–10 dialects), `.kicad_pro` rules |
| `crates/ferroute-ipc` | Pure-Rust NNG REQ0 over IPC, hand-written KiCad protobufs, capability negotiation, headless server |
| `crates/ferroute-gpu` | `ClearanceBackend`: rayon CPU + wgpu WGSL GPU (feature `gpu`) with proven error bounds |
| `crates/ferroute-render` | Deterministic SVG renderer (visual diffs) |
| `crates/ferroute-cli` | `ferroute` binary: `route`, `drc`, `render`, `gen`, `bench`, `ipc {probe,route,export-svg}`, `info` |
| `testdata/boards` | KiCad 10 and KiCad 9 fixtures (+ `.kicad_pro`) |
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
- [ ] **INV-CONN-1** connectivity never decreases.
- [ ] **INV-COST-1** PathFinder costs non-negative; present factor and history monotone.
- [ ] **INV-ACCEL-1** accelerators never change results (legality tables equal with/without).
- [ ] **INV-DET-1/2/3** parallel == sequential; PRNG stream frozen; no host-libm transcendental in decisions (use `ferroute_geom::detmath`).
- [ ] **INV-IO-1** KiCad writer touches only edited nodes; same input ⇒ byte-identical output.

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
| 3 | Differential pairs (Dubins-aware) and length matching | Lead | M2 |
| 4 | Region-window LNS (rip-up + nudge around failures) | Lead | M2 |
| 5 | Specctra DSN/SES import/export (Freerouting comparisons) | Worker | contract: `Board` ↔ DSN |
| 6 | Native IPC commits (`BeginCommit`/`CreateItems`/`EndCommit`) for live mode | Worker | needs KiCad 10 GUI test rig |
| 7 | GPU A\* batches (GAMER-style sweeps) behind `ClearanceBackend`-like trait | Lead + worker | M3 |
| 8 | Dense BGA / 4–8-layer benchmark suites | Worker | extend `synth` |
| 9 | Proto drift check against upstream KiCad `api/proto` in nightly lane | Worker | |
| 10 | Copper pour awareness, blind/buried/micro vias | Lead | M4 |

## Environment notes for cloud sessions

* GitHub web downloads, Debian mirrors and Launchpad may be blocked from sandboxes; crates.io,
  npm, PyPI, Docker Hub (rate-limited) and gitlab.com usually work. KiCad sources/protos:
  `https://gitlab.com/api/v4/projects/kicad%2Fcode%2Fkicad/repository/files/<path>/raw?ref=10.0`.
* The KiCad 10 container `kicad/kicad:10.0.8` provides `kicad-cli` for DRC/export checks.
