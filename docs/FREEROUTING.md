# Comparing ferroute with Freerouting

Freerouting is GPL-3.0. ferroute only ever runs it as an external process on exported
files; it is never vendored, linked or committed.

## Setup

* `scripts/fetch_freerouting.sh` downloads `freerouting-2.5.0-executable.jar` from Maven
  Central into `~/.cache/ferroute/freerouting` (outside the repository) and verifies the
  published `.sha1`. Maven Central rate-limits (HTTP 429): the script retries.
* Freerouting 2.5.0 is compiled for **Java 25**. `scripts/freerouting_compare.sh` runs it in
  the official `eclipse-temurin:25` Docker image (jar and work dir bind-mounted). Set
  `FERROUTE_JAVA=/path/to/java` (JDK 25) to use a local JVM instead.
* CLI flags used (from `--help`): `-de in.dsn -do out.ses -mp <passes> -mt <threads>
  --gui.enabled=false`.

## Method

`scripts/freerouting_compare.sh [--dense] [--passes N] [--timeout S] [boards...]`:

1. ferroute: `ferroute route` in fixed placement (no `--allow-move`), `--threads $(nproc)`.
2. Freerouting: `ferroute dsn export` -> Freerouting (20 passes, `-mt $(nproc)`) ->
   `ferroute dsn import-ses`. Rules come from the board's `.kicad_pro`.
3. Both results are scored by the same `ferroute drc` oracle (completion, error count);
   vias and wirelength are counted from the output `.kicad_pcb`; wall time is the router
   process only (Freerouting includes JVM start-up and container start).
   Freerouting runs with the `--router.*` settings from `ferroute_dsn::freerouting_settings` (below).

## Results

Same machine (4 cores, 4 threads each), 2026-11-25, Freerouting 2.5.0, 20 passes,
per-board timeout 1500 s, Freerouting run with the settings below. Wirelength is track
length only.

| board | router | completion % | DRC errors | vias | wirelength mm | wall s |
|---|---|---:|---:|---:|---:|---:|
| demo_2layer | ferroute | 100.0 | 0 | 8 | 339.7 | 0.6 |
| demo_2layer | freerouting 2.5.0 | 100.0 | 0 | 4 | 370.2 | 27.5 |
| bus_match | ferroute | 100.0 | 0 | 2 | 195.1 | 0.5 |
| bus_match | freerouting 2.5.0 | 100.0 | 0 | 0 | 221.2 | 7.1 |
| qfn_dense_2l | ferroute | 99.3 | 0 | 56 | 583.4 | 8.4 |
| qfn_dense_2l | freerouting 2.5.0 | 95.7 | 0 | 31 | 559.9 | 78.4 |
| bga_256_4l | ferroute | 90.6 | 0 | 373 | 1281.6 | 32.4 |
| bga_256_4l | freerouting 2.5.0 | 62.3 | 0 | 0 | 1131.0 | 488.2 |

Freerouting is now DRC-clean under the board's own rules on all four boards, so completion
is comparable. Remaining differences, stated honestly:

* The clean result costs Freerouting completion on the dense boards: its fanout stage is
  what escapes BGA/QFN pins with vias, and it cannot be kept without its trace narrowing
  (below). With the fanout stage on (the earlier run: bga_256_4l 93.8 %, qfn_dense_2l
  100 %) the results contained 979 / 95 DRC errors, so those higher completions were
  not valid results. A fair "Freerouting at its best" number for dense BGAs is therefore
  between the two; we report the DRC-clean one.
* Freerouting uses no vias on bga_256_4l (fanout off), ferroute 373; wirelength is not
  comparable at different completion.
* Wall time: ferroute `--threads 4`; Freerouting includes JVM and container start.
* Only a single run per board; Freerouting's multi-threaded result varies between runs.

## Settings Freerouting needs (what a DSN cannot express)

`ferroute_dsn::freerouting_settings(&board).args()` (CLI: `cargo run -p ferroute-dsn
--example freerouting_args -- board.kicad_pcb`; the compare script passes them via
`FR_SETTINGS`). Each was verified in isolation against the real 2.5.0 jar on
`qfn_dense_2l`:

| error (our oracle) | cause in Freerouting | fix |
|---|---|---|
| `TrackWidth` (traces of 112 / 90 um for a 150 um net) | the class `(rule (width W))` is only the default width. Freerouting necks traces down to 3/4, 3/5 and 1/2 of W: always in its fanout stage ("micro neckdown"), and elsewhere when `automatic_neckdown` is on. No DSN keyword (`use_via`, class/layer rules, `(control ...)`) prevents this | `--router.fanout.enabled=false --router.automatic_neckdown=false` (both are needed: fanout alone leaves 51 narrow traces, neckdown alone 50). With both, all 213 wires keep their class width. `--router.neck_width_um=0` is already the default |
| `HoleClearance` | drill-to-copper clearance is not in the DSN | `--router.hole_clearance_um=<board hole clearance, um>`: 0 errors (12 before) |
| `Short` against pours | `(plane ...)` only makes the pour connectable for its own net | `--router.plane_as_obstacle=true`: 0 errors (12 before). The DSN still exports `(plane net (polygon ...))` so own-net pins connect |

The width class in the DSN is the net's `required_width` (class, current and impedance
rules, whichever is widest), so the default width Freerouting starts from is already the
minimum. Unrelated Freerouting settings seen in the jar (`copper_to_edge_clearance_um`,
`clearance_tolerance_um`, `strict_drc`) were not needed for these boards and are not set.

## Interchange findings (fixed)

* Freerouting 2.5.0 overflows its coordinates at nanometre resolution (diameters read as
  -1, keep-outs "degenerate", nothing routes). `(resolution um 10)` reads correctly but its
  SES writer then scales numbers by the resolution twice (10x too large). The export now
  uses `(resolution um 1)`: positions round to the nearest micrometre, sizes round up.
* Pours are exported as `(plane net (polygon ...))` on the zone outline (as KiCad does); other nets are kept out with `--router.plane_as_obstacle=true` (see above).
* `testdata/dsn/demo_2layer.freerouting.ses` is a real Freerouting session for the demo
  export; `freerouting_session_for_demo_imports_clean` imports it and requires 0 DRC errors.
