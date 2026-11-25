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

## Results

Same machine (4 cores, 4 threads each), 2026-11-24, Freerouting 2.5.0, 20 passes,
per-board timeout 900 s. Wirelength is track length only.

| board | router | completion % | DRC errors | vias | wirelength mm | wall s |
|---|---|---:|---:|---:|---:|---:|
| demo_2layer | ferroute | 100.0 | 0 | 8 | 339.7 | 0.6 |
| demo_2layer | freerouting 2.5.0 | 100.0 | 0 | 4 | 393.3 | 7.8 |
| bus_match | ferroute | 100.0 | 0 | 2 | 195.1 | 0.2 |
| bus_match | freerouting 2.5.0 | 100.0 | 0 | 0 | 212.9 | 4.9 |
| qfn_dense_2l | ferroute | 99.3 | 0 | 56 | 583.4 | 3.5 |
| qfn_dense_2l | freerouting 2.5.0 | 100.0 | 95 | 15 | 661.8 | 29.6 |
| bga_256_4l | ferroute | 90.6 | 0 | 373 | 1281.6 | 7.3 |
| bga_256_4l | freerouting 2.5.0 | 93.8 | 979 | 256 | 1784.8 | 665.2 |

Reading the DRC errors of the Freerouting dense results (our oracle, board's own rules):

* `TrackWidth` (71 / 101): Freerouting narrows traces below the net-class width (112 um,
  90 um instead of 150 um) to squeeze through fine-pitch pads. ferroute never does.
* `HoleClearance` (12 / 80): drill clearance is not modelled in the DSN.
* `Short` against copper pours (12 / 798): Freerouting treats the exported `(plane ...)`
  conduction areas as connectable but routes other nets through them. Pours are not
  yet exported as obstacles (open follow-up).

So Freerouting's higher completion on the dense boards is not DRC-clean under the same
rules; ferroute's results are all error-free.

## Interchange findings (fixed)

* Freerouting 2.5.0 overflows its coordinates at nanometre resolution (diameters read as
  -1, keep-outs "degenerate", nothing routes). `(resolution um 10)` reads correctly but its
  SES writer then scales numbers by the resolution twice (10x too large). The export now
  uses `(resolution um 1)`: positions round to the nearest micrometre, sizes round up.
* Pours are exported as `(plane net (polygon ...))` on the zone outline (as KiCad does).
* `testdata/dsn/demo_2layer.freerouting.ses` is a real Freerouting session for the demo
  export; `freerouting_session_for_demo_imports_clean` imports it and requires 0 DRC errors.
