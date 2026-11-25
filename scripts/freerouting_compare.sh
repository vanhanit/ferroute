#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Benchmarks ferroute against Freerouting (run as an external tool, GPL-3.0, never vendored)
# on the testdata boards and, optionally, the dense synthetic suite. Both routers get the
# same board in fixed placement (ferroute without --allow-move). Prints a markdown table.
#
# Usage: scripts/freerouting_compare.sh [--dense] [--passes N] [--timeout S] [board.kicad_pcb ...]
# Env:   FERROUTE_BIN (default target/release/ferroute), FERROUTE_JAVA (a JDK >= 25 `java`),
#        FERROUTE_CACHE (default ~/.cache/ferroute), THREADS (default: all cores),
#        FERROUTE_WORK (work dir, default: a fresh mktemp dir), FR_EXTRA_ARGS,
#        FR_SETTINGS (set per board from ferroute-dsn: --router.* settings, see docs/FREEROUTING.md).
set -euo pipefail
cd "$(dirname "$0")/.."

DENSE=0 PASSES=20 TIMEOUT=600 BOARDS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --dense) DENSE=1 ;;
    --passes) PASSES="$2"; shift ;;
    --timeout) TIMEOUT="$2"; shift ;;
    *) BOARDS+=("$1") ;;
  esac
  shift
done
if [ ${#BOARDS[@]} -eq 0 ]; then
  BOARDS=(testdata/boards/demo_2layer.kicad_pcb testdata/boards/bus_match.kicad_pcb)
  # "qfn" is the dense synthetic QFN board (qfn_dense_2l), generated below with --dense.
fi

BIN="${FERROUTE_BIN:-target/release/ferroute}"
[ -x "$BIN" ] || cargo build --release -p ferroute-cli
THREADS="${THREADS:-$(nproc)}"
WORK="${FERROUTE_WORK:-$(mktemp -d)}"
mkdir -p "$WORK"

# Freerouting 2.5 needs Java 25 (class file 69). Default: the official eclipse-temurin:25
# Docker image (jar cache and work dir bind-mounted). FERROUTE_JAVA=/path/to/java overrides.
CACHE="${FERROUTE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/ferroute}"
JAVA="${FERROUTE_JAVA:-}"
JAR=$(scripts/fetch_freerouting.sh)
fr() { # fr <dsn> <ses> : paths below $WORK
  local dsn="$1" ses="$2"
  if [ -n "$JAVA" ]; then
    "$JAVA" -jar "$JAR" -de "$dsn" -do "$ses" -mp "$PASSES" -mt "$THREADS" --gui.enabled=false ${FR_SETTINGS:-} ${FR_EXTRA_ARGS:-}
  else
    docker run --rm --name "ferroute-fr-$$" -v "$(dirname "$JAR"):/fr:ro" -v "$WORK:/work" -u "$(id -u):$(id -g)" \
      -e HOME=/tmp eclipse-temurin:25 java -jar "/fr/$(basename "$JAR")" \
      -de "/work/${dsn#"$WORK"/}" -do "/work/${ses#"$WORK"/}" -mp "$PASSES" -mt "$THREADS" \
      --gui.enabled=false ${FR_SETTINGS:-} ${FR_EXTRA_ARGS:-}
  fi
}

if [ "$DENSE" = 1 ]; then
  "$BIN" gen --suite dense -o "$WORK/dense" >/dev/null
  BOARDS+=("$WORK/dense/qfn_dense_2l.kicad_pcb" "$WORK/dense/bga_256_4l.kicad_pcb")
fi

export JAVA JAR WORK PASSES THREADS FR_EXTRA_ARGS FR_SETTINGS
export -f fr

now() { date +%s.%N; }

# metrics <board.kicad_pcb> -> "completion drc_errors vias wirelength_mm"
metrics() {
  local b="$1" js="$WORK/m.json"
  "$BIN" drc "$b" --json "$js" >/dev/null || true
  python3 -I - "$b" "$js" <<'PY'
import json, math, re, sys
board, js = sys.argv[1:3]
r = json.load(open(js))
c = r["connectivity"]
tot = c["connections_total"]
done = tot - c["connections_missing"]
comp = 100.0 * done / tot if tot else 100.0
text = open(board).read()
wl = 0.0
for m in re.finditer(r"\(segment\s+\(start ([-\d.]+) ([-\d.]+)\)\s+\(end ([-\d.]+) ([-\d.]+)\)", text):
    x1, y1, x2, y2 = map(float, m.groups())
    wl += math.hypot(x2 - x1, y2 - y1)
vias = len(re.findall(r"\(via\b", text))
print(f"{comp:.1f} {len(r['violations'])} {vias} {wl:.1f}")
PY
}

echo "| board | router | completion % | DRC errors | vias | wirelength mm | wall s |"
echo "|---|---|---:|---:|---:|---:|---:|"
row() { echo "| $1 | $2 | $3 | $4 | $5 | $6 | $7 |"; }

for b in "${BOARDS[@]}"; do
  name=$(basename "$b" .kicad_pcb)
  d="$WORK/$name"
  mkdir -p "$d"
  cp "$b" "$d/in.kicad_pcb"
  if [ -f "${b%.kicad_pcb}.kicad_pro" ]; then
    for o in in ferroute freerouting; do cp "${b%.kicad_pcb}.kicad_pro" "$d/$o.kicad_pro"; done
  fi

  # --- ferroute (fixed placement: no --allow-move) ---
  t0=$(now)
  "$BIN" route "$d/in.kicad_pcb" -o "$d/ferroute.kicad_pcb" --threads "$THREADS" >/dev/null 2>&1 || true
  t1=$(now)
  read -r comp errs vias wl < <(metrics "$d/ferroute.kicad_pcb")
  row "$name" ferroute "$comp" "$errs" "$vias" "$wl" "$(printf '%.1f' "$(echo "$t1 - $t0" | bc -l)")"

  # --- Freerouting (export, route, import) ---
  "$BIN" dsn export "$d/in.kicad_pcb" -o "$d/in.dsn" >/dev/null
  # Settings a DSN cannot carry (no neckdown/fanout, hole clearance, pours as obstacles).
  FR_SETTINGS=$(cargo run -q --release -p ferroute-dsn --example freerouting_args -- "$d/in.kicad_pcb" | tr "\n" " ")
  export FR_SETTINGS
  t0=$(now)
  timeout "$TIMEOUT" bash -c 'fr "$0" "$1"' "$d/in.dsn" "$d/out.ses" >"$d/freerouting.log" 2>&1 || true
  docker kill "ferroute-fr-$$" >/dev/null 2>&1 || true
  t1=$(now)
  secs=$(printf '%.1f' "$(echo "$t1 - $t0" | bc -l)")
  if [ -s "$d/out.ses" ] && "$BIN" dsn import-ses "$d/in.kicad_pcb" "$d/out.ses" -o "$d/freerouting.kicad_pcb" >"$d/import.log" 2>&1; then
    read -r comp errs vias wl < <(metrics "$d/freerouting.kicad_pcb")
    row "$name" "freerouting 2.5.0" "$comp" "$errs" "$vias" "$wl" "$secs"
  else
    row "$name" "freerouting 2.5.0" "n/a (see $d/freerouting.log)" - - - "$secs"
  fi
done
echo
echo "threads: $THREADS (ferroute --threads); Freerouting passes: $PASSES; work dir: $WORK"
