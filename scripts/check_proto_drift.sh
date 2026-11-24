#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Nightly proto drift check. Downloads the upstream KiCad API protobufs that ferroute-ipc
# mirrors (listed in crates/ferroute-ipc/proto.lock) at the KiCad ref each was mirrored from
# and compares their sha256 with the lock. On drift it names each changed file and exits 1.
#
#   scripts/check_proto_drift.sh           verify (nightly CI)
#   scripts/check_proto_drift.sh --update  rewrite the lock after reviewing a drift
#
# Exit codes: 0 in sync, 1 drift, 2 download or usage error.
set -euo pipefail
cd "$(dirname "$0")/.."

lock=crates/ferroute-ipc/proto.lock
api=https://gitlab.com/api/v4/projects/kicad%2Fcode%2Fkicad/repository/files

update=0
case "${1:-}" in
  "") ;;
  --update) update=1 ;;
  *) echo "usage: $0 [--update]" >&2; exit 2 ;;
esac

[ -r "$lock" ] || { echo "missing $lock" >&2; exit 2; }

# Lock entries are "<sha256>  <path>  <ref>" lines; paths are relative to the KiCad repository root.
entries=$(awk 'NF == 3 && $1 !~ /^#/ { print $1, $2, $3 }' "$lock")
[ -n "$entries" ] || { echo "$lock: no entries" >&2; exit 2; }

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
: > "$tmp/results"

drift=0
i=0
while read -r want path pin; do
  i=$((i + 1))
  dest="$tmp/$i.proto"
  enc=${path//\//%2F}
  if ! curl -sSfL --retry 3 --retry-all-errors -o "$dest" "$api/$enc/raw?ref=$pin" || [ ! -s "$dest" ]; then
    echo "fetch failed: $path (KiCad ref $pin)" >&2
    exit 2
  fi
  got=$(sha256sum "$dest" | cut -d' ' -f1)
  printf '%s  %s  %s\n' "$got" "$path" "$pin" >> "$tmp/results"
  if [ "$update" = 0 ] && [ "$got" != "$want" ]; then
    echo "DRIFT: $path changed upstream at KiCad ref $pin (locked ${want:0:12}, upstream ${got:0:12})"
    drift=1
  fi
done <<< "$entries"

if [ "$update" = 1 ]; then
  { grep '^#' "$lock" || true; cat "$tmp/results"; } > "$tmp/lock.new"
  cat "$tmp/lock.new" > "$lock"
  echo "updated $lock:"
  cat "$tmp/results"
  exit 0
fi

if [ "$drift" = 1 ]; then
  echo "proto drift: upstream api/proto changed since $lock was written." >&2
  echo "Review the change against crates/ferroute-ipc/src/proto.rs, then run scripts/check_proto_drift.sh --update." >&2
  exit 1
fi
echo "proto drift: ok ($i files match $lock)"
