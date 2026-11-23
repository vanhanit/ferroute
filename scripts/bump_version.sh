#!/usr/bin/env bash
# Sets the workspace version (and the matching version requirements of internal
# ferroute-* path dependencies), then refreshes Cargo.lock.
# Usage: scripts/bump_version.sh 0.2.0
set -euo pipefail
ver="${1:?usage: $0 X.Y.Z}"
[[ "$ver" =~ ^[0-9]+\.[0-9]+\.[0-9]+([-+].*)?$ ]] || { echo "not a SemVer version: $ver" >&2; exit 2; }
root="$(cd "$(dirname "$0")/.." && pwd)"
manifest="$root/Cargo.toml"
sed -i -E '/^\[workspace\.package\]/,/^\[/ s/^version = "[^"]*"/version = "'"$ver"'"/' "$manifest"
sed -i -E 's/^(ferroute-[a-z]+ = \{ path = "[^"]*", version = )"[^"]*"/\1"'"$ver"'"/' "$manifest"
(cd "$root" && cargo update --workspace --quiet)
grep -q "^version = \"$ver\"" "$manifest"
echo "workspace version is now $ver"
