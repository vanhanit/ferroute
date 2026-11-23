#!/usr/bin/env bash
# Cross-compiles and packages ferroute release archives for every supported target from
# one Linux host (zig as the cross linker via cargo-zigbuild). Runs inside the Docker
# `cross` stage (`make dist`) and in the release workflow.
# Usage: scripts/dist.sh <version-tag> [target...]
set -euo pipefail
tag="${1:?usage: $0 <tag> [target...]}"
shift || true
targets=("$@")
if [ ${#targets[@]} -eq 0 ]; then
  targets=(
    x86_64-unknown-linux-musl
    aarch64-unknown-linux-musl
    x86_64-apple-darwin
    aarch64-apple-darwin
    x86_64-pc-windows-gnu
  )
fi
root="$(cd "$(dirname "$0")/.." && pwd)"
out="$root/dist"
target_dir="${CARGO_TARGET_DIR:-$root/target}"
mkdir -p "$out"
for t in "${targets[@]}"; do
  echo "==> $t"
  cargo zigbuild --release --locked -p ferroute-cli --target "$t" --manifest-path "$root/Cargo.toml"
  ext=""; [[ "$t" == *windows* ]] && ext=".exe"
  name="ferroute-${tag}-${t}"
  stage="$(mktemp -d)"
  mkdir -p "$stage/$name"
  cp "$target_dir/$t/release/ferroute$ext" "$stage/$name/"
  cp "$root"/{LICENSE,NOTICE,COMMERCIAL.md,README.md} "$stage/$name/"
  if [[ "$t" == *windows* ]]; then
    archive="$name.zip"
    (cd "$stage" && python3 -I -c 'import sys,zipfile,pathlib
src=pathlib.Path(sys.argv[2]); z=zipfile.ZipFile(sys.argv[1],"w",zipfile.ZIP_DEFLATED)
[z.write(p, p.relative_to(src.parent)) for p in sorted(src.rglob("*"))]; z.close()' "$out/$archive" "$name")
  else
    archive="$name.tar.gz"
    tar -C "$stage" -czf "$out/$archive" "$name"
  fi
  (cd "$out" && sha256sum "$archive" > "$archive.sha256")
  rm -rf "$stage"
  echo "    $out/$archive"
done
