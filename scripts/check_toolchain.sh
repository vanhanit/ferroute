#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Fails if the Rust toolchain pinned in rust-toolchain.toml disagrees with the Docker base
# image or the workspace `rust-version` (major.minor). Run by `make lint` and CI `fmt`.
set -euo pipefail
cd "$(dirname "$0")/.."
pin=$(sed -n 's/^channel *= *"\([0-9.]*\)".*/\1/p' rust-toolchain.toml)
docker=$(sed -n 's/^FROM rust:\([0-9.]*\)-.*/\1/p' Dockerfile | head -n1)
msrv=$(sed -n 's/^rust-version *= *"\([0-9.]*\)".*/\1/p' Cargo.toml)
fail=0
[ -n "$pin" ] || { echo "rust-toolchain.toml: no channel pin"; exit 1; }
if [ "$docker" != "$pin" ]; then
  echo "Dockerfile base rust:$docker != rust-toolchain.toml $pin"; fail=1
fi
if [ "${pin%.*}" != "$msrv" ]; then
  echo "Cargo.toml rust-version $msrv != rust-toolchain.toml ${pin%.*}"; fail=1
fi
[ "$fail" = 0 ] && echo "toolchain aligned: $pin"
exit "$fail"
