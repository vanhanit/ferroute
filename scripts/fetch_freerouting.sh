#!/usr/bin/env bash
# SPDX-License-Identifier: AGPL-3.0-only
# Downloads the Freerouting executable jar from Maven Central into a cache directory
# OUTSIDE the repository and verifies its published sha1. Freerouting is GPL-3.0: it is
# only ever run as an external tool, never vendored or linked. Prints the jar path.
set -euo pipefail
VERSION="${FREEROUTING_VERSION:-2.5.0}"
CACHE="${FERROUTE_CACHE:-${XDG_CACHE_HOME:-$HOME/.cache}/ferroute}/freerouting"
BASE="https://repo1.maven.org/maven2/app/freerouting/freerouting/$VERSION"
JAR="$CACHE/freerouting-$VERSION-executable.jar"
mkdir -p "$CACHE"
if [ ! -f "$JAR" ]; then
  curl -fsSL --retry 5 --retry-delay 10 -o "$JAR.part" "$BASE/freerouting-$VERSION-executable.jar"
  curl -fsSL --retry 5 --retry-delay 10 -o "$JAR.sha1" "$BASE/freerouting-$VERSION-executable.jar.sha1"
  want=$(tr -d ' \n' <"$JAR.sha1" | cut -c1-40)
  got=$(sha1sum "$JAR.part" | cut -c1-40)
  if [ "$want" != "$got" ]; then
    echo "sha1 mismatch for $JAR: want $want got $got" >&2
    rm -f "$JAR.part"
    exit 1
  fi
  mv "$JAR.part" "$JAR"
fi
echo "$JAR"
