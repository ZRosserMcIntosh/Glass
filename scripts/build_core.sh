#!/bin/sh
# Glass — build the flight core (the core/ submodule, Aviation Dynamics) to
# WebAssembly and stage it with the glass-profile aircraft in dist/.
# SPDX-License-Identifier: MIT
# Usage: WASI_SDK=/path/to/wasi-sdk scripts/build_core.sh   (wasi-sdk 25, as the core's CI)
set -eu
HERE=$(cd "$(dirname "$0")/.." && pwd)
OUT="$HERE/dist"
CORE="$HERE/core"
[ -f "$CORE/web/build_wasi.sh" ] || { echo "core/ is empty: git submodule update --init" >&2; exit 1; }
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT/core"
sh "$CORE/web/build_wasi.sh" "$OUT/core"
# The aircraft whose panel profile is a glass cockpit.
for f in "$CORE"/data/aircraft/*.json; do
  if grep -q '"profile": *"business_jet_glass"' "$f"; then cp "$f" "$OUT/core/"; fi
done
