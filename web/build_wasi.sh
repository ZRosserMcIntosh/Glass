#!/bin/sh
# Glass Dynamics — build the browser demo's WebAssembly module with wasi-sdk.
# SPDX-License-Identifier: MIT
# Usage: WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh [output-dir]   (default: web/dist)
# Tested with wasi-sdk 25. The core builds without exceptions or RTTI here.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT"
EXPORTS=""
for f in gls_text gls_text_length gls_load gls_resolve gls_error_text gls_catalog gls_new_pane gls_set_floor \
         gls_fire gls_advance gls_time gls_last_step gls_state gls_plies gls_impactors gls_report gls_events \
         gls_cracks gls_fragment_shapes gls_fragment_poses gls_sprays gls_crushes gls_fragment_map gls_field gls_count_fragments gls_bench \
         gls_select gls_free_pane gls_blast gls_new_pane_sized gls_fire_as gls_drop_fragment gls_fragment_sheet gls_carried_sheet gls_sheet_mass; do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O2 -fno-exceptions -fno-rtti -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/src/core/include" \
  "$HERE/glass_web.cpp" "$REPO"/src/core/src/*.cpp \
  $EXPORTS -Wl,--initial-memory=67108864 -Wl,--max-memory=1073741824 -Wl,--strip-all \
  -o "$OUT/glass.wasm"
cp "$HERE/glass_loader.mjs" "$OUT/glass.mjs"
# One library file for the page: materials, glazings and impactors.
node -e '
const fs = require("fs"), d = process.argv[1];
const out = { materials: [], glazings: [], impactors: [] };
for (const f of ["materials", "glazings", "impactors"]) {
  const j = JSON.parse(fs.readFileSync(`${d}/${f}.json`, "utf8"));
  for (const k of Object.keys(out)) if (j[k]) out[k].push(...j[k]);
}
fs.writeFileSync(process.argv[2], JSON.stringify(out));
' "$REPO/data" "$OUT/library.json"
ls -l "$OUT"
