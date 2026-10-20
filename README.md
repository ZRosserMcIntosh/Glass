# Glass Dynamics

A physically grounded glazing fracture engine in C++20 for the São Paulo open
world. It breaks annealed, heat-strengthened, tempered, chemically strengthened,
laminated, bullet-resistant and aircraft glass under balls, hail, tools,
bodies, birds and bullets, and hands the cracks, pieces and forces to whatever
draws them. It has no renderer and no game-engine dependency; glass, layups and
impactors are JSON with a provenance note on every value. The same core runs
natively and in the browser as WebAssembly, where it drives the lab at
[zrossermcintosh.com/sao-paulo/glass](https://zrossermcintosh.com/sao-paulo/glass/).

It is the glazing partner of the two other engines of the project:
**OpenRoad Dynamics** (vehicles: windscreens, door glass, the crash dummies'
head strikes) and **Aviation Dynamics** (windshields, bird strikes, cabin
pressure). [docs/integration.md](docs/integration.md) shows the coupling.

## What it does

| Area | What is implemented | Evidence |
|---|---|---|
| Plate dynamics | The layup as one Kirchhoff plate: Bennison-Wölfel shear coupling through the interlayers, Navier modes of the simply supported pane advanced exactly (any step, 50 ns to ms), the logarithmic stress peak under a concentrated load added analytically within the modal resolution and limited to the flexural reach √(t·√(D/ρh)), and Zener's infinite-plate impedance 8√(Dρh) under the contact above the highest mode | `tests/test_foundations.cpp` (Navier deflections, frequencies, a 600 × 600-mode reference, energy conservation) |
| Strength | Weibull flaw populations per face and edge (EN 16612 edge factor), the stress-corrosion rate law (a microsecond load meets glass about twice as strong as a minute-long one), residual-stress profiles: parabolic thermal tempering, linear ion exchange | `tests/test_foundations.cpp` |
| Contact | Hertz with Thornton's elastic-plastic flattening and Flores' viscoelastic damping; cone indentation at the glass hardness for sharp tips (Lawn-Evans-Marshall median cracks); flat punches that crush (timber, brick); Wilbeck's Hugoniot and steady-flow pressures for birds; Hertzian cones by Auerbach's law, blocked by surface compression | `tests/test_foundations.cpp`, `tests/test_fracture.cpp` |
| Cracks | Through-cracks grown tip by tip at up to the terminal crack speed (Freund), turned by the maximum hoop-stress kink criterion (Erdogan-Sih), forked by the fracture-mirror relation σ√r = A_b, shielded by neighbours, arrested (and restarted) by K_Ic, stopped at existing cracks; radial stars, concentric rings at the roots of the bent wedges, petals snapping round the impactor, Hertzian cones, crush zones | `tests/test_fracture.cpp` |
| Tempered glass | Core tension forks running cracks into dice whose size comes from the energy balance (stored strain energy against new surface); the pane crazes at 1.5 km/s, then comes down procedurally: dice held only by friction with their neighbours and the frame let go from the hole up, in slabs that burst into dice on the floor, leaving a fringe in the frame. Fragment counts fall in the ECE R43 window | `tests/test_fracture.cpp` |
| Laminates | Broken plies hand the load to the interlayer: an axisymmetric membrane with 1/r tension that tears at its rim; post-breakage bulge; plies on the compression side keep carrying load across closed cracks | `tests/test_fracture.cpp` (ECE R43 2.26 kg ball, road stone, window punch, head from inside) |
| Ballistics | A crushed cone through each glass ply at 68°, plug momentum, soft noses mushrooming (Taylor), interlayer tearing and a polymer backing; comminution energy fitted to UL 752 classes. Oblique rounds meet each layer at its slant thickness and glance off near grazing; a round into an earlier crater only has to push the rubble aside. Every crushed disc is reported (white to the eye, and it takes away the view) | `tests/test_fracture.cpp` (Levels 1, 3, 8; multi-shot; obliquity, ricochet, craters; an 80-round barrage on an armoured windscreen) |
| Pieces | Enclosed regions cut out by flood fill and traced into polygons, flown as rigid bodies (gravity, drag, floor friction), broken again by hard landings; pieces wedged in the frame or gripped by the gasket stay | `tests/test_fracture.cpp` |
| Hosts | Pane mounts in the OpenRoad (ISO 8855) and Aviation (FRD) body frames, host-frame strikes, and a status back (visibility, barrier, open area, bulge) | `tests/test_bridge` cases in `tests/test_fracture.cpp` |
| Any device | Five tiers (potato to ultra) change resolution, not physics: plate modes, crack grid, tip and piece budgets. Results do not depend on how the host chops its frames | `tests/test_fracture.cpp` (same outcome on every tier; frame-rate independence) |
| Browser | WebAssembly build (wasi-sdk, no exceptions or RTTI, 310 KB) with a C ABI; Node smoke test | `web/`, `web/smoke_test.mjs` |

53 test cases, on Linux (gcc, clang, sanitizers), macOS, Windows and WASM in CI.
What each scenario gives against its standard is in
[docs/validation.md](docs/validation.md); how far each model goes, and its
main gap, in [docs/physics.md](docs/physics.md).

## Build and run

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/tests/gls_tests                  # 48 test cases
./build/tests/gls_tests "UL 752"         # a filter
./build/glassdemo validate               # the scenario table
./build/glassdemo run windscreen steel_ball_2260 8.86 --svg out.svg   # one shot, its event log and an SVG of the cracks
./build/glassdemo run tempered_6 window_punch --tier 0                 # the same physics at potato resolution
./build/glassdemo list | bench
```

Options: `GLS_BUILD_TESTS`, `GLS_BUILD_TOOLS`, `GLS_WARNINGS_AS_ERRORS`.

The browser build needs [wasi-sdk](https://github.com/WebAssembly/wasi-sdk);
version 25 is tested.

```sh
WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh   # web/dist: glass.wasm, glass.mjs, library.json
node web/smoke_test.mjs
```

## Using it

```cpp
#include "gls/library.hpp"
#include "gls/pane.hpp"

gls::Library lib;
lib.load(materialsJson); lib.load(glazingsJson); lib.load(impactorsJson);
lib.resolve();

gls::Pane pane(*lib.glazing("windscreen"), gls::budgetForTier(2), /*seed*/ 1);
pane.fire(*lib.impactor("steel_ball_2260"), {0.72, 0.42}, 8.86);
while (!pane.quiet()) pane.advance(pane.time() + 1.0 / 60);   // whole internal steps

for (const auto& seg : pane.glass()[0].net.segments()) { /* a, b, birth times ta, tb */ }
for (const auto& f : pane.fragments().fragments())    { /* outline, pos, rot, state */ }
const gls::ImpactReport& r = pane.impactors()[0].report;      // outcome, forces, energy, depth
const gls::PaneStatus s = pane.status();                       // visibility, barrier, open area
```

## Repository layout

```
src/core/      gls_core: the engine-independent fracture core (namespace gls)
tools/glassdemo/  headless runs, the validation table, SVG export, benchmark
tests/         dependency-free test harness and suites
data/          materials, glazing layups and impactors (JSON, with provenance)
web/           C ABI, WASM build, JS loader, smoke test
docs/          physics, validation, integration, roadmap
```

## Principles

Shared with the vehicle and flight cores:

- **Nothing canned.** Every crack, piece and force comes out of the mechanics;
  the renderer only draws.
- **Lower tiers lower resolution, not physics.** A weak device gets fewer modes
  and coarser cracks, and the clock slows down rather than skipping physics.
- **Scientific honesty.** Every value is tagged `published`, `calibrated` or
  `estimated` with its source. The few calibration constants are listed with
  what they were fitted to. Layups for bullet-resistant and aircraft glazing are
  representative builds, not certified products.

## Licence

MIT. Test-standard names (ECE R43, ANSI Z26.1 and Z97.1, 16 CFR 1201, UL 752,
ASTM E1996, 14 CFR 25.775) describe the published test conditions the scenarios
follow; nothing here is a certification or a substitute for one.
