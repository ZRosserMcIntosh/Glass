# Glass

The glass cockpit for [Aviation Dynamics](https://github.com/ZRosserMcIntosh/Aviation-Dynamics):
a primary flight display, a navigation display and an EICAS, and the avionics
computer behind them. It covers the six aircraft whose panel profile is
`business_jet_glass` (Top G, Phenom 300E, Citation CJ4, Learjet 45, CRJ700,
737-800).

Aviation Dynamics remains the flight core: aerodynamics, engines, systems,
sensors and instruments, compiled to WebAssembly. Everything glass lives here:
what the displays compute from the sensors (the "glass physics") and how they
draw it. The core is a git submodule (`core/`), pinned to a commit, and Glass
reads only its documented C ABI (`core/web/README.md`).

```
core/ (Aviation Dynamics)            Glass
  flight model, sensors,     ──►   src/avionics/   air data computer, speed tape physics,
  instruments, GPS, systems         (pure JS)        wind triangle, FPV, alerting, CAS, navigation
  avd_state / avd_cockpit /                         │
  avd_jet / avd_gps (WASM)                          ▼
                                   src/display/    PFD, ND, EICAS canvas renderers
                                   src/app.js      browser host: loop, keys, panel
```

## What the avionics compute

The displays never draw the core's truth channels. The avionics computer
(`src/avionics/glass.js`) takes the sensed readouts each frame and derives:

| Module | Computes |
|---|---|
| `air_data.js` | Pressure altitude by inverting the core altimeter's own ISA law (both the troposphere and the isothermal stratosphere), then Mach from impact and static pressure, TAS from the SAT, and TAT. Also the CAS of a Mach number, which gives the MMO crossover. |
| `speeds.js` | The top barber pole: the lowest of VMO, the CAS of MMO, VFE (flaps out) and VLE (gear not up). Low-speed bands: Vs1g·√n for stall and Vs1g·√(1.3 n) for minimum manoeuvre. V2 = 1.13 VSR (25.107), VREF = 1.23 VSR0 (25.125), VAPP = VREF + half the headwind (5–20 kt). A 10 s speed trend, Mach display hysteresis and overspeed. |
| `flight_path.js` | The wind from the triangle: GPS ground velocity minus TAS along the true heading. Head- and crosswind components, the flight path vector (inertial FPA and drift) and a 6 s altitude trend. |
| `navigation.js` | The HSI source (LOC, GPS or VOR, or AUTO). Deviations in dots, magnetic course and bearing. The heading-up map projection, route and runway geometry, and ETE and fuel to the waypoint. |
| `alerting.js` | Altitude alerting (900 ft approach, 200 ft capture, deviation), BARO/RA minimums, and the radio-altitude callouts. |
| `eicas.js` | Engine limit colours and crew alerting: warnings, cautions, advisories and memos. Master warning and caution latch until acknowledged, cautions are inhibited during take-off (80 kt to 400 ft), and some conditions must persist before they post. |

Every formula, source and estimate is listed in [docs/avionics.md](docs/avionics.md).

## Build and run

```sh
git clone --recurse-submodules https://github.com/ZRosserMcIntosh/Glass.git && cd Glass
export WASI_SDK=/path/to/wasi-sdk-25.0     # https://github.com/WebAssembly/wasi-sdk/releases (wasi-sdk 25)
npm run build                               # core → dist/core/avd.wasm; page → dist/glass.html (one self-contained file)
npm test                                    # avionics unit tests (no build needed)
npm run test:core                           # the avionics flown against the real flight core, every glass aircraft
npm run test:browser                        # headless Chromium check of dist/glass.html (needs playwright)
```

Open `dist/glass.html` in a browser.

Keys:

- **Stick and rudder:** <kbd>W</kbd>/<kbd>S</kbd> pitch, <kbd>A</kbd>/<kbd>D</kbd> roll, <kbd>Q</kbd>/<kbd>E</kbd> rudder or tiller.
- **Thrust and trim:** <kbd>1</kbd>–<kbd>0</kbd> set thrust, <kbd>+</kbd>/<kbd>−</kbd> fine-tune it, <kbd>[</kbd>/<kbd>]</kbd> trim.
- **Configuration:** <kbd>G</kbd> gear, <kbd>F</kbd>/<kbd>V</kbd> flaps down/up, <kbd>B</kbd>/<kbd>N</kbd> speedbrake out/in.
- **Brakes:** <kbd>Space</kbd> brakes, <kbd>P</kbd> parking brake.
- **Displays:** <kbd>Z</kbd>/<kbd>X</kbd> map range, <kbd>Enter</kbd> acknowledge the master lights, <kbd>R</kbd> restart, <kbd>Esc</kbd> pause.

The panel also sets the selected altitude, the minimums, the navigation source, the baro setting and the destination. The destination tunes the ILS and builds the GPS route in the core.

## Updating the core

```sh
git -C core fetch && git -C core checkout <commit>
npm run build && npm run test:core && npm run test:browser
git add core && git commit -m "core: <commit>"
```

`src/avionics/readout.js` names every index Glass reads from the core's
arrays and checks the core's ABI version (`EXPECTED_CORE_ABI`). If the core
bumps `kAbiVersion`, update the index tables there and the constant together.

## Licence

MIT, as the core.
