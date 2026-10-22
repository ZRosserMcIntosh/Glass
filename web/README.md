# Browser build

`build_wasi.sh` compiles the core and `glass_web.cpp` (a C ABI) with wasi-sdk
(no exceptions, no RTTI, reactor model, `-O3 -flto`) twice: `glass.wasm` with
WebAssembly SIMD and `glass-scalar.wasm` without, for browsers that lack it.
The core's sums keep their order and wasm has no fused multiply-add, so the two
give bit-identical results. It copies the loader as `glass.mjs` and merges
`data/*.json` into `library.json`.

```js
import createGlassModule from './glass.mjs';
const G = await createGlassModule();
G.load(await (await fetch('library.json')).text());   // returns error strings
const cat = G.catalog();                              // glazings and impactors, with provenance
G.gls_new_pane(glazingIndex, tier, seed);
G.gls_fire(impactorIndex, x, y, speed, yaw, pitch, fromBack, lead);
G.gls_advance(t, maxSteps);                           // whole steps until time >= t
```

Numbers come back in typed arrays; the small ones (`state`, `plies`,
`impactors`) are copies, the big ones are views into the module's memory, valid
until the next call. `G.memory` is the module's memory, for a host that reads
the layouts itself (a worker copying them out, say).

| Call | Layout |
|---|---|
| `G.state()` | 32 doubles: time, quiet, glass plies, impactors, fragment records, fragment revision, cracked, penetrated, barrier intact, open area, visibility, max bulge, free pieces, thickness, width, height, fundamental Hz, highest mode Hz, modes x, y, crack cell, events, last step, when pieces were last stepped, dice sprays, crazed dice still hanging in the frame, crushed discs, blast pressure now (Pa) |
| `G.plies(n)` | 12 per glass ply: z0, z1, segments, cell, fragmented, dicing, central tension, surface compression, energy-balance dice size, drawn dice spacing, ply index, running tips |
| `G.impactors(n)` | 20 per impactor: position, velocity, travel axis, phase (0 flight, 1 contact, 2 ballistic, 3 membrane, 4 done), force, squash, contact radius, radius, length, kind, side, hit x, hit y, pushing |
| `G.report(k)` | `{v, outcome}`: speed in/out, energy in/absorbed, peak force, contact start/time, max stress, depth, bulge, broke, penetrated, rebounded, embedded, diced |
| `G.cracks(g, from)` | float32, 7 per new segment of glass ply g: ax, ay, bx, by, birth time of each end, kind |
| `G.fragmentShapes(from)` | new pieces: index, ply, chip, thickness, outline about the centroid, holes (loops cut out of it) |
| `G.fragmentMap(ply)` | which piece each crack-grid cell of a glazing ply belongs to (`-1` none), rows from the bottom |
| `G.fragmentPoses(n)` | float32, 12 per piece: position, quaternion (w, x, y, z), velocity, state (0 held, 1 pending, 2 free, 3 resting, −1 replaced), birth time |
| `G.sprays(from)` | float32, 15 per new spray: position, velocity, dice count, dice size, thickness, time, ply, fragment index, spin (a piece drawn coarser than the real dice that came apart into them; its last pose stays in `fragmentPoses`) |
| `G.crushes(from)` | float32, 6 per new crushed disc: glass ply, x, y, radius, time, kind (0 contact damage or cone, 1 projectile); comminuted glass, white to the eye |
| `G.field(g, nx, ny)` | float32: deflection, then the front and back largest principal stress of ply g on an nx × ny lattice |
| `G.events(from)` | the engine's log lines |
| `gls_count_fragments(g, x, y, size)` | pieces with their centroid in the square (ECE R43 count) |
| `gls_bench(n)` | fixed work for a device benchmark |
| `gls_select(slot)` | picks which of 32 pane slots every other call works on (a building's windows); each keeps its own pane and clock |
| `gls_free_pane()` | drops the selected slot's pane |
| `gls_blast(arrival, peak, duration, decay, gasPeak, gasRise, gasTau)` | a blast wave on the pane: Friedlander shock (reflected peak Pa, positive phase s) arriving at engine time `arrival`, plus a confined charge's gas pressure rising and venting; + pushes toward the back face |

Positions are in the pane frame: x along the width, y up from the bottom edge,
z into the glass from the struck face, metres.
