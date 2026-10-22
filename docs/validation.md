# Validation

What each scenario gives, against the test standard it follows. Produced by
`./build/glassdemo validate` (high tier, seed 1) and checked by
`tests/test_fracture.cpp` on several seeds and on every tier. The standards
define test conditions and pass criteria; the panes here are flat,
representative builds, so a "pass" means the model behaves the way glazing
built to that standard does, not a certification.

## Against standards

| Scenario | Standard and criterion | Model |
|---|---|---|
| 227 g steel ball from 2 m on 4 mm tempered car side glass | UN ECE R43 Annex 5: must not break | Holds (peak 3.3 kN) |
| Window punch on 4 mm tempered side glass | ECE R43 fragmentation: 40–400 pieces in any 50 × 50 mm square | Dices; energy-balance dice 4.0 mm, ≈156 per square |
| 2.26 kg ball from 4 m on a 2.1 / 0.76 PVB / 2.1 windscreen | ECE R43 Annex 6, ANSI Z26.1 test 26: the ball must not pass | Both plies break, the PVB holds: 62 mm bulge |
| The same ball at 20 m/s | (beyond the test) | Tears through |
| Road stone, 6 g at 30 m/s, on the windscreen | Field experience: a star break in the outer ply | Outer ply cracks, inner ply intact, stone bounces |
| Window punch on the windscreen | Laminated glass is why punches don't work on windscreens | Outer ply chipped, barrier intact |
| Occupant head (4.5 kg, 7 m/s) from inside | Windscreen as a restraint | Held |
| 45 kg bag from 48 in, annealed 6 mm | 16 CFR 1201 / ANSI Z97.1: annealed is not safety glazing | Breaks into large shards (ten largest > 10 in²) |
| The same bag, tempered 6 mm | 16 CFR 1201: unbroken, or the ten largest pieces under 10 in² | Holds on some seeds, dices safely on others |
| 2x4 (4.1 kg) at 15.2 m/s on a hurricane laminate | ASTM E1996 missile D: no penetration | Glass cracks, interlayer holds |
| 9 mm FMJ at 380 m/s on a 30 mm all-glass laminate | UL 752 Level 1 | Stopped 13 mm in (third glass ply) |
| .44 Magnum at 430 m/s on 32 mm glass-clad polycarbonate | UL 752 Level 3 | Stopped 19 mm in |
| 7.62 × 51 at 850 m/s on the Level 3 panel | Not rated: should perforate | Perforates, exits at 561 m/s |
| 7.62 × 51 at 850 m/s on 52 mm rifle-rated GCP | UL 752 Level 8 | Stopped 35 mm in |
| Three .44 shots on one Level 3 panel | UL 752 multi-shot | All three stopped |
| 1.8 kg bird at 128.6 m/s on the airliner windshield | 14 CFR 25.775: no penetration | Holds; cabin barrier intact |
| 9 mm submachine-gun rounds (400 m/s) on a 32 mm armoured car windscreen | EN 1063 BR4 / VPAM 4 class: stops 9 mm | Square on: stopped 12 mm in; at 55°: 7.6 mm in; past 72°: ricochets |
| 80 rounds scattered over the armoured windscreen, 50° rake | (beyond the test: the rating is for 3 shots) | None through, barrier intact; a round into an old crater goes 30 % deeper; the driver's view falls to 58 % |

## Other checks

| Scenario | Expected | Model |
|---|---|---|
| 2.26 kg ball from 4 m on 6 mm annealed | Goes through, radial star | Goes through at ~7.5 m/s, 4–7 radials, petal rings round the hole |
| 227 g ball, breaking speed | Annealed < heat-strengthened ≤ tempered | 8.1, 12.4, 12.4 m/s |
| Steel BB at 120 m/s on 6 mm annealed | A conical (Hertzian) hole | Cone hole, plug ejected |
| Baseball at 40 m/s on 4 mm annealed | Breaks the window | Breaks through, keeps ~80 % of its energy |
| 9 mm on 6 mm annealed / on a windscreen | Glass does not stop pistol bullets | Through at 316 / 346 m/s |
| Window punch on tempered 6 mm / car side glass, 2 s later | The pane empties from the hole up, a fringe stays in the frame | Above the hole goes first; 7 % of the dice left, all within 6 cm of the frame |
| BB, 9 mm, .44 Magnum, 7.62 mm through 6 mm annealed | The pane stays in its frame round the hole | Over 99.9 % of the pane held; only chips round the hole fall |
| 00 buckshot (9 pellets, 395 m/s, from 7 m) on 6 mm annealed / on the armoured windscreen | Through plate glass; stopped by handgun-rated armour | All through at 210-240 m/s, a ragged hole larger than the pattern; all stopped in the third ply |
| Same seed twice; another seed | Identical; different | Identical; different |
| Same shot at 60 and 997 host frames per second | Identical | Identical |
| Same shot on every tier | Same outcome | Same outcome (penetrated / held) |

## Large deflection

A pane bent past its own thickness stretches in its plane (membrane action,
[physics.md](physics.md), section 1). The model is checked for centre
deflection and for the largest principal stress of either face, and where it
is, against p a⁴/(E h⁴) (a the shorter side, h the thickness, ν = 0.22, edges
simply supported and free to slide in their plane):

- **ASTM E1300**'s centre deflection of four-sided simply supported glass,
  its fit to the nonlinear analysis behind its charts: w = t exp(r₀ + r₁ x +
  r₂ x²), x = ln(ln(q (a b)² / (E t⁴))), with r₀ = 0.553 − 3.83 AR + 1.11 AR²
  − 0.0969 AR³, r₁ = −2.29 + 5.83 AR − 2.17 AR² + 0.2067 AR³, r₂ = 1.485 −
  1.908 AR + 0.815 AR² − 0.0822 AR³ (AR = long / short side). E1300 publishes
  no stress chart (its load charts are probabilities of breakage), so for
  stress and its location:
- **A converged von Kármán solution** by finite differences on the Airy stress
  function, the method of Vallabhan and Wang's analysis behind E1300, sharing
  no code with the engine: [`tools/vonkarman`](../tools/vonkarman/main.cpp)
  (quarter plate, ghost nodes reflected about the edges and the symmetry
  lines, Picard iteration, banded LU; grids of 32 and 64 intervals per half
  side, Richardson-extrapolated). It reproduces the linear limits
  (w = 0.00406 · 12 (1 − ν²) p a⁴/(E h³), σ = 0.287 p a²/h² for ν = 0.3) and
  E1300's fit to 0.2–1.8 % for p a⁴/(E h⁴) = 40–500 on panes up to 4 : 3 (to
  4 % on 2 : 1, near the fit's own scatter). Its peak stress converges at
  second order (4 : 3 at 400: 62.6, 69.1, 71.4, 72.1 on 8 to 64 intervals).
  Levy's large-deflection tables for simply supported plates (NACA TN 846,
  reproduced in Timoshenko & Woinowsky-Krieger's *Theory of Plates and
  Shells*) are for edges kept straight in their plane, a stiffer condition
  than a gasket's; they could not be retrieved for this check and are not
  compared.

The engine column is the static solution of the plate the pane builds
(`ModalPlate::configureStretch`, 32 bending modes along the shorter side as on
the high tier); 36 stretching modes are the low to high tiers, 49 the ultra
tier. Checked by `tests/test_foundations.cpp`; `./build/vonkarman 1.3333 0.22 32`
prints the reference.

**Square**

| p a⁴/(E h⁴) | w/h reference | E1300 | engine, 36 modes | peak σ a²/(E h²), reference | where | engine, 36 modes | where | engine, 49 modes | centre σ, reference | engine, 36 |
|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 0.447 | — | 0.447 (+0.0 %) | 2.75 | centre | 2.74 (-0.1 %) | centre | 2.74 (-0.2 %) | 2.75 | 2.74 (-0.1 %) |
| 40 | 1.377 | 1.380 (+0.2 %) | 1.377 (+0.0 %) | 8.68 | centre | 8.67 (-0.1 %) | centre | 8.67 (-0.1 %) | 8.64 | 8.65 (+0.1 %) |
| 100 | 2.387 | 2.379 (-0.3 %) | 2.388 (+0.0 %) | 17.02 | corner, 2 / 2 % in | 16.91 (-0.7 %) | corner, 2 / 3 % in | 16.90 (-0.7 %) | 14.27 | 14.35 (+0.5 %) |
| 200 | 3.417 | 3.409 (-0.2 %) | 3.416 (-0.0 %) | 31.13 | corner, 2 / 2 % in | 30.70 (-1.4 %) | corner, 3 / 3 % in | 30.89 (-0.8 %) | 19.40 | 19.57 (+0.9 %) |
| 300 | 4.165 | 4.139 (-0.6 %) | 4.162 (-0.1 %) | 44.56 | corner, 2 / 2 % in | 43.41 (-2.6 %) | corner, 3 / 2 % in | 44.23 (-0.7 %) | 23.06 | 23.32 (+1.1 %) |
| 400 | 4.776 | 4.721 (-1.2 %) | 4.767 (-0.2 %) | 57.73 | corner, 2 / 2 % in | 55.27 (-4.3 %) | corner, 2 / 2 % in | 57.18 (-1.0 %) | 26.10 | 26.49 (+1.5 %) |
| 500 | 5.301 | 5.210 (-1.7 %) | 5.284 (-0.3 %) | 70.71 | corner, 2 / 2 % in | 66.27 (-6.3 %) | corner, 2 / 2 % in | 69.67 (-1.5 %) | 28.78 | 29.36 (+2.0 %) |
| 1000 | 7.262 | 6.962 (-4.1 %) | 7.176 (-1.2 %) | 134.83 | edge, 3 % from a corner | 111.75 (-17.1 %) | edge, 4 % from a corner | 125.62 (-6.8 %) | 39.49 | 41.98 (+6.3 %) |

**4 : 3 (b/a = 1.33, the 0.9 × 1.2 m windows)**

| p a⁴/(E h⁴) | w/h reference | E1300 | engine, 36 modes | peak σ a²/(E h²), reference | where | engine, 36 modes | where | engine, 49 modes | centre σ, reference | engine, 36 |
|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 0.702 | — | 0.702 (+0.0 %) | 3.97 | centre | 3.96 (-0.2 %) | centre | 3.96 (-0.2 %) | 3.97 | 3.96 (-0.2 %) |
| 40 | 1.959 | 1.951 (-0.4 %) | 1.958 (-0.1 %) | 10.56 | centre | 10.54 (-0.2 %) | centre | 10.54 (-0.2 %) | 10.53 | 10.53 (+0.0 %) |
| 100 | 3.264 | 3.312 (+1.5 %) | 3.264 (+0.0 %) | 20.79 | corner, 2 / 2 % in | 20.73 (-0.3 %) | corner, 2 / 2 % in | 20.73 (-0.3 %) | 15.75 | 15.76 (+0.1 %) |
| 200 | 4.612 | 4.697 (+1.8 %) | 4.610 (-0.0 %) | 38.48 | corner, 2 / 2 % in | 38.08 (-1.1 %) | corner, 2 / 2 % in | 38.08 (-1.1 %) | 22.22 | 22.16 (-0.3 %) |
| 300 | 5.593 | 5.669 (+1.4 %) | 5.585 (-0.1 %) | 55.57 | corner, 2 / 2 % in | 53.96 (-2.9 %) | corner, 2 / 2 % in | 53.96 (-2.9 %) | 28.02 | 27.88 (-0.5 %) |
| 400 | 6.389 | 6.438 (+0.8 %) | 6.371 (-0.3 %) | 72.36 | corner, 2 / 1 % in | 68.53 (-5.3 %) | corner, 2 / 1 % in | 68.53 (-5.3 %) | 32.91 | 32.61 (-0.9 %) |
| 500 | 7.071 | 7.083 (+0.2 %) | 7.037 (-0.5 %) | 89.20 | edge, 2 % from a corner | 81.84 (-8.3 %) | corner, 2 / 1 % in | 81.84 (-8.3 %) | 37.21 | 36.66 (-1.5 %) |
| 1000 | 9.588 | 9.373 (-2.2 %) | 9.432 (-1.6 %) | 173.45 | edge, 2 % from a corner | 138.09 (-20.4 %) | edge, 4 % from a corner | 138.09 (-20.4 %) | 54.10 | 51.41 (-5.0 %) |

**2 : 1**

| p a⁴/(E h⁴) | w/h reference | E1300 | engine, 36 modes | peak σ a²/(E h²), reference | where | engine, 36 modes | where | engine, 49 modes | centre σ, reference | engine, 36 |
|---|---|---|---|---|---|---|---|---|---|---|
| 10 | 1.077 | — | 1.077 (+0.0 %) | 5.60 | centre | 5.59 (-0.2 %) | centre | 5.60 (-0.1 %) | 5.60 | 5.59 (-0.2 %) |
| 40 | 3.064 | 2.947 (-3.8 %) | 3.063 (-0.0 %) | 15.20 | centre | 15.10 (-0.6 %) | centre | 15.20 (+0.0 %) | 15.20 | 15.10 (-0.6 %) |
| 100 | 5.168 | 5.071 (-1.9 %) | 5.156 (-0.2 %) | 26.43 | corner, 3 / 2 % in | 25.54 (-3.4 %) | corner, 3 / 2 % in | 26.43 (-0.0 %) | 23.80 | 23.47 (-1.4 %) |
| 200 | 7.314 | 7.314 (+0.0 %) | 7.256 (-0.8 %) | 50.22 | edge, 3 % from a corner | 45.46 (-9.5 %) | x/a 0.21, y/b 0.14 | 48.87 (-2.7 %) | 30.59 | 30.12 (-1.5 %) |
| 300 | 8.852 | 8.930 (+0.9 %) | 8.724 (-1.4 %) | 74.26 | edge, 2 % from a corner | 62.48 (-15.9 %) | x/a 0.20, y/b 0.87 | 69.33 (-6.6 %) | 38.62 | 39.56 (+2.4 %) |
| 400 | 10.090 | 10.230 (+1.4 %) | 9.880 (-2.1 %) | 98.25 | edge, 2 % from a corner | 77.04 (-21.6 %) | x/a 0.19, y/b 0.87 | 88.14 (-10.3 %) | 45.89 | 47.78 (+4.1 %) |
| 500 | 11.142 | 11.333 (+1.7 %) | 10.845 (-2.7 %) | 121.96 | edge, 2 % from a corner | 89.90 (-26.3 %) | x/a 0.19, y/b 0.87 | 105.79 (-13.3 %) | 52.30 | 55.12 (+5.4 %) |
| 1000 | 14.990 | 15.339 (+2.3 %) | 14.264 (-4.8 %) | 240.63 | edge, 2 % from a corner | 139.85 (-41.9 %) | x/a 0.19, y/b 0.87 | 179.35 (-25.5 %) | 77.13 | 83.70 (+8.5 %) |

Under uniform pressure only the modes odd both ways respond, so what counts
is how many of those the box of stretching modes holds: on the 4 : 3 pane the
ultra tier's 6 × 8 box holds the same ones as the 5 × 7 box of the others.

What it shows: up to about 20 the pane is the linear plate. By 100 the peak
stress has left the centre for the corners, 2–3 % of the side in along the
diagonal, and above about 500 it sits on the long edge next to the corner; at
400 a square pane deflects a quarter as far as the linear plate (18.5 h) and
its peak stress is half the linear plate's. The engine's deflection stays
within 1 % to 500 (3 % on 2 : 1) and its centre stress within 2 %; its peak
stress, and where it is, within 3 % up to 300 on panes to 4 : 3 and to 100 on
2 : 1, falling behind beyond (the corner needs higher modes than it lets
stretch). Coarser tiers bend with fewer modes and smooth the corner peak more
(the low tier's 16: −9 % at 400 on the square).

**Swinging.** Let go from a static bend of twice its thickness, a 2 mm
0.30 × 0.25 m pane rings at 1.2 times its small-swing frequency, the same
within 1 % for pane steps of 20 µs and 0.5 ms, its energy held within 5 %
over 0.1 s (test). Under a tone (picture glass, 2 mm, 135 Hz small-swing
fundamental):

| Tone | Linear plate | Model |
|---|---|---|
| 135 dB held on the small-swing note | 7 thicknesses of swing | 0.8 thickness: detuned by its own stiffening |
| 135 dB gliding up from the note, 27 Hz/s | — | rides the climbing peak to 3.5 thicknesses at 206 Hz, then drops off it |
| 145 dB held on the note, or 20 % flat | breaks in a fraction of a second | holds |
| 145 dB gliding up, 34 Hz/s | — | breaks after 2.6 s at 1.65 times the note (~220 Hz), on every tier and frame rate |
| 125 dB gliding up | — | holds |

**Blasts.** A 0.9 × 1.2 m pane of 6 mm fully tempered glass under a 150 kPa
reflected shock and 60 kPa of gas (p a⁴/(E h⁴) ≈ 420 under the gas alone)
fails 4.8–5.5 ms after the shock and leaves at 26–32 m/s as dice (seeds 1–3,
low and high tiers); fully tempered glass of that size fails near 30 kPa (ASTM
E1300, about 4 × annealed). A uniform (Berger-type) tension, tried first, let
it survive.

## Unit checks

Simply supported plate: fundamental frequency (exact), centre deflection under
a point load (0.0116 F a²/D) and under pressure (0.00406 p a⁴/D) within 1 %;
bent past its thickness, deflection, peak stress and its place against the
von Kármán reference above;
the near-field correction within 4 % of a 600 × 600-mode reference; the exact
modal step conserves energy to 10⁻⁹ at mixed step sizes. Hertz contact time on
a rigid wall within 1 % of 2.94 δ_max/v; restitution of the damped contact
within 0.1 of the target; Wilbeck impulse within 12 % of m v; median-crack
scaling c ∝ F^(2/3); tempering and ion-exchange profiles self-balanced.

## Cost

`./build/glassdemo bench` (one core of the development container; four
scenarios of 0.3 s each):

| Tier | Total |
|---|---|
| potato | 0.22 s |
| low | 0.67 s |
| medium | 1.3 s |
| high | 3.2 s |
| ultra | 9.7 s |

Membrane action adds 5–40 % here (most on the lowest tiers, where the plate
was cheapest), and only while a pane swings past a quarter of its thickness:
a tone holding a big swing costs 2–4 times what the linear plate did. Each
new shape of pane builds its stretch tables once (20–100 ms).

In the browser the same work is spread over the slow-motion playback; when a
device cannot keep up the clock slows instead of skipping physics.
