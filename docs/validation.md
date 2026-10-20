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

## Unit checks

Simply supported plate: fundamental frequency (exact), centre deflection under
a point load (0.0116 F a²/D) and under pressure (0.00406 p a⁴/D) within 1 %;
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
| potato | 0.17 s |
| low | 0.44 s |
| medium | 0.98 s |
| high | 2.7 s |
| ultra | 8.0 s |

In the browser the same work is spread over the slow-motion playback; when a
device cannot keep up the clock slows instead of skipping physics.
