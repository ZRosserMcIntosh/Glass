# The physics

How a strike becomes cracks and pieces, in the order the core runs it, with the
sources and the calibration constants. Source: [`src/core`](../src/core).

## 1. The layup as a plate

A layup (glass, interlayers, polymer plies) is reduced to one Kirchhoff plate
of mass per area ρh and bending stiffness

```
D = D_layered + Γ (D_mono − D_layered)
```

where `D_mono` is the fully bonded section, `D_layered` the plies sliding
freely, and Γ the Bennison-Wölfel shear-transfer coefficient (ASTM E1300),
averaged over the interlayers, with each interlayer's shear modulus taken at
impact rates (PVB ~20 MPa at milliseconds against <1 MPa for wind loads). The
bending stress at depth z of ply i is

```
σ = E/(1−ν²) · κ · [(z − z_i) + Γ (z_i − z_NA)]
```

so a laminate at impact rates behaves nearly monolithically and the ply
farthest from the load carries the most tension.

The pane is simply supported. Its deflection is a sum of Navier modes
`sin(mπx/a) sin(nπy/b)`, each advanced **exactly** for a force held over the
step (a damped oscillator's closed form), so the step can be 50 ns inside a
bullet strike and milliseconds while the pane rings down, with no stability
limit. Loads enter as uniform discs (factor 2J₁(kr)/kr per mode). Lanczos σ
factors suppress Gibbs ringing in the curvature.

**Near field.** A finite set of modes cannot hold the logarithmic stress peak
under a concentrated load. Within the series' resolution radius
r_m = max(a/M, b/N) (calibrated against a 600 × 600-mode reference to 4 %), the
core adds the isotropic curvature

```
Δκ = F / (4π D) · ln(r_reach / max(r, r_e)),   r_reach = min(r_m, √(t·√(D/ρh)))
```

with Westergaard's equivalent radius r_e for thick-plate contact. The
`√(t…)` limit is the distance a flexural disturbance has covered since the load
began: a 10 µs stone strike cannot bend the plate quasi-statically out to r_m.

**Local impedance.** Under a point load an infinite plate moves like a dashpot,
v = F / (8√(Dρh)) (Zener). Above the highest mode the truncated series is too
stiff, so the contact sees that dashpot, relaxing into the modes with the time
constant 1/(2π f_max). Without it a steel ball on glass reaches about twice the
real force.

## 2. Contact

| Impactor | Model |
|---|---|
| Spheres (balls, hail, bags, headforms) | Hertz F = 4/3 E* √R δ^1.5; past the impactor's flow pressure, Thornton's flattening F = F_y + π p_y R (δ − δ_y) with elastic unloading; viscoelastic loss by Flores et al. (2011), F (1 + 8(1−e)/(5e) δ̇/v₀), good to restitution ~0.3 |
| Sharp tips (window punch, stone corners) | Cone indentation F = π tan²ψ H δ² at the glass hardness (capped by the impactor's own crushing pressure), 40 % elastic recovery |
| Flat ends (2x4, brick faces) | Boussinesq 2E*a δ, capped at the crushing load σ_c π a² |
| Birds | Wilbeck: Hugoniot pressure ρ v (c₀ + k v) decaying over r/c₀ on a growing contact area, then the momentum flux ρ v² A for L/v (porosity lowers c₀) |
| Bullets | Section 6 |

The impactor and plate are stepped together at a tenth of the contact period
estimated from the stiffness at the expected peak.

## 3. When glass breaks

**Strength.** Each face of each glass ply carries a lattice of flaw probes,
each with a Weibull strength over its area (σ_θ at 1 m², 60 s; m = 7), edges at
0.8 × (EN 16612). Under a load lasting t the strength rises by the
stress-corrosion law (60 s / t)^(1/(n+1)), n = 16, capped at 3×. Under the load
itself a fresh draw over the stressed patch (about 1 cm²) is used.

**Residual stress.** Thermal tempering leaves σ(ζ) = CT(1 − 3ζ²) (surface
compression 2CT, zero at 0.211 t); ion exchange a linear layer over its depth,
balanced by a uniform core tension. Surface compression is subtracted from the
applied surface tension; the core tension drives running cracks.

**Initiation.**

- *Bending at the load:* principal stress on either face of each ply against
  the local strength; once a ply on the tension side is broken under the load,
  the intact plies carry the near field on their own (layered, with their own
  stiffness). A broken ply on the compression side still carries load across
  its closed cracks.
- *Hertzian cone:* Auerbach's law F_c = 5×10⁴ N/m × R (with flaw scatter), and
  the edge tension (1−2ν)p₀/2 must also exceed the surface compression. The
  cone (Roesler length, depth 0.375 c) cuts into the struck ply, and the rest
  of it bends harder ((h/(h−c))²). A small hard ball (R < 1.5 h) on a
  monolithic untempered pane drives the cone through and punches a plug.
- *Sharp contact:* the Lawn-Evans-Marshall median crack c = (0.016√(E/H) F /
  K_Ic)^(2/3); in tempered glass, reaching the compression depth releases the
  core.
- *Away from the load:* any probe whose principal stress beats its strength
  (concentric cracks on the impact face, edge breaks); probes inside the
  near-field zone of a load leave it to the load-point model.

**Broken laminate.** Once every glass ply under a blunt load is broken, the
interlayer takes it as an axisymmetric membrane: tension N(r) = N_c R_c / r
from the rim R_c of what presses on it, stiffened ×2 by the glass still bonded
to it, tearing when the rim strain reaches the interlayer's elongation at
break. Three things set how much it takes:

- *Span.* The dished zone starts as the ring round the hit (5 R_c, at least
  0.2 m) and spreads as the bend travels out through the sheet, at
  √(N/μ) with the tension at its edge and μ the laminate's mass per area, out
  to the frame: the geometric mean of the distances to the four edges, so a
  hit near a corner still dishes away from it. A slow heavy push (a 2x4, a
  falling ball) spreads it far; a fast one tears first.
- *Spread.* Broken laminate is not a bare film: its glass pieces stay stuck
  to the interlayer and lock together where they are pushed, so it keeps a
  bending stiffness D ≈ E t h² (interlayer in tension, glass in compression
  over half the laminate) and spreads a load over the bending length √(D/N_y)
  beyond what presses on it, building up at the same bend speed. A stiff
  interlayer spreads it about twice as far as PVB (ionoplast 33 mm, PVB
  9–19 mm in the library's layups): the post-breakage stiffness ionoplast is
  chosen for. Behind a projectile there is nothing to spread it: the
  ballistic model has crushed the glass in its path to powder.
- *Crack bridging.* Where the interlayer spans a crack in the glass and the
  shard edges bite into it, its strain runs above the mean: a per-material
  factor (PVB 3, ionoplast 1.25, calibrated below) on the rim strain against
  the elongation at break.

The membrane's pull at the nearest frame edge, N_c R_c / d, against the
frame's hold (a sealant bond over the bite, or the glazing's own value)
decides whether the edge slides out of its rebate and the whole laminate goes.

## 4. Running cracks

Each crack is a polyline with a birth time at every vertex, grown tip by tip
on a grid that records which crack owns each cell.

- **Driving stress** at the tip: the normal traction across its path on the
  more tensile face, plus core tension. Ring cracks and the radial cracks of a
  star whose impactor still pushes are also driven by the wedge mechanics
  (below).
- **Speed:** K = σ √(π · 0.6 h); below K_Ic the tip goes dormant (and is looked
  at again every 8 µs while the plate moves); above, v = v_t (1 − (K_Ic/K)²)
  (Freund's form with the terminal speed v_t ≈ 1,520 m/s for soda-lime).
- **Direction:** the maximum hoop-stress kink angle for the local K_II/K_I
  (Erdogan & Sih), developed over about a plate thickness of growth, plus a
  small random walk and a slow wander (an Ornstein–Uhlenbeck offset, 0.12 rad
  rms over 4 cm): glass is not perfectly uniform, and real cracks meander a
  few degrees either side of where the stress points them. The kink is scaled
  by the deviatoric share of the stress, dev / (dev + residual), since a
  residual tension pulls equally every way; and a running crack bends no
  tighter than a 0.1 m radius (sharp changes of direction come as forks).
  Without these, heat-strengthened cracks looped round on the plate's
  ring-down and annealed radials ran ruler-straight.
- **Forking:** when σ √(distance since the last fork) passes 3 × the mirror
  relation (A_b = 2.1 MPa√m) and σ exceeds A_b/√(0.6 h). Forks at ±17°
  (±28° in tempered glass).
- **Shielding:** a crack running alongside another within 1.5 h is unloaded by
  it; the shielded one stops, and one cell away they coalesce.
- **Arrest at cracks and edges:** a tip that enters another crack's cell
  stops there, so cracks end on cracks as in real glass.

**Radial count and the crushed zone.** A pane breaking under a load sends out
4 + 4 ln(σ/strength) + 1.5 ln(1 + E/100 J) radials (plus up to 3 at random,
4–18 in all): a ball that barely breaks the glass leaves a handful, a
cannonball a sunburst, never the same twice. Crushed discs are never clean
circles: their edge is ragged (five harmonics, 14 % of the radius).
Inside a crack star the plate is wedges split by its radials, not the
continuous plate the modes describe, so flaw probes there do not fire on the
plate's bending wave (the wedges make their own rings).

**Wedges.** Radial cracks from a load point leave wedges loaded at their tips.
As cantilevers their roots carry 3F/(πh²) on the face toward the impact
(monolithic glass; 15 % of that when bonded to an interlayer), which opens
concentric rings when it beats the strength at the roots, and the same stress
around the radial tips keeps them running. As the impactor pushes through
monolithic glass the wedges' stiffness 2πD/L² falls with the crack length L,
so the force collapses as fast as the cracks run; petals bent round the
impactor (σ ≈ E h δ / r_c² at the contact circle) snap off in rings until the
hole passes the impactor.

**Tempered dicing.** If the core tension exceeds the forking threshold of the
core, every running crack forks repeatedly. The fork spacing reproduces the
energy balance: the stored energy U = t (1−ν) CT² / (5E) (parabolic profile)
pays for the new faces of fragments of size a = 10 K_Ic² / ((1−ν) η CT²), with
η = 0.5 of the energy going into new surface (calibrated so 4–6 mm fully
tempered glass gives the 40–400 fragments per 50 × 50 mm square of ECE R43).
Lower tiers draw coarser dice; the result panel reports the real size.

## 5. Pieces

When a monolithic ply's cracks have been quiet for 150 µs and nothing presses
on it, its grid is flood-filled (4-connected) into regions, crack cells are
shared out to their neighbours, and each region's outline is traced and
simplified (Douglas-Peucker). When a ply has more regions than the tier's
piece budget left, the smallest merge into a neighbour until they fit, so the
pieces always cover the whole ply (coarser, never missing). Each piece gets the plate's local velocity and
spin, the impactor's velocity if it lies in its path, and for tempered glass a
small kick from the released strain energy. A piece stays if it sits on the
bottom edge, is wedged across two edges, or the gasket's grip on its edge
(N/m) holds its weight, whatever the plate is doing under it; only pieces
small beside an object passing through (four times its push radius) go with
it. A region with loops inside it (a hole punched through) keeps them as
holes, out of its area, mass and inertia. Free pieces fly with gravity and quadratic drag on a fixed 0.5 ms step, hit the
floor with restitution 0.25 and friction 0.45, sleep when they settle, and
break again along a random line if a landing's energy beats 40 × the new crack
surface energy. A piece resting on the floor loses spin to rolling friction,
and sleeps once its edge moves slower than 0.12 m/s.

### How a crazed tempered pane comes down

Once the dicing front has crossed a tempered ply, the pane is a crazed sheet
([`collapse.cpp`](../src/core/src/collapse.cpp)): thousands of dice with no
strength between them, held only by friction where they press on each other
and on the frame. Every contact between two dice (pieces either side of a
crack) gets a grip drawn once from the seed, scaled by the length of crack they
share. A die's capacity is the grip of its neighbours still in place,
counted fully from the die below (bearing), 12 % from the sides and 5 % from
above, times (neighbours left / neighbours it had)² because friction needs
wedging all round, plus the frame's grip for edge dice. A die falls when
that is less than its weight.

Each release queues a check of its neighbours after the time to slide clear of
a 0.2–1.5 mm gap under gravity, so the sheet unzips from the hole at the speed
gravity allows: above the hole first, which bears on nothing, then the rest.
Neighbours that would also fall and grip it hard come away with it as a slab
(a rigid clump that breaks up after 0.15–0.65 s or on the floor), and an
exposed die that only just holds may crumble off later on its own (an
exponential wait whose mean grows with its margin). The crushed ring round a
hole touches nothing, so dice on the rim are checked directly; dice under an
object still driving through the glass (and the ring it shoves aside) go with
it, so a punch that crazes the pane before it has made its hole still starts
the collapse. What is left is a fringe the frame holds. A piece drawn coarser
than the real dice is only a handful of them barely locked together: 40 to
160 ms after it comes free (seeded per piece) it comes apart in the air, or on
the floor if it lands first, into a spray of real-size dice that renderers
scatter (`G.sprays`). New impacts make the dice near them re-check.

## 6. Bullets

A shot load (`pellets` > 1, e.g. 00 buckshot) fires each pellet as its own
projectile: a seeded angle off the aim (rms `spread_mrad`, landing where it
carries over `range_m`), a 2 % speed scatter and a place in a shot string
`shot_string_m` long, so the pellets arrive one after another.

Each glass ply crushes a cone (68° half-angle) whose radius grows through the
ply from the damage in front, up to 30 mm; the energy absorbed is the
comminution energy (40 MJ/m³ for soda-lime) times the cone volume, the plug
ahead takes its share of momentum, and a soft nose flattens toward
0.3 √(1 + ρv²/2Y) of its radius (lead does, a hard core does not).
Interlayers tear at 1 % of σ_t ε_f per volume over the cone's area; a polymer
backing absorbs 5 % of σ_t ε_f per volume and bulges. The constants are fitted
so the representative UL 752 builds stop their rated rounds and Level 3 does
not stop a rifle; the fit is checked on five other cases (tests). The momentum
lost each step loads the plate, so the spider web and rings come from the same
crack model as everything else, ply by ply.

A round that arrives off square keeps its speed and meets every layer at its
line-of-sight thickness (thickness / cos θ); the plate feels the normal part of
its force, and the depth reported is through the pane. Beyond 72° from square
it glances off, leaving a scuff and a few short cracks. A round that lands in
an earlier crater pays only 15 % of the comminution energy for the glass that
is already rubble there (estimated: confined rubble still has to be displaced),
so armour hit repeatedly in one place does give way. Every crushed disc is kept
(`Pane::crushes()`, `G.crushes`): comminuted glass is white, and the status
counts it as opaque.

## 7. Blast

`Pane::blast(BlastLoad)` loads the whole pane with a blast wave (+ toward the
back face; `gls_blast` in the browser): the shock reflected at the glass as a
Friedlander pulse, p(τ) = p_r (1 − τ/t_d) e^(−bτ/t_d), and, for a charge in a
closed room, the quasi-static pressure of its gas, p_g (1 − e^(−τ/t_r))
e^(−τ/τ_v), venting with time constant τ_v. The caller works out p_r, t_d, b
and the gas from the charge and the room (the glass lab's house scene uses the
Kinney–Graham fits for a TNT-equivalent charge in free air, the normal
reflection factor, and the UFC 3-340-02 gas-pressure fit). The step lands on
the shock and resolves its positive phase in 50 steps; the pane bends under
it, and the flaw probes break it like any other load (with the rate factor
counted from the shock's arrival).

**A charge beside the glass.** Averaged over the pane, a charge close to it is
badly wrong: a grenade on the floor 0.3 m from the bottom of a 2.55 m pane
puts tens of MPa on the glass beside it 0.08 ms after it goes off, and a few
tens of kPa on the far corner 3.7 ms later. Given the charge instead
(`BlastLoad::point`: its TNT mass, its foot on the pane's plane and its
standoff; `gls_blast_point` in the browser), the engine gives every point of
the glass its own shock: the arrival integrated at the front's own speed
c₀ √(1 + 6p/7P₀), the side-on peak and positive phase from Kinney & Graham at
that point's distance (held at their 0.3 m values closer in), reflected head-on
and side-on at grazing, p = p_s + (p_r − p_s) cos²θ, and the Friedlander decay
for the right impulse, all tabulated by distance when the blast is set. The
plate takes the field on a grid of about 5 cm cells, projected exactly onto
its modes; the step follows the shortest pulse still sweeping the pane. What
the blast does to the pieces (below) uses the pressure where each piece is.
Ten-millimetre annealed plate under that grenade cracks 0.6 ms after the
shock reaches it and leaves at 12–14 m/s from the bottom up (about the
impulse over its mass per area); averaged over the pane, the same charge
does not break it (tests).

While the blast presses above 1 kPa, every loose piece whose pressure force
(p × area) beats its frame grip leaves the frame — a crazed tempered sheet
does not hold at all, since its friction is a few times a die's weight — with
the glass's own speed and a turbulent scatter (± 0.4 m/s, a few rad/s).
Pieces that are still in the opening and within 0.6 m of the pane's plane are
then driven by the blast where they are, p × area / mass, fading to nothing at
0.6 m.
A 0.9 × 1.2 m pane of 6 mm fully tempered glass under a 150 kPa, 1.5 ms shock
and 60 kPa of gas fails about 5 ms after the shock arrives and leaves at
15–40 m/s as a cloud of dice (tests).

## 8. Tones (resonance)

`Pane::tone(ToneLoad)` plays a steady note at the pane (`gls_tone` in the
browser): a sinusoidal load of a given frequency and peak level, which follows
a new target level with a time constant and keeps its phase when retuned, so
a slider or a voice can sweep it. Sound arriving square on is a uniform
pressure on the struck face (the caller passes the peak pressure at the
glass, already doubled by the reflection), which drives only the modes odd in
both directions; `shaker` pushes at one point instead (a 1 cm foot) and drives
them all. The step is at most a 32nd of a cycle, and flaws are checked 24
times a cycle. A flaw grows only near the peaks of a cyclic tension: with the
stress-corrosion exponent n ≈ 16, ∫ max(0, sin)ⁿ over a cycle is about a tenth
of it, so the strength is the one for 0.1 × the time the note has played
(estimated). `toneMeter()` gives the highest share of its own strength any
flaw has reached since it was last read, and `toneSwing()` the antinode swing
from the plate's energy.

On a mode the swing builds up to the static deflection × 1/(2ζ) over
τ = 1/(ζω) (tested: 102 for ζ = 0.005). 2 mm picture glass, 0.30 × 0.25 m,
rings at 135 Hz; at the glass, 130 dB never breaks it, 135 dB breaks it in
0.8 s and 145 dB in 0.15 s. Those swings are 7 or more times the glass's
thickness, past where the linear plate holds (see the gaps below), so they
are lower bounds on what a real pane needs.

## 8b. Spontaneous breakage (nickel sulfide)

`Pane::nickelSulfide(at, diameter)` (`gls_nickel_sulfide`): a nickel-sulfide
stone that has changed phase years after tempering and opened a crack its own
size. In a tempered ply's core tension that is a penny crack driven by
K = 2σ_t √(a/π); past K_Ic the ply starts dicing from the point, exactly as if a
flaw had failed there (critical diameter 2π (K_Ic / 2σ_t)², 0.35 mm for the
10 mm storefront panel). Annealed and laminated plies have no core tension to
drive it. A pane that broke by itself is not "quiet" until its cracks have been
cut into pieces.

## 8c. Scoring (a glass cutter)

`Pane::score(line, closed, back, vent)` (`gls_score_*`): a cutter's score as a
vent crack of depth c (12 % of the ply by default) along a polyline or loop.
It is checked with the flaw probes, every 4 mm along it, and gives way where
the tension across it first beats K_Ic / (1.12 √(πc)) (no loading-rate factor:
the vent fails at K_Ic itself); a crack then starts there along it, both ways.
A tip within 1.5 cells (3 mm) of a score, heading within 60° of it, is steered
onto it, runs on at 0.6 K_Ic (estimated), does not wander or fork, and may turn
as tightly as the score does, so a scored circle comes out as a disc. The
Hertzian cone check now compares the edge tension with the one at Auerbach's
load under a hard (steel) ball of the same radius, so a soft ball (a rubber
mallet) does not start cones.

## 9. Prince Rupert's drop

`RupertDrop` (`gls/rupert.hpp`, `gls_drop_*` in the browser) is a body of
revolution, not a pane: a spherical head thinning into a long tail, with the
frozen axial stress σ(u) = σ_t − (σ_t + σ_c) u^k over the share of the radius
u, and k = 2σ_c/σ_t, so that every section carries no net force. Snapping the
tail starts a front at the glass's terminal crack speed that runs to the head;
every fragment it makes (size from the tempered-dice energy balance on the
core tension, speed from 5 % of the strain energy where it was, estimated) is
decided when it starts, with the time the front reaches it, so stepping only
moves the clock. A strike on the head breaks it only if a crack gets through
the compressed skin, of depth d:

- a hard object, as a Hertz contact, by Roesler's cone crack closed by the
  skin's mean compression: χF/d^1.5 > K_Ic + 1.12 σ̄_c √(πd) (about 20 kN for
  a 5 mm head, against the order of 15 kN that compression tests find);
- a soft bullet that flows, at the stagnation pressure Y + ½ρv² (Tate), by the
  pull (1 − 2ν)/2 p beside its face (Love) against the surface compression:
  1.12(σ − σ_c)√(πd) > K_Ic (a .38 lead bullet splashes, a 7.62 rifle round
  bursts the drop).

## Calibration constants

All in [`pane.cpp`](../src/core/src/pane.cpp) and
[`contact.cpp`](../src/core/src/contact.cpp), with what they were fitted to:

| Constant | Value | Fitted to / basis |
|---|---|---|
| Resolution radius factor | 1.0 | 600 × 600-mode reference (test) |
| Fork spacing | 3 × (A_b/σ)² | the look of annealed fractures: a few forks near an impact, none far out |
| Dice energy fraction η | 0.5 | ECE R43 counts for 4–6 mm fully tempered |
| Wedge support (monolithic / laminated) | 1.0 / 0.15 | tip-loaded cantilever / bonded wedges |
| Auerbach constant | 5×10⁴ N/m | literature range 10⁴–10⁵ for soda-lime |
| Interlayer membrane stiffening, rim strain concentration | 2.0, 1.0 | ECE R43: 2.26 kg from 4 m held, from ~15 m/s torn |
| Crack-bridging strain factor (per interlayer) | PVB 3.0, ionoplast 1.25 | EN 356: 0.76 mm PVB laminates rate P2A (hold three 3 m drops, not three 9 m ones), 0.38 mm ones not P2A; ASTM E1996: 2.28 mm ionoplast with heat-strengthened glass passes missiles D and E, corner shots included |
| Hurricane glazing's frame hold | 75 kN/m | structural silicone on both faces over a 1 in bite (ASTM C1135 joint strength ~1.5 MPa); E1996 corner shots need 62–69 kN/m |
| Comminution energy, interlayer and backing fractions, mushroom coefficient | 40 MJ/m³, 0.01, 0.05, 0.3 | UL 752 Levels 1, 3, 8 |
| Rubble fraction, ricochet angle | 0.15, 72° | estimated (no published per-hit data for repeat hits on one crater; FMJ on glass ricochets near grazing) |
| Light band of a crack | 6 mm | estimated: glints either side of a crack line; with crushed area and open area this gives the status's visibility |
| Crazed-sheet grip, side and hanging shares, confinement exponent | 1.3 × die weight per die width, 0.12, 0.05, 2 | a holed pane empties in 0.3–1 s to a fringe the frame holds, top first (footage of tempered panes emptying); tested, not measured |
| Crumbling time | 0.8 s × margin / 2 | edges keep shedding dice for a few seconds |
| Crack wander, its length, tightest bend; ragged crush edge | 0.12 rad, 4 cm, 0.1 m; 14 % | estimated: the meander of radial cracks in photographs of broken annealed and heat-strengthened panes |
| Blast hold threshold, gas reach | 1 kPa, 0.6 m | estimated: a kPa beats any piece's weight and frame grip per area; the gas jet acts over about half a window's width |

## Main gaps

- Panes are flat and simply supported; curved windscreens are flattened and
  clamped or bonded edges are not modelled.
- The plate is linear: a pane bent further than its own thickness does not
  stretch and stiffen in its plane (membrane action). Very large swings, such
  as a note held on resonance or a big pane under a blast, are therefore
  generous. A Berger-type stretch was tried and set aside: with uniform
  tension it relieved the stress so much that a 6 mm tempered window survived
  a room blast that breaks it.
- The plate model does not soften as it cracks (except the wedge and layered
  load paths under the impactor); a heavily cracked laminate is stiffer than
  the real one.
- Polymer plies (polycarbonate, acrylic) do not crack or craze.
- Thermal stress (sunlit panes, heated windshields) is not modelled.
- Fragments do not collide with each other.
