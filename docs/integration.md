# Coupling to the vehicle and flight engines

Glass Dynamics does not link either engine. A host owns its body and its
collision detection; it tells the glazing where and how hard something hit,
steps the pane alongside its own clock, and reads back what it needs. The
frames are written out in [`bridge.hpp`](../src/core/include/gls/bridge.hpp)
and tested against the conventions each engine documents.

| | OpenRoad Dynamics (vehicles) | Aviation Dynamics (aircraft) |
|---|---|---|
| Body frame | ISO 8855: x forward, y left, z up | FRD: x forward, y right, z down (ADR 0002) |
| Mounts | `vehicle::windscreen(base, w, h, rake)`, `vehicle::sideWindow(base, w, h, left, tumblehome)` | `aircraft::windshield(base, w, h, rake, sweep)` |
| Glazing | `windscreen` (laminated), `car_side_tempered` | `aircraft_windshield` (pressurised, fail-safe) |
| What hits it | road stones, hail, pedestrians, a crash dummy's head from inside, bullets (the armoured S 600 Guard) | birds, hail |
| What goes back | `PaneStatus.visibility` for the driver's view, `barrierIntact`, `openArea` for wind noise and rain | `visibility` for the pilot, `barrierIntact` and `openArea` for cabin depressurisation |

## A stone at highway speed

```cpp
// Once, with the car: a windscreen raked 60 degrees, base 1.2 m ahead of the CG.
gls::PaneMount mount = gls::vehicle::windscreen({1.2, 0, 1.0}, 1.45, 0.85, 60);
gls::Pane glass(*lib.glazing("windscreen"), gls::budgetForTier(1), seed);

// When the host's collision query finds a stone on the windscreen: its
// velocity relative to the body (a stone at rest, the car at 30 m/s -> -30 x).
gls::strike(glass, mount, *lib.impactor("stone"), hitPointBody, {-30, 0, 0});

// Every frame, after the vehicle step:
glass.advance(simTime);
driverView = glass.status().visibility;
```

## A crash dummy's head

OpenRoad's crash module moves each occupant's head relative to the seat at
0.1 ms and knows when it reaches the windscreen ("hard contact"). Hand that
contact to the glazing as a strike from inside (velocity pointing out of the
cabin): the pane flips to its inner face by itself (`side = −1`), breaks the
inner ply first, and its membrane force (the report's `peakForce`, the bulge)
is what the head felt, which can go back into the dummy's head model in place
of the rigid-pillar stiffness.

```cpp
gls::strike(glass, mount, *lib.impactor("headform"), headPointBody, headVelocityRelativeToBody);
```

## A bird at 250 knots

```cpp
gls::PaneMount ws = gls::aircraft::windshield({4.0, 0.4, -1.2}, 0.85, 0.6, 55, 20);
gls::Pane glass(*lib.glazing("aircraft_windshield"), gls::budgetForTier(2), seed);
const gls::Vec3 v = gls::aircraft::relativeAirVelocity(tas, alpha, beta);  // body frame
gls::strike(glass, ws, *lib.impactor("bird_4lb"), contactPointBody, v);
```

The windshield is loaded by the cabin differential (`pressure_kPa: -59.3`,
outward) from the start. `status().barrierIntact` going false is the moment the
host's pressurisation model should start venting through `openArea`.

## Time

The pane advances in whole internal steps chosen by what is happening (50 ns in
a bullet, 0.5 ms for falling pieces), independent of the host's frame, so
`advance(t)` from a 240 Hz flight loop or a 360 Hz vehicle loop gives the same
fracture. Between impacts the pane costs nothing (`quiet()`). A host that wants
slow motion advances it by less than real time; the vertex birth times let a
renderer replay the fracture at any speed.

## Budgets

`budgetForTier(0..4)` sets the resolution: plate modes, crack grid, tip and
piece budgets. A shot well clear of a threshold ends the same on every budget,
but near one (a ball just fast enough to break the pane) the resolution can
tip it either way. Where every player or viewer must see the same result, give
everyone the same budget and let slower machines take longer over it: the
Glass Lab runs `budgetForTier(3)` on every device, in a worker, and slows its
clock when the device cannot keep up. A scene of many panes can give each a
coarser budget, so long as it is the same one for everybody.
