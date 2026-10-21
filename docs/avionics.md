# Avionics reference

What each display value is computed from, the relation used, and where it
comes from. "Core" means the Aviation Dynamics flight core (`core/`); index
names are those in `src/avionics/readout.js`.

## Inputs

The avionics read **sensed** channels only:

| Input | From the core | Notes |
|---|---|---|
| IAS, indicated altitude, Kollsman setting, VSI | `avd_cockpit` 0–3 | The core's pitot-static instruments, with their lags, icing and failures |
| Pitch, roll, heading, ball | `avd_cockpit` 4, 5, 7, 12 | AHRS: heading slaved to the magnetometer |
| SAT, load factor, α, FPA, track, radio altitude | `avd_jet` 41, 42, 43, 36, 37, 40 | |
| Limits: VMO, MMO, VFE, VLE, 1-g stall CAS (now, take-off flap, landing flap), flap settings | `avd_jet` 24, 25, 47, 48, 32–34, 49, 50 | Stall CAS from the core's trimmed CLmax at the present weight and Mach |
| Engines: N1, N2, ITT, fuel flow, oil pressure, levers, generators, reversers | `avd_jet` 8 + 8i, 30, 58, 60, 52 | Per-engine sensed channels (sender lags; blank without gauge power) |
| Systems: bus volts, battery, hydraulics, brake supply | `avd_jet` 44–45, 62–73 | |
| NAV 1 (CDI, GS, OBS) and the GPS leg (DTK, CDI, VDI, distance, bearing, magnetic variation) | `avd_cockpit` 32–42, `avd_gps` | |
| Ground speed, position, drift | `avd_state` 0, 1, 12, 62 | The GPS position and velocity |

The truth channels (`avd_cockpit` 48–68, `avd_jet` 5, 26, 38, 39, 74+) are
decoded under `truth` for the tests and never reach a display.

## Air data computer (`air_data.js`)

- **Static pressure** comes from inverting the core's altimeter calibration
  (`InstrumentPanel::altitudeFromPressure`). Up to the 11 km tropopause the
  troposphere law applies relative to the setting:
  h = 44,330.77 m · (1 − (p/p_set)^0.190263). Above it (22,632.06 Pa) the
  isothermal law applies: h = h₁₁ + 6,341.62 m · ln(p₁₁/p).
- **Pressure altitude** is the same law with p_set = 101,325 Pa.
- **Impact pressure** comes from CAS: q_c = p₀ [(1 + 0.2 (V_c/a₀)²)^3.5 − 1],
  with a₀ = 340.294 m/s as in the core's ASI. IAS stands in for CAS, because
  the glass profiles publish no position-error table.
- **Mach** is M = √(5 [(q_c/p + 1)^(2/7) − 1]). These subsonic St Venant
  relations are exact for γ = 1.4 below M 1.
- **TAS** is M · a(SAT). **TAT** is SAT · (1 + 0.2 r M²), with recovery r = 1.
- **The CAS of a Mach number** at the present static pressure gives the MMO
  speed on the tape. It is below VMO above the crossover altitude.

Checks: the round trip holds within 0.01 ft from −1,000 to 51,000 ft at three
settings. M 0.82 at FL350 is 279.6 KCAS (standard tables). 300 KCAS / M 0.78
cross over near FL300. In the core, Mach agrees within 0.003, TAS within
2 kt and pressure altitude within 30 ft for all six aircraft
(`test/core.test.mjs`).

## Speed tape (`speeds.js`)

- **Maximum speed** is min(VMO, CAS(MMO), VFE if flaps > 0, VLE if the gear is
  not up), labelled with the limit that governs.
- **Red band (stall)** is Vs1g √n. The wing's lift at stall is fixed, so the
  stall speed grows with the square root of the load factor; n is lagged 0.5 s.
- **Amber band (minimum manoeuvre)** is Vs1g √(1.3 n): the speed with a 1.3 g
  margin to the stall, which is a 40° bank's worth.
- **V2** is 1.13 × the take-off-flap stall CAS (CS/14 CFR 25.107(b)).
  **VREF** is 1.23 × the landing-flap stall CAS (25.125(b)(2)).
  **VAPP** is VREF + ½ × the headwind on the landing runway, at least 5 kt
  and at most 20 kt. The core spawns jets on final at its own
  1.23 Vs + 5 kt, and in calm air VAPP agrees with it within 3 kt for all six
  aircraft.
- **Trend** is 10 s × dIAS/dt. The rate is the step of a 1 s first-order lag,
  so a steady ramp reads its exact slope at any frame rate.
- **Mach readout** appears at M 0.40 and goes away below M 0.38.
- **Overspeed** is IAS > VMO + 2 kt or M > MMO + 0.01. **Low speed** is
  airborne with IAS below the amber band.

## Flight path and wind (`flight_path.js`)

- **Wind triangle:** wind = ground velocity (GPS track, ground speed) − air
  velocity (TAS along the true heading). True heading = AHRS magnetic heading
  + magnetic variation (from `avd_gps`). It is solved only airborne above
  60 kt TAS, because a heading error spoils the solution as the TAS falls.
  The speed is filtered over 4 s, and the direction as a vector weighted by
  the speed.
- **Agreement with the core:** within 5° and 2 kt in a 20 kt wind
  (`test/core.test.mjs`), and within 8° and 3 kt aloft at SBGR's
  intermediate fix in a 52 kt wind (`test/browser.test.mjs`).
- **FPV:** the inertial flight-path angle above the horizon and the drift off
  the nose, drawn in the rotated horizon frame.
- **Altitude trend:** 6 s × the rate of the altimeter reading, through the
  same 1 s rate filter, so it leads the VSI's capillary lag.

## Navigation (`navigation.js`)

- **AUTO source:** LOC once the localizer is received, else the GPS while
  it has an active leg, else VOR (NAV 1 against the OBS).
- **Deviations:** two dots are full scale. The core's needles run −1..1, so
  dots = 2 × needle. GPS full scale follows the core's Garmin-style scaling
  (2 / 1 / 0.3 NM, angular on final).
- **Courses:** the GPS courses and bearings are true in the core and are shown
  magnetic (true − variation).
- **The map** is heading-up: the true heading is up, and the labels are
  magnetic. Runways are drawn at least 6 px wide.

## Alerting (`alerting.js`)

- **Altitude alert:** inside 900 ft of the selected altitude it is
  "approaching" (box highlighted). Inside 200 ft it captures. Leaving 200 ft
  after capture is a deviation (amber) until a new selection.
- **Minimums:** BARO (DA/MDA on the altimeter) or RA (DH). "Approaching" shows
  in the last 100 ft, "MINIMUMS" when reached descending, and it resets after
  climbing 75 ft back above. The default is a CAT I DA, 200 ft above the
  landing field.
- **Callouts:** 2,500, 1,000, 500, 400, 300, 200, 100, 50, 40, 30, 20 and 10 ft
  RA, called once descending and re-armed after climbing 100 ft above.

## EICAS and crew alerting (`eicas.js`)

Engine colours:

| Gauge | Limits from | Colours |
|---|---|---|
| N1 | Panel markings (tachometer red line, 102 % in the glass profiles) | White; red above the red line |
| ITT | The engine's rated take-off ITT (`itt_takeoff_k`) | Amber above it; red above it + 25 °C (**estimate**: the definitions give no ITT red line) |
| Oil pressure | Panel markings (green range, red low and high) | Amber outside green, red outside red; only while the engine runs |

Messages (all are predicates on the sensed frame):

| Level | Message | Condition |
|---|---|---|
| Warning | OVERSPEED | IAS > VMO + 2 or M > MMO + 0.01 |
| Warning | STALL / STICK PUSHER | The core's stall warning (airborne) / pusher active |
| Warning | CONFIG FLAPS / SPOILERS / PARK BRK | On the ground with a lever above 60 % and the flaps up or at the landing setting / the speedbrake lever out / the parking brake set |
| Warning | CONFIG GEAR | Airborne, gear not down and locked, with landing flap, or below 800 ft RA with every lever below 20 % |
| Caution | ENG n FAIL | Fuel switch on, the engine neither running nor starting |
| Caution | ENG n OIL PRESS | Running below the red low |
| Caution | GEN n OFF | Per-engine generators: that engine running, its generator off line. With one alternator: GEN OFF, with an engine running and no generator on line |
| Caution | ELEC BUS VOLTS LO | Bus below 24 V |
| Caution | BATT DISCHARGE | More than 2 A discharge with an engine running, for 10 s |
| Caution | HYD A/B LO PRESS, BRAKE PRESS LO | Where fitted: the system seen above 2,000 psi and now below 1,500 psi / brake supply below 50 % |
| Caution | GEAR DISAGREE | Gear position not matching the lever for 20 s |
| Caution | FUEL LOW | Below 10 % of the usable capacity (from the load sheet's tanks) |
| Caution | SPEEDBRAKE EXTENDED | Airborne with the speedbrake out between 15 and 800 ft RA, or with a lever above 60 % |
| Caution | LOW SPEED | Below the amber band, airborne |
| Advisory | ENG n SHUTDOWN, REV n UNLOCKED, GEAR EMER RELEASE, GND SPOILERS | |
| Status | PARKING BRAKE, SPEEDBRAKE ARMED | |

A new warning or caution lights MASTER WARNING or MASTER CAUTION. The light
stays on until acknowledged; the message stays while its condition holds. From
80 kt on the take-off roll until 400 ft RA (levers still at take-off power),
cautions and advisories are inhibited.

## Known limits

- **CAS:** IAS is taken as CAS; there is no static-source position error.
- **ITT:** the red line is an estimate (see above).
- **Gusts:** the VAPP additive has no gust term, because the computer knows
  only the steady wind it solves.
- **Map:** the ND has one mode (heading-up arc). There is no plan mode, no
  terrain, no traffic and no weather radar.
- **Not modelled:** no autopilot or flight director modes (the core has none
  in the aircraft), and no FMS performance (V1, VR, top of descent).
