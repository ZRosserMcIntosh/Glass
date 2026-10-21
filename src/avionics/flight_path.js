// Glass — flight path vector, the wind triangle and the altitude trend.
// SPDX-License-Identifier: MIT

import { Lag, Rate, AngleLag, wrap180, wrap360 } from './filters.js';

const D2R = Math.PI / 180;

// The wind from the triangle the FMS solves: ground velocity (GPS track and
// ground speed) minus air velocity (TAS along the true heading). Returns the
// direction it blows from (deg true) and its speed (kt).
export function windTriangle({ tasKt, trueHeadingDeg, groundSpeedKt, trackTrueDeg }) {
  const ge = groundSpeedKt * Math.sin(trackTrueDeg * D2R), gn = groundSpeedKt * Math.cos(trackTrueDeg * D2R);
  const ae = tasKt * Math.sin(trueHeadingDeg * D2R), an = tasKt * Math.cos(trueHeadingDeg * D2R);
  const we = ge - ae, wn = gn - an;   // the air's velocity over the ground
  return { fromDeg: wrap360(Math.atan2(-we, -wn) / D2R), speedKt: Math.hypot(we, wn) };
}

// Headwind (+) and crosswind (+ from the right) components of a wind on a heading.
export function windComponents(fromDeg, speedKt, headingDeg) {
  const rel = (fromDeg - headingDeg) * D2R;
  return { headwindKt: speedKt * Math.cos(rel), crosswindKt: speedKt * Math.sin(rel) };
}

// The wind is solved only where the triangle is well conditioned: airborne at
// a useful TAS (the error grows as 1/TAS with a heading error).
export const WIND_MIN_TAS_KT = 60;

export class FlightPathComputer {
  constructor() {
    this.wind = new AngleLag(4);        // the triangle's wind, filtered over a few seconds
    this.windSpeed = new Lag(4);
    this.altTrend = new Rate(1);
    this.windValid = false;
  }
  update(f, air, trueHeadingDeg, dt) {
    // Wind: the speed is filtered on its own; the direction is filtered as a
    // vector weighted by the speed, so a calm wind does not spin the arrow.
    let wind = null;
    this.windValid = !f.onGround && air.tasKt >= WIND_MIN_TAS_KT && f.groundSpeedKt > 20;
    if (this.windValid) {
      const w = windTriangle({ tasKt: air.tasKt, trueHeadingDeg, groundSpeedKt: f.groundSpeedKt, trackTrueDeg: f.inertial.trackTrueDeg });
      const dir = this.wind.update(w.fromDeg, dt, Math.max(w.speedKt, 0.1));
      const spd = this.windSpeed.update(w.speedKt, dt);
      wind = { fromDeg: dir.deg, speedKt: spd, ...windComponents(dir.deg, spd, trueHeadingDeg) };
    } else {
      this.wind.reset();
      this.windSpeed.reset();
    }
    // Flight path vector: the inertial flight-path angle above the horizon and
    // the drift angle off the nose, so the symbol sits where the airplane is going.
    const fpv = { fpaDeg: f.inertial.fpaDeg, driftDeg: wrap180(f.driftDeg), valid: f.groundSpeedKt > 30 };
    // Altitude trend: 6 s of the present vertical speed from the altimeter
    // (its rate through a 1 s washout, so it leads the VSI's capillary lag).
    const altRate = this.altTrend.update(f.air.altitudeFt, dt);    // ft/s
    return { wind, fpv, altitudeTrendFt: 6 * altRate, verticalSpeedFpm: f.air.vsiFpm };
  }
}
