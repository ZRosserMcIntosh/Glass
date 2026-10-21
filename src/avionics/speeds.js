// Glass — the speed tape's physics: limit speeds, low-speed awareness,
// reference speeds and the speed trend.
// SPDX-License-Identifier: MIT

import { casFromMach } from './air_data.js';
import { Lag, Rate, clamp } from './filters.js';

// The maximum speed the tape's upper barber pole starts at (kt IAS): the
// lowest of VMO, the CAS of MMO at the present static pressure (MMO governs
// above the crossover altitude), the flap limit with the flaps out and the
// gear limit with the gear not up.
export function maximumSpeed({ vmoKt, mmo, vfeKt, vleKt, flapDeg, gear, staticPa }) {
  const limits = [{ kt: vmoKt, by: 'VMO' }];
  if (mmo > 0 && staticPa > 0) limits.push({ kt: casFromMach(mmo, staticPa), by: 'MMO' });
  if (flapDeg > 0.5 && vfeKt > 0) limits.push({ kt: vfeKt, by: 'VFE' });
  if (gear > 0.01 && vleKt > 0) limits.push({ kt: vleKt, by: 'VLE' });
  return limits.reduce((a, b) => (b.kt < a.kt ? b : a));
}

// Low-speed awareness. The core reports the 1-g stall CAS for the present
// flap, weight and Mach (from the trimmed CLmax). At load factor n the wing
// stalls at Vs1g sqrt(n), so the red band follows the manoeuvre; the amber
// band ends where 1.3 g more would stall it (a 40 deg bank's margin), the
// usual minimum-manoeuvre speed.
export const MANOEUVRE_MARGIN_G = 1.3;
export function lowSpeedBands(vs1gKt, loadFactor) {
  const n = Math.max(loadFactor, 0.5);
  return { stallKt: vs1gKt * Math.sqrt(n), minManoeuvreKt: vs1gKt * Math.sqrt(MANOEUVRE_MARGIN_G * n) };
}

// Reference speeds from the reference stall speeds (CS/14 CFR 25):
// V2 >= 1.13 VSR at the take-off flap (25.107(b)), VREF >= 1.23 VSR0 at the
// landing flap (25.125(b)(2)). The approach speed adds half the steady
// headwind component, at least 5 kt and at most 20 kt (the common airline
// wind additive; no gust is known to the computer).
export function referenceSpeeds({ vsTakeoffKt, vsLandingKt, headwindKt = 0 }) {
  const v2 = 1.13 * vsTakeoffKt, vref = 1.23 * vsLandingKt;
  const additive = clamp(0.5 * Math.max(headwindKt, 0), 5, 20);
  return { v2, vref, vapp: vref + additive, additive };
}

// Speed trend: where the speed will be in `horizon` seconds at the present
// acceleration, the rate seen through a 1 s washout (Boeing shows 10 s).
export class SpeedTrend {
  constructor(horizon = 10, tau = 1) { this.horizon = horizon; this.rate = new Rate(tau); }
  update(iasKt, dt) { return this.horizon * this.rate.update(iasKt, dt); }
  reset() { this.rate.reset(); }
}

// The speed tape's computations each frame.
export class SpeedComputer {
  constructor() {
    this.trend = new SpeedTrend();
    this.n = new Lag(0.5);          // load factor: turbulence would shake the bands
    this.machShown = false;
  }
  update(f, air, dt, headwindKt) {
    const L = f.limits;
    const n = this.n.update(f.attitude.loadFactor, dt);
    const bands = lowSpeedBands(L.vs1gKt, n);
    const max = maximumSpeed({ ...L, flapDeg: f.config.flapDeg, gear: f.config.gear, staticPa: air.staticPa });
    // Mach readout below the tape from 0.40, off again under 0.38.
    this.machShown = air.mach >= 0.4 || (this.machShown && air.mach >= 0.38);
    const ias = f.air.iasKt;
    return {
      iasKt: ias,
      trendKt: f.onGround && ias < 40 ? 0 : this.trend.update(ias, dt),
      ...bands,
      maxKt: max.kt, maxBy: max.by,
      mmoCasKt: L.mmo > 0 ? casFromMach(L.mmo, air.staticPa) : Infinity,
      mach: air.mach, machShown: this.machShown,
      ...referenceSpeeds({ vsTakeoffKt: L.vsTakeoffKt, vsLandingKt: L.vsLandingKt, headwindKt }),
      // Overspeed: past VMO by 2 kt or MMO by 0.01, the usual warning thresholds' tolerance.
      overspeed: ias > L.vmoKt + 2 || (L.mmo > 0 && air.mach > L.mmo + 0.01),
      lowSpeed: !f.onGround && ias < bands.minManoeuvreKt && ias > 30,
    };
  }
}
