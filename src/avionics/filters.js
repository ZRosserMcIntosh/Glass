// Glass — the small filters the display computations share.
// SPDX-License-Identifier: MIT
//
// Exact discretisations (1 - e^(-dt/tau)), so a filter behaves the same at any
// frame rate the page happens to run at.

const alpha = (dt, tau) => (tau > 0 ? 1 - Math.exp(-Math.max(dt, 0) / tau) : 1);

// First-order lag: y' = (x - y) / tau.
export class Lag {
  constructor(tau, initial = null) { this.tau = tau; this.y = initial; }
  update(x, dt) {
    if (this.y === null || !Number.isFinite(this.y)) this.y = x;
    else this.y += (x - this.y) * alpha(dt, this.tau);
    return this.y;
  }
  reset(x = null) { this.y = x; }
}

// Rate estimator: the derivative of x seen through a first-order lag of time
// constant tau, s / (tau s + 1). The trend vectors of the speed and altitude
// tapes use it: it rejects the frame-to-frame noise a raw difference would
// amplify. Taken as the lag output's own step over the frame, so a ramp reads
// its exact slope at any frame rate.
export class Rate {
  constructor(tau) { this.tau = tau; this.lag = new Lag(tau); this.rate = 0; }
  update(x, dt) {
    const before = this.lag.y;
    const y = this.lag.update(x, dt);
    if (before === null) this.rate = 0;
    else if (dt > 0) this.rate = (y - before) / dt;
    return this.rate;
  }
  reset() { this.lag.reset(); this.rate = 0; }
}

// A lag on a direction (deg): filters the unit vector, so 359 -> 1 does not
// swing through 180.
export class AngleLag {
  constructor(tau) { this.x = new Lag(tau); this.y = new Lag(tau); }
  update(deg, dt, weight = 1) {
    const r = (deg * Math.PI) / 180;
    const x = this.x.update(weight * Math.sin(r), dt), y = this.y.update(weight * Math.cos(r), dt);
    return { deg: wrap360((Math.atan2(x, y) * 180) / Math.PI), magnitude: Math.hypot(x, y) };
  }
  reset() { this.x.reset(); this.y.reset(); }
}

// A condition that must hold for `seconds` before it counts (and clears at once).
export class Persist {
  constructor(seconds) { this.seconds = seconds; this.t = 0; }
  update(on, dt) { this.t = on ? this.t + Math.max(dt, 0) : 0; return this.t >= this.seconds; }
  reset() { this.t = 0; }
}

export const wrap360 = (d) => ((d % 360) + 360) % 360;
export const wrap180 = (d) => ((((d + 180) % 360) + 360) % 360) - 180;
export const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
