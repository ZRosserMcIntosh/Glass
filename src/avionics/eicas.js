// Glass — engine indication and crew alerting (EICAS / CAS).
// SPDX-License-Identifier: MIT
//
// Every message is a predicate on the decoded frame (readout.js) plus the
// limits below; the alerting logic around it is the usual one: warnings (red,
// MASTER WARNING), cautions (amber, MASTER CAUTION), advisories (cyan) and
// status memos (white); a new warning or caution lights its master light until
// acknowledged, the message itself stays while its condition holds; cautions
// and advisories are inhibited through the take-off from 80 kt to 400 ft.

import { Persist } from './filters.js';

export const LEVEL = { WARNING: 3, CAUTION: 2, ADVISORY: 1, STATUS: 0 };
export const TAKEOFF_INHIBIT = { fromKt: 80, toRaFt: 400 };

// Engine limits for the gauges. N1 and oil pressure come from the aircraft's
// panel markings (avd_markings: tachometer red line, oil green and red
// limits); the ITT's top of normal is the engine's rated take-off ITT from its
// definition, and the red line, which the definitions do not give, is set
// 25 C above it (an estimate, flagged as such in docs/avionics.md).
export function engineLimits(markings, turbofan) {
  const ittTakeoffC = turbofan && turbofan.itt_takeoff_k ? turbofan.itt_takeoff_k - 273.15 : 800;
  return {
    n1Red: markings[11] || 102, n1Max: markings[12] || 110,
    oilGreen: [markings[13] || 35, markings[14] || 75], oilRedLow: markings[15] || 25, oilRedHigh: markings[16] || 90,
    ittAmber: ittTakeoffC, ittRed: ittTakeoffC + 25, ittMax: ittTakeoffC + 150,
  };
}

// Colour of one engine reading: 'white' normal, 'amber' caution range, 'red' limit, 'off' no indication.
export function gaugeColour(kind, v, lim, running) {
  if (!Number.isFinite(v)) return 'off';
  switch (kind) {
    case 'n1': return v > lim.n1Red ? 'red' : 'white';
    case 'itt': return v > lim.ittRed ? 'red' : v > lim.ittAmber ? 'amber' : 'white';
    case 'oil':
      if (!running) return 'white';
      return v < lim.oilRedLow || v > lim.oilRedHigh ? 'red' : v < lim.oilGreen[0] || v > lim.oilGreen[1] ? 'amber' : 'white';
    default: return 'white';
  }
}

const anyLever = (f, x) => f.engines.some((e) => e.lever > x);
const allLevers = (f, x) => f.engines.length > 0 && f.engines.every((e) => e.lever < x);
const TAKEOFF_LEVER = 0.6;

// The message table: [text, level, condition(f, ctx)]. ctx carries the
// avionics' own state (overspeed, fuel capacity, persistence timers).
function messages(f, ctx0) {
  const m = [];
  const ra = f.radioAltFt, air = !f.onGround;
  const L = f.limits, C = f.config;
  // --- warnings
  m.push(['OVERSPEED', LEVEL.WARNING, (ctx) => ctx.overspeed]);
  m.push(['STALL', LEVEL.WARNING, () => air && f.protection.stallWarning]);
  m.push(['STICK PUSHER', LEVEL.WARNING, () => f.protection.pusher]);
  m.push(['CONFIG FLAPS', LEVEL.WARNING, () => f.onGround && anyLever(f, TAKEOFF_LEVER) && (C.flapDeg < 0.5 || C.flapDeg > L.landingFlapDeg - 0.5)]);
  m.push(['CONFIG SPOILERS', LEVEL.WARNING, () => f.onGround && anyLever(f, TAKEOFF_LEVER) && C.speedbrakeDetent > 0]);
  m.push(['CONFIG PARK BRK', LEVEL.WARNING, () => f.onGround && anyLever(f, TAKEOFF_LEVER) && f.parkingBrake]);
  m.push(['CONFIG GEAR', LEVEL.WARNING, () => air && C.gear < 0.999 && (C.flapDeg > L.landingFlapDeg - 0.5 || (ra < 800 && allLevers(f, 0.2)))]);
  // --- cautions
  f.engines.forEach((e, i) => {
    const n = i + 1;
    m.push([`ENG ${n} FAIL`, LEVEL.CAUTION, () => e.fuelSwitch && !e.running && !e.starting]);
    m.push([`ENG ${n} OIL PRESS`, LEVEL.CAUTION, (ctx) => e.running && e.oilPsi < ctx.engineLimits.oilRedLow]);
    if (ctx0.perEngineGenerators) m.push([`GEN ${n} OFF`, LEVEL.CAUTION, () => e.running && !e.generator]);
  });
  if (!ctx0.perEngineGenerators) m.push(['GEN OFF', LEVEL.CAUTION, () => f.engines.some((e) => e.running) && !f.systems.anyGenerator]);
  m.push(['ELEC BUS VOLTS LO', LEVEL.CAUTION, () => f.systems.busVolts > 1 && f.systems.busVolts < 24]);
  m.push(['BATT DISCHARGE', LEVEL.CAUTION, (ctx) => ctx.persist('batt', f.systems.batteryAmps > 2 && f.engines.some((e) => e.running), 10)]);
  if (f.systems.hydraulicsFitted) {
    ['A', 'B'].forEach((sys, i) => m.push([`HYD ${sys} LO PRESS`, LEVEL.CAUTION, (ctx) => ctx.seen(`hyd${i}`, f.systems.hydPsi[i] > 2000) && f.systems.hydPsi[i] < 1500]));
    m.push(['BRAKE PRESS LO', LEVEL.CAUTION, () => f.systems.brakePower < 0.5]);
  }
  m.push(['GEAR DISAGREE', LEVEL.CAUTION, (ctx) => ctx.persist('gear', Math.abs(C.gear - (C.gearLeverDown ? 1 : 0)) > 0.01, 20)]);
  m.push(['FUEL LOW', LEVEL.CAUTION, (ctx) => ctx.fuelCapacityKg > 0 && f.fuel.kg < 0.1 * ctx.fuelCapacityKg]);
  m.push(['SPEEDBRAKE EXTENDED', LEVEL.CAUTION, () => air && C.speedbrake > 0.05 && ((ra > 15 && ra < 800) || anyLever(f, TAKEOFF_LEVER))]);
  m.push(['LOW SPEED', LEVEL.CAUTION, (ctx) => ctx.lowSpeed]);
  // --- advisories
  f.engines.forEach((e, i) => {
    m.push([`ENG ${i + 1} SHUTDOWN`, LEVEL.ADVISORY, () => !e.fuelSwitch && !e.running && air]);
    if (C.reversersFitted) m.push([`REV ${i + 1} UNLOCKED`, LEVEL.ADVISORY, () => air && e.reverser > 0.01]);
  });
  m.push(['GEAR EMER RELEASE', LEVEL.ADVISORY, () => C.gearEmergency]);
  m.push(['GND SPOILERS', LEVEL.ADVISORY, () => C.groundSpoilers]);
  // --- status memos
  m.push(['PARKING BRAKE', LEVEL.STATUS, () => f.parkingBrake]);
  m.push(['SPEEDBRAKE ARMED', LEVEL.STATUS, () => air && C.gear > 0.999 && C.speedbrake < 0.05 && C.flapDeg > L.landingFlapDeg - 0.5]);
  return m;
}

export class Cas {
  constructor({ fuelCapacityKg = 0, engineLimits: lim, perEngineGenerators = false } = {}) {
    this.fuelCapacityKg = fuelCapacityKg;
    this.installation = { perEngineGenerators };
    this.engineLimits = lim;
    this.timers = new Map();
    this.seenFlags = new Set();
    this.active = new Map();      // text -> { level, since }
    this.masterWarning = false;
    this.masterCaution = false;
    this.time = 0;
  }
  // Returns { list: [{ text, level, since }] sorted by level then age, masterWarning, masterCaution, inhibited }.
  update(f, dt, { overspeed = false, lowSpeed = false } = {}) {
    this.time += dt;
    const used = new Set();
    const ctx = {
      overspeed, lowSpeed, fuelCapacityKg: this.fuelCapacityKg, engineLimits: this.engineLimits,
      persist: (key, on, seconds) => {
        used.add(key);
        let p = this.timers.get(key);
        if (!p) this.timers.set(key, (p = new Persist(seconds)));
        return p.update(on, dt);
      },
      seen: (key, now) => { if (now) this.seenFlags.add(key); return this.seenFlags.has(key); },
    };
    const inhibited = this.takeoffInhibit(f);
    const now = new Map();
    for (const [text, level, cond] of messages(f, this.installation)) {
      if (!cond(ctx)) continue;
      if (inhibited && (level === LEVEL.CAUTION || level === LEVEL.ADVISORY)) continue;
      now.set(text, level);
    }
    for (const [text, level] of now) {
      if (!this.active.has(text)) {
        this.active.set(text, { level, since: this.time });
        if (level === LEVEL.WARNING) this.masterWarning = true;
        if (level === LEVEL.CAUTION) this.masterCaution = true;
      }
    }
    for (const text of [...this.active.keys()]) if (!now.has(text)) this.active.delete(text);
    for (const key of [...this.timers.keys()]) if (!used.has(key)) this.timers.delete(key);
    const list = [...this.active].map(([text, a]) => ({ text, ...a })).sort((a, b) => b.level - a.level || b.since - a.since);
    return { list, masterWarning: this.masterWarning, masterCaution: this.masterCaution, inhibited };
  }
  // Take-off inhibit: on the ground above 80 kt, and airborne until 400 ft RA
  // with the levers still at take-off power.
  takeoffInhibit(f) {
    const ias = f.air.iasKt, levers = anyLever(f, TAKEOFF_LEVER);
    if (f.onGround) { this.inTakeoff = levers && ias > TAKEOFF_INHIBIT.fromKt; return this.inTakeoff; }
    if (this.inTakeoff && f.radioAltFt < TAKEOFF_INHIBIT.toRaFt && levers) return true;
    this.inTakeoff = false;
    return false;
  }
  acknowledge() { this.masterWarning = false; this.masterCaution = false; }
  reset() { this.active.clear(); this.timers.clear(); this.seenFlags.clear(); this.acknowledge(); this.inTakeoff = false; }
}
