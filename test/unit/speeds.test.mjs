// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { maximumSpeed, lowSpeedBands, referenceSpeeds, SpeedTrend, SpeedComputer } from '../../src/avionics/speeds.js';
import { pressureFromAltitude, FT, solveAirData } from '../../src/avionics/air_data.js';
import { frame } from './frame.mjs';

test('maximum speed: VMO low, MMO high, flap and gear limits', () => {
  const base = { vmoKt: 320, mmo: 0.8, vfeKt: 200, vleKt: 250, flapDeg: 0, gear: 0 };
  assert.equal(maximumSpeed({ ...base, staticPa: pressureFromAltitude(10000 * FT) }).by, 'VMO');
  const high = maximumSpeed({ ...base, staticPa: pressureFromAltitude(41000 * FT) });
  assert.equal(high.by, 'MMO');
  assert.ok(high.kt > 230 && high.kt < 250, `M0.8 at FL410 ${high.kt}`);
  assert.equal(maximumSpeed({ ...base, flapDeg: 10, staticPa: pressureFromAltitude(5000 * FT) }).by, 'VFE');
  assert.equal(maximumSpeed({ ...base, gear: 1, staticPa: pressureFromAltitude(5000 * FT) }).kt, 250);
});

test('low-speed bands follow the load factor', () => {
  const one = lowSpeedBands(120, 1), two = lowSpeedBands(120, 2);
  assert.equal(one.stallKt, 120);
  assert.ok(Math.abs(two.stallKt / one.stallKt - Math.SQRT2) < 1e-9, 'Vs grows with sqrt(n)');
  assert.ok(Math.abs(one.minManoeuvreKt - 120 * Math.sqrt(1.3)) < 1e-9);
});

test('reference speeds and the wind additive', () => {
  const r = referenceSpeeds({ vsTakeoffKt: 100, vsLandingKt: 100, headwindKt: 0 });
  assert.ok(Math.abs(r.v2 - 113) < 1e-9 && Math.abs(r.vref - 123) < 1e-9 && r.vapp === r.vref + 5);
  assert.equal(referenceSpeeds({ vsTakeoffKt: 100, vsLandingKt: 100, headwindKt: 24 }).additive, 12);
  assert.equal(referenceSpeeds({ vsTakeoffKt: 100, vsLandingKt: 100, headwindKt: 80 }).additive, 20);
});

test('speed trend: a steady 1 kt/s acceleration predicts +10 kt', () => {
  const t = new SpeedTrend();
  let out = 0;
  for (let i = 0; i <= 600; ++i) out = t.update(150 + i / 60, 1 / 60);
  assert.ok(Math.abs(out - 10) < 0.05, `trend ${out}`);
  // Frame-rate independent.
  const u = new SpeedTrend();
  for (let i = 0; i <= 100; ++i) out = u.update(150 + i / 10, 1 / 10);
  assert.ok(Math.abs(out - 10) < 0.05, `trend at 10 Hz ${out}`);
});

test('speed computer: overspeed past VMO, Mach readout hysteresis', () => {
  const sc = new SpeedComputer();
  const f = frame((f) => { f.air.iasKt = 325; f.air.altitudeFt = 10000; });
  const air = solveAirData({ iasKt: 325, indicatedFt: 10000, settingInHg: 29.92, satC: -4.8 });
  const out = sc.update(f, air, 0.1, 0);
  assert.equal(out.overspeed, true);
  assert.equal(out.machShown, true);
  assert.equal(sc.update(f, { ...air, mach: 0.39 }, 0.1, 0).machShown, true, 'stays on above 0.38');
  assert.equal(sc.update(f, { ...air, mach: 0.37 }, 0.1, 0).machShown, false);
});
