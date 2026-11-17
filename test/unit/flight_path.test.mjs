// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { windTriangle, windComponents } from '../../src/avionics/flight_path.js';
import { AngleLag, Lag, wrap180 } from '../../src/avionics/filters.js';

test('wind triangle: a headwind and a crosswind', () => {
  const head = windTriangle({ tasKt: 200, trueHeadingDeg: 90, groundSpeedKt: 180, trackTrueDeg: 90 });
  assert.ok(Math.abs(head.fromDeg - 90) < 1e-6 && Math.abs(head.speedKt - 20) < 1e-6);
  // 20 kt from the west on a northerly heading: the airplane drifts right.
  const gs = Math.hypot(20, 100), trk = (Math.atan2(20, 100) * 180) / Math.PI;
  const cross = windTriangle({ tasKt: 100, trueHeadingDeg: 0, groundSpeedKt: gs, trackTrueDeg: trk });
  assert.ok(Math.abs(cross.fromDeg - 270) < 1e-6 && Math.abs(cross.speedKt - 20) < 1e-6, JSON.stringify(cross));
});

test('wind components', () => {
  const c = windComponents(300, 20, 270);
  assert.ok(Math.abs(c.headwindKt - 20 * Math.cos(Math.PI / 6)) < 1e-9);
  assert.ok(c.crosswindKt > 0, 'from the right');
});

test('filters: lag step response, angle lag across north', () => {
  const l = new Lag(1);
  l.update(0, 0);
  let y = 0;
  for (let i = 0; i < 100; ++i) y = l.update(1, 0.01);
  assert.ok(Math.abs(y - (1 - Math.exp(-1))) < 1e-9);
  const a = new AngleLag(1);
  a.update(350, 0);
  const out = a.update(10, 1).deg;
  assert.ok(Math.abs(wrap180(out - 0)) < 10, `filtered direction ${out} stays near north`);
});
