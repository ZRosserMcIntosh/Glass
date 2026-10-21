// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { projector, selectSource, navGuidance, NM, estimates } from '../../src/avionics/navigation.js';
import { frame } from './frame.mjs';

test('heading-up projection', () => {
  const p = projector({ east: 0, north: 0 }, 90);
  const ahead = p(NM, 0), left = p(0, NM);
  assert.ok(Math.abs(ahead.x) < 1e-9 && Math.abs(ahead.y - 1) < 1e-9, 'east is up on a 090 heading');
  assert.ok(Math.abs(left.x + 1) < 1e-9 && Math.abs(left.y) < 1e-9, 'north is to the left');
});

test('source selection: localizer first, then GPS, then VOR', () => {
  assert.equal(selectSource(frame()), 'VOR');
  assert.equal(selectSource(frame((f) => { f.gps.valid = true; f.gps.active = 1; })), 'GPS');
  assert.equal(selectSource(frame((f) => { f.gps.valid = true; f.gps.active = 1; f.nav.valid = true; f.nav.localizer = true; })), 'LOC');
  assert.equal(selectSource(frame(), 'GPS'), 'GPS');
});

test('GPS guidance: magnetic course, dots, distance', () => {
  const f = frame((f) => { Object.assign(f.gps, { valid: true, active: 2, desiredTrackTrueDeg: 80, cdi: 0.5, distanceToWaypointM: 5 * NM, bearingTrueDeg: 82, verticalValid: true, vdi: -0.25 }); });
  const g = navGuidance(f, 'GPS', -21.5);
  assert.ok(Math.abs(g.courseMagDeg - 101.5) < 1e-9);
  assert.equal(g.lateralDots, 1);
  assert.equal(g.verticalDots, -0.5);
  assert.equal(g.distanceNm, 5);
});

test('estimates', () => {
  const e = estimates(60, 240, 1000);
  assert.equal(e.minutes, 15);
  assert.equal(e.fuelKg, 250);
  assert.equal(estimates(10, 0, 100).minutes, null);
});
