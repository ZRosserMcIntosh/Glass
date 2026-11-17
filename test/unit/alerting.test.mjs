// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { AltitudeAlert, Minimums, Callouts } from '../../src/avionics/alerting.js';

test('altitude alert: approach, capture, deviation', () => {
  const a = new AltitudeAlert(10000);
  assert.equal(a.update(8000).approaching, false);
  assert.equal(a.update(9200).approaching, true);
  assert.equal(a.update(9900).captured, true);
  assert.equal(a.update(10150).deviation, false);
  assert.equal(a.update(10250).deviation, true);
  a.select(12000);
  assert.equal(a.update(10250).deviation, false, 'a new selection clears the deviation');
});

test('minimums: approaching, reached, reset on the go-around', () => {
  const m = new Minimums('BARO', 2800);
  assert.equal(m.update(2950, 300, false).approaching, false);
  assert.equal(m.update(2880, 230, false).approaching, true);
  assert.equal(m.update(2795, 150, false).reached, true);
  assert.equal(m.update(2850, 200, false).reached, true, 'stays reached in the flare band');
  assert.equal(m.update(2900, 250, false).reached, false, 'reset climbing back above');
  const ra = new Minimums('RA', 200);
  assert.equal(ra.update(9999, 190, false).reached, true);
});

test('callouts: descending, each once', () => {
  const c = new Callouts();
  const heard = [];
  for (let h = 3000; h >= 0; h -= 5) heard.push(...c.update(h, -700, false));
  assert.deepEqual(heard, ['TWENTY FIVE HUNDRED', '1000', '500', '400', '300', '200', '100', '50', '40', '30', '20', '10']);
  for (let h = 0; h <= 3000; h += 5) assert.deepEqual(c.update(h, 1500, false), [], 'silent climbing');
});
