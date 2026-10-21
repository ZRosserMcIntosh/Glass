// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Cas, LEVEL, engineLimits, gaugeColour } from '../../src/avionics/eicas.js';
import { frame } from './frame.mjs';

const lim = engineLimits([], { itt_takeoff_k: 1073.15 });
const texts = (out) => out.list.map((m) => m.text);

test('clean cruise: no messages, no master light', () => {
  const cas = new Cas({ engineLimits: lim, perEngineGenerators: true, fuelCapacityKg: 5000 });
  const out = cas.update(frame((f) => { f.fuel.kg = 3000; }), 0.1);
  assert.deepEqual(texts(out), []);
  assert.equal(out.masterCaution || out.masterWarning, false);
});

test('take-off configuration warnings', () => {
  const cas = new Cas({ engineLimits: lim });
  const out = cas.update(frame((f) => { f.onGround = true; f.air.iasKt = 0; f.config.flapDeg = 0; f.parkingBrake = true; f.config.speedbrakeDetent = 1; }), 0.1);
  for (const t of ['CONFIG FLAPS', 'CONFIG PARK BRK', 'CONFIG SPOILERS']) assert.ok(texts(out).includes(t), t);
  assert.equal(out.masterWarning, true);
  const ok = cas.update(frame((f) => { f.onGround = true; f.air.iasKt = 0; f.config.flapDeg = 15; }), 0.1);
  assert.ok(!texts(ok).some((t) => t.startsWith('CONFIG')));
});

test('master caution latches until acknowledged; the message follows its condition', () => {
  const cas = new Cas({ engineLimits: lim, perEngineGenerators: true });
  const failed = frame((f) => { f.engines[0].generator = false; });
  assert.ok(texts(cas.update(failed, 0.1)).includes('GEN 1 OFF'));
  assert.equal(cas.masterCaution, true);
  cas.acknowledge();
  const again = cas.update(failed, 0.1);
  assert.equal(again.masterCaution, false, 'acknowledged');
  assert.ok(texts(again).includes('GEN 1 OFF'), 'message stays while the generator is off');
  assert.deepEqual(texts(cas.update(frame(), 0.1)), []);
});

test('one alternator: GEN OFF only with no generator on line', () => {
  const cas = new Cas({ engineLimits: lim, perEngineGenerators: false });
  assert.deepEqual(texts(cas.update(frame((f) => { f.engines[1].generator = false; }), 0.1)), []);
  assert.ok(texts(cas.update(frame((f) => { f.systems.anyGenerator = false; }), 0.1)).includes('GEN OFF'));
});

test('take-off inhibit: cautions held from 80 kt to 400 ft', () => {
  const cas = new Cas({ engineLimits: lim, perEngineGenerators: true });
  const roll = (ias) => frame((f) => { f.onGround = true; f.air.iasKt = ias; f.config.flapDeg = 15; f.engines[0].generator = false; f.engines.forEach((e) => { e.lever = 0.95; }); });
  assert.ok(texts(cas.update(roll(60), 0.1)).includes('GEN 1 OFF'));
  cas.reset();
  const out = cas.update(roll(100), 0.1);
  assert.equal(out.inhibited, true);
  assert.deepEqual(texts(out), []);
  const climb = (ra) => frame((f) => { f.radioAltFt = ra; f.engines[0].generator = false; f.engines.forEach((e) => { e.lever = 0.95; }); });
  assert.deepEqual(texts(cas.update(climb(200), 0.1)), []);
  assert.ok(texts(cas.update(climb(450), 0.1)).includes('GEN 1 OFF'));
});

test('persistent conditions: gear disagree after 20 s', () => {
  const cas = new Cas({ engineLimits: lim });
  const stuck = frame((f) => { f.config.gearLeverDown = true; f.config.gear = 0.4; f.radioAltFt = 3000; });
  let out;
  for (let t = 0; t < 19; t += 1) out = cas.update(stuck, 1);
  assert.ok(!texts(out).includes('GEAR DISAGREE'));
  for (let t = 0; t < 3; t += 1) out = cas.update(stuck, 1);
  assert.ok(texts(out).includes('GEAR DISAGREE'));
  assert.equal(out.list.find((m) => m.text === 'GEAR DISAGREE').level, LEVEL.CAUTION);
});

test('gauge colours', () => {
  assert.equal(gaugeColour('n1', 101, lim), 'white');
  assert.equal(gaugeColour('n1', 103, lim), 'red');
  assert.equal(gaugeColour('itt', 810, lim), 'amber');
  assert.equal(gaugeColour('itt', 830, lim), 'red');
  assert.equal(gaugeColour('oil', 20, lim, true), 'red');
  assert.equal(gaugeColour('oil', 20, lim, false), 'white');
});
