// Glass — the avionics against the flight core itself (dist/core, built by
// scripts/build_core.sh): each glass-profile aircraft flown in the core, the
// avionics fed its sensed readouts only, and the result checked against the
// core's truth channels.
// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFile, readdir, access } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import { readFrame, EXPECTED_CORE_ABI } from '../src/avionics/readout.js';
import { GlassAvionics } from '../src/avionics/glass.js';
import { casFromMach, pressureFromAltitude, FT } from '../src/avionics/air_data.js';

const core = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'dist', 'core');
await access(path.join(core, 'avd.mjs')).catch(() => { throw new Error('dist/core missing: run scripts/build_core.sh first'); });
const { default: createAvdModule } = await import(path.join(core, 'avd.mjs'));
const ids = [];
for (const f of (await readdir(core)).filter((f) => f.endsWith('.json')).sort()) {
  if (JSON.parse(await readFile(path.join(core, f), 'utf8')).panel?.profile === 'business_jet_glass') ids.push(f.replace(/\.json$/, ''));
}

async function fly(id, { mode = 3, weather = null } = {}) {
  const json = await readFile(path.join(core, id + '.json'), 'utf8');
  const A = await createAvdModule();
  assert.equal(A.abiVersion, EXPECTED_CORE_ABI, 'core ABI');
  assert.deepEqual(A.load(json), []);
  if (weather) A.avd_set_weather(...weather);
  A.avd_set_assist(2);
  A.avd_reset(mode);
  const G = new GlassAvionics({ aircraft: JSON.parse(json), markings: A.markings(), loadsheet: A.loadsheet(false), controls: A.cockpit() });
  const lever = { v: mode === 1 ? 0 : A.avd_trim_throttle() };
  let m = null;
  const run = (seconds, inputs = () => {}) => {
    for (let t = 0; t < seconds; t += 1 / 30) {
      const u = { pitch: 0, roll: 0, brake: 0 };
      inputs(u);
      A.avd_set_input(u.pitch, u.roll, 0, lever.v, 1, u.brake, u.brake, 0, 0);
      A.avd_advance(1 / 30);
      m = G.update(readFrame(A), 1 / 30);
    }
    return m;
  };
  return { A, G, run, lever, frame: () => readFrame(A) };
}
const texts = (m) => m.eicas.cas.list.map((c) => c.text);

test('the glass-profile aircraft are all here', () => {
  assert.deepEqual(ids, ['b738', 'cj4', 'crj700', 'learjet45', 'phenom300', 'top_g']);
});

for (const id of ids) {
  test(`${id}: cruise air data against the core's truth, a clean EICAS`, async () => {
    const s = await fly(id);
    const m = s.run(10), f = s.frame();
    assert.ok(Math.abs(m.air.mach - f.truth.mach) < 0.003, `Mach ${m.air.mach} vs ${f.truth.mach}`);
    assert.ok(Math.abs(m.air.tasKt - f.truth.tasKt) < 2, `TAS ${m.air.tasKt} vs ${f.truth.tasKt}`);
    assert.ok(Math.abs(m.air.pressureAltFt - f.truth.pressureAltFt) < 30, `pressure altitude ${m.air.pressureAltFt} vs ${f.truth.pressureAltFt}`);
    assert.equal(m.speed.maxBy, 'VMO');
    assert.ok(m.speed.iasKt > m.speed.minManoeuvreKt && !m.speed.lowSpeed && !m.speed.overspeed);
    assert.deepEqual(texts(m).filter((t) => t !== 'GND SPOILERS'), []);
    assert.equal(m.eicas.cas.masterWarning || m.eicas.cas.masterCaution, false);
    assert.equal(m.eicas.engines.length, 2);
    for (const e of m.eicas.engines) assert.ok(e.n1Colour === 'white' && e.ittColour === 'white' && e.oilColour === 'white', JSON.stringify(e));
  });

  test(`${id}: on the ILS final, VAPP is the core's own approach speed`, async () => {
    const s = await fly(id, { mode: 2 });
    const spawnIas = s.frame().air.iasKt;
    const m = s.run(3);
    assert.equal(m.nav.source, 'LOC');
    assert.ok(Math.abs(m.nav.lateralDots) < 0.3 && m.nav.verticalValid && Math.abs(m.nav.verticalDots) < 0.5, JSON.stringify(m.nav));
    // The core spawns at 1.23 Vs(landing flap) + 5 kt; Glass: VREF = 1.23 VSR0, VAPP = VREF + 5 in calm air.
    assert.ok(Math.abs(m.speed.vapp - spawnIas) < 3, `VAPP ${m.speed.vapp} vs spawn ${spawnIas}`);
    assert.ok(m.speed.iasKt > m.speed.minManoeuvreKt, 'final speed above the amber band');
    assert.ok(m.altitude.radioAltShown);
    assert.ok(!texts(m).includes('CONFIG GEAR'), texts(m).join());
  });

  test(`${id}: take-off configuration warning`, async () => {
    const s = await fly(id, { mode: 1 });
    s.run(1);
    s.lever.v = 1;
    let m = s.run(0.5, (u) => { u.brake = 1; });
    assert.ok(texts(m).includes('CONFIG FLAPS'), `flaps up at take-off power: ${texts(m).join()}`);
    assert.equal(m.eicas.cas.masterWarning, true);
    // Set the take-off flap (the detent whose label is the core's take-off flap) at idle, then advance the levers.
    s.lever.v = 0;
    const flaps = () => s.A.cockpit().find((c) => c.id === 'flaps.lever');
    const want = s.frame().limits.takeoffFlapDeg;
    for (let k = 0; k < flaps().positions.length && Math.abs(parseFloat(flaps().positions[flaps().position]) - want) > 0.5; ++k) s.A.avd_cockpit_cycle(flaps().index);
    // The engines-running spawn has the parking brake set: CONFIG PARK BRK until released.
    assert.ok(texts(m).includes('CONFIG PARK BRK'));
    s.A.avd_cockpit_cycle(s.A.cockpit().find((c) => c.id === 'brakes.parking').index);
    s.run(20, (u) => { u.brake = 1; });
    s.lever.v = 1;
    m = s.run(0.5, (u) => { u.brake = 1; });
    assert.ok(!texts(m).some((t) => t.startsWith('CONFIG')), `take-off flap ${m.eicas.flapDeg} set, brake off: ${texts(m).join()}`);
  });
}

test('wind triangle against the core\'s wind, and its components', async () => {
  const s = await fly('top_g', { weather: [20, 300, 0, 0, 0.6] });
  const m = s.run(25), f = s.frame();
  const d = Math.abs(((m.wind.fromDeg - f.truth.windFromDeg + 540) % 360) - 180);
  assert.ok(d < 5 && Math.abs(m.wind.speedKt - f.truth.windKt) < 2, `wind ${m.wind.fromDeg}/${m.wind.speedKt} vs ${f.truth.windFromDeg}/${f.truth.windKt}`);
});

test('high altitude: MMO sets the barber pole, at the CAS of MMO', async () => {
  const json = await readFile(path.join(core, 'b738.json'), 'utf8');
  const A = await createAvdModule();
  A.load(json);
  A.avd_reset_air(0, -20000, 10000, 330, 250, 0);
  const G = new GlassAvionics({ aircraft: JSON.parse(json), markings: A.markings(), loadsheet: A.loadsheet(false), controls: A.cockpit() });
  let m;
  for (let i = 0; i < 60; ++i) { A.avd_set_input(0, 0, 0, A.avd_trim_throttle(), 1, 0, 0, 0, 0); A.avd_advance(1 / 30); m = G.update(readFrame(A), 1 / 30); }
  const f = readFrame(A);
  assert.ok(f.air.altitudeFt > 33000, `altitude ${f.air.altitudeFt}`);
  assert.equal(m.speed.maxBy, 'MMO');
  const expected = casFromMach(f.limits.mmo, pressureFromAltitude(f.truth.pressureAltFt * FT));
  assert.ok(Math.abs(m.speed.maxKt - expected) < 1.5 && m.speed.maxKt < f.limits.vmoKt, `max ${m.speed.maxKt} vs ${expected}`);
});

test('generator failure: per-engine GEN 1 OFF on the Top G, master caution', async () => {
  const s = await fly('top_g');
  s.run(1);
  s.A.avd_failure(4, 1);
  const m = s.run(1);
  assert.ok(texts(m).includes('GEN 1 OFF'), texts(m).join());
  assert.equal(m.eicas.cas.masterCaution, true);
});

test('engine shutdown in flight: advisory, the other engine carries on', async () => {
  const s = await fly('cj4');
  s.run(1);
  const sw = s.A.cockpit().find((c) => c.id === 'engine1.fuel');
  s.A.avd_cockpit_cycle(sw.index);
  const m = s.run(15);
  assert.ok(texts(m).includes('ENG 1 SHUTDOWN'), texts(m).join());
  assert.ok(m.eicas.engines[0].n1 < 30 && m.eicas.engines[1].running);
});
