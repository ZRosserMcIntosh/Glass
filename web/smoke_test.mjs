// Glass Dynamics — Node smoke test of the WebAssembly build (web/dist).
// SPDX-License-Identifier: MIT
import { readFile } from 'node:fs/promises';
import createGlassModule from './dist/glass.mjs';

const G = await createGlassModule();
const errors = G.load(await readFile(new URL('./dist/library.json', import.meta.url), 'utf8'));
if (errors.length) { console.error(errors); process.exit(1); }
const cat = G.catalog();
const fail = (m) => { console.error('FAIL', m); process.exit(1); };
console.log(`${cat.glazings.length} glazings, ${cat.impactors.length} impactors`);
const gi = (id) => cat.glazings.findIndex((g) => g.id === id);
const ii = (id) => cat.impactors.findIndex((g) => g.id === id);

function run(glazing, impactor, speed, tier = 1) {
  if (!G.gls_new_pane(gi(glazing), tier, 1)) fail('new pane ' + glazing);
  const g = cat.glazings[gi(glazing)];
  G.gls_fire(ii(impactor), g.width / 2, g.height / 2, speed ?? cat.impactors[ii(impactor)].speed, 0, 0, 0, 0.05);
  const t0 = performance.now();
  for (let f = 0; f < 120; ++f) G.gls_advance(G.gls_time() + 1 / 60, 1 << 30);
  const s = G.state(), r = G.report(0);
  console.log(`${glazing} <- ${impactor}: ${r.outcome} | segs ${G.plies(s[2])[2]} frags ${s[4]} | ${(performance.now() - t0).toFixed(0)} ms`);
  return { s: Array.from(s), r };
}

const a = run('annealed_6', 'steel_ball_2260', 8.86);
if (!a.r.v[10]) fail('annealed pane should break');
const t = run('car_side_tempered', 'window_punch');
if (t.s[4] < 50) fail('tempered side glass should dice under a window punch');
const w = run('windscreen', 'steel_ball_2260', 8.86);
if (w.r.v[11]) fail('windscreen must hold the 2.26 kg ball from 4 m');
const b = run('ul752_l3', 'magnum_44');
if (!b.r.v[13]) fail('.44 Magnum must stop in the Level 3 panel');
const cr = G.cracks(0, 0);
if (cr.length % 7 !== 0 || cr.length === 0) fail('crack buffer');
const shapes = G.fragmentShapes(0);
const field = G.field(0, 16, 12);
if (field.length !== 3 * 16 * 12) fail('field');
console.log(`ok (${shapes.length} fragment shapes read, bench checksum ${G.gls_bench(2).toFixed(3)})`);
