// Glass — a synthetic decoded frame for the unit tests: every channel zero
// except a level, powered, two-engine business jet, then the overrides.
// SPDX-License-Identifier: MIT
import { decode } from '../../src/avionics/readout.js';

export function frame(edit = () => {}) {
  const s = new Float64Array(64), c = new Float64Array(96), j = new Float64Array(96), g = new Float64Array(16);
  c[0] = 250; c[1] = 10000; c[2] = 29.92; c[43] = 1; c[37] = 0;
  j[0] = 2; j[24] = 0.8; j[25] = 320; j[32] = 110; j[33] = 100; j[34] = 95; j[41] = -4.8; j[42] = 1; j[45] = 28; j[49] = 15; j[50] = 35;
  for (const i of [0, 1]) { const b = 8 + 8 * i; j[b] = 80; j[b + 1] = 90; j[b + 2] = 700; j[b + 3] = 800; j[b + 5] = 1; j[b + 7] = 1; j[30 + i] = 60; j[58 + i] = 0.7; j[60 + i] = 1; }
  j[44] = 1; j[68] = 1;
  s[12] = 280;
  const f = decode(s, c, j, g);
  edit(f);
  return f;
}
