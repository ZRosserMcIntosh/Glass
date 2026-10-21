// Glass — engine indication and crew alerting display.
// SPDX-License-Identifier: MIT

import { P, MONO, D2R, prepare, text, line, box, stripes, colourOf, fmt } from './draw.js';
import { LEVEL } from '../avionics/eicas.js';

export const EICAS_SIZE = { w: 600, h: 820 };
const LEVEL_COLOUR = { [LEVEL.WARNING]: P.red, [LEVEL.CAUTION]: P.amber, [LEVEL.ADVISORY]: P.cyan, [LEVEL.STATUS]: P.white };

export function drawEicas(canvas, m) {
  const ctx = prepare(canvas, EICAS_SIZE.w, EICAS_SIZE.h);
  if (!m || !m.powered) return;
  const e = m.eicas, L = e.limits;
  const cols = e.engines.length === 1 ? [300] : [150, 450];
  e.engines.forEach((eng, i) => {
    const x = cols[i];
    dial(ctx, x, 138, 80, eng.n1, 0, L.n1Max, [{ v: L.n1Red, c: P.red }], eng.n1Colour, eng.running || eng.n1 > 1, fmt(eng.n1, 1));
    if (eng.reverserState) text(ctx, 'REV', x - 60, 40, { size: 22, weight: 700, colour: eng.reverserState === 'REV' ? P.green : P.amber });
    dial(ctx, x, 312, 64, eng.ittC, 0, L.ittMax, [{ v: L.ittAmber, c: P.amber }, { v: L.ittRed, c: P.red }], eng.ittColour, true, fmt(eng.ittC));
    readout(ctx, x, 404, fmt(eng.n2, 1), 'white');
    readout(ctx, x, 448, fmt(eng.fuelFlowKgH), 'white');
    readout(ctx, x, 492, fmt(eng.oilPsi), eng.oilColour);
    if (eng.starting) text(ctx, 'START', x, 236, { size: 18, colour: P.green });
  });
  text(ctx, 'N1', 300, 138, { size: 20, colour: P.grey });
  text(ctx, 'ITT', 300, 312, { size: 20, colour: P.grey });
  text(ctx, 'N2', 300, 404, { size: 18, colour: P.grey });
  text(ctx, 'FF KG/H', 300, 448, { size: 16, colour: P.grey });
  text(ctx, 'OIL PSI', 300, 492, { size: 16, colour: P.grey });

  // Fuel, weight and temperatures.
  line(ctx, 20, 522, 580, 522, P.dim, 1.5);
  text(ctx, `FUEL ${fmt(e.fuelKg)} KG`, 24, 546, { size: 20, align: 'left', font: MONO, colour: e.cas.list.some((c) => c.text === 'FUEL LOW') ? P.amber : P.white });
  text(ctx, `GW ${fmt(e.grossKg)} KG`, 576, 546, { size: 18, align: 'right', font: MONO });
  text(ctx, `SAT ${fmt(e.satC)}°C  TAT ${fmt(e.tatC)}°C`, 24, 574, { size: 17, align: 'left', font: MONO, colour: P.grey });

  // Configuration: gear, flaps, speedbrake, trim.
  gear(ctx, 480, 632, e.gear, e.gearLeverDown);
  flaps(ctx, 24, 626, e.flapDeg, e.landingFlapDeg, e.takeoffFlapDeg);
  text(ctx, `SPDBRK ${e.groundSpoilers ? 'GND' : e.speedbrake > 0.05 ? fmt(e.speedbrake * 100) + '%' : 'RET'}`, 24, 680, { size: 17, align: 'left', colour: e.speedbrake > 0.05 ? P.amber : P.white });
  text(ctx, `TRIM ${e.trim >= 0 ? 'NU' : 'ND'} ${fmt(Math.abs(e.trim * 100))}`, 220, 680, { size: 17, align: 'left' });

  // Crew alerting messages, warnings first, newest first within a level.
  line(ctx, 20, 700, 580, 700, P.dim, 1.5);
  const list = e.cas.list.slice(0, 4);
  list.forEach((c, i) => text(ctx, c.text, 30, 722 + i * 24, { size: 21, align: 'left', weight: 700, colour: LEVEL_COLOUR[c.level] }));
  if (e.cas.list.length > list.length) text(ctx, `+${e.cas.list.length - list.length} MORE`, 570, 800, { size: 16, align: 'right', colour: P.grey });
  if (e.cas.inhibited) text(ctx, 'T/O INHIBIT', 570, 722, { size: 16, align: 'right', colour: P.grey });
}

function dial(ctx, x, y, r, v, lo, hi, marks, colourName, live, label) {
  const a = (val) => (-120 + (225 * (Math.min(Math.max(val, lo), hi) - lo)) / (hi - lo)) * D2R;
  const colour = colourOf(colourName);
  if (live && Number.isFinite(v)) {
    ctx.fillStyle = 'rgba(150,155,165,0.35)';
    ctx.beginPath(); ctx.moveTo(x, y); ctx.arc(x, y, r, a(lo) - Math.PI / 2, a(v) - Math.PI / 2); ctx.closePath(); ctx.fill();
  }
  ctx.strokeStyle = P.white; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.arc(x, y, r, a(lo) - Math.PI / 2, a(hi) - Math.PI / 2); ctx.stroke();
  for (const mk of marks) {
    const t = a(mk.v) - Math.PI / 2;
    line(ctx, x + Math.cos(t) * (r - 4), y + Math.sin(t) * (r - 4), x + Math.cos(t) * (r + 12), y + Math.sin(t) * (r + 12), mk.c, 4);
  }
  if (live && Number.isFinite(v)) {
    const t = a(v) - Math.PI / 2;
    line(ctx, x, y, x + Math.cos(t) * r, y + Math.sin(t) * r, colour, 3.5);
  }
  box(ctx, x + 8, y - r - 28, 92, 32, { stroke: colour === P.white ? P.grey : colour, fill: '#000', width: 2 });
  text(ctx, live ? label : '--', x + 94, y - r - 11, { size: 24, align: 'right', font: MONO, colour });
}

function readout(ctx, x, y, s, colourName) {
  text(ctx, s, x + (x < 300 ? -40 : 40), y, { size: 24, font: MONO, colour: colourOf(colourName) });
}

function gear(ctx, x, y, pos, leverDown) {
  const transit = pos > 0.01 && pos < 0.999;
  for (const [dx, dy] of [[0, 0], [-46, 40], [46, 40]]) {
    const bx = x + dx - 22, by = y + dy - 16;
    if (transit) stripes(ctx, bx, by, 44, 32, P.amber, '#000', 10);
    else if (pos >= 0.999) box(ctx, bx, by, 44, 32, { stroke: P.green, fill: '#000' });
    else box(ctx, bx, by, 44, 32, { stroke: P.white, fill: '#000' });
    if (!transit) text(ctx, pos >= 0.999 ? 'DN' : 'UP', x + dx, y + dy, { size: 18, colour: pos >= 0.999 ? P.green : P.white, weight: 700 });
  }
  text(ctx, 'GEAR', x, y - 32, { size: 15, colour: leverDown ? P.green : P.grey });
}

function flaps(ctx, x, y, deg, landing, takeoff) {
  const w = 160, frac = landing > 0 ? Math.min(deg / landing, 1) : 0;
  text(ctx, 'FLAPS', x, y - 14, { size: 15, align: 'left', colour: P.grey });
  box(ctx, x, y, w, 16, { stroke: P.white, fill: '#000' });
  ctx.fillStyle = P.white; ctx.fillRect(x, y, w * frac, 16);
  if (landing > 0) {
    const tx = x + (w * takeoff) / landing;
    line(ctx, tx, y - 4, tx, y + 20, P.cyan, 2);
  }
  text(ctx, `${fmt(deg)}°`, x + w + 12, y + 8, { size: 20, align: 'left', font: MONO });
}
