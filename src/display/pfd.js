// Glass — primary flight display: attitude with the flight path vector,
// speed and altitude tapes, vertical speed, HSI and the mode strip.
// SPDX-License-Identifier: MIT

import { P, MONO, D2R, prepare, text, line, box, stripes, fmt, pad3 } from './draw.js';

export const PFD_SIZE = { w: 800, h: 820 };
const CX = 400, CY = 300, PPD = 8;                       // attitude centre, px per degree of pitch
const ADI = { x: 230, y: 95, w: 340, h: 410 };
const SPD = { x: 80, w: 104, ppk: 4.2 };                 // px per kt
const ALT = { x: 616, w: 112, ppf: 0.6 };                // px per ft
const VSI = { x: 738, w: 50, top: 130, bottom: 470 };
const HSI = { cx: 400, cy: 712, r: 160 };

export function drawPfd(canvas, m, opts = {}) {
  const ctx = prepare(canvas, PFD_SIZE.w, PFD_SIZE.h);
  if (!m || !m.powered) return;
  drawAttitude(ctx, m, opts);
  drawSpeedTape(ctx, m);
  drawAltitudeTape(ctx, m);
  drawVsi(ctx, m);
  drawHsi(ctx, m);
  drawModes(ctx, m);
}

// ------------------------------------------------------------- attitude ----
function drawAttitude(ctx, m, opts) {
  const a = m.attitude;
  if (a.failed) {
    box(ctx, ADI.x, ADI.y, ADI.w, ADI.h, { stroke: P.dim });
    text(ctx, 'ATT', CX, CY, { size: 40, colour: P.red, weight: 700 });
    return;
  }
  const pitch = a.pitchDeg, roll = a.rollDeg;
  ctx.save();
  roundRect(ctx, ADI.x, ADI.y, ADI.w, ADI.h, 18);
  ctx.clip();
  ctx.translate(CX, CY);
  ctx.rotate(-roll * D2R);
  ctx.translate(0, pitch * PPD);
  ctx.fillStyle = P.sky; ctx.fillRect(-800, -1600, 1600, 1600);
  ctx.fillStyle = P.earth; ctx.fillRect(-800, 0, 1600, 1600);
  line(ctx, -800, 0, 800, 0, P.white, 2);
  // Pitch ladder: 2.5 deg marks, labelled every 10.
  for (let d = -90; d <= 90; d += 2.5) {
    if (d === 0 || Math.abs(d - pitch) > 24) continue;
    const y = -d * PPD, ten = d % 10 === 0, five = d % 5 === 0;
    const half = ten ? 56 : five ? 30 : 14;
    line(ctx, -half, y, half, y, P.white, ten ? 2.2 : 1.6);
    if (ten) { text(ctx, String(Math.abs(d)), -half - 22, y, { size: 17 }); text(ctx, String(Math.abs(d)), half + 22, y, { size: 17 }); }
  }
  ctx.restore();

  // Roll scale and pointer, with the slip/skid indication under the pointer.
  const R = 172;
  ctx.save();
  ctx.translate(CX, CY);
  ctx.strokeStyle = P.white; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(0, 0, R, (-90 - 60) * D2R, (-90 + 60) * D2R); ctx.stroke();
  for (const t of [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]) {
    const len = Math.abs(t) === 30 || Math.abs(t) === 60 ? 18 : 10;
    const c = Math.cos((t - 90) * D2R), s = Math.sin((t - 90) * D2R);
    line(ctx, c * R, s * R, c * (R + len), s * (R + len));
  }
  ctx.fillStyle = P.white;
  ctx.beginPath(); ctx.moveTo(0, -R - 2); ctx.lineTo(-9, -R - 16); ctx.lineTo(9, -R - 16); ctx.fill();
  ctx.rotate(-roll * D2R);
  const bankColour = Math.abs(roll) > 35 ? P.amber : P.white;
  ctx.fillStyle = bankColour;
  ctx.beginPath(); ctx.moveTo(0, -R + 2); ctx.lineTo(-10, -R + 18); ctx.lineTo(10, -R + 18); ctx.fill();
  const slip = Math.max(-16, Math.min(16, a.ball)) * 1.6;
  ctx.fillRect(-12 + slip, -R + 21, 24, 6);
  ctx.restore();

  // Flight path vector: the inertial flight-path angle and drift, rotated with the horizon.
  const fp = m.flightPath;
  if (fp.valid && !m.onGround) {
    const px = fp.driftDeg * PPD, py = (pitch - fp.fpaDeg) * PPD;
    const c = Math.cos(-roll * D2R), s = Math.sin(-roll * D2R);
    const x = CX + px * c - py * s, y = CY + px * s + py * c;
    if (x > ADI.x + 20 && x < ADI.x + ADI.w - 20 && y > ADI.y + 20 && y < ADI.y + ADI.h - 20) {
      ctx.save(); ctx.translate(x, y); ctx.rotate(-roll * D2R);
      ctx.strokeStyle = P.green; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.arc(0, 0, 9, 0, 2 * Math.PI); ctx.stroke();
      line(ctx, -26, 0, -9, 0, P.green, 2.5); line(ctx, 9, 0, 26, 0, P.green, 2.5); line(ctx, 0, -9, 0, -20, P.green, 2.5);
      ctx.restore();
    }
  }

  // Aircraft symbol (fixed).
  ctx.fillStyle = '#000'; ctx.strokeStyle = P.white; ctx.lineWidth = 2;
  for (const sgn of [-1, 1]) {
    ctx.beginPath();
    ctx.moveTo(CX + sgn * 50, CY - 4); ctx.lineTo(CX + sgn * 120, CY - 4); ctx.lineTo(CX + sgn * 120, CY + 4);
    ctx.lineTo(CX + sgn * 62, CY + 4); ctx.lineTo(CX + sgn * 62, CY + 20); ctx.lineTo(CX + sgn * 50, CY + 20); ctx.closePath();
    ctx.fill(); ctx.stroke();
  }
  ctx.fillRect(CX - 5, CY - 5, 10, 10); ctx.strokeRect(CX - 5, CY - 5, 10, 10);

  // Deviation scales: lateral under the ADI, vertical at its right.
  const n = m.nav, devColour = n.source === 'GPS' ? P.magenta : P.green;
  if (n.valid) {
    scaleDots(ctx, CX, ADI.y + ADI.h - 22, true);
    diamond(ctx, CX + clampDots(n.lateralDots) * 34, ADI.y + ADI.h - 22, devColour, Math.abs(n.lateralDots) > 2.2);
  }
  if (n.verticalValid) {
    scaleDots(ctx, ADI.x + ADI.w - 18, CY, false);
    diamond(ctx, ADI.x + ADI.w - 18, CY - clampDots(n.verticalDots) * 34, devColour, Math.abs(n.verticalDots) > 2.2);
  }

  // Radio altitude and minimums.
  const alt = m.altitude;
  if (alt.radioAltShown) {
    const reached = alt.minimums.kind === 'RA' && alt.minimums.reached;
    text(ctx, fmt(Math.max(alt.radioAltFt, 0)), CX, ADI.y + ADI.h - 56, { size: 28, colour: reached ? P.amber : P.white, font: MONO });
  }
  if (alt.minimums.reached) text(ctx, 'MINIMUMS', CX, ADI.y + 150, { size: 24, colour: P.amber, weight: 700 });
  else if (alt.minimums.approaching) text(ctx, 'APPR MINS', CX, ADI.y + 150, { size: 18, colour: P.white });

  // Wind (upper left of the ADI): arrow relative to the nose and from/speed.
  if (m.wind) {
    const w = m.wind;
    text(ctx, `${pad3(w.fromDeg - m.heading.magVarDeg)}°/${fmt(w.speedKt)}`, ADI.x + 10, ADI.y + 22, { size: 17, align: 'left', font: MONO });
    ctx.save(); ctx.translate(ADI.x + 30, ADI.y + 60); ctx.rotate((w.relativeDeg + 180) * D2R);
    line(ctx, 0, -14, 0, 14, P.white, 2.5);
    ctx.fillStyle = P.white; ctx.beginPath(); ctx.moveTo(0, -18); ctx.lineTo(-6, -8); ctx.lineTo(6, -8); ctx.fill();
    ctx.restore();
  }
  if (opts.callout) text(ctx, opts.callout, CX, ADI.y + 190, { size: 20, colour: P.green, weight: 700 });
}

const clampDots = (d) => Math.max(-2.4, Math.min(2.4, d));
function scaleDots(ctx, x, y, horizontal) {
  ctx.strokeStyle = P.white; ctx.lineWidth = 2;
  for (const k of [-2, -1, 1, 2]) {
    ctx.beginPath(); ctx.arc(horizontal ? x + k * 34 : x, horizontal ? y : y + k * 34, 5, 0, 2 * Math.PI); ctx.stroke();
  }
  horizontal ? line(ctx, x, y - 10, x, y + 10, P.white, 2) : line(ctx, x - 10, y, x + 10, y, P.white, 2);
}
function diamond(ctx, x, y, colour, hollow) {
  ctx.beginPath(); ctx.moveTo(x, y - 11); ctx.lineTo(x + 8, y); ctx.lineTo(x, y + 11); ctx.lineTo(x - 8, y); ctx.closePath();
  if (hollow) { ctx.strokeStyle = colour; ctx.lineWidth = 2; ctx.stroke(); } else { ctx.fillStyle = colour; ctx.fill(); }
}
function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}

// ----------------------------------------------------------- speed tape ----
function drawSpeedTape(ctx, m) {
  const s = m.speed, ias = Math.max(s.iasKt, 30), k = SPD.ppk, x = SPD.x, w = SPD.w;
  const top = ADI.y, bottom = ADI.y + ADI.h, yOf = (kt) => CY - (kt - ias) * k;
  ctx.save();
  ctx.beginPath(); ctx.rect(x, top, w, bottom - top); ctx.clip();
  ctx.fillStyle = P.tape; ctx.fillRect(x, top, w, bottom - top);
  for (let v = Math.floor((ias - 60) / 10) * 10; v <= ias + 60; v += 10) {
    if (v < 30) continue;
    const y = yOf(v);
    line(ctx, x + w - 18, y, x + w, y, P.white, 2);
    if (v % 20 === 0) text(ctx, String(v), x + w - 26, y, { size: 22, align: 'right' });
  }
  // Low-speed bands (airborne) and the barber pole above the maximum speed.
  const bx = x + w - 10;
  if (!m.onGround) {
    stripes(ctx, bx, yOf(s.stallKt), 10, bottom - yOf(s.stallKt), P.red, '#000', 12);
    ctx.fillStyle = P.amber; ctx.fillRect(bx + 3, yOf(s.minManoeuvreKt), 4, yOf(s.stallKt) - yOf(s.minManoeuvreKt));
  }
  stripes(ctx, bx, top, 10, Math.max(0, yOf(s.maxKt) - top), P.red, '#000', 12);
  // Reference speed bugs.
  const bug = (kt, label, colour) => {
    const y = yOf(kt);
    if (y < top || y > bottom) return;
    line(ctx, x + w - 4, y, x + w + 12, y, colour, 3);
    text(ctx, label, x + w + 14, y, { size: 15, align: 'left', colour });
  };
  ctx.restore();
  if (m.onGround) bug(s.v2, 'V2', P.cyan);
  else if (m.eicas.flapDeg > 0.5 || m.altitude.ft < 10000) { bug(s.vref, 'REF', P.green); bug(s.vapp, 'APP', P.magenta); }
  // Trend vector.
  if (Math.abs(s.trendKt) > 2) {
    const y1 = Math.max(top, Math.min(bottom, CY - s.trendKt * k));
    line(ctx, x + w - 2, CY, x + w - 2, y1, P.green, 3);
    const d = Math.sign(s.trendKt);
    ctx.fillStyle = P.green; ctx.beginPath(); ctx.moveTo(x + w - 2, y1 - d * 8); ctx.lineTo(x + w - 9, y1 + d * 2); ctx.lineTo(x + w + 5, y1 + d * 2); ctx.fill();
  }
  // Readout.
  const colour = s.overspeed ? P.red : s.lowSpeed ? P.amber : P.white;
  readoutBox(ctx, x + 4, CY - 24, w - 22, 48, colour);
  text(ctx, fmt(s.iasKt), x + w - 24, CY + 1, { size: 32, align: 'right', font: MONO, colour });
  text(ctx, 'IAS', x + w / 2, top - 16, { size: 16, colour: P.grey });
  if (s.machShown) text(ctx, '.' + String(Math.round(s.mach * 1000)).padStart(3, '0'), x + w / 2, bottom + 22, { size: 24, font: MONO });
  else text(ctx, `GS ${fmt(m.heading.groundSpeedKt)}`, x + w / 2, bottom + 22, { size: 18, colour: P.white });
}

function readoutBox(ctx, x, y, w, h, stroke) {
  ctx.fillStyle = '#000'; ctx.fillRect(x, y, w, h);
  ctx.strokeStyle = stroke; ctx.lineWidth = 2.5; ctx.strokeRect(x, y, w, h);
}

// -------------------------------------------------------- altitude tape ----
function drawAltitudeTape(ctx, m) {
  const a = m.altitude, alt = a.ft, k = ALT.ppf, x = ALT.x, w = ALT.w;
  const top = ADI.y, bottom = ADI.y + ADI.h, yOf = (ft) => CY - (ft - alt) * k;
  ctx.save();
  ctx.beginPath(); ctx.rect(x, top, w, bottom - top); ctx.clip();
  ctx.fillStyle = P.tape; ctx.fillRect(x, top, w, bottom - top);
  for (let v = Math.floor((alt - 400) / 100) * 100; v <= alt + 400; v += 100) {
    const y = yOf(v);
    line(ctx, x, y, x + (v % 500 === 0 ? 18 : 10), y, P.white, 2);
    if (v % 200 === 0) text(ctx, String(v), x + 24, y, { size: 19, align: 'left', font: MONO });
  }
  // Ground under the radio altitude (amber hatching), and the minimums bug.
  if (a.radioAltShown) {
    const yg = yOf(alt - a.radioAltFt);
    if (yg < bottom) stripes(ctx, x, yg, 14, bottom - yg, P.amber, '#000', 10);
  }
  const mins = a.minimums, minsAlt = mins.kind === 'RA' ? (a.radioAltShown ? alt - a.radioAltFt + mins.valueFt : null) : mins.valueFt;
  if (minsAlt !== null) line(ctx, x - 2, yOf(minsAlt), x + w, yOf(minsAlt), mins.reached ? P.amber : P.green, 3);
  // Selected altitude bug.
  const ys = Math.max(top + 6, Math.min(bottom - 6, yOf(a.alert.selectedFt)));
  ctx.strokeStyle = P.cyan; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(x, ys - 16); ctx.lineTo(x + 10, ys - 16); ctx.lineTo(x + 10, ys - 6); ctx.lineTo(x + 3, ys); ctx.lineTo(x + 10, ys + 6); ctx.lineTo(x + 10, ys + 16); ctx.lineTo(x, ys + 16); ctx.stroke();
  ctx.restore();
  // Trend vector (6 s).
  if (Math.abs(a.trendFt) > 20) {
    const y1 = Math.max(top, Math.min(bottom, CY - a.trendFt * k));
    line(ctx, x + 2, CY, x + 2, y1, P.green, 3);
  }
  // Readout: thousands large, hundreds and below smaller.
  const colour = P.white;
  readoutBox(ctx, x + 6, CY - 26, w - 4, 52, colour);
  const v = Math.round(alt / 20) * 20, neg = v < 0, av = Math.abs(v);
  const thousands = Math.floor(av / 1000), rest = String(av % 1000).padStart(3, '0');
  text(ctx, (neg ? '-' : '') + (thousands ? String(thousands) : ''), x + 56, CY + 1, { size: 32, align: 'right', font: MONO });
  text(ctx, rest, x + 58, CY + 3, { size: 24, align: 'left', font: MONO });
  // Selected altitude above, the baro setting below.
  const al = a.alert, selColour = al.deviation ? P.amber : P.cyan;
  if (al.approaching || al.deviation) box(ctx, x + 2, top - 44, w, 36, { stroke: al.deviation ? P.amber : P.white, width: 2.5 });
  text(ctx, String(al.selectedFt), x + w / 2 + 2, top - 26, { size: 24, font: MONO, colour: selColour });
  text(ctx, a.std ? 'STD' : `${a.settingInHg.toFixed(2)} IN`, x + w / 2, bottom + 22, { size: 20, colour: P.cyan, font: MONO });
  text(ctx, `${mins.kind} ${mins.valueFt}`, x + w / 2, bottom + 48, { size: 16, colour: mins.reached ? P.amber : P.green });
}

// ----------------------------------------------------------------- VSI ----
// Nonlinear scale: 0-1000 fpm on 40 % of the half height, 1000-2000 on the
// next 30 %, 2000-6000 on the rest.
export function vsiFraction(fpm) {
  const a = Math.min(Math.abs(fpm), 6000);
  const f = a <= 1000 ? 0.4 * a / 1000 : a <= 2000 ? 0.4 + 0.3 * (a - 1000) / 1000 : 0.7 + 0.3 * (a - 2000) / 4000;
  return Math.sign(fpm) * f;
}
function drawVsi(ctx, m) {
  const vs = m.altitude.vsFpm, x = VSI.x, mid = (VSI.top + VSI.bottom) / 2, half = (VSI.bottom - VSI.top) / 2;
  ctx.fillStyle = P.tape;
  ctx.beginPath(); ctx.moveTo(x, VSI.top + 30); ctx.lineTo(x + 22, VSI.top); ctx.lineTo(x + VSI.w, VSI.top); ctx.lineTo(x + VSI.w, VSI.bottom); ctx.lineTo(x + 22, VSI.bottom); ctx.lineTo(x, VSI.bottom - 30); ctx.closePath(); ctx.fill();
  for (const t of [0, 500, 1000, 1500, 2000, 4000, 6000]) for (const sg of t ? [-1, 1] : [1]) {
    const y = mid - sg * vsiFraction(t) * half, major = t % 1000 === 0;
    line(ctx, x + 6, y, x + (major ? 18 : 12), y, P.white, 2);
    if (major && t) text(ctx, String(t / 1000), x + 28, y, { size: 16, align: 'left' });
  }
  const y = mid - vsiFraction(vs) * half;
  line(ctx, x + VSI.w + 30, mid, x + 8, y, P.white, 3);
  if (Math.abs(vs) >= 400) text(ctx, String(Math.round(vs / 50) * 50), x + VSI.w / 2, vs > 0 ? VSI.top - 14 : VSI.bottom + 14, { size: 17, font: MONO });
}

// ----------------------------------------------------------------- HSI ----
function drawHsi(ctx, m) {
  const h = m.heading, hdg = h.magDeg, { cx, cy, r } = HSI;
  ctx.save();
  ctx.beginPath(); ctx.rect(0, 560, 800, 260); ctx.clip();
  ctx.fillStyle = '#0b0d10'; ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI); ctx.fill();
  ctx.translate(cx, cy);
  ctx.save();
  ctx.rotate(-hdg * D2R);
  for (let d = 0; d < 360; d += 5) {
    const ten = d % 10 === 0, len = ten ? 16 : 9;
    const c = Math.sin(d * D2R), s = -Math.cos(d * D2R);
    line(ctx, c * r, s * r, c * (r - len), s * (r - len), P.white, 2);
    if (d % 30 === 0) {
      ctx.save(); ctx.rotate(d * D2R);
      text(ctx, { 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[d] ?? String(d / 10), 0, -r + 32, { size: 20 });
      ctx.restore();
    }
  }
  // Heading bug.
  ctx.restore();
  // Course pointer and deviation bar.
  const n = m.nav, colour = n.source === 'GPS' ? P.magenta : P.green;
  if (n.valid || n.source !== 'GPS') {
    ctx.save();
    ctx.rotate((n.courseMagDeg - hdg) * D2R);
    line(ctx, 0, -r + 22, 0, -r * 0.42, colour, 4);
    line(ctx, 0, r * 0.42, 0, r - 22, colour, 4);
    ctx.fillStyle = colour; ctx.beginPath(); ctx.moveTo(0, -r + 18); ctx.lineTo(-10, -r + 38); ctx.lineTo(10, -r + 38); ctx.fill();
    for (const k of [-2, -1, 1, 2]) { ctx.strokeStyle = P.white; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(k * 30, 0, 4.5, 0, 2 * Math.PI); ctx.stroke(); }
    if (n.valid) line(ctx, clampDots(n.lateralDots) * 30, -r * 0.38, clampDots(n.lateralDots) * 30, r * 0.38, colour, 4);
    if (n.valid && n.toFrom) {
      const d = n.toFrom > 0 ? -1 : 1;
      ctx.fillStyle = colour; ctx.beginPath(); ctx.moveTo(0, d * 70); ctx.lineTo(-10, d * 52); ctx.lineTo(10, d * 52); ctx.fill();
    }
    ctx.restore();
  }
  // Track diamond.
  ctx.save(); ctx.rotate((h.trackMagDeg - hdg) * D2R);
  ctx.strokeStyle = P.magenta; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(0, -r + 2); ctx.lineTo(6, -r + 12); ctx.lineTo(0, -r + 22); ctx.lineTo(-6, -r + 12); ctx.closePath(); ctx.stroke();
  ctx.restore();
  // Aircraft symbol and lubber.
  ctx.strokeStyle = P.white; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(0, -22); ctx.lineTo(0, 26); ctx.moveTo(-22, 0); ctx.lineTo(22, 0); ctx.moveTo(-9, 20); ctx.lineTo(9, 20); ctx.stroke();
  ctx.restore();
  readoutBox(ctx, cx - 34, cy - r - 40, 68, 34, P.white);
  text(ctx, pad3(hdg), cx, cy - r - 22, { size: 26, font: MONO });
  ctx.fillStyle = P.white; ctx.beginPath(); ctx.moveTo(cx, cy - r + 2); ctx.lineTo(cx - 8, cy - r - 6); ctx.lineTo(cx + 8, cy - r - 6); ctx.fill();
  // Source and distance.
  const label = n.source === 'GPS' ? `GPS ${n.mode}` : n.source === 'LOC' ? `LOC1 ${fmt(n.frequency, 2)}` : `VOR1 ${fmt(n.frequency, 2)}`;
  text(ctx, label, 120, 600, { size: 20, colour, align: 'left' });
  text(ctx, `CRS ${pad3(n.courseMagDeg)}`, 120, 628, { size: 18, colour, align: 'left' });
  if (n.distanceNm !== null) text(ctx, `${fmt(n.distanceNm, 1)} NM`, 120, 656, { size: 18, colour: P.white, align: 'left' });
  text(ctx, `TRK ${pad3(h.trackMagDeg)}`, 680, 600, { size: 18, colour: P.magenta, align: 'right' });
  text(ctx, `GS ${fmt(h.groundSpeedKt)}`, 680, 628, { size: 18, align: 'right' });
  text(ctx, `TAS ${fmt(m.air.tasKt)}`, 680, 656, { size: 18, align: 'right' });
}

// ---------------------------------------------------------- mode strip ----
function drawModes(ctx, m) {
  box(ctx, 230, 12, 340, 48, { stroke: P.dim, width: 1.5 });
  const n = m.nav, colour = n.source === 'GPS' ? P.magenta : P.green;
  const lat = n.source === 'GPS' ? 'GPS' : n.source;
  const vert = n.verticalValid ? (n.source === 'GPS' ? 'VPATH' : 'GS') : '';
  text(ctx, n.valid ? lat : '', 300, 36, { size: 22, colour, weight: 700 });
  text(ctx, vert, 400, 36, { size: 22, colour, weight: 700 });
  text(ctx, m.onGround ? 'GND' : n.mode ?? '', 500, 36, { size: 20, colour: P.white });
  if (m.altitude.alert.deviation) text(ctx, 'ALT', 660, 36, { size: 22, colour: P.amber, weight: 700 });
}
