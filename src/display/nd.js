// Glass — navigation display: heading-up arc map with the route, the
// region's runways, the track line, wind and the active waypoint.
// SPDX-License-Identifier: MIT

import { P, MONO, D2R, prepare, text, line, fmt, pad3 } from './draw.js';
import { projector, routeForDisplay, runwaysForDisplay, estimates } from '../avionics/navigation.js';

export const ND_SIZE = { w: 800, h: 820 };
const AX = 400, AY = 700, R = 600;   // aircraft symbol and the arc's radius (= the selected range)
export const ND_RANGES = [2.5, 5, 10, 20, 40, 80, 160];

// map: { route: A.route(), region: A.region() } (refreshed by the host when the destination changes).
export function drawNd(canvas, m, map, rangeNm = 20) {
  const ctx = prepare(canvas, ND_SIZE.w, ND_SIZE.h);
  if (!m || !m.powered) return;
  const h = m.heading, ppn = R / rangeNm;
  const project = projector(m.position, h.trueDeg);
  const toScreen = (p) => ({ x: AX + p.x * ppn, y: AY - p.y * ppn });

  ctx.save();
  ctx.beginPath(); ctx.arc(AX, AY, R, 0, 2 * Math.PI); ctx.rect(0, AY, 800, 120); ctx.clip();
  // Half-range ring.
  ctx.setLineDash([6, 10]); ctx.strokeStyle = P.grey; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(AX, AY, R / 2, Math.PI, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]);
  text(ctx, String(rangeNm / 2), AX - R / 2 - 4, AY - 14, { size: 16, colour: P.grey, align: 'right' });
  // Runways.
  // Runways: drawn at least 6 px wide (an outline with a dark core), so a
  // 45 m runway stays visible at any range.
  for (const r of runwaysForDisplay(map?.region ?? [], project, rangeNm)) {
    const a = toScreen(r.a), b = toScreen(r.b), wpx = Math.max(6, Math.hypot(r.corners[0].x - r.corners[3].x, r.corners[0].y - r.corners[3].y) * ppn);
    line(ctx, a.x, a.y, b.x, b.y, P.white, wpx);
    line(ctx, a.x, a.y, b.x, b.y, '#000', wpx - 3);
  }
  // Route: legs white, the active leg magenta, waypoints as four-point stars.
  const route = routeForDisplay(map?.route, map?.route?.active ?? -1, project);
  const pts = route.points.map((p) => ({ ...p, ...toScreen(p) }));
  for (let i = 1; i < pts.length; ++i) line(ctx, pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y, P.white, 2.5);
  if (route.activeLeg) {
    const a = toScreen(route.activeLeg.from), b = toScreen(route.activeLeg.to);
    line(ctx, a.x, a.y, b.x, b.y, P.magenta, 3.5);
  }
  for (const p of pts) {
    const c = p.active ? P.magenta : P.white;
    star(ctx, p.x, p.y, c);
    text(ctx, p.ident, p.x + 14, p.y - 12, { size: 17, colour: c, align: 'left' });
    if (p.altM > 0) text(ctx, String(Math.round(p.altM / 0.3048 / 10) * 10), p.x + 14, p.y + 8, { size: 14, colour: P.grey, align: 'left' });
  }
  // Track line.
  const trk = (h.trackTrueDeg - h.trueDeg) * D2R;
  ctx.setLineDash([12, 8]);
  line(ctx, AX, AY, AX + Math.sin(trk) * R, AY - Math.cos(trk) * R, P.white, 2);
  ctx.setLineDash([]);
  ctx.restore();

  // Compass arc (heading up: 0 deg relative is the true heading; labels magnetic).
  ctx.strokeStyle = P.white; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(AX, AY, R, (-90 - 42) * D2R, (-90 + 42) * D2R); ctx.stroke();
  for (let d = Math.ceil((h.magDeg - 45) / 5) * 5; d <= h.magDeg + 45; d += 5) {
    const rel = (d - h.magDeg) * D2R, ten = d % 10 === 0;
    const sx = Math.sin(rel), sy = -Math.cos(rel);
    if (Math.abs(rel) > 42 * D2R) continue;
    line(ctx, AX + sx * R, AY + sy * R, AX + sx * (R - (ten ? 18 : 10)), AY + sy * (R - (ten ? 18 : 10)));
    if (d % 30 === 0) text(ctx, String((((d % 360) + 360) % 360) / 10 || 36), AX + sx * (R - 36), AY + sy * (R - 36), { size: 20 });
  }
  // Heading pointer and box.
  ctx.fillStyle = P.white; ctx.beginPath(); ctx.moveTo(AX, AY - R + 2); ctx.lineTo(AX - 10, AY - R - 14); ctx.lineTo(AX + 10, AY - R - 14); ctx.fill();
  text(ctx, `HDG ${pad3(h.magDeg)} M`, AX, 52, { size: 24, font: MONO });
  // Aircraft symbol.
  ctx.strokeStyle = P.white; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(AX, AY - 26); ctx.lineTo(AX, AY + 22); ctx.moveTo(AX - 22, AY - 4); ctx.lineTo(AX + 22, AY - 4); ctx.moveTo(AX - 9, AY + 18); ctx.lineTo(AX + 9, AY + 18); ctx.stroke();

  // Corners: speeds and wind; the active waypoint; range.
  text(ctx, `GS ${fmt(h.groundSpeedKt)}  TAS ${fmt(m.air.tasKt)}`, 20, 30, { size: 20, align: 'left', font: MONO });
  if (m.wind) {
    const w = m.wind;
    text(ctx, `${pad3(w.fromDeg - h.magVarDeg)}°/${fmt(w.speedKt)}`, 20, 60, { size: 20, align: 'left', font: MONO });
    ctx.save(); ctx.translate(40, 110); ctx.rotate((w.relativeDeg + 180) * D2R);
    line(ctx, 0, -20, 0, 20, P.white, 3);
    ctx.fillStyle = P.white; ctx.beginPath(); ctx.moveTo(0, -26); ctx.lineTo(-8, -12); ctx.lineTo(8, -12); ctx.fill();
    ctx.restore();
  }
  const n = m.nav;
  const wp = map?.route?.wp?.[map?.route?.active ?? -1];
  if (n.source === 'GPS' && n.valid && wp) {
    const est = estimates(n.distanceNm, h.groundSpeedKt, m.eicas.fuelFlowKgH);
    text(ctx, wp.ident, 780, 30, { size: 22, colour: P.magenta, align: 'right' });
    text(ctx, `${fmt(n.distanceNm, 1)} NM`, 780, 58, { size: 20, align: 'right', font: MONO });
    if (est.minutes !== null) text(ctx, `${fmt(est.minutes, 0)} MIN`, 780, 84, { size: 18, align: 'right', colour: P.green, font: MONO });
  } else if (n.source !== 'GPS') {
    text(ctx, n.source === 'LOC' ? 'ILS' : 'VOR', 780, 30, { size: 22, colour: P.green, align: 'right' });
    text(ctx, fmt(n.frequency, 2), 780, 58, { size: 20, colour: P.green, align: 'right', font: MONO });
  }
  text(ctx, `${rangeNm} NM`, 20, 800, { size: 18, align: 'left', colour: P.cyan });
  if (m.eicas.enduranceH !== null) {
    const min = Math.round(m.eicas.enduranceH * 60);
    text(ctx, `ENDUR ${Math.floor(min / 60)}:${String(min % 60).padStart(2, '0')}`, 780, 800, { size: 18, align: 'right', font: MONO });
  }
}

function star(ctx, x, y, colour) {
  ctx.strokeStyle = colour; ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(x, y - 12); ctx.lineTo(x + 3, y - 3); ctx.lineTo(x + 12, y); ctx.lineTo(x + 3, y + 3);
  ctx.lineTo(x, y + 12); ctx.lineTo(x - 3, y + 3); ctx.lineTo(x - 12, y); ctx.lineTo(x - 3, y - 3); ctx.closePath();
  ctx.stroke();
}
