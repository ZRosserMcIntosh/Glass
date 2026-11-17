// Glass — the HSI's navigation source and the navigation display's geometry.
// SPDX-License-Identifier: MIT

import { wrap360, wrap180 } from './filters.js';

export const NM = 1852;               // m
const D2R = Math.PI / 180;

// Lateral and vertical deviation from the selected source, in dots (two dots
// full scale, the convention on both the PFD and the HSI). The core reports
// every needle as -1..1 of full scale, + needle right / up (fly toward it).
//   source 'LOC': NAV 1 localizer and glideslope (ILS tuned and received)
//   source 'GPS': the GPS leg and, in approach mode, its LPV-style path
//   source 'VOR': NAV 1 radial against the OBS
// 'AUTO' takes the localizer once it is alive, else the GPS while it has an
// active leg, else NAV 1.
export function selectSource(f, wanted = 'AUTO') {
  const loc = f.nav.localizer && f.nav.valid, gps = f.gps.valid && f.gps.active >= 0;
  if (wanted === 'AUTO') return loc ? 'LOC' : gps ? 'GPS' : 'VOR';
  return wanted;
}

export function navGuidance(f, source, magVarDeg) {
  const toMag = (trueDeg) => wrap360(trueDeg - magVarDeg);
  if (source === 'GPS') {
    const g = f.gps, ok = g.valid && g.active >= 0;
    return {
      source, valid: ok, courseMagDeg: toMag(g.desiredTrackTrueDeg), lateralDots: ok ? 2 * g.cdi : 0,
      verticalValid: ok && g.verticalValid, verticalDots: ok && g.verticalValid ? 2 * g.vdi : 0,
      toFrom: 1, distanceNm: ok ? g.distanceToWaypointM / NM : null, bearingMagDeg: ok ? toMag(g.bearingTrueDeg) : null,
      mode: g.mode, fullScaleNm: g.fullScaleM / NM,
    };
  }
  const n = f.nav;
  return {
    source, valid: n.valid, courseMagDeg: n.obsDeg, lateralDots: n.valid ? 2 * n.cdi : 0,
    verticalValid: source === 'LOC' && n.gsValid, verticalDots: source === 'LOC' && n.gsValid ? 2 * n.gs : 0,
    toFrom: n.toFrom, distanceNm: null, bearingMagDeg: null, mode: source === 'LOC' ? 'ILS' : 'VOR', frequency: n.activeMHz,
  };
}

// Heading-up map projection about the aircraft: a world point (ENU m) to
// screen offsets in NM (+x right, +y up the display), for a map oriented to
// `upTrueDeg` (heading-up: the true heading; track-up: the track).
export function projector(position, upTrueDeg) {
  const c = Math.cos(upTrueDeg * D2R), s = Math.sin(upTrueDeg * D2R);
  return (east, north) => {
    const de = (east - position.east) / NM, dn = (north - position.north) / NM;
    return { x: de * c - dn * s, y: de * s + dn * c };
  };
}

// The route as the ND draws it: projected waypoints, the active one flagged,
// and the active leg (from the previous waypoint, or the aircraft for a
// direct-to) for the magenta line.
export function routeForDisplay(route, active, project) {
  if (!route || !route.wp.length) return { points: [], activeLeg: null };
  const points = route.wp.map((w, i) => ({ ident: w.ident, ...project(w.east, w.north), active: i === active, faf: i === route.faf, altM: w.alt }));
  const activeLeg = active >= 0 && active < points.length ? { from: active > 0 ? points[active - 1] : { x: 0, y: 0 }, to: points[active] } : null;
  return { points, activeLeg };
}

// Runways (A.region(): each end's paved-end ENU position, true heading,
// length and width) as centreline segments in NM screen offsets, with the
// outline's four corners for scales where the width shows.
export function runwaysForDisplay(region, project, rangeNm) {
  const out = [];
  for (const ap of region) {
    for (let i = 0; i < ap.ends.length; i += 2) {
      const end = ap.ends[i];
      const h = end.heading * D2R, ux = Math.sin(h), uy = Math.cos(h), w = end.width / 2;
      const a = { e: end.east, n: end.north }, b = { e: end.east + ux * end.length, n: end.north + uy * end.length };
      const corners = [
        project(a.e - uy * w, a.n + ux * w), project(b.e - uy * w, b.n + ux * w),
        project(b.e + uy * w, b.n - ux * w), project(a.e + uy * w, a.n - ux * w),
      ];
      const pa = project(a.e, a.n), pb = project(b.e, b.n);
      if (Math.min(Math.hypot(pa.x, pa.y), Math.hypot(pb.x, pb.y)) < rangeNm * 1.5) out.push({ a: pa, b: pb, corners });
    }
  }
  return out;
}

// Time and fuel to the waypoint at the present ground speed and fuel flow.
export function estimates(distanceNm, groundSpeedKt, fuelFlowKgH) {
  if (!(distanceNm >= 0) || !(groundSpeedKt > 30)) return { minutes: null, fuelKg: null };
  const hours = distanceNm / groundSpeedKt;
  return { minutes: hours * 60, fuelKg: hours * fuelFlowKgH };
}

export const relativeBearing = (bearingDeg, headingDeg) => wrap180(bearingDeg - headingDeg);
