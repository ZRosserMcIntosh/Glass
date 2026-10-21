// Glass — air data computer: pressure altitude, Mach, TAS and TAT from the
// sensed pitot-static readings, and the CAS of a Mach limit.
// SPDX-License-Identifier: MIT
//
// The altimeter law is the one the flight core's altimeter is calibrated to
// (InstrumentPanel::altitudeFromPressure in core/src/core/src/instruments.cpp):
// the ISA troposphere law relative to the Kollsman setting up to the 11 km
// tropopause (22,632.06 Pa absolute), then the isothermal stratosphere. The
// computer inverts it to recover the static pressure, so its pressure altitude
// and Mach agree with the core's instruments rather than with a textbook
// approximation. Compressible-flow relations are the subsonic St Venant forms
// (gamma = 1.4), valid below Mach 1, which every glass profile's MMO is.

export const FT = 0.3048;                 // m per ft
export const KT = 1852 / 3600;            // m/s per kt
export const P0 = 101325;                 // Pa, ISA sea level
export const T0 = 288.15;                 // K, ISA sea level
export const A0 = 340.294;                // m/s, ISA sea-level speed of sound (the core's CAS calibration)
export const INHG = 3386.389;             // Pa per inHg
export const P11 = 22632.06;              // Pa at the 11 km tropopause
const TROP_K = 44330.77;                  // m: T0 / L
const TROP_N = 0.190263;                  // R L / g
const STRAT_H = 6341.62;                  // m per e-fold of pressure at 216.65 K

// Altitude (m) the core's altimeter reads at static pressure p (Pa) with the
// Kollsman window at setting (Pa).
export function altitudeFromPressure(p, setting = P0) {
  const ps = Math.max(setting, 1);
  const trop = (pp) => TROP_K * (1 - Math.pow(Math.max(pp, 1) / ps, TROP_N));
  if (p >= P11) return trop(p);
  return trop(P11) + STRAT_H * Math.log(P11 / Math.max(p, 1));
}

// The inverse: static pressure (Pa) behind an indicated altitude (m) at a setting (Pa).
export function pressureFromAltitude(h, setting = P0) {
  const ps = Math.max(setting, 1);
  const hTrop = TROP_K * (1 - Math.pow(P11 / ps, TROP_N));
  if (h <= hTrop) return ps * Math.pow(Math.max(1 - h / TROP_K, 1e-9), 1 / TROP_N);
  return P11 * Math.exp(-(h - hTrop) / STRAT_H);
}

// Pressure altitude (ft) from the altimeter (ft) and its setting (inHg).
export function pressureAltitudeFt(indicatedFt, settingInHg) {
  const p = pressureFromAltitude(indicatedFt * FT, settingInHg * INHG);
  return altitudeFromPressure(p, P0) / FT;
}

// Impact pressure (Pa) of a calibrated airspeed (kt), and back.
export function impactPressureFromCas(casKt) {
  const r = (casKt * KT) / A0;
  return P0 * (Math.pow(1 + 0.2 * r * r, 3.5) - 1);
}
export function casFromImpactPressure(qc) {
  return (A0 * Math.sqrt(Math.max(0, 5 * (Math.pow(Math.max(qc, 0) / P0 + 1, 2 / 7) - 1)))) / KT;
}

// Mach from impact pressure and static pressure (Pa), and back.
export function machFromPressures(qc, p) {
  return Math.sqrt(Math.max(0, 5 * (Math.pow(Math.max(qc, 0) / Math.max(p, 1) + 1, 2 / 7) - 1)));
}
export function impactPressureFromMach(mach, p) {
  return p * (Math.pow(1 + 0.2 * mach * mach, 3.5) - 1);
}

// The CAS (kt) at which a Mach number is reached at static pressure p (Pa):
// where an MMO crosses below VMO on the speed tape.
export function casFromMach(mach, p) {
  return casFromImpactPressure(impactPressureFromMach(mach, p));
}

// Speed of sound (kt) at a static air temperature (deg C).
export function speedOfSoundKt(satC) {
  return (A0 * Math.sqrt(Math.max(satC + 273.15, 1) / T0)) / KT;
}

// One air-data solution from the sensed readings. IAS stands in for CAS: the
// core's ASI is calibrated to CAS and the glass profiles publish no position
// error table, so the two are the same reading here.
//   iasKt, indicatedFt, settingInHg, satC  ->  { pressureAltFt, staticPa, qc, mach, tasKt, tatC }
export function solveAirData({ iasKt, indicatedFt, settingInHg, satC, recovery = 1.0 }) {
  const p = pressureFromAltitude(indicatedFt * FT, settingInHg * INHG);
  const qc = impactPressureFromCas(Math.max(iasKt, 0));
  const mach = machFromPressures(qc, p);
  const tasKt = mach * speedOfSoundKt(satC);
  const tatC = (satC + 273.15) * (1 + 0.2 * recovery * mach * mach) - 273.15;
  return { pressureAltFt: altitudeFromPressure(p, P0) / FT, staticPa: p, qc, mach, tasKt, tatC };
}
