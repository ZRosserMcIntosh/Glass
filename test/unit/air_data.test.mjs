// SPDX-License-Identifier: MIT
import { test } from 'node:test';
import assert from 'node:assert/strict';
import * as ad from '../../src/avionics/air_data.js';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg ?? ''} ${a} vs ${b} (tol ${tol})`);

test('altimeter law: ISA sea level, tropopause, and its inverse', () => {
  near(ad.altitudeFromPressure(ad.P0), 0, 1e-6);
  near(ad.altitudeFromPressure(ad.P11) / ad.FT, 36089, 3, 'tropopause');
  for (const ft of [-1000, 0, 5000, 18000, 36089, 41000, 51000]) {
    for (const inHg of [28.5, 29.92, 30.6]) {
      const p = ad.pressureFromAltitude(ft * ad.FT, inHg * ad.INHG);
      near(ad.altitudeFromPressure(p, inHg * ad.INHG) / ad.FT, ft, 0.01, `round trip ${ft} ft at ${inHg}`);
    }
  }
});

test('pressure altitude: STD reads through, a high setting reads lower', () => {
  near(ad.pressureAltitudeFt(12345, 29.92), 12345, 15);
  const pa = ad.pressureAltitudeFt(3000, 30.92);
  // 30.92 against 29.92 is 1.0334 in pressure: about 900 ft lower near sea level.
  assert.ok(pa > 2050 && pa < 2150, `1 inHg high reads about 900 ft lower: ${pa}`);
});

test('CAS and Mach: sea level, FL350 and the crossover', () => {
  // At sea level CAS = TAS = M a0.
  near(ad.casFromMach(0.5, ad.P0), 0.5 * 661.4788, 0.05);
  // M 0.82 at FL350 is about 280 KCAS (standard tables).
  const p350 = ad.pressureFromAltitude(35000 * ad.FT);
  near(ad.casFromMach(0.82, p350), 279.6, 1.5, 'M0.82 FL350');
  // 300 KCAS / M 0.78 cross over near FL300.
  const crossover = [...Array(600).keys()].map((k) => k * 100).find((ft) => ad.casFromMach(0.78, ad.pressureFromAltitude(ft * ad.FT)) < 300);
  assert.ok(crossover > 29000 && crossover < 31500, `crossover ${crossover} ft`);
  // Mach back from the pressures.
  const qc = ad.impactPressureFromCas(279.6);
  near(ad.machFromPressures(qc, p350), 0.82, 0.003);
});

test('air data solution: TAS and TAT', () => {
  const s = ad.solveAirData({ iasKt: 279.6, indicatedFt: 35000, settingInHg: 29.92, satC: -54.3 });
  near(s.mach, 0.82, 0.003);
  near(s.tasKt, 0.82 * 38.967854 * Math.sqrt(218.85), 2, 'TAS');
  near(s.tatC, 218.85 * (1 + 0.2 * 0.82 * 0.82) - 273.15, 0.5, 'TAT');
});
