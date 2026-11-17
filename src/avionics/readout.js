// Glass — named views of the flight core's readouts.
// SPDX-License-Identifier: MIT
//
// The core exports flat Float64Arrays (core/web/README.md: avd_state,
// avd_cockpit, avd_jet, avd_gps). Every index the glass cockpit reads is named
// here and nowhere else, so an ABI change is one edit. The avionics read the
// sensed channels only; the truth fields are kept for the debug overlay and
// the tests, never for a display.

export const EXPECTED_CORE_ABI = 1;

const STATE = { east: 0, north: 1, up: 2, groundSpeed: 12, heightAboveField: 14, onGround: 28, parkingBrake: 49, time: 42, driftDeg: 62 };
const COCKPIT = {
  ias: 0, altitude: 1, kollsman: 2, vsi: 3, pitch: 4, roll: 5, tumbled: 6, heading: 7, headingBug: 8, ball: 12,
  oat: 26, navActive: 32, obs: 36, navValid: 37, cdi: 38, toFrom: 39, gsValid: 40, gs: 41, localizer: 42, avionics: 43,
  // truth, debug only
  tTas: 48, tCas: 49, tPressureAlt: 52, tPitch: 54, tRoll: 55, tTrueHeading: 56, tMagHeading: 57,
};
const JET = {
  engines: 0, gear: 1, gearLever: 2, speedbrake: 3, speedbrakeDetent: 4, mach: 5, fuelKg: 6, fuelFlow: 7,
  mmo: 24, vmo: 25, tas: 26, flapDeg: 27, trim: 28, weightOnWheels: 29, oilPsi: 30,
  vs1g: 32, vsTakeoff: 33, vsLanding: 34, grossKg: 35, fpaDeg: 36, trackTrue: 37, windFrom: 38, windKt: 39,
  radioAltFt: 40, satC: 41, loadFactor: 42, alphaDeg: 43, anyGenerator: 44, busVolts: 45, stallWarning: 46,
  vfe: 47, vle: 48, takeoffFlap: 49, landingFlap: 50, pusher: 51, reverserPos: 52, reverserPhase: 53,
  groundSpoilers: 56, reversersFitted: 57, lever: 58, generator: 60, hydPsi: 62, brakePower: 68, accumulator: 69,
  gearEmergency: 70, hydraulicsFitted: 71, batteryCharge: 72, batteryAmps: 73,
};
const ENGINE = { base: 8, stride: 8, n1: 0, n2: 1, itt: 2, fuelFlow: 3, thrustKn: 4, running: 5, starting: 6, fuelSwitch: 7 };
const GPS = {
  valid: 0, active: 1, mode: 2, desiredTrackTrue: 3, crossTrackM: 4, fullScaleM: 5, cdi: 6, distanceToWaypointM: 7,
  bearingTrue: 8, distanceToDestinationM: 9, verticalValid: 10, verticalDeviationM: 11, vdi: 12, magneticVariation: 13,
};
export const INDEX = { STATE, COCKPIT, JET, ENGINE, GPS };

const GPS_MODES = ['ENR', 'TERM', 'APR'];

// Decode one frame of the core's readouts into the plain object the avionics
// take. `A` is the loader's module (core/web/avd_loader.mjs). Copies every
// value, so the result outlives the next avd_advance.
export function readFrame(A) {
  const s = A.state(), c = A.cockpitState(), j = A.jet(), g = A.gps();
  return decode(s, c, j, g);
}

export function decode(s, c, j, g) {
  const nEng = Math.max(0, Math.min(2, j[JET.engines] | 0));
  const engines = [];
  for (let i = 0; i < nEng; ++i) {
    const b = ENGINE.base + ENGINE.stride * i;
    engines.push({
      n1: j[b + ENGINE.n1], n2: j[b + ENGINE.n2], ittC: j[b + ENGINE.itt], fuelFlowKgH: j[b + ENGINE.fuelFlow],
      thrustKn: j[b + ENGINE.thrustKn], running: j[b + ENGINE.running] > 0.5, starting: j[b + ENGINE.starting] > 0.5,
      fuelSwitch: j[b + ENGINE.fuelSwitch] > 0.5, oilPsi: j[JET.oilPsi + i], lever: j[JET.lever + i],
      generator: j[JET.generator + i] > 0.5, reverser: j[JET.reverserPos + 2 * i], reverserPhase: j[JET.reverserPhase + 2 * i] | 0,
    });
  }
  return {
    time: s[STATE.time],
    position: { east: s[STATE.east], north: s[STATE.north], up: s[STATE.up] },
    groundSpeedKt: s[STATE.groundSpeed],
    onGround: s[STATE.onGround] > 0.5 || j[JET.weightOnWheels] > 0.5,
    parkingBrake: s[STATE.parkingBrake] > 0.5,
    driftDeg: s[STATE.driftDeg],
    air: {
      iasKt: c[COCKPIT.ias], altitudeFt: c[COCKPIT.altitude], kollsmanInHg: c[COCKPIT.kollsman], vsiFpm: c[COCKPIT.vsi],
      satC: j[JET.satC],
    },
    attitude: {
      pitchDeg: c[COCKPIT.pitch], rollDeg: c[COCKPIT.roll], failed: c[COCKPIT.tumbled] > 0.5,
      headingMagDeg: c[COCKPIT.heading], ball: c[COCKPIT.ball], loadFactor: j[JET.loadFactor], alphaDeg: j[JET.alphaDeg],
    },
    inertial: { fpaDeg: j[JET.fpaDeg], trackTrueDeg: j[JET.trackTrue] },
    radioAltFt: j[JET.radioAltFt],
    limits: {
      vmoKt: j[JET.vmo], mmo: j[JET.mmo], vfeKt: j[JET.vfe], vleKt: j[JET.vle],
      vs1gKt: j[JET.vs1g], vsTakeoffKt: j[JET.vsTakeoff], vsLandingKt: j[JET.vsLanding],
      takeoffFlapDeg: j[JET.takeoffFlap], landingFlapDeg: j[JET.landingFlap],
    },
    config: {
      gear: j[JET.gear], gearLeverDown: j[JET.gearLever] > 0.5, gearEmergency: j[JET.gearEmergency] > 0.5,
      flapDeg: j[JET.flapDeg], speedbrake: j[JET.speedbrake], speedbrakeDetent: j[JET.speedbrakeDetent],
      groundSpoilers: j[JET.groundSpoilers] > 0.5, reversersFitted: j[JET.reversersFitted] > 0.5, trim: j[JET.trim],
    },
    protection: { stallWarning: j[JET.stallWarning] > 0.5, pusher: j[JET.pusher] > 0.5 },
    engines,
    fuel: { kg: j[JET.fuelKg], flowKgH: j[JET.fuelFlow] },
    grossKg: j[JET.grossKg],
    systems: {
      anyGenerator: j[JET.anyGenerator] > 0.5, busVolts: j[JET.busVolts], batteryCharge: j[JET.batteryCharge],
      batteryAmps: j[JET.batteryAmps], hydraulicsFitted: j[JET.hydraulicsFitted] > 0.5,
      hydPsi: [j[JET.hydPsi], j[JET.hydPsi + 1]], brakePower: j[JET.brakePower], accumulator: j[JET.accumulator],
      avionics: c[COCKPIT.avionics] > 0.5,
    },
    nav: {
      activeMHz: c[COCKPIT.navActive], obsDeg: c[COCKPIT.obs], valid: c[COCKPIT.navValid] > 0.5, cdi: c[COCKPIT.cdi],
      toFrom: c[COCKPIT.toFrom], localizer: c[COCKPIT.localizer] > 0.5, gsValid: c[COCKPIT.gsValid] > 0.5, gs: c[COCKPIT.gs],
    },
    gps: {
      valid: g[GPS.valid] > 0.5, active: g[GPS.active] | 0, mode: GPS_MODES[g[GPS.mode] | 0] ?? 'ENR',
      desiredTrackTrueDeg: g[GPS.desiredTrackTrue], crossTrackM: g[GPS.crossTrackM], fullScaleM: g[GPS.fullScaleM],
      cdi: g[GPS.cdi], distanceToWaypointM: g[GPS.distanceToWaypointM], bearingTrueDeg: g[GPS.bearingTrue],
      distanceToDestinationM: g[GPS.distanceToDestinationM], verticalValid: g[GPS.verticalValid] > 0.5,
      verticalDeviationM: g[GPS.verticalDeviationM], vdi: g[GPS.vdi], magneticVariationDeg: g[GPS.magneticVariation],
    },
    truth: {
      casKt: c[COCKPIT.tCas], tasKt: c[COCKPIT.tTas], pressureAltFt: c[COCKPIT.tPressureAlt], pitchDeg: c[COCKPIT.tPitch],
      rollDeg: c[COCKPIT.tRoll], trueHeadingDeg: c[COCKPIT.tTrueHeading], magHeadingDeg: c[COCKPIT.tMagHeading],
      mach: j[JET.mach], tasJetKt: j[JET.tas], windFromDeg: j[JET.windFrom], windKt: j[JET.windKt],
    },
  };
}
