// Glass — the avionics computer: one frame of sensed readouts in, one display
// model out. The renderers (src/display) draw this model and nothing else.
// SPDX-License-Identifier: MIT

import { solveAirData } from './air_data.js';
import { SpeedComputer } from './speeds.js';
import { FlightPathComputer, windComponents } from './flight_path.js';
import { selectSource, navGuidance } from './navigation.js';
import { Cas, engineLimits, gaugeColour } from './eicas.js';
import { AltitudeAlert, Minimums, Callouts } from './alerting.js';
import { wrap360 } from './filters.js';

// Fuel capacity (kg) from a load sheet (A.loadsheet()): every tank's capacity at the fuel density.
export function fuelCapacityKg(loadsheet) {
  if (!loadsheet || !loadsheet.tanks) return 0;
  return loadsheet.tanks.reduce((s, t) => s + (t.capacity - (t.unusable || 0)), 0) * (loadsheet.density || 0.8);
}

export class GlassAvionics {
  // aircraft: the parsed definition JSON; markings: A.markings(); loadsheet:
  // A.loadsheet(false); controls: A.cockpit() (the switch registry: per-engine
  // generator switches mean per-engine generators).
  constructor({ aircraft, markings, loadsheet, controls = [] }) {
    const turbofan = Array.isArray(aircraft?.turbofans) ? aircraft.turbofans[0] : null;
    this.name = aircraft?.display_name ?? 'Aircraft';
    this.profile = aircraft?.panel?.profile ?? '';
    this.limits = engineLimits(markings, turbofan);
    this.speed = new SpeedComputer();
    this.path = new FlightPathComputer();
    const perEngineGenerators = controls.some((c) => /^electrical\.generator\d+$/.test(c.id));
    this.cas = new Cas({ fuelCapacityKg: fuelCapacityKg(loadsheet), engineLimits: this.limits, perEngineGenerators });
    this.altAlert = new AltitudeAlert(10000);
    this.minimums = new Minimums('BARO', 3000);
    this.callouts = new Callouts();
    this.navSource = 'AUTO';
    this.runwayHeadingTrueDeg = null;   // for the approach speed's wind additive
  }

  reset() {
    this.speed = new SpeedComputer();
    this.path = new FlightPathComputer();
    this.cas.reset();
    this.callouts = new Callouts();
    this.minimums.reached = false;
  }

  update(f, dt) {
    const powered = f.systems.avionics && f.systems.busVolts > 18;
    const magVar = f.gps.magneticVariationDeg;
    const headingMag = f.attitude.headingMagDeg;
    const headingTrue = wrap360(headingMag + magVar);

    const air = solveAirData({ iasKt: f.air.iasKt, indicatedFt: f.air.altitudeFt, settingInHg: f.air.kollsmanInHg, satC: f.air.satC });
    const flight = this.path.update(f, air, headingTrue, dt);
    const rwyWind = flight.wind && this.runwayHeadingTrueDeg !== null
      ? windComponents(flight.wind.fromDeg, flight.wind.speedKt, this.runwayHeadingTrueDeg) : null;
    const speed = this.speed.update(f, air, dt, rwyWind ? rwyWind.headwindKt : 0);

    const source = selectSource(f, this.navSource);
    const nav = navGuidance(f, source, magVar);

    const altitude = {
      ft: f.air.altitudeFt, trendFt: flight.altitudeTrendFt, vsFpm: f.air.vsiFpm, settingInHg: f.air.kollsmanInHg,
      std: Math.abs(f.air.kollsmanInHg - 29.92) < 0.005, pressureAltFt: air.pressureAltFt,
      alert: this.altAlert.update(f.air.altitudeFt),
      minimums: this.minimums.update(f.air.altitudeFt, f.radioAltFt, f.onGround),
      radioAltFt: f.radioAltFt, radioAltShown: f.radioAltFt < 2500,
    };
    const callouts = this.callouts.update(f.radioAltFt, f.air.vsiFpm, f.onGround);
    if (altitude.minimums.reached && !this.minimumsCalled) callouts.push('MINIMUMS');
    this.minimumsCalled = altitude.minimums.reached;

    const engines = f.engines.map((e) => ({
      ...e,
      n1Colour: gaugeColour('n1', e.n1, this.limits, e.running),
      ittColour: gaugeColour('itt', e.ittC, this.limits, e.running),
      oilColour: gaugeColour('oil', e.oilPsi, this.limits, e.running),
      reverserState: e.reverser > 0.99 ? 'REV' : e.reverser > 0.01 ? 'TRANSIT' : null,
    }));
    const cas = this.cas.update(f, dt, { overspeed: speed.overspeed, lowSpeed: speed.lowSpeed });
    const enduranceH = f.fuel.flowKgH > 1 ? f.fuel.kg / f.fuel.flowKgH : null;

    return {
      powered, name: this.name, onGround: f.onGround, time: f.time,
      air: { ...air, satC: f.air.satC },
      speed,
      attitude: { pitchDeg: f.attitude.pitchDeg, rollDeg: f.attitude.rollDeg, failed: f.attitude.failed, ball: f.attitude.ball, loadFactor: f.attitude.loadFactor },
      flightPath: flight.fpv,
      altitude,
      heading: {
        magDeg: headingMag, trueDeg: headingTrue, trackMagDeg: wrap360(f.inertial.trackTrueDeg - magVar),
        trackTrueDeg: f.inertial.trackTrueDeg, magVarDeg: magVar, groundSpeedKt: f.groundSpeedKt,
      },
      wind: flight.wind ? { ...flight.wind, relativeDeg: wrap360(flight.wind.fromDeg - headingTrue), runway: rwyWind } : null,
      nav,
      position: f.position,
      eicas: {
        limits: this.limits, engines, cas,
        fuelKg: f.fuel.kg, fuelFlowKgH: f.fuel.flowKgH, enduranceH, grossKg: f.grossKg,
        flapDeg: f.config.flapDeg, landingFlapDeg: f.limits.landingFlapDeg, takeoffFlapDeg: f.limits.takeoffFlapDeg,
        gear: f.config.gear, gearLeverDown: f.config.gearLeverDown, speedbrake: f.config.speedbrake, trim: f.config.trim,
        groundSpoilers: f.config.groundSpoilers, systems: f.systems, satC: f.air.satC, tatC: air.tatC,
      },
      callouts,
    };
  }

  acknowledge() { this.cas.acknowledge(); }
}
