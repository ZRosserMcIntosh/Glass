// Glass — altitude alerting, minimums and the radio-altitude callouts.
// SPDX-License-Identifier: MIT

// Altitude alerting against the selected altitude (the usual transport
// logic): approaching within 900 ft the selected-altitude box is highlighted;
// once inside 200 ft the altitude is captured, and leaving 200 ft again after
// capture is a deviation (amber) until the crew selects a new altitude or
// returns inside it.
export const ALERT = { approachFt: 900, captureFt: 200 };

export class AltitudeAlert {
  constructor(selectedFt = 10000) { this.select(selectedFt); }
  select(ft) { this.selectedFt = Math.round(ft / 100) * 100; this.captured = false; }
  update(altitudeFt) {
    const dev = altitudeFt - this.selectedFt, a = Math.abs(dev);
    if (a <= ALERT.captureFt) this.captured = true;
    const deviation = this.captured && a > ALERT.captureFt;
    const approaching = !this.captured && a <= ALERT.approachFt;
    return { selectedFt: this.selectedFt, deviationFt: dev, approaching, captured: this.captured && !deviation, deviation };
  }
}

// Minimums: a decision height on the radio altimeter (RA) or a decision /
// minimum descent altitude on the barometric altimeter (BARO). "Approaching
// minimums" 100 ft above, "MINIMUMS" when reached on the way down; reset
// climbing 75 ft back above them (a go-around).
export class Minimums {
  constructor(kind = 'BARO', valueFt = 3000) { this.set(kind, valueFt); }
  set(kind, valueFt) { this.kind = kind; this.valueFt = valueFt; this.reached = false; }
  update(altitudeFt, radioAltFt, onGround) {
    const h = this.kind === 'RA' ? radioAltFt : altitudeFt;
    const above = h - this.valueFt;
    if (above <= 0 && !onGround) this.reached = true;
    if (above > 75) this.reached = false;
    return { kind: this.kind, valueFt: this.valueFt, aboveFt: above, approaching: !this.reached && above > 0 && above <= 100, reached: this.reached };
  }
}

// Radio-altitude callouts, descending only, each once per approach. The ones
// in the list are the common business-jet set; a callout re-arms after
// climbing 100 ft above it.
export const CALLOUTS = [2500, 1000, 500, 400, 300, 200, 100, 50, 40, 30, 20, 10];
export class Callouts {
  constructor(list = CALLOUTS) { this.list = list; this.armed = new Set(list); this.last = null; }
  update(radioAltFt, verticalSpeedFpm, onGround) {
    const out = [];
    if (this.last !== null && !onGround && verticalSpeedFpm < 0) {
      for (const h of this.list) if (this.armed.has(h) && this.last > h && radioAltFt <= h) { out.push(h === 2500 ? 'TWENTY FIVE HUNDRED' : String(h)); this.armed.delete(h); }
    }
    for (const h of this.list) if (radioAltFt > h + 100) this.armed.add(h);
    this.last = radioAltFt;
    return out;
  }
}
