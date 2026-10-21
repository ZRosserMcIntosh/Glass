// Glass — the browser host: runs the flight core, feeds the avionics
// computer, draws the PFD, ND and EICAS, and maps the keyboard to the
// controls. Entry: boot({ createAvdModule, wasmBytes, aircraft }) with
// aircraft { id: definition JSON text }.
// SPDX-License-Identifier: MIT

import { readFrame, EXPECTED_CORE_ABI } from './avionics/readout.js';
import { GlassAvionics } from './avionics/glass.js';
import { drawPfd } from './display/pfd.js';
import { drawNd, ND_RANGES } from './display/nd.js';
import { drawEicas } from './display/eicas_view.js';

export const GLASS_VERSION = '0.1.0';

const $ = (id) => document.getElementById(id);
const store = {
  get(k, d) { try { const v = localStorage.getItem('glass:' + k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem('glass:' + k, JSON.stringify(v)); } catch { /* storage unavailable */ } },
};
const SPAWNS = [
  { mode: 1, label: 'Runway, engines running' },
  { mode: 2, label: 'Final, 6 km out' },
  { mode: 3, label: 'Level, 8 km before the runway' },
  { mode: 4, label: 'Intermediate fix, 10 NM' },
  { mode: 5, label: "Destination's final" },
];

export async function boot({ createAvdModule, wasmBytes, aircraft }) {
  const status = $('status');
  let A;
  try {
    A = await createAvdModule(wasmBytes);
    if (A.abiVersion !== EXPECTED_CORE_ABI) throw new Error(`flight core ABI ${A.abiVersion}, Glass reads ABI ${EXPECTED_CORE_ABI}`);
  } catch (e) {
    status.textContent = 'The flight core could not start: ' + e.message;
    return;
  }
  const ids = Object.keys(aircraft);
  const names = Object.fromEntries(ids.map((id) => [id, JSON.parse(aircraft[id]).display_name ?? id]));

  // ------------------------------------------------------------ state ----
  const ui = {
    aircraft: ids.includes(store.get('aircraft', 'top_g')) ? store.get('aircraft', 'top_g') : ids[0],
    spawn: store.get('spawn', 3),
    range: store.get('range', 20),
    paused: false,
    speech: store.get('speech', false),
    destination: store.get('destination', null),
  };
  let G = null, map = { route: null, region: [] }, lastCallout = { text: '', t: 0 };
  const input = { pitch: 0, roll: 0, yaw: 0, throttle: 0.4, brake: 0, trim: 0 };
  const keys = new Set();

  // ---------------------------------------------------------- choices ----
  const acSel = $('aircraft');
  acSel.innerHTML = ids.map((id) => `<option value="${id}">${names[id]}</option>`).join('');
  acSel.value = ui.aircraft;
  const spSel = $('spawn');
  spSel.innerHTML = SPAWNS.map((s) => `<option value="${s.mode}">${s.label}</option>`).join('');
  spSel.value = String(ui.spawn);
  const destSel = $('destination');

  function fillDestinations() {
    const list = A.destinations();
    destSel.innerHTML = '<option value="">None (runway in use)</option>' + list.flatMap((d, ai) => d.ends.map((e, ri) =>
      `<option value="${ai}:${ri}">${d.icao} ${e.runway}${e.ils ? ' · ILS ' + e.ils : ''}</option>`)).join('');
    destSel.value = ui.destination ?? '';
  }

  function controlIndex(id) { return A.cockpit().find((c) => c.id === id); }
  function stepControl(id, dir) {
    const c = controlIndex(id);
    if (!c) return;
    const n = c.positions.length, target = Math.max(0, Math.min(n - 1, c.position + dir));
    for (let k = 0; k < ((target - c.position + n) % n); ++k) A.avd_cockpit_cycle(c.index);
  }
  function toggleControl(id) { const c = controlIndex(id); if (c) A.avd_cockpit_cycle(c.index); }

  function load() {
    const json = aircraft[ui.aircraft];
    const errors = A.load(json);
    if (errors.length) { status.textContent = 'Aircraft data rejected: ' + errors[0]; return false; }
    fillDestinations();
    if (ui.destination) { const [ai, ri] = ui.destination.split(':').map(Number); A.avd_set_destination(ai, ri); }
    A.avd_set_assist(1);
    spawn();
    return true;
  }
  function spawn() {
    A.avd_reset(ui.spawn);
    if (ui.destination) { const [ai, ri] = ui.destination.split(':').map(Number); A.avd_set_destination(ai, ri); }
    input.throttle = ui.spawn === 1 ? 0 : A.avd_trim_throttle();
    input.pitch = input.roll = input.yaw = input.trim = 0;
    G = new GlassAvionics({ aircraft: JSON.parse(aircraft[ui.aircraft]), markings: A.markings(), loadsheet: A.loadsheet(false), controls: A.cockpit() });
    const f = readFrame(A);
    G.altAlert.select(ui.spawn === 1 ? 10000 : Math.round(f.air.altitudeFt / 1000) * 1000);
    setLandingRunway();
    refreshMap();
    syncPanel();
  }
  function refreshMap() { map = { route: A.route(), region: A.region() }; }
  // Landing runway: the destination's, else the runway in use. Minimums
  // default to a CAT I decision altitude, 200 ft above its field.
  function setLandingRunway() {
    const rwy = A.runway();
    let elevationM = rwy[2], headingDeg = rwy[3];
    if (ui.destination) {
      const [ai, ri] = ui.destination.split(':').map(Number), ap = A.region()[ai];
      if (ap?.ends[ri]) { elevationM = ap.elevation; headingDeg = ap.ends[ri].heading; }
    }
    G.runwayHeadingTrueDeg = headingDeg;
    G.minimums.set('BARO', Math.round((elevationM / 0.3048 + 200) / 10) * 10);
  }

  // ----------------------------------------------------------- panel ----
  function syncPanel() {
    $('sel-alt').value = G.altAlert.selectedFt;
    $('mins').value = G.minimums.valueFt;
    $('mins-kind').value = G.minimums.kind;
    $('nav-src').value = G.navSource;
    $('range').textContent = `${ui.range} NM`;
    $('speech').checked = ui.speech;
    $('pause').setAttribute('aria-pressed', String(ui.paused));
  }
  acSel.onchange = () => { ui.aircraft = acSel.value; store.set('aircraft', ui.aircraft); load(); };
  spSel.onchange = () => { ui.spawn = +spSel.value; store.set('spawn', ui.spawn); spawn(); };
  destSel.onchange = () => {
    ui.destination = destSel.value || null; store.set('destination', ui.destination);
    if (ui.destination) { const [ai, ri] = ui.destination.split(':').map(Number); A.avd_set_destination(ai, ri); } else A.avd_set_destination(-1, 0);
    setLandingRunway();
    refreshMap();
    syncPanel();
  };
  $('reset').onclick = spawn;
  $('pause').onclick = () => { ui.paused = !ui.paused; syncPanel(); };
  $('sel-alt').onchange = (e) => G.altAlert.select(+e.target.value || 0);
  $('mins').onchange = (e) => G.minimums.set(G.minimums.kind, +e.target.value || 0);
  $('mins-kind').onchange = (e) => G.minimums.set(e.target.value, G.minimums.valueFt);
  $('nav-src').onchange = (e) => { G.navSource = e.target.value; };
  $('baro-std').onclick = () => { const c = readFrame(A).air.kollsmanInHg; A.avd_altimeter_adjust(Math.round((29.92 - c) * 100)); };
  $('baro-dn').onclick = () => A.avd_altimeter_adjust(-1);
  $('baro-up').onclick = () => A.avd_altimeter_adjust(1);
  $('range-dn').onclick = () => setRange(-1);
  $('range-up').onclick = () => setRange(1);
  $('speech').onchange = (e) => { ui.speech = e.target.checked; store.set('speech', ui.speech); };
  $('master-warning').onclick = $('master-caution').onclick = () => G.acknowledge();
  function setRange(d) {
    const i = Math.max(0, Math.min(ND_RANGES.length - 1, ND_RANGES.indexOf(ui.range) + d));
    ui.range = ND_RANGES[i] ?? 20; store.set('range', ui.range); syncPanel();
  }

  // -------------------------------------------------------- keyboard ----
  const FLIGHT_KEYS = new Set(['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'Space', 'BracketLeft', 'BracketRight']);
  addEventListener('keydown', (e) => {
    if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
    if (FLIGHT_KEYS.has(e.code)) { keys.add(e.code); e.preventDefault(); }
    if (e.repeat) return;
    switch (e.code) {
      case 'KeyG': toggleControl('gear.lever'); break;
      case 'KeyF': stepControl('flaps.lever', 1); break;
      case 'KeyV': stepControl('flaps.lever', -1); break;
      case 'KeyB': stepControl('speedbrake.lever', 1); break;
      case 'KeyN': stepControl('speedbrake.lever', -1); break;
      case 'KeyP': toggleControl('brakes.parking'); break;
      case 'KeyZ': setRange(-1); break;
      case 'KeyX': setRange(1); break;
      case 'Enter': G.acknowledge(); break;
      case 'KeyR': spawn(); break;
      case 'Escape': ui.paused = !ui.paused; syncPanel(); break;
      default:
        if (/^Digit[0-9]$/.test(e.code)) input.throttle = e.code === 'Digit0' ? 1 : (+e.code.slice(5)) / 10;
        else if (e.code === 'Equal' || e.code === 'NumpadAdd') input.throttle = Math.min(1, input.throttle + 0.05);
        else if (e.code === 'Minus' || e.code === 'NumpadSubtract') input.throttle = Math.max(0, input.throttle - 0.05);
    }
  });
  addEventListener('keyup', (e) => keys.delete(e.code));
  addEventListener('blur', () => keys.clear());

  // Held keys move the stick toward a deflection and it centres when released.
  function updateInput(dt) {
    const toward = (v, target, rate) => v + Math.max(-rate * dt, Math.min(rate * dt, target - v));
    const axis = (neg, pos) => (keys.has(pos[0]) || keys.has(pos[1]) ? 1 : 0) - (keys.has(neg[0]) || keys.has(neg[1]) ? 1 : 0);
    input.pitch = toward(input.pitch, 0.45 * axis(['ArrowUp', 'KeyW'], ['ArrowDown', 'KeyS']), 1.5);
    input.roll = toward(input.roll, 0.6 * axis(['ArrowLeft', 'KeyA'], ['ArrowRight', 'KeyD']), 2.5);
    input.yaw = toward(input.yaw, axis(['KeyQ', 'KeyQ'], ['KeyE', 'KeyE']), 3);
    input.brake = keys.has('Space') ? 1 : 0;
    input.trim = (keys.has('BracketRight') ? 1 : 0) - (keys.has('BracketLeft') ? 1 : 0);
  }

  // ------------------------------------------------------------ loop ----
  if (!load()) return;
  status.textContent = '';
  $('build').textContent = `Glass ${GLASS_VERSION} · flight core ${A.buildId()} · ABI ${A.abiVersion}`;
  let last = performance.now(), mapTimer = 0, model = null;
  function frame(now) {
    const dt = Math.min(0.25, Math.max(0, (now - last) / 1000));
    last = now;
    updateInput(dt);
    if (!ui.paused) {
      A.avd_set_input(input.pitch, input.roll, input.yaw, input.throttle, 1, input.brake, input.brake, input.trim, 0);
      A.avd_advance(dt);
      model = G.update(readFrame(A), dt);
      if ((mapTimer += dt) > 1) { mapTimer = 0; refreshMap(); }
      for (const c of model.callouts) {
        lastCallout = { text: c, t: now };
        if (ui.speech && globalThis.speechSynthesis) speechSynthesis.speak(new SpeechSynthesisUtterance(c.toLowerCase()));
      }
    }
    if (model) {
      drawPfd($('pfd'), model, { callout: now - lastCallout.t < 1500 ? lastCallout.text : '' });
      drawNd($('nd'), model, map, ui.range);
      drawEicas($('eicas'), model);
      $('master-warning').classList.toggle('lit', model.eicas.cas.masterWarning);
      $('master-caution').classList.toggle('lit', model.eicas.cas.masterCaution);
      $('thr').textContent = `${Math.round(input.throttle * 100)} %`;
    }
    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
  globalThis.GLASS = { A, get model() { return model; }, get avionics() { return G; }, input, ui };
}
