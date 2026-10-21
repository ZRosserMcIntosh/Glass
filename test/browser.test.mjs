// Glass — headless-Chromium test of the bundled page.
// SPDX-License-Identifier: MIT
// Usage: node test/browser.test.mjs [dist/glass.html] [screenshot.png]   (needs the `playwright` package)
// Boots the self-contained page, checks that the three displays draw from the
// core, that alerts reach the master lights, and that keys act on the core.
import { createRequire } from 'node:module';
import { execSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const require = createRequire(import.meta.url);
let playwright;
try { playwright = require('playwright'); } catch {
  playwright = require(path.join(execSync('npm root -g').toString().trim(), 'playwright'));
}
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const pagePath = path.resolve(process.argv[2] ?? path.join(root, 'dist', 'glass.html'));
const shot = process.argv[3] ?? path.join(root, 'dist', 'glass.png');

let failures = 0;
const check = (ok, what) => { console.log(`${ok ? 'ok  ' : 'FAIL'} ${what}`); if (!ok) ++failures; };
const launch = {};
if (process.env.PLAYWRIGHT_CHROMIUM) launch.executablePath = process.env.PLAYWRIGHT_CHROMIUM;
const browser = await playwright.chromium.launch(launch);
try {
  const errors = [];
  const page = await browser.newPage({ viewport: { width: 1500, height: 1100 } });
  page.on('pageerror', (e) => errors.push(e.message));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  // Google Fonts may be unreachable in CI; serve nothing (the page has fallbacks).
  await page.route(/fonts\.(googleapis|gstatic)\.com/, (r) => r.fulfill({ status: 200, contentType: 'text/css', body: '' }));
  await page.addInitScript(() => { try { localStorage.clear(); } catch { /* none */ } });
  await page.goto('file://' + pagePath);
  await page.waitForFunction(() => globalThis.GLASS?.model, null, { timeout: 20000 });
  check(true, 'page boots the flight core and the avionics');
  check(/flight core .+ · ABI 1/.test(await page.textContent('#build')), `build line: ${await page.textContent('#build')}`);
  check((await page.$$('#aircraft option')).length === 6, 'six glass-profile aircraft offered');

  // Each display has drawn: sample a few pixels.
  const pixel = (id, fx, fy) => page.$eval('#' + id, (c, [fx, fy]) => Array.from(c.getContext('2d').getImageData(Math.round(c.width * fx), Math.round(c.height * fy), 1, 1).data), [fx, fy]);
  await page.waitForTimeout(1500);
  const sky = await pixel('pfd', 0.5, 0.18);
  check(sky[2] > 150 && sky[2] > sky[0] + 60, `PFD: sky above the horizon (${sky})`);
  const m = await page.evaluate(() => { const m = GLASS.model; return { powered: m.powered, ias: m.speed.iasKt, mach: m.air.mach, truth: GLASS.A.jet()[5], source: m.nav.source }; });
  check(m.powered && Math.abs(m.ias - 250) < 6, `level spawn at ${m.ias.toFixed(1)} KIAS`);
  check(Math.abs(m.mach - m.truth) < 0.005, `air data computer Mach ${m.mach.toFixed(3)} vs core ${m.truth.toFixed(3)}`);
  const lit = async (id) => page.$eval('#' + id, (b) => b.classList.contains('lit'));
  check(!(await lit('master-caution')) && !(await lit('master-warning')), 'no master light in a clean cruise');

  // A generator failure: GEN 1 OFF on the EICAS and MASTER CAUTION; Enter acknowledges.
  await page.evaluate(() => GLASS.A.avd_failure(4, 1));
  await page.waitForTimeout(800);
  const cas = await page.evaluate(() => GLASS.model.eicas.cas.list.map((c) => c.text));
  check(cas.includes('GEN 1 OFF'), `generator 1 failure posts GEN 1 OFF (${cas.join(', ')})`);
  check(await lit('master-caution'), 'MASTER CAUTION lit');
  await page.keyboard.press('Enter');
  await page.waitForTimeout(300);
  check(!(await lit('master-caution')), 'Enter acknowledges the master caution');
  await page.evaluate(() => GLASS.A.avd_failure(4, 0));

  // Gear key: G lowers the gear; the EICAS sees it travel.
  await page.keyboard.press('KeyG');
  await page.waitForTimeout(1500);
  const gear = await page.evaluate(() => GLASS.model.eicas.gear);
  check(gear > 0.05, `G lowers the gear (position ${gear.toFixed(2)})`);

  // Another aircraft loads and the displays follow.
  await page.selectOption('#aircraft', 'b738');
  await page.waitForTimeout(1200);
  const name = await page.evaluate(() => GLASS.model.name);
  check(name === '737-800', `aircraft switch: ${name}`);

  // A destination's GPS route from its intermediate fix, in a wind: the ND
  // draws the route and the wind the triangle solves.
  await page.evaluate(() => GLASS.A.avd_set_weather(15, 200, 0, 0, 0.6));
  const dest = await page.$$eval('#destination option', (os) => os.find((o) => /^SBGR 10L/.test(o.textContent))?.value);
  await page.selectOption('#destination', dest);
  await page.selectOption('#spawn', '4');
  await page.waitForTimeout(6000);
  const nd = await page.evaluate(() => ({ wps: GLASS.A.route().wp.length, src: GLASS.model.nav.source, wind: GLASS.model.wind, truth: [GLASS.A.jet()[38], GLASS.A.jet()[39]] }));
  check(nd.wps >= 2 && (nd.src === 'GPS' || nd.src === 'LOC'), `route to SBGR 10L: ${nd.wps} waypoints, source ${nd.src}`);
  const dWind = nd.wind ? Math.abs(((nd.wind.fromDeg - nd.truth[0] + 540) % 360) - 180) : 999;
  check(nd.wind && dWind < 8 && Math.abs(nd.wind.speedKt - nd.truth[1]) < 3,
    `wind triangle ${nd.wind?.fromDeg.toFixed(0)}°/${nd.wind?.speedKt.toFixed(1)} kt vs core ${nd.truth[0].toFixed(0)}°/${nd.truth[1].toFixed(1)} kt`);

  await page.screenshot({ path: shot, fullPage: true });
  console.log(`screenshot: ${shot}`);
  check(errors.length === 0, `no page errors${errors.length ? ': ' + errors.join(' | ') : ''}`);
} finally {
  await browser.close();
}
process.exit(failures ? 1 : 0);
