// BWR recirculation tests: MG set, jet pump, core flow, natural circulation, flow control of power.
import test from 'node:test';
import assert from 'node:assert/strict';
import { B, K, near, runPlant, run, rated, RATED_U, RATED_C, Q0 } from './bwr-helpers.mjs';
// rated settings with the kinetics in charge and the recirculation on the page's controls
const KC = s => { const c = { ...RATED_C(), rod: s.core.x * 100, mg: 100, wset: 100, rp: true }; delete c.heat; c.auto.mg = true; return c; };

test('rated recirculation: 4300 kg/s drive flow at 1.6 MPa pump head entrains twice that; 12,900 kg/s core flow', () => {
  const s = rated();
  near(s.rc.Wd, 4300, 5, 'drive flow'); near(s.rc.Wc, 12900, 10, 'core flow');
  near((s.rc.Wc - s.rc.Wd) / s.rc.Wd, 2, 0.01, 'M ratio');
  near(B.HS - B.KPP * 4300 ** 2, 1.6e6, 1, 'pump head at rated');
  assert.ok(s.d.Vb > 25 && s.d.Vb < 40, `core and chimney voids ${s.d.Vb} m³`);
});

test('flow control of power along the rod line: 40 % flow gives about 60 % power, and back (dt 0.25 and 2 s)', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), c = KC(s), k = K.init(c);
    run(s, k, c, 60, dt); c.wset = 40; run(s, k, c, 900, dt);
    near(s.f.Wc / 12900, 0.4, 0.01, `flow follows the setpoint dt=${dt}`);
    assert.ok(s.f.Pth / Q0 > 0.55 && s.f.Pth / Q0 < 0.7, `power ${s.f.Pth / Q0} dt=${dt}`);
    assert.ok(s.rc.ng > 0.2 && s.rc.ng < 0.4, `MG set speed ${s.rc.ng}`);
    near(s.d.P / 1e6, 7, 0.01, 'pressure held'); near(s.d.level, 14.3, 0.1, 'level held');
    c.wset = 100; run(s, k, c, 900, dt);
    near(s.f.Pth / Q0, 1, 0.01, `back at rated dt=${dt}`);
  }
});

test('MG set speed limiter: the speed ramps at 2 %/s and stops at 20 % while the breaker is closed', () => {
  const s = rated(), u = { ...RATED_U, mg: 0 };
  runPlant(s, u, 10, 0.5); near(s.rc.ng, 0.8, 1e-9, 'ramp');
  runPlant(s, u, 60, 0.5); near(s.rc.ng, B.NMIN, 1e-9, 'minimum speed');
  assert.ok(s.f.Wc > 0.2 * 12900 && s.f.Wc < 0.5 * 12900, `core flow at minimum speed ${s.f.Wc}`);
});

test('recirculation pump trip at rated: coastdown, power falls with the flow, natural circulation about a third', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), c = KC(s), k = K.init(c); c.auto.mg = false;
    run(s, k, c, 10, dt); c.rp = false; let pmin = 9;
    run(s, k, c, 10, dt, x => { pmin = Math.min(pmin, x.f.Pth); return true; });
    near(s.rc.ng, Math.exp(-10 / B.TCD), 0.01, `speed coasts down dt=${dt}`);
    run(s, k, c, 290, dt, x => { pmin = Math.min(pmin, x.f.Pth); return true; });
    assert.ok(s.f.Wc / 12900 > 0.25 && s.f.Wc / 12900 < 0.4, `natural circulation ${s.f.Wc / 12900} dt=${dt}`);
    assert.ok(s.f.Pth / Q0 > 0.45 && s.f.Pth / Q0 < 0.7, `power ${s.f.Pth / Q0} dt=${dt}`);
    assert.ok(pmin < s.f.Pth, 'power dips below where it settles');
    // the throat sits below the downcomer pressure, so some water still flows forward through the stopped pump
    assert.ok(Math.abs(s.rc.Wd) < 0.2 * 4300, `drive flow small: ${s.rc.Wd}`);
    assert.equal(B.fault(s), '');
  }
});

test('natural circulation with the pump off: about a third of rated flow at 30 % heat, none without heat', () => {
  const s = B.init({ P: 7e6, L: 14.3, rp: false, mg: 0, q: 0.3 * Q0, gen: 0.3 * Q0 / 1.6e6, tfw: 190 });
  assert.ok(s.f.Wc / 12900 > 0.25 && s.f.Wc / 12900 < 0.4, `${s.f.Wc / 12900}`);
  const z = B.init({ P: 7e6, L: 14.3, rp: false, mg: 0 });
  assert.ok(Math.abs(z.f.Wc) < 0.03 * 12900, `no heat, little flow: ${z.f.Wc}`);
});

test('swell depends on flow: at the same heat, less core flow, more voids', () => {
  const a = runPlant(rated(), { ...RATED_U, heat: 0.5 * Q0, mg: 1 }, 120, 0.5), b = runPlant(rated(), { ...RATED_U, heat: 0.5 * Q0, mg: 0.3 }, 120, 0.5);
  assert.ok(b.d.Vb > a.d.Vb + 5, `voids ${b.d.Vb} at low flow vs ${a.d.Vb}`);
});

test('core flow controller: in M the notch is the operator speed plus the correction; A starts there', () => {
  const s = rated(), c = { ...KC(s), auto: { ...KC(s).auto, mg: false } }, k = K.init(c);
  run(s, k, c, 30, 0.5); near(k.dem.mg, 1, 0.01, 'balanced');
  c.wset = 90; K.demands(k, s, c, 0.5);
  near(k.dem.mg, 1 + K.KW * (0.9 * K.WCR - s.f.Wc), 1e-9, 'demand lowers by KW·error');
  c.auto.mg = true; const u = K.update(k, s, c, 0.5); near(u.mg, k.dem.mg, 1e-12, 'A from the notch');
});
