// Startup level control: the startup (low-flow) feed valve, the single-element startup level controller,
// the single/three-element selector on the main controller and the transfer between them.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { B, K, near, run, rated, RATED_C, P0 } from './bwr-helpers.mjs';

test('startup feed valve: parallel conductance, about 290 kg/s wide open at 2.2 MPa over the dome', () => {
  assert.equal(K.RSU, B.RSU, 'controller copy of RSU');
  const o = { cp: true, xr: 0, P: P0, pc: B.PCD, Mh: B.HWM / 2, Thw: B.THW };
  // the feed pump speed at which the discharge sits 2.2 MPa over the dome with the startup valve wide open
  let a = 0, b = 1, nw;
  for (let i = 0; i < 40; i++) { const m = (a + b) / 2; nw = B.network({ ...o, n: m, x: 0, xs: 1 }); if (nw.Pd - P0 < 2.2e6) a = m; else b = m; }
  near(nw.fw, 290, 15, 'wide open');
  near(nw.wsu, nw.fw, 1e-9, 'all through the startup valve');
  // both valves open: the flow splits in proportion to the conductances x² and RSU·xs²
  const w = B.network({ ...o, n: 1, x: 0.3, xs: 0.6 });
  near(w.wsu / w.fw, B.RSU * 0.36 / (0.09 + B.RSU * 0.36), 1e-9, 'split');
  // the valve strokes at the feed valve's rate
  const s = rated(); B.step(s, { heat: 3293e6, n: 1, fv: s.fv, sfv: 1, ext: 1, ms1: true, ms2: true, sv: true, cv: s.cv, bp: 0, srv: [] }, 1);
  near(s.sfv, B.FRATE, 1e-12, 'stroke');
});

// depressurise to 1 MPa at 200 MW (fixed heat), as in the feed pump speed test; optionally hand over to
// the startup level controller at 6.9 MPa (main valve to M and closed 1 % every 10 s)
const lowP = (dt, startup) => {
  const c = RATED_C(), s = rated(), k = K.init(c, s);
  run(s, k, c, 300, dt); c.heat = 200; run(s, k, c, 300, dt);
  if (startup) { c.auto.sfv = true; c.auto.fv = false; c.fv = s.fv * 100; while (c.fv > 0) { c.fv = Math.max(0, c.fv - 1); run(s, k, c, 10, dt); } }
  for (let p = 6.9; p >= 0.999; p -= 0.1) { c.pset = p * 1e6; run(s, k, c, 60, dt); }
  run(s, k, c, 2400, dt); // the bypass is wide open (its capacity falls with pressure) until about 1 MPa
  let lo = Infinity, hi = -Infinity, flo = Infinity, fhi = 0;
  run(s, k, c, 1200, dt, x => { lo = Math.min(lo, x.d.level); hi = Math.max(hi, x.d.level); flo = Math.min(flo, x.f.fw); fhi = Math.max(fhi, x.f.fw); return true; });
  return { s, lo, hi, flo, fhi };
};

test('at 1 MPa the main controller swings the feed with the core voids; the startup level controller holds it', () => {
  for (const dt of [0.25, 2]) {
    const m = lowP(dt, false);
    assert.ok(m.hi - m.lo > 0.15 && m.fhi - m.flo > 100, `main, three-element: level ${m.lo.toFixed(3)}–${m.hi.toFixed(3)}, feed ${m.flo.toFixed(0)}–${m.fhi.toFixed(0)} dt=${dt}`);
    const r = lowP(dt, true);
    assert.equal(B.fault(r.s), '');
    near(r.s.d.P / 1e6, 1, 0.01, `pressure dt=${dt}`);
    assert.ok(r.hi - r.lo < 0.02 && r.fhi - r.flo < 10, `startup valve: level ${r.lo.toFixed(3)}–${r.hi.toFixed(3)}, feed ${r.flo.toFixed(0)}–${r.fhi.toFixed(0)} dt=${dt}`);
    near((r.lo + r.hi) / 2, 14.3, 0.02, `on the setpoint dt=${dt}`);
    assert.equal(r.s.fv, 0, 'main valve shut');
  }
});

test('transfer at 300 MW: startup valve → main valve single-element → three-element, the level within 0.08 m', () => {
  for (const dt of [0.25, 2]) {
    const q = 300e6, hb = B.heatBalance(q, P0, 1), x0 = B.fwValve(hb.W, 1, P0);
    const s = B.init({ P: P0, L: 14.3, gen: hb.W, cv: B.cvPos(hb.W, P0), fv: 0, sfv: x0 / Math.sqrt(B.RSU), n: 1, tfw: hb.Tfw - 273.15 });
    const c = { ...RATED_C(), heat: 300, cv: B.cvPos(hb.W, P0) * 100, fv: 0, sfv: s.sfv * 100, fwe: 1 };
    c.auto.sfv = true; c.auto.fv = false;
    const k = K.init(c, s);
    let lo = Infinity, hi = -Infinity;const rec = x => { lo = Math.min(lo, x.d.level); hi = Math.max(hi, x.d.level); return true; };
    run(s, k, c, 600, dt, rec);
    near(s.f.wsu, hb.W, 0.02 * hb.W, `startup valve passes the feed dt=${dt}`);
    c.auto.fv = true; run(s, k, c, 120, dt, rec);
    assert.ok(s.fv < 0.01, `main valve in A stays near shut while the startup valve passes the flow: ${s.fv} dt=${dt}`);
    c.auto.sfv = false; c.sfv = s.sfv * 100;
    while (c.sfv > 0) { c.sfv = Math.max(0, c.sfv - 2); run(s, k, c, 10, dt, rec); }
    run(s, k, c, 300, dt, rec);
    assert.equal(s.sfv, 0, 'startup valve shut');
    near(s.f.fw, hb.W, 0.03 * hb.W, `main valve took the flow dt=${dt}`);
    // the selector moves the feedforward into the integral: the flow demand does not step
    const F0 = k.dem.fw; c.fwe = 3; run(s, k, c, dt, dt, rec);
    near(k.dem.fw, F0, 3, `no bump on the selector dt=${dt}`);
    run(s, k, c, 600, dt, rec);
    assert.ok(lo > 14.22 && hi < 14.38, `level ${lo.toFixed(3)}–${hi.toFixed(3)} dt=${dt}`);
    near(s.d.level, 14.3, 0.01, `settled dt=${dt}`);
  }
});

test('startup valve in M: the integrals track the slider and the measured flow, so the notch sits on the slider', () => {
  const q = 300e6, hb = B.heatBalance(q, P0, 1), x0 = B.fwValve(hb.W, 1, P0);
  const s = B.init({ P: P0, L: 14.3, gen: hb.W, cv: B.cvPos(hb.W, P0), fv: 0, sfv: x0 / Math.sqrt(B.RSU), n: 1, tfw: hb.Tfw - 273.15 });
  const c = { ...RATED_C(), heat: 300, cv: B.cvPos(hb.W, P0) * 100, fv: 0, sfv: s.sfv * 100, fwe: 1 };
  c.auto.sfv = true; c.auto.fv = false;
  const k = K.init(c, s);
  run(s, k, c, 600, 1);
  // to M at the present opening: steady, so the notch stays on the slider
  c.auto.sfv = false; c.sfv = s.sfv * 100; run(s, k, c, 60, 1);
  near(k.dem.sfv * 100, c.sfv, 0.5, 'notch on the slider');
  // switching to A starts from there: no step in the valve
  const x = s.sfv; c.auto.sfv = true; run(s, k, c, 5, 1); near(s.sfv, x, 0.01, 'bumpless');
});
