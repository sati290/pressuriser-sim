// BWR condensate and feed train tests (roadmap steps 5–7). Run: node --test tests/  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, B, K, near, runPlant, run, rated, RATED_U, RATED_C, Q0, W0, P0 } from './bwr-helpers.mjs';

test('rated split: condensate pump, polisher, LP heater, feed pump; same discharge as the lumped pump; two extractions', () => {
  for (const dt of [0.25, 2]) {
    const s = runPlant(rated(), RATED_U, 600, dt), f = s.f;
    near(f.Pd, B.PCD + B.H0 - B.KPU * W0 * W0, 0.01e6, `discharge pressure as before dt=${dt}`);
    assert.ok(f.Ps > 2.0e6 && f.Ps < 2.3e6, `feed pump suction ${f.Ps} dt=${dt}`);
    assert.ok(f.Pcd > f.Ps + 0.2e6 && f.Pcd < B.HC0, `polisher and LP heater drop: ${f.Pcd - f.Ps} dt=${dt}`);
    near(f.wcp, f.fw, 1e-6, `condensate flow = feed flow with the min-flow valve shut dt=${dt}`);
    near(f.Tlp - 273.15, 126.5, 0.3, `LP heater outlet dt=${dt}`);
    near(f.Tfw - 273.15, 216, 0.05, `HP heater outlet dt=${dt}`);
    assert.ok(f.ws > 400 && f.ws < 440, `HP extraction ${f.ws} dt=${dt}`);
    assert.ok(f.wsl > 355 && f.wsl < 395, `LP extraction ${f.wsl} dt=${dt}`);
    near(f.pel / 1e6, 0.28, 1e-3, `LP extraction pressure dt=${dt}`);
    near(f.npsh, 3.0, 0.05, `NPSH available, hotwell half full and saturated dt=${dt}`);
    assert.equal(f.phi, 1, 'no cavitation at rated');
    // the second extraction does more work in the turbine: about 1050 MWe (920 with one stage)
    assert.ok(f.Pgen > 1020e6 && f.Pgen < 1070e6, `gross output ${f.Pgen / 1e6} dt=${dt}`);
    near((f.Qc + f.Pgen / B.ETAG) / Q0, 1, 0.01, `heat balance closes dt=${dt}`);
  }
});

test('condensate pump NPSH: the head falls as the hotwell empties; colder hotwell water helps', () => {
  const pc = B.PCD, T = B.THW;
  const sc = h => B.suction(pc, h * B.HWM, T);
  near(sc(0.5).A, 1 + 2, 1e-3, 'pit 1 m + 2 m of water, saturated');
  // full head where NPSHa ≥ NPSHr = 2 m·(w/WR)²; less at low level and high flow
  const Pcd = 2.3e6, w50 = B.cpFlow(true, Pcd, sc(0.5)), w10 = B.cpFlow(true, Pcd, sc(0.1)), w01 = B.cpFlow(true, Pcd, sc(0.01));
  assert.ok(w10 < w50 && w01 < w10, `flow falls with the hotwell: ${w50}, ${w10}, ${w01}`);
  assert.equal(B.cpFlow(true, Pcd, sc(0)), 0, 'empty: no flow');
  assert.ok(B.cpFlow(true, Pcd, B.suction(pc, 0.1 * B.HWM, T - 10)) > w10, 'subcooled hotwell: more NPSH');
  assert.equal(B.cpFlow(false, Pcd, sc(0.5)), 0, 'stopped: nothing against a pressurised line');
  // in the plant: hotwell drawn down at rated with no makeup, the feed flow falls off
  const s = rated(); s.Mh = 0.08 * B.HWM;
  runPlant(s, RATED_U, 20, 0.5);
  assert.ok(s.f.phi < 1 && s.f.fw < 0.95 * W0, `cavitating condensate pump: φ ${s.f.phi}, feed ${s.f.fw}`);
});

test('minimum-flow valve in A: opens as the forward flow falls, keeps the feed pump above 25 % flow', () => {
  for (const dt of [0.25, 2]) {
    const c = { ...RATED_C(), auto: { ...RATED_C().auto, rv: true } }, s = rated(), k = K.init(c);
    run(s, k, c, 60, dt);
    assert.equal(s.xr, 0, `shut at rated dt=${dt}`);
    // feed valve shut by hand: the forward flow stops, the valve opens fully
    c.auto.fv = false; c.fv = 0; run(s, k, c, 30, dt);
    near(s.xr, 1, 1e-9, `open at no forward flow dt=${dt}`);
    assert.ok(s.f.wp > 0.9 * B.WMIN, `pump flow kept up: ${s.f.wp} dt=${dt}`);
    near(s.f.wcp, s.f.wp, 1e-6, `the condensate pump supplies the recirculation dt=${dt}`);
    assert.equal(s.f.fw, 0, `no forward flow dt=${dt}`);
  }
});

test('feed pump low-suction trip: condensate pump stopped at rated, latched until the run switch is cycled', () => {
  for (const dt of [0.25, 2]) {
    const s = rated();
    let tTrip = 0;
    runPlant(s, { ...RATED_U, cp: false }, 8, dt, x => { if (!tTrip && x.fpt) tTrip = x.t; return true; });
    assert.ok(tTrip >= B.TLO && tTrip < B.TLO + 3, `tripped after ${tTrip} s dt=${dt}`);
    assert.ok(s.f.Ps < B.PLO, `suction pressure ${s.f.Ps} dt=${dt}`);
    // condensate pump back: the feed pump stays tripped and its speed runs down
    runPlant(s, { ...RATED_U }, 4, dt);
    assert.equal(s.fpt, true, `latched dt=${dt}`);
    assert.ok(s.n < 0.9, `speed ran down: ${s.n} dt=${dt}`);
    // switch off, then on again: reset, and the speed comes back
    runPlant(s, { ...RATED_U, fp: false }, dt, dt);
    assert.equal(s.fpt, false, `reset dt=${dt}`);
    runPlant(s, { ...RATED_U }, 12, dt);
    assert.equal(B.fault(s), '');
    assert.ok(s.n > 0.99 && s.f.fw > 1000, `running again: ${s.n}, ${s.f.fw} dt=${dt}`);
  }
});

test('startup: at low vessel pressure the condensate pump alone feeds through the idle feed pump', () => {
  const s = B.init({ P: 0.5e6, L: 13 }), u = { heat: 20e6, n: 0, fv: 0.2, ext: 0, mk: 0, mv: 0, fp: false, srv: [] };
  runPlant(s, u, 10, 0.5);
  assert.equal(s.n, 0, 'feed pump stopped');
  assert.ok(s.f.fw > 50, `condensate pump feeds the vessel: ${s.f.fw}`);
  assert.ok(s.f.Pd < s.f.Ps, `the idle feed pump is a resistance: ${s.f.Ps} → ${s.f.Pd}`);
  // above the condensate pump's 2.6 MPa shutoff nothing gets in without the feed pump
  const h = B.init({ P: 3e6, L: 13 });
  runPlant(h, u, 10, 0.5);
  assert.equal(h.f.fw, 0, 'no feed above the condensate pump shutoff');
});

test('mass is conserved through the train with the min-flow line open (vessel + hotwell + condenser steam)', () => {
  for (const dt of [0.25, 2]) {
    const s = rated();
    let tot = s.M + s.Mh + s.cd.Ms;
    runPlant(s, { ...RATED_U, rv: 1, mk: 10, srv: [true] }, 300, dt, x => { tot += (x.f.mk - x.f.wsrv) * dt; return true; });
    assert.equal(B.fault(s), '');
    assert.ok(s.f.wr > 100, `recirculating: ${s.f.wr}`);
    near(s.M + s.Mh + s.cd.Ms, tot, 1e-3, `mass dt=${dt}`);
  }
});
