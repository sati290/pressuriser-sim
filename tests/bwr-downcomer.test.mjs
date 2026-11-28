// BWR downcomer tests: feed subcooling carried to the core inlet, mass and energy with the core in charge.
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, B, K, near, runPlant, run, rated, RATED_U, RATED_C, Q0, P0, TFW } from './bwr-helpers.mjs';

test('rated: the downcomer is about 9 K subcooled, the core region saturated; inlet subcooling from the feed mixing', () => {
  const s = rated(), q = common.sat(s.d.T), uf = q.uf, ufw = common.hLiq(TFW, P0) - P0 / common.liqRho(TFW, P0);
  near(s.d.sub, 0, 1e-4, 'core region saturated');
  assert.ok(s.d.subdc > 7 && s.d.subdc < 11, `downcomer subcooling ${s.d.subdc} K`);
  near(s.f.dhin, s.f.fw * 0 + B.WR * (uf - ufw) / s.f.Wc, 300, 'inlet subcooling = fw·(u′ − u_fw)/W');
  for (const dt of [0.25, 2]) { const r = runPlant(rated(), RATED_U, 300, dt); near(r.d.subdc, s.d.subdc, 0.05, `steady dt=${dt}`); }
});

test('colder feed reaches the core inlet with the downcomer transit delay (~6 s at rated flow)', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(); runPlant(s, RATED_U, 4, dt); const d0 = s.f.dhin;
    s.Th -= 30; // a 30 K step in the feed temperature (heater outlet)
    runPlant(s, { ...RATED_U, ext: 0 }, 2, dt); const d2 = s.f.dhin;
    runPlant(s, { ...RATED_U, ext: 0 }, 40, dt); const d40 = s.f.dhin;
    assert.ok(d40 > d0 + 10e3, `inlet subcooling rises: ${d0} → ${d40}`);
    assert.ok((d2 - d0) / (d40 - d0) < 0.5, `delayed: ${(d2 - d0) / (d40 - d0)} after 2 s dt=${dt}`);
  }
});

test('pressure rise: the downcomer falls further below saturation, the core voids shrink', () => {
  const s = rated(), E0 = s.d.E, a0 = s.f.ac;
  runPlant(s, { ...RATED_U, mv: 0 }, 5, 0.25);
  assert.ok(s.d.E > E0 * 1.2, `downcomer deficit ${s.d.E} from ${E0}`);
  assert.ok(s.f.ac < a0 - 0.03, `core void ${s.f.ac} from ${a0}`);
});

test('mass and energy are conserved with the kinetics, recirculation and downcomer, through a valve closure and scram', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), c = { ...RATED_C(), rod: s.core.x * 100, mg: 100, wset: 100, rp: true }; delete c.heat; c.auto.mg = true; c.auto.mv = false;
    const k = K.init(c); let m = s.M, e = s.U;
    const acc = x => { const f = x.f; m += (f.fw - f.wst - f.wsrv) * dt; e += (f.Q + f.fw * f.hfw - (f.wst + f.wsrv) * f.hg) * dt; return true; };
    run(s, k, c, 60, dt, acc); c.mv = 0;
    run(s, k, c, 2, dt, acc); B.scram(s); c.rod = 0;
    run(s, k, c, 300, dt, acc);
    near(s.M, m, 1e-6 * m, `mass dt=${dt}`);
    near(s.U, e, 1e-9 * Math.abs(e), `energy dt=${dt}`);
    assert.equal(B.fault(s), '');
  }
});
