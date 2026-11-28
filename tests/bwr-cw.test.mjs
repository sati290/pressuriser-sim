// BWR circulating water tests (roadmap step 2). Run: node --test tests/  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, B, K, near, runPlant, rated, RATED_U, RATED_C } from './bwr-helpers.mjs';

test('circulating water: open valve with the pump on passes the design flow; throttling and the pump switch', () => {
  near(B.cwTarget(true, 1), B.WCW, 1e-6, 'design flow');
  assert.equal(B.cwTarget(false, 1), 0, 'pump stopped');
  assert.equal(B.cwTarget(true, 0), 0, 'valve shut');
  const w50 = B.cwTarget(true, 0.5), w20 = B.cwTarget(true, 0.2);
  assert.ok(w50 < B.WCW && w20 < w50 && w20 > 0, `throttled: ${w50}, ${w20}`);
  // the flow follows with the pump lag: one time constant after a trip it is down to 1/e
  for (const dt of [0.25, 2]) {
    const s = rated(); runPlant(s, { ...RATED_U, cwp: false }, 12, dt);
    near(s.f.wcw / B.WCW, Math.exp(-1), 0.01, `coast-down dt=${dt}`);
  }
});

test('loss of circulating water at rated raises the back pressure until the exhaust diaphragms would blow', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(); let p30 = 0, tf = 0;
    runPlant(s, { ...RATED_U, cwp: false }, 600, dt, x => { if (x.t <= 30) p30 = x.f.Pc; tf = x.t; });
    assert.ok(p30 > 12e3, `Pc after 30 s: ${p30}`);
    assert.equal(B.fault(s), 'condenser', `condenser fault (t ${tf} s, Pc ${s.f.Pc})`);
    assert.ok(tf < 300, `fault within 5 min: ${tf} s`);
  }
});

test('warm circulating water inlet and a throttled CW valve raise the back pressure', () => {
  for (const dt of [0.25, 2]) {
    const p = u => runPlant(rated(), u, 900, dt).f.Pc;
    const p20 = p(RATED_U), p30 = p({ ...RATED_U, tcw: 30 + 273.15 }), pv = p({ ...RATED_U, cwv: 0.3 });
    assert.ok(p30 > p20 + 3e3, `30 °C inlet: ${p30} vs ${p20} dt=${dt}`);
    assert.ok(pv > p20 + 1e3, `CW valve 30 %: ${pv} vs ${p20} dt=${dt}`);
  }
  // the outlet temperature follows the duty and the actual flow
  const s = runPlant(rated(), { ...RATED_U, cwv: 0.5 }, 300, 1);
  near(s.f.Tcw - s.f.Tcwi, s.f.Qc / (s.f.wcw * B.CPW), 1e-6, 'CW rise');
});

test('controller passes the circulating water and air removal settings through in SI', () => {
  const c = { ...RATED_C(), cwp: false, cwv: 40, tcw: 25, sj: true, vp: true, vb: false }, k = K.init(c), s = rated();
  const u = K.update(k, s, c, 1);
  assert.deepEqual([u.cwp, u.sj, u.vp, u.vb], [false, true, true, false]);
  near(u.cwv, 0.4, 1e-12, 'valve'); near(u.tcw, 298.15, 1e-9, 'inlet');
  const u0 = K.update(k, s, RATED_C(), 1);
  assert.ok(!('cwp' in u0) && !('tcw' in u0), 'absent settings stay absent (plant normal line-up)');
});
