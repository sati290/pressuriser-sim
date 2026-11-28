// BWR step 7: condensate storage tank, makeup and reject valves, split-range hotwell level control.
import test from 'node:test';
import assert from 'node:assert/strict';
import { B, K, near, run, runPlant, rated, RATED_C, RATED_U, P0, W0, FV0 } from './bwr-helpers.mjs';

const C7 = () => ({ ...RATED_C(), auto: { n: true, fv: true, mv: true, mu: true, rj: true } });
const total = s => s.M + s.Mh + s.cd.Ms + s.Mhs + s.Mls + s.Mcst;

test('CST valve flows: makeup from the tank head and the condenser vacuum, reject from the condensate pump discharge', () => {
  near(B.muFlow(1, 0.7 * B.CSTM, 6.5e3), 100, 1e-9, 'makeup fully open, 70 % full, under vacuum');
  assert.ok(B.muFlow(1, 0.7 * B.CSTM, 0.1013e6) < 60, 'less with the vacuum lost');
  assert.ok(B.muFlow(1, 0.2 * B.CSTM, 6.5e3) < B.muFlow(1, 0.9 * B.CSTM, 6.5e3), 'less with the tank low');
  assert.equal(B.muFlow(1, 0, 6.5e3), 0, 'empty tank: no makeup');
  assert.equal(B.muFlow(0, 0.7 * B.CSTM, 6.5e3), 0, 'shut');
  const s = rated();
  assert.equal(s.Mcst, 0.7 * B.CSTM, 'starts 70 % full');
  runPlant(s, { ...RATED_U, rj: 1 }, 30, 0.5);
  assert.ok(s.f.wrj > 80 && s.f.wrj < 120, `reject fully open at rated: ${s.f.wrj} kg/s`);
  near(s.f.wcp - s.f.wrj + s.f.wd, s.f.wp, 1e-6, 'the condensate pump carries the reject too');
  assert.ok(s.f.mk < -80, `net makeup negative: ${s.f.mk}`);
  assert.ok(s.Tcst > B.TMK, 'the rejected condensate warms the tank');
});

test('split range hotwell control in A: rejects when high, makes up when low, never both (dt 0.25 and 2 s)', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: P0, L: 14.3, gen: W0, mv: W0 / B.WMS, fv: FV0, n: 1, tfw: 216, hw: 0.6 }), c = C7(), k = K.init(c);
    let both = false;
    let lo = 1;
    run(s, k, c, 3000, dt, x => { if (k.dem.mu > 0 && k.dem.rj > 0) both = true; lo = Math.min(lo, x.f.hw); return true; });
    assert.ok(!both, `never both asked open dt=${dt}`);
    // a PI on an integrating level overshoots from an initial offset (about a third here)
    assert.ok(lo > 0.45, `undershoot ${lo} dt=${dt}`);
    near(s.f.hw, 0.5, 0.01, `high hotwell brought down by reject dt=${dt}`);
    assert.ok(s.Mcst > 0.7 * B.CSTM + 40e3, `rejected into the CST: ${s.Mcst} dt=${dt}`);
    // SRV open: the steam goes to the pool, the makeup valve replaces it from the CST
    c.srv = [true]; run(s, k, c, 600, dt, x => { if (k.dem.mu > 0 && k.dem.rj > 0) both = true; return true; });
    assert.ok(!both, `never both asked open dt=${dt}`);
    assert.ok(s.f.wmu > 50 && s.f.xrj === 0, `makeup from the CST: ${s.f.wmu} dt=${dt}`);
    near(s.f.hw, 0.5, 0.03, `hotwell held dt=${dt}`);
    assert.equal(B.fault(s), '');
  }
});

test('whole-plant water mass: vessel + hotwell + condenser steam + heater shells + CST, minus SRV discharge (dt 0.25 and 2 s)', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: P0, L: 14.3, gen: W0, mv: W0 / B.WMS, fv: FV0, n: 1, tfw: 216, hw: 0.6 }), c = C7(), k = K.init(c);
    let tot = total(s), rj = 0, mu = 0;
    run(s, k, c, 300, dt, x => { tot -= x.f.wsrv * dt; rj = Math.max(rj, x.f.wrj); return true; });
    c.srv = [true]; B.fail(s, 'hdv', true);
    run(s, k, c, 600, dt, x => { tot -= x.f.wsrv * dt; mu = Math.max(mu, x.f.wmu); return true; });
    assert.ok(s.f.wsrv > 80 && rj > 50 && mu > 50, `relieving, rejected ${rj}, made up ${mu} kg/s`);
    assert.equal(B.fault(s), '');
    near(total(s), tot, 1e-3, `mass dt=${dt}`);
  }
});

test('CST nearly empty: no makeup, the hotwell drains with an SRV open; M holds the operator openings', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: P0, L: 14.3, gen: W0, mv: W0 / B.WMS, fv: FV0, n: 1, tfw: 216, cst: 0.002 }), c = C7(), k = K.init(c);
    c.srv = [true]; run(s, k, c, 600, dt);
    assert.ok(s.f.xmu > 0.9, `makeup valve asked open dt=${dt}`);
    assert.ok(s.f.wmu < 5, `but little to give: ${s.f.wmu} dt=${dt}`);
    assert.ok(s.f.hw < 0.42, `hotwell falling: ${s.f.hw} dt=${dt}`);
  }
  const s = rated(), c = { ...C7(), auto: { n: true, fv: true, mv: true, mu: false, rj: false }, mu: 30, rj: 0 }, k = K.init(c);
  run(s, k, c, 60, 0.5);
  near(s.xmu, 0.3, 1e-9, 'makeup valve at the operator value');
  assert.equal(s.xrj, 0, 'reject shut');
  assert.ok(k.dem.rj > 0 || k.dem.mu < 0.3, 'demand notch shows the controller would close the makeup');
});
