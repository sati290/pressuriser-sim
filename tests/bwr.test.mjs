// BWR vessel regression tests. Run: node --test tests/bwr.test.mjs  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import '../common.js';
import '../bwr-sim.js';
import '../bwr-control.js';

const common = globalThis.PZCommon, B = globalThis.BWRSim, K = globalThis.BWRCtl;
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, expected ${b} ± ${tol}`);
const MPa = p => p / 1e6;

// fixed command, no controller; stops at a fault
const runPlant = (s, u, seconds, dt = 1, onStep) => {
  for (let i = 0; i < Math.round(seconds / dt); i++) {
    B.step(s, u, dt); if (B.fault(s)) break; if (onStep && onStep(s) === false) break;
  }
  return s;
};
// plant + controller; c in panel units (see bwr-control.js)
const run = (s, k, c, seconds, dt = 1, onStep) => {
  for (let i = 0; i < Math.round(seconds / dt); i++) {
    B.step(s, K.update(k, s, c, dt), dt); if (B.fault(s)) break; if (onStep && onStep(s) === false) break;
  }
  return s;
};

// rated conditions: 3293 MW, 7.0 MPa, 216 °C feedwater; steam flow from the heat balance
const P0 = 7e6, Q0 = 3293e6, TFW = 216 + 273.15;
const W0 = Q0 / (common.sat(common.Tsat(P0)).hg - common.hLiq(TFW, P0));
const rated = () => B.init({ P: P0, L: 14.3, gen: W0, mv: W0 / B.WMS });
const RATED_U = { heat: Q0, fw: W0, tfw: TFW, mv: W0 / B.WMS, srv: 0 };
const AUTO = () => ({ fw: true, mv: true, srv: true });
const RATED_C = () => ({ heat: 3293, fw: W0, tfw: 216, mv: W0 / B.WMS * 100, srv: 0, pset: 7e6, lset: 14.3, relSet: 7.7e6, auto: AUTO() });

test('init round-trips pressure and level, with the voids inside the level', () => {
  const s = rated();
  near(MPa(s.d.P), 7.0, 1e-4, 'P');
  near(s.d.level, 14.3, 1e-3, 'level');
  assert.ok(s.d.collapsed < s.d.level - 0.8, `collapsed level ${s.d.collapsed} well below the two-phase level`);
  near(s.d.sub, 0, 1e-4, 'saturated');
});

test('rated heat balance is a steady state at dt 0.25 and 2 s', () => {
  for (const dt of [0.25, 2]) {
    const s = runPlant(rated(), RATED_U, 1200, dt);
    near(MPa(s.d.P), 7.0, 0.002, `P dt=${dt}`);
    near(s.d.level, 14.3, 0.01, `level dt=${dt}`);
    near(s.f.wst, W0, 1, `steam flow dt=${dt}`);
  }
});

test('mass and energy are conserved: closed vessel, then with feed and steam', () => {
  const s = B.init({ P: 7e6, L: 14.3 }), M0 = s.M, U0 = s.U;
  runPlant(s, { heat: 20e6, fw: 0, tfw: 400, mv: 0, srv: 0 }, 600, 1);
  assert.equal(s.f.nsrv, 0, 'no SRV lifted');
  near(s.M, M0, 1e-6 * M0, 'mass');
  near(s.U - U0, 20e6 * 600, 1e-6 * U0, 'energy');
  // with flows: integrate what went in and out at the start-of-step enthalpies the plant records
  let m = s.M, u = s.U;
  runPlant(s, { heat: 100e6, fw: 60, tfw: 450, mv: 0.03, srv: 1 }, 300, 1, x => {
    const f = x.f; m += f.fw - f.wst - f.wsrv; u += f.Q + f.fw * f.hfw - (f.wst + f.wsrv) * f.hg; return true;
  });
  near(s.M, m, 1e-3, 'mass with flows');
  near(s.U, u, 1e-9 * Math.abs(u), 'energy with flows');
});

test('closing the steam valve at rated heat: voids collapse (shrink), pressure rises, SRVs lift', () => {
  const s = rated(), L0 = s.d.level;let Lmin = L0, Pmax = 0, srv = 0;
  runPlant(s, { ...RATED_U, mv: 0 }, 6, 0.25, x => { Lmin = Math.min(Lmin, x.d.level); return true; });
  assert.ok(Lmin < L0 - 0.4, `level shrinks: min ${Lmin} from ${L0}`);
  assert.ok(s.d.P > 7.4e6, `pressure rises: ${MPa(s.d.P)}`);
  // same, SRVs in auto: they open in groups and hold the pressure below the safety setpoint for a while
  const s2 = rated(), k = K.init(), c = { ...RATED_C(), heat: 300, auto: { fw: true, mv: false, srv: true }, mv: 0 };
  run(s2, k, c, 900, 0.5, x => { Pmax = Math.max(Pmax, x.d.P); srv = Math.max(srv, x.f.nsrv); return true; });
  assert.ok(srv >= 4, `SRVs opened: ${srv}`);
  assert.ok(Pmax < B.PSV, `relief mode keeps below the safety setpoint: ${MPa(Pmax)}`);
  assert.equal(B.fault(s2), '');
});

test('opening the steam valve depressurises: the water flashes and the level swells above the collapsed level', () => {
  const s = B.init({ P: 7e6, L: 14.0 }), L0 = s.d.level;
  runPlant(s, { heat: 20e6, fw: 0, tfw: 400, mv: 0.3, srv: 0 }, 2, 0.25);
  assert.ok(s.d.level > L0 + 0.1, `level swells at first: ${s.d.level} from ${L0}`);
  runPlant(s, { heat: 20e6, fw: 0, tfw: 400, mv: 0.3, srv: 0 }, 18, 0.25);
  assert.ok(s.d.P < 6.8e6, `pressure falls: ${MPa(s.d.P)}`);
  assert.ok(s.d.level - s.d.collapsed > 0.2, `swell over collapsed level: ${s.d.level - s.d.collapsed} m`);
  assert.ok(s.d.Vb > 5, `voids from flashing: ${s.d.Vb} m³`);
});

test('cold feedwater condenses steam: pressure falls, voids vanish, water subcooled only slightly', () => {
  const s = B.init({ P: 7e6, L: 14.3 });
  runPlant(s, { heat: 0, fw: 100, tfw: 50 + 273.15, mv: 0, srv: 0 }, 60, 0.5);
  assert.ok(s.d.P < 6.75e6, `pressure falls: ${MPa(s.d.P)}`);
  assert.ok(s.d.sub > 0 && s.d.sub < 1, `subcooling ${s.d.sub} K`);
  near(s.d.Vb, 0, 1e-6, 'no voids');
});

test('safety mode opens every SRV above 8.5 MPa with relief in M at zero', () => {
  const s = B.init({ P: 8.3e6, L: 14.3 });let n = 0;
  runPlant(s, { heat: 400e6, fw: 0, tfw: 400, mv: 0, srv: 0 }, 300, 0.5, x => { n = Math.max(n, x.f.nsrv); return true; });
  assert.equal(n, B.NV);
  assert.ok(s.d.P < 8.8e6, `held near the safety setpoint: ${MPa(s.d.P)}`);
});

test('pressure regulator and three-element feedwater recover from a load step (dt 0.25 and 2 s)', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), k = K.init(), c = RATED_C();
    run(s, k, c, 60, dt);
    c.heat = 2000; run(s, k, c, 1200, dt);
    near(MPa(s.d.P), 7.0, 0.005, `P back to setpoint dt=${dt}`);
    near(s.d.level, 14.3, 0.05, `level back to setpoint dt=${dt}`);
    near(s.f.fw, s.f.wst, 5, `feed matches steam dt=${dt}`);
    c.pset = 6.5e6; run(s, k, c, 900, dt);
    near(MPa(s.d.P), 6.5, 0.005, `new setpoint dt=${dt}`);
  }
});

test('switching the valve and the feedwater from M to A is bumpless', () => {
  const s = rated(), k = K.init(), c = { ...RATED_C(), auto: { fw: false, mv: false, srv: true } };
  run(s, k, c, 30, 0.5);
  c.auto = AUTO(); const u = K.update(k, s, c, 0.5);
  near(u.mv, c.mv / 100, 0.01, 'valve demand at switch-over');
  near(u.fw, c.fw, 5, 'feed demand at switch-over');
});

test('startup heat-up with the valve shut: pressure and level rise at the expected rate', () => {
  const s = B.init({ P: 0.1e6, L: 11.5 }), T0 = s.d.T, L0 = s.d.level;
  runPlant(s, { heat: 30e6, fw: 0, tfw: 323, mv: 0, srv: 0 }, 3600, 2);
  const rate = (s.d.T - T0);
  assert.ok(rate > 60 && rate < 110, `heat-up ${rate} K/h at 30 MW`);
  assert.ok(s.d.level > L0 + 0.8, `thermal expansion raises the level: ${s.d.level}`);
  assert.ok(s.d.P > 0.6e6, `pressurised: ${MPa(s.d.P)}`);
});

test('faults: core uncovered without feedwater, steam lines flooded with too much', () => {
  const s = B.init({ P: 7e6, L: 14.3, gen: W0, mv: W0 / B.WMS });
  runPlant(s, { ...RATED_U, fw: 0 }, 2000, 1);
  assert.equal(B.fault(s), 'uncovered');
  const s2 = B.init({ P: 7e6, L: 14.3 });
  runPlant(s2, { heat: 20e6, fw: 500, tfw: 450, mv: 0.01, srv: 0 }, 2000, 1);
  assert.equal(B.fault(s2), 'flooded');
});
