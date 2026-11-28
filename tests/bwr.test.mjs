// BWR vessel regression tests. Run: node --test tests/bwr.test.mjs  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import * as common from '../common.js';
import * as B from '../bwr-sim.js';
import * as K from '../bwr-control.js';
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
// feed valve opening that passes the rated flow with the pump at full speed
const FV0 = B.fwValve(W0, 1, P0);
const rated = () => B.init({ P: P0, L: 14.3, gen: W0, mv: W0 / B.WMS, fv: FV0, n: 1, tfw: 216 });
const RATED_U = { heat: Q0, n: 1, fv: FV0, ext: 1, mk: 0, mv: W0 / B.WMS, srv: [] };
// no feed: pump stopped, valve shut
const NOFEED = { n: 0, fv: 0, ext: 0, mk: 0 };
const AUTO = () => ({ n: true, fv: true, mv: true, mk: true });
const RATED_C = () => ({ heat: 3293, n: 100, fv: FV0 * 100, ext: 100, mk: 0, mv: W0 / B.WMS * 100, srv: [], pset: 7e6, lset: 14.3, dpset: 2.2e6, auto: AUTO() });

test('init round-trips pressure and level, with the voids inside the level', () => {
  const s = rated();
  near(MPa(s.d.P), 7.0, 1e-4, 'P');
  near(s.d.level, 14.3, 1e-3, 'level');
  assert.ok(s.d.collapsed < s.d.level - 0.8, `collapsed level ${s.d.collapsed} well below the two-phase level`);
  near(s.d.sub, 0, 1e-4, 'saturated');
});

test('rated heat balance is a steady state at dt 0.25 and 2 s: pump, valve, heater and hotwell included', () => {
  near(B.WR, W0, 1e-6, 'plant rated steam flow');
  near(FV0, 0.745, 0.005, 'feed valve about 75 % open at rated');
  for (const dt of [0.25, 2]) {
    const s = runPlant(rated(), RATED_U, 1200, dt);
    near(MPa(s.d.P), 7.0, 0.002, `P dt=${dt}`);
    near(s.d.level, 14.3, 0.01, `level dt=${dt}`);
    near(s.f.wst, W0, 1, `steam flow dt=${dt}`);
    near(s.f.fw, W0, 1, `feed flow dt=${dt}`);
    near(s.f.Tfw - 273.15, 216, 0.05, `feed temperature dt=${dt}`);
    near(s.f.hw, 0.5, 0.002, `hotwell holds: the condensate return matches the feed dt=${dt}`);
  }
  // heat balance helper agrees
  const hb = B.heatBalance(Q0, P0, 1);
  near(hb.W, W0, 0.01, 'heatBalance W'); near(hb.Tfw, TFW, 0.01, 'heatBalance Tfw');
});

test('feed pump and valve: flow from the pump curve against vessel pressure', () => {
  near(B.fwFlow(1, FV0, P0), W0, 1e-6, 'fwValve inverts fwFlow');
  near(B.fwFlow(1, 1, P0), 2400, 10, 'wide open at 7 MPa');
  assert.ok(B.fwFlow(1, FV0, 5e6) > 1.2 * W0, 'more flow at lower vessel pressure');
  assert.equal(B.fwFlow(1, 1, B.H0 + B.PC + 1), 0, 'nothing above the shutoff head');
  assert.equal(B.fwFlow(0.8, 1, P0), 0, '80 % speed: shutoff 6.4 MPa is below 7 MPa');
  assert.equal(B.fwFlow(1, 0, P0), 0, 'valve shut');
  assert.equal(B.fwValve(100, 0.8, P0), null, 'no opening helps');
  near(B.fwFlow(1, FV0, P0, 0.025 * B.HWM), W0 / 2, 1e-6, 'losing suction below 5 % hotwell');
  // the valve strokes at FRATE
  const s = rated(); B.step(s, { ...RATED_U, fv: 1 }, 1);
  near(s.fv, FV0 + B.FRATE, 1e-9, 'valve travel limit');
});

test('feedwater heater: the extraction valve sets the shell pressure, the shell the outlet temperature', () => {
  const r = B.heater(B.WR, 1, B.WR);
  near(r.T - 273.15, 216, 1e-6, 'rated outlet');
  near(r.ps, r.pe * 0.95, 1e3, '5 % drop across the open valve at rated');
  near(r.ws, B.WR * (common.hLiq(TFW, 7e6) - common.hLiq(B.THW, 7e6)) / (common.sat(r.T + B.TTD).hg - common.sat(r.T + B.TTD).hf), 1e-3, 'shell condenses what heats the feed');
  near(B.heaterT(0, 1), B.THW, 1e-9, 'no steam flow, no extraction: hotwell temperature');
  near(B.heaterT(B.WR, 0), B.THW, 1e-9, 'extraction valve shut');
  near(B.THW - 273.15, 45.8, 0.1, 'hotwell at condenser saturation, 0.01 MPa');
  // throttling: the outlet falls monotonically as the valve closes; less feed flow, hotter outlet
  let last = 1e9;
  for (const e of [1, 0.6, 0.3, 0.1, 0.02]) { const T = B.heaterT(B.WR, e); assert.ok(T < last, `outlet falls as the valve closes: ${e} → ${T}`); last = T; }
  assert.ok(B.heaterT(B.WR, 0.3, B.WR / 2) > B.heaterT(B.WR, 0.3, B.WR), 'half the feed, hotter');
  const half = B.heaterT(B.WR / 2, 1) - 273.15;
  assert.ok(half > 170 && half < 200, `half steam flow, lower extraction pressure: ${half}`);
  // the "lose feedwater heating" event: valve to 20 % at rated, about 55 K colder over a few minutes
  const s = rated();
  runPlant(s, { ...RATED_U, ext: 0.2 }, 60, 0.5);
  const T60 = s.f.Tfw - 273.15;
  runPlant(s, { ...RATED_U, ext: 0.2 }, 300, 0.5);
  assert.ok(T60 < 216 - 25 && T60 > 216 - 50, `after 1 min: ${T60}`);
  assert.ok(s.f.Tfw - 273.15 < 216 - 45 && s.f.Tfw - 273.15 > 216 - 75, `after 6 min: ${s.f.Tfw - 273.15}`);
});

test('feed pump speed in A holds the discharge-to-dome differential pressure, at rated and through a depressurisation', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 300, dt);
    near((s.f.Pd - s.d.P) / 1e6, 2.2, 0.005, `rated dt=${dt}`);
    c.heat = 200; c.pset = 1e6; run(s, k, c, 5400, dt);
    near(MPa(s.d.P), 1.0, 0.02, `depressurised dt=${dt}`);
    near((s.f.Pd - s.d.P) / 1e6, 2.2, 0.02, `ΔP held dt=${dt}`);
    assert.ok(s.n < 0.6, `the pump slowed with the vessel pressure: ${s.n} dt=${dt}`);
    near(s.d.level, 14.3, 0.15, `level held dt=${dt}`);
  }
  // the speed moves at most NRATE per second
  const s = rated(); B.step(s, { ...RATED_U, n: 0 }, 1);
  near(s.n, 1 - B.NRATE, 1e-12, 'speed ramp');
});

test('mass and energy are conserved: closed vessel, then with feed and steam', () => {
  const s = B.init({ P: 7e6, L: 14.3, tfw: 177 }), M0 = s.M, U0 = s.U;
  runPlant(s, { heat: 10e6, ...NOFEED, mv: 0, srv: [] }, 600, 1);
  assert.equal(s.f.nsrv, 0, 'no SRV lifted');
  near(s.M, M0, 1e-6 * M0, 'mass');
  near(s.U - U0, 10e6 * 600, 1e-6 * U0, 'energy');
  // with flows: integrate what went in and out at the start-of-step enthalpies the plant records
  // vessel plus hotwell change only by makeup in and relief out (the steam returns as condensate)
  let m = s.M, u = s.U, tot = s.M + s.Mh;
  runPlant(s, { heat: 100e6, n: 1, fv: 0.1, ext: 0.5, mk: 20, mv: 0.03, srv: [true] }, 300, 1, x => {
    const f = x.f; m += f.fw - f.wst - f.wsrv; u += f.Q + f.fw * f.hfw - (f.wst + f.wsrv) * f.hg; tot += f.mk - f.wsrv; return true;
  });
  assert.ok(s.f.fw > 10, `feed flowing: ${s.f.fw}`);
  near(s.M, m, 1e-3, 'mass with flows');
  near(s.U, u, 1e-9 * Math.abs(u), 'energy with flows');
  near(s.M + s.Mh, tot, 1e-3, 'vessel + hotwell mass');
});

test('closing the steam valve at rated heat: voids collapse (shrink), pressure rises, SRVs lift', () => {
  const s = rated(), L0 = s.d.level;let Lmin = L0, Pmax = 0, srv = 0;
  runPlant(s, { ...RATED_U, mv: 0 }, 6, 0.25, x => { Lmin = Math.min(Lmin, x.d.level); return true; });
  assert.ok(Lmin < L0 - 0.4, `level shrinks: min ${Lmin} from ${L0}`);
  assert.ok(s.d.P > 7.4e6, `pressure rises: ${MPa(s.d.P)}`);
  // same at 300 MW: the springs lift the lowest bank, which holds pressure just above its set point
  const s2 = rated(), k = K.init(), c = { ...RATED_C(), heat: 300, auto: { fv: true, mv: false, mk: true }, mv: 0 };
  run(s2, k, c, 900, 0.5, x => { Pmax = Math.max(Pmax, x.d.P); srv = Math.max(srv, x.f.nsrv); return true; });
  assert.ok(srv >= 4, `SRVs opened: ${srv}`);
  assert.ok(Pmax < B.SRV[B.NV - 1].p + 0.1e6, `springs hold it near the set pressures: ${MPa(Pmax)}`);
  assert.equal(B.fault(s2), '');
});

test('valve closure at rated heat, no scram: pressure climbs past the feed pump shutoff head, feed stops, core uncovers', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), k = K.init(), c = { ...RATED_C(), auto: { fv: true, mv: false, mk: true } };
    run(s, k, c, 10, dt); c.mv = 0;
    let fvMax = 0, tStop = 0;
    run(s, k, c, 600, dt, x => { fvMax = Math.max(fvMax, x.fv); if (!tStop && x.d.P > B.H0 + B.PC) tStop = x.t; return true; });
    assert.ok(tStop > 0, `pressure passed the shutoff head dt=${dt}`);
    assert.ok(fvMax > 0.99, `the feed valve opened fully, in vain: ${fvMax} dt=${dt}`);
    assert.equal(s.f.fw, 0, `no feed flow dt=${dt}`);
    assert.equal(B.fault(s), 'uncovered', `dt=${dt}`);
  }
});

test('scram: heat follows the decay curve, ignores the slider, reset hands it back', () => {
  near(B.decayFrac(0), 1, 1e-9, 'full power at the scram');
  near(B.decayFrac(60), 0.033, 0.004, '1 min');
  near(B.decayFrac(3600), 0.013, 0.002, '1 h');
  const s = runPlant(rated(), RATED_U, 1, 0.5); B.scram(s); // heat at the scram is the last applied
  runPlant(s, { ...RATED_U, heat: 1e9 }, 60, 0.5);
  near(s.f.Q, Q0 * B.decayFrac(s.t - 0.5 - s.scram.t), 1e3, 'heat at 1 min is decay heat (taken at the start of the step), not the command');
  B.scram(s); near(s.scram.P0, Q0, 1, 'a second scram keeps the first');
  B.reset(s); runPlant(s, { ...RATED_U, heat: 50e6 }, 1, 0.5);
  near(s.f.Q, 50e6, 1, 'after reset the command applies');
});

test('valve closure with scram: pressure held by the relief valves, level kept by feedwater, hotwell by makeup', () => {
  const s = rated(), k = K.init(), c = { ...RATED_C(), auto: { fv: true, mv: false, mk: true } };
  run(s, k, c, 10, 0.5); c.mv = 0; B.scram(s);
  let Pmax = 0, Lmin = 99, Lmax = 0, Tf = 999;
  run(s, k, c, 1800, 0.5, x => { Pmax = Math.max(Pmax, x.d.P); Lmin = Math.min(Lmin, x.d.level); Lmax = Math.max(Lmax, x.d.level); Tf = Math.min(Tf, x.f.Tfw); return true; });
  assert.equal(B.fault(s), '');
  assert.ok(Pmax > B.SRV[0].p, `lowest bank lifts: ${MPa(Pmax)}`);
  assert.ok(Pmax < B.SRV[4].p, `only the lowest bank is needed: ${MPa(Pmax)}`);
  assert.ok(Lmin > 12, `level min ${Lmin}`);
  assert.ok(Lmax < B.Z.MSL - 0.3, `level max ${Lmax}`);
  near(Tf, B.THW, 0.5, 'no steam to the turbine, no extraction: feed cools to the hotwell temperature');
  near(s.f.hw, 0.5, 0.03, 'makeup keeps the hotwell');
});

test('hotwell: drains without makeup when steam is lost to the pool; pump loses suction when it is empty; overfills', () => {
  // isolation at 200 MW with makeup in M at zero: the relief steam never returns
  const hb = B.heatBalance(200e6, P0, 1), s = B.init({ P: P0, L: 14.3, gen: hb.W, mv: 0, fv: B.fwValve(hb.W, 1, P0), tfw: hb.Tfw - 273.15 });
  const k = K.init(), c = { ...RATED_C(), heat: 200, mv: 0, mk: 0, auto: { fv: true, mv: false, mk: false } };
  run(s, k, c, 1800, 1);
  assert.equal(B.fault(s), '');
  assert.ok(s.f.hw < 0.45, `hotwell drains: ${s.f.hw}`);
  assert.equal(s.f.ret, 0, 'no condensate return while isolated');
  // nearly empty hotwell: the pump delivers less than its curve
  const e = B.init({ P: P0, L: 14.3, hw: 0.02, n: 1 });
  B.step(e, { heat: 0, n: 1, fv: 0, ext: 0, mk: 0, mv: 0, srv: [] }, 0.5); e.fv = 1;
  B.step(e, { heat: 0, n: 1, fv: 1, ext: 0, mk: 0, mv: 0, srv: [] }, 0.5);
  near(e.f.fw, 0.4 * B.fwFlow(1, 1, P0), 0.05 * B.fwFlow(1, 1, P0), 'suction-limited feed');
  // open steam valve with no feed: the condensate fills the hotwell until the condenser would flood
  const o = B.init({ P: P0, L: 14.3, hw: 0.95 });
  runPlant(o, { heat: 0, ...NOFEED, mv: 1, srv: [] }, 300, 0.5);
  assert.equal(B.fault(o), 'hotwell');
});

test('opening the steam valve depressurises: the water flashes and the level swells above the collapsed level', () => {
  const s = B.init({ P: 7e6, L: 14.0 }), L0 = s.d.level;
  runPlant(s, { heat: 20e6, ...NOFEED, mv: 0.3, srv: [] }, 2, 0.25);
  assert.ok(s.d.level > L0 + 0.1, `level swells at first: ${s.d.level} from ${L0}`);
  runPlant(s, { heat: 20e6, ...NOFEED, mv: 0.3, srv: [] }, 18, 0.25);
  assert.ok(s.d.P < 6.8e6, `pressure falls: ${MPa(s.d.P)}`);
  assert.ok(s.d.level - s.d.collapsed > 0.2, `swell over collapsed level: ${s.d.level - s.d.collapsed} m`);
  assert.ok(s.d.Vb > 5, `voids from flashing: ${s.d.Vb} m³`);
});

test('cold feedwater condenses steam: pressure falls, voids vanish, water subcooled only slightly', () => {
  const s = B.init({ P: 7e6, L: 14.3, fv: B.fwValve(100, 1, 7e6) });
  runPlant(s, { heat: 0, n: 1, fv: B.fwValve(100, 1, 7e6), ext: 0, mk: 0, mv: 0, srv: [] }, 60, 0.5);
  assert.ok(s.f.fw > 100, `about 100 kg/s of 46 °C feed (more as the pressure falls): ${s.f.fw}`);
  assert.ok(s.d.P < 6.75e6, `pressure falls: ${MPa(s.d.P)}`);
  assert.ok(s.d.sub > 0 && s.d.sub < 1, `subcooling ${s.d.sub} K`);
  near(s.d.Vb, 0, 1e-6, 'no voids');
});

test('SRV springs: each valve opens above its own set pressure and recloses its blowdown lower', () => {
  // realistic figures: 1090/1100/1110 psig, 800k/808k/815k lb/h
  near(B.SRV[0].p, (1090 + 14.696) * 6894.757, 1, 'lowest set pressure');
  near(B.SRV[10].w, 815e3 * 1.25998e-4, 0.01, 'highest-bank capacity');
  // the valves act on the start-of-step pressure: record it before each step
  const s = B.init({ P: 7.5e6, L: 14.3 });let first = 0, n = 0, Pb = s.d.P;
  runPlant(s, { heat: 400e6, ...NOFEED, mv: 0, srv: [] }, 200, 0.25, x => {
    if (!first && x.f.nsrv) first = Pb;
    Pb = x.d.P; n = Math.max(n, x.f.nsrv); return true; });
  assert.ok(first > B.SRV[0].p && first < B.SRV[0].p + 0.02e6, `first lift just above the lowest set pressure: ${MPa(first)}`);
  assert.equal(n, 4, 'the lowest bank (four valves) carries 400 MW');
  // flow of one open valve at its set pressure is its rated capacity
  near(B.wSRV(0, B.SRV[0].p), B.SRV[0].w, 1e-9, 'rated capacity at set pressure');
  // reclose: from just above the lowest set pressure with no heat, the bank lifts, blows down and reseats
  // BD below its set pressure (valves act on the start-of-step pressure, recorded before each step)
  const r = B.init({ P: B.SRV[0].p + 0.005e6, L: 14.3 });let Pshut = 0, was = false;Pb = r.d.P;
  runPlant(r, { heat: 0, ...NOFEED, mv: 0, srv: [] }, 120, 0.25, x => {
    const o = x.f.nsrv > 0; if (!Pshut && was && !o) Pshut = Pb; was = o; Pb = x.d.P; return true; });
  assert.ok(Math.abs(Pshut - (B.SRV[0].p - B.BD)) < 0.01e6, `reseat at set − blowdown: ${MPa(Pshut)}`);
  assert.equal(r.f.nsrv, 0, 'stays shut');
});

test('SRV opened by hand stays open below its set pressure, and only that valve', () => {
  const s = B.init({ P: 7e6, L: 14.3 }), srv = []; srv[6] = true;
  runPlant(s, { heat: 20e6, ...NOFEED, mv: 0, srv }, 60, 0.5);
  assert.deepEqual(s.f.open.map((o, i) => o ? i : -1).filter(i => i >= 0), [6]);
  near(s.f.wsrv, B.wSRV(6, s.d.P), 0.5, 'flow of valve 7 (at the start-of-step pressure)');
  assert.ok(s.d.P < 6.9e6, `depressurising: ${MPa(s.d.P)}`);
});

test('pressure regulator and three-element feedwater recover from a load step (dt 0.25 and 2 s)', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 60, dt);
    c.heat = 2000; run(s, k, c, 1200, dt);
    near(MPa(s.d.P), 7.0, 0.005, `P back to setpoint dt=${dt}`);
    near(s.d.level, 14.3, 0.05, `level back to setpoint dt=${dt}`);
    near(s.f.fw, s.f.wst, 5, `feed matches steam dt=${dt}`);
    near(s.f.hw, 0.5, 0.01, `hotwell at its setpoint dt=${dt}`);
    assert.ok(s.f.Tfw - 273.15 < 200, `less steam, less extraction, colder feed: ${s.f.Tfw - 273.15} dt=${dt}`);
    c.pset = 6.5e6; run(s, k, c, 900, dt);
    near(MPa(s.d.P), 6.5, 0.005, `new setpoint dt=${dt}`);
  }
});

test('in M the demand is the operator value plus the proportional correction; switching to A starts there', () => {
  const s = rated(), k = K.init(), c = { ...RATED_C(), auto: { fv: false, mv: false, mk: false } };
  run(s, k, c, 30, 0.5);
  // balanced: the notch sits on the slider
  near(k.dem.mv, c.mv / 100, 0.01, 'valve demand at balance');
  near(k.dem.fv, c.fv / 100, 0.002, 'feed valve demand at balance');
  near(k.dem.mk, 0, 2, 'makeup demand at balance');
  // pressure 0.02 MPa over the setpoint and level 0.2 m low: the demand moves off the slider the right way
  c.pset = 6.98e6; c.lset = 14.5; K.demands(k, s, c, 0.5);
  near(k.dem.mv, c.mv / 100 + K.KP * (s.d.P - c.pset), 0.005, 'valve demand opens by KP·e');
  near(k.dem.fw, s.f.fw + K.KL * (c.lset - s.d.level), 1e-6, 'flow demand: present flow + KL·e');
  near(k.dem.fv, c.fv / 100 + K.KX * (k.dem.fw - s.f.fw), 1e-9, 'feed valve demand opens by KX·flow error');
  c.auto = AUTO(); const u = K.update(k, s, c, 0.5);
  near(u.mv, k.dem.mv, 1e-12, 'A starts from the indicated valve demand');
  near(u.fv, k.dem.fv, 1e-9, 'A starts from the indicated feed valve demand');
  // valve shut by hand with pressure below the setpoint: demand stays shut (was a clamp artefact before)
  const s2 = B.init({ P: 6.5e6, L: 14.3 }), k2 = K.init(), c2 = { ...RATED_C(), heat: 20, mv: 0, n: 0, fv: 0, auto: { fv: false, mv: false, mk: false } };
  run(s2, k2, c2, 10, 0.5);
  assert.equal(k2.dem.mv, 0, 'valve demand shut');
  // above the setpoint with the valve shut by hand: the demand opens in proportion, not to an arbitrary limit
  c2.pset = 6.45e6; K.demands(k2, s2, c2, 0.5);
  near(k2.dem.mv, K.KP * (s2.d.P - c2.pset), 1e-3, 'valve demand from shut');
});

test('startup heat-up with the valve shut: pressure and level rise at the expected rate', () => {
  const s = B.init({ P: 0.1e6, L: 11.5 }), T0 = s.d.T, L0 = s.d.level;
  runPlant(s, { heat: 30e6, ...NOFEED, mv: 0, srv: [] }, 3600, 2);
  const rate = (s.d.T - T0);
  assert.ok(rate > 60 && rate < 110, `heat-up ${rate} K/h at 30 MW`);
  assert.ok(s.d.level > L0 + 0.8, `thermal expansion raises the level: ${s.d.level}`);
  assert.ok(s.d.P > 0.6e6, `pressurised: ${MPa(s.d.P)}`);
});

test('faults: core uncovered without feedwater, steam lines flooded with too much', () => {
  const s = rated();
  runPlant(s, { ...RATED_U, fv: 0 }, 2000, 1);
  assert.equal(B.fault(s), 'uncovered');
  const s2 = B.init({ P: 7e6, L: 14.3 });
  runPlant(s2, { heat: 20e6, n: 1, fv: 0.5, ext: 0, mk: 0, mv: 0.01, srv: [] }, 2000, 1);
  assert.equal(B.fault(s2), 'flooded');
});
