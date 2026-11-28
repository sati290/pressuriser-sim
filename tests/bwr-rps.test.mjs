// Reactor protection (bwr-rps.js): trips, latching, bypasses, reset, and the plant acting on them.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as R from '../bwr-rps.js';
import * as BS from '../bwr-sim.js';
import { K, near, MPa, P0, W0, FV0, CV0, RATED_C } from './bwr-helpers.mjs';

// signals at rated, nothing tripping
const OK = () => ({ P: 7e6, lin: 35, aprm: 1, ms: 1, sv: 1, fsp: 1, Pl: 6.7e6, Pc: 7.3e3, wst: 1 });
const ratedP = (o = {}) => BS.init({ P: P0, L: 14.3, gen: W0, cv: CV0, fv: FV0, n: 1, tfw: 216, ...o });
const runC = (s, k, c, seconds, dt = 1) => { for (let i = 0; i < Math.round(seconds / dt); i++) { BS.step(s, K.update(k, s, c, dt), dt); if (BS.fault(s)) break; } return s; };

test('setpoints: typical BWR/4 values in SI', () => {
  near(MPa(R.SP.P), 7.307, 0.002, 'high pressure 1045 psig');
  near(MPa(R.SP.plow), 5.962, 0.002, 'low steam line pressure 850 psig');
  near(R.SP.vacScram / 1e3, 23.4, 0.1, 'low vacuum scram 23 inHg vacuum');
  near(R.SP.vacIso / 1e3, 77.6, 0.1, 'low vacuum isolation 7 inHg vacuum');
  assert.ok(R.SP.L8 > R.SP.L3 && R.SP.L3 > R.SP.L2, 'L8 > L3 > L2');
});

test('each trip fires on its own signal, latches, and is logged once', () => {
  const cases = [['P', { P: 7.4e6 }, 'scram'], ['L3', { lin: 10 }, 'scram'], ['aprm', { aprm: 1.25 }, 'scram'],
    ['msc', { ms: 0.85 }, 'scram'], ['tsv', { sv: 0.5 }, 'scram'], ['vac', { Pc: 30e3 }, 'scram'],
    ['L2', { lin: -40 }, 'iso'], ['plow', { Pl: 5.5e6 }, 'iso'], ['wHigh', { wst: 1.5 }, 'iso'],
    ['vacIso', { Pc: 80e3 }, 'iso'], ['L8', { lin: 60 }, 'turb'], ['vacT', { Pc: 26e3 }, 'turb'], ['L8f', { lin: 60 }, 'fwp']];
  for (const [k, sig, g] of cases) {
    const r = R.rpsInit('run');
    assert.equal(R.rpsCheck(r, OK(), 0)[g], false, `${k}: quiet at rated`);
    assert.equal(R.rpsCheck(r, { ...OK(), ...sig }, 1)[g], true, `${k}: fires`);
    assert.ok(r.lat[k], `${k}: latched`);
    assert.equal(R.rpsCheck(r, OK(), 2)[g], true, `${k}: stays in after the signal clears`);
    assert.equal(r.log.filter(e => e.text === R.TRIPS.find(t => t.k === k).t).length, 1, `${k}: logged once`);
  }
});

test('bypasses: mode switch and low power', () => {
  const su = R.rpsInit('startup');
  const g = R.rpsCheck(su, { ...OK(), aprm: 0.1, fsp: 0.1, ms: 0, Pl: 1e6, Pc: 90e3 }, 0);
  assert.equal(g.scram, false, 'Startup: MSIV closure, low vacuum scram bypassed');
  assert.equal(g.iso, false, 'Startup: low steam line pressure and low vacuum isolation bypassed');
  assert.equal(g.turb, true, 'the turbine low vacuum trip is never bypassed');
  assert.equal(R.rpsCheck(su, { ...OK(), aprm: 0.16, fsp: 0.1 }, 1).scram, true, 'Startup: APRM trips at 15 %');
  const run = R.rpsInit('run');
  assert.equal(R.rpsCheck(run, { ...OK(), aprm: 0.25, sv: 0, fsp: 0.25 }, 0).scram, false, 'stop valve closure bypassed below 30 % first-stage pressure');
  assert.equal(R.rpsCheck(run, { ...OK(), aprm: 1.15 }, 0).scram, false, 'Run: APRM below 120 % does not trip');
  const off = R.rpsInit('run', true);
  assert.equal(R.rpsCheck(off, { ...OK(), P: 9e6, lin: -100 }, 0).scram, false, 'protection off (tests only)');
});

test('reset clears only the trips whose signal has cleared', () => {
  const r = R.rpsInit('run');
  R.rpsCheck(r, { ...OK(), P: 7.5e6, lin: 5 }, 0);
  let left = R.rpsReset(r, { ...OK(), lin: 5 }, 1);
  assert.deepEqual(left, ['Reactor low level (L3)'], 'level still low: L3 stays in');
  assert.ok(!r.lat.P && r.lat.L3, 'pressure trip reset');
  left = R.rpsReset(r, OK(), 2);
  assert.equal(left.length, 0); assert.equal(R.groups(r).scram, false, 'all clear');
  R.rpsMode(r, 'shutdown', 3); assert.equal(R.rpsCheck(r, OK(), 3).scram, true, 'mode switch to Shutdown scrams');
  R.rpsMode(r, 'startup', 4); R.rpsReset(r, { ...OK(), aprm: 0.05 }, 5); assert.equal(R.groups(r).scram, false, 'resets once out of Shutdown');
});

test('rated with protection and everything in A: steady, nothing trips (dt 0.25 and 2 s)', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = ratedP(), k = K.init(c);
    runC(s, k, c, 600, dt);
    assert.equal(s.rps.log.length, 0, `no event dt=${dt}: ${JSON.stringify(s.rps.log)}`);
    assert.equal(s.scram, null); near(MPa(s.d.P), 7, 0.003, `P dt=${dt}`);
  }
});

test('turbine trip at rated: the stop valve closure scrams in the same step, the bypass and SRVs take the steam', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = ratedP(), k = K.init(c);
    runC(s, k, c, 10, dt); c.sv = false;
    BS.step(s, K.update(k, s, c, dt), dt);
    assert.ok(s.scram && s.rps.lat.tsv, `scrammed on the stop valve dt=${dt}`);
    let pmax = 0; for (let i = 0; i < 120 / dt; i++) { BS.step(s, K.update(k, s, c, dt), dt); pmax = Math.max(pmax, s.d.P); }
    assert.ok(pmax < 7.75e6, `pressure peak held below the upper SRV banks: ${MPa(pmax)} dt=${dt}`);
    assert.equal(BS.fault(s), '', `no fault dt=${dt}`);
  }
});

test('MSIV closure at rated: scram on valve position before the valves are shut; in Startup only the isolation', () => {
  const c = RATED_C(), s = ratedP(), k = K.init(c);
  runC(s, k, c, 10); c.ms1 = false; c.ms2 = false;
  BS.step(s, K.update(k, s, c, 1), 1);
  assert.ok(s.scram && s.rps.lat.msc, 'scram at 75 % open');
  assert.ok(s.ms[0] > 0.5, 'the valves are still travelling');
  runC(s, k, c, 600);
  assert.equal(BS.fault(s), '', 'decay heat to the SRVs: no fault');
  assert.ok(s.d.P < 7.75e6, `SRVs hold the pressure: ${MPa(s.d.P)}`);
});

test('high pressure scram when the steam has nowhere to go (Startup, 6 % power, valves shut by hand)', () => {
  const W = 200e6 / 2.6e6, s = BS.init({ P: P0, L: 14.3, gen: W, cv: BS.cvPos(W, P0), fv: BS.fwValve(W, 1, P0), n: 1, mode: 'startup' });
  const c = { ...RATED_C(), heat: 200, cv: BS.cvPos(W, P0) * 100, fv: BS.fwValve(W, 1, P0) * 100, auto: { n: true, fv: true, mk: true, cv: false, bp: false }, ms1: false, ms2: false }, k = K.init(c);
  runC(s, k, c, 120);
  assert.ok(s.rps.lat.P, `high pressure trip: ${JSON.stringify(s.rps.log)}`);
  assert.ok(!s.rps.lat.msc, 'MSIV closure scram bypassed in Startup');
  assert.ok(s.scram, 'scrammed');
});

test('L8: high level trips the turbine and the feed pump', () => {
  const c = RATED_C(), s = ratedP(), k = K.init(c);
  c.auto.fv = false; c.fv = 100; // feed valve wide open by hand
  for (let i = 0; i < 300 && !s.rps.lat.L8; i++) BS.step(s, K.update(k, s, c, 1), 1);
  assert.ok(s.rps.lat.L8 && s.rps.lat.L8f, 'L8 tripped');
  BS.step(s, K.update(k, s, c, 1), 1);
  assert.ok(s.sv < 0.01 && s.scram, 'stop valve shut, and its closure scrammed the reactor');
  const n0 = s.n; BS.step(s, K.update(k, s, c, 1), 1);
  assert.ok(s.n < n0, 'feed pump running down');
});

test('low condenser vacuum: turbine trip, bypass blocked; reset needs the vacuum back', () => {
  const c = { ...RATED_C(), cwp: false }, s = ratedP(), k = K.init(c);
  for (let i = 0; i < 60 && !s.rps.lat.vacT; i++) BS.step(s, K.update(k, s, c, 1), 1);
  assert.ok(s.rps.lat.vacT && s.rps.lat.vac, `turbine trip and scram on low vacuum: ${JSON.stringify(s.rps.log)}`);
  assert.ok(s.scram && s.sv < 0.01, 'scrammed, stop valve shut');
  // (once the turbine is off the condenser recovers within a step or two; hold the reading low)
  s.f.Pc = 30e3; BS.reset(s); assert.ok(s.scram && s.rps.lat.vac, 'reset refused while the vacuum is low');
  // with the condenser above 7 inHg vacuum (77.6 kPa) the bypass interlock holds the bypass shut
  s.f.Pc = 90e3; BS.step(s, { ...K.update(k, s, c, 1), bp: 1 }, 1);
  assert.equal(s.bp, 0, 'bypass blocked');
});

test('reactor water cleanup reject: mass and energy go to the condenser, and in A it holds the level on a heat-up', () => {
  const s = BS.init({ P: P0, L: 14.3, rps: false });
  const m0 = s.M + s.Mh + s.cd.Ms;
  BS.step(s, { heat: 0, n: 0, fv: 0, ext: 0, mk: 0, cv: 0, bp: 0, rw: 1, srv: [] }, 1);
  near(s.f.wrw, BS.RWMAX, 1e-9, 'full flow');
  near(s.M + s.Mh + s.cd.Ms, m0, 1e-3, 'vessel + hotwell + condenser mass');
  // protected startup from 100 °C, shut down, MG set at 20 %: the heat rises to 30 MW over half an hour
  // (standing for the rod pull) with the feed valve and the reject in A and the level setpoint at 14.0 m:
  // the voids that appear at low pressure swell the level about 0.5 m, then the reject takes the swell
  const su = BS.init({ P: 0.1e6, L: 14.0, sv: 0, vac: false, mode: 'startup', mg: 0.2 });
  const c = { ...RATED_C(), heat: 0, lset: 14.0, n: 0, fv: 0, cv: 0, bp: 0, ext: 0, sv: false, sj: false, vp: true, auto: { n: true, fv: true, mk: true, cv: true, bp: true, rw: true } };
  const k = K.init(c);
  let lmax = 0; for (let i = 0; i < 3 * 3600 / 2; i++) { c.heat = Math.min(30, su.t / 60); BS.step(su, K.update(k, su, c, 2), 2); lmax = Math.max(lmax, su.d.level); }
  assert.equal(BS.fault(su), '');
  assert.equal(su.scram, null, 'no scram');
  assert.ok(su.d.P > 2e6, `heated up: ${MPa(su.d.P)} MPa`);
  assert.ok(lmax < BS.fromIn(BS.LV.L8) - 0.1 && su.f.wrw > 3, `level held by the reject: max ${lmax}, reject ${su.f.wrw}`);
  assert.ok(!su.rps.lat.L8, 'no L8 trip');
  assert.ok(su.rps.lat.vacT, 'the turbine stays tripped on low vacuum until reset');
});

test('scram at rated with the kinetics: the level setpoint setdown keeps the level off L8, so the feed pump stays in', () => {
  for (const dt of [0.25, 2]) {
    const s = ratedP(), c = { ...RATED_C(), rod: s.core.x * 100, mg: 100, wset: 100 }; delete c.heat; c.auto.mg = true; c.auto.rw = true;
    const k = K.init(c); runC(s, k, c, 10, dt); BS.manualScram(s); c.rod = 0;
    let lmax = 0; for (let i = 0; i < 2400 / dt; i++) { BS.step(s, K.update(k, s, c, dt), dt); lmax = Math.max(lmax, s.d.level); }
    assert.ok(!s.rps.lat.L8, `no L8: max ${lmax} dt=${dt}`);
    near(s.d.level, 14.3, 0.25, `back near the normal level dt=${dt}`);
    assert.equal(BS.fault(s), '');
  }
});

test('a reset of the scram keeps the level setpoint setdown ramping (no step back up)', () => {
  const s = ratedP(), c = { ...RATED_C(), rod: s.core.x * 100, mg: 100, wset: 100 }; delete c.heat; c.auto.mg = true;
  const k = K.init(c); runC(s, k, c, 10, 1); BS.manualScram(s); c.rod = 0; runC(s, k, c, 700, 1);
  const before = K.setdown(k, s); assert.ok(before > 0.3, `set down: ${before}`);
  BS.reset(s); assert.equal(s.scram, null, 'scram reset');
  runC(s, k, c, 1, 1); assert.ok(Math.abs(K.setdown(k, s) - before) < 0.01, `still ${K.setdown(k, s)} after the reset`);
  runC(s, k, c, 1300, 1); assert.equal(K.setdown(k, s), 0, 'ramped out');
});

test('no windup with the pumps off: MG set speed and feed pump speed integrals track the pumps', () => {
  const s = ratedP(), c = { ...RATED_C(), rod: s.core.x * 100, mg: 100, wset: 60 }; delete c.heat; c.auto.mg = true;
  const k = K.init(c); runC(s, k, c, 600, 0.5); const ng0 = s.f.ng;
  c.rp = false; runC(s, k, c, 120, 0.5); assert.ok(k.iW < ng0, `MG integral follows the coast-down: ${k.iW} < ${ng0}`);
  c.rp = true; let wmax = 0; for (let i = 0; i < 1200; i++) { BS.step(s, K.update(k, s, c, 0.5), 0.5); wmax = Math.max(wmax, s.f.Wc); }
  assert.ok(wmax < 0.66 * K.WCR, `core flow back to 60 % without a big overshoot: peak ${(wmax / K.WCR * 100).toFixed(1)} %`);
  const s2 = ratedP(), c2 = { ...RATED_C(), rod: s2.core.x * 100 }; delete c2.heat; const k2 = K.init(c2);
  runC(s2, k2, c2, 10, 0.5); c2.fp = false; runC(s2, k2, c2, 60, 0.5);
  assert.ok(k2.iN < 0.05, `feed pump integral follows the stopped pump: ${k2.iN}`);
});
