// BWR core tests: point kinetics, feedback, fuel, decay heat, control rod groups. Run: node --test tests/
import test from 'node:test';
import assert from 'node:assert/strict';
import { B, K, near, runPlant, run, rated, RATED_U, RATED_C, Q0, P0, QL0, ISO, rodsC, ALLIN } from './bwr-helpers.mjs';
import * as CORE from '../bwr-core.js';
const WCR0 = B.WCR;
// rated command and settings with the kinetics in charge (no fixed heat), rods where init made it critical
const KU = s => { const u = { ...RATED_U, rods: s.core.x.slice(), crd: true }; delete u.heat; return u; };
const KC = s => { const c = { ...RATED_C(), rods: rodsC(s), crd: true }; delete c.heat; return c; };
// hot zero power: the critical rod pattern along the withdrawal sequence (positions 00–48)
const HZP = B.steady(0, WCR0, 0, P0), XC = HZP.x.map(v => v * B.NPOS);
const STILL = { n: 0, fv: 0, ext: 0, mk: 0, cv: 0, bp: 0, srv: [], crd: true };

test('rated: critical with all rods out, fuel near 600 °C; steady at dt 0.25 and 2 s with the controllers in A', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), c = KC(s), k = K.init(c);
    assert.ok(s.core.x.every(x => x > 1 - 1e-9), `all rods out ${s.core.x}`);
    near(s.core.Tf - 273.15, 600, 30, 'fuel temperature');
    near(s.f.ac, 0.38, 0.05, 'core-average void');
    run(s, k, c, 900, dt);
    // all rods out is critical at the calibration point; init's heat balance (vessel losses, the downcomer's
    // own subcooling) differs a little, so the power settles 0.3 % low
    near(s.f.Pth / Q0, 1, 0.005, `power dt=${dt}`);
    near(s.d.P / 1e6, 7.0, 0.002, `pressure dt=${dt}`);
    near(s.d.level, 14.3, 0.02, `level dt=${dt}`);
    assert.equal(B.fault(s), '');
  }
});

test('decay heat groups: 6.5 % of the total after long operation, then the (1 + t)^−0.2 shape', () => {
  near(B.DSUM / (1 + B.DSUM), 0.065, 0.001, 'equilibrium share');
  for (const t of [1, 10, 100, 1000, 1e4, 1e5]) {
    const m = B.DF.reduce((a, f, j) => a + f * Math.exp(-B.DL[j] * t), 0) / (1 + B.DSUM);
    near(m / (0.065 * (1 + t) ** -0.2), 1, 0.03, `t=${t}`);
  }
});

test('scram from rated: rods in within 3 s, heat follows the decay curve', () => {
  // insertion: nothing for the pilot valve delay, then a steady stroke; 90 % in within 3 s (BWR/4 limit)
  { const s = rated(), u = { ...KU(s), cv: 0.05 }; B.scram(s); runPlant(s, u, 0.15, 0.05);
    assert.ok(s.core.x.every(x => x > 1 - 1e-9), `pilot valve delay ${s.core.x}`);
    runPlant(s, u, 2.5, 0.05); assert.ok(s.core.x.every(x => x < 0.1), `90 % in by 2.65 s: ${s.core.x}`); }
  for (const dt of [0.25, 2]) {
    const s = rated(), u = { ...KU(s), cv: 0.05 }; B.scram(s);
    runPlant(s, u, 4, dt === 2 ? 2 : 0.25);
    assert.deepEqual(s.core.x, ALLIN(), 'rods fully in');
    runPlant(s, u, 6, dt);
    const r10 = s.f.Pth / Q0;
    assert.ok(r10 > 0.6 * B.decayFrac(10) && r10 < 1.1 * B.decayFrac(10), `10 s: ${r10} vs ${B.decayFrac(10)} dt=${dt}`);
    runPlant(s, u, 50, dt);
    near(s.f.Pth / Q0 / B.decayFrac(60), 1, 0.15, `60 s dt=${dt}`);
    assert.ok(s.f.nf < 0.01 * Q0, `fission power small: ${s.f.nf / Q0}`);
  }
});

test('closing the steam valve at rated with no scram: the pressure rise collapses voids and power spikes', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), a0 = s.f.ac; let pk = 0, amin = 1;
    runPlant(s, { ...KU(s), cv: 0 }, 12, dt, x => { pk = Math.max(pk, x.f.Pth); amin = Math.min(amin, x.f.ac); return true; });
    assert.ok(pk > 1.08 * Q0 && pk < 2 * Q0, `power peak ${pk / Q0} dt=${dt}`);
    assert.ok(amin < a0 - 0.01, `fewer voids ${amin} from ${a0}`);
  }
});

test('colder feedwater raises the power (inlet subcooling, fewer voids); the controllers follow', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), c = KC(s), k = K.init(c);
    run(s, k, c, 60, dt); c.ext = 20; c.exl = 100; run(s, k, c, 600, dt); // HP extraction throttled
    assert.ok(s.f.Tfw - 273.15 < 190, `feed colder ${s.f.Tfw - 273.15}`);
    // about 34 K colder: +11 % (a real BWR/4 gains roughly 0.3 % per K of feedwater temperature)
    assert.ok(s.f.Pth > 1.06 * Q0 && s.f.Pth < 1.16 * Q0, `power up ${s.f.Pth / Q0} dt=${dt}`);
    assert.equal(B.fault(s), '');
  }
});

test('axial shape: bottom-skewed at rated with all rods out; at hot zero power the rods in from the bottom push it up', () => {
  const C = B.RATED, pk = B.peak(C.s);
  near(pk.pk, 1.5, 0.08, 'rated axial peaking'); assert.ok(pk.zpk < 0.5, `rated peak in the lower half: ${pk.zpk}`);
  const s = rated(); near(s.core.pk, pk.pk, 0.01, 'init lands on the rated shape'); near(s.f.pk, s.core.pk, 1e-12, 'reported');
  near(B.steady(Q0, WCR0, C.dh, P0).rho, 0, 1e-9, 'rated, all rods out: critical');
  // hot zero power: groups 1 and 2 out, critical on group 3, group 4 in; the flux sits above the rod tips
  assert.deepEqual(HZP.x.slice(0, 2), [1, 1], 'deep groups out'); assert.equal(HZP.x[3], 0, 'power group in');
  assert.ok(XC[2] > 10 && XC[2] < 30, `critical on group 3 at ${XC[2]}`);
  const h = B.peak(HZP.s); assert.ok(h.zpk > 0.6 && h.pk < 2.8, `no voids, rods in from below: the flux up high ${h.pk} at ${h.zpk}`);
  // the same rods but group 3 two notches further in: the shape moves further up
  const lo = B.steady(0, WCR0, 0, P0, HZP.x.map((x, g) => g === 2 ? x - 4 / B.NPOS : x));
  assert.ok(lo.rho < 0 && B.peak(lo.s).zpk >= h.zpk, `inserted: subcritical ${lo.rho}, shape up ${B.peak(lo.s).zpk}`);
});

test('rod worth: about −12 % all in at hot zero power, about 0.2 $ a notch of group 3 near critical, +6 % all out', () => {
  const f = HZP.v, fb = B.localFb(f.a, f.hs, B.TREF, B.TREF, HZP.s);
  const rho = x => CORE.shapeAt(fb, x, [...HZP.s]);
  near(rho(ALLIN()), -0.12, 0.02, 'all rods in');
  near(rho([1, 1, 1, 1]), 0.062, 0.01, 'all rods out: the rated feedback');
  const up = HZP.x.map((x, g) => g === 2 ? x + B.NOTCH / B.NPOS : x), w = (rho(up) - rho(HZP.x)) / B.BETA;
  assert.ok(w > 0.1 && w < 0.35, `notch worth ${w} $`);
  // rods enter from the bottom: a notch of group 4 in from fully out (the bottom 6 in) is worth more at
  // rated, where the power peaks low, than the top notch of travel
  const R = B.RATED, fr = B.localFb(R.v.a, R.v.hs, B.fuelT(Q0, B.TREF), B.TREF, R.s), rr = x => CORE.shapeAt(fr, x, [...R.s]);
  const out = [1, 1, 1, 1], bot = rr([1, 1, 1, 1 - B.NOTCH / B.NPOS]), top = rr([1, 1, 1, B.NOTCH / B.NPOS]) - rr([1, 1, 1, 0]);
  assert.ok(rr(out) - bot > top, `bottom notch ${rr(out) - bot} vs top notch ${top}`);
});

test('rods at rated: group 4 half in takes power off and moves the peak up; a few notches in hardly change the power', () => {
  for (const dt of [0.5, 2]) {
    const s = rated(), c = KC(s), k = K.init(c); const z0 = s.f.zpk;
    run(s, k, c, 30, dt); c.rods = [48, 48, 48, 24]; run(s, k, c, 900, dt);
    near(s.core.x[3] * B.NPOS, 24, 1e-6, 'group 4 at 24');
    assert.ok(s.f.Pth < 0.97 * Q0 && s.f.Pth > 0.85 * Q0, `power down ${s.f.Pth / Q0} dt=${dt}`);
    assert.ok(s.f.zpk > z0 + 0.1, `peak up from ${z0} to ${s.f.zpk} dt=${dt}`);
  }
  // shallow (position 40, the bottom 8 % of the fuel): the power moves up, the boiling starts higher and the
  // voids shrink, which nearly cancels the rods' worth (a known shortcoming: a real core loses a little power)
  const s = rated(), c = KC(s), k = K.init(c); c.rods = [48, 48, 48, 40]; run(s, k, c, 600, 1);
  assert.ok(Math.abs(s.f.Pth / Q0 - 1) < 0.03, `shallow rods: power ${s.f.Pth / Q0}`);
});

test('loss of feedwater heating (HP drain valve fails shut) with protection on: about +17 %, no scram', () => {
  // generic BWR/4 safety analyses: about 55 K colder feed raises the power some 15–20 % and stays under the
  // 120 % APRM trip. Before the axial shape the point model gained about +33 % and scrammed.
  for (const dt of [0.5, 2]) {
    const s = B.init({ P: P0, L: 14.3, gen: B.WR, cv: B.cvPos(B.WR, P0), fv: B.fwValve(B.WR, 1, P0), n: 1, tfw: 216, rps: true }), c = KC(s), k = K.init(c);
    run(s, k, c, 30, dt); B.fail(s, 'hdv', true); let pmax = 0;
    run(s, k, c, 900, dt, x => { pmax = Math.max(pmax, x.f.nf * (1 + B.DSUM)); return true; });
    assert.ok(s.f.Tfw - 273.15 < 170, `feed colder ${s.f.Tfw - 273.15} dt=${dt}`);
    assert.ok(s.f.Pth > 1.12 * Q0 && s.f.Pth < 1.2 * Q0, `power ${s.f.Pth / Q0} dt=${dt}`);
    assert.ok(pmax < 1.2 * Q0, `neutron power peak ${pmax / Q0} under the APRM trip dt=${dt}`);
    assert.ok(!s.scram, `no scram dt=${dt}`);
    assert.ok(s.f.pk > B.peak(B.RATED.s).pk + 0.03 && s.f.zpk <= B.peak(B.RATED.s).zpk, `the shape moves down: ${s.f.pk} at ${s.f.zpk}`);
  }
});

test('startup: subcritical with the rods in; withdrawing group 3 multiplies the source, critical where steady() says, positive period', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: P0, L: 14.3, q: 0 });
    // groups 1 and 2 out (192 s each at a notch per 4 s), group 3 two notches short of critical; then one
    // notch past it
    assert.deepEqual(s.core.x, ALLIN(), 'rods in by default with no heat');
    assert.ok(s.f.nf > 0 && s.f.nf < 100, `source level ${s.f.nf} W`);
    assert.ok(s.f.rho < -0.1, `shut down ${s.f.rho}`);
    const u = { ...STILL, rods: [1, 1, (XC[2] - 4) / B.NPOS, 0] };
    runPlant(s, u, 600, dt);
    assert.deepEqual(s.core.x.slice(0, 2), [1, 1], 'deep groups out');
    assert.ok(s.f.rho < 0 && s.f.nf < 1e4, `still subcritical: ${s.f.rho}, ${s.f.nf} W`);
    u.rods = [1, 1, (XC[2] + B.NOTCH) / B.NPOS, 0]; let per = Infinity;
    runPlant(s, u, 600, dt, x => { per = x.f.per; return x.f.nf < 1e6; });
    assert.ok(s.f.nf >= 1e6, `power rising past 1 MW: ${s.f.nf}`);
    // a notch past critical is about 0.2 $: a stable period of about half a minute
    assert.ok(per > 15 && per < 60, `positive period ${per} s dt=${dt}`);
  }
});

test('rods move a notch per 4 s with the CRD pump running, not at all with it stopped; the scram works either way', () => {
  const s = B.init({ P: P0, L: 14.3, q: 0, rods: [1, 1, 0, 0] });
  runPlant(s, { ...STILL, rods: [1, 1, 0.5, 0], crd: false }, 40, 0.5);
  assert.equal(s.core.x[2], 0, 'no drive water: no motion');
  runPlant(s, { ...STILL, rods: [1, 1, 0.5, 0] }, 40, 0.5);
  near(s.core.x[2] * B.NPOS, 20, 1e-9, '10 notches in 40 s');
  B.scram(s); runPlant(s, { ...STILL, rods: [1, 1, 0.5, 0], crd: false }, 4, 0.25);
  assert.deepEqual(s.core.x, ALLIN(), 'scram on the accumulators');
});

test('CRD pump: 3.8 kg/s from the storage tank into the vessel, mass conserved', () => {
  const s = B.init({ P: P0, L: 14.3, q: 0 }), M0 = s.M + s.Mcst;
  runPlant(s, { ...STILL, ...ISO, rods: ALLIN() }, 100, 1);
  near(s.f.wcd, B.WCRD, 1e-9, 'flow'); near(s.M + s.Mcst, M0, 1e-3, 'vessel + tank');
});

test('rods pulled far past critical at once: prompt critical, the model stops', () => {
  const s = B.init({ P: P0, L: 14.3, q: 0, rods: HZP.x }); s.core.x = [1, 1, 1, 0.2];
  runPlant(s, { ...STILL, rods: s.core.x.slice() }, 5, 0.25);
  assert.equal(B.fault(s), 'prompt');
});

test('hot standby: critical at 1.5 % power with the pressure regulator in A; steady at dt 0.25 and 2 s', () => {
  for (const dt of [0.25, 2]) {
    const q = 0.015 * Q0, hb = B.heatBalance(q, P0, 1), s = B.init({ P: P0, L: 14.3, gen: hb.W, cv: B.cvPos(hb.W, P0), fv: B.fwValve(hb.W, 1, P0), n: 1, tfw: hb.Tfw - 273.15 });
    near(s.f.Pth, q + QL0, 1e-3 * q, 'init power: the heat balance plus the steady loss through the vessel steel');
    assert.deepEqual(s.core.x.slice(0, 2), [1, 1], 'deep groups out'); assert.equal(s.core.x[3], 0, 'power group in');
    assert.ok(s.core.x[2] * B.NPOS > XC[2] && s.core.x[2] * B.NPOS < XC[2] + 12, `critical on group 3 a little above hot zero power: ${s.core.x[2] * B.NPOS}`);
    const c = { ...KC(s), cv: B.cvPos(hb.W, P0) * 100, fv: s.fv * 100 }, k = K.init(c);
    // with the rods fixed the feed flow and the power swing together (cold feed, about a 240 s period,
    // see docs/bwr.md): the mean holds, the swing stays bounded
    let sum = 0, n = 0, lo = Infinity, hi = 0;
    run(s, k, c, 1200, dt, x => { if (x.t > 720) { const p = x.f.Pth / q; sum += p; n++; lo = Math.min(lo, p); hi = Math.max(hi, p); } return true; });
    near(sum / n, 1, 0.08, `mean power holds dt=${dt}`);
    assert.ok(lo > 0.75 && hi < 1.3, `swing bounded: ${lo}–${hi} dt=${dt}`);
    near(s.d.P / 1e6, 7.0, 0.01, `pressure dt=${dt}`);
  }
});
