// BWR core tests: point kinetics, feedback, fuel, decay heat, control rod lever. Run: node --test tests/
import test from 'node:test';
import assert from 'node:assert/strict';
import { B, K, near, runPlant, run, rated, RATED_U, RATED_C, Q0, P0, QL0 } from './bwr-helpers.mjs';
// rated command and settings with the kinetics in charge (no fixed heat), rods where init made it critical
const KU = s => { const u = { ...RATED_U, rod: s.core.x }; delete u.heat; return u; };
const KC = s => { const c = { ...RATED_C(), rod: s.core.x * 100 }; delete c.heat; return c; };

test('rated: critical with the rods near 80 %, fuel near 600 °C; steady at dt 0.25 and 2 s with the controllers in A', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(), c = KC(s), k = K.init(c);
    near(s.core.x, 0.8, 0.01, 'rods withdrawn');
    near(s.core.Tf - 273.15, 600, 30, 'fuel temperature');
    near(s.f.ac, 0.38, 0.05, 'core-average void');
    run(s, k, c, 900, dt);
    near(s.f.Pth / Q0, 1, 0.002, `power dt=${dt}`);
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
  for (const dt of [0.25, 2]) {
    const s = rated(), u = { ...KU(s), cv: 0.05 }; B.scram(s);
    runPlant(s, u, 4, dt === 2 ? 2 : 0.25);
    assert.equal(s.core.x, 0, 'rods fully in');
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
    assert.ok(s.f.Tfw - 273.15 < 180, `feed colder ${s.f.Tfw - 273.15}`);
    assert.ok(s.f.Pth > 1.08 * Q0 && s.f.Pth < 1.45 * Q0, `power up ${s.f.Pth / Q0} dt=${dt}`);
    assert.equal(B.fault(s), '');
  }
});

test('startup: subcritical with the rods in; withdrawing them multiplies the source, critical near 60 %, positive period', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: P0, L: 14.3, q: 0, rod: 0 });
    assert.ok(s.f.nf > 0 && s.f.nf < 100, `source level ${s.f.nf} W`);
    assert.ok(s.f.rho < -0.1, `shut down ${s.f.rho}`);
    const u = { n: 0, fv: 0, ext: 0, mk: 0, cv: 0, bp: 0, srv: [], rod: 0.58 };
    runPlant(s, u, 600, dt);
    assert.ok(s.f.nf < 1e4, `still subcritical at 58 %: ${s.f.nf}`);
    u.rod = 0.605; let per = Infinity;
    runPlant(s, u, 200, dt, x => { if (x.f.per > 0) per = Math.min(per, x.f.per); return x.f.nf < 1e6; });
    assert.ok(s.f.nf >= 1e6, `power rising past 1 MW: ${s.f.nf}`);
    assert.ok(per > 2 && per < 200, `positive period ${per} s dt=${dt}`);
  }
});

test('rods pulled far past critical at once: prompt critical, the model stops', () => {
  const s = B.init({ P: P0, L: 14.3, q: 0, rod: 0.6 }); s.core.x = 0.66;
  runPlant(s, { n: 0, fv: 0, ext: 0, mk: 0, cv: 0, bp: 0, srv: [], rod: 0.66 }, 5, 0.25);
  assert.equal(B.fault(s), 'prompt');
});

test('hot standby: critical at 1.5 % power with the pressure regulator in A; steady at dt 0.25 and 2 s', () => {
  for (const dt of [0.25, 2]) {
    const q = 0.015 * Q0, hb = B.heatBalance(q, P0, 1), s = B.init({ P: P0, L: 14.3, gen: hb.W, cv: B.cvPos(hb.W, P0), fv: B.fwValve(hb.W, 1, P0), n: 1, tfw: hb.Tfw - 273.15 });
    near(s.f.Pth, q + QL0, 1e-3 * q, 'init power: the heat balance plus the steady loss through the vessel steel');
    assert.ok(s.core.x > 0.6 && s.core.x < 0.63, `rods ${s.core.x}`);
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
