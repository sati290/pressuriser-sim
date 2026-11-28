// BWR condenser air removal tests (roadmap step 3). Run: node --test tests/  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, B, near, runPlant, rated, RATED_U } from './bwr-helpers.mjs';

const COLD = { heat: 0, n: 0, fv: 0, ext: 0, mk: 0, mv: 0, srv: [] };

test('rated design point: SJAE holds a few hundred Pa of air, back pressure about 7 kPa, steady', () => {
  near(B.PA0, 260, 10, 'design air partial pressure');
  for (const dt of [0.25, 2]) {
    const s = rated(), p0 = s.f.Pc;
    runPlant(s, RATED_U, 1800, dt); const p1 = s.f.Pc, pa1 = s.f.pa;
    runPlant(s, RATED_U, 1800, dt);
    assert.ok(s.f.pa > 150 && s.f.pa < 400, `air partial pressure ${s.f.pa} dt=${dt}`);
    assert.ok(s.f.Pc > 6.5e3 && s.f.Pc < 7.6e3, `back pressure ${s.f.Pc}`);
    near(s.f.Pc, p1, 5, 'steady'); near(s.f.pa, pa1, 2, 'air steady'); near(s.f.Pc, p0, 30, 'starts steady');
    near(s.f.wsj, B.WSJ, 1e-9, 'motive steam'); near(s.f.waout, s.f.wain, 1e-4, 'removal = in-leakage');
  }
});

test('vacuum pull from atmosphere with the mechanical vacuum pump: below 25 kPa within an hour, never below 7 kPa', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: 0.1e6, L: 11.5, vac: false });
    near(s.f.Pc, B.PATM, 1, 'starts at atmospheric');
    const u = { ...COLD, vp: true, sj: false }; let p30 = 0;
    runPlant(s, u, 3600, dt, x => { if (x.t <= 1800) p30 = x.f.Pc; });
    assert.ok(p30 > 30e3 && p30 < 50e3, `30 min: ${p30} dt=${dt}`);
    assert.ok(s.f.Pc > 15e3 && s.f.Pc < 22e3, `1 h: ${s.f.Pc} dt=${dt}`);
    runPlant(s, u, 4 * 3600, dt);
    assert.ok(s.f.Pc > 7e3 && s.f.Pc < 12e3, `5 h: ${s.f.Pc}`);
  }
});

test('startup: vacuum pump, then steam to the condenser with the SJAE in service pulls rated vacuum', () => {
  for (const dt of [0.5, 2]) {
    const s = B.init({ P: 7e6, L: 14.3, vac: false });
    runPlant(s, { ...COLD, vp: true, sj: true }, 3600, dt);
    assert.ok(s.f.Pc < 22e3 && s.f.wsj === 0, `SJAE idle with the steam valve shut: ${s.f.Pc}`);
    runPlant(s, { ...COLD, vp: true, sj: true, mv: 30 / B.WMS, heat: 45e6 }, 1800, dt);
    assert.equal(B.fault(s), '');
    assert.ok(s.f.sjc > 0.99, 'SJAE at full capacity');
    assert.ok(s.f.Pc < 4e3 && s.f.pa < 500, `vacuum ${s.f.Pc}, air ${s.f.pa} dt=${dt}`);
  }
});

test('SJAE without motive steam: air builds up and vacuum degrades; ejector limits', () => {
  for (const dt of [0.25, 2]) {
    const s0 = runPlant(rated(), RATED_U, 3600, dt), s = runPlant(rated(), { ...RATED_U, sj: false }, 3600, dt);
    near(s.f.Ma - rated().f.Ma, (B.WLK + B.WRAD) * 3600, 0.5, `air accumulates dt=${dt}`);
    assert.ok(s.f.pa > 2e3, `air partial pressure ${s.f.pa}`);
    assert.ok(s.f.Pc > s0.f.Pc + 3e3, `vacuum degrades: ${s.f.Pc} vs ${s0.f.Pc}`);
  }
  assert.equal(B.sjCap(0.6e6, 7e3), 0, 'motive steam below 0.7 MPa');
  near(B.sjCap(0.95e6, 7e3), 0.5, 1e-12, 'half capacity at 0.95 MPa');
  assert.equal(B.sjCap(7e6, 36e3), 0, 'ejector broken above 35 kPa suction');
  assert.equal(B.vpCap(6e3), 0, 'vacuum pump useless below 7 kPa');
});

test('air blanketing reduces UA and raises the back pressure for the same load', () => {
  near(B.blanket(0.1), 0.5, 1e-12, 'half UA at 10 % air');
  near(B.UA0 * B.blanket(B.yAir(39.1 + 273.15, B.PA0)), B.UA, 1, 'clean UA recalibrated to the design value');
  const hx = B.expansion(7e6, common.sat(common.Tsat(7e6)).hg)(7e3), ins = [[1000, hx]];
  const T0 = B.condSteady(ins, undefined, 0), T1 = B.condSteady(ins, undefined, B.PA0), T2 = B.condSteady(ins, undefined, 2e3);
  assert.ok(T0 < T1 && T2 > T1 + 2, `shell temperature rises with air: ${T0} ${T1} ${T2}`);
});

test('air mass is conserved: in-leakage, radiolytic gas and breaker in, SJAE and vacuum pump out', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(); let M = s.cd.Ma, wi = 0, wo = 0;
    const U = x => ({ ...RATED_U, vp: x.t > 600, vb: x.t > 1200 && x.t < 1300, sj: x.t < 1500 || x.t > 1800 });
    runPlant(s, U, 2400, dt, x => { M += (x.f.wain - x.f.waout) * dt; wi += x.f.wvb; wo += x.f.waout; });
    near(s.cd.Ma, M, 1e-6, `air balance dt=${dt}`);
    assert.ok(wi > 0 && wo > 0, 'breaker and removal both acted');
  }
});

test('vacuum breaker: opening it at no load brings the shell up to atmospheric, not past it', () => {
  for (const dt of [0.25, 2]) {
    const s = rated(); let pmax = 0;
    runPlant(s, { ...COLD, sj: false, vb: true }, 900, dt, x => { pmax = Math.max(pmax, x.f.Pc); });
    assert.ok(s.f.Pc > 0.095e6, `near atmospheric after 15 min: ${s.f.Pc} dt=${dt}`);
    assert.ok(pmax < B.PATM + 200, `not above atmospheric: ${pmax}`);
    assert.equal(B.fault(s), '');
  }
});
