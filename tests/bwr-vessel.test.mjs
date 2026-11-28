// BWR vessel steel, level instruments, suppression pool (roadmap step 12). Run: npm test
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, B, near, runPlant, rated, RATED_U, NOFEED, P0 } from './bwr-helpers.mjs';

test('vessel steel: lags the water on heat-up, steady loss about 1 MW at rated', () => {
  const s = rated();
  near(-s.f.Qs, 0, 1, 'init: no record yet');
  runPlant(s, RATED_U, 60, 1);
  assert.ok(s.f.Ql > 0.8e6 && s.f.Ql < 1.2e6, `loss ${s.f.Ql / 1e6} MW`);
  near(s.f.Qs, -s.f.Ql, 1e3, 'steady: the water supplies the loss');
  const h = B.init({ P: 0.1e6, L: 11.5 });
  runPlant(h, { heat: 30e6, ...NOFEED, cv: 0, srv: [] }, 1800, 2);
  assert.ok(h.d.Tl - h.Ts > 5 && h.d.Tl - h.Ts < 20, `steel lags ${h.d.Tl - h.Ts} K at ~55 K/h`);
  assert.ok(h.f.Qs < -5e6, `steel absorbs heat: ${h.f.Qs / 1e6} MW`);
});

test('level instruments: inches above instrument zero, BWR/4 setpoints in order', () => {
  near(B.toIn(B.Z.TAF), -162, 0.5, 'top of fuel');
  near(B.toIn(B.Z.NWL), 35, 1, 'normal level');
  near(B.fromIn(0), B.IZ, 1e-12, 'round trip');
  const L = B.LV; assert.ok(L.L8 > L.L7 && L.L7 > L.L4 && L.L4 > L.L3 && L.L3 > L.L2 && L.L2 > L.L1 && B.fromIn(L.L1) > B.Z.TAF, 'ordered, L1 above the fuel');
  const s = rated(); near(s.f.lin, B.toIn(s.d.level), 1e-9, 'recorded');
});

test('bulk flashing on a depressurisation swells the level more slowly than core boiling clears', () => {
  const s = B.init({ P: 7e6, L: 13.5 });
  let peak = 0; runPlant(s, { heat: 0, ...NOFEED, cv: 0.5, bp: 1, srv: [] }, 20, 0.25, x => { peak = Math.max(peak, x.Mfl); return true; });
  assert.ok(peak > 500, `flash voids ${peak} kg`);
  assert.ok(s.Mb >= s.Mfl - 1e-9 && s.d.Vb * s.d.rg >= s.Mfl - 1e-6, 'flash voids are in the level');
});

test('suppression pool: SRV steam condenses in it; cooling removes heat; energy balance', () => {
  for (const dt of [0.25, 2]) {
    const s = B.init({ P: 7e6, L: 14.3 }), srv = [true, true];
    const M0 = s.pool.M, U0 = s.pool.M * common.sat(s.pool.T).hf;let win = 0, Ein = 0;
    runPlant(s, { heat: 200e6, ...NOFEED, cv: 0, srv }, 600, dt, x => { const w = x.f.wsrv * dt; win += w; Ein += w * x.f.hg; return true; });
    near(s.pool.M - M0, win, 1e-6 * M0, `pool mass gains the steam dt=${dt}`);
    near(s.pool.M * common.sat(s.pool.T).hf - U0, Ein, 2e-3 * Ein, `pool energy dt=${dt}`);
    assert.ok(s.f.Tp - B.TP0 > 5, `pool heats: ${s.f.Tp - 273.15} °C`);
    assert.ok(s.f.Lp > 0.05, `level up ${s.f.Lp} m`);
    const T1 = s.pool.T; runPlant(s, { heat: 0, ...NOFEED, cv: 0, srv: [], pc: true }, 600, dt);
    assert.ok(s.pool.T < T1 - 0.5 && s.f.Qpc > 10e6, `cooling: ${s.f.Qpc / 1e6} MW dt=${dt}`);
  }
  const h = B.init({ P: P0, L: 14.3, tpool: 99.9 });
  runPlant(h, { heat: 300e6, ...NOFEED, cv: 0, srv: [true, true, true] }, 60, 1);
  assert.equal(B.fault(h), 'pool');
});
