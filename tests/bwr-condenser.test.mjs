// BWR turbine stub and condenser tests. Run: node --test tests/  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, B, near, runPlant, rated, RATED_U, Q0, W0 } from './bwr-helpers.mjs';

test('rated heat balance: condenser near 7 kPa, realistic duty, stub output, hotwell at the shell temperature', () => {
  const s = runPlant(rated(), RATED_U, 600, 1);
  assert.ok(s.f.Pc > 5e3 && s.f.Pc < 7.5e3, `back pressure ${s.f.Pc} Pa`);
  near(s.f.Tc - 273.15, 37.4, 0.5, 'shell temperature (two heater stages since step 5, drains pumped forward since step 6)');
  assert.ok(s.f.Qc > 2.1e9 && s.f.Qc < 2.5e9, `condenser duty ${s.f.Qc / 1e6} MW`);
  assert.ok(s.f.Pgen > 850e6 && s.f.Pgen < 1150e6, `gross output ${s.f.Pgen / 1e6} MW`);
  near(s.f.Tcw - B.TCW, s.f.Qc / (B.WCW * B.CPW), 1e-6, 'circulating water rise from the duty');
  near(s.f.Thw, s.f.Tc, 0.2, 'hotwell at the shell temperature');
  // energy: core heat = condenser duty + electrical output / ETAG (steady, no losses modelled)
  near((s.f.Qc + s.f.Pgen / B.ETAG) / Q0, 1, 0.01, 'heat balance closes');
  // exhaust is wet
  assert.ok(s.f.hx < common.sat(s.f.Tc).hg, 'wet exhaust');
});

test('condenser: less cooling water raises the back pressure; vacuum follows the load', () => {
  const c = { Tc: 312, Ms: 0 }, line = B.expansion(7e6, common.sat(common.Tsat(7e6)).hg), hx = line(7e3);
  const steady = (w, cw) => { const T = B.condSteady([[w, hx]], cw); return common.sat(T).p; };
  const p100 = steady(1000, { w: B.WCW, T: B.TCW }), p50 = steady(500, { w: B.WCW, T: B.TCW });
  assert.ok(p50 < p100 - 1500, `half the exhaust, better vacuum: ${p50} vs ${p100}`);
  assert.ok(steady(1000, { w: B.WCW / 2, T: B.TCW }) > p100 + 3000, 'half the cooling water');
  assert.ok(steady(1000, { w: B.WCW, T: B.TCW + 10 }) > p100 + 3000, 'warmer inlet');
  // with no cooling water the shell fills with steam: pressure climbs step by step
  let Pc = 0; for (let i = 0; i < 20; i++) Pc = B.condense(c, [[1000, hx]], 1, { w: 0, T: B.TCW }).Pc;
  assert.ok(Pc > 0.1e6, `no cooling: ${Pc}`);
});

test('steam valve closed: the condenser cools to the circulating water, hotwell stops gaining', () => {
  const s = runPlant(rated(), { ...RATED_U, mv: 0, heat: 0, n: 0, fv: 0 }, 300, 1);
  assert.ok(s.f.Pc < 3.5e3, `vacuum improves with no load: ${s.f.Pc}`);
  near(s.f.ret, 0, 1, 'no condensate');
  assert.equal(B.fault(s), '');
});
