// Turbine and generator (roadmap step 15): stage pressures, output, shaft speed, breaker, load set,
// load rejection, turbine trip, turbine roll.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { common, B, K, near, run, runPlant, P0, W0, CV0, FV0, RATED_C, RATED_U } from './bwr-helpers.mjs';

// rated with protection on and the controllers in A (rods at the critical position, kinetics on)
const ratedA = () => {
  const s = B.init({ P: P0, L: 14.3, gen: W0, cv: CV0, fv: FV0, n: 1, tfw: 216, rps: true });
  const c = { ...RATED_C(), mw: 1200, rod: s.core.x * 100 };
  delete c.heat;
  return [s, c, K.init(c)];
};

test('stage pressures: rated points, Stodola fall with flow, the condenser pressure at no flow', () => {
  const L = B.lineAt(B.WTR).ps;
  near(L.p1 / 1e6, 5.896, 0.002, 'first stage');
  near(L.peH, B.PE, 1, 'HP extraction at its design pressure');
  near(L.pX / 1e6, 1.38, 1e-6, 'HP exhaust');
  near(L.pLi / 1e6, 1.325, 1e-6, 'LP inlet');
  near(L.peL, B.PEL, 1, 'LP extraction at its design pressure');
  const h = B.lineAt(B.WTR / 2).ps, z = B.lineAt(0).ps;
  assert.ok(h.peH / L.peH > 0.48 && h.peH / L.peH < 0.52, `about half at half flow: ${h.peH / L.peH}`);
  near(z.p1, B.PCD, 1e-6, 'no flow: every stage at the condenser pressure');
  assert.ok(B.lineAt(B.WTR, 3 * B.PCD).ps.peL > L.peL, 'a worse vacuum raises the LP extraction pressure');
  // a stage that takes more steam draws its own pressure down
  const more = B.stagePs(B.TK, B.WTR, 1.5 * B.WSD, B.WSLD, B.PCD);
  assert.ok(more.peH < L.peH - 0.1e6, `more HP extraction lowers its pressure: ${more.peH}`);
  // the controller's constants match the plant
  assert.equal(K.NR, B.NR); assert.equal(K.HIN, B.HIN);
  near(K.MWR / (B.PGR * B.WR / B.WTR), 1, 0.005, 'MW per unit of rated steam');
});

test('rated: on the grid at 1800 rpm, about 1110 MWe, steady at dt 0.25 and 2 s', () => {
  for (const dt of [0.25, 2]) {
    const [s, c, k] = ratedA();
    run(s, k, c, 300, dt);
    assert.equal(s.tg.brk, true); near(s.tg.w, 1, 0, 'grid speed');
    near(s.f.Pgen / 1e6, 1110, 10, `gross output dt=${dt}`);
    near(s.d.P / 1e6, 7, 0.005, `pressure dt=${dt}`);
    near(s.f.p1 / 1e6, 5.9, 0.03, `first stage dt=${dt}`);
    assert.equal(s.rps.log.length, 0, `no event dt=${dt}`);
  }
});

test('load rejection at rated: fast closure and scram, speed peaks about 105 %, the governor holds 1800 rpm', () => {
  for (const dt of [0.25, 2]) {
    const [s, c, k] = ratedA();
    run(s, k, c, 60, dt);
    B.breaker(s, false);
    let pk = 0, pm = 0;
    run(s, k, c, 120, dt, x => { pk = Math.max(pk, x.tg.w); pm = Math.max(pm, x.d.P); return true; });
    assert.ok(s.rps.lat.tcv, `control valve fast closure scram dt=${dt}`);
    assert.ok(s.scram, 'scrammed');
    assert.ok(!s.rps.lat.ovsp, `no overspeed trip: peak ${pk} dt=${dt}`);
    assert.ok(pk > 1.02 && pk < 1.08, `speed peak ${pk} dt=${dt}`);
    assert.ok(pm < 7.6e6, `pressure peak ${pm} dt=${dt}`);
    run(s, k, c, 600, dt);
    near(s.tg.w, 1, 0.006, `back near 1800 rpm dt=${dt}`);
    assert.equal(s.f.Pgen, 0, 'no output off the grid');
  }
});

test('turbine trip: the breaker trips and locks, the intercept valves shut, no overspeed; it coasts down', () => {
  const [s, c, k] = ratedA();
  run(s, k, c, 60, 0.5);
  c.sv = false;
  let pk = 0;
  run(s, k, c, 60, 0.5, x => { pk = Math.max(pk, x.tg.w); return true; });
  assert.equal(s.tg.brk, false); assert.equal(s.tg.lock, true);
  assert.equal(s.f.iv, 0, 'intercept valves shut');
  assert.ok(pk < 1.01, `no overspeed: ${pk}`);
  assert.ok(s.rps.log.some(e => e.text.includes('Generator trip')), 'logged');
  run(s, k, c, 600, 2);
  assert.ok(s.tg.w < 0.97 && s.tg.w > 0.5, `coasting down: ${s.tg.w}`);
  // locked: a close is refused until the breaker is opened by hand; off speed the sync check refuses it
  assert.equal(B.breaker(s, true), 'lock');
  B.breaker(s, false);
  assert.equal(B.breaker(s, true), 'sync');
  assert.equal(s.tg.brk, false);
});

test('load set: the control valve follows it, the bypass takes the rest, pressure held', () => {
  for (const dt of [0.25, 2]) {
    const [s, c, k] = ratedA();
    run(s, k, c, 60, dt);
    c.mw = 800; run(s, k, c, 300, dt);
    near(s.f.Pgen / 1e6, 800, 2, `output dt=${dt}`);
    near(s.d.P / 1e6, 7, 0.01, `pressure dt=${dt}`);
    assert.ok(s.f.wb > 350, `bypass ${s.f.wb} dt=${dt}`);
    assert.ok(s.f.pe < 1.9e6 && s.f.pe > 1.6e6, `HP extraction falls with the turbine flow: ${s.f.pe} dt=${dt}`);
    c.mw = 1200; run(s, k, c, 300, dt);
    near(s.f.Pgen / 1e6, 1110, 10, `back dt=${dt}`);
    assert.ok(s.f.wb < 1, 'bypass shut');
    assert.equal(s.rps.log.length, 0, 'no trip');
  }
});

test('turbine roll from hot standby: the governor brings it to 1800 rpm, the breaker closes, load picks up', () => {
  for (const dt of [0.25, 2]) {
    const P = 7e6, Q = 0.05 * 3293e6, W = Q / (common.sat(common.Tsat(P)).hg - common.sat(B.THW).hf);
    const s = B.init({ P, L: 14.3, gen: W, cv: 0, bp: W / (B.KB * P), sv: 0, fv: B.fwValve(W, 1, P), n: 1, tfw: B.THW - 273.15, mg: 0.3, mode: 'startup', rps: true });
    assert.equal(s.tg.brk, false); assert.equal(s.tg.w, 0);
    const c = { n: 100, fv: B.fwValve(W, 1, P) * 100, ext: 0, exl: 0, ms1: true, ms2: true, sv: false, cv: 0, bp: s.bp * 100, ll: 105, srv: [], pset: 7e6, lset: 14.3, dpset: 2.2e6,
      rod: s.core.x * 100, mg: 30, mw: 1200, auto: { n: true, fv: true, cv: true, bp: true, mu: true, rj: true, rw: true, rv: true } };
    const k = K.init(c);
    run(s, k, c, 60, dt);
    assert.equal(B.breaker(s, true), 'sync', 'not at speed');
    c.sv = true; run(s, k, c, 400, dt);
    near(s.tg.w, 1, 0.005, `at speed dt=${dt}`);
    assert.equal(B.breaker(s, true), '', `breaker closes dt=${dt}`);
    run(s, k, c, 200, dt);
    assert.ok(s.f.Pgen > 20e6, `on the grid with the reactor's steam: ${s.f.Pgen / 1e6} MWe dt=${dt}`);
    assert.ok(s.f.wb < 1, `the turbine took the bypass steam dt=${dt}`);
    near(s.d.P / 1e6, 7, 0.01, 'pressure held');
  }
});

test('reverse power: the control valve shut on the grid trips the breaker 10 s after the shaft power runs out', () => {
  const s = B.init({ P: P0, L: 14.3, gen: W0, cv: CV0, fv: FV0, n: 1, tfw: 216 });
  // the valve strokes shut in about 4 s; the LP turbine's lagged power (5 s) keeps the shaft power above
  // the windage for about 25 s more
  runPlant(s, { ...RATED_U, cv: 0, bp: 1 }, 30, 0.5);
  assert.equal(s.tg.brk, true, 'still closed after 30 s');
  runPlant(s, { ...RATED_U, cv: 0, bp: 1 }, 15, 0.5);
  assert.equal(s.tg.brk, false); assert.equal(s.tg.lock, true);
  assert.ok(s.rps.log.some(e => e.text.includes('reverse power')));
});
