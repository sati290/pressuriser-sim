// BWR main steam line tests: MSIVs, turbine stop and control valves, bypass, EHC pressure control.
// Run: node --test tests/  (or npm test)
import test from 'node:test';
import assert from 'node:assert/strict';
import { B, K, near, MPa, run, runPlant, rated, RATED_U, RATED_C, OPEN, ISO, CV0, W0, P0 } from './bwr-helpers.mjs';

const line = (P, v, pc = 7e3) => B.steamLine(P, pc, { ms: [1, 1], sv: 1, cv: 0, bp: 0, ...v });

test('steam line: 0.3 MPa to the throttle at rated, control valve about 87 % open, 105 % wide open, bypass 25 %', () => {
  const r = line(P0, { cv: CV0 });
  near(r.w, W0, 1e-6, 'rated flow at the rated lift');
  near(MPa(P0 - r.Pl), 0.3, 1e-6, 'dome-to-throttle drop');
  assert.ok(CV0 > 0.85 && CV0 < 0.9, `rated lift ${CV0}`);
  near(line(P0, { cv: 1 }).w / B.WR, 1.05, 1e-6, 'valves wide open');
  near(B.KB * B.PL0 / B.WR, 0.25, 1e-12, 'bypass capacity at the rated throttle pressure');
  const b = line(P0, { sv: 0, cv: 1, bp: 1 });
  assert.ok(b.wt === 0 && b.wb / B.WR > 0.25 && b.wb / B.WR < 0.27, `bypass alone (header nearer dome pressure): ${b.wb / B.WR}`);
  const m = line(P0, { cv: CV0, bp: 0.5 });
  near(m.w, m.wt + m.wb, 1e-9, 'line flow = turbine + bypass');
  // MSIVs: flow area in proportion to position; series restrictions
  const h = line(P0, { ms: [0.5, 1], cv: CV0 }).w, q = line(P0, { ms: [0.2, 0.2], cv: CV0 }).w;
  assert.ok(h < W0 && h > 0.95 * W0, `one MSIV half open: ${h}`);
  assert.ok(q < 0.85 * W0, `both at 20 %: ${q}`);
  // isolated: no flow, the header drains to the condenser if a valve beyond it is open
  const iso = line(P0, { ms: [0, 1], cv: CV0 });
  assert.equal(iso.w, 0, 'no flow with an MSIV shut'); near(iso.Pl, 7e3, 1e-9, 'header at condenser pressure');
  assert.equal(line(P0, {}).Pl, P0, 'header at dome pressure with the turbine and bypass valves shut');
  assert.equal(line(5e3, { cv: 1 }).w, 0, 'no reverse flow');
  // inverse
  near(line(5e6, { cv: B.cvPos(800, 5e6) }).w, 800, 1e-6, 'cvPos inverts the line');
  assert.equal(B.cvPos(3000, P0), null, 'more than wide open');
});

test('controller constants match the plant', () => {
  near(K.WR, B.WR, 1e-9, 'WR'); near(K.PL0, B.PL0, 1e-6, 'PL0'); near(K.KT, B.KT, 1e-15, 'KT'); near(K.BPF, B.BPF, 0, 'BPF');
  for (const x of [0, 0.3, 0.86, 1]) near(K.cvFlow(x) * B.WR, B.KT * B.PL0 * B.fcv(x), 1e-6, `cvFlow ${x}`);
  near(K.cvLift(1), CV0, 1e-9, 'rated demand gives the rated lift');
});

test('valve strokes: MSIVs 4 s, stop valve 0.2 s shut, bypass 0.5 s; trip flags override the switches', () => {
  const s = rated(), u = { ...RATED_U, ...ISO, sv: false, bp: 1 };
  B.step(s, u, 0.1);
  near(s.ms[0], 1 - 0.1 * B.MSRATE, 1e-12, 'MSIV travel'); near(s.sv, 0.5, 1e-12, 'stop valve half shut in 0.1 s'); near(s.bp, 0.2, 1e-12, 'bypass travel');
  runPlant(s, u, 4, 0.1); assert.equal(s.ms[0], 0); assert.equal(s.ms[1], 0); assert.equal(s.sv, 0); assert.equal(s.bp, 1);
  // undefined switch commands hold the valves; trip flags shut them whatever the switches say
  const t = rated(); B.step(t, { ...RATED_U, ms1: undefined, ms2: undefined, sv: undefined }, 1);
  assert.deepEqual([t.ms[0], t.ms[1], t.sv], [1, 1, 1], 'undefined holds');
  runPlant(t, { ...RATED_U, trip: { turbine: true } }, 1, 0.25);
  assert.equal(t.sv, 0, 'turbine trip shuts the stop valve'); assert.equal(t.f.wt, 0, 'no turbine flow');
  runPlant(t, { ...RATED_U, trip: { msiv: true } }, 5, 0.25);
  assert.deepEqual(t.ms, [0, 0], 'isolation trip shuts both MSIVs'); assert.equal(t.f.wst, 0, 'no steam flow');
});

test('rated with everything in A at dt 0.25 and 2 s: control valve about 87 %, bypass shut, header 6.7 MPa', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 600, dt);
    near(MPa(s.d.P), 7.0, 0.002, `P dt=${dt}`);
    near(MPa(s.f.Pl), 6.7, 0.003, `header dt=${dt}`);
    near(s.cv, CV0, 0.003, `control valve dt=${dt}`);
    assert.equal(s.f.wb, 0, `bypass shut dt=${dt}`);
    near(s.f.wt + s.f.wsj, W0, 2, `turbine flow plus the air ejector's motive steam dt=${dt}`);
    near(s.d.level, 14.3, 0.01, `level dt=${dt}`);
  }
});

test('turbine trip at rated, bypass in A, no scram: bypass opens to its capacity at once, SRVs lift for the rest', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 10, dt); c.sv = false;
    let tFull = 0, n = 0;
    run(s, k, c, 20, dt, x => { if (!tFull && x.bp > 0.999) tFull = x.t - 10; n = Math.max(n, x.f.nsrv); return true; });
    assert.ok(tFull > 0 && tFull <= Math.max(1, 2 * dt), `bypass fully open ${tFull} s after the trip dt=${dt}`);
    assert.equal(s.f.wt, 0, `no turbine flow dt=${dt}`);
    assert.ok(s.f.wb > 0.26 * B.WR, `bypass passes its capacity (more at the higher pressure): ${s.f.wb} dt=${dt}`);
    assert.ok(n >= 4, `SRVs lift: ${n} dt=${dt}`);
    assert.ok(s.d.P > B.SRV[0].p, `pressure above the lowest set pressure dt=${dt}`);
    assert.ok(s.f.ret > 400, `condenser takes the bypass steam: ${s.f.ret} dt=${dt}`);
  }
});

test('turbine trip with scram: bypass alone holds the pressure, back to the setpoint', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 10, dt); c.sv = false; B.scram(s);
    let Pmax = 0, n = 0, wb = 0, nb = 0;
    run(s, k, c, 1200, dt, x => { Pmax = Math.max(Pmax, x.d.P); n = Math.max(n, x.f.nsrv); if (x.t > 600) { wb += x.f.wb; nb++; } return true; });
    assert.equal(B.fault(s), '', `dt=${dt}`);
    assert.ok(Pmax < B.SRV[0].p, `no SRV needed: peak ${MPa(Pmax)} dt=${dt}`);
    near(MPa(s.d.P), 7.0, 0.01, `back at the setpoint dt=${dt}`);
    // the cold feed (no extraction steam) comes in bursts that condense steam: average over the last 10 min
    assert.ok(wb / nb > 5 && wb / nb < 60, `bypass passes the decay-heat steam: ${wb / nb} dt=${dt}`);
    // the level setpoint is set down after a scram and is still ramping back
    near(s.d.level, 14.3 - K.setdown(k, s), 0.2, `level dt=${dt}`);
    assert.ok(s.f.Pc < 5e3, `condenser easy: ${s.f.Pc} dt=${dt}`);
  }
});

test('MSIV closure isolates the condenser: no steam reaches it, SRVs lift', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 10, dt); Object.assign(c, ISO); B.scram(s);
    let after = 0, n = 0;
    run(s, k, c, 600, dt, x => { if (x.t > 16) after = Math.max(after, x.f.wst + x.f.wt + x.f.wb); n = Math.max(n, x.f.nsrv); return true; });
    assert.equal(after, 0, `no steam past the MSIVs once shut dt=${dt}`);
    assert.ok(n >= 4, `SRVs lift dt=${dt}`);
    assert.equal(s.f.ret, 0, `no condensate dt=${dt}`);
    assert.equal(B.fault(s), '', `dt=${dt}`);
  }
});

test('load limit below the demand: the bypass takes the excess, pressure stays at the setpoint', () => {
  for (const dt of [0.25, 2]) {
    const c = RATED_C(), s = rated(), k = K.init(c);
    run(s, k, c, 10, dt); c.ll = 80; run(s, k, c, 600, dt);
    near(MPa(s.d.P), 7.0, 0.002, `P dt=${dt}`);
    near(k.dem.cv, K.cvLift(0.8), 1e-9, `control valve at the load limit dt=${dt}`);
    near(s.f.wt / B.WR, 0.8, 0.01, `turbine at 80 % dt=${dt}`);
    assert.ok(s.f.wb / B.WR > 0.15 && s.f.wb / B.WR < 0.2, `bypass takes the rest: ${s.f.wb / B.WR} dt=${dt}`);
    near(s.d.level, 14.3, 0.02, `level dt=${dt}`);
  }
});

test('startup: stop valve shut, the bypass in A passes the steam once pressure reaches the setpoint', () => {
  const c = { ...RATED_C(), heat: 60, pset: 1e6, sv: false, cv: 0 }, s = B.init({ P: 0.9e6, L: 14.3, sv: 0, cv: 0 }), k = K.init(c);
  run(s, k, c, 1200, 1);
  // the cold feed (no extraction steam) comes in bursts, so the pressure dips a little now and then
  let lo = 9e9, hi = 0, wb = 0, n = 0;
  run(s, k, c, 1200, 1, x => { lo = Math.min(lo, x.d.P); hi = Math.max(hi, x.d.P); wb += x.f.wb; n++; assert.equal(x.f.wt, 0, 'turbine shut'); return true; });
  assert.ok(lo > 0.9e6 && hi < 1.02e6, `held near the setpoint: ${MPa(lo)}–${MPa(hi)}`);
  assert.ok(wb / n > 15 && wb / n < 35, `bypass passing the 60 MW of steam: ${wb / n}`);
});

test('mass and energy: vessel + hotwell + condenser with the bypass open', () => {
  const s = rated();
  let m = s.M, u = s.U, tot = s.M + s.Mh + s.cd.Ms + s.Mhs + s.Mls;
  runPlant(s, { ...RATED_U, cv: 0.5, bp: 0.7, mu: 0.1, srv: [true] }, 300, 1, x => {
    const f = x.f; m += f.fw - f.wst - f.wsrv; u += f.Q + f.Qs + f.fw * f.hfw - (f.wst + f.wsrv) * f.hg; tot += f.mk - f.wsrv; return true;
  });
  assert.ok(s.f.wb > 200 && s.f.wt > 500, `both paths flowing: ${s.f.wb}, ${s.f.wt}`);
  near(s.M, m, 1e-3, 'vessel mass'); near(s.U, u, 1e-9 * Math.abs(u), 'vessel energy');
  near(s.M + s.Mh + s.cd.Ms + s.Mhs + s.Mls, tot, 1e-3, 'vessel + hotwell + condenser steam + heater shell mass');
});
