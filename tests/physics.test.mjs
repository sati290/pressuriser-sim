// Physics regression tests. Run: node tests/physics.test.mjs  (or npm test)
// Uses node:test; no dependencies.
import test from 'node:test';
import assert from 'node:assert/strict';
import { phys, run, C, MPa } from './load-physics.mjs';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, expected ${b} ± ${tol}`);

// Control sets. inv: 'man' | 'pres' (letdown holds pressure) | 'lev' (charging holds level).
// Assists never switch mode themselves; tests that cross solid <-> two-phase switch like an operator would.
const RELIEF_COLD = { relAuto: true, relSet: 3.6e6, relMan: false };
const RELIEF_HOT  = { relAuto: true, relSet: 16.2e6, relMan: false };
const COLD_CTL = { aP: false, inv: 'pres', set: 2.8e6, lset: 25, heat: 0, ch: 5, ld: 5, q: 1800, sp: 0, loss: 60, ...RELIEF_COLD };
const HOT_CTL  = { aP: true, inv: 'lev', set: 15.5e6, lset: 25, heat: 0, ch: 6, ld: 6, q: 200, sp: 0, loss: 60, ...RELIEF_HOT };
const HOT_MAN  = { aP: false, inv: 'man', set: 15.5e6, lset: 25, heat: 0, ch: 6, ld: 6, q: 60, sp: 0, loss: 60, ...RELIEF_HOT };
// operator hands over from solid to bubble control: heater/spray assist on, charging on level
const toBubble = (s, c) => { c.ld = s.f.ld; c.aP = true; c.inv = 'lev'; phys.engage(s, c); };
const toSolid  = (s, c) => { c.ld = s.f.ld; c.ch = s.f.ch; c.aP = false; c.q = 0; c.inv = 'pres'; phys.engage(s, c); };

test('saturation properties match IAPWS steam tables', () => {
  // [°C, p MPa, rho_f, rho_g, h_f kJ/kg, h_g kJ/kg]
  const ref = [
    [100, 0.101418, 958.35, 0.5976, 419.17, 2675.6],
    [200, 1.5551, 864.66, 7.8610, 852.27, 2792.0],
    [300, 8.5879, 712.14, 46.168, 1344.8, 2749.6],
    [344.8, 15.502, 594.2, 101.9, 1630.0, 2596.3],
  ];
  for (const [c, p, rf, rg, hf, hg] of ref) {
    const [P, , RF, RG, HF, HG] = phys.satX(c + 273.15);
    near(MPa(P), p, p * 0.002, `p_sat(${c})`);
    near(RF, rf, rf * 0.002, `rho_f(${c})`);
    near(RG, rg, rg * 0.003, `rho_g(${c})`);
    near(HF / 1e3, hf, 1.5, `h_f(${c})`);
    near(HG / 1e3, hg, 1.5, `h_g(${c})`);
  }
});

test('tabulated sat() agrees with the exact equations', () => {
  for (let c = 20; c < 370; c += 7.3) {
    const T = c + 273.15, ex = phys.satX(T), q = phys.sat(T);
    near(q.p, ex[0], ex[0] * 1e-4 + 1, `p at ${c}`);
    near(q.hf, ex[4], 50, `hf at ${c}`);
  }
});

test('two-phase pressuriser state round-trips through init', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  assert.equal(s.d.mode, 'two');
  near(MPa(s.d.P), 15.5, 0.005, 'P');
  near(s.d.level, 0.25, 0.002, 'level');
});

test('water-solid state round-trips through init', () => {
  const s = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  assert.equal(s.d.mode, 'solid');
  near(MPa(s.d.P), 2.8, 0.01, 'P');
});

test('hot standby is a steady state', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  run(s, HOT_CTL, 3600, 0.5);
  near(MPa(s.d.P), 15.5, 0.05, 'P after 1 h');
  near(s.d.level * 100, 25, 0.5, 'level after 1 h');
  near(C(s.Tr), 291.7, 0.5, 'Tr after 1 h');
});

test('load step: insurge raises pressure, spray responds', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  run(s, HOT_CTL, 1800, 0.5);
  const P0 = s.d.P;
  s.extra = { Q: 40e6, end: s.t + 120 };
  let Pmax = P0, sprayMax = 0, surgeMax = 0;
  run(s, HOT_CTL, 300, 0.25, x => { Pmax = Math.max(Pmax, x.d.P); sprayMax = Math.max(sprayMax, x.f.ws); surgeMax = Math.max(surgeMax, x.surge); });
  assert.ok(surgeMax > 5, `insurge expected, max surge ${surgeMax}`);
  assert.ok(Pmax - P0 > 0.05e6, `pressure should rise on insurge, rise ${MPa(Pmax - P0)} MPa`);
  assert.ok(sprayMax > 0, 'spray should open');
});

test('cold start: bubble forms near Tsat(2.8 MPa) and level settles at setpoint', () => {
  const s = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  const c = { ...COLD_CTL };
  let bubbleAt = null, Tbubble = null, Pmax = 0;
  run(s, c, 9 * 3600, 1, x => {
    Pmax = Math.max(Pmax, x.d.P);
    if (!bubbleAt && x.d.mode === 'two') { bubbleAt = x.t; Tbubble = C(x.d.T); toBubble(x, c); }
  });
  assert.ok(bubbleAt, 'bubble never formed');
  near(Tbubble, 230, 3, 'bubble temperature');
  near(bubbleAt / 3600, 5.6, 0.6, 'hours to bubble');
  assert.ok(MPa(Pmax) < 3.0, `solid-mode pressure control overshot to ${MPa(Pmax)} MPa`);
  near(s.d.level * 100, 25, 1.5, 'final level');
});

test('heat-up to hot standby keeps subcooling and level', () => {
  const s = phys.init({ solid: false, Tr: 75, P: 2.8e6, L: 0.25 });
  const c = { ...COLD_CTL, aP: true, inv: 'lev', heat: 19.5, ...RELIEF_HOT };
  c.set = 8e6; run(s, c, 3 * 3600, 1);
  c.set = 15.5e6; run(s, c, 4 * 3600, 1, x => C(x.Tr) < 291.7);
  assert.ok(C(s.Tr) >= 291, `coolant reached ${C(s.Tr)} °C`);
  assert.ok(phys.Tsat(s.Pr) - s.Tr > 20, 'subcooling margin below 20 K');
  near(s.d.level * 100, 25, 3, 'level');
});

test('refill to water-solid at low temperature hands control back to letdown', () => {
  const s = phys.init({ solid: false, Tr: 150, P: 2.8e6, L: 0.25 });
  const c = { ...COLD_CTL, aP: true, inv: 'lev', lset: 100, heat: -1 };
  let prev = s.d.mode, wentSolid = false;
  run(s, c, 3 * 3600, 2, x => {
    if (prev === 'two' && x.d.mode === 'solid') { wentSolid = true; toSolid(x, c); }
    prev = x.d.mode;
  });
  assert.ok(wentSolid, 'never went solid');
  assert.equal(s.d.mode, 'solid');
  near(MPa(s.d.P), 2.8, 0.05, 'P held by letdown');
});

test('mass is conserved across pressuriser + RCS with no boundary flows', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  const c = { ...HOT_CTL, inv: 'man', ch: 0, ld: 0 };
  const m0 = s.M + s.Ms + s.Mr;
  s.extra = { Q: 40e6, end: 120 };
  run(s, c, 600, 0.5);
  const m1 = s.M + s.Ms + s.Mr;
  near(m1, m0, 1, 'total mass (kg)');
});

test('hot standby with every assist off is steady when heaters match losses', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  run(s, HOT_MAN, 3600, 0.5);
  near(MPa(s.d.P), 15.5, 0.05, 'P after 1 h');
  near(s.d.level * 100, 25, 0.5, 'level after 1 h');
});

test('assists do not hand over by themselves: letdown pressure control keeps running after the bubble', () => {
  const s = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  const c = { ...COLD_CTL };
  run(s, c, 6 * 3600, 1);
  assert.equal(s.d.mode, 'two');
  assert.equal(s.f.Q, 1.8e6, 'heaters stay where the operator left them');
  // letdown cannot hold pressure with a steam space, winds up to full flow and drains the level
  near(s.f.ld, 20, 0.01, 'letdown');
  assert.ok(s.d.level < 0.6, `level should fall when nobody switches to level control, got ${s.d.level}`);
});

test('net heat control: negative value cools the coolant', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  run(s, { ...HOT_CTL, heat: -13.5 }, 1800, 0.5);
  assert.ok(C(s.Tr) < 291.7 - 5, `coolant at ${C(s.Tr)} °C after 30 min`);
});

test('relief valve opened by hand depressurises and closes again', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  const c = { ...HOT_MAN, relMan: true };
  run(s, c, 60, 0.25);
  assert.ok(s.rel && s.f.wr > 10, 'relief open with steam flow');
  assert.ok(MPa(s.d.P) < 15.3, `pressure fell to ${MPa(s.d.P)}`);
  c.relMan = false; run(s, c, 1, 0.25);
  assert.equal(s.rel, false);
});

test('automatic relief opens at the configured pressure', () => {
  for (const set of [15.8e6, 16.6e6]) {
    const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
    const c = { ...HOT_MAN, q: 1800, relSet: set };
    let Popen = null, Pmax = 0;
    run(s, c, 3600, 0.25, x => { Pmax = Math.max(Pmax, x.d.P); if (Popen === null && x.rel) Popen = x.d.P; });
    assert.ok(Popen, 'relief never opened');
    near(MPa(Popen), MPa(set), 0.02, 'opening pressure');
    assert.ok(Pmax < set + 0.1e6, `overshoot to ${MPa(Pmax)} MPa`);
  }
});
