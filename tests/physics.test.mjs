// Physics regression tests. Run: node tests/physics.test.mjs  (or npm test)
// Uses node:test; no dependencies.
import test from 'node:test';
import assert from 'node:assert/strict';
import { common, phys, ctrl, ctlOf, engage, run, runPlant, C, MPa } from './load-physics.mjs';

const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, expected ${b} ± ${tol}`);

// Control settings (panel units, see control.js). run() passes them through PZCtl.update to get the
// command for PZSim.step. Every actuator (ch, ld, q, sp, rel) is manual unless c.auto[k] is true; then it follows
// the demand the automatic logic computes. inv picks the inventory logic: 'pres' (letdown holds
// pressure) or 'lev' (charging holds level). Nothing switches by itself; tests that cross
// solid <-> two-phase switch like an operator would.
const RELIEF_COLD = { relSet: 3.6e6, relMan: 0 };
const RELIEF_HOT  = { relSet: 16.2e6, relMan: 0 };
const BUBBLE_AUTO = () => ({ q: true, sp: true, ch: true, ld: true, rel: true });
const SOLID_AUTO  = () => ({ ld: true, rel: true });
const COLD_CTL = { auto: SOLID_AUTO(), inv: 'pres', set: 2.8e6, lset: 25, heat: 0, ch: 5, ld: 5, q: 1800, sp: 0, amb: 40, ...RELIEF_COLD };
const HOT_CTL  = { auto: BUBBLE_AUTO(), inv: 'lev', set: 15.5e6, lset: 25, heat: 0, ch: 6, ld: 6, q: 200, sp: 0, amb: 40, ...RELIEF_HOT };
const HOT_MAN  = { auto: { rel: true }, inv: 'lev', set: 15.5e6, lset: 25, heat: 0, ch: 6, ld: 6, q: 60, sp: 0, amb: 40, ...RELIEF_HOT };
// operator hands over from solid to bubble control: everything to auto, charging on level
// (letdown in auto holds its flow and takes over when charging reaches zero)
const toBubble = (s, c) => { c.ld = s.f.ld; c.auto = BUBBLE_AUTO(); c.inv = 'lev'; engage(s, c); };
const toSolid  = (s, c) => { c.ld = s.f.ld; c.ch = s.f.ch; c.auto = SOLID_AUTO(); c.q = 0; c.inv = 'pres'; engage(s, c); };

test('saturation properties match IAPWS steam tables', () => {
  // [°C, p MPa, rho_f, rho_g, h_f kJ/kg, h_g kJ/kg]
  const ref = [
    [100, 0.101418, 958.35, 0.5976, 419.17, 2675.6],
    [200, 1.5551, 864.66, 7.8610, 852.27, 2792.0],
    [300, 8.5879, 712.14, 46.168, 1344.8, 2749.6],
    [344.8, 15.502, 594.2, 101.9, 1630.0, 2596.3],
  ];
  for (const [c, p, rf, rg, hf, hg] of ref) {
    const [P, , RF, RG, HF, HG] = common.satX(c + 273.15);
    near(MPa(P), p, p * 0.002, `p_sat(${c})`);
    near(RF, rf, rf * 0.002, `rho_f(${c})`);
    near(RG, rg, rg * 0.003, `rho_g(${c})`);
    near(HF / 1e3, hf, 1.5, `h_f(${c})`);
    near(HG / 1e3, hg, 1.5, `h_g(${c})`);
  }
});

test('tabulated sat() agrees with the exact equations', () => {
  for (let c = 20; c < 370; c += 7.3) {
    const T = c + 273.15, ex = common.satX(T), q = common.sat(T);
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
  const c = { ...COLD_CTL, auto: BUBBLE_AUTO(), inv: 'lev', heat: 19.5, ...RELIEF_HOT };
  c.set = 8e6; run(s, c, 3 * 3600, 1);
  c.set = 15.5e6; run(s, c, 4 * 3600, 1, x => C(x.Tr) < 291.7);
  assert.ok(C(s.Tr) >= 291, `coolant reached ${C(s.Tr)} °C`);
  assert.ok(common.Tsat(s.Pr) - s.Tr > 20, 'subcooling margin below 20 K');
  near(s.d.level * 100, 25, 3, 'level');
});

test('refill to water-solid at low temperature hands control back to letdown', () => {
  const s = phys.init({ solid: false, Tr: 150, P: 2.8e6, L: 0.25 });
  const c = { ...COLD_CTL, auto: BUBBLE_AUTO(), inv: 'lev', lset: 100, heat: -1 };
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
  const c = { ...HOT_CTL, auto: { q: true, sp: true, rel: true }, ch: 0, ld: 0 };
  const m0 = s.M + s.Ms + s.Mr;
  s.extra = { Q: 40e6, end: 120 };
  run(s, c, 600, 0.5);
  const m1 = s.M + s.Ms + s.Mr;
  near(m1, m0, 1, 'total mass (kg)');
});

test('hot standby with every control manual is steady when heaters match losses', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  run(s, HOT_MAN, 3600, 0.5);
  near(MPa(s.d.P), 15.5, 0.05, 'P after 1 h');
  near(s.d.level * 100, 25, 0.5, 'level after 1 h');
});

test('automatic control does not hand over by itself: letdown pressure control keeps running after the bubble', () => {
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
  const c = { ...HOT_MAN, auto: {}, relMan: 100 };
  run(s, c, 60, 0.25);
  assert.ok(s.rel === 1 && s.f.wr > 10, 'relief fully open with steam flow');
  assert.ok(MPa(s.d.P) < 15.3, `pressure fell to ${MPa(s.d.P)}`);
  c.relMan = 25; run(s, c, 1, 0.25);
  near(s.f.wr, 0.25 * phys.relW(s.d.P, true), 0.5, 'flow at 25 % opening');
  c.relMan = 0; run(s, c, 1, 0.25);
  assert.equal(s.rel, 0);
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

test('fault() stops the model when the coolant reaches saturation', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  assert.equal(phys.fault(s), '');
  let f = '';
  run(s, { ...HOT_MAN, q: 0, relMan: 100, auto: {} }, 3600, 0.5, x => (f = phys.fault(x)) === '');
  assert.equal(f, 'boiling');
});

test('demands are computed while manual and applied only in auto', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  // all manual, pressure below setpoint: the heater demand rises but the heaters stay where they are
  const c = { ...HOT_MAN, auto: {}, set: 15.8e6 };
  run(s, c, 10, 0.5);
  near(s.f.Q, 60e3, 1, 'manual heaters unchanged');
  assert.ok(ctlOf(s).dem.q > 1.5e6, `heater demand ${ctlOf(s).dem.q}`);
  c.auto = { q: true }; run(s, c, 0.5, 0.5);
  near(s.f.Q, ctlOf(s).dem.q, 1, 'heaters follow the demand in auto');
  // relief demand latches open above relSet even with the valve in manual; the valve stays shut
  const r = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  run(r, { ...HOT_MAN, auto: {}, relSet: 15e6 }, 1, 0.5);
  assert.equal(ctlOf(r).dem.rel, 1); assert.equal(r.rel, 0);
});

test('level control, both in A: charging controls, letdown is held and takes over only when charging reaches zero', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.40 });
  const c = { ...HOT_CTL, ch: 2, ld: 2 };
  engage(s, c);
  run(s, c, 60, 0.5);
  near(s.f.ch, 0, 1e-9, 'charging at its limit');
  assert.ok(s.f.ld > 2, `letdown should rise above 2 kg/s, got ${s.f.ld}`);
  run(s, c, 3600, 0.5);
  near(s.d.level * 100, 25, 0.5, "level settles");
  near(s.f.ld, 2, 0.05, "letdown back to the flow it held");
  assert.ok(s.d.level < 0.40, 'level falling toward setpoint');
});

test('manual integrator does not wind past the actuator range', () => {
  const s = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  // letdown manual far from what the pressure controller wants: the demand saturates and stays bounded
  run(s, { ...COLD_CTL, auto: { rel: true }, q: 0, ld: 5, ch: 5, set: 3.2e6 }, 600, 1);
  near(ctlOf(s).dem.ld, 0, 1e-9, 'letdown demand at its limit');
  assert.ok(Math.abs(ctlOf(s).iL) < 5.1, `integrator held near the limit, got ${ctlOf(s).iL}`);
});

test('ΔT mode: heaters and spray hold pressuriser − coolant ΔT, also while cooling down', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  const c = { ...HOT_CTL, pzMode: 'dT', dT: 53 };
  const margin = x => x.d.T - x.Tr;
  run(s, c, 1800, 0.5);
  near(margin(s), 53, 1, 'ΔT at hot standby');
  near(MPa(s.d.P), 15.5, 0.15, 'default 53 K reproduces 15.5 MPa at 291.7 °C');
  c.heat = -13.5; let lo = 99, hi = 0;
  run(s, c, 2 * 3600, 1, x => { if (x.t > 2400) { lo = Math.min(lo, margin(x)); hi = Math.max(hi, margin(x)); } });
  assert.ok(C(s.Tr) < 240, `coolant cooled to ${C(s.Tr)} °C`);
  assert.ok(lo > 50 && hi < 56, `ΔT during cooldown ${lo.toFixed(1)}–${hi.toFixed(1)} K`);
});

test('ΔT mode water-solid: heaters warm the pressuriser to coolant + ΔT and stop there', () => {
  const s = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  run(s, { ...COLD_CTL, auto: { ...SOLID_AUTO(), q: true, sp: true }, pzMode: 'dT', dT: 53 }, 6 * 3600, 1);
  assert.equal(s.d.mode, 'solid');
  near(C(s.d.T) - C(s.Tr), 53, 1.5, 'pressuriser − coolant');
  near(MPa(s.d.P), 2.8, 0.1, 'letdown still holds the pressure setpoint');
});

test('inventory: a single flow in auto does the controlling, whichever the mode prefers', () => {
  // level mode, only letdown in A: letdown draws the level down to the setpoint, charging stays manual
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.35 });
  const c = { ...HOT_CTL, auto: { q: true, sp: true, ld: true, rel: true }, ch: 6, ld: 6 };
  engage(s, c);
  run(s, c, 3 * 3600, 0.5);
  near(s.d.level * 100, 25, 0.5, 'level held by letdown');
  near(s.f.ch, 6, 1e-9, 'charging untouched');
  // pressure mode, only charging in A: charging holds the water-solid pressure
  const w = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  const cw = { ...COLD_CTL, auto: { ch: true, rel: true }, q: 0, heat: 2, ch: 5, ld: 5 };
  engage(w, cw);
  run(w, cw, 3600, 1);
  near(MPa(w.d.P), 2.8, 0.05, 'pressure held by charging');
  assert.ok(w.f.ch < 5, `charging should fall below letdown to offset the expansion, got ${w.f.ch}`);
  near(w.f.ld, 5, 1e-9, 'letdown untouched');
});

// plant alone, fixed SI commands, no controller

const HOLD = { ch: 0, ld: 0, q: 0, sp: 0, rel: 0, heat: 0 };

test('plant alone: closed system conserves mass with heaters, spray and a load step', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  const m0 = s.M + s.Ms + s.Mr;
  s.extra = { Q: 40e6, end: 120 };
  runPlant(s, { ...HOLD, q: 1e6, sp: 10 }, 600, 0.5);
  near(s.M + s.Ms + s.Mr, m0, 1, 'total mass (kg)');
});

test('plant clamps commands to the actuator limits and trips the heaters at low level', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  runPlant(s, { ...HOLD, ch: 50, ld: -3, q: 5e6, sp: 99, rel: 2 }, 0.5, 0.5);
  assert.deepEqual([s.f.ch, s.f.ld, s.f.Q, s.f.ws, s.rel], [common.LIM.ch, 0, common.LIM.q, common.LIM.sp, 1]);
  const low = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.10 });
  runPlant(low, { ...HOLD, q: 1e6 }, 0.5, 0.5);
  assert.ok(low.f.trip && low.f.Q === 0, 'heaters tripped below 12 %');
});

test('controller command: demand in A, operator value in M, converted to SI', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  const k = ctrl.init(), c = { ...HOT_MAN, auto: { sp: true }, q: 600, relMan: 40, heat: 2.5, amb: 40 };
  const u = ctrl.update(k, s, c, 0.5);
  assert.deepEqual(u, { ch: 6, ld: 6, q: 600e3, sp: k.dem.sp, rel: 0.4, n2: 0, heat: 2.5e6, amb: 313.15 });
  assert.equal(ctrl.update(k, s, { ...c, n2: 40 }, 0.5).n2, 0.4, 'nitrogen valve is always the operator\'s');
});

// nitrogen cushion: the gas space holds nitrogen and water vapour (Dalton), same pz() as steam and solid

const N2_COLD = { n2: true, Tpz: 60, Tr: 60, P: 2.8e6, L: 0.25 };
const N2_CTL = { auto: { ch: true, ld: true, rel: true }, inv: 'lev', set: 2.8e6, lset: 25, heat: 0, ch: 5, ld: 5, q: 0, sp: 0, amb: 40, relSet: 3.6e6, relMan: 0, n2: 0 };

test('nitrogen cushion round-trips through init', () => {
  const s = phys.init(N2_COLD);
  assert.equal(s.d.mode, 'two');
  near(MPa(s.d.P), 2.8, 1e-6, 'P');
  near(s.d.level, 0.25, 1e-6, 'level');
  near(C(s.d.T), 60, 1e-4, 'T');
  near(MPa(s.d.pS), 0.0199, 0.0005, 'steam partial pressure is p_sat(60 °C)');
  near(s.Mn, 2.78e6 * 0.75 * phys.VP / (phys.RN * 333.15), 1, 'nitrogen mass, ideal gas');
});

test('the nitrogen branch tends to the steam and water-solid states as nitrogen goes to zero', () => {
  for (const o of [{ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 }, { solid: false, Tr: 200, P: 2.8e6, L: 0.6 }, { solid: true, Tpz: 60, Tr: 60, P: 2.8e6 }, { solid: true, Tpz: 220, Tr: 200, P: 3e6 }]) {
    const s = phys.init(o), d = s.d, e = 1e-6, n = phys.pz(s.M, s.U + e * phys.CVN * d.T, phys.VP, e);
    near(n.P, d.P, 20, `P, ${d.mode} at ${MPa(d.P)} MPa`);
    near(n.T, d.T, 1e-3, `T, ${d.mode}`);
    near(n.level, d.level, 1e-4, `level, ${d.mode}`);
  }
});

test('nitrogen cushion, cold, everything balanced: steady for an hour', () => {
  const s = phys.init(N2_COLD);
  run(s, { ...N2_CTL, auto: { rel: true }, q: phys.UA * 20 / 1e3 }, 3600, 1); // heaters match the loss to 40 °C air
  near(MPa(s.d.P), 2.8, 0.01, 'P');
  near(s.d.level * 100, 25, 0.1, 'level');
});

test('closed system with nitrogen conserves water and nitrogen; relief vents both in proportion', () => {
  const s = phys.init({ ...N2_COLD, Tpz: 150, Tr: 150 });
  const w0 = s.M + s.Ms + s.Mr, n0 = s.Mn;
  runPlant(s, { ...HOLD, q: 1e6, sp: 5, heat: 2e6 }, 1800, 0.5);
  near(s.M + s.Ms + s.Mr, w0, 1, 'water (kg)');
  near(s.Mn, n0, 1e-6, 'nitrogen (kg)');
  // open the relief: what leaves is the gas mixture, nitrogen by its mass share
  let wOut = 0, nOut = 0; const w1 = s.M + s.Ms + s.Mr, n1 = s.Mn;
  for (let i = 0; i < 240; i++) { phys.step(s, { ...HOLD, rel: 1 }, 0.5); wOut += (s.f.wr - s.f.wrn) * 0.5; nOut += s.f.wrn * 0.5; }
  assert.ok(nOut > 50 && wOut > 10, `vented ${nOut.toFixed(0)} kg nitrogen, ${wOut.toFixed(0)} kg steam`);
  near(s.M + s.Ms + s.Mr, w1 - wOut, 1, 'water balance with venting');
  near(s.Mn, n1 - nOut, 1e-6, 'nitrogen balance with venting');
});

test('relief flow: unchanged for steam, ideal-gas choked ratio for nitrogen', () => {
  const st = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  assert.equal(phys.gasF(st.d), 1);
  const n = phys.init(N2_COLD), f = phys.gasF(n.d);
  near(f, 1.52, 0.03, 'cold nitrogen at 2.8 MPa vs saturated steam at 2.8 MPa');
});

test('nitrogen supply: flows below the header pressure through a check valve', () => {
  near(phys.n2W(2.8e6), 0.5 * Math.sqrt(1.2), 1e-9, 'flow at 2.8 MPa');
  assert.equal(phys.n2W(5e6), 0, 'no backflow above the header');
  const t = phys.init(N2_COLD);
  runPlant(t, { ...HOLD, n2: 0.25 }, 0.5, 0.5);
  near(t.f.wn, 0.25 * phys.n2W(t.d.P), 0.01, 'flow scales with the valve opening');
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  runPlant(s, { ...HOLD, n2: 1 }, 10, 0.5);
  assert.equal(s.Mn, 0, 'no nitrogen enters at hot standby pressure');
});

test('nitrogen into a water-solid pressuriser opens a gas space; letdown holds the pressure', () => {
  const s = phys.init({ solid: true, Tpz: 60, Tr: 60, P: 2.8e6 });
  const c = { ...COLD_CTL, q: 0, n2: 100 };
  engage(s, c);
  let Pmax = 0;
  // the operator shuts the supply once the level is down to 40 %
  run(s, c, 2 * 3600, 1, x => { Pmax = Math.max(Pmax, x.d.P); if (x.d.level < 0.4) c.n2 = 0; });
  assert.equal(s.d.mode, 'two');
  assert.ok(s.Mn > 500, `nitrogen in: ${s.Mn.toFixed(0)} kg`);
  // after the valve shuts, letdown keeps drawing water until the gas has expanded back to the setpoint
  assert.ok(s.d.level > 0.25 && s.d.level < 0.4, `letdown made room for the gas: level ${s.d.level}`);
  // while the supply was open, nitrogen came in faster (by volume) than full letdown could make room
  assert.ok(MPa(Pmax) > 2.9 && MPa(Pmax) < 3.6, `pressure rose to ${MPa(Pmax)} MPa`);
  near(MPa(s.d.P), 2.8, 0.05, 'pressure back at the setpoint, the gas a soft spring');
  assert.ok(s.d.yN > 0.99, 'cold gas space is almost all nitrogen');
});

test('nitrogen startup: heaters plus venting replace nitrogen with steam; the gas becomes steam alone near Tsat', () => {
  const s = phys.init(N2_COLD);
  const c = { ...N2_CTL, q: 1800, relSet: 3.0e6 };
  engage(s, c);
  let at = null, jump = 0, prev = s.d.P, Pmax = 0;
  run(s, c, 4 * 3600, 1, x => {
    if (at === null && x.Mn === 0) { at = x.t; jump = Math.abs(x.d.P - prev); }
    prev = x.d.P; Pmax = Math.max(Pmax, x.d.P);
  });
  assert.ok(at, 'nitrogen never purged');
  assert.ok(at < 3.5 * 3600, `took ${(at / 3600).toFixed(2)} h`);
  assert.ok(jump < 0.01e6, `pressure step at the switch ${MPa(jump)} MPa`);
  near(C(s.d.T), C(common.Tsat(s.d.P)), 1e-6, 'steam alone: saturated');
  assert.ok(MPa(Pmax) < 3.1, `relief held pressure, max ${MPa(Pmax)} MPa`);
  near(s.d.level * 100, 25, 1, 'level held by charging');
});

test('nitrogen cooldown: spray cools the pressuriser, nitrogen from the line replaces condensed steam', () => {
  const s = phys.init({ solid: false, Tr: 175, P: 2.8e6, L: 0.25 });
  const c = { ...N2_CTL, heat: -1, sp: 4 };
  engage(s, c);
  let Pmin = 1e9;
  run(s, c, 4 * 3600, 1, x => {
    if (x.d.P < 2.75e6) c.n2 = 100; else if (x.d.P > 2.85e6) c.n2 = 0; // operator works the valve
    if (x.t > 600) Pmin = Math.min(Pmin, x.d.P);
  });
  assert.ok(s.d.T - s.Tr < 2, `pressuriser cooled to the coolant: ΔT ${(s.d.T - s.Tr).toFixed(1)} K`);
  assert.ok(s.d.yN > 0.6, `gas mostly nitrogen: ${(s.d.yN * 100).toFixed(0)} %`);
  assert.ok(MPa(Pmin) > 2.7, `pressure held, min ${MPa(Pmin)} MPa`);
  assert.equal(phys.fault(s), '');
});

test('relief flow stops at the relief-tank back pressure (pressures are absolute)', () => {
  assert.equal(phys.relW(phys.PB, true), 0, 'no steam flow at atmospheric');
  assert.equal(phys.relW(0.05e6, true), 0, 'no backflow below it');
  assert.equal(phys.relW(phys.PB, false), 0, 'no liquid flow at atmospheric');
  assert.equal(phys.subF(0.5e6), 1, 'choked well above the critical ratio');
  const f = phys.subF(0.15e6);
  assert.ok(f > 0.5 && f < 1, `subsonic between, factor ${f}`);
  // a hot bubble vented with the valve wide open ends at atmospheric, not at zero
  const s = phys.init({ solid: false, Tr: 90, P: 0.3e6, L: 0.5 });
  runPlant(s, { ch: 0, ld: 0, q: 0, sp: 0, rel: 1, heat: 0 }, 3600, 0.5);
  assert.ok(s.d.P > phys.PB - 2e3, `vented down to ${MPa(s.d.P)} MPa`);
});

test('pressuriser heat loss is UA·(T − ambient)', () => {
  const s = phys.init({ solid: false, Tr: 291.7, P: 15.5e6, L: 0.25 });
  const T = s.d.T;
  phys.step(s, { ch: 0, ld: 0, q: 0, sp: 0, rel: 0, heat: 0, amb: 30 + 273.15 }, 0.5);
  near(s.f.loss, phys.UA * (T - 303.15), 1, 'loss at hot standby, 30 °C air');
  assert.ok(s.f.loss > 50e3 && s.f.loss < 70e3, `hot standby loss ${s.f.loss / 1e3} kW`);
});
