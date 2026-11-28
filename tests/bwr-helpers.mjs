// Shared helpers for the BWR test files (not a test file itself).
import assert from 'node:assert/strict';
import * as common from '../common.js';
import * as B from '../bwr-sim.js';
import * as K from '../bwr-control.js';
export { common, B, K };
export const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: got ${a}, expected ${b} ± ${tol}`);
export const MPa = p => p / 1e6;
// fixed command, no controller; stops at a fault
export const runPlant = (s, u, seconds, dt = 1, onStep) => {
  for (let i = 0; i < Math.round(seconds / dt); i++) {
    B.step(s, typeof u === 'function' ? u(s) : u, dt); if (B.fault(s)) break; if (onStep && onStep(s) === false) break;
  }
  return s;
};
// plant + controller; c in panel units (see bwr-control.js)
export const run = (s, k, c, seconds, dt = 1, onStep) => {
  for (let i = 0; i < Math.round(seconds / dt); i++) {
    B.step(s, K.update(k, s, c, dt), dt); if (B.fault(s)) break; if (onStep && onStep(s) === false) break;
  }
  return s;
};
// rated conditions: 3293 MW, 7.0 MPa, 216 °C feedwater; steam flow from the heat balance
export const P0 = 7e6, Q0 = 3293e6, TFW = 216 + 273.15;
export const W0 = Q0 / (common.sat(common.Tsat(P0)).hg - common.hLiq(TFW, P0));
export const FV0 = B.fwValve(W0, 1, P0);
export const rated = () => B.init({ P: P0, L: 14.3, gen: W0, mv: W0 / B.WMS, fv: FV0, n: 1, tfw: 216 });
export const RATED_U = { heat: Q0, n: 1, fv: FV0, ext: 1, mk: 0, mv: W0 / B.WMS, srv: [] };
export const NOFEED = { n: 0, fv: 0, ext: 0, mk: 0 };
export const AUTO = () => ({ n: true, fv: true, mv: true, mk: true });
export const RATED_C = () => ({ heat: 3293, n: 100, fv: FV0 * 100, ext: 100, mk: 0, mv: W0 / B.WMS * 100, srv: [], pset: 7e6, lset: 14.3, dpset: 2.2e6, auto: AUTO() });
