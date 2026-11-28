// Loads the model headless in Node, with no DOM: common.js (steam tables, limits), sim.js (plant)
// and control.js (automatic control), each an ES module, exposed here as namespaces.
import * as common from '../common.js';
import * as phys from '../sim.js';
import * as ctrl from '../control.js';

export { common, phys, ctrl };

// Controller memory for a plant state, created on first use, so a test can keep passing the
// same `s` to run() the way the page keeps its plant and controller side by side.
const ctls = new WeakMap();
export const ctlOf = s => { if (!ctls.has(s)) ctls.set(s, ctrl.init()); return ctls.get(s); };
export const engage = (s, c) => ctrl.engage(ctlOf(s), c);

// Run plant + controller for `seconds` with settings c (panel units, see control.js);
// optional onStep(s) hook, return false to stop.
export function run(s, c, seconds, dt = 1, onStep) {
  const k = ctlOf(s), n = Math.round(seconds / dt);
  for (let i = 0; i < n; i++) {
    phys.step(s, ctrl.update(k, s, c, dt), dt);
    if (onStep && onStep(s) === false) break;
    if (s.d.err) break;
  }
  return s;
}

// Run the plant alone with a fixed SI command u (see sim.js step), no controller involved.
export function runPlant(s, u, seconds, dt = 1) {
  const n = Math.round(seconds / dt);
  for (let i = 0; i < n && !s.d.err; i++) phys.step(s, u, dt);
  return s;
}

export const C = T => T - 273.15;
export const MPa = p => p / 1e6;
