// Loads the simulation model (sim.js) headless in Node, with no DOM.
// sim.js is a plain script that defines globalThis.PZSim, so the browser can load it from file://.
import '../sim.js';

export const phys = globalThis.PZSim;

// Helper: run the model for `seconds` with fixed controls; optional onStep(s) hook.
export function run(s, c, seconds, dt = 1, onStep) {
  const n = Math.round(seconds / dt);
  for (let k = 0; k < n; k++) {
    phys.step(s, c, dt);
    if (onStep && onStep(s) === false) break;
    if (s.d.err) break;
  }
  return s;
}

export const C = T => T - 273.15;
export const MPa = p => p / 1e6;
