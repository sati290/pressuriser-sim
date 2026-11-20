// Loads the physics block (between /*PHYS*/ and /*END*/) out of index.html
// so it can be exercised headless in Node, with no DOM.
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const html = readFileSync(join(here, '..', 'index.html'), 'utf8');
const start = html.indexOf('/*PHYS*/'), end = html.indexOf('/*END*/');
if (start < 0 || end < 0) throw new Error('PHYS markers not found in index.html');
const code = html.slice(start + 8, end);

export const phys = new Function(
  code + '\nreturn {satX,sat,Tsat,kap,pz,liqRho,rcsP,hLiq,cpL,init,step,VP,VR,CM,TAU,T0,THI};'
)();

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
