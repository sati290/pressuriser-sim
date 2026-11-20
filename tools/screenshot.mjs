#!/usr/bin/env node
// Render index.html (with its scripts) headless at three widths × light/dark and report JS errors /
// horizontal overflow. Exits 1 if any page has either.
//
// Usage:  node tools/screenshot.mjs [--wait SECONDS] [--speed 600] [--out shots/] [--chrome PATH]
// Needs:  Playwright for Node with a Chromium it can find: `npm i -D playwright && npx playwright
//         install chromium`, or a global install on NODE_PATH, or --chrome /path/to/chrome.
import { createRequire } from 'node:module';
import { mkdirSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { parseArgs } from 'node:util';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const { values: a } = parseArgs({ options: {
  wait: { type: 'string', default: '4' },      // seconds to let the sim run before the shot
  speed: { type: 'string', default: '500' },   // which speed button to press (1…1000)
  out: { type: 'string', default: join(ROOT, 'shots') },
  chrome: { type: 'string' },                  // path to a Chrome/Chromium executable
} });

// require() rather than import: ESM resolution ignores NODE_PATH, which is where a global
// Playwright usually lives
let chromium;
try { ({ chromium } = createRequire(import.meta.url)('playwright')); }
catch { console.error('Playwright not found: npm i -D playwright && npx playwright install chromium'); process.exit(2); }

mkdirSync(a.out, { recursive: true });
const url = pathToFileURL(join(ROOT, 'index.html')).href;
const b = await chromium.launch(a.chrome ? { executablePath: a.chrome } : {});
let ok = true;
for (const [w, h, name] of [[1400, 1100, 'wide'], [720, 1400, 'mid'], [390, 1600, 'phone']]) {
  for (const theme of ['light', 'dark']) {
    const pg = await b.newPage({ viewport: { width: w, height: h }, colorScheme: theme });
    const errs = [];
    pg.on('pageerror', e => errs.push(String(e)));
    pg.on('console', m => { if (m.type() === 'error') errs.push(m.text()); });
    await pg.goto(url);
    await pg.click(`#speed [data-speed="${a.speed}"]`);
    await pg.waitForTimeout(+a.wait * 1000);
    const file = `${name}-${theme}.png`;
    await pg.screenshot({ path: join(a.out, file), fullPage: name !== 'wide' });
    const sw = await pg.evaluate('document.documentElement.scrollWidth');
    const overflow = sw > w;
    if (errs.length || overflow) ok = false;
    console.log(`${file.padEnd(18)} errors=${errs.length ? JSON.stringify(errs) : 'none'}  scrollWidth=${sw}${overflow ? '  OVERFLOW' : ''}`);
    await pg.close();
  }
}
await b.close();
process.exit(ok ? 0 : 1);
