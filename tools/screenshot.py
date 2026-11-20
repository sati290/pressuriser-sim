#!/usr/bin/env python3
"""Render index.html headless at three widths and report JS errors / horizontal overflow.

Usage:  python3 tools/screenshot.py [--wait SECONDS] [--speed 600] [--out shots/]
Needs:  pip install playwright && playwright install chromium
        (or pass --chrome /path/to/chrome to use an existing Chrome)
"""
import argparse, pathlib, sys
from playwright.sync_api import sync_playwright

ROOT = pathlib.Path(__file__).resolve().parent.parent
p = argparse.ArgumentParser()
p.add_argument('--wait', type=float, default=4, help='seconds to let the sim run before the shot')
p.add_argument('--speed', default='600', help='value for the #speed select')
p.add_argument('--out', default=str(ROOT / 'shots'))
p.add_argument('--chrome', default=None, help='path to a Chrome/Chromium executable')
a = p.parse_args()
out = pathlib.Path(a.out); out.mkdir(exist_ok=True)

ok = True
with sync_playwright() as pw:
    b = pw.chromium.launch(executable_path=a.chrome) if a.chrome else pw.chromium.launch()
    for w, h, name in [(1400, 1100, 'wide'), (720, 1400, 'mid'), (390, 1600, 'phone')]:
        for theme in ('light', 'dark'):
            pg = b.new_page(viewport={'width': w, 'height': h}, color_scheme=theme)
            errs = []
            pg.on('pageerror', lambda e: errs.append(str(e)))
            pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None)
            pg.goto((ROOT / 'index.html').as_uri())
            pg.select_option('#speed', a.speed)
            pg.wait_for_timeout(int(a.wait * 1000))
            f = out / f'{name}-{theme}.png'
            pg.screenshot(path=str(f), full_page=(name != 'wide'))
            sw = pg.evaluate('document.documentElement.scrollWidth')
            overflow = sw > w
            ok &= not errs and not overflow
            print(f'{f.name:18} errors={errs or "none"}  scrollWidth={sw}{"  OVERFLOW" if overflow else ""}')
            pg.close()
    b.close()
sys.exit(0 if ok else 1)
