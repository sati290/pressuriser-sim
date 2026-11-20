# Pressuriser simulator — project context for Claude Code

Interactive, single-file HTML simulator of a PWR pressuriser coupled to a lumped reactor
coolant system (RCS). It is a sandbox: **manual control is primary**; automatic controllers are
optional assists the operator switches on, and they never change mode by themselves. Built to explore how pressure, temperature and level relate, and to run
startup (water-solid → bubble draw → hot standby) and shutdown (cooldown → refill solid).

The owner is iterating on it for understanding the physics, not for licensing-grade analysis.
Physical correctness and honest labelling of limitations matter more than features.

## Files

| Path | What |
|---|---|
| `index.html` | The whole app: CSS, markup, physics, UI. Must stay one self-contained file (it is opened straight from GitHub via raw.githack; only Google Fonts loads externally). |
| `tests/load-physics.mjs` | Extracts the code between `/*PHYS*/` and `/*END*/` from `index.html` and evaluates it in Node. |
| `tests/physics.test.mjs` | Regression tests (node:test, no deps). ~7 s. |
| `tools/screenshot.py` | Playwright: renders 3 widths × light/dark, fails on JS errors or horizontal overflow. |
| `docs/physics.md` | Model equations, state variables, controllers, constants, validation numbers. |
| `docs/backlog.md` | Ideas for next iterations, roughly prioritised. |

## Commands

```bash
npm test                      # physics regression suite
npm run shots                 # screenshots into shots/ (needs: pip install playwright && playwright install chromium)
python3 tools/screenshot.py --chrome /usr/bin/google-chrome   # use an existing Chrome
npm run serve                 # http://localhost:8000 — or just open index.html
```

Always run `npm test` after touching anything inside `/*PHYS*/ … /*END*/`, and the screenshot
tool after any CSS/markup/drawing change.

## Code structure inside index.html

The `<script>` is one block `{ … }` (no globals leak). Order:

1. `/*PHYS*/ … /*END*/` — **pure physics, no DOM.** Everything the tests use lives here and must
   stay DOM-free so `load-physics.mjs` can evaluate it. If you add a function the tests need,
   also add it to the `return {…}` list in `tests/load-physics.mjs`.
   - `satX(T)` exact Wagner–Pruss saturation equations → `[p, dp/dT, ρf, ρg, hf, hg]` (SI).
   - `sat(T)` fast linear interpolation in a 0.02 K table built from `satX`; returns a **shared,
     reused object `Q`** — copy fields out before calling `sat` again.
   - `kap(°C)` approximate isothermal compressibility of liquid (1/Pa).
   - `pz(M,U,V)` pressuriser state from mass, internal energy, available volume → solid
     (compressed liquid) or two-phase (saturated mixture); returns `{mode,T,P,x,level,…}` or `{err}`.
   - `rcsP(Mr,Tr)` RCS pressure from mass and temperature (compressed liquid).
   - `init(o)` builds a state; `step(s,c,dt)` advances one step (see docs/physics.md).
2. DOM wiring: `ids`/`checks`/`fmt`/`readCtl()` map controls → `ctl` (SI units except where noted:
   `q`, `loss` kW; `heat` MW, signed net heat to coolant; flows kg/s; `set`, `relSet` stored in Pa;
   `inv` is `'man'|'pres'|'lev'`; booleans `aP`, `relAuto`, `relMan`). Manual controls sit in the
   right-hand card; assists in the "Automatic assists" section below the main panel.
3. `physTick()` runs `step` repeatedly, halts on invalid states, records `hist` every 2 sim-s.
   No solid↔two-phase side effects: the operator switches modes.
4. `syncAuto()` greys out and back-fills sliders that an assist is driving, and writes the assist
   status line (warns when the chosen mode no longer suits the pressuriser state).
5. Renderers: `updText()` (gauges + stats), `drawRec()` (SVG recorder), `drawSat()` (P–T plane SVG),
   `drawVessel()` (canvas mimic with particles). Charts redraw at ~7 Hz; canvas every frame.
6. `frame()` rAF loop. Sim time step depends on speed: `ctl_dt()` = 0.25/0.5/1/2 s.

## Conventions

- SI units internally (K, Pa, kg, J). Convert only at display.
- Canvas colours must be resolved via the hidden `probe` element (`css('--var')`) — canvas
  can't parse `light-dark()`.
- Theme: page must read in light, dark, and unset (`data-theme` on root set by the viewer).
- Layout must not overflow at 390 px; phone layout stacks with sticky gauges.
- Design language: "precision instrument" — Geist, aluminium ground, black ink, one orange
  accent (#ff4f00) for pressure / pointer, water blue. No new colours without reason.
- Keep user-facing copy plain and specific; state limitations next to the feature they affect.

## Known limitations (keep the notes section in the page in sync)

- Thermal equilibrium in the saturated region; only insurge stratification is modelled (a
  cold layer mixing with τ = 300 s). No outsurge flashing delay, no steam superheat on
  fast compression, no wall heat capacity in the pressuriser.
- RCS is one well-mixed liquid node; no loops, no hot/cold leg split, no SG secondary side.
- Charging enters at RCS temperature (no cooling effect). Spray water is at RCS temperature.
- Level is liquid volume fraction (straight-cylinder geometry).
- Compressibility table is approximate (± ~10 %).
- Coolant pumps are not modelled: one signed net-heat control; pumps assumed running.
- Relief valve flow is a rough scaling (steam ∝ P, liquid ∝ √ΔP), not a valve model.
- Generic 4-loop Westinghouse-like numbers; not a VVER.

## Publishing

The repo is public; the page is opened straight from GitHub through raw.githack, which serves
`index.html` from any branch or commit with the right content type. There is no build or deploy step:
pushing a branch is publishing it.

- Branch: `https://raw.githack.com/sati290/pressuriser-sim/<branch>/index.html`
  (slashes in the branch name stay as-is, e.g. `.../claude/some-branch/index.html`).
  raw.githack caches branch URLs for a few minutes, so a fresh push may take a moment to show.
- Exact commit: `https://raw.githack.com/sati290/pressuriser-sim/<commit-sha>/index.html`.
- `main`: `https://raw.githack.com/sati290/pressuriser-sim/main/index.html`.

The old claude.ai artifact is no longer the live version; don't republish there.

**When you finish a task that changed `index.html`, commit and push it, then end your reply with
the raw.githack URL for the branch you pushed** (and the commit URL if the owner may want to compare
versions).
