# Pressuriser simulator — project context for Claude Code

Interactive HTML simulator of a PWR pressuriser coupled to a lumped reactor
coolant system (RCS). It is a sandbox: **manual control is primary**; automatic controllers are
optional: each actuator has an M/A switch the operator sets, and nothing changes mode by itself. Built to explore how pressure, temperature and level relate, and to run
startup (water-solid → bubble draw → hot standby) and shutdown (cooldown → refill solid).

The owner is iterating on it for understanding the physics, not for licensing-grade analysis.
Physical correctness and honest labelling of limitations matter more than features.

## Files

| Path | What |
|---|---|
| `index.html` | Presentation: CSS, markup, UI wiring, renderers. Loads `common.js`, `sim.js`, `control.js` (in that order) with plain relative `<script src>`; nothing else external except Google Fonts. |
| `common.js` | What both plant and controller need: steam tables, `clamp`, actuator limits `LIM`. Defines `globalThis.PZCommon`. |
| `sim.js` | The plant: pure physics, no DOM, no automatic control. Defines `globalThis.PZSim`. |
| `control.js` | Automatic control: demands and the final command for the plant. Defines `globalThis.PZCtl`. Needs `common.js`, not `sim.js`. |
| `tests/load-physics.mjs` | Imports the three scripts in Node; exposes `PZCommon` as `common`, `PZSim` as `phys`, `PZCtl` as `ctrl`, plus `run` (plant + controller), `runPlant` (fixed command) and `ctlOf(s)`. Tests never load `index.html`. |
| `tests/physics.test.mjs` | Regression tests (node:test, no deps). ~7 s. |
| `tools/screenshot.mjs` | Playwright (Node): renders 3 widths × light/dark, fails on JS errors or horizontal overflow. |
| `tools/serve.mjs` | Dependency-free static server for `npm run serve` (port 8000, or `node tools/serve.mjs <port>`). |
| `docs/physics.md` | Model equations, state variables, controllers, constants, validation numbers. |
| `docs/backlog.md` | Ideas for next iterations, roughly prioritised. |

## Commands

```bash
npm test                      # physics regression suite
npm run shots                 # screenshots into shots/ (needs: npm i -D playwright && npx playwright install chromium)
node tools/screenshot.mjs --chrome /usr/bin/google-chrome   # use an existing Chrome
npm run serve                 # http://localhost:8000 — or just open index.html
```

Always run `npm test` after touching `common.js`, `sim.js` or `control.js`, and the screenshot
tool after any CSS/markup/drawing change.

## Code structure

### The three scripts

Each is one IIFE assigning one global; everything the page or the tests use is in its `return {…}`
list — add a new function there to make it available. They must stay plain scripts (no ES
`export`): browsers refuse module scripts from `file://`, and Node imports them as ES modules just
fine since they only touch `globalThis`. Dependencies: `sim.js` and `control.js` each read
`PZCommon` at load; neither reads the other. Anything both need goes into `common.js`.

### common.js — shared

- `satX(T)` exact Wagner–Pruss saturation equations → `[p, dp/dT, ρf, ρg, hf, hg]` (SI).
- `sat(T)` fast linear interpolation in a 0.02 K table built from `satX`; returns a **shared,
  reused object `Q`** — copy fields out before calling `sat` again.
- `Tsat(p)` saturation temperature by bisection; `clamp`; `LIM` actuator ranges in SI
  (`ch`, `ld` 20 kg/s, `q` 1.8 MW, `sp` 40 kg/s, `rel` 1). The plant clamps commands to `LIM`, and the
  controller's anti-windup uses the same limits.

### sim.js — the plant

**Pure physics, no DOM, no M/A, no setpoints.**
- `kap(°C)` approximate isothermal compressibility of liquid (1/Pa).
- `pz(M,U,V,Mn)` pressuriser state from water mass, internal energy, available volume, nitrogen mass →
  solid (compressed liquid) or two-phase (saturated mixture); with `Mn > 0` (`pzN`) liquid + a gas space
  of nitrogen and saturated vapour (Dalton), which tends to the other two as `Mn → 0`. Returns
  `{mode,T,P,x,level,pN,pS,yN,…}` or `{err}`. `mode` is `'two'` whenever there is a gas space; nitrogen
  is `s.Mn > 0`. Nitrogen below 0.1 % of the gas with the supply shut is dropped in `step`.
- `relW`, `gasF(d)` relief flow and its gas-mixture factor; `n2W(P)` nitrogen supply flow.
- `rcsP(Mr,Tr)` RCS pressure from mass and temperature (compressed liquid).
- `init(o)` builds a plant state; `step(s,u,dt)` advances one step from the final command
  `u = {ch, ld, sp kg/s, q W, rel 0–1, n2 0/1, heat W, loss W}` (see docs/physics.md). It clamps to `LIM` and
  applies the plant's own protection (heater trip below 12 % level, safety valve); `s.f` records
  what was applied.
- `fault(s)` why the model must stop (`'dry'|'range'|'critical'|'boiling'`) or `''`; the page maps
  codes to the alert text.

### control.js — automatic control

- `init()` controller memory `cs = {iL, iC, hold, porv, dem}`, kept apart from the plant state.
- `demands(cs,s,c,dt)` runs the automatic logic every step (in M too, for the notches) and stores
  `cs.dem`; `command(cs,c)` picks demand (A) or operator value (M) per actuator and converts the
  panel units of `c` to SI; `update(cs,s,c,dt)` does both and returns `u`. Call it before every
  `step`, with the same dt.
- `engage(cs,c)` bumpless preload of the inventory integrators (on an inventory-mode change).
- `relHys(set)` relief reclose hysteresis.

### index.html — presentation

The three `<script src>` tags (`common.js`, `sim.js`, `control.js`) come first; the page's own `<script>` is one block `{ … }` (no globals
leak) that destructures what it needs from `PZSim`. Order:

1. Imports from `PZCommon`, `PZSim`, and `PZCtl` (as `Ctl`). The page keeps `s` (plant) and `cs` (controller).
2. DOM wiring: `ids`/`checks`/`fmt`/`readCtl()` map controls → `ctl` (SI units except where noted:
   `q`, `loss` kW; `heat` MW, signed net heat to coolant; flows kg/s; `set`, `relSet` stored in Pa; `relMan` relief opening %;
   `inv` is `'pres'|'lev'`; `n2` nitrogen supply valve open (boolean, always manual); `pzMode` is `'pres'|'dT'` (heater/spray logic), `dT` the pressuriser − coolant target in K; `auto` is `{ch,ld,q,sp,rel}` booleans, one per actuator). Manual controls sit
   in the card right of the mimic; each actuator slider has a compact M/A switch (`<k>Am`), a demand
   readout (`<k>d`) and a demand notch on its track (`<k>m`). The logic selectors and setpoints sit in
   the "Automatic control" section below the main panel.
   Every slider gets − / + buttons from JS (wrapped in `.ctl`; ids `<k>Dn`/`<k>Up`). Step sizes live in
   `bump`: fine where precision matters (charging/letdown 0.1 kg/s, pressures 0.1 MPa, net heat 0.5 MW),
   otherwise ~10 round steps over the range (heaters 200 kW). Presses snap to multiples of the step;
   holding repeats. A new slider needs an entry in `ids`, `fmt` and `bump`.
3. `physTick()` runs `step(s, Ctl.update(cs,s,ctl,dt), dt)` repeatedly, halts when `fault(s)` reports one, records `hist` every 2 sim-s.
   No solid↔two-phase side effects: the operator switches modes.
4. `syncAuto()` greys out and back-fills sliders in A from `cs.dem`, moves the notches on sliders in M,
   and writes the status line (warns when the chosen logic no longer suits the pressuriser state).
5. Renderers: `updText()` (gauges + stats), `drawRec()` (SVG recorder), `drawSat()` (P–T plane SVG),
   `drawVessel()` (canvas mimic: pressuriser + reactor vessel joined by surge and spray lines, zone temperatures, particles). Charts redraw at ~7 Hz; canvas every frame.
6. `frame()` rAF loop. Sim time step depends on speed: `ctl_dt()` = 0.25/0.5/1/2 s (≤20×, ≤50×, ≤200×, above). Speed buttons (Pause, 1×…1000×) set `speed`/`running`.

## Conventions

- SI units internally (K, Pa, kg, J). Convert only at display.
- Canvas colours must be resolved via the hidden `probe` element (`css('--var')`) — canvas
  can't parse `light-dark()`.
- Theme: page must read in light, dark, and unset (`data-theme` on root set by the viewer).
- **Primary target is desktop, full screen or close to it (≈1440–1920 px wide).** Design and check
  layouts there first. Mobile is not a priority: it only has to stay usable (no horizontal overflow at
  390 px; below 1100 px everything stacks in one column, gauges sticky on phones). Don't compromise
  the desktop layout for phones.
- Desktop layout: gauges strip, then the wide mimic canvas, with the manual controls in a column to the
  right; stats, recorder and P–T plane in a row underneath; automatic-control settings and notes below that.
- Design language: "precision instrument" — Geist, aluminium ground, black ink, one orange
  accent (#ff4f00) for pressure / pointer, water blue. No new colours without reason.
- Keep user-facing copy plain and specific; state limitations next to the feature they affect.

## Known limitations (keep the notes section in the page in sync)

- Nitrogen: ideal gas in equilibrium with the water (no adiabatic heating on compression), no
  solubility, supply from a fixed 4.0 MPa header, valve open/shut only (no M/A).
- Thermal equilibrium in the saturated region; only insurge stratification is modelled (a
  cold layer mixing with τ = 300 s). No outsurge flashing delay, no steam superheat on
  fast compression, no wall heat capacity in the pressuriser.
- RCS is one well-mixed liquid node; no loops, no hot/cold leg split, no SG secondary side.
- Charging enters at RCS temperature (no cooling effect). Spray water is at RCS temperature.
- Level is liquid volume fraction; the mimic draws it by volume in a cylinder with hemispherical heads (0 % = bottom, 100 % = top).
- Compressibility table is approximate (± ~10 %).
- Coolant pumps are not modelled: one signed net-heat control; pumps assumed running.
- Relief valve flow is a rough scaling (steam ∝ P, liquid ∝ √ΔP), not a valve model.
- Generic 4-loop Westinghouse-like numbers; not a VVER.

## Publishing

The repo is public; the page is opened straight from GitHub through raw.githack, which serves
`index.html` and the scripts beside it from any branch or commit with the right content types. There is no build or deploy step:
pushing a branch is publishing it.

- Branch: `https://raw.githack.com/sati290/pressuriser-sim/<branch>/index.html`
  (slashes in the branch name stay as-is, e.g. `.../claude/some-branch/index.html`).
  raw.githack caches branch URLs for a few minutes, so a fresh push may take a moment to show.
- Exact commit: `https://raw.githack.com/sati290/pressuriser-sim/<commit-sha>/index.html`.
- `main`: `https://raw.githack.com/sati290/pressuriser-sim/main/index.html`.

The old claude.ai artifact is no longer the live version; don't republish there.

**When you finish a task that changed `index.html` or any of the scripts, commit and push it, then end your reply with
the raw.githack URL for the branch you pushed** (and the commit URL if the owner may want to compare
versions).
