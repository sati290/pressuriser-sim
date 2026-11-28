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
| `index.html` | Presentation: CSS, markup, UI wiring, renderers. Its page script is a `<script type="module">` importing `./common.js`, `./sim.js`, `./control.js`; nothing else external except Google Fonts. Needs http(s), not `file://`. |
| `common.js` | What both plant and controller need, on both pages: steam tables, liquid properties (`kap`, `liqRho`, `hLiq`, `cpL`, `Tuf`, `inv`), `clamp`, actuator limits `LIM`. ES module, no imports. |
| `sim.js` | The plant: pure physics, no DOM, no automatic control. ES module; imports `common.js`. |
| `control.js` | Automatic control: demands and the final command for the plant. ES module; imports `common.js`, not `sim.js`. |
| `bwr.html` | Separate BWR page: reactor vessel with core kinetics and recirculation, main steam line (MSIVs, stop, control and bypass valves), safety/relief valves, suppression pool, turbine stub, condenser with circulating water and air removal, condensate and feed train (hotwell, condensate pump, polisher, LP heater, feed pump, HP heater, heater drains, storage tank, feed valve), reactor protection with an event log. The mimic draws the whole plant: modelled parts solid, layout-only parts dashed grey (docs/bwr.md, "Plant layout"). Its layout differs from index.html: the mimic spans the full width with the controls in three columns under it, and the canvas is at least 1180 px wide, scrolling sideways on small screens (no phone layout; phones use landscape). Module page script importing `./common.js`, `./bwr-sim.js`, `./bwr-control.js`; needs http(s) like `index.html`. Same stylesheet as `index.html` (copied; keep the two in step) plus a few BWR rules at its end. |
| `bwr-sim.js` | BWR plant: pure physics. ES module; composes the per-system modules below and re-exports what the page and tests use. See docs/bwr.md, "Code layout". |
| `bwr-vessel.js`, `bwr-core.js`, `bwr-recirc.js`, `bwr-steam.js`, `bwr-condenser.js`, `bwr-feed.js`, `bwr-containment.js`, `bwr-rps.js` | BWR systems (vessel solve, downcomer, steel, level instruments; point kinetics, fuel, decay heat, rod lever, core voids; recirculation pump, MG set, jet pump; rated conditions, steam line valves, SRVs; turbine stub, condenser, circulating water, condenser air; condensate and feed train, heaters and drains, storage tank; suppression pool; reactor protection: trips, latches, mode switch, event log). ES modules importing `common.js` and each other (no cycles); one system per module so a roadmap step touches one. |
| `bwr-control.js` | BWR automatic control (pressure regulator on the control and bypass valves, three-element feedwater cascaded onto the feed valve with the scram level setdown, feed pump speed, core flow on the MG set, min-flow valve, heater drain levels, split-range hotwell makeup/reject, cleanup reject; rods, SRVs and the pump switches are manual; protection is in the plant, not here). ES module; imports `common.js`, not `bwr-sim.js`. |
| `tests/load-physics.mjs` | Imports the three modules in Node as namespaces `common`, `phys` (sim.js), `ctrl` (control.js), plus `run` (plant + controller), `runPlant` (fixed command) and `ctlOf(s)`. Tests never load `index.html`. |
| `tests/physics.test.mjs` | Regression tests (node:test, no deps). ~7 s. |
| `tests/bwr*.test.mjs` | BWR regression tests, one file per system (`bwr-helpers.mjs` holds the shared runners and rated state). `npm test` runs every `tests/*.test.mjs`. |
| `docs/bwr.md` | BWR model equations, constants, controllers, validation numbers, what is not modelled. |
| `tools/screenshot.mjs` | Playwright (Node): renders both pages at 3 widths × light/dark (`--page` for one), fails on JS errors or horizontal overflow. Serves the repo itself over http. |
| `version.js` | `VERSION` ('0.1.x'), shown in the footer of both pages so the owner can tell whether the browser has the latest. ES module, no imports. |
| `tools/vapour-table.mjs` | Regenerates the superheated-steam table `VT` in sim.js from IAPWS-IF97 region 2. |
| `tools/serve.mjs` | Dependency-free static server for `npm run serve` (port 8000, or `node tools/serve.mjs <port>`); also exports `serve(port)`. |
| `docs/physics.md` | Model equations, state variables, controllers, constants, validation numbers. |
| `docs/backlog.md` | Ideas for next iterations, roughly prioritised. |
| `docs/roadmap.md` | BWR plant roadmap: ordered, shippable steps to make the layout's components functional, with open questions. |

## Commands

```bash
npm test                      # physics regression suites (PWR and BWR)
npm run shots                 # screenshots into shots/ (needs: npm i -D playwright && npx playwright install chromium)
node tools/screenshot.mjs --chrome /usr/bin/google-chrome   # use an existing Chrome
npm run serve                 # http://localhost:8000 (index.html can't be opened from file://)
```

Always run `npm test` after touching any of the scripts, and the screenshot
tool after any CSS/markup/drawing change.

## Code structure

### The three scripts

Each is an ES module with no build step: browsers and Node load the same files. Everything the page
or the tests use is in the `export {…}` list at its end — add a new function there to make it
available. Imports use relative paths with the `.js` extension (`./common.js`). Dependencies:
`sim.js` and `control.js` each import `common.js`; neither imports the other. Anything both need
goes into `common.js`. No globals: in devtools use `await import('./sim.js')`. Modules don't load
from `file://`; the page shows a warning there (and whenever its module script failed to finish).

### common.js — shared

- `satX(T)` exact Wagner–Pruss saturation equations → `[p, dp/dT, ρf, ρg, hf, hg]` (SI).
- `sat(T)` fast linear interpolation in a 0.02 K table built from `satX`; returns a **shared,
  reused object `Q`** — copy fields out before calling `sat` again.
- `Tsat(p)` saturation temperature by bisection; `clamp`; `LIM` actuator ranges in SI
  (`ch`, `ld` 20 kg/s, `hp` 1, `hb` 3, `sp` 40 kg/s, `rel`, `n2` 1). Heaters: 78 elements of `QE` 23.1 kW
  as one proportional group (`QP` 415 kW, `hp` fraction) and `NB` = 3 on/off backup groups (`QB` 462 kW). The plant clamps commands to `LIM`, and the
  controller's anti-windup uses the same limits.

### sim.js — the plant

**Pure physics, no DOM, no M/A, no setpoints.**
- `kap(°C)` approximate isothermal compressibility of liquid (1/Pa).
- The pressuriser is always two regions at one pressure, never forced into equilibrium: water (`s.Ml`,
  `s.Ul`, `s.Vl`; compressed liquid, may be sub- or superheated) and a gas space (`s.Mv` steam, `s.Mn`
  nitrogen, `s.Ug`, `s.Vg`) at its own temperature; no gas space = water-solid. `s.M`, `s.U` are totals.
  `vap(ρ,u)` / `vapT(ρ,T)` steam (superheated from the IF97 fit `VT`, or wet); `gas(Mv,Mn,Ug,Vg)` the gas
  space (Dalton). `neSolve`/`neAt` solve the pressure each step with the surge, boundary work and the
  water–gas transfers (time constants `TAUNE`, per state `s.tau`, `init({tau})`). `derive` builds `s.d`
  `{mode,T,Ti,Tl,Tg,P,level,pS,pN,yN,…}`; `mode` is `'two'` whenever there is a gas space; `T` is the
  interface temperature T_sat(steam partial pressure), or the water temperature when solid.
  Nitrogen below 0.1 % of the gas with the supply shut is dropped; steam below 1 g with no nitrogen joins the water.
- `relW`, `gasF(d)` relief flow and its gas-mixture factor; `n2W(P)` nitrogen supply flow.
- `rcsP(Mr,Tr)` RCS pressure from mass and temperature (compressed liquid).
- `init(o)` builds a plant state; `step(s,u,dt)` advances one step from the final command
  `u = {ch, ld, sp kg/s, hp 0–1, hb groups, rel 0–1, n2 0–1, heat W}` (see docs/physics.md). It clamps to `LIM` and
  applies the plant's own protection (heater trip below 12 % level, safety valve); `s.f` records
  what was applied.
- `API` version number (`const API`, exported): bump it (and `SIM_API` in index.html) when the state or `s.d` fields the page
  reads change; the page warns when a cached older sim.js is loaded beside it.
- `fault(s)` why the model must stop (`'dry'|'range'|'critical'|'boiling'`) or `''`; the page maps
  codes to the alert text.

### control.js — automatic control

- `init()` controller memory `cs = {iL, iC, hold, porv, hb, dem}`, kept apart from the plant state.
- Heaters: `propH(ep)` proportional group from the pressure error; `backH(n, ep)` backup groups staged
  on/off with hysteresis (`BON`/`BOFF`), so they don't cycle.
- `demands(cs,s,c,dt)` runs the automatic logic every step (in M too, for the notches) and stores
  `cs.dem`; `command(cs,c)` picks demand (A) or operator value (M) per actuator and converts the
  panel units of `c` to SI; `update(cs,s,c,dt)` does both and returns `u`. Call it before every
  `step`, with the same dt.
- `engage(cs,c)` bumpless preload of the inventory integrators (on an inventory-mode change).
- `relHys(set)` relief reclose hysteresis.

### index.html — presentation

A small classic `<script>` warns in `#alert` on `load` if `window.PZready` is unset (module failed or
`file://`). Then the page's own `<script type="module">`, which sets `PZready` as its last line. Order:

1. Imports: named from `common.js`; `sim.js` as namespace `PZSim` (so the `API` check sees a stale
   cached copy as `undefined` rather than a link error), destructured; `control.js` as namespace `Ctl`. The page keeps `s` (plant) and `cs` (controller).
2. DOM wiring: `ids`/`checks`/`fmt`/`readCtl()` map controls → `ctl` (SI units except where noted:
   `hp` proportional heaters %; `hb` backup heater groups on; `heat` MW, signed net heat to coolant; flows kg/s; `set`, `relSet` stored in Pa; `relMan` relief opening %;
   `inv` is `'pres'|'lev'`; `n2` nitrogen supply valve opening % (always manual, no M/A); `pzMode` is `'pres'|'dT'` (heater/spray logic), `dT` the pressuriser − coolant target in K; `auto` is `{ch,ld,hp,hb,sp,rel}` booleans, one per actuator). Manual controls sit
   in the card right of the mimic; each actuator slider has a compact M/A switch (`<k>Am`), a demand
   readout (`<k>d`) and a demand notch on its track (`<k>m`). The logic selectors and setpoints sit in
   the "Automatic control" section below the main panel.
   Every slider gets − / + buttons from JS (wrapped in `.ctl`; ids `<k>Dn`/`<k>Up`). Step sizes live in
   `bump`: fine where precision matters (charging/letdown 0.1 kg/s, pressures 0.1 MPa, net heat 0.5 MW),
   otherwise ~10 round steps over the range (proportional heaters 5 %, backup heaters 1 group). Presses snap to multiples of the step;
   holding repeats. A new slider needs an entry in `ids`, `fmt` and `bump`.
3. `physTick()` runs `step(s, Ctl.update(cs,s,ctl,dt), dt)` repeatedly, halts when `fault(s)` reports one, records `hist` every 2 sim-s.
   No solid↔two-phase side effects: the operator switches modes.
4. `syncAuto()` greys out and back-fills sliders in A from `cs.dem`, moves the notches on sliders in M,
   and writes the status line (warns when the chosen logic no longer suits the pressuriser state).
5. Renderers: `updText()` (gauges + stats), `drawRec()` (SVG recorder), `drawSat()` (P–T plane SVG),
   `drawVessel()` (canvas mimic: pressuriser + reactor vessel joined by surge and spray lines, zone temperatures, particles). Charts redraw at ~7 Hz; canvas every frame.
6. `frame()` rAF loop. Sim time step depends on speed: `ctl_dt()` = 0.25/0.5/1/2 s (≤20×, ≤50×, ≤200×, above). Speed buttons (Pause, 1×…1000×) set `speed`/`running`.

### The BWR page (bwr.html, bwr-sim.js, bwr-control.js)

Same split and conventions as the PWR page: the plant never decides M/A, the controller produces
demands every step and `command` picks demand or operator value per actuator (`mg` MG set speed, `n` feed pump
speed, `fv` feed valve, `rv` min-flow valve, `hdv`/`ldv` heater drain valves, `mu`/`rj` storage tank makeup and
reject, `rw` cleanup reject, `cv` control valve, `bp` bypass valve have M/A; the rod lever, recirculation pump
breaker, MSIV and stop valve switches, extraction valves `ext`/`exl`, pump run switches `cp`/`fp`/`hdp`, the
condenser line-up and the per-valve SRV toggles `srv[]` are manual only; each SRV's spring opens it on pressure
in the plant). Reactor protection (bwr-rps.js) runs inside `step()`: latched trips scram, isolate, trip the
turbine and the feed pump whatever the switches say; `reset(s)` clears only trips whose signal has cleared, the
page's Scram button is `manualScram`, the mode switch `setMode`. `init({rps:false})` gives the bare physics
(the physics tests use it through tests/bwr-helpers.mjs). The vessel is one rigid volume: totals `M`, `U`,
saturated steam `Mv` and its void part `Mb`, with a downcomer region (`udc`); each step solves the saturation
temperature at which water and steam fill the vessel. Level includes the voids (swell); `d.collapsed` is water
alone. The core heat comes from point kinetics (`u.heat` as a number is a fixed-heat test hook). See
docs/bwr.md, "Code layout", for the step order and the per-module state. `API` in bwr-sim.js and
`SIM_API` in bwr.html follow the same versioning rule as sim.js `API` (bwr-sim.js is imported as a namespace too). Equations in docs/bwr.md; keep the
page's notes in step with its limitations.

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

- Nitrogen: ideal gas, no solubility, supply from a fixed 4.0 MPa header, throttled valve, manual only (no M/A).
- Water and gas relax towards each other with illustrative time constants (boiling/evaporation 5 s,
  surface condensation 300 s, gas–water heat exchange 30 s), not fitted to data, the same with nitrogen
  (no diffusion-limited evaporation); the water is one well-mixed region (no hot surface layer); spray
  reaches the interface temperature. Superheated steam is a quadratic fit to IF97 region 2 (~1 % to 10 K
  superheat; extrapolated above ~350 °C). Insurge layer mixes with τ = 300 s. No wall heat capacity.
- RCS is one well-mixed liquid node; no loops, no hot/cold leg split, no SG secondary side.
- Charging enters at RCS temperature (no cooling effect). Spray water is at RCS temperature.
- Level is liquid volume fraction; the mimic draws it by volume in a cylinder with hemispherical heads (0 % = bottom, 100 % = top).
- Compressibility table is approximate (± ~10 %).
- Coolant pumps are not modelled: one signed net-heat control; pumps assumed running.
- All pressures are absolute. Relief valve flow is a rough scaling (steam ∝ P while choked, subsonic
  nozzle factor near the 0.1 MPa relief-tank back pressure; liquid ∝ √(P − 0.1 MPa)), not a valve model;
  the relief tank is a fixed 0.1 MPa sink.
- Heaters: one proportional group and three backup groups switched on pressure only (no level-deviation
  switching); no element heat-up lag.
- Pressuriser heat loss is UA·(T − 40 °C), UA = 200 W/K, both fixed; RCS losses are inside net heat.
- Generic 4-loop Westinghouse-like numbers; not a VVER.

BWR page: see docs/bwr.md, "Not modelled" (point kinetics with a quasi-static axial shape only and one rod lever, one recirculation loop,
saturated steam only, no air in the dome, uniform-area level, one steel node, turbine stub, two heater stages,
one pump of each kind, protection on one logic channel without RCIC/HPCI/ECCS, cleanup as a reject placeholder,
no containment pressure).

## Publishing

The repo is public; the page is opened straight from GitHub through raw.githack, which serves
`index.html` and the scripts beside it from any branch or commit with the right content types. There is no build or deploy step:
pushing a branch is publishing it.

- Branch: `https://raw.githack.com/sati290/pressuriser-sim/<branch>/index.html`
  (slashes in the branch name stay as-is, e.g. `.../claude/some-branch/index.html`).
  raw.githack caches branch URLs for a few minutes, so a fresh push may take a moment to show.
- Exact commit: `https://raw.githack.com/sati290/pressuriser-sim/<commit-sha>/index.html`.
- `main`: `https://raw.githack.com/sati290/pressuriser-sim/main/index.html`.
- The BWR page is `bwr.html` at the same places (e.g. `.../<branch>/bwr.html`).

The old claude.ai artifact is no longer the live version; don't republish there.

**Version:** every change you push bumps the last number of `VERSION` in `version.js` by one (0.1.0 →
0.1.1 → …), once per push, whatever files changed. Keep `version` in package.json equal to it. Mention the new version in your reply. The owner
uses the footer to check that the browser isn't showing a cached copy.

**When you finish a task that changed `index.html` or any of the scripts, commit and push it, then end your reply with
the raw.githack URL for the branch you pushed** (and the commit URL if the owner may want to compare
versions).
