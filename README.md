# Pressuriser simulator

Interactive simulator of a PWR steam pressuriser coupled to the reactor coolant system:
water-solid operation, bubble draw, heat-up, hot standby, load transients, cooldown and refill.

A second page, `bwr.html`, simulates a BWR reactor pressure vessel with its main steam line and valve,
safety/relief valves (manual or automatic) and feedwater line: pressure, level, core voids and swell.

- Open `index.html` in a browser (it loads `common.js`, `sim.js` and `control.js` next to it), or
  `bwr.html` (it loads `common.js`, `bwr-sim.js`, `bwr-control.js`). No build step.
- `npm test` runs the physics regression suite (Node ≥ 20, no dependencies).
- `npm run shots` renders screenshots (needs Playwright).

See `CLAUDE.md` for architecture and conventions, `docs/physics.md` for the model,
`docs/bwr.md` for the BWR model, `docs/backlog.md` for ideas.
