# Pressuriser simulator

Interactive simulator of a PWR steam pressuriser coupled to the reactor coolant system:
water-solid operation, bubble draw, heat-up, hot standby, load transients, cooldown and refill.

- Open `index.html` in a browser (it loads the model from `sim.js` next to it). No build step.
- `npm test` runs the physics regression suite (Node ≥ 20, no dependencies).
- `npm run shots` renders screenshots (needs Playwright).

See `CLAUDE.md` for architecture and conventions, `docs/physics.md` for the model,
`docs/backlog.md` for ideas.
