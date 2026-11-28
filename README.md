# Pressuriser simulator

Interactive simulator of a PWR steam pressuriser coupled to the reactor coolant system:
water-solid operation, bubble draw, heat-up, hot standby, load transients, cooldown and refill.

- `npm run serve` and open http://localhost:8000/ (the scripts are ES modules, which browsers won't load from `file://`). No build step. Or open a branch through raw.githack (see `CLAUDE.md`).
- `npm test` runs the physics regression suite (Node ≥ 20, no dependencies).
- `npm run shots` renders screenshots (needs Playwright).

See `CLAUDE.md` for architecture and conventions, `docs/physics.md` for the model,
`docs/backlog.md` for ideas.
