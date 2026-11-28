# Backlog

Roughly in order of value for understanding the physics. None of these are committed; pick with the owner.

## Physics fidelity
Done: water and gas never forced into equilibrium — steam superheat, outsurge flashing delay, adiabatic
nitrogen compression (see docs/physics.md). Follow-ups:
- **Tune the time constants** against published insurge/outsurge test data, or expose them on the page.
- **Water surface layer** — a thin top layer that heats on condensation would make surface condensation
  self-limiting instead of the 300 s constant on the whole water mass.
- **Diffusion-limited evaporation with nitrogen** — today the same time constants apply with or without nitrogen.

1. **Pressuriser wall heat capacity** — steel shell slows heat-up and stores energy during cooldown.
2. **Auxiliary spray** — spray from charging (cold) when no RCPs run; enforce spray ΔT limit and show
   thermal-shock warnings.
3. **Charging enthalpy** — regenerative heat exchanger outlet temperature as a function of letdown flow.
4. **Real level geometry** — hemispherical heads; level in % of span instead of volume fraction.
5. **Nitrogen solubility** — the cushion itself is done (Dalton, venting, supply line); add Henry's-law
   solution and degassing as the water heats.

## Operations
6. **P–T limit curves** — illustrative heat-up/cooldown brittle-fracture limit and minimum-pressure
   (RCP NPSH / seal) curves on the P–T plane, with violations flagged.
7. **Level programme** — level setpoint as a function of Tavg (25 % → 60 %), with power operation.
8. **Power operation** — core power with simple SG heat sink and Tavg programme; load-following transients.
9. **Event log** — timestamped list (bubble formed, relief opened, heaters tripped, solid…).
10. **Save/restore state** — copy a JSON snapshot; paste to restore (artifact sandbox blocks downloads).

## Plant variants
11. **Parameter presets** — e.g. VVER-440 / VVER-1000 vs Westinghouse 4-loop (volumes, setpoints,
    heater power). Needs sourced numbers; don't invent them.
12. **BWR condenser** (next) — close the loop: turbine exhaust and bypass steam into a condenser with
    its own pressure (cooling water, air), condensate to the hotwell at the condenser temperature. Replaces
    `condensate()` in bwr-sim.js, which today returns the main steam flow at once at 45.8 °C; the steam
    valve's back pressure `PC` would then follow the condenser.

## UX
12. Units toggle (SI / psi-°F).
13. Annotated recorder (mark mode changes on the time axis).
