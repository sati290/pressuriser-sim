# Backlog

Roughly in order of value for understanding the physics. None of these are committed; pick with the owner.

## Physics fidelity
Done: non-equilibrium steam bubble with outsurge flashing delay (see docs/physics.md). Follow-ups:
- **Tune the time constants** against published insurge/outsurge test data, or expose them (and an
  equilibrium/finite-rate switch) on the page for comparison.
- **Water surface layer** — a thin top layer that heats on condensation would make surface condensation
  self-limiting instead of the 300 s constant on the whole water mass.
- **Non-equilibrium with nitrogen** — today the gas space goes back to equilibrium when nitrogen flows in.

1. **Pressuriser wall heat capacity** — steel shell slows heat-up and stores energy during cooldown.
2. **Auxiliary spray** — spray from charging (cold) when no RCPs run; enforce spray ΔT limit and show
   thermal-shock warnings.
3. **Charging enthalpy** — regenerative heat exchanger outlet temperature as a function of letdown flow.
4. **Real level geometry** — hemispherical heads; level in % of span instead of volume fraction.
5. **Nitrogen solubility** — the cushion itself is done (Dalton, venting, supply line); add Henry's-law
   solution and degassing as the water heats, and non-equilibrium (adiabatic) gas compression.

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

## UX
12. Units toggle (SI / psi-°F).
13. Annotated recorder (mark mode changes on the time axis).
