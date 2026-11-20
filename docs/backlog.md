# Backlog

Roughly in order of value for understanding the physics. None of these are committed; pick with the owner.

## Physics fidelity
1. **Non-equilibrium steam region** — let steam superheat on fast compression and condense with a
   finite rate (interface + wall + spray efficiency). Gives the real insurge pressure overshoot.
2. **Outsurge flashing delay** — liquid superheat relaxes to saturation with a time constant, so
   pressure dips before recovering.
3. **Pressuriser wall heat capacity** — steel shell slows heat-up and stores energy during cooldown.
4. **Auxiliary spray** — spray from charging (cold) when no RCPs run; enforce spray ΔT limit and show
   thermal-shock warnings.
5. **Charging enthalpy** — regenerative heat exchanger outlet temperature as a function of letdown flow.
6. **Real level geometry** — hemispherical heads; level in % of span instead of volume fraction.
7. **Nitrogen solubility** — the cushion itself is done (Dalton, venting, supply line); add Henry's-law
   solution and degassing as the water heats, and non-equilibrium (adiabatic) gas compression.

## Operations
8. **P–T limit curves** — illustrative heat-up/cooldown brittle-fracture limit and minimum-pressure
   (RCP NPSH / seal) curves on the P–T plane, with violations flagged.
9. **Level programme** — level setpoint as a function of Tavg (25 % → 60 %), with power operation.
10. **Power operation** — core power with simple SG heat sink and Tavg programme; load-following transients.
11. **Event log** — timestamped list (bubble formed, relief opened, heaters tripped, solid…).
12. **Save/restore state** — copy a JSON snapshot; paste to restore (artifact sandbox blocks downloads).

## Plant variants
13. **Parameter presets** — e.g. VVER-440 / VVER-1000 vs Westinghouse 4-loop (volumes, setpoints,
    heater power). Needs sourced numbers; don't invent them.

## UX
14. Units toggle (SI / psi-°F).
15. Annotated recorder (mark mode changes on the time axis).
