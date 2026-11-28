# Backlog

Roughly in order of value for understanding the physics. None of these are committed; pick with the owner.
PWR pressuriser page first, then the BWR page (its own numbering, B1…).

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
10. **Save/restore state** — download a JSON snapshot (or copy it to the clipboard) and load or paste it to restore.

## Plant variants
11. **Parameter presets** — e.g. VVER-440 / VVER-1000 vs Westinghouse 4-loop (volumes, setpoints,
    heater power). Needs sourced numbers; don't invent them.

## UX
12. Units toggle (SI / psi-°F).
13. Annotated recorder (mark mode changes on the time axis).

## BWR page (bwr.html)

Roadmap order: the feedwater system (done), then a simple condenser to close the loop.
The rest is roughly in order of value. See docs/bwr.md, "Not modelled", for what each one replaces.

### Steam–feedwater loop
B1. **Feedwater system** (done, 0.1.6) — condenser hotwell as the suction source, feedwater pump,
    feedwater heater, feed valve, so feed flow and temperature follow from the train instead of being set.
B2. **Simple condenser** — closes the loop: main steam (and later bypass) condenses into the hotwell
    that feeds B1; cooling-water heat sink, condenser pressure from the steam load, hotwell level.
    Replaces `condensate()` in bwr-sim.js (today: the main steam flow back at once at 45.8 °C); the steam
    valve's back pressure `PC` would then follow the condenser. On the mimic it goes at the turbine end of
    the steam line, above the hotwell, where the dashed condensate line is now.
B3. **Turbine and bypass valves** — turbine control valves and a steam bypass to the condenser, so a
    load rejection has somewhere to send steam besides the SRVs.

### Core
B4. **Void and pressure reactivity feedback** — point kinetics (one or six delayed groups) with void,
    Doppler and moderator-temperature coefficients, so power responds to pressure, voids and feedwater
    temperature: the pressurisation power spike on valve closure, the power rise with colder feed.
    Needs sourced coefficients for a BWR/4-like core; don't invent them.
B5. **Recirculation flow** — recirculation pumps and jet pumps, core flow as a state; power follows flow
    through the void feedback of B4 (flow control of power along a rod line). Lets void clearance depend on
    flow instead of one time constant.
B6. **Automatic scram** — trips on high pressure, low level, main steam valve closure (position switch),
    high power once B4 exists; scram stays available as the operator button.

### Vessel and containment
B7. **Downcomer and subcooling** — a separate downcomer node mixing feedwater with separator return, so
    core inlet subcooling responds to feed temperature with a delay.
B8. **Suppression pool** — pool temperature and mass rising with SRV discharge, pool cooling; heat-up
    limits during an isolation.
B9. **Isolation condenser / RCIC** — decay-heat removal and makeup with the main steam valve shut.
B10. **Vessel and internals heat capacity** — steel stores heat on heat-up and returns it on cooldown
    (same idea as PWR item 1).
B11. **Level by real geometry** — level span on the instrument taps instead of a uniform-area vessel;
    swell from bulk flashing rising more slowly during depressurisation.
