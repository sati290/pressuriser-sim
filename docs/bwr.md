# BWR vessel model (bwr.html)

A separate page beside the PWR pressuriser. Same architecture: `bwr-sim.js` is the plant (pure physics,
`globalThis.BWRSim`), `bwr-control.js` the automatic control (`globalThis.BWRCtl`), both on top of
`common.js` (steam tables, liquid properties). Tests: `tests/bwr.test.mjs`.

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `V`, `H`, `A` | 600 m³, 21 m, 28.6 m² | Free fluid volume, inside height, uniform free cross-section V/H |
| `Z` | BAF 5.6, TAF 9.3, SHR 11.5, FW 13.0, NWL 14.3, MSL 16.5 m | Elevations above the inside bottom: active fuel, top of shroud, feedwater sparger, normal water level, main steam nozzle |
| Net heat | −100 … +3400 MW (page) | Into the water: core or decay heat minus cleanup/shutdown cooling and losses. Rated 3293 MW |
| Main steam valve | 2000 kg/s full open at 7.0 MPa (`WMS`, `PR`), ∝ P × subsonic factor ψ(PC/P)/ψ* (κ = 1.3), condenser PC = 0.01 MPa | Stands for MSIVs + turbine control and bypass valves. Travel ≤ 20 %/s (`RATE`, full stroke 5 s). `s.mv` is the position, `u.mv` the demand |
| SRVs | `NV` = 11 × 113 kg/s at 7.8 MPa, ∝ P × subsonic factor to the pool at 0.1 MPa | Relief mode commanded as a count (`u.srv`); safety mode (plant) opens all above 8.5 MPa, recloses below 8.1 MPa |
| Feedwater | 0–2500 kg/s (`FWMAX`), 20–230 °C, enthalpy `hLiq(T, P)` | Pumps deliver at any vessel pressure |
| `TAU.c` | 10 s | Subcooled water condenses steam towards saturation |
| `TAU.b` | 0.7 s | Steam bubbles (voids) leave the two-phase mixture; gives ~34 m³ of voids (1.2 m swell) at rated steaming |

## State and step

State: total mass `M`, total internal energy `U`, steam mass `Mv` (dome + voids, always saturated), void
steam `Mb` (part of `Mv`), valve position `mv`, safety-mode latch `sv`.

Each step (`step(s, u, dt)`):
1. Valve moves toward its demand at `RATE`; SRV safety latch from P.
2. Flows at the start-of-step pressure: steam out (main steam + SRVs) at h″, feedwater in at h(T_fw, P),
   heat Q. `M` and `U` updated; `Mv` loses the steam that left.
3. If the water (U − Mv·u″)/(M − Mv) is below u′, it condenses steam: Ml·(u′ − u)/(u″ − u′)·(1 − e^(−dt/τc)).
4. Pressure solve on the saturation temperature T (Illinois regula falsi): at trial T the steam has
   u″(T); if the water would be above u′(T) it flashes to equilibrium, Mv = (U − M·u′)/(u″ − u′); the
   residual V − V_water − Mv/ρ″ rises with T. Water volume from the compressed-liquid density at its own
   temperature (`liqRho`, `Tuf` from common.js).
5. Net generation g = (flash − condensation)/dt feeds the voids: Mb relaxes to max(0, g)·τb.

Level = (V_water + Mb/ρ″)/A; collapsed level = V_water/A. Mass and energy are conserved exactly
(the vessel is rigid, so there is no external work; the tests check both).

`fault(s)`: `'uncovered'` level below TAF, `'flooded'` level at the steam nozzle, `'critical'` above
21.5 MPa, `'range'` if the solve fails.

## Control (bwr-control.js)

- Pressure regulator: steam valve demand = KP·(P − pset) + I, KP = 3×10⁻⁶ /Pa (0.1 MPa → 30 %), integral
  time 30 s, anti-windup at 0 and 1.
- Feedwater, three-element: demand = main steam flow + KL·(lset − level) + I, KL = 150 kg/s per m, integral
  time 120 s, 0 … 2500 kg/s. SRV flow is not in the steam signal.
- SRV relief mode: groups of 4, 4, 3 valves opening at relSet, +0.07, +0.14 MPa, each reclosing 0.35 MPa
  below its opening pressure.
- In M an actuator's integrator tracks the operator's value, so switching to A is bumpless.

## Validation numbers (tests/bwr.test.mjs)

- Rated heat balance (3293 MW, 216 °C feed, 7.0 MPa → 1788 kg/s) stays steady at dt 0.25 and 2 s.
- Valve closure at rated heat: level shrinks > 0.4 m in 6 s (void collapse), pressure > 7.4 MPa.
- Cold feedwater (100 kg/s at 50 °C, no heat): pressure falls ~0.35 MPa/min, subcooling < 1 K.
- Startup 30 MW from 100 °C, valve shut: 60–110 K/h, level up > 0.8 m in an hour from expansion.
- Load step 3293 → 2000 MW with both controllers in A: pressure and level back to setpoint within 20 min.

## Not modelled

Neutronics and void/pressure reactivity feedback, scram, recirculation pumps and jet pumps, downcomer
subcooling separate from the core, steam superheat, air in the dome (below 100 °C the pressure goes
sub-atmospheric), vessel and internals heat capacity, turbine, condenser, feedwater heaters, cleanup
reject flow, suppression-pool heat-up. Void clearance uses one time constant at every power and pressure,
so swell from bulk flashing during a depressurisation is underestimated.
