# BWR vessel model (bwr.html)

A separate page beside the PWR pressuriser. Same architecture: `bwr-sim.js` is the plant (pure physics),
`bwr-control.js` the automatic control, both ES modules importing `common.js` (steam tables, liquid
properties). Tests: `tests/bwr.test.mjs`.

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `V`, `H`, `A` | 600 m³, 21 m, 28.6 m² | Free fluid volume, inside height, uniform free cross-section V/H |
| `Z` | BAF 5.6, TAF 9.3, SHR 11.5, FW 13.0, NWL 14.3, MSL 16.5 m | Elevations above the inside bottom: active fuel, top of shroud, feedwater sparger, normal water level, main steam nozzle |
| Net heat | −100 … +3400 MW (page) | Into the water: core or decay heat minus cleanup/shutdown cooling and losses. Rated 3293 MW |
| Main steam valve | 2000 kg/s full open at 7.0 MPa (`WMS`, `PR`), ∝ P × subsonic factor ψ(PC/P)/ψ* (κ = 1.3), condenser PC = 0.01 MPa | Stands for MSIVs + turbine control and bypass valves. Travel ≤ 20 %/s (`RATE`, full stroke 5 s). `s.mv` is the position, `u.mv` the demand |
| SRVs | `SRV` = 11 valves `{p, w}`: 4 × 1090 psig (7.617 MPa abs) rated 800,000 lb/h (100.8 kg/s), 4 × 1100 psig (7.686 MPa) 808,000 lb/h (101.8 kg/s), 3 × 1110 psig (7.755 MPa) 815,000 lb/h (102.7 kg/s); flow w·P/p × subsonic factor to the pool at 0.1 MPa | From a GE BWR/4 training lesson plan (NRC-hosted; group counts partly illegible in the OCR, read so that capacity ∝ set pressure, which it is for one valve model). Each valve: spring opens it fully above p, recloses below p − `BD` (0.25 MPa, illustrative; not in the source), latch `s.spr[i]`; or opened by hand, `u.srv[i]` true. Open = spring or hand. No automatic relief logic, low-low set or ADS |
| Hotwell | `HWM` 600 t capacity, normal half full, water at `THW` = T_sat(PC) = 45.8 °C | Source of the feed. `condensate(s, wst)` returns the main steam flow at once at THW (ideal condenser placeholder, to be replaced by a condenser model). Makeup from / reject to the condensate storage tank `u.mk`, ±100 kg/s (`MKMAX`) |
| Feed pump | ΔP = n²·`H0` − `KPU`·w², H0 = 10 MPa, KPU = 0.24 Pa/(kg/s)²; suction at PC | Condensate, booster and feed pumps lumped. Speed `u.n` 0–1 (affinity), state `s.n`, changes at most 10 %/s (`NRATE`). Shutoff 10.01 MPa at full speed, 6.4 MPa at 80 %. Loses suction below 5 % hotwell: flow × Mh/(0.05·HWM) |
| Heater + piping drop | `KLN`·w², KLN = 0.094 Pa/(kg/s)² | 0.3 MPa at rated flow |
| Feed valve | drop w²/(`KV`·x²)², KV = 2.31 (kg/s)/√Pa, quadratic characteristic; stroke ≤ 10 %/s (`FRATE`) | Flow w = √((n²H0 + PC − P)/(KPU + KLN + 1/(KV x²)²)), zero if the root is not positive (check valve). `fwFlow(n, x, P, Mh)`, inverse `fwValve(w, n, P)`. Rated 1788 kg/s at 74.5 % open, 2400 kg/s wide open at 7 MPa |
| Feedwater heater | Extraction pressure pe = `PE`·wst/`WR` (PE = 2.40 MPa, WR = 1788 kg/s); extraction valve flow `KXT`·e·√(pe² − ps²); shell condenses ws = fw·(h(T_out) − h(T_in))/(h″ − h′)(ps); T_out = T_sat(ps) − `TTD` (3 K) | One stage, a shell-and-tube condensing heater, for the whole heater train. Extraction pressure ∝ main steam flow (turbine stage pressures are). The shell pressure ps is where the valve passes what the shell condenses (bisection on the shell saturation temperature). Design point: rated flows, 216 °C out, 5 % drop across the open valve, ws ≈ 700 kg/s. `u.ext` (e) 0–1 valve opening. Outlet follows T_out with `TAU.h`. `heater(wst, e, fw)` → {T, pe, ps, ws}; `heaterT`; `heatBalance(Q, P, e)` the steady {W, Tfw}. Drains go to the hotwell (inside the condensate return) |
| `TAU.c` | 10 s | Subcooled water condenses steam towards saturation |
| `TAU.b` | 0.7 s | Steam bubbles (voids) leave the two-phase mixture; gives ~34 m³ of voids (1.2 m swell) at rated steaming |
| `TAU.h` | 60 s | Feedwater heater outlet temperature (heater and piping holdup) |

## Scram

`scram(s, q)` records the time and P0 = q, the net heat at that moment (≥ 0; default the last applied `s.f.Q`; the page passes the commanded heat); from then on `step` ignores
`u.heat` and uses P0·`decayFrac`(t − t_scram) until `reset(s)`. `decayFrac(t)` = 0.935·(0.93·e^(−t/2) +
0.06·e^(−t/10) + 0.01·e^(−t/80)) + 0.065·(1 + t)^(−0.2): fission power with the rods in (prompt drop,
then two delayed-neutron tails) plus decay heat in Way–Wigner form for long operation at P0. Gives
100 % at 0, ~7.5 % at 10 s, 3.3 % at 1 min, 1.3 % at 1 h, 0.7 % at a day — rough, not a fitted curve.
The page greys the net-heat slider while scrammed and back-fills it with the decay heat, so reset
continues from that value.

## State and step

State: total mass `M`, total internal energy `U`, steam mass `Mv` (dome + voids, always saturated), void
steam `Mb` (part of `Mv`), valve position `mv`, SRV spring latches `spr[]`; feedwater train: feed valve
position `fv`, heater outlet temperature `Th`, hotwell mass `Mh`.

Each step (`step(s, u, dt)`):
1. Valve moves toward its demand at `RATE`; each SRV spring latch from P; an SRV is open if its spring or `u.srv[i]` holds it.
2. Flows at the start-of-step pressure: steam out (main steam + SRVs) at h″, feedwater in at h(T_fw, P)
   with T_fw the heater outlet `Th` and the flow from `fwFlow` (pump speed, feed valve position after its
   travel, hotwell), heat Q. `M` and `U` updated; `Mv` loses the steam that left. Hotwell: Mh += (return
   + makeup − feed)·dt; heater outlet relaxes towards `heaterT(wst, e)`.
3. If the water (U − Mv·u″)/(M − Mv) is below u′, it condenses steam: Ml·(u′ − u)/(u″ − u′)·(1 − e^(−dt/τc)).
4. Pressure solve on the saturation temperature T (Illinois regula falsi): at trial T the steam has
   u″(T); if the water would be above u′(T) it flashes to equilibrium, Mv = (U − M·u′)/(u″ − u′); the
   residual V − V_water − Mv/ρ″ rises with T. Water volume from the compressed-liquid density at its own
   temperature (`liqRho`, `Tuf` from common.js).
5. Net generation g = (flash − condensation)/dt feeds the voids: Mb relaxes to max(0, g)·τb.

Level = (V_water + Mb/ρ″)/A; collapsed level = V_water/A. Mass and energy are conserved exactly
(the vessel is rigid, so there is no external work; the tests check both).

`fault(s)`: `'uncovered'` level below TAF, `'flooded'` level at the steam nozzle, `'critical'` above
21.5 MPa, `'hotwell'` hotwell over capacity (condenser flooding), `'range'` if the solve fails.

Command `u` = {heat W, n 0–1 pump speed, fv 0–1 feed valve demand, ext 0–1 heater extraction, mk kg/s
makeup (negative: reject), mv 0–1 steam valve demand, srv[]}. `s.f` records what was applied, with the feed
train's fw, Tfw, Pd (pump discharge), pe (extraction pressure), Tt (heater target), ret (condensate
return), mk, Mh, hw (hotwell fraction).

## Control (bwr-control.js)

- Pressure regulator: steam valve demand = KP·(P − pset) + I, KP = 3×10⁻⁶ /Pa (0.1 MPa → 30 %), integral
  time 30 s, anti-windup at 0 and 1.
- Feedwater, three-element cascade: flow demand F = main steam flow + KL·(lset − level) + I, KL = 800 kg/s
  per m, integral time 60 s, 0 … 2500 kg/s; then a flow controller on the feed valve, x = KX·(F − fw) + I,
  KX = 10⁻⁴ per kg/s, integral time 4 s, anti-windup at 0 and 1 (the level integral also holds while the
  valve is saturated). SRV flow is not in the steam signal, so after a valve closure the level term alone
  has to call for feedwater. The pump speed is manual: no valve opening helps once vessel pressure is above
  the shutoff head.
- Hotwell makeup: mk = (fw − return) + KH·(0.5 − hotwell fraction) + I, KH = 1000 kg/s per unit fraction,
  integral 300 s, ±100 kg/s. Setpoint fixed at half full.
- Feed pump speed: n = KN·(dpset − (Pd − P)) + I, KN = 1.5×10⁻⁸ /Pa, integral 10 s, 0 … 1: holds the pump
  discharge 2.2 MPa (default) above the dome, as turbine-driven feed pumps are run, so the feed valve keeps a
  steady drop at any vessel pressure.
- Heater extraction valve: manual only.
- `init(c)` preloads the integrators from the operator's settings, so an actuator that starts in A starts
  from its slider.
- SRVs: no automatic logic in the controller; `u.srv` passes the operator's per-valve hand openings
  through. The springs act in the plant.
- In M an actuator's integral holds the operator's value (for the level loop, the measured feed flow net
  of the steam-flow feedforward; for makeup, net of its feedforward), so the demand — the notch — is the operator's value plus the proportional correction for
  the present error. At balance the notch sits on the slider; switching to A starts from the notch.

## Validation numbers (tests/bwr.test.mjs)

- Rated heat balance (3293 MW, 216 °C feed, 7.0 MPa → 1788 kg/s) stays steady at dt 0.25 and 2 s, with the
  pump at full speed, the feed valve 74.5 % open, the heater at 216 °C and the hotwell half full.
- Valve closure at rated heat: level shrinks > 0.4 m in 6 s (void collapse), pressure > 7.4 MPa. With no scram,
  pressure climbs past the feed pump's 10 MPa shutoff head in about 30 s, feed stops although the valve opens
  fully, and the core uncovers after about 80 s. With a scram, only the 1090 psig bank lifts and pressure
  peaks just above 7.6 MPa; the feed cools to 46 °C (no extraction steam) and the level peaks about 1.6 m
  above normal (about 1.2 m with the pump speed in A). Isolation at 200 MW: the lowest bank cycles 7.35–7.62 MPa, about one lift per 45 s.
- Extraction valve to 20 % at rated: feed about 35 K colder after 1 min, 54 K after 6 min. With everything in
  A the steam flow settles 12 % lower and the level hardly moves; in M the falling pressure lets the pump
  deliver more, and the vessel floods in under 3 min.
- Pump speed in A through a depressurisation to 1 MPa: discharge held 2.2 MPa above the dome, speed down to
  about 55 %.
- Hotwell: without makeup it drains during an isolation (relief steam does not return); with makeup in A it
  stays half full. A full hotwell with the steam valve open is the `'hotwell'` fault.
- Cold feedwater (about 100 kg/s at 46 °C, no heat): pressure falls ~0.35 MPa/min, subcooling < 1 K.
- Startup 30 MW from 100 °C, valve shut: 60–110 K/h, level up > 0.8 m in an hour from expansion.
- Load step 3293 → 2000 MW with both controllers in A: pressure and level back to setpoint within 20 min.

## Plant layout (draft)

The mimic draws the whole plant, one of each component (no redundant trains). It spans the page width with
the controls below it; the canvas is at least 1180 px wide and scrolls sideways on smaller screens (phones:
use landscape). There is no separate phone layout. Solid parts are modelled; dashed grey parts are layout only, to be made functional step by step.

- Inside the primary containment (dotted outline): vessel, SRVs and suppression pool, one recirculation
  loop (suction from the downcomer, pump below the core, back to one jet pump in the downcomer), the
  inboard MSIV. Feed check valve where the feed line leaves containment.
- Main steam: inboard and outboard MSIV, then a tee for the bypass line (down into the condenser upper
  shell) and the reheat steam (up to the MSR), then the turbine stop valve and the control valve. The
  model's one main steam valve (`mv`) is drawn as the control valve.
- Turbine hall: HP turbine → crossover up into the moisture separator reheater (drains to the heaters) →
  LP turbine (double flow, exhausts down into the condenser) → generator on the same shaft.
- Condenser: shell, tube bundle with circulating water in/out, air removal to the steam jet air ejector
  (off-gas; its motive steam comes from the main steam header over the turbine hall) and the mechanical
  vacuum pump (startup), hotwell at the bottom (the model's hotwell).
- Condensate and feed: hotwell → condensate pump → polisher → LP heater → feed pump → HP heater (the
  model's one heater, its extraction now drawn from the HP turbine casing) → feed valve → containment.

Not on the layout yet, for the steam/feed circuit: feed heater and MSR drain lines (drain tank, drain
pumps or cascade to the condenser), extraction lines to the LP heater and the MSR/heater non-return
valves, the off-gas train (recombiner, delay beds, stack), turbine gland
seal steam and its condenser, circulating water pumps and the cooling tower or river, the condensate
storage tank (shown only as the makeup label), reactor water cleanup (RWCU), control rod drive
hydraulics, main steam drains to the condenser, generator output and grid breaker. Emergency cooling
(RCIC, HPCI, core spray, RHR) is deliberately left for later.

## Not modelled

Neutronics and void/pressure reactivity feedback, automatic scram (scram is an operator button), recirculation pumps and jet pumps, downcomer
subcooling separate from the core, steam superheat, air in the dome (below 100 °C the pressure goes
sub-atmospheric), vessel and internals heat capacity, turbine, condenser (the main steam returns to the hotwell at once
as 45.8 °C condensate), moisture separator reheaters, separate heater stages and drain coolers (one throttled extraction line stands for the train; real extraction lines have only isolation and non-return valves), pump trips (no
level-8 or low-suction trip; suction loss is a flow taper), pump minimum-flow recirculation, startup
low-flow valve, condensate and feed water inventory in the pipes, cleanup reject flow, suppression-pool heat-up. Void clearance uses one time constant at every power and pressure,
so swell from bulk flashing during a depressurisation is underestimated.
