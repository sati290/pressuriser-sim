# BWR plant roadmap: from layout to functional systems

The BWR mimic (layout draft, PR 17) draws the whole plant, one of each component, with the parts that are
not modelled dashed grey. This roadmap orders the work of making them functional. Each step is one
shippable increment: its own PR, its own tests in `tests/bwr.test.mjs`, the part turning solid on the
mimic, docs/bwr.md updated (constants, validation numbers, "Not modelled"), and the page notes kept in
step. Steps say what they depend on; where two steps don't depend on each other the order is a preference.

Reference plant (decided): generic BWR/4-class, not one named plant; numbers sourced where possible
from that class.

Ground rules carried over from the project: one of each component (no redundant trains); realistic
arrangements, control schemes and sourced numbers, with placeholders labelled as such; manual control
primary, every new actuator gets M/A and a demand notch; emergency core cooling is out of scope for now.

**Controls (Attila, 2026-11-28): realistic but simple.** The operator moves what a real operator moves:
valve positions, pump start/stop and speed, breaker and switch positions, not flows or heat rates; flows
follow from the plant. Keep each control simple: one valve or one lever where the real plant has a
system of them (for example no full control rod setup for now, one reactivity lever until step 13). The
present flow-valued inputs (hotwell makeup in kg/s, net heat in MW) are converted to valves or levers as
the steps that touch them land.

Priority from the owner: **turbine and MSR are low priority**, so the turbine island stays a lumped stub
until late (step 15). **Control rods and RWCU (steps 13, 14) are deprioritised** as well: until
then reactivity is a single control (step 8) and cleanup stays inside net heat.

The existing backlog items B1–B11 (docs/backlog.md) map onto the steps below; this file is the order.

## Overview

| # | Step | Depends on | Replaces / adds | Backlog |
|---|---|---|---|---|
| 1 | Condenser with a turbine stub | — | `condensate()`, fixed `PC` | B2 |
| 2 | Circulating water (in/out pipe) | 1 | fixed cooling-water flow and inlet temperature | — |
| 3 | Condenser air removal (SJAE, vacuum pump, off-gas sink) | 1 | vacuum as a given | — |
| 4 | Main steam valves: MSIVs, stop, control, bypass | 1 | the one `mv` valve | B3 |
| 5 | Condensate and feed train split | 1 | the lumped pump and one heater | — |
| 6 | Heater drains | 5 | drains returned to the hotwell at once | — |
| 7 | Condensate storage tank | 5 | makeup from an infinite source | — |
| 8 | Point kinetics with void, Doppler, moderator feedback | — | net heat as an input | B4 |
| 9 | Recirculation loop and jet pump | 8 (useful without it) | one void time constant, no core flow | B5 |
| 10 | Downcomer node and core inlet subcooling | 9 | one well-mixed water region | B7 |
| 11 | Reactor protection: automatic scram and isolation | 4, 8 | scram on the button only | B6 |
| 12 | Vessel heat capacity, real level geometry, suppression pool heat-up | — | steel heat capacity, level span, pool temperature | B8, B10, B11 |
| 13 | Control rods and CRD hydraulics (low priority) | 8, 11 | rods as an all-in/all-out scram | — |
| 14 | Reactor water cleanup (low priority) | 5, 10 | cleanup inside net heat | — |
| 15 | Turbine, MSR, generator and grid (low priority; basic version done) | 1, 4, 5 | the turbine stub | — |
| 16 | Remaining auxiliaries: gland seal, main steam drains, off-gas train (low priority) | 3, 15 | — | — |
| — | Emergency cooling (RCIC, HPCI, core spray, RHR, ADS, isolation condenser) | 11, 12 | deferred | B9 |

Steps 1–7 close and complete the steam–feed loop. Steps 8–11 make the reactor respond like a reactor.
Step 12 can slot in anywhere. Steps 13–16 are low priority and come last: control rods, cleanup, and the turbine island.

## Steam–feed loop

### 1. Condenser with a turbine stub (done)
**Models:** the condenser shell as a steam space over the hotwell. Main steam (later also the bypass and
main steam drains) enters, condenses on the tube bundle, and the condensate falls into the hotwell
at the shell saturation temperature, so condenser pressure and hotwell temperature follow the load
instead of being the fixed 0.01 MPa / 45.8 °C.

**Approach:** one lumped steam space, saturated, at pressure Pc. Condensation rate
w_c = UA·ΔT_lm/h_fg, with ΔT_lm the log-mean difference between T_sat(Pc) and the circulating water
(inlet T_in, outlet T_in + Q/(m_cw·c_p)); at this stage m_cw and T_in are constants (step 2 makes them
operator inputs). Steam-space mass balance dMs/dt = w_in − w_c with Ms·ρ″(Pc) fixing Pc (small volume, so fast,
time constant of a few seconds; solve implicitly or substep). Hotwell gains w_c at T_sat(Pc) (or a little
subcooled, an illustrative 1–2 K), and its temperature becomes a state (mixing with makeup and drains).
Main steam valve and feed pump suction see Pc in place of the constant `PC`.

**Turbine stub** (decided: acceptable as a placeholder): without a turbine the condenser would receive the full steam enthalpy and reject
~3290 MW instead of ~2200 MW. Until step 15, the steam passes a fixed expansion line from the throttle
to Pc with one isentropic efficiency (≈ 0.85), so it arrives wet at the right enthalpy and the stub
reports gross electrical output (≈ 1100 MW at rated). Labelled as a stub on the page.

**Checks:** rated heat balance with Pc ≈ 5–7 kPa and a realistic condenser duty; loss of condenser
cooling raises Pc; vacuum degrades with load. Hotwell level and temperature now meaningful.

**Open:** source for a generic BWR/4-class condenser UA, circulating flow (~30–50 m³/s) and design back pressure;
whether to model hotwell subcooling or keep saturated condensate.

### 2. Circulating water — done
**Models:** the circulating water as an in/out pipe through the condenser (decided: no river, cooling
tower or pump model for now), with a circulating water valve (or pump on/off) and the inlet temperature as
operator inputs, so loss of
circulating water and a warm inlet are things the operator can cause and see.

**Approach:** flow from the valve position against a fixed system head (not a flow setpoint); inlet
temperature an input; condenser outlet temperature from the step 1 heat balance. Tube transit delay optional. Small
enough to ship together with step 1. A pump curve and a heat sink (river or cooling tower) can come later
if scenarios need them.

**Done:** pump on/off against a fixed head (flat, no curve) and system resistance, butterfly valve
w = √(H/(Ksys + Kv/x²)) sized for 38 t/s open, 12 s flow lag, inlet temperature an operator slider 0–35 °C.
The valve has no M/A: there is no controller for it (a CW valve is set open and left), so it is manual
like the pump. No tube transit delay.

### 3. Condenser air removal — done
**Models:** air in-leakage into the shell, removed by the steam jet air ejector (motive steam from the
main steam header) at power and by the mechanical vacuum pump at startup; off-gas as a sink.
Makes "pulling vacuum" a startup step and air binding a failure mode.

**Approach:** air mass in the shell as a state (Dalton with the steam, ideal gas, as the PWR pressuriser
nitrogen does). In-leakage a constant (illustrative, ~10–30 scfm) plus radiolytic H₂/O₂ ∝ power.
SJAE: capacity curve vs suction pressure, needs motive steam pressure above a minimum; vacuum pump:
fixed volumetric flow, only useful above ~20 kPa. Air fraction near the tubes reduces UA (a simple
blanketing factor). A condenser open to atmosphere starts at 0.1 MPa full of air.

**Decided:** the startup scenario includes pulling vacuum: it starts with the condenser at atmospheric
pressure full of air, the mechanical vacuum pump draws it down, then the SJAE takes over once main steam is
available.

**Done:** air mass state, Dalton with the steam; 0.009 kg/s leakage plus 0.02 kg/s radiolytic at rated
(both illustrative); SJAE as an effective 10 m³/s of shell gas (giving 260 Pa of air at rated), motive
steam 2.5 kg/s; vacuum pump 2.2 m³/s fading to 0 at 7 kPa (19 kPa after 1 h); blanketing
UA0/(1 + (ya/0.1)²) with UA0 recalibrated so the rated shell temperature is unchanged (Pc 7.05 → 7.32 kPa,
the air adds its partial pressure); vacuum breaker (manual). The SJAE motive steam is tapped off the steam header between the MSIVs and the turbine valves
(header pressure `s.Pl`), taken out of the line flow ahead of the turbine, then the bypass. No low-vacuum trip
(step 11) and no bypass vacuum interlock (step 4/11).

### 4. Main steam valves: MSIVs, turbine stop and control valves, bypass (done)
**Models:** the one `mv` becomes four valves in series and parallel, as drawn: inboard and outboard
MSIV (open/shut, ~3–5 s closure), turbine stop valve (open/shut, fast), turbine control valve
(throttling, to the turbine stub and on to the condenser), bypass valve (throttling, straight to the
condenser). Gives a load rejection somewhere to send steam besides the SRVs.

**Approach:** each valve a flow coefficient with a characteristic and stroke rate, compressible flow with
the existing subsonic factor; pressures at two nodes (steam line between MSIVs and valves, and the
condenser). Steam line volume optional (a node with a short time constant if it helps stability).
Controller: the pressure regulator moves the control valve to hold dome pressure against a load
reference (load limit); the bypass opens when the pressure demand exceeds what the control valve is
allowed to pass, as on the GE EHC scheme (pressure demand split between control and bypass valves).

**Checks:** turbine trip (stop valve closes) at rated with bypass in A: bypass takes its capacity, SRVs
lift for the rest; MSIV closure isolates the condenser.

**Decided:** bypass capacity about 25 % of rated steam (typical BWR/4; full-capacity bypass is the exception),
so a turbine trip from full power still lifts SRVs.

**Done:** header pressure algebraic (no steam-line volume: a node would have τ ≈ 0.2 s, too fast for the
page's 2 s step). Guesses, labelled illustrative: 0.3 MPa dome-to-throttle drop at rated, MSIVs 15 % of it each
with flow area ∝ position, valves wide open 105 % of rated, control valve characteristic 1 − (1 − x)^1.5 (86 % open
at rated), stroke times except the MSIV closure (stop valve 0.2 s shut / 5 s open, control valve 5 s, bypass
0.5 s). The bypass follows D minus the turbine flow reference (control valve demand × stop valve position)
rather than the measured turbine flow, which made the valves chase each other at dt 2 s. The regulator still
senses dome pressure. Protection hook for step 11: `u.trip = {turbine, msiv}`.

### 5. Condensate and feed train split (done)
**Models:** the lumped pump and single heater split into what the layout draws: condensate pump
(hotwell suction, NPSH from hotwell level and Pc), condensate polisher (pressure drop), LP heater
(extraction from the turbine stub's LP end), feed pump (suction from the condensate header), HP heater
(the present heater), feed valve, check valve.

**Approach:** two pump curves in series with the line resistances between them, solved for one flow (the
pipes hold no inventory) and the intermediate pressure. Feed pump low-suction-pressure trip and
condensate pump cavitation as flow tapers or trips. Two heaters, each the existing condensing-heater
model with its own extraction pressure from the stub's expansion line (∝ steam flow until step 15).
Pump minimum-flow recirculation valve on the feed pump. Feed pump speed control stays (turbine-driven
feed pump; the drive steam itself is not modelled yet).

**Open:** keep one feed pump driven at variable speed (BWR/4 turbine-driven) or motor-driven with a
startup low-flow valve? Booster pump as part of the condensate pump?

**Done (decided in the step):** one turbine-driven variable-speed feed pump, as BWR/4 (drive steam not
modelled), with a run/stop switch; no startup low-flow valve (the feed valve throttles to zero and the
condensate pump alone feeds through the idle feed pump at low vessel pressure). The booster pump is part
of the condensate pump (one constant-speed pump, 2.6 MPa shutoff). The train is solved for three node
pressures (`network()`, nested bisection), not one flow, because the min-flow line branches off.
Illustrative, not sourced: pump curves (split so the rated point of step 1 is unchanged), NPSH required
(2 m at rated) and the 1 m pump pit, the 0.7 MPa/5 s low-suction trip, polisher and LP heater drops, the
0.28 MPa LP extraction. Gross output of the stub with two stages: 1046 MWe (920 with one).

### 6. Heater drains (done)
**Models:** each heater's shell level and its drain path: HP heater drains cascade to the LP heater, LP
drains are pumped forward into the condensate line. Heater level control; high shell level as a
failure (tube leak, drain valve stuck).

**Approach:** shell water inventory per heater (and the drain tank), drain valve on level, drains flashing as they cascade
to lower pressure (energy carried at h_f). Replaces "drains go to the hotwell inside the return".

**Decided:** HP drains cascade to the LP heater, LP heater drains are pumped forward into the condensate
line downstream of the LP heater (drain tank and drain pump), as is common on BWRs; it raises the feed
temperature slightly and keeps the heat out of the condenser.

**Done:** HP shell 30 m³, LP shell with its drain tank 60 m³ as one inventory (one level, labelled), each
saturated at its shell pressure; HP drain valve and drain pump discharge valve on PI level control with
M/A; drain pump start/stop; tube flooding above 60 % (heating ∝ uncovered tubes), high-level dump to the
condenser from 80 %; LP shell vent for flash steam its tubes can't condense; event "HP drain valve fails
shut" (`fail(s, 'hdv')`). All illustrative. With the drains forward the LP extraction falls to 163 kg/s,
the condenser duty to 2158 MW and the stub output rises to 1130 MWe. The feed pump constant was re-chosen so
the rated feed valve stays 74.5 % open.

### 7. Condensate storage tank (done)
**Models:** the CST as a finite tank: hotwell makeup draws from it, reject returns to it; later the
suction source for RCIC/HPCI and CRD.

**Approach:** one mass and temperature state; makeup and reject become valves (position, flow from the
head between tank and condenser vacuum) in place of today's ±100 kg/s flow input. Trivial on its own; good to ship with or right after step 5.

**Done:** 1900 t tank, 12 m tall, one mass and temperature, starting 70 % full at 30 °C; makeup valve to the
condenser driven by the tank head and the vacuum, reject valve from the condensate pump discharge (a branch of
the train solve); split-range PI on the hotwell level with M/A on each valve; an empty tank gives no makeup;
overfilling is a fault. Whole-plant water mass test. Sizes illustrative. Not yet a suction for RCIC/HPCI/CRD.

## Reactor

### 8. Point kinetics with reactivity feedback (done)
**Models:** core power from neutron kinetics instead of a net-heat slider: the pressurisation power
spike on a valve closure, power rising with colder feedwater, power falling with void.

**Approach:** point kinetics with six delayed groups (or one group plus a prompt-jump approximation
for speed at large steps; stiff, so implicit). Fuel temperature as one lumped node with a heat transfer
coefficient to the coolant (time constant ~5 s). Reactivity = rods + void coefficient × core void
fraction + Doppler × (T_fuel − ref) + moderator temperature × (T_mod − ref). Decay heat from the present
`decayFrac` shape built up from the power history (or a few Way–Wigner groups as ODEs). Core void
fraction needs a core node: until step 9, a void fraction derived from the existing void mass Mb and
an assumed core flow. The net-heat slider becomes the rod/reactivity control (step 13 makes it rods).

**Open:** sourced void coefficient (about −1×10⁻³ /% void BOC on a BWR/4?), Doppler, β and Λ for a
BWR/4 core; whether to model the void spatially at all (one node is crude: void feedback depends
strongly on where the voids are).

**Done** (bwr-core.js; numbers in docs/bwr.md, "Core"): six groups, prompt-jump with implicit precursors,
β 0.0056; void −1×10⁻³/%, Doppler −2×10⁻⁵/K, moderator −2×10⁻⁵/K on the inlet temperature, generic BOC
values, labelled illustrative (no plant source found offline). Core void from the core thermal-hydraulics
(one channel, flat axial power, drift flux), not from Mb. Seven decay-heat groups fitted to the old
(1 + t)^−0.2 shape. One rod lever, S-shaped worth, critical at 60 % HZP and 80 % at rated; manual only.
Decided: void spatially as boiling boundary + linear quality, one node; the flat power shape made the
feedwater-temperature response about twice a real core's. Fixed later by a quasi-static axial shape
(one-group diffusion, 24 nodes, local feedback; docs/bwr.md, "Axial shape"): 52 K colder feed now gives +17 %. `u.heat` stays as a fixed-heat test hook.

### 9. Recirculation loop and jet pump (done)
**Models:** one recirculation pump driving one jet pump (standing for all), core flow as a state; power
following flow along a rod line through the void feedback of step 8 (flow control of power). Void
clearance and carry-under depend on flow instead of one time constant.

**Approach:** loop momentum equation, L·dW/dt = ΔP_pump(W, n) + ΔP_jet − ΔP_core(W, void) − ΔP_loop,
with the jet pump as a nozzle/mixing-section model (drive flow, suction flow, M-ratio, N-ratio curve).
Natural circulation at zero pump speed from the downcomer/core density head. Pump speed through the MG set
with a speed limiter; recirculation pump trip as a transient.

**Decided:** motor-generator set drive (BWR/4): the pump motor is fed from a variable-speed MG set whose
speed follows the flow controller through a speed limiter. One loop drawn; scale its flow to stand for two
loops and 20 jet pumps.

**Done** (bwr-recirc.js): drive and core momentum equations with a mixing-section jet pump model, natural
circulation head, homogeneous two-phase multiplier; loss coefficients set from the rated point (4300 kg/s,
1.6 MPa, M = 2, 12,900 kg/s); MG set speed limiter 2 %/s, minimum 20 %, coastdown 5 s; core flow
controller (PI on the MG set speed) with M/A, breaker as a plain switch. Mb now comes from the core and
chimney voids (plus flash voids beyond core boiling with τb). Committed together with step 10: the void
model needs the downcomer for a sensible pressurisation collapse.

### 10. Downcomer node and core inlet subcooling (done)
**Models:** a separate downcomer region where feedwater mixes with the separator return, so core inlet
subcooling responds to feed temperature with a transport delay, and the jet pump suction sees
downcomer water.

**Approach:** split the vessel's liquid into downcomer (subcooled, own energy) and core/riser
(two-phase), linked by the recirculation flow from step 9; the dome and separators stay as today. Level
measured in the downcomer (which is where the instrument taps are).

**Done** (bwr-vessel.js, bwr-sim.js): the downcomer is a fixed 100 m³ (illustrative) with its own specific
internal energy u_dc inside the existing M, U totals; the solve flashes only the core region (u_dc held, so a
pressure rise subcools it); mixing time ~6 s at rated flow stands for the transport delay. Level is the
single free surface, read as the downcomer level. Mass and energy stay exact.

### 11. Reactor protection: automatic scram and isolation (done)
**Done:** bwr-rps.js inside the plant step: the trips listed in docs/bwr.md ("Reactor protection") with
typical BWR/4 setpoints, a mode switch (Shutdown, Startup, Run) with its bypasses, latched trips with an event
log and annunciator tiles on the page, reset only once the signal has cleared, the bypass valve interlock on
low vacuum, and the scram level setpoint setdown in the feedwater controller. High drywell pressure waits for
a drywell model; one logic channel (no voting). A pumped cleanup reject (placeholder for step 14) takes the
startup swell to the condenser.

**Models:** RPS trips (high pressure, low level L3, high flux, MSIV closure position switch, turbine
stop valve closure, low condenser vacuum, high drywell pressure later) and the isolation logic (low-low
level L2 and low steam line pressure close the MSIVs; L8 trips the turbine and feed pump). Scram stays
on the button too; reset needs the trip cleared.

**Approach:** a protection module separate from bwr-control.js (it is not a controller and has no M/A),
latched trips with sourced setpoints and the trip that fired shown on the page. Event log (PWR backlog
item 9) is natural here.

**Open:** sourced BWR/4 trip setpoints (level instrument zero, L1–L8 in inches above instrument zero).

### 12. Vessel and containment details (any time)
**Done** (this branch): steel node (`steelStep`), level in inches above instrument zero with BWR/4 setpoints, bulk-flash voids clearing with their own 2 s time constant, suppression pool heat-up with a placeholder cooler. Guesses: steel mass and UA, the setpoints (typical, plants differ), pool cooler size.
- **Vessel and internals heat capacity** (B10): one steel node with UA to the water.
- **Level by real geometry** (B11): instrument span and zero, swell from bulk flashing slower on
  depressurisation.
- **Suppression pool heat-up** (B8): pool mass and temperature from SRV discharge, a pool cooling
  placeholder (RHR is deferred, so a fixed-UA cooler labelled as such), heat capacity temperature limit.

## Low priority

Done after steps 1–12 (Attila, 2026-11-28: turbine and MSR low priority; control rods and RWCU
deprioritised too).

### 13. Control rods and CRD hydraulics
**Models:** rod position as rod groups (a rod pattern, not 137 rods) withdrawn in notches by the
operator, rod worth curve for reactivity; scram insertion from hydraulic accumulators over ~3 s; CRD
pump flow into the vessel (a small, steady makeup, from the CST).

**Approach:** integral worth curve per group; insertion profile on scram replacing the instant rod
insertion inside `decayFrac`. Rod blocks from the rod worth minimiser / rod sequence control are
optional.

### 14. Reactor water cleanup
**Models:** RWCU: suction from the recirculation loop, regenerative and non-regenerative heat
exchangers, demineraliser, return to the feed line; the reject line to the condenser/radwaste used to
lower level during heatup (water expands; today only the feed valve can manage level).

**Approach:** fixed flow on/off, heat removed by the non-regen exchanger (component cooling not
modelled), regen exchanger effectiveness; reject flow a manual valve. Replaces the cleanup part of the
net-heat input.


### 15. Turbine, MSR, generator and grid (basic version done)
**Done (basic, bwr-turbine.js):** HP and LP turbines with Stodola stage pressures for the four stage groups
(first stage, HP extraction, HP exhaust, LP inlet, LP extraction, condenser), so the extraction pressures come
from the turbine and fall with its flow; the MSR as a pass-through pressure drop; one shaft with an inertia
constant of 5 s, the LP power lagged 5 s behind intercept valves; generator on an infinite 60 Hz grid with a
breaker (sync check, reverse power and turbine-trip trips); the EHC's speed/load channel (load set in MWe on
the grid, a droop speed governor off it) as a third limit on the control valves; power–load unbalance fast
closure, the control valve fast closure scram and the 110 % overspeed trip. Load changes, load rejection,
turbine trip and the turbine roll at startup work. **Still to do:** moisture separation and reheat (the MSR
drains to the heaters, live-steam reheat), more LP casings and heater stages, choked/subsonic control valves
against the first-stage pressure, a grid with frequency.

**Models:** HP turbine, MSR (moisture removal and reheat), LP turbine, generator on one shaft; grid
breaker; turbine speed during a load rejection; extraction pressures from the turbine rather than ∝
steam flow.

**Approach:** stage pressures from Stodola's cone law (stage groups between the extraction points),
stage efficiencies fixed, moisture at the HP exhaust removed in the MSR separator and reheated by live
steam; generator power from the enthalpy drop; shaft speed from torque balance when the breaker is open
(overspeed protection trips the stop valve). Replaces the step 1 stub without changing its interface.

### 16. Remaining auxiliaries
Gland seal steam and its condenser, main steam drains to the condenser (warm-up), off-gas train
(recombiner, delay beds, stack: mostly radiological, a flow path here), extraction non-return valves.
Each small, picked as scenarios need them.

## Deferred

Emergency core cooling: RCIC and HPCI (turbine-driven, from CST or pool), core spray, RHR (LPCI,
shutdown cooling, pool cooling), ADS, isolation condenser. Waits until the protection logic (11), CST
(7) and suppression pool (12) exist, so they have something to start on and somewhere to pump from.

## Cross-cutting

- **Code structure (decided):** bwr-sim.js is split by system at step 1, into modules
  (`bwr-condenser.js`, `bwr-feed.js`, `bwr-core.js`, …) that bwr-sim.js composes, each with pure functions
  and its own state slice, so a later step touches one module.

## Decisions (Attila, 2026-11-28)

- Generic BWR/4-class plant, not one named plant.
- Controls realistic but simple: valve positions and pump/breaker switches, not flows; one lever where
  the plant has many (no full rod setup for now).
- Turbine stub until step 15 is fine.
- Control rods/CRD (13) and RWCU (14) deprioritised, done with the turbine island.
- Circulating water is an in/out pipe for now.
- Bypass about 25 % of rated steam, as is typical.
- Startup includes pulling condenser vacuum.
- Heater drains pumped forward (LP), HP cascading to LP.
- Recirculation pump on an MG set.
- bwr-sim.js split into per-system modules at step 1.

- Feed pump at step 5: one turbine-driven variable-speed pump, no startup low-flow valve.

Still open: the sourced numbers named under each step (condenser UA, kinetics
coefficients, trip setpoints).
