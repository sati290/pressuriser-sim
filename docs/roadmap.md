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
| 15 | Turbine, MSR, generator and grid (low priority) | 1, 4, 5 | the turbine stub | — |
| 16 | Remaining auxiliaries: gland seal, main steam drains, off-gas train (low priority) | 3, 15 | — | — |
| — | Emergency cooling (RCIC, HPCI, core spray, RHR, ADS, isolation condenser) | 11, 12 | deferred | B9 |

Steps 1–7 close and complete the steam–feed loop. Steps 8–11 make the reactor respond like a reactor.
Step 12 can slot in anywhere. Steps 13–16 are low priority and come last: control rods, cleanup, and the turbine island.

## Steam–feed loop

### 1. Condenser with a turbine stub
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
the air adds its partial pressure); vacuum breaker (manual). Guess: the SJAE motive steam is taken off the
main steam flow ahead of the turbine, because the one `mv` includes the MSIVs (so a shut `mv` leaves the
SJAE without steam); step 4 should tap it from the steam-line node between the MSIVs and the turbine
valves (`motiveP(s)` in bwr-sim.js is the one place to change the pressure). No low-vacuum trip
(step 11) and no bypass vacuum interlock (step 4/11).

### 4. Main steam valves: MSIVs, turbine stop and control valves, bypass
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

### 5. Condensate and feed train split
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

### 6. Heater drains
**Models:** each heater's shell level and its drain path: HP heater drains cascade to the LP heater, LP
drains are pumped forward into the condensate line. Heater level control; high shell level as a
failure (tube leak, drain valve stuck).

**Approach:** shell water inventory per heater (and the drain tank), drain valve on level, drains flashing as they cascade
to lower pressure (energy carried at h_f). Replaces "drains go to the hotwell inside the return".

**Decided:** HP drains cascade to the LP heater, LP heater drains are pumped forward into the condensate
line downstream of the LP heater (drain tank and drain pump), as is common on BWRs; it raises the feed
temperature slightly and keeps the heat out of the condenser.

### 7. Condensate storage tank
**Models:** the CST as a finite tank: hotwell makeup draws from it, reject returns to it; later the
suction source for RCIC/HPCI and CRD.

**Approach:** one mass and temperature state; makeup and reject become valves (position, flow from the
head between tank and condenser vacuum) in place of today's ±100 kg/s flow input. Trivial on its own; good to ship with or right after step 5.

## Reactor

### 8. Point kinetics with reactivity feedback
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

### 9. Recirculation loop and jet pump
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

### 10. Downcomer node and core inlet subcooling
**Models:** a separate downcomer region where feedwater mixes with the separator return, so core inlet
subcooling responds to feed temperature with a transport delay, and the jet pump suction sees
downcomer water.

**Approach:** split the vessel's liquid into downcomer (subcooled, own energy) and core/riser
(two-phase), linked by the recirculation flow from step 9; the dome and separators stay as today. Level
measured in the downcomer (which is where the instrument taps are).

### 11. Reactor protection: automatic scram and isolation
**Models:** RPS trips (high pressure, low level L3, high flux, MSIV closure position switch, turbine
stop valve closure, low condenser vacuum, high drywell pressure later) and the isolation logic (low-low
level L2 and low steam line pressure close the MSIVs; L8 trips the turbine and feed pump). Scram stays
on the button too; reset needs the trip cleared.

**Approach:** a protection module separate from bwr-control.js (it is not a controller and has no M/A),
latched trips with sourced setpoints and the trip that fired shown on the page. Event log (PWR backlog
item 9) is natural here.

**Open:** sourced BWR/4 trip setpoints (level instrument zero, L1–L8 in inches above instrument zero).

### 12. Vessel and containment details (any time)
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


### 15. Turbine, MSR, generator and grid
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

Still open: the feed pump drive at step 5 (turbine-driven variable speed, as now, or motor-driven with a
startup low-flow valve), and the sourced numbers named under each step (condenser UA, kinetics
coefficients, trip setpoints).
