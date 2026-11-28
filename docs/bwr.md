# BWR vessel model (bwr.html)

A separate page beside the PWR pressuriser. Same architecture: `bwr-sim.js` is the plant (pure physics),
`bwr-control.js` the automatic control, both ES modules importing `common.js` (steam tables, liquid
properties). Tests: `tests/bwr*.test.mjs` (shared helpers in `tests/bwr-helpers.mjs`).

## Code layout

`bwr-sim.js` composes per-system modules, each pure functions over its own part of the state, and
re-exports what the page and tests use (so they import `bwr-sim.js` only):

| Module | System | State it owns |
|---|---|---|
| `bwr-vessel.js` | Vessel geometry, water–steam solve (`at`, `solve`, `derive`, `vesselInit`) | `M`, `U`, `Mv`, `Mb`, `d` |
| `bwr-steam.js` | Rated conditions (`QR`, `PR`, `WR`), main steam valve, SRVs (`srvStep`), nozzle factor | `mv`, `spr[]` |
| `bwr-condenser.js` | Turbine stub (`expansion`, `turbinePower`), condenser shell (`condense`, `condSteady`) | `cd {Tc, Ms}` |
| `bwr-feed.js` | Hotwell (`hotwellMix`), condensate pump (`suction`, `cpFlow`), polisher, LP and HP heaters (`stage`, `heaterL`, `heater`, `train`), heater drains (`tubes`, `dumpX`, drain pump `dpFlow`), feed pump, min-flow line, feed valve; the train solve (`network`) | `Mh`, `Thw`, `n`, `fv`, `xr`, `fpt`, `tlo`, `Tsu`, `Th`, shells `Mhs`, `Mls`, `psh`, `psl`, `Ths`, `Tls`, drain valves `xdh`, `xdl`, failures `fail` |

`step()` runs the systems in this order on the start-of-step state: valves and SRVs, steam flow, pump
switches and valve travel, the condensate/feed train solve (`network`), feed pump trip, heaters and turbine
stub, condenser, hotwell, then the vessel mass and energy and the pressure solve.

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `V`, `H`, `A` | 600 m³, 21 m, 28.6 m² | Free fluid volume, inside height, uniform free cross-section V/H |
| `Z` | BAF 5.6, TAF 9.3, SHR 11.5, FW 13.0, NWL 14.3, MSL 16.5 m | Elevations above the inside bottom: active fuel, top of shroud, feedwater sparger, normal water level, main steam nozzle |
| Net heat | −100 … +3400 MW (page) | Into the water: core or decay heat minus cleanup/shutdown cooling and losses. Rated 3293 MW |
| Main steam valve | 2000 kg/s full open at 7.0 MPa (`WMS`, `PR`), ∝ P × subsonic factor ψ(PC/P)/ψ* (κ = 1.3), condenser pressure from the condenser model | Stands for MSIVs + turbine control and bypass valves. Travel ≤ 20 %/s (`RATE`, full stroke 5 s). `s.mv` is the position, `u.mv` the demand |
| SRVs | `SRV` = 11 valves `{p, w}`: 4 × 1090 psig (7.617 MPa abs) rated 800,000 lb/h (100.8 kg/s), 4 × 1100 psig (7.686 MPa) 808,000 lb/h (101.8 kg/s), 3 × 1110 psig (7.755 MPa) 815,000 lb/h (102.7 kg/s); flow w·P/p × subsonic factor to the pool at 0.1 MPa | From a GE BWR/4 training lesson plan (NRC-hosted; group counts partly illegible in the OCR, read so that capacity ∝ set pressure, which it is for one valve model). Each valve: spring opens it fully above p, recloses below p − `BD` (0.25 MPa, illustrative; not in the source), latch `s.spr[i]`; or opened by hand, `u.srv[i]` true. Open = spring or hand. No automatic relief logic, low-low set or ADS |
| Turbine stub | `ETA` 0.85 isentropic along one expansion line from the throttle (dome, saturated) to the condenser pressure; generator `ETAG` 0.985 | `expansion(p0, h0)` → h(p): h0 − η·(h0 − h_s(p)), h_s from the throttle entropy (`satS` in common.js, IAPWS supplementary release). Extraction leaves at h(pe); the rest exhausts wet at h(Pc). `turbinePower(wt, h0, ext, hx)`. Rated: exhaust h 1950 kJ/kg (x ≈ 0.76), about 920 MWe gross with one 2.4 MPa heater stage, 1046 MWe with the LP stage of step 5 and drains to the condenser, 1130 MWe with the drains pumped forward (step 6; a real BWR/4 with five or six stages, moisture losses and reheat makes about 1100 MWe) |
| Condenser | Steam space `VC` 4000 m³; circulating water `WCW` 38,000 kg/s at `TCW` 20 °C (fixed until the circulating water step); `UA` 2.4×10⁸ W/K; `PCMAX` 0.135 MPa | Saturated at Tc. Inflows `[w, h]` split into vapour w·(h − h′)/h_fg at Tc and liquid. Bundle: Q = m·c_p·(Tc − T_in)·(1 − e^(−UA/(m c_p))), w_c = Q/h_fg. Implicit each step: bisection on Tc for ρ″(Tc)·VC = Ms + (vapour − w_c)·dt. Liquid to the hotwell = inflow − storage change, at h′(Tc). Rated (two heater stages, drains pumped forward, step 6): 6.43 kPa, 37.4 °C, 2158 MW (drains to the condenser, step 5: 6.68 kPa, 38.1 °C, 2244 MW; one stage: 7.05 kPa, 39.1 °C, 2370 MW). Illustrative numbers (38 m³/s and ~7 kPa are typical of an 1100 MWe unit; UA chosen to give them). `condSteady(ins, cw)` the steady Tc. Fault `'condenser'` above PCMAX (LP exhaust rupture diaphragms) |
| Hotwell | `HWM` 600 t capacity, normal half full, `HWD` 4 m deep at full (illustrative); its own mass `Mh` and temperature `Thw` | Mixes the condensate (at Tc), makeup at `TMK` 30 °C, and the condensate pump's draw. Design temperature `THW` 37.4 °C (the condenser at rated; the extraction constants are computed with it). Makeup from the condensate storage tank through the makeup valve, reject to it through the reject valve (step 7; the ±100 kg/s `u.mk` input is gone, `MKMAX` is kept as the controller's range) |
| Condensate storage tank (step 7) | `CSTM` 1900 t (about 500,000 US gal), `HCST` 12 m tall, level ∝ mass, open to the air (`PATM`); starts 70 % full (`init({cst})`) at `TMK` 30 °C; its own mass `Mcst` and temperature `Tcst` | Makeup valve (CST → condenser): `KMU`·x·√(PATM + ρg·(h − `ZMU`) − Pc), ZMU 3 m, about 100 kg/s fully open from a 70 % full tank under vacuum (`muFlow`), falling to 0 as the tank goes from 2 % to empty (outlet uncovering); enters the hotwell at h′(Tcst). Reject valve (condensate pump discharge → CST): `KRJ`·x·√(Pcd − PATM − ρgh), about 100 kg/s from 2.4 MPa (`rjBack` the back pressure); a branch of `network` at the condensate pump discharge, so the condensate pump carries it (`wcp` = LP line + reject). The CST mixes the reject at h′(Thw); no heat loss to the air. Both valves stroke ≤ 10 %/s (`CRATE`); `u.mu`, `u.rj` demands, `s.xmu`, `s.xrj` positions. Fault `'cst'` when it overfills. Sizes and elevations illustrative; no fill from demineralised water, no other users (RCIC, HPCI, CRD) yet |
| Train | Hotwell → condensate pump → polisher → LP heater → feed pump → HP heater → feed valve → check valve; min-flow line from the feed pump discharge to the condenser | Pipes hold no water. `network(o)` solves each step for the condensate pump discharge Pcd, feed pump suction Ps and discharge Pd: bisection on Ps outside (supply from the condensate side falls, the feed pump's draw rises with Ps), on Pd and Pcd inside; each branch flow is monotonic in its node pressures. Every pump passes exactly what the lines downstream take (mass). Overall numbers kept from the lumped pump of step 1: total shutoff `H0` 10 MPa over the hotwell, rated discharge PCD + H0 − `KPU`·W² (KPU 0.24), so the feed valve is still 74.5 % open at rated |
| Condensate pump | ΔP = φ·`HC0` − `KCP`·w², HC0 2.6 MPa, KCP 0.08 Pa/(kg/s)² (0.26 MPa at rated); constant speed, run switch `u.cp` | Suction at the impeller Pin = Pc + ρg·(`ZS` + `HWD`·fraction full), ZS 1 m pit. NPSHa = (Pin − p_sat(Thw))/ρg (3 m at half full, saturated); NPSHr = `NPR`·(w/WR)², NPR 2 m. φ = 1 while NPSHa ≥ NPSHr, falling linearly to 0 at NPSHa = NPSHr/2 (closed form: a quadratic in w², `cpFlow`); times hw/0.02 below 2 % (suction uncovering, vortexing). Stopped: a resistance KCP·w² (check valve: no reverse flow). All illustrative |
| Heater drains (step 6) | HP shell `VHS` 30 m³, LP shell with drain tank `VLS` 60 m³ (fraction full `Mhs`/`MHS`, `Mls`/`MLS`), saturated at the shell pressure; HP drain valve `KDVH`·x·√(psH − psL) (linear); drain pump f·`HDP0` − `KDPP`·w², HDP0 3.5 MPa, 0.5 MPa lost at rated drains, suction psL + ρg·`ZDT` (5 m), f = level/0.1 below 10 %, discharge valve conductance `KDVL`·x, check valve; both valves half open (`XD0`) at rated; travel 10 %/s (`DRATE`) | HP drains cascade to the LP shell (they flash there: `stage(…, ins)` takes them as heat w·(h − h′(ps))); LP + cascaded drains are pumped into the condensate line after the LP heater (a branch into the feed pump suction node of `network`). Rated: drains 579 kg/s (`WDD`), LP extraction only 163 kg/s, HP heater inlet `THPI` 127.4 °C (LP outlet 126.5 °C mixed with the drains), condensate pump 1209 kg/s; `KFP` re-chosen (0.181) so the feed valve stays 74.5 % open. Tube flooding: above `LFL` 60 % the heated fraction a = (1 − level)/0.4 and the outlet is Tin + a·(T_sat(ps) − TTD − Tin). High-level dump (plant overflow): above `LDU` 80 % a valve opens (fully at 90 %) to the condenser, `KDDH`/`KDDL`·x·√(ps − Pc), sized for 1.4 × the shell's rated drains. With extraction steam the shell is at most at the extraction pressure (non-return valve); flash steam the tubes can't condense vents to the condenser (`f.wv`). `fail(s, 'hdv', on)`: the HP drain valve fails shut (event). All volumes, levels and pump numbers illustrative |
| Polisher, LP heater drop | `KPOL`·w² 0.13 MPa, `KLP`·w² 0.10 MPa at rated | Polisher a pressure drop only (no chemistry) |
| Feed pump | ΔP = n²·`HF0` − `KFP`·w², HF0 = H0 − HC0 = 7.4 MPa, KFP 0.181 (chosen so the rated discharge is unchanged; 0.099 before the drains were pumped forward); turbine-driven, run switch `u.fp`, speed `u.n` | Speed state `s.n` changes at most 10 %/s (`NRATE`); stopped, tripped or at zero speed the pump is the resistance KFP·w², so below ~2.5 MPa the condensate pump feeds the vessel through it. Shutoff of the two pumps at speed n: `shutoff(n, pc, Mh, dp)` = max(Pin + HC0, drain pump suction + HDP0) + n²·HF0 (10.04 MPa full speed, 7.37 MPa at 80 % on the condensate pump; about 11.2 MPa while the drain pump runs: it injects at the feed pump suction with more head). Low suction pressure trip: Ps < `PLO` 0.7 MPa for `TLO` 5 s sets the latch `s.fpt` (illustrative setting), cleared only by `u.fp` false; while tripped the speed runs down to 0. If Ps would fall below the vapour pressure at the LP heater outlet, the pump cavitates and delivers only what reaches its suction (`f.cav`). Drive steam not modelled |
| Min-flow valve | flow `KR`·xr·√(Pd − Pc), linear; KR sized for `WMIN` = 25 % of rated (447 kg/s) open at 9.9 MPa; stroke ≤ 20 %/s (`RRATE`); `u.rv` demand, `s.xr` position | Returns feed pump discharge water to the condenser at the LP heater outlet enthalpy (it flashes there) |
| HP heater + piping drop | `KLN`·w², KLN = 0.094 Pa/(kg/s)² | 0.3 MPa at rated flow |
| Feed valve | drop w²/(`KV`·x²)², KV = 2.31 (kg/s)/√Pa, quadratic characteristic; stroke ≤ 10 %/s (`FRATE`) | Zero flow if the drop would be negative (check valve). `fwFlow(n, x, P, Mh)` the steady train (condensate pump on, min-flow shut, drain pump at its design opening `DPD`), inverse `fwValve(w, n, P)` (rated drains `WDD` forward). Rated 1788 kg/s at 74.5 % open; 2364 kg/s wide open at 7 MPa (2261 at step 5, the condensate pump short of NPSH; 2400 kg/s before step 5) |
| Feedwater heaters | LP: extraction pressure pe = `PEL`·wst/`WR`, PEL 0.28 MPa; HP: `PE`·wst/WR, PE 2.40 MPa; valve flow `KX`·e·√(pe² − ps²) (`KXL`, `KXT`); shell condenses ws = fw·(h(T_out) − h(T_in))/(h(pe) − h′(ps)); T_out = T_sat(ps) − `TTD` (3 K) | Two stages of the same model (`stage(H, …)`): LP before the feed pump (condensate flow), HP after it (feed flow to the vessel). Extraction pressures ∝ main steam flow (turbine stage pressures are). The shell pressure ps is where the valve passes what the shell condenses (bisection on the shell saturation temperature). Design point: rated flows, 5 % drop across each open valve; LP 126.5 °C out (`TLPD`), ws ≈ 163 kg/s with the HP drains cascaded in (375 kg/s without); HP 216 °C out from `THPI`, ws ≈ 417 kg/s (wet steam on the expansion line). `u.exl` and `u.ext` (e) 0–1 valve openings (`exl` defaults to `ext` if absent). HP outlet follows T_out with `TAU.h`. `heaterL`, `heater(wst, e, fw, Tin, line)` → {T, pe, ps, ws, hx, hd}; `train(wst, eL, eH, fw, Tin, line)` the steady train with the drains cascaded and pumped forward; `heaterT(wst, e, fw, Tin)` both valves at e; `heatBalance(Q, P, e)` the steady {W, Tfw}. Drains: see the row above |
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

Command `u` = {heat W, n 0–1 pump speed, fv 0–1 feed valve demand, ext 0–1 HP heater extraction, exl 0–1
LP heater extraction (absent: = ext), cp, fp pump run switches (absent: running), rv 0–1 min-flow valve
demand, hdv, ldv 0–1 HP drain valve and drain pump valve demands (absent: 0.5), hdp drain pump run switch
(absent: running), mu, rj 0–1 CST makeup and reject valve demands (step 7; replace mk), mv 0–1 steam valve demand, srv[]}. `s.f` records what was
applied, with the feed train's fw, Tfw, Pd (feed pump discharge), Ps (its suction), Pcd (condensate pump
discharge), wp, wcp, wr (feed pump, condensate pump, min-flow), npsh, phi, cav, fpt, pe/ps/ws (HP
extraction pressure, shell, steam), pel/psl/wsl and Tlp (LP), Tt (HP target), ret (condensate return),
mk, Mh, hw (hotwell fraction); drains: LH, LL (shell fractions full), aH, aL (heated tube fractions), wdh
(HP drains to LP), wd (pumped forward), wduh, wdul (dumps to the condenser), wv (LP vent), Tm (HP heater
inlet), xdh, xdl, hdp, hdvf; CST: wmu, wrj (makeup, reject kg/s), xmu, xrj, Mcst, cst (fraction full), Tcst;
mk is now the net makeup wmu − wrj. Reactor protection (step 11) can trip the feed pump by sending `fp` false
(and the condensate or drain pump with `cp`, `hdp`).

## Control (bwr-control.js)

- Pressure regulator: steam valve demand = KP·(P − pset) + I, KP = 3×10⁻⁶ /Pa (0.1 MPa → 30 %), integral
  time 30 s, anti-windup at 0 and 1.
- Feedwater, three-element cascade: flow demand F = main steam flow + KL·(lset − level) + I, KL = 800 kg/s
  per m, integral time 60 s, 0 … 2500 kg/s; then a flow controller on the feed valve, x = KX·(F − fw) + I,
  KX = 10⁻⁴ per kg/s, integral time 4 s, anti-windup at 0 and 1 (the level integral also holds while the
  valve is saturated). SRV flow is not in the steam signal, so after a valve closure the level term alone
  has to call for feedwater. The pump speed is manual: no valve opening helps once vessel pressure is above
  the shutoff head.
- Hotwell level, split range (step 7): mk = (condensate pump flow − reject − return) + KH·(0.5 − hotwell fraction) + I,
  KH = 1000 kg/s per unit fraction, integral 300 s, ±100 kg/s, setpoint fixed at half full. The makeup valve
  demand is mk/100 above 0, the reject valve's −mk/100 below 0 (1 % per kg/s, about each valve's flow at a 70 %
  full CST), so they are never both asked open. Each valve has M/A (`auto.mu`, `auto.rj`; absent keys follow
  `auto.mk`); with both in M the integral holds the operators' net (mu − rj, % read as kg/s). The reject is
  left out of the feedforward because it is the loop's own output. A PI on an integrating level overshoots
  from an initial offset: from 60 % full it undershoots to about 46.6 % and settles in about 40 min.
- Feed pump speed: n = KN·(dpset − (Pd − P)) + I, KN = 1.5×10⁻⁸ /Pa, integral 10 s, 0 … 1: holds the pump
  discharge 2.2 MPa (default) above the dome, as turbine-driven feed pumps are run, so the feed valve keeps a
  steady drop at any vessel pressure.
- Heater extraction valves (LP and HP): manual only. Pump run switches: manual only.
- Heater drain levels: each valve x = KD·(level − 0.5) + I, KD 4 (a quarter of the shell over the setpoint
  strokes it fully), integral 30 s, direct acting, anti-windup at 0 and 1: the HP drain valve on the HP
  shell, the drain pump valve on the LP shell/drain tank. A loop whose key is absent from `c.auto` runs in A
  (callers older than step 6).
- Feed pump minimum flow: rv = clamp((`WMIN` − fw)/WMIN, 0, 1), proportional only on the forward feed
  flow (WMIN 447 kg/s, 25 % of rated): fully open at no forward flow, shut above 25 %.
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
  peaks just above 7.6 MPa; the feed cools to the hotwell temperature (no extraction steam) and the level peaks about 1.6 m
  above normal (about 1.2 m with the pump speed in A). Isolation at 200 MW: the lowest bank cycles 7.35–7.62 MPa, about one lift per 45 s.
- Both extraction valves to 20 % at rated: feed about 38 K colder after 1 min, 72 K after 6 min (one stage: 35 and 54 K). With everything in
  A the steam flow settles 12 % lower and the level hardly moves; in M the falling pressure lets the pump
  deliver more, and the vessel floods in under 3 min.
- Pump speed in A through a depressurisation to 1 MPa: discharge held 2.2 MPa above the dome, speed down to
  about 55 %.
- Hotwell: without makeup it drains during an isolation (relief steam does not return); with makeup in A it
  stays half full.
- CST (tests/bwr-cst.test.mjs): makeup fully open 100 kg/s at 70 % full under vacuum, less with the vacuum lost;
  reject fully open at rated 80–120 kg/s; whole-plant water (vessel + hotwell + condenser steam + heater shells +
  CST) changes only by SRV discharge, to 1 kg, at dt 0.25 and 2 s; an empty CST gives no makeup and the hotwell
  drains under an open SRV. A full hotwell with the steam valve open is the `'hotwell'` fault.
- Cold feedwater (about 100 kg/s at the hotwell temperature, no heat): pressure falls ~0.35 MPa/min, subcooling < 1 K.
- Condensate and feed train (tests/bwr-feed.test.mjs): rated suction 2.16 MPa, condensate pump discharge 2.38 MPa,
  feed pump discharge 9.24 MPa as with the lumped pump, LP heater 126.5 °C out, extractions 421 kg/s (HP) and
  375 kg/s (LP), stub output 1046 MWe; condensate pump stopped at rated: the feed pump cavitates at once and
  trips after 5 s, stays tripped until its switch is cycled, then runs back up at 10 %/s; condensate pump
  alone feeds a vessel at 0.5 MPa through the idle feed pump, nothing above its 2.6 MPa shutoff; min-flow
  valve in A fully open with the feed valve shut, pump flow ≥ 0.9·WMIN.
- Heater drains (tests/bwr-feed.test.mjs): rated drains 579 kg/s pumped forward, levels held at 50 % through
  the 3293 → 2000 MW load step in A (dt 0.25 and 2 s), no dump. HP drain valve failed shut at rated: shell
  levels off near 83 % on its dump, about 57 % of the tubes flooded, feed about 158 °C after 10 min, steam
  flow 12 % lower; repaired, back to 216 °C. Drain pump stopped: LP shell dumps ~640 kg/s, LP outlet ~65 °C,
  feed ~211 °C, output ~1015 MWe. Whole-loop mass (vessel, hotwell, condenser steam, both shells) conserved
  to 1 g with a dump and an SRV open.
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
- Heater drains (step 6, solid; not on the original layout): HP drain from the HP heater down through its
  valve to the LP heater, LP drain up to the drain tank, drain pump and its discharge valve into the
  condensate line just before the feed pump. The high-level dumps to the condenser are not drawn (their
  flows are labelled at the heaters).
- Condensate and feed (step 5, solid): hotwell → condensate pump → polisher → LP heater (extraction from the
  LP turbine) → feed pump (min-flow line back into the condenser shell) → HP heater (extraction from the HP
  turbine casing) → feed valve → containment. The condensate storage tank (step 7, solid) stands right of the hotwell: makeup line from its foot
  into the hotwell through its valve, reject line from the condensate pump discharge under the hotwell into the
  tank's bottom, each valve labelled with its flow.

Not on the layout yet, for the steam/feed circuit: MSR drain lines, the MSR/heater non-return
valves, the off-gas train (recombiner, delay beds, stack), turbine gland
seal steam and its condenser, circulating water pumps and the cooling tower or river, reactor water cleanup (RWCU), control rod drive
hydraulics, main steam drains to the condenser, generator output and grid breaker. Emergency cooling
(RCIC, HPCI, core spray, RHR) is deliberately left for later.

## Not modelled

Neutronics and void/pressure reactivity feedback, automatic scram (scram is an operator button), recirculation pumps and jet pumps, downcomer
subcooling separate from the core, steam superheat, air in the dome (below 100 °C the pressure goes
sub-atmospheric), vessel and internals heat capacity, turbine (a stub: one expansion line), air in the condenser, circulating water as a system (fixed flow and inlet), moisture separator reheaters, more than two heater stages, drain coolers, heater shell steam volume and heat capacity (the shell pressure follows the steam flow at once) (one throttled extraction line per stage; real extraction lines have only isolation and non-return valves), redundant pumps (one condensate pump stands for the condensate and booster pumps, one feed pump for three), feed pump drive steam and the pump's heating of the water, polisher chemistry, level-8 feed pump trip, startup
low-flow valve, condensate and feed water inventory in the pipes, cleanup reject flow, suppression-pool heat-up. Void clearance uses one time constant at every power and pressure,
so swell from bulk flashing during a depressurisation is underestimated.
