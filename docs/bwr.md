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
| `bwr-steam.js` | Rated conditions (`QR`, `PR`, `WR`), main steam line (`steamLine`, `cvPos`, `valveStep`: MSIVs, stop, control and bypass valves), SRVs (`srvStep`), nozzle factor | `ms[2]`, `sv`, `cv`, `bp`, `Pl`, `spr[]` |
| `bwr-condenser.js` | Turbine stub (`expansion`, `turbinePower`), condenser shell (`condense`, `condSteady`), circulating water (`cwTarget`, `cwStep`), condenser air (`airFlows`, `airStep`, `sjSteam`, `sjCap`, `vpCap`, `blanket`) | `cd {Tc, Ms, Ma}`, `wcw`, `vb` |
| `bwr-feed.js` | Hotwell (`hotwellMix`), feed pump, heater, feed valve | `Mh`, `Thw`, `n`, `fv`, `Th` |
| `bwr-vessel.js` (end) | Vessel steel (`steelStep`), level instruments (`IZ`, `LV`, `toIn`, `fromIn`), bulk-flash void clearance `TAUF` | `Ts`, `Mbf` |
| `bwr-containment.js` | Suppression pool (`poolStep`, `poolLevel`, limits `PLIM`) | `pool {M, T}` |

`step()` runs the systems in this order on the start-of-step state: valves and SRVs, steam line (header pressure, turbine and bypass flows), feed
pump and valve, circulating water and condenser air flows, heater and turbine stub, condenser, hotwell, then the vessel mass and energy and the
pressure solve.

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `V`, `H`, `A` | 600 m³, 21 m, 28.6 m² | Free fluid volume, inside height, uniform free cross-section V/H |
| `Z` | BAF 5.6, TAF 9.3, SHR 11.5, FW 13.0, NWL 14.3, MSL 16.5 m | Elevations above the inside bottom: active fuel, top of shroud, feedwater sparger, normal water level, main steam nozzle |
| Net heat | −100 … +3400 MW (page) | Into the water: core or decay heat minus cleanup/shutdown cooling and losses. Rated 3293 MW |
| Main steam line | Dome → line → inboard MSIV → outboard MSIV → header (throttle, pressure `Pl`). Inflow w = √((P² − Pl²)/(`RL` + `RM`/a1² + `RM`/a2²)), a = MSIV position (flow area ∝ position). `DPL` 0.3 MPa dome-to-throttle drop at rated flow (`PL0` 6.7 MPa), each MSIV 15 % of the R | Pipes hold no steam: Pl solved algebraically each step (bisection) so the inflow equals the outflow (a header node would have τ ≈ 0.2 s, too fast for dt 2 s). MSIVs shut and a valve beyond open: Pl = condenser pressure; everything beyond shut: Pl = P. Illustrative numbers (typical of the class), not from one plant |
| Turbine stop + control valves | w_t = `KT`·sv·fcv(cv)·Pl·ψ(Pc/Pl)/ψ*; fcv(x) = 1 − (1 − x)^1.5; `KT` so that valves wide open pass 105 % of rated (`VWO`) at 7.0 MPa dome | Choked ∝ Pl, subsonic factor near the condenser pressure (κ = 1.3). Control valve 86 % open at rated (`cvPos(w, P)` the lift for a flow). The characteristic is an illustrative shape, flattening near full open as a set of sequentially opened valves does. Stop valve: open/shut switch `u.sv`, trips shut in 0.2 s (`SVCLOSE`), opens in 5 s (`SVOPEN`). Control valve strokes in 5 s (`CVRATE`) |
| Bypass valve | w_b = `KB`·bp·Pl·ψ, `KB` = 0.25·WR/PL0 (`BPF` 25 % of rated at the rated throttle pressure, decided), linear; strokes in 0.5 s (`BPRATE`) | From the header straight to the condenser upper shell, throttled at constant enthalpy, so it arrives superheated (h of the dome steam); `condense()` takes any enthalpy. Stroke time illustrative (fast opening on a trip) |
| MSIVs | Two in series, open/shut switches `u.ms1`, `u.ms2`; 4 s stroke each way (`MSRATE`) | Closure 3–5 s is the usual specification. Undefined switch commands hold the valve. Protection hook for step 11: `u.trip = {turbine, msiv}` shuts the stop valve / both MSIVs whatever the switches |
| SRVs | `SRV` = 11 valves `{p, w}`: 4 × 1090 psig (7.617 MPa abs) rated 800,000 lb/h (100.8 kg/s), 4 × 1100 psig (7.686 MPa) 808,000 lb/h (101.8 kg/s), 3 × 1110 psig (7.755 MPa) 815,000 lb/h (102.7 kg/s); flow w·P/p × subsonic factor to the pool at 0.1 MPa | From a GE BWR/4 training lesson plan (NRC-hosted; group counts partly illegible in the OCR, read so that capacity ∝ set pressure, which it is for one valve model). Each valve: spring opens it fully above p, recloses below p − `BD` (0.25 MPa, illustrative; not in the source), latch `s.spr[i]`; or opened by hand, `u.srv[i]` true. Open = spring or hand. No automatic relief logic, low-low set or ADS |
| Turbine stub | `ETA` 0.85 isentropic along one expansion line from the throttle (the header at Pl, with the dome steam's enthalpy) to the condenser pressure; generator `ETAG` 0.985 | `expansion(p0, h0)` → h(p): h0 − η·(h0 − h_s(p)), h_s from the throttle entropy (`satS` in common.js, IAPWS supplementary release). Extraction leaves at h(pe); the rest exhausts wet at h(Pc). `turbinePower(wt, h0, ext, hx)`. Rated: exhaust h 1950 kJ/kg (x ≈ 0.76), about 920 MWe gross with the one 2.4 MPa heater stage (a real BWR/4 with several stages and reheat makes about 1100 MWe; step 5 adds the LP stage) |
| Condenser | Shell `VC` 4000 m³ of steam and air at one temperature Tc: Pc = p_sat(Tc) + Ma·R·Tc/VC (Dalton, R = 287); design circulating water `WCW` 38,000 kg/s at `TCW` 20 °C; bundle `UA0` 2.70×10⁸ W/K clean, `UAD` (`UA`) 2.4×10⁸ W/K at the rated air fraction; blanketing UA = UA0/(1 + (ya/`YA`)²), ya = pa/Pc, YA = 0.1; `PCMAX` 0.135 MPa | Steam saturated at Tc. Inflows `[w, h]` split into vapour w·(h − h′)/h_fg at Tc and liquid. Bundle: Q = m·c_p·(Tc − T_in)·(1 − e^(−UA/(m c_p))), w_c = Q/h_fg. Implicit each step: bisection on Tc for ρ″(Tc)·VC = Ms + (vapour − w_c)·dt, the air mass (updated first) held fixed over the solve, so the solve stays one-dimensional; UA follows ya(Tc) inside it. Liquid to the hotwell = inflow − storage change, at h′(Tc). Rated: 7.32 kPa (7.06 kPa steam + 0.26 kPa air), 39.2 °C, 2374 MW, CW out 34.9 °C. Illustrative numbers (38 m³/s and ~7 kPa are typical of an 1100 MWe unit; UA chosen to give them). `condSteady(ins, cw, pa)` the steady Tc with air partial pressure pa at the tubes (default `PA0`). Condensate leaves at Tc, i.e. subcooled against Pc when there is air. Fault `'condenser'` above PCMAX (LP exhaust rupture diaphragms) |
| Circulating water | One pump (`u.cwp` on/off) against a fixed system: w = √(`HCW`/(`KSYS` + `KCV`/x²)), HCW 0.15 MPa flat (no pump curve), 10 % of it across the open valve, so x = 1 passes WCW; x = 0.5 passes 33.3 t/s, 0.3 passes 26.8 t/s. First-order lag `TAUCW` 12 s (run-up and coast-down). Inlet temperature `u.tcw`, an operator input | In/out pipe only (decided): no river, cooling tower or pump curve. Butterfly valve `u.cwv` 0–1, no stroke limit (the flow lag covers it). Illustrative numbers. State `s.wcw` |
| Air in-leakage | `WLK` 0.009 kg/s (≈ 15 scfm) × nozzle factor from `PATM` 0.101325 MPa (constant at vacuum, zero at atmospheric); radiolytic gas `WRAD` 0.02 kg/s × Q/QR × main steam share of the steam leaving the vessel | Illustrative. The radiolytic H₂/O₂ is counted as air (R = 287) and is not taken out of the vessel inventory (10⁻⁵ of the steam flow). State `s.cd.Ma` |
| Steam jet air ejector | Removes `VSJ` 10 m³/s of shell-average gas (VSJ·Ma/VC kg/s of air) × clamp((Pm − 0.7 MPa)/0.5 MPa) × clamp((35 kPa − Pc)/10 kPa); motive steam `WSJ` 2.5 kg/s × clamp(Pm/1.2 MPa, 0, 1) | `u.sj` motive steam valve. Pm = `motiveP(s)`, the header pressure Pl (downstream of the MSIVs). The motive steam is taken off the main steam line flow: from the turbine flow first, then the bypass flow, at most wst; with the MSIVs shut the SJAE has no steam. It goes to the condenser at h″. The heater's extraction pressure still follows wst (0.14 % off at rated). VSJ is an effective volume: a real ejector draws air-rich gas from the air-cooling section through a much smaller suction. Rated air partial pressure `PA0` = (WLK + WRAD)·R·T/VSJ = 260 Pa |
| Mechanical vacuum pump | `VVP` 2.2 m³/s × clamp((Pc − 7 kPa)/23 kPa, 0, 1) | `u.vp` on/off. From atmosphere (empty condenser, 20 °C water) to 39 kPa in 30 min, 19 kPa in 1 h, 12 kPa in 2 h, ~9 kPa in 5 h |
| Vacuum breaker | `WVB` 20 kg/s × opening × nozzle factor from atmosphere, never filling past atmospheric within a step; strokes in 20 s (`VBRATE`) | `u.vb` open/shut, manual. From rated vacuum to atmospheric in about 5 min. No reverse flow when the shell is above atmospheric |
| Hotwell | `HWM` 600 t capacity, normal half full; its own mass `Mh` and temperature `Thw` | Mixes the condensate (at Tc), makeup at `TMK` 30 °C, and the feed drawn off. Design temperature `THW` 39.1 °C (the condenser at rated; the extraction constant is computed with it). Makeup from / reject to the condensate storage tank `u.mk`, ±100 kg/s (`MKMAX`) |
| Feed pump | ΔP = n²·`H0` − `KPU`·w², H0 = 10 MPa, KPU = 0.24 Pa/(kg/s)²; suction at the condenser pressure | Condensate, booster and feed pumps lumped. Speed `u.n` 0–1 (affinity), state `s.n`, changes at most 10 %/s (`NRATE`). Shutoff 10.01 MPa at full speed, 6.4 MPa at 80 %. Loses suction below 5 % hotwell: flow × Mh/(0.05·HWM) |
| Heater + piping drop | `KLN`·w², KLN = 0.094 Pa/(kg/s)² | 0.3 MPa at rated flow |
| Feed valve | drop w²/(`KV`·x²)², KV = 2.31 (kg/s)/√Pa, quadratic characteristic; stroke ≤ 10 %/s (`FRATE`) | Flow w = √((n²H0 + PC − P)/(KPU + KLN + 1/(KV x²)²)), zero if the root is not positive (check valve). `fwFlow(n, x, P, Mh)`, inverse `fwValve(w, n, P)`. Rated 1788 kg/s at 74.5 % open, 2400 kg/s wide open at 7 MPa |
| Feedwater heater | Extraction pressure pe = `PE`·wst/`WR` (PE = 2.40 MPa, WR = 1788 kg/s); extraction valve flow `KXT`·e·√(pe² − ps²); shell condenses ws = fw·(h(T_out) − h(T_in))/(h″ − h′)(ps); T_out = T_sat(ps) − `TTD` (3 K) | One stage, a shell-and-tube condensing heater, for the whole heater train. Extraction pressure ∝ main steam flow (turbine stage pressures are). The shell pressure ps is where the valve passes what the shell condenses (bisection on the shell saturation temperature). Design point: rated flows, 216 °C out, 5 % drop across the open valve, ws ≈ 815 kg/s of wet steam (latent heat taken as h(pe) on the expansion line minus h′(ps)). `u.ext` (e) 0–1 valve opening. Outlet follows T_out with `TAU.h`. `heater(wst, e, fw, Tin, line)` → {T, pe, ps, ws, hx, hd}; `heaterT`; `heatBalance(Q, P, e)` the steady {W, Tfw}. Drains go to the condenser at h′(ps), where they flash |
| `TAU.c` | 10 s | Subcooled water condenses steam towards saturation |
| `TAU.b` | 0.7 s | Steam bubbles (voids) leave the two-phase mixture; gives ~34 m³ of voids (1.2 m swell) at rated steaming |
| `TAU.h` | 60 s | Feedwater heater outlet temperature (heater and piping holdup) |
| `TAUF` | 2 s | Voids from bulk flashing (generation beyond what the heat makes: gen − (Q + Q_steel)/h_fg) rise out more slowly than core boiling voids; state `Mbf`, part of the level. Illustrative |
| Vessel steel | `CST` 6×10⁸ J/K (≈1200 t at 500 J/kg/K), `UAS` 1 MW/K to the water, `UAL` 4200 W/K to the drywell at `TDW` 50 °C | One node, implicit exponential (`steelStep`), exact energy split: heat to the water Qs and loss Ql are the step means. About 1 MW lost at rated; lags the water ~10 K at 55 °C/h. Illustrative. Init: steady between water and drywell |
| Level instruments | Instrument zero `IZ` 528 in (13.41 m) above vessel zero; `LV` L8 +54, L7 +39, L4 +27, L3 +12.5, L2 −38, L1 −129 in | Typical BWR/4 (TAF = −162 in, normal ≈ +35 in). `toIn(z)`, `fromIn(in)`. `s.f.lin` the level in inches. No reference-leg or density errors: the instrument reads the model level |
| Suppression pool | `MPOOL` 3.8×10⁶ kg at `TP0` 32 °C, surface `APOOL` 1100 m²; cooler `UAP` 1.5 MW/K against `TSW` 30 °C when `u.pc`; stop at `TPMAX` 100 °C | Mark I-type, generic. SRV steam condenses completely (mass and h″ into the pool). Limits `PLIM`: 35 °C normal, 43.3 °C (110 °F) scram, 48.9 °C (120 °F) depressurise (typical BWR/4 technical specifications); shown on the page, no automatic action. Cooler is a placeholder for RHR pool cooling (deferred). Fault `'pool'` |

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
steam `Mb` (part of `Mv`), main steam valve positions `ms[0]`, `ms[1]` (MSIVs), `sv`, `cv`, `bp` and the header
pressure `Pl`, SRV spring latches `spr[]`; feedwater train: feed valve
position `fv`, heater outlet temperature `Th`, hotwell mass `Mh`.

Each step (`step(s, u, dt)`):
1. Each main steam valve moves toward its demand at its stroke rate (`valveStep`); `steamLine` gives the header pressure and the turbine and bypass flows; each SRV spring latch from P; an SRV is open if its spring or `u.srv[i]` holds it.
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

Air balance each step (`airStep`): dMa/dt = w_in − V_r·Ma/VC with w_in (leakage, radiolytic, breaker) and V_r (SJAE + vacuum pump, their capacity factors at the start-of-step Pc) held over the step, integrated exactly: Ma' = Me + (Ma − Me)·e^(−V_r·dt/VC), Me = w_in·VC/V_r. `s.f.wain` and `s.f.waout` are the air flows in and out; their integral closes the air mass exactly.

`fault(s)`: `'uncovered'` level below TAF, `'flooded'` level at the steam nozzle, `'critical'` above
21.5 MPa, `'hotwell'` hotwell over capacity (condenser flooding), `'range'` if the solve fails.

Command `u` = {heat W, n 0–1 pump speed, fv 0–1 feed valve demand, ext 0–1 heater extraction, mk kg/s
makeup (negative: reject), ms1, ms2, sv booleans (MSIV and stop valve switches, true open; undefined holds),
cv 0–1 control valve and bp 0–1 bypass demands, trip {turbine, msiv} (protection hook), srv[], cwp CW pump on, cwv 0–1 CW valve, tcw K CW
inlet, sj SJAE motive steam open, vp vacuum pump on, vb vacuum breaker open}. Absent cwp, cwv, tcw, sj mean the
normal running line-up (on, open, 20 °C, in service); absent vp, vb mean off and shut. A protection trip
(step 11) can act through the same fields (cwp false is a CW pump trip). `s.f` adds
`Pl` (header pressure), `wt` (turbine flow), `wb` (bypass flow), and the valve positions; `wst` = wt + wb is the
main steam line flow (what the feedwater controller's steam-flow signal measures). `s.f` records what was applied, with the feed
train's fw, Tfw, Pd (pump discharge), pe (extraction pressure), Tt (heater target), ret (condensate
return), mk, Mh, hw (hotwell fraction); condenser: Pc, Tc, pa (air partial pressure), ya, Ma, wcw, Tcwi
(inlet), Tcw (outlet), wsj (motive steam), sjc and vpc (capacity factors), wlk, wrad, wvb, wain, waout (air kg/s).

## Control (bwr-control.js)

- Pressure regulator (GE EHC scheme, simplified): steam flow demand D (fraction of rated) = KP·(P − pset) + I,
  KP = 3.4×10⁻⁶ /Pa (0.1 MPa → 34 % of rated), integral time 30 s. Control valve: min(D, load limit `c.ll`,
  default 105 %); bypass: D − turbine flow reference, the reference being the control valve's flow demand (its
  slider's flow when in M) times the measured stop valve position, so a turbine trip hands the whole demand to
  the bypass. Each flow becomes a lift through the valve characteristic at the rated throttle pressure (no
  pressure compensation): `cvLift(D)`, bp = (D − ref)/0.25. The brief suggested the measured turbine flow for
  the bypass; with the page's 2 s step that lags a step and the two valves chase each other, so the reference
  is used. Anti-windup: rising stops when no valve in A can pass more; falling stops at D = 0 (I falls only as
  far as −KP·e, so the pressure still returns to the setpoint after a big transient). With both valves in M the
  integral tracks the flow the two sliders pass. The real EHC senses throttle pressure; this one the dome.
  Constants duplicated from bwr-steam.js (`WR`, `PL0`, `KT`, `BPF`); a test checks they match.
- MSIVs and the turbine stop valve: switches, always manual.
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
- Circulating water (pump, valve, inlet temperature), SJAE motive steam, vacuum pump and vacuum breaker:
  manual only, passed through by `command()` (no controller: the CW valve is set open and left there in
  normal operation, so an M/A would have nothing to do).
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
- Steam line (tests/bwr-steam.test.mjs): 0.3 MPa dome-to-throttle at rated, control valve 86 % open, 105 % wide
  open, bypass 25 % at 6.7 MPa (26 % with the turbine shut, the header nearer dome pressure). Rated with
  everything in A steady at dt 0.25 and 2 s. Turbine trip at rated, no scram: bypass fully open 0.75 s after the
  trip (dt 0.25; two steps at dt 2), all 11 SRVs open within 20 s and pressure passes 8 MPa. With a scram: peak
  7.18 MPa (7.42 at dt 2), no SRV lifts, back to 7.0 MPa on the bypass (decay heat about 25 kg/s after 20 min).
  Load limit 80 % at rated: turbine 80 %, bypass 17 % of rated, pressure at the setpoint. MSIV closure: no
  steam past the MSIVs, no condensate. Startup with the stop valve shut: the bypass in A holds 1 MPa.
- MSIV closure at rated heat (the old main steam valve closure tests): level shrinks > 0.4 m in 6 s (void collapse), pressure > 7.4 MPa. With no scram,
  pressure climbs past the feed pump's 10 MPa shutoff head in about 30 s, feed stops although the valve opens
  fully, and the core uncovers after about 80 s. With a scram, only the 1090 psig bank lifts and pressure
  peaks just above 7.6 MPa; the feed cools to the hotwell temperature (no extraction steam) and the level peaks about 1.6 m
  above normal (about 1.2 m with the pump speed in A). Isolation at 200 MW: the lowest bank cycles 7.35–7.62 MPa, about one lift per 45 s.
- Extraction valve to 20 % at rated: feed about 35 K colder after 1 min, 54 K after 6 min. With everything in
  A the steam flow settles 12 % lower and the level hardly moves; in M the falling pressure lets the pump
  deliver more, and the vessel floods in under 3 min.
- Pump speed in A through a depressurisation to 1 MPa: discharge held 2.2 MPa above the dome, speed down to
  about 55 %.
- Hotwell: without makeup it drains during an isolation (relief steam does not return); with makeup in A it
  stays half full. A full hotwell with the steam valve open is the `'hotwell'` fault.
- Cold feedwater (about 100 kg/s at the hotwell temperature, no heat): pressure falls ~0.35 MPa/min, subcooling < 1 K.
- Startup 30 MW from 100 °C, valve shut: 60–110 K/h, level up > 0.8 m in an hour from expansion.
- Load step 3293 → 2000 MW with both controllers in A: pressure and level back to setpoint within 20 min.

- Circulating water and air (tests/bwr-cw.test.mjs, tests/bwr-air.test.mjs): loss of the CW pump at rated
  takes the condenser to 0.135 MPa in about 28 s (15 kPa at 10 s); CW inlet 10/20/30/35 °C gives 4.6/7.3/11.9/15.1
  kPa at rated; CW valve 50 %/30 % gives 7.9/9.3 kPa. SJAE holds 260 Pa of air (11.6 kg) at rated, removing the
  104 kg/h that leaks and arrives with the steam; without its motive steam the air builds at 104 kg/h and the
  back pressure reaches 19 kPa in 1 h, 52 kPa in 4 h. Vacuum pump pull-down above; startup: after an hour on
  the vacuum pump, 30 kg/s of steam with the SJAE in service brings the shell to < 4 kPa within 30 min.
  The air mass integral closes exactly at dt 0.25 and 2 s.

## Plant layout (draft)

The mimic draws the whole plant, one of each component (no redundant trains). It spans the page width with
the controls below it; the canvas is at least 1180 px wide and scrolls sideways on smaller screens (phones:
use landscape). There is no separate phone layout. Solid parts are modelled; dashed grey parts are layout only, to be made functional step by step.

- Inside the primary containment (dotted outline): vessel, SRVs and suppression pool, one recirculation
  loop (suction from the downcomer, pump below the core, back to one jet pump in the downcomer), the
  inboard MSIV. Feed check valve where the feed line leaves containment.
- Main steam (modelled, solid): inboard and outboard MSIV, then a tee for the bypass line (down into the
  condenser upper shell, solid) and the reheat steam (up to the MSR, dashed), then the turbine stop valve and
  the control valve. Header pressure shown over the tee.
- Turbine hall: HP turbine → crossover up into the moisture separator reheater (drains to the heaters) →
  LP turbine (double flow, exhausts down into the condenser) → generator on the same shaft.
- Condenser: shell, tube bundle with circulating water in/out, air removal to the steam jet air ejector
  (off-gas; its motive steam comes from the main steam header over the turbine hall) and the mechanical
  vacuum pump (startup), vacuum breaker on the exhaust hood, hotwell at the bottom (the model's hotwell).
  All modelled (solid).
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

Neutronics and void/pressure reactivity feedback, automatic scram (scram is an operator button), automatic
turbine trip or MSIV closure (only the `u.trip` hook), steam-line inventory and pressure waves (the header is
algebraic), separate main steam lines (one line stands for four), several control/bypass valves (one each),
turbine speed control and load reference (the load limit is the only turbine-side setting), recirculation pumps and jet pumps, downcomer
subcooling separate from the core, steam superheat, air in the dome (below 100 °C the pressure goes
sub-atmospheric), separate vessel and internals temperatures (one steel node), turbine (a stub: one expansion line), circulating water pump curve, river or cooling tower (an in/out pipe with a set inlet temperature), air
distribution in the condenser (one shell-average air fraction; no air-cooling section), SJAE stages and
inter/after-condensers, off-gas train, low-vacuum turbine trip, moisture separator reheaters, separate heater stages and drain coolers (one throttled extraction line stands for the train; real extraction lines have only isolation and non-return valves), pump trips (no
level-8 or low-suction trip; suction loss is a flow taper), pump minimum-flow recirculation, startup
low-flow valve, condensate and feed water inventory in the pipes, cleanup reject flow, containment pressure and drywell (the pool has no air space; its heat-up is modelled), RHR (pool cooling is a fixed placeholder cooler). Core void clearance uses one time constant at every power and pressure; bulk-flash voids a second
one (2 s), both illustrative.
