# BWR vessel model (bwr.html)

A separate page beside the PWR pressuriser. Same architecture: `bwr-sim.js` is the plant (pure physics),
`bwr-control.js` the automatic control, both ES modules importing `common.js` (steam tables, liquid
properties). Tests: `tests/bwr*.test.mjs` (shared helpers in `tests/bwr-helpers.mjs`).

## Code layout

`bwr-sim.js` composes per-system modules, each pure functions over its own part of the state, and
re-exports what the page and tests use (so they import `bwr-sim.js` only):

| Module | System | State it owns |
|---|---|---|
| `bwr-vessel.js` | Vessel geometry, water–steam solve with the downcomer region (`at`, `solve`, `derive`, `dcMass`, `vesselInit`) | `M`, `U`, `Mv`, `Mb`, `Mfl`, `udc`, `d` |
| `bwr-core.js` | Point kinetics, axial shape (`steady`, `localFb`, `peak`, `GZ`, `STGT`), fuel node, decay heat groups, control rod groups (`rodR`, `cov`, `seq`, `rodsFor`, `shapeAt`), CRD pump flow (`WCRD`), core voids (`voids`, `voidVol`), `coreInit`, `coreStep` (substeps) | `core {n, c[6], D[7], Tf, x, s[24], a[24], hs[24], fb, pk, zpk, ac, ae, xe, zb, W, dh, rho, per, q, pc}` |
| `bwr-recirc.js` | Recirculation pump and MG set (`mgStep`), jet pump (`jet`), natural circulation head, loop momentum (`resid`, `loopStep`), `recircInit` | `rc {on, ng, dm, Wd, Wc}` |
| `bwr-steam.js` | Rated conditions (`QR`, `PR`, `WR`), main steam line (`steamLine`, `cvPos`, `valveStep`: MSIVs, stop, control and bypass valves), SRVs (`srvStep`), nozzle factor | `ms[2]`, `sv`, `cv`, `bp`, `Pl`, `spr[]` |
| `bwr-turbine.js` | Turbine (step 15): expansion lines (`expansion`, `turbineLine`), Stodola stage pressures (`calib`, `stagePs`), turbine work (`work`); shaft and generator (`tgStep`), breaker (`breaker`, `brkTrips`), power–load unbalance and intercept valves (`pluStep`) | `tg {w, brk, lock, iv, PL, plu, trp}` |
| `bwr-condenser.js` | Condenser shell (`condense`, `condSteady`), circulating water (`cwTarget`, `cwStep`), condenser air (`airFlows`, `airStep`, `sjSteam`, `sjCap`, `vpCap`, `blanket`) | `cd {Tc, Ms, Ma}`, `wcw`, `vb` |
| `bwr-feed.js` | Hotwell (`hotwellMix`), condensate pump (`suction`, `cpFlow`), polisher, LP and HP heaters (`stage`, `heaterL`, `heater`, `train`), heater drains (`tubes`, `dumpX`, drain pump `dpFlow`), feed pump, min-flow line, feed valve and startup feed valve; the train solve (`network`) | `Mh`, `Thw`, `n`, `fv`, `sfv`, `xr`, `fpt`, `tlo`, `Tsu`, `Th`, shells `Mhs`, `Mls`, `psh`, `psl`, `Ths`, `Tls`, drain valves `xdh`, `xdl`, failures `fail` |
| `bwr-vessel.js` (end) | Vessel steel (`steelStep`), level instruments (`IZ`, `LV`, `toIn`, `fromIn`), bulk-flash void clearance `TAUF` | `Ts`, `Mbf` |
| `bwr-containment.js` | Suppression pool (`poolStep`, `poolLevel`, limits `PLIM`) | `pool {M, T}` |
| `bwr-rps.js` | Reactor protection: trip list `TRIPS` with setpoints `SP`, mode switch (`rpsMode`), `rpsCheck` (latch and log), `rpsReset`, bypass valve interlock `bpBlock` | `rps {mode, lat, log, off}` |

`step()` runs the systems in this order on the start-of-step state: generator breaker trips and the turbine's
power–load unbalance (bwr-turbine.js), reactor protection (bwr-rps.js), valves
and SRVs, steam line (header pressure, turbine and bypass flows), CST valves, pump switches and feed train
valve travel, heater drain flows, the condensate/feed train solve (`network`) and the feed pump's
low-suction trip, core and recirculation (substeps), downcomer mixing, circulating water and condenser air
flows, turbine stage pressures (with the last step's extraction flows) and heaters, turbine work and the
shaft (substeps), heater shell inventories, cleanup reject, condenser, hotwell and CST, steel
and suppression pool, then the vessel mass and energy and the pressure solve.

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `V`, `H`, `A` | 600 m³, 21 m, 28.6 m² | Free fluid volume, inside height, uniform free cross-section V/H |
| `Z` | BAF 5.6, TAF 9.3, SHR 11.5, FW 13.0, NWL 14.3, MSL 16.5 m | Elevations above the inside bottom: active fuel, top of shroud, feedwater sparger, normal water level, main steam nozzle |
| Core heat | from the kinetics (`u.heat` a number: that fixed heat, a test hook) | Heat to the coolant from the fuel node. Rated 3293 MW (`QC`) |
| Main steam line | Dome → line → inboard MSIV → outboard MSIV → header (throttle, pressure `Pl`). Inflow w = √((P² − Pl²)/(`RL` + `RM`/a1² + `RM`/a2²)), a = MSIV position (flow area ∝ position). `DPL` 0.3 MPa dome-to-throttle drop at rated flow (`PL0` 6.7 MPa), each MSIV 15 % of the R | Pipes hold no steam: Pl solved algebraically each step (bisection) so the inflow equals the outflow (a header node would have τ ≈ 0.2 s, too fast for dt 2 s). MSIVs shut and a valve beyond open: Pl = condenser pressure; everything beyond shut: Pl = P. Illustrative numbers (typical of the class), not from one plant |
| Turbine stop + control valves | w_t = `KT`·sv·fcv(cv)·Pl·ψ(Pc/Pl)/ψ*; fcv(x) = 1 − (1 − x)^1.5; `KT` so that valves wide open pass 105 % of rated (`VWO`) at 7.0 MPa dome | Choked ∝ Pl, subsonic factor near the condenser pressure (κ = 1.3). Control valve 86 % open at rated (`cvPos(w, P)` the lift for a flow). The characteristic is an illustrative shape, flattening near full open as a set of sequentially opened valves does. Stop valve: open/shut switch `u.sv`, trips shut in 0.2 s (`SVCLOSE`), opens in 5 s (`SVOPEN`). Control valve strokes in 5 s (`CVRATE`) |
| Bypass valve | w_b = `KB`·bp·Pl·ψ, `KB` = 0.25·WR/PL0 (`BPF` 25 % of rated at the rated throttle pressure, decided), linear; strokes in 0.5 s (`BPRATE`) | From the header straight to the condenser upper shell, throttled at constant enthalpy, so it arrives superheated (h of the dome steam); `condense()` takes any enthalpy. Stroke time illustrative (fast opening on a trip) |
| MSIVs | Two in series, open/shut switches `u.ms1`, `u.ms2`; 4 s stroke each way (`MSRATE`) | Closure 3–5 s is the usual specification. Undefined switch commands hold the valve. Protection hook for step 11: `u.trip = {turbine, msiv}` shuts the stop valve / both MSIVs whatever the switches |
| SRVs | `SRV` = 11 valves `{p, w}`: 4 × 1090 psig (7.617 MPa abs) rated 800,000 lb/h (100.8 kg/s), 4 × 1100 psig (7.686 MPa) 808,000 lb/h (101.8 kg/s), 3 × 1110 psig (7.755 MPa) 815,000 lb/h (102.7 kg/s); flow w·P/p × subsonic factor to the pool at 0.1 MPa | From a GE BWR/4 training lesson plan (NRC-hosted; group counts partly illegible in the OCR, read so that capacity ∝ set pressure, which it is for one valve model). Each valve: spring opens it fully above p, recloses below p − `BD` (0.25 MPa, illustrative; not in the source), latch `s.spr[i]`; or opened by hand, `u.srv[i]` true. Open = spring or hand. No automatic relief logic, low-low set or ADS |
| Turbine stages (step 15) | Throttle (header, Pl, h of the dome steam) → stop and control valves (isenthalpic) → first stage p1 → HP turbine, HP heater extraction at peH → HP exhaust pX → MSR and crossover (pressure drop `DMSR`·(w/w_R)², 0.055 MPa at rated; pass-through: no moisture separation or reheat) → LP inlet pLi → LP turbine, LP heater extraction at peL → condenser Pc. Rated (illustrative): p1 `P1R` 5.90 MPa (88 % of PL0), peH 2.40, pX `PXR` 1.38, pLi `PLIR` 1.325, peL 0.28 MPa | Stodola's cone law for each of the four stage groups, w = K·√((p_in² − p_out²)/(p_in·v_in)), with p·v at the inlet held at its rated value, so p_in = √(p_out² + (w/K)²) solved back from Pc (`stagePs(TK, wt, wsH, wsL, Pc, Pl)`; groups carry wt, wt − wsH, wt − wsH, wt − wsH − wsL; p1 ≤ Pl). K from the rated point (`calib`, `TK` in bwr-feed.js: the rated extraction flows come from the heater design). The step uses the last step's extraction flows. 75 % flow: p1 4.42, peH 1.79, peL 0.21 MPa; 25 %: 1.47, 0.59, 0.07 MPa; at no flow every stage is at Pc. The control valves stay choked ∝ Pl whatever p1 |
| Turbine expansion | `ETAH` 0.83 HP from (p1, h0), `ETAL` 0.87 LP from (pLi, hX) | `expansion(p0, h0, η)` → h(p) = h0 − η·(h0 − h_s(p)), h_s from the inlet entropy (`satS` in common.js). `turbineLine(ps, h0)` the whole line, carrying the stage pressures `.ps`; extraction leaves at h(pe), the rest exhausts wet at h(Pc) plus the windage heat. `work(wt, h0, line, wsH, wsL, hx)` → {hp, lp} W. Rated: hX 2553, exhaust 1941 kJ/kg; 1110 MWe gross (`PGR` 1113 MWe at the design condenser pressure). Efficiencies chosen for that output with the pass-through MSR |
| Shaft, generator, grid | One shaft at `NR` 1800 rpm; inertia constant `HIN` 5 s on PGR (E = HIN·PGR·ω²); generator `ETAG` 0.985; windage `PW0F`·PGR·ω³ (0.5 %); LP power lags its steam with `TRH` 5 s (MSR and crossover steam) behind the intercept valves iv | `tgStep(tg, hp, lp, PGR, dt)` in substeps of ≤ 0.05 s. Breaker closed: ω = 1 (infinite 60 Hz grid), Pe = ETAG·(hp + iv·PL − windage). Open: dE/dt = hp + iv·PL − windage. Intercept valves shut at once on a turbine trip (stop valve < 90 %) and on the fast closure; breaker open they are governed, open below 102 % and shut above 105 % speed; reopen at `IVRATE` 10 %/s. All illustrative |
| Breaker and turbine protection | Sync check `SYNC` 0.5 %; reverse power (shaft power below windage) for `TRP` 10 s trips it; a turbine trip (stop valve < 10 %) trips it; power–load unbalance `PLU` 40 % (pLi/PLIR − Pe/PGR) fast-closes the control valve (`trip.cvfast` in `valveStep`) and the intercept valves | `breaker(s, close)` the operator's switch (logged; returns '' or 'sync' / 'lock'). A trip locks the breaker open until the operator opens it. Overspeed 110 % and the control valve fast closure scram are in bwr-rps.js. Typical settings, not one plant's |
| Condenser | Shell `VC` 4000 m³ of steam and air at one temperature Tc: Pc = p_sat(Tc) + Ma·R·Tc/VC (Dalton, R = 287); design circulating water `WCW` 38,000 kg/s at `TCW` 20 °C; bundle `UA0` 2.70×10⁸ W/K clean, `UAD` (`UA`) 2.4×10⁸ W/K at the rated air fraction; blanketing UA = UA0/(1 + (ya/`YA`)²), ya = pa/Pc, YA = 0.1; `PCMAX` 0.135 MPa | Steam saturated at Tc. Inflows `[w, h]` split into vapour w·(h − h′)/h_fg at Tc and liquid. Bundle: Q = m·c_p·(Tc − T_in)·(1 − e^(−UA/(m c_p))), w_c = Q/h_fg. Implicit each step: bisection on Tc for ρ″(Tc)·VC = Ms + (vapour − w_c)·dt, the air mass (updated first) held fixed over the solve, so the solve stays one-dimensional; UA follows ya(Tc) inside it. Liquid to the hotwell = inflow − storage change, at h′(Tc). Rated (two heater stages, drains pumped forward): 6.78 kPa (6.52 kPa steam + 0.26 kPa air), 37.7 °C, 2172 MW, CW out 33.7 °C (one heater stage, before step 5: 7.32 kPa, 39.2 °C, 2374 MW). Illustrative numbers (38 m³/s and ~7 kPa are typical of an 1100 MWe unit; UA chosen to give them). `condSteady(ins, cw, pa)` the steady Tc with air partial pressure pa at the tubes (default `PA0`). Condensate leaves at Tc, i.e. subcooled against Pc when there is air. Fault `'condenser'` above PCMAX (LP exhaust rupture diaphragms) |
| Circulating water | One pump (`u.cwp` on/off) against a fixed system: w = √(`HCW`/(`KSYS` + `KCV`/x²)), HCW 0.15 MPa flat (no pump curve), 10 % of it across the open valve, so x = 1 passes WCW; x = 0.5 passes 33.3 t/s, 0.3 passes 26.8 t/s. First-order lag `TAUCW` 12 s (run-up and coast-down). Inlet temperature `u.tcw`, an operator input | In/out pipe only (decided): no river, cooling tower or pump curve. Butterfly valve `u.cwv` 0–1, no stroke limit (the flow lag covers it). Illustrative numbers. State `s.wcw` |
| Air in-leakage | `WLK` 0.009 kg/s (≈ 15 scfm) × nozzle factor from `PATM` 0.101325 MPa (constant at vacuum, zero at atmospheric); radiolytic gas `WRAD` 0.02 kg/s × Q/QR × main steam share of the steam leaving the vessel | Illustrative. The radiolytic H₂/O₂ is counted as air (R = 287) and is not taken out of the vessel inventory (10⁻⁵ of the steam flow). State `s.cd.Ma` |
| Steam jet air ejector | Removes `VSJ` 10 m³/s of shell-average gas (VSJ·Ma/VC kg/s of air) × clamp((Pm − 0.7 MPa)/0.5 MPa) × clamp((35 kPa − Pc)/10 kPa); motive steam `WSJ` 2.5 kg/s × clamp(Pm/1.2 MPa, 0, 1) | `u.sj` motive steam valve. Pm = `motiveP(s)`, the header pressure Pl (downstream of the MSIVs). The motive steam is taken off the main steam line flow: from the turbine flow first, then the bypass flow, at most wst; with the MSIVs shut the SJAE has no steam. It goes to the condenser at h″. The extraction pressures follow the turbine flow wt; the heaters are designed on the rated turbine flow `WTR` = WR − WSJ. VSJ is an effective volume: a real ejector draws air-rich gas from the air-cooling section through a much smaller suction. Rated air partial pressure `PA0` = (WLK + WRAD)·R·T/VSJ = 260 Pa |
| Mechanical vacuum pump | `VVP` 2.2 m³/s × clamp((Pc − 7 kPa)/23 kPa, 0, 1) | `u.vp` on/off. From atmosphere (empty condenser, 20 °C water) to 39 kPa in 30 min, 19 kPa in 1 h, 12 kPa in 2 h, ~9 kPa in 5 h |
| Vacuum breaker | `WVB` 20 kg/s × opening × nozzle factor from atmosphere, never filling past atmospheric within a step; strokes in 20 s (`VBRATE`) | `u.vb` open/shut, manual. From rated vacuum to atmospheric in about 5 min. No reverse flow when the shell is above atmospheric |
| Hotwell | `HWM` 600 t capacity, normal half full, `HWD` 4 m deep at full (illustrative); its own mass `Mh` and temperature `Thw` | Mixes the condensate (at Tc), makeup at `TMK` 30 °C, and the condensate pump's draw. Design temperature `THW` 37.7 °C (the condenser at rated; the extraction constants are computed with it). Makeup from the condensate storage tank through the makeup valve, reject to it through the reject valve (step 7; the ±100 kg/s `u.mk` input is gone, `MKMAX` is kept as the controller's range) |
| Condensate storage tank (step 7) | `CSTM` 1900 t (about 500,000 US gal), `HCST` 12 m tall, level ∝ mass, open to the air (`PATM`); starts 70 % full (`init({cst})`) at `TMK` 30 °C; its own mass `Mcst` and temperature `Tcst` | Makeup valve (CST → condenser): `KMU`·x·√(PATM + ρg·(h − `ZMU`) − Pc), ZMU 3 m, about 100 kg/s fully open from a 70 % full tank under vacuum (`muFlow`), falling to 0 as the tank goes from 2 % to empty (outlet uncovering); enters the hotwell at h′(Tcst). Reject valve (condensate pump discharge → CST): `KRJ`·x·√(Pcd − PATM − ρgh), about 100 kg/s from 2.4 MPa (`rjBack` the back pressure); a branch of `network` at the condensate pump discharge, so the condensate pump carries it (`wcp` = LP line + reject). The CST mixes the reject at h′(Thw); no heat loss to the air. Both valves stroke ≤ 10 %/s (`CRATE`); `u.mu`, `u.rj` demands, `s.xmu`, `s.xrj` positions. Fault `'cst'` when it overfills. Sizes and elevations illustrative; no fill from demineralised water, no other users (RCIC, HPCI, CRD) yet |
| Train | Hotwell → condensate pump → polisher → LP heater → feed pump → HP heater → feed valve → check valve; min-flow line from the feed pump discharge to the condenser | Pipes hold no water. `network(o)` solves each step for the condensate pump discharge Pcd, feed pump suction Ps and discharge Pd: bisection on Ps outside (supply from the condensate side falls, the feed pump's draw rises with Ps), on Pd and Pcd inside; each branch flow is monotonic in its node pressures. Every pump passes exactly what the lines downstream take (mass). Overall numbers kept from the lumped pump of step 1: total shutoff `H0` 10 MPa over the hotwell, rated discharge PCD + H0 − `KPU`·W² (KPU 0.24), so the feed valve is still 74.5 % open at rated |
| Condensate pump | ΔP = φ·`HC0` − `KCP`·w², HC0 2.6 MPa, KCP 0.08 Pa/(kg/s)² (0.26 MPa at rated); constant speed, run switch `u.cp` | Suction at the impeller Pin = Pc + ρg·(`ZS` + `HWD`·fraction full), ZS 1 m pit. NPSHa = (Pin − p_sat(Thw))/ρg (3 m at half full, saturated); NPSHr = `NPR`·(w/WR)², NPR 2 m. φ = 1 while NPSHa ≥ NPSHr, falling linearly to 0 at NPSHa = NPSHr/2 (closed form: a quadratic in w², `cpFlow`); times hw/0.02 below 2 % (suction uncovering, vortexing). Stopped: a resistance KCP·w² (check valve: no reverse flow). All illustrative |
| Heater drains (step 6) | HP shell `VHS` 30 m³, LP shell with drain tank `VLS` 60 m³ (fraction full `Mhs`/`MHS`, `Mls`/`MLS`), saturated at the shell pressure; HP drain valve `KDVH`·x·√(psH − psL) (linear); drain pump f·`HDP0` − `KDPP`·w², HDP0 3.5 MPa, 0.5 MPa lost at rated drains, suction psL + ρg·`ZDT` (5 m), f = level/0.1 below 10 %, discharge valve conductance `KDVL`·x, check valve; both valves half open (`XD0`) at rated; travel 10 %/s (`DRATE`) | HP drains cascade to the LP shell (they flash there: `stage(…, ins)` takes them as heat w·(h − h′(ps))); LP + cascaded drains are pumped into the condensate line after the LP heater (a branch into the feed pump suction node of `network`). Rated: drains 579 kg/s (`WDD`), LP extraction only 163 kg/s, HP heater inlet `THPI` 127.4 °C (LP outlet 126.5 °C mixed with the drains), condensate pump 1209 kg/s; `KFP` re-chosen (0.181) so the feed valve stays 74.5 % open. Tube flooding: above `LFL` 60 % the heated fraction a = (1 − level)/0.4 and the outlet is Tin + a·(T_sat(ps) − TTD − Tin). High-level dump (plant overflow): above `LDU` 80 % a valve opens (fully at 90 %) to the condenser, `KDDH`/`KDDL`·x·√(ps − Pc), sized for 1.4 × the shell's rated drains. With extraction steam the shell is at most at the extraction pressure (non-return valve); flash steam the tubes can't condense vents to the condenser (`f.wv`). `fail(s, 'hdv', on)`: the HP drain valve fails shut (event). All volumes, levels and pump numbers illustrative |
| Polisher, LP heater drop | `KPOL`·w² 0.13 MPa, `KLP`·w² 0.10 MPa at rated | Polisher a pressure drop only (no chemistry) |
| Feed pump | ΔP = n²·`HF0` − `KFP`·w², HF0 = H0 − HC0 = 7.4 MPa, KFP 0.181 (chosen so the rated discharge is unchanged; 0.099 before the drains were pumped forward); turbine-driven, run switch `u.fp`, speed `u.n` | Speed state `s.n` changes at most 10 %/s (`NRATE`); stopped, tripped or at zero speed the pump is the resistance KFP·w², so below ~2.5 MPa the condensate pump feeds the vessel through it. Shutoff of the two pumps at speed n: `shutoff(n, pc, Mh, dp)` = max(Pin + HC0, drain pump suction + HDP0) + n²·HF0 (10.04 MPa full speed, 7.37 MPa at 80 % on the condensate pump; about 11.2 MPa while the drain pump runs: it injects at the feed pump suction with more head). Low suction pressure trip: Ps < `PLO` 0.7 MPa for `TLO` 5 s sets the latch `s.fpt` (illustrative setting), cleared only by `u.fp` false; while tripped the speed runs down to 0. If Ps would fall below the vapour pressure at the LP heater outlet, the pump cavitates and delivers only what reaches its suction (`f.cav`). Drive steam not modelled |
| Min-flow valve | flow `KR`·xr·√(Pd − Pc), linear; KR sized for `WMIN` = 25 % of rated (447 kg/s) open at 9.9 MPa; stroke ≤ 20 %/s (`RRATE`); `u.rv` demand, `s.xr` position | Returns feed pump discharge water to the condenser at the LP heater outlet enthalpy (it flashes there) |
| HP heater + piping drop | `KLN`·w², KLN = 0.094 Pa/(kg/s)² | 0.3 MPa at rated flow |
| Feed valve | drop w²/(`KV`·x²)², KV = 2.31 (kg/s)/√Pa, quadratic characteristic; stroke ≤ 10 %/s (`FRATE`) | Zero flow if the drop would be negative (check valve). `fwFlow(n, x, P, Mh)` the steady train (condensate pump on, min-flow shut, drain pump at its design opening `DPD`), inverse `fwValve(w, n, P)` (rated drains `WDD` forward). Rated 1788 kg/s at 74.5 % open; 2364 kg/s wide open at 7 MPa (2261 at step 5, the condensate pump short of NPSH; 2400 kg/s before step 5) |
| Startup feed valve | In parallel with the feed valve, same quadratic characteristic, conductance `RSU` = 0.085 of it: the pair pass w = KV·(x² + RSU·xs²)·√ΔP; stroke ≤ 10 %/s | Wide open about 290 kg/s (16 % of rated) with the feed pump holding 2.2 MPa over the dome; `f.wsu` the flow through it (its share of the conductance). Size illustrative (startup and low-power valves are typically 10 to 20 % of rated flow) |
| Feedwater heaters | LP: extraction pressure pe = the LP turbine's stage pressure peL, rated `PEL` 0.28 MPa; HP: the HP turbine's peH, rated `PE` 2.40 MPa (Stodola, see Turbine stages); valve flow `KX`·e·√(pe² − ps²) (`KXL`, `KXT`); shell condenses ws = fw·(h(T_out) − h(T_in))/(h(pe) − h′(ps)); T_out = T_sat(ps) − `TTD` (3 K) | Two stages of the same model (`stage(H, …)`): LP before the feed pump (condensate flow), HP after it (feed flow to the vessel). Extraction pressures are the turbine's stage pressures: they fall with the turbine flow, and a stage that takes more steam draws its own pressure down (less flow through the groups below it). The shell pressure ps is where the valve passes what the shell condenses (bisection on the shell saturation temperature). Design point: rated flows, 5 % drop across each open valve; LP 126.5 °C out (`TLPD`), ws ≈ 163 kg/s with the HP drains cascaded in (375 kg/s without); HP 216 °C out from `THPI`, ws ≈ 417 kg/s (wet steam on the expansion line). `u.exl` and `u.ext` (e) 0–1 valve openings (`exl` defaults to `ext` if absent). HP outlet follows T_out with `TAU.h`. `heaterL`, `heater(wst, e, fw, Tin, line)` → {T, pe, ps, ws, hx, hd}; `train(wst, eL, eH, fw, Tin, line)` the steady train with the drains cascaded and pumped forward; `heaterT(wst, e, fw, Tin)` both valves at e; `heatBalance(Q, P, e)` the steady {W, Tfw}. Drains: see the row above |
| `TAU.c` | 10 s | Subcooled water condenses steam towards saturation |
| `TAU.b` | 0.7 s | Flash voids `Mfl` (steam made beyond what the core boils, as in a depressurisation) leave the mixture |
| `VDC` | 100 m³ | Downcomer + lower plenum liquid (illustrative); mass `dcMass` = VDC·ρ, at most 60 % of the water |
| `TAU.h` | 60 s | Feedwater heater outlet temperature (heater and piping holdup) |
| `TAUF` | 2 s | Voids from bulk flashing (generation beyond what the heat makes: gen − (Q + Q_steel)/h_fg) rise out more slowly than core boiling voids; state `Mbf`, part of the level. Illustrative |
| Vessel steel | `CST` 6×10⁸ J/K (≈1200 t at 500 J/kg/K), `UAS` 1 MW/K to the water, `UAL` 4200 W/K to the drywell at `TDW` 50 °C | One node, implicit exponential (`steelStep`), exact energy split: heat to the water Qs and loss Ql are the step means. About 1 MW lost at rated; lags the water ~10 K at 55 °C/h. Illustrative. Init: steady between water and drywell |
| Level instruments | Instrument zero `IZ` 528 in (13.41 m) above vessel zero; `LV` L8 +54, L7 +39, L4 +27, L3 +12.5, L2 −38, L1 −129 in | Typical BWR/4 (TAF = −162 in, normal ≈ +35 in). `toIn(z)`, `fromIn(in)`. `s.f.lin` the level in inches. No reference-leg or density errors: the instrument reads the model level |
| Suppression pool | `MPOOL` 3.8×10⁶ kg at `TP0` 32 °C, surface `APOOL` 1100 m²; cooler `UAP` 1.5 MW/K against `TSW` 30 °C when `u.pc`; stop at `TPMAX` 100 °C | Mark I-type, generic. SRV steam condenses completely (mass and h″ into the pool). Limits `PLIM`: 35 °C normal, 43.3 °C (110 °F) scram, 48.9 °C (120 °F) depressurise (typical BWR/4 technical specifications); shown on the page, no automatic action. Cooler is a placeholder for RHR pool cooling (deferred). Fault `'pool'` |

## Core (bwr-core.js)

All numbers generic BWR/4 class; illustrative unless stated.

| Symbol | Value | Meaning |
|---|---|---|
| `QC`, `PRC`, `WCR` | 3293 MW, 7.0 MPa, 12,900 kg/s | Rated power, dome pressure, core flow (BWR/4 ~102 Mlb/h) |
| `LAM`, `BF`, `BETA` | λ 0.0124 … 3.01 /s, U-235 group fractions (Keepin), β = 0.0056 | Six delayed groups; β lowered from 0.0065 for a mid-cycle core with Pu. `LAMBDA` 4×10⁻⁵ s is not needed by the prompt-jump form (kept for reference) |
| `SRC` | 1 W | Source in power units: subcritical power SRC/(β − ρ), ~8 W with the rods all in |
| `AV`, `AD`, `AM` | −1×10⁻³ /% void, −2×10⁻⁵ /K, −2×10⁻⁵ /K | Local void, Doppler (fuel temperature along the shape), moderator (subcooled water). Reference hot zero power, `TREF` = T_sat(7 MPa). GE BWR Technology Manual ch. 1.7 (NRC ML11258A296) gives −1×10⁻³ per % void and −1×10⁻⁵ per °F fuel as typical |
| `NZ`, `M2`, `DEXT` | 24, 65 cm², 0.1 m | Axial nodes, migration area (one group), extrapolation distance beyond the fuel ends |
| `BSK`, `STGT`, `GZ` | 0.35, peaking 1.53 at 44 % height | Rated shape: chopped sine × (1 + BSK·(1 − 2z)); G(z) the fixed k∞ profile (fuel zoning) that gives it at rated with all rods out, k = 1 there |
| `MCF`, `RF` | 5×10⁷ J/K, 1×10⁻⁷ K/W | Fuel node: τ = 5 s, 615 °C average at rated. All heat is deposited in the fuel |
| `DL`, `DF` | λ 1/1.5 … 1/1.5×10⁶ s; Σf = 0.0693 | Seven decay-heat groups driven by fission power; least-squares fit to 0.065·(1 + t)^−0.2 of the total (within 3 % from 1 s to 3 days) |
| `AF`, `LC`, `VCH` | 7.5 m², 3.7 m, 30 m³ | Core flow area (764 bundles × ~0.0098 m²), heated length, chimney + standpipes + separators volume at the exit void |
| `C0`, `VGJ`, `TAUV` | 1.1, 0.25 m/s, 0.7 s | Zuber–Findlay drift flux; voids relax to their targets with TAUV |
| `NG`, `NPOS`, `NOTCH` | 4, 48, 2 | Rod groups; positions 00 (in) … 48 (out) span the active fuel; a notch is 2 positions (6 in) |
| `GW` | 0.045, 0.045, 0.016, 0.058 | k∞ lost where group g covers a node (its share of the rods × a rodded lattice's loss). Groups in withdrawal order: 1, 2 deep (shutdown margin), 3 startup (critical on it), 4 power. Illustrative |
| `XRATED` | all out | Rated rod pattern (the calibration point of GZ) |
| `RODRATE` | a notch per 4 s | Drive speed (x per second: 1/96) |
| `TSD`, `VSC`, `TSCRAM` | 0.2 s, 1/2.7 per s, 2.9 s | Scram: pilot valve delay, then the accumulators drive every rod in at VSC; all in from fully out by TSCRAM (BWR/4 technical specifications: 90 % in within about 3 s) |
| `WCRD` | 3.8 kg/s | CRD pump flow (60 gpm) from the storage tank into the vessel while the pump runs (`u.crd` true) |

Kinetics (prompt-jump, precursors in power units c_i = Λλ_iC_i): n = (Σc_i + SRC)/(β − ρ),
dc_i/dt = λ_i(β_i n − c_i), implicit Euler for c_i solved together with n; ρ ≥ β (or the implicit
denominator below 1 % of β) latches `core.pc` → fault `'prompt'`. Decay groups dD_j/dt = λ_j(f_j n − D_j).
Fuel: MCF·dTf/dt = n + ΣD − (Tf − T_sat)/RF, exact per substep; the heat to the coolant is the fuel
energy balance over the substep. Reactivity ρ = 1 − 1/k from the axial shape with the feedback and the rods.

Axial shape (quasi-static; the fix for the cold-feed response): one-group diffusion over NZ nodes,
−M²φ″ + φ = (k∞(z)/k)φ, φ = 0 at DEXT beyond the fuel ends, finite differences. k∞(z) = G(z)·(1 + f(z))·(1 − R(z)),
f = AV·100·α(z) + AD·(Tc + (Tf − Tc)·s(z) − TREF) + AM·(Tc − h_sub(z)/c_p − TREF), with the node voids α,
the node subcooling h_sub and the lumped fuel temperature Tf spread along the shape s. G(z) is computed once:
G = (A·s*)/(s*·(1 + f_rated)) for the rated target shape s* (STGT) with all rods out (so k = 1 there).
R(z) = Σ_g GW_g·c_g(z), c_g the fraction of node z the group covers: the rods enter from the bottom, so a
group at withdrawn fraction x covers the fuel up to (1 − x)·LC. ρ = 1 − 1/k, k from the Rayleigh quotient of
the current shape with the rods where they are now; each substep two shifted inverse (Wielandt, shift
0.003 below 1/k) iterations then move the shape towards the new feedback and rods. The fission power
is n·s(z) and goes into the coolant along s. `steady(Q, W, Δh, P, x)` solves shape, voids and (without x)
the critical rods along the withdrawal sequence together (init); `seq(t)` the sequence (groups 1 to 4 one
after the other), `rodsFor(f, s, ρ)` the sequence point for reactivity ρ with the shape held, `shapeAt(f, x,
s)` the converged shape and its ρ.

Control rods: four groups that move as gangs (a BWR/4 pulls one rod at a time through a sequence of many
groups; a group stands for a step of it), bottom entry. Hot zero power: critical with groups 1 and 2 out,
group 3 near 19, group 4 in; a notch of group 3 there is about 0.2 $ (a period near 30 s). At 100 °C critical
near 11 (about 0.35 $ a notch). All in −0.12 hot (k ≈ 0.89), all out +0.062 (the rated feedback). Rated:
all out. With rods in from below and no voids the shape is top-peaked (about 2.4 at 73 % height at hot zero
power; the old uniform lever gave 2.1 at 65 %, from the fuel zoning alone): bottom-entry rods do not cure
that, they add to it, as in a real BWR at startup; the voids at power bring it down. Group 4 half in at rated:
−6 % power, peak moves up. A few notches of group 4 at rated (shallow rods) move the power up, the
boiling boundary rises and the core-average void falls (0.395 → 0.38 at position 40) enough to cancel their
worth: +1.6 % power at position 40, +3 % at 32. A real core loses a little power to shallow rods. The same thing makes the scram worth build up slowly at
first: from rated, −0.13 $ at 0.4 s, −0.8 $ at 1 s, −2 $ at 1.5 s, −35 $ all in (a real BWR/4 scram passes −1 $ in
well under a second), so pressure transients that scram peak higher than with the old lever. And the
one-group axial model (M² 65 cm²) is too loosely coupled, so the shape moves too far for a small change in
k∞. A larger M² (100 cm², closer to a voided lattice) only trims that (+1.2 % at 40), and 150 cm² breaks the
cold-feed calibration, so M² stays.

CRD hydraulics (minimal): the CRD pump (`u.crd`, absent: stopped, so the other systems' tests run without
it; the page starts it running) takes `WCRD` from the storage tank into the downcomer at the tank's
temperature, mixing with the feed. Rods move only while it runs; the scram uses the accumulators (always
charged) and works either way. The scram moves every rod in after `TSD` at `VSC`, from wherever it is.
Why: in a point model a colder feed removed voids at the bottom of the boiling length, where α is steepest
in x, while the extra power went in along the whole height, mostly into the high-quality upper core, so it
restored few voids: 54 K colder feed gave +33 % (a real BWR/4: some 15–20 %, no scram). A cosine or
bottom-peaked fixed shape (+25 to +38 %) or subcooled boiling (+29 to +32 %) did not change that; letting the
shape move down to where the voids were lost does (+17 %).

Core voids (`voids(Q, W, Δh_in, P, s)`): the heat goes in along the shape s; equilibrium quality from the
heat so far, x = (Q·∫s − W·Δh_in)/(W·h_fg) (W at least 100 kg/s); boiling boundary zb where it reaches 0;
α(x) = x/(C0(x + (1 − x)ρ″/ρ′) + ρ″·Vgj/G), G = W/AF; each node's void the mean over four points in it,
the core average the mean of the nodes (each relaxes with TAUV). Without s, the steady shape (`steady`).
Rated: zb 0.239, x_e 0.138, average 39.5 %, exit 67.7 %.

`coreStep(k, inp, dt, flow)` runs ceil(dt/0.05) substeps with P, T_sat, Δh_in and T_in held: rods (to
`inp.rods` at the drive speed while `inp.crd`, or the scram),
kinetics, decay, fuel, then `flow(h, k)` (the recirculation substep) and the void relaxation. Returns the
mean heat to the coolant. `coreInit(q, Tc, Tin, v, x)`: steady at heat q (decay groups at equilibrium,
long operation), rods along the sequence where it is critical unless x (array of 4) is given; q ≤ 0 gives
a shut-down core (rods x, default all in) at the source level.

## Recirculation (bwr-recirc.js)

One loop standing for two loops and 20 jet pumps. Pump on an MG set: speed follows the demand `u.mg`
through the speed limiter (2 %/s, ≥ 20 % while the breaker `u.rp` is closed); breaker open: coastdown
e^(−t/5 s). Two momentum equations, implicit (linearised, numerical Jacobian) per substep:

- drive: `ID`·dWd/dt = n²·HS − KPP·Wd|Wd| − KLP·Wd|Wd| − (1 + KN)·Wd|Wd|/(2ρAN²) + ΔP_suction
- core: `IC`·dWc/dt = ΔP_jet + ΔP_nat − (KC1 + KC2·φ²)·Wc|Wc|/ρ, φ² = 1 + x_e(ρ′/ρ″ − 1)

Jet pump (mixing-section momentum over the throat AT): ΔP_jet = [Wd²/(ρAN) + Ws²/(ρ(AT − AN)) − Wc²/(ρAT)]/AT
− (1 + KS)Ws²/(2ρ(AT − AN)²) + (KDF − KFT)Wc²/(2ρAT²), Ws = Wc − Wd (signed squares). Natural head
g[(LC + HCH)ρ_dc − LC·ρ̄_core − HCH·ρ̄_chimney]. Losses are linear below 2 % of rated drive flow so a
stopped loop settles.

| Symbol | Value | Meaning |
|---|---|---|
| `WDR`, `HPR` | 4300 kg/s, 1.6 MPa | Rated drive flow and pump head (M = 2) |
| `AN`, `AT` | 0.09 m², 0.566 m² | Nozzle and throat areas, 20 jet pumps together |
| `KN`, `KS`, `KDF`, `KFT` | 0.05, 0.1, 0.73, 0.1 | Nozzle, suction, diffuser recovery, throat friction |
| `HS`, `KPP` | 2.0 MPa shutoff, (HS − HPR)/WDR² | Pump curve n²·HS − KPP·W² |
| `KLP`, `KC1`, `KC2` | from the rated point | Drive loop loss; core single-phase (40 %) and two-phase (60 %) losses; rated core Δp 0.171 MPa = jet 0.135 + natural head 0.036 |
| `HCH`, `ID`, `IC` | 5.3 m, 150 /m, 15 /m | Chimney height (TAF to separators), loop inertias L/A |
| `MGRATE`, `NMIN`, `TCD` | 2 %/s, 20 %, 5 s | Speed limiter, minimum speed, coastdown time constant |

## Scram

`scram(s)` latches `s.scram = {t, P0}`: the rods drive fully in (`TSD`, `VSC`) whatever `u.rods` says, and the
kinetics and decay groups give the power (about 5 % at 10 s, 3.3 % at 1 min, within 15 % of `decayFrac`
from 1 min on). `reset(s)` hands the rods back to `u.rods` (the page puts the group sliders at 00 on scram). With the
fixed-heat test hook, P0 = q (default `s.f.Q`) and the heat is P0·`decayFrac`(t − t_scram) as before. `decayFrac(t)` = 0.935·(0.93·e^(−t/2) +
0.06·e^(−t/10) + 0.01·e^(−t/80)) + 0.065·(1 + t)^(−0.2): fission power with the rods in (prompt drop,
then two delayed-neutron tails) plus decay heat in Way–Wigner form for long operation at P0. Gives
100 % at 0, ~7.5 % at 10 s, 3.3 % at 1 min, 1.3 % at 1 h, 0.7 % at a day — rough, not a fitted curve.
`decayFrac` remains for the fixed-heat hook and as the reference curve for the decay groups.

## Reactor protection (bwr-rps.js)

Protection is part of the plant, not a controller: `step()` runs it first, on the start-of-step signals, and it
acts whatever the operator's switches and M/A settings say. Each trip compares one signal with its setpoint;
a trip that fires latches (seals in) and is logged with the sim time. Trips are grouped: any scram trip scrams
(`scram(s)`), any isolation trip shuts both MSIVs (`u.trip.msiv` inside the plant), a turbine trip shuts the
stop valve, a feed pump trip stops the feed pump (its speed runs down at the normal rate). The MSIV and stop
valve position switches are read again after the valves have moved in the step, so a closure scrams in the
same step. `reset(s)` clears every latched trip whose signal has cleared (or that the mode switch now
bypasses) and logs what stays in; the scram clears once no scram trip is left. The page's Scram button is
`manualScram(s)` (logged); the mode switch is `setMode(s, m)` (Shutdown scrams at once). `init({rps: false})`
runs the bare physics (the physics tests do).

Setpoints are typical BWR/4 values from technical specifications and training material, not those of one
plant. Level setpoints are inches above instrument zero.

| Group | Trip | Setpoint | Modes |
|---|---|---|---|
| Scram | Reactor high pressure | 1045 psig (7.31 MPa abs) | all |
| Scram | Reactor low level | L3, +12.5 in | all |
| Scram | APRM high flux | 120 % (the flow-biased trip's clamp) in Run, 15 % in Startup and Shutdown | all |
| Scram | MSIV closure | either MSIV below 90 % open | Run |
| Scram | Turbine stop valve closure | below 90 % open with first-stage pressure above 30 % of rated | all |
| Scram | Turbine control valve fast closure | the power–load unbalance (load rejection) with first-stage pressure above 30 % | all |
| Scram | Condenser low vacuum | 23 inHg vacuum (23.4 kPa abs) | Run |
| Scram | Mode switch in Shutdown | | Shutdown |
| MSIV isolation | Reactor low-low level | L2, −38 in (some BWR/4s use L1) | all |
| MSIV isolation | Main steam line low pressure | 850 psig at the header | Run |
| MSIV isolation | Main steam line high flow | 140 % of rated | all |
| MSIV isolation | Condenser low vacuum | 7 inHg vacuum (77.6 kPa abs) | Run (as with the startup bypass switch) |
| Turbine trip | Reactor high level | L8, +54 in | all |
| Turbine trip | Condenser low vacuum | 22.5 inHg vacuum (25.1 kPa abs) | all |
| Turbine trip | Turbine overspeed | 110 % of 1800 rpm | all |
| Feed pump trip | Reactor high level | L8, +54 in | all |
| Bypass valves | Condenser low vacuum interlock | shut while below 7 inHg vacuum (not latched) | all |

APRM reads the neutron (fission) power scaled to rated thermal power, not the heat through the fuel: a
power spike trips it before the fuel heats up. One logic channel: no one-out-of-two-twice voting, no channel
tests, no instrument errors or delays (the signals are the model's own values). After a scram the feedwater
controller sets its level setpoint down by `SDL` 0.45 m for `SDT` 600 s and ramps it back over `SDR` 1200 s
(see Control), so the shrink and refill do not overshoot to L8.

## State and step

State: total mass `M`, total internal energy `U`, steam mass `Mv` (dome + voids, always saturated), void
steam `Mb` (part of `Mv`), flash voids `Mfl`, downcomer specific internal energy `udc`, the core `core` and
recirculation `rc` states, main steam valve positions `ms[0]`, `ms[1]` (MSIVs), `sv`, `cv`, `bp` and the header
pressure `Pl`, SRV spring latches `spr[]`, protection `rps {mode, lat, log}`; feedwater train: feed valve
position `fv`, heater outlet temperature `Th`, hotwell mass `Mh`.

Each step (`step(s, u, dt)`):
1. Each main steam valve moves toward its demand at its stroke rate (`valveStep`); `steamLine` gives the header pressure and the turbine and bypass flows; each SRV spring latch from P; an SRV is open if its spring or `u.srv[i]` holds it.
2. Flows at the start-of-step pressure: steam out (main steam + SRVs) at h″, feedwater in at h(T_fw, P)
   with T_fw the heater outlet `Th` and the flow from `fwFlow` (pump speed, feed valve position after its
   travel, hotwell), heat Q. `M` and `U` updated; `Mv` loses the steam that left. Hotwell: Mh += (return
   + makeup − feed)·dt; heater outlet relaxes towards `heaterT(wst, e)`.
3. Core and recirculation substeps (above) with Δh_in = u′ − u_dc and T_in the downcomer temperature;
   then the downcomer mixes: Mdc·du_dc/dt = fw·(u_fw − u_dc) + (W − fw)·(u_core − u_dc), exact per step
   (W < fw: the surplus feed spills into the core region).
4. If the core-region water (U − Mv·u″ − Mdc·u_dc)/(M − Mv − Mdc) is below u′, it condenses steam: deficit
   D/(u″ − u′)·(1 − e^(−dt/τc)).
5. Pressure solve on the saturation temperature T (Illinois regula falsi): at trial T the steam has
   u″(T); if the core-region water would be above u′(T) it flashes to equilibrium,
   Mv = (U − Mdc·u_e − (M − Mdc)·u′)/(u″ − u′) with u_e = min(u_dc, u′(T)) (a downcomer above saturation
   flashes too); the residual V − V_water − Mv/ρ″ rises with T. Water volume from the compressed-liquid
   density at the mean water temperature (`liqRho`, `Tuf` from common.js). Because u_dc is held while T
   changes, a pressure rise leaves the downcomer further below saturation.
6. Voids: Mb = ρ″·(AF·LC·ac + VCH·ae) + Mfl, Mfl relaxing to max(0, gen − W·x_e)·τb.

Level = (V_water + Mb/ρ″)/A, the downcomer level (the voids inside the shroud push their volume into the
downcomer); collapsed level = V_water/A. `d.Tl`/`d.sub` are the core region, `d.Tdc`/`d.subdc` the
downcomer, `d.Tm` the mean, `d.E` the downcomer's deficit below saturation. Mass and energy are conserved exactly
(the vessel is rigid, so there is no external work; the tests check both).

Air balance each step (`airStep`): dMa/dt = w_in − V_r·Ma/VC with w_in (leakage, radiolytic, breaker) and V_r (SJAE + vacuum pump, their capacity factors at the start-of-step Pc) held over the step, integrated exactly: Ma' = Me + (Ma − Me)·e^(−V_r·dt/VC), Me = w_in·VC/V_r. `s.f.wain` and `s.f.waout` are the air flows in and out; their integral closes the air mass exactly.

`fault(s)`: `'uncovered'` level below TAF, `'flooded'` level at the steam nozzle, `'critical'` above
21.5 MPa, `'hotwell'` hotwell over capacity (condenser flooding), `'range'` if the solve fails.

Command `u` = {rods array of 4 group positions 0–1, withdrawn fraction (undefined, or an undefined entry:
the group stays), crd CRD pump running (absent: stopped; the rods move only with it), heat W (a number: fixed heat, test hook), rp
recirculation breaker (undefined: keep), mg 0–1 MG set speed demand (undefined: keep), n 0–1 pump speed, fv 0–1
feed valve demand, ext 0–1 HP heater extraction, exl 0–1 LP heater extraction (absent: = ext), cp, fp pump run
switches (absent: running), rv 0–1 min-flow valve demand, hdv, ldv 0–1 HP drain valve and drain pump valve
demands (absent: 0.5), hdp drain pump run switch (absent: running), mu, rj 0–1 CST makeup and reject valve
demands (they replace the old mk flow), ms1, ms2, sv booleans (MSIV and stop valve switches, true open;
undefined holds), cv 0–1 control valve and bp 0–1 bypass demands, rw 0–1 cleanup reject, pc pool cooling,
trip {turbine, msiv, bypass} (extra protection hook; the protection's own trips act whatever it says), srv[],
cwp CW pump on, cwv 0–1 CW valve, tcw K CW inlet, sj SJAE motive steam open, vp vacuum pump on, vb vacuum
breaker open}. Absent cwp, cwv, tcw, sj mean the normal running line-up (on, open, 20 °C, in service); absent
vp, vb mean off and shut. The protection's L8 feed pump trip stops the feed pump while it is latched; an
outside trip can act through the same fields (cwp false is a CW pump trip, fp false a feed pump trip). `s.f`
adds `Pl` (header pressure), `wt` (turbine flow), `wb` (bypass flow), and the valve positions; `wst` = wt + wb
is the main steam line flow (what the feedwater controller's steam-flow signal measures). `s.f` records what
was applied, with the feed train's fw, Tfw, Pd (feed pump discharge), Ps (its suction), Pcd (condensate pump
discharge), wp, wcp, wr (feed pump, condensate pump, min-flow), npsh, phi, cav, fpt, pe/ps/ws (HP extraction
pressure, shell, steam), pel/psl/wsl and Tlp (LP), Tt (HP target), ret (condensate return), mk (net makeup
wmu − wrj), Mh, hw (hotwell fraction); drains: LH, LL (shell fractions full), aH, aL (heated tube fractions),
wdh (HP drains to LP), wd (pumped forward), wduh, wdul (dumps to the condenser), wv (LP vent), Tm (HP heater
inlet), xdh, xdl, hdp, hdvf; CST: wmu, wrj (makeup, reject kg/s), xmu, xrj, Mcst, cst (fraction full), Tcst;
condenser: Pc, Tc, pa (air partial pressure), ya, Ma, wcw, Tcwi (inlet), Tcw (outlet), wsj (motive steam), sjc
and vpc (capacity factors), wlk, wrad, wvb, wain, waout (air kg/s); core: Pth (fission + decay), nf, rho, per
(period), Tf, ac, ae, xe, Wc, dhin, rods (group positions 0–1), pc; crd, wcd (CRD pump flow); recirculation: rp, ng, mg, Wd; wrw, hrw (cleanup reject); trip
{scram, iso, turb, fwp} (latched protection groups). `fault` adds `'prompt'`, `'pool'` and `'cst'`. `fail(s,
'hdv', on)` fails the HP drain valve shut. Turbine (step 15): p1, pX, pLi (stage pressures), spd (shaft speed,
fraction), brk, lock (breaker), iv (intercept valves), plu (fast closure), Pm, Pw (shaft power, windage), Php,
Plp (HP and LP steam work), Pgen (electrical output); `breaker(s, close)` the operator's breaker switch. `API` 13.

## Control (bwr-control.js)

- Rods: manual only (`c.rods`, four group positions 00–48), as in real BWRs; `c.crd` the CRD pump switch. Recirculation breaker `c.rp`: a plain switch.
- Core flow (MG set speed, M/A `mg`): speed demand = KW·(wset·WCR − Wc) + I, KW = 2.3×10⁻⁵ per kg/s
  (10 % of rated flow → 3 %), integral 5 s, anti-windup 0 … 1. Only when `c.wset` is given. With the pump
  breaker open the integral tracks the MG set's actual speed (no windup while the pump makes no flow).

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
  per m, integral time 60 s, 0 … 2500 kg/s; then a flow controller on the feed valve, characterised on √flow:
  x = KXS·(√F − √fw) + I, KXS = KX·2√WR (KX = 10⁻⁴ per kg/s, the same gain as a linear loop at rated flow),
  integral time 4 s, anti-windup at 0 and 1 (the level integral also holds while the valve is saturated). The
  quadratic valve passes flow nearly as the square of its opening; a loop on the flow error itself had a gain
  that vanished near shut, so after an isolation it closed the valve ever more slowly and leaked enough cold feed
  to reach L8. SRV flow is not in the steam signal, so after a valve closure the level term alone has to call
  for feedwater. Level setpoint setdown after a scram: lset − `SDL` (0.45 m) for `SDT` 600 s, ramped back over
  `SDR` 1200 s, kept in the controller memory (`k.sdt`, the scram time) so a reset of the scram does not snap
  the setpoint back up; `setdown(k, s)`. Selector `c.fwe` (1 or 3, default 3): single-element drops the
  steam-flow feedforward (F = KL·e + I, same gains), as plants run at low power where the steam flow signal is
  small; changing it adds or removes the present steam flow to the integral, so F does not step. Operator-selected
  only (real systems transfer at about 20 to 25 % steam flow, and fall back to single-element on a bad flow signal).
- Startup level control (startup feed valve, M/A `sfv`): single-element, F_s = KLS·(lset − level) + I_s, KLS = 200
  kg/s per m, integral 120 s, 0 … 350 kg/s (`WSMAX`), then a flow controller on the startup valve against the
  startup line's flow `f.wsu`, on √flow with gain KXSU = KXS/√RSU (the same loop gain as the main one), integral
  4 s. Same setpoint and scram setdown as the main controller. In M the level integral holds the line's measured
  flow and the valve integral the slider. Why a separate, softer loop: at low pressure the cold feed's subcooling
  collapses core voids, so more feed first lowers the level (inverse response) and the void fraction per kelvin of
  subcooling is large. With the main controller (800 kg/s per m, tuned for rated) at 1 MPa and 200 MW the feed
  cycled between ~0 and ~220 kg/s every ~250 s and the level 14.17 … 14.48 m, growing; on the startup valve it
  holds within 0.01 m. Transfer up: main valve to A (single-element; its integral has tracked the measured total
  flow, and its flow loop works on the total, so it stays shut while the startup valve passes the flow), startup
  valve to M and closed in steps (the main flow loop picks the flow up), then three-element. With both in A the
  two level integrals share the error. `init(c, p)` with the plant state preloads both level integrals for the
  plant's initial steam flow, split between the valves by their openings.
- Hotwell level, split range (step 7): mk = (condensate pump flow − reject − return) + KH·(0.5 − hotwell fraction) + I,
  KH = 1000 kg/s per unit fraction, integral 300 s, ±100 kg/s, setpoint fixed at half full. The makeup valve
  demand is mk/100 above 0, the reject valve's −mk/100 below 0 (1 % per kg/s, about each valve's flow at a 70 %
  full CST), so they are never both asked open. Each valve has M/A (`auto.mu`, `auto.rj`; absent keys follow
  `auto.mk`); with both in M the integral holds the operators' net (mu − rj, % read as kg/s). The reject is
  left out of the feedforward because it is the loop's own output. A PI on an integrating level overshoots
  from an initial offset: from 60 % full it undershoots to about 46.6 % and settles in about 40 min.
- Feed pump speed: n = KN·(dpset − (Pd − P)) + I, KN = 1.5×10⁻⁸ /Pa, integral 10 s, 0 … 1: holds the pump
  discharge 2.2 MPa (default) above the dome, as turbine-driven feed pumps are run, so the feed valve keeps a
  steady drop at any vessel pressure. While the pump is stopped or tripped (switch, low-suction trip, L8) the
  integral tracks its actual speed.
- Heater extraction valves (LP and HP): manual only. Pump run switches: manual only.
- Turbine speed/load channel (EHC, step 15): a third limit on the control valves beside the pressure
  regulator and the load limit; the valves take the lowest, the bypass the rest of the pressure demand.
  Breaker closed: the load reference ramps to `c.mw` (MWe) at `LRATE` 5 MWe/s and becomes a flow demand
  through `MWR` (1115 MWe per unit of rated steam flow) plus a MW trim (integral `TIMW` 20 s, ±15 %, only
  while this channel holds the valves). Without `c.mw` it does not limit. Breaker open: proportional speed
  governor, droop `DROOP` 5 %, gain cut to `HIN`/dt at large steps (2.5 at dt 2 s, so the loop with the 5 s
  shaft stays stable); the speed reference ramps to `c.nset` (rpm, default 1800) at `ACC` 0.5 %/s and follows
  the actual speed while the stop valve is shut. The breaker itself is an operator action on the plant.
- Circulating water (pump, valve, inlet temperature), SJAE motive steam, vacuum pump and vacuum breaker:
  manual only, passed through by `command()` (no controller: the CW valve is set open and left there in
  normal operation, so an M/A would have nothing to do).
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
- On the page each controller is one box in the "Automatic control" card, like a unit in a control rack:
  the actuators it drives (chips, filled while in A), 3–5 readouts of the signals it works on, and its
  setpoints. A controller driving several actuators is one box (pressure regulator: control and bypass
  valves, with the load and speed sets; feedwater: feed and startup feed valves; heater drains; hotwell
  makeup and reject). `dem.ls` is the level setpoint in use after any scram setdown, for that readout.
  Each M/A switch's tooltip has a second line saying what drives its automatic demand.

## Validation numbers (tests/bwr.test.mjs, bwr-core, bwr-recirc, bwr-downcomer)

- Kinetics at rated: critical with all rods out (the power settles 0.3 % under the init heat balance), fuel 615 °C, void 39.5 %, axial peaking 1.53 at 44 % height; steady at dt 0.25 and 2 s.
- Control valve shut at rated, no scram: power peaks 1.50× within 12 s with the other valves fixed (1.62× before the
  axial shape), 1.37× in 5 s with the controllers (1.44×); core void 0.395 → 0.38; pressure past 15 MPa and core uncovered after ~80 s. With scram: level
  13.8 … 15.4 m, pressure peak 7.62 MPa.
- Lose feedwater heating with everything in A (rods fixed): HP drain valve fails shut: feed 47 K colder,
  power +16 %, no scram; HP extraction to 20 %: 27 K, +9 %; both extraction valves to 20 %: 34 K and +11 %
  at 1 min, 51 K and +17 % at 5 min, no trip. With the turbine model (step 15) the stage pressures rise as the
  extractions fall, so the heaters lose less than with extraction pressures ∝ flow (then: 52 K / +17 %,
  34 K / +11 %, 63 K / +21 % with an APRM trip after about 2 min). Before the axial shape: +33 % on the drain
  valve failure, +46 % with both valves (scram after ~50 s).
- Turbine and generator (tests/bwr-turbine.test.mjs): rated 1110 MWe, first stage 5.87 MPa (all rods out leaves the power 0.3 % under the heat balance). Load rejection at
  rated in A: fast closure and scram in the same step, speed peaks 105 % (dt 0.25 and 2 s), no overspeed trip,
  back to 1800 rpm (1793 rpm at dt 2) on the governor; pressure peaks 7.54 MPa, no SRV lifts (7.62 at dt 2, four SRVs
  lift); with the old uniform rod lever 7.39 / 7.48 MPa (the bottom-entry rods' scram worth builds up more slowly, see "Control rods").
  Turbine trip at rated: the breaker trips at once, the intercept valves shut, no overspeed; the shaft coasts
  down (61 % after 20 min). Load set 800 MWe at rated: output 800 MWe, the bypass 90 % open, HP extraction
  1.79 MPa, feed 200 °C; back to 1200: 1110 MWe. Turbine roll from hot standby at 5 % power: 1800 rpm in
  about 4 min, breaker closes on the sync check.
- Scram from rated: no motion for 0.2 s, 90 % in by 2.65 s, all in by 2.9 s; 5 % at 10 s, 3.3 % at 60 s (decayFrac 3.3 %).
- Startup: rods in, 8 W; groups 1, 2 out, group 3 two notches short of critical still subcritical; a notch
  past critical gives a stable period of about 28 s. Critical near position 19 hot, 11 at 100 °C.
- Hot standby 1.5 %: groups 1, 2 out, group 3 a few notches above its hot zero power position, group 4 in;
  holds within 5 % for 20 min with the pressure regulator.
- Rods at rated: group 4 to 24 → power −6 %, peak from 44 % to about 60 % height; to 40 → +1.6 % (see above).
- CRD pump: 3.8 kg/s, vessel + tank mass conserved; rods don't move with it stopped, the scram still works.
- Startup level control (tests/bwr-level.test.mjs): 1 MPa, 200 MW, main three-element: level swing > 0.15 m and feed
  swing > 100 kg/s; on the startup valve < 0.02 m and < 10 kg/s (dt 0.25 and 2 s). Transfer at 300 MW (startup valve
  → main single-element → startup valve closed 2 % per 10 s → three-element): level within 14.22 … 14.38 m, no
  step in the flow demand on the selector.
- Flow control: 40 % core flow → 58.4 % power (64.5 % before the axial shape), MG set ~26 %; back to 100 %.
- Pump trip at rated: coastdown, power dips to ~37 %, settles at 52 % on 33 % natural circulation.
- Natural circulation alone at 30 % heat: 31 % flow.
- Downcomer at rated: 9.1 K subcooled (Δh_in 46.5 kJ/kg); a feed temperature step reaches the core inlet
  with the ~6 s mixing time; mass and energy exact through closure + scram with kinetics.

- Rated heat balance (3293 MW, 216 °C feed, 7.0 MPa → 1788 kg/s) stays steady at dt 0.25 and 2 s, with the
  pump at full speed, the feed valve 74.5 % open, the heater at 216 °C and the hotwell half full.
- Steam line (tests/bwr-steam.test.mjs): 0.3 MPa dome-to-throttle at rated, control valve 86 % open, 105 % wide
  open, bypass 25 % at 6.7 MPa (26 % with the turbine shut, the header nearer dome pressure). Rated with
  everything in A steady at dt 0.25 and 2 s. Turbine trip at rated, no scram: bypass fully open 0.75 s after the
  trip (dt 0.25; two steps at dt 2), all 11 SRVs open within 20 s and pressure passes 8 MPa. With a scram: peak
  7.18 MPa (7.42 at dt 2), no SRV lifts, back to 7.0 MPa on the bypass (decay heat about 25 kg/s after 20 min).
  Load limit 80 % at rated: turbine 80 %, bypass 17 % of rated, pressure at the setpoint. MSIV closure: no
  steam past the MSIVs, no condensate. Startup with the stop valve shut: the bypass in A holds 1 MPa.
- MSIV closure at rated heat: the swell shrinks > 0.08 m and the void volume > 2 m³ in 6 s (core void
  collapse from the downcomer subcooling; before steps 9–10 the level fell > 0.4 m), pressure > 7.4 MPa. With no scram,
  pressure climbs past the feed pump's 10 MPa shutoff head in about 30 s, feed stops although the valve opens
  fully, and the core uncovers after about 80 s. With a scram, only the 1090 psig bank lifts and pressure
  peaks just above 7.6 MPa; the feed cools to the hotwell temperature (no extraction steam) and the level peaks about 1.6 m
  above normal (about 1.2 m with the pump speed in A). Isolation at 200 MW: the lowest bank cycles 7.35–7.62 MPa, about one lift per 45 s.
- Both extraction valves to 20 % at rated: feed about 38 K colder after 1 min, 72 K after 6 min (one stage: 35 and 54 K). With everything in
  A the steam flow settles 12 % lower and the level hardly moves; in M the falling pressure lets the pump
  deliver more, and the vessel floods in under 3 min.
- Pump speed in A through a depressurisation to 1 MPa: discharge held 2.2 MPa above the dome, speed down to
  about 55 %. The level wanders ±0.3 m there: at 1 MPa the core voids swing with the feed's subcooling,
  against the level controller (was ±0.15 m before steps 9–10).
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
  flow 12 % lower; repaired, back to 216 °C. Drain pump stopped: LP shell dumps ~615 kg/s, LP outlet ~67 °C,
  the HP extraction takes ~610 kg/s and draws its pressure down to 2.08 MPa, feed ~203 °C, power +4 %,
  output ~1053 MWe (with extraction pressures ∝ flow: feed ~211 °C, ~1015 MWe). Whole-loop mass (vessel, hotwell, condenser steam, both shells) conserved
  to 1 g with a dump and an SRV open.
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
  loop (suction from the downcomer, pump below the core, back to one jet pump in the downcomer; modelled,
  solid since steps 8–10), the
  inboard MSIV. Feed check valve where the feed line leaves containment.
- Main steam (modelled, solid): inboard and outboard MSIV, then a tee for the bypass line (down into the
  condenser upper shell, solid) and the reheat steam (up to the MSR, dashed), then the turbine stop valve and
  the control valve. Header pressure shown over the tee.
- Turbine hall (step 15, solid): HP turbine → crossover up into the moisture separator reheater (its drains and
  reheat steam dashed: the MSR is a pressure drop) → LP turbine (double flow, exhausts down into the
  condenser) → generator on the same shaft. The shaft speed is shown over the LP turbine, the output and the
  breaker state under the generator.
- Condenser: shell, tube bundle with circulating water in/out, air removal to the steam jet air ejector
  (off-gas; its motive steam comes from the main steam header over the turbine hall) and the mechanical
  vacuum pump (startup), vacuum breaker on the exhaust hood, hotwell at the bottom (the model's hotwell).
  All modelled (solid).
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
seal steam and its condenser, circulating water pumps and the cooling tower or river, reactor water cleanup (RWCU), the CRD pump
and hydraulic control units (modelled minimally, not drawn), main steam drains to the condenser, the generator's line to the grid (the breaker state is labelled). Emergency cooling
(RCIC, HPCI, core spray, RHR) is deliberately left for later.

## Not modelled

Radial and 3-D neutronics (point kinetics for the level, one-group quasi-static axial shape only: no
radial shape, xenon, burnup or pin peaking; generic feedback coefficients), individual rods (four ganged groups with
illustrative worths; no rod worth minimiser, rod sequence control or rod blocks; shallow rods give back their
worth through the voids, see "Control rods"), CRD hydraulics beyond the pump flow (accumulators always charged,
no drive water pressure or flow control, no scram discharge volume), subcooled
boiling, core bypass flow,
separate recirculation loops and jet pumps, recirculation pump heat, valves and limits, downcomer
stratification and steam carry-under, protection beyond the trips listed (no high drywell pressure, no
RCIC/HPCI/ECCS start on L2/L1, no ADS, no end-of-cycle recirculation pump trip, single logic channel: no
one-out-of-two-twice voting or channel tests), steam-line inventory and pressure waves (the header is
algebraic), separate main steam lines (one line stands for four), several control/bypass valves (one each),
steam superheat,
air in the dome (below 100 °C the pressure goes sub-atmospheric), separate vessel and internals temperatures
(one steel node), turbine details (one HP and one LP turbine, p·v held in the cone law, choked control valves,
pass-through MSR, no gland seal, drains, exhaust hood or rotor stresses; an infinite grid, gross output only),
circulating water pump curve, river or cooling tower
(an in/out pipe with a set inlet temperature), air distribution in the condenser (one shell-average air
fraction; no air-cooling section), SJAE stages and inter/after-condensers, off-gas train, moisture separation
and reheat, more than two heater stages, drain coolers, heater shell steam volume and heat capacity (the
shell pressure follows the steam flow at once; one throttled extraction line per stage, where real
extraction lines have only isolation and non-return valves), redundant pumps (one condensate pump stands for
the condensate and booster pumps, one feed pump for three), feed pump drive steam and the pump's heating of
the water, polisher chemistry, condensate and feed water inventory in
the pipes, reactor water cleanup (only a pumped reject placeholder to the condenser), containment pressure and
drywell (the pool has no air space; its heat-up is modelled), RHR (pool cooling is a fixed placeholder
cooler). Level control: no automatic single/three-element transfer and no signal validation; the startup
level controller's gains are illustrative, tuned on the 1 MPa case. Voids from bulk flashing beyond what the core boils rise out with one
time constant (2 s, illustrative).
