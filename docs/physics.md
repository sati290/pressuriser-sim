# Physics model

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `VP` | 51 m³ | Pressuriser volume |
| `VR` | 280 m³ | RCS liquid volume (excluding pressuriser) |
| `CM` | 0.6 GJ/K | RCS metal heat capacity |
| `TAU` | 300 s | Insurge layer mixing time constant |
| `TAUNE` | fl 5 s, sc 300 s, sh 30 s | Water–gas transfers: water above the interface temperature boils or evaporates (fl), water below it condenses steam at its surface (sc), the gas exchanges heat with the water (sh). Round illustrative values, not fitted to plant data; the same with nitrogen. Per state as `s.tau` (`init({tau})`) |
| `MVMIN` | 1 g | Steam left with no nitrogen below this condenses into the water at once (the pressuriser goes solid) |
| Net heat | −60 … +60 MW | One operator control: pump work + core/decay heat − RHR/steam dump − losses. Pumps are assumed running (mixed RCS, spray head available) |
| Heater max | 1.8 MW | Pressuriser heaters (interlock: trip below 12 % level, always on) |
| Spray max | 40 kg/s | Main spray at RCS temperature |
| Relief valve | 15 kg/s steam at 16.2 MPa (∝ P while choked, × subsonic nozzle factor ψ(PB/P)/ψ* above the critical ratio 0.546, κ = 1.3, zero at PB); 30 kg/s liquid at 3.6 MPa (∝ √(P − PB)) | `relW(P, steam)`, `subF(P)`. Discharge to a relief tank at PB = 0.1 MPa abs (fixed: the tank does not fill or pressurise). All pressures in the model are absolute |
| Pressuriser heat loss | UA·(T − T_amb), UA = 200 W/K (61 kW at hot standby in 40 °C air) | T_amb = 40 °C fixed (`TAMB`); T is the saturated-region (or solid liquid) temperature; recorded as `s.f.loss` |
| Safety valve | 4 × relief valve flow | Fixed 17.2 MPa open / 16.9 MPa reclose |
| Gas with nitrogen | relief flow × √(R_s·T_sat(P) / (R_mix·T)) | `gasF(d)`: ideal-gas choked-flow ratio (∝ P/√(RT)) of the mixture vs saturated steam at the same P; exactly 1 for steam alone |
| Nitrogen supply | 4.0 MPa header (`N2P`), 0.5 kg/s·√(ΔP / 1 MPa) (`N2W`), 20 °C, check valve | `n2W(P)`; flow × valve opening 0–1 (`u.n2`), operator only (no M/A) |
| Nitrogen | R = 296.8, c_v = 743, c_p = 1039.8 J/(kg·K), ideal gas, u = c_v·T | `RN`, `CVN` |
| `N2MIN` | 0.1 % | Below this share of the gas (partial pressure / P), with the supply shut, the nitrogen is dropped |

## Properties

- Saturation: IAPWS 1992 supplementary release (Wagner & Pruss) — vapour pressure, ρ′, ρ″,
  auxiliary α → h′ = α + (T/ρ′)·dp/dT, h″ = α + (T/ρ″)·dp/dT. Good to ~0.1 % to near critical.
- Compressed liquid: u ≈ u_f(T); ρ(T,P) = ρ_f(T) / (1 − κ(T)·(P − p_sat(T))).
  So P(ρ,T) = p_sat(T) + (1 − ρ_f/ρ)/κ. κ from a table (0.44 /GPa at 50 °C … 3 /GPa at 340 °C).
- Liquid enthalpy at (T,P): h ≈ h_f(T) + (P − p_sat)/ρ_f.
- Superheated steam (`vap(ρ, u)` in sim.js): at constant density ρ = ρ″(Ts) and T = Ts + θ,
  P − p_sat(Ts) = a1·θ + a2·θ², u − u″(Ts) = b1·θ + b2·θ², with a1, a2, b1, b2 tabulated against Ts
  (`VT`, 40–365 °C) from IAPWS-IF97 region 2 by `tools/vapour-table.mjs` (slopes at θ → 0, curvature fitted
  at 20 K). Within ~1 % up to 10 K superheat, ~3 % at 40 K. Above ~350 °C (16.5 MPa) saturated steam is in
  IF97 region 3 and the fit is extrapolated. Below u″ at that density the steam is a saturated mixture
  (Newton on the mixture energy). Ts from ρ and the liquid temperature from u_f use 4000-interval inverse
  tables built at load.

## Files

| File | Global | Contents |
|---|---|---|
| `common.js` | `PZCommon` | Steam tables (`satX`, `sat`, `Tsat`), `clamp`, actuator limits `LIM`. Used by both of the others |
| `sim.js` | `PZSim` | The plant: state, `init`, `step(s, u, dt)`, `fault`. Knows nothing about M/A or setpoints |
| `control.js` | `PZCtl` | Automatic control: its own memory, demands, and the final command `u` |

Each step the page (and the tests' `run`) calls `u = PZCtl.update(cs, s, c, dt)` and then
`PZSim.step(s, u, dt)`, with the same dt, so the controller sees the plant state at the start of the step.

## Plant state variables (`s`)

| Field | Meaning |
|---|---|
| `Ml`, `Ul`, `Vl` | Water region (above the insurge layer): mass, internal energy, volume |
| `Mv`, `Ug`, `Vg` | Gas space: steam mass, internal energy of the gas (steam u + nitrogen c_v·T), volume. All 0 when water-solid |
| `M`, `U` | Totals `Ml + Mv`, `Ul + Ug` (kept for tests and displays) |
| `tau` | The transfer time constants |
| `Mn` | Nitrogen mass in the gas space (0 = steam alone or water-solid) |
| `Ms`, `hs`, `rs` | Insurge layer: mass, specific enthalpy, density (bottom of pressuriser) |
| `Mr`, `Tr` | RCS liquid mass and temperature |
| `d` | Derived state (`derive`, at the step's solved pressure): `mode` ('solid' with no gas space, else 'two'), `P`, `Tl` water, `Tg` gas, `Ti` interface temperature = T_sat(steam partial pressure) (T_sat(P) with steam alone or solid), `T` = `Ti` with a gas space or `Tl` when solid (what the controller and gauges use), `level`, `pS` steam partial pressure, `pN`, `yN`, `Mg` steam mass, `Ml`, `Vg`, `hl`, `hg` steam enthalpy |
| `Pr` | RCS pressure from `rcsP(Mr, Tr)` (equals `d.P` after the surge solve) |
| `sv`, `rel` | Safety-valve latch, relief valve opening fraction 0–1 (as commanded) |
| `extra` | Timed extra RCS heat pulse `{Q, end}` (load step) |
| `f` | What was applied in the last step: `{Q, ws, ch, ld, wr, wrn, wn, n2, trip, Qr, loss}` (after limits and interlocks; `wrn` nitrogen part of the relief flow, `wn` nitrogen supply) |

## Controller memory (`cs`, from `PZCtl.init()`)

| Field | Meaning |
|---|---|
| `iL`, `iC` | Net-inflow integrators of the pressure ('pres') and level ('lev') inventory logic |
| `dem` | Automatic demand for every actuator `{ch, ld, q (W), sp, rel (0–1)}`, plus `pset` |
| `hold` | Charging/letdown flows held by a secondary actuator in A |
| `porv` | Auto-relief latch (runs in M too) |

## Pressuriser model: water and gas, never forced into equilibrium

Above the insurge layer the pressuriser is two regions at one pressure P. **Water** (Ml, Ul): compressed
liquid at T_l from u_f(T_l) = u_l, ρ_l(T_l, P); it may be below or above saturation. **Gas space** (Mv, Mn,
Ug, volume Vg): steam and nitrogen at one gas temperature T_g. Nitrogen is an ideal gas, p_N = Mn·R_N·T_g/Vg;
steam at density Mv/Vg and T_g is superheated (`vapT`, from the IF97 fit) or, below the saturation
temperature for its density, wet; P = p_v + p_N (Dalton). With steam alone T_g comes straight from the energy
(`vap`); with both, T_g solves Ug = Mv·u_v(ρ_v, T_g) + Mn·c_v·T_g (safeguarded Newton, `gas`).
With no gas space the pressuriser is **water-solid**: the water fills what the layer leaves.

The interface temperature is T_i = T_sat(p_v) (T_sat(P) with steam alone or when solid). The same equations
cover water-solid, the steam bubble and the nitrogen cushion; nothing switches model. A bubble forms when
water-solid water gets above T_sat(P) and boils into the empty gas space; the gas space disappears when the
last steam (below `MVMIN`, no nitrogen) condenses. As Mn → 0 the gas tends to steam alone (tested to 20 Pa,
1 mK at Mn = 1 µg).

`init` builds water at Tpz (solid), a bubble with water and steam at T_sat(P), or a cushion with water and
gas at Tpz, saturated steam plus nitrogen making up P − p_sat(Tpz).

## Control step `PZCtl.update(cs, s, c, dt)` (control.js)

Manual control is primary. The automatic logic runs every step (`demands(cs, s, c, dt)`) and writes a
demand for each actuator to `cs.dem`. `command(cs, c)` then builds the plant command: an actuator
takes its demand only when `c.auto[k]` is true (keys `ch, ld, q, sp, rel`), otherwise the operator's
value (`c.ch, c.ld, c.sp` kg/s, `c.q` kW, `c.relMan` %), converted to SI along with `c.heat` (MW). The logic acts regardless of pressuriser state;
**nothing switches between M and A, or between inventory modes, automatically**.

1. **Demands**:
   - Heaters and spray, `c.pzMode === 'pres'`: heaters = 1.8 MW·clamp((SP − P)/0.15 MPa + 0.2);
     spray = 40·clamp((P − SP − 0.1 MPa)/0.4 MPa).
     `c.pzMode === 'dT'` (hold pressuriser − coolant ΔT `c.dT`, K): error eT = (T_pz − Tr) − c.dT, with
     T_pz = `d.T`, the interface temperature (or the water temperature when solid). With a steam bubble T_pz = Tsat(P), so this holds
     P = p* = p_sat(Tr + c.dT), and the ΔT equals the coolant subcooling margin. Water-solid it acts on the
     liquid temperature while letdown sets the pressure. Default 53 K (15.5 MPa at 291.7 °C);
     heaters = 1.8 MW·clamp(0.2 − eT/1.25 K); spray = 40·clamp((eT − 0.5 K)/2.5 K). Bands are in kelvin
     so control stays proportionate at low pressure, where 0.15 MPa would be tens of kelvin.
     p* is stored as `cs.dem.pset`. Letdown pressure control always uses `c.set`.
   - Charging/letdown: a PI controller asks for net inflow N = charging − letdown.
     `c.inv === 'pres'` (water-solid): N = −5e-5·(P − SP) + ∫−2e-7·(P − SP)dt, preferred flow letdown.
     `c.inv === 'lev'` (bubble): N = 20·e + ∫0.025·e dt (e = level error, %; full charging range over 1 %), preferred flow charging.
     Who controls: if only one flow is in A, that one (charging = other + N, or letdown = other − N);
     if both are in A, the preferred one, with the other held at `cs.hold` (its flow when it stopped
     controlling, so holding is bumpless). The held flow's demand is preferred ∓ N, which equals its
     held value until the preferred flow is clamped at 0 or 20 kg/s; then it takes over.
     A flow's demand (`cs.dem`) is what it would do if it were in A with the other as it is now, so
     the notch on a manual slider shows what switching it to A gives.
     Anti-windup: the integrator (`iL` for 'pres', `iC` for 'lev') stops when the controlling flow,
     at 0 or 20 kg/s, cannot deliver N in the direction the error pushes. So in M the demand runs
     up to the actuator limit and stops there; it does not wind beyond.
     With a bubble, 'pres' winds letdown to its limit and drains the level.
   - `engage(cs, c)` preloads both integrators with charging − letdown (call on an inventory-mode change).
2. **Relief**: the auto latch `cs.porv` sets at P > `c.relSet` and clears below relSet − max(0.2 MPa, 2 %);
   demand = 1 or 0. Commanded opening = demand in A, `c.relMan`/100 in M.

The controller reads the plant state directly (ideal, instantaneous measurements).

## Plant step `PZSim.step(s, u, dt)` (sim.js)

`u = {ch, ld, sp (kg/s), q (W), rel (0–1), n2 (0–1), heat (W)}`, the final command.

1. **Limits and protection**: each actuator is clamped to `LIM` (charging/letdown 0–20 kg/s, heaters
   0–1.8 MW, spray 0–40 kg/s, relief 0–1). Heaters trip below 12 % level (not when solid). Safety valve
   17.2/16.9 MPa always armed. Relief flow = opening × `relW` (× `gasF` with nitrogen); vents gas, or
   water when solid. These act whoever commands, and are recorded in `s.f`.
2. **RCS**: dTr = (net heat + pulse)·dt / (Mr·cp + CM); Mr += (charging − letdown − spray)·dt.
3. **Explicit transfers**: heat loss UA·(T − 40 °C) shared by volume fraction, each region at its own
   temperature; heaters into the water. Relief takes gas, nitrogen by its mass share
   Mn·(1 − e^(−m/(Mv + Mn))) (so it can't overdraw) at c_p·T_g, the rest steam at h_v; or water at h_l when solid.
   Nitrogen supply adds `u.n2`·`n2W(P)` at c_p·293 K. The gas exchanges heat with the water,
   (Ug − Ug at T_t)·(1 − e^(−dt/τ_sh)) at constant density, with T_t = T_i for steam alone (superheat only;
   steam below saturation is wet and rains out instead) and T_t = T_l with nitrogen. Spray joins the water.
   The insurge layer mixes into the water (m = Ms·dt/τ plus a small floor, enthalpy h_s; all at once when solid).
4. **Pressure solve** (one unknown, P; Illinois regula falsi; `neSolve`/`neAt`):
   - The surge follows from P explicitly, dm = Mr − VR·ρ_l(Tr, P): insurge to the layer (straight into the
     water when solid), outsurge from the layer first, then water at h_l. Hot outsurge water heats the RCS.
   - Interface transfers, with T_i taken at P (linearised about the start of the step, through the steam
     partial pressure), which keeps a small gas space stable:
     spray condenses w_s·(h′ − h_r)/(h_v − h′) of steam (drops reach T_i);
     water above T_i, Δ = u_l − u′(T_i) > 0, boils or evaporates m = Ml·Δ·(1 − e^(−dt/τ_fl))/(h″ − h′) into the gas
     at h″ (with nitrogen at most up to ρ″(T_l)·Vg − Mv, saturation at the water temperature);
     water below T_i condenses m = Ml·(−Δ)·(1 − e^(−dt/τ_sc))/(h_v − h′) of steam.
   - Each region does boundary work −P·ΔV against its start-of-step volume (two fixed-point passes for the water).
   - Residual: P_gas(Vg, Ug) − P with a gas space, else −(room left)·10¹⁰; it falls as P rises.
5. **Rain-out**: wet steam's condensate moves into the water at u′(T_g) with its volume.
6. **Nitrogen gone**: with the supply shut and nitrogen below `N2MIN` of the gas, Mn = 0 and its c_v·T leaves Ug.
   **Steam gone**: with no nitrogen and Mv < `MVMIN`, the steam joins the water (water-solid).
7. Re-derive `d` and `Pr`.

All transfers carry enthalpy and every region does its own boundary work, so water and nitrogen mass and
energy are conserved between regions. The implicit surge solve is essential: RCS stiffness per kg is
~4–8 kPa when liquid, so an explicit coupling would oscillate whenever the pressuriser is solid.

## Halts (`fault(s)` in sim.js; `physTick` stops on it)

Pressuriser dry, state solve error, P > 21.5 MPa, or RCS subcooling margin < 0 (the single-node
RCS can't represent boiling).

## Validation numbers (from tests/physics.test.mjs)

| Check | Result |
|---|---|
| Saturation props vs IAPWS tables at 100/200/300/345 °C | within 0.2–0.3 % |
| Hot standby (15.5 MPa, 291.7 °C, 25 %) for 1 h, controls in A or all in M | steady |
| ΔT mode, 53 K, hot standby then −13.5 MW cooldown 292 → 235 °C | ΔT 53.2–53.6 K throughout, 15.5 MPa at start |
| ΔT mode, 53 K, cold water-solid start, letdown on pressure | pressuriser settles 53 K above the coolant at 2.8 MPa |
| Level 40 → 25 % with charging and letdown both in A | charging goes to 0, letdown takes over, then returns to its held flow |
| Only letdown in A in level mode; only charging in A in pressure mode | the flow in A controls; the manual one is untouched |
| +40 MW for 120 s at hot standby, charging/letdown in M | insurge ~12 kg/s, P +~0.1 MPa, spray opens |
| Same, charging/letdown in A on level | level peaks 25.6 %, back at 25 % in ~2 min, no undershoot below 24.9 % |
| Cold solid start, 1.8 MW heaters, net heat 0, letdown on pressure | bubble at ~230 °C after ~5.4 h; operator switches to level control; level to 25 % in ~2 h |
| Same, but nobody switches at the bubble | letdown winds up to 20 kg/s and drains the pressuriser |
| Heat-up, net heat +19.5 MW | coolant ~40 °C/h; letdown carries expansion |
| Refill to solid at 150 °C, operator switches back to letdown | goes solid, letdown holds 2.8 MPa |
| Manual relief at hot standby | ~13 kg/s steam, pressure falls |
| Auto relief at 15.8 and 16.6 MPa | opens within 0.02 MPa of the setting |
| Mass conservation, closed system (with the controller, and plant alone with fixed commands) | < 1 kg drift over 10 min |
| Superheated steam fit vs IF97 region 2 at 345 °C + 10 K and 250 °C + 15 K | P − p_sat within 1 %, T within 0.2 K |
| Hot standby, 1 h, heaters matching losses | water and steam within 0.05 K of saturation |
| +40 MW for 120 s at hot standby, everything manual | peak +0.60 MPa; steam superheats ~0.9 K, water ~3 K subcooled; overshoot decays over ~10 min |
| Same with τ = 0.25/1/1 s (near equilibrium) | peak +0.31 MPa (the former equilibrium model: +0.30), within 0.15 K of saturation |
| Relief fully open 60 s at hot standby, then shut | water superheats ~0.7 K; pressure recovers ~30 kPa in the next minute (fast rates: none) |
| Insurge peak, dt 2 s vs 0.25 s | within 2 % |
| Gas at Mn = 1 µg vs steam alone | P within 20 Pa, T within 1 mK |
| Nitrogen into a steam bubble at 2.5 MPa | no pressure step; cold nitrogen cools the gas below the water |
| 2.4 t insurge into a cold cushion in 2 min | gas heats above the water, relaxes within 10 min |
| Cold nitrogen cushion (60 °C, 2.8 MPa, 25 %), balanced, 1 h | steady |
| Closed system with nitrogen, then relief open | water and nitrogen balances close (< 1 kg, < 1 µg) |
| Nitrogen into water-solid, letdown on pressure | gas space forms; pressure rises to ~3.1 MPa while open (inflow beats 20 kg/s letdown), back to 2.8 MPa once shut |
| Nitrogen startup, 1.8 MW heaters, relief in A at 3.0 MPa, level in A | nitrogen purged ~2.8 h at ~232 °C; pressure step at the switch < 0.01 MPa |
| Cooldown from 175 °C steam bubble, spray 3 kg/s, operator works the nitrogen valve | pressuriser reaches coolant temperature, gas > 60 % nitrogen, P ≥ 2.7 MPa |
