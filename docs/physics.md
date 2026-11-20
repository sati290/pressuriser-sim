# Physics model

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `VP` | 51 m³ | Pressuriser volume |
| `VR` | 280 m³ | RCS liquid volume (excluding pressuriser) |
| `CM` | 0.6 GJ/K | RCS metal heat capacity |
| `TAU` | 300 s | Insurge layer mixing time constant |
| Net heat | −60 … +60 MW | One operator control: pump work + core/decay heat − RHR/steam dump − losses. Pumps are assumed running (mixed RCS, spray head available) |
| Heater max | 1.8 MW | Pressuriser heaters (interlock: trip below 12 % level, always on) |
| Spray max | 40 kg/s | Main spray at RCS temperature |
| Relief valve | 15 kg/s steam at 16.2 MPa (∝ P); 30 kg/s liquid at 3.6 MPa (∝ √(P − 0.1 MPa)) | `relW(P, steam)` |
| Safety valve | 4 × relief valve flow | Fixed 17.2 MPa open / 16.9 MPa reclose |

## Properties

- Saturation: IAPWS 1992 supplementary release (Wagner & Pruss) — vapour pressure, ρ′, ρ″,
  auxiliary α → h′ = α + (T/ρ′)·dp/dT, h″ = α + (T/ρ″)·dp/dT. Good to ~0.1 % to near critical.
- Compressed liquid: u ≈ u_f(T); ρ(T,P) = ρ_f(T) / (1 − κ(T)·(P − p_sat(T))).
  So P(ρ,T) = p_sat(T) + (1 − ρ_f/ρ)/κ. κ from a table (0.44 /GPa at 50 °C … 3 /GPa at 340 °C).
- Liquid enthalpy at (T,P): h ≈ h_f(T) + (P − p_sat)/ρ_f.

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
| `M`, `U` | Mass and internal energy of the pressuriser's **saturated region** (or the whole liquid when solid) |
| `Ms`, `hs`, `rs` | Insurge layer: mass, specific enthalpy, density (bottom of pressuriser) |
| `Mr`, `Tr` | RCS liquid mass and temperature |
| `d` | Derived pressuriser state from `pz(M, U, VP − Ms/rs)` |
| `Pr` | RCS pressure from `rcsP(Mr, Tr)` (equals `d.P` after the surge solve) |
| `sv`, `rel` | Safety-valve latch, relief valve opening fraction 0–1 (as commanded) |
| `extra` | Timed extra RCS heat pulse `{Q, end}` (load step) |
| `f` | What was applied in the last step: `{Q, ws, ch, ld, wr, trip, Qr}` (after limits and interlocks) |

## Controller memory (`cs`, from `PZCtl.init()`)

| Field | Meaning |
|---|---|
| `iL`, `iC` | Net-inflow integrators of the pressure ('pres') and level ('lev') inventory logic |
| `dem` | Automatic demand for every actuator `{ch, ld, q (W), sp, rel (0–1)}`, plus `pset` |
| `hold` | Charging/letdown flows held by a secondary actuator in A |
| `porv` | Auto-relief latch (runs in M too) |

## Pressuriser state solve `pz(M,U,V)`

1. Find T with u_f(T) = U/M. If ρ̄ = M/V ≥ ρ_f(T) → **solid**: P from compressed-liquid relation.
2. Otherwise **two-phase**: find T with u_mix(T; v̄) = ū where x = (v̄ − v_f)/(v_g − v_f),
   restricted to temperatures where 0 ≤ x ≤ 1 (bisection). P = p_sat(T).
3. Errors: `dry` (no liquid left), `cold` (below table), `solid?` (no consistent state).

Solid↔two-phase transition is continuous: at x = 0, v̄ = v_f and P = p_sat.

## Control step `PZCtl.update(cs, s, c, dt)` (control.js)

Manual control is primary. The automatic logic runs every step (`demands(cs, s, c, dt)`) and writes a
demand for each actuator to `cs.dem`. `command(cs, c)` then builds the plant command: an actuator
takes its demand only when `c.auto[k]` is true (keys `ch, ld, q, sp, rel`), otherwise the operator's
value (`c.ch, c.ld, c.sp` kg/s, `c.q` kW, `c.relMan` %), converted to SI along with `c.heat` (MW) and
`c.loss` (kW). The logic acts regardless of pressuriser state;
**nothing switches between M and A, or between inventory modes, automatically**.

1. **Demands**:
   - Heaters and spray, `c.pzMode === 'pres'`: heaters = 1.8 MW·clamp((SP − P)/0.15 MPa + 0.2);
     spray = 40·clamp((P − SP − 0.1 MPa)/0.4 MPa).
     `c.pzMode === 'dT'` (hold pressuriser − coolant ΔT `c.dT`, K): error eT = (T_pz − Tr) − c.dT, with
     T_pz the saturated-region (or solid liquid) temperature. With a bubble T_pz = Tsat(P), so this holds
     P = p* = p_sat(Tr + c.dT), and the ΔT equals the coolant subcooling margin. Water-solid it acts on the
     liquid temperature while letdown sets the pressure. Default 53 K (15.5 MPa at 291.7 °C);
     heaters = 1.8 MW·clamp(0.2 − eT/1.25 K); spray = 40·clamp((eT − 0.5 K)/2.5 K). Bands are in kelvin
     so control stays proportionate at low pressure, where 0.15 MPa would be tens of kelvin.
     p* is stored as `cs.dem.pset`. Letdown pressure control always uses `c.set`.
   - Charging/letdown: a PI controller asks for net inflow N = charging − letdown.
     `c.inv === 'pres'` (water-solid): N = −5e-5·(P − SP) + ∫−2e-7·(P − SP)dt, preferred flow letdown.
     `c.inv === 'lev'` (bubble): N = 1.5·e + ∫0.004·e dt (e = level error, %), preferred flow charging.
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

`u = {ch, ld, sp (kg/s), q (W), rel (0–1), heat (W), loss (W)}`, the final command.

1. **Limits and protection**: each actuator is clamped to `LIM` (charging/letdown 0–20 kg/s, heaters
   0–1.8 MW, spray 0–40 kg/s, relief 0–1). Heaters trip below 12 % level in either mode. Safety valve
   17.2/16.9 MPa always armed. Relief flow = opening × `relW`; vents steam (two-phase) or liquid (solid).
   These act whoever commands, and are recorded in `s.f`.
2. **RCS energy**: dTr = (net heat + pulse)·dt / (Mr·cp + CM).
3. **Boundary flows**: Mr += (charging − letdown − spray)·dt; pressuriser saturated region gains
   spray at RCS enthalpy, heater power, minus losses and relief.
4. **Layer mixing**: m = Ms·dt/τ (+ small floor) moves from the layer into the saturated region with
   energy m·(hs − P/ρs) and volume m/ρs. If the pressuriser is solid the layer merges at once.
5. **Surge solve** (implicit): find dm (RCS → pressuriser, kg) such that
   P_pz(after dm) = P_rcs(Mr − dm, Tr). Bracket ±400 kg, expand ×3, then 44 bisections.
   - Insurge (dm > 0, two-phase): mass goes to the layer; saturated region gets compression work
     P·dm/ρr and loses volume dm/ρr.
   - Outsurge (dm < 0): drawn from the layer first, then saturated liquid at h_l. Hot outsurge
     water heats the RCS.
6. Re-derive `d`, total level = (saturated liquid volume + layer volume)/VP, and `Pr`.

The implicit surge solve is essential: RCS stiffness per kg is ~4–8 kPa when liquid, so an explicit
coupling would oscillate whenever the pressuriser is solid.

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
| +40 MW for 120 s at hot standby | insurge ~12 kg/s, P +~0.1 MPa, spray opens |
| Cold solid start, 1.8 MW heaters, net heat 0, letdown on pressure | bubble at ~230 °C after ~5.5 h; operator switches to level control; level to 25 % in ~2 h |
| Same, but nobody switches at the bubble | letdown winds up to 20 kg/s and drains the pressuriser |
| Heat-up, net heat +19.5 MW | coolant ~40 °C/h; letdown carries expansion |
| Refill to solid at 150 °C, operator switches back to letdown | goes solid, letdown holds 2.8 MPa |
| Manual relief at hot standby | ~13 kg/s steam, pressure falls |
| Auto relief at 15.8 and 16.6 MPa | opens within 0.02 MPa of the setting |
| Mass conservation, closed system (with the controller, and plant alone with fixed commands) | < 1 kg drift over 10 min |
