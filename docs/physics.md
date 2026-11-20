# Physics model

## Geometry and constants

| Symbol | Value | Meaning |
|---|---|---|
| `VP` | 51 m³ | Pressuriser volume |
| `VR` | 280 m³ | RCS liquid volume (excluding pressuriser) |
| `CM` | 0.6 GJ/K | RCS metal heat capacity |
| `TAU` | 300 s | Insurge layer mixing time constant |
| RCP heat | 5 MW per pump | Pump work into coolant |
| RCS losses | 0.5 MW | Fixed ambient loss |
| Heater max | 1.8 MW | Pressuriser heaters (trip below 12 % level) |
| Spray max | 40 kg/s | Main spray, needs ≥1 RCP running |

## Properties

- Saturation: IAPWS 1992 supplementary release (Wagner & Pruss) — vapour pressure, ρ′, ρ″,
  auxiliary α → h′ = α + (T/ρ′)·dp/dT, h″ = α + (T/ρ″)·dp/dT. Good to ~0.1 % to near critical.
- Compressed liquid: u ≈ u_f(T); ρ(T,P) = ρ_f(T) / (1 − κ(T)·(P − p_sat(T))).
  So P(ρ,T) = p_sat(T) + (1 − ρ_f/ρ)/κ. κ from a table (0.44 /GPa at 50 °C … 3 /GPa at 340 °C).
- Liquid enthalpy at (T,P): h ≈ h_f(T) + (P − p_sat)/ρ_f.

## State variables (`s`)

| Field | Meaning |
|---|---|
| `M`, `U` | Mass and internal energy of the pressuriser's **saturated region** (or the whole liquid when solid) |
| `Ms`, `hs`, `rs` | Insurge layer: mass, specific enthalpy, density (bottom of pressuriser) |
| `Mr`, `Tr` | RCS liquid mass and temperature |
| `d` | Derived pressuriser state from `pz(M, U, VP − Ms/rs)` |
| `Pr` | RCS pressure from `rcsP(Mr, Tr)` (equals `d.P` after the surge solve) |
| `iL`, `iC` | Integrators for letdown (solid pressure) and charging (level) controllers |
| `porv`, `sv`, `lt` | Relief valve latches (PORV, safety valve, LTOP) |
| `extra` | Timed extra RCS heat pulse `{Q, end}` (load step) |

## Pressuriser state solve `pz(M,U,V)`

1. Find T with u_f(T) = U/M. If ρ̄ = M/V ≥ ρ_f(T) → **solid**: P from compressed-liquid relation.
2. Otherwise **two-phase**: find T with u_mix(T; v̄) = ū where x = (v̄ − v_f)/(v_g − v_f),
   restricted to temperatures where 0 ≤ x ≤ 1 (bisection). P = p_sat(T).
3. Errors: `dry` (no liquid left), `cold` (below table), `solid?` (no consistent state).

Solid↔two-phase transition is continuous: at x = 0, v̄ = v_f and P = p_sat.

## One time step `step(s, c, dt)`

1. **Controllers** (`c.autoP`, `c.autoL`):
   - Solid + autoP: letdown = charging + 5e-5·(P − SP) + ∫2e-7·(P − SP)dt, clamped 0–20 kg/s.
   - Two-phase + autoP: heaters = 1.8 MW·clamp((SP − P)/0.15 MPa + 0.2); spray = 40·clamp((P − SP − 0.1 MPa)/0.4 MPa).
   - Two-phase + autoL: net = 1.5·e + ∫0.004·e dt (e in %); charging = letdown + net; if that is
     negative, letdown is raised instead.
2. **Relief**: below 150 °C RCS, LTOP opens > 3.6 MPa (30 kg/s); above, PORV 16.2/15.9 MPa
   (15 kg/s) and safety valve 17.2/16.9 MPa (60 kg/s). Vents steam (two-phase) or liquid (solid).
3. **RCS energy**: dTr = (RCP + core − cooling − loss + pulse)·dt / (Mr·cp + CM).
4. **Boundary flows**: Mr += (charging − letdown − spray)·dt; pressuriser saturated region gains
   spray at RCS enthalpy, heater power, minus losses and relief.
5. **Layer mixing**: m = Ms·dt/τ (+ small floor) moves from the layer into the saturated region with
   energy m·(hs − P/ρs) and volume m/ρs. If the pressuriser is solid the layer merges at once.
6. **Surge solve** (implicit): find dm (RCS → pressuriser, kg) such that
   P_pz(after dm) = P_rcs(Mr − dm, Tr). Bracket ±400 kg, expand ×3, then 44 bisections.
   - Insurge (dm > 0, two-phase): mass goes to the layer; saturated region gets compression work
     P·dm/ρr and loses volume dm/ρr.
   - Outsurge (dm < 0): drawn from the layer first, then saturated liquid at h_l. Hot outsurge
     water heats the RCS.
7. Re-derive `d`, total level = (saturated liquid volume + layer volume)/VP, and `Pr`.

The implicit surge solve is essential: RCS stiffness per kg is ~4–8 kPa when liquid, so an explicit
coupling would oscillate whenever the pressuriser is solid.

## Halts (in `physTick`, UI side)

Pressuriser dry, state solve error, P > 21.5 MPa, or RCS subcooling margin < 0 (the single-node
RCS can't represent boiling).

## Validation numbers (from tests/physics.test.mjs)

| Check | Result |
|---|---|
| Saturation props vs IAPWS tables at 100/200/300/345 °C | within 0.2–0.3 % |
| Hot standby (15.5 MPa, 291.7 °C, 25 %) for 1 h | steady |
| +40 MW for 120 s at hot standby | insurge ~12 kg/s, P +~0.1 MPa, spray opens |
| Cold solid start, 1.8 MW heaters, 2 RCPs, RHR 9.5 MW | bubble at ~230 °C after ~5.5 h; level to 25 % in ~2 h |
| Heat-up 4 RCPs, no cooling | coolant ~40 °C/h; letdown carries expansion |
| Refill to solid at 150 °C | goes solid, letdown holds 2.8 MPa |
| Mass conservation, closed system | < 1 kg drift over 10 min |
