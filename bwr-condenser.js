// BWR turbine stub, main condenser, circulating water and condenser air removal. Pure functions, no DOM.
// An ES module; imports common.js and bwr-steam.js. See docs/bwr.md, "Turbine stub", "Condenser",
// "Circulating water" and "Condenser air".
import {sat,satS,Tsat,T0,clamp} from './common.js';
import {PR,WR,QR as QRA,subF} from './bwr-steam.js';
// ---- turbine stub (until the turbine island is modelled) ----
// The steam that passes the turbine valves expands along one fixed expansion line from the throttle
// state to the condenser pressure with isentropic efficiency ETA: at pressure p the steam has
// h(p) = h0 − ETA·(h0 − h_s(p)), h_s the isentropic enthalpy from the throttle entropy. Extraction steam
// for the feed heaters leaves the line at its pressure; the rest reaches the condenser wet. The gross
// electrical output is the work of each flow over its part of the line times the generator efficiency
// ETAG. No moisture separation or reheat (the MSR is drawn only), no shaft speed, no stage pressures:
// the extraction pressures are in proportion to the steam flow (see bwr-feed.js).
const ETA=0.85,ETAG=0.985;
// entropy of steam at (p, h): wet from the saturation tables, a little superheat as dh/T
const sPH=(p,h)=>{const T=Tsat(p),q=sat(T),hf=q.hf,hg=q.hg,[sf,sg]=satS(T);
  return h<=hg?sf+(h-hf)/(hg-hf)*(sg-sf):sg+(h-hg)/T};
// the expansion line from the throttle (p0, h0): a function giving h at p
const expansion=(p0,h0)=>{const s0=sPH(p0,h0);
  return p=>{if(!(p<p0))return h0;const T=Tsat(p),q=sat(T),hf=q.hf,hg=q.hg,[sf,sg]=satS(T),x=(s0-sf)/(sg-sf);
    const hs=x<=1?hf+x*(hg-hf):hg+(s0-sg)*T;return h0-ETA*(h0-hs)}};
// gross electrical output (W) of the stub: steam wt enters at h0; ext = [{w, h}] leaves in order of
// falling pressure; the rest leaves at the exhaust enthalpy hx
const turbinePower=(wt,h0,ext,hx)=>{let w=wt,h=h0,W=0;
  for(const e of ext){W+=w*(h-e.h);w=Math.max(0,w-e.w);h=e.h}
  return ETAG*(W+w*(h-hx))};
// ---- condenser ----
// One lumped shell of volume VC (m³) over the hotwell, holding steam and air at one temperature Tc
// (Dalton): the steam is saturated at Tc, so its partial pressure is p_sat(Tc); the air (mass Ma, ideal
// gas, R = 287 J/(kg K)) adds pa = Ma·R·Tc/VC, and the shell pressure is Pc = p_sat(Tc) + pa. Everything that
// enters (turbine exhaust, drains, SJAE motive steam, later bypass steam) is split, on its enthalpy, into
// the vapour it holds at Tc and the liquid that falls straight to the hotwell. The tube bundle condenses
// wc = Q/h_fg, Q = m·cp·(Tc − T_in)·(1 − e^(−UA/(m·cp))) (the effectiveness form of the log-mean
// temperature difference, exact for a condensing shell). Air blanketing: noncondensable gas collects at
// the tubes and the steam has to diffuse through it, so UA = UA0/(1 + (ya/YA)²), ya = pa/Pc the air mole
// fraction of the shell (shell average; a real condenser concentrates the air in its air-cooling section).
// The steam mass ρ″(Tc)·VC is solved implicitly each step (a few hundred kg, a time constant of a fraction
// of a second); the air mass is fixed over the solve, so the Tc solve stays one-dimensional. Condensate
// leaves at Tc (below the saturation temperature at Pc when there is air: condensate subcooling).
// Numbers: illustrative for a 1100 MWe-class unit (about 38 m³/s of circulating water, design back
// pressure near 7 kPa). UA is chosen to give that, not sourced: UAD 2.4×10⁸ W/K is the blanketed value
// at the rated design point (shell 39.1 °C with the rated air partial pressure PA0), UA0 the clean bundle.
const VC=4000,WCW=38000,TCW=20+273.15,CPW=4180,UAD=2.4e8,PCMAX=0.135e6,TCD=39.1+273.15;
const cwQ=(T,w,Tin,ua)=>w>0&&T>Tin?w*CPW*(T-Tin)*(1-Math.exp(-ua/(w*CPW))):0;
// ---- condenser air ----
// Air in-leakage WLK (0.009 kg/s ≈ 15 scfm, a typical design allowance for a large condenser,
// illustrative) through glands, flanges and the LP turbine casing, scaled by the nozzle factor from
// atmosphere PATM (constant while choked, zero once the shell is at atmospheric). Radiolytic H₂/O₂ from the
// core reaches the condenser with the main steam: WRAD (0.02 kg/s at rated power, counted as air,
// illustrative) × Q/QR × the main steam share of the steam leaving the vessel; it is not taken out of the
// vessel inventory (it is ~10⁻⁵ of the steam flow).
// Steam jet air ejector (SJAE): an effective volumetric flow VSJ (m³/s) of shell-average gas, so it removes
// VSJ·Ma/VC of air. A real ejector draws air-rich gas from the air-cooling section through a much smaller
// suction volume; VSJ is sized so that at rated the air partial pressure is a few hundred Pa (PA0, below).
// It needs motive steam (WSJ 2.5 kg/s, from the main steam header through a reducing station set at PSJ1
// 1.2 MPa, so the flow falls ∝ supply pressure below it): capacity × clamp((Pm − 0.7 MPa)/0.5 MPa, 0, 1).
// The ejector breaks (its compression ratio is exceeded) when the suction rises above 25 kPa: capacity falls
// linearly to 0 at 35 kPa. Its motive steam is taken off the main steam ahead of the turbine and ends up in
// the condenser (inter- and after-condenser drains), counted at its supply enthalpy. Mechanical vacuum pump (startup): volumetric
// VVP, capacity falling from full at 30 kPa to 0 at 7 kPa (a liquid-ring pump near its limit), so it pulls the
// shell from atmosphere to about 17 kPa in an hour and leaves the rest to the SJAE.
// Vacuum breaker: a valve from atmosphere into the shell (opening x, stroke 20 s), choked flow WVB·x at
// deep vacuum, zero at atmospheric; it breaks vacuum from 7 kPa in about 4 min. All numbers illustrative.
const RA=287,PATM=0.101325e6,WLK=0.009,WRAD=0.02,VSJ=10,WSJ=2.5,PSJ0=0.7e6,PSJ1=1.2e6,PSB0=25e3,PSB1=35e3,
  VVP=2.2,PVP0=7e3,PVP1=30e3,WVB=20,VBRATE=0.05,YA=0.1;
const pAir=(Ma,T)=>Math.max(0,Ma||0)*RA*T/VC;
const blanket=ya=>1/(1+(ya/YA)**2);
const yAir=(T,pa)=>pa>0?pa/(sat(T).p+pa):0;
// rated air partial pressure: in-leakage plus radiolytic gas against the SJAE at the design shell temperature
const PA0=(WLK+WRAD)*RA*TCD/VSJ,UA0=UAD/blanket(yAir(TCD,PA0)),UA=UAD;
// SJAE motive steam flow and capacity factor from the motive supply pressure Pm and the suction pressure pc
const sjSteam=(Pm,pc)=>Pm>pc?WSJ*clamp(Pm/PSJ1,0,1):0;
const sjCap=(Pm,pc)=>clamp((Pm-PSJ0)/(PSJ1-PSJ0),0,1)*clamp((PSB1-pc)/(PSB1-PSB0),0,1);
const vpCap=pc=>clamp((pc-PVP0)/(PVP1-PVP0),0,1);
// air flows into and out of the shell at the start-of-step shell pressure pc, temperature Tc, air Ma:
// o {Q W core heat, share main steam / all steam out, sj fraction of the SJAE motive steam supplied (0–1),
// Pm motive supply Pa,
// vp vacuum pump running, vb breaker opening 0–1, dt}. Returns {wlk, wrad, wvb, wi kg/s in, Vsj, Vvp, Vr m³/s}.
const airFlows=(pc,Tc,o)=>{
  const fa=subF(PATM,pc),wlk=WLK*fa,wrad=WRAD*Math.max(0,o.Q)/QRA*clamp(o.share||0,0,1);
  // the breaker cannot fill the shell above atmospheric within the step
  const room=Math.max(0,(PATM-pc)*VC/(RA*Tc)/(o.dt||1));
  const wvb=Math.min(WVB*clamp(o.vb||0,0,1)*fa,room);
  const Vsj=VSJ*clamp(+o.sj||0,0,1)*sjCap(o.Pm,pc),Vvp=o.vp?VVP*vpCap(pc):0;
  return {wlk,wrad,wvb,wi:wlk+wrad+wvb,Vsj,Vvp,Vr:Vsj+Vvp};
};
// air mass after dt with inflow wi and volumetric removal Vr of shell gas: dMa/dt = wi − Vr·Ma/VC, exact
const airStep=(Ma,wi,Vr,dt)=>{const k=Vr/VC;if(!(k>0))return Ma+wi*dt;const Me=wi/k;return Me+(Ma-Me)*Math.exp(-k*dt)};
// ---- circulating water ----
// An in/out pipe through the condenser tubes: one pump (start/stop) against a fixed system resistance and
// a butterfly valve (opening x). With the pump running, w = √(HCW/(KSYS + KCV/x²)), HCW the pump head taken
// as flat (no pump curve), sized so the open valve passes the design WCW with 10 % of the head across the
// valve. The flow follows that with a first-order lag TAUCW (pump run-up and coast-down, water inertia).
// Inlet temperature is an operator input (river or tower water, no heat sink model). Illustrative numbers.
const HCW=1.5e5,KCV=0.1*HCW/WCW**2,KSYS=0.9*HCW/WCW**2,TAUCW=12;
const cwTarget=(on,x)=>{x=clamp(x,0,1);return on&&x>0?Math.sqrt(HCW/(KSYS+KCV/(x*x))):0};
const cwStep=(w,on,x,dt)=>{const wt=cwTarget(on,x);return wt+(w-wt)*Math.exp(-dt/TAUCW)};
// one step of the shell: c {Tc, Ms, Ma}; ins = [[w kg/s, h J/kg], …]; cw {w, T}; air {wi kg/s, Vr m³/s}
// (absent: the air mass stays as it is). Updates c, returns {Pc, Tc, pa air partial pressure, ya, wc
// condensed kg/s, wl liquid to the hotwell kg/s (all at h′(Tc)), Q to the circulating water W, Tout
// circulating water outlet K, wout air removed kg/s}
const condense=(c,ins,dt,cw={w:WCW,T:TCW},air)=>{
  const Ma0=c.Ma||0,Ma=air?Math.max(0,airStep(Ma0,air.wi,air.Vr,dt)):Ma0;
  const vap=T=>{const q=sat(T),hf=q.hf,hfg=q.hg-q.hf;let v=0;for(const[w,h]of ins)v+=w*(h-hf)/hfg;return v};
  const ua=T=>UA0*blanket(yAir(T,pAir(Ma,T)));
  const wcT=T=>{const q=sat(T),hfg=q.hg-q.hf;return cwQ(T,cw.w,cw.T,ua(T))/hfg};
  const f=T=>sat(T).rg*VC-c.Ms-(vap(T)-wcT(T))*dt;
  let a=T0+0.01,b=473.15;
  if(f(a)>=0)b=a;else for(let i=0;i<60;i++){const m=(a+b)/2;if(f(m)<0)a=m;else b=m}
  const T=b,wc=wcT(T),q=sat(T),pa=pAir(Ma,T),Pc=q.p+pa,Ms=q.rg*VC,hfg=q.hg-q.hf;
  const win=ins.reduce((x,[w])=>x+w,0),wl=win-(Ms-c.Ms)/dt;
  const wout=air?(Ma0+air.wi*dt-Ma)/dt:0;
  c.Tc=T;c.Ms=Ms;c.Ma=Ma;
  const Q=wc*hfg;
  return {Pc,Tc:T,pa,ya:pa/Pc,wc,wl,Q,Tout:cw.w>0?cw.T+Q/(cw.w*CPW):cw.T,wout};
};
// the condenser temperature at which the shell is steady for steady inflows ins, with air partial pressure
// pa at the tubes (default the rated PA0)
const condSteady=(ins,cw={w:WCW,T:TCW},pa=PA0)=>{
  const f=T=>{const q=sat(T),hf=q.hf,hfg=q.hg-q.hf;let v=0;for(const[w,h]of ins)v+=w*(h-hf)/hfg;return v-cwQ(T,cw.w,cw.T,UA0*blanket(yAir(T,pa)))/hfg};
  let a=cw.T+1e-3,b=473.15;if(f(a)<=0)return a;
  for(let i=0;i<60;i++){const m=(a+b)/2;if(f(m)>0)a=m;else b=m}return (a+b)/2};
export {ETA,ETAG,sPH,expansion,turbinePower,VC,WCW,TCW,CPW,UA,UA0,UAD,PCMAX,cwQ,condense,condSteady,
  RA,PATM,WLK,WRAD,VSJ,WSJ,PSJ0,PSJ1,PSB0,PSB1,VVP,PVP0,PVP1,WVB,VBRATE,YA,PA0,pAir,yAir,blanket,sjSteam,sjCap,vpCap,airFlows,airStep,
  HCW,KCV,KSYS,TAUCW,cwTarget,cwStep};
