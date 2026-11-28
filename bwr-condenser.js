// BWR turbine stub and main condenser. Pure functions, no DOM. An ES module; imports common.js and
// bwr-steam.js. See docs/bwr.md, "Turbine stub" and "Condenser".
import {sat,satS,Tsat,T0} from './common.js';
import {PR,WR} from './bwr-steam.js';
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
// One lumped shell of steam space VC (m³) over the hotwell, saturated at its temperature Tc (pressure
// p_sat(Tc)). Everything that enters (turbine exhaust, drains, later bypass steam) is split, on its
// enthalpy, into the vapour it holds at Tc and the liquid that falls straight to the hotwell. The tube
// bundle condenses wc = Q/h_fg, Q = m·cp·(Tc − T_in)·(1 − e^(−UA/(m·cp))) (the effectiveness form of the
// log-mean temperature difference, exact for a condensing shell). The steam space mass ρ″(Tc)·VC is
// solved implicitly each step (it holds only a few hundred kg, so its time constant is a fraction of a
// second). Condensate leaves at Tc. Circulating water flow WCW and inlet TCW are fixed until the
// circulating water system is modelled. Numbers: illustrative for a 1100 MWe-class unit (about 38 m³/s
// of circulating water, design back pressure near 6.5 kPa); UA chosen to give that, not sourced.
const VC=4000,WCW=38000,TCW=20+273.15,CPW=4180,UA=2.4e8,PCMAX=0.135e6;
const cwQ=(T,w,Tin,ua)=>w>0&&T>Tin?w*CPW*(T-Tin)*(1-Math.exp(-ua/(w*CPW))):0;
// one step of the shell: c {Tc, Ms}; ins = [[w kg/s, h J/kg], …]; cw {w, T}. Updates c, returns
// {Pc, Tc, wc condensed kg/s, wl liquid to the hotwell kg/s (all at h′(Tc)), Q to the circulating water W,
// Tout circulating water outlet K}
const condense=(c,ins,dt,cw={w:WCW,T:TCW})=>{
  const vap=T=>{const q=sat(T),hf=q.hf,hfg=q.hg-q.hf;let v=0;for(const[w,h]of ins)v+=w*(h-hf)/hfg;return v};
  const wcT=T=>{const q=sat(T),hfg=q.hg-q.hf;return cwQ(T,cw.w,cw.T,UA)/hfg};
  const f=T=>sat(T).rg*VC-c.Ms-(vap(T)-wcT(T))*dt;
  let a=T0+0.01,b=473.15;
  if(f(a)>=0)b=a;else for(let i=0;i<60;i++){const m=(a+b)/2;if(f(m)<0)a=m;else b=m}
  const T=b,q=sat(T),Pc=q.p,Ms=q.rg*VC,hfg=q.hg-q.hf;
  const wc=wcT(T),win=ins.reduce((x,[w])=>x+w,0),wl=win-(Ms-c.Ms)/dt;
  c.Tc=T;c.Ms=Ms;
  const Q=wc*hfg;
  return {Pc,Tc:T,wc,wl,Q,Tout:cw.w>0?cw.T+Q/(cw.w*CPW):cw.T};
};
// the condenser temperature at which the shell is steady for steady inflows ins
const condSteady=(ins,cw={w:WCW,T:TCW})=>{
  const f=T=>{const q=sat(T),hf=q.hf,hfg=q.hg-q.hf;let v=0;for(const[w,h]of ins)v+=w*(h-hf)/hfg;return v-cwQ(T,cw.w,cw.T,UA)/hfg};
  let a=cw.T+1e-3,b=473.15;if(f(a)<=0)return a;
  for(let i=0;i<60;i++){const m=(a+b)/2;if(f(m)>0)a=m;else b=m}return (a+b)/2};
export {ETA,ETAG,sPH,expansion,turbinePower,VC,WCW,TCW,CPW,UA,PCMAX,cwQ,condense,condSteady};
