// BWR primary containment: the suppression pool (roadmap step 12). Pure functions, no DOM.
// An ES module; imports common.js. See docs/bwr.md, "Suppression pool".
import {sat,inv} from './common.js';
// Mark I-type pool (generic BWR/4): about 3800 m³ of water (MPOOL kg) over a free surface APOOL m²,
// normally at 32 °C (90 °F). The SRV tailpipes discharge under water through quenchers: the steam
// condenses completely, so the pool gains the steam's mass and enthalpy. Pool cooling (an RHR loop in
// the real plant, which is deferred) is a placeholder cooler: Q = UAP·(T − TSW) when on, against service
// water at TSW. Limits shown on the page (typical BWR/4 technical specifications): 95 °F (35 °C) normal
// maximum, 110 °F (43.3 °C) scram required, 120 °F (48.9 °C) depressurise; near 100 °C the pool would
// boil and pressurise the containment, which is not modelled, so the model stops there.
const MPOOL=3.8e6,APOOL=1100,TP0=32+273.15,UAP=1.5e6,TSW=30+273.15,TPMAX=100+273.15;
const PLIM=Object.freeze({normal:35+273.15,scram:(110-32)/1.8+273.15,depress:(120-32)/1.8+273.15});
const Thp=inv(T=>sat(T).hf);
// one step: pool {M, T}; w kg/s of steam at h J/kg in; cool true for the cooler. Returns {M, T, Qc W}
const poolStep=(p,w,h,cool,dt)=>{
  const Qc=cool?UAP*Math.max(0,p.T-TSW):0,M=p.M+w*dt,U=p.M*sat(p.T).hf+w*dt*h-Qc*dt;
  return {M,T:Thp(U/M),Qc}};
// pool water level above its normal level (m)
const poolLevel=M=>(M-MPOOL)/(sat(TP0).rf*APOOL);
export {MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolStep,poolLevel};
