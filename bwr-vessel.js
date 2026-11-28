// BWR reactor pressure vessel: geometry and the water–steam solve. Pure functions, no DOM.
// An ES module; imports common.js. bwr-sim.js composes it with the other systems. See docs/bwr.md.
//
// The vessel is one fixed volume at one pressure holding water and saturated steam. The water is two
// regions: the downcomer, which carries the feedwater's subcooling, and the core region, which may be
// subcooled (after cooling or a pressure rise) but never stays above saturation: water above saturation
// flashes at once. Subcooled core-region water condenses steam at a finite rate.
// Part of the steam is in transit as bubbles in the core and chimney (the voids): they occupy volume
// inside the two-phase mixture and so raise the level (swell) without changing the pressure.
import {sat,Tsat,liqRho,Tuf,T0,THI} from './common.js';
// geometry: free fluid volume V (m³) over height H (m), taken as a uniform free cross-section A.
// Elevations above the inside bottom of the vessel (m): bottom and top of active fuel, top of the core
// shroud (separator standpipes start there), feedwater sparger, normal water level, main steam nozzle.
const V=600,H=21,A=V/H,Z={BAF:5.6,TAF:9.3,SHR:11.5,FW:13.0,NWL:14.3,MSL:16.5};
// time constants (s), overridable per state as s.tau: subcooled water condenses steam (c); steam
// bubbles rise out of the two-phase mixture (b); feedwater heater outlet temperature (h). Round illustrative values, not fitted to plant data.
const TAU=Object.freeze({c:10,b:0.7,h:60});
// Downcomer: the water outside the shroud, from the separators down to the jet pump suction, plus the
// lower plenum, VDC = 100 m³ of liquid (illustrative). Its own specific internal energy udc carries the
// feedwater's subcooling (bwr-sim.js mixes it each step); the rest of the water, the core region
// (core, chimney, separator return), may not be above saturation. Mass Mdc per step from its density.
const VDC=100;
// State at trial steam (saturation) temperature T, from the totals M, U, the steam mass Mv carried in,
// and the downcomer (o.Mdc kg at o.udc; none if o.Mdc is 0): if the core-region water, after giving the
// steam its saturated energy, is above saturation it flashes to equilibrium; otherwise the steam stays
// as it is and the water is subcooled. A downcomer above saturation (after a pressure drop) flashes too,
// counted at u′. Residual r = V − V_water − V_steam rises with T (steam denser, less flashing).
const at=(o,T)=>{
  const q=sat(T),p=q.p,uf=q.uf,ug=q.ug,rg=q.rg,Md=o.Mdc||0,ue=Md?Math.min(o.udc,uf):uf;
  let Mv=o.Mv;const Ml0=o.M-Mv;
  if(!(Ml0>Md)||(o.U-Mv*ug-Md*ue)/(Ml0-Md)>uf)Mv=Math.max(Mv,(o.U-Md*ue-(o.M-Md)*uf)/(ug-uf));
  const Ml=o.M-Mv;if(!(Ml>Md))return {r:-Infinity};
  // water volume from the mean water temperature
  const Ul=o.U-Mv*ug,Tm=Math.min(Tuf(Ul/Ml),T),Vl=Ml/liqRho(Tm,p),Vv=Mv/rg;
  return {r:V-Vl-Vv,T,P:p,Mv,Ml,Ul,Tm,Vl,rg,uf,ue,Md};
};
// bracket around o.T0, then Illinois regula falsi on the temperature
const solve=o=>{
  const f=T=>at(o,T).r;let dT=0.5,lo=Math.max(T0+0.01,o.T0-dT),hi=Math.min(THI,o.T0+dT),flo=f(lo),fhi=f(hi);
  for(let k=0;k<40&&flo>0&&lo>T0+0.01;k++){hi=lo;fhi=flo;dT*=2;lo=Math.max(T0+0.01,lo-dT);flo=f(lo)}
  for(let k=0;k<40&&fhi<0&&hi<THI;k++){lo=hi;flo=fhi;dT*=2;hi=Math.min(THI,hi+dT);fhi=f(hi)}
  if(!(flo<=0&&fhi>=0))return null;
  let side=0,m=(lo+hi)/2;
  for(let i=0;i<100;i++){
    m=isFinite(flo)&&isFinite(fhi)&&fhi!==flo?(lo*fhi-hi*flo)/(fhi-flo):(lo+hi)/2;if(!(m>lo&&m<hi))m=(lo+hi)/2;
    const fm=f(m);if(Math.abs(fm)<1e-6||hi-lo<1e-8)break;
    if(fm<0){lo=m;flo=fm;if(side<0)fhi/=2;side=-1}else{hi=m;fhi=fm;if(side>0)flo/=2;side=1}
  }
  const e=at(o,m);return isFinite(e.r)?e:null;
};
// derived quantities the page and the controller read: Tl the core-region water (saturated while it
// boils), sub its subcooling; Tdc the downcomer water, subdc its subcooling; Tm the mean water
// temperature; E the downcomer's energy deficit below saturation (J), Mdc its mass. The level is the
// downcomer water surface, where the instrument taps are: water plus the voids inside the shroud,
// which push their volume of water out into the downcomer, over the uniform cross-section.
const derive=(s,e)=>{
  const Vb=s.Mb/e.rg,level=Math.min(H,(e.Vl+Vb)/A),Mc=e.Ml-e.Md,Tl=Math.min(Tuf((e.Ul-e.Md*e.ue)/Mc),e.T),Tdc=e.Md?Math.min(Tuf(e.ue),e.T):Tl,q=sat(Tl);
  return {P:e.P,T:e.T,Tl,sub:e.T-Tl,Tdc,subdc:e.T-Tdc,Tm:e.Tm,E:e.Md*(e.uf-e.ue),Mdc:e.Md,udc:e.ue,level,collapsed:e.Vl/A,Vb,Vl:e.Vl,Vv:V-e.Vl,Ml:e.Ml,Mv:s.Mv,Mb:s.Mb,rg:e.rg,rl:e.Ml/e.Vl,hf:q.hf};
};
// downcomer mass at temperature T, pressure P, for liquid mass Ml in the vessel (at most 60 % of it)
const dcMass=(T,P,Ml)=>Math.min(VDC*liqRho(T,P),0.6*Ml);
// vessel part of a new state from o {P Pa, L m two-phase level, Tl °C water (default saturated), gen kg/s
// steaming the voids start in equilibrium with, dh J/kg downcomer subcooling (internal energy below the
// rest of the water, default 0)}; sets s.M, s.U, s.Mv, s.Mb (unless already set), s.udc, s.d. Iterates
// the water mass until the level is o.L.
const vesselInit=(s,o)=>{
  const T=Tsat(o.P);
  const q=sat(T),rg=q.rg,ug=q.ug,uf=q.uf,Tl=o.Tl!=null?Math.min(T,o.Tl+273.15):T;
  s.Mb=s.Mb??(o.gen||0)*s.tau.b;
  const ul=Tl<T?sat(Tl).uf:uf,udc=ul-(o.dh||0),Tdc=Tuf(udc);let Vt=o.L*A-s.Mb/rg,Ml=Vt*liqRho(Tl,o.P),e;
  // the water at its mean temperature fills Vl, the steam the rest, so the solve lands on T
  for(let i=0;i<8;i++){
    const Md=dcMass(Tdc,o.P,Ml),Ul=(Ml-Md)*ul+Md*udc,Vl=Ml/liqRho(Math.min(Tuf(Ul/Ml),T),o.P);
    s.Mv=(V-Vl)*rg;s.M=Ml+s.Mv;s.U=Ul+s.Mv*ug;
    e=solve({M:s.M,U:s.U,Mv:s.Mv,T0:T,Mdc:Md,udc});s.Mv=e.Mv;s.udc=e.ue;s.d=derive(s,e);
    Ml*=1+(o.L-s.d.level)*A/Vl;
  }
};
export {V,H,A,Z,TAU,VDC,at,solve,derive,dcMass,vesselInit};
