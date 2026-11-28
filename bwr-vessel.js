// BWR reactor pressure vessel: geometry and the water–steam solve. Pure functions, no DOM.
// An ES module; imports common.js. bwr-sim.js composes it with the other systems. See docs/bwr.md.
//
// The vessel is one fixed volume at one pressure holding water and saturated steam. The water is one
// well-mixed region that may be subcooled (after cold feedwater or cooling) but never stays above
// saturation: water above saturation flashes at once. Subcooled water condenses steam at a finite rate.
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
// State at trial steam (saturation) temperature T, from the totals M, U and the steam mass Mv carried
// in: if the water, after giving the steam its saturated energy, is above saturation it flashes to
// equilibrium; otherwise the steam stays as it is and the water is subcooled.
// Residual r = V − V_water − V_steam rises with T (steam denser, less flashing).
const at=(o,T)=>{
  const q=sat(T),p=q.p,uf=q.uf,ug=q.ug,rg=q.rg;
  let Mv=o.Mv;const Ml0=o.M-Mv;
  if(!(Ml0>0)||(o.U-Mv*ug)/Ml0>uf)Mv=Math.max(Mv,(o.U-o.M*uf)/(ug-uf));
  const Ml=o.M-Mv;if(!(Ml>0))return {r:-Infinity};
  const Ul=o.U-Mv*ug,Tl=Math.min(Tuf(Ul/Ml),T),Vl=Ml/liqRho(Tl,p),Vv=Mv/rg;
  return {r:V-Vl-Vv,T,P:p,Mv,Ml,Ul,Tl,Vl,rg};
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
// derived quantities the page and the controller read
const derive=(s,e)=>{
  const Vb=(s.Mb+(s.Mbf||0))/e.rg,level=Math.min(H,(e.Vl+Vb)/A),q=sat(e.Tl);
  return {P:e.P,T:e.T,Tl:e.Tl,sub:e.T-e.Tl,level,collapsed:e.Vl/A,Vb,Vl:e.Vl,Vv:V-e.Vl,Ml:e.Ml,Mv:s.Mv,Mb:s.Mb,rg:e.rg,rl:e.Ml/e.Vl,hf:q.hf};
};
// vessel part of a new state from o {P Pa, L m two-phase level, Tl °C water (default saturated), gen kg/s
// steaming the voids start in equilibrium with}; sets s.M, s.U, s.Mv, s.Mb, s.d
const vesselInit=(s,o)=>{
  const T=Tsat(o.P);
  const q=sat(T),rg=q.rg,ug=q.ug,uf=q.uf,Tl=o.Tl!=null?Math.min(T,o.Tl+273.15):T;
  s.Mb=(o.gen||0)*s.tau.b;
  const Vl=o.L*A-s.Mb/rg,Ml=Vl*liqRho(Tl,o.P),ul=Tl<T?sat(Tl).uf:uf;
  s.Mv=(V-Vl)*rg;s.M=Ml+s.Mv;s.U=Ml*ul+s.Mv*ug;
  const e=solve({M:s.M,U:s.U,Mv:s.Mv,T0:T});s.Mv=e.Mv;s.d=derive(s,e);
};
export {V,H,A,Z,TAU,at,solve,derive,vesselInit};
// ---- vessel steel and level instruments (roadmap step 12) ----
// Vessel and internals steel as one node: heat capacity CST (about 1200 t of steel at 500 J/kg/K:
// vessel ~800 t plus internals; illustrative), exchanging UAS with the water (time constant CST/UAS =
// 10 min) and losing UAL·(Ts − TDW) through the insulation to the drywell at a fixed TDW (about 1 MW at
// rated; the drywell coolers are not modelled). Illustrative numbers, not a fitted plant model.
const CST=6e8,UAS=1e6,UAL=4200,TDW=50+273.15;
// one step of the steel node at water temperature Tl: implicit, returns {Ts, Qw heat into the water W,
// Ql heat lost to the drywell W} (both as the mean over the step)
const steelStep=(Ts,Tl,dt)=>{
  // C dTs/dt = −UAS(Ts − Tl) − UAL(Ts − TDW): exponential towards the weighted temperature
  const U=UAS+UAL,Te=(UAS*Tl+UAL*TDW)/U,T1=Te+(Ts-Te)*Math.exp(-U*dt/CST),Tm=Te+(Ts-Te)*CST/(U*dt)*(1-Math.exp(-U*dt/CST));
  return {Ts:T1,Qw:UAS*(Tm-Tl),Ql:UAL*(Tm-TDW)}};
// Level instruments (generic BWR/4, typical values; plants differ): levels are read in inches above
// instrument zero IZ, 528 in (13.41 m) above the inside bottom (vessel zero). Top of active fuel is then
// −162 in. Narrow range 0 … +60 in, wide range −150 … +60 in. Setpoints L8 high (turbine and feed pump
// trip), L7 high alarm, L4 low alarm, L3 low (scram), L2 low-low (isolation, RCIC/HPCI start), L1
// low-low-low (ECCS, ADS permissive).
const INCH=0.0254,IZ=528*INCH,LV=Object.freeze({L8:54,L7:39,L4:27,L3:12.5,L2:-38,L1:-129});
const toIn=z=>(z-IZ)/INCH,fromIn=x=>IZ+x*INCH;
// time constant (s) for voids from bulk flashing to rise out (a depressurisation flashes water throughout
// the downcomer and lower plenum, and those bubbles have further to go than the core's); illustrative
const TAUF=2;
export {CST,UAS,UAL,TDW,steelStep,INCH,IZ,LV,toIn,fromIn,TAUF};
// Reactor water cleanup reject (a placeholder for the RWCU system, which is not modelled): a pumped line
// that takes vessel water at up to RWMAX kg/s to the main condenser, so the level can be held while the
// water swells on a heat-up. Opening u.rw 0–1, flow ∝ opening, no pump curve. Real RWCU rejects run at
// some 5–15 kg/s; illustrative.
const RWMAX=10;
export {RWMAX};
