// BWR reactor pressure vessel plant model: pure physics, no DOM, no automatic control.
// step() takes final actuator commands (from the operator or from bwr-control.js; the plant can't tell).
// An ES module; imports common.js. Equations and constants are described in docs/bwr.md.
//
// The vessel is one fixed volume at one pressure holding water and saturated steam. The water is one
// well-mixed region that may be subcooled (after cold feedwater or cooling) but never stays above
// saturation: water above saturation flashes at once. Subcooled water condenses steam at a finite rate.
// Part of the steam is in transit as bubbles in the core and chimney (the voids): they occupy volume
// inside the two-phase mixture and so raise the level (swell) without changing the pressure.
import {sat,Tsat,clamp,liqRho,hLiq,Tuf,T0,THI} from './common.js';
// geometry: free fluid volume V (m³) over height H (m), taken as a uniform free cross-section A.
// Elevations above the inside bottom of the vessel (m): bottom and top of active fuel, top of the core
// shroud (separator standpipes start there), feedwater sparger, normal water level, main steam nozzle.
const V=600,H=21,A=V/H,Z={BAF:5.6,TAF:9.3,SHR:11.5,FW:13.0,NWL:14.3,MSL:16.5};
// main steam line: the valve stands for the MSIVs plus the turbine control and bypass valves together.
// Full open passes WMS kg/s at PR; flow ∝ P while choked, subsonic factor near the condenser pressure PC.
// The valve travels at most RATE of its stroke per second (full stroke in 5 s).
const WMS=2000,PR=7.0e6,PC=0.01e6,RATE=0.2;
// safety/relief valves (SRVs): NV valves, WV kg/s each at PV, discharging to the suppression pool at PB.
// Relief mode is commanded (operator or bwr-control.js); safety mode (spring) opens every valve above
// PSV and recloses below PSVR, whoever commands.
const NV=11,WV=113,PV=7.8e6,PB=0.1e6,PSV=8.5e6,PSVR=8.1e6;
// feedwater pump limit, kg/s; net heat range is the page's business
const FWMAX=2500;
// time constants (s), overridable per state as s.tau: subcooled water condenses steam (c); steam
// bubbles rise out of the two-phase mixture (b). Round illustrative values, not fitted to plant data.
const TAU=Object.freeze({c:10,b:0.7});
// steam nozzle flow factor against a back pressure pb: 1 while choked, ideal-gas subsonic nozzle
// factor (κ = 1.3) above the critical pressure ratio, 0 at pb
const KAP=1.3,RCR=(2/(KAP+1))**(KAP/(KAP-1));
const psi=r=>Math.sqrt(Math.max(0,r**(2/KAP)-r**((KAP+1)/KAP))),PSI=psi(RCR);
const subF=(P,pb)=>{if(P<=pb)return 0;const r=pb/P;return r<=RCR?1:psi(r)/PSI};
const wMS=(a,P)=>a*WMS*P/PR*subF(P,PC);
const wSRV=(n,P)=>n*WV*P/PV*subF(P,PB);
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
  const Vb=s.Mb/e.rg,level=Math.min(H,(e.Vl+Vb)/A),q=sat(e.Tl);
  return {P:e.P,T:e.T,Tl:e.Tl,sub:e.T-e.Tl,level,collapsed:e.Vl/A,Vb,Vl:e.Vl,Vv:V-e.Vl,Ml:e.Ml,Mv:s.Mv,Mb:s.Mb,rg:e.rg,rl:e.Ml/e.Vl,hf:q.hf};
};
// o: {P Pa, L m two-phase level, Tl °C water temperature (default saturated), gen kg/s steaming the
// voids start in equilibrium with, mv valve position 0–1, tau overrides}
const init=o=>{
  const s={t:0,mv:o.mv||0,sv:false,tau:{...TAU,...o.tau}};
  const T=Tsat(o.P);
  const q=sat(T),rg=q.rg,ug=q.ug,uf=q.uf,Tl=o.Tl!=null?Math.min(T,o.Tl+273.15):T;
  s.Mb=(o.gen||0)*s.tau.b;
  const Vl=o.L*A-s.Mb/rg,Ml=Vl*liqRho(Tl,o.P),ul=Tl<T?sat(Tl).uf:uf;
  s.Mv=(V-Vl)*rg;s.M=Ml+s.Mv;s.U=Ml*ul+s.Mv*ug;
  const e=solve({M:s.M,U:s.U,Mv:s.Mv,T0:T});s.Mv=e.Mv;s.d=derive(s,e);
  s.f={Q:0,fw:0,Tfw:0,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,gen:o.gen||0,cond:0,mv:s.mv};
  return s;
};
// One step. u is the final command, SI: {heat W net heat into the water; fw kg/s feedwater, tfw K its
// temperature; mv 0–1 main steam valve demand; srv SRVs open in relief mode (rounded)}.
// Each command is clamped to its range; the valve moves toward its demand at RATE; the SRV safety mode
// acts on top of the command.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau;
  const q=sat(d.T),hg=q.hg,ug=q.ug,uf=q.uf; // sat() reuses one object: copy fields first
  const a=clamp(u.mv||0,0,1);s.mv+=clamp(a-s.mv,-RATE*dt,RATE*dt);
  if(P>PSV)s.sv=true;else if(P<PSVR)s.sv=false;
  const nsrv=s.sv?NV:Math.round(clamp(u.srv||0,0,NV)),fw=clamp(u.fw||0,0,FWMAX),Tfw=clamp(u.tfw||T0,T0,THI),Q=u.heat||0;
  const wst=wMS(s.mv,P),wsrv=wSRV(nsrv,P),hfw=hLiq(Tfw,P);
  // flows at the start-of-step pressure: steam leaves at h″, feedwater joins the water, heat into the water
  const mo=Math.min((wst+wsrv)*dt,0.5*s.M);
  let M=s.M+fw*dt-mo,U=s.U+fw*dt*hfw-mo*hg+Q*dt,Mv=Math.max(0,s.Mv-mo);
  // subcooled water condenses steam, relaxing towards saturation at the start-of-step steam temperature
  let mc=0;{const Ml=M-Mv,ul=(U-Mv*ug)/Ml;
    if(Mv>0&&ul<uf)mc=Math.min(Ml*(uf-ul)/(ug-uf)*(1-Math.exp(-dt/tau.c)),0.9*Mv);Mv-=mc}
  const e=solve({M,U,Mv,T0:d.T});
  if(!e){s.d={err:'range',P:0};s.t+=dt;return}
  // net steam generation (flashing/boiling minus condensation) feeds the voids, which rise out with τb
  const gen=(e.Mv-Mv-mc)/dt,g=Math.max(0,gen)*tau.b;
  s.Mb=Math.min(g+(s.Mb-g)*Math.exp(-dt/tau.b),0.9*e.Mv);
  Object.assign(s,{M,U,Mv:e.Mv});s.t+=dt;
  s.d=derive(s,e);
  s.f={Q,fw,Tfw,hfw,hg,wst,wsrv,nsrv,gen,cond:mc/dt,mv:s.mv};
};
// why the model must stop, or '' while the state is valid
const fault=s=>{
  const d=s.d;
  if(d.err)return 'range';
  if(d.level<Z.TAF)return 'uncovered';
  if(d.level>=Z.MSL)return 'flooded';
  if(d.P>21.5e6)return 'critical';
  return '';
};
const API=1;
export {API,V,H,A,Z,WMS,PR,PC,RATE,NV,WV,PV,PB,PSV,PSVR,FWMAX,TAU,subF,wMS,wSRV,init,step,fault};
