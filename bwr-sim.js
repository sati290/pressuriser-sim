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
// safety/relief valves (SRVs), discharging to the suppression pool at PB. Each valve has a spring
// that opens it fully above its set pressure p (Pa abs) and recloses it BD below, and can also be
// opened by hand (relief mode, a command from the operator). Capacity w (kg/s) is rated at its set
// pressure; the flow scales ∝ P (choked) with the subsonic factor near PB. Values from a GE BWR/4
// training lesson plan: 4 valves at 1090 psig rated 800,000 lb/h, 4 at 1100 psig 808,000 lb/h,
// 3 at 1110 psig 815,000 lb/h (the same valve model; capacity ∝ set pressure). Blowdown BD is
// illustrative (about 3 %); the source gives none.
const psig=x=>(x+14.696)*6894.757,lbh=x=>x*1.25998e-4;
const SRV=Object.freeze([[1090,800e3],[1090,800e3],[1090,800e3],[1090,800e3],[1100,808e3],[1100,808e3],[1100,808e3],[1100,808e3],[1110,815e3],[1110,815e3],[1110,815e3]]
  .map(([p,w])=>Object.freeze({p:psig(p),w:lbh(w)})));
const NV=SRV.length,PB=0.1e6,BD=0.25e6;
// Feedwater train: condenser hotwell → feed pump → feedwater heater → feed control valve → sparger.
// Hotwell: a tank of water at the condenser saturation temperature THW (Tsat at PC), capacity HWM kg,
// normal half full. Until a condenser is modelled the main steam returns to it at once as condensate
// (an ideal condenser: return = main steam flow, at THW); makeup from / reject to the condensate storage
// tank up to MKMAX kg/s either way. condensate(s,wst) is the one place that decides the return, so a
// condenser model can replace it.
// Pump (the condensate, booster and feed pumps lumped as one): developed pressure n²·H0 − KP·w² at
// speed n (affinity laws), suction at PC; it loses suction as the hotwell runs below 5 % (flow tapers
// to zero). Heater and piping drop KL·w². Feed valve: drop w²/(KV·x²)² (quadratic characteristic,
// opening x), strokes at most FRATE per second. Check valve: no reverse flow. Numbers chosen so that
// at full speed and 7 MPa the valve passes the rated 1788 kg/s about 75 % open and 2400 kg/s wide open.
const H0=10e6,KPU=0.24,KLN=0.094,KV=2.31,FRATE=0.1,HWM=600e3,MKMAX=100;
const THW=Tsat(PC);
// pump flow at speed n (0–1), valve opening x (0–1), vessel pressure P, hotwell mass Mh
const fwFlow=(n,x,P,Mh=HWM/2)=>{const D=n*n*H0+PC-P,f=x*x;if(!(D>0)||!(f>0))return 0;
  return Math.sqrt(D/(KPU+KLN+1/(KV*f)**2))*clamp(Mh/(0.05*HWM),0,1)};
// valve opening that passes w at speed n against P (null if the pump can't), the inverse of fwFlow
const fwValve=(w,n,P)=>{const D=n*n*H0+PC-P;if(!(w>0))return 0;const r=D/(w*w)-KPU-KLN;return r>0?Math.sqrt(1/(KV*Math.sqrt(r))):null};
// Feedwater heater: one stage standing for the whole heater train, heated by steam extracted from the
// turbine. The extraction pressure is in proportion to the main steam flow (the turbine's stage
// pressures are), set so that at rated steam flow WR the feed leaves at 216 °C, TTD (3 K) below the
// extraction saturation temperature. Extraction valve e (0–1) scales the heating from THW up to that.
// The outlet temperature follows with time constant TAU.h (heater and pipe holdup).
const TTD=3,TFR=216+273.15,WR=3293e6/(sat(Tsat(PR)).hg-hLiq(TFR,PR)),PE=sat(TFR+TTD).p;
const heaterT=(wst,e)=>{const pe=PE*Math.max(0,wst)/WR,Tx=pe>sat(THW+TTD).p?Tsat(pe)-TTD:THW;return THW+clamp(e,0,1)*(Tx-THW)};
// steady heat balance at pressure P: steam flow W that heat Q makes from feed heated by that same W
const heatBalance=(Q,P,e=1)=>{const hg=sat(Tsat(P)).hg;let T=THW,W=0;
  for(let i=0;i<60;i++){W=Q/(hg-hLiq(T,P));T=heaterT(W,e)}return {W,Tfw:T}};
// the condensate returning to the hotwell (kg/s, K): an ideal condenser for now
const condensate=(s,wst)=>({w:wst,T:THW});
// Scram: the rods go in and core heat falls to the fission tail plus decay heat, a fraction of the
// power at the scram P0 after t seconds. Fission (93.5 % of steady power): prompt drop as the rods
// insert, then the delayed-neutron tail (a 10 s and an 80 s group). Decay heat 6.5 % at the scram,
// falling as (1 + t)^−0.2 (Way–Wigner form, long operation at P0 assumed). Rough: about 7 % at 10 s,
// 3.3 % at 1 min, 1.3 % at 1 h, 0.6 % at a day.
const decayFrac=t=>{t=Math.max(0,t);return 0.935*(0.93*Math.exp(-t/2)+0.06*Math.exp(-t/10)+0.01*Math.exp(-t/80))+0.065*(1+t)**-0.2};
// time constants (s), overridable per state as s.tau: subcooled water condenses steam (c); steam
// bubbles rise out of the two-phase mixture (b); feedwater heater outlet temperature (h). Round illustrative values, not fitted to plant data.
const TAU=Object.freeze({c:10,b:0.7,h:60});
// steam nozzle flow factor against a back pressure pb: 1 while choked, ideal-gas subsonic nozzle
// factor (κ = 1.3) above the critical pressure ratio, 0 at pb
const KAP=1.3,RCR=(2/(KAP+1))**(KAP/(KAP-1));
const psi=r=>Math.sqrt(Math.max(0,r**(2/KAP)-r**((KAP+1)/KAP))),PSI=psi(RCR);
const subF=(P,pb)=>{if(P<=pb)return 0;const r=pb/P;return r<=RCR?1:psi(r)/PSI};
const wMS=(a,P)=>a*WMS*P/PR*subF(P,PC);
const wSRV=(i,P)=>SRV[i].w*P/SRV[i].p*subF(P,PB);
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
// voids start in equilibrium with, mv valve position 0–1, fv feed valve position 0–1, tfw °C heater
// outlet (default THW), hw hotwell fraction full (default 0.5), tau overrides}
const init=o=>{
  const s={t:0,mv:o.mv||0,fv:o.fv||0,Th:o.tfw!=null?o.tfw+273.15:THW,Mh:(o.hw??0.5)*HWM,spr:Array(NV).fill(false),scram:null,tau:{...TAU,...o.tau}};
  const T=Tsat(o.P);
  const q=sat(T),rg=q.rg,ug=q.ug,uf=q.uf,Tl=o.Tl!=null?Math.min(T,o.Tl+273.15):T;
  s.Mb=(o.gen||0)*s.tau.b;
  const Vl=o.L*A-s.Mb/rg,Ml=Vl*liqRho(Tl,o.P),ul=Tl<T?sat(Tl).uf:uf;
  s.Mv=(V-Vl)*rg;s.M=Ml+s.Mv;s.U=Ml*ul+s.Mv*ug;
  const e=solve({M:s.M,U:s.U,Mv:s.Mv,T0:T});s.Mv=e.Mv;s.d=derive(s,e);
  s.f={Q:0,fw:0,Tfw:s.Th,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,open:Array(NV).fill(false),spr:Array(NV).fill(false),gen:o.gen||0,cond:0,mv:s.mv,
    n:0,fv:s.fv,ext:0,Tt:s.Th,pe:0,Pd:PC,ret:0,Tret:THW,mk:0,Mh:s.Mh,hw:s.Mh/HWM};
  return s;
};
// One step. u is the final command, SI: {heat W net heat into the water; n 0–1 feed pump speed;
// fv 0–1 feed valve demand; ext 0–1 heater extraction valve; mk kg/s hotwell makeup (negative: reject);
// mv 0–1 main steam valve demand; srv array, true for each SRV opened by hand}.
// After scram(s) the core heat is the decay curve and u.heat is ignored until reset(s).
// Each command is clamped to its range; the valves move toward their demands at RATE and FRATE; each
// SRV's spring opens it on pressure whatever the command. Feed flow follows from the pump, the valve and
// the vessel pressure; it enters at the heater outlet temperature.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau;
  const q=sat(d.T),hg=q.hg,ug=q.ug,uf=q.uf; // sat() reuses one object: copy fields first
  const a=clamp(u.mv||0,0,1);s.mv+=clamp(a-s.mv,-RATE*dt,RATE*dt);
  // SRVs: spring on pressure (with blowdown), or opened by hand
  const man=u.srv||[],open=SRV.map((v,i)=>{if(P>v.p)s.spr[i]=true;else if(P<v.p-BD)s.spr[i]=false;return s.spr[i]||!!man[i]});
  let wsrv=0,nsrv=0;open.forEach((o,i)=>{if(o){wsrv+=wSRV(i,P);nsrv++}});
  const Q=s.scram?s.scram.P0*decayFrac(s.t-s.scram.t):u.heat||0,wst=wMS(s.mv,P);
  // feedwater train: valve travel, pump and valve flow against the vessel pressure, hotwell, heater
  const n=clamp(u.n||0,0,1),ext=clamp(u.ext||0,0,1),mk=clamp(u.mk||0,-MKMAX,MKMAX);
  s.fv+=clamp(clamp(u.fv||0,0,1)-s.fv,-FRATE*dt,FRATE*dt);
  const Tfw=s.Th,hfw=hLiq(Tfw,P),ret=condensate(s,wst);
  const fw=Math.min(fwFlow(n,s.fv,P,s.Mh),Math.max(0,s.Mh/dt)),Pd=PC+Math.max(0,n*n*H0-KPU*fw*fw);
  s.Mh=Math.max(0,s.Mh+(ret.w+mk-fw)*dt);
  const Tt=heaterT(wst,ext);s.Th=Tt+(s.Th-Tt)*Math.exp(-dt/tau.h);
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
  s.f={Q,fw,Tfw,hfw,hg,wst,wsrv,nsrv,open,spr:s.spr.slice(),gen,cond:mc/dt,mv:s.mv,
    n,fv:s.fv,ext,Tt,pe:PE*wst/WR*(ext>0),Pd,ret:ret.w,Tret:ret.T,mk,Mh:s.Mh,hw:s.Mh/HWM};
};
// scram from heat q (W; default the last applied net heat, taken as core power; none if not positive);
// a second scram keeps the first; reset hands the heat back to the command
const scram=(s,q=s.f.Q)=>{if(!s.scram)s.scram={t:s.t,P0:Math.max(0,q)}};
const reset=s=>{s.scram=null};
// why the model must stop, or '' while the state is valid
const fault=s=>{
  const d=s.d;
  if(d.err)return 'range';
  if(d.level<Z.TAF)return 'uncovered';
  if(d.level>=Z.MSL)return 'flooded';
  if(d.P>21.5e6)return 'critical';
  if(s.Mh>HWM)return 'hotwell';
  return '';
};
const API=4;
export {API,V,H,A,Z,WMS,PR,PC,RATE,SRV,NV,PB,BD,TAU,H0,KPU,KLN,KV,FRATE,HWM,MKMAX,THW,TTD,WR,PE,fwFlow,fwValve,heaterT,heatBalance,condensate,subF,wMS,wSRV,decayFrac,init,step,scram,reset,fault};
