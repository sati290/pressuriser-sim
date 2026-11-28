// BWR plant model: pure physics, no DOM, no automatic control. step() takes final actuator commands
// (from the operator or from bwr-control.js; the plant can't tell). An ES module. It composes the
// per-system modules, each with pure functions and its own part of the state:
//   bwr-vessel.js    reactor vessel geometry and the water–steam solve
//   bwr-steam.js     rated conditions, main steam valve, safety/relief valves
//   bwr-condenser.js turbine stub and main condenser
//   bwr-feed.js      hotwell, feed pump, feedwater heater, feed valve
//   bwr-containment.js suppression pool
// Equations and constants are described in docs/bwr.md.
import {sat,clamp,hLiq} from './common.js';
import {V,H,A,Z,TAU,solve,derive,vesselInit} from './bwr-vessel.js';
import {CST,UAS,UAL,TDW,steelStep,INCH,IZ,LV,toIn,fromIn,TAUF} from './bwr-vessel.js';
import {MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolStep,poolLevel} from './bwr-containment.js';
import {QR,PR,TFR,WR,WMS,RATE,SRV,NV,PB,BD,subF,wMS,wSRV,srvStep} from './bwr-steam.js';
import {ETA,ETAG,expansion,turbinePower,VC,WCW,TCW,CPW,UA,PCMAX,condense,condSteady} from './bwr-condenser.js';
import {H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,KXT,fwFlow,fwValve,heater,heaterT,heatBalance,hotwellMix} from './bwr-feed.js';
// Scram: the rods go in and core heat falls to the fission tail plus decay heat, a fraction of the
// power at the scram P0 after t seconds. Fission (93.5 % of steady power): prompt drop as the rods
// insert, then the delayed-neutron tail (a 10 s and an 80 s group). Decay heat 6.5 % at the scram,
// falling as (1 + t)^−0.2 (Way–Wigner form, long operation at P0 assumed). Rough: about 7 % at 10 s,
// 3.3 % at 1 min, 1.3 % at 1 h, 0.6 % at a day.
const decayFrac=t=>{t=Math.max(0,t);return 0.935*(0.93*Math.exp(-t/2)+0.06*Math.exp(-t/10)+0.01*Math.exp(-t/80))+0.065*(1+t)**-0.2};
// hotwell makeup water enthalpy
const HMK=sat(TMK).hf;
// o: {P Pa, L m two-phase level, Tl °C water temperature (default saturated), gen kg/s steaming the
// voids start in equilibrium with, mv valve position 0–1, fv feed valve position 0–1, n pump speed 0–1, tfw °C heater
// outlet (default the hotwell temperature), hw hotwell fraction full (default 0.5), ext 0–1 extraction valve the
// condenser starts steady with (default 1), ts °C vessel steel (default the water temperature), tpool °C
// suppression pool (default 32), tau overrides}
const init=o=>{
  const s={t:0,mv:o.mv||0,fv:o.fv||0,n:o.n||0,Mh:(o.hw??0.5)*HWM,spr:Array(NV).fill(false),scram:null,tau:{...TAU,...o.tau}};
  vesselInit(s,o);
  s.Ts=o.ts!=null?o.ts+273.15:(UAS*s.d.Tl+UAL*TDW)/(UAS+UAL);s.Mbf=0;s.pool={M:MPOOL,T:o.tpool!=null?o.tpool+273.15:TP0};
  // condenser and hotwell start steady for the initial steam flow (turbine exhaust plus heater drains)
  const P=s.d.P,hg=sat(s.d.T).hg,line=expansion(P,hg),w0=wMS(s.mv,P,PCD);let Tc=TCW+0.1,ht={ws:0,hd:0};
  for(let i=0;i<4;i++){const pc=sat(Tc).p;ht=heater(w0,o.ext??1,w0,Tc,line);const ws=Math.min(ht.ws,w0);
    Tc=w0>0?condSteady([[w0-ws,line(pc)],[ws,ht.hd]]):TCW+0.1}
  s.cd={Tc,Ms:sat(Tc).rg*VC};s.Thw=Tc;
  s.Th=o.tfw!=null?o.tfw+273.15:Tc;
  const Pc=sat(Tc).p;
  s.f={Q:0,fw:0,Tfw:s.Th,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,open:Array(NV).fill(false),spr:Array(NV).fill(false),gen:o.gen||0,cond:0,mv:s.mv,
    n:s.n,fv:s.fv,ext:0,Tt:s.Th,pe:0,ps:0,ws:0,Pd:Pc,ret:0,Tret:Tc,mk:0,Mh:s.Mh,hw:s.Mh/HWM,Thw:Tc,
    Pc,Tc,wc:0,Qc:0,Tcw:TCW,wcw:WCW,Pgen:0,wx:0,hx:0,Ts:s.Ts,Qs:0,Ql:0,Tp:s.pool.T,Mp:s.pool.M,Lp:0,Qpc:0,pc:false,lin:toIn(s.d.level)};
  return s;
};
// One step. u is the final command, SI: {heat W net heat into the water; n 0–1 feed pump speed;
// fv 0–1 feed valve demand; ext 0–1 heater extraction valve; mk kg/s hotwell makeup (negative: reject);
// mv 0–1 main steam valve demand; srv array, true for each SRV opened by hand; pc true: pool cooling on}.
// After scram(s) the core heat is the decay curve and u.heat is ignored until reset(s).
// Each command is clamped to its range; the valves move toward their demands at RATE and FRATE; each
// SRV's spring opens it on pressure whatever the command. Feed flow follows from the pump, the valve and
// the vessel pressure; it enters at the heater outlet temperature. The main steam passes the turbine
// stub to the condenser; the condensate collects in the hotwell, which the feed pump draws from.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau,pc0=s.f.Pc;
  const q=sat(d.T),hg=q.hg,hf0=q.hf,ug=q.ug,uf=q.uf; // sat() reuses one object: copy fields first
  const a=clamp(u.mv||0,0,1);s.mv+=clamp(a-s.mv,-RATE*dt,RATE*dt);
  // SRVs: spring on pressure (with blowdown), or opened by hand
  const {open,wsrv,nsrv}=srvStep(s.spr,u.srv||[],P);
  const Q=s.scram?s.scram.P0*decayFrac(s.t-s.scram.t):u.heat||0,wst=wMS(s.mv,P,pc0);
  // feedwater train: valve travel, pump and valve flow against the vessel pressure, hotwell, heater
  const ext=clamp(u.ext||0,0,1),mk=clamp(u.mk||0,-MKMAX,MKMAX);
  s.n+=clamp(clamp(u.n||0,0,1)-s.n,-NRATE*dt,NRATE*dt);const n=s.n;
  s.fv+=clamp(clamp(u.fv||0,0,1)-s.fv,-FRATE*dt,FRATE*dt);
  const Tfw=s.Th,hfw=hLiq(Tfw,P);
  const fw=Math.min(fwFlow(n,s.fv,P,s.Mh,pc0),Math.max(0,s.Mh/dt)),Pd=pc0+Math.max(0,n*n*H0-KPU*fw*fw);
  // turbine stub: extraction to the heater, the rest to the condenser along the expansion line
  const line=expansion(P,hg),ht=heater(wst,ext,fw,s.Thw,line),Tt=ht.T,ws=Math.min(ht.ws,wst);
  s.Th=Tt+(s.Th-Tt)*Math.exp(-dt/tau.h);
  const wx=wst-ws,hx=wx>0?line(pc0):0,Pgen=wst>0?turbinePower(wst,hg,[{w:ws,h:ht.hx}],hx):0;
  // condenser: exhaust steam and heater drains in, condensate down to the hotwell
  const cd=condense(s.cd,[[wx,hx],[ws,ht.hd]],dt);
  const hw=hotwellMix({M:s.Mh,T:s.Thw},[[cd.wl,sat(cd.Tc).hf],[mk,HMK],[-fw,0]],dt);
  s.Mh=hw.M;s.Thw=hw.T;
  // flows at the start-of-step pressure: steam leaves at h″, feedwater joins the water, heat into the water
  const mo=Math.min((wst+wsrv)*dt,0.5*s.M);
  // vessel steel: heat from (or to) the water, losses to the drywell
  const st=steelStep(s.Ts,d.Tl,dt);s.Ts=st.Ts;
  // suppression pool: the SRV steam condenses in it
  const pl=poolStep(s.pool,wsrv*Math.min(1,mo/((wst+wsrv)*dt||1)),hg,!!u.pc,dt);s.pool={M:pl.M,T:pl.T};
  let M=s.M+fw*dt-mo,U=s.U+fw*dt*hfw-mo*hg+(Q+st.Qw)*dt,Mv=Math.max(0,s.Mv-mo);
  // subcooled water condenses steam, relaxing towards saturation at the start-of-step steam temperature
  let mc=0;{const Ml=M-Mv,ul=(U-Mv*ug)/Ml;
    if(Mv>0&&ul<uf)mc=Math.min(Ml*(uf-ul)/(ug-uf)*(1-Math.exp(-dt/tau.c)),0.9*Mv);Mv-=mc}
  const e=solve({M,U,Mv,T0:d.T});
  if(!e){s.d={err:'range',P:0};s.t+=dt;return}
  // net steam generation (flashing/boiling minus condensation) feeds the voids, which rise out with τb
  // The part made by the heat (boiling in the core) clears with τb; what flashes beyond that (bulk
  // flashing on a depressurisation, all through the water) rises more slowly, with TAUF.
  const gen=(e.Mv-Mv-mc)/dt,hfg=hg-hf0,gq=Math.min(Math.max(0,gen),Math.max(0,Q+st.Qw)/hfg),gf=Math.max(0,gen)-gq;
  const g=gq*tau.b,gF=gf*TAUF;
  s.Mb=Math.min(g+(s.Mb-g)*Math.exp(-dt/tau.b),0.9*e.Mv);
  s.Mbf=Math.min(gF+(s.Mbf-gF)*Math.exp(-dt/TAUF),Math.max(0,0.9*e.Mv-s.Mb));
  Object.assign(s,{M,U,Mv:e.Mv});s.t+=dt;
  s.d=derive(s,e);
  s.f={Q,fw,Tfw,hfw,hg,wst,wsrv,nsrv,open,spr:s.spr.slice(),gen,cond:mc/dt,mv:s.mv,
    n,fv:s.fv,ext,Tt,pe:ht.pe,ps:ht.ps,ws,Pd,ret:cd.wl,Tret:cd.Tc,mk,Mh:s.Mh,hw:s.Mh/HWM,Thw:s.Thw,
    Pc:cd.Pc,Tc:cd.Tc,wc:cd.wc,Qc:cd.Q,Tcw:cd.Tout,wcw:WCW,Pgen,wx,hx,
    Ts:s.Ts,Qs:st.Qw,Ql:st.Ql,Tp:pl.T,Mp:pl.M,Lp:poolLevel(pl.M),Qpc:pl.Qc,pc:!!u.pc,lin:toIn(s.d.level)};
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
  if(s.f.Pc>PCMAX)return 'condenser';
  if(s.pool.T>TPMAX)return 'pool';
  return '';
};
// the condensate pump suction pressure the page reads for the shutoff-head message
const PC=PCD;
const API=6;
export {CST,UAS,UAL,TDW,INCH,IZ,LV,toIn,fromIn,TAUF,MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolLevel};
export {API,V,H,A,Z,QR,PR,TFR,WMS,RATE,SRV,NV,PB,BD,TAU,H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PC,PCD,TTD,WR,PE,KXT,
  ETA,ETAG,VC,WCW,TCW,CPW,UA,PCMAX,expansion,turbinePower,condense,condSteady,
  fwFlow,fwValve,heater,heaterT,heatBalance,subF,wMS,wSRV,decayFrac,init,step,scram,reset,fault};
