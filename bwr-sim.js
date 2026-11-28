// BWR plant model: pure physics, no DOM, no automatic control. step() takes final actuator commands
// (from the operator or from bwr-control.js; the plant can't tell). An ES module. It composes the
// per-system modules, each with pure functions and its own part of the state:
//   bwr-vessel.js    reactor vessel geometry and the water–steam solve
//   bwr-steam.js     rated conditions, main steam valve, safety/relief valves
//   bwr-condenser.js turbine stub and main condenser
//   bwr-feed.js      hotwell, feed pump, feedwater heater, feed valve
//   bwr-core.js      point kinetics, fuel, decay heat, control rod lever, core voids
//   bwr-recirc.js    recirculation pump on its MG set, jet pump, core flow, natural circulation
// Equations and constants are described in docs/bwr.md.
import {sat,Tsat,clamp,hLiq,cpL,liqRho} from './common.js';
import {V,H,A,Z,TAU,VDC,solve,derive,dcMass,vesselInit} from './bwr-vessel.js';
import {QR,PR,TFR,WR,WMS,RATE,SRV,NV,PB,BD,subF,wMS,wSRV,srvStep} from './bwr-steam.js';
import {ETA,ETAG,expansion,turbinePower,VC,WCW,TCW,CPW,UA,PCMAX,condense,condSteady} from './bwr-condenser.js';
import {QC,WCR,BETA,LAMBDA,SRC,AV,AD,AM,MCF,RF,X0,XR,WT,RODRATE,TSCRAM,DSUM,DL,DF,VCH,AF,LC,WMIN,voids,voidVol,rodRho,rodFor,coreInit,coreStep} from './bwr-core.js';
import {WDR,HPR,HS,KPP,MGRATE,NMIN,TCD,mgStep,loopStep,recircInit} from './bwr-recirc.js';
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
// condenser starts steady with (default 1), tau overrides}
const init=o=>{
  const s={t:0,mv:o.mv||0,fv:o.fv||0,n:o.n||0,Mh:(o.hw??0.5)*HWM,spr:Array(NV).fill(false),scram:null,tau:{...TAU,...o.tau}};
  // core and recirculation: steady at heat o.q (default: the heat that makes the steam o.gen from feed
  // at o.tfw), pump breaker o.rp (default closed) at MG set speed o.mg (default 1), critical with the
  // rods at o.rod (0–1; default the critical position init solves for). Core flow and voids are solved
  // together; the voids set the void steam Mb inside the level.
  // The downcomer starts steady too: feed mixed with the separator return, subcooled by fw·(u′ − u_fw)/W.
  let dh0;
  {const P=o.P,T=Tsat(P),q=sat(T),uf=q.uf,hg=q.hg,rf=q.rf,rg=q.rg,Tfw=o.tfw!=null?o.tfw+273.15:T,hfw=hLiq(Tfw,P),ufw=hfw-P/liqRho(Tfw,P),fw=o.gen||0,Q=o.q??fw*(hg-hfw),cp=cpL(T);
   let W=WCR,v,dh,rc;const rp=o.rp??true,mg=o.mg??1;
   for(let i=0;i<8;i++){dh=Math.max(0,fw*(uf-ufw))/Math.max(W,WMIN,fw);v=voids(Q,W,dh,P);
     rc=recircInit(rp,mg,{rd:liqRho(T-dh/cp,P),rc:rf,rg,ac:v.ac,ae:v.ae,xe:v.xe});W=rc.Wc}
   s.rc=rc;s.core=coreInit(Q,T,T-dh/cp,v,o.rod);s.core.W=W;s.core.dh=dh;s.Mb=rg*voidVol(s.core);s.Mfl=0;dh0=dh}
  vesselInit(s,{...o,dh:dh0});
  // condenser and hotwell start steady for the initial steam flow (turbine exhaust plus heater drains)
  const P=s.d.P,hg=sat(s.d.T).hg,line=expansion(P,hg),w0=wMS(s.mv,P,PCD);let Tc=TCW+0.1,ht={ws:0,hd:0};
  for(let i=0;i<4;i++){const pc=sat(Tc).p;ht=heater(w0,o.ext??1,w0,Tc,line);const ws=Math.min(ht.ws,w0);
    Tc=w0>0?condSteady([[w0-ws,line(pc)],[ws,ht.hd]]):TCW+0.1}
  s.cd={Tc,Ms:sat(Tc).rg*VC};s.Thw=Tc;
  s.Th=o.tfw!=null?o.tfw+273.15:Tc;
  const Pc=sat(Tc).p;
  s.f={Q:0,fw:0,Tfw:s.Th,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,open:Array(NV).fill(false),spr:Array(NV).fill(false),gen:o.gen||0,cond:0,mv:s.mv,
    n:s.n,fv:s.fv,ext:0,Tt:s.Th,pe:0,ps:0,ws:0,Pd:Pc,ret:0,Tret:Tc,mk:0,Mh:s.Mh,hw:s.Mh/HWM,Thw:Tc,
    Pc,Tc,wc:0,Qc:0,Tcw:TCW,wcw:WCW,Pgen:0,wx:0,hx:0};
  coreF(s.f,s.core,s.rc);
  return s;
};
// core readings into s.f: Pth core thermal power (fission + decay, W), nf fission power, rho reactivity,
// per period (s), Tf fuel temperature, ac core-average void, ae exit void, xe exit quality, Wc core
// flow, dhin inlet subcooling (J/kg), rod lever position 0–1, pc prompt critical
// recirculation: rp pump breaker closed, ng pump (MG set) speed 0–1, mg speed demand, Wd drive flow
const coreF=(f,k,r)=>Object.assign(f,{Pth:k.q,nf:k.n,rho:k.rho,per:k.per,Tf:k.Tf,ac:k.ac,ae:k.ae,xe:k.xe,Wc:k.W,dhin:k.dh,rod:k.x,pc:k.pc,
  rp:r.on,ng:r.ng,mg:r.dm,Wd:r.Wd});
// One step. u is the final command, SI: {rod 0–1 control rod lever demand (manual only; undefined: the
// rods stay where they are); heat W fixed net heat into the water (a test hook: if it is a number the
// core heat is that value and the kinetics stand still); n 0–1 feed pump speed;
// fv 0–1 feed valve demand; ext 0–1 heater extraction valve; mk kg/s hotwell makeup (negative: reject);
// mv 0–1 main steam valve demand; srv array, true for each SRV opened by hand}.
// After scram(s) the rods drive fully in over TSCRAM and the kinetics follow; with the fixed-heat hook
// the heat is the decay curve and u.heat is ignored until reset(s).
// Each command is clamped to its range; the valves move toward their demands at RATE and FRATE; each
// SRV's spring opens it on pressure whatever the command. Feed flow follows from the pump, the valve and
// the vessel pressure; it enters at the heater outlet temperature. The main steam passes the turbine
// stub to the condenser; the condensate collects in the hotwell, which the feed pump draws from.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau,pc0=s.f.Pc;
  const q=sat(d.T),hg=q.hg,ug=q.ug,uf=q.uf,hf=q.hf,rf=q.rf,rg=q.rg; // sat() reuses one object: copy fields first
  const a=clamp(u.mv||0,0,1);s.mv+=clamp(a-s.mv,-RATE*dt,RATE*dt);
  // SRVs: spring on pressure (with blowdown), or opened by hand
  const {open,wsrv,nsrv}=srvStep(s.spr,u.srv||[],P);
  const wst=wMS(s.mv,P,pc0);
  // feedwater train: valve travel, pump and valve flow against the vessel pressure, hotwell, heater
  const ext=clamp(u.ext||0,0,1),mk=clamp(u.mk||0,-MKMAX,MKMAX);
  s.n+=clamp(clamp(u.n||0,0,1)-s.n,-NRATE*dt,NRATE*dt);const n=s.n;
  s.fv+=clamp(clamp(u.fv||0,0,1)-s.fv,-FRATE*dt,FRATE*dt);
  const Tfw=s.Th,hfw=hLiq(Tfw,P);
  const fw=Math.min(fwFlow(n,s.fv,P,s.Mh,pc0),Math.max(0,s.Mh/dt)),Pd=pc0+Math.max(0,n*n*H0-KPU*fw*fw);
  // core: kinetics, fuel, voids and the recirculation loop in substeps at the start-of-step pressure.
  // The core inlet is downcomer water: inlet subcooling u′ − u_dc, at the downcomer temperature.
  // Recirculation: pump breaker u.rp (true closed; undefined keeps it), MG set speed demand u.mg 0–1
  // (undefined keeps it).
  const k=s.core,rc=s.rc,fix=Number.isFinite(u.heat),W0=k.W;
  if(u.rp!=null)rc.on=!!u.rp;if(u.mg!=null)rc.dm=clamp(u.mg,0,1);
  k.dh=Math.max(0,uf-Math.min(s.udc,uf));
  const Tin=d.Tdc,lp={rd:liqRho(Tin,P),rc:rf,rg,ac:0,ae:0,xe:0};
  const hq=coreStep(k,{P,Tc:d.T,dh:k.dh,Tin,rod:u.rod,scram:!!s.scram,heat:fix?(s.scram?s.scram.P0*decayFrac(s.t-s.scram.t):u.heat):undefined},dt,
    (h,c)=>{mgStep(rc,h);lp.ac=c.ac;lp.ae=c.ae;lp.xe=c.xe;loopStep(rc,lp,h);return rc.Wc});
  const Q=hq;
  // downcomer: the feed and the separator return (core-region water) mix into it and the core flow draws
  // it off (exact for the step; when the core flow is below the feed flow the surplus feed spills over
  // into the core region)
  const Md=dcMass(d.Tdc,P,d.Ml);
  {const W=Math.max(0,(W0+k.W)/2),Wi=Math.max(W,fw),ufw=hfw-P/liqRho(Tfw,P),ur=sat(d.Tl).uf;
   if(Wi>0){const ut=(fw*ufw+Math.max(0,W-fw)*ur)/Wi;s.udc=ut+(s.udc-ut)*Math.exp(-Wi*dt/Md)}}
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
  let M=s.M+fw*dt-mo,U=s.U+fw*dt*hfw-mo*hg+Q*dt,Mv=Math.max(0,s.Mv-mo);
  // subcooled core-region water condenses steam, relaxing towards saturation at the start-of-step steam
  // temperature (the downcomer's subcooling is carried into the core first)
  let mc=0;{const Ml=M-Mv,ue=Math.min(s.udc,uf),D=(Ml-Md)*uf-(U-Mv*ug-Md*ue);
    if(Mv>0&&D>0)mc=Math.min(D/(ug-uf)*(1-Math.exp(-dt/tau.c)),0.9*Mv);Mv-=mc}
  const e=solve({M,U,Mv,T0:d.T,Mdc:Md,udc:s.udc});
  if(!e){s.d={err:'range',P:0};s.t+=dt;return}
  s.udc=e.ue; // a downcomer above saturation has flashed
  // voids inside the level: the core and chimney voids from the core thermal-hydraulics, plus the
  // steam that flashing makes beyond what the core boils (a depressurisation flashes water throughout
  // the vessel), which rises out with τb
  const gen=(e.Mv-Mv-mc)/dt,g=Math.max(0,gen-k.W*k.xe)*tau.b;
  s.Mfl=g+(s.Mfl-g)*Math.exp(-dt/tau.b);
  s.Mb=Math.min(sat(e.T).rg*voidVol(k)+s.Mfl,0.9*e.Mv);
  Object.assign(s,{M,U,Mv:e.Mv});s.t+=dt;
  s.d=derive(s,e);
  s.f={Q,fw,Tfw,hfw,hg,wst,wsrv,nsrv,open,spr:s.spr.slice(),gen,cond:mc/dt,mv:s.mv,
    n,fv:s.fv,ext,Tt,pe:ht.pe,ps:ht.ps,ws,Pd,ret:cd.wl,Tret:cd.Tc,mk,Mh:s.Mh,hw:s.Mh/HWM,Thw:s.Thw,
    Pc:cd.Pc,Tc:cd.Tc,wc:cd.wc,Qc:cd.Q,Tcw:cd.Tout,wcw:WCW,Pgen,wx,hx};
  coreF(s.f,k,rc);
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
  if(s.core.pc)return 'prompt';
  return '';
};
// the condensate pump suction pressure the page reads for the shutoff-head message
const PC=PCD;
const API=6;
export {API,V,H,A,Z,QR,PR,TFR,WMS,RATE,SRV,NV,PB,BD,TAU,H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PC,PCD,TTD,WR,PE,KXT,
  ETA,ETAG,VC,WCW,TCW,CPW,UA,PCMAX,expansion,turbinePower,condense,condSteady,
  fwFlow,fwValve,heater,heaterT,heatBalance,subF,wMS,wSRV,decayFrac,init,step,scram,reset,fault,
  QC,WCR,BETA,LAMBDA,SRC,AV,AD,AM,MCF,RF,X0,XR,WT,RODRATE,TSCRAM,DSUM,DL,DF,VCH,AF,LC,VDC,voids,voidVol,rodRho,rodFor,WDR,HPR,HS,KPP,MGRATE,NMIN,TCD};
