// BWR plant model: pure physics, no DOM, no automatic control. step() takes final actuator commands
// (from the operator or from bwr-control.js; the plant can't tell). An ES module. It composes the
// per-system modules, each with pure functions and its own part of the state:
//   bwr-vessel.js    reactor vessel geometry and the water–steam solve
//   bwr-steam.js     rated conditions, main steam valve, safety/relief valves
//   bwr-condenser.js turbine stub and main condenser
//   bwr-feed.js      hotwell, condensate pump, polisher, LP heater, feed pump, min-flow line, HP heater, feed valve
// Equations and constants are described in docs/bwr.md.
import {sat,clamp,hLiq} from './common.js';
import {V,H,A,Z,TAU,solve,derive,vesselInit} from './bwr-vessel.js';
import {QR,PR,TFR,WR,WMS,RATE,SRV,NV,PB,BD,subF,wMS,wSRV,srvStep} from './bwr-steam.js';
import {ETA,ETAG,expansion,turbinePower,VC,WCW,TCW,CPW,UA,PCMAX,condense,condSteady} from './bwr-condenser.js';
import {H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,KXT,fwFlow,fwValve,heater,heaterT,heatBalance,hotwellMix,
  PEL,KXL,TLPD,HC0,HF0,KCP,KPOL,KLP,KFP,ZS,HWD,NPR,PLO,TLO,WMIN,KR,RRATE,network,shutoff,heaterL,train,suction,cpFlow,
  THPI,HDP0,ZDT,KDPP,KDVL,KDVH,KDDH,KDDL,WSD,WSLD,WDD,VHS,VLS,LFL,LDU,XD0,MHS,MLS,DPD,mixT,dpFlow,tubes,dumpX,Thf} from './bwr-feed.js';
// heater drain valves (HP drain valve, LP drain pump discharge valve) travel at most DRATE per second
const DRATE=0.1;
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
// outlet (default the hotwell temperature), hw hotwell fraction full (default 0.5), ext, exl 0–1 HP and LP extraction
// valves the condenser starts steady with (default 1; exl defaults to ext), xr min-flow valve 0–1, xdh, xdl
// heater drain valves 0–1 (default the design 0.5), hs, ls HP and LP heater shell fraction full (default 0.5),
// tau overrides}
const init=o=>{
  const s={t:0,mv:o.mv||0,fv:o.fv||0,n:o.n||0,xr:o.xr||0,fpt:false,tlo:0,xdh:o.xdh??XD0,xdl:o.xdl??XD0,Mhs:(o.hs??0.5)*MHS,Mls:(o.ls??0.5)*MLS,fail:{},Mh:(o.hw??0.5)*HWM,spr:Array(NV).fill(false),scram:null,tau:{...TAU,...o.tau}};
  vesselInit(s,o);
  // condenser and hotwell start steady for the initial steam flow (turbine exhaust; the heater drains are
  // pumped forward)
  const P=s.d.P,hg=sat(s.d.T).hg,line=expansion(P,hg),w0=wMS(s.mv,P,PCD);let Tc=TCW+0.1,ht=null;
  for(let i=0;i<4;i++){const pc=sat(Tc).p;ht=train(w0,o.exl??o.ext??1,o.ext??1,w0,Tc,line);const wsH=Math.min(ht.hp.ws,w0),wsL=Math.min(ht.lp.ws,w0-wsH);
    Tc=w0>0?condSteady([[w0-wsH-wsL,line(pc)]]):TCW+0.1}
  s.cd={Tc,Ms:sat(Tc).rg*VC};s.Thw=Tc;s.Tsu=w0>0?ht.Tm:Tc;
  // heater shells saturated at their initial pressures (the hotwell temperature with no extraction)
  s.psh=ht.hp.ps||sat(Tc).p;s.psl=ht.lp.ps||sat(Tc).p;s.Ths=ht.hp.ps?Thf(ht.hp.hd):Tc;s.Tls=ht.lp.ps?Thf(ht.lp.hd):Tc;
  s.Th=o.tfw!=null?o.tfw+273.15:Tc;
  const Pc=sat(Tc).p;
  s.f={Q:0,fw:0,Tfw:s.Th,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,open:Array(NV).fill(false),spr:Array(NV).fill(false),gen:o.gen||0,cond:0,mv:s.mv,
    n:s.n,fv:s.fv,ext:0,Tt:s.Th,pe:0,ps:0,ws:0,Pd:Pc,ret:0,Tret:Tc,mk:0,Mh:s.Mh,hw:s.Mh/HWM,Thw:Tc,
    Pc,Tc,wc:0,Qc:0,Tcw:TCW,wcw:WCW,Pgen:0,wx:0,hx:0,
    exl:0,pel:0,psl:0,wsl:0,Tlp:Tc,Ps:Pc,Pcd:Pc,wp:0,wr:0,wcp:0,xr:s.xr,cp:false,fp:false,fpt:false,npsh:0,phi:1,cav:false,
    Tm:s.Tsu,wdh:0,wd:0,wduh:0,wdul:0,LH:s.Mhs/MHS,LL:s.Mls/MLS,aH:1,aL:1,xdh:s.xdh,xdl:s.xdl,hdp:false,hdvf:false};
  return s;
};
// One step. u is the final command, SI: {heat W net heat into the water; n 0–1 feed pump speed;
// fv 0–1 feed valve demand; ext 0–1 HP heater extraction valve; exl 0–1 LP heater extraction valve
// (absent: the same as ext); cp, fp condensate and feed pump run switches (absent: running); rv 0–1
// min-flow valve demand; hdv 0–1 HP heater drain valve demand, ldv 0–1 LP drain pump discharge valve demand
// (absent: the design opening 0.5), hdp heater drain pump run switch (absent: running);
// mk kg/s hotwell makeup (negative: reject);
// mv 0–1 main steam valve demand; srv array, true for each SRV opened by hand}.
// Hook for reactor protection (step 11): a trip of the feed pump is a false fp.
// After scram(s) the core heat is the decay curve and u.heat is ignored until reset(s).
// Each command is clamped to its range; the valves move toward their demands at RATE and FRATE; each
// SRV's spring opens it on pressure whatever the command. Feed flow follows from the pump, the valve and
// the vessel pressure; it enters at the heater outlet temperature. The main steam passes the turbine
// stub to the condenser; the condensate collects in the hotwell, which the feed pump draws from.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau,pc0=s.f.Pc;
  const q=sat(d.T),hg=q.hg,ug=q.ug,uf=q.uf; // sat() reuses one object: copy fields first
  const a=clamp(u.mv||0,0,1);s.mv+=clamp(a-s.mv,-RATE*dt,RATE*dt);
  // SRVs: spring on pressure (with blowdown), or opened by hand
  const {open,wsrv,nsrv}=srvStep(s.spr,u.srv||[],P);
  const Q=s.scram?s.scram.P0*decayFrac(s.t-s.scram.t):u.heat||0,wst=wMS(s.mv,P,pc0);
  // feedwater train: valve travel, pump and valve flow against the vessel pressure, hotwell, heater
  const ext=clamp(u.ext||0,0,1),exl=clamp(u.exl??u.ext??0,0,1),mk=clamp(u.mk||0,-MKMAX,MKMAX);
  // pump switches; the feed pump's low-suction trip holds it stopped until its switch is cycled
  const cp=u.cp!==false,fp=u.fp!==false;if(!fp){s.fpt=false;s.tlo=0}
  s.n+=clamp((fp&&!s.fpt?clamp(u.n||0,0,1):0)-s.n,-NRATE*dt,NRATE*dt);const n=s.n;
  s.fv+=clamp(clamp(u.fv||0,0,1)-s.fv,-FRATE*dt,FRATE*dt);
  s.xr+=clamp(clamp(u.rv||0,0,1)-s.xr,-RRATE*dt,RRATE*dt);
  // heater drains (step 6): valve travel (the HP drain valve fails shut on the event s.fail.hdv); flows on
  // the start-of-step shell pressures and levels. HP drains cascade to the LP shell, both shells dump to
  // the condenser above 80 % full; the drain pump forwards the LP drains into the condensate line.
  const hdp=u.hdp!==false;
  s.xdh=s.fail.hdv?0:s.xdh+clamp(clamp(u.hdv??XD0,0,1)-s.xdh,-DRATE*dt,DRATE*dt);
  s.xdl+=clamp(clamp(u.ldv??XD0,0,1)-s.xdl,-DRATE*dt,DRATE*dt);
  const LH=s.Mhs/MHS,LL=s.Mls/MLS,hdH=sat(s.Ths).hf,hdL0=sat(s.Tls).hf;
  let wdh=KDVH*s.xdh*Math.sqrt(Math.max(0,s.psh-s.psl)),wduh=KDDH*dumpX(LH)*Math.sqrt(Math.max(0,s.psh-pc0));
  {const k=(wdh+wduh)*dt>s.Mhs?s.Mhs/((wdh+wduh)*dt):1;wdh*=k;wduh*=k}
  const wdul0=KDDL*dumpX(LL)*Math.sqrt(Math.max(0,s.psl-pc0));
  const Tfw=s.Th,hfw=hLiq(Tfw,P);
  // the whole train at once: condensate pump, polisher, LP heater, drain pump, feed pump, min-flow line,
  // HP heater, feed valve
  const dp={on:hdp,x:s.xdl,Pin:s.psl+992*9.81*ZDT,f:clamp(LL/0.1,0,1)};
  const pv=sat(s.Tsu).p,nw=network({cp,n,x:s.fv,xr:s.xr,P,pc:pc0,Mh:s.Mh,Thw:s.Thw,pv,dp}),fw=nw.fw,Pd=nw.Pd;
  if(fp&&!s.fpt){s.tlo=nw.Ps<PLO?s.tlo+dt:0;if(s.tlo>=TLO)s.fpt=true}
  // turbine stub: extractions to the LP heater (condensate flow; the cascaded HP drains flash in its shell)
  // and the HP heater (feed flow, entering at the condensate and pumped drains mixed), the rest to the
  // condenser along the expansion line. Flooded tubes heat less.
  const aH=tubes(LH),aL=tubes(LL),line=expansion(P,hg),lp=heaterL(wst,exl,nw.wl,s.Thw,line,wdh>0?[[wdh,hdH]]:[],aL);
  const hdL=lp.ps>0?lp.hd:hdL0,Tm=mixT(nw.wl,lp.T,nw.wd,hdL),ht=heater(wst,ext,fw,Tm,line,[],aH),Tt=ht.T;
  const ws=Math.min(ht.ws,wst),wsl=Math.min(lp.ws,wst-ws);s.Tsu=Tm;
  // shell inventories; the LP dump takes only what the shell holds
  // (the LP shell's vent takes the flash steam the condensate can't condense; it comes out of its drains)
  const wv=Math.min(lp.wv,Math.max(0,s.Mls/dt+wsl+wdh-nw.wd)),wdul=Math.max(0,Math.min(wdul0,s.Mls/dt+wsl+wdh-nw.wd-wv));
  s.Mhs+=(ws-wdh-wduh)*dt;s.Mls+=(wsl+wdh-nw.wd-wdul-wv)*dt;
  if(ht.ps>0){s.psh=ht.ps;s.Ths=Thf(ht.hd)}if(lp.ps>0){s.psl=lp.ps;s.Tls=Thf(lp.hd)}
  s.Th=Tt+(s.Th-Tt)*Math.exp(-dt/tau.h);
  const wx=wst-ws-wsl,hx=wx>0?line(pc0):0,Pgen=wst>0?turbinePower(wst,hg,[{w:ws,h:ht.hx},{w:wsl,h:lp.hx}],hx):0;
  // condenser: exhaust steam, the heaters' high-level dumps and the min-flow recirculation in, condensate
  // down to the hotwell
  const hr=sat(Tm).hf,cd=condense(s.cd,[[wx,hx],[wduh,hdH],[wdul,hdL],[wv,sat(Thf(hdL)).hg],[nw.wr,hr]],dt);
  const hw=hotwellMix({M:s.Mh,T:s.Thw},[[cd.wl,sat(cd.Tc).hf],[mk,HMK],[-nw.wcp,0]],dt);
  s.Mh=hw.M;s.Thw=hw.T;
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
    n,fv:s.fv,ext,Tt,pe:ht.pe,ps:ht.ps,ws,Pd,ret:cd.wl,Tret:cd.Tc,mk,Mh:s.Mh,hw:s.Mh/HWM,Thw:s.Thw,
    Pc:cd.Pc,Tc:cd.Tc,wc:cd.wc,Qc:cd.Q,Tcw:cd.Tout,wcw:WCW,Pgen,wx,hx,
    exl,pel:lp.pe,psl:lp.ps,wsl,Tlp:lp.T,Ps:nw.Ps,Pcd:nw.Pcd,wp:nw.wp,wr:nw.wr,wcp:nw.wcp,xr:s.xr,cp,fp,fpt:s.fpt,npsh:nw.A,phi:nw.phi,cav:nw.cav,
    Tm,wdh,wd:nw.wd,wduh,wdul,wv,LH:s.Mhs/MHS,LL:s.Mls/MLS,aH,aL,xdh:s.xdh,xdl:s.xdl,hdp,hdvf:!!s.fail.hdv};
};
// scram from heat q (W; default the last applied net heat, taken as core power; none if not positive);
// a second scram keeps the first; reset hands the heat back to the command
const scram=(s,q=s.f.Q)=>{if(!s.scram)s.scram={t:s.t,P0:Math.max(0,q)}};
const reset=s=>{s.scram=null};
// equipment failures (events): fail(s, 'hdv', true) fails the HP heater drain valve shut (its actuator
// lost: it closes at once and ignores its demand); false repairs it
const fail=(s,k,on=true)=>{s.fail[k]=!!on};
// why the model must stop, or '' while the state is valid
const fault=s=>{
  const d=s.d;
  if(d.err)return 'range';
  if(d.level<Z.TAF)return 'uncovered';
  if(d.level>=Z.MSL)return 'flooded';
  if(d.P>21.5e6)return 'critical';
  if(s.Mh>HWM)return 'hotwell';
  if(s.f.Pc>PCMAX)return 'condenser';
  return '';
};
// the condensate pump suction pressure the page reads for the shutoff-head message
const PC=PCD;
const API=6;
export {API,V,H,A,Z,QR,PR,TFR,WMS,RATE,SRV,NV,PB,BD,TAU,H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PC,PCD,TTD,WR,PE,KXT,
  PEL,KXL,TLPD,HC0,HF0,KCP,KPOL,KLP,KFP,ZS,HWD,NPR,PLO,TLO,WMIN,KR,RRATE,network,shutoff,heaterL,train,suction,cpFlow,
  THPI,HDP0,KDPP,KDVL,KDVH,KDDH,KDDL,WSD,WSLD,WDD,VHS,VLS,LFL,LDU,XD0,MHS,MLS,DRATE,DPD,tubes,dumpX,dpFlow,fail,
  ETA,ETAG,VC,WCW,TCW,CPW,UA,PCMAX,expansion,turbinePower,condense,condSteady,
  fwFlow,fwValve,heater,heaterT,heatBalance,subF,wMS,wSRV,decayFrac,init,step,scram,reset,fault};
