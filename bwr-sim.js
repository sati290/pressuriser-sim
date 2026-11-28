// BWR plant model: pure physics, no DOM, no automatic control. step() takes final actuator commands
// (from the operator or from bwr-control.js; the plant can't tell). An ES module. It composes the
// per-system modules, each with pure functions and its own part of the state:
//   bwr-vessel.js    reactor vessel geometry and the water–steam solve
//   bwr-steam.js     rated conditions, main steam line and turbine/bypass valves, safety/relief valves
//   bwr-condenser.js turbine stub, main condenser, circulating water, condenser air removal
//   bwr-feed.js      hotwell, feed pump, feedwater heater, feed valve
//   bwr-containment.js suppression pool
//   bwr-rps.js       reactor protection: scram, isolation, turbine and feed pump trips (latched, logged)
//   bwr-core.js      point kinetics, fuel, decay heat, control rod lever, core voids
//   bwr-recirc.js    recirculation pump on its MG set, jet pump, core flow, natural circulation
// Equations and constants are described in docs/bwr.md.
import {sat,Tsat,clamp,hLiq,cpL,liqRho} from './common.js';
import {V,H,A,Z,TAU,VDC,solve,derive,dcMass,vesselInit} from './bwr-vessel.js';
import {CST,UAS,UAL,TDW,steelStep,INCH,IZ,LV,toIn,fromIn,TAUF,RWMAX} from './bwr-vessel.js';
import {SP,MODES,TRIPS,GROUP,inHgVac,rpsInit,rpsCheck,rpsReset,rpsMode,bpBlock,rpsActive,groups} from './bwr-rps.js';
import {MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolStep,poolLevel} from './bwr-containment.js';
import {QR,PR,TFR,WR,SRV,NV,PB,BD,subF,wSRV,srvStep} from './bwr-steam.js';
import {DPL,PL0,VWO,KT,BPF,KB,fcv,fcvInv,MSRATE,SVCLOSE,SVOPEN,CVRATE,BPRATE,steamLine,cvPos,valveStep} from './bwr-steam.js';
import {ETA,ETAG,expansion,turbinePower,VC,WCW,TCW,CPW,UA,PCMAX,condense,condSteady} from './bwr-condenser.js';
import {UA0,RA,PATM,WLK,WRAD,VSJ,WSJ,VVP,WVB,VBRATE,PA0,PSB0,PSB1,PSJ0,PSJ1,PVP0,PVP1,pAir,yAir,blanket,sjSteam,sjCap,vpCap,airFlows,cwStep,cwTarget,TAUCW} from './bwr-condenser.js';
import {QC,WCR,BETA,LAMBDA,SRC,AV,AD,AM,MCF,RF,X0,XR,WT,RODRATE,TSCRAM,DSUM,DL,DF,VCH,AF,LC,WMIN,voids,voidVol,rodRho,rodFor,coreInit,coreStep} from './bwr-core.js';
import {WDR,HPR,HS,KPP,MGRATE,NMIN,TCD,mgStep,loopStep,recircInit} from './bwr-recirc.js';
import {WTR,H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,KXT,fwFlow,fwValve,heater,heaterT,heatBalance,hotwellMix} from './bwr-feed.js';
// Scram: the rods go in and core heat falls to the fission tail plus decay heat, a fraction of the
// power at the scram P0 after t seconds. Fission (93.5 % of steady power): prompt drop as the rods
// insert, then the delayed-neutron tail (a 10 s and an 80 s group). Decay heat 6.5 % at the scram,
// falling as (1 + t)^−0.2 (Way–Wigner form, long operation at P0 assumed). Rough: about 7 % at 10 s,
// 3.3 % at 1 min, 1.3 % at 1 h, 0.6 % at a day.
const decayFrac=t=>{t=Math.max(0,t);return 0.935*(0.93*Math.exp(-t/2)+0.06*Math.exp(-t/10)+0.01*Math.exp(-t/80))+0.065*(1+t)**-0.2};
// hotwell makeup water enthalpy
const HMK=sat(TMK).hf;
// SJAE motive steam supply pressure: the main steam header
const motiveP=s=>s.Pl??s.d.P;
// o: {P Pa, L m two-phase level, Tl °C water temperature (default saturated), gen kg/s steaming the
// voids start in equilibrium with, cv control valve and bp bypass valve positions 0–1 (default shut), ms MSIV
// and sv stop valve positions 0–1 (default open), fv feed valve position 0–1, n pump speed 0–1, tfw °C heater
// outlet (default the hotwell temperature), hw hotwell fraction full (default 0.5), ext 0–1 extraction valve the
// condenser starts steady with (default 1), vac false for a condenser open to atmosphere (full of air at
// 0.1 MPa; default: under vacuum, air at its steady value with the SJAE in service), ts °C vessel steel (default
// steady between water and drywell), tpool °C suppression pool (default 32), mode switch 'shutdown' |
// 'startup' | 'run' (default run at 6 MPa and above, else startup), rps false to run without protection
// (tests of the bare physics), tau overrides}
const init=o=>{
  const s={t:0,wcw:WCW,vb:0,ms:[o.ms??1,o.ms??1],sv:o.sv??1,cv:o.cv||0,bp:o.bp||0,fv:o.fv||0,n:o.n||0,Mh:(o.hw??0.5)*HWM,spr:Array(NV).fill(false),scram:null,tau:{...TAU,...o.tau}};
  // core and recirculation: steady at heat o.q (default: the heat that makes the steam o.gen from feed
  // at o.tfw), pump breaker o.rp (default closed) at MG set speed o.mg (default 1), critical with the
  // rods at o.rod (0–1; default the critical position init solves for). Core flow and voids are solved
  // together; the voids set the void steam Mb inside the level.
  // The downcomer starts steady too: feed mixed with the separator return, subcooled by fw·(u′ − u_fw)/W.
  let dh0;
  {const P=o.P,T=Tsat(P),q=sat(T),uf=q.uf,hg=q.hg,rf=q.rf,rg=q.rg,Tfw=o.tfw!=null?o.tfw+273.15:T,hfw=hLiq(Tfw,P),ufw=hfw-P/liqRho(Tfw,P),fw=o.gen||0,Q=o.q??(fw>0?fw*(hg-hfw)+UAL*UAS/(UAS+UAL)*(T-TDW):0),cp=cpL(T);
   let W=WCR,v,dh,rc;const rp=o.rp??true,mg=o.mg??1;
   for(let i=0;i<8;i++){dh=Math.max(0,fw*(uf-ufw))/Math.max(W,WMIN,fw);v=voids(Q,W,dh,P);
     rc=recircInit(rp,mg,{rd:liqRho(T-dh/cp,P),rc:rf,rg,ac:v.ac,ae:v.ae,xe:v.xe});W=rc.Wc}
   s.rc=rc;s.core=coreInit(Q,T,T-dh/cp,v,o.rod);s.core.W=W;s.core.dh=dh;s.Mb=rg*voidVol(s.core);s.Mfl=0;dh0=dh}
  vesselInit(s,{...o,dh:dh0});
  s.rps=rpsInit(o.mode??(o.P>=6e6?'run':'startup'),o.rps===false);
  s.Ts=o.ts!=null?o.ts+273.15:(UAS*s.d.Tl+UAL*TDW)/(UAS+UAL);s.pool={M:MPOOL,T:o.tpool!=null?o.tpool+273.15:TP0};
  // condenser and hotwell start steady for the initial steam flow (turbine exhaust plus heater drains)
  const P=s.d.P,hg=sat(s.d.T).hg;let Tc=TCW+0.1,ht={ws:0,hd:0},sl=steamLine(P,PCD,s);
  // air: steady against the SJAE in service (in-leakage plus radiolytic gas, taken ∝ the initial steam
  // flow), or (vac false) a shell open to atmosphere, full of air at the circulating water temperature
  const vac=o.vac!==false,wsj0=vac?Math.min(sjSteam(P,0),sl.w):0;
  const MaV=T=>{const c=sjCap(P,sat(T).p+PA0);return c>0?(WLK+WRAD*Math.min(1,sl.w/WR))*VC/(VSJ*c):0};
  for(let i=0;i<4;i++){const pa=pAir(MaV(Tc),Tc),pc=sat(Tc).p+pa;sl=steamLine(P,pc,s);const line=expansion(sl.Pl,hg),w0=Math.max(0,sl.wt-wsj0);
    ht=heater(w0,o.ext??1,sl.w,Tc,line);const ws=Math.min(ht.ws,w0);
    Tc=sl.w>0?condSteady([[w0-ws,line(pc)],[ws,ht.hd],[sl.wb,hg],[wsj0,hg]],undefined,pa):TCW+0.1}
  if(!vac)Tc=TCW;
  const Ma=vac?MaV(Tc):(PATM-sat(Tc).p)*VC/(RA*Tc);
  s.Pl=sl.Pl;
  s.cd={Tc,Ms:sat(Tc).rg*VC,Ma};s.Thw=Tc;
  s.Th=o.tfw!=null?o.tfw+273.15:Tc;
  const pa0=pAir(Ma,Tc),Pc=sat(Tc).p+pa0;
  s.f={Q:0,fw:0,Tfw:s.Th,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,open:Array(NV).fill(false),spr:Array(NV).fill(false),gen:o.gen||0,cond:0,
    ms:s.ms.slice(),sv:s.sv,cv:s.cv,bp:s.bp,Pl:s.Pl,wt:sl.wt,wb:sl.wb,
    n:s.n,fv:s.fv,ext:0,Tt:s.Th,pe:0,ps:0,ws:0,Pd:Pc,ret:0,Tret:Tc,mk:0,Mh:s.Mh,hw:s.Mh/HWM,Thw:Tc,
    Pc,Tc,wc:0,Qc:0,Tcw:TCW,wcw:WCW,Pgen:0,wx:0,hx:0,
    Tcwi:TCW,cwp:true,cwv:1,pa:pa0,ya:pa0/Pc,Ma,wsj:0,sj:vac,vp:false,vb:0,sjc:0,vpc:0,wlk:0,wrad:0,wvb:0,wain:0,waout:0,Vsj:0,Vvp:0,
    Ts:s.Ts,Qs:0,Ql:0,Tp:s.pool.T,Mp:s.pool.M,Lp:0,Qpc:0,pc:false,lin:toIn(s.d.level),wrw:0,trip:groups(s.rps)};
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
// ms1, ms2 inboard and outboard MSIV and sv turbine stop valve switches (true open, false shut; left
// undefined, the valve stays where it is); cv 0–1 turbine control valve and bp 0–1 bypass valve demands;
// trip {turbine, msiv} protection hook: shuts the stop valve / the MSIVs whatever the switches;
// srv array, true for each SRV opened by hand; circulating water and
// condenser air removal: cwp CW pump running, cwv 0–1 CW valve, tcw K CW inlet temperature, sj SJAE motive
// steam valve open, vp mechanical vacuum pump running, vb vacuum breaker open (booleans)}. An absent cwp,
// cwv, tcw or sj means the normal running line-up (pump on, valve open, 20 °C, SJAE in service); absent vp
// and vb mean off and shut. These are also the hooks for protection: a CW pump trip is cwp false.
// pc true: suppression pool cooling on. rw 0–1 reactor water cleanup reject to the condenser.
// rp recirculation pump breaker (true closed; undefined keeps it), mg 0–1 MG set speed demand.
// After scram(s) the rods drive fully in over TSCRAM and the kinetics follow; with the fixed-heat hook
// the heat is the decay curve and u.heat is ignored until reset(s).
// Each command is clamped to its range; the valves move toward their demands at their stroke rates; each
// SRV's spring opens it on pressure whatever the command. Feed flow follows from the pump, the valve and
// the vessel pressure; it enters at the heater outlet temperature. The main steam passes the MSIVs to the
// header; from there the stop and control valves feed the turbine stub, which exhausts to the condenser,
// and the bypass valve feeds the condenser directly (throttled at constant enthalpy: superheated there); the condensate collects in the hotwell, which the feed pump draws from.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau,pc0=s.f.Pc;
  const q=sat(d.T),hg=q.hg,ug=q.ug,uf=q.uf,hf=q.hf,rf=q.rf,rg=q.rg; // sat() reuses one object: copy fields first
  // reactor protection on the start-of-step signals: the latched turbine trip and isolation shut their
  // valves this step; the MSIV and stop valve position switches are read after the valves have moved,
  // with the turbine first-stage pressure (∝ turbine flow) from before the step
  const sg=rpsSignals(s);let pr=rpsCheck(s.rps,sg,s.t);
  const ut=u.trip||{};
  valveStep(s,{...u,trip:{turbine:ut.turbine||pr.turb,msiv:ut.msiv||pr.iso,bypass:ut.bypass||bpBlock(s.rps,pc0)}},dt);
  pr=rpsCheck(s.rps,{...sg,ms:Math.min(...s.ms),sv:s.sv},s.t);
  if(pr.scram&&!s.scram)scram(s,s.f.Q);
  // SRVs: spring on pressure (with blowdown), or opened by hand
  const {open,wsrv,nsrv}=srvStep(s.spr,u.srv||[],P);
  // main steam line: header pressure and the turbine and bypass flows at the start-of-step pressures
  const sl=steamLine(P,pc0,s,s.Pl),wst=sl.w;s.Pl=sl.Pl;
  // feedwater train: valve travel, pump and valve flow against the vessel pressure, hotwell, heater
  const ext=clamp(u.ext||0,0,1),mk=clamp(u.mk||0,-MKMAX,MKMAX);
  s.n+=clamp((pr.fwp?0:clamp(u.n||0,0,1))-s.n,-NRATE*dt,NRATE*dt);const n=s.n;
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
  // circulating water (flow lags the pump and valve) and condenser air: in-leakage, radiolytic gas with the
  // main steam, vacuum breaker; removal by the SJAE and the mechanical vacuum pump. The SJAE motive steam is
  // tapped off the steam header (downstream of the MSIVs, so isolated with them): it is taken out of the
  // line flow ahead of the turbine (then the bypass) and reaches the condenser at its supply enthalpy.
  const cwp=u.cwp??true,cwv=clamp(u.cwv??1,0,1),Tcwi=u.tcw??TCW;s.wcw=cwStep(s.wcw,cwp,cwv,dt);
  s.vb+=clamp((u.vb?1:0)-s.vb,-VBRATE*dt,VBRATE*dt);
  const Pm=motiveP(s),sj=u.sj??true,vp=!!u.vp,wsjD=sj?sjSteam(Pm,pc0):0,wsj=Math.min(wsjD,wst),fsj=wsjD>0?wsj/wsjD:0;
  const wt=sl.wt-Math.min(wsj,sl.wt),wb=sl.wb-Math.max(0,wsj-sl.wt);
  const af=airFlows(pc0,s.cd.Tc,{Q,share:wst+wsrv>0?wst/(wst+wsrv):0,sj:fsj,Pm,vp,vb:s.vb,dt});
  // turbine stub (throttle at the header): extraction to the heater, the rest to the condenser along the
  // expansion line; extraction pressure in proportion to the turbine flow
  const line=expansion(sl.Pl,hg),ht=heater(wt,ext,fw,s.Thw,line),Tt=ht.T,ws=Math.min(ht.ws,wt);
  s.Th=Tt+(s.Th-Tt)*Math.exp(-dt/tau.h);
  const wx=wt-ws,hx=wx>0?line(pc0):0,Pgen=wt>0?turbinePower(wt,hg,[{w:ws,h:ht.hx}],hx):0;
  // condenser: exhaust steam, heater drains, bypass steam and SJAE steam (both at the dome enthalpy) in,
  // condensate down to the hotwell
  // reactor water cleanup reject: vessel water to the condenser (it flashes there)
  const wrw=Math.min(clamp(u.rw||0,0,1)*RWMAX,0.1*Math.max(0,d.Ml)/dt),hrw=hLiq(d.Tl,P);
  const cd=condense(s.cd,[[wx,hx],[ws,ht.hd],[wb,hg],[wsj,hg],[wrw,hrw]],dt,{w:s.wcw,T:Tcwi},af);
  const hw=hotwellMix({M:s.Mh,T:s.Thw},[[cd.wl,sat(cd.Tc).hf],[mk,HMK],[-fw,0]],dt);
  s.Mh=hw.M;s.Thw=hw.T;
  // flows at the start-of-step pressure: steam leaves at h″, feedwater joins the water, heat into the water
  const mo=Math.min((wst+wsrv)*dt,0.5*s.M);
  // vessel steel: heat from (or to) the water, losses to the drywell
  const st=steelStep(s.Ts,d.Tl,dt);s.Ts=st.Ts;
  // suppression pool: the SRV steam condenses in it
  const pl=poolStep(s.pool,wsrv*Math.min(1,mo/((wst+wsrv)*dt||1)),hg,!!u.pc,dt);s.pool={M:pl.M,T:pl.T};
  let M=s.M+(fw-wrw)*dt-mo,U=s.U+fw*dt*hfw-wrw*dt*hrw-mo*hg+(Q+st.Qw)*dt,Mv=Math.max(0,s.Mv-mo);
  // subcooled core-region water condenses steam, relaxing towards saturation at the start-of-step steam
  // temperature (the downcomer's subcooling is carried into the core first)
  let mc=0;{const Ml=M-Mv,ue=Math.min(s.udc,uf),D=(Ml-Md)*uf-(U-Mv*ug-Md*ue);
    if(Mv>0&&D>0)mc=Math.min(D/(ug-uf)*(1-Math.exp(-dt/tau.c)),0.9*Mv);Mv-=mc}
  const e=solve({M,U,Mv,T0:d.T,Mdc:Md,udc:s.udc});
  if(!e){s.d={err:'range',P:0};s.t+=dt;return}
  s.udc=e.ue; // a downcomer above saturation has flashed
  // voids inside the level: the core and chimney voids from the core thermal-hydraulics, plus the
  // steam that flashing makes beyond what the core boils (a depressurisation flashes water throughout
  // the vessel: the downcomer and lower plenum too, and those bubbles have further to go), which rises
  // out with TAUF
  const gen=(e.Mv-Mv-mc)/dt,g=Math.max(0,gen-k.W*k.xe)*TAUF;
  s.Mfl=g+(s.Mfl-g)*Math.exp(-dt/TAUF);
  s.Mb=Math.min(sat(e.T).rg*voidVol(k)+s.Mfl,0.9*e.Mv);
  Object.assign(s,{M,U,Mv:e.Mv});s.t+=dt;
  s.d=derive(s,e);
  s.f={Q,fw,Tfw,hfw,hg,wst,wsrv,nsrv,open,spr:s.spr.slice(),gen,cond:mc/dt,
    ms:s.ms.slice(),sv:s.sv,cv:s.cv,bp:s.bp,Pl:sl.Pl,wt,wb,
    n,fv:s.fv,ext,Tt,pe:ht.pe,ps:ht.ps,ws,Pd,ret:cd.wl,Tret:cd.Tc,mk,Mh:s.Mh,hw:s.Mh/HWM,Thw:s.Thw,
    Pc:cd.Pc,Tc:cd.Tc,wc:cd.wc,Qc:cd.Q,Tcw:cd.Tout,wcw:s.wcw,Pgen,wx,hx,
    Tcwi,cwp,cwv,pa:cd.pa,ya:cd.ya,Ma:s.cd.Ma,wsj,sj,vp,vb:s.vb,sjc:fsj*sjCap(Pm,pc0),vpc:vp?vpCap(pc0):0,
    wlk:af.wlk,wrad:af.wrad,wvb:af.wvb,wain:af.wi,waout:cd.wout,Vsj:af.Vsj,Vvp:af.Vvp,
    Ts:s.Ts,Qs:st.Qw,Ql:st.Ql,Tp:pl.T,Mp:pl.M,Lp:poolLevel(pl.M),Qpc:pl.Qc,pc:!!u.pc,lin:toIn(s.d.level),wrw,hrw,trip:pr};
  coreF(s.f,k,rc);
};
// scram from heat q (W; default the last applied net heat, taken as core power; none if not positive);
// a second scram keeps the first; reset hands the heat back to the command
const scram=(s,q=s.f.Q)=>{if(!s.scram)s.scram={t:s.t,P0:Math.max(0,q)}};
// Protection signals from the plant state at the start of a step: dome pressure, level in inches above
// instrument zero, APRM (the fission power as a fraction of its rated value), MSIV and stop
// valve positions, turbine first-stage pressure as the turbine flow fraction, header and condenser
// pressures, main steam line flow fraction
const rpsSignals=s=>({P:s.d.P,lin:toIn(s.d.level),aprm:s.core?s.core.n*(1+DSUM)/QR:Math.max(0,s.f.Q)/QR,ms:Math.min(...s.ms),sv:s.sv,
  fsp:(s.f.wt||0)/WR,Pl:s.Pl,Pc:s.f.Pc,wst:(s.f.wst||0)/WR});
// reset: the protection resets every trip whose signal has cleared; the scram resets once no scram trip is
// left in, and the heat goes back to the command
const reset=s=>{rpsReset(s.rps,rpsSignals(s),s.t);if(!groups(s.rps).scram)s.scram=null};
// the reactor mode switch (Shutdown scrams)
const setMode=(s,m)=>{rpsMode(s.rps,m,s.t);const pr=rpsCheck(s.rps,rpsSignals(s),s.t);if(pr.scram&&!s.scram)scram(s,s.f.Q)};
// the operator's scram button: logged with the protection's events
const manualScram=(s,q)=>{if(!s.scram)s.rps.log.push({t:s.t,text:'Manual scram'});scram(s,q)};
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
  if(s.core.pc)return 'prompt';
  return '';
};
// the condensate pump suction pressure the page reads for the shutoff-head message
const PC=PCD;
const API=9;
export {CST,UAS,UAL,TDW,INCH,IZ,LV,toIn,fromIn,TAUF,MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolLevel};
export {UA0,RA,PATM,WLK,WRAD,VSJ,WSJ,VVP,WVB,PA0,PSB0,PSB1,PSJ0,PSJ1,PVP0,PVP1,pAir,yAir,blanket,sjSteam,sjCap,vpCap,cwTarget,TAUCW,motiveP};
export {API,V,H,A,Z,QR,PR,TFR,SRV,NV,PB,BD,TAU,H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PC,PCD,TTD,WR,PE,KXT,
  ETA,ETAG,VC,WCW,TCW,CPW,UA,PCMAX,expansion,turbinePower,condense,condSteady,
  WTR,fwFlow,fwValve,heater,heaterT,heatBalance,subF,wSRV,decayFrac,init,step,scram,reset,fault,rpsSignals,setMode,manualScram,SP,MODES,TRIPS,GROUP,inHgVac,rpsActive,RWMAX,
  DPL,PL0,VWO,KT,BPF,KB,fcv,fcvInv,MSRATE,SVCLOSE,SVOPEN,CVRATE,BPRATE,steamLine,cvPos,
  QC,WCR,BETA,LAMBDA,SRC,AV,AD,AM,MCF,RF,X0,XR,WT,RODRATE,TSCRAM,DSUM,DL,DF,VCH,AF,LC,VDC,voids,voidVol,rodRho,rodFor,WDR,HPR,HS,KPP,MGRATE,NMIN,TCD};
