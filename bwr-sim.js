// BWR plant model: pure physics, no DOM, no automatic control. step() takes final actuator commands
// (from the operator or from bwr-control.js; the plant can't tell). An ES module. It composes the
// per-system modules, each with pure functions and its own part of the state:
//   bwr-vessel.js    reactor vessel geometry and the water–steam solve
//   bwr-steam.js     rated conditions, main steam line and turbine/bypass valves, safety/relief valves
//   bwr-turbine.js   HP and LP turbines (stage pressures), MSR, generator, shaft speed, grid breaker
//   bwr-condenser.js main condenser, circulating water, condenser air removal
//   bwr-feed.js      hotwell, condensate pump, polisher, LP heater, feed pump, min-flow line, HP heater,
//                    feed valve, heater drains, condensate storage tank
//   bwr-containment.js suppression pool
//   bwr-rps.js       reactor protection: scram, isolation, turbine and feed pump trips (latched, logged)
//   bwr-core.js      point kinetics, fuel, decay heat, control rod groups and drives, core voids
//   bwr-recirc.js    recirculation pump on its MG set, jet pump, core flow, natural circulation
// Equations and constants are described in docs/bwr.md.
import {sat,Tsat,clamp,hLiq,cpL,liqRho} from './common.js';
import {V,H,A,Z,TAU,VDC,solve,derive,dcMass,vesselInit} from './bwr-vessel.js';
import {CST,UAS,UAL,TDW,steelStep,INCH,IZ,LV,toIn,fromIn,TAUF,RWMAX} from './bwr-vessel.js';
import {SP,MODES,TRIPS,GROUP,inHgVac,rpsInit,rpsCheck,rpsReset,rpsMode,bpBlock,rpsActive,groups} from './bwr-rps.js';
import {MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolStep,poolLevel} from './bwr-containment.js';
import {QR,PR,TFR,WR,SRV,NV,PB,BD,subF,wSRV,srvStep} from './bwr-steam.js';
import {DPL,PL0,VWO,KT,BPF,KB,fcv,fcvInv,MSRATE,SVCLOSE,SVOPEN,CVRATE,BPRATE,steamLine,cvPos,valveStep} from './bwr-steam.js';
import {VC,WCW,TCW,CPW,UA,PCMAX,condense,condSteady} from './bwr-condenser.js';
import {expansion,turbineLine,stagePs,work,P1R,PXR,PLIR,ETAH,ETAL,NR,HIN,ETAG,PW0F,TRH,IVRATE,PLU,SYNC,TRP,tgInit,tgStep,breaker as tgBreaker,brkTrips,pluStep} from './bwr-turbine.js';
import {UA0,RA,PATM,WLK,WRAD,VSJ,WSJ,VVP,WVB,VBRATE,PA0,PSB0,PSB1,PSJ0,PSJ1,PVP0,PVP1,pAir,yAir,blanket,sjSteam,sjCap,vpCap,airFlows,cwStep,cwTarget,TAUCW} from './bwr-condenser.js';
import {NZ,STGT,GZ,RATED,steady,localFb,peak,fuelT,TREF,QC,WCR,BETA,LAMBDA,SRC,AV,AD,AM,MCF,RF,NG,NPOS,NOTCH,GW,XRATED,RODRATE,TSD,VSC,TSCRAM,WCRD,DSUM,DL,DF,VCH,AF,LC,WMIN,voids,voidVol,cov,rodR,seq,rodsFor,coreInit,coreStep} from './bwr-core.js';
import {WDR,HPR,HS,KPP,MGRATE,NMIN,TCD,mgStep,loopStep,recircInit} from './bwr-recirc.js';
import {WTR,H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,KXT,fwFlow,fwValve,heater,heaterT,heatBalance,hotwellMix,
  PEL,KXL,TLPD,HC0,HF0,KCP,KPOL,KLP,KFP,ZS,HWD,NPR,PLO,TLO,WMIN as WMINF,KR,RRATE,network,shutoff,heaterL,train,suction,cpFlow,
  THPI,HDP0,ZDT,KDPP,KDVL,KDVH,KDDH,KDDL,WSD,WSLD,WDD,VHS,VLS,LFL,LDU,XD0,MHS,MLS,DPD,mixT,dpFlow,tubes,dumpX,Thf,
  CSTM,HCST,CRATE,KMU,KRJ,muFlow,rjBack,RSU,TK,PGR,lineAt} from './bwr-feed.js';
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
// SJAE motive steam supply pressure: the main steam header
const motiveP=s=>s.Pl??s.d.P;
// o: {P Pa, L m two-phase level, Tl °C water temperature (default saturated), gen kg/s steaming the
// voids start in equilibrium with, cv control valve and bp bypass valve positions 0–1 (default shut), ms MSIV
// and sv stop valve positions 0–1 (default open), fv feed valve and sfv startup feed valve positions 0–1, n pump speed 0–1, tfw °C heater
// outlet (default the hotwell temperature), hw hotwell fraction full (default 0.5), ext, exl 0–1 HP and LP
// extraction valves the condenser starts steady with (default 1; exl defaults to ext), xr min-flow valve 0–1,
// xdh, xdl heater drain valves 0–1 (default the design 0.5), hs, ls HP and LP heater shell fraction full
// (default 0.5), cst condensate storage tank fraction full (default 0.7; its water at TMK), xmu, xrj makeup and
// reject valves, vac false for a condenser open to atmosphere (full of air at 0.1 MPa; default: under vacuum,
// air at its steady value with the SJAE in service), ts °C vessel steel (default steady between water and
// drywell), tpool °C suppression pool (default 32), mode switch 'shutdown' | 'startup' | 'run' (default run at
// 6 MPa and above, else startup), rps false to run without protection (tests of the bare physics), tau overrides}
const init=o=>{
  const s={t:0,wcw:WCW,vb:0,ms:[o.ms??1,o.ms??1],sv:o.sv??1,cv:o.cv||0,bp:o.bp||0,fv:o.fv||0,sfv:o.sfv||0,n:o.n||0,
    xr:o.xr||0,fpt:false,tlo:0,xdh:o.xdh??XD0,xdl:o.xdl??XD0,Mhs:(o.hs??0.5)*MHS,Mls:(o.ls??0.5)*MLS,fail:{},
    Mcst:(o.cst??0.7)*CSTM,Tcst:TMK,xmu:o.xmu||0,xrj:o.xrj||0,
    Mh:(o.hw??0.5)*HWM,spr:Array(NV).fill(false),scram:null,tau:{...TAU,...o.tau}};
  // core and recirculation: steady at heat o.q (default: the heat that makes the steam o.gen from feed
  // at o.tfw), pump breaker o.rp (default closed) at MG set speed o.mg (default 1), critical with the
  // rods at o.rods (array of NG group positions 0–1, withdrawn fraction; default the critical pattern along
  // the withdrawal sequence that init solves for; with no heat, all in). Core flow and voids are solved
  // together; the voids set the void steam Mb inside the level.
  // The downcomer starts steady too: feed mixed with the separator return, subcooled by fw·(u′ − u_fw)/W.
  let dh0;
  {const P=o.P,T=Tsat(P),q=sat(T),uf=q.uf,hg=q.hg,rf=q.rf,rg=q.rg,Tfw=o.tfw!=null?o.tfw+273.15:T,hfw=hLiq(Tfw,P),ufw=hfw-P/liqRho(Tfw,P),fw=o.gen||0,Q=o.q??(fw>0?fw*(hg-hfw)+UAL*UAS/(UAS+UAL)*(T-TDW):0),cp=cpL(T);
   let W=WCR,v,dh,rc;const rp=o.rp??true,mg=o.mg??1;
   for(let i=0;i<8;i++){dh=Math.max(0,fw*(uf-ufw))/Math.max(W,WMIN,fw);v=voids(Q,W,dh,P);
     rc=recircInit(rp,mg,{rd:liqRho(T-dh/cp,P),rc:rf,rg,ac:v.ac,ae:v.ae,xe:v.xe});W=rc.Wc}
   s.rc=rc;s.core=coreInit(Q,T,T-dh/cp,v,o.rods);s.core.W=W;s.core.dh=dh;s.Mb=rg*voidVol(s.core);s.Mfl=0;dh0=dh}
  vesselInit(s,{...o,dh:dh0});
  s.rps=rpsInit(o.mode??(o.P>=6e6?'run':'startup'),o.rps===false);
  s.Ts=o.ts!=null?o.ts+273.15:(UAS*s.d.Tl+UAL*TDW)/(UAS+UAL);s.pool={M:MPOOL,T:o.tpool!=null?o.tpool+273.15:TP0};
  // condenser and hotwell start steady for the initial steam flow (turbine exhaust; the heater drains are
  // pumped forward)
  const P=s.d.P,hg=sat(s.d.T).hg;let Tc=TCW+0.1,ht=null,sl=steamLine(P,PCD,s);
  // air: steady against the SJAE in service (in-leakage plus radiolytic gas, taken ∝ the initial steam
  // flow), or (vac false) a shell open to atmosphere, full of air at the circulating water temperature
  const vac=o.vac!==false,wsj0=vac?Math.min(sjSteam(P,0),sl.w):0;
  const MaV=T=>{const c=sjCap(P,sat(T).p+PA0);return c>0?(WLK+WRAD*Math.min(1,sl.w/WR))*VC/(VSJ*c):0};
  // the turbine's stage pressures depend on the extraction flows and these on the stage pressures: iterate,
  // starting from the extractions in their rated proportion; the windage heat goes to the exhaust steam
  // (breaker closed, at grid speed, when steam passes the turbine)
  const brk=o.brk??Math.max(0,sl.wt-wsj0)>0.02*WR;
  let wsH=0,wsL=0,w0=0,line=null,pc=PCD,wk={hp:0,lp:0};
  for(let i=0;i<10;i++){const pa=pAir(MaV(Tc),Tc);pc=sat(Tc).p+pa;sl=steamLine(P,pc,s);w0=Math.max(0,sl.wt-wsj0);
    if(!i){wsH=w0/WTR*WSD;wsL=w0/WTR*WSLD}
    line=turbineLine(stagePs(TK,w0,wsH,wsL,pc,sl.Pl),hg);
    ht=train(w0,o.exl??o.ext??1,o.ext??1,sl.w,Tc,line);wsH=Math.min(ht.hp.ws,w0);wsL=Math.min(ht.lp.ws,w0-wsH);
    const wx=w0-wsH-wsL,hx=line(pc)+(brk&&wx>10?PW0F*PGR/wx:0);wk=w0>0?work(w0,hg,line,wsH,wsL,line(pc)):{hp:0,lp:0};
    Tc=sl.w>0?condSteady([[wx,hx],[sl.wb,hg],[wsj0,hg]],undefined,pa):TCW+0.1}
  // shaft and generator: on the grid at rated speed, the LP power steady
  s.tg=tgInit({brk,w:o.spd??(brk?1:0),PL:wk.lp});
  if(!vac)Tc=TCW;
  const Ma=vac?MaV(Tc):(PATM-sat(Tc).p)*VC/(RA*Tc);
  s.Pl=sl.Pl;
  s.cd={Tc,Ms:sat(Tc).rg*VC,Ma};s.Thw=Tc;s.Tsu=sl.w>0?ht.Tm:Tc;
  // heater shells saturated at their initial pressures (the hotwell temperature with no extraction)
  s.psh=ht.hp.ps||sat(Tc).p;s.psl=ht.lp.ps||sat(Tc).p;s.Ths=ht.hp.ps?Thf(ht.hp.hd):Tc;s.Tls=ht.lp.ps?Thf(ht.lp.hd):Tc;
  s.Th=o.tfw!=null?o.tfw+273.15:Tc;
  const pa0=pAir(Ma,Tc),Pc=sat(Tc).p+pa0;
  s.f={Q:0,fw:0,Tfw:s.Th,hfw:0,hg:0,wst:0,wsrv:0,nsrv:0,open:Array(NV).fill(false),spr:Array(NV).fill(false),gen:o.gen||0,cond:0,
    ms:s.ms.slice(),sv:s.sv,cv:s.cv,bp:s.bp,Pl:s.Pl,wt:sl.wt,wb:sl.wb,
    n:s.n,fv:s.fv,sfv:s.sfv,wsu:0,ext:0,Tt:s.Th,pe:0,ps:0,Pd:Pc,ret:0,Tret:Tc,mk:0,Mh:s.Mh,hw:s.Mh/HWM,Thw:Tc,
    Pc,Tc,wc:0,Qc:0,Tcw:TCW,wcw:WCW,Pgen:brk?ETAG*(wk.hp+wk.lp-PW0F*PGR):0,wx:0,hx:0,
    ...tbF(line.ps,s.tg,{Pm:wk.hp+wk.lp,Pw:PW0F*PGR*s.tg.w**3},wk),ws:wsH,wsl:wsL,
    Tcwi:TCW,cwp:true,cwv:1,pa:pa0,ya:pa0/Pc,Ma,wsj:0,sj:vac,vp:false,vb:0,sjc:0,vpc:0,wlk:0,wrad:0,wvb:0,wain:0,waout:0,Vsj:0,Vvp:0,
    Ts:s.Ts,Qs:0,Ql:0,Tp:s.pool.T,Mp:s.pool.M,Lp:0,Qpc:0,pc:false,lin:toIn(s.d.level),wrw:0,trip:groups(s.rps),
    exl:0,pel:0,psl:0,Tlp:Tc,Ps:Pc,Pcd:Pc,wp:0,wr:0,wcp:0,xr:s.xr,cp:false,fp:false,fpt:false,npsh:0,phi:1,cav:false,
    Tm:s.Tsu,wdh:0,wd:0,wduh:0,wdul:0,LH:s.Mhs/MHS,LL:s.Mls/MLS,aH:1,aL:1,xdh:s.xdh,xdl:s.xdl,hdp:false,hdvf:false,
    wmu:0,wrj:0,xmu:s.xmu,xrj:s.xrj,Mcst:s.Mcst,cst:s.Mcst/CSTM,Tcst:s.Tcst,crd:false,wcd:0};
  coreF(s.f,s.core,s.rc);
  return s;
};
// turbine readings into s.f: stage pressures p1 first stage, pX HP exhaust, pLi LP inlet; spd shaft speed
// (fraction of rated), brk generator breaker closed, lock tripped open, iv intercept valves, plu fast
// closure in, Pm shaft power, Pw windage, Php, Plp the HP and LP steam work (W)
const tbF=(ps,tg,q,wk)=>({p1:ps.p1,pX:ps.pX,pLi:ps.pLi,spd:tg.w,brk:tg.brk,lock:tg.lock,iv:tg.iv,plu:tg.plu,Pm:q.Pm,Pw:q.Pw,Php:wk.hp,Plp:wk.lp});
// core readings into s.f: Pth core thermal power (fission + decay, W), nf fission power, rho reactivity,
// per period (s), Tf fuel temperature, pk axial peaking, zpk its height (fraction of the core), sh the shape, ac core-average void, ae exit void, xe exit quality, Wc core
// flow, dhin inlet subcooling (J/kg), rods group positions 0–1 (withdrawn fraction), st time since the scram
// started driving them (null when not scrammed), pc prompt critical
// recirculation: rp pump breaker closed, ng pump (MG set) speed 0–1, mg speed demand, Wd drive flow
const coreF=(f,k,r)=>Object.assign(f,{Pth:k.q,nf:k.n,rho:k.rho,per:k.per,Tf:k.Tf,ac:k.ac,ae:k.ae,xe:k.xe,Wc:k.W,dhin:k.dh,rods:k.x.slice(),pc:k.pc,pk:k.pk,zpk:k.zpk,sh:k.s,
  rp:r.on,ng:r.ng,mg:r.dm,Wd:r.Wd});
// One step. u is the final command, SI: {rods array of NG control rod group position demands 0–1 (withdrawn
// fraction; manual only; undefined, or an undefined entry: the group stays where it is); crd CRD pump run
// switch (true running; absent or false stopped, so the other systems' tests run without its flow; stopped,
// the rods can't be moved, the scram still works); heat W fixed net heat into the water (a test hook: if it is a number the
// core heat is that value and the kinetics stand still); n 0–1 feed pump speed;
// fv 0–1 feed valve demand; sfv 0–1 startup feed valve demand (absent: shut); ext 0–1 HP heater extraction valve; exl 0–1 LP heater extraction valve
// (absent: the same as ext); cp, fp condensate and feed pump run switches (absent: running); rv 0–1
// min-flow valve demand; hdv 0–1 HP heater drain valve demand, ldv 0–1 LP drain pump discharge valve demand
// (absent: the design opening 0.5), hdp heater drain pump run switch (absent: running);
// mu, rj 0–1 makeup valve (CST → condenser) and reject valve (condensate pump discharge → CST) demands
// (f.mk is the net makeup, makeup − reject);
// ms1, ms2 inboard and outboard MSIV and sv turbine stop valve switches (true open, false shut; left
// undefined, the valve stays where it is); cv 0–1 turbine control valve and bp 0–1 bypass valve demands;
// trip {turbine, msiv, bypass} protection hook: shuts the stop valve / the MSIVs / the bypass whatever the
// switches; srv array, true for each SRV opened by hand; circulating water and
// condenser air removal: cwp CW pump running, cwv 0–1 CW valve, tcw K CW inlet temperature, sj SJAE motive
// steam valve open, vp mechanical vacuum pump running, vb vacuum breaker open (booleans)}. An absent cwp,
// cwv, tcw or sj means the normal running line-up (pump on, valve open, 20 °C, SJAE in service); absent vp
// and vb mean off and shut. These are also the hooks for protection: a CW pump trip is cwp false, a feed
// pump trip fp false (the protection's own L8 feed pump trip stops it too).
// pc true: suppression pool cooling on. rw 0–1 reactor water cleanup reject to the condenser.
// rp recirculation pump breaker (true closed; undefined keeps it), mg 0–1 MG set speed demand.
// After scram(s) the rods drive fully in (all in TSCRAM from fully out) and the kinetics follow; with the fixed-heat hook
// the heat is the decay curve and u.heat is ignored until reset(s).
// Each command is clamped to its range; the valves move toward their demands at their stroke rates; each
// SRV's spring opens it on pressure whatever the command. Feed flow follows from the pump, the valve and
// the vessel pressure; it enters at the heater outlet temperature. The main steam passes the MSIVs to the
// header; from there the stop and control valves feed the turbine (HP, MSR, LP), which exhausts to the condenser,
// and the bypass valve feeds the condenser directly (throttled at constant enthalpy: superheated there); the condensate collects in the hotwell, which the feed pump draws from.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,tau=s.tau,pc0=s.f.Pc;
  const q=sat(d.T),hg=q.hg,ug=q.ug,uf=q.uf,hf=q.hf,rf=q.rf,rg=q.rg; // sat() reuses one object: copy fields first
  // turbine and generator on the start-of-step state: the breaker trips on a turbine trip or reverse power;
  // the power–load unbalance (a load rejection) fast-closes the control and intercept valves, before the
  // protection reads it
  const tg=s.tg,lg=text=>s.rps.log.push({t:s.t,text});
  brkTrips(tg,s.sv,s.f.Pm||0,s.f.Pw||0,dt,lg);pluStep(tg,s.f.pLi||0,tg.brk?s.f.Pgen:0,PGR,s.sv,dt);
  // reactor protection on the start-of-step signals: the latched turbine trip and isolation shut their
  // valves this step; the MSIV and stop valve position switches are read after the valves have moved,
  // with the turbine first-stage pressure from before the step
  const sg=rpsSignals(s);let pr=rpsCheck(s.rps,sg,s.t);
  const ut=u.trip||{};
  valveStep(s,{...u,trip:{turbine:ut.turbine||pr.turb,msiv:ut.msiv||pr.iso,bypass:ut.bypass||bpBlock(s.rps,pc0),cvfast:tg.plu}},dt);
  pr=rpsCheck(s.rps,{...sg,ms:Math.min(...s.ms),sv:s.sv},s.t);
  if(pr.scram&&!s.scram)scram(s,s.f.Q);
  // SRVs: spring on pressure (with blowdown), or opened by hand
  const {open,wsrv,nsrv}=srvStep(s.spr,u.srv||[],P);
  // main steam line: header pressure and the turbine and bypass flows at the start-of-step pressures
  const sl=steamLine(P,pc0,s,s.Pl),wst=sl.w;s.Pl=sl.Pl;
  // feedwater train: valve travel, pump and valve flow against the vessel pressure, hotwell, heater
  const ext=clamp(u.ext||0,0,1),exl=clamp(u.exl??u.ext??0,0,1);
  // condensate storage tank: makeup by the tank's head and the condenser vacuum; the reject valve
  // is a branch of the train solve at the condensate pump discharge
  s.xmu+=clamp(clamp(u.mu||0,0,1)-s.xmu,-CRATE*dt,CRATE*dt);s.xrj+=clamp(clamp(u.rj||0,0,1)-s.xrj,-CRATE*dt,CRATE*dt);
  const wmu=Math.min(muFlow(s.xmu,s.Mcst,pc0),s.Mcst/dt);
  // pump switches; the feed pump's low-suction trip holds it stopped until its switch is cycled; the
  // protection's L8 trip stops it while latched
  const cp=u.cp!==false,fp=u.fp!==false;if(!fp){s.fpt=false;s.tlo=0}
  s.n+=clamp((fp&&!s.fpt&&!pr.fwp?clamp(u.n||0,0,1):0)-s.n,-NRATE*dt,NRATE*dt);const n=s.n;
  s.fv+=clamp(clamp(u.fv||0,0,1)-s.fv,-FRATE*dt,FRATE*dt);
  s.sfv=(s.sfv||0)+clamp(clamp(u.sfv||0,0,1)-(s.sfv||0),-FRATE*dt,FRATE*dt);
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
  const pv=sat(s.Tsu).p,nw=network({cp,n,x:s.fv,xs:s.sfv,xr:s.xr,P,pc:pc0,Mh:s.Mh,Thw:s.Thw,pv,dp,rj:{x:s.xrj,Pb:rjBack(s.Mcst)}}),fw=nw.fw,Pd=nw.Pd;
  if(fp&&!s.fpt){s.tlo=nw.Ps<PLO?s.tlo+dt:0;if(s.tlo>=TLO)s.fpt=true}
  // core: kinetics, fuel, voids and the recirculation loop in substeps at the start-of-step pressure.
  // The core inlet is downcomer water: inlet subcooling u′ − u_dc, at the downcomer temperature.
  // Recirculation: pump breaker u.rp (true closed; undefined keeps it), MG set speed demand u.mg 0–1
  // (undefined keeps it).
  const k=s.core,rc=s.rc,fix=Number.isFinite(u.heat),W0=k.W,gc0=k.W*k.xe;
  if(u.rp!=null)rc.on=!!u.rp;if(u.mg!=null)rc.dm=clamp(u.mg,0,1);
  k.dh=Math.max(0,uf-Math.min(s.udc,uf));
  const Tin=d.Tdc,lpc={rd:liqRho(Tin,P),rc:rf,rg,ac:0,ae:0,xe:0};
  const hq=coreStep(k,{P,Tc:d.T,dh:k.dh,Tin,rods:u.rods,crd:u.crd===true,scram:!!s.scram,heat:fix?(s.scram?s.scram.P0*decayFrac(s.t-s.scram.t):u.heat):undefined},dt,
    (h,c)=>{mgStep(rc,h);lpc.ac=c.ac;lpc.ae=c.ae;lpc.xe=c.xe;loopStep(rc,lpc,h);return rc.Wc});
  const Q=hq;
  // downcomer: the feed and the separator return (core-region water) mix into it and the core flow draws
  // it off (exact for the step; when the core flow is below the feed flow the surplus feed spills over
  // into the core region)
  // The CRD pump's drive and cooling water (from the storage tank, at its temperature) joins the feed there.
  const crd=u.crd===true,wcd=crd?Math.min(WCRD,s.Mcst/dt):0,hcd=hLiq(s.Tcst,P),ucd=hcd-P/liqRho(s.Tcst,P);
  const Md=dcMass(d.Tdc,P,d.Ml);
  {const W=Math.max(0,(W0+k.W)/2),fi=fw+wcd,Wi=Math.max(W,fi),ufw=hfw-P/liqRho(Tfw,P),ur=sat(d.Tl).uf;
   if(Wi>0){const ut=(fw*ufw+wcd*ucd+Math.max(0,W-fi)*ur)/Wi;s.udc=ut+(s.udc-ut)*Math.exp(-Wi*dt/Md)}}
  // circulating water (flow lags the pump and valve) and condenser air: in-leakage, radiolytic gas with the
  // main steam, vacuum breaker; removal by the SJAE and the mechanical vacuum pump. The SJAE motive steam is
  // tapped off the steam header (downstream of the MSIVs, so isolated with them): it is taken out of the
  // line flow ahead of the turbine (then the bypass) and reaches the condenser at its supply enthalpy.
  const cwp=u.cwp??true,cwv=clamp(u.cwv??1,0,1),Tcwi=u.tcw??TCW;s.wcw=cwStep(s.wcw,cwp,cwv,dt);
  s.vb+=clamp((u.vb?1:0)-s.vb,-VBRATE*dt,VBRATE*dt);
  const Pm=motiveP(s),sj=u.sj??true,vp=!!u.vp,wsjD=sj?sjSteam(Pm,pc0):0,wsj=Math.min(wsjD,wst),fsj=wsjD>0?wsj/wsjD:0;
  const wt=sl.wt-Math.min(wsj,sl.wt),wb=sl.wb-Math.max(0,wsj-sl.wt);
  const af=airFlows(pc0,s.cd.Tc,{Q,share:wst+wsrv>0?wst/(wst+wsrv):0,sj:fsj,Pm,vp,vb:s.vb,dt});
  // turbine (bwr-turbine.js): stage pressures from the turbine flow, the last step's extraction flows and the
  // condenser pressure (Stodola), the expansion line through the HP turbine, the MSR and the LP turbine;
  // extractions to the LP heater (condensate flow; the cascaded HP drains flash in its shell) and the HP
  // heater (feed flow, entering at the condensate and pumped drains mixed), the rest to the condenser.
  // Flooded tubes heat less.
  const aH=tubes(LH),aL=tubes(LL),line=turbineLine(stagePs(TK,wt,s.f.ws||0,s.f.wsl||0,pc0,sl.Pl),hg),lp=heaterL(wt,exl,nw.wl,s.Thw,line,wdh>0?[[wdh,hdH]]:[],aL);
  const hdL=lp.ps>0?lp.hd:hdL0,Tm=mixT(nw.wl,lp.T,nw.wd,hdL),ht=heater(wt,ext,fw,Tm,line,[],aH),Tt=ht.T;
  const ws=Math.min(ht.ws,wt),wsl=Math.min(lp.ws,wt-ws);s.Tsu=Tm;
  // shell inventories; the LP dump takes only what the shell holds
  // (the LP shell's vent takes the flash steam the condensate can't condense; it comes out of its drains)
  const wv=Math.min(lp.wv,Math.max(0,s.Mls/dt+wsl+wdh-nw.wd)),wdul=Math.max(0,Math.min(wdul0,s.Mls/dt+wsl+wdh-nw.wd-wv));
  s.Mhs+=(ws-wdh-wduh)*dt;s.Mls+=(wsl+wdh-nw.wd-wdul-wv)*dt;
  if(ht.ps>0){s.psh=ht.ps;s.Ths=Thf(ht.hd)}if(lp.ps>0){s.psl=lp.ps;s.Tls=Thf(lp.hd)}
  s.Th=Tt+(s.Th-Tt)*Math.exp(-dt/tau.h);
  // shaft and generator: the steam's work in the two turbines drives the shaft (LP lagged, through the
  // intercept valves); on the grid the generator turns it into electricity, off it the speed changes. The
  // windage heats the exhaust steam.
  const wx=wt-ws-wsl,hx0=wx>0?line(pc0):0,wk=wt>0?work(wt,hg,line,ws,wsl,hx0):{hp:0,lp:0};
  const tq=tgStep(tg,wk.hp,wk.lp,PGR,dt),Pgen=tq.Pe,hx=hx0+(wx>10?tq.Pw/wx:0);
  // reactor water cleanup reject: vessel water to the condenser (it flashes there)
  const wrw=Math.min(clamp(u.rw||0,0,1)*RWMAX,0.1*Math.max(0,d.Ml)/dt),hrw=hLiq(d.Tl,P);
  // condenser: exhaust steam, the heaters' high-level dumps and vent, the min-flow recirculation, bypass
  // steam and SJAE steam (both at the dome enthalpy) and the cleanup reject in, condensate down to the hotwell
  const hr=sat(Tm).hf,cd=condense(s.cd,[[wx,hx],[wduh,hdH],[wdul,hdL],[wv,sat(Thf(hdL)).hg],[nw.wr,hr],[wb,hg],[wsj,hg],[wrw,hrw]],dt,{w:s.wcw,T:Tcwi},af);
  const hcst=sat(s.Tcst).hf,hw=hotwellMix({M:s.Mh,T:s.Thw},[[cd.wl,sat(cd.Tc).hf],[wmu,hcst],[-nw.wcp,0]],dt);
  {const c=hotwellMix({M:s.Mcst,T:s.Tcst},[[nw.wrj,sat(s.Thw).hf],[-wmu-wcd,0]],dt);s.Mcst=c.M;s.Tcst=c.T}
  const mk=wmu-nw.wrj;
  s.Mh=hw.M;s.Thw=hw.T;
  // flows at the start-of-step pressure: steam leaves at h″, feedwater joins the water, heat into the water
  const mo=Math.min((wst+wsrv)*dt,0.5*s.M);
  // vessel steel: heat from (or to) the water, losses to the drywell
  const st=steelStep(s.Ts,d.Tl,dt);s.Ts=st.Ts;
  // suppression pool: the SRV steam condenses in it
  const pl=poolStep(s.pool,wsrv*Math.min(1,mo/((wst+wsrv)*dt||1)),hg,!!u.pc,dt);s.pool={M:pl.M,T:pl.T};
  let M=s.M+(fw+wcd-wrw)*dt-mo,U=s.U+fw*dt*hfw+wcd*dt*hcd-wrw*dt*hrw-mo*hg+(Q+st.Qw)*dt,Mv=Math.max(0,s.Mv-mo);
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
  // (the core's steaming is taken as the larger of its start- and end-of-step values: after a scram it falls
  // within a step, while the vessel still boils off the stored heat, which is not flashing)
  const gen=(e.Mv-Mv-mc)/dt,g=Math.max(0,gen-Math.max(gc0,k.W*k.xe))*TAUF;
  s.Mfl=g+(s.Mfl-g)*Math.exp(-dt/TAUF);
  s.Mb=Math.min(sat(e.T).rg*voidVol(k)+s.Mfl,0.9*e.Mv);
  Object.assign(s,{M,U,Mv:e.Mv});s.t+=dt;
  s.d=derive(s,e);
  s.f={Q,fw,Tfw,hfw,hg,wst,wsrv,nsrv,open,spr:s.spr.slice(),gen,cond:mc/dt,
    ms:s.ms.slice(),sv:s.sv,cv:s.cv,bp:s.bp,Pl:sl.Pl,wt,wb,
    n,fv:s.fv,sfv:s.sfv,wsu:nw.wsu,ext,Tt,pe:ht.pe,ps:ht.ps,ws,Pd,ret:cd.wl,Tret:cd.Tc,mk,Mh:s.Mh,hw:s.Mh/HWM,Thw:s.Thw,
    Pc:cd.Pc,Tc:cd.Tc,wc:cd.wc,Qc:cd.Q,Tcw:cd.Tout,wcw:s.wcw,Pgen,wx,hx,...tbF(line.ps,tg,tq,wk),
    Tcwi,cwp,cwv,pa:cd.pa,ya:cd.ya,Ma:s.cd.Ma,wsj,sj,vp,vb:s.vb,sjc:fsj*sjCap(Pm,pc0),vpc:vp?vpCap(pc0):0,
    wlk:af.wlk,wrad:af.wrad,wvb:af.wvb,wain:af.wi,waout:cd.wout,Vsj:af.Vsj,Vvp:af.Vvp,
    Ts:s.Ts,Qs:st.Qw,Ql:st.Ql,Tp:pl.T,Mp:pl.M,Lp:poolLevel(pl.M),Qpc:pl.Qc,pc:!!u.pc,lin:toIn(s.d.level),wrw,hrw,trip:pr,
    exl,pel:lp.pe,psl:lp.ps,wsl,Tlp:lp.T,Ps:nw.Ps,Pcd:nw.Pcd,wp:nw.wp,wr:nw.wr,wcp:nw.wcp,xr:s.xr,cp,fp,fpt:s.fpt,npsh:nw.A,phi:nw.phi,cav:nw.cav,
    Tm,wdh,wd:nw.wd,wduh,wdul,wv,LH:s.Mhs/MHS,LL:s.Mls/MLS,aH,aL,xdh:s.xdh,xdl:s.xdl,hdp,hdvf:!!s.fail.hdv,
    wmu,wrj:nw.wrj,xmu:s.xmu,xrj:s.xrj,Mcst:s.Mcst,cst:s.Mcst/CSTM,Tcst:s.Tcst,crd,wcd};
  coreF(s.f,k,rc);
};
// scram from heat q (W; default the last applied net heat, taken as core power; none if not positive);
// a second scram keeps the first; reset hands the heat back to the command
const scram=(s,q=s.f.Q)=>{if(!s.scram)s.scram={t:s.t,P0:Math.max(0,q)}};
// Protection signals from the plant state at the start of a step: dome pressure, level in inches above
// instrument zero, APRM (the fission power as a fraction of its rated value), MSIV and stop
// valve positions, turbine first-stage pressure (fraction of rated), shaft speed, the power–load unbalance,
// header and condenser pressures, main steam line flow fraction
const rpsSignals=s=>({P:s.d.P,lin:toIn(s.d.level),aprm:s.core?s.core.n*(1+DSUM)/QR:Math.max(0,s.f.Q)/QR,ms:Math.min(...s.ms),sv:s.sv,
  fsp:(s.f.p1||0)/P1R,spd:s.tg?s.tg.w:0,plu:!!(s.tg&&s.tg.plu),Pl:s.Pl,Pc:s.f.Pc,wst:(s.f.wst||0)/WR});
// reset: the protection resets every trip whose signal has cleared; the scram resets once no scram trip is
// left in, and the heat goes back to the command
const reset=s=>{rpsReset(s.rps,rpsSignals(s),s.t);if(!groups(s.rps).scram)s.scram=null};
// the reactor mode switch (Shutdown scrams)
const setMode=(s,m)=>{rpsMode(s.rps,m,s.t);const pr=rpsCheck(s.rps,rpsSignals(s),s.t);if(pr.scram&&!s.scram)scram(s,s.f.Q)};
// the operator's generator breaker switch: close (with the sync check) or open (a load rejection at power);
// returns '' or why it did not close ('sync', 'lock')
const breaker=(s,close)=>tgBreaker(s.tg,close,text=>s.rps.log.push({t:s.t,text}));
// the operator's scram button: logged with the protection's events
const manualScram=(s,q)=>{if(!s.scram)s.rps.log.push({t:s.t,text:'Manual scram'});scram(s,q)};
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
  if(s.Mcst>CSTM)return 'cst';
  if(s.f.Pc>PCMAX)return 'condenser';
  if(s.pool.T>TPMAX)return 'pool';
  if(s.core.pc)return 'prompt';
  return '';
};
// the condensate pump suction pressure the page reads for the shutoff-head message
const PC=PCD;
const API=14;
export {CST,UAS,UAL,TDW,INCH,IZ,LV,toIn,fromIn,TAUF,MPOOL,APOOL,TP0,UAP,TSW,TPMAX,PLIM,poolLevel};
export {UA0,RA,PATM,WLK,WRAD,VSJ,WSJ,VVP,WVB,PA0,PSB0,PSB1,PSJ0,PSJ1,PVP0,PVP1,pAir,yAir,blanket,sjSteam,sjCap,vpCap,cwTarget,TAUCW,motiveP};
export {PEL,KXL,TLPD,HC0,HF0,KCP,KPOL,KLP,KFP,ZS,HWD,NPR,PLO,TLO,WMINF as WMIN,KR,RRATE,network,shutoff,heaterL,train,suction,cpFlow,
  THPI,HDP0,KDPP,KDVL,KDVH,KDDH,KDDL,WSD,WSLD,WDD,VHS,VLS,LFL,LDU,XD0,MHS,MLS,DRATE,DPD,tubes,dumpX,dpFlow,fail,
  CSTM,HCST,CRATE,KMU,KRJ,muFlow,rjBack};
export {API,V,H,A,Z,QR,PR,TFR,SRV,NV,PB,BD,TAU,H0,KPU,KLN,KV,RSU,FRATE,NRATE,HWM,MKMAX,TMK,THW,PC,PCD,TTD,WR,PE,KXT,
  ETAG,VC,WCW,TCW,CPW,UA,PCMAX,expansion,condense,condSteady,breaker,
  turbineLine,stagePs,work,P1R,PXR,PLIR,ETAH,ETAL,NR,HIN,PW0F,TRH,IVRATE,PLU,SYNC,TRP,TK,PGR,lineAt,
  WTR,fwFlow,fwValve,heater,heaterT,heatBalance,subF,wSRV,decayFrac,init,step,scram,reset,fault,rpsSignals,setMode,manualScram,SP,MODES,TRIPS,GROUP,inHgVac,rpsActive,RWMAX,
  DPL,PL0,VWO,KT,BPF,KB,fcv,fcvInv,MSRATE,SVCLOSE,SVOPEN,CVRATE,BPRATE,steamLine,cvPos,
  NZ,STGT,GZ,RATED,steady,localFb,peak,fuelT,TREF,QC,WCR,BETA,LAMBDA,SRC,AV,AD,AM,MCF,RF,NG,NPOS,NOTCH,GW,XRATED,RODRATE,TSD,VSC,TSCRAM,WCRD,DSUM,DL,DF,VCH,AF,LC,VDC,voids,voidVol,cov,rodR,seq,rodsFor,WDR,HPR,HS,KPP,MGRATE,NMIN,TCD};
