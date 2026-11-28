// Automatic control for the BWR page: no DOM, no plant equations.
// It reads the plant state (s.d, s.f) and the operator's settings, and produces the final command
// that bwr-sim.js step() takes. An ES module; imports only common.js (not bwr-sim.js). See docs/bwr.md.
//
// Settings c are in panel units: rods array of the control rod group positions, 0 (00, fully in) … 48 (fully
// out) (manual only: BWRs have no automatic rod control; an undefined entry leaves that group where it is);
// crd CRD pump run switch (boolean, absent: stopped); heat MW (optional fixed core heat, a test hook; leave it out for the kinetics); n % feed
// pump speed; fv % feed valve; ext % heater extraction; mk kg/s net hotwell makeup (older callers, see step 7); cv %
// turbine control valve; bp % bypass valve; ms1, ms2, sv booleans (inboard/outboard MSIV and turbine stop
// valve switches, true open); ll % load limit (of rated steam flow); srv array, true for each SRV opened by
// hand; pset Pa; lset m; dpset Pa; rw % reactor water cleanup reject; rp recirculation pump breaker (true
// closed; a plain switch, no M/A); mg % MG set speed; wset % rated core flow setpoint; auto
// {n,fv,cv,bp,mk,rw,mg,rv,hdv,ldv,mu,rj} booleans; cwp, sj, vp, vb booleans, cwv % and tcw °C for the circulating water and
// condenser air removal (manual only). The rods, heater extraction, the MSIV and stop valve switches and the
// SRVs are always manual (the SRV springs are in the plant). The command is SI:
// {rods array 0–1 (withdrawn fraction), crd, heat W (undefined unless c.heat is set), rp, mg 0–1 (undefined unless c.mg is set), n 0–1, fv 0–1,
// ext 0–1, ms1, ms2, sv, cv 0–1, bp 0–1, rw 0–1, srv array, cwp, cwv 0–1, tcw K, sj, vp, vb}.
// Condensate and feed train (step 5): exl % LP heater extraction (manual; absent: the same as ext);
// cp, fp condensate and feed pump run switches (booleans, plain on/off, absent: on); rv % feed pump
// minimum-flow valve, with M/A.
// Heater drains (step 6): hdv % HP heater drain valve, ldv % LP drain pump discharge valve, both with M/A
// (a key absent from c.auto runs in A, for callers older than step 6; absent values are 50 %); hdp drain
// pump run switch (absent: on).
// Condensate storage tank (step 7): mu % makeup valve (CST → condenser), rj % reject valve (condensate pump
// discharge → CST), each with M/A; they replace mk. The hotwell level loop is split range: its one output,
// in kg/s of net makeup, opens the makeup valve above 0 and the reject valve below (1 % of stroke per kg/s,
// about the valves' flow at a 70 % full CST). Keys absent from c.auto follow auto.mk, absent values follow
// mk (positive: makeup, negative: reject), for callers older than step 7.
// Turbine and generator (step 15): mw MWe load set, nset rpm speed set (both optional: without mw the
// load channel does not limit, without nset the speed set is the rated 1800 rpm). The generator breaker
// is an operator action on the plant (bwr-sim.js breaker()), not a command.
import {clamp} from './common.js';
// must match bwr-sim.js (the controller does not load the plant): flow demand limit, makeup limit
const FWMAX=2500,MKMAX=100;
// pressure regulator (GE EHC scheme, simplified): PI on dome pressure P − pset gives a steam flow demand D
// as a fraction of rated steam flow; 0.1 MPa over asks for 34 % of rated more, integral time 30 s. (A
// real EHC senses throttle pressure at the turbine; here the dome.) The control valves get min(D, load
// limit); the bypass gets what the turbine is not taking: D minus the turbine flow reference, which is the
// control valves' flow demand times the measured stop valve position (so a turbine trip hands the whole
// demand to the bypass, up to its capacity). (Measured turbine flow in its place lags a step and, with
// the page's 2 s step, makes the two valves chase each other.) Each flow is turned into a valve position with the valve's
// characteristic at the rated throttle pressure (no pressure compensation). These must match
// bwr-steam.js (the tests check): rated steam WR, rated throttle pressure PL0, turbine valve constant KT,
// bypass capacity BPF of rated.
const KP=3.4e-6,TIP=30;
const WR=1788.0622120891583,PL0=6.7e6,KT=2.815429037009538e-4,BPF=0.25;
const fcvInv=f=>1-(1-clamp(f,0,1))**(2/3),fcv=x=>1-(1-clamp(x,0,1))**1.5;
// Speed/load channel of the EHC, the third limit on the control valves beside the pressure regulator and
// the load limit (the valves take the lowest of the three; the bypass the rest of the pressure regulator's
// demand). Breaker closed: the load reference ramps towards the load set at LRATE (5 MWe/s, quicker than a
// plant would load, for the sandbox) and becomes a steam flow demand through MWR, the gross output per unit of
// rated steam flow at rated, plus a slow MW feedback trim (integral time TIMW 20 s, ±15 %, only while this
// channel is the one holding the valves). Breaker open: a proportional speed governor, droop DROOP 5 %
// (5 % overspeed shuts the valves), on the speed reference, which ramps towards the speed set at ACC
// (0.5 %/s, 9 rpm/s) and follows the actual speed while the stop valve is shut. The shaft is fast (inertia
// constant HIN 5 s in the plant) against the controller's step, so the governor's gain is cut to HIN/dt at
// large steps (2.5 at the page's 2 s step: 40 % droop), which keeps the loop stable. Must match
// bwr-sim.js: NR, HIN, MWR (the tests check). Illustrative settings.
const NR=1800,HIN=5,MWR=1115e6,LRATE=5e6,TIMW=20,DROOP=0.05,ACC=0.005;
// flow fraction of rated that the control valve passes at lift x at the rated throttle pressure
const cvFlow=x=>KT*PL0*fcv(x)/WR,cvLift=D=>fcvInv(D*WR/(KT*PL0));
// feedwater, three-element: main steam flow (feedforward) + PI on the level error, 800 kg/s per m,
// integral time 60 s. The gain is high enough that about 3 m low asks for full flow even with no
// steam-flow signal: after a main steam valve closure the steam leaves through the SRVs, which the
// steam-flow signal does not see, and the level term alone has to call for the feedwater.
const KL=800,TIL=60;
// scram level setpoint setdown (as in GE feedwater level control): after a scram the level setpoint drops
// by SDL, so the controller does not refill the vessel on the shrink and overshoot to L8 (the feed would then
// trip); after SDT seconds it ramps back over SDR seconds (cold feed brought in fast collapses the pressure).
// Illustrative: 0.45 m (about 18 in), held 10 min, back over 20 min.
const SDL=0.45,SDT=600,SDR=1200;
// The setdown starts with the scram and runs its course in the controller's memory k.sdt (the scram time),
// so a reset of the scram does not snap the setpoint back up. setdown(k, s) the present setdown, m.
const setdown=(k,s)=>{if(s.scram&&k.sdt!==s.scram.t)k.sdt=s.scram.t;if(k.sdt==null)return 0;
  const x=SDL*clamp(1-(s.t-k.sdt-SDT)/SDR,0,1);if(x<=0&&!s.scram)k.sdt=null;return x};
// The feed flow demand goes to a flow controller on the feed valve (cascade): PI on the error in √flow, with
// the gain that gives 0.0001 of stroke per kg/s of flow error at rated flow, integral time 4 s. The valve's own stroke limit (10 %/s) is in the plant.
const KX=1e-4,TIX=4,KXS=KX*2*Math.sqrt(WR);
// hotwell level: makeup = condensate pump flow − condensate return (feedforward) + PI on the hotwell level error
// against a fixed half-full setpoint, 1000 kg/s per unit of fraction full (10 % low asks for the full
// 100 kg/s), integral time 300 s
const HSET=0.5,KH=1000,TIH=300;
// feed pump speed: holds the differential pressure between the pump discharge and the dome (dpset), as
// turbine-driven feed pumps do, so the feed valve keeps a steady pressure drop to work with at any vessel
// pressure. PI, 0.1 MPa of error moves the speed 0.15 %, integral time 10 s.
const KN=1.5e-8,TIN=10;
// reactor water cleanup reject: proportional only, opening from RDB above the level setpoint to fully open
// RBAND higher. Its band sits above the feedwater controller's setpoint, so the two do not fight: the feed
// valve closes as the level rises and the reject takes the swell of a heat-up.
const RDB=0.1,RBAND=0.3;
// core flow (recirculation MG set speed): PI on the core flow against the setpoint wset (% of the rated
// WCR, which must match bwr-core.js): 10 % of rated flow low raises the speed demand 3 %, integral time
// 5 s; anti-windup at 0 and 1. The MG set's speed limiter (2 %/s, minimum 20 %) is in the plant, so a
// big setpoint change ramps.
const WCR=12900,KW=2.3e-5,TIW=5;
// feed pump minimum flow: the recirculation valve opens as the forward feed flow falls below WMIN (25 %
// of rated pump flow), fully at zero forward flow; proportional only, as the valve's own job is to keep
// the pump above its minimum flow. Must match bwr-feed.js WMIN.
const WMIN=447;
// heater shell levels (fraction full): PI on each drain valve, direct acting (high level opens it), 0.25
// of level error strokes the valve fully (gain 4), integral time 30 s, setpoint half full (LSH)
const KD=4,TID=30,LSH=0.5;
// controller memory: iP, iF, iX, iH, iN, iW integrators (steam flow demand as a fraction of rated, feed flow kg/s, feed valve
// fraction, makeup kg/s); dem the latest demand for every automatic actuator (SI; fw is the flow demand
// the level controller passes to the flow controller)
// init(c) preloads the integrators from the operator's settings c (panel units), so an actuator that
// starts in A starts from its slider; the level loop's flow integral starts at 0 (feed = steam flow).
const init=(c)=>({Lr:null,nr:null,iL:0,iP:c?cvFlow(c.cv/100)+BPF*c.bp/100:0,iF:0,iX:c?c.fv/100:0,iH:c?manMu(c)-manRj(c):0,iN:c?c.n/100:0,iW:c&&c.mg!=null?c.mg/100:1,iDH:(c?.hdv??50)/100,iDL:(c?.ldv??50)/100,
  dem:{D:0,cv:0,bp:0,fw:0,fv:0,mk:0,n:0,rw:0,mg:1,rv:0,hdv:0.5,ldv:0.5,mu:0,rj:0}});
// the drain level loops are in A unless c.auto says otherwise
const autoD=(A,k)=>A[k]??true;
// the makeup and reject valves follow the old makeup switch when their own keys are absent; the operator's
// openings (%, read as kg/s of net makeup) likewise follow mk
const autoM=(A,k)=>!!(A[k]??A.mk);
const manMu=c=>c.mu??Math.max(0,c.mk||0),manRj=c=>c.rj??Math.max(0,-(c.mk||0));
// Automatic logic runs all the time and produces a demand for every actuator (k.dem). For an actuator
// in M the integral holds the operator's value (net of the steam-flow feedforward for feedwater), so the
// demand is the operator's value plus the proportional correction for the present error: the notch shows
// which way, and how far, the controller would move it, and switching to A starts from there. (Holding
// the integral at operator value − proportional term would make the demand always equal the slider.)
const demands=(k,s,c,dt)=>{
  const d=s.d,f=s.f,A=c.auto||{};
  // pressure regulator: flow demand D, split between the control valves (up to the load limit) and the
  // bypass. With both valves in M the integral holds what the two sliders pass. Anti-windup: the integral
  // stops rising when no valve in A can pass more (control valve at the load limit, in M, or its stop
  // valve not open; bypass fully open or in M), and stops falling at D = 0.
  // speed/load channel SL (fraction of rated steam flow)
  const brk=f.brk??true,spd=f.spd??1;let SL=Infinity;
  if(brk){k.nr=1;
    if(c.mw==null){k.Lr=f.Pgen??0;k.iL=0}
    else{if(k.Lr==null)k.Lr=f.Pgen??0;k.Lr+=clamp(c.mw*1e6-k.Lr,-LRATE*dt,LRATE*dt);SL=Math.max(0,k.Lr/MWR+k.iL)}}
  else{k.Lr=0;k.iL=0;const ns=(c.nset??NR)/NR;
    if(k.nr==null||(f.sv??1)<0.99)k.nr=spd;else k.nr+=clamp(ns-k.nr,-ACC*dt,ACC*dt);
    SL=Math.max(0,Math.min(1/DROOP,HIN/dt)*(k.nr-spd))}
  const e=d.P-c.pset,LL=Math.min((c.ll??105)/100,SL),sv=f.sv??1,ref=x=>(A.cv?Math.min(Math.max(0,x),LL):cvFlow(c.cv/100))*sv;
  if(!A.cv&&!A.bp)k.iP=cvFlow(c.cv/100)+BPF*c.bp/100;
  else{const I=k.iP+KP/TIP*e*dt,v=KP*e+I;
    const cvSat=!A.cv||v>=LL||sv<0.99,bpSat=!A.bp||v-ref(v)>=BPF;
    // high: hold while saturated; low: fall only as far as D = 0 (holding instead would pin the integral
    // wherever it was when D first reached 0, and the pressure would settle below the setpoint)
    if(v<0&&e<0)k.iP=Math.max(I,Math.min(k.iP,-KP*e));else if(!(cvSat&&bpSat&&e>0))k.iP=I}
  k.iP=clamp(k.iP,-0.5,Math.min(LL,1.2)+BPF+0.1);const D=Math.max(0,KP*e+k.iP);
  // MW trim: only while the load channel holds the valves below both the pressure regulator and the load limit
  if(brk&&c.mw!=null&&A.cv&&SL<D&&SL<(c.ll??105)/100&&sv>=0.99)k.iL=clamp(k.iL+(k.Lr-(f.Pgen??0))/MWR/TIMW*dt,-0.15,0.15);
  const cv=cvLift(Math.min(D,LL)),bp=clamp((D-ref(D))/BPF,0,1);
  // feedwater level, three-element: steam flow measured in the main steam line only (SRV discharge
  // is not). Its output is the flow demand F for the flow controller. In M the integral holds the
  // measured feed flow net of the feedforward, so F is the present flow plus the level correction.
  const lset=c.lset-setdown(k,s),eL=lset-d.level,ff=f.wst,fvA=!!A.fv;
  // the flow controller's valve demand before this step's level update, for anti-windup
  const sat1=k.dem.fv>=1&&fvA,sat0=k.dem.fv<=0&&fvA;
  if(!fvA)k.iF=f.fw-ff;
  else{const I=k.iF+KL/TIL*eL*dt,v=ff+KL*eL+I;if(!((v>FWMAX||sat1)&&eL>0||(v<0||sat0)&&eL<0))k.iF=I}
  k.iF=clamp(k.iF,-FWMAX,FWMAX);const fw=clamp(ff+KL*eL+k.iF,0,FWMAX);
  // flow controller on the feed valve
  // characterised on √flow: the quadratic valve's flow goes nearly as the square of its opening, so an error
  // in flow would give the loop a gain that vanishes near shut (it would close ever more slowly and leak cold
  // feed into an isolated vessel); in √flow the loop gain is about the same at every flow. KXS matches KX at rated.
  const eF=KXS*(Math.sqrt(fw)-Math.sqrt(Math.max(0,f.fw)));
  if(!fvA)k.iX=c.fv/100;
  else{const I=k.iX+eF/TIX*dt,v=eF+I;if(!((v>1&&eF>0)||(v<0&&eF<0)))k.iX=I}
  k.iX=clamp(k.iX,0,1);const fv=clamp(eF+k.iX,0,1);
  // hotwell level, split range onto the makeup and reject valves
  // feedforward: what the condensate pump sends on through the LP heater (feed + min-flow less the pumped
  // drains; not the reject, which is this loop's own output) less the condensate
  const eH=HSET-f.hw,fh=(f.wcp??f.fw)-(f.wrj||0)-f.ret;
  const aMu=autoM(A,'mu'),aRj=autoM(A,'rj');
  if(!aMu&&!aRj)k.iH=manMu(c)-manRj(c)-fh;
  else{const I=k.iH+KH/TIH*eH*dt,v=fh+KH*eH+I;if(!((v>MKMAX&&eH>0)||(v<-MKMAX&&eH<0)))k.iH=I}
  k.iH=clamp(k.iH,-2*MKMAX,2*MKMAX);const mk=clamp(fh+KH*eH+k.iH,-MKMAX,MKMAX);
  // feed pump speed on the discharge-to-dome differential pressure
  const eN=c.dpset-(f.Pd-d.P);
  // while the pump is stopped or tripped it makes no head: the integral tracks the speed it runs at
  if(!A.n)k.iN=c.n/100;else if(c.fp===false||f.fpt||f.trip?.fwp)k.iN=f.n??0;
  else{const I=k.iN+KN/TIN*eN*dt,v=KN*eN+I;if(!((v>1&&eN>0)||(v<0&&eN<0)))k.iN=I}
  k.iN=clamp(k.iN,0,1);const n=clamp(KN*eN+k.iN,0,1);
  // core flow on the MG set speed (only when the page gives a flow setpoint)
  let mg=k.dem.mg;
  if(c.wset!=null){const eW=c.wset/100*WCR-(f.Wc||0);
    // breaker open: the pump makes no flow, the integral tracks the MG set's actual speed
    if(!A.mg)k.iW=c.mg/100;else if(f.rp===false)k.iW=f.ng??k.iW;
    else{const I=k.iW+KW/TIW*eW*dt,v=KW*eW+I;if(!((v>1&&eW>0)||(v<0&&eW<0)))k.iW=I}
    k.iW=clamp(k.iW,0,1);mg=clamp(KW*eW+k.iW,0,1)}
  const rw=clamp((d.level-c.lset-RDB)/RBAND,0,1);
  // minimum-flow valve on the measured forward feed flow
  const rv=clamp((WMIN-f.fw)/WMIN,0,1);
  // heater drain levels: HP shell on the HP drain valve, LP shell and drain tank on the drain pump's valve
  const lvl=(key,iK,L,man)=>{const e=(L??LSH)-LSH;
    if(!autoD(A,key))k[iK]=man;else{const I=k[iK]+KD/TID*e*dt,v=KD*e+I;if(!((v>1&&e>0)||(v<0&&e<0)))k[iK]=I}
    k[iK]=clamp(k[iK],0,1);return clamp(KD*e+k[iK],0,1)};
  const hdv=lvl('hdv','iDH',f.LH,(c.hdv??50)/100),ldv=lvl('ldv','iDL',f.LL,(c.ldv??50)/100);
  const mu=clamp(mk/MKMAX,0,1),rj=clamp(-mk/MKMAX,0,1);
  k.dem={D,SL,Lr:k.Lr,nr:k.nr,cv,bp,fw,fv,mk,n,rw,mg,rv,hdv,ldv,mu,rj};
};
// final command: the demand for actuators in A, the operator's value for those in M, in SI
const command=(k,c)=>{
  const A=c.auto||{},D=k.dem;
  const u={heat:c.heat!=null?c.heat*1e6:undefined,rods:c.rods?c.rods.map(p=>p==null?undefined:p/48):undefined,crd:!!c.crd,rp:c.rp,mg:c.mg!=null?(A.mg?D.mg:c.mg/100):undefined,n:A.n?D.n:c.n/100,fv:A.fv?D.fv:c.fv/100,ext:c.ext/100,
    ms1:!!c.ms1,ms2:!!c.ms2,sv:!!c.sv,cv:A.cv?D.cv:c.cv/100,bp:A.bp?D.bp:c.bp/100,rw:A.rw?D.rw:(c.rw||0)/100,srv:c.srv||[],
    exl:(c.exl??c.ext)/100,cp:c.cp!==false,fp:c.fp!==false,rv:A.rv?D.rv:(c.rv||0)/100,
    hdv:autoD(A,'hdv')?D.hdv:(c.hdv??50)/100,ldv:autoD(A,'ldv')?D.ldv:(c.ldv??50)/100,hdp:c.hdp!==false,
    mu:autoM(A,'mu')?D.mu:manMu(c)/100,rj:autoM(A,'rj')?D.rj:manRj(c)/100};
  // circulating water and condenser air removal: manual only, passed through (absent: the plant's normal
  // line-up). cwp CW pump, sj SJAE motive steam, vp vacuum pump, vb vacuum breaker (booleans); cwv % CW
  // valve; tcw °C CW inlet temperature (an operator input standing for the river or tower).
  for(const k of ['cwp','sj','vp','vb'])if(c[k]!=null)u[k]=!!c[k];
  if(c.cwv!=null)u.cwv=c.cwv/100;if(c.tcw!=null)u.tcw=c.tcw+273.15;
  return u;
};
// one control step: update the demands from the plant state, return the command for bwr-sim.js step().
// Call it before every plant step, with the same dt.
const update=(k,s,c,dt)=>{demands(k,s,c,dt);return command(k,c)};
export {NR,HIN,MWR,LRATE,TIMW,DROOP,ACC,init,demands,command,update,WCR,KW,TIW,KP,TIP,WR,PL0,KT,BPF,cvFlow,cvLift,KL,TIL,KX,TIX,HSET,KH,TIH,KN,TIN,KXS,RDB,RBAND,SDL,SDT,SDR,setdown,WMIN,KD,TID,LSH};
