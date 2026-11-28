// Automatic control for the BWR page: no DOM, no plant equations.
// It reads the plant state (s.d, s.f) and the operator's settings, and produces the final command
// that bwr-sim.js step() takes. An ES module; imports only common.js (not bwr-sim.js). See docs/bwr.md.
//
// Settings c are in panel units: rod % control rod lever withdrawn (manual only: BWRs have no automatic
// rod control); heat MW (optional fixed core heat, a test hook; leave it out for the kinetics); n % feed
// pump speed; fv % feed valve; ext % heater extraction; mk kg/s hotwell makeup (negative: reject); cv %
// turbine control valve; bp % bypass valve; ms1, ms2, sv booleans (inboard/outboard MSIV and turbine stop
// valve switches, true open); ll % load limit (of rated steam flow); srv array, true for each SRV opened by
// hand; pset Pa; lset m; dpset Pa; rw % reactor water cleanup reject; rp recirculation pump breaker (true
// closed; a plain switch, no M/A); mg % MG set speed; wset % rated core flow setpoint; auto
// {n,fv,cv,bp,mk,rw,mg} booleans; cwp, sj, vp, vb booleans, cwv % and tcw °C for the circulating water and
// condenser air removal (manual only). The rods, heater extraction, the MSIV and stop valve switches and the
// SRVs are always manual (the SRV springs are in the plant). The command is SI:
// {rod 0–1, heat W (undefined unless c.heat is set), rp, mg 0–1 (undefined unless c.mg is set), n 0–1, fv 0–1,
// ext 0–1, mk kg/s, ms1, ms2, sv, cv 0–1, bp 0–1, rw 0–1, srv array, cwp, cwv 0–1, tcw K, sj, vp, vb}.
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
const setdown=(s,t)=>s.scram?SDL*clamp(1-(s.t-s.scram.t-SDT)/SDR,0,1):0;
// The feed flow demand goes to a flow controller on the feed valve (cascade): PI, 0.0001 of stroke per
// kg/s of flow error, integral time 4 s. The valve's own stroke limit (10 %/s) is in the plant.
const KX=1e-4,TIX=4;
// hotwell level: makeup = feed − condensate return (feedforward) + PI on the hotwell level error
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
// controller memory: iP, iF, iX, iH, iN, iW integrators (steam flow demand as a fraction of rated, feed flow kg/s, feed valve
// fraction, makeup kg/s); dem the latest demand for every automatic actuator (SI; fw is the flow demand
// the level controller passes to the flow controller)
// init(c) preloads the integrators from the operator's settings c (panel units), so an actuator that
// starts in A starts from its slider; the level loop's flow integral starts at 0 (feed = steam flow).
const init=(c)=>({iP:c?cvFlow(c.cv/100)+BPF*c.bp/100:0,iF:0,iX:c?c.fv/100:0,iH:c?c.mk:0,iN:c?c.n/100:0,iW:c&&c.mg!=null?c.mg/100:1,dem:{D:0,cv:0,bp:0,fw:0,fv:0,mk:0,n:0,rw:0,mg:1}});
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
  const e=d.P-c.pset,LL=(c.ll??105)/100,sv=f.sv??1,ref=x=>(A.cv?Math.min(Math.max(0,x),LL):cvFlow(c.cv/100))*sv;
  if(!A.cv&&!A.bp)k.iP=cvFlow(c.cv/100)+BPF*c.bp/100;
  else{const I=k.iP+KP/TIP*e*dt,v=KP*e+I;
    const cvSat=!A.cv||v>=LL||sv<0.99,bpSat=!A.bp||v-ref(v)>=BPF;
    // high: hold while saturated; low: fall only as far as D = 0 (holding instead would pin the integral
    // wherever it was when D first reached 0, and the pressure would settle below the setpoint)
    if(v<0&&e<0)k.iP=Math.max(I,Math.min(k.iP,-KP*e));else if(!(cvSat&&bpSat&&e>0))k.iP=I}
  k.iP=clamp(k.iP,-0.5,LL+BPF+0.1);const D=Math.max(0,KP*e+k.iP);
  const cv=cvLift(Math.min(D,LL)),bp=clamp((D-ref(D))/BPF,0,1);
  // feedwater level, three-element: steam flow measured in the main steam line only (SRV discharge
  // is not). Its output is the flow demand F for the flow controller. In M the integral holds the
  // measured feed flow net of the feedforward, so F is the present flow plus the level correction.
  const lset=c.lset-setdown(s),eL=lset-d.level,ff=f.wst,fvA=!!A.fv;
  // the flow controller's valve demand before this step's level update, for anti-windup
  const sat1=k.dem.fv>=1&&fvA,sat0=k.dem.fv<=0&&fvA;
  if(!fvA)k.iF=f.fw-ff;
  else{const I=k.iF+KL/TIL*eL*dt,v=ff+KL*eL+I;if(!((v>FWMAX||sat1)&&eL>0||(v<0||sat0)&&eL<0))k.iF=I}
  k.iF=clamp(k.iF,-FWMAX,FWMAX);const fw=clamp(ff+KL*eL+k.iF,0,FWMAX);
  // flow controller on the feed valve
  const eF=fw-f.fw;
  if(!fvA)k.iX=c.fv/100;
  else{const I=k.iX+KX/TIX*eF*dt,v=KX*eF+I;if(!((v>1&&eF>0)||(v<0&&eF<0)))k.iX=I}
  k.iX=clamp(k.iX,0,1);const fv=clamp(KX*eF+k.iX,0,1);
  // hotwell makeup
  const eH=HSET-f.hw,fh=f.fw-f.ret;
  if(!A.mk)k.iH=c.mk-fh;
  else{const I=k.iH+KH/TIH*eH*dt,v=fh+KH*eH+I;if(!((v>MKMAX&&eH>0)||(v<-MKMAX&&eH<0)))k.iH=I}
  k.iH=clamp(k.iH,-2*MKMAX,2*MKMAX);const mk=clamp(fh+KH*eH+k.iH,-MKMAX,MKMAX);
  // feed pump speed on the discharge-to-dome differential pressure
  const eN=c.dpset-(f.Pd-d.P);
  if(!A.n)k.iN=c.n/100;
  else{const I=k.iN+KN/TIN*eN*dt,v=KN*eN+I;if(!((v>1&&eN>0)||(v<0&&eN<0)))k.iN=I}
  k.iN=clamp(k.iN,0,1);const n=clamp(KN*eN+k.iN,0,1);
  // core flow on the MG set speed (only when the page gives a flow setpoint)
  let mg=k.dem.mg;
  if(c.wset!=null){const eW=c.wset/100*WCR-(f.Wc||0);
    if(!A.mg)k.iW=c.mg/100;
    else{const I=k.iW+KW/TIW*eW*dt,v=KW*eW+I;if(!((v>1&&eW>0)||(v<0&&eW<0)))k.iW=I}
    k.iW=clamp(k.iW,0,1);mg=clamp(KW*eW+k.iW,0,1)}
  const rw=clamp((d.level-c.lset-RDB)/RBAND,0,1);
  k.dem={D,cv,bp,fw,fv,mk,n,rw,mg};
};
// final command: the demand for actuators in A, the operator's value for those in M, in SI
const command=(k,c)=>{
  const A=c.auto||{},D=k.dem;
  const u={heat:c.heat!=null?c.heat*1e6:undefined,rod:c.rod!=null?c.rod/100:undefined,rp:c.rp,mg:c.mg!=null?(A.mg?D.mg:c.mg/100):undefined,n:A.n?D.n:c.n/100,fv:A.fv?D.fv:c.fv/100,ext:c.ext/100,mk:A.mk?D.mk:c.mk,
    ms1:!!c.ms1,ms2:!!c.ms2,sv:!!c.sv,cv:A.cv?D.cv:c.cv/100,bp:A.bp?D.bp:c.bp/100,rw:A.rw?D.rw:(c.rw||0)/100,srv:c.srv||[]};
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
export {init,demands,command,update,WCR,KW,TIW,KP,TIP,WR,PL0,KT,BPF,cvFlow,cvLift,KL,TIL,KX,TIX,HSET,KH,TIH,KN,TIN,RDB,RBAND,SDL,SDT,SDR,setdown};
