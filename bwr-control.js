// Automatic control for the BWR page: no DOM, no plant equations.
// It reads the plant state (s.d, s.f) and the operator's settings, and produces the final command
// that bwr-sim.js step() takes. An ES module; imports only common.js (not bwr-sim.js). See docs/bwr.md.
//
// Settings c are in panel units: heat MW; n % feed pump speed; fv % feed valve; ext % heater
// extraction; mk kg/s hotwell makeup (negative: reject); mv % main steam valve; srv array, true for each
// SRV opened by hand; pset Pa; lset m; dpset Pa; auto {n,fv,mv,mk,rv} booleans. Net heat, heater extraction
// and the SRVs are always manual (the SRV springs are in the plant). The command is SI:
// {heat W, n 0–1, fv 0–1, ext 0–1, mk kg/s, mv 0–1, srv array, …}.
// Condensate and feed train (step 5): exl % LP heater extraction (manual; absent: the same as ext);
// cp, fp condensate and feed pump run switches (booleans, plain on/off, absent: on); rv % feed pump
// minimum-flow valve, with M/A.
// Heater drains (step 6): hdv % HP heater drain valve, ldv % LP drain pump discharge valve, both with M/A
// (a key absent from c.auto runs in A, for callers older than step 6; absent values are 50 %); hdp drain
// pump run switch (absent: on).
import {clamp} from './common.js';
// must match bwr-sim.js (the controller does not load the plant): flow demand limit, makeup limit
const FWMAX=2500,MKMAX=100;
// pressure regulator (main steam valve), PI on P − pset: 0.1 MPa over opens the valve 30 % more,
// integral time 30 s
const KP=3e-6,TIP=30;
// feedwater, three-element: main steam flow (feedforward) + PI on the level error, 800 kg/s per m,
// integral time 60 s. The gain is high enough that about 3 m low asks for full flow even with no
// steam-flow signal: after a main steam valve closure the steam leaves through the SRVs, which the
// steam-flow signal does not see, and the level term alone has to call for the feedwater.
const KL=800,TIL=60;
// The feed flow demand goes to a flow controller on the feed valve (cascade): PI, 0.0001 of stroke per
// kg/s of flow error, integral time 4 s. The valve's own stroke limit (10 %/s) is in the plant.
const KX=1e-4,TIX=4;
// hotwell level: makeup = condensate pump flow − condensate return (feedforward) + PI on the hotwell level error
// against a fixed half-full setpoint, 1000 kg/s per unit of fraction full (10 % low asks for the full
// 100 kg/s), integral time 300 s
const HSET=0.5,KH=1000,TIH=300;
// feed pump speed: holds the differential pressure between the pump discharge and the dome (dpset), as
// turbine-driven feed pumps do, so the feed valve keeps a steady pressure drop to work with at any vessel
// pressure. PI, 0.1 MPa of error moves the speed 0.15 %, integral time 10 s.
const KN=1.5e-8,TIN=10;
// feed pump minimum flow: the recirculation valve opens as the forward feed flow falls below WMIN (25 %
// of rated pump flow), fully at zero forward flow; proportional only, as the valve's own job is to keep
// the pump above its minimum flow. Must match bwr-feed.js WMIN.
const WMIN=447;
// heater shell levels (fraction full): PI on each drain valve, direct acting (high level opens it), 0.25
// of level error strokes the valve fully (gain 4), integral time 30 s, setpoint half full (LSH)
const KD=4,TID=30,LSH=0.5;
// controller memory: iP, iF, iX, iH integrators (steam valve fraction, feed flow kg/s, feed valve
// fraction, makeup kg/s); dem the latest demand for every automatic actuator (SI; fw is the flow demand
// the level controller passes to the flow controller)
// init(c) preloads the integrators from the operator's settings c (panel units), so an actuator that
// starts in A starts from its slider; the level loop's flow integral starts at 0 (feed = steam flow).
const init=(c)=>({iP:c?c.mv/100:0,iF:0,iX:c?c.fv/100:0,iH:c?c.mk:0,iN:c?c.n/100:0,iDH:(c?.hdv??50)/100,iDL:(c?.ldv??50)/100,dem:{mv:0,fw:0,fv:0,mk:0,n:0,rv:0,hdv:0.5,ldv:0.5}});
// the drain level loops are in A unless c.auto says otherwise
const autoD=(A,k)=>A[k]??true;
// Automatic logic runs all the time and produces a demand for every actuator (k.dem). For an actuator
// in M the integral holds the operator's value (net of the steam-flow feedforward for feedwater), so the
// demand is the operator's value plus the proportional correction for the present error: the notch shows
// which way, and how far, the controller would move it, and switching to A starts from there. (Holding
// the integral at operator value − proportional term would make the demand always equal the slider.)
const demands=(k,s,c,dt)=>{
  const d=s.d,f=s.f,A=c.auto||{};
  // pressure regulator
  const e=d.P-c.pset;
  if(!A.mv)k.iP=c.mv/100;
  else{const I=k.iP+KP/TIP*e*dt,v=KP*e+I;if(!((v>1&&e>0)||(v<0&&e<0)))k.iP=I}
  k.iP=clamp(k.iP,-1,1);const mv=clamp(KP*e+k.iP,0,1);
  // feedwater level, three-element: steam flow measured in the main steam line only (SRV discharge
  // is not). Its output is the flow demand F for the flow controller. In M the integral holds the
  // measured feed flow net of the feedforward, so F is the present flow plus the level correction.
  const eL=c.lset-d.level,ff=f.wst,fvA=!!A.fv;
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
  const eH=HSET-f.hw,fh=(f.wcp??f.fw)-f.ret; // what the condensate pump draws (feed + min-flow) less the condensate
  if(!A.mk)k.iH=c.mk-fh;
  else{const I=k.iH+KH/TIH*eH*dt,v=fh+KH*eH+I;if(!((v>MKMAX&&eH>0)||(v<-MKMAX&&eH<0)))k.iH=I}
  k.iH=clamp(k.iH,-2*MKMAX,2*MKMAX);const mk=clamp(fh+KH*eH+k.iH,-MKMAX,MKMAX);
  // feed pump speed on the discharge-to-dome differential pressure
  const eN=c.dpset-(f.Pd-d.P);
  if(!A.n)k.iN=c.n/100;
  else{const I=k.iN+KN/TIN*eN*dt,v=KN*eN+I;if(!((v>1&&eN>0)||(v<0&&eN<0)))k.iN=I}
  k.iN=clamp(k.iN,0,1);const n=clamp(KN*eN+k.iN,0,1);
  // minimum-flow valve on the measured forward feed flow
  const rv=clamp((WMIN-f.fw)/WMIN,0,1);
  // heater drain levels: HP shell on the HP drain valve, LP shell and drain tank on the drain pump's valve
  const lvl=(key,iK,L,man)=>{const e=(L??LSH)-LSH;
    if(!autoD(A,key))k[iK]=man;else{const I=k[iK]+KD/TID*e*dt,v=KD*e+I;if(!((v>1&&e>0)||(v<0&&e<0)))k[iK]=I}
    k[iK]=clamp(k[iK],0,1);return clamp(KD*e+k[iK],0,1)};
  const hdv=lvl('hdv','iDH',f.LH,(c.hdv??50)/100),ldv=lvl('ldv','iDL',f.LL,(c.ldv??50)/100);
  k.dem={mv,fw,fv,mk,n,rv,hdv,ldv};
};
// final command: the demand for actuators in A, the operator's value for those in M, in SI
const command=(k,c)=>{
  const A=c.auto||{},D=k.dem;
  return {heat:c.heat*1e6,n:A.n?D.n:c.n/100,fv:A.fv?D.fv:c.fv/100,ext:c.ext/100,mk:A.mk?D.mk:c.mk,mv:A.mv?D.mv:c.mv/100,srv:c.srv||[],
    exl:(c.exl??c.ext)/100,cp:c.cp!==false,fp:c.fp!==false,rv:A.rv?D.rv:(c.rv||0)/100,
    hdv:autoD(A,'hdv')?D.hdv:(c.hdv??50)/100,ldv:autoD(A,'ldv')?D.ldv:(c.ldv??50)/100,hdp:c.hdp!==false};
};
// one control step: update the demands from the plant state, return the command for bwr-sim.js step().
// Call it before every plant step, with the same dt.
const update=(k,s,c,dt)=>{demands(k,s,c,dt);return command(k,c)};
export {init,demands,command,update,KP,TIP,KL,TIL,KX,TIX,HSET,KH,TIH,KN,TIN,WMIN,KD,TID,LSH};
