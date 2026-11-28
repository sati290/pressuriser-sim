// Automatic control for the BWR page: no DOM, no plant equations.
// It reads the plant state (s.d, s.f) and the operator's settings, and produces the final command
// that bwr-sim.js step() takes. An ES module; imports only common.js (not bwr-sim.js). See docs/bwr.md.
//
// Settings c are in panel units: heat MW; n % feed pump speed; fv % feed valve; ext % heater
// extraction; mk kg/s hotwell makeup (negative: reject); mv % main steam valve; srv array, true for each
// SRV opened by hand; pset Pa; lset m; auto {fv,mv,mk} booleans. Net heat, pump speed, heater extraction
// and the SRVs are always manual (the SRV springs are in the plant). The command is SI:
// {heat W, n 0–1, fv 0–1, ext 0–1, mk kg/s, mv 0–1, srv array}.
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
// hotwell level: makeup = feed − condensate return (feedforward) + PI on the hotwell level error
// against a fixed half-full setpoint, 1000 kg/s per unit of fraction full (10 % low asks for the full
// 100 kg/s), integral time 300 s
const HSET=0.5,KH=1000,TIH=300;
// controller memory: iP, iF, iX, iH integrators (steam valve fraction, feed flow kg/s, feed valve
// fraction, makeup kg/s); dem the latest demand for every automatic actuator (SI; fw is the flow demand
// the level controller passes to the flow controller)
const init=()=>({iP:0,iF:0,iX:0,iH:0,dem:{mv:0,fw:0,fv:0,mk:0}});
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
  const eH=HSET-f.hw,fh=f.fw-f.ret;
  if(!A.mk)k.iH=c.mk-fh;
  else{const I=k.iH+KH/TIH*eH*dt,v=fh+KH*eH+I;if(!((v>MKMAX&&eH>0)||(v<-MKMAX&&eH<0)))k.iH=I}
  k.iH=clamp(k.iH,-2*MKMAX,2*MKMAX);const mk=clamp(fh+KH*eH+k.iH,-MKMAX,MKMAX);
  k.dem={mv,fw,fv,mk};
};
// final command: the demand for actuators in A, the operator's value for those in M, in SI
const command=(k,c)=>{
  const A=c.auto||{},D=k.dem;
  return {heat:c.heat*1e6,n:c.n/100,fv:A.fv?D.fv:c.fv/100,ext:c.ext/100,mk:A.mk?D.mk:c.mk,mv:A.mv?D.mv:c.mv/100,srv:c.srv||[]};
};
// one control step: update the demands from the plant state, return the command for bwr-sim.js step().
// Call it before every plant step, with the same dt.
const update=(k,s,c,dt)=>{demands(k,s,c,dt);return command(k,c)};
export {init,demands,command,update,KP,TIP,KL,TIL,KX,TIX,HSET,KH,TIH};
