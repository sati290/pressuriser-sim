// Automatic control for the BWR page: no DOM, no plant equations.
// It reads the plant state (s.d, s.f) and the operator's settings, and produces the final command
// that BWRSim.step() takes. Loaded after common.js (it does not need bwr-sim.js), as a classic
// <script> (bwr.html) or a module (tests); it only defines globalThis.BWRCtl. See docs/bwr.md.
//
// Settings c are in panel units: heat MW; fw kg/s; tfw °C; mv % main steam valve; srv SRVs open;
// pset, relSet Pa; lset m; auto {fw,mv,srv} booleans. Net heat and feedwater temperature are
// always manual. The command is SI: {heat W, fw kg/s, tfw K, mv 0–1, srv count}.
globalThis.BWRCtl = (() => {
'use strict';
const {clamp}=globalThis.PZCommon;
// must match bwr-sim.js (the controller does not load the plant): SRV count and feedwater pump limit
const NV=11,FWMAX=2500;
// pressure regulator (main steam valve), PI on P − pset: 0.1 MPa over opens the valve 30 % more,
// integral time 30 s
const KP=3e-6,TIP=30;
// feedwater, three-element: main steam flow (feedforward) + PI on the level error, 150 kg/s per m,
// integral time 120 s
const KL=150,TIL=120;
// SRV relief mode: three groups (valves) opening at relSet, +0.07 and +0.14 MPa; each recloses RESEAT below its opening pressure
const GROUPS=[4,4,3],GSTEP=0.07e6,RESEAT=0.35e6;
// controller memory: iP, iF integrators (valve fraction, kg/s); g open state of each SRV group;
// dem the latest demand for every actuator (SI)
const init=()=>({iP:0,iF:0,g:GROUPS.map(()=>false),dem:{mv:0,fw:0,srv:0}});
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
  // feedwater: steam flow measured in the main steam line only (SRV discharge is not)
  const eL=c.lset-d.level,ff=f.wst;
  if(!A.fw)k.iF=c.fw-ff;
  else{const I=k.iF+KL/TIL*eL*dt,v=ff+KL*eL+I;if(!((v>FWMAX&&eL>0)||(v<0&&eL<0)))k.iF=I}
  k.iF=clamp(k.iF,-FWMAX,FWMAX);const fw=clamp(ff+KL*eL+k.iF,0,FWMAX);
  // SRV groups with hysteresis
  let n=0;GROUPS.forEach((m,i)=>{const po=c.relSet+i*GSTEP;if(d.P>po)k.g[i]=true;else if(d.P<po-RESEAT)k.g[i]=false;if(k.g[i])n+=m});
  k.dem={mv,fw,srv:Math.min(n,NV)};
};
// final command: the demand for actuators in A, the operator's value for those in M, in SI
const command=(k,c)=>{
  const A=c.auto||{},D=k.dem;
  return {heat:c.heat*1e6,fw:A.fw?D.fw:c.fw,tfw:c.tfw+273.15,mv:A.mv?D.mv:c.mv/100,srv:A.srv?D.srv:c.srv};
};
// one control step: update the demands from the plant state, return the command for BWRSim.step.
// Call it before every plant step, with the same dt.
const update=(k,s,c,dt)=>{demands(k,s,c,dt);return command(k,c)};
return {init,demands,command,update,KP,TIP,KL,TIL,GROUPS,GSTEP,RESEAT};
})();
