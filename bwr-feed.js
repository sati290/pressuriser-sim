// BWR condensate and feedwater train: hotwell, feed pump, feedwater heater, feed valve. Pure functions,
// no DOM. An ES module; imports common.js, bwr-steam.js, bwr-condenser.js. See docs/bwr.md.
import {sat,Tsat,clamp,hLiq,inv} from './common.js';
import {PR,WR,TFR,PL0} from './bwr-steam.js';
import {expansion,condSteady} from './bwr-condenser.js';
// Feedwater train: condenser hotwell → feed pump → feedwater heater → feed control valve → sparger.
// Hotwell: the water at the bottom of the condenser, capacity HWM kg, normal half full, its own mass and
// temperature (mixing condensate at the shell temperature, makeup, drains). Makeup from / reject to the
// condensate storage tank up to MKMAX kg/s either way; makeup water at TMK (an ambient-temperature tank).
// Pump (the condensate, booster and feed pumps lumped as one): developed pressure n²·H0 − KP·w² at
// speed n (affinity laws; speed changes at most NRATE per second), suction at the condenser pressure; it
// loses suction as the hotwell runs below 5 % (flow tapers to zero). Heater and piping drop KL·w². Feed
// valve: drop w²/(KV·x²)² (quadratic characteristic, opening x), strokes at most FRATE per second. Check
// valve: no reverse flow. Numbers chosen so that at full speed and 7 MPa the valve passes the rated
// 1788 kg/s about 75 % open and 2400 kg/s wide open.
const H0=10e6,KPU=0.24,KLN=0.094,KV=2.31,FRATE=0.1,NRATE=0.1,HWM=600e3,MKMAX=100,TMK=30+273.15;
// liquid temperature from its saturated-liquid enthalpy (the hotwell is near atmospheric pressure)
const Thf=inv(T=>sat(T).hf);
// pump flow at speed n (0–1), valve opening x (0–1), vessel pressure P, hotwell mass Mh, suction pc
const fwFlow=(n,x,P,Mh=HWM/2,pc=PCD)=>{const D=n*n*H0+pc-P,f=x*x;if(!(D>0)||!(f>0))return 0;
  return Math.sqrt(D/(KPU+KLN+1/(KV*f)**2))*clamp(Mh/(0.05*HWM),0,1)};
// valve opening that passes w at speed n against P (null if the pump can't), the inverse of fwFlow
const fwValve=(w,n,P,pc=PCD)=>{const D=n*n*H0+pc-P;if(!(w>0))return 0;const r=D/(w*w)-KPU-KLN;return r>0?Math.sqrt(1/(KV*Math.sqrt(r))):null};
// Feedwater heater: one stage standing for the whole heater train, a shell where steam extracted from
// the turbine condenses on the feed tubes. The turbine's stage pressures are in proportion to its steam
// flow, so the extraction point is at pe = PE·wst/WR. The extraction line has a valve (opening e) with
// flow KX·e·√(pe² − ps²) (compressible flow through a restriction, not choked), which sets the shell
// pressure ps. The feed leaves TTD (3 K) below the shell saturation temperature, and the shell condenses
// what it takes to heat the feed: ws = fw·(h_out − h_in)/(hx − h′(ps)), hx the extraction steam enthalpy
// on the turbine stub's expansion line. The shell pressure is where the valve passes what the shell
// condenses. Design point: rated steam and feed flow WR, feed out at 216 °C, 5 % pressure drop across the
// open valve; the turbine's throttle is the main steam header at its rated pressure PL0. The drains (saturated at the shell pressure) go to the condenser, which flashes them.
const TTD=3,PSR=sat(TFR+TTD).p,PE=PSR/0.95,HG0=sat(Tsat(PR)).hg,LINE0=expansion(PL0,HG0),HX0=LINE0(PE);
// design hotwell temperature: the condenser at rated, receiving the stub's exhaust and the heater drains.
// The extraction valve constant KXT depends on it, so iterate a few times.
const design=()=>{let Th=311,KXT=0,Pc=6.6e3;
  for(let i=0;i<8;i++){const q=sat(TFR+TTD),hd=q.hf,ws=WR*(hLiq(TFR,PR)-hLiq(Th,PR))/(HX0-hd);
    KXT=ws/Math.sqrt(PE*PE-PSR*PSR);const Tc=condSteady([[WR-ws,LINE0(Pc)],[ws,hd]]);Th=Tc;Pc=sat(Tc).p}
  return {THW:Th,KXT,PCD:Pc}};
const {THW,KXT,PCD}=design();
// heater state at main steam flow wst, extraction valve e, feed flow fw entering at Tin, extraction
// steam enthalpy hx(pe) (a function of pressure; default the rated expansion line):
// {T outlet target K, pe extraction pressure, ps shell pressure, ws extraction steam kg/s, hx, hd drain enthalpy}.
// Bisection on the shell saturation temperature: the valve flow falls and the condensing demand rises with it.
const heater=(wst,e,fw,Tin=THW,line=LINE0)=>{
  const pe=PE*Math.max(0,wst)/WR,e1=clamp(e,0,1),Tlo=Tin+TTD;
  if(!(e1>0)||!(pe>sat(Tlo).p))return {T:Tin,pe,ps:0,ws:0,hx:0,hd:0};
  const hin=hLiq(Tin,PR),Thi=Tsat(pe),hx=line(pe);
  if(!(fw>1e-6)){const q=sat(Thi);return {T:Thi-TTD,pe,ps:q.p,ws:0,hx,hd:q.hf}}
  let a=Tlo,b=Thi;
  for(let i=0;i<40;i++){const m=(a+b)/2,q=sat(m),p=q.p,dh=hx-q.hf,dem=fw*(hLiq(m-TTD,PR)-hin)/dh,sup=KXT*e1*Math.sqrt(Math.max(0,pe*pe-p*p));if(sup>dem)a=m;else b=m}
  const m=(a+b)/2,q=sat(m),ps=q.p,hd=q.hf;return {T:m-TTD,pe,ps,ws:fw*(hLiq(m-TTD,PR)-hin)/(hx-hd),hx,hd};
};
const heaterT=(wst,e,fw=wst,Tin=THW)=>heater(wst,e,fw,Tin).T;
// steady heat balance at pressure P: steam flow W that heat Q makes from feed heated by that same W
const heatBalance=(Q,P,e=1)=>{const hg=sat(Tsat(P)).hg;let T=THW,W=0;
  for(let i=0;i<60;i++){W=Q/(hg-hLiq(T,P));T=heaterT(W,e,W)}return {W,Tfw:T}};
// hotwell mixing: h {M kg, T K}; ins = [[w kg/s, h J/kg], …] (negative w leaves at the hotwell
// temperature). Returns the new {M, T}.
const hotwellMix=(hw,ins,dt)=>{const h0=sat(hw.T).hf;let M=hw.M,U=M*h0;
  for(const[w,h]of ins){const dm=w*dt;M+=dm;U+=dm*(w>=0?h:h0)}
  return M>1?{M,T:Thf(U/M)}:{M:Math.max(0,M),T:hw.T}};
export {H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,KXT,HX0,LINE0,Thf,fwFlow,fwValve,heater,heaterT,heatBalance,hotwellMix};
