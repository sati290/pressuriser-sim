// BWR main steam: rated conditions, the main steam line with its MSIVs, turbine stop, control and bypass
// valves, the safety/relief valves (SRVs) and the steam nozzle flow factor. Pure functions, no DOM. An ES module; imports common.js. See docs/bwr.md.
import {sat,Tsat,hLiq} from './common.js';
// rated conditions (generic BWR/4 class): core thermal power QR, dome pressure PR, feedwater 216 °C;
// rated steam flow WR from the heat balance
const QR=3293e6,PR=7.0e6,TFR=216+273.15;
const WR=QR/(sat(Tsat(PR)).hg-hLiq(TFR,PR));
// ---- main steam line and turbine valves ----
// Dome → main steam line → inboard MSIV → outboard MSIV → header (the throttle) → turbine stop valve →
// turbine control valve → turbine stub; a bypass valve from the header straight to the condenser.
// The pipes hold no inventory: the header pressure Pl is solved each step so that what comes in through
// the line and the two MSIVs in series, w = √((P² − Pl²)/(RL + RM/a1² + RM/a2²)) (compressible flow
// through restrictions, not choked; a1, a2 the MSIV flow-area fractions), equals what leaves through
// the turbine and bypass valves, choked ∝ Pl with the subsonic factor against the condenser pressure:
// w = (KT·asv·fcv(cv) + KB·bp)·Pl·ψ. A header node holding steam would have a time constant of about
// 0.2 s, too fast for the page's 2 s step, so it is algebraic.
// Numbers (illustrative, typical of the class, not from one plant): 0.3 MPa dome-to-throttle drop at
// rated flow (DPL), each MSIV taking 15 % of it; turbine valves wide open pass 105 % of rated steam at
// rated dome pressure (VWO); bypass capacity 25 % of rated at the rated throttle pressure (decided).
const DPL=0.3e6,PL0=PR-DPL,RS0=(PR*PR-PL0*PL0)/(WR*WR),RM=0.15*RS0,RL=RS0-2*RM;
const VWO=1.05,PLV=Math.sqrt(PR*PR-RS0*(VWO*WR)**2),KT=VWO*WR/PLV,BPF=0.25,KB=BPF*WR/PL0;
// control valve characteristic: flow-area fraction at lift x, steep at small lift and flattening near
// full open as a set of sequentially opening valves does (illustrative shape), and its inverse; with it
// the valve is about 87 % open at rated
const fcv=x=>1-(1-Math.min(1,Math.max(0,x)))**1.5,fcvInv=f=>1-(1-Math.min(1,Math.max(0,f)))**(2/3);
// strokes (fraction per second): MSIVs 4 s each way (closure 3–5 s is the usual specification); stop
// valve trips shut in 0.2 s, opens over 5 s; control valve 5 s full stroke; bypass 0.5 s (fast opening
// on a turbine trip). Illustrative except the MSIV closure time.
const MSRATE=0.25,SVCLOSE=5,SVOPEN=0.2,CVRATE=0.2,BPRATE=2;
// header pressure and flows for dome pressure P, condenser pressure pc and valve positions
// v {ms:[a1,a2], sv, cv, bp}; Pl0 the last header pressure (kept while the header is shut in at both ends).
// Returns {Pl, w line flow, wt turbine, wb bypass} (kg/s); w = wt + wb exactly.
const steamLine=(P,pc,v,Pl0=P)=>{
  const g1=v.ms[0]**2,g2=v.ms[1]**2,K=KT*v.sv*fcv(v.cv)+KB*v.bp,none={w:0,wt:0,wb:0};
  if(!(g1>0&&g2>0))return {Pl:K>0?pc:Math.min(P,Math.max(pc,Pl0)),...none};
  if(!(K>0)||!(P>pc))return {Pl:P,...none};
  const Rs=RL+RM/g1+RM/g2,g=Pl=>Math.sqrt((P*P-Pl*Pl)/Rs)-K*Pl*subF(Pl,pc);
  let a=pc,b=P;for(let i=0;i<50;i++){const m=(a+b)/2;if(g(m)>0)a=m;else b=m}
  const Pl=(a+b)/2,w=K*Pl*subF(Pl,pc),wt=KT*v.sv*fcv(v.cv)*Pl*subF(Pl,pc);
  return {Pl,w,wt,wb:w-wt};
};
// control valve lift that passes w (kg/s) at dome pressure P, MSIVs and stop valve open, bypass shut
// (choked); null if wide open would not pass it
const cvPos=(w,P)=>{const Pl=Math.sqrt(P*P-RS0*w*w),f=w/(KT*Pl);return Pl>0&&f<=1?fcvInv(f):null};
// valve travel: each moves toward its target at its stroke rate. u.trip is the hook for reactor
// protection (bwr-rps.js): trip.turbine shuts the stop valve, trip.msiv the MSIVs, trip.bypass the bypass
// valve, whatever the switches and demands.
// A switch command left undefined holds its valve where it is.
const valveStep=(s,u,dt)=>{
  const mv=(x,t,up,dn)=>x+Math.min(Math.max(t-x,-dn*dt),up*dt),tr=u.trip||{};
  const msT=[u.ms1,u.ms2].map((c,i)=>tr.msiv?0:c==null?(s.ms[i]>0.5?1:0):c?1:0);
  s.ms=s.ms.map((x,i)=>mv(x,msT[i],MSRATE,MSRATE));
  const svT=tr.turbine?0:u.sv==null?(s.sv>0.5?1:0):u.sv?1:0;s.sv=mv(s.sv,svT,SVOPEN,SVCLOSE);
  s.cv=mv(s.cv,Math.min(1,Math.max(0,u.cv||0)),CVRATE,CVRATE);
  s.bp=mv(s.bp,tr.bypass?0:Math.min(1,Math.max(0,u.bp||0)),BPRATE,BPRATE);
};
// safety/relief valves (SRVs), discharging to the suppression pool at PB. Each valve has a spring
// that opens it fully above its set pressure p (Pa abs) and recloses it BD below, and can also be
// opened by hand (relief mode, a command from the operator). Capacity w (kg/s) is rated at its set
// pressure; the flow scales ∝ P (choked) with the subsonic factor near PB. Values from a GE BWR/4
// training lesson plan: 4 valves at 1090 psig rated 800,000 lb/h, 4 at 1100 psig 808,000 lb/h,
// 3 at 1110 psig 815,000 lb/h (the same valve model; capacity ∝ set pressure). Blowdown BD is
// illustrative (about 3 %); the source gives none.
const psig=x=>(x+14.696)*6894.757,lbh=x=>x*1.25998e-4;
const SRV=Object.freeze([[1090,800e3],[1090,800e3],[1090,800e3],[1090,800e3],[1100,808e3],[1100,808e3],[1100,808e3],[1100,808e3],[1110,815e3],[1110,815e3],[1110,815e3]]
  .map(([p,w])=>Object.freeze({p:psig(p),w:lbh(w)})));
const NV=SRV.length,PB=0.1e6,BD=0.25e6;
// steam nozzle flow factor against a back pressure pb: 1 while choked, ideal-gas subsonic nozzle
// factor (κ = 1.3) above the critical pressure ratio, 0 at pb
const KAP=1.3,RCR=(2/(KAP+1))**(KAP/(KAP-1));
const psi=r=>Math.sqrt(Math.max(0,r**(2/KAP)-r**((KAP+1)/KAP))),PSI=psi(RCR);
const subF=(P,pb)=>{if(P<=pb)return 0;const r=pb/P;return r<=RCR?1:psi(r)/PSI};
const wSRV=(i,P)=>SRV[i].w*P/SRV[i].p*subF(P,PB);
// SRV springs and hand openings at pressure P: updates the spring latches spr[], returns {open, wsrv, nsrv}
const srvStep=(spr,man,P)=>{
  const open=SRV.map((v,i)=>{if(P>v.p)spr[i]=true;else if(P<v.p-BD)spr[i]=false;return spr[i]||!!man[i]});
  let wsrv=0,nsrv=0;open.forEach((o,i)=>{if(o){wsrv+=wSRV(i,P);nsrv++}});
  return {open,wsrv,nsrv};
};
export {QR,PR,TFR,WR,SRV,NV,PB,BD,psig,lbh,subF,wSRV,srvStep,
  DPL,PL0,RS0,RM,RL,VWO,KT,BPF,KB,fcv,fcvInv,MSRATE,SVCLOSE,SVOPEN,CVRATE,BPRATE,steamLine,cvPos,valveStep};
