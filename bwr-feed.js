// BWR condensate and feedwater train: hotwell, condensate pump, polisher, LP heater, feed pump with its
// minimum-flow line, HP heater, feed valve. Pure functions, no DOM. An ES module; imports common.js,
// bwr-steam.js, bwr-condenser.js. See docs/bwr.md, "Condensate and feed train".
import {sat,Tsat,clamp,hLiq,inv} from './common.js';
import {PR,WR,TFR} from './bwr-steam.js';
import {expansion,condSteady} from './bwr-condenser.js';
// Condenser hotwell → condensate pump → polisher → LP heater → feed pump → HP heater → feed valve →
// check valve → sparger; a minimum-flow line from the feed pump discharge back to the condenser.
// Hotwell: the water at the bottom of the condenser, capacity HWM kg, normal half full, its own mass and
// temperature (mixing condensate at the shell temperature, makeup, drains). Makeup from / reject to the
// condensate storage tank up to MKMAX kg/s either way; makeup water at TMK (an ambient-temperature tank).
// The pipes hold no water: every step the flows through the whole train are solved together (network()).
// Overall numbers kept from the lumped pump this replaces: at full speed and 7 MPa the feed valve passes
// the rated 1788 kg/s about 75 % open and 2400 kg/s wide open; the two pumps together shut off at
// 10 MPa (H0) over the hotwell. Feed valve: drop w²/(KV·x²)² (quadratic characteristic, opening x),
// strokes at most FRATE per second. HP heater and piping drop KLN·w².
const H0=10e6,KPU=0.24,KLN=0.094,KV=2.31,FRATE=0.1,NRATE=0.1,HWM=600e3,MKMAX=100,TMK=30+273.15;
// Condensate pump: constant-speed motor, on or off; developed pressure φ·HC0 − KCP·w² (shutoff 2.6 MPa,
// 0.26 MPa down at rated flow). Suction at the hotwell bottom: condenser pressure plus the water over the
// first-stage impeller, ρg·(ZS + HWD·fraction full) (ZS 1 m pit, HWD 4 m deep hotwell). NPSH available is
// that over the vapour pressure at the hotwell temperature; NPSH required NPR·(w/WR)² (2 m at rated).
// When NPSHa falls below NPSHr the pump cavitates: φ falls linearly to 0 at NPSHa = NPSHr/2. Below 2 %
// the hotwell no longer covers the suction (vortexing) and φ falls to 0 at empty. All illustrative.
// Polisher (deep-bed demineraliser) KPOL·w², 0.13 MPa at rated; LP heater and piping KLP·w², 0.1 MPa.
const HC0=2.6e6,KCP=0.08,KPOL=0.04,KLP=0.03,ZS=1.0,HWD=4.0,NPR=2.0,RHW=992,G=9.81;
// Feed pump: turbine-driven (BWR/4), variable speed n (affinity, n²·HF0 − KFP·w²), the rest of the old
// lumped head: HF0 = H0 − HC0. Its drive steam is not modelled. Stopped (or tripped, or at zero speed)
// it passes flow with the resistance KFP·w², so at low vessel pressure the condensate pump alone feeds
// the vessel through the idle feed pump, as in a startup. KFP is chosen so that the train's rated
// discharge pressure is that of the lumped pump. Low suction pressure trip: below PLO (0.7 MPa) for TLO
// (5 s) the pump trips (plant equipment protection), latched until its run switch is cycled. If its
// suction would fall below the vapour pressure it cavitates: it delivers only what reaches its suction.
const HF0=H0-HC0,KFP=KPU-KCP-KPOL-KLP+RHW*G*(ZS+HWD/2)/WR**2,PLO=0.7e6,TLO=5;
// Minimum-flow recirculation: valve opening xr from the feed pump discharge to the condenser,
// flow KR·xr·√(Pd − Pc) (linear), about WMIN = 25 % of rated pump flow open near shutoff; strokes RRATE/s.
const WMIN=0.25*WR,KR=WMIN/Math.sqrt(9.9e6),RRATE=0.2;
// liquid temperature from its saturated-liquid enthalpy (the hotwell is near atmospheric pressure)
const Thf=inv(T=>sat(T).hf);
// condensate pump suction: {Pin pressure at the impeller, A NPSH available m}
const suction=(pc,Mh,Thw)=>{const hl=clamp(Mh/HWM,0,1)*HWD,Pin=pc+RHW*G*(ZS+hl);return {Pin,A:(Pin-sat(Thw).p)/(RHW*G),f:clamp(Mh/(0.02*HWM),0,1)}};
// condensate pump flow against discharge pressure Pcd (closed form; on false: a stopped pump's resistance)
const cpFlow=(on,Pcd,sc)=>{
  if(!on){const D=sc.Pin-Pcd;return D>0?Math.sqrt(D/KCP):0}
  const R=NPR/WR**2,H=HC0*sc.f,y1=(sc.Pin+H-Pcd)/KCP;
  if(!(y1>0))return 0;if(R*y1<=sc.A)return Math.sqrt(y1);
  // cavitating: φ = 2A/(R y) − 1 (y = w²), φ·H − KCP·y = Pcd − Pin, a quadratic in y
  if(!(sc.A>0))return 0;const b=H+Pcd-sc.Pin,c=2*sc.A*H/R,y=(-b+Math.sqrt(b*b+4*KCP*c))/(2*KCP);
  return y>0?Math.sqrt(Math.min(y,y1)):0};
// One solve of the whole train for one step. o: {cp condensate pump on, n feed pump speed 0–1, x feed
// valve, xr min-flow valve, P vessel, pc condenser, Mh, Thw hotwell, pv vapour pressure at the feed pump
// suction}. Unknowns: condensate pump discharge Pcd, feed pump suction Ps, discharge Pd. Bisection on
// Ps outside, on Pd and Pcd inside (each flow is monotonic in its node pressure).
// Returns {fw to the vessel, wr min-flow, wp feed pump, wcp condensate pump, wl through the LP heater,
// Pcd, Ps, Pd, cav feed pump cavitating, Pin, A NPSHa m, phi condensate pump head factor}.
const network=o=>{
  const sc=suction(o.pc,o.Mh,o.Thw),n2=o.n*o.n*HF0,fv=o.x*o.x;
  const fwOf=Pd=>{const D=Pd-o.P;return D>0&&fv>0?Math.sqrt(D/(KLN+1/(KV*fv)**2)):0};
  const wrOf=Pd=>Pd>o.pc?KR*o.xr*Math.sqrt(Pd-o.pc):0;
  const wpOf=(Ps,Pd)=>{const D=Ps+n2-Pd;return D>0?Math.sqrt(D/KFP):0};
  const KL=KPOL+KLP,wlOf=(Pcd,Ps)=>Pcd>Ps?Math.sqrt((Pcd-Ps)/KL):0;
  // feed side at suction Ps: Pd where the pump passes what the vessel line and the min-flow line take
  const side=Ps=>{let a=o.pc,b=Math.max(o.pc,Ps+n2);for(let i=0;i<40;i++){const m=(a+b)/2;if(wpOf(Ps,m)>fwOf(m)+wrOf(m))a=m;else b=m}return (a+b)/2};
  // condensate side at suction Ps: Pcd where the condensate pump passes what the LP line takes
  const cside=Ps=>{let a=Math.min(sc.Pin,Ps),b=sc.Pin+HC0+1;for(let i=0;i<40;i++){const m=(a+b)/2;if(cpFlow(o.cp,m,sc)>wlOf(m,Ps))a=m;else b=m}return (a+b)/2};
  const r=Ps=>{const Pd=side(Ps),Pcd=cside(Ps);return {Pd,Pcd,wl:wlOf(Pcd,Ps),wp:wpOf(Ps,Pd)}};
  let lo=Math.max(o.pv||0,1e3),hi=Math.max(lo,sc.Pin+HC0)+1,cav=false,Ps,q;
  q=r(lo);
  if(q.wl<q.wp){cav=true;Ps=lo;
    // the feed pump takes only what reaches its suction: Pd where the lines downstream take wl
    let a=o.pc,b=Math.max(o.pc,Ps+n2);for(let i=0;i<40;i++){const m=(a+b)/2;if(fwOf(m)+wrOf(m)<q.wl)a=m;else b=m}
    q={...q,Pd:(a+b)/2,wp:q.wl}}
  else{for(let i=0;i<40;i++){const m=(lo+hi)/2,t=r(m);if(t.wl>t.wp)lo=m;else hi=m}Ps=(lo+hi)/2;q=r(Ps)}
  // the flows from the converged pressures; the pumps pass exactly what the lines downstream take (mass)
  const fw=fwOf(q.Pd),wr=wrOf(q.Pd);
  return {fw,wr,wp:fw+wr,wcp:fw+wr,wl:fw+wr,Pcd:q.Pcd,Ps,Pd:q.Pd,cav,Pin:sc.Pin,A:sc.A,phi:sc.f*(sc.A>=NPR*(q.wl/WR)**2?1:clamp(2*sc.A/(NPR*(q.wl/WR)**2)-1,0,1))};
};
// pump flow to the vessel at feed pump speed n (0–1), valve opening x (0–1), vessel pressure P, hotwell
// mass Mh, condenser pc: the steady train with the condensate pump running and the min-flow valve shut
const fwFlow=(n,x,P,Mh=HWM/2,pc=PCD)=>network({cp:true,n,x,xr:0,P,pc,Mh,Thw:Tsat(pc)}).fw;
// highest vessel pressure the train can feed against at speed n (both pumps at shutoff, hotwell at Mh)
const shutoff=(n,pc=PCD,Mh=HWM/2)=>pc+RHW*G*(ZS+clamp(Mh/HWM,0,1)*HWD)+HC0+n*n*HF0;
// valve opening that passes w at speed n against P (null if the pumps can't), the inverse of fwFlow
const fwValve=(w,n,P,pc=PCD)=>{if(!(w>0))return 0;
  const Pd=pc+RHW*G*(ZS+HWD/2)+HC0-(KCP+KPOL+KLP)*w*w+n*n*HF0-KFP*w*w,D=Pd-P-KLN*w*w;
  return D>0?Math.sqrt(1/(KV*Math.sqrt(D/(w*w)))):null};
// ---- feedwater heaters ----
// Two stages, LP (before the feed pump) and HP (after it), each a shell where steam extracted from the
// turbine stub condenses on the feed tubes. The turbine's stage pressures are in proportion to its steam
// flow, so each extraction point is at pe = PE·wst/WR (HP PE 2.40 MPa; LP PEL 0.28 MPa, a typical
// low-pressure extraction, giving 126 °C out of the LP heater). Each extraction line has a valve
// (opening e) with flow KX·e·√(pe² − ps²) (compressible flow through a restriction, not choked), which
// sets the shell pressure ps. The feed leaves TTD (3 K) below the shell saturation temperature, and
// the shell condenses what it takes to heat the feed: ws = fw·(h_out − h_in)/(hx − h′(ps)), hx the
// extraction steam enthalpy on the turbine stub's expansion line. Design point of each: rated steam and
// feed flow, 5 % pressure drop across the open valve. The drains (saturated at the shell pressure) go to
// the condenser, which flashes them. Real extraction lines have only isolation and non-return valves;
// the throttling valves here stand for losing or isolating a stage.
const TTD=3,PSR=sat(TFR+TTD).p,PE=PSR/0.95,PEL=0.28e6,PSL=0.95*PEL,TLPD=Tsat(PSL)-TTD;
const HG0=sat(Tsat(PR)).hg,LINE0=expansion(PR,HG0),HX0=LINE0(PE),HXL0=LINE0(PEL);
// design hotwell temperature: the condenser at rated, receiving the stub's exhaust and both heaters'
// drains. The extraction valve constants depend on it, so iterate a few times.
const design=()=>{let Th=311,KXT=0,KXL=0,Pc=6.6e3;
  for(let i=0;i<8;i++){const hdH=sat(TFR+TTD).hf,hdL=sat(TLPD+TTD).hf;
    const ws=WR*(hLiq(TFR,PR)-hLiq(TLPD,PR))/(HX0-hdH),wsl=WR*(hLiq(TLPD,PR)-hLiq(Th,PR))/(HXL0-hdL);
    KXT=ws/Math.sqrt(PE*PE-PSR*PSR);KXL=wsl/Math.sqrt(PEL*PEL-PSL*PSL);
    const Tc=condSteady([[WR-ws-wsl,LINE0(Pc)],[ws,hdH],[wsl,hdL]]);Th=Tc;Pc=sat(Tc).p}
  return {THW:Th,KXT,KXL,PCD:Pc}};
const {THW,KXT,KXL,PCD}=design();
const HP=Object.freeze({PE,KX:KXT}),LP=Object.freeze({PE:PEL,KX:KXL});
// one stage H {PE, KX} at main steam flow wst, extraction valve e, feed flow fw entering at Tin,
// extraction steam enthalpy line(p): {T outlet target K, pe extraction pressure, ps shell pressure,
// ws extraction steam kg/s, hx, hd drain enthalpy}. Bisection on the shell saturation temperature:
// the valve flow falls and the condensing demand rises with it.
const stage=(H,wst,e,fw,Tin,line=LINE0)=>{
  const pe=H.PE*Math.max(0,wst)/WR,e1=clamp(e,0,1),Tlo=Tin+TTD;
  if(!(e1>0)||!(pe>sat(Tlo).p))return {T:Tin,pe,ps:0,ws:0,hx:0,hd:0};
  const hin=hLiq(Tin,PR),Thi=Tsat(pe),hx=line(pe);
  if(!(fw>1e-6)){const q=sat(Thi);return {T:Thi-TTD,pe,ps:q.p,ws:0,hx,hd:q.hf}}
  let a=Tlo,b=Thi;
  for(let i=0;i<40;i++){const m=(a+b)/2,q=sat(m),p=q.p,dh=hx-q.hf,dem=fw*(hLiq(m-TTD,PR)-hin)/dh,sup=H.KX*e1*Math.sqrt(Math.max(0,pe*pe-p*p));if(sup>dem)a=m;else b=m}
  const m=(a+b)/2,q=sat(m),ps=q.p,hd=q.hf;return {T:m-TTD,pe,ps,ws:fw*(hLiq(m-TTD,PR)-hin)/(hx-hd),hx,hd};
};
// HP heater (default inlet: the LP heater's design outlet) and LP heater (default inlet: the hotwell)
const heater=(wst,e,fw,Tin=TLPD,line=LINE0)=>stage(HP,wst,e,fw,Tin,line);
const heaterL=(wst,e,fw,Tin=THW,line=LINE0)=>stage(LP,wst,e,fw,Tin,line);
// both stages in series, the same feed flow through both: {T HP outlet, lp, hp}
const train=(wst,eL,eH,fw,Tin=THW,line=LINE0)=>{const lp=heaterL(wst,eL,fw,Tin,line),hp=heater(wst,eH,fw,lp.T,line);return {T:hp.T,lp,hp}};
// outlet of the train with both extraction valves at e
const heaterT=(wst,e,fw=wst,Tin=THW)=>train(wst,e,e,fw,Tin).T;
// steady heat balance at pressure P: steam flow W that heat Q makes from feed heated by that same W
const heatBalance=(Q,P,e=1)=>{const hg=sat(Tsat(P)).hg;let T=THW,W=0;
  for(let i=0;i<60;i++){W=Q/(hg-hLiq(T,P));T=heaterT(W,e,W)}return {W,Tfw:T}};
// hotwell mixing: h {M kg, T K}; ins = [[w kg/s, h J/kg], …] (negative w leaves at the hotwell
// temperature). Returns the new {M, T}.
const hotwellMix=(hw,ins,dt)=>{const h0=sat(hw.T).hf;let M=hw.M,U=M*h0;
  for(const[w,h]of ins){const dm=w*dt;M+=dm;U+=dm*(w>=0?h:h0)}
  return M>1?{M,T:Thf(U/M)}:{M:Math.max(0,M),T:hw.T}};
export {H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,PEL,KXT,KXL,HX0,LINE0,TLPD,Thf,
  HC0,HF0,KCP,KPOL,KLP,KFP,ZS,HWD,NPR,PLO,TLO,WMIN,KR,RRATE,suction,cpFlow,network,shutoff,
  fwFlow,fwValve,stage,heater,heaterL,train,heaterT,heatBalance,hotwellMix};
