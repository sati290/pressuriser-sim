// BWR condensate and feedwater train: hotwell, condensate pump, polisher, LP heater, feed pump with its
// minimum-flow line, HP heater, feed valve. Pure functions, no DOM. An ES module; imports common.js,
// bwr-steam.js, bwr-condenser.js. See docs/bwr.md, "Condensate and feed train".
import {sat,Tsat,clamp,hLiq,inv} from './common.js';
import {PR,WR,TFR,PL0} from './bwr-steam.js';
import {expansion,condSteady,WSJ,PATM} from './bwr-condenser.js';
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
const HF0=H0-HC0,PLO=0.7e6,TLO=5;
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
// heater drain pump (step 6): constant-speed motor, run switch; suction from the LP heater drain tank at
// its shell pressure plus ZDT (5 m) of water; developed pressure f·HDP0 − KDPP·w² (3.5 MPa shutoff, 0.5 MPa
// down at rated drain flow), f falling to 0 as the drain tank empties (below 10 %); discharge valve
// (opening x, linear conductance KDVL·x) into the condensate line after the LP heater. Check valve.
const HDP0=3.5e6,ZDT=5;
const dpFlow=(dp,Ps)=>{if(!dp||!dp.on||!(dp.x>0))return 0;const D=dp.Pin+dp.f*HDP0-Ps;return D>0?Math.sqrt(D/(KDPP+1/(KDVL*dp.x)**2)):0};
// Condensate storage tank (step 7): one mass and temperature, open to the atmosphere (PATM). Capacity
// CSTM 1900 t (about 500,000 US gal, the size of a BWR/4 CST), HCST 12 m tall, level ∝ mass. Makeup valve
// from the CST to the condenser hotwell (opening x, linear): flow KMU·x·√(PATM + ρg·(h − ZMU) − Pc), the
// tank's head plus the condenser vacuum, ZMU 3 m the condenser inlet over the tank bottom; about 100 kg/s
// fully open at 70 % full under vacuum. Below 2 % the tank's outlet uncovers and the flow falls to 0.
// Reject valve from the condensate pump discharge to the CST: KRJ·x·√(Pcd − PATM − ρg·h), about
// 100 kg/s fully open from 2.4 MPa. Both stroke at most CRATE per second. Sizes and elevations illustrative.
const CSTM=1.9e6,HCST=12,ZMU=3,CRATE=0.1;
const KMU=100/Math.sqrt(PATM+992*9.81*(0.7*HCST-ZMU)-6.5e3),KRJ=100/Math.sqrt(2.3e6);
const cstH=Mc=>HCST*clamp(Mc/CSTM,0,1.2);
const muFlow=(x,Mc,pc)=>{const D=PATM+RHW*G*(cstH(Mc)-ZMU)-pc;return D>0&&x>0?KMU*x*Math.sqrt(D)*clamp(Mc/(0.02*CSTM),0,1):0};
const rjBack=Mc=>PATM+RHW*G*cstH(Mc);
// One solve of the whole train for one step. o: {cp condensate pump on, n feed pump speed 0–1, x feed
// valve, xr min-flow valve, P vessel, pc condenser, Mh, Thw hotwell, pv vapour pressure at the feed pump
// suction, dp drain pump {on, x valve, Pin suction pressure, f head factor} or none, rj reject valve
// {x, Pb back pressure} or none}. Unknowns: condensate
// pump discharge Pcd, feed pump suction Ps, discharge Pd. Bisection on Ps outside, on Pd and Pcd inside
// (each flow is monotonic in its node pressures).
// Returns {fw to the vessel, wr min-flow, wp feed pump, wcp condensate pump, wl through the LP heater,
// wd drains pumped forward, wrj reject to the CST, Pcd, Ps, Pd, cav feed pump cavitating, Pin, A NPSHa m, phi condensate pump
// head factor}.
const network=o=>{
  const sc=suction(o.pc,o.Mh,o.Thw),n2=o.n*o.n*HF0,fv=o.x*o.x,wdOf=Ps=>dpFlow(o.dp,Ps);
  const fwOf=Pd=>{const D=Pd-o.P;return D>0&&fv>0?Math.sqrt(D/(KLN+1/(KV*fv)**2)):0};
  const wrOf=Pd=>Pd>o.pc?KR*o.xr*Math.sqrt(Pd-o.pc):0;
  const wpOf=(Ps,Pd)=>{const D=Ps+n2-Pd;return D>0?Math.sqrt(D/KFP):0};
  const KL=KPOL+KLP,wlOf=(Pcd,Ps)=>Pcd>Ps?Math.sqrt((Pcd-Ps)/KL):0;
  // feed side at suction Ps: Pd where the pump passes what the vessel line and the min-flow line take
  const side=Ps=>{let a=o.pc,b=Math.max(o.pc,Ps+n2);for(let i=0;i<40;i++){const m=(a+b)/2;if(wpOf(Ps,m)>fwOf(m)+wrOf(m))a=m;else b=m}return (a+b)/2};
  // condensate side at suction Ps: Pcd where the condensate pump passes what the LP line takes
  // (and the reject line to the CST, which branches off there)
  const wrjOf=Pcd=>o.rj&&o.rj.x>0&&Pcd>o.rj.Pb?KRJ*o.rj.x*Math.sqrt(Pcd-o.rj.Pb):0;
  const cside=Ps=>{let a=Math.min(sc.Pin,Ps,o.rj?o.rj.Pb:Infinity),b=sc.Pin+HC0+1;for(let i=0;i<40;i++){const m=(a+b)/2;if(cpFlow(o.cp,m,sc)>wlOf(m,Ps)+wrjOf(m))a=m;else b=m}return (a+b)/2};
  // supply to the feed pump suction: condensate through the LP heater plus the drains pumped forward
  const r=Ps=>{const Pd=side(Ps),Pcd=cside(Ps),wl=wlOf(Pcd,Ps);return {Pd,Pcd,wl,ws:wl+wdOf(Ps),wp:wpOf(Ps,Pd)}};
  let lo=Math.max(o.pv||0,1e3),hi=Math.max(lo,sc.Pin+HC0,o.dp&&o.dp.on?o.dp.Pin+HDP0:0)+1,cav=false,Ps,q;
  q=r(lo);
  if(q.ws<q.wp){cav=true;Ps=lo;
    // the feed pump takes only what reaches its suction: Pd where the lines downstream take it
    let a=o.pc,b=Math.max(o.pc,Ps+n2);for(let i=0;i<40;i++){const m=(a+b)/2;if(fwOf(m)+wrOf(m)<q.ws)a=m;else b=m}
    q={...q,Pd:(a+b)/2,wp:q.ws}}
  else{for(let i=0;i<40;i++){const m=(lo+hi)/2,t=r(m);if(t.ws>t.wp)lo=m;else hi=m}Ps=(lo+hi)/2;q=r(Ps)}
  // the flows from the converged pressures; the pumps pass exactly what the lines downstream take (mass):
  // the drains at the final suction pressure, the condensate pump the rest
  const fw=fwOf(q.Pd),wr=wrOf(q.Pd),wp=fw+wr,wd=Math.min(wdOf(Ps),wp),wl=wp-wd;
  const wrj=wrjOf(q.Pcd),wcp=wl+wrj,yr=NPR*(wcp/WR)**2;
  return {fw,wr,wp,wcp,wl,wd,wrj,Pcd:q.Pcd,Ps,Pd:q.Pd,cav,Pin:sc.Pin,A:sc.A,phi:sc.f*(sc.A>=yr?1:clamp(2*sc.A/yr-1,0,1))};
};
// pump flow to the vessel at feed pump speed n (0–1), valve opening x (0–1), vessel pressure P, hotwell
// mass Mh, condenser pc: the steady train with the condensate pump running, the min-flow valve shut and
// the rated heater drains (WDD) pumped forward
const fwFlow=(n,x,P,Mh=HWM/2,pc=PCD)=>network({cp:true,n,x,xr:0,P,pc,Mh,Thw:Tsat(pc),dp:DPD}).fw;
// highest vessel pressure the train can feed against at speed n: the feed pump's shutoff on top of the
// higher of the condensate pump's (hotwell at Mh) and, if given and running, the drain pump's (dp as in
// network(); it injects at the feed pump suction with 3.5 MPa of head)
const shutoff=(n,pc=PCD,Mh=HWM/2,dp)=>Math.max(pc+RHW*G*(ZS+clamp(Mh/HWM,0,1)*HWD)+HC0,dp&&dp.on&&dp.x>0?dp.Pin+dp.f*HDP0:0)+n*n*HF0;
// valve opening that passes w at speed n against P (null if the pumps can't), the inverse of fwFlow
// (with the rated drain flow WDD pumped forward, as fwFlow)
const fwValve=(w,n,P,pc=PCD)=>{if(!(w>0))return 0;const wl=Math.max(0,w-WDD);
  const Pd=pc+RHW*G*(ZS+HWD/2)+HC0-(KCP+KPOL+KLP)*wl*wl+n*n*HF0-KFP*w*w,D=Pd-P-KLN*w*w;
  return D>0?Math.sqrt(1/(KV*Math.sqrt(D/(w*w)))):null};
// ---- feedwater heaters ----
// Two stages, LP (before the feed pump) and HP (after it), each a shell where steam extracted from the
// turbine stub condenses on the feed tubes. The turbine's stage pressures are in proportion to its steam
// flow wt, so each extraction point is at pe = PE·wt/WTR, WTR = WR − WSJ the rated turbine flow (the air
// ejector's motive steam is tapped off the header ahead of the turbine) (HP PE 2.40 MPa; LP PEL 0.28 MPa, a typical
// low-pressure extraction, giving 126 °C out of the LP heater). Each extraction line has a valve
// (opening e) with flow KX·e·√(pe² − ps²) (compressible flow through a restriction, not choked), which
// sets the shell pressure ps. The feed leaves TTD (3 K) below the shell saturation temperature, and
// the shell condenses what it takes to heat the feed: ws = fw·(h_out − h_in)/(hx − h′(ps)), hx the
// extraction steam enthalpy on the turbine stub's expansion line, which starts at the main steam header
// (rated PL0). Design point of each: rated turbine flow WTR and feed flow WR, 5 % pressure drop across the
// open valve. Drains (step 6): each shell holds its condensate
// (HP shell VHS 30 m³, LP shell with its drain tank VLS 60 m³, illustrative), saturated at the shell
// pressure. HP drains cascade through the drain valve (linear conductance KDVH·x on √(psH − psL)) to the
// LP shell, where they flash and help heat the condensate; LP drains and the cascaded HP drains collect
// in the drain tank and the drain pump forwards them into the condensate line after the LP heater. Both
// drain valves are half open at the rated design point. A shell over 60 % full starts to flood its tubes:
// the heated fraction a = (1 − level)/0.4, so a flooded heater heats less. Over 80 % the plant's
// high-level dump valve opens (fully at 90 %) to the condenser: flow KDD·x·√(ps − Pc), sized to pass
// 1.4 × the rated drains of that shell. Real extraction lines have only isolation and non-return valves;
// the throttling valves here stand for losing or isolating a stage.
const TTD=3,PSR=sat(TFR+TTD).p,PE=PSR/0.95,PEL=0.28e6,PSL=0.95*PEL,TLPD=Tsat(PSL)-TTD;
const WTR=WR-WSJ,HG0=sat(Tsat(PR)).hg,LINE0=expansion(PL0,HG0),HX0=LINE0(PE),HXL0=LINE0(PEL);
const VHS=30,VLS=60,LFL=0.6,LDU=0.8,XD0=0.5;
// feedwater liquid temperature from its enthalpy at PR (the heaters work in hLiq(T, PR))
const ThP=inv(T=>hLiq(T,PR));
// mixing at the feed pump suction: condensate wl at Tl and drains wd at hd (J/kg, saturated liquid)
const mixT=(wl,Tl,wd,hd)=>{const w=wl+wd;return w>1e-9&&wd>0?ThP((wl*hLiq(Tl,PR)+wd*hLiq(Thf(hd),PR))/w):Tl};
// design: hotwell temperature (the condenser at rated, receiving only the stub's exhaust: the drains are
// pumped forward), both extraction flows, the drains, the mixed HP heater inlet THPI, the valve and pump
// constants. They depend on one another, so iterate.
const design=()=>{let Th=311,KXT=0,KXL=0,Pc=6.6e3,Tm=TLPD,ws=0,wsl=0;
  const hdH=sat(TFR+TTD).hf,hdL=sat(TLPD+TTD).hf;
  for(let i=0;i<12;i++){
    ws=WR*(hLiq(TFR,PR)-hLiq(Tm,PR))/(HX0-hdH);
    const wd0=ws+wsl,wl=WR-wd0;
    wsl=Math.max(0,(wl*(hLiq(TLPD,PR)-hLiq(Th,PR))-ws*(hdH-hdL))/(HXL0-hdL));
    Tm=mixT(WR-ws-wsl,TLPD,ws+wsl,hdL);
    KXT=ws/Math.sqrt(PE*PE-PSR*PSR);KXL=wsl/Math.sqrt(PEL*PEL-PSL*PSL);
    const Tc=condSteady([[WTR-ws-wsl,LINE0(Pc)],[WSJ,HG0]]);Th=Tc;Pc=sat(Tc).p}
  const wd=ws+wsl,wl=WR-wd,Pin=Pc+RHW*G*(ZS+HWD/2),Kcs=KCP+KPOL+KLP;
  // feed pump: the rated discharge of the lumped pump of step 1, PCD + H0 − KPU·WR²
  const KFP=(Pin-Pc-Kcs*wl*wl+KPU*WR*WR)/WR**2;
  // drain pump: 0.5 MPa of its head lost at the rated drain flow; its valve half open there
  const KDPP=0.5e6/wd**2,Ps=Pin+HC0-Kcs*wl*wl,dpv=PSL+RHW*G*ZDT+HDP0-KDPP*wd*wd-Ps,KDVL=wd/(XD0*Math.sqrt(dpv));
  const KDVH=ws/(XD0*Math.sqrt(PSR-PSL)),KDDH=1.4*ws/Math.sqrt(PSR-Pc),KDDL=1.4*wd/Math.sqrt(PSL-Pc);
  return {THW:Th,KXT,KXL,PCD:Pc,THPI:Tm,KFP,KDPP,KDVL,KDVH,KDDH,KDDL,WSD:ws,WSLD:wsl,WDD:wd}};
const {THW,KXT,KXL,PCD,THPI,KFP,KDPP,KDVL,KDVH,KDDH,KDDL,WSD,WSLD,WDD}=design();
// the drain pump as fwFlow/fwValve assume it: running, valve at its design opening, passing about WDD at rated
const DPD=Object.freeze({on:true,x:XD0,Pin:PSL+RHW*G*ZDT,f:1});
const HP=Object.freeze({PE,KX:KXT}),LP=Object.freeze({PE:PEL,KX:KXL});
// one stage H {PE, KX} at turbine flow wst, extraction valve e, feed flow fw entering at Tin,
// extraction steam enthalpy line(p), drains cascaded in ins [[w kg/s, h J/kg], …] (they flash in the
// shell and give it their heat), heated fraction a of the tubes (1: none flooded):
// {T outlet target K, pe extraction pressure, ps shell pressure, ws extraction steam kg/s, hx, hd drain
// enthalpy, wv vent steam kg/s}. The outlet is Tin + a·(T_sat(ps) − TTD − Tin). Bisection on the shell
// saturation temperature: the valve flow falls and the condensing demand rises with it. With extraction
// steam the shell sits at most at the extraction pressure (non-return valve); if the cascaded drains then
// flash more steam than the feed condenses (little feed flow), the excess leaves through the shell's vent
// to the condenser (wv, saturated steam). Without extraction the drains alone set the shell temperature.
const stage=(H,wst,e,fw,Tin,line=LINE0,ins=[],a=1)=>{
  const pe=H.PE*Math.max(0,wst)/WTR,e1=clamp(e,0,1),Tlo=Tin+TTD,ext=e1>0&&pe>sat(Tlo).p;
  let Thi=Tlo;if(ext)Thi=Tsat(pe);else for(const[w,h]of ins)if(w>1e-9)Thi=Math.max(Thi,Thf(h));
  if(!(Thi>Tlo))return {T:Tin,pe,ps:0,ws:0,hx:0,hd:0,wv:0};
  const hin=hLiq(Tin,PR),hx=ext?line(pe):0,a1=clamp(a,0,1),To=m=>Tin+a1*(m-TTD-Tin);
  const dem=m=>{const q=sat(m),hf=q.hf,hg=q.hg;let Qx=0;for(const[w,h]of ins)Qx+=w*(h-hf);return (fw*(hLiq(To(m),PR)-hin)-Qx)/((ext?hx:hg)-hf)};
  if(!(fw>1e-6)&&!ins.some(([w])=>w>1e-9)){const q=sat(Thi);return {T:To(Thi),pe,ps:q.p,ws:0,hx,hd:q.hf,wv:0}}
  let lo=Tlo,hi=Thi;
  for(let i=0;i<40;i++){const m=(lo+hi)/2,p=sat(m).p,sup=ext?H.KX*e1*Math.sqrt(Math.max(0,pe*pe-p*p)):0;if(sup>dem(m))lo=m;else hi=m}
  const m=(lo+hi)/2,dm=dem(m),q=sat(m),ps=q.p,hd=q.hf,hg=q.hg;
  return {T:To(m),pe,ps,ws:ext?Math.max(0,dm):0,hx,hd,wv:ext&&dm<0?-dm*(hx-hd)/(hg-hd):0};
};
// HP heater (default inlet: its rated design inlet, LP outlet mixed with the drains) and LP heater
// (default inlet: the hotwell)
const heater=(wst,e,fw,Tin=THPI,line=LINE0,ins,a)=>stage(HP,wst,e,fw,Tin,line,ins,a);
const heaterL=(wst,e,fw,Tin=THW,line=LINE0,ins,a)=>stage(LP,wst,e,fw,Tin,line,ins,a);
// both stages in steady state for feed flow fw: HP drains cascaded to the LP shell, all drains pumped
// forward after the LP heater, so the LP heater carries fw − drains: {T HP outlet, lp, hp, Tm HP inlet, wd}
const train=(wst,eL,eH,fw,Tin=THW,line=LINE0)=>{let hp={ws:0,hd:0},lp=null,wd=0,Tm=Tin;
  for(let i=0;i<8;i++){lp=heaterL(wst,eL,Math.max(0,fw-wd),Tin,line,hp.ws>0?[[hp.ws,hp.hd]]:[]);
    const w2=Math.min(fw,hp.ws+lp.ws);Tm=mixT(fw-w2,lp.T,w2,lp.hd||hp.hd);hp=heater(wst,eH,fw,Tm,line);wd=w2}
  return {T:hp.T,lp,hp,Tm,wd}};
// shell water fraction full: HP (VHS) and LP with its drain tank (VLS), saturated at the shell pressure
// (density at the design shell temperature; the level is a volume fraction)
const RHS=sat(TFR+TTD).rf,RLS=sat(TLPD+TTD).rf,MHS=VHS*RHS,MLS=VLS*RLS;
// heated tube fraction at shell level L, and the high-level dump opening
const tubes=L=>clamp((1-L)/(1-LFL),0,1),dumpX=L=>clamp((L-LDU)/0.1,0,1);
// outlet of the train with both extraction valves at e
const heaterT=(wst,e,fw=wst,Tin=THW)=>train(wst,e,e,fw,Tin).T;
// steady heat balance at pressure P: steam flow W that heat Q makes from feed heated by that same W
// (the turbine passing W less the air ejector's motive steam)
const heatBalance=(Q,P,e=1)=>{const hg=sat(Tsat(P)).hg;let T=THW,W=0;
  for(let i=0;i<60;i++){W=Q/(hg-hLiq(T,P));T=heaterT(Math.max(0,W-WSJ),e,W)}return {W,Tfw:T}};
// hotwell mixing: h {M kg, T K}; ins = [[w kg/s, h J/kg], …] (negative w leaves at the hotwell
// temperature). Returns the new {M, T}.
const hotwellMix=(hw,ins,dt)=>{const h0=sat(hw.T).hf;let M=hw.M,U=M*h0;
  for(const[w,h]of ins){const dm=w*dt;M+=dm;U+=dm*(w>=0?h:h0)}
  return M>1?{M,T:Thf(U/M)}:{M:Math.max(0,M),T:hw.T}};
export {WTR,CSTM,HCST,ZMU,CRATE,KMU,KRJ,cstH,muFlow,rjBack,
  H0,KPU,KLN,KV,FRATE,NRATE,HWM,MKMAX,TMK,THW,PCD,TTD,PE,PEL,KXT,KXL,HX0,LINE0,TLPD,Thf,
  HC0,HF0,KCP,KPOL,KLP,KFP,ZS,HWD,NPR,PLO,TLO,WMIN,KR,RRATE,suction,cpFlow,network,shutoff,
  THPI,HDP0,ZDT,KDPP,KDVL,KDVH,KDDH,KDDL,WSD,WSLD,WDD,DPD,VHS,VLS,LFL,LDU,XD0,RHS,RLS,MHS,MLS,mixT,dpFlow,tubes,dumpX,
  fwFlow,fwValve,stage,heater,heaterL,train,heaterT,heatBalance,hotwellMix};
