// BWR turbine island: HP turbine, moisture separator reheater (pass-through), LP turbine, and the
// generator on one shaft with its grid breaker. Pure functions, no DOM. An ES module; imports common.js
// and bwr-steam.js. See docs/bwr.md, "Turbine and generator".
import {sat,satS,Tsat,clamp} from './common.js';
import {PR,PL0} from './bwr-steam.js';
// ---- expansion lines ----
// entropy of steam at (p, h): wet from the saturation tables, a little superheat as dh/T
const sPH=(p,h)=>{const T=Tsat(p),q=sat(T),hf=q.hf,hg=q.hg,[sf,sg]=satS(T);
  return h<=hg?sf+(h-hf)/(hg-hf)*(sg-sf):sg+(h-hg)/T};
// one expansion line from (p0, h0) with isentropic efficiency eta: h at p is h0 − eta·(h0 − h_s(p)),
// h_s the isentropic enthalpy from the inlet entropy
const expansion=(p0,h0,eta=ETAH)=>{const s0=sPH(p0,h0);
  return p=>{if(!(p<p0))return h0;const T=Tsat(p),q=sat(T),hf=q.hf,hg=q.hg,[sf,sg]=satS(T),x=(s0-sf)/(sg-sf);
    const hs=x<=1?hf+x*(hg-hf):hg+(s0-sg)*T;return h0-eta*(h0-hs)}};
// ---- stage pressures ----
// The steam passes the stop and control valves (throttled at constant enthalpy from the header) into the
// HP turbine's first stage at p1, expands through the HP turbine (the HP heater's extraction point at peH
// on the way) to the HP exhaust pX, crosses the MSR to the LP inlet pLi, and expands through the LP
// turbine (the LP heater's extraction at peL) to the condenser. Four stage groups between those points,
// each obeying Stodola's cone law, w = K·√((p_in² − p_out²)/(p_in·v_in)); p·v at each group's inlet is
// taken at its rated value (it moves a few % over the load range), so w = K·√(p_in² − p_out²), solved
// backwards from the condenser: p_in = √(p_out² + (w/K)²). So the pressures fall with the flow, roughly in
// proportion at high load and towards the condenser pressure at none. The MSR and crossover piping drop
// DMSR·(w/w_R)² (4 % of the HP exhaust at rated).
// Rated points (illustrative, typical of a BWR/4 turbine, not from one plant): first stage P1R 88 % of the
// rated throttle; HP exhaust PXR 1.38 MPa (about 200 psia); the extraction pressures are those the heaters
// were designed on (bwr-feed.js), passed to calib().
const P1R=0.88*PL0,PXR=1.38e6,DMSR=0.055e6,PLIR=PXR-DMSR;
// Isentropic efficiencies of the two turbines (illustrative, chosen with the pass-through MSR so the
// gross output at rated is about 1120 MWe; a real HP turbine works in wet steam and loses more, a real
// MSR dries and reheats the steam before the LP turbine and gains it back)
const ETAH=0.83,ETAL=0.87;
// the expansion line of the whole turbine: HP from (p1, h0) to the exhaust pX, the MSR at constant
// enthalpy, LP from (pLi, hX) to the condenser. A function h(p); it carries the stage pressures as .ps.
const turbineLine=(ps,h0)=>{const hp=expansion(ps.p1,h0,ETAH),hX=hp(ps.pX),lp=expansion(ps.pLi,hX,ETAL);
  const f=p=>p>=ps.pX?hp(p):p>=ps.pLi?hX:lp(p);f.ps=ps;f.hX=hX;return f};
// Stodola constants from the rated flows: wt turbine flow, wsH, wsL the two extractions, pc condenser,
// peH, peL the rated extraction pressures
const calib=({wt,wsH,wsL,pc,peH,peL})=>{const g=(w,a,b)=>w/Math.sqrt(a*a-b*b);
  return Object.freeze({K1:g(wt,P1R,peH),K2:g(wt-wsH,peH,PXR),K3:g(wt-wsH,PLIR,peL),K4:g(wt-wsH-wsL,peL,pc),W3:wt-wsH})};
// stage pressures for turbine flow wt, extractions wsH, wsL, condenser pc; the first stage never above the
// header Pl (the control valves can't make pressure)
const stagePs=(K,wt,wsH,wsL,pc,Pl=Infinity)=>{
  const w1=Math.max(0,wt),w2=Math.max(0,w1-wsH),w4=Math.max(0,w2-wsL),up=(p,w,k)=>Math.sqrt(p*p+(w/k)**2);
  const peL=up(pc,w4,K.K4),pLi=up(peL,w2,K.K3),pX=pLi+DMSR*(w2/K.W3)**2,peH=up(pX,w2,K.K2),p1=Math.min(up(peH,w1,K.K1),Math.max(Pl,pX));
  return {p1,peH,pX,pLi,peL,pc}};
// shaft work (W) of the two turbines for turbine flow wt entering at h0 along line (turbineLine), the
// extractions wsH at line(peH) and wsL at line(peL), the rest exhausting at hx: {hp, lp}
const work=(wt,h0,line,wsH,wsL,hx)=>{const ps=line.ps,hH=line(ps.peH),hX=line.hX,hL=line(ps.peL);
  const w2=Math.max(0,wt-wsH),w4=Math.max(0,w2-wsL);
  return {hp:wt*(h0-hH)+w2*(hH-hX),lp:w2*(hX-hL)+w4*(hL-hx)}};
// ---- shaft, generator and grid ----
// One shaft: HP, LP, generator, at the rated NR 1800 rpm (four-pole, 60 Hz). Inertia constant H 5 s on the
// rated gross output (illustrative for a large nuclear unit): the stored energy at speed ω (fraction of
// rated) is E = H·PGR·ω². The generator turns ETAG of its shaft power into electricity. Windage and friction
// PW0·ω³ (0.5 % of rated at rated speed, illustrative); its heat goes to the exhaust steam.
// The LP turbine's power follows its steam with the lag TRH (5 s): the steam held in the MSR and the
// crossover piping keeps driving it after the control valves close (the usual reheat-turbine model). The
// combined intermediate (intercept) valves ahead of the LP turbine cut that off: their position iv scales
// the LP power. They shut at once on a turbine trip (stop valve below 90 % open) and on the fast closure;
// with the breaker open they are governed on speed, fully open below 102 % and shut above 105 %, so the
// steam held in the MSR cannot overspeed an unloaded turbine; they reopen at IVRATE per second.
// Breaker closed: the shaft is held at grid speed (an infinite grid at 60 Hz) and the electrical output is
// ETAG·(shaft power − windage). Open: the shaft speeds up or slows down with the unbalance.
const NR=1800,HIN=5,ETAG=0.985,PW0F=0.005,TRH=5,IVRATE=0.1;
// Power–load unbalance (PLU, part of the turbine's electro-hydraulic control): when the turbine power (from
// the LP inlet pressure, as the crossover pressure signal) exceeds the generator load by PLU (40 % of
// rated), the control valves and the intercept valves fast-close (about 0.15 s, inside one step); they are
// held shut while the unbalance stays above it. Overspeed trip at 110 % (protection, bwr-rps.js). Sync check:
// the breaker closes only within SYNC (0.5 %) of grid speed. Reverse power: shaft power below the windage
// (the generator motoring the turbine) for TRP 10 s trips the breaker. A turbine trip (stop valve below 10 %
// open) trips the generator breaker at once. Every trip of the breaker locks it open until the operator
// opens it (breaker(s, false)) and closes it again.
const PLU=0.4,SYNC=0.005,TRP=10;
// shaft state: w speed (fraction of rated), brk breaker closed, lock tripped open, iv intercept valve
// position, PL lagged LP power (W), plu fast closure in, trp reverse power timer (s)
const tgInit=(o={})=>({w:o.w??(o.brk?1:0),brk:!!o.brk,lock:false,iv:1,PL:o.PL||0,plu:false,trp:0});
// one step of the shaft: hp, lp the steam's turbine work (W), PGR rated gross output (W), dt. Updates tg;
// returns {Pe electrical W (step average), Pm shaft W (average), Pw windage W (average)}
const tgStep=(tg,hp,lp,PGR,dt)=>{
  const n=Math.max(1,Math.ceil(dt/0.05)),h=dt/n,E0=HIN*PGR,a=Math.exp(-h/TRH),PW=PGR*PW0F;let Pe=0,Pm=0,Pw=0;
  for(let i=0;i<n;i++){
    tg.PL=lp+(tg.PL-lp)*a;const pm=hp+tg.iv*tg.PL,pw=PW*tg.w**3;
    if(tg.brk){tg.w=1;Pe+=ETAG*(pm-pw)}
    else{const E=Math.max(0,E0*tg.w*tg.w+(pm-pw)*h);tg.w=Math.sqrt(E/E0)}
    Pm+=pm;Pw+=pw}
  return {Pe:Pe/n,Pm:Pm/n,Pw:Pw/n};
};
// the operator's breaker switch: close (sync check against grid speed) or open. Returns '' or why it did
// not close. log(text) records it.
const breaker=(tg,close,log=()=>{})=>{
  if(!close){if(tg.brk)log('Generator breaker opened');tg.brk=false;tg.lock=false;return ''}
  if(tg.brk)return '';
  if(tg.lock){log('Generator breaker: tripped, open it before closing');return 'lock'}
  if(Math.abs(tg.w-1)>SYNC){log(`Sync check: ${(tg.w*NR).toFixed(0)} rpm, not closed`);return 'sync'}
  tg.brk=true;tg.trp=0;log('Generator breaker closed');return ''};
// protective trips of the breaker, once per step before the valves move: sv stop valve position, Pm the
// last shaft power, Pw the last windage
const brkTrips=(tg,sv,Pm,Pw,dt,log=()=>{})=>{
  if(!tg.brk)return;
  if(sv<0.1){tg.brk=false;tg.lock=true;log('Generator trip (turbine tripped)');return}
  tg.trp=Pm<Pw?tg.trp+dt:0;
  if(tg.trp>=TRP){tg.brk=false;tg.lock=true;tg.trp=0;log('Generator trip (reverse power)')}};
// power–load unbalance and intercept valves: pLi the LP inlet pressure, Pe the generator load (W, 0 with the
// breaker open), sv the stop valve position: sets tg.plu and moves tg.iv
const pluStep=(tg,pLi,Pe,PGR,sv,dt)=>{
  tg.plu=pLi/PLIR-Math.max(0,Pe)/PGR>PLU;
  const t=tg.plu||sv<0.9?0:tg.brk?1:clamp((1.05-tg.w)/0.03,0,1);
  tg.iv=t<tg.iv?t:Math.min(t,tg.iv+IVRATE*dt)};
export {sPH,expansion,P1R,PXR,DMSR,PLIR,ETAH,ETAL,turbineLine,calib,stagePs,work,
  NR,HIN,ETAG,PW0F,TRH,IVRATE,PLU,SYNC,TRP,tgInit,tgStep,breaker,brkTrips,pluStep};
