// BWR recirculation: one loop standing for the two loops and 20 jet pumps of a BWR/4, its pump driven
// by a variable-speed motor-generator (MG) set, and the core flow loop with natural circulation. Pure
// functions, no DOM. An ES module; imports common.js and bwr-core.js. See docs/bwr.md.
//
// Flow paths: the pump draws drive flow Wd from the downcomer and pushes it through the jet pump
// nozzles; the jets entrain suction flow Ws = Wc − Wd from the downcomer, and the mixed flow Wc goes
// through the diffusers to the lower plenum, up through the core and chimney, through the separators
// and back down the downcomer. Two momentum equations, I·dW/dt = ΔP(W):
//   drive:  ID·dWd/dt = ΔP_pump(n, Wd) − KLP·Wd|Wd| − (1 + KN)·Wd|Wd|/(2ρAN²) + ΔP_suction
//   (the nozzles discharge into the throat, which sits ΔP_suction below the downcomer)
//   core:   IC·dWc/dt = ΔP_jet + ΔP_natural − (KC1 + KC2·φ²)·Wc|Wc|/ρ
// Jet pump (one-dimensional mixing-section momentum balance over the throat area AT): the nozzle jet
// (area AN) and the suction stream (area AT − AN) mix to one stream; ΔP_suction = (1 + KS)·Ws|Ws|/(2ρ(AT − AN)²)
// accelerates the suction flow, the diffuser recovers KDF of the throat velocity head, throat friction
// KFT. ΔP_jet = discharge − downcomer pressure. Natural circulation head: the downcomer water column
// against the two-phase core and chimney, g·[(LC + HCH)·ρ_dc − LC·ρ̄_core − HCH·ρ̄_chimney]. Core
// resistance: a single-phase part (inlet orifices, lower tie plates) and a two-phase part (spacers,
// upper tie plates, separators) with the homogeneous multiplier φ² = 1 + x_e·(ρ′/ρ″ − 1).
// The numbers are illustrative, chosen for a BWR/4: rated drive flow 4300 kg/s at 1.6 MPa pump head,
// M ratio Ws/Wd = 2, core flow 12,900 kg/s; the loss coefficients KLP, KC1 and KC2 are set so that the
// rated point is exactly that. Nothing here is fitted to plant data.
import {sat,Tsat,clamp,liqRho,cpL} from './common.js';
import {WCR,LC,RATED,TREF,PRC} from './bwr-core.js';
const g=9.81,WDR=4300,HPR=1.6e6,AN=0.09,AT=0.566,KN=0.05,KS=0.1,KDF=0.73,KFT=0.1,HCH=5.3,ID=150,IC=15;
// pump: ΔP = n²·HS − KPP·W|W|, shutoff 1.25 × the rated head; the MG set speed follows its demand
// through the speed limiter at MGRATE (2 %/s), never below NMIN (20 %) while the pump breaker is closed;
// with the breaker open the pump coasts down with TCD (5 s). Pump speed = MG set speed (no slip).
const HS=1.25*HPR,KPP=(HS-HPR)/WDR**2,MGRATE=0.02,NMIN=0.2,TCD=5;
// signed square for the losses, linear below 2 % of rated flow (laminar-like), so a stopped loop
// settles instead of creeping
const sq=x=>x*Math.max(Math.abs(x),0.02*WDR);
const jet=(Wd,Wc,r)=>{const Ws=Wc-Wd,As=AT-AN,suc=(1+KS)*sq(Ws)/(2*r*As*As);
  return {dp:(sq(Wd)/(r*AN)+sq(Ws)/(r*As)-sq(Wc)/(r*AT))/AT-suc+(KDF-KFT)*sq(Wc)/(2*r*AT*AT),suc}};
// natural circulation head: downcomer density rd, core liquid rc, steam rg, voids ac (core) ae (chimney)
const natural=(rd,rc,rg,ac,ae)=>g*((LC+HCH)*rd-LC*((1-ac)*rc+ac*rg)-HCH*((1-ae)*rc+ae*rg));
// rated design: liquid at the rated core inlet, saturated core liquid and steam at 7 MPa
const DES=(()=>{const q=sat(TREF),rc=q.rf,rg=q.rg,r=liqRho(TREF-RATED.dh/cpL(TREF),PRC),v=RATED.v,phi=1+v.xe*(rc/rg-1);
  const J=jet(WDR,WCR,r),KLP=(HPR-(1+KN)*WDR**2/(2*r*AN*AN)+J.suc)/WDR**2,D=J.dp+natural(r,rc,rg,v.ac,v.ae);
  return {r,KLP,KC1:0.4*D*r/WCR**2,KC2:0.6*D*r/(WCR**2*phi),D,jet:J.dp,nat:natural(r,rc,rg,v.ac,v.ae)}})();
const {KLP,KC1,KC2}=DES;
// loop residuals [drive, core] (Pa) at flows Wd, Wc, pump speed n; p {rd downcomer density, rc, rg, ac, ae, xe}
const resid=(Wd,Wc,n,p)=>{const J=jet(Wd,Wc,p.rd),phi=1+Math.max(0,p.xe)*(p.rc/p.rg-1);
  return [n*n*HS-KPP*sq(Wd)-KLP*sq(Wd)-(1+KN)*sq(Wd)/(2*p.rd*AN*AN)+J.suc,
    J.dp+natural(p.rd,p.rc,p.rg,p.ac,p.ae)-(KC1+KC2*phi)*sq(Wc)/p.rd]};
// MG set and pump speed over h: breaker r.on, speed demand r.dm 0–1
const mgStep=(r,h)=>{
  if(r.on){const t=clamp(r.dm,NMIN,1);r.ng+=clamp(t-r.ng,-MGRATE*h,MGRATE*h)}
  else r.ng*=Math.exp(-h/TCD);
};
// One substep h of the loop momentum: implicit Euler linearised about the present flows (numerical
// Jacobian), stable for the stiff friction terms at any h used here.
const loopStep=(r,p,h)=>{
  const F=resid(r.Wd,r.Wc,r.ng,p),e=[Math.max(1,1e-4*Math.abs(r.Wd)),Math.max(1,1e-4*Math.abs(r.Wc))];
  const Fd=resid(r.Wd+e[0],r.Wc,r.ng,p),Fc=resid(r.Wd,r.Wc+e[1],r.ng,p);
  const a=ID-h*(Fd[0]-F[0])/e[0],b=-h*(Fc[0]-F[0])/e[1],c=-h*(Fd[1]-F[1])/e[0],d=IC-h*(Fc[1]-F[1])/e[1],det=a*d-b*c;
  r.Wd+=h*(d*F[0]-b*F[1])/det;r.Wc+=h*(a*F[1]-c*F[0])/det;
};
// new loop state: breaker rp (default closed), speed n (default 1), flows from a steady solve with p
const recircInit=(rp,n,p)=>{const r={on:rp,ng:rp?Math.max(n,NMIN):0,dm:n,Wd:rp?WDR*n:0,Wc:WCR*Math.max(n,0.3)};
  for(let i=0;i<1500;i++)loopStep(r,p,0.2);return r};
export {WDR,HPR,AN,AT,KN,KS,KDF,KFT,HCH,ID,IC,HS,KPP,MGRATE,NMIN,TCD,DES,KLP,KC1,KC2,jet,natural,resid,mgStep,loopStep,recircInit};
