// BWR core: point kinetics with reactivity feedback, one lumped fuel node, decay heat, and the core
// void fraction from the core thermal-hydraulics. Pure functions, no DOM. An ES module; imports
// common.js and bwr-vessel.js. bwr-sim.js composes it with the other systems. See docs/bwr.md.
//
// Generic BWR/4-class numbers. β, Λ and the delayed-neutron groups are textbook values (U-235 group
// shape, Keepin; β lowered to 0.0056 for a mid-cycle core with plutonium); the feedback coefficients
// are round generic beginning-of-cycle values, illustrative, not from one plant's data.
import {sat,Tsat,clamp} from './common.js';
import {Z} from './bwr-vessel.js';
// rated thermal power, dome pressure and core flow (BWR/4 class: 3293 MW, 7.0 MPa, 12,900 kg/s ≈ 102 Mlb/h)
const QC=3293e6,PRC=7.0e6,WCR=12900;
// six delayed-neutron groups: decay constants (1/s) and fractions of β
const LAM=Object.freeze([0.0124,0.0305,0.111,0.301,1.14,3.01]),BF=Object.freeze([0.033,0.219,0.196,0.395,0.115,0.042]);
const BETA=0.0056,LAMBDA=4e-5,BI=BF.map(b=>b*BETA);
// neutron source (W in power units, i.e. Λ·S·energy per fission): a subcritical core sits at about
// SRC/(β − ρ), some tens of watts with the rods in. Illustrative.
const SRC=1;
// reactivity feedback: void −1×10⁻³ per % core-average void; Doppler −2×10⁻⁵ /K on the lumped fuel
// temperature; moderator −2×10⁻⁵ /K on the core inlet temperature. Reference: hot zero power (no void,
// fuel and inlet at T_sat(7.0 MPa)). Illustrative generic BOC values.
const AV=-1e-3,AD=-2e-5,AM=-2e-5,TREF=Tsat(PRC);
// fuel: one node, heat capacity MCF (J/K) and resistance to the coolant RF (K/W), time constant
// MCF·RF = 5 s; about 615 °C average at rated. All fission and decay heat is deposited in the fuel.
const MCF=5e7,RF=1e-7;
// Decay heat: seven exponential groups D_j driven by the fission power, dD_j/dt = λ_j·(f_j·n − D_j).
// After long operation at n they sum to 6.9 % of n (6.5 % of the total) and decay as 0.0695·(1 + t)^−0.2
// within 3 % up to 3 days (least-squares fit to the Way–Wigner shape the old decayFrac uses).
const DL=Object.freeze([1/1.5,1/15,1/150,1/1500,1/15000,1/150000,1/1.5e6]),DF=Object.freeze([0.01551,0.01929,0.01269,0.00797,0.00534,0.00223,0.00631]);
const DSUM=DF.reduce((a,b)=>a+b,0);
// Core thermal-hydraulics: flow area AF (764 bundles × ~0.0098 m², illustrative), heated length LC
// (bottom to top of active fuel). Uniform axial power: the water heats to saturation over the boiling
// boundary zb = W·Δh_in/Q, then the quality rises linearly to the exit quality x_e = (Q − W·Δh_in)/(W·h_fg).
// Void from the Zuber–Findlay drift flux: α = x/(C0·(x + (1 − x)·ρ″/ρ′) + ρ″·Vgj/G), C0 1.1, Vgj 0.25 m/s.
// Core average over the length (Simpson, 8 intervals); the chimney above sees the exit void.
// The voids follow their targets with TAUV (transit and bubble growth, illustrative).
// Void volume for the level: the core (AF·LC) at the core-average void and the chimney, standpipes and
// separators (VCH, 30 m³ of mixture, illustrative) at the exit void.
const AF=7.5,LC=Z.TAF-Z.BAF,C0=1.1,VGJ=0.25,TAUV=0.7,WMIN=100,VCH=30;
const voidVol=k=>AF*LC*k.ac+VCH*k.ae;
const voids=(Q,W,dh,P)=>{
  const q=sat(Tsat(P)),rg=q.rg,rf=q.rf,hfg=q.hg-q.hf,w=Math.max(W,WMIN),G=w/AF,dh1=Math.max(0,dh);
  if(!(Q>w*dh1))return {ac:0,ae:0,xe:0,zb:1};
  const zb=w*dh1/Q,xe=Math.min(1,(Q-w*dh1)/(w*hfg)),a=x=>x>0?x/(C0*(x+(1-x)*rg/rf)+rg*VGJ/G):0;
  let m=0;for(let i=0;i<=8;i++)m+=(i===0||i===8?1:i%2?4:2)*a(xe*i/8);
  return {ac:(1-zb)*m/24,ae:a(xe),xe,zb};
};
// Control rods: one lever, x 0 (all in) … 1 (all out), standing for the whole rod pattern. Integral
// worth S-shaped from a sin² differential worth, ρ_rod = WT·(F(x) − F(X0)), F(x) = x − sin(2πx)/(2π).
// Critical at hot zero power at X0 = 60 %; WT chosen so that the rods balance the rated feedback
// (void, Doppler, moderator) at XR = 80 %. Withdrawal/insertion rate RODRATE (0.1 %/s, illustrative);
// scram drives them fully in in TSCRAM (3 s, a placeholder for the CRD hydraulics of step 13).
const F=x=>x-Math.sin(2*Math.PI*x)/(2*Math.PI),X0=0.6,XR=0.8,RODRATE=0.001,TSCRAM=3;
// feedback reactivity at fuel temperature Tf, core-average void ac, inlet temperature Tin (K)
const feedback=(Tf,ac,Tin)=>AV*100*ac+AD*(Tf-TREF)+AM*(Tin-TREF);
// steady fuel temperature at heat q and coolant temperature Tc
const fuelT=(q,Tc)=>Tc+q*RF;
// rated design: feedback at rated power, flow and inlet subcooling (feed at 216 °C mixed into the core flow)
const RATED=(()=>{const q=sat(TREF),hf=q.hf,hfg=q.hg-q.hf,hfw=sat(216+273.15).hf,W=QC/(hfg+hf-hfw),dh=W*(hf-hfw)/WCR,v=voids(QC,WCR,dh,PRC);
  const cp=sat(TREF+0.5).hf-sat(TREF-0.5).hf;return {dh,v,fb:feedback(fuelT(QC,TREF),v.ac,TREF-dh/cp)}})();
const WT=-RATED.fb/(F(XR)-F(X0));
const rodRho=x=>WT*(F(clamp(x,0,1))-F(X0));
// rod position for reactivity r (bisection; rodRho rises with x)
const rodFor=r=>{let a=0,b=1;for(let i=0;i<50;i++){const m=(a+b)/2;if(rodRho(m)<r)a=m;else b=m}return (a+b)/2};
// Steady core at heat to coolant q (W), coolant temperature Tc, inlet temperature Tin, voids v: fission
// power n with the decay groups at equilibrium (long operation at n), precursors at β_i·n, fuel at its
// steady temperature, and the rod position x that makes it so (net reactivity −SRC/n, the source's share;
// default: solved for). With no heat (q ≤ 0) the core is shut down: rods at x (default all in), source
// level n = SRC/(β − ρ), no decay heat.
const coreInit=(q,Tc,Tin,v,x)=>{
  if(!(q>0)){const xs=x??0,r=rodRho(xs)+feedback(Tc,0,Tin),n=r<0?SRC/(BETA-r):SRC/BETA;
    return {n,c:BI.map(b=>b*n),D:DF.map(()=>0),Tf:Tc,x:xs,ac:0,ae:0,xe:0,zb:1,rho:r,per:Infinity,q:n,pc:false}}
  const n=q/(1+DSUM),c=BI.map(b=>b*n),D=DF.map(f=>f*n),Tf=fuelT(q,Tc);
  const fb=feedback(Tf,v.ac,Tin),xs=x!=null?x:rodFor(-fb-SRC/n);
  return {n,c,D,Tf,x:xs,ac:v.ac,ae:v.ae,xe:v.xe,zb:v.zb,rho:rodRho(xs)+fb,per:Infinity,q,pc:false};
};
// Advance the core by dt in substeps of at most HSUB. inp: {P dome pressure, Tc coolant (saturation)
// temperature, dh core inlet subcooling J/kg, Tin inlet temperature, rod 0–1 lever demand (undefined:
// stay), scram true while scrammed, heat W fixed heat to coolant (test hook: no kinetics, the fuel node
// and kinetics stand still)}. flow(h, k) advances the core flow by h given the core state k (voids) and
// returns the core flow W (kg/s). Returns the mean heat to the coolant over dt (W).
// Kinetics: prompt-jump approximation with the precursors c_i in power units (c_i = Λ·λ_i·C_i):
// n = (Σc_i + SRC)/(β − ρ), dc_i/dt = λ_i·(β_i·n − c_i); implicit Euler for c_i, solved together with n.
// ρ ≥ β (prompt critical; the approximation fails there) latches k.pc, which the plant reports as a fault.
const HSUB=0.05;
const coreStep=(k,inp,dt,flow)=>{
  const N=Math.max(1,Math.ceil(dt/HSUB-1e-9)),h=dt/N,fix=typeof inp.heat==='number',n0=k.n;let qs=0;
  for(let j=0;j<N;j++){
    // rods
    if(inp.scram)k.x=Math.max(0,k.x-h/TSCRAM);
    else if(inp.rod!=null)k.x+=clamp(clamp(inp.rod,0,1)-k.x,-RODRATE*h,RODRATE*h);
    let q;
    if(fix){q=inp.heat;k.q=q;k.Tf=fuelT(Math.max(0,q),inp.Tc)}
    else{
      const rho=rodRho(k.x)+feedback(k.Tf,k.ac,inp.Tin);k.rho=rho;
      let S=SRC,B=0;const a=LAM.map(l=>1/(1+h*l));
      for(let i=0;i<6;i++){S+=a[i]*k.c[i];B+=a[i]*h*LAM[i]*BI[i]}
      const den=BETA-rho-B;
      if(rho>=BETA||den<=0.01*BETA)k.pc=true;
      else{const n1=S/den;k.n=n1;for(let i=0;i<6;i++)k.c[i]=a[i]*(k.c[i]+h*LAM[i]*BI[i]*n1)}
      const n=k.n;
      let Dt=0;for(let i=0;i<DL.length;i++){const e=Math.exp(-h*DL[i]);k.D[i]=DF[i]*n+(k.D[i]-DF[i]*n)*e;Dt+=k.D[i]}
      // fuel node, exact for the substep; the heat to the coolant conserves the fuel's energy
      const Pt=n+Dt,Tf0=k.Tf,Te=fuelT(Pt,inp.Tc);k.Tf=Te+(Tf0-Te)*Math.exp(-h/(MCF*RF));
      q=Pt-MCF*(k.Tf-Tf0)/h;k.q=Pt;
    }
    qs+=q;
    const W=flow(h,k),v=voids(q,W,inp.dh,inp.P),e=1-Math.exp(-h/TAUV);
    k.ac+=(v.ac-k.ac)*e;k.ae+=(v.ae-k.ae)*e;k.xe=v.xe;k.zb=v.zb;k.W=W;
  }
  // reactor period from the change of the fission power over the step
  if(!fix){const r=k.n/n0;k.per=r>0&&Math.abs(Math.log(r))>1e-9?dt/Math.log(r):Infinity}
  return qs/N;
};
export {VCH,voidVol,QC,PRC,WCR,LAM,BF,BETA,LAMBDA,BI,SRC,AV,AD,AM,TREF,MCF,RF,DL,DF,DSUM,AF,LC,C0,VGJ,TAUV,WMIN,X0,XR,WT,RODRATE,TSCRAM,HSUB,RATED,
  voids,feedback,fuelT,rodRho,rodFor,coreInit,coreStep};
