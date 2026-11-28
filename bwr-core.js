// BWR core: point kinetics with reactivity feedback through a quasi-static axial power shape, one lumped
// fuel node, decay heat, and the axial void distribution from the core thermal-hydraulics. Pure functions, no DOM. An ES module; imports
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
// reactivity feedback, local along the core (see the axial shape below): void −1×10⁻³ per % void; Doppler
// −2×10⁻⁵ /K on the fuel temperature; moderator −2×10⁻⁵ /K on the subcooled water. Reference: hot zero
// power (no void, fuel and water at T_sat(7.0 MPa)). Generic values: the GE BWR technology manual lists
// −1×10⁻³ per % void and −1×10⁻⁵ per °F fuel (−1.8×10⁻⁵ /K) as typical.
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
// (bottom to top of active fuel), NZ axial nodes. The heat goes into the coolant along the axial power
// shape s (node values, mean 1). The water heats to saturation (boiling boundary zb, where the heat
// so far is W·Δh_in), then the equilibrium quality rises with the heat to the exit quality
// x_e = (Q − W·Δh_in)/(W·h_fg). Void from the Zuber–Findlay drift flux: α = x/(C0·(x + (1 − x)·ρ″/ρ′) +
// ρ″·Vgj/G), C0 1.1, Vgj 0.25 m/s; each node's void is the mean over four points inside it, the core
// average the mean over the nodes; the chimney above sees the exit void. The node voids follow their
// targets with TAUV (transit and bubble growth, illustrative).
// Void volume for the level: the core (AF·LC) at the core-average void and the chimney, standpipes and
// separators (VCH, 30 m³ of mixture, illustrative) at the exit void.
const AF=7.5,LC=Z.TAF-Z.BAF,C0=1.1,VGJ=0.25,TAUV=0.7,WMIN=100,VCH=30,NZ=24;
const voidVol=k=>AF*LC*k.ac+VCH*k.ae;
// voids along the core for heat Q, flow W, inlet subcooling dh (J/kg), pressure P and power shape s.
// Returns {ac core average, ae exit, xe exit quality, zb boiling boundary (fraction of LC), a node voids,
// hs node subcooling J/kg (0 where boiling)}. Without s: the steady shape for these conditions (steady()).
const voids=(Q,W,dh,P,s)=>{
  if(!s)return steady(Q,W,dh,P).v;
  const q=sat(Tsat(P)),rg=q.rg,rf=q.rf,hfg=q.hg-q.hf,w=Math.max(W,WMIN),G=w/AF,dh1=Math.max(0,dh),Q1=Math.max(0,Q);
  const al=x=>x>0?(x=Math.min(1,x),x/(C0*(x+(1-x)*rg/rf)+rg*VGJ/G)):0;
  const a=new Array(NZ),hs=new Array(NZ);let c0=0,ac=0,zb=1;
  for(let i=0;i<NZ;i++){
    const c1=c0+s[i]/NZ;let m=0,h=0;
    for(let j=0;j<4;j++){const he=Q1*(c0+(c1-c0)*(j+0.5)/4)/w-dh1;m+=al(he/hfg);h+=Math.max(0,-he)}
    a[i]=m/4;hs[i]=h/4;ac+=a[i]/NZ;
    if(zb===1&&Q1*c1>w*dh1&&Q1>0)zb=(i+(w*dh1/Q1-c0)/(c1-c0))/NZ;
    c0=c1;
  }
  const xe=Q1>w*dh1?Math.min(1,(Q1-w*dh1)/(w*hfg)):0;
  return {ac,ae:al(xe),xe,zb,a,hs};
};
// Axial power shape (roadmap: the fix for the cold-feed response). The fission power is one point-kinetics
// amplitude n times a quasi-static axial shape from one-group diffusion over the NZ nodes:
// −M²·φ″ + φ = (k∞(z)/k)·φ, zero flux DEXT beyond the ends of the active fuel (extrapolation distance).
// k∞(z) = G(z)·(1 + f(z)), f the local feedback: AV·100·α(z) + AD·(T_fuel(z) − TREF) + AM·(T_water(z) − TREF),
// with the lumped fuel temperature spread along the shape (T_fuel = Tc + (Tf − Tc)·s) and the water below
// saturation in the subcooled nodes. G(z) is fixed: it stands for the fuel loading, burnable poison and
// the rod pattern's shape, chosen so that at rated the shape is STGT (bottom-skewed, peaking about 1.45
// near 35 % of the height, a generic beginning-of-cycle BWR shape), and scaled so that with no feedback
// (hot zero power) k = 1. The rod lever stays one uniform reactivity, as before. The feedback reactivity
// is ρ_fb = 1 − 1/k, from the Rayleigh quotient of the current shape (first order in the shape error); the
// shape follows the feedback by two shifted inverse (Wielandt) iterations per substep. Without the shape a
// colder feed lost voids low in the core but the extra power went in all along it, so the power rose about
// twice as much as in a real core (docs/bwr.md, "Axial shape"). M² 65 cm² (hot, voided BWR lattice, round
// textbook value), DEXT 0.1 m; illustrative.
const M2=0.0065,DEXT=0.1,SHIFT=0.003,DZ=LC/NZ,CZ=M2/(DZ*DZ),CB=M2/(DZ*(DZ/2+DEXT));
const zc=i=>(i+0.5)/NZ;
const Adiag=i=>i===0||i===NZ-1?1+CZ+CB:1+2*CZ;
const Amul=p=>p.map((v,i)=>Adiag(i)*v-(i>0?CZ*p[i-1]:0)-(i<NZ-1?CZ*p[i+1]:0));
// solve (A − mu·diag(F))·x = b (Thomas)
const tri=(F,mu,b)=>{
  const c=new Array(NZ),d=new Array(NZ),x=new Array(NZ);let m=Adiag(0)-mu*F[0];c[0]=-CZ/m;d[0]=b[0]/m;
  for(let i=1;i<NZ;i++){m=Adiag(i)-mu*F[i]+CZ*c[i-1];c[i]=-CZ/m;d[i]=(b[i]+CZ*d[i-1])/m}
  x[NZ-1]=d[NZ-1];for(let i=NZ-2;i>=0;i--)x[i]=d[i]-c[i]*x[i+1];return x};
// λ = 1/k for shape p (Rayleigh quotient)
const lam=(F,p)=>{const Ap=Amul(p);let a=0,f=0;for(let i=0;i<NZ;i++){a+=p[i]*Ap[i];f+=F[i]*p[i]*p[i]}return a/f};
// it shifted inverse iterations on p in place (kept at mean 1); returns λ of the new shape
const iterate=(F,p,it)=>{
  for(let j=0;j<it;j++){
    const l=lam(F,p);let x=tri(F,l-SHIFT,p.map((v,i)=>F[i]*v));
    if(!x.every(v=>v>0&&isFinite(v)))x=tri(F,0,p.map((v,i)=>F[i]*v)); // plain inverse iteration
    let m=0;for(let i=0;i<NZ;i++)m+=x[i];m/=NZ;for(let i=0;i<NZ;i++)p[i]=x[i]/m;
  }
  return lam(F,p);
};
// local feedback along the core: node voids a, node subcooling hs (J/kg), lumped fuel Tf, coolant Tc, shape s
const localFb=(a,hs,Tf,Tc,s)=>{const cp=sat(Tc+0.5).hf-sat(Tc-0.5).hf;
  return a.map((ai,i)=>AV*100*ai+AD*(Tc+(Tf-Tc)*s[i]-TREF)+AM*(Tc-hs[i]/cp-TREF))};
const kinf=f=>f.map((v,i)=>GZ[i]*(1+v));
// steady fuel temperature at heat q and coolant temperature Tc
const fuelT=(q,Tc)=>Tc+q*RF;
// the generic rated shape: chopped sine with zeros DEXT beyond the fuel, skewed to the bottom by BSK
const BSK=0.35,STGT=(()=>{const r=[...Array(NZ)].map((_,i)=>Math.sin(Math.PI*(zc(i)*LC+DEXT)/(LC+2*DEXT))*(1+BSK*(1-2*zc(i))));
  const m=r.reduce((a,b)=>a+b,0)/NZ;return Object.freeze(r.map(v=>v/m))})();
// rated design (feed at 216 °C mixed into the core flow), G(z) from the rated shape, scaled to k = 1 at hot zero power
let GZ;
const RATED=(()=>{const q=sat(TREF),hf=q.hf,hfg=q.hg-q.hf,hfw=sat(216+273.15).hf,W=QC/(hfg+hf-hfw),dh=W*(hf-hfw)/WCR,s=[...STGT],v=voids(QC,WCR,dh,PRC,s);
  const f=localFb(v.a,v.hs,fuelT(QC,TREF),TREF,s),As=Amul(s);GZ=s.map((si,i)=>As[i]/(si*(1+f[i])));
  const p=Array(NZ).fill(1),lh=iterate(GZ,p,200);GZ=Object.freeze(GZ.map(g=>g*lh));
  return {dh,v,s:STGT,fb:1-lam(kinf(f),s),hzp:Object.freeze(p)}})();
// Steady core at heat Q to the coolant, flow W, subcooling dh, pressure P (fuel at its steady temperature):
// the shape and the voids solved together. Returns {s, v (voids with v.s), fb}.
const steady=(Q,W,dh,P)=>{
  const Tc=Tsat(P),Tf=fuelT(Math.max(0,Q),Tc),s=[...(Q>0?STGT:RATED.hzp)];let v,l;
  for(let i=0;i<80;i++){v=voids(Q,W,dh,P,s);const sp=[...s];l=iterate(kinf(localFb(v.a,v.hs,Tf,Tc,s)),sp,2);
    let e=0;for(let j=0;j<NZ;j++){e=Math.max(e,Math.abs(sp[j]-s[j]));s[j]=0.5*(s[j]+sp[j])}if(e<1e-7&&i>4)break}
  v=voids(Q,W,dh,P,s);l=lam(kinf(localFb(v.a,v.hs,Tf,Tc,s)),s);v.s=s;return {s,v,fb:1-l}};
const peak=s=>{let m=0,j=0;for(let i=0;i<NZ;i++)if(s[i]>m){m=s[i];j=i}return {pk:m,zpk:zc(j)}};
// Control rods: one lever, x 0 (all in) … 1 (all out), standing for the whole rod pattern. Integral
// worth S-shaped from a sin² differential worth, ρ_rod = WT·(F(x) − F(X0)), F(x) = x − sin(2πx)/(2π).
// Critical at hot zero power at X0 = 60 %; WT chosen so that the rods balance the rated feedback
// (void, Doppler, moderator) at XR = 80 %. Withdrawal/insertion rate RODRATE (0.1 %/s, illustrative);
// scram drives them fully in in TSCRAM (3 s, a placeholder for the CRD hydraulics of step 13).
const F=x=>x-Math.sin(2*Math.PI*x)/(2*Math.PI),X0=0.6,XR=0.8,RODRATE=0.001,TSCRAM=3;
const WT=-RATED.fb/(F(XR)-F(X0));
const rodRho=x=>WT*(F(clamp(x,0,1))-F(X0));
// rod position for reactivity r (bisection; rodRho rises with x)
const rodFor=r=>{let a=0,b=1;for(let i=0;i<50;i++){const m=(a+b)/2;if(rodRho(m)<r)a=m;else b=m}return (a+b)/2};
// Steady core at heat to coolant q (W), coolant temperature Tc, voids v (from voids() without a shape, so
// with v.s, v.a, v.hs), rods x: fission power n with the decay groups at equilibrium (long operation at
// n), precursors at β_i·n, fuel at its steady temperature, and the rod position x that makes it so (net
// reactivity −SRC/n, the source's share; default: solved for). With no heat (q ≤ 0) the core is shut
// down: rods at x (default all in), source level n = SRC/(β − ρ), no decay heat. Tin is not used (the
// node water temperatures come from v.hs); kept for the call signature.
const coreInit=(q,Tc,Tin,v,x)=>{
  const s=[...(v.s||STGT)],a=[...v.a],hs=[...v.hs];
  if(!(q>0)){const xs=x??0,fb=1-lam(kinf(localFb(a,hs,Tc,Tc,s)),s),r=rodRho(xs)+fb,n=r<0?SRC/(BETA-r):SRC/BETA;
    return {n,c:BI.map(b=>b*n),D:DF.map(()=>0),Tf:Tc,x:xs,ac:0,ae:0,xe:0,zb:1,s,a,hs,fb,...peak(s),rho:r,per:Infinity,q:n,pc:false}}
  const n=q/(1+DSUM),c=BI.map(b=>b*n),D=DF.map(f=>f*n),Tf=fuelT(q,Tc);
  const fb=1-lam(kinf(localFb(a,hs,Tf,Tc,s)),s),xs=x!=null?x:rodFor(-fb-SRC/n);
  return {n,c,D,Tf,x:xs,ac:v.ac,ae:v.ae,xe:v.xe,zb:v.zb,s,a,hs,fb,...peak(s),rho:rodRho(xs)+fb,per:Infinity,q,pc:false};
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
      const rho=rodRho(k.x)+k.fb;k.rho=rho;
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
    const W=flow(h,k),v=voids(q,W,inp.dh,inp.P,k.s),e=1-Math.exp(-h/TAUV);
    let ac=0;for(let i=0;i<NZ;i++){k.a[i]+=(v.a[i]-k.a[i])*e;ac+=k.a[i]/NZ}
    k.hs=v.hs;k.ac=ac;k.ae+=(v.ae-k.ae)*e;k.xe=v.xe;k.zb=v.zb;k.W=W;
    // the shape follows the new feedback (quasi-static), and gives the feedback reactivity for the next substep
    k.fb=1-iterate(kinf(localFb(k.a,k.hs,k.Tf,inp.Tc,k.s)),k.s,2);
  }
  Object.assign(k,peak(k.s));
  // reactor period from the change of the fission power over the step
  if(!fix){const r=k.n/n0;k.per=r>0&&Math.abs(Math.log(r))>1e-9?dt/Math.log(r):Infinity}
  return qs/N;
};
export {NZ,M2,DEXT,BSK,STGT,GZ,steady,localFb,peak,VCH,voidVol,QC,PRC,WCR,LAM,BF,BETA,LAMBDA,BI,SRC,AV,AD,AM,TREF,MCF,RF,DL,DF,DSUM,AF,LC,C0,VGJ,TAUV,WMIN,X0,XR,WT,RODRATE,TSCRAM,HSUB,RATED,
  voids,fuelT,rodRho,rodFor,coreInit,coreStep};
