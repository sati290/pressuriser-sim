// BWR core: point kinetics with reactivity feedback through a quasi-static axial power shape, control rod
// groups that enter from the bottom, one lumped fuel node, decay heat, and the axial void distribution from the core thermal-hydraulics. Pure functions, no DOM. An ES module; imports
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
// saturation in the subcooled nodes, times (1 − R(z)) for the control rods (below). G(z) is fixed: it
// stands for the fuel loading and burnable poison (axial zoning), chosen so that at rated with all rods out
// the shape is STGT (bottom-skewed, peaking about 1.45 near 35 % of the height, a generic beginning-of-cycle
// BWR shape) and k = 1. The reactivity is ρ = 1 − 1/k, from the Rayleigh quotient of the current shape
// (first order in the shape error); the shape follows the feedback and the rods by two shifted inverse
// (Wielandt) iterations per substep. Without the shape a colder feed lost voids low in the core but the
// extra power went in all along it, so the power rose about twice as much as in a real core (docs/bwr.md,
// "Axial shape"). The zoning that keeps the rated shape off the bottom makes the shape sit high with no
// voids, and rods in from the bottom push it higher still (as in a real BWR at startup). M² 65 cm² (hot,
// voided BWR lattice, round textbook value), DEXT 0.1 m; illustrative.
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
// Control rods (roadmap step 13). NG rod groups, bottom entry (the drives sit under the vessel). Every rod of
// a group moves together (a gang, as in an ABWR's gang mode; a BWR/4 moves its rods one at a time through a
// withdrawal sequence, and a group here stands for a step of that sequence). Positions as on a BWR's rod
// display: 00 fully in … 48 fully out, a notch is 2 positions (6 in, 15.2 cm), the 48 positions span the
// active fuel. x = position/48, the withdrawn fraction: the group covers the fuel from its bottom up to
// (1 − x)·LC. A node a fraction c covered by group g has its k∞ multiplied by (1 − GW[g]·c) (GW the group's
// share of the rods times what a fully rodded lattice loses). Groups, in withdrawal order: 1 and 2 the deep
// startup groups (taken fully out first, the reactor still subcritical; most of the shutdown margin), 3 the
// small startup group the reactor goes critical on (about 0.2 $ a notch there), 4 the power group (pulled
// during the power ascension; a few notches in at rated are shallow rods that trim the bottom peak).
// Rated pattern: all rods out. Hot zero power: critical with group 3 near position 20, group 4 in; all
// rods in about −12 % (k ≈ 0.89). Illustrative worths, not one plant's.
const NG=4,NPOS=48,NOTCH=2,GW=Object.freeze([0.045,0.045,0.016,0.058]);
const XRATED=Object.freeze(Array(NG).fill(1));
// drive speed: a notch per 4 s (RODRATE in x per second; a BWR/4 drive moves a notch in about 2 s, but one rod
// at a time). Scram: after TSD (the scram pilot valves venting) the accumulators drive every rod in at VSC
// per second, all in by about 2.9 s from fully out (BWR/4 technical specifications: 90 % inserted within
// about 3 s at full pressure).
const RODRATE=NOTCH/NPOS/4,TSD=0.2,VSC=1/2.7,TSCRAM=TSD+1/VSC;
// CRD hydraulics, minimal: the CRD pump (from the storage tank) supplies the drive water the rods need to
// move, charges the scram accumulators (taken as always charged) and cools the drives; that water, WCRD
// (60 gpm, a typical BWR/4 figure), ends up in the vessel. Stopped, the rods can't be moved; the scram
// still works.
const WCRD=3.8;
// fraction of node i covered by rods at x
const cov=(x,i)=>clamp((1-clamp(x,0,1))*NZ-i,0,1);
// rod absorption by node: k∞ multiplier is 1 − R
const rodR=x=>{const R=new Array(NZ).fill(0);for(let g=0;g<NG;g++)for(let i=0;i<NZ;i++)R[i]+=GW[g]*cov(x[g],i);return R};
const kinf=(f,R)=>f.map((v,i)=>GZ[i]*(1+v)*(1-R[i]));
// the withdrawal sequence: t 0 (all in) … 1 (all out), the groups one after the other
const seq=t=>[...Array(NG)].map((_,g)=>clamp(NG*t-g,0,1));
// steady fuel temperature at heat q and coolant temperature Tc
const fuelT=(q,Tc)=>Tc+q*RF;
// the generic rated shape: chopped sine with zeros DEXT beyond the fuel, skewed to the bottom by BSK
const BSK=0.35,STGT=(()=>{const r=[...Array(NZ)].map((_,i)=>Math.sin(Math.PI*(zc(i)*LC+DEXT)/(LC+2*DEXT))*(1+BSK*(1-2*zc(i))));
  const m=r.reduce((a,b)=>a+b,0)/NZ;return Object.freeze(r.map(v=>v/m))})();
// rated design (feed at 216 °C mixed into the core flow, all rods out): G(z) from the rated shape, so the
// core is exactly critical there
let GZ;
const RATED=(()=>{const q=sat(TREF),hf=q.hf,hfg=q.hg-q.hf,hfw=sat(216+273.15).hf,W=QC/(hfg+hf-hfw),dh=W*(hf-hfw)/WCR,s=[...STGT],v=voids(QC,WCR,dh,PRC,s);
  const f=localFb(v.a,v.hs,fuelT(QC,TREF),TREF,s),As=Amul(s);GZ=Object.freeze(s.map((si,i)=>As[i]/(si*(1+f[i]))));
  return {dh,v,s:STGT,x:XRATED}})();
// shape and reactivity with fixed feedback f and rods x, from shape s (converged in place); returns ρ
const shapeAt=(f,x,s,it=200)=>1-iterate(kinf(f,rodR(x)),s,it);
// rods along the sequence that give reactivity r with feedback f, shape s held (bisection; ρ rises with t)
const rodsFor=(f,s,r)=>{let a=0,b=1;for(let i=0;i<40;i++){const m=(a+b)/2;if(1-lam(kinf(f,rodR(seq(m))),s)<r)a=m;else b=m}return seq((a+b)/2)};
// Steady core at heat Q to the coolant, flow W, subcooling dh, pressure P (fuel at its steady temperature),
// rods x (array of NG, 0–1; default: along the sequence, critical): the shape, the voids and (by default)
// the rods solved together. Returns {s, v (voids with v.s), x, rho}.
const steady=(Q,W,dh,P,x)=>{
  const Tc=Tsat(P),Tf=fuelT(Math.max(0,Q),Tc),s=[...STGT];let v,xs=x?[...x]:[...XRATED];
  for(let i=0;i<80;i++){v=voids(Q,W,dh,P,s);const f=localFb(v.a,v.hs,Tf,Tc,s);if(!x)xs=rodsFor(f,s,0);const sp=[...s];iterate(kinf(f,rodR(xs)),sp,2);
    let e=0;for(let j=0;j<NZ;j++){e=Math.max(e,Math.abs(sp[j]-s[j]));s[j]=0.5*(s[j]+sp[j])}if(e<1e-7&&i>4)break}
  v=voids(Q,W,dh,P,s);const f=localFb(v.a,v.hs,Tf,Tc,s);if(!x)xs=rodsFor(f,s,0);v.s=s;return {s,v,x:xs,rho:1-lam(kinf(f,rodR(xs)),s)}};
const peak=s=>{let m=0,j=0;for(let i=0;i<NZ;i++)if(s[i]>m){m=s[i];j=i}return {pk:m,zpk:zc(j)}};
// Steady core at heat to coolant q (W), coolant temperature Tc, voids v (from voids() without a shape, so
// with v.s, v.a, v.hs), rods x (array of NG, 0–1): fission power n with the decay groups at equilibrium (long
// operation at n), precursors at β_i·n, fuel at its steady temperature, and the rods along the sequence that
// make it so (net reactivity −SRC/n, the source's share), or rods x given (the shape then solved for them).
// With no heat (q ≤ 0) the core is shut down: rods at x (default all in), source level n = SRC/(β − ρ), no
// decay heat. Tin is not used (the node water temperatures come from v.hs); kept for the call signature.
const coreInit=(q,Tc,Tin,v,x)=>{
  const s=[...(v.s||STGT)],a=[...v.a],hs=[...v.hs],base={per:Infinity,pc:false,st:null};
  if(!(q>0)){const xs=x?[...x]:Array(NG).fill(0),f=localFb(a,hs,Tc,Tc,s),r=shapeAt(f,xs,s),n=r<0?SRC/(BETA-r):SRC/BETA;
    return {...base,n,c:BI.map(b=>b*n),D:DF.map(()=>0),Tf:Tc,x:xs,ac:0,ae:0,xe:0,zb:1,s,a,hs,f,...peak(s),rho:r,q:n}}
  const n=q/(1+DSUM),c=BI.map(b=>b*n),D=DF.map(f=>f*n),Tf=fuelT(q,Tc),f=localFb(a,hs,Tf,Tc,s);
  let xs;if(x){xs=[...x];shapeAt(f,xs,s)}else xs=rodsFor(f,s,-SRC/n);
  return {...base,n,c,D,Tf,x:xs,ac:v.ac,ae:v.ae,xe:v.xe,zb:v.zb,s,a,hs,f,...peak(s),rho:1-lam(kinf(f,rodR(xs)),s),q};
};
// Advance the core by dt in substeps of at most HSUB. inp: {P dome pressure, Tc coolant (saturation)
// temperature, dh core inlet subcooling J/kg, Tin inlet temperature, rods array of NG 0–1 group position
// demands (undefined: stay), crd true while the CRD pump supplies drive water (otherwise the rods cannot be
// moved; the scram still works on the accumulators), scram true while scrammed, heat W fixed heat to coolant (test hook:
// no kinetics, the fuel node and kinetics stand still)}. flow(h, k) advances the core flow by h given the core
// state k (voids) and returns the core flow W (kg/s). Returns the mean heat to the coolant over dt (W).
// Kinetics: prompt-jump approximation with the precursors c_i in power units (c_i = Λ·λ_i·C_i):
// n = (Σc_i + SRC)/(β − ρ), dc_i/dt = λ_i·(β_i·n − c_i); implicit Euler for c_i, solved together with n.
// ρ = 1 − 1/k from the shape of the last substep with the rods where they are now (the shape then follows).
// ρ ≥ β (prompt critical; the approximation fails there) latches k.pc, which the plant reports as a fault.
const HSUB=0.05;
const coreStep=(k,inp,dt,flow)=>{
  const N=Math.max(1,Math.ceil(dt/HSUB-1e-9)),h=dt/N,fix=typeof inp.heat==='number',n0=k.n;let qs=0;
  for(let j=0;j<N;j++){
    // rods: the scram drives them in after the pilot valve delay; otherwise they follow the demand at the
    // drive speed while the CRD pump supplies drive water
    if(inp.scram){const t0=k.st??0;k.st=t0+h;const e=Math.max(0,k.st-Math.max(t0,TSD));if(e>0)for(let g=0;g<NG;g++)k.x[g]=Math.max(0,k.x[g]-VSC*e)}
    else{k.st=null;if(inp.rods&&inp.crd)for(let g=0;g<NG;g++)if(inp.rods[g]!=null)k.x[g]+=clamp(clamp(inp.rods[g],0,1)-k.x[g],-RODRATE*h,RODRATE*h)}
    const R=rodR(k.x);
    let q;
    if(fix){q=inp.heat;k.q=q;k.Tf=fuelT(Math.max(0,q),inp.Tc);k.rho=1-lam(kinf(k.f,R),k.s)}
    else{
      const rho=1-lam(kinf(k.f,R),k.s);k.rho=rho;
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
    // the shape follows the new feedback and the rods (quasi-static)
    k.f=localFb(k.a,k.hs,k.Tf,inp.Tc,k.s);iterate(kinf(k.f,R),k.s,2);
  }
  Object.assign(k,peak(k.s));
  // reactor period from the change of the fission power over the step
  if(!fix){const r=k.n/n0;k.per=r>0&&Math.abs(Math.log(r))>1e-9?dt/Math.log(r):Infinity}
  return qs/N;
};
export {NZ,M2,DEXT,BSK,STGT,GZ,steady,localFb,peak,VCH,voidVol,QC,PRC,WCR,LAM,BF,BETA,LAMBDA,BI,SRC,AV,AD,AM,TREF,MCF,RF,DL,DF,DSUM,AF,LC,C0,VGJ,TAUV,WMIN,
  NG,NPOS,NOTCH,GW,XRATED,RODRATE,TSD,VSC,TSCRAM,WCRD,HSUB,RATED,cov,rodR,kinf,seq,rodsFor,shapeAt,lam,iterate,
  voids,fuelT,coreInit,coreStep};
