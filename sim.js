// Pressuriser + RCS simulation model: pure physics, no DOM.
// Loaded by index.html as a classic <script> and by the tests as a module; either way it only
// defines globalThis.PZSim. Equations and constants are described in docs/physics.md.
globalThis.PZSim = (() => {
'use strict';
const Tc=647.096,pc=22.064e6,rc=322;
const A=[-7.85951783,1.84408259,-11.7866497,22.6807411,-15.9618719,1.80122502];
const B=[1.99274064,1.09965342,-0.510839303,-1.75493479,-45.5170352,-6.74694450e5];
const C=[-2.03150240,-2.68302940,-5.38626492,-17.2991605,-44.7586581,-63.9201063];
const D=[-5.65134998e-8,2690.66631,127.287297,-135.003439,0.981825814];
const satX=T=>{
  const t=1-T/Tc,th=T/Tc;
  const S=A[0]*t+A[1]*t**1.5+A[2]*t**3+A[3]*t**3.5+A[4]*t**4+A[5]*t**7.5;
  const Sd=A[0]+1.5*A[1]*t**0.5+3*A[2]*t**2+3.5*A[3]*t**2.5+4*A[4]*t**3+7.5*A[5]*t**6.5;
  const lnp=S/th,p=pc*Math.exp(lnp),dpdT=-(p/T)*(lnp+Sd);
  const rf=rc*(1+B[0]*t**(1/3)+B[1]*t**(2/3)+B[2]*t**(5/3)+B[3]*t**(16/3)+B[4]*t**(43/3)+B[5]*t**(110/3));
  const rg=rc*Math.exp(C[0]*t**(1/3)+C[1]*t**(2/3)+C[2]*t**(4/3)+C[3]*t**3+C[4]*t**(37/6)+C[5]*t**(71/6));
  const al=1000*(-1135.905627715+D[0]*th**-19+D[1]*th+D[2]*th**4.5+D[3]*th**5+D[4]*th**54.5);
  return [p,dpdT,rf,rg,al+T/rf*dpdT,al+T/rg*dpdT];
};
// tabulate for speed
const T0=273.16,THI=Tc-0.3,DT=0.02,N=Math.floor((THI-T0)/DT)+2,TB=new Float64Array(N*6);
for(let i=0;i<N;i++)TB.set(satX(Math.min(T0+i*DT,THI)),i*6);
const Q={T:0,p:0,dpdT:0,rf:0,rg:0,hf:0,hg:0,uf:0,ug:0};
const sat=T=>{let f=(T-T0)/DT;if(f<0)f=0;if(f>N-1.0001)f=N-1.0001;const i=f|0,w=f-i,a=i*6,b=a+6;
  Q.T=T;Q.p=TB[a]+w*(TB[b]-TB[a]);Q.dpdT=TB[a+1]+w*(TB[b+1]-TB[a+1]);Q.rf=TB[a+2]+w*(TB[b+2]-TB[a+2]);Q.rg=TB[a+3]+w*(TB[b+3]-TB[a+3]);
  Q.hf=TB[a+4]+w*(TB[b+4]-TB[a+4]);Q.hg=TB[a+5]+w*(TB[b+5]-TB[a+5]);Q.uf=Q.hf-Q.p/Q.rf;Q.ug=Q.hg-Q.p/Q.rg;return Q};
// isothermal compressibility of liquid water, 1/Pa (approximate, near saturation)
const KT=[[0,.509],[25,.452],[50,.441],[75,.456],[100,.49],[150,.58],[200,.74],[250,1.03],[300,1.6],[325,2.2],[340,3],[350,3.8],[360,5.5],[374,12]];
const kap=c=>{if(c<=0)return .509e-9;for(let i=1;i<KT.length;i++)if(c<=KT[i][0]){const[a,x]=KT[i-1],[b,y]=KT[i];return (x+(y-x)*(c-a)/(b-a))*1e-9}return 12e-9};
const Tsat=p=>{let a=T0,b=THI;for(let i=0;i<50;i++){const m=(a+b)/2;if(sat(m).p<p)a=m;else b=m}return (a+b)/2};
const VP=51,VR=280,CM=0.6e9,TAU=300;
// pressuriser state from mass and energy
const pz=(M,U,V=VP)=>{
  const v=V/M,u=U/M,rho=M/V;
  if(sat(T0).uf>u)return {err:'cold',P:0};
  if(sat(THI).uf>u){
    let a=T0,b=THI;for(let i=0;i<46;i++){const m=(a+b)/2;if(sat(m).uf<u)a=m;else b=m}
    const T=(a+b)/2,q=sat(T);
    if(rho>=q.rf){const P=q.p+(1-q.rf/rho)/kap(T-273.15);return {mode:'solid',T,P,x:0,level:V/VP,rf:q.rf,rg:q.rg,hl:u+P/rho,hg:q.hg,dpdT:q.dpdT,Ml:M,Mg:0,Vg:0}}
  }
  const g=T=>{const q=sat(T);return Math.min(1/q.rg-v,v-1/q.rf)};
  if(g(T0)<0)return {err:'dry',P:0};
  let a=T0,b=THI,Tmax=THI;
  if(g(THI)<0){for(let i=0;i<46;i++){const m=(a+b)/2;if(g(m)>=0)a=m;else b=m}Tmax=a}
  const mixU=T=>{const q=sat(T),x=(v-1/q.rf)/(1/q.rg-1/q.rf);return q.uf+x*(q.ug-q.uf)};
  if(mixU(Tmax)<u)return {err:v<1/rc?'solid?':'dry',P:0};
  a=T0;b=Tmax;for(let i=0;i<46;i++){const m=(a+b)/2;if(mixU(m)<u)a=m;else b=m}
  const T=(a+b)/2,q=sat(T),x=(v-1/q.rf)/(1/q.rg-1/q.rf),Ml=M*(1-x);
  return {mode:'two',T,P:q.p,x,level:Ml/q.rf/VP,rf:q.rf,rg:q.rg,hl:q.hf,hg:q.hg,dpdT:q.dpdT,Ml,Mg:M*x,Vg:M*x/q.rg};
};
const liqRho=(T,P)=>{const q=sat(T);return q.rf/(1-kap(T-273.15)*(P-q.p))};
const rcsP=(Mr,Tr)=>{const q=sat(Tr),rho=Mr/VR;return q.p+(1-q.rf/rho)/kap(Tr-273.15)};
const hLiq=(T,P)=>{const q=sat(T);return q.hf+(P-q.p)/q.rf};
const cpL=T=>(sat(T+0.5).hf-sat(T-0.5).hf);
const init=o=>{
  const s={t:0,Tr:o.Tr+273.15,iL:0,iC:0,porv:false,sv:false,rel:0,hold:null,dem:{q:0,sp:0,ch:0,ld:0,rel:0},extra:null,Ms:0,hs:0,rs:1000};
  if(o.solid){const T=o.Tpz+273.15,q=sat(T),rho=liqRho(T,o.P);s.M=rho*VP;s.U=s.M*q.uf}
  else{const T=Tsat(o.P),q=sat(T),Mf=o.L*VP*q.rf,Mg=(1-o.L)*VP*q.rg;s.M=Mf+Mg;s.U=Mf*q.uf+Mg*q.ug}
  s.d=pz(s.M,s.U);s.Mr=liqRho(s.Tr,s.d.P)*VR;s.Pr=s.d.P;s.surge=0;s.f={Q:0,ws:0,ch:0,ld:0,wr:0,trip:false,Qr:0};
  return s;
};
const clamp=(x,a,b)=>Math.min(b,Math.max(a,x));
// relief valve flow, kg/s: choked steam ∝ P (15 kg/s at 16.2 MPa); liquid ∝ √ΔP (30 kg/s at 3.6 MPa)
const relW=(P,steam)=>steam?15*P/16.2e6:30*Math.sqrt(Math.max(0,P-0.1e6)/3.5e6);
const relHys=set=>Math.max(0.2e6,0.02*set);
// bumpless engage: preload the inventory integrators from the current charging/letdown, so a
// change of inventory mode starts from the present net flow
const engage=(s,c)=>{s.iL=s.iC=clamp(c.ch-c.ld,-15,15)};
// Automatic logic runs all the time and produces a demand for every actuator (s.dem). Each actuator
// uses its demand only if c.auto[k] is set; otherwise the operator's value in c. Nothing switches
// between auto and manual, or between inventory modes, by itself.
const demands=(s,c,dt)=>{
  const d=s.d,P=d.P,A=c.auto||{},e=P-c.set;
  // heaters and spray (proportional): 'pres' holds the pressure setpoint; 'sub' holds the coolant
  // subcooling margin c.sub (K), i.e. a pressure target Psat(Tr + c.sub) that follows the coolant
  // temperature. The 'sub' bands are in kelvin of margin so they stay sensible at low pressure.
  let q,sp,pset;
  if(c.pzMode==='sub'){const Q=sat(Math.min(s.Tr+c.sub,THI));pset=Q.p;const eT=(P-pset)/Q.dpdT;
    q=1.8e6*clamp(0.2-eT/1.25,0,1);sp=40*clamp((eT-0.5)/2.5,0,1)}
  else{pset=c.set;q=1.8e6*clamp((c.set-P)/0.15e6+0.2,0,1);sp=40*clamp((e-0.1e6)/0.4e6,0,1)}
  // charging and letdown: a PI controller asks for a net inflow N = charging − letdown.
  // 'pres': letdown is primary and holds pressure (water-solid); 'lev': charging is primary and holds level.
  // The other one, in auto, holds the flow it had when switched to auto (s.hold) and moves off it
  // only while the primary is at a limit.
  const lev=c.inv==='lev',err=lev?c.lset-d.level*100:-e,kp=lev?1.5:5e-5,ki=lev?0.004:2e-7,key=lev?'iC':'iL';
  const H=s.hold||(s.hold={ch:c.ch,ld:c.ld});if(!A.ch)H.ch=c.ch;if(!A.ld)H.ld=c.ld;
  const flows=I=>{const N=kp*err+I;let ch,ld;
    if(lev){ch=clamp(H.ld+N,0,20);ld=clamp((A.ch?ch:c.ch)-N,0,20)}
    else{ld=clamp(H.ch-N,0,20);ch=clamp((A.ld?ld:c.ld)+N,0,20)}
    return {N,ch,ld,got:(A.ch?ch:c.ch)-(A.ld?ld:c.ld)}};
  // integrate unless the demand is already at a limit in the direction the error pushes (anti-windup)
  let I=clamp(s[key]+ki*err*dt,-15,15),r=flows(I);
  if(Math.abs(r.got-r.N)>1e-9&&Math.sign(r.N-r.got)===Math.sign(err)){I=s[key];r=flows(I)}
  s[key]=I;
  // relief valve: on/off with hysteresis at c.relSet
  if(P>c.relSet)s.porv=true;else if(P<c.relSet-relHys(c.relSet))s.porv=false;
  s.dem={q,sp,ch:r.ch,ld:r.ld,rel:s.porv?1:0,pset};
};
const step=(s,c,dt)=>{
  const d=s.d,P=d.P,solid=d.mode==='solid',A=c.auto||{};
  demands(s,c,dt);const D=s.dem;
  let Q=A.q?D.q:c.q*1e3;const ws=A.sp?D.sp:c.sp,ch=A.ch?D.ch:c.ch,ld=A.ld?D.ld:c.ld;
  const trip=!solid&&d.level<0.12;if(trip)Q=0;
  // relief valve (manual opening or auto on/off) plus a fixed safety valve, always armed
  if(P>17.2e6)s.sv=true;else if(P<16.9e6)s.sv=false;
  s.rel=A.rel?D.rel:clamp((c.relMan||0)/100,0,1); // opening fraction 0–1
  const w1=relW(P,!solid),wr=s.rel*w1+(s.sv?4*w1:0);
  const hrel=solid?d.hl:d.hg;
  const hr=hLiq(s.Tr,P),rr=liqRho(s.Tr,P),cap=s.Mr*cpL(s.Tr)+CM;
  const pulse=s.extra&&s.t<s.extra.end?s.extra.Q:0;if(s.extra&&s.t>=s.extra.end)s.extra=null;
  const Qr=c.heat*1e6+pulse;
  s.Tr+=Qr*dt/cap;
  s.Mr+=(ch-ld-ws)*dt;
  s.M+=(ws-wr)*dt;s.U+=(ws*hr+Q-c.loss*1e3-wr*hrel)*dt;
  // insurge layer mixes into the saturated region with time constant TAU
  if(s.Ms>0){
    if(solid){s.U+=s.Ms*(s.hs-P/s.rs);s.M+=s.Ms;s.Ms=0}
    else{const m=Math.min(s.Ms,s.Ms*dt/TAU+0.02*dt);s.U+=m*(s.hs-P/s.rs);s.M+=m;s.Ms-=m}
  }
  // surge: dm>0 is insurge from RCS into pressuriser; equalise pressures
  const hout=d.hl,M=s.M,U=s.U,Mr=s.Mr,Tr=s.Tr,Ms=s.Ms,rs=s.rs;
  const sat_=(dm)=>{
    if(dm>=0){if(solid)return [M+dm,U+dm*hr,VP];return [M,U+P*dm/rr,VP-Ms/rs-dm/rr]}
    const m=-dm,ml=Math.min(m,Ms),mr=m-ml;return [M-mr,U-P*ml/rs-mr*hout,VP-(Ms-ml)/rs];
  };
  const f=dm=>{const[a,b,v]=sat_(dm);return pz(a,b,v).P-rcsP(Mr-dm,Tr)};
  let lo=-400,hi=400,k=0;
  while(f(lo)>0&&k++<12)lo*=3;k=0;
  while(f(hi)<0&&k++<12)hi*=3;
  for(let i=0;i<44;i++){const m=(lo+hi)/2;if(f(m)<0)lo=m;else hi=m}
  const dm=(lo+hi)/2;
  const[a2,b2]=sat_(dm);s.M=a2;s.U=b2;s.Mr-=dm;
  if(dm>0&&!solid){s.hs=(s.Ms*s.hs+dm*hr)/(s.Ms+dm);s.rs=(s.Ms+dm)/(s.Ms/s.rs+dm/rr);s.Ms+=dm}
  if(dm<0){const m=-dm,ml=Math.min(m,Ms),mr=m-ml;s.Ms-=ml;s.Tr+=(ml*(s.hs-hr)+mr*(hout-hr))/(s.Mr*cpL(s.Tr)+CM)}
  s.surge=dm/dt;s.t+=dt;
  s.d=pz(s.M,s.U,VP-s.Ms/s.rs);
  if(!s.d.err&&s.Ms>0){s.d.level=(s.d.Ml/s.d.rf+s.Ms/s.rs)/VP}
  s.Pr=rcsP(s.Mr,s.Tr);
  s.f={Q,ws,ch,ld,wr,trip,Qr};
};
// why the model must stop, or '' while the state is valid:
// 'dry' pressuriser boiled dry, 'range' state outside the model, 'critical' near the critical point,
// 'boiling' coolant at saturation
const fault=s=>{
  const d=s.d;
  if(d.err==='dry'||(d.mode==='two'&&d.level<0.002))return 'dry';
  if(d.err)return 'range';
  if(d.P>21.5e6)return 'critical';
  if(Tsat(s.Pr)-s.Tr<0)return 'boiling';
  return '';
};
return {satX,sat,Tsat,kap,pz,liqRho,rcsP,hLiq,cpL,relW,relHys,engage,demands,init,step,fault,clamp,VP,VR,CM,TAU,T0,THI};
})();
