// Pressuriser + RCS plant model: pure physics, no DOM, no automatic control.
// step() takes final actuator commands (from the operator or from control.js; the plant can't tell).
// Loaded after common.js, as a classic <script> (index.html) or a module (tests); it only
// defines globalThis.PZSim. Equations and constants are described in docs/physics.md.
globalThis.PZSim = (() => {
'use strict';
const {sat,Tsat,clamp,LIM,T0,THI,rc}=globalThis.PZCommon;
// isothermal compressibility of liquid water, 1/Pa (approximate, near saturation)
const KT=[[0,.509],[25,.452],[50,.441],[75,.456],[100,.49],[150,.58],[200,.74],[250,1.03],[300,1.6],[325,2.2],[340,3],[350,3.8],[360,5.5],[374,12]];
const kap=c=>{if(c<=0)return .509e-9;for(let i=1;i<KT.length;i++)if(c<=KT[i][0]){const[a,x]=KT[i-1],[b,y]=KT[i];return (x+(y-x)*(c-a)/(b-a))*1e-9}return 12e-9};
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
  const s={t:0,Tr:o.Tr+273.15,sv:false,rel:0,extra:null,Ms:0,hs:0,rs:1000};
  if(o.solid){const T=o.Tpz+273.15,q=sat(T),rho=liqRho(T,o.P);s.M=rho*VP;s.U=s.M*q.uf}
  else{const T=Tsat(o.P),q=sat(T),Mf=o.L*VP*q.rf,Mg=(1-o.L)*VP*q.rg;s.M=Mf+Mg;s.U=Mf*q.uf+Mg*q.ug}
  s.d=pz(s.M,s.U);s.Mr=liqRho(s.Tr,s.d.P)*VR;s.Pr=s.d.P;s.surge=0;s.f={Q:0,ws:0,ch:0,ld:0,wr:0,trip:false,Qr:0};
  return s;
};
// relief valve flow, kg/s: choked steam ∝ P (15 kg/s at 16.2 MPa); liquid ∝ √ΔP (30 kg/s at 3.6 MPa)
const relW=(P,steam)=>steam?15*P/16.2e6:30*Math.sqrt(Math.max(0,P-0.1e6)/3.5e6);
// One step. u is the final command, SI: {ch, ld, sp kg/s; q W heaters; rel 0–1 relief opening;
// heat W net heat to coolant; loss W pressuriser heat loss}. Each actuator is clamped to LIM; the plant's
// own protection (heater trip below 12 % level, spring safety valve) acts on top, whoever commands.
const step=(s,u,dt)=>{
  const d=s.d,P=d.P,solid=d.mode==='solid';
  let Q=clamp(u.q,0,LIM.q);const ws=clamp(u.sp,0,LIM.sp),ch=clamp(u.ch,0,LIM.ch),ld=clamp(u.ld,0,LIM.ld);
  const trip=!solid&&d.level<0.12;if(trip)Q=0;
  // relief valve (as commanded) plus a fixed spring safety valve, always armed
  if(P>17.2e6)s.sv=true;else if(P<16.9e6)s.sv=false;
  s.rel=clamp(u.rel||0,0,LIM.rel); // opening fraction 0–1
  const w1=relW(P,!solid),wr=s.rel*w1+(s.sv?4*w1:0);
  const hrel=solid?d.hl:d.hg;
  const hr=hLiq(s.Tr,P),rr=liqRho(s.Tr,P),cap=s.Mr*cpL(s.Tr)+CM;
  const pulse=s.extra&&s.t<s.extra.end?s.extra.Q:0;if(s.extra&&s.t>=s.extra.end)s.extra=null;
  const Qr=u.heat+pulse;
  s.Tr+=Qr*dt/cap;
  s.Mr+=(ch-ld-ws)*dt;
  s.M+=(ws-wr)*dt;s.U+=(ws*hr+Q-u.loss-wr*hrel)*dt;
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
return {kap,pz,liqRho,rcsP,hLiq,cpL,relW,init,step,fault,VP,VR,CM,TAU};
})();
