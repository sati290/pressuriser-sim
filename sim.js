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
// nitrogen (ideal gas): gas constant, cv, cp (J/kg/K); steam gas constant; supply header
// pressure (Pa), supply flow at 1 MPa across the valve (kg/s), supply temperature (K); below N2MIN of
// the gas (by partial pressure) with the supply shut, the remaining nitrogen is dropped
const RN=296.8,CVN=743,CPN=RN+CVN,RS=461.5,N2P=4e6,N2W=0.5,TN2=293.15,N2MIN=1e-3;
// pressuriser heat loss through the insulation to ambient air at TAMB (fixed): UA·(T − TAMB), W/K;
// 61 kW at hot standby
const UA=200,TAMB=313.15;
// pressuriser state from mass and energy (water M, U includes the nitrogen's cv·T), nitrogen mass Mn
const pz=(M,U,V=VP,Mn=0)=>{
  if(Mn>0)return pzN(M,U,V,Mn);
  const v=V/M,u=U/M,rho=M/V;
  if(sat(T0).uf>u)return {err:'cold',P:0};
  if(sat(THI).uf>u){
    let a=T0,b=THI;for(let i=0;i<46;i++){const m=(a+b)/2;if(sat(m).uf<u)a=m;else b=m}
    const T=(a+b)/2,q=sat(T);
    if(rho>=q.rf){const P=q.p+(1-q.rf/rho)/kap(T-273.15);return {mode:'solid',T,P,x:0,level:V/VP,rf:q.rf,rg:q.rg,hl:u+P/rho,hg:q.hg,dpdT:q.dpdT,Ml:M,Mg:0,Vg:0,Mn:0,pN:0,yN:0}}
  }
  const g=T=>{const q=sat(T);return Math.min(1/q.rg-v,v-1/q.rf)};
  if(g(T0)<0)return {err:'dry',P:0};
  let a=T0,b=THI,Tmax=THI;
  if(g(THI)<0){for(let i=0;i<46;i++){const m=(a+b)/2;if(g(m)>=0)a=m;else b=m}Tmax=a}
  const mixU=T=>{const q=sat(T),x=(v-1/q.rf)/(1/q.rg-1/q.rf);return q.uf+x*(q.ug-q.uf)};
  if(mixU(Tmax)<u)return {err:v<1/rc?'solid?':'dry',P:0};
  a=T0;b=Tmax;for(let i=0;i<46;i++){const m=(a+b)/2;if(mixU(m)<u)a=m;else b=m}
  const T=(a+b)/2,q=sat(T),x=(v-1/q.rf)/(1/q.rg-1/q.rf),Ml=M*(1-x);
  return {mode:'two',T,P:q.p,x,level:Ml/q.rf/VP,rf:q.rf,rg:q.rg,hl:q.hf,hg:q.hg,dpdT:q.dpdT,Ml,Mg:M*x,Vg:M*x/q.rg,Mn:0,pN:0,yN:0};
};
// Liquid + gas space holding nitrogen and water vapour, one temperature T (equilibrium). The vapour is
// saturated at T (partial pressure p_sat(T), density ρ″(T)); nitrogen is an ideal gas filling the same
// volume; P = p_sat(T) + Mn·RN·T/Vg (Dalton). The liquid is compressed liquid at P, u = u_f(T).
// For a given T, P solves P = p_sat + Mn·RN·T/Vg(P) (safeguarded Newton; Vg grows with P as the liquid
// compresses), then T is found by bisection on the energy. There is always a gas space: squeezing it
// raises the pressure without limit, so this branch never goes water-solid.
const pzN=(M,U,V,Mn)=>{
  const at=T=>{
    const q=sat(T),ps=q.p,rf=q.rf,rg=q.rg,uf=q.uf,ug=q.ug,hg=q.hg,k=kap(T-273.15),K=Mn*RN*T,A=M-rg*V;
    if(A<=0)return null; // the water would all be vapour
    // x = P − p_sat; liquid density, gas volume and residual r(x) = K/Vg − x (decreasing in x)
    const rl=x=>rf/(1-k*x),Vg=x=>V-A/(rl(x)-rg),r=x=>{const g=Vg(x);return g>0?K/g-x:Infinity};
    let lo=0,hi,x;const g0=Vg(0);
    if(g0>0)hi=K/g0;else{hi=1e5;while(r(hi)>0&&hi<0.4/k)hi*=2}
    x=hi;
    for(let i=0;i<60;i++){
      const g=Vg(x),rv=g>0?K/g-x:Infinity;
      if(rv>0)lo=x;else hi=x;
      if(hi-lo<=1e-9*(ps+hi)+1e-3)break;
      let xn=NaN;if(g>0){const l=rl(x),dg=A*(l*l*k/rf)/((l-rg)**2);xn=x-rv/(-K/(g*g)*dg-1)}
      x=xn>lo&&xn<hi?xn:(lo+hi)/2;
    }
    const P=ps+x,l=rl(x),Vl=A/(l-rg),Ml=l*Vl,Mg=M-Ml;
    return {T,P,ps,rl:l,rg,Ml,Mg,Vg:V-Vl,hl:uf+P/l,hg,E:Ml*uf+Mg*ug+Mn*CVN*T};
  };
  let Tmax=THI;
  if(sat(THI).rg*V>=M){let a=T0,b=THI;for(let i=0;i<46;i++){const m=(a+b)/2;if(sat(m).rg*V<M)a=m;else b=m}Tmax=a}
  const e0=at(T0);if(!e0||e0.E>U)return {err:'cold',P:0};
  const e1=at(Tmax);if(e1&&e1.E<U)return {err:'dry',P:0};
  // Illinois (regula falsi) on E(T) = U: E is smooth and increasing, so this takes ~6–10 evaluations
  let a=T0,b=Tmax,fa=e0.E-U,fb=e1?e1.E-U:Infinity,side=0,e=null;
  if(!isFinite(fb)){for(let i=0;i<60&&!isFinite(fb);i++){const m=(a+b)/2,em=at(m);if(em&&em.E<U){a=m;fa=em.E-U}else{b=m;fb=em?em.E-U:Infinity}}}
  for(let i=0;i<60;i++){
    let m=isFinite(fb)?(a*fb-b*fa)/(fb-fa):(a+b)/2;if(!(m>a&&m<b))m=(a+b)/2;
    e=at(m);const fm=e?e.E-U:Infinity;
    if(fm<0){a=m;fa=fm;if(side<0)fb/=2;side=-1}else{b=m;fb=fm;if(side>0)fa/=2;side=1}
    if(b-a<1e-10||Math.abs(fm)<1e-11*Math.abs(U))break;
  }
  if(!e||e.Ml<=0)return {err:'dry',P:0};
  const q=sat(e.T),pN=e.P-e.ps;
  return {mode:'two',T:e.T,P:e.P,x:e.Mg/M,level:e.Ml/e.rl/VP,rf:e.rl,rg:e.rg,hl:e.hl,hg:e.hg,dpdT:q.dpdT,
    Ml:e.Ml,Mg:e.Mg,Vg:e.Vg,Mn,pN,pS:e.ps,yN:pN/e.P};
};
const liqRho=(T,P)=>{const q=sat(T);return q.rf/(1-kap(T-273.15)*(P-q.p))};
const rcsP=(Mr,Tr)=>{const q=sat(Tr),rho=Mr/VR;return q.p+(1-q.rf/rho)/kap(Tr-273.15)};
const hLiq=(T,P)=>{const q=sat(T);return q.hf+(P-q.p)/q.rf};
const cpL=T=>(sat(T+0.5).hf-sat(T-0.5).hf);
// ---------- non-equilibrium steam bubble (no nitrogen) ----------
// The saturated region splits into water (Ml, Ul at Tl: compressed liquid, may be sub- or superheated
// against Tsat(P)) and steam (Mg, Ug at Tg: superheated, or wet for one step before the condensate rains
// out). They share the pressure P; the interface is at Tsat(P). Transfers between them relax with time
// constants (s), overridable per state as s.tau: superheated water flashes (fl), subcooled water
// condenses steam at the surface (sc), superheated steam gives its superheat to the water (sh).
// Below VNE (m³) of steam, or with nitrogen, the region is one equilibrium mixture again (pz).
const TAUNE=Object.freeze({fl:5,sc:300,sh:30}),VNE=0.05;
// superheated steam near saturation, at constant density ρ″(Ts) and temperature Ts + θ:
// P − p_sat(Ts) = a1·θ + a2·θ², u − u″(Ts) = b1·θ + b2·θ²   [°C, a1 Pa/K, a2 Pa/K², b1 J/(kg·K), b2 J/(kg·K²)]
// From IAPWS-IF97 region 2 (generated by tools/vapour-table.mjs): within ~1 % up to 10 K superheat,
// ~3 % at 40 K; above ~350 °C (16.5 MPa) region 2 is extrapolated
const VT=[[40,23.96,-0.004242,1456,-0.7933],[50,39.01,-0.007717,1467,-0.8227],[60,61.45,-0.01337,1480,-0.8307],[70,93.94,-0.02246,1494,-0.829],[80,139.9,-0.03713,1511,-0.8363],[90,203.3,-0.0613,1530,-0.8751],[100,289.4,-0.1018,1554,-0.9692],[110,404.3,-0.1699,1582,-1.141],[120,555.4,-0.2836,1615,-1.409],[130,751.9,-0.4682,1655,-1.782],[140,1005,-0.7568,1700,-2.256],[150,1326,-1.189,1751,-2.811],[160,1731,-1.811,1807,-3.421],[170,2239,-2.673,1867,-4.057],[180,2868,-3.834,1931,-4.693],[190,3645,-5.364,1998,-5.315],[200,4598,-7.344,2067,-5.915],[210,5763,-9.868,2140,-6.489],[220,7182,-13.03,2217,-7.032],[230,8903,-16.91,2296,-7.531],[240,10990,-21.59,2379,-7.973],[250,13500,-27.12,2465,-8.348],[260,16530,-33.6,2554,-8.666],[270,20190,-41.3,2647,-8.968],[280,24620,-50.71,2746,-9.33],[290,29990,-62.5,2854,-9.827],[300,36540,-77.2,2972,-10.48],[310,44550,-94.76,3099,-11.23],[320,54440,-114.9,3234,-11.99],[330,66830,-139.1,3378,-12.8],[340,82790,-171,3543,-13.87],[350,103800,-199.6,3723,-14.59],[355,116800,-193,3799,-13.85],[360,131800,-122,3833,-10.81],[365,149100,37.41,3771,-4.121]];
const vtc=T=>{const c=T-273.15;let i=0;while(i<VT.length-2&&c>VT[i+1][0])i++;
  const a=VT[i],b=VT[i+1],w=clamp((c-a[0])/(b[0]-a[0]),0,1);return [a[1]+w*(b[1]-a[1]),a[2]+w*(b[2]-a[2]),a[3]+w*(b[3]-a[3]),a[4]+w*(b[4]-a[4])]};
// inverse tables, T from u_f(T) and from ln ρ″(T) (both increasing), 4000 intervals, linear
const NI=4000,inv=f=>{const y0=f(T0),y1=f(THI),k=NI/(y1-y0),tb=new Float64Array(NI+1);
  for(let i=0;i<=NI;i++){const y=y0+i/k;let a=T0,b=THI;for(let j=0;j<48;j++){const m=(a+b)/2;if(f(m)<y)a=m;else b=m}tb[i]=(a+b)/2}
  return y=>{let g=(y-y0)*k;if(g<0)g=0;if(g>NI-1e-9)g=NI-1e-9;const i=g|0;return tb[i]+(g-i)*(tb[i+1]-tb[i])}};
const Tuf=inv(T=>sat(T).uf),Trg=inv(T=>Math.log(sat(T).rg));
const mixU=(T,v)=>{const q=sat(T),x=(v-1/q.rf)/(1/q.rg-1/q.rf);return q.uf+x*(q.ug-q.uf)};
// steam of density rho, specific internal energy u → {P, T, x}: superheated from VT; below u″ at that
// density, a saturated mixture (x < 1) found by Newton on the mixture energy
const vap=(rho,u)=>{
  const Ts=Trg(Math.log(rho)),q=sat(Ts),du=u-q.ug;
  if(du>=0){const [a1,a2,b1,b2]=vtc(Ts),D=b1*b1+4*b2*du,th=D>0?2*du/(b1+Math.sqrt(D)):-b1/(2*b2);return {P:q.p+a1*th+a2*th*th,T:Ts+th,x:1}}
  const v=1/rho;let T=Ts;
  for(let i=0;i<10;i++){const f1=mixU(T,v)-u,f2=mixU(T-0.01,v)-u,dT=f1*0.01/(f1-f2);T-=dT;if(Math.abs(dT)<1e-7)break}
  const r=sat(T),x=(v-1/r.rf)/(1/r.rg-1/r.rf);return {P:r.p,T,x};
};
// state at trial pressure P. o: water Ml, Ul; steam Mg, Ug; layer volume Vs. Optional: start-of-step
// volumes Vl0, Vg0 (each region then does boundary work −P·ΔV); surge from an RCS of mass Mr at Tr
// (dm = Mr − VR·ρ(Tr, P) into the pressuriser: insurge to the layer, outsurge from the layer first (Ms,
// rs), then water at hl); interface transfers o.x, whose saturation targets h′, h″ are taken at P
// (linearised about the start of the step), which keeps a small steam space stable.
// Residual r = P_steam − P falls as P rises.
const neAt=(o,P)=>{
  let Ml=o.Ml,Ul=o.Ul,Mg=o.Mg,Ugx=o.Ug,Vs=o.Vs,dm=0;
  if(o.x){const x=o.x,dP=P-x.P0,hf=x.hf+x.dhf*dP,hg=x.hg+x.dhg*dP,hfg=hg-hf;
    // spray drops heated to saturation by condensing steam (spray efficiency 1); spray and condensate join the water
    const mc=hf>x.hr?Math.min(x.wsdt*(hf-x.hr)/(x.hv-hf),0.5*Mg):0;Ml+=mc;Ul+=mc*x.hv;Mg-=mc;Ugx-=mc*x.hv;
    // water above saturation flashes into steam; water below it condenses steam at the surface
    const dl=x.hl-hf;
    if(dl>0){const m=Math.min(x.Ml*dl*x.ffl/hfg,0.5*Ml);Ml-=m;Ul-=m*hg;Mg+=m;Ugx+=m*hg}
    else{const m=Math.min(x.Ml*-dl*x.fsc/(x.hv-hf),0.5*Mg);Ml+=m;Ul+=m*x.hv;Mg-=m;Ugx-=m*x.hv}}
  if(o.Mr!==undefined){dm=o.Mr-VR*liqRho(o.Tr,P);
    if(dm>=0)Vs+=dm/o.rr;else{const m=-dm,ml=Math.min(m,o.Ms),mr=m-ml;Vs-=ml/o.rs;Ml-=mr;Ul-=mr*o.hl}}
  if(!(Ml>0))return {r:-Infinity};
  const W=o.Vl0!==undefined;let Vl=W?o.Vl0:Ml/liqRho(Tuf(Ul/Ml),P),Tl=0;
  for(let k=0;k<2;k++){Tl=Tuf((Ul-(W?P*(Vl-o.Vl0):0))/Ml);Vl=Ml/liqRho(Tl,P)}
  const Vg=VP-Vs-Vl;if(!(Vg>0))return {r:Infinity};
  const Ug=Ugx-(W?P*(Vg-o.Vg0):0),g=vap(Mg/Vg,Ug/Mg);
  return {r:g.P-P,P,dm,Vs,Ml,Ul:Ul-(W?P*(Vl-o.Vl0):0),Mg,Vl,Vg,Ug,Tg:g.T,xg:g.x};
};
// bracket around o.P0, then Illinois regula falsi
const neSolve=o=>{
  const f=P=>neAt(o,P).r;
  let dP=Math.max(2e4,0.005*o.P0),lo=o.P0-dP,hi=o.P0+dP,flo=f(lo),fhi=f(hi);
  for(let k=0;k<60&&flo<0&&lo>200;k++){hi=lo;fhi=flo;dP*=2;lo=Math.max(100,lo-dP);flo=f(lo)}
  for(let k=0;k<60&&fhi>0;k++){lo=hi;flo=fhi;dP*=2;hi+=dP;fhi=f(hi)}
  if(!(flo>=0&&fhi<=0))return null;
  let side=0,m=(lo+hi)/2;
  for(let i=0;i<100;i++){
    m=isFinite(flo)&&isFinite(fhi)&&fhi!==flo?(lo*fhi-hi*flo)/(fhi-flo):(lo+hi)/2;if(!(m>lo&&m<hi))m=(lo+hi)/2;
    const fm=f(m);if(Math.abs(fm)<0.02||hi-lo<0.02)break;
    if(fm>0){lo=m;flo=fm;if(side>0)fhi/=2;side=1}else{hi=m;fhi=fm;if(side<0)flo/=2;side=-1}
  }
  const e=neAt(o,m);return isFinite(e.r)?e:null;
};
// derived state of a split pressuriser at pressure P; T is the interface (saturation) temperature
const neD=(s,P)=>{
  const Ml=s.M-s.Mg;if(!(Ml>0)||!(s.Vl>0))return {err:'dry',P:0};
  const ul=(s.U-s.Ug)/Ml,rl=Ml/s.Vl,rg=s.Mg/s.Vg,g=vap(rg,s.Ug/s.Mg),T=Tsat(P);
  return {mode:'two',ne:true,T,P,Tl:Tuf(ul),Tg:g.T,x:s.Mg/s.M,level:(s.Vl+s.Ms/s.rs)/VP,rf:rl,rg,hl:ul+P/rl,hg:s.Ug/s.Mg+P/rg,
    dpdT:sat(T).dpdT,Ml,Mg:s.Mg,Vg:s.Vg,Mn:0,pN:0,pS:P,yN:0};
};
// equilibrium steam bubble → split at the common temperature (no jump)
const split=s=>{const d=s.d;s.ne=true;s.Mg=d.Mg;s.Ug=d.Mg*sat(d.T).ug;s.Vg=d.Vg;s.Vl=VP-s.Ms/s.rs-d.Vg;s.d=neD(s,d.P)};
// split → one equilibrium region (relaxes any non-equilibrium at once)
const merge=s=>{s.ne=false;s.Mg=s.Ug=s.Vl=s.Vg=0;s.d=pz(s.M,s.U,VP-s.Ms/s.rs,s.Mn);if(!s.d.err&&s.Ms>0)s.d.level=(s.d.Ml/s.d.rf+s.Ms/s.rs)/VP};
// one step of the split pressuriser (after the RCS update in step); returns the heat loss
const stepNE=(s,d,dt,Q,ws,wr,hr,rr)=>{
  const P=d.P,q=sat(d.T),hfS=q.hf,hgS=q.hg,hv=d.hg,hl=d.hl,tau=s.tau;
  const a=sat(d.T+0.05),pa=a.p,hfa=a.hf,hga=a.hg,b=sat(d.T-0.05),dp=pa-b.p,dhf=(hfa-b.hf)/dp,dhg=(hga-b.hg)/dp;
  let Mg=s.Mg,Ug=s.Ug,Ml=s.M-s.Mg,Ul=s.U-s.Ug,Ms=s.Ms;
  // heat loss from each region by its share of the volume; heaters in the water
  const fg=s.Vg/VP,lossG=UA*fg*(d.Tg-TAMB),lossL=UA*(1-fg)*(d.Tl-TAMB);
  Ul+=(Q-lossL)*dt;Ug-=lossG*dt;
  // relief takes steam (at most half the steam in one step)
  const mv=Math.min(wr*dt,0.5*Mg);Mg-=mv;Ug-=mv*hv;
  // spray water joins the water region (its condensing effect is in neAt, with the flashing)
  const Ml0=Ml;Ml+=ws*dt;Ul+=ws*dt*hr;
  // superheated steam hands its superheat to the water (walls and surface)
  const dg=hv-hgS;if(dg>0){const E=Mg*dg*(1-Math.exp(-dt/tau.sh));Ug-=E;Ul+=E}
  // insurge layer mixes into the water
  if(Ms>0){const m=Math.min(Ms,Ms*dt/TAU+0.02*dt);Ml+=m;Ul+=m*s.hs;Ms-=m}
  const x={P0:P,hf:hfS,hg:hgS,dhf,dhg,hv,hl,hr,wsdt:ws*dt,Ml:Ml0,ffl:1-Math.exp(-dt/tau.fl),fsc:1-Math.exp(-dt/tau.sc)};
  const e=neSolve({Ml,Ul,Mg,Ug,Vs:Ms/s.rs,Vl0:s.Vl,Vg0:s.Vg,P0:P,Mr:s.Mr,Tr:s.Tr,Ms,rs:s.rs,rr,hl,x});
  if(!e){s.d={err:'range',P:0};s.t+=dt;return lossG+lossL}
  const dm=e.dm;s.Mr-=dm;
  if(dm>0){s.hs=(Ms*s.hs+dm*hr)/(Ms+dm);s.rs=(Ms+dm)/(Ms/s.rs+dm/rr);Ms+=dm}
  else if(dm<0){const m=-dm,ml=Math.min(m,Ms),mr=m-ml;Ms-=ml;s.Tr+=(ml*(s.hs-hr)+mr*(hl-hr))/(s.Mr*cpL(s.Tr)+CM)}
  Ml=e.Ml;Ul=e.Ul;Mg=e.Mg;Ug=e.Ug;let Vl=e.Vl,Vg=e.Vg;
  // wet steam (after expansion or cooling): the condensate rains out into the water
  if(e.xg<1){const r=sat(e.Tg),m=Mg*(1-e.xg),uf=r.uf,vf=1/r.rf;Ml+=m;Ul+=m*uf;Mg-=m;Ug-=m*uf;Vl+=m*vf;Vg-=m*vf}
  s.Ms=Ms;s.M=Ml+Mg;s.U=Ul+Ug;s.Mg=Mg;s.Ug=Ug;s.Vl=Vl;s.Vg=Vg;s.surge=dm/dt;s.t+=dt;
  s.d=Vg<VNE?(merge(s),s.d):neD(s,e.P);
  return lossG+lossL;
};
const init=o=>{
  const s={t:0,Tr:o.Tr+273.15,sv:false,rel:0,extra:null,Ms:0,hs:0,rs:1000,Mn:0,ne:false,Mg:0,Ug:0,Vl:0,Vg:0,
    neq:o.neq!==false,tau:{...TAUNE,...o.tau}};
  if(o.n2){ // nitrogen cushion: liquid and gas at Tpz, nitrogen makes up P − p_sat(Tpz)
    const T=o.Tpz+273.15,q=sat(T),ps=q.p,uf=q.uf,ug=q.ug,rg=q.rg,Vl=o.L*VP,Vg=VP-Vl,Ml=liqRho(T,o.P)*Vl,Mg=rg*Vg;
    s.Mn=Math.max(0,o.P-ps)*Vg/(RN*T);s.M=Ml+Mg;s.U=Ml*uf+Mg*ug+s.Mn*CVN*T}
  else if(o.solid){const T=o.Tpz+273.15,q=sat(T),rho=liqRho(T,o.P);s.M=rho*VP;s.U=s.M*q.uf}
  else{const T=Tsat(o.P),q=sat(T),Mf=o.L*VP*q.rf,Mg=(1-o.L)*VP*q.rg;s.M=Mf+Mg;s.U=Mf*q.uf+Mg*q.ug}
  s.d=pz(s.M,s.U,VP,s.Mn);s.Mr=liqRho(s.Tr,s.d.P)*VR;s.Pr=s.d.P;s.surge=0;s.f={Q:0,ws:0,ch:0,ld:0,wr:0,wrn:0,wn:0,n2:0,trip:false,Qr:0,loss:0};
  if(s.neq&&s.d.mode==='two'&&!(s.Mn>0)&&s.d.Vg>2*VNE)split(s);
  return s;
};
// Relief valve flow, kg/s, into the relief tank at atmospheric pressure PB (all pressures are absolute).
// Steam: choked ∝ P (15 kg/s at 16.2 MPa) while PB/P is below the critical ratio, then the ideal-gas
// subsonic nozzle factor (κ = 1.3), which falls to zero at PB. Liquid ∝ √(P − PB) (30 kg/s at 3.6 MPa).
const PB=0.1e6,KAP=1.3,RCR=(2/(KAP+1))**(KAP/(KAP-1));
const psi=r=>Math.sqrt(Math.max(0,r**(2/KAP)-r**((KAP+1)/KAP))),PSI=psi(RCR);
const subF=P=>{if(P<=PB)return 0;const r=PB/P;return r<=RCR?1:psi(r)/PSI};
const relW=(P,steam)=>steam?15*P/16.2e6*subF(P):30*Math.sqrt(Math.max(0,P-PB)/3.5e6);
// gas with nitrogen: the steam figure times the ideal-gas choked-flow ratio (∝ P/√(R·T)) of the mixture
// against saturated steam at the same pressure; exactly 1 for steam alone
const gasF=d=>{if(!(d.Mn>0))return 1;const Rm=(d.Mn*RN+d.Mg*RS)/(d.Mn+d.Mg);return Math.sqrt(RS*Tsat(d.P)/(Rm*d.T))};
// nitrogen supply, kg/s, valve fully open: from the N2P header through a check valve, ∝ √ΔP (N2W at 1 MPa);
// the flow scales with the valve opening
const n2W=P=>N2W*Math.sqrt(Math.max(0,N2P-P)/1e6);
// One step. u is the final command, SI: {ch, ld, sp kg/s; q W heaters; rel 0–1 relief opening;
// n2 0–1 nitrogen supply valve opening; heat W net heat to coolant}. Each actuator is clamped to LIM; the plant's
// own protection (heater trip below 12 % level, spring safety valve) acts on top, whoever commands.
const step=(s,u,dt)=>{
  const n2=clamp(u.n2||0,0,LIM.n2);
  if(s.ne&&n2*n2W(s.d.P)>0)merge(s); // nitrogen coming in: the gas space is one equilibrium region again
  const d=s.d,P=d.P,solid=d.mode==='solid';
  let Q=clamp(u.q,0,LIM.q);const ws=clamp(u.sp,0,LIM.sp),ch=clamp(u.ch,0,LIM.ch),ld=clamp(u.ld,0,LIM.ld);
  const trip=!solid&&d.level<0.12;if(trip)Q=0;
  // relief valve (as commanded) plus a fixed spring safety valve, always armed
  if(P>17.2e6)s.sv=true;else if(P<16.9e6)s.sv=false;
  s.rel=clamp(u.rel||0,0,LIM.rel); // opening fraction 0–1
  const w1=relW(P,!solid)*(solid?1:gasF(d)),wr=s.rel*w1+(s.sv?4*w1:0);
  const hrel=solid?d.hl:d.hg; // steam leaves at h″(T) whether or not nitrogen is present
  // vented gas carries nitrogen in proportion to its share of the gas mass (exponential, so it can't overdraw)
  const wrn=!solid&&s.Mn>0?s.Mn*(1-Math.exp(-wr*dt/(s.Mn+Math.max(0,d.Mg))))/dt:0,wrs=wr-wrn;
  const wn=n2*n2W(P);
  const hr=hLiq(s.Tr,P),rr=liqRho(s.Tr,P),cap=s.Mr*cpL(s.Tr)+CM;
  const pulse=s.extra&&s.t<s.extra.end?s.extra.Q:0;if(s.extra&&s.t>=s.extra.end)s.extra=null;
  const Qr=u.heat+pulse;
  s.Tr+=Qr*dt/cap;
  s.Mr+=(ch-ld-ws)*dt;
  if(s.ne){const loss=stepNE(s,d,dt,Q,ws,wr,hr,rr);s.Pr=rcsP(s.Mr,s.Tr);s.f={Q,ws,ch,ld,wr,wrn:0,wn:0,n2,trip,Qr,loss};return}
  const loss=UA*(d.T-TAMB);
  s.M+=(ws-wrs)*dt;s.U+=(ws*hr+Q-loss-wrs*hrel+(wn*TN2-wrn*d.T)*CPN)*dt;s.Mn+=(wn-wrn)*dt;
  // insurge layer mixes into the saturated region with time constant TAU
  if(s.Ms>0){
    if(solid){s.U+=s.Ms*(s.hs-P/s.rs);s.M+=s.Ms;s.Ms=0}
    else{const m=Math.min(s.Ms,s.Ms*dt/TAU+0.02*dt);s.U+=m*(s.hs-P/s.rs);s.M+=m;s.Ms-=m}
  }
  // surge: dm>0 is insurge from RCS into pressuriser; equalise pressures
  const hout=d.hl,M=s.M,U=s.U,Mr=s.Mr,Tr=s.Tr,Ms=s.Ms,rs=s.rs,Mn=s.Mn;
  const sat_=(dm)=>{
    if(dm>=0){if(solid)return [M+dm,U+dm*hr,VP];return [M,U+P*dm/rr,VP-Ms/rs-dm/rr]}
    const m=-dm,ml=Math.min(m,Ms),mr=m-ml;return [M-mr,U-P*ml/rs-mr*hout,VP-(Ms-ml)/rs];
  };
  const f=dm=>{const[a,b,v]=sat_(dm);return pz(a,b,v,Mn).P-rcsP(Mr-dm,Tr)};
  let lo=-400,hi=400,k=0;
  while(f(lo)>0&&k++<12)lo*=3;k=0;
  while(f(hi)<0&&k++<12)hi*=3;
  for(let i=0;i<44;i++){const m=(lo+hi)/2;if(f(m)<0)lo=m;else hi=m}
  const dm=(lo+hi)/2;
  const[a2,b2]=sat_(dm);s.M=a2;s.U=b2;s.Mr-=dm;
  if(dm>0&&!solid){s.hs=(s.Ms*s.hs+dm*hr)/(s.Ms+dm);s.rs=(s.Ms+dm)/(s.Ms/s.rs+dm/rr);s.Ms+=dm}
  if(dm<0){const m=-dm,ml=Math.min(m,Ms),mr=m-ml;s.Ms-=ml;s.Tr+=(ml*(s.hs-hr)+mr*(hout-hr))/(s.Mr*cpL(s.Tr)+CM)}
  s.surge=dm/dt;s.t+=dt;
  s.d=pz(s.M,s.U,VP-s.Ms/s.rs,s.Mn);
  // the gas becomes steam alone once the nitrogen is a negligible share and none is coming in
  if(s.Mn>0&&!wn&&!s.d.err&&s.d.yN<N2MIN){s.U-=s.Mn*CVN*s.d.T;s.Mn=0;s.d=pz(s.M,s.U,VP-s.Ms/s.rs)}
  if(!s.d.err&&s.Ms>0){s.d.level=(s.d.Ml/s.d.rf+s.Ms/s.rs)/VP}
  if(s.neq&&!s.d.err&&s.d.mode==='two'&&!(s.Mn>0)&&s.d.Vg>2*VNE)split(s);
  s.Pr=rcsP(s.Mr,s.Tr);
  s.f={Q,ws,ch,ld,wr,wrn,wn,n2,trip,Qr,loss};
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
return {kap,pz,vap,neSolve,TAUNE,VNE,liqRho,rcsP,hLiq,cpL,relW,subF,gasF,n2W,init,step,fault,VP,VR,CM,TAU,RN,CVN,N2P,N2MIN,UA,TAMB,PB};
})();
