// Shared by sim.js (plant) and control.js (automatic control): steam tables, actuator limits, clamp.
// An ES module with no imports. Neither plant nor controller depends on the other, only on this file.
// IAPWS 1992 supplementary release (Wagner & Pruss) saturation equations
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
const Tsat=p=>{let a=T0,b=THI;for(let i=0;i<50;i++){const m=(a+b)/2;if(sat(m).p<p)a=m;else b=m}return (a+b)/2};
const clamp=(x,a,b)=>Math.min(b,Math.max(a,x));
// heaters, as in a 4-loop Westinghouse-type pressuriser: 78 elements of about 23 kW, 1.8 MW in all,
// wired as one proportional (control) group of 18 elements, whose power is varied continuously, and
// NB backup groups of 20 elements each, switched fully on or off
const QE=1.8e6/78,QP=18*QE,QB=20*QE,NB=3;
// actuator ranges: charging/letdown/spray kg/s; hp proportional group power fraction; hb backup groups
// on; relief and nitrogen valve opening fractions. The plant clamps every command to these; the
// controller's anti-windup needs the same limits.
const LIM=Object.freeze({ch:20,ld:20,hp:1,hb:NB,sp:40,rel:1,n2:1});
export {satX,sat,Tsat,clamp,LIM,QE,QP,QB,NB,T0,THI,Tc,pc,rc};
