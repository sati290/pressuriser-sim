// Generates the superheated-steam coefficient table VT in sim.js (run: node tools/vapour-table.mjs).
// Vapour near saturation, at constant density ρ = ρ″(Ts), temperature Ts + θ:
//   P − p_sat(Ts) ≈ a1·θ + a2·θ²,   u − u″(Ts) ≈ b1·θ + b2·θ²
// from IAPWS-IF97 region 2 (basic equation; checked below against the release's own verification
// values). a1, b1 are slopes at θ → 0; a2, b2 fit the curvature at θ = 20 K (10 K above 355 °C).
// Above 16.5 MPa (≈ 350 °C) saturated steam is in IF97 region 3 and region 2 is extrapolated.
import {satX} from '../common.js';
const J0=[0,1,-5,-4,-3,-2,-1,2,3],n0=[-0.96927686500217e1,0.10086655968018e2,-0.56087911283020e-2,0.71452738081455e-1,-0.40710498223928,0.14240819171444e1,-0.43839511319450e1,-0.28408632460772,0.21268463753307e-1];
const IJn=[[1,0,-0.17731742473213e-2],[1,1,-0.17834862292358e-1],[1,2,-0.45996013696365e-1],[1,3,-0.57581259083432e-1],[1,6,-0.50325278727930e-1],[2,1,-0.33032641670203e-4],[2,2,-0.18948987516315e-3],[2,4,-0.39392777243355e-2],[2,7,-0.43797295650573e-1],[2,36,-0.26674547914087e-4],[3,0,0.20481737692309e-7],[3,1,0.43870667284435e-6],[3,3,-0.32277677238570e-4],[3,6,-0.15033924542148e-2],[3,35,-0.40668253562649e-1],[4,1,-0.78847309559367e-9],[4,2,0.12790717852285e-7],[4,3,0.48225372718507e-6],[5,7,0.22922076337661e-5],[6,3,-0.16714766451061e-10],[6,16,-0.21171472321355e-2],[6,35,-0.23895741934104e2],[7,0,-0.59059564324270e-17],[7,11,-0.12621808899101e-5],[7,25,-0.38946842435739e-1],[8,8,0.11256211360459e-10],[8,36,-0.82311340897998e1],[9,13,0.19809712802088e-7],[10,4,0.10406965210174e-18],[10,10,-0.10234747095929e-12],[10,14,-0.10018179379511e-8],[16,29,-0.80882908646985e-10],[16,50,0.10693031879409],[18,57,-0.33662250574171],[20,20,0.89185845355421e-24],[20,35,0.30629316876232e-12],[20,48,-0.42002467698208e-5],[21,21,-0.59056029685639e-25],[22,53,0.37826947613457e-5],[23,39,-0.12768608934681e-14],[24,26,0.73087610595061e-28],[24,40,0.55414715350778e-16],[24,58,-0.94369707241210e-6]];
const R=461.526;
const r2=(p,T)=>{const pi=p/1e6,tau=540/T,t5=tau-0.5;let g0t=0,gp=0,gt=0;
  for(let i=0;i<9;i++)g0t+=n0[i]*J0[i]*tau**(J0[i]-1);
  for(const[I,J,n]of IJn){gp+=n*I*pi**(I-1)*t5**J;gt+=n*pi**I*J*t5**(J-1)}
  const gpi=1/pi+gp;return {v:R*T/p*pi*gpi,h:R*T*tau*(g0t+gt),u:R*T*(tau*(g0t+gt)-pi*gpi)}};
// IF97 verification values (Table 15): T, p, v (m³/kg), h (kJ/kg)
for(const[T,p,v,h]of[[300,3500,39.4913866,2549.91145],[700,3500,92.3015898,3335.68375],[700,30e6,0.00542946619,2631.49474]]){
  const r=r2(p,T);if(Math.abs(r.v/v-1)>1e-8||Math.abs(r.h/1e3/h-1)>1e-8)throw new Error(`IF97 check failed at ${T} K, ${p} Pa`)}
const pAt=(rho,T,p0)=>{let p=p0;for(let i=0;i<80;i++){const a=1/r2(p,T).v,b=1/r2(p*1.000001,T).v;p+=(rho-a)/((b-a)/(p*1e-6))}return p};
const rows=[];
for(let C=40;C<=365;C+=C<350?10:5){
  const Ts=C+273.15,[ps,,,rg]=satX(Ts),z=pAt(rg,Ts,ps),u0=r2(z,Ts).u,th=C>355?10:20;
  const at=t=>{const p=pAt(rg,Ts+t,ps);return [p-z,r2(p,Ts+t).u-u0]};
  const [p1,u1]=at(0.05),[p2,u2]=at(th),a1=p1/0.05,b1=u1/0.05;
  rows.push([C,+a1.toPrecision(4),+((p2-a1*th)/th**2).toPrecision(4),+b1.toPrecision(4),+((u2-b1*th)/th**2).toPrecision(4)]);
}
console.log('const VT='+JSON.stringify(rows)+';');
