// BWR main steam: rated conditions, the main steam valve, the safety/relief valves (SRVs) and the
// steam nozzle flow factor. Pure functions, no DOM. An ES module; imports common.js. See docs/bwr.md.
import {sat,Tsat,hLiq} from './common.js';
// rated conditions (generic BWR/4 class): core thermal power QR, dome pressure PR, feedwater 216 °C;
// rated steam flow WR from the heat balance
const QR=3293e6,PR=7.0e6,TFR=216+273.15;
const WR=QR/(sat(Tsat(PR)).hg-hLiq(TFR,PR));
// main steam line: the valve stands for the MSIVs plus the turbine control and bypass valves together.
// Full open passes WMS kg/s at PR; flow ∝ P while choked, subsonic factor near the condenser pressure.
// The valve travels at most RATE of its stroke per second (full stroke in 5 s).
const WMS=2000,RATE=0.2;
// safety/relief valves (SRVs), discharging to the suppression pool at PB. Each valve has a spring
// that opens it fully above its set pressure p (Pa abs) and recloses it BD below, and can also be
// opened by hand (relief mode, a command from the operator). Capacity w (kg/s) is rated at its set
// pressure; the flow scales ∝ P (choked) with the subsonic factor near PB. Values from a GE BWR/4
// training lesson plan: 4 valves at 1090 psig rated 800,000 lb/h, 4 at 1100 psig 808,000 lb/h,
// 3 at 1110 psig 815,000 lb/h (the same valve model; capacity ∝ set pressure). Blowdown BD is
// illustrative (about 3 %); the source gives none.
const psig=x=>(x+14.696)*6894.757,lbh=x=>x*1.25998e-4;
const SRV=Object.freeze([[1090,800e3],[1090,800e3],[1090,800e3],[1090,800e3],[1100,808e3],[1100,808e3],[1100,808e3],[1100,808e3],[1110,815e3],[1110,815e3],[1110,815e3]]
  .map(([p,w])=>Object.freeze({p:psig(p),w:lbh(w)})));
const NV=SRV.length,PB=0.1e6,BD=0.25e6;
// steam nozzle flow factor against a back pressure pb: 1 while choked, ideal-gas subsonic nozzle
// factor (κ = 1.3) above the critical pressure ratio, 0 at pb
const KAP=1.3,RCR=(2/(KAP+1))**(KAP/(KAP-1));
const psi=r=>Math.sqrt(Math.max(0,r**(2/KAP)-r**((KAP+1)/KAP))),PSI=psi(RCR);
const subF=(P,pb)=>{if(P<=pb)return 0;const r=pb/P;return r<=RCR?1:psi(r)/PSI};
// main steam valve flow at opening a, dome pressure P, condenser pressure pc
const wMS=(a,P,pc)=>a*WMS*P/PR*subF(P,pc);
const wSRV=(i,P)=>SRV[i].w*P/SRV[i].p*subF(P,PB);
// SRV springs and hand openings at pressure P: updates the spring latches spr[], returns {open, wsrv, nsrv}
const srvStep=(spr,man,P)=>{
  const open=SRV.map((v,i)=>{if(P>v.p)spr[i]=true;else if(P<v.p-BD)spr[i]=false;return spr[i]||!!man[i]});
  let wsrv=0,nsrv=0;open.forEach((o,i)=>{if(o){wsrv+=wSRV(i,P);nsrv++}});
  return {open,wsrv,nsrv};
};
export {QR,PR,TFR,WR,WMS,RATE,SRV,NV,PB,BD,psig,lbh,subF,wMS,wSRV,srvStep};
