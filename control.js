// Automatic control for the pressuriser simulator: no DOM, no plant equations.
// It reads the plant state (s.d, s.Tr) and the operator's settings, and produces the final command
// that sim.js step() takes. An ES module; imports only common.js (not sim.js). See docs/physics.md.
//
// Settings c are in panel units: ch, ld, sp kg/s; hp proportional heaters %;
// hb backup heater groups on; relMan %; heat MW; set, relSet Pa;
// lset %; dT K; inv 'pres'|'lev'; pzMode 'pres'|'dT'; auto {ch,ld,hp,hb,sp,rel} booleans; n2 %
// (nitrogen supply valve opening; always manual, no automatic logic).
// The command is SI: {ch, ld, sp kg/s; hp 0–1; hb groups; rel 0–1; n2 0–1; heat W}.
import {sat,clamp,LIM,NB,THI} from './common.js';
// controller memory, kept apart from the plant state:
// iL, iC net-inflow integrators of the 'pres' and 'lev' inventory logic (kg/s);
// hold the charging/letdown flows a held flow keeps when both are in A; porv the auto-relief latch;
// hb the backup heater groups the on/off logic has on; dem the latest demand for every actuator (SI)
// plus pset, the heater/spray pressure target.
const init=()=>({iL:0,iC:0,hold:null,porv:false,hb:0,dem:{hp:0,hb:0,sp:0,ch:0,ld:0,rel:0,pset:0}});
// heater logic on the pressure error ep = P − target (Pa). Proportional group: 15 % at the target
// (about the hot-standby heat loss), full 0.085 MPa below, off 0.015 MPa above. Backup group i
// (1…NB) on/off with hysteresis: on below −BON(i), off again above −BOFF(i), so the groups come in one
// after another as pressure falls and none cycles while the proportional group holds pressure.
const BON=i=>0.10e6+0.04e6*i,BOFF=i=>BON(i)-0.07e6;
const propH=ep=>clamp(0.15-ep/0.1e6,0,1);
const backH=(n,ep)=>{let m=0;for(let i=1;i<=NB;i++)if(ep<-BON(i)||(i<=n&&ep<-BOFF(i)))m=i;return m};
const relHys=set=>Math.max(0.2e6,0.02*set);
// bumpless engage: preload the inventory integrators from the current charging/letdown, so a
// change of inventory mode starts from the present net flow
const engage=(k,c)=>{k.iL=k.iC=clamp(c.ch-c.ld,-15,15)};
// Automatic logic runs all the time and produces a demand for every actuator (k.dem). Each actuator
// uses its demand only if c.auto[k] is set (see command); otherwise the operator's value in c.
// Nothing switches between auto and manual, or between inventory modes, by itself.
const demands=(k,s,c,dt)=>{
  const d=s.d,P=d.P,A=c.auto||{},e=P-c.set;
  // heaters and spray: 'pres' holds the pressure setpoint; 'dT' holds the pressuriser temperature c.dT (K)
  // above the coolant. With a bubble that is a pressure target Psat(Tr + c.dT) that follows the
  // coolant; water-solid it acts on the liquid temperature. The heater logic sees a pressure error ep;
  // in 'dT' the temperature error counts 0.12 MPa per kelvin. Spray bands are in kelvin there.
  let ep,sp,pset;
  if(c.pzMode==='dT'){pset=sat(Math.min(s.Tr+c.dT,THI)).p;const eT=d.T-s.Tr-c.dT;
    ep=eT*0.12e6;sp=LIM.sp*clamp((eT-0.5)/2.5,0,1)}
  else{pset=c.set;ep=e;sp=LIM.sp*clamp((e-0.1e6)/0.4e6,0,1)}
  const hp=propH(ep);k.hb=backH(k.hb,ep);
  // charging and letdown: a PI controller asks for a net inflow N = charging − letdown.
  // 'pres' holds the pressure setpoint (water-solid), 'lev' the level setpoint (steam bubble).
  // Who controls: the one flow in auto; with both in auto, the mode's preferred flow ('pres' letdown,
  // 'lev' charging) while the other is held at the flow it had when both went to auto (k.hold),
  // taking over only while the preferred flow is at a limit.
  // A flow's demand is what it would do if it were in auto with the other as it is now, so the
  // notch on a manual slider shows what switching it to A would give.
  const lev=c.inv==='lev',err=lev?c.lset-d.level*100:-e,kp=lev?20:5e-5,ki=lev?0.025:2e-7,key=lev?'iC':'iL';
  const pref=lev?'ch':'ld',H=k.hold||(k.hold={ch:c.ch,ld:c.ld});
  const held=f=>A.ch&&A.ld&&f!==pref,eff=f=>A[f]?H[f]:c[f]; // eff: the flow as the other one sees it
  const ctrl=A.ch&&!A.ld?'ch':A.ld&&!A.ch?'ld':pref;
  // A held flow takes over while the preferred one is at its limit: its demand is the preferred
  // demand ∓ N, which equals its held value until the preferred flow saturates.
  const flows=I=>{const N=kp*err+I;let ch=clamp(eff('ld')+N,0,LIM.ch),ld=clamp(eff('ch')-N,0,LIM.ld);
    if(held('ld'))ld=clamp(ch-N,0,LIM.ld);if(held('ch'))ch=clamp(ld+N,0,LIM.ch);
    const both=A.ch&&A.ld;
    return {N,ch,ld,got:both?ch-ld:ctrl==='ch'?ch-eff('ld'):eff('ch')-ld}};
  // integrate unless the controlling flow is already at a limit in the direction the error pushes (anti-windup)
  let I=clamp(k[key]+ki*err*dt,-15,15),r=flows(I);
  if(Math.abs(r.got-r.N)>1e-9&&Math.sign(r.N-r.got)===Math.sign(err)){I=k[key];r=flows(I)}
  k[key]=I;
  // a flow that is not held keeps its hold value current, so holding starts bumplessly
  for(const f of ['ch','ld'])if(!held(f))H[f]=A[f]?r[f]:c[f];
  // relief valve: on/off with hysteresis at c.relSet
  if(P>c.relSet)k.porv=true;else if(P<c.relSet-relHys(c.relSet))k.porv=false;
  k.dem={hp,hb:k.hb,sp,ch:r.ch,ld:r.ld,rel:k.porv?1:0,pset};
};
// final command: the demand for actuators in A, the operator's value for those in M, in SI;
// the nitrogen valve is always the operator's
const command=(k,c)=>{
  const A=c.auto||{},D=k.dem;
  return {ch:A.ch?D.ch:c.ch,ld:A.ld?D.ld:c.ld,hp:A.hp?D.hp:c.hp/100,hb:A.hb?D.hb:c.hb,sp:A.sp?D.sp:c.sp,
    rel:A.rel?D.rel:(c.relMan||0)/100,n2:(c.n2||0)/100,heat:c.heat*1e6};
};
// one control step: update the demands from the plant state, return the command for PZSim.step.
// Call it before every plant step, with the same dt.
const update=(k,s,c,dt)=>{demands(k,s,c,dt);return command(k,c)};
export {init,engage,demands,command,update,relHys,propH,backH,BON,BOFF};
