// BWR reactor protection: the reactor protection system (RPS) scram trips, the main steam isolation
// logic and the turbine and feed pump trips. Pure functions, no DOM. An ES module; imports common.js and
// the system modules for their constants only. bwr-sim.js runs it inside step(): protection is part of the
// plant, not a controller, so it has no M/A and acts whatever the operator's switches say. See docs/bwr.md.
//
// Each trip compares one signal from the start-of-step state with its setpoint. A trip that fires latches
// (seals in) and is logged; it stays in until the operator resets it and its signal has cleared. Some trips
// are bypassed by the mode switch (Shutdown, Startup, Run) or by low power, as in the plant. Setpoints are
// typical BWR/4 values from technical specifications and training material; plants differ, so they are
// labelled typical, not those of one plant. Level setpoints are inches above instrument zero (bwr-vessel.js).
import {psig} from './bwr-steam.js';
import {LV} from './bwr-vessel.js';
// condenser pressure from inches of mercury vacuum (against a standard atmosphere)
const inHgVac=x=>101325-x*3386.39;
// Setpoints (Pa abs, fractions of rated, inches above instrument zero):
//  scram: high dome pressure 1045 psig; low level L3; APRM high flux 120 % in Run (the flow-biased trip's
//  clamp) and 15 % in Startup; MSIVs below 90 % open (Run only); turbine stop valve below 90 % open with
//  first-stage pressure above 30 % power; low condenser vacuum 23 inHg (Run only); mode switch to Shutdown.
//  isolation (closes both MSIVs): low-low level L2 (some BWR/4s use L1); main steam line low pressure
//  850 psig at the header (Run only); main steam line high flow 140 % of rated; low condenser vacuum 7 inHg
//  (bypassed outside Run, as with the startup bypass switch).
//  turbine control valve fast closure (the power–load unbalance, on a load rejection) with first-stage
//  pressure above 30 % (as the stop valve closure).
//  turbine: high level L8 (also trips the feed pump); low condenser vacuum 22.5 inHg; overspeed 110 %.
//  bypass valves: close while the condenser vacuum is below 7 inHg (an interlock, not latched).
const SP=Object.freeze({P:psig(1045),aprmRun:1.20,aprmSU:0.15,ms:0.9,sv:0.9,fsp:0.30,vacScram:inHgVac(23),
  plow:psig(850),wHigh:1.40,vacIso:inHgVac(7),vacTurb:inHgVac(22.5),ovsp:1.10,vacBP:inHgVac(7),L3:LV.L3,L2:LV.L2,L8:LV.L8});
const MODES=['shutdown','startup','run'];
// Trip list: key, group (scram | iso | turb | fwp), text for the log and the annunciator, the modes it is
// active in (all if absent), and the test on the signals g.
const TRIPS=[
  {k:'P',g:'scram',t:'Reactor high pressure',f:g=>g.P>SP.P},
  {k:'L3',g:'scram',t:'Reactor low level (L3)',f:g=>g.lin<SP.L3},
  {k:'aprm',g:'scram',t:'APRM high flux',f:g=>g.aprm>(g.mode==='run'?SP.aprmRun:SP.aprmSU)},
  {k:'msc',g:'scram',t:'MSIV closure',m:['run'],f:g=>g.ms<SP.ms},
  {k:'tsv',g:'scram',t:'Turbine stop valve closure',f:g=>g.sv<SP.sv&&g.fsp>SP.fsp},
  {k:'tcv',g:'scram',t:'Turbine control valve fast closure',f:g=>!!g.plu&&g.fsp>SP.fsp},
  {k:'vac',g:'scram',t:'Condenser low vacuum',m:['run'],f:g=>g.Pc>SP.vacScram},
  {k:'mode',g:'scram',t:'Mode switch in Shutdown',m:['shutdown'],f:()=>true},
  {k:'L2',g:'iso',t:'Reactor low-low level (L2)',f:g=>g.lin<SP.L2},
  {k:'plow',g:'iso',t:'Main steam line low pressure',m:['run'],f:g=>g.Pl<SP.plow},
  {k:'wHigh',g:'iso',t:'Main steam line high flow',f:g=>g.wst>SP.wHigh},
  {k:'vacIso',g:'iso',t:'Condenser low vacuum (isolation)',m:['run'],f:g=>g.Pc>SP.vacIso},
  {k:'L8',g:'turb',t:'Reactor high level (L8)',f:g=>g.lin>SP.L8},
  {k:'vacT',g:'turb',t:'Condenser low vacuum (turbine)',f:g=>g.Pc>SP.vacTurb},
  {k:'ovsp',g:'turb',t:'Turbine overspeed',f:g=>g.spd>SP.ovsp},
  {k:'L8f',g:'fwp',t:'Reactor high level (L8): feed pump trip',f:g=>g.lin>SP.L8}];
const GROUP={scram:'Scram',iso:'MSIV isolation',turb:'Turbine trip',fwp:'Feed pump trip'};
const LOGN=60;
// protection state: mode switch, latched trips by key, event log [{t, text}], off (tests only: no protection)
const rpsInit=(mode='run',off=false)=>({mode,lat:{},log:[],off});
const on=(tr,g)=>!tr.m||tr.m.includes(g.mode);
const log=(r,t,text)=>{r.log.push({t,text});if(r.log.length>LOGN)r.log.splice(0,r.log.length-LOGN)};
// latched groups: {scram, iso, turb, fwp} booleans
const groups=r=>{const o={scram:false,iso:false,turb:false,fwp:false};for(const tr of TRIPS)if(r.lat[tr.k])o[tr.g]=true;return o};
// check every trip against the signals g (adds g.mode); latch and log the ones that fire. Returns groups().
const rpsCheck=(r,g,t)=>{
  if(r.off)return groups(r);
  g.mode=r.mode;const was=groups(r);
  for(const tr of TRIPS)if(!r.lat[tr.k]&&on(tr,g)&&tr.f(g)){r.lat[tr.k]=true;log(r,t,tr.t)}
  const now=groups(r);
  for(const k in now)if(now[k]&&!was[k])log(r,t,`→ ${GROUP[k]}`);
  return now;
};
// reset: clears every latched trip whose signal has cleared (or that its mode now bypasses); those still
// present stay in. Returns the trips left latched.
const rpsReset=(r,g,t)=>{
  g.mode=r.mode;const left=[];
  for(const tr of TRIPS)if(r.lat[tr.k]){if(on(tr,g)&&tr.f(g))left.push(tr.t);else delete r.lat[tr.k]}
  log(r,t,left.length?`Reset: still in: ${left.join(', ')}`:'Reset');
  return left;
};
// the mode switch; to Shutdown scrams at once
const rpsMode=(r,mode,t)=>{if(!MODES.includes(mode)||mode===r.mode)return;r.mode=mode;log(r,t,`Mode switch to ${mode[0].toUpperCase()+mode.slice(1)}`)};
// bypass valve interlock (not latched): shut while the condenser vacuum is low
const bpBlock=(r,Pc)=>!r.off&&Pc>SP.vacBP;
// latched trips with their text, for the page
const rpsActive=r=>TRIPS.filter(tr=>r.lat[tr.k]).map(tr=>({k:tr.k,g:tr.g,t:tr.t}));
export {SP,MODES,TRIPS,GROUP,inHgVac,rpsInit,rpsCheck,rpsReset,rpsMode,bpBlock,rpsActive,groups};
