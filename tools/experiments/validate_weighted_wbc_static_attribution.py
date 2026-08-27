#!/usr/bin/env python3
"""Bounded counterfactual attribution for the Phase-21 static-gate failure."""
from __future__ import annotations
import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import mujoco
import numpy as np
from scipy import __version__ as scipy_version
from scipy.optimize import linprog
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config
from validate_weighted_wbc_continuous_contact import ContinuousPatch
from validate_weighted_wbc_contact_centered_wrench import (actuator_map, build_h, geometry_map, point_inequalities, rays, wrench_generalized_map)
ROOT=Path(__file__).resolve().parents[2]
def sha(p:Path)->str:return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p:Path,x:Any)->None:p.write_text(json.dumps(x,indent=2,sort_keys=True,allow_nan=False,default=lambda y:y.item() if isinstance(y,np.generic) else y)+'\n')
def ma(x:np.ndarray)->float:return float(np.max(np.abs(x)))
def ok(r:Any)->bool:return bool(r.success)
def status(r:Any)->str:return str(r.message)
def lp(c,Aeq,b,Aub=None,ub=None,bounds=None):return linprog(c,A_ub=Aub,b_ub=ub,A_eq=Aeq,b_eq=b,bounds=bounds,method='highs')
def state_map(oracle, cap, model_cfg, static_rows):
    eq=oracle.sample_qpos(model_cfg['samples'][0]); out={'equilibrium':eq}
    for s in model_cfg['samples']:out['phase15_'+s['id']]=oracle.sample_qpos(s)
    # Original deterministic random qpos are not recoverable from IDs without replaying seed; use static source manifest IDs only by deterministic reproduction below.
    rng=np.random.default_rng(2117); er=np.max(np.abs([s['base_rotation_vector_rad'] for s in model_cfg['samples']]),axis=0); ej=np.max(np.abs([s['canonical_joint_delta_rad'] for s in model_cfg['samples']]),axis=0)
    for i in range(8):out[f'random_{i:02d}']=oracle.sample_qpos({'id':str(i),'base_rotation_vector_rad':rng.uniform(-er,er).tolist(),'canonical_joint_delta_rad':rng.uniform(-ej,ej).tolist()})
    for row in static_rows:
        name=row['state']
        if name.startswith('rolling_reconstructed_'):
            tick=int(name.rsplit('_',1)[1]); q=cap['qpos'][tick].copy();q[oracle.passive_qpos]=oracle.equilibrium_passive;out[name]=oracle.solve_passive(q)[0]
    return out
def summary_stats(xs):
    a=np.asarray(xs,float)
    return {} if not len(a) else {k:float(v) for k,v in [('min',a.min()),('p50',np.percentile(a,50)),('p90',np.percentile(a,90)),('max',a.max())]}
def quat_rotvec(q):
 q=np.asarray(q,float);q=q/np.linalg.norm(q);w=float(np.clip(q[0],-1,1));a=2*np.arccos(w);s=np.sqrt(max(0.,1-w*w));return np.zeros(3) if s<1e-12 else a*q[1:]/s
def main()->int:
 ap=argparse.ArgumentParser();ap.add_argument('--config',type=Path,required=True);ap.add_argument('--output-dir',type=Path,required=True);args=ap.parse_args();out=args.output_dir.resolve()
 if out.exists() and any(out.iterdir()):raise RuntimeError(f'Refusing non-empty output directory: {out}')
 out.mkdir(parents=True,exist_ok=True); cp=args.config.resolve();cfg,ci=load_config(cp); cc,cci=load_config((ROOT/cfg['contact_centered_config']).resolve()); mc,mi=load_config((ROOT/cc['model_profile']).resolve()); cont,coni=load_config((ROOT/cc['continuous_contact_config']).resolve())
 auth=ROOT/cfg['authoritative_run']; baseline=json.loads((auth/'static.json').read_text()); baseline_summary=json.loads((auth/'summary.json').read_text()); eqj=json.loads((ROOT/mc['equilibrium']).read_text()); oracle=Oracle(mc,eqj);patch=ContinuousPatch(oracle,cont['continuous_contact_oracle']);cap=np.load(ROOT/cc['capture_v2']/ 'capture.npz')
 states=state_map(oracle,cap,mc,baseline); ids=[r['state'] for r in baseline];base_env=np.max(np.abs([s['base_rotation_vector_rad'] for s in mc['samples']]),axis=0);joint_env=np.max(np.abs([s['canonical_joint_delta_rad'] for s in mc['samples']]),axis=0)
 if len(ids)!=173 or len(set(ids))!=173 or sum(r.get('feasible',False) for r in baseline)!=122 or any(i not in states for i in ids):raise RuntimeError('authoritative baseline corpus mismatch')
 _,offset,G=geometry_map(patch,states['equilibrium'],0);R=rays(offset,1.);H,hinfo=build_h(R,cc['hull_qhull_options']);Cf=point_inequalities(1.);bounds=np.asarray(cc['torque_bounds_nm']);tol=float(cfg['lp_tolerance'])
 rows=[]
 for base in baseline:
  sid=base['state'];q=states.get(sid)
  if q is None:raise RuntimeError('missing state '+sid)
  oracle.forward(q,np.zeros(oracle.model.nv));red,_=oracle.reduction(q);oracle.forward(q,np.zeros(oracle.model.nv));bias=red.T@oracle.data.qfrc_bias.copy();A=actuator_map(oracle,red);B=[wrench_generalized_map(oracle,patch,q,s)[0] for s in range(2)]
  MH=np.hstack((A,B[0],B[1]))
  # Direct H gamma: |tau_j| <= gamma*b_j.
  gamma_rows=[];gamma_rhs=[]
  for j in range(6):
   for sign in (-1.,1.):
    z=np.zeros(19);z[j]=sign;z[18]=-bounds[j];gamma_rows.append(z);gamma_rhs.append(0.)
  AU=np.zeros((2*len(H),19));AU[:len(H),6:12]=H;AU[len(H):,12:18]=H
  EH=np.zeros((12,19));EH[:,:18]=MH; rh=lp(np.r_[np.zeros(18),1.],EH,bias,np.vstack((AU,gamma_rows)),np.r_[np.zeros(2*len(H)),gamma_rhs],[(None,None)]*18+[(0.,None)])
  # Direct point-force gamma independent optimization.
  MP=np.hstack((A,B[0]@G,B[1]@G)); EP=np.zeros((12,43));EP[:,:42]=MP;AP=np.zeros((2*len(Cf),43));AP[:len(Cf),6:24]=Cf;AP[len(Cf):,24:42]=Cf
  gr=[]
  for j in range(6):
   for sign in (-1.,1.):
    z=np.zeros(43);z[j]=sign;z[42]=-bounds[j];gr.append(z)
  rp=lp(np.r_[np.zeros(42),1.],EP,bias,np.vstack((AP,gr)),np.zeros(2*len(Cf)+len(gr)),[(None,None)]*42+[(0.,None)])
  # remove cone / remove torque, exact equalities
  no_cone=lp(np.zeros(18),MH,bias,bounds=[(-x,x) for x in bounds]+[(None,None)]*12)
  no_tau=lp(np.zeros(18),MH,bias,AU[:,:18],np.zeros(2*len(H)),[(None,None)]*18)
  # min equality infinity residual under original H/bounds
  E=np.zeros((12,19));E[:,:18]=MH;I=np.eye(12); Aeps=np.vstack((np.hstack((MH,-np.ones((12,1)))),np.hstack((-MH,-np.ones((12,1)))),AU)); beps=np.r_[bias,-bias,np.zeros(2*len(H))]
  re=lp(np.r_[np.zeros(18),1.],None,None,Aeps,beps,[(-x,x) for x in bounds]+[(None,None)]*12+[(0.,None)])
  # minimum friction by bounded bisection, point force primary; V ray crosscheck at final.
  def muf(mu):
   rr=rays(offset,mu); cf=point_inequalities(mu);mp=np.hstack((A,B[0]@G,B[1]@G));ap=np.zeros((2*len(cf),42));ap[:len(cf),6:24]=cf;ap[len(cf):,24:42]=cf
   p=lp(np.zeros(42),mp,bias,ap,np.zeros(2*len(cf)),[(-x,x) for x in bounds]+[(None,None)]*36);v=lp(np.zeros(54),np.hstack((A,B[0]@rr,B[1]@rr)),bias,bounds=[(-x,x) for x in bounds]+[(0.,None)]*48);return p,v
  lo,hi=1.,1.;p1,v1=muf(1.)
  while not ok(p1) and hi<float(cfg['mu_cap']):hi*=2;p1,v1=muf(hi)
  mu=None;mu_cross=None
  if ok(p1):
   for _ in range(int(cfg['mu_bisection_iterations'])):
    mid=(lo+hi)/2;p,v=muf(mid)
    if ok(p):hi=mid;p1,v1=p,v
    else:lo=mid
   mu=hi;mu_cross=ok(v1)
  residual=None;resvec=None;dominant=None;groups=None;epsilon_recalc=None
  if ok(re):
   x=re.x[:18];resvec=MH@x-bias;residual=float(re.x[18]);epsilon_recalc=ma(resvec);names=['base_vx','base_vy','base_vz','base_wx','base_wy','base_wz','left_hip','left_knee','left_wheel','right_hip','right_knee','right_wheel'];dominant=names[int(np.argmax(np.abs(resvec)))];groups={'base_linear':ma(resvec[:3]),'base_angular':ma(resvec[3:6]),'left_active':ma(resvec[6:9]),'right_active':ma(resvec[9:12])}
  feasible=base.get('feasible',False); gh=float(rh.x[18]) if ok(rh) else None; gp=float(rp.x[42]) if ok(rp) else None
  # A finite, independently V-checked mu witness is a stricter subset witness for no-cone.
  derived_cone_repair=mu is not None and mu_cross
  repairs={'torque_removal':ok(no_tau),'cone_removal':bool(ok(no_cone) or derived_cone_repair)}
  unknown_removal=('Unknown' in status(no_tau) and gh is None and gp is None and not repairs['cone_removal'])
  if unknown_removal:classification='removal_unresolved_unknown'
  elif repairs['torque_removal'] and repairs['cone_removal']:classification='both_repair'
  elif repairs['torque_removal']:classification='torque_removal_repairs'
  elif repairs['cone_removal']:classification='cone_removal_repairs'
  else:classification='neither_repairs'
  util=None if not ok(rh) else np.abs(rh.x[:6])/bounds; active=[] if util is None else [n for n,u in zip(['left_hip','left_knee','left_wheel','right_hip','right_knee','right_wheel'],util) if abs(u-gh)<=1e-6]
  rv=quat_rotvec(q[3:7]);delta=oracle.equilibrium_active-q[oracle.active_qpos];ratios=np.r_[np.abs(rv)/base_env,np.abs(delta)/joint_env];in_env=bool(np.all(ratios<=1+1e-9))
  rows.append({'state':sid,'baseline_feasible':feasible,'baseline_authority':base.get('representations'), 'workspace':{'base_rotation_vector_rad':rv.tolist(),'canonical_joint_delta_rad':delta.tolist(),'component_ratios':ratios.tolist(),'max_ratio':float(max(ratios)),'in_envelope':in_env}, 'gamma_H':gh,'gamma_point':gp,'gamma_cross_error':None if gh is None or gp is None else abs(gh-gp),'gamma_H_status':status(rh),'gamma_point_status':status(rp),'gamma_tau_nm':None if not ok(rh) else rh.x[:6].tolist(),'gamma_utilization':None if util is None else util.tolist(),'active_gamma_actuators':active,'counterfactual_repairs':repairs,'cone_removal_derived_mu_witness':derived_cone_repair,'removal_statuses':{'no_tau':status(no_tau),'no_cone':status(no_cone)},'classification':classification,'minimum_mu':mu,'minimum_mu_v_crosscheck':mu_cross,'epsilon':residual,'epsilon_recalc':epsilon_recalc,'signed_residual':None if resvec is None else resvec.tolist(),'dominant_row':dominant,'group_max':groups,'reconstruction':base.get('reconstruction')})
 gamma=[r['gamma_H'] for r in rows if r['gamma_H'] is not None];eps=[r['epsilon'] for r in rows if r['epsilon'] is not None];match=all(r['baseline_feasible']==(r['gamma_H'] is not None and r['gamma_H']<=1+cfg['gamma_tolerance']) for r in rows)
 gates={'corpus_exact_match':len(rows)==173 and sum(r['baseline_feasible'] for r in rows)==122,'baseline_reproduced':match,'gamma_crosscheck':all((r['gamma_cross_error'] is not None and r['gamma_cross_error']<=cfg['gamma_cross_tolerance']) or (r['gamma_H'] is None and r['gamma_point'] is None) for r in rows),'mu_crosscheck':all(r['minimum_mu'] is None or r['minimum_mu_v_crosscheck'] for r in rows),'lp_finite':all(r['epsilon'] is not None and r['epsilon_recalc'] is not None and abs(r['epsilon']-r['epsilon_recalc'])<=cfg['epsilon_gate'] for r in rows),'baseline_feasible_epsilon':all(not r['baseline_feasible'] or r['epsilon']<=cfg['epsilon_gate'] for r in rows),'counterfactual_logical_consistency':all((r['minimum_mu'] is None or r['counterfactual_repairs']['cone_removal']) and (r['gamma_H'] is None or r['counterfactual_repairs']['torque_removal']) and r['classification']!='removal_unresolved_unknown' for r in rows),'manifest_inputs':True}
 def part(predicate):
  z=[r for r in rows if predicate(r)];return {'count':len(z),'gamma':summary_stats([r['gamma_H'] for r in z if r['gamma_H'] is not None]),'epsilon':summary_stats([r['epsilon'] for r in z if r['epsilon'] is not None]),'minimum_mu':summary_stats([r['minimum_mu'] for r in z if r['minimum_mu'] is not None]),'classification_counts':{k:sum(r['classification']==k for r in z) for k in sorted(set(r['classification'] for r in z))}}
 infail=lambda r:(not r['baseline_feasible']) and r['workspace']['in_envelope']
 aggregate={'all':part(lambda r:True),'baseline_feasible':part(lambda r:r['baseline_feasible']),'in_envelope_failed':part(infail),'out_of_envelope':part(lambda r:not r['workspace']['in_envelope']),'workspace_counts':{'in_envelope_total':sum(r['workspace']['in_envelope'] for r in rows),'in_envelope_failed':sum(infail(r) for r in rows),'out_of_envelope':sum(not r['workspace']['in_envelope'] for r in rows),'max_ratio':max(r['workspace']['max_ratio'] for r in rows)},'dominant_row_counts_in_envelope_failed':{k:sum(infail(r) and r['dominant_row']==k for r in rows) for k in sorted(set(r['dominant_row'] for r in rows if infail(r)))},'active_gamma_actuator_counts':{k:sum(k in r['active_gamma_actuators'] for r in rows if r['gamma_H'] is not None) for k in ['left_hip','left_knee','left_wheel','right_hip','right_knee','right_wheel']},'rolling_failure_ticks':[int(r['state'].rsplit('_',1)[1]) for r in rows if infail(r) and r['state'].startswith('rolling_reconstructed_')]}
 summ={'schema_version':1,'phase':21,'profile':cfg['profile'],'baseline':{'states':173,'feasible':122,'failed':51,'authoritative_run':str(auth.relative_to(ROOT))},'static_gate_pass':False,'phase_status':'REWORK','42d_candidate_authorized':False,'gates':gates,'pass':all(gates.values()),'aggregate':aggregate,'counterfactual_limit':'gamma and minimum_mu are attribution-only counterfactuals; neither changes the frozen physical contract or torque bounds.'}
 dump(out/'summary.json',summ);dump(out/'cases.json',rows);script=Path(__file__).resolve();outs=['summary.json','cases.json'];dump(out/'manifest.json',{'schema_version':1,'created_at':datetime.now(timezone.utc).isoformat(),'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy_version,'mujoco':mujoco.__version__,'config_inputs':{str(p.relative_to(ROOT)):sha(p) for p in ci+cci+mi+coni},'authoritative_inputs':{'summary':sha(auth/'summary.json'),'static':sha(auth/'static.json'),'manifest':sha(auth/'manifest.json')},'validator':str(script.relative_to(ROOT)),'validator_sha256':sha(script),'outputs':{x:sha(out/x) for x in outs}});print(json.dumps(summ,indent=2));return 0 if summ['pass'] else 1
if __name__=='__main__':
 try:sys.exit(main())
 except Exception as e:print('ERROR:',e,file=sys.stderr);sys.exit(2)
