#!/usr/bin/env python3
"""Phase-21 closing attribution: frozen reduced cone versus an offline plant LP."""
from __future__ import annotations
import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import mujoco
import numpy as np
from scipy import __version__ as scipy_version
from scipy.optimize import LinearConstraint, linprog, minimize
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config
from validate_weighted_wbc_continuous_contact import ContinuousPatch
from validate_weighted_wbc_contact_centered_wrench import actuator_map, build_h, geometry_map, rc, rays, wrench_generalized_map
from validate_weighted_wbc_static_attribution import quat_rotvec, state_map
from validate_weighted_wbc_tasks import Plant
ROOT = Path(__file__).resolve().parents[2]
NAMES = ["base_Fx", "base_Fy", "base_Fz", "base_Mx", "base_My", "base_Mz", "left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"]
LABELS = ["rolling force", "lateral force", "normal unilateral", "roll-moment support", "pitch-moment support", "torsional moment"]
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p, x): p.write_text(json.dumps(x, indent=2, sort_keys=True, allow_nan=False, default=lambda v: v.item() if isinstance(v, np.generic) else v)+"\n")
def ma(x): return float(np.max(np.abs(x)))
def lp(c, ae=None, be=None, au=None, bu=None, bounds=None): return linprog(c, A_ub=au, b_ub=bu, A_eq=ae, b_eq=be, bounds=bounds, method="highs")
def normw(w, fs, ms): return np.asarray(w)/np.r_[np.full(3,fs),np.full(3,ms)]
def result(r): return {"feasible":bool(r.success), "status":str(r.message)}
def constraints(H, bounds):
    au=np.zeros((2*len(H),18)); au[:len(H),6:12]=H; au[len(H):,12:18]=H
    return au, [(-x,x) for x in bounds]+[(None,None)]*12
def static_layers(M,b,H,bounds):
    au, bd=constraints(H,bounds); out={}
    for key, rows in (("base_only",np.arange(6)),("active_only",np.arange(6,12)),("full",np.arange(12))):
        r=lp(np.zeros(18),M[rows],b[rows],au,np.zeros(len(au)),bd); out[key]=result(r)
    # minimum infinity equality residual and rowwise witness.
    A=np.vstack((np.c_[M,-np.ones((12,1))],np.c_[-M,-np.ones((12,1))],np.c_[au,np.zeros(len(au))]))
    r=lp(np.r_[np.zeros(18),1.],None,None,A,np.r_[b,-b,np.zeros(len(au))],bd+[(0,None)])
    x=np.zeros(18) if not r.success else r.x[:18]; e=M@x-b
    out["full_min_linf"]={**result(r),"epsilon":None if not r.success else float(r.x[-1]),"residual_vector":e.tolist(),"row_groups":{"base":ma(e[:6]),"active":ma(e[6:])},"first_failing_layer":None if out["full"]["feasible"] else ("base" if not out["base_only"]["feasible"] else "active")}
    return out
def l1_witness(M, b, bounds, fs, ms):
    # Signed variables plus epigraphs make the normalized L1 rule deterministic.
    scale=np.r_[bounds,np.tile(np.r_[np.full(3,fs),np.full(3,ms)],2)]
    E=np.c_[M,np.zeros((12,18))]; I=np.eye(18)
    au=np.vstack((np.c_[ I/scale[:,None],-I],np.c_[-I/scale[:,None],-I]))
    r=lp(np.r_[np.zeros(18),np.ones(18)],E,b,au,np.zeros(36),[(-x,x) for x in bounds]+[(None,None)]*12+[(0,None)]*18)
    return r, (np.zeros(18) if not r.success else r.x[:18])
def projection(w,H,fs,ms,tol):
    scale=np.r_[np.full(3,fs),np.full(3,ms)]; z0=np.zeros(6) # zero is cone feasible
    fun=lambda z: .5*np.dot(z-normw(w,fs,ms),z-normw(w,fs,ms))
    con=LinearConstraint(H*scale[None,:], -np.inf, np.zeros(len(H)))
    r=minimize(fun,z0,method="SLSQP",constraints=[con],options={"ftol":tol,"maxiter":1000})
    z=np.asarray(r.x); q=z*scale
    # Explicit deterministic L-infinity LP in the identical normalized metric.
    A=np.vstack((np.c_[np.eye(6),-np.ones(6)],np.c_[-np.eye(6),-np.ones(6)],np.c_[H*scale[None,:],np.zeros(len(H))]))
    bb=np.r_[normw(w,fs,ms),-normw(w,fs,ms),np.zeros(len(H))]
    ri=lp(np.r_[np.zeros(6),1.],None,None,A,bb,[(None,None)]*6+[(0,None)])
    qi=np.zeros(6) if not ri.success else ri.x[:6]*scale
    return {"l2_success":bool(r.success),"nearest_l2":q.tolist(),"delta_l2":(q-w).tolist(),"l2_normalized":float(np.linalg.norm(z-normw(w,fs,ms))),"active_facets":np.flatnonzero(np.abs(H@q)<=tol).tolist(),"linf_success":bool(ri.success),"nearest_linf":qi.tolist(),"delta_linf":(qi-w).tolist(),"linf_normalized":None if not ri.success else float(ri.x[-1])}
def plant_lp(plant, oracle, q, bounds, patch, tol):
    d=plant.data; d.qpos[:]=q; d.qvel[:]=0; d.ctrl[:]=0; mujoco.mj_forward(plant.model,d)
    # Discover contacts only; contactForce is deliberately not used as an LP answer.
    cols=[]; meta=[]; C=[]
    for i,c in enumerate(d.contact):
      for side,gid in enumerate(plant.wheel_geoms):
       if {c.geom1,c.geom2}!={plant.floor,gid}: continue
       # local contact normal is frame row 0; use actual normal plus its two tangents.
       frame=c.frame.reshape(3,3).T; dirs=[frame[:,0],frame[:,1],frame[:,2]]
       sign=1. if c.geom2==gid else -1.
       for axis,v in enumerate(dirs):
        gf=np.zeros(plant.model.nv); mujoco.mj_applyFT(plant.model,d,sign*v,np.zeros(3),c.pos,int(plant.wheel_bodies[side]),gf); C.append(gf); cols.append((len(meta),axis));
       meta.append({"side":side,"point_world":c.pos.tolist(),"friction":float(c.friction[0]),"contact_index":i})
    C=np.asarray(C).T if C else np.zeros((plant.model.nv,0)); A=np.zeros((plant.model.nv,6))
    for j,a in enumerate(plant.actuators):
      joint=int(plant.model.actuator_trnid[a,0]); A[int(plant.model.jnt_dofadr[joint]),j]=-float(plant.model.actuator_gear[a,0])
    J=[]
    for first,second in oracle.closure_sites:
      a=np.zeros((3,plant.model.nv)); b=np.zeros_like(a); mujoco.mj_jacSite(plant.model,d,a,np.zeros_like(a),first);mujoco.mj_jacSite(plant.model,d,b,np.zeros_like(b),second);J.extend(a-b)
    J=np.asarray(J).T; K=np.c_[A,J,C]; au=[]
    # per actual contact [fn,ft1,ft2], fn>=0 and |ft|<=mu fn
    for k,m in enumerate(meta):
      off=6+J.shape[1]+3*k; mu=m["friction"]
      for t in (1,2):
       for s in (-1.,1.): row=np.zeros(K.shape[1]);row[off+t]=s;row[off]=-mu;au.append(row)
    bd=[(-x,x) for x in bounds]+[(None,None)]*J.shape[1]+[(0,None) if a==0 else (None,None) for _,a in cols]
    rr=lp(np.zeros(K.shape[1]),K,d.qfrc_bias.copy(),np.asarray(au),np.zeros(len(au)),bd)
    # A bounded least-Linf residual proves infeasibility magnitude without dynamic contact forces.
    Z=np.c_[K,-np.ones((plant.model.nv,1))]; zz=np.c_[-K,-np.ones((plant.model.nv,1))]
    ri=lp(np.r_[np.zeros(K.shape[1]),1.],None,None,np.r_[Z,zz,np.c_[np.asarray(au),np.zeros((len(au),1))]],np.r_[d.qfrc_bias,-d.qfrc_bias,np.zeros(len(au))],bd+[(0,None)])
    x=np.zeros(K.shape[1]) if not rr.success and not ri.success else (rr.x if rr.success else ri.x[:-1]); f=x[6+J.shape[1]:]
    out={"feasible":bool(rr.success),"status":str(rr.message),"minimum_linf_residual":None if not ri.success else float(ri.x[-1]),"equilibrium_residual":None if not rr.success else ma(K@x-d.qfrc_bias),"actuator_tau":None if not rr.success else x[:6].tolist(),"minimum_residual_witness":{"tau":x[:6].tolist(),"full_residual":(K@x-d.qfrc_bias).tolist()},"contact_count":len(meta),"contacts":meta,"closure_columns":int(J.shape[1]),"crosscheck_Nt_full":ma((oracle.reduction(q)[0].T@(K@x-d.qfrc_bias)))}
    # Resultants use only the solved LP forces, about frozen analytic centres and frames.
    wr=[]
    for side in range(2):
      g,_,_=geometry_map(patch,q,side); center=np.asarray(g["contact_center"]); frame=rc(g,patch.n); F=np.zeros(3);T=np.zeros(3)
      for k,m in enumerate(meta):
       if m["side"]!=side:continue
       ff=f[3*k:3*k+3]; cf=plant.data.contact[m["contact_index"]]; world=(1. if cf.geom2==plant.wheel_geoms[side] else -1.)*(cf.frame.reshape(3,3).T@ff);F+=world;T+=np.cross(np.asarray(m["point_world"])-center,world)
      wr.append(np.r_[frame.T@F,frame.T@T].tolist())
    out["wrench_C"]=wr; out["minimum_residual_witness"]["wrench_C"]=wr; return out
def main():
 p=argparse.ArgumentParser();p.add_argument("--config",type=Path,required=True);p.add_argument("--output-dir",type=Path,required=True);a=p.parse_args();out=a.output_dir.resolve()
 if out.exists() and any(out.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {out}")
 out.mkdir(parents=True,exist_ok=True); cfg,ci=load_config(a.config.resolve()); cc,cci=load_config(ROOT/cfg["contact_centered_config"]); mc,mci=load_config(ROOT/cc["model_profile"]); cont,coni=load_config(ROOT/cc["continuous_contact_config"])
 eqj=json.loads((ROOT/mc["equilibrium"]).read_text()); oracle=Oracle(mc,eqj);patch=ContinuousPatch(oracle,cont["continuous_contact_oracle"]);plant=Plant(mc,eqj);cap=np.load(ROOT/cfg["capture"]); static=json.loads((ROOT/cfg["authoritative_static"]).read_text()); allstates=state_map(oracle,cap,mc,static)
 for t in set(int(x.split("_")[1]) for x in cfg["failure_states"]+cfg["control_states"] if x.startswith("tick_")):
  q=cap["qpos"][t].copy();q[oracle.passive_qpos]=oracle.equilibrium_passive;allstates[f"tick_{t}"]=oracle.solve_passive(q)[0]
 _,off,_=geometry_map(patch,allstates["equilibrium"],0);H,_=build_h(rays(off,1.),cc["hull_qhull_options"]); bounds=np.asarray(cc["torque_bounds_nm"]);fs=float(cfg["force_scale_n"]) or float(-oracle.model.opt.gravity[2]*np.sum(oracle.model.body_mass)/2);ms=fs*float(cfg["wheel_radius_m"]); rows=[]
 for sid in cfg["failure_states"]+cfg["control_states"]:
  q=allstates[sid];oracle.forward(q,np.zeros(oracle.model.nv));N,recon=oracle.reduction(q);h=N.T@oracle.data.qfrc_bias.copy();A=actuator_map(oracle,N);B=[];geo=[]
  for s in range(2): b,_,g=wrench_generalized_map(oracle,patch,q,s);B.append(b);geo.append({"frame":rc(g,patch.n).tolist(),"center":np.asarray(g["contact_center"]).tolist(),"B":b.tolist()})
  M=np.c_[A,B[0],B[1]]; no,witness=l1_witness(M,h,bounds,fs,ms); wl,wr=witness[6:12],witness[12:18]; facets=[]
  for side,w in enumerate((wl,wr)):
   v=H@w; facets.append({"H_times_w":v.tolist(),"violations":[{"index":int(i),"magnitude":float(v[i]),"expression":H[i].tolist(),"physical_label":LABELS[int(np.argmax(np.abs(H[i])))]} for i in np.flatnonzero(v>float(cfg["lp_tolerance"]))]})
  proj=[projection(w,H,fs,ms,float(cfg["projection_tolerance"])) for w in (wl,wr)]; wp=np.r_[np.asarray(proj[0]["nearest_l2"]),np.asarray(proj[1]["nearest_l2"])]; residual=h-M@np.r_[witness[:6],wp]
  pl=plant_lp(plant,oracle,q,bounds,patch,float(cfg["lp_tolerance"])); rv=quat_rotvec(q[3:7]); er=np.max(np.abs([s["base_rotation_vector_rad"] for s in mc["samples"]]),axis=0);ej=np.max(np.abs([s["canonical_joint_delta_rad"] for s in mc["samples"]]),axis=0);ratio=np.r_[np.abs(rv)/er,np.abs(oracle.equilibrium_active-q[oracle.active_qpos])/ej]
  failure=sid in cfg["failure_states"]; contract=bool(cfg["phase15_workspace_static_admissible"])
  cls="A" if failure and max(ratio)<=1+1e-9 and not pl["feasible"] and not contract else ("B" if failure and pl["feasible"] and not static_layers(M,h,H,bounds)["full"]["feasible"] else ("C" if failure and not pl["feasible"] and contract else "D"))
  rows.append({"state":sid,"failure":failure,"qpos":q.tolist(),"active_qpos":q[oracle.active_qpos].tolist(),"passive_qpos":q[oracle.passive_qpos].tolist(),"N":N.tolist(),"h_r":h.tolist(),"S_r":A.tolist(),"geometry":geo,"H":H.tolist(),"torque_bounds_nm":bounds.tolist(),"reconstruction":recon,"workspace":{"base_rotvec":rv.tolist(),"ratios":ratio.tolist(),"in_envelope":bool(max(ratio)<=1+1e-9)},"cone_removed_witness":{"rule":"normalized L1 epigraph LP over signed canonical tau,wrenches","feasible":bool(no.success),"tau":witness[:6].tolist(),"wL":wl.tolist(),"wR":wr.tolist(),"facets":facets},"nearest_cone_correction":proj,"with_original_tau_12d_residual":residual.tolist(),"left_right_Fn_redistribution":float(wp[2]-wp[8]),"reduced_layers":static_layers(M,h,H,bounds),"plant_static":pl,"classification":cls})
 # Dynamic probe: full finite-difference velocity captures, independently projected; diagnostic only.
 dynamic=[]
 for t in (210,211,212,213,217,218,219,220):
  q=allstates[f"tick_{t}"];N,_=oracle.reduction(q); vel=np.linalg.lstsq(N,cap["qvel"][t],rcond=None)[0]; qacc=(cap["qvel"][t+1]-cap["qvel"][t-1])/(2*float(cfg["dynamic_dt_s"]))
  # Evaluate bias at the reconstructed configuration with its tangent-consistent
  # velocity; the raw plant velocity contains compliant-closure components.
  oracle.forward(q,N@vel)
  # MuJoCo's dense mass matrix must be expanded before projection.
  full=np.zeros((oracle.model.nv,oracle.model.nv));mujoco.mj_fullM(oracle.model,full,oracle.data.qM); lhs=N.T@(full@qacc+oracle.data.qfrc_bias); tau=cap["physical_solution"][t][12:18]; rhs=actuator_map(oracle,N)@tau; truth=[]
  for s in range(2):
   b,_,g=wrench_generalized_map(oracle,patch,q,s); frame=rc(g,patch.n); center=np.asarray(g["contact_center"]); force=cap["truth_force"][t,s]; mw=cap["truth_moment_about_wheel"][t,s]; wc=cap["wheel_center"][t,s]; w=np.r_[frame.T@force,frame.T@(mw-np.cross(center-wc,force))]; rhs+=b@w;truth.append(w.tolist())
  e=lhs-rhs; rel=ma(e)/max(1.,ma(lhs));dynamic.append({"tick":t,"reduced_qvel":vel.tolist(),"full_qacc_fd":qacc.tolist(),"captured_tau":tau.tolist(),"captured_truth_wrench_C":truth,"row_residual":e.tolist(),"absolute_max":ma(e),"relative_max":rel,"pass":ma(e)<=cfg["dynamic_absolute_limit"] and rel<=cfg["dynamic_relative_limit"],"limit":"diagnostic validation-only"})
 failures=[r for r in rows if r["failure"]]
 gates={"required_witnesses":all(r["cone_removed_witness"]["feasible"] and np.all(np.isfinite(r["cone_removed_witness"]["tau"]+r["cone_removed_witness"]["wL"]+r["cone_removed_witness"]["wR"])) for r in rows),"cone_projections":all(p["l2_success"] and p["linf_success"] and np.isfinite(p["l2_normalized"]) and p["linf_normalized"] is not None for r in rows for p in r["nearest_cone_correction"]),"reduced_lp_residuals":all(r["reduced_layers"]["full_min_linf"]["feasible"] and r["reduced_layers"]["full_min_linf"]["epsilon"] is not None for r in rows),"plant_oracle_finite":all(r["plant_static"]["minimum_linf_residual"] is not None and r["plant_static"]["contact_count"]>0 and np.isfinite(r["plant_static"]["crosscheck_Nt_full"]) for r in rows),"reconstruction":all(np.isfinite(r["reconstruction"]["closure_residual_m"]) and np.isfinite(r["reconstruction"]["passive_condition_number"]) for r in rows),"dynamic_probes":all(x["pass"] for x in dynamic),"classification":all(r["classification"]=="A" for r in failures)}
 close=all(gates.values())
 summary={"schema_version":1,"phase":21,"profile":cfg["profile"],"fixed_contract":{"mu":1,"H_rows":len(H),"torque_bounds_nm":bounds.tolist(),"state":"12D passive reconstruction"},"classification_counts":{k:sum(r["classification"]==k for r in failures) for k in "ABCD"},"gates":gates,"dynamic_probe":dynamic,"DG21_01_close_recommended":close,"42d_candidate_authorized":close,"scope_limit":"Authorization is only for the next 42D candidate; it is not a QP, solver, Core, simulation, or hardware PASS."}
 dump(out/"summary.json",summary);dump(out/"cases.json",rows); script=Path(__file__).resolve(); configs=ci+cci+mci+coni; static_path=ROOT/cfg["authoritative_static"]; auth={"static":sha(static_path),"capture":sha(ROOT/cfg["capture"])}
 for name in ("summary.json","manifest.json"):
  candidate=static_path.parent/name
  if candidate.exists(): auth[name]=sha(candidate)
 dump(out/"manifest.json",{"schema_version":1,"created_at":datetime.now(timezone.utc).isoformat(),"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy_version,"mujoco":mujoco.__version__,"config_inputs":{str(x.relative_to(ROOT)):sha(x) for x in configs},"authoritative_inputs":auth,"validator":str(script.relative_to(ROOT)),"validator_sha256":sha(script),"outputs":{x:sha(out/x) for x in ("summary.json","cases.json")}});print(json.dumps(summary,indent=2)); return 0 if close else 1
if __name__=="__main__":
 try: sys.exit(main())
 except Exception as e: print("ERROR:",e,file=sys.stderr);sys.exit(2)
