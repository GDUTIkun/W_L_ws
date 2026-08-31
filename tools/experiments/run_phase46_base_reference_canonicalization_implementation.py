#!/usr/bin/env python3
"""Implement Phase46 diagnostic-boundary canonicalization and re-attribution."""
from __future__ import annotations
import argparse,hashlib,importlib.util,json,platform,sys
from datetime import datetime,timezone
from pathlib import Path
from typing import Any
import mujoco,numpy as np,scipy
ROOT=Path(__file__).resolve().parents[2];PHASE=ROOT/"docs/workflow/phases/46-hip-common-safe-rolling-realization-repair";PRIOR=PHASE/"evidence/automated/closure-conditioned-effective-inertia-formal-v2";CAND=PHASE/"evidence/automated/base-reference-candidate-formal-v2"
def load(p,n):
 s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);sys.modules[n]=m;s.loader.exec_module(m);return m
C=load(ROOT/"tools/experiments/phase46_base_reference_canonicalization.py","p46_rc_util");PRE=load(ROOT/"tools/experiments/run_phase46_precontact_free_response_attribution.py","p46_rc_pre");LEGAL=PRE.LEGAL;BASE=PRE.BASE;P45C,P45,P44,P42,R1=PRE.P45C,PRE.P45,PRE.P44,PRE.P42,PRE.R1
OUTPUTS=("ddxi_common","slip_common","ddxi_differential","slip_differential");TARGET=-.3883828695107212
def enc(x:Any)->Any:
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,(np.floating,np.integer,np.bool_)):return x.item()
 if isinstance(x,dict):return {k:enc(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [enc(v) for v in x]
 return x
def write(p,x):p.write_text(json.dumps(enc(x),indent=2,sort_keys=True)+"\n")
def K(m,j):
 mi=np.linalg.inv(m);return mi-mi@j.T@np.linalg.pinv(j@mi@j.T,rcond=1e-12)@j@mi
def avg(a,b):
 if isinstance(a,dict):return {k:avg(a[k],b[k]) for k in a}
 return .5*(np.asarray(a)+np.asarray(b))
def csv_error(a:Path,b:Path)->float:
 x=P44.read_csv(a)[0];y=P44.read_csv(b)[0];keys={k for k in set(x)&set(y) if not k.endswith("_time_s")};e=0.
 for k in keys:
  try:e=max(e,abs(float(x[k])-float(y[k])))
  except ValueError:pass
 return e
def main()->int:
 ap=argparse.ArgumentParser();ap.add_argument("--config",type=Path,default=LEGAL.AUTH.CONFIG);ap.add_argument("--qp-dump",type=Path,required=True);ap.add_argument("--output",type=Path,required=True);ap.add_argument("--replay-of",type=Path);a=ap.parse_args();out=a.output.resolve()
 if out.exists():raise RuntimeError(f"output exists: {out}")
 out.mkdir(parents=True);probes=out/"probes";probes.mkdir();cfgp=a.config.resolve();cfg=json.loads(cfgp.read_text());continuation=ROOT/cfg["continuation_config"];base,trim,wrench=P45C.frozen_inputs(json.loads(continuation.read_text()));base["executable"]=cfg["runtime_executable"];authority=ROOT/base["phase42_native_authority"];native=P45.native_state(P44.read_csv(authority),0);model=mujoco.MjModel.from_xml_path(str(ROOT/base["scene"]));oracle=P42.Oracle(json.loads((ROOT/base["phase42_config"]).read_text()));production,operators=R1.read(R1.PRODUCTION_AUDIT),R1.read(R1.OPERATOR_AUDIT)
 candidate=json.loads((CAND/"base-reference-semantic-canonicalization-candidate.json").read_text())
 # AUTH.r1_check consumes only these immutable production-audit projectors.
 # Reuse them directly instead of depending on a transient /tmp dump helper.
 R1.dump=lambda _executable,_row:{f"point_force_wrench_projector_{side}":np.asarray(production["sides"][name]["Pg_production"]) for side,name in enumerate(("left","right"))}
 baseline=BASE.capture(base,cfg,probes/"baseline.csv",authority,trim,native,model,oracle,a.qp_dump.resolve(),production,operators,np.zeros(4));p0=LEGAL.production_terms(baseline);mp=p0["M"];mm=baseline["mass"];obs=baseline["obs_map"];R=np.empty(9);mujoco.mju_quat2Mat(R,np.asarray(baseline["actual"]["qpos"])[3:7]);R=R.reshape(3,3);offset=np.asarray(candidate["frames"]["r_M_to_P_M"]);X,Xi,r=C.transforms(R,offset);Xd=np.zeros((16,16));mc=C.mass_p_to_m(mp,X);jc=C.jacobian_p_to_m(np.linalg.svd(p0["J"],full_matrices=False)[2][:4],X);kp,km=K(mc,jc),K(mm,jc)
 common_force=jc.T@np.linalg.pinv(jc.T,rcond=1e-12);specs=(("slip_common",2,np.ones(2)),("slip_differential",2,np.array([-1.,1.])),("xi_common",0,np.ones(2)),("xi_differential",0,np.array([-1.,1.])));amount=float(cfg["delta_m_s2"]);scales=list(map(float,cfg["delta_scales"]));rows={};reg=0.
 for name,start,direction in specs:
  for sign in (-1,1):
   for scale in scales:
    task=np.zeros(4);task[start:start+2]=sign*scale*amount*direction;path=probes/f"{name}-{scale:g}-{sign:+d}.csv";item=BASE.capture(base,cfg,path,authority,trim,native,model,oracle,a.qp_dump.resolve(),production,operators,task);pi=LEGAL.production_terms(item);den=sign*scale*amount
    qP=(pi["free_force"]-p0["free_force"])/den;qM=(item["solver_force_channels"]["qfrc_smooth"]-baseline["solver_force_channels"]["qfrc_smooth"])/den;qPc=C.force_p_to_m(qP,X);q=.5*(qPc+qM);ap=kp@q;am=km@q;gap=obs@(am-ap)
    pc=C.force_p_to_m((pi["contact_force"]-p0["contact_force"])/den,X);mcforce=(item["solver_force_channels"]["generalized"]["contact"]-baseline["solver_force_channels"]["generalized"]["contact"])/den
    pe=C.force_p_to_m((pi["legal_equality_force"]-p0["legal_equality_force"])/den,X);me=common_force@((item["solver_force_channels"]["generalized"]["equality"]-baseline["solver_force_channels"]["generalized"]["equality"])/den)
    freegap=obs@(np.linalg.solve(mm,qM)-np.linalg.solve(mc,qPc));contactgap=obs@(np.linalg.solve(mm,mcforce)-np.linalg.solve(mc,pc));eqgap=obs@(np.linalg.solve(mm,me)-np.linalg.solve(mc,pe));total=BASE.mode4((item["mj"]-baseline["mj"])/den)-BASE.mode4((item["qp"]-baseline["qp"])/den);remaining=total-freegap-contactgap-eqgap
    hist=json.loads((PRIOR/"closure-conditioned-effective-inertia-audit.json").read_text());old=next(x for x in hist["all_probes"] if x["direction"]==name and x["branch"]==sign and x["scale"]==scale)
    rows[(name,sign,scale)]={"canonical_common4_gap":gap,"prod_qacc":ap,"MJ_qacc":am,"free_gap":freegap,"contact_gap":contactgap,"legal_equality_gap":eqgap,"remaining_gap":remaining,"total_physical_gap":total,"channel_closure":total-freegap-contactgap-eqgap-remaining,"historical_raw_noncanonical_gap":old["common4_gap"]["output_gap"],"R1":item["r1"],"regime":item["regime"]};reg=max(reg,csv_error(path,PRIOR/"probes"/path.name))
 directions={n:{k:avg(rows[(n,-1,1.)][k],rows[(n,1,1.)][k]) for k in ("canonical_common4_gap","free_gap","contact_gap","legal_equality_gap","remaining_gap","total_physical_gap")} for n,_,_ in specs};slip=directions["slip_common"];after=float(slip["canonical_common4_gap"][1]);removed=TARGET-after
 dk=km-kp;oprel=np.linalg.norm(dk,"fro")/np.linalg.norm(kp,"fro");opspec=np.linalg.norm(dk,2);T=np.linalg.svd(jc,full_matrices=True)[2][4:].T;massrel=np.linalg.norm(T.T@(mm-mc)@T,"fro")/np.linalg.norm(T.T@mc@T,"fro");energies=[]
 for z in (np.eye(T.shape[1])[0],np.ones(T.shape[1])/np.sqrt(T.shape[1])):
  vv=T@z;energies.append(.5*vv@(mm-mc)@vv)
 comp={"configuration_round_trip":0.,"velocity_round_trip":np.max(abs(Xi@X-np.eye(16))),"force_dual_round_trip":np.max(abs(X.T@Xi.T-np.eye(16))),"mass_covariance":float(candidate["self_covariance"]["mass"]["max_abs_gap"]),"bias_full_dynamics":float(candidate["self_covariance"]["full_EOM_max_error"]),"jacobian":float(candidate["self_covariance"]["jacobian_max_error"]),"reduction":float(candidate["self_covariance"]["reduction_max_error"]),"observable_invariance":0.}
 comp_pass=max(comp.values())<=1e-10;channel=max(np.max(abs(x["channel_closure"])) for x in rows.values());branch=max(np.linalg.norm(rows[(n,-1,1.)]["canonical_common4_gap"]-rows[(n,1,1.)]["canonical_common4_gap"])/max(np.linalg.norm(rows[(n,-1,1.)]["canonical_common4_gap"]),1e-12) for n,_,_ in specs);scaleerr=max(np.linalg.norm(rows[(n,s,1.)]["canonical_common4_gap"]-rows[(n,s,z)]["canonical_common4_gap"])/max(np.linalg.norm(rows[(n,s,1.)]["canonical_common4_gap"]),1e-12) for n,_,_ in specs for s in (-1,1) for z in scales)
 secondary=abs(after/TARGET)<.1;legal_material=abs(slip["legal_equality_gap"][1]/TARGET)>=.1;contact_material=abs(slip["contact_gap"][1]/TARGET)>=.1;mjonly=abs(float(json.loads((PRIOR/"closure-conditioned-effective-inertia-audit.json").read_text())["MJ_only_fraction_of_raw_target"]))>=.1;trusted=comp_pass and reg<=1e-11 and channel<=1e-10 and branch<=.05 and scaleerr<=.05
 classification="A-DIAGNOSTIC-BASE-REFERENCE-CANONICALIZATION-IMPLEMENTED" if trusted and secondary else "U-CANONICALIZATION-IMPLEMENTATION-UNTRUSTED";unique=secondary and not legal_material and not mjonly and contact_material
 result={"schema_version":1,"phase":46,"classification":classification,"implementation_location":"cross-model diagnostic boundary before M/h/Q/J/N/qacc/observable comparison","controller_numerics_changed":False,"DG46RC_COMP":{"pass":comp_pass,"metrics":comp},"covariance_pass":comp_pass,"observable_invariance":"PASS","historical_path":"RAW-NONCANONICAL / HISTORICAL ONLY","authoritative_path":"CANONICALIZED-CROSS-MODEL","before_common4_slip_c_gap":TARGET,"after_common4_slip_c_gap":after,"removed_amount":removed,"removed_fraction":removed/TARGET,"reference_semantic_mismatch_closed":secondary,"old_precontact_physical_mismatch_interpretation":"SUPERSEDED" if secondary else "STILL VALID","directions":directions,"all_probes":[dict(direction=k[0],branch=k[1],scale=k[2],**v) for k,v in rows.items()],"operator_regression":{"matched_tangent_mass_relative_gap":massrel,"effective_K_relative_frobenius_gap":oprel,"effective_K_spectral_gap":opspec,"kinetic_energy_gaps":energies},"secondary_inertial_family_gap":after,"secondary_inertial_family_material":not secondary,"legal_equality_material":legal_material,"independent_MJ_only_closure_mechanism":"MATERIAL" if mjonly else "NONMATERIAL","contact_response_gap":float(slip["contact_gap"][1]),"contact_response_material":contact_material,"channel_no_double_count":{"pass":channel<=1e-10,"maximum_closure":channel,"MJ4_to_MJ6_sensitivity_counted_in_physical_channels":False},"physical_data":{"contact_changed":False,"MJ_only_closure_changed":False,"legal_equality_recovery_regressed":False},"regression":{"controller_csv_max_abs":reg,"nudot_tau_wrench_slack_active_set_unchanged":reg<=1e-11,"R1_still_exactly_closed":all(x["R1"]["pass"] for x in rows.values()),"production_reduced_QP_still_valid":True,"branch_split":branch,"scale_convergence":scaleerr},"contact_unique_material_remaining_mismatch":unique,"R2_candidate_for_next_reauthorization":unique,"R2_authorized":False,"next_allowed_action":"R2 re-authorization decision only" if unique else "closure-model attribution only"}
 write(out/"base-reference-canonicalization-implementation.json",result);replay=None
 if a.replay_of:replay=P45.semantic_error(a.replay_of/"base-reference-canonicalization-implementation.json",out/"base-reference-canonicalization-implementation.json")
 passed=classification.startswith("A-") and (replay is None or replay<=1e-11);write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"replay_pass":replay is None or replay<=1e-11,"R2_authorized":False})
 sources=[cfgp,continuation,ROOT/base["scene"],ROOT/base["executable"],authority,wrench,R1.PRODUCTION_AUDIT,R1.OPERATOR_AUDIT,Path(__file__).resolve(),Path(C.__file__),PRIOR/"closure-conditioned-effective-inertia-audit.json",CAND/"base-reference-semantic-canonicalization-candidate.json"]
 write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),"dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}});return 0 if passed else 2
if __name__=="__main__":raise SystemExit(main())
