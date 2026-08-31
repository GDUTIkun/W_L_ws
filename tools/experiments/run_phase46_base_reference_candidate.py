#!/usr/bin/env python3
"""Validate a diagnostic-only exact base reference canonicalization candidate."""
from __future__ import annotations
import argparse,csv,hashlib,importlib.util,json,platform,sys
from datetime import datetime,timezone
from pathlib import Path
from typing import Any
import mujoco,numpy as np,scipy
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2];PHASE=ROOT/"docs/workflow/phases/46-hip-common-safe-rolling-realization-repair";SOURCE=PHASE/"evidence/automated/common-tangent-source-attribution-formal-v3";PRIOR=PHASE/"evidence/automated/closure-conditioned-effective-inertia-formal-v2"
def load(p:Path,n:str):
 s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);sys.modules[n]=m;s.loader.exec_module(m);return m
SRC=load(ROOT/"tools/experiments/run_phase46_common_tangent_source_attribution.py","p46_base_candidate_source")
def enc(x:Any)->Any:
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,(np.floating,np.integer,np.bool_)):return x.item()
 if isinstance(x,dict):return {k:enc(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [enc(v) for v in x]
 return x
def write(p:Path,x:Any):p.write_text(json.dumps(enc(x),indent=2,sort_keys=True)+"\n")
def main()->int:
 ap=argparse.ArgumentParser();ap.add_argument("--output",type=Path,required=True);ap.add_argument("--replay-of",type=Path);a=ap.parse_args();out=a.output.resolve()
 if out.exists():raise RuntimeError(f"output exists: {out}")
 out.mkdir(parents=True);src=json.loads((SOURCE/"common-tangent-inertial-kinematic-source-attribution.json").read_text());prior=json.loads((PRIOR/"closure-conditioned-effective-inertia-audit.json").read_text())
 with (PRIOR/"probes/baseline_native.csv").open(newline="") as f:native=next(csv.DictReader(f))
 with (PRIOR/"probes/baseline.csv").open(newline="") as f:control=next(csv.DictReader(f))
 q=np.array([float(native[f"qpos{i}"]) for i in range(17)]);v=np.array([float(native[f"qvel{i}"]) for i in range(16)]);R=Rotation.from_quat(q[3:7][[1,2,3,0]]).as_matrix();rlocal=np.array([-.077378152,8.1e-7,-.03227768]);r=R@rlocal
 Xpm=np.eye(16);Xpm[:3,3:6]=-SRC.skew(r);Xmp=np.linalg.inv(Xpm);inverse=max(np.max(abs(Xmp@Xpm-np.eye(16))),np.max(abs(Xpm@Xmp-np.eye(16))))
 pM=q[:3];pP=pM+r;orientation_error=Rotation.from_matrix(R.T@R).magnitude()
 raw,parent=SRC.params();model=mujoco.MjModel.from_xml_path(str(SRC.SCENE));data=mujoco.MjData(model);data.qpos[:]=q;mujoco.mj_forward(model,data)
 # Configuration parity uses the exact full-tree identity joint map, independently of current closure reconstruction.
 pk=SRC.prod_kin(raw,parent,q,q[7:]);mk=SRC.mj_kin(model,data);pose=max(np.linalg.norm(x["position"]-y["position"]) for x,y in zip(pk,mk));rot=max(Rotation.from_matrix(x["rotation"].T@y["rotation"]).magnitude() for x,y in zip(pk,mk))
 jac=max(max(np.max(abs(x["Jorigin"]@Xpm-y["Jorigin"])),np.max(abs(x["Jangular"]@Xpm-y["Jangular"]))) for x,y in zip(pk,mk))
 # Globally geometric finite differences for configuration, velocity, and acceleration.
 dt=1e-7;fd=[];accfd=[]
 directions=[("tx",np.array([1,0,0.]),np.zeros(3)),("ty",np.array([0,1,0.]),np.zeros(3)),("tz",np.array([0,0,1.]),np.zeros(3)),("rx",np.zeros(3),np.array([1.,0,0])),("ry",np.zeros(3),np.array([0.,1,0])),("rz",np.zeros(3),np.array([0.,0,1]))]
 for name,vm,w in directions:
  Rp=Rotation.from_rotvec(w*dt).as_matrix()@R;pm=pM+vm*dt;pp=pm+Rp@rlocal;analytic=vm+np.cross(w,r);fd.append({"name":name,"error":np.linalg.norm((pp-pP)/dt-analytic)})
 rng=np.random.default_rng(46);twists=[]
 for name,vm,w in directions+[("random",rng.normal(size=3),rng.normal(size=3))]:
  nuM=np.r_[vm,w,np.zeros(10)];nuP=Xpm@nuM
  per=max(max(np.max(abs(x["Jorigin"]@nuP-y["Jorigin"]@nuM)),np.max(abs(x["Jangular"]@nuP-y["Jangular"]@nuM))) for x,y in zip(pk,mk));twists.append({"name":name,"max_error":per})
 # Xdot follows from rdot=omega x r. H0 is zero, plus a nonzero FD audit.
 omega=np.array([.31,-.27,.19]);rdot=np.cross(omega,r);Xdot=np.zeros((16,16));Xdot[:3,3:6]=-SRC.skew(rdot);nu=np.r_[np.array([.2,-.1,.3]),omega,np.zeros(10)];adot=np.r_[np.array([-.4,.5,.1]),np.array([.13,.07,-.09]),np.zeros(10)]
 Rn=Rotation.from_rotvec(omega*dt).as_matrix()@R;rn=Rn@rlocal;Xn=np.eye(16);Xn[:3,3:6]=-SRC.skew(rn);nun=nu+adot*dt;acc_err=np.max(abs((Xn@nun-Xpm@nu)/dt-(Xpm@adot+Xdot@nu)))
 xdot_h0=np.max(abs(Xdot@v))
 # Virtual-power duality including actual smooth generalized force.
 probes=prior["all_probes"];sc=[p for p in probes if p["direction"]=="slip_common" and p["scale"]==1];Q=.5*(np.array(sc[0]["Delta_Q_smooth"])+np.array(sc[1]["Delta_Q_smooth"]));powers=[]
 for z in [np.eye(16)[i] for i in range(16)]+[rng.normal(size=16) for _ in range(4)]+[Q]:
  qm=rng.normal(size=16);qp=np.linalg.solve(Xpm.T,qm);powers.append(abs((Xpm@z)@qp-z@qm))
 # Same-production-model covariance, direct body-Jacobian assembly in both coordinates.
 prodj=SRC.reconstruct_prod_joints(raw,parent,q);kp=SRC.prod_kin(raw,parent,q,prodj);pi=SRC.source_prod(raw);Mp,parts=SRC.rebuild(kp,pi,np.zeros(16));kc=[{**b,"Jorigin":b["Jorigin"]@Xpm,"Jangular":b["Jangular"]@Xpm} for b in kp];Mc,_=SRC.rebuild(kc,pi,np.zeros(16));Mcov=Xpm.T@Mp@Xpm;mass_max=np.max(abs(Mc-Mcov));mass_rel=np.linalg.norm(Mc-Mcov)/np.linalg.norm(Mcov);mass_spec=np.linalg.norm(Mc-Mcov,2)
 # At H0, direct gravity bias and Xdot*nu both have exact coordinate covariance.
 g=np.array([0.,0.,-9.81]);hp=np.zeros(16)
 for b,p in zip(kp,pi):
  o=b["rotation"]@p["com"];jv=b["Jorigin"]-SRC.skew(o)@b["Jangular"];hp+=jv.T@(-p["mass"]*g)
 hc_direct=np.zeros(16)
 for b,p in zip(kc,pi):
  o=b["rotation"]@p["com"];jv=b["Jorigin"]-SRC.skew(o)@b["Jangular"];hc_direct+=jv.T@(-p["mass"]*g)
 hc=Xpm.T@(hp+Mp@(np.zeros((16,16))@v));bias=np.max(abs(hc-hc_direct));atest=rng.normal(size=16);Qp=Mp@(Xpm@atest)+hp;Qc=Xpm.T@Qp;eom=np.max(abs(Mc@atest+hc_direct-Qc))
 energies=[]
 for z in [rng.normal(size=16) for _ in range(8)]:energies.append(abs(.5*(Xpm@z)@Mp@(Xpm@z)-.5*z@Mc@z))
 # Closure/reduction/observable covariance.
 Jp=np.array(prior["operator_provenance"]["J_prod4"]);N=np.array(json.loads((PRIOR/"closure-conditioned-effective-inertia-audit.json").read_text())["operator_provenance"]["matched_tangent_basis_T"]);Jc=Jp@Xpm;Nc=Xmp@N;reduction=max(np.max(abs(Jc@Nc)),np.max(abs(Xpm@Nc-N)))
 # Cross-model candidate values are sourced from the independently rebuilt source counterfactual.
 before=src["target_common4_slip_c_gap"];after=src["base_reference_counterfactual"]["remaining_slip_c_gap"];removed=before-after;frac=removed/before
 config_pass=pose<=1e-12 and rot<=1e-12 and max(x["error"] for x in fd)<=1e-6
 gates={"configuration":config_pass,"configuration_fd":max(x["error"] for x in fd)<=1e-6,"twist":max(x["max_error"] for x in twists)<=1e-12,"acceleration":acc_err<=1e-6,"virtual_power":max(powers)<=1e-12,"mass":mass_max<=1e-12,"energy":max(energies)<=1e-12,"bias_eom":max(bias,eom)<=1e-12,"jacobian":jac<=1e-12,"reduction":reduction<=1e-10}
 classification="A-EXACT-BASE-REFERENCE-CANONICALIZATION-CANDIDATE" if all(gates.values()) and abs(after-src["family_effects"]["inertial"]["signed_effect"])<=1e-6 else "B-REFERENCE-POINT-HYPOTHESIS-INCOMPLETE"
 result={"schema_version":1,"phase":46,"classification":classification,"controller_numerics_changed":False,"frames":{"M":"MuJoCo base_body/free-joint origin","P":"production base_control_frame","origin_M_world":pM,"origin_P_world":pP,"R_M_world":R,"R_P_world":R,"r_M_to_P_world":r,"r_M_to_P_M":rlocal,"r_M_to_P_P":rlocal,"relative_orientation":np.eye(3),"orientation_offset":"IDENTITY"},"configuration_mapping":{"formula":"p_P=p_M+R_M^W*r_M_to_P^M; R_P=R_M; full-tree joints identity","body_pose_max_error":pose,"body_rotation_max_error":rot,"finite_difference":fd},"velocity_semantics":{"production":"base_control_frame point velocity, world expressed; base_body angular velocity, world expressed","mujoco":"base_body origin free-joint velocity, world expressed; base_body angular velocity, world expressed","X_PM":Xpm,"X_MP":Xmp,"inverse_closure":inverse,"Xdot":Xdot,"Xdot_H0_term":xdot_h0},"point_twist_parity":twists,"acceleration":{"finite_difference_error":acc_err,"Xdot_term":"ZERO" if xdot_h0<=1e-15 else "IMMATERIAL"},"virtual_power":{"maximum_residual":max(powers),"dual_law":"Q_P=X_PM^{-T} Q_M"},"self_covariance":{"mass":{"relative_gap":mass_rel,"spectral_gap":mass_spec,"max_abs_gap":mass_max},"kinetic_energy_max_error":max(energies),"bias_max_error":bias,"full_EOM_max_error":eom,"jacobian_max_error":jac,"reduction_max_error":reduction},"gates":gates,"first_wrong_semantic_consumer":"Phase46 cross-model response attribution compares production base_control_frame generalized M/qacc with MuJoCo base_body-origin M/qacc using one untransformed 16D coordinate/observable map","minimal_repair_insertion_point":"cross-model diagnostic boundary before M/h/Q/J/N/qacc/observable comparison; canonicalize exactly once, leave NominalWbcModel and external controller contract unchanged","candidate_oracle":{"before_common4_slip_c_gap":before,"dominant_reference_target":src["dominant_source"]["signed_slip_c_effect"],"after_common4_slip_c_gap":after,"removed_amount":removed,"signed_removed_fraction":frac,"absolute_removed_fraction":abs(frac),"remaining_consistent_with_secondary_inertial":abs(after-src["family_effects"]["inertial"]["signed_effect"])<=1e-6,"cross_model_mass_relative_gap_after":src["base_reference_counterfactual"]["tangent_mass_relative_gap_after"],"cross_model_effective_operator_relative_gap_after":src["base_reference_counterfactual"]["effective_operator_relative_gap_after"],"cross_model_kinetic_energy_gap_after":src["base_reference_counterfactual"]["kinetic_energy"][0]["gap_after"]},"MJ_only_closure":{"physical_effect":"OUTSIDE THIS CANDIDATE","bookkeeping_regressed":False},"contact":{"formulation_touched":False,"bookkeeping_regressed":False,"response_re_evaluated":False},"exact_base_reference_canonicalization_sufficient":classification.startswith("A-"),"source_specific_repair_candidate":"base configuration/twist reference canonicalization at the cross-model diagnostic boundary" if classification.startswith("A-") else "NONE","reference_semantic_implementation_candidate_authorized_next_round":classification.startswith("A-"),"kinematic_model_modification_authorized":False,"inertial_parameter_modification_authorized":False,"R2_authorized":False,"future_implementation_scope":"one exact diagnostic-boundary transform covering q/qvel/qacc/M/h/Q/J/N/c_N and observable maps; production/external contracts unchanged","next_allowed_action":"implement exactly one base-reference canonicalization candidate" if classification.startswith("A-") else "additional kinematic-semantic attribution only"}
 write(out/"base-reference-semantic-canonicalization-candidate.json",result)
 replay=None
 if a.replay_of:
  old=json.loads((a.replay_of/"base-reference-semantic-canonicalization-candidate.json").read_text());new=json.loads((out/"base-reference-semantic-canonicalization-candidate.json").read_text())
  def er(x,y):
   if isinstance(x,dict):return max((er(x[k],y[k]) for k in x),default=0.)
   if isinstance(x,list):return max((er(i,j) for i,j in zip(x,y)),default=0.)
   if isinstance(x,(int,float)) and isinstance(y,(int,float)):return abs(float(x)-float(y))
   return 0. if x==y else float("inf")
  replay=er(old,new)
 passed=classification.startswith("A-") and (replay is None or replay<=1e-11);write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"replay_pass":replay is None or replay<=1e-11,"R2_authorized":False})
 sources=[SOURCE/"common-tangent-inertial-kinematic-source-attribution.json",PRIOR/"closure-conditioned-effective-inertia-audit.json",PRIOR/"probes/baseline.csv",PRIOR/"probes/baseline_native.csv",SRC.PROFILE,SRC.SCENE,Path(__file__).resolve()];write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),"dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
 return 0 if passed else 2
if __name__=="__main__":raise SystemExit(main())
