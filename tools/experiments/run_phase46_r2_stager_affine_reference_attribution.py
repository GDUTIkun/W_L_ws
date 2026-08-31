#!/usr/bin/env python3
"""Phase46 Stage-R affine reaction-map reference attribution; diagnostic only."""

from __future__ import annotations

import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

import run_phase46_r2_contact_response_reauthorization as R2
import run_phase46_r2_contact_reaction_commuting_diagram_attribution as COMM

ROOT=Path(__file__).resolve().parents[2]
PHASE=ROOT/"docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"


def enc(value:Any)->Any:
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,(np.floating,np.integer,np.bool_)):return value.item()
    if isinstance(value,dict):return {k:enc(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [enc(v) for v in value]
    return value


def write(path:Path,value:Any)->None:path.write_text(json.dumps(enc(value),indent=2,sort_keys=True)+"\n",encoding="utf-8")
def maxabs(value:np.ndarray)->float:return float(np.max(np.abs(value)))


def main()->int:
    parser=argparse.ArgumentParser();parser.add_argument("--config",type=Path,default=R2.LEGAL.AUTH.CONFIG)
    parser.add_argument("--qp-dump",type=Path,required=True);parser.add_argument("--output",type=Path,required=True);parser.add_argument("--replay-of",type=Path)
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True);probes=out/"probes";probes.mkdir()
    cfgp,cfg,continuation,base,trim,wrench,authority,native,model,oracle,production,operators=COMM.setup_capture(args)
    amount=float(cfg["delta_m_s2"]);scales=list(map(float,cfg["delta_scales"]));specs=[("baseline",np.zeros(4),0.,0.)]
    for name,start,direction,these in (("slip-common",2,np.ones(2),scales),("xi-common",0,np.ones(2),[1.]),("slip-differential",2,np.array([-1.,1.]),[1.])):
        for sign in (-1,1):
            for scale in these:
                delta=np.zeros(4);delta[start:start+2]=sign*scale*amount*direction;specs.append((f"{name}-{scale:g}-{sign:+d}",delta,float(sign),float(scale)))
    captures={};dumps={}
    for label,delta,_,_ in specs:
        path=probes/f"{label}.csv";captures[label]=R2.BASE.capture(base,cfg,path,authority,trim,native,model,oracle,args.qp_dump.resolve(),production,operators,delta);dumps[label]=R2.R1.dump(args.qp_dump.resolve(),path)
    baseline=captures["baseline"];maps=COMM.fresh_maps(model,baseline)

    # Producer: fixed-H0 MuJoCo active-set Schur map with affine origin tau=0.
    oracle.evaluate(native,[],ctrl_override=-baseline["tau"]);data=oracle.data
    j=np.asarray(data.efc_J).reshape(data.nefc,model.nv).copy();d=np.asarray(data.efc_D).copy();aref=np.asarray(data.efc_aref).copy();types=np.asarray(data.efc_type,dtype=int)
    eq=np.flatnonzero(types==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY));contact=np.flatnonzero(types==int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL));physical=np.concatenate((eq,contact))
    b=np.zeros((model.nv,6));saved=np.asarray(data.ctrl).copy()
    for col in range(6):data.ctrl[:]=0.;data.ctrl[col]=-1.;mujoco.mj_forward(model,data);b[:,col]=data.qfrc_actuator
    data.ctrl[:]=saved;mujoco.mj_forward(model,data)
    mass=baseline["mass"];smooth=np.asarray(data.qfrc_smooth).copy();smooth_known=smooth-b@baseline["tau"]
    system=mass+j[physical].T@(d[physical,None]*j[physical]);inverse=np.linalg.inv(system)
    q0=inverse@(smooth_known+j[physical].T@(d[physical]*aref[physical]));qt=inverse@b
    fc0=d[contact]*(aref[contact]-j[contact]@q0);fct=-(d[contact,None]*j[contact])@qt
    qc0_m=j[contact].T@fc0;qct_m=j[contact].T@fct
    candidate=json.loads((PHASE/"evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json").read_text())
    rot=np.empty(9);mujoco.mju_quat2Mat(rot,np.asarray(baseline["actual"]["qpos"])[3:7]);rot=rot.reshape(3,3)
    x_pm,x_mp,_=R2.CANON.transforms(rot,np.asarray(candidate["frames"]["r_M_to_P_M"]))
    tq=x_mp.T;qc0_p=tq@qc0_m;qct_p=tq@qct_m;aw_m=maps["Aw"];aw_p=tq@aw_m
    covariance={"TQ":tq,"Qc0_residual":qc0_p-tq@qc0_m,"Qct_residual":qct_p-tq@qct_m,
                "Qct_column_max_abs":[maxabs(qct_p[:,i]-tq@qct_m[:,i]) for i in range(6)]}
    historical_offset=qc0_m-qc0_p;historical_slope=qct_m@baseline["tau"]-qct_p@baseline["tau"]
    historical_total=historical_offset+historical_slope

    # Consumer correction: transform Aw*W to P at the diagnostic equality boundary only.
    corrected={}
    for label,_,_,_ in specs:
        dump=dumps[label];h=R2.matrix(dump,"h");g=R2.matrix(dump,"g").reshape(-1);a=R2.matrix(dump,"a")
        lower=R2.matrix(dump,"lower").reshape(-1);upper=R2.matrix(dump,"upper").reshape(-1);vs=R2.matrix(dump,"variable_scale").reshape(-1)
        extra=np.zeros((16,42));extra[:,12:18]=-qct_p*vs[None,12:18];extra[:,18:30]=aw_p*vs[None,18:30]
        solved=R2.active_set_qp(h,g,a,lower,upper,extra,qc0_p);physical_solution=vs*solved["x"]
        tau=physical_solution[12:18];w=physical_solution[18:30].copy()
        for side,name in enumerate(("left","right")):w[6*side:6*side+6]=np.asarray(production["sides"][name]["Pg_production"])@w[6*side:6*side+6]
        qa=q0+qt@tau;map_residual=aw_p@w-(qc0_p+qct_p@tau)
        corrected[label]={"tau":tau,"wrench":w,"qacc":qa,"outputs":baseline["obs_map"]@qa,
            "map_residual":map_residual,"R1_residual":max(maxabs((np.eye(6)-np.asarray(production["sides"][name]["Pg_production"]))@w[6*i:6*i+6]) for i,name in enumerate(("left","right"))),
            **{k:v for k,v in solved.items() if k!="x"}}

    def gain(label:str,sign:int,scale:float)->np.ndarray:
        return (np.asarray(corrected[f"{label}-{scale:g}-{sign:+d}"]["outputs"])-np.asarray(corrected["baseline"]["outputs"]))/(sign*scale*amount)
    branch=0.;scale_error=0.
    for name,_,_,these in (("slip-common",2,np.ones(2),scales),("xi-common",0,np.ones(2),[1.]),("slip-differential",2,np.array([-1.,1.]),[1.])):
        minus=gain(name,-1,1.);plus=gain(name,1,1.);center=.5*(minus+plus)
        branch=max(branch,float(np.linalg.norm(minus-plus)/max(np.linalg.norm(center),1e-12)))
        for sign in (-1,1):
            ref=gain(name,sign,1.)
            for scale in these:scale_error=max(scale_error,float(np.linalg.norm(gain(name,sign,scale)-ref)/max(np.linalg.norm(ref),1e-12)))
    h0=corrected["baseline"]
    h0_pass=h0["maximum_violation"]<=2e-7 and h0["kkt_residual"]<=1e-7 and h0["R1_residual"]<=1e-8 and maxabs(h0["map_residual"])<=1e-8 and abs(h0["outputs"][0])<=.05
    directional_pass=h0_pass and branch<=.05 and scale_error<=.05 and all(row["maximum_violation"]<=2e-7 for row in corrected.values())

    producer_consumer={"Qc0_Qct_builder":"run_phase46_r2_contact_response_reauthorization.py::main fixed-active-set diagnostic Schur block",
      "Qc0_Qct_consumers":["run_phase46_r2_contact_response_reauthorization.py::main Stage-R extra equality","Phase46 attribution scripts importing/rebuilding the diagnostic relation"],
      "diagnostic_only":True,"production_equivalent_consumer":False,
      "production_evidence":"WeightedWbcProblem assembles reduced dynamics with independent aggregate wrench plus soft tasks; no Qc0/Qct or constitutive affine reaction constraint exists in ros_ws/src"}
    classification="A-DIAGNOSTIC-STAGER-REFERENCE-MIX-CLOSED" if h0_pass and directional_pass else "C-AFFINE-REFERENCE-FIX-INCOMPLETE"
    result={"schema_version":1,"phase":46,"classification":classification,"controller_numerics_changed":False,
      "producer_consumer":producer_consumer,"frames":{"Qc0_source":"M","Qct_source":"M","AwW_source":"M","historical_StageR_target":"P","corrected_comparison":"P"},
      "affine_origin":{"kind":"tau=0","Qc0_meaning":"fixed-H0 state/active-set contact generalized reaction at zero actuator torque with qfrc_smooth-known held fixed",
        "Qct_meaning":"d Qcontact / d tau at the frozen H0 state and active set","formula":"Qc(tau)=Qc0+Qct*tau; not H0-centered"},
      "historical":{"offset_frame_contribution":historical_offset,"offset_max_abs":maxabs(historical_offset),"slope_frame_contribution":historical_slope,"slope_max_abs":maxabs(historical_slope),
        "total":historical_total,"total_max_abs":maxabs(historical_total),"reproduced_4p836644":abs(maxabs(historical_total)-4.836644038376806)<=1e-10},
      "force_dual_covariance":{**covariance,"pass":max(maxabs(covariance["Qc0_residual"]),maxabs(covariance["Qct_residual"]))<=1e-12,
        "authoritative_law":"Q_P = X_MP^T Q_M; generalized-force axes only"},
      "wrong_edge":{"first_wrong_producer_consumer":"diagnostic Stage-R extra equality assembly in run_phase46_r2_contact_response_reauthorization.py",
        "first_mixed_reference_edge":"Aw_full(M generalized force) * W_prod compared with qc0p + qctp*tau (P generalized force)",
        "missing_transform":"left-multiply diagnostic Aw_full by X_MP^T, or equivalently keep both sides in M; applied once after wrench-to-generalized-force mapping"},
      "corrected_diagnostic":{"correction":"Aw_P = X_MP^T Aw_M; retain Qc0_P=X_MP^T Qc0_M and Qct_P=X_MP^T Qct_M",
        "H0_map_residual":maxabs(h0["map_residual"]),"H0":h0,"H0_pass":h0_pass,"all_probes":corrected,"branch_split":branch,"scale_convergence":scale_error,
        "branch_scale_pass":directional_pass,"xi_common_pass":directional_pass,"slip_differential_pass":directional_pass},
      "production_relevance":{"relation":"DIAGNOSTIC-ONLY","production_consumes_equivalent_affine_law":False,"R2_candidate_next_reauthorization":False},
      "aggregate_representation_change_required":False,"R2_authorized":False,"R2_implemented":False,
      "next_allowed_action":"production contact-response integration attribution" if classification.startswith("A-") else "additional Stage-R attribution only"}
    decision=out/"r2-stager-affine-reference-attribution.json";write(decision,result)
    replay=None if args.replay_of is None else R2.P45.semantic_error(args.replay_of/decision.name,decision)
    finite=all(np.all(np.isfinite(v)) for v in (qc0_m,qct_m,tq,historical_total,h0["qacc"]))
    passed=classification!="U-UNTRUSTED" and finite and (replay is None or replay<=1e-11)
    write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"nonfinite":not finite,"controller_numerics_changed":False})
    sources=[cfgp,continuation,ROOT/base["scene"],ROOT/base["executable"],authority,wrench,args.qp_dump.resolve(),
      ROOT/"ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",Path(__file__).resolve(),Path(R2.__file__).resolve(),Path(COMM.__file__).resolve()]
    write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),
      "dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if passed else 2


if __name__=="__main__":raise SystemExit(main())
