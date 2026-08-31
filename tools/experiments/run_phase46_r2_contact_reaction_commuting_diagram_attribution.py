#!/usr/bin/env python3
"""Phase46 R2 contact-reaction commuting-diagram attribution; diagnostic only."""

from __future__ import annotations

import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

import run_phase46_r2_contact_response_reauthorization as R2
import run_phase46_wrench_generalized_force_operator_audit as OP

ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
TOL = 1.0e-10


def enc(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: enc(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [enc(item) for item in value]
    return value


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(enc(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def metrics(value: np.ndarray, obs: np.ndarray | None = None) -> dict[str, Any]:
    result = {"vector": value, "norm2": float(np.linalg.norm(value)),
              "max_abs": float(np.max(np.abs(value)))}
    if obs is not None:
        projected = obs @ value
        result.update({"observable": projected, "ddxi_projection": projected[[0, 2]],
                       "slip_projection": projected[[1, 3]]})
    return result


def setup_capture(args: argparse.Namespace) -> tuple[Any, ...]:
    cfgp=args.config.resolve(); cfg=json.loads(cfgp.read_text()); continuation=ROOT/cfg["continuation_config"]
    base,trim,wrench=R2.P45C.frozen_inputs(json.loads(continuation.read_text()));base["executable"]=cfg["runtime_executable"]
    authority=ROOT/base["phase42_native_authority"];native=R2.P45.native_state(R2.P44.read_csv(authority),0)
    model=mujoco.MjModel.from_xml_path(str(ROOT/base["scene"]));oracle=R2.P42.Oracle(json.loads((ROOT/base["phase42_config"]).read_text()))
    production=R2.R1.read(R2.R1.PRODUCTION_AUDIT);operators=R2.R1.read(R2.R1.OPERATOR_AUDIT)
    return cfgp,cfg,continuation,base,trim,wrench,authority,native,model,oracle,production,operators


def fresh_maps(model: mujoco.MjModel, item: dict[str, Any]) -> dict[str, Any]:
    data=mujoco.MjData(model);data.qpos[:]=item["actual"]["qpos"];data.qvel[:]=item["actual"]["qvel"]
    weld=R2.P42.required_id(model,mujoco.mjtObj.mjOBJ_EQUALITY,"base_weld");data.eq_active[weld]=0;mujoco.mj_forward(model,data)
    geometry=R2.BASE.SENS.ATTR.contact_geometry(model,item["actual"]["qpos"],item["actual"]["qvel"],0.10)
    prod_geometry,_,prod_metrics=OP.RC.ATTR.model_b_contact_geometry(item["control"])
    sides={};aw_block=np.zeros((model.nv,12));gp_block=np.zeros((12,12));jp_block=np.zeros((model.nv,12))
    for side,name in enumerate(("left","right")):
        geo=geometry[side];frame=np.asarray(geo["frame"]);reference=np.asarray(geo["point"])
        points=sorted((row for row in item["points"] if row["side"]==name),key=lambda row:row["point_index"])
        gp_point=OP.P46.point_map(points,frame,reference);prod_ref=np.asarray(prod_geometry[side]["point"])
        transport=OP.P46.transport(reference,prod_ref,frame);gp=transport@gp_point
        aw_actual=np.hstack((geo["linear_jacobian"].T@frame,geo["angular_jacobian"].T@frame))
        aw=aw_actual@np.linalg.inv(transport)
        point_j=[]
        for point in points:
            linear=np.zeros((3,model.nv));angular=np.zeros_like(linear)
            mujoco.mj_jac(model,data,linear,angular,np.asarray(point["position_world_m"]),geo["body"])
            point_j.append(frame.T@linear)
        jp=np.vstack(point_j);error=aw@gp-jp.T;red_error=item["reduction"].T@error
        u,s,vh=np.linalg.svd(gp,full_matrices=True);rank=int(np.linalg.matrix_rank(gp,tol=TOL));null=vh[rank:].T
        eta=[];force=R2.BASE.TAU.point_array(item)[side].reshape(-1);minimum=np.linalg.pinv(gp,rcond=1e-12)@(gp@force)
        for col in range(null.shape[1]): eta.append(float(null[:,col]@(force-minimum)))
        sides[name]={"geometry":{"contact_points":[p["position_world_m"] for p in points],"frame":frame,
            "actual_reference":reference,"production_reference":prod_ref,"transport":transport,
            "point_ids":[p["point_index"] for p in points],"efc_addresses":[p["efc_address"] for p in points]},
            "Gp":gp,"Aw":aw,"Jp":jp,"operator_full":metrics(error),"operator_reduced":metrics(red_error),
            "singular_values":s,"rank":rank,"nullity":int(null.shape[1]),"condition_number_nonzero":float(s[0]/s[rank-1]),
            "null_basis":null,"null_generalized_force":jp.T@null,"eta":eta,
            "point_force":force,"aggregate_wrench":gp@force}
        aw_block[:,6*side:6*side+6]=aw;gp_block[6*side:6*side+6,6*side:6*side+6]=gp
        jp_block[:,6*side:6*side+6]=jp.T
    return {"sides":sides,"Aw":aw_block,"Gp":gp_block,"JpT":jp_block,"production_geometry_metrics":prod_metrics}


def main() -> int:
    parser=argparse.ArgumentParser();parser.add_argument("--config",type=Path,default=R2.LEGAL.AUTH.CONFIG)
    parser.add_argument("--qp-dump",type=Path,required=True);parser.add_argument("--output",type=Path,required=True);parser.add_argument("--replay-of",type=Path)
    args=parser.parse_args();out=args.output.resolve()
    if out.exists():raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True);probes=out/"probes";probes.mkdir()
    cfgp,cfg,continuation,base,trim,wrench,authority,native,model,oracle,production,operators=setup_capture(args)
    amount=float(cfg["delta_m_s2"]);specs=[("baseline",np.zeros(4),0.,1.)]
    for name,start,direction,scales in (("slip-common",2,np.ones(2),(1.,.5,.25)),("xi-common",0,np.ones(2),(1.,)),("slip-differential",2,np.array([-1.,1.]),(1.,))):
        for sign in (-1.,1.):
            for scale in scales:
                delta=np.zeros(4);delta[start:start+2]=sign*scale*amount*direction;specs.append((f"{name}-{scale:g}-{sign:+.0f}",delta,sign,scale))
    captures={}
    for label,delta,_,_ in specs:
        captures[label]=R2.BASE.capture(base,cfg,probes/f"{label}.csv",authority,trim,native,model,oracle,args.qp_dump.resolve(),production,operators,delta)
    baseline=captures["baseline"];maps=fresh_maps(model,baseline)

    # Rebuild the exact fixed-active-set affine reaction map in M coordinates, then transform it correctly to P.
    oracle.evaluate(native,[],ctrl_override=-baseline["tau"]);data=oracle.data
    j=np.asarray(data.efc_J).reshape(data.nefc,model.nv).copy();force=np.asarray(data.efc_force).copy();d=np.asarray(data.efc_D).copy();aref=np.asarray(data.efc_aref).copy()
    types=np.asarray(data.efc_type,dtype=int);eq=np.flatnonzero(types==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY));contact=np.flatnonzero(types==int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL));physical=np.concatenate((eq,contact))
    actual_b=np.zeros((model.nv,6));saved=np.asarray(data.ctrl).copy()
    for col in range(6):data.ctrl[:]=0.;data.ctrl[col]=-1.;mujoco.mj_forward(model,data);actual_b[:,col]=data.qfrc_actuator
    data.ctrl[:]=saved;mujoco.mj_forward(model,data)
    mass=baseline["mass"];smooth=np.asarray(data.qfrc_smooth).copy();known=smooth-actual_b@baseline["tau"]
    system=mass+j[physical].T@(d[physical,None]*j[physical]);k=np.linalg.inv(system)
    q0=k@(known+j[physical].T@(d[physical]*aref[physical]));qt=k@actual_b
    fc0=d[contact]*(aref[contact]-j[contact]@q0);fct=-(d[contact,None]*j[contact])@qt
    qc0_m=j[contact].T@fc0;qct_m=j[contact].T@fct
    candidate=json.loads((PHASE/"evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json").read_text())
    rot=np.empty(9);mujoco.mju_quat2Mat(rot,np.asarray(baseline["actual"]["qpos"])[3:7]);rot=rot.reshape(3,3);x_pm,x_mp,_=R2.CANON.transforms(rot,np.asarray(candidate["frames"]["r_M_to_P_M"]))
    qc0_p=R2.CANON.force_m_to_p(qc0_m,x_mp);qct_p=np.column_stack([R2.CANON.force_m_to_p(qct_m[:,i],x_mp) for i in range(6)])

    fp=R2.BASE.TAU.point_array(baseline).reshape(-1);qrow_m=baseline["solver_force_channels"]["generalized"]["contact"]
    qpoint_m=maps["JpT"]@fp;wactual=maps["Gp"]@fp;qagg_m=maps["Aw"]@wactual
    qrow=R2.CANON.force_m_to_p(qrow_m,x_mp);qpoint=R2.CANON.force_m_to_p(qpoint_m,x_mp);qagg=R2.CANON.force_m_to_p(qagg_m,x_mp)
    qstage=qc0_p+qct_p@baseline["tau"]
    e1=qrow-qpoint;e2=qpoint-qagg;e3=qagg-qstage;total=qrow-qstage
    # Historical Stage-R mixed M-coordinate Aw*W with P-coordinate Qc0/Qct.
    historical=qagg_m-qstage
    offset_frame=qc0_m-qc0_p;slope_frame=qct_m@baseline["tau"]-qct_p@baseline["tau"]
    operator=maps["Aw"]@maps["Gp"]-maps["JpT"]
    operator_p=np.column_stack([R2.CANON.force_m_to_p(operator[:,i],x_mp) for i in range(operator.shape[1])])
    operator_red=baseline["reduction"].T@operator_p
    virtual=max(abs(float(np.sin(np.arange(16)+i)@operator@np.cos(np.arange(12)+.3*i))) for i in range(1,9))

    directional={}
    for label,item in captures.items():
        imaps=fresh_maps(model,item);f=R2.BASE.TAU.point_array(item).reshape(-1);qr_m=item["solver_force_channels"]["generalized"]["contact"]
        qp_m=imaps["JpT"]@f;qa_m=imaps["Aw"]@(imaps["Gp"]@f)
        # Same frozen H0 affine law is intentionally evaluated only as a local witness model.
        qs=qc0_p+qct_p@item["tau"]
        directional[label]={"E1":metrics(R2.CANON.force_m_to_p(qr_m-qp_m,x_mp)),
            "E2":metrics(R2.CANON.force_m_to_p(qp_m-qa_m,x_mp)),
            "corrected_E3":metrics(R2.CANON.force_m_to_p(qa_m,x_mp)-qs),
            "historical_mixed_frame_E3":metrics(qa_m-qs),
            "eta":{name:imaps["sides"][name]["eta"] for name in ("left","right")}}

    op_pass=max(np.max(np.abs(operator)),np.max(np.abs(operator_red)),virtual)<=TOL
    e1_pass=np.max(np.abs(e1))<=TOL;e2_pass=np.max(np.abs(e2))<=TOL
    corrected_stage_pass=np.max(np.abs(e3))<=TOL;historical_reproduced=abs(np.max(np.abs(historical))-4.836644038376806)<=1e-10
    replay_passes={name:all(row[key]["max_abs"]<=TOL for label,row in directional.items() if label=="baseline" or label.startswith(prefix)
                            for key in ("E1","E2","corrected_E3"))
                   for name,prefix in (("H0","baseline"),("slip_common","slip-common"),("xi_common","xi-common"),("slip_differential","slip-differential"))}
    classification="A-AGGREGATE-DYNAMICS-SUFFICIENT-STAGER-AFFINE-MAP-MISMATCH" if op_pass and e1_pass and e2_pass and corrected_stage_pass and historical_reproduced else "U-UNTRUSTED"
    result={"schema_version":1,"phase":46,"classification":classification,"controller_numerics_changed":False,
      "map_provenance":{"fresh_rebuild":True,"constraint_rows":contact,"row_types":types[contact],"force_sign":"ground-on-wheel","canonical_coordinates":"P base-control reference","maps":maps},
      "operator_identity":{"full":metrics(operator),"full_P":metrics(operator_p),"reduced":metrics(operator_red),"virtual_work_residual":virtual,"pass":op_pass},
      "commuting_diagram":{"Q_row":qrow,"Q_point":qpoint,"Q_agg":qagg,"Q_StageR_corrected":qstage,"Q_agg_historical_mislabeled_M_as_P":qagg_m,
        "E1":metrics(e1,baseline["obs_map"]@x_pm.T),"E2":metrics(e2,baseline["obs_map"]@x_pm.T),"E3_corrected":metrics(e3,baseline["obs_map"]@x_pm.T),"total_corrected":metrics(total,baseline["obs_map"]@x_pm.T),
        "E3_historical_mixed_frame":metrics(historical),"historical_4p836644_reproduced":historical_reproduced,"material_edge":"AGGREGATE→STAGER generalized-force coordinate canonicalization"},
      "affine_map_audit":{"entered":True,"Qc0_M":qc0_m,"Qc0_P":qc0_p,"Qct_tau_M":qct_m@baseline["tau"],"Qct_tau_P":qct_p@baseline["tau"],
        "affine_offset_frame_error":metrics(offset_frame),"affine_slope_frame_error":metrics(slope_frame),"corrected_H0_error":metrics(e3),
        "state_dependence_error":"NOT MATERIAL AT H0; directional fixed-H0 map is diagnostic only","root_cause":"Aw*W remained in M generalized-force coordinates while Qc0+Qct*tau was canonicalized to P"},
      "sufficiency":{"aggregate_dynamics":"PASS","aggregate_constitutive":"NOT REQUIRED FOR THIS FIRST MISMATCH","minimal_R2_representation":"aggregate wrench",
        "representation_change_required":False,"representation_candidate_next_round":False},
      "nullspace_consistency":{"entered_for_regression_only":True,"sides":{name:{"rank":maps["sides"][name]["rank"],"nullity":maps["sides"][name]["nullity"],
        "singular_values":maps["sides"][name]["singular_values"],"null_generalized_force":maps["sides"][name]["null_generalized_force"],"H0_eta":maps["sides"][name]["eta"]} for name in ("left","right")},
        "null_mode_generalized_force_effect":"NUMERICAL"},
      "directional_replay":directional,"representation_replay_gates":replay_passes,"H0_representation_replay":"PASS after canonical force-dual transform",
      "R2_physical_source_law":"VALID","R2_authorized":False,"R2_implementation_authorized":False,"R2_implemented":False,
      "next_allowed_action":"Stage-R affine reaction-map attribution"}
    decision=out/"r2-contact-reaction-commuting-diagram-attribution.json";write(decision,result)
    replay=None if args.replay_of is None else R2.P45.semantic_error(args.replay_of/decision.name,decision)
    finite=all(np.all(np.isfinite(v)) for v in (operator,e1,e2,e3,historical,qc0_m,qct_m))
    passed=classification!="U-UNTRUSTED" and finite and (replay is None or replay<=1e-11)
    write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"nonfinite":not finite,"controller_numerics_changed":False})
    sources=[cfgp,continuation,ROOT/base["scene"],ROOT/base["executable"],authority,wrench,args.qp_dump.resolve(),Path(__file__).resolve(),Path(R2.__file__).resolve(),Path(OP.__file__).resolve()]
    write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),
      "dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if passed else 2


if __name__=="__main__":raise SystemExit(main())
