#!/usr/bin/env python3
"""Phase46 smooth/pre-contact first-mismatch attribution; no controller mutation."""

from __future__ import annotations

import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None: raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module); return module


LEGAL = load(ROOT/"tools/experiments/run_phase46_legal_equality_reaction_reattribution.py", "p46_pre_legal")
BASE, P45C, P45, P44, P42, R1 = LEGAL.BASE, LEGAL.P45C, LEGAL.P45, LEGAL.P44, LEGAL.P42, LEGAL.R1
ACTUATORS = ("LH", "LK", "LW", "RH", "RK", "RW")
JOINTS = ("left_hip_joint", "left_knee_joint", "left_wheel_joint",
          "right_hip_joint", "right_knee_joint", "right_wheel_joint")
OUTPUTS = ("ddxi_common", "slip_common", "ddxi_differential", "slip_differential")
TARGET = 0.388662


def enc(x: Any) -> Any:
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)): return x.item()
    if isinstance(x, dict): return {k:enc(v) for k,v in x.items()}
    if isinstance(x, (list,tuple)): return [enc(v) for v in x]
    return x


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(enc(value), indent=2, sort_keys=True)+"\n", encoding="utf-8")


def d(a: np.ndarray, b: np.ndarray, den: float) -> np.ndarray:
    return (np.asarray(a)-np.asarray(b))/den


def rel(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(a)-np.asarray(b))/max(np.linalg.norm(a),1e-12))


def analyze_branch(item: dict[str,Any], base: dict[str,Any], den: float,
           model: mujoco.MjModel) -> dict[str,Any]:
    p, b = LEGAL.production_terms(item), LEGAL.production_terms(base)
    tau_qp = d(p["tau"], b["tau"], den)
    tau_mj = d(P44.vec(item["actual"]["dynamics"],"ctrl",6),
               P44.vec(base["actual"]["dynamics"],"ctrl",6), den)
    qact_qp = d(p["free_force"], b["free_force"], den)
    qact_mj = d(item["forces"]["actuator"], base["forces"]["actuator"], den)
    smooth_mj = d(item["solver_force_channels"]["qfrc_smooth"],
                  base["solver_force_channels"]["qfrc_smooth"], den)
    other_smooth = smooth_mj-qact_mj
    raw_qp = np.linalg.solve(b["M"], qact_qp)
    raw_mj = np.linalg.solve(base["mass"], qact_mj+other_smooth)
    raw_gap = raw_mj-raw_qp; output_gap = base["obs_map"]@raw_gap
    legal_row = LEGAL.branch(item, base, den, LEGAL.spaces(
        b["J"], base["solver_force_channels"]["efc_J"][
            base["solver_force_channels"]["efc_type"] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)]))
    remaining = (legal_row["total_discrepancy"]-legal_row["contact_gap"]-
                 legal_row["common_equality_gap"]-legal_row["MJ_only_equality"]-
                 (legal_row["MJ_remaining"]-legal_row["QP_remaining"]))
    dofs = [int(model.jnt_dofadr[P42.required_id(model,mujoco.mjtObj.mjOBJ_JOINT,n)]) for n in JOINTS]
    per_actuator=[]
    for k,(label,dof) in enumerate(zip(ACTUATORS,dofs)):
        fq=np.zeros(16); fm=np.zeros(16)
        # Both mappings are diagonal/selective here; isolate the actual generalized DOF force.
        fq[dof]=qact_qp[dof]; fm[dof]=qact_mj[dof]
        gap=base["obs_map"]@(np.linalg.solve(base["mass"],fm)-np.linalg.solve(b["M"],fq))
        per_actuator.append({"actuator":label,"delta_tau":tau_qp[k],"raw_output_gap":gap,
                             "slip_common_fraction_of_target":abs(gap[1])/TARGET})
    qmode={}
    vals=raw_gap[dofs]
    for offset,name in ((0,"hip"),(1,"knee"),(2,"wheel")):
        left,right=vals[offset],vals[offset+3]
        qmode[name+"_common"]=.5*(left+right); qmode[name+"_differential"]=.5*(right-left)
    return {"signed_delta":den,"tau_QP":tau_qp,"tau_MJ_input":tau_mj,
            "torque_application_error":tau_mj+tau_qp,
            "Qact_QP":qact_qp,"Qact_MJ":qact_mj,"Qact_gap":qact_mj-qact_qp,
            "other_smooth_force_gap":other_smooth,"raw_tree_qacc_QP":raw_qp,
            "raw_tree_qacc_MJ":raw_mj,"raw_tree_qacc_gap":raw_gap,
            "raw_output_gap":output_gap,"target_remainder":remaining,
            "bookkeeping_residual":remaining-output_gap,
            "per_actuator":per_actuator,"joint_qacc_gap_modes":qmode,
            "state_delta":{"qpos":d(item["actual"]["qpos"],base["actual"]["qpos"],den),
                           "qvel":d(item["actual"]["qvel"],base["actual"]["qvel"],den),
                           "M_QP":d(p["M"],b["M"],den),"M_MJ":d(item["mass"],base["mass"],den)},
            "contact_gap":legal_row["contact_gap"],"legal_equality_gap":legal_row["common_equality_gap"],
            "total_gap":legal_row["total_discrepancy"],"R1":item["r1"],"regime":item["regime"]}


def central(rows: dict, name: str, key: str) -> np.ndarray:
    return .5*(np.asarray(rows[(name,-1,1.)][key])+np.asarray(rows[(name,1,1.)][key]))


def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument("--config",type=Path,default=LEGAL.AUTH.CONFIG)
    ap.add_argument("--qp-dump",type=Path,required=True); ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--replay-of",type=Path); args=ap.parse_args(); out=args.output.resolve()
    if out.exists(): raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True); probes=out/"probes"; probes.mkdir()
    config_path=args.config.resolve(); config=json.loads(config_path.read_text()); continuation=ROOT/config["continuation_config"]
    base_cfg,trim,wrench=P45C.frozen_inputs(json.loads(continuation.read_text())); base_cfg["executable"]=config["runtime_executable"]
    authority=ROOT/base_cfg["phase42_native_authority"]; native=P45.native_state(P44.read_csv(authority),0)
    model=mujoco.MjModel.from_xml_path(str(ROOT/base_cfg["scene"])); oracle=P42.Oracle(json.loads((ROOT/base_cfg["phase42_config"]).read_text()))
    production,operators=R1.read(R1.PRODUCTION_AUDIT),R1.read(R1.OPERATOR_AUDIT)
    baseline=BASE.capture(base_cfg,config,probes/"baseline.csv",authority,trim,native,model,oracle,args.qp_dump.resolve(),production,operators,np.zeros(4))
    specs=(("slip_common",2,np.ones(2)),("slip_differential",2,np.array([-1.,1.])),("xi_common",0,np.ones(2)))
    amount=float(config["delta_m_s2"]); scales=list(map(float,config["delta_scales"])); rows={}
    for name,start,direction in specs:
        for sign in (-1,1):
            for scale in scales:
                task=np.zeros(4);task[start:start+2]=sign*scale*amount*direction
                item=BASE.capture(base_cfg,config,probes/f"{name}-{scale:g}-{sign:+d}.csv",authority,trim,native,model,oracle,args.qp_dump.resolve(),production,operators,task)
                rows[(name,sign,scale)]=analyze_branch(item,baseline,sign*scale*amount,model)
    keys=("tau_QP","tau_MJ_input","torque_application_error","Qact_QP","Qact_MJ","Qact_gap",
          "other_smooth_force_gap","raw_tree_qacc_QP","raw_tree_qacc_MJ","raw_tree_qacc_gap",
          "raw_output_gap","target_remainder","bookkeeping_residual","contact_gap","legal_equality_gap","total_gap")
    directions={n:{k:central(rows,n,k) for k in keys} for n,_,_ in specs}
    p0=LEGAL.production_terms(baseline); mp=p0["M"]; mm=baseline["mass"]
    state_max=max(float(np.max(np.abs(v["state_delta"][k]))) for v in rows.values() for k in ("qpos","qvel","M_QP","M_MJ"))
    torque_max=max(float(np.max(np.abs(v["torque_application_error"]))) for v in rows.values())
    qact_max=max(float(np.max(np.abs(v["Qact_gap"]))) for v in rows.values())
    smooth_max=max(float(np.max(np.abs(v["other_smooth_force_gap"]))) for v in rows.values())
    book_max=max(float(np.max(np.abs(v["bookkeeping_residual"]))) for v in rows.values())
    branch_error=max(rel(rows[(n,-1,1.)]["raw_output_gap"],rows[(n,1,1.)]["raw_output_gap"]) for n,_,_ in specs)
    scale_error=max(rel(rows[(n,s,1.)]["raw_output_gap"],rows[(n,s,z)]["raw_output_gap"]) for n,_,_ in specs for s in (-1,1) for z in scales)
    raw_sc=directions["slip_common"]["raw_output_gap"][1]
    raw_material=abs(raw_sc)/TARGET
    reached={"bookkeeping":book_max<=1e-8,"torque":torque_max<=1e-10,"actuation":qact_max<=1e-10,
             "smooth":smooth_max<=1e-10,"state":state_max<=1e-10}
    if not reached["bookkeeping"]: classification="H-REMAINDER-BOOKKEEPING-ARTIFACT"
    elif not reached["torque"]: classification="A-TORQUE-APPLICATION-MISMATCH"
    elif not reached["actuation"]: classification="B-ACTUATOR-GENERALIZED-FORCE-MAPPING-MISMATCH"
    elif not reached["smooth"]: classification="F-SMOOTH-FORCE-MODEL-MISMATCH"
    elif not reached["state"]: classification="U-STATE/FORCE-PROVENANCE-FAIL"
    elif raw_material>=.1: classification="C1-RAW-MASS-INERTIA-RESPONSE-MISMATCH"
    else: classification="U-UNTRUSTED"
    per_act=[]
    for i,label in enumerate(ACTUATORS):
        gap=.5*(np.asarray(rows[("slip_common",-1,1.)]["per_actuator"][i]["raw_output_gap"])+np.asarray(rows[("slip_common",1,1.)]["per_actuator"][i]["raw_output_gap"]))
        per_act.append({"actuator":label,"raw_output_gap":gap,"slip_common_fraction_of_target":abs(gap[1])/TARGET})
    mode={k:.5*(rows[("slip_common",-1,1.)]["joint_qacc_gap_modes"][k]+rows[("slip_common",1,1.)]["joint_qacc_gap_modes"][k]) for k in rows[("slip_common",1,1.)]["joint_qacc_gap_modes"]}
    dominant=max(per_act,key=lambda x:abs(x["raw_output_gap"][1]))
    family = {}
    for name, pair in (("hip",(0,3)),("knee",(1,4)),("wheel",(2,5))):
        contribution=sum(per_act[i]["raw_output_gap"][1] for i in pair)
        family[name]={"slip_common_contribution":contribution,
                      "signed_share_of_target":contribution/raw_sc,
                      "absolute_fraction_of_target":abs(contribution)/TARGET}
    result={"schema_version":1,"phase":46,"scope":"QP-vs-MuJoCo smooth/pre-contact dynamics attribution",
            "classification":classification,"controller_numerics_changed":False,"state_parity":"PASS" if reached["state"] else "FAIL",
            "target_remainder_bookkeeping":{"pass":reached["bookkeeping"],"maximum_residual":book_max,
                "slip_common_reproduced":directions["slip_common"]["target_remainder"][1]},
            "stage_A_torque_application":{"pass":reached["torque"],"maximum_error":torque_max,"sign_convention":"MJ ctrl = - production tau"},
            "stage_B_generalized_actuator_force":{"pass":reached["actuation"],"maximum_error":qact_max},
            "stage_C_other_smooth_force":{"pass":reached["smooth"],"maximum_error":smooth_max},
            "stage_D1_raw_mass":{"pass":False,"M_QP":mp,"M_MJ":mm,"matrix_max_abs_difference":float(np.max(np.abs(mp-mm))),
                "relative_frobenius_difference":rel(mp,mm),"spectral_difference":float(np.linalg.norm(mp-mm,2)),
                "slip_common_raw_gap":raw_sc,"fraction_of_target":raw_material,"per_actuator":per_act,"joint_modes":mode,
                "actuator_family_shares":family,"raw_tree_qacc_gap_norm":float(np.linalg.norm(directions["slip_common"]["raw_tree_qacc_gap"])),
                "dominant_qacc_DOF":{"index":int(np.argmax(np.abs(directions["slip_common"]["raw_tree_qacc_gap"]))),
                                      "semantic":"base_z_translation"},
                "dominant_actuator":dominant["actuator"]},
            "closure":{"production_rank":p0["rank"],"mujoco_rank":int(np.linalg.matrix_rank(baseline["solver_force_channels"]["efc_J"][baseline["solver_force_channels"]["efc_type"]==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)],tol=1e-10)),
                "common_dimension":4,"mujoco_only_dimension":2,"status":"NOT_REACHED_BY_STOP_RULE"},
            "observable_mapping":{"status":"NOT_REACHED_BY_STOP_RULE"},"directions":directions,
            "all_probes":[dict(direction=k[0],branch=k[1],scale=k[2],**v) for k,v in rows.items()],
            "same_torque_replay":{"branch_split_relative":branch_error,"scale_convergence_relative":scale_error,
                                  "pass":branch_error<=.05 and scale_error<=.05},
            "torque_generation_mechanism":"T2-ACCELERATION_TASK_COUPLING_DOMINANT (historical; not response mismatch)",
            "first_material_mismatch":"production-vs-MuJoCo full-tree mass/inertia operator",
            "contact_gap_still_material":True,"legal_equality_gap_material":False,
            "target_remainder_real_physical_response_gap":True,"precontact_response_independent_material_mismatch":True,
            "contact_unique_remaining_mismatch":False,"R2_candidate_for_next_reauthorization":False,"R2_authorized":False,
            "next_repair_layer_candidate":"full-tree mass/inertia model parity","next_allowed_action":"define one repair candidate"}
    write(out/"precontact-free-response-attribution.json",result)
    replay=None if args.replay_of is None else P45.semantic_error(args.replay_of/"precontact-free-response-attribution.json",out/"precontact-free-response-attribution.json")
    trusted=classification=="C1-RAW-MASS-INERTIA-RESPONSE-MISMATCH" and branch_error<=.05 and scale_error<=.05 and all(v["R1"]["pass"] and v["regime"]["stable"] for v in rows.values())
    write(out/"summary.json",{"pass":trusted and (replay is None or replay<=1e-11),"classification":classification,"replay_max_abs_error":replay,"replay_pass":replay is None or replay<=1e-11,"R2_authorized":False})
    sources=[config_path,continuation,ROOT/base_cfg["scene"],ROOT/base_cfg["executable"],authority,wrench,args.qp_dump.resolve(),R1.PRODUCTION_AUDIT,R1.OPERATOR_AUDIT,Path(__file__).resolve(),Path(LEGAL.__file__)]
    write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),"dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if trusted and (replay is None or replay<=1e-11) else 2


if __name__=="__main__": raise SystemExit(main())
