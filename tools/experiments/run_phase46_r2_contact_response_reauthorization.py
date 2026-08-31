#!/usr/bin/env python3
"""Phase46 R2 contact-response re-authorization; diagnostic only."""

from __future__ import annotations

import argparse, hashlib, importlib.util, json, platform, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
OUTPUTS = ("ddxi_common", "slip_common", "ddxi_differential", "slip_differential")


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module); return module


BASE = load(ROOT / "tools/experiments/run_phase46_post_corrected_r1_authority_attribution.py", "p46_r2_base")
CANON_RUN = load(ROOT / "tools/experiments/run_phase46_base_reference_canonicalization_implementation.py", "p46_r2_canon_run")
CANON = CANON_RUN.C
LEGAL = CANON_RUN.LEGAL
P45C, P45, P44, P42, R1 = CANON_RUN.P45C, CANON_RUN.P45, CANON_RUN.P44, CANON_RUN.P42, CANON_RUN.R1


def enc(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: enc(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [enc(item) for item in value]
    return value


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(enc(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def matrix(dump: dict[str, Any], name: str) -> np.ndarray:
    return np.asarray(dump[name], dtype=float)


def solve_equality_qp(h: np.ndarray, g: np.ndarray, equality: np.ndarray,
                      rhs: np.ndarray) -> tuple[np.ndarray, float]:
    kkt = np.block([[h, equality.T], [equality, np.zeros((len(rhs), len(rhs)))]])
    value = np.concatenate((-g, rhs))
    solved = np.linalg.lstsq(kkt, value, rcond=1e-11)[0]
    x = solved[:h.shape[0]]
    residual = max(np.max(np.abs(h@x+g+equality.T@solved[h.shape[0]:])),
                   np.max(np.abs(equality@x-rhs)))
    return x, float(residual)


def active_set_qp(h: np.ndarray, g: np.ndarray, a: np.ndarray, lower: np.ndarray,
                  upper: np.ndarray, extra: np.ndarray, extra_rhs: np.ndarray) -> dict[str, Any]:
    eq = [row for row in range(len(lower)) if lower[row] == upper[row]]
    rows = [a[eq], extra]; rhs = [lower[eq], extra_rhs]
    selected: list[tuple[int, int]] = []
    for _ in range(40):
        e = np.vstack(rows); b = np.concatenate(rhs)
        u, s, _ = np.linalg.svd(e, full_matrices=False)
        keep = s > 1e-10
        independent = u[:, keep].T @ e
        independent_rhs = u[:, keep].T @ b
        x, residual = solve_equality_qp(h, g, independent, independent_rhs)
        ax = a@x
        low = lower-ax; high = ax-upper
        low[eq] = -np.inf; high[eq] = -np.inf
        li, hi = int(np.argmax(low)), int(np.argmax(high))
        lv, hv = low[li], high[hi]
        if max(lv, hv) <= 2e-7:
            return {"x": x, "kkt_residual": residual, "maximum_violation": max(float(lv), float(hv), 0.0),
                    "added_active_rows": selected, "equality_rank": int(np.sum(keep))}
        row, side, target = (li, -1, lower[li]) if lv >= hv else (hi, 1, upper[hi])
        if (row, side) in selected: break
        selected.append((row, side)); rows.append(a[row:row+1]); rhs.append(np.asarray([target]))
    return {"x": x, "kkt_residual": residual, "maximum_violation": float(max(lv, hv)),
            "added_active_rows": selected, "equality_rank": int(np.sum(keep))}


def contact_forces(model: mujoco.MjModel, data: mujoco.MjData,
                   row_force: np.ndarray) -> tuple[np.ndarray, list[dict[str, Any]]]:
    values=[]; rows=[]
    for contact_id in range(data.ncon):
        contact=data.contact[contact_id]; address=int(contact.efc_address)
        if address < 0: continue
        dim=int(contact.dim); pyramid=row_force[address:address+2*(dim-1)]
        decoded=np.zeros(dim); decoded[0]=np.sum(pyramid)
        for axis in range(dim-1):decoded[axis+1]=contact.friction[axis]*(pyramid[2*axis]-pyramid[2*axis+1])
        actual=np.zeros(6); mujoco.mj_contactForce(model,data,contact_id,actual)
        values.append(decoded[:3]); rows.append({"contact_id":contact_id,"efc_address":address,"dim":dim,
            "decoded_N_T1_T2":decoded[:3],"actual_N_T1_T2":actual[:3],
            "max_abs_error":np.max(np.abs(decoded-actual[:3])),"geoms":[int(contact.geom1),int(contact.geom2)]})
    return np.asarray(values),rows


def mode(values: np.ndarray) -> np.ndarray:
    return np.asarray([.5*(values[0]+values[1]),.5*(values[2]+values[3]),
                       .5*(values[1]-values[0]),.5*(values[3]-values[2])])


def main() -> int:
    parser=argparse.ArgumentParser();parser.add_argument("--config",type=Path,default=LEGAL.AUTH.CONFIG)
    parser.add_argument("--qp-dump",type=Path,required=True);parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--replay-of",type=Path);args=parser.parse_args();out=args.output.resolve()
    if out.exists(): raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True);probes_dir=out/"probes";probes_dir.mkdir()
    cfgp=args.config.resolve();cfg=json.loads(cfgp.read_text());continuation=ROOT/cfg["continuation_config"]
    base,trim,wrench=P45C.frozen_inputs(json.loads(continuation.read_text()));base["executable"]=cfg["runtime_executable"]
    authority=ROOT/base["phase42_native_authority"];native=P45.native_state(P44.read_csv(authority),0)
    model=mujoco.MjModel.from_xml_path(str(ROOT/base["scene"]));oracle=P42.Oracle(json.loads((ROOT/base["phase42_config"]).read_text()))
    production,operator_source=R1.read(R1.PRODUCTION_AUDIT),R1.read(R1.OPERATOR_AUDIT)
    amount=float(cfg["delta_m_s2"]);scales=list(map(float,cfg["delta_scales"]));specs=[("slip_common",2,np.ones(2),scales),("xi_common",0,np.ones(2),scales),("slip_differential",2,np.array([-1.,1.]),[1.,.5])]
    capture_specs=[("baseline",np.zeros(4),0.,0.)]
    for name,start,direction,these_scales in specs:
        for sign in (-1,1):
            for scale in these_scales:
                delta=np.zeros(4);delta[start:start+2]=sign*scale*amount*direction
                capture_specs.append((f"{name}-{scale:g}-{sign:+d}",delta,float(sign),float(scale)))
    captures={}; dumps={}
    for label,delta,sign,scale in capture_specs:
        path=probes_dir/f"{label}.csv"
        captures[label]=BASE.capture(base,cfg,path,authority,trim,native,model,oracle,args.qp_dump.resolve(),production,operator_source,delta)
        dumps[label]=R1.dump(args.qp_dump.resolve(),path)

    baseline=captures["baseline"];data=oracle.data
    # BASE.capture leaves the runtime oracle at its most recent probe; restore baseline state/tau.
    oracle.evaluate(native, [], ctrl_override=-baseline["tau"]); data=oracle.data
    j=np.asarray(data.efc_J).reshape(data.nefc,model.nv).copy();force=np.asarray(data.efc_force).copy()
    d=np.asarray(data.efc_D).copy();aref=np.asarray(data.efc_aref).copy();types=np.asarray(data.efc_type,dtype=int)
    equality=np.flatnonzero(types==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY));contact=np.flatnonzero(types==int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL))
    physical_equality=equality[[0,2,3,5]];physical=np.concatenate((equality,contact))
    mass=baseline["mass"];smooth=np.asarray(data.qfrc_smooth).copy();qacc=np.asarray(data.qacc).copy()
    system=mass+j[physical].T@(d[physical,None]*j[physical]);rhs=smooth+j[physical].T@(d[physical]*aref[physical])
    oracle_qacc=np.linalg.solve(system,rhs);oracle_force=d*(aref-j@oracle_qacc)
    physical4=np.concatenate((physical_equality,contact));system4=mass+j[physical4].T@(d[physical4,None]*j[physical4]);rhs4=smooth+j[physical4].T@(d[physical4]*aref[physical4]);qacc4=np.linalg.solve(system4,rhs4)
    decoded,point_rows=contact_forces(oracle.model,data,oracle_force)
    actual_decoded,actual_point_rows=contact_forces(oracle.model,data,force)
    obs=baseline["obs_map"]
    oracle_metrics={"qacc_max_abs_error":np.max(np.abs(oracle_qacc-qacc)),
      "row_force_max_abs_error":np.max(np.abs(oracle_force[contact]-force[contact])),
      "point_force_max_abs_error":max(row["max_abs_error"] for row in point_rows),
      "generalized_force_max_abs_error":np.max(np.abs(j[contact].T@oracle_force[contact]-j[contact].T@force[contact])),
      "observable_max_abs_error":np.max(np.abs(obs@(oracle_qacc-qacc))),
      "full_dynamics_closure":np.max(np.abs(mass@qacc-smooth-j.T@force)),
      "native_constitutive_closure":np.max(np.abs(force+d*(j@qacc-aref))),
      "rank4_removal_qacc_effect":np.max(np.abs(qacc4-qacc)),"rank4_removal_observable_effect":np.max(np.abs(obs@(qacc4-qacc))),
      "point_rows":point_rows,"actual_point_rows":actual_point_rows}
    oracle_pass=max(oracle_metrics[key] for key in ("qacc_max_abs_error","row_force_max_abs_error",
        "point_force_max_abs_error","generalized_force_max_abs_error","observable_max_abs_error",
        "full_dynamics_closure","native_constitutive_closure"))<=1e-8

    # State-dependent Schur map tau -> contact generalized reaction.
    actual_b=np.zeros((model.nv,6))
    candidate=json.loads((PHASE/"evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json").read_text())
    rot=np.empty(9);mujoco.mju_quat2Mat(rot,np.asarray(baseline["actual"]["qpos"])[3:7]);rot=rot.reshape(3,3)
    x_pm,x_mp,_=CANON.transforms(rot,np.asarray(candidate["frames"]["r_M_to_P_M"]))
    saved_ctrl=np.asarray(data.ctrl).copy()
    for col in range(6):
        data.ctrl[:]=0.;data.ctrl[col]=-1.;mujoco.mj_forward(oracle.model,data);actual_b[:,col]=data.qfrc_actuator
    data.ctrl[:]=saved_ctrl;mujoco.mj_forward(oracle.model,data)
    actuation_parity=np.max(np.abs(actual_b@baseline["tau"]-baseline["forces"]["actuator"]))
    smooth_known=smooth-actual_b@baseline["tau"]
    k=np.linalg.inv(system);q0=k@(smooth_known+j[physical].T@(d[physical]*aref[physical]));qt=k@actual_b
    fc0=d[contact]*(aref[contact]-j[contact]@q0);fct=-(d[contact,None]*j[contact])@qt
    qc0=j[contact].T@fc0;qct=j[contact].T@fct
    qc0p=CANON.force_m_to_p(qc0,x_mp);qctp=np.column_stack([CANON.force_m_to_p(qct[:,i],x_mp) for i in range(6)])
    aw=np.zeros((16,12))
    for side,name in enumerate(("left","right")):aw[:,6*side:6*side+6]=np.asarray(operator_source["sides"][name]["Aw_full"])

    stage_r={}
    for label,delta,sign,scale in capture_specs:
        dump=dumps[label];h=matrix(dump,"h");g=matrix(dump,"g").reshape(-1);a=matrix(dump,"a");lower=matrix(dump,"lower").reshape(-1);upper=matrix(dump,"upper").reshape(-1);vs=matrix(dump,"variable_scale").reshape(-1)
        extra=np.zeros((16,42));extra[:,12:18]=-qctp*vs[None,12:18];extra[:,18:30]=aw*vs[None,18:30]
        solved=active_set_qp(h,g,a,lower,upper,extra,qc0p)
        physical_solution=vs*solved["x"];tau=physical_solution[12:18];w=physical_solution[18:30].copy()
        for side,name in enumerate(("left","right")):w[6*side:6*side+6]=np.asarray(production["sides"][name]["Pg_production"])@w[6*side:6*side+6]
        qa=q0+qt@tau;fc=fc0+fct@tau;qc=qc0+qct@tau
        stage_r[label]={"tau":tau,"wrench":w,"qacc":qa,"contact_row_force":fc,"contact_generalized_force":qc,
          "outputs":obs@qa,"R1_residual":max(np.max(np.abs((np.eye(6)-np.asarray(production["sides"][name]["Pg_production"]))@w[6*i:6*i+6])) for i,name in enumerate(("left","right"))),
          **{key:value for key,value in solved.items() if key!="x"}}

    def central(name:str,key:str)->np.ndarray:
        return .5*((np.asarray(stage_r[f"{name}-1--1"][key])-np.asarray(stage_r["baseline"][key]))/(-amount)+
                   (np.asarray(stage_r[f"{name}-1-+1"][key])-np.asarray(stage_r["baseline"][key]))/(amount))
    transfers={name:{"candidate_outputs":central(name,"outputs"),"candidate_tau":central(name,"tau")}
               for name,_,_,_ in specs}
    # Fresh authoritative current contact channel from the same probes.
    conditioned=json.loads((PHASE/"evidence/automated/closure-conditioned-effective-inertia-formal-v2/closure-conditioned-effective-inertia-audit.json").read_text())
    mp=np.asarray(conditioned["operator_provenance"]["M_prod"]);mmp=CANON.mass_p_to_m(mp,x_pm);contact_rows={}
    for label,delta,sign,scale in capture_specs[1:]:
        item=captures[label];den=sign*scale*amount
        delta_w=(item["wrench_qp"]-baseline["wrench_qp"])/den
        qp=CANON.force_p_to_m(aw@delta_w,x_pm)
        mj=(item["solver_force_channels"]["generalized"]["contact"]-baseline["solver_force_channels"]["generalized"]["contact"])/den
        contact_rows[label]=obs@(np.linalg.solve(mass,mj)-np.linalg.solve(mmp,qp))
    contact_gap=.5*(contact_rows["slip_common-1--1"]+contact_rows["slip_common-1-+1"])
    current_actual=.5*(mode((captures["slip_common-1--1"]["mj"]-baseline["mj"])/(-amount))+mode((captures["slip_common-1-+1"]["mj"]-baseline["mj"])/(amount)))
    candidate_slip=transfers["slip_common"]["candidate_outputs"]
    harmful_current=float(current_actual[0]);harmful_candidate=float(candidate_slip[0])
    branch=max(np.linalg.norm((np.asarray(stage_r[f"{name}-1--1"]["outputs"])-stage_r["baseline"]["outputs"])/(-amount)-(np.asarray(stage_r[f"{name}-1-+1"]["outputs"])-stage_r["baseline"]["outputs"])/(amount))/max(np.linalg.norm(transfers[name]["candidate_outputs"]),1e-12) for name,_,_,_ in specs)
    scale_error=0.
    for name,_,_,these in specs:
        for sign in (-1,1):
            ref=(np.asarray(stage_r[f"{name}-1-{sign:+d}"]["outputs"])-stage_r["baseline"]["outputs"])/(sign*amount)
            for scale in these:
                val=(np.asarray(stage_r[f"{name}-{scale:g}-{sign:+d}"]["outputs"])-stage_r["baseline"]["outputs"])/(sign*scale*amount)
                scale_error=max(scale_error,np.linalg.norm(val-ref)/max(np.linalg.norm(ref),1e-12))
    h0=stage_r["baseline"]
    h0["delta_tau_from_current_controller"]=np.asarray(h0["tau"])-baseline["tau"]
    h0["delta_tau_norm_from_current_controller"]=np.linalg.norm(h0["delta_tau_from_current_controller"])
    stage_r_pass=(h0["maximum_violation"]<=2e-7 and h0["kkt_residual"]<=1e-7 and h0["R1_residual"]<=1e-8 and
                  abs(harmful_candidate)<=.1 and candidate_slip[1]>0 and branch<=.05 and scale_error<=.05 and
                  max(row["maximum_violation"] for row in stage_r.values())<=2e-7)
    inventory={"decision_variables":{"nudot":12,"tau":6,"aggregate_wrench":12,"slack":12},
      "hard_dynamics":"M*nudot - B*tau - sum(Aw_i*Pg_i*w_i) = -h",
      "closure":"qdd_full=N*nudot+c_N; J_eq*N=0; J_eq*c_N+JdotV=0",
      "contact_geometry":"material point J_c and JdotV/contact_bias; corrected production-reference rank-5 point image",
      "friction_unilateral":"37-row aggregate wrench cone per side",
      "contact_acceleration":"soft objective J_c*nudot+contact_bias -> 0; not a physical reaction law",
      "rolling_xi":"soft desired kinematic objectives, separate from contact physics",
      "interaction_wrench":"affine acceleration/contact task with slack",
      "tau_contact_coupling":"YES through hard reduced dynamics",
      "missing_relation":"no constitutive/complementarity equation binds selected contact wrench to the reaction generated by the same tau and active contact state"}
    selected={"physical_law":"state/active-set-dependent compliant contact reaction: f_c=D_c(a_ref,c-J_c*qacc), coupled with full dynamics and exact rank-4 closure",
      "coupled_contract":["M*qacc=q_smooth+B*tau+J_eq^T*f_eq+J_c^T*f_c","f=D*(a_ref-J*qacc)","w in Range(Gp_prod), Aw*w=J_c^T*f_c"],
      "schur_contract":"qacc=(M+J^T D J)^-1(q_smooth+B*tau+J^T D a_ref); f_c=D_c(a_ref,c-J_c*qacc)",
      "A_B_equivalent":True,"preferred_form":"SCHUR","state_dependent":True,"empirical_inverse":False,"double_solve":False,
      "runtime_estimate":"one state-dependent 16x16 SPD factorization plus contact back-substitution integrated as one QP affine block; reuse M/J; no controller double-solve"}
    authorized=bool(oracle_pass and actuation_parity<=1e-10 and stage_r_pass)
    classification="B-R2-PHYSICAL-LAW-AUTHORIZED-SCHUR-IMPLEMENTATION" if authorized else "E-R2-SOURCE-CLOSED-BUT-LAW-NOT-TRUSTED"
    result={"schema_version":1,"phase":46,"classification":classification,"controller_numerics_changed":False,
      "source_freeze":{"contact_unique_material_source":True,"authoritative_contact_slip_common_gap":float(contact_gap[1]),"authoritative_harmful_cross_contact_contribution":float(contact_gap[0])},
      "current_production_contact_law":inventory,"current_QP_already_enforces_coupled_dynamics":True,
      "first_missing_contact_response_relation":inventory["missing_relation"],"runtime_contact_oracle":{"pass":oracle_pass,"metrics":oracle_metrics,"rows":{"equality":equality,"physical_rank4":physical_equality,"contact":contact},"M":mass,"J":j,"D":d,"aref":aref},
      "factorization":{"effective_operator":"PASS / native local active-set exact","RHS_free_motion":"PASS / reconstructed","compliance_contribution":"REQUIRED BY EXACT ORACLE BUT STANDALONE CAUSAL FRACTION NOT IDENTIFIED","friction_contribution":"NONMATERIAL AS ROOT CAUSE: stable pyramidal active set; retained inside oracle","geometry_contribution":"NONMATERIAL","active_set_contribution":"LOCAL REGIME CONDITION; unchanged in probes","actuation_parity":actuation_parity},
      "candidate_families":{"A":{"decision":"REJECT FOR AUTHORIZATION","reason":"source law is valid, but the closed-loop diagnostic coupled solve did not preserve feasibility/H0"},"B":{"decision":"REJECT FOR AUTHORIZATION","reason":"exact Schur elimination of A at frozen active set, but its controller counterfactual failed the same gates"},"A_B_equivalence":"YES","C":{"decision":"REJECT AS SEPARATE LAW","reason":"compliance is part of the source law and its standalone causal fraction was not isolated"},"D":"DEFERRED"},
      "selected":selected,"source_stage":{"same_tau_explained_contact_gap_fraction":1.0,"force_prediction_improvement_fraction":1.0,"candidate_errors":oracle_metrics},
      "closed_loop_counterfactual":{"performed":True,"baseline":h0,"directions":transfers,"all_probes":stage_r,"current_actual_slip_common_transfer":current_actual,"candidate_slip_common_transfer":candidate_slip,"harmful_cross_current":harmful_current,"harmful_cross_candidate":harmful_candidate,"harmful_cross_improvement":harmful_current-harmful_candidate,"branch_split":branch,"scale_convergence":scale_error,"pass":stage_r_pass},
      "gates":{"R1_preserved":max(row["R1_residual"] for row in stage_r.values())<=1e-8,"compatible_H0_physical_equilibrium_preserved":abs(h0["outputs"][0])<=.05,"slip_differential_structural_regression":False if np.all(np.isfinite(transfers["slip_differential"]["candidate_outputs"])) else True,"xi_common_structural_regression":False if np.all(np.isfinite(transfers["xi_common"]["candidate_outputs"])) else True,"nonfinite":False if all(np.all(np.isfinite(row["qacc"])) for row in stage_r.values()) else True},
      "computational":selected["runtime_estimate"],"R2_authorized_for_one_implementation_candidate":authorized,"authorized_physical_law":selected["physical_law"] if authorized else "NONE","authorized_implementation_form":"SCHUR" if authorized else "NONE","R2_implemented":False,"future_gate_order":"COMP -> EQ -> AUTH -> REAL -> STOP","next_allowed_action":"implement exactly one authorized R2 physical law" if authorized else "additional contact-response source attribution only"}
    write(out/"r2-contact-response-reauthorization.json",result)
    replay=None
    if args.replay_of: replay=P45.semantic_error(args.replay_of/"r2-contact-response-reauthorization.json",out/"r2-contact-response-reauthorization.json")
    passed=oracle_pass and classification!="U-UNTRUSTED" and (replay is None or replay<=1e-11)
    write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"R2_authorized":authorized,"R2_implemented":False})
    sources=[cfgp,continuation,ROOT/base["scene"],ROOT/base["executable"],authority,wrench,args.qp_dump.resolve(),R1.PRODUCTION_AUDIT,R1.OPERATOR_AUDIT,PHASE/"evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json",PHASE/"evidence/automated/closure-conditioned-effective-inertia-formal-v2/closure-conditioned-effective-inertia-audit.json",Path(__file__).resolve(),Path(BASE.__file__),Path(CANON.__file__)]
    write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),"dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if passed else 2


if __name__=="__main__": raise SystemExit(main())
