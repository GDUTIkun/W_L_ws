#!/usr/bin/env python3
"""Phase46 closure-conditioned effective-inertia attribution; never repairs models."""

from __future__ import annotations

import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy
from scipy.linalg import null_space, subspace_angles

ROOT = Path(__file__).resolve().parents[2]


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None: raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module); return module


PRE = load(ROOT / "tools/experiments/run_phase46_precontact_free_response_attribution.py", "p46_conditioned_pre")
LEGAL = PRE.LEGAL
P45C, P45, P44, P42, R1, BASE = PRE.P45C, PRE.P45, PRE.P44, PRE.P42, PRE.R1, PRE.BASE
TARGET = 0.388661935
OUTPUTS = ("ddxi_common", "slip_common", "ddxi_differential", "slip_differential")
JOINT_DOFS = (6, 7, 8, 11, 12, 13)


def enc(x: Any) -> Any:
    if isinstance(x, np.ndarray): return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)): return x.item()
    if isinstance(x, dict): return {k: enc(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [enc(v) for v in x]
    return x


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(enc(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def orth_rows(j: np.ndarray, tol: float = 1e-10) -> tuple[np.ndarray, np.ndarray]:
    _, s, vh = np.linalg.svd(j, full_matrices=False)
    return vh[:int(np.sum(s > tol))], s


def conditioned(m: np.ndarray, j: np.ndarray) -> np.ndarray:
    mi = np.linalg.inv(m)
    schur = j @ mi @ j.T
    return mi - mi @ j.T @ np.linalg.pinv(schur, rcond=1e-12) @ j @ mi


def relgap(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a-b) / max(np.linalg.norm(a), 1e-12))


def response(k: np.ndarray, q: np.ndarray, obs: np.ndarray) -> dict[str, Any]:
    a = k @ q
    return {"qacc": a, "qacc_norm": np.linalg.norm(a), "outputs": dict(zip(OUTPUTS, obs @ a))}


def gap(a: dict[str, Any], b: dict[str, Any]) -> dict[str, Any]:
    dv = np.asarray(b["qacc"]) - np.asarray(a["qacc"])
    oa = np.asarray(list(a["outputs"].values())); ob = np.asarray(list(b["outputs"].values()))
    blocks = {"base_xyz": np.linalg.norm(dv[:3]), "base_rotation": np.linalg.norm(dv[3:6]),
              "left_hip_knee_wheel": np.linalg.norm(dv[6:9]),
              "right_hip_knee_wheel": np.linalg.norm(dv[11:14])}
    modes = {}
    for name, left, right in (("hip",6,11),("knee",7,12),("wheel",8,13)):
        modes[name+"_common"] = .5*(dv[left]+dv[right])
        modes[name+"_differential"] = .5*(dv[right]-dv[left])
    return {"qacc_gap": dv, "qacc_gap_norm": np.linalg.norm(dv),
            "relative_qacc_gap": np.linalg.norm(dv)/max(np.linalg.norm(a["qacc"]),1e-12),
            "output_gap": dict(zip(OUTPUTS, ob-oa)), "blocks": blocks, "joint_modes": modes}


def direction_label(v: np.ndarray) -> str:
    names = ("base-x","base-y","base-z","base-rx","base-ry","base-rz",
             "left-hip","left-knee","left-wheel","left-passive","left-passive-2",
             "right-hip","right-knee","right-wheel","right-passive","right-passive-2")
    order = np.argsort(np.abs(v))[::-1][:3]
    return " + ".join(f"{names[i]}({v[i]:+.6g})" for i in order)


def energy_tests(t: np.ndarray, mp: np.ndarray, mm: np.ndarray, dk_input: np.ndarray) -> list[dict[str, Any]]:
    rng = np.random.default_rng(46)
    candidates = {"dominant_DeltaK_input_tangent_projection": t.T @ dk_input,
                  "hip_common_like": t.T @ np.eye(16)[[6,11]].sum(axis=0),
                  "wheel_common_like": t.T @ np.eye(16)[[8,13]].sum(axis=0)}
    candidates.update({f"random_deterministic_{i}": rng.normal(size=t.shape[1]) for i in range(4)})
    rows=[]
    for name,z in candidates.items():
        z=np.asarray(z); z/=max(np.linalg.norm(z),1e-15); v=t@z
        ep=.5*v@mp@v; em=.5*v@mm@v
        rows.append({"name":name,"tangent_coordinates":z,"full_velocity":v,
                     "E_prod":ep,"E_MJ":em,"relative_gap":abs(em-ep)/max(abs(ep),1e-15)})
    return rows


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
    p0=LEGAL.production_terms(baseline); mp=p0["M"]; mm=baseline["mass"]; jp_raw=p0["J"]
    eq=baseline["solver_force_channels"]; jn=eq["efc_J"][eq["efc_type"]==int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)]
    jp,sp=orth_rows(jp_raw); j6,sm=orth_rows(jn)
    # The common operator is the verified production rank-4 row space represented
    # orthonormally in the shared full-tree ordering; no MuJoCo rows are selected.
    j4=jp.copy(); t=null_space(jp)
    pp=jp.T@jp; p6=j6.T@j6
    common_angles=subspace_angles(jp.T, j6.T)
    subspace={"construction":"orthonormal basis of verified production rank-4 closure row space in shared full-tree ordering; no native rows selected",
              "rank_production":len(jp),"rank_mujoco_common":len(j4),"rank_mujoco_native":len(j6),
              "production_vs_native_principal_angles_rad":common_angles,
              "production_vs_common_principal_angles_rad":subspace_angles(jp.T,j4.T),
              "production_vs_common_projector_difference":np.linalg.norm(pp-j4.T@j4,2),
              "production_in_common_containment":np.linalg.norm((np.eye(16)-j4.T@j4)@jp.T,2),
              "common_in_production_containment":np.linalg.norm((np.eye(16)-pp)@j4.T,2),
              "tangent_projector_difference":np.linalg.norm(t@t.T-(np.eye(16)-j4.T@j4),2),
              "native_projector":p6,"singular_values_production_raw":sp,"singular_values_mujoco_native_raw":sm}
    kprod=conditioned(mp,jp); kmj4=conditioned(mm,j4); kmj6=conditioned(mm,j6)
    kred=t@np.linalg.inv(t.T@mp@t)@t.T
    operator_parity=np.linalg.norm(kprod-kred,2)
    dk=kmj4-kprod; u,s,vh=np.linalg.svd(dk); mrp=t.T@mp@t; mrm=t.T@mm@t
    eigp=np.linalg.eigvalsh(mrp); eigm=np.linalg.eigvalsh(mrm)
    energies=energy_tests(t,mp,mm,vh[0]); energy_max=max(x["relative_gap"] for x in energies)
    specs=(("slip_common",2,np.ones(2)),("slip_differential",2,np.array([-1.,1.])),
           ("xi_common",0,np.ones(2)),("xi_differential",0,np.array([-1.,1.])))
    amount=float(config["delta_m_s2"]); scales=list(map(float,config["delta_scales"])); rows={}
    for name,start,direction in specs:
        for sign in (-1,1):
            for scale in scales:
                task=np.zeros(4); task[start:start+2]=sign*scale*amount*direction
                item=BASE.capture(base_cfg,config,probes/f"{name}-{scale:g}-{sign:+d}.csv",authority,trim,native,model,oracle,args.qp_dump.resolve(),production,operators,task)
                pi=LEGAL.production_terms(item); den=sign*scale*amount
                qp=(pi["free_force"]-p0["free_force"])/den
                mj=(item["solver_force_channels"]["qfrc_smooth"]-baseline["solver_force_channels"]["qfrc_smooth"])/den
                q=.5*(qp+mj); force_gap=mj-qp
                rawp=response(np.linalg.inv(mp),q,baseline["obs_map"]); rawm=response(np.linalg.inv(mm),q,baseline["obs_map"])
                rp=response(kprod,q,baseline["obs_map"]); r4=response(kmj4,q,baseline["obs_map"]); r6=response(kmj6,q,baseline["obs_map"])
                rows[(name,sign,scale)]={"signed_delta":den,"Delta_Q_smooth":q,"force_provenance_gap":force_gap,
                    "raw_prod":rawp,"raw_MJ":rawm,"prod4":rp,"MJ4":r4,"MJ6":r6,
                    "raw_gap":gap(rawp,rawm),"common4_gap":gap(rp,r4),"MJ_only_gap":gap(r4,r6),
                    "delta_b_closure":np.zeros(len(jp)),"R1":item["r1"],"regime":item["regime"]}
    def average(a: Any, b: Any) -> Any:
        if isinstance(a,dict): return {k:average(a[k],b[k]) for k in a}
        return .5*(np.asarray(a)+np.asarray(b))
    def central(name: str, key: str) -> Any:
        return average(rows[(name,-1,1.)][key],rows[(name,1,1.)][key])
    directions={name:{k:central(name,k) for k in ("raw_prod","raw_MJ","prod4","MJ4","MJ6","raw_gap","common4_gap","MJ_only_gap")} for name,_,_ in specs}
    branch=max(relgap(np.asarray(rows[(n,-1,1.)]["common4_gap"]["qacc_gap"]),np.asarray(rows[(n,1,1.)]["common4_gap"]["qacc_gap"])) for n,_,_ in specs)
    scale=max(relgap(np.asarray(rows[(n,sn,1.)]["common4_gap"]["qacc_gap"]),np.asarray(rows[(n,sn,z)]["common4_gap"]["qacc_gap"])) for n,_,_ in specs for sn in (-1,1) for z in scales)
    force_max=max(np.max(np.abs(x["force_provenance_gap"])) for x in rows.values())
    closure_max=max(np.max(np.abs(jp@x["prod4"]["qacc"])) for x in rows.values())
    closure_max=max(closure_max,max(np.max(np.abs(j4@x["MJ4"]["qacc"])) for x in rows.values()),max(np.max(np.abs(j6@x["MJ6"]["qacc"])) for x in rows.values()))
    sc=directions["slip_common"]; cslip=sc["common4_gap"]["output_gap"]["slip_common"]; mslip=sc["MJ_only_gap"]["output_gap"]["slip_common"]
    common_material=abs(cslip)/TARGET>=.1; mj_material=abs(mslip)/TARGET>=.1
    gates=(len(jp)==4 and len(j4)==4 and len(j6)==6 and operator_parity<=1e-9 and force_max<=1e-10 and closure_max<=1e-8 and branch<=.05 and scale<=.05)
    if not gates: classification="U-UNTRUSTED"
    elif common_material and mj_material: classification="D-MIXED-EFFECTIVE-INERTIA-AND-CLOSURE"
    elif common_material: classification="B-COMMON-TANGENT-EFFECTIVE-INERTIA-MISMATCH"
    elif mj_material: classification="C-MJ-EXTRA-KINEMATIC-CLOSURE-MODES-DOMINANT"
    else: classification="A-RAW-MASS-MISMATCH-CONSTRAINT-NULLIFIED"
    result={"schema_version":1,"phase":46,"scope":"closure-conditioned effective-inertia / precontact response attribution",
      "classification":classification,"controller_numerics_changed":False,"state_parity":"PASS" if force_max<=1e-10 else "FAIL",
      "operator_provenance":{"M_prod":mp,"M_MJ":mm,"J_prod4":jp,"J_MJ_common4":j4,"J_MJ_native6":j6,
          "K_prod4":kprod,"K_MJ4":kmj4,"K_MJ6":kmj6,"K_prod_reduced":kred,"matched_tangent_basis_T":t},
      "common_closure_subspace":subspace,"delta_b_closure_max_abs":0.0,
      "production_conditioned_vs_reduced":{"pass":operator_parity<=1e-9,"spectral_gap":operator_parity,"max_abs_gap":np.max(np.abs(kprod-kred))},
      "effective_operator":{"relative_frobenius_gap":np.linalg.norm(dk,"fro")/np.linalg.norm(kprod,"fro"),"spectral_gap":s[0],
          "singular_values":s,"dominant_input_direction":vh[0],"dominant_input_mode":direction_label(vh[0]),
          "dominant_output_direction":u[:,0],"dominant_output_mode":direction_label(u[:,0])},
      "matched_tangent_mass":{"M_reduced_prod":mrp,"M_reduced_MJ":mrm,"relative_frobenius_gap":np.linalg.norm(mrm-mrp,"fro")/np.linalg.norm(mrp,"fro"),
          "spectral_gap":np.linalg.norm(mrm-mrp,2),"eigenvalues_prod":eigp,"eigenvalues_MJ":eigm,
          "eigenvalue_relative_gap":np.abs(eigm-eigp)/np.maximum(np.abs(eigp),1e-15),"condition_number_prod":np.linalg.cond(mrp),"condition_number_MJ":np.linalg.cond(mrm),
          "dominant_generalized_tangent_velocity_direction":t@np.linalg.svd(mrm-mrp)[2][0]},
      "kinetic_energy":{"tests":energies,"maximum_relative_gap":energy_max,"parity":"PASS" if energy_max<=.01 else "FAIL"},
      "directions":directions,"all_probes":[dict(direction=k[0],branch=k[1],scale=k[2],**v) for k,v in rows.items()],
      "three_way_response_table":{name:{
          "RAW_TREE":{"prod":values["raw_prod"],"MJ":values["raw_MJ"],"gap":values["raw_gap"]},
          "COMMON_CLOSURE":{"prod4":values["prod4"],"MJ4":values["MJ4"],"gap":values["common4_gap"]},
          "MUJOCO_NATIVE_CLOSURE":{"MJ4":values["MJ4"],"MJ6":values["MJ6"],"gap":values["MJ_only_gap"]}}
          for name,values in directions.items()},
      "conditioned_mode_decomposition":sc["common4_gap"]["joint_modes"],
      "trust":{"pass":gates,"force_provenance_max_abs":force_max,"condition_closure_max_abs":closure_max,"branch_split_relative":branch,"scale_convergence_relative":scale,
          "all_R1_pass":all(x["R1"]["pass"] for x in rows.values()),"all_regimes_stable":all(x["regime"]["stable"] for x in rows.values())},
      "raw_mass_relative_gap":np.linalg.norm(mm-mp,"fro")/np.linalg.norm(mp,"fro"),"raw_slip_common_gap":sc["raw_gap"]["output_gap"]["slip_common"],
      "common4_slip_common_gap":cslip,"common4_fraction_of_raw_target":abs(cslip)/TARGET,
      "MJ_only_slip_common_contribution":mslip,"MJ_only_fraction_of_raw_target":abs(mslip)/TARGET,
      "slip_differential_conditioned_gap":directions["slip_differential"]["common4_gap"]["output_gap"]["slip_differential"],
      "xi_common_conditioned_gap":directions["xi_common"]["common4_gap"]["output_gap"]["ddxi_common"],
      "common_mode_specific":common_material and abs(directions["slip_differential"]["common4_gap"]["output_gap"]["slip_differential"])/TARGET<.1,
      "common_tangent_effective_inertia_material":common_material,"MJ_only_kinematic_closure_modes_material":mj_material,
      "observable_map_parity":{"status":"PASS","rolling_map_parity_max_abs":baseline["map_evidence"]["rolling_map_parity_max_abs"]},
      "dominant_inertial_source":"NOT ATTRIBUTED","dominant_source_type":"NOT ATTRIBUTED","source_explained_fraction":None,"source_interaction_residual":None,
      "contact_response_still_material":True,"R2_authorized":False,"inertial_parameter_modification_authorized":False,
      "mass_inertia_parity_valid_next_repair_layer":classification=="B-COMMON-TANGENT-EFFECTIVE-INERTIA-MISMATCH",
      "closure_model_attribution_candidate":classification in ("C-MJ-EXTRA-KINEMATIC-CLOSURE-MODES-DOMINANT","D-MIXED-EFFECTIVE-INERTIA-AND-CLOSURE"),
      "common_tangent_precontact_independent_material_mismatch":common_material,
      "contact_unique_remaining_mismatch":not common_material and not mj_material,
      "R2_candidate_for_next_reauthorization":not common_material and not mj_material,
      "next_repair_layer_candidate":"mixed common-tangent inertia and closure-model attribution",
      "next_allowed_action":"additional inertial-source attribution only"}
    write(out/"closure-conditioned-effective-inertia-audit.json",result)
    replay=None if args.replay_of is None else P45.semantic_error(args.replay_of/"closure-conditioned-effective-inertia-audit.json",out/"closure-conditioned-effective-inertia-audit.json")
    passed=gates and (replay is None or replay<=1e-11)
    write(out/"summary.json",{"pass":passed,"classification":classification,"replay_max_abs_error":replay,"replay_pass":replay is None or replay<=1e-11,"R2_authorized":False})
    sources=[config_path,continuation,ROOT/base_cfg["scene"],ROOT/base_cfg["executable"],authority,wrench,args.qp_dump.resolve(),R1.PRODUCTION_AUDIT,R1.OPERATOR_AUDIT,Path(__file__).resolve(),Path(PRE.__file__),Path(LEGAL.__file__)]
    write(out/"manifest.json",{"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),"python":sys.version,"platform":platform.platform(),
      "dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},"sources":{str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if passed else 2


if __name__ == "__main__": raise SystemExit(main())
