#!/usr/bin/env python3
"""Phase46 legal equality-reaction recovery and corrected-R1 re-attribution."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
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
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


BASE = load(ROOT / "tools/experiments/run_phase46_post_corrected_r1_authority_attribution.py", "p46_legal_base")
AUTH, R1, P45C, P45, P44, P42 = BASE.AUTH, BASE.R1, BASE.P45C, BASE.P45, BASE.P44, BASE.P42
OUTPUTS = ("ddxi_common", "slip_common", "ddxi_differential", "slip_differential")


def encode(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: encode(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [encode(item) for item in value]
    return value


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(encode(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def vec(row: dict[str, str], name: str, n: int) -> np.ndarray:
    return np.asarray([float(row[f"{name}{i}"]) for i in range(n)])


def matrix(row: dict[str, str], name: str, nr: int, nc: int) -> np.ndarray:
    return np.asarray([[float(row[f"{name}{i}_{j}"]) for j in range(nc)] for i in range(nr)])


def basis(a: np.ndarray, tol: float = 1e-10) -> tuple[np.ndarray, np.ndarray]:
    u, s, _ = np.linalg.svd(a, full_matrices=False)
    return u[:, :int(np.sum(s > tol))], s


def projector(a: np.ndarray) -> tuple[np.ndarray, int, np.ndarray]:
    q, s = basis(a)
    return q @ q.T, q.shape[1], s


def spaces(jp: np.ndarray, jm: np.ndarray) -> dict[str, Any]:
    qp, sp = basis(jp.T); qm, sm = basis(jm.T)
    cos = np.linalg.svd(qp.T @ qm, compute_uv=False)
    angles = np.arccos(np.clip(cos, -1, 1))
    stacked = np.hstack((qp, qm))
    rank_union = int(np.linalg.matrix_rank(stacked, tol=1e-10))
    exact_intersection = qp.shape[1] + qm.shape[1] - rank_union
    uc, sc, vhc = np.linalg.svd(qp.T @ qm, full_matrices=False)
    common_dimension = int(np.sum(np.arccos(np.clip(sc, -1, 1)) <= 1e-2))
    common_p = qp @ uc[:, :common_dimension]
    common_m = qm @ vhc.T[:, :common_dimension]
    pcp, pcm = common_p @ common_p.T, common_m @ common_m.T
    pp, pm = qp @ qp.T, qm @ qm.T
    return {"rank_production": qp.shape[1], "rank_mujoco": qm.shape[1],
            "rank_union": rank_union, "exact_algebraic_intersection_dimension": exact_intersection,
            "operational_common_angle_tolerance_rad": 1e-2,
            "common_dimension": common_dimension,
            "production_only_dimension": qp.shape[1]-common_dimension,
            "mujoco_only_dimension": qm.shape[1]-common_dimension,
            "singular_values_production": sp, "singular_values_mujoco": sm,
            "principal_angles_rad": angles,
            "production_in_mujoco_containment": float(np.linalg.norm((np.eye(jp.shape[1])-pm) @ qp, 2)),
            "mujoco_in_production_containment": float(np.linalg.norm((np.eye(jp.shape[1])-pp) @ qm, 2)),
            "projectors": {"production": pp, "mujoco": pm,
                           "common_production": pcp, "common_mujoco": pcm,
                           "production_only": pp-pcp, "mujoco_only": pm-pcm}}


def production_terms(item: dict[str, Any]) -> dict[str, Any]:
    row = item["control"]
    sol = vec(row, "physical_solution", 42)
    n = matrix(row, "reduction_", 16, 12); c = vec(row, "reduction_bias", 16)
    j = matrix(row, "equality_jacobian_", 6, 16)
    mass = matrix(row, "full_mass_", 16, 16); bias = vec(row, "full_bias", 16)
    actuation = matrix(row, "full_actuation_", 16, 6)
    aw = [matrix(row, f"full_wrench_map_{side}_", 16, 6) for side in range(2)]
    qacc = n @ sol[:12] + c; tau = sol[12:18]
    contact = aw[0] @ sol[18:24] + aw[1] @ sol[24:30]
    raw = mass @ qacc + bias - actuation @ tau - contact
    p, rank, singular = projector(j.T)
    # M*qacc + h - B*tau - Qcontact - Qeq = 0, hence Qeq = P_range(JT)*rfull.
    legal = p @ raw
    return {"qacc": qacc, "tau": tau, "free_force": actuation @ tau,
            "contact_force": contact, "raw_residual": raw, "legal_equality_force": legal,
            "orthogonal_residual": (np.eye(16)-p) @ raw,
            "legal_range_residual": (np.eye(16)-p) @ legal,
            "lambda_minnorm": np.linalg.pinv(j.T, rcond=1e-12) @ legal,
            "reconstruction_residual": j.T @ np.linalg.pinv(j.T, rcond=1e-12) @ legal - legal,
            "J": j, "JdotV": vec(row, "equality_jdot_v", 6), "M": mass,
            "rank": rank, "singular_values": singular, "N": n, "c": c,
            "slack": sol[30:42]}


def derivative(probe: np.ndarray, base: np.ndarray, denominator: float) -> np.ndarray:
    return (np.asarray(probe)-np.asarray(base))/denominator


def y(obs: np.ndarray, mass: np.ndarray, force: np.ndarray) -> np.ndarray:
    return obs @ np.linalg.solve(mass, force)


def branch(item: dict[str, Any], baseline: dict[str, Any], denominator: float,
           geometry: dict[str, Any]) -> dict[str, Any]:
    p, b = production_terms(item), production_terms(baseline)
    obs, mmj = baseline["obs_map"], baseline["mass"]
    pro_free = y(obs, b["M"], derivative(p["free_force"], b["free_force"], denominator))
    pro_contact = y(obs, b["M"], derivative(p["contact_force"], b["contact_force"], denominator))
    pro_eq_force = derivative(p["legal_equality_force"], b["legal_equality_force"], denominator)
    pro_eq = y(obs, b["M"], pro_eq_force)
    qp_out = BASE.mode4(derivative(item["qp"], baseline["qp"], denominator))
    pro_remaining = qp_out-pro_free-pro_contact-pro_eq
    mj_free_force = derivative(item["forces"]["actuator"], baseline["forces"]["actuator"], denominator)
    mj_contact_force = derivative(item["solver_force_channels"]["generalized"]["contact"],
                                  baseline["solver_force_channels"]["generalized"]["contact"], denominator)
    mj_eq_force = derivative(item["solver_force_channels"]["generalized"]["equality"],
                             baseline["solver_force_channels"]["generalized"]["equality"], denominator)
    pcp = geometry["projectors"]["common_production"]
    pcm = geometry["projectors"]["common_mujoco"]
    pmjo = geometry["projectors"]["mujoco_only"]
    pp = geometry["projectors"]["production_only"]
    mj_free, mj_contact = y(obs, mmj, mj_free_force), y(obs, mmj, mj_contact_force)
    mj_common, mj_only = y(obs, mmj, pcm @ mj_eq_force), y(obs, mmj, pmjo @ mj_eq_force)
    mj_out = BASE.mode4(derivative(item["mj"], baseline["mj"], denominator))
    mj_remaining = mj_out-mj_free-mj_contact-mj_common-mj_only
    qp_common, qp_only = y(obs, b["M"], pcp @ pro_eq_force), y(obs, b["M"], pp @ pro_eq_force)
    return {"signed_delta": denominator, "QP_output": qp_out, "MJ_output": mj_out,
            "FREE_QP": pro_free, "FREE_MJ": mj_free,
            "QP_contact": pro_contact, "QP_legal_equality": pro_eq,
            "QP_common_equality": qp_common, "QP_production_only_equality": qp_only,
            "QP_remaining": pro_remaining, "MJ_contact": mj_contact,
            "MJ_common_equality": mj_common, "MJ_only_equality": mj_only,
            "MJ_remaining": mj_remaining, "contact_gap": mj_contact-pro_contact,
            "common_equality_gap": mj_common-qp_common,
            "total_discrepancy": mj_out-qp_out,
            "QP_source_closure_max_abs": float(np.max(np.abs(qp_out-pro_free-pro_contact-pro_eq-pro_remaining))),
            "MJ_source_closure_max_abs": float(np.max(np.abs(mj_out-mj_free-mj_contact-mj_common-mj_only-mj_remaining))),
            "production_recovery": {"raw_residual_norm": np.linalg.norm(p["raw_residual"]),
                "orthogonal_residual_norm": np.linalg.norm(p["orthogonal_residual"]),
                "legal_range_residual_norm": np.linalg.norm(p["legal_range_residual"]),
                "reconstruction_residual_norm": np.linalg.norm(p["reconstruction_residual"])},
            "R1": item["r1"], "regime": item["regime"]}


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(a)-np.asarray(b))/max(np.linalg.norm(a), 1e-12))


def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("--config", type=Path, default=AUTH.CONFIG)
    ap.add_argument("--qp-dump", type=Path, required=True); ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--replay-of", type=Path); args = ap.parse_args()
    out = args.output.resolve()
    if out.exists(): raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True); probes = out/"probes"; probes.mkdir()
    config_path = args.config.resolve(); config = json.loads(config_path.read_text())
    continuation_path = ROOT/config["continuation_config"]
    base, trim, wrench_source = P45C.frozen_inputs(json.loads(continuation_path.read_text()))
    base["executable"] = config["runtime_executable"]
    authority = ROOT/base["phase42_native_authority"]; native = P45.native_state(P44.read_csv(authority), 0)
    model = mujoco.MjModel.from_xml_path(str(ROOT/base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT/base["phase42_config"]).read_text()))
    production, operators = R1.read(R1.PRODUCTION_AUDIT), R1.read(R1.OPERATOR_AUDIT)
    baseline = BASE.capture(base, config, probes/"baseline.csv", authority, trim, native, model, oracle,
                            args.qp_dump.resolve(), production, operators, np.zeros(4))
    prod0 = production_terms(baseline)
    eq_mask = baseline["solver_force_channels"]["efc_type"] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    jmj = baseline["solver_force_channels"]["efc_J"][eq_mask]
    geom = spaces(prod0["J"], jmj)
    ac = np.hstack([matrix(baseline["control"], f"full_wrench_map_{side}_", 16, 6) for side in range(2)])
    contact_prod = spaces(prod0["J"], ac.T)
    contact_mj_j = baseline["solver_force_channels"]["efc_J"][
        baseline["solver_force_channels"]["efc_type"] >= int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS)]
    contact_mj = spaces(jmj, contact_mj_j)
    specs = (("slip_common", 2, np.ones(2)), ("slip_differential", 2, np.array([-1., 1.])),
             ("xi_common", 0, np.ones(2)), ("xi_differential", 0, np.array([-1., 1.])))
    amount, scales = float(config["delta_m_s2"]), list(map(float, config["delta_scales"]))
    rows = {}; items = []
    for name, start, direction in specs:
        for sign in (-1, 1):
            for scale in scales:
                task = np.zeros(4); task[start:start+2] = sign*scale*amount*direction
                item = BASE.capture(base, config, probes/f"{name}-{scale:g}-{sign:+d}.csv", authority, trim,
                                    native, model, oracle, args.qp_dump.resolve(), production, operators, task)
                row = branch(item, baseline, sign*scale*amount, geom)
                row.update(direction=name, branch=sign, scale=scale); rows[(name, sign, scale)] = row; items.append(row)
    keys = ("QP_output","MJ_output","FREE_QP","FREE_MJ","QP_contact","QP_legal_equality",
            "QP_common_equality","QP_production_only_equality","QP_remaining","MJ_contact",
            "MJ_common_equality","MJ_only_equality","MJ_remaining","contact_gap",
            "common_equality_gap","total_discrepancy")
    directions = {name: {key: .5*(rows[(name,-1,1.)][key]+rows[(name,1,1.)][key]) for key in keys}
                  for name,_,_ in specs}
    expected_index = {"xi_common":0, "slip_common":1, "xi_differential":2, "slip_differential":3}
    directional_metrics = {}
    for name, values in directions.items():
        index = expected_index[name]
        others = np.delete(values["total_discrepancy"], index)
        directional_metrics[name] = {
            "QP_self_gain": values["QP_output"][index], "MJ_self_gain": values["MJ_output"][index],
            "self_gain_gap": values["total_discrepancy"][index],
            "cross_contamination_norm": np.linalg.norm(others),
            "cross_to_self_gap_ratio": float(np.linalg.norm(others)/max(abs(values["total_discrepancy"][index]),1e-12)),
            "contact_gap_norm": np.linalg.norm(values["contact_gap"]),
            "common_equality_gap_norm": np.linalg.norm(values["common_equality_gap"])}
    branch_error = max(relative(rows[(n,-1,1.)]["total_discrepancy"], rows[(n,1,1.)]["total_discrepancy"])
                       for n,_,_ in specs)
    scale_error = max(relative(rows[(n,s,1.)]["total_discrepancy"], rows[(n,s,z)]["total_discrepancy"])
                      for n,_,_ in specs for s in (-1,1) for z in scales)
    range_max = max(x["production_recovery"]["legal_range_residual_norm"] for x in items)
    reconstruction_max = max(x["production_recovery"]["reconstruction_residual_norm"] for x in items)
    closure = max(max(x["QP_source_closure_max_abs"], x["MJ_source_closure_max_abs"]) for x in items)
    trusted = (range_max <= 1e-10 and reconstruction_max <= 1e-8 and closure <= 1e-10 and
               branch_error <= .05 and scale_error <= .05 and all(x["R1"]["pass"] and x["regime"]["stable"] for x in items))
    slip = directions["slip_common"]; disc = slip["total_discrepancy"]
    contact_fraction = float(np.linalg.norm(slip["contact_gap"])/max(np.linalg.norm(disc),1e-12))
    equality_fraction = float(np.linalg.norm(slip["common_equality_gap"])/max(np.linalg.norm(disc),1e-12))
    if not trusted: classification = "U-UNTRUSTED"
    elif equality_fraction >= .8: classification = "B-LEGAL-EQUALITY-REACTION-MISMATCH-REMAINS"
    elif contact_fraction >= .8: classification = "A-CONTACT-RESPONSE-MISMATCH-CONFIRMED"
    elif np.linalg.norm(slip["MJ_only_equality"])/max(np.linalg.norm(disc),1e-12) >= .8: classification = "C-MUJoco-ONLY-EQUALITY-MODE-DOMINATES"
    elif max(np.linalg.norm(slip["QP_remaining"]),np.linalg.norm(slip["MJ_remaining"]))/max(np.linalg.norm(disc),1e-12) >= .8: classification = "D-PRODUCTION-PASSIVE/OTHER-RESPONSE-MISMATCH"
    else: classification = "E-MIXED-REMAINING-MECHANISMS"
    result = {"schema_version":1,"phase":46,"scope":"legal equality reaction recovery + post-corrected-R1 authority re-decomposition",
              "old_QP_equality_reaction_status":"SUPERSEDED","old_equality_gap_status":"SUPERSEDED",
              "production_recovery_gate":{"pass":range_max<=1e-10 and reconstruction_max<=1e-8,
                  "legal_range_residual_max_norm":range_max,"reconstruction_residual_max_norm":reconstruction_max},
              "equality_force_spaces":{k:v for k,v in geom.items() if k!="projectors"},
              "contact_equality_overlap":{"production":{k:v for k,v in contact_prod.items() if k!="projectors"},
                                            "mujoco":{k:v for k,v in contact_mj.items() if k!="projectors"}},
              "production_J":prod0["J"],"mujoco_raw_equality_J":jmj,"outputs":OUTPUTS,
              "directions":directions,"all_probe_decompositions":items,
              "directional_metrics":directional_metrics,
              "trust":{"pass":trusted,"branch_split_relative":branch_error,"scale_convergence_relative":scale_error,
                       "maximum_source_closure":closure,"all_R1_pass":all(x["R1"]["pass"] for x in items),
                       "all_regimes_stable":all(x["regime"]["stable"] for x in items)},
              "slip_common":{"contact_gap_fraction":contact_fraction,"common_equality_gap_fraction":equality_fraction},
              "classification":classification,"corrected_R1_status":"CLOSED","explicit_lambda_controller_repair":"NO",
              "R2_authorized":False,"next_allowed_action":"evidence interpretation only" if trusted else "diagnostic implementation fix only"}
    write(out/"legal-equality-reaction-reattribution.json", result)
    replay = None if args.replay_of is None else P45.semantic_error(args.replay_of/"legal-equality-reaction-reattribution.json", out/"legal-equality-reaction-reattribution.json")
    write(out/"summary.json", {"pass":trusted and (replay is None or replay<=1e-11),"classification":classification,
          "replay_max_abs_error":replay,"replay_pass":replay is None or replay<=1e-11,"R2_authorized":False})
    sources=[config_path,continuation_path,ROOT/base["scene"],ROOT/base["executable"],authority,wrench_source,
             args.qp_dump.resolve(),R1.PRODUCTION_AUDIT,R1.OPERATOR_AUDIT,Path(__file__).resolve(),Path(BASE.__file__)]
    write(out/"manifest.json", {"created_utc":datetime.now(timezone.utc).isoformat(),"command":" ".join(sys.argv),
          "python":sys.version,"platform":platform.platform(),"dependencies":{"mujoco":mujoco.__version__,"numpy":np.__version__,"scipy":scipy.__version__},
          "sources":{str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if trusted and (replay is None or replay<=1e-11) else 2


if __name__ == "__main__": raise SystemExit(main())
