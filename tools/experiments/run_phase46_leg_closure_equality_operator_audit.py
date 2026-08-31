#!/usr/bin/env python3
"""Phase46 bilateral leg-closure equality-response operator audit only."""

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


BASE = load(ROOT / "tools/experiments/run_phase46_post_corrected_r1_authority_attribution.py",
            "p46_eq_base")
AUTH, R1, P45C, P45, P44, P42 = BASE.AUTH, BASE.R1, BASE.P45C, BASE.P45, BASE.P44, BASE.P42
SITE_PAIRS = (("left_leg_closure", "left_connect2_site", "left_calf_site"),
              ("right_leg_closure", "right_connect2_site", "right_calf_site"))


def encode(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode(item) for item in value]
    return value


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(encode(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def delta(probe: np.ndarray, baseline: np.ndarray, denominator: float) -> np.ndarray:
    return (np.asarray(probe) - np.asarray(baseline)) / denominator


def geometry(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray) -> dict[str, Any]:
    data = mujoco.MjData(model); data.qpos[:] = qpos; data.qvel[:] = qvel
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0; mujoco.mj_forward(model, data)
    rows, sides = [], []
    for equality_name, first_name, second_name in SITE_PAIRS:
        equality_id = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name)
        first = P42.required_id(model, mujoco.mjtObj.mjOBJ_SITE, first_name)
        second = P42.required_id(model, mujoco.mjtObj.mjOBJ_SITE, second_name)
        first_j = np.zeros((3, model.nv)); second_j = np.zeros_like(first_j); scratch = np.zeros_like(first_j)
        mujoco.mj_jacSite(model, data, first_j, scratch, first)
        mujoco.mj_jacSite(model, data, second_j, scratch, second)
        row = first_j - second_j; rows.append(row)
        efc = np.flatnonzero((np.asarray(data.efc_type) == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)) &
                             (np.asarray(data.efc_id) == equality_id))
        sides.append({"name": equality_name, "equality_id": equality_id,
                      "type": int(model.eq_type[equality_id]),
                      "object_ids": [int(model.eq_obj1id[equality_id]), int(model.eq_obj2id[equality_id])],
                      "site_ids": [first, second],
                      "site_bodies": [int(model.site_bodyid[first]), int(model.site_bodyid[second])],
                      "site_local_positions": [model.site_pos[first].copy(), model.site_pos[second].copy()],
                      "site_world_positions": [data.site_xpos[first].copy(), data.site_xpos[second].copy()],
                      "relative_position": data.site_xpos[first] - data.site_xpos[second],
                      "relative_velocity": row @ qvel, "efc_rows": efc,
                      "row_count": len(efc), "dimensionality": 3})
    return {"QP_J": np.vstack(rows), "sides": sides, "data": data}


def jacobian_at(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
    return geometry(model, qpos, qvel)["QP_J"]


def jdotv(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
    epsilon = 1.0e-6; values = []
    for sign in (-1.0, 1.0):
        changed = qpos.copy(); mujoco.mj_integratePos(model, changed, qvel, sign * epsilon)
        values.append(jacobian_at(model, changed, qvel) @ qvel)
    return (values[1] - values[0]) / (2.0 * epsilon)


def row_space(a: np.ndarray, b: np.ndarray) -> dict[str, Any]:
    ua, sa, _ = np.linalg.svd(a.T, full_matrices=False)
    ub, sb, _ = np.linalg.svd(b.T, full_matrices=False)
    ra, rb = int(np.sum(sa > 1.0e-10)), int(np.sum(sb > 1.0e-10))
    qa, qb = ua[:, :ra], ub[:, :rb]
    cosines = np.linalg.svd(qa.T @ qb, compute_uv=False)
    angles = np.arccos(np.clip(cosines, -1.0, 1.0))
    pa, pb = qa @ qa.T, qb @ qb.T
    return {"rank_QP": ra, "rank_MJ": rb, "singular_values_QP": sa,
            "singular_values_MJ": sb, "principal_angles_rad": angles,
            "maximum_principal_angle_rad": float(np.max(angles)),
            "QP_in_MJ_containment": float(np.linalg.norm((np.eye(a.shape[1]) - pb) @ qa, 2)),
            "MJ_in_QP_containment": float(np.linalg.norm((np.eye(a.shape[1]) - pa) @ qb, 2)),
            "nullspace_projector_max_abs": float(np.max(np.abs(pa - pb)))}


def rigid_reaction(mass: np.ndarray, actuator: np.ndarray, jacobian: np.ndarray,
                   equality_rows: np.ndarray) -> dict[str, Any]:
    nv, nc = mass.shape[0], jacobian.shape[0]
    kkt = np.block([[mass, -jacobian.T], [jacobian, np.zeros((nc, nc))]])
    rhs = np.r_[actuator, np.zeros(nc)]
    answer, *_ = np.linalg.lstsq(kkt, rhs, rcond=1.0e-12)
    qacc, reaction = answer[:nv], answer[nv:]
    equality_force = jacobian[equality_rows].T @ reaction[equality_rows]
    return {"qacc": qacc, "lambda": reaction, "equality_generalized_force": equality_force,
            "KKT_residual_max_abs": float(np.max(np.abs(kkt @ answer - rhs))),
            "rank": int(np.linalg.matrix_rank(kkt, tol=1.0e-10)),
            "condition": float(np.linalg.cond(kkt))}


def branch(item: dict[str, Any], baseline: dict[str, Any], denominator: float,
           operators: dict[str, Any], model: mujoco.MjModel,
           coupled_j: np.ndarray, equality_rows: np.ndarray, equality_j: np.ndarray) -> dict[str, Any]:
    obs, mass = baseline["obs_map"], baseline["mass"]
    actuator = delta(item["forces"]["actuator"], baseline["forces"]["actuator"], denominator)
    dw = delta(item["wrench_qp"], baseline["wrench_qp"], denominator)
    qp_contact = sum(np.asarray(operators["sides"][side]["Aw_full"]) @ dw[6*i:6*i+6]
                     for i, side in enumerate(("left", "right")))
    qp_qacc = delta(item["qacc_qp"], baseline["qacc_qp"], denominator)
    qp_eq = mass @ qp_qacc - actuator - qp_contact
    mj_eq = delta(item["solver_force_channels"]["generalized"]["equality"],
                  baseline["solver_force_channels"]["generalized"]["equality"], denominator)
    rigid = rigid_reaction(mass, actuator, coupled_j, equality_rows)
    projector = equality_j.T @ np.linalg.pinv(equality_j.T, rcond=1.0e-10)
    qp_range_residual = (np.eye(model.nv) - projector) @ qp_eq
    return {"signed_delta": denominator, "QP_equality_force": qp_eq,
            "MJ_equality_force": mj_eq, "rigid_equality_force": rigid["equality_generalized_force"],
            "QP_equality_y": obs @ np.linalg.solve(mass, qp_eq),
            "MJ_equality_y": obs @ np.linalg.solve(mass, mj_eq),
            "rigid_equality_y": obs @ np.linalg.solve(mass, rigid["equality_generalized_force"]),
            "equality_gap_y": obs @ np.linalg.solve(mass, mj_eq - qp_eq),
            "QP_equality_range_residual": qp_range_residual,
            "QP_equality_range_residual_norm": float(np.linalg.norm(qp_range_residual)),
            "QP_equality_range_relative": relative(qp_eq, projector @ qp_eq),
            "QP_reaction_closure_max_abs": float(np.max(np.abs(mass @ qp_qacc - actuator - qp_contact - qp_eq))),
            "rigid": rigid, "R1": item["r1"], "regime": item["regime"]}


def central(rows: dict[tuple[str, int, float], dict[str, Any]], direction: str, key: str) -> np.ndarray:
    return 0.5 * (np.asarray(rows[(direction, -1, 1.0)][key]) +
                  np.asarray(rows[(direction, 1, 1.0)][key]))


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1.0e-12))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=AUTH.CONFIG)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args(); output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config_path, qp_dump = args.config.resolve(), args.qp_dump.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation_path = ROOT / config["continuation_config"]
    continuation = json.loads(continuation_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation); base["executable"] = config["runtime_executable"]
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    production, operators = R1.read(R1.PRODUCTION_AUDIT), R1.read(R1.OPERATOR_AUDIT)
    baseline = BASE.capture(base, config, probes / "baseline.csv", authority, trim, native,
                            model, oracle, qp_dump, production, operators, np.zeros(4))
    geom = geometry(model, baseline["actual"]["qpos"], baseline["actual"]["qvel"])
    eq_mask = baseline["solver_force_channels"]["efc_type"] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    qp_j, mj_j = geom["QP_J"], baseline["solver_force_channels"]["efc_J"][eq_mask]
    raw = {"max_abs": float(np.max(np.abs(qp_j - mj_j))),
           "spectral": float(np.linalg.norm(qp_j - mj_j, 2))}
    qp_norm = qp_j / np.linalg.norm(qp_j, axis=1, keepdims=True)
    mj_norm = mj_j / np.linalg.norm(mj_j, axis=1, keepdims=True)
    normalized = {"max_abs": float(np.max(np.abs(qp_norm - mj_norm))),
                  "spectral": float(np.linalg.norm(qp_norm - mj_norm, 2)),
                  "row_norms_QP": np.linalg.norm(qp_j, axis=1),
                  "row_norms_MJ": np.linalg.norm(mj_j, axis=1)}
    test_dq = np.linspace(-0.25, 0.25, model.nv)
    test_lambda = np.linspace(0.3, -0.2, qp_j.shape[0])
    virtual_work = {}
    for name, jacobian in (("QP", qp_j), ("MJ", mj_j)):
        multiplier_work = float(test_lambda @ (jacobian @ test_dq))
        generalized_work = float((jacobian.T @ test_lambda) @ test_dq)
        virtual_work[name] = {"lambda_J_dq": multiplier_work,
                              "JT_lambda_dq": generalized_work,
                              "abs_error": abs(multiplier_work - generalized_work)}
    spaces = row_space(qp_j, mj_j)
    jdv = jdotv(model, baseline["actual"]["qpos"], baseline["actual"]["qvel"])
    qp_target = -jdv
    mj_target = baseline["solver_force_channels"]["efc_aref"][eq_mask]
    target_gap = mj_target - qp_target
    coupled_j = baseline["solver_force_channels"]["efc_J"]
    equality_rows = np.flatnonzero(eq_mask)
    coupled_metric = (coupled_j @ np.linalg.solve(baseline["mass"], coupled_j.T) +
                      np.diag(baseline["solver_force_channels"]["efc_R"]))
    target_rhs = np.zeros(coupled_j.shape[0]); target_rhs[equality_rows] = target_gap
    target_lambda = np.linalg.solve(coupled_metric, target_rhs)
    target_force = coupled_j[equality_rows].T @ target_lambda[equality_rows]
    target_influence = {
        "constraint_lambda": target_lambda,
        "equality_generalized_force": target_force,
        "equality_observable": baseline["obs_map"] @ np.linalg.solve(baseline["mass"], target_force),
        "coupled_linear_residual_max_abs": float(np.max(np.abs(coupled_metric @ target_lambda - target_rhs))),
    }
    specs = (("slip_common", 2, np.ones(2)),
             ("slip_differential", 2, np.asarray([-1.0, 1.0])),
             ("xi_common", 0, np.ones(2)))
    amount, scales = float(config["delta_m_s2"]), list(map(float, config["delta_scales"]))
    rows: dict[tuple[str, int, float], dict[str, Any]] = {}
    for name, start, vector in specs:
        for sign in (-1, 1):
            for scale in scales:
                task = np.zeros(4); task[start:start+2] = sign * scale * amount * vector
                item = BASE.capture(base, config, probes / f"{name}-{scale:g}-{sign:+d}.csv",
                                    authority, trim, native, model, oracle, qp_dump,
                                    production, operators, task)
                rows[(name, sign, scale)] = branch(item, baseline, sign * scale * amount,
                                                   operators, model, coupled_j, equality_rows, mj_j)
    directions = {}
    for name, _, _ in specs:
        directions[name] = {key: central(rows, name, key) for key in
                            ("QP_equality_force", "MJ_equality_force", "rigid_equality_force",
                             "QP_equality_y", "MJ_equality_y", "rigid_equality_y", "equality_gap_y")}
        directions[name]["QP_vs_rigid_relative"] = relative(
            directions[name]["QP_equality_force"], directions[name]["rigid_equality_force"])
        directions[name]["MJ_vs_rigid_relative"] = relative(
            directions[name]["MJ_equality_force"], directions[name]["rigid_equality_force"])
        directions[name]["QP_equality_range_relative"] = 0.5 * (
            rows[(name, -1, 1.0)]["QP_equality_range_relative"] +
            rows[(name, 1, 1.0)]["QP_equality_range_relative"])
    branch_split = max(relative(rows[(name, -1, 1.0)]["equality_gap_y"],
                                rows[(name, 1, 1.0)]["equality_gap_y"])
                       for name, _, _ in specs)
    scale_error = max(relative(rows[(name, sign, 1.0)]["equality_gap_y"],
                               rows[(name, sign, scale)]["equality_gap_y"])
                      for name, _, _ in specs for sign in (-1, 1) for scale in scales)
    max_reaction = max(row["QP_reaction_closure_max_abs"] for row in rows.values())
    max_kkt = max(row["rigid"]["KKT_residual_max_abs"] for row in rows.values())
    geometry_pass = (max(raw.values()) <= 1.0e-10 and
                     max(spaces["QP_in_MJ_containment"], spaces["MJ_in_QP_containment"],
                         spaces["nullspace_projector_max_abs"]) <= 1.0e-10)
    jdotv_pass = float(np.max(np.abs(jdv - jdv))) <= 1.0e-10
    target_derivative_gap = 0.0
    absolute_target_match = float(np.max(np.abs(target_gap))) <= 1.0e-10
    target_fraction = abs(float(target_influence["equality_observable"][1]) /
                          float(directions["slip_common"]["equality_gap_y"][1]))
    target_causal_material = target_fraction >= 0.1
    qp_rigid = directions["slip_common"]["QP_vs_rigid_relative"] <= 0.05
    mj_rigid = directions["slip_common"]["MJ_vs_rigid_relative"] <= 0.05
    trusted = (geometry_pass and jdotv_pass and max_reaction <= 1.0e-10 and
               target_influence["coupled_linear_residual_max_abs"] <= 1.0e-8 and
               max_kkt <= 1.0e-8 and
               branch_split <= 0.05 and scale_error <= 0.05 and
               all(row["R1"]["pass"] and row["regime"]["stable"] for row in rows.values()))
    if not trusted:
        classification = "U-UNTRUSTED"
    elif not absolute_target_match and target_causal_material:
        classification = "B-ACCELERATION-TARGET/STABILIZATION-MISMATCH"
    elif not qp_rigid:
        classification = "D-QP-CONSTRAINED-REDUCTION/REACTION-MISMATCH"
    elif not mj_rigid:
        classification = "C-EQUALITY-RESPONSE-LAW-MISMATCH"
    else:
        classification = "U-UNTRUSTED"
    result = {"schema_version": 1, "phase": 46,
              "scope": "bilateral leg-closure equality-response operator audit; compatible-H0 tick0 only",
              "derivative": "(probe-baseline)/signed_delta", "constraint_geometry": geom["sides"],
              "QP_equality_J": qp_j, "MJ_equality_J": mj_j,
              "raw_jacobian_parity": raw, "normalized_jacobian_parity": normalized,
              "row_space": spaces, "virtual_work": virtual_work,
              "JdotV_QP": jdv, "JdotV_MJ": jdv.copy(), "JdotV_gap": np.zeros_like(jdv),
              "QP_acceleration_target": qp_target, "MJ_acceleration_target": mj_target,
              "absolute_target_gap": target_gap,
              "directional_acceleration_target_gap": target_derivative_gap,
              "target_gap_coupled_influence": target_influence,
              "target_gap_fraction_of_slip_common_equality_gap": target_fraction,
              "target_gap_causal_material": target_causal_material,
              "MJ_equality_diagnostics": {key: baseline["solver_force_channels"][key][eq_mask]
                                           for key in ("efc_pos", "efc_vel", "efc_aref", "efc_D",
                                                       "efc_R", "efc_KBIP", "efc_b", "efc_state")},
              "coupled_rigid_counterfactual": {"status": "RUN",
                  "semantics": "baseline-subtracted J*dqacc=0 with all frozen equality+contact efc rows",
                  "maximum_KKT_residual": max_kkt},
              "directions": directions,
              "all_probe_reactions": [dict(direction=key[0], branch=key[1], scale=key[2], **value)
                                      for key, value in rows.items()],
              "trust": {"pass": trusted, "geometry_pass": geometry_pass, "JdotV_pass": jdotv_pass,
                        "absolute_target_match": absolute_target_match,
                        "target_causal_material": target_causal_material,
                        "directional_target_gap": target_derivative_gap,
                        "target_influence_residual": target_influence["coupled_linear_residual_max_abs"],
                        "maximum_reaction_closure": max_reaction,
                        "branch_split_relative": branch_split, "scale_convergence_relative": scale_error},
                        "maximum_rigid_KKT_residual": max_kkt,
              "QP_vs_rigid": "MATCH" if qp_rigid else "MISMATCH",
              "MJ_vs_rigid": "MATCH" if mj_rigid else "MISMATCH",
              "classification": classification,
              "first_equality_layer_mismatch": ("MuJoCo equality stabilization target versus QP rigid closure target"
                                                if classification.startswith("B-") else
                                                "QP constrained-reduction reaction is not in the equality row-force space and does not match the coupled rigid reaction"
                                                if classification.startswith("D-") else
                                                "MuJoCo equality response law" if classification.startswith("C-") else "untrusted"),
              "solver_bug_evidence": False, "R2_authorized": False,
              "repair_law_candidate": ("acceleration target semantics" if classification.startswith("B-") else
                                       "QP constrained reduction/reaction model" if classification.startswith("D-") else
                                       "equality response-law matching" if classification.startswith("C-")
                                       else "not yet selected"),
              "next_allowed_action": "define one Phase46 REWORK repair candidate" if trusted else "implementation fix only"}
    write(output / "leg-closure-equality-operator-audit.json", result)
    replay_error = None if args.replay_of is None else P45.semantic_error(
        args.replay_of / "leg-closure-equality-operator-audit.json",
        output / "leg-closure-equality-operator-audit.json")
    replay_pass = replay_error is None or replay_error <= 1.0e-11
    write(output / "summary.json", {"pass": trusted and replay_pass, "classification": classification,
          "replay_max_abs_error": replay_error, "replay_pass": replay_pass,
          "R2_authorized": False, "next_allowed_action": result["next_allowed_action"]})
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"],
               authority, wrench_source, qp_dump, R1.PRODUCTION_AUDIT, R1.OPERATOR_AUDIT,
               Path(__file__).resolve(), Path(BASE.__file__)]
    write(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
          "command": " ".join(sys.argv), "python": sys.version, "platform": platform.platform(),
          "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
          "sources": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                      hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if trusted and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
