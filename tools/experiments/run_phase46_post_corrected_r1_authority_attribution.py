#!/usr/bin/env python3
"""Phase46 post-corrected-R1 fixed-state authority attribution only."""

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
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


AUTH = load(ROOT / "tools/experiments/run_phase46_corrected_exact_r1_auth.py", "p46_pc_auth")
SENS = load(ROOT / "tools/experiments/run_phase46_contact_realization_sensitivity.py", "p46_pc_sens")
TAU = load(ROOT / "tools/experiments/run_phase46_torque_free_contact_attribution.py", "p46_pc_tau")
ROOT_CAUSE = load(ROOT / "tools/experiments/run_phase46_root_cause_closure.py", "p46_pc_root")
R1, P45C, P45, P44, P42 = AUTH.R1, AUTH.P45C, AUTH.P45, AUTH.P44, AUTH.P42
ACTUATORS = ("LH", "LK", "LW", "RH", "RK", "RW")
JOINTS = ("left_hip_joint", "left_knee_joint", "left_wheel_joint",
          "right_hip_joint", "right_knee_joint", "right_wheel_joint")
OUTPUTS = ("ddxi_common", "slip_common", "ddxi_differential", "slip_differential")


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


def mode4(per_side: np.ndarray) -> np.ndarray:
    return np.asarray([0.5 * (per_side[0] + per_side[1]),
                       0.5 * (per_side[2] + per_side[3]),
                       0.5 * (per_side[1] - per_side[0]),
                       0.5 * (per_side[3] - per_side[2])])


def observable_map(model: mujoco.MjModel, native: dict[str, str], oracle: Any,
                   actual: dict[str, Any], reduction: np.ndarray,
                   control: dict[str, str]) -> tuple[np.ndarray, dict[str, Any]]:
    qacc = P44.vec(actual["dynamics"], "qacc", model.nv)
    ddxi = np.asarray([actual["dynamics"]["ddxi_left_m_s2"],
                       actual["dynamics"]["ddxi_right_m_s2"]])
    xi, _ = P44.native_xi_acceleration_map(
        oracle, actual["qpos"], actual["qvel"], float(native["time_s"]), qacc, ddxi)
    data = mujoco.MjData(model)
    data.qpos[:] = actual["qpos"]; data.qvel[:] = actual["qvel"]
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0; mujoco.mj_forward(model, data)
    floor = P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    wheel_geoms = [P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in ("left_wheel_collision", "right_wheel_collision")]
    wheel_bodies = [P42.required_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                    for name in ("left_wheel_body", "right_wheel_body")]
    slip = np.zeros((2, model.nv)); frame_rows = []
    for side, (geom, body) in enumerate(zip(wheel_geoms, wheel_bodies)):
        contact = next(c for c in data.contact if {int(c.geom1), int(c.geom2)} == {geom, floor})
        normal = np.asarray(contact.frame).reshape(3, 3)[0].copy()
        if int(contact.geom2) != geom:
            normal *= -1.0
        normal /= np.linalg.norm(normal)
        tangent = np.asarray([1.0, 0.0, 0.0]); tangent -= normal * (tangent @ normal)
        tangent /= np.linalg.norm(tangent)
        lateral = np.cross(normal, tangent)
        point = np.asarray(contact.pos); lever = point - np.asarray(data.xpos[body])
        jp = np.zeros((3, model.nv)); jr = np.zeros_like(jp)
        mujoco.mj_jacBody(model, data, jp, jr, body)
        point_map = jp - ROOT_CAUSE.skew(lever) @ jr
        slip[side] = tangent @ point_map
        frame_rows.append({"side": side, "point": point, "tangent": tangent,
                           "lateral": lateral, "normal": normal, "map": point_map})
    qp_rolling = P44.matrix(control, "rolling_map_", 2, 12)
    parity = float(np.max(np.abs(slip @ reduction - qp_rolling)))
    per_side = np.vstack((xi[0], xi[1], slip[0], slip[1]))
    common_differential = np.vstack((0.5 * (per_side[0] + per_side[1]),
                                     0.5 * (per_side[2] + per_side[3]),
                                     0.5 * (per_side[1] - per_side[0]),
                                     0.5 * (per_side[3] - per_side[2])))
    return common_differential, {"rolling_map_parity_max_abs": parity,
                                 "contact_frames": frame_rows}


def solver_force_channels(oracle: Any) -> dict[str, Any]:
    data, model = oracle.data, oracle.model
    jacobian = np.asarray(data.efc_J).reshape(data.nefc, model.nv).copy()
    force = np.asarray(data.efc_force).copy()
    types = np.asarray(data.efc_type, dtype=int).copy()
    categories = {
        "equality": types == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY),
        "friction_loss": np.isin(types, [int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF),
                                          int(mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON)]),
        "joint_limit": np.isin(types, [int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT),
                                        int(mujoco.mjtConstraint.mjCNSTR_LIMIT_TENDON)]),
        "contact": types >= int(mujoco.mjtConstraint.mjCNSTR_CONTACT_FRICTIONLESS),
    }
    covered = np.logical_or.reduce(list(categories.values())) if len(types) else np.zeros(0, dtype=bool)
    categories["other"] = ~covered
    generalized = {name: jacobian[mask].T @ force[mask] for name, mask in categories.items()}
    generalized["total"] = jacobian.T @ force
    constraint = np.asarray(data.qfrc_constraint).copy()
    return {"efc_type": types, "efc_id": np.asarray(data.efc_id, dtype=int).copy(),
            "efc_J": jacobian, "efc_force": force, "generalized": generalized,
            "efc_pos": np.asarray(data.efc_pos).copy(),
            "efc_vel": np.asarray(data.efc_vel).copy(),
            "efc_aref": np.asarray(data.efc_aref).copy(),
            "efc_D": np.asarray(data.efc_D).copy(),
            "efc_R": np.asarray(data.efc_R).copy(),
            "efc_KBIP": np.asarray(data.efc_KBIP).copy(),
            "efc_b": np.asarray(data.efc_b).copy(),
            "efc_state": np.asarray(data.efc_state, dtype=int).copy(),
            "qfrc_constraint": constraint,
            "qfrc_smooth": np.asarray(data.qfrc_smooth).copy(),
            "xfrc_applied": np.asarray(data.xfrc_applied).copy(),
            "row_reconstruction_max_abs": float(np.max(np.abs(generalized["total"] - constraint)))}


def capture(base: dict[str, Any], config: dict[str, Any], path: Path, authority: Path,
            trim: np.ndarray, native: dict[str, str], model: mujoco.MjModel, oracle: Any,
            qp_dump: Path, production: dict[str, Any], operators: dict[str, Any],
            delta: np.ndarray) -> dict[str, Any]:
    item = AUTH.observe(base, config, path, authority, trim, native, model, oracle, qp_dump,
                        production, operators, None, None, delta)
    solver_channels = solver_force_channels(oracle)
    control, actual = item["control"], item["actual"]
    reduction = P44.matrix(control, "reduction_", model.nv, 12)
    qacc_qp = reduction @ P44.vec(control, "physical_solution", 12) + P44.vec(
        control, "reduction_bias", model.nv)
    mass = P44.vec(actual["dynamics"], "mass", model.nv ** 2).reshape(model.nv, model.nv)
    obs_map, map_evidence = observable_map(model, native, oracle, actual, reduction, control)
    geometry = SENS.ATTR.contact_geometry(
        model, actual["qpos"], actual["qvel"], float(oracle.config["canonical_wheel_radius_m"]))
    points, _ = SENS.point_forces(actual, geometry)
    point_values = TAU.point_array({"points": points})
    wrench = np.concatenate([
        np.asarray(production["sides"][name]["Gp_production"]) @ point_values[side].reshape(-1)
        for side, name in enumerate(("left", "right"))])
    actual_forces = SENS.ATTR.full_force_terms(actual, reduction)
    return {**item, "reduction": reduction, "qacc_qp": qacc_qp,
            "qacc_mj": P44.vec(actual["dynamics"], "qacc", model.nv), "mass": mass,
            "bias": P44.vec(actual["dynamics"], "qfrc_bias", model.nv),
            "obs_map": obs_map, "map_evidence": map_evidence,
            "tau": P44.vec(control, "tau", 6),
            "wrench_qp": P44.vec(control, "physical_solution", 30)[18:30],
            "wrench_mj": wrench, "points": points, "forces": actual_forces,
            "solver_force_channels": solver_channels,
            "active_constraints": ROOT_CAUSE.active_rows(control),
            "hard": float(control["hard"]), "slack": float(control["maximum_normalized_slack"]),
            "torque_margins": [float(control[f"tau_margin{i}"]) for i in range(6)]}


def gain(probe: np.ndarray, baseline: np.ndarray, denominator: float) -> np.ndarray:
    return (probe - baseline) / denominator


def metrics(discrepancy: np.ndarray, candidate: np.ndarray) -> dict[str, float]:
    denominator = max(float(discrepancy @ discrepancy), 1.0e-24)
    norm = max(float(np.linalg.norm(discrepancy)), 1.0e-12)
    return {"alpha": float(candidate @ discrepancy / denominator),
            "residual_ratio": float(np.linalg.norm(discrepancy - candidate) / norm),
            "norm_ratio": float(np.linalg.norm(candidate) / norm)}


def decompose(probe: dict[str, Any], baseline: dict[str, Any], denominator: float,
              production: dict[str, Any], operator_source: dict[str, Any],
              model: mujoco.MjModel, point_maps: list[dict[str, Any]]) -> dict[str, Any]:
    obs = baseline["obs_map"]
    delta_tau = gain(probe["tau"], baseline["tau"], denominator)
    free_qacc = np.linalg.solve(baseline["mass"],
                                gain(probe["forces"]["actuator"], baseline["forces"]["actuator"], denominator))
    delta_w_qp = gain(probe["wrench_qp"], baseline["wrench_qp"], denominator)
    qp_contact_force = sum(np.asarray(operator_source["sides"][side]["Aw_full"]) @
                           delta_w_qp[6 * index:6 * index + 6]
                           for index, side in enumerate(("left", "right")))
    qp_contact_qacc = np.linalg.solve(baseline["mass"], qp_contact_force)
    mj_contact_force = gain(probe["forces"]["contact"], baseline["forces"]["contact"], denominator)
    mj_contact_qacc = np.linalg.solve(baseline["mass"], mj_contact_force)
    qp_output = mode4(gain(probe["qp"], baseline["qp"], denominator))
    mj_output = mode4(gain(probe["mj"], baseline["mj"], denominator))
    free, qp_contact, mj_contact = obs @ free_qacc, obs @ qp_contact_qacc, obs @ mj_contact_qacc
    qp_other, mj_other = qp_output - free - qp_contact, mj_output - free - mj_contact
    contact_gap, other_gap = mj_contact - qp_contact, mj_other - qp_other
    discrepancy = mj_output - qp_output
    point_rows = []
    actual_points = gain(TAU.point_array(probe), TAU.point_array(baseline), denominator)
    qp_points = np.zeros_like(actual_points)
    for side, name in enumerate(("left", "right")):
        gp = np.asarray(production["sides"][name]["Gp_production"])
        qp_points[side] = (np.linalg.pinv(gp, rcond=1.0e-12) @
                           delta_w_qp[6 * side:6 * side + 6]).reshape(2, 3)
        error = (actual_points[side] - qp_points[side]).reshape(-1)
        row = np.linalg.pinv(gp, rcond=1.0e-12) @ gp
        aggregate, null = row @ error, (np.eye(6) - row) @ error
        point_rows.append({"side": name, "qp_representative_Fr_Fl_Fn": qp_points[side],
                           "actual_Fr_Fl_Fn": actual_points[side],
                           "aggregate_changing": aggregate.reshape(2, 3),
                           "nullspace_redistribution": null.reshape(2, 3),
                           "aggregate_norm": np.linalg.norm(aggregate), "null_norm": np.linalg.norm(null),
                           "null_wrench_max_abs": np.max(np.abs(gp @ null)),
                           "wrench_gap_closure_max_abs": np.max(np.abs(
                               gp @ error - (gain(probe["wrench_mj"], baseline["wrench_mj"], denominator)[6*side:6*side+6] -
                                             delta_w_qp[6*side:6*side+6])))} )
    actuator = []
    for label, joint in zip(ACTUATORS, JOINTS):
        dof = int(model.jnt_dofadr[P42.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)])
        force = np.zeros(model.nv)
        force[dof] = gain(probe["forces"]["actuator"], baseline["forces"]["actuator"], denominator)[dof]
        actuator.append({"actuator": label, "delta_tau_per_input": delta_tau[ACTUATORS.index(label)],
                         "free_y": obs @ np.linalg.solve(baseline["mass"], force)})
    point_acc = ROOT_CAUSE.point_acceleration(point_maps, free_qacc)
    point_summary = {}
    for axis, index in (("rolling_tangent", 0), ("lateral", 1), ("normal", 2)):
        side_means = np.mean(point_acc[:, :, index], axis=1)
        point_summary[axis] = {"per_side_mean": side_means,
                               "common": 0.5 * (side_means[0] + side_means[1]),
                               "differential": 0.5 * (side_means[1] - side_means[0])}
    closure = discrepancy - contact_gap - other_gap
    return {"outputs": OUTPUTS, "qp_output": qp_output, "mj_output": mj_output,
            "delta_tau": delta_tau, "delta_nudot_qp": gain(
                P44.vec(probe["control"], "physical_solution", 12),
                P44.vec(baseline["control"], "physical_solution", 12), denominator),
            "delta_wrench_qp_Fr_Fl_Fn_Mr_Ml_Mn": delta_w_qp,
            "free": free, "qp_contact": qp_contact, "mj_contact": mj_contact,
            "contact_gap": contact_gap, "qp_other": qp_other, "mj_other": mj_other,
            "other_gap": other_gap, "discrepancy": discrepancy,
            "qp_balance_closure_max_abs": np.max(np.abs(qp_output-free-qp_contact-qp_other)),
            "mj_balance_closure_max_abs": np.max(np.abs(mj_output-free-mj_contact-mj_other)),
            "gap_closure_max_abs": np.max(np.abs(closure)),
            "contact_gap_metrics": metrics(discrepancy, contact_gap),
            "other_gap_metrics": metrics(discrepancy, other_gap),
            "per_actuator_free": actuator,
            "free_contact_point_acceleration_RLN": point_acc,
            "free_contact_point_acceleration_modes": point_summary,
            "point_force_gap": point_rows,
            "solution_structure": {"active_constraints": probe["active_constraints"],
                                   "hard": probe["hard"], "slack": probe["slack"],
                                   "torque_margins": probe["torque_margins"]},
            "actual_reaction_opposes_free_slip": bool(np.all(free[[1, 3]] * mj_contact[[1, 3]] <= 0.0))}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=AUTH.CONFIG)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
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
    production, operator_source = R1.read(R1.PRODUCTION_AUDIT), R1.read(R1.OPERATOR_AUDIT)
    baseline = capture(base, config, probes / "baseline.csv", authority, trim, native, model,
                       oracle, qp_dump, production, operator_source, np.zeros(4))
    geometry = SENS.ATTR.contact_geometry(
        model, baseline["actual"]["qpos"], baseline["actual"]["qvel"],
        float(oracle.config["canonical_wheel_radius_m"]))
    point_maps = TAU.point_jacobians(model, native, geometry, baseline)
    delta = float(config["delta_m_s2"]); scales = list(map(float, config["delta_scales"]))
    specs = (("slip_common", 2, np.ones(2)),
             ("slip_differential", 2, np.asarray([-1.0, 1.0])),
             ("xi_common", 0, np.ones(2)))
    branches, all_items, captured_items = {}, [], []
    for name, start, direction in specs:
        for sign in (-1, 1):
            for scale in scales:
                task_delta = np.zeros(4); task_delta[start:start+2] = sign * scale * delta * direction
                item = capture(base, config, probes / f"{name}-{scale:g}-{sign:+d}.csv", authority,
                               trim, native, model, oracle, qp_dump, production, operator_source, task_delta)
                row = decompose(item, baseline, sign * scale * delta, production, operator_source,
                                model, point_maps)
                row.update({"direction": name, "branch": sign, "scale": scale,
                            "signed_delta": sign * scale * delta, "r1": item["r1"], "regime": item["regime"]})
                branches[(name, sign, scale)] = row; all_items.append(row); captured_items.append(item)
    directions = {}
    for name, _, _ in specs:
        directions[name] = {key: 0.5 * (np.asarray(branches[(name, -1, 1.0)][key]) +
                                         np.asarray(branches[(name, 1, 1.0)][key]))
                            for key in ("qp_output", "mj_output", "free", "qp_contact", "mj_contact",
                                        "contact_gap", "qp_other", "mj_other", "other_gap", "discrepancy")}
        directions[name]["contact_gap_metrics"] = metrics(directions[name]["discrepancy"], directions[name]["contact_gap"])
        directions[name]["other_gap_metrics"] = metrics(directions[name]["discrepancy"], directions[name]["other_gap"])
        directions[name]["representative_positive_branch"] = branches[(name, 1, 1.0)]
    max_closure = max(max(float(row[key]) for key in ("qp_balance_closure_max_abs",
                                                       "mj_balance_closure_max_abs", "gap_closure_max_abs"))
                      for row in all_items)
    parity_metrics = {
        "q_qdot_max_abs": max(max(float(np.max(np.abs(item["actual"][key] - baseline["actual"][key])))
                                   for key in ("qpos", "qvel")) for item in captured_items),
        "mass_bias_max_abs": max(max(float(np.max(np.abs(item[key] - baseline[key])))
                                      for key in ("mass", "bias")) for item in captured_items),
        "reduction_max_abs": max(float(np.max(np.abs(item["reduction"] - baseline["reduction"])))
                                  for item in captured_items),
        "observable_map_max_abs": max(float(np.max(np.abs(item["obs_map"] - baseline["obs_map"])))
                                       for item in captured_items),
        "rolling_map_internal_parity_max_abs": max(
            [baseline["map_evidence"]["rolling_map_parity_max_abs"]] +
            [item["map_evidence"]["rolling_map_parity_max_abs"] for item in captured_items]),
    }
    # capture() uses one frozen native state/model for every probe; the directly measured
    # matrices are recorded here rather than inferred from branch convergence.
    parity = {"q_qdot_identical": parity_metrics["q_qdot_max_abs"] <= 1.0e-12,
              "M_bias_identical": parity_metrics["mass_bias_max_abs"] <= 1.0e-12,
              "B_S_identical": parity_metrics["reduction_max_abs"] <= 1.0e-12,
              "Gp_Pg_identical": True, "point_jacobians_identical": True,
              "xi_slip_maps_identical": max(parity_metrics["observable_map_max_abs"],
                                              parity_metrics["rolling_map_internal_parity_max_abs"]) <= 1.0e-8,
              "contact_frame_solver_signature_stable": all(row["regime"]["stable"] for row in all_items),
              "r1_projector_range_point_full_reduced_virtual_work": all(row["r1"]["pass"] for row in all_items)}
    trusted = all(parity.values()) and max_closure <= 1.0e-8
    slip = directions["slip_common"]
    dominant = (slip["contact_gap_metrics"]["alpha"] >= 0.8 and
                slip["contact_gap_metrics"]["residual_ratio"] <= 0.25)
    classification = ("U-UNTRUSTED" if not trusted else
                      "A-CONTACT-RESPONSE-MISMATCH-AFTER-CORRECTED-R1" if dominant else
                      "B-OTHER-CONSTRAINT/PASSIVE-RESPONSE-MISMATCH" if
                      directions["slip_common"]["other_gap_metrics"]["alpha"] >= 0.8 else
                      "E-MULTIPLE-REMAINING-MECHANISMS")
    r2 = classification == "A-CONTACT-RESPONSE-MISMATCH-AFTER-CORRECTED-R1"
    result = {"schema_version": 1, "phase": 46,
              "scope": "post-corrected-R1 compatible-H0 tick0 fixed-state authority attribution only",
              "baseline_subtraction_definition": "(probe - baseline) / signed_delta",
              "outputs": OUTPUTS, "parity_gate": parity, "parity_metrics": parity_metrics,
              "directions": directions,
              "all_probe_decompositions": all_items, "maximum_causal_closure_max_abs": max_closure,
              "classification": classification, "R1_still_exactly_closed": parity["r1_projector_range_point_full_reduced_virtual_work"],
              "state_contact_regime": "STABLE" if parity["contact_frame_solver_signature_stable"] else "CHANGED",
              "contact_response_first_mismatch": r2,
              "solver_bug_evidence": False if trusted else None,
              "R2_authorized": r2,
              "next_repair_layer": "contact-response-consistent formulation" if r2 else "additional fixed-state attribution",
              "next_allowed_action": "define one Phase46 REWORK repair candidate" if r2 else "additional attribution only"}
    write(output / "post-corrected-r1-authority-attribution.json", result)
    replay_error = None if args.replay_of is None else P45.semantic_error(
        args.replay_of / "post-corrected-r1-authority-attribution.json",
        output / "post-corrected-r1-authority-attribution.json")
    write(output / "summary.json", {"pass": trusted, "classification": classification,
          "replay_max_abs_error": replay_error, "replay_pass": replay_error is None or replay_error <= 1.0e-11,
          "R2_authorized": r2, "next_allowed_action": result["next_allowed_action"]})
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"],
               authority, wrench_source, qp_dump, R1.PRODUCTION_AUDIT, R1.OPERATOR_AUDIT,
               Path(__file__).resolve(), Path(AUTH.__file__), Path(SENS.__file__), Path(TAU.__file__)]
    write(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
          "command": " ".join(sys.argv), "python": sys.version, "platform": platform.platform(),
          "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
          "sources": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                      hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if trusted and (replay_error is None or replay_error <= 1.0e-11) else 2


if __name__ == "__main__":
    raise SystemExit(main())
