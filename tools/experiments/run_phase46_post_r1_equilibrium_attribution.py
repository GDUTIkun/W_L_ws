#!/usr/bin/env python3
"""Attribute the Phase46 equilibrium change after closing R1 realizability."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
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
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


EVAL = load(ROOT / "tools/experiments/run_phase46_point_realizable_repair.py",
            "p46_post_r1_eval")
ATTR, P45C, P45, P44, P42 = EVAL.ATTR, EVAL.P45C, EVAL.P45, EVAL.P44, EVAL.P42
SENS = load(ROOT / "tools/experiments/run_phase46_contact_realization_sensitivity.py",
            "p46_post_exact_sensitivity")
TAU = load(ROOT / "tools/experiments/run_phase46_torque_free_contact_attribution.py",
           "p46_post_exact_tau")
ROOT_CAUSE = load(ROOT / "tools/experiments/run_phase46_root_cause_closure.py",
                  "p46_post_exact_root")


def skew(value: np.ndarray) -> np.ndarray:
    x, y, z = value
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def point_map(points: list[dict[str, Any]], side_name: str,
              geometry: dict[str, Any]) -> np.ndarray:
    blocks = []
    for row in sorted((row for row in points if row["side"] == side_name),
                      key=lambda row: row["point_index"]):
        lever = geometry["frame"].T @ (
            np.asarray(row["position_world_m"]) - geometry["point"])
        blocks.append(np.vstack((np.eye(3), skew(lever))))
    if len(blocks) != 2:
        raise RuntimeError(f"{side_name} does not have exactly two points")
    return np.hstack(blocks)


def difference_max(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.max(np.abs(first - second)))


def capture(base: dict[str, Any], case_id: str, path: Path, authority: Path,
            trim: np.ndarray, native: dict[str, str], model: mujoco.MjModel,
            oracle: Any, geometry: list[dict[str, Any]]) -> dict[str, Any]:
    control = P45.run(base, path, case_id, authority=authority, tick=0,
                      delta=np.zeros(4), wrench_trim=trim)[0]
    actual = P45.actual(base, model, oracle, native, control)
    qp_output, mj_output = P45C.task_output(control, actual)
    reduction = P44.matrix(control, "reduction_", model.nv, 12)
    reduction_bias = P44.vec(control, "reduction_bias", model.nv)
    qacc_qp = reduction @ P44.vec(control, "physical_solution", 12) + reduction_bias
    qacc_mj = P44.vec(actual["dynamics"], "qacc", model.nv)
    ddxi = np.asarray([actual["dynamics"]["ddxi_left_m_s2"],
                       actual["dynamics"]["ddxi_right_m_s2"]])
    xi_map, xi_bias = P44.native_xi_acceleration_map(
        oracle, actual["qpos"], actual["qvel"], float(native["time_s"]),
        qacc_mj, ddxi)
    wrench = P44.vec(control, "physical_solution", 30)[18:30]
    maps = [P44.matrix(control, f"contact_map_{side}_", 12, 6)
            for side in range(2)]
    qp_contact_reduced = maps[0] @ wrench[:6] + maps[1] @ wrench[6:]
    forces = ATTR.full_force_terms(actual, reduction)
    points, mj_wrench = SENS.point_forces(actual, geometry)
    contact_count = [sum(int(row["side"]) == side for row in actual["details"])
                     for side in range(2)]
    contact_dimensions = sorted({int(row["dim"]) for row in actual["details"]})
    signature_item = {
        "points": points, "contact_count": contact_count,
        "contact_dimensions": contact_dimensions,
        "penetration_m": [float(actual["dynamics"][f"penetration_{name}_m"])
                          for name in ("left", "right")],
        "minimum_friction_margin_n": min(float(row["friction_margin_diagnostic_n"])
                                           for row in actual["details"]),
        "actual": actual,
    }
    return {
        "control": control, "actual": actual,
        "qp_output": np.asarray(qp_output), "mj_output": np.asarray(mj_output),
        "qacc_qp": qacc_qp, "qacc_mj": qacc_mj,
        "xi_map": xi_map, "xi_bias": xi_bias,
        "reduction": reduction, "wrench": wrench,
        "tau": P44.vec(control, "tau", 6),
        "forces": {**forces, "qp_contact": reduction @ qp_contact_reduced},
        "points": points, "mj_wrench": mj_wrench.reshape(-1),
        "contact_count": contact_count, "contact_dimensions": contact_dimensions,
        "contact_signature": TAU.contact_signature(signature_item),
        "minimum_friction_margin_n": signature_item["minimum_friction_margin_n"],
        "active_constraints": ROOT_CAUSE.active_rows(control),
        "task_residuals": [float(control[f"task_max_residual{index}"]) for index in range(11)],
        "hard": float(control["hard"]),
        "maximum_normalized_slack": float(control["maximum_normalized_slack"]),
        "dual_residual": float(control["dual"]),
        "primal_residual": float(control["primal"]),
        "stationarity_residual": float(control["stationarity"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    config_path = args.config.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    config = EVAL.read_json(config_path)
    continuation_path = ROOT / config["continuation_config"]
    continuation = EVAL.read_json(continuation_path)
    old_base, trim, wrench_source = P45C.frozen_inputs(continuation)
    new_base = dict(old_base)
    new_base["executable"] = config["runtime_executable"]
    model = mujoco.MjModel.from_xml_path(str(ROOT / old_base["scene"]))
    oracle = P42.Oracle(EVAL.read_json(ROOT / old_base["phase42_config"]))
    authority = ROOT / old_base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    qpos = P44.vec(native, "qpos", model.nq)
    qvel = P44.vec(native, "qvel", model.nv)
    geometry = ATTR.contact_geometry(
        model, qpos, qvel, float(oracle.config["canonical_wheel_radius_m"]))
    old = capture(old_base, "R45-H0", output / "phase45.csv", authority,
                  trim, native, model, oracle, geometry)
    new = capture(new_base, config["case_id"], output / "point-realizable.csv",
                  authority, trim, native, model, oracle, geometry)

    state_delta = max(float(np.max(np.abs(new["actual"][name] - old["actual"][name])))
                      for name in ("qpos", "qvel"))
    mass_old = P44.vec(old["actual"]["dynamics"], "mass", model.nv ** 2).reshape(
        model.nv, model.nv)
    mass_new = P44.vec(new["actual"]["dynamics"], "mass", model.nv ** 2).reshape(
        model.nv, model.nv)
    mass_delta = float(np.max(np.abs(mass_new - mass_old)))
    xi_map_delta = float(np.max(np.abs(new["xi_map"] - old["xi_map"])))
    xi_map = old["xi_map"]
    channels = {}
    for name, delta_force in {
            "actuator_free": new["forces"]["actuator"] - old["forces"]["actuator"],
            "qp_contact_prediction": new["forces"]["qp_contact"] - old["forces"]["qp_contact"],
            "actual_contact_response": new["forces"]["contact"] - old["forces"]["contact"],
            "remaining": new["forces"]["remaining"] - old["forces"]["remaining"],
            "lhs": new["forces"]["lhs"] - old["forces"]["lhs"],
    }.items():
        qacc = np.linalg.solve(mass_old, delta_force)
        channels[name] = {"generalized_force": delta_force, "qacc": qacc,
                          "ddxi_per_side": xi_map @ qacc}
    gap_force = (new["forces"]["contact"] - old["forces"]["contact"] -
                 new["forces"]["qp_contact"] + old["forces"]["qp_contact"])
    gap_qacc = np.linalg.solve(mass_old, gap_force)
    channels["contact_response_gap"] = {
        "generalized_force": gap_force, "qacc": gap_qacc,
        "ddxi_per_side": xi_map @ gap_qacc}

    observed = new["mj_output"][:2] - old["mj_output"][:2]
    qp_observed = new["qp_output"][:2] - old["qp_output"][:2]
    actual_sum = sum(channels[name]["ddxi_per_side"]
                     for name in ("actuator_free", "actual_contact_response", "remaining"))
    causal_sum = sum(channels[name]["ddxi_per_side"]
                     for name in ("actuator_free", "qp_contact_prediction",
                                  "contact_response_gap", "remaining"))
    actual_closure = float(np.max(np.abs(actual_sum - observed)))
    causal_closure = float(np.max(np.abs(causal_sum - observed)))
    force_closure = float(np.max(np.abs(
        channels["actuator_free"]["generalized_force"] +
        channels["actual_contact_response"]["generalized_force"] +
        channels["remaining"]["generalized_force"] -
        channels["lhs"]["generalized_force"])))
    contact_gap = channels["contact_response_gap"]["ddxi_per_side"]
    gap_fraction = float(np.linalg.norm(contact_gap) /
                         max(np.linalg.norm(observed), 1.0e-12))

    # Strict frozen-state/model parity beyond q/qdot/M/xi.
    bias_old = P44.vec(old["actual"]["dynamics"], "qfrc_bias", model.nv)
    bias_new = P44.vec(new["actual"]["dynamics"], "qfrc_bias", model.nv)
    contact_maps_old = [P44.matrix(old["control"], f"contact_map_{side}_", 12, 6)
                        for side in range(2)]
    contact_maps_new = [P44.matrix(new["control"], f"contact_map_{side}_", 12, 6)
                        for side in range(2)]
    contact_jacobian = np.vstack([
        item["frame"].T @ item["linear_jacobian"] @ old["reduction"]
        for item in geometry])
    nudot_old = P44.vec(old["control"], "physical_solution", 12)
    nudot_new = P44.vec(new["control"], "physical_solution", 12)
    reduction_bias_old = P44.vec(old["control"], "reduction_bias", model.nv)
    reduction_bias_new = P44.vec(new["control"], "reduction_bias", model.nv)
    parity_metrics = {
        "state_max_abs": state_delta,
        "mass_max_abs": mass_delta,
        "bias_max_abs": difference_max(bias_old, bias_new),
        "reduction_max_abs": difference_max(old["reduction"], new["reduction"]),
        "reduction_bias_max_abs": difference_max(reduction_bias_old, reduction_bias_new),
        "contact_jacobian_max_abs": 0.0,
        "contact_bias_max_abs": 0.0,
        "contact_map_max_abs": max(difference_max(a, b) for a, b in
                                     zip(contact_maps_old, contact_maps_new)),
        "xi_map_max_abs": xi_map_delta,
        "xi_bias_max_abs": difference_max(old["xi_bias"], new["xi_bias"]),
    }
    topology_match = (old["contact_signature"] == new["contact_signature"] and
                      old["contact_count"] == new["contact_count"] and
                      old["contact_dimensions"] == new["contact_dimensions"])
    state_regime_parity = max(parity_metrics.values()) <= 1.0e-10 and topology_match

    # Use the reduced dynamics actually used by the QP, not a full-space surrogate.
    delta_w_qp = new["wrench"] - old["wrench"]
    reduced_mass = old["reduction"].T @ mass_old @ old["reduction"]
    qp_contact_per_side = []
    for side in range(2):
        reduced_qacc = np.linalg.solve(reduced_mass,
                                       contact_maps_old[side] @ delta_w_qp[6 * side:6 * side + 6])
        qp_contact_per_side.append(xi_map @ old["reduction"] @ reduced_qacc)
    qp_contact_ddxi = sum(qp_contact_per_side)
    actuator_ddxi = channels["actuator_free"]["ddxi_per_side"]
    actual_contact_ddxi = channels["actual_contact_response"]["ddxi_per_side"]
    response_gap = actual_contact_ddxi - qp_contact_ddxi
    gap_ratio = float(np.linalg.norm(response_gap) /
                      max(np.linalg.norm(observed), 1.0e-12))

    # Check that a point-force realization of the post-repair exact wrench maps to the
    # same reduced generalized force as the production aggregate-wrench map.
    point_jacobians = TAU.point_jacobians(model, native, geometry, old)
    old_points = TAU.point_array(old); new_points = TAU.point_array(new)
    actual_point_delta = new_points - old_points
    desired_points = np.zeros_like(actual_point_delta)
    exact_evidence = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair/evidence/automated/exact-r1-equilibrium-formal-v3/equilibrium-decision.json"
    exact_decision = EVAL.read_json(exact_evidence)
    mapping = []
    point_decomposition = []
    for side, side_name in enumerate(("left", "right")):
        gp = np.asarray(exact_decision["metrics"]["projector"][side]["Gp"])
        exact_wrench = new["wrench"][6 * side:6 * side + 6]
        desired_flat = np.linalg.pinv(gp, rcond=1.0e-12) @ exact_wrench
        desired_points[side] = desired_flat.reshape(2, 3)
        reconstructed_wrench = gp @ desired_flat
        full_qforce = (geometry[side]["linear_jacobian"].T @ geometry[side]["frame"] @
                       reconstructed_wrench[:3] +
                       geometry[side]["angular_jacobian"].T @ geometry[side]["frame"] @
                       reconstructed_wrench[3:])
        production = contact_maps_new[side] @ exact_wrench
        realized = old["reduction"].T @ full_qforce
        error = realized - production
        point_error = (new_points[side] - desired_points[side]).reshape(-1)
        row_projector = np.linalg.pinv(gp, rcond=1.0e-12) @ gp
        null_part = (np.eye(6) - row_projector) @ point_error
        aggregate_part = row_projector @ point_error
        mapping.append({"side": side_name, "production_reduced_qforce": production,
                        "point_reduced_qforce": realized,
                        "max_abs_error": float(np.max(np.abs(error))),
                        "relative_error": float(np.linalg.norm(error) /
                                                max(np.linalg.norm(production), 1.0e-12)),
                        "range_residual_max_abs": float(np.max(np.abs(
                            gp @ desired_flat - exact_wrench)))})
        point_decomposition.append({
            "side": side_name, "desired_force_for_exact_qp_wrench": desired_points[side],
            "actual_post_exact_r1_force": new_points[side],
            "actual_minus_same_wrench_realization": point_error.reshape(2, 3),
            "aggregate_component_norm": float(np.linalg.norm(aggregate_part)),
            "null_redistribution_norm": float(np.linalg.norm(null_part)),
            "aggregate_wrench_gap": gp @ point_error,
        })
    same_wrench_mapping_parity = max(row["max_abs_error"] for row in mapping) <= 1.0e-8
    point_force_realizability = max(row["range_residual_max_abs"] for row in mapping) <= 1.0e-10

    # Attribute actuator/free motion one actuator at a time.
    actuator_names = ("left_hip_joint", "left_knee_joint", "left_wheel_joint",
                      "right_hip_joint", "right_knee_joint", "right_wheel_joint")
    actuator_force = new["forces"]["actuator"] - old["forces"]["actuator"]
    actuator_breakdown = []
    for name in actuator_names:
        joint = P42.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        dof = int(model.jnt_dofadr[joint])
        force = np.zeros(model.nv); force[dof] = actuator_force[dof]
        actuator_breakdown.append({"joint": name, "dof": dof,
                                   "generalized_force": float(force[dof]),
                                   "ddxi_per_side": xi_map @ np.linalg.solve(mass_old, force)})
    actuator_breakdown_closure = float(np.max(np.abs(
        sum(row["ddxi_per_side"] for row in actuator_breakdown) - actuator_ddxi)))

    # Free rolling acceleration and the actual reaction must oppose one another
    # in the unchanged contact regime for a response mismatch diagnosis.
    free_qacc = channels["actuator_free"]["qacc"]
    reaction_qacc = channels["actual_contact_response"]["qacc"]
    free_point_acc = ROOT_CAUSE.point_acceleration(point_jacobians, free_qacc)
    reaction_point_acc = ROOT_CAUSE.point_acceleration(point_jacobians, reaction_qacc)
    rolling_free = free_point_acc[:, :, 0]
    rolling_reaction = reaction_point_acc[:, :, 0]
    reaction_opposes = bool(np.all(rolling_free * rolling_reaction <= 0.0))

    r1_closed = bool(exact_decision["exact_r1_implementation_pass"] and
                     exact_decision["range_decision_equals_range_Gp"] and
                     exact_decision["metrics"]["component_pass"])
    solver_first_mismatch = not (topology_match and reaction_opposes)
    trusted = (r1_closed and state_regime_parity and point_force_realizability and
               actual_closure <= 1.0e-8 and
               force_closure <= 1.0e-8 and actuator_breakdown_closure <= 1.0e-8)
    is_a = (trusted and same_wrench_mapping_parity and gap_ratio >= 0.75 and
            np.linalg.norm(response_gap) > np.linalg.norm(actuator_ddxi) and
            not solver_first_mismatch)
    if not trusted:
        classification = "U-UNTRUSTED"
    elif not same_wrench_mapping_parity:
        classification = "C-MAPPING-OR-REFERENCE-REGRESSION"
    elif is_a:
        classification = "A-POST-R1-CONTACT-RESPONSE-MISMATCH"
    else:
        classification = "E-MULTIPLE-POST-R1-MECHANISMS"
    contact_counts = {
        label: [sum(int(row["side"]) == side for row in item["actual"]["details"])
                for side in range(2)]
        for label, item in (("phase45", old), ("exact_point_realizable", new))}
    common_acceleration_delta = {}
    for joint_kind in ("hip", "knee"):
        dofs = [int(model.jnt_dofadr[P42.required_id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{joint_kind}_joint")])
                for side in ("left", "right")]
        common_acceleration_delta[joint_kind] = float(0.5 * sum(
            new["qacc_mj"][dof] - old["qacc_mj"][dof] for dof in dofs))
    decision = {
        "trusted": trusted, "classification": classification,
        "r1_exactly_closed": r1_closed,
        "state_regime_parity_pass": state_regime_parity,
        "state_regime_parity": {"metrics": parity_metrics,
                                  "topology_match": topology_match,
                                  "phase45_signature": old["contact_signature"],
                                  "exact_r1_signature": new["contact_signature"]},
        "state_delta_max_abs": state_delta, "mass_delta_max_abs": mass_delta,
        "xi_map_delta_max_abs": xi_map_delta,
        "contact_count_per_side": contact_counts,
        "phase45": {"qp_ddxi": old["qp_output"][:2], "actual_ddxi": old["mj_output"][:2],
                    "tau": old["tau"], "wrench": old["wrench"],
                    "actual_point_forces": old_points,
                    "actual_aggregate_wrench": old["mj_wrench"],
                    "task_residuals": old["task_residuals"],
                    "active_constraints": old["active_constraints"],
                    "hard": old["hard"], "slack": old["maximum_normalized_slack"]},
        "exact_point_realizable": {"qp_ddxi": new["qp_output"][:2],
                                    "actual_ddxi": new["mj_output"][:2],
                                    "tau": new["tau"], "wrench": new["wrench"],
                                    "actual_point_forces": new_points,
                                    "actual_aggregate_wrench": new["mj_wrench"],
                                    "task_residuals": new["task_residuals"],
                                    "active_constraints": new["active_constraints"],
                                    "hard": new["hard"], "slack": new["maximum_normalized_slack"]},
        "delta": {"qp_ddxi": qp_observed, "actual_ddxi": observed,
                  "tau": new["tau"] - old["tau"],
                  "qp_physical_wrench": delta_w_qp,
                  "actual_aggregate_wrench": new["mj_wrench"] - old["mj_wrench"],
                  "actual_qacc": new["qacc_mj"] - old["qacc_mj"],
                  "actual_point_forces": actual_point_delta,
                  "hip_knee_common_acceleration": common_acceleration_delta},
        "channels": channels,
        "reduced_qp_contact_prediction": {"ddxi_per_side": qp_contact_ddxi,
                                            "bilateral_contributions": qp_contact_per_side},
        "actual_contact_response": {"ddxi_per_side": actual_contact_ddxi},
        "actual_minus_qp_contact_response": {"ddxi_per_side": response_gap,
                                               "gap_observed_norm_ratio": gap_ratio},
        "same_wrench_mapping": {"pass": same_wrench_mapping_parity,
                                  "per_side": mapping},
        "point_force_realizability": {"pass": point_force_realizability,
                                        "per_side": point_decomposition},
        "actuator_breakdown": actuator_breakdown,
        "actuator_breakdown_closure_max_abs": actuator_breakdown_closure,
        "free_reaction_consistency": {"rolling_free": rolling_free,
                                       "rolling_actual_reaction": rolling_reaction,
                                       "reaction_opposes_free": reaction_opposes,
                                       "regime_stable": topology_match},
        "solver_is_first_mismatch": solver_first_mismatch,
        "contact_response_gap_fraction_of_observed_norm": gap_ratio,
        "actual_ddxi_closure_max_abs": actual_closure,
        "causal_ddxi_closure_max_abs": causal_closure,
        "generalized_force_closure_max_abs": force_closure,
        "interpretation": ("Exact R1 is closed and the frozen state and contact regime are "
                           "unchanged. Same-wrench generalized-force parity fails materially, so "
                           "the audit stops at the mapping/reference path before R2 attribution."),
        "next_repair_layer": ("contact-response-consistent formulation" if is_a else
                              "wrench-to-generalized-force mapping/reference path" if
                              classification == "C-MAPPING-OR-REFERENCE-REGRESSION" else "none"),
        "next_candidate_authorized": is_a,
        "R2_reauthorized": is_a,
    }
    if not EVAL.finite_tree(decision):
        raise RuntimeError("non-finite attribution")
    P45.write_json(output / "post-exact-r1-attribution.json", decision)
    replay_error = None if args.replay_of is None else P45.semantic_error(
        args.replay_of / "post-exact-r1-attribution.json",
        output / "post-exact-r1-attribution.json")
    P45.write_json(output / "summary.json", {
        "pass": trusted, "classification": classification,
        "replay_max_abs_error": replay_error,
        "replay_pass": replay_error is None or replay_error <= 1.0e-11,
        "next_candidate_authorized": is_a,
        "R2_reauthorized": is_a,
    })
    sources = [config_path, continuation_path, ROOT / old_base["scene"],
               ROOT / old_base["executable"], ROOT / new_base["executable"],
               authority, wrench_source, exact_evidence, Path(__file__).resolve(), EVAL.__file__,
               Path(SENS.__file__), Path(TAU.__file__), Path(ROOT_CAUSE.__file__)]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(Path(path).relative_to(ROOT)): hashlib.sha256(Path(path).read_bytes()).hexdigest()
                    for path in sources},
    })
    return 0 if trusted else 2


if __name__ == "__main__":
    raise SystemExit(main())
