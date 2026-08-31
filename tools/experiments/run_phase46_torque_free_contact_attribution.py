#!/usr/bin/env python3
"""Phase46 fixed-state torque replay and free-contact-acceleration audit."""

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
DEFAULT_SOURCE = (ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair/"
                  "evidence/automated/contact-realization-sensitivity-formal-v4")
SCALES = (1.0, 0.5, 0.25)
DIRECTIONS = ("Fr_L", "Fn_L", "Fr_R", "Fn_R")
ACTUATORS = ("left_hip", "left_knee", "left_wheel",
             "right_hip", "right_knee", "right_wheel")


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SENS = load(ROOT / "tools/experiments/run_phase46_contact_realization_sensitivity.py", "p46_tau_sens")
ATTR, P45C, P45, P44, P42 = SENS.ATTR, SENS.P45C, SENS.P45, SENS.P44, SENS.P42


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1.0e-12))


def point_array(item: dict[str, Any]) -> np.ndarray:
    result = np.zeros((2, 2, 3))
    for row in item["points"]:
        side = 0 if row["side"] == "left" else 1
        result[side, int(row["point_index"])] = row["wheel_force_Fr_Fl_Fn_n"]
    return result


def contact_signature(item: dict[str, Any]) -> str:
    topology = []
    for side_name in ("left", "right"):
        topology.append(sorted((min(int(row["geom1"]), int(row["geom2"])),
                                max(int(row["geom1"]), int(row["geom2"])),
                                int(row["dimension"]), int(row["efc_address"]))
                               for row in item["points"] if row["side"] == side_name))
    value = {
        "topology": topology, "counts": item["contact_count"],
        "dimensions": item["contact_dimensions"],
        "normal_load_positive": [float(item["actual"]["dynamics"][f"normal_load_{side}_n"]) > 0.0
                                 for side in ("left", "right")],
        "penetration_positive": [float(value) > 0.0 for value in item["penetration_m"]],
        "friction_interior": float(item["minimum_friction_margin_n"]) > 0.0,
    }
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def observe_tau(tau: np.ndarray, native: dict[str, str], model: mujoco.MjModel,
                oracle: Any, geometry: list[dict[str, Any]], hip: tuple[int, int]) -> dict[str, Any]:
    details: list[dict[str, Any]] = []
    dynamics = oracle.evaluate(native, details, -tau)
    qpos = P44.vec(native, "qpos", model.nq); qvel = P44.vec(native, "qvel", model.nv)
    actual = {"dynamics": dynamics, "details": details,
              "qpos": qpos, "qvel": qvel,
              "material": P44.material_point_metrics(model, qpos, qvel, -tau, 1.0e-6)}
    wrench, _ = ATTR.actual_contact_wrench(actual, geometry)
    points, reconstructed = SENS.point_forces(actual, geometry)
    qacc = P44.vec(dynamics, "qacc", model.nv)
    item = {
        "tau": tau, "actual": actual, "mj_wrench": wrench.reshape(-1), "points": points,
        "contact_count": [sum(int(row["side"]) == side for row in details) for side in range(2)],
        "contact_dimensions": sorted({int(row["dim"]) for row in details}),
        "penetration_m": [float(dynamics[f"penetration_{side}_m"])
                          for side in ("left", "right")],
        "minimum_friction_margin_n": min(float(row["friction_margin_diagnostic_n"])
                                          for row in details),
        "hip_common_acceleration_rad_s2": float(0.5 * (qacc[hip[0]] + qacc[hip[1]])),
        "point_aggregate_closure_max_abs": float(np.max(np.abs(reconstructed - wrench))),
        "whole_dynamics_closure": float(dynamics["full_dynamics_residual_max_abs"]),
        "contact_applyft_closure": float(dynamics["contact_applyft_jacobian_max_abs"]),
    }
    item["contact_solver_signature"] = contact_signature(item)
    return item


def point_jacobians(model: mujoco.MjModel, native: dict[str, str], geometry: list[dict[str, Any]],
                    baseline: dict[str, Any]) -> list[dict[str, Any]]:
    data = mujoco.MjData(model)
    data.qpos[:] = P44.vec(native, "qpos", model.nq)
    data.qvel[:] = P44.vec(native, "qvel", model.nv)
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0
    mujoco.mj_forward(model, data)
    result = []
    for row in baseline["points"]:
        side = 0 if row["side"] == "left" else 1
        point = np.asarray(row["position_world_m"])
        linear = np.zeros((3, model.nv)); angular = np.zeros_like(linear)
        mujoco.mj_jac(model, data, linear, angular, point, geometry[side]["body"])
        result.append({"side": side, "point_index": int(row["point_index"]),
                       "map": geometry[side]["frame"].T @ linear})
    return result


def qforce(item: dict[str, Any], name: str, nv: int) -> np.ndarray:
    return P44.vec(item["actual"]["dynamics"], name, nv)


def attribution(item: dict[str, Any], baseline: dict[str, Any], mass: np.ndarray,
                jacobians: list[dict[str, Any]], nv: int) -> dict[str, Any]:
    actuator = qforce(item, "qfrc_actuator", nv) - qforce(baseline, "qfrc_actuator", nv)
    contact = ((qforce(item, "qfrc_contact_left", nv) + qforce(item, "qfrc_contact_right", nv)) -
               (qforce(baseline, "qfrc_contact_left", nv) +
                qforce(baseline, "qfrc_contact_right", nv)))
    other = qforce(item, "qfrc_other_constraint", nv) - qforce(baseline, "qfrc_other_constraint", nv)
    passive = ((qforce(item, "qfrc_passive", nv) + qforce(item, "qfrc_applied", nv)) -
               (qforce(baseline, "qfrc_passive", nv) + qforce(baseline, "qfrc_applied", nv)))
    accelerations = {"actuator_free": np.linalg.solve(mass, actuator),
                     "contact_reaction": np.linalg.solve(mass, contact),
                     "other_constraint": np.linalg.solve(mass, other),
                     "passive_applied": np.linalg.solve(mass, passive)}
    accelerations["sum"] = sum(accelerations.values())
    qacc_delta = (qforce(item, "qacc", nv) - qforce(baseline, "qacc", nv))
    points = {name: np.zeros((2, 2, 3)) for name in accelerations}
    for row in jacobians:
        for name, value in accelerations.items():
            points[name][row["side"], row["point_index"]] = row["map"] @ value
    point_force = point_array(item) - point_array(baseline)
    return {
        "generalized_force": {"actuator": actuator, "contact": contact,
                              "other_constraint": other, "passive_applied": passive},
        "qacc": accelerations, "point_acceleration_RLN": points,
        "point_force_RLN": point_force,
        "aggregate_wrench": np.asarray(item["mj_wrench"]) - np.asarray(baseline["mj_wrench"]),
        "qacc_balance_closure_max_abs": float(np.max(np.abs(accelerations["sum"] - qacc_delta))),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    source = args.source.resolve(); output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    records = json.loads((source / "probe-records.json").read_text(encoding="utf-8"))
    sensitivity = json.loads((source / "contact-realization-sensitivity.json").read_text(encoding="utf-8"))
    original_baseline = records["baseline"]
    original = {(row["direction"], float(row["scale"]), int(row["branch"])): row
                for row in records["probes"] if row["stage"] == "target"}
    config_path = ROOT / "simulation/mujoco/config/phase46_hip_common_increment_limited_v1.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation = json.loads((ROOT / config["continuation_config"]).read_text(encoding="utf-8"))
    base, _, wrench_source = P45C.frozen_inputs(continuation)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    qpos = P44.vec(native, "qpos", model.nq); qvel = P44.vec(native, "qvel", model.nv)
    geometry = ATTR.contact_geometry(model, qpos, qvel,
                                     float(oracle.config["canonical_wheel_radius_m"]))
    hip = ATTR.hip_dofs(model)
    tau0 = np.asarray(original_baseline["tau"])
    replay_baseline = observe_tau(tau0, native, model, oracle, geometry, hip)
    jacobians = point_jacobians(model, native, geometry, replay_baseline)
    mass = P44.vec(replay_baseline["actual"]["dynamics"], "mass", model.nv ** 2).reshape(
        model.nv, model.nv)
    baseline_parity = {
        "aggregate_wrench_relative_error": relative(
            np.asarray(replay_baseline["mj_wrench"]), np.asarray(original_baseline["mj_wrench"])),
        "point_force_relative_error": relative(
            point_array(replay_baseline), point_array(original_baseline)),
        "hip_common_relative_error": abs(
            replay_baseline["hip_common_acceleration_rad_s2"] -
            float(original_baseline["actual_hip_common_acceleration_rad_s2"])) /
            max(abs(float(original_baseline["actual_hip_common_acceleration_rad_s2"])), 1.0e-12),
        "contact_solver_signature_match": (
            replay_baseline["contact_solver_signature"] == contact_signature(original_baseline)),
    }

    replays: dict[tuple[str, float, int], dict[str, Any]] = {}
    parity_rows = []
    stage0_records = []
    for direction in DIRECTIONS:
        for sign in (-1, 1):
            full_delta = np.asarray(original[(direction, 1.0, sign)]["tau"]) - tau0
            for scale in SCALES:
                tau = tau0 + scale * full_delta
                item = observe_tau(tau, native, model, oracle, geometry, hip)
                reference = original[(direction, scale, sign)]
                original_wrench = np.asarray(reference["mj_wrench"])
                original_points = point_array(reference)
                replay_wrench_delta = np.asarray(item["mj_wrench"]) - np.asarray(replay_baseline["mj_wrench"])
                original_wrench_delta = original_wrench - np.asarray(original_baseline["mj_wrench"])
                replay_point_delta = point_array(item) - point_array(replay_baseline)
                original_point_delta = original_points - point_array(original_baseline)
                hip_delta = item["hip_common_acceleration_rad_s2"] - replay_baseline["hip_common_acceleration_rad_s2"]
                original_hip_delta = (float(reference["actual_hip_common_acceleration_rad_s2"]) -
                                      float(original_baseline["actual_hip_common_acceleration_rad_s2"]))
                row = {
                    "direction": direction, "branch": sign, "scale": scale,
                    "torque_input_relative_error": relative(
                        tau - tau0, np.asarray(reference["tau"]) - tau0),
                    "aggregate_wrench_relative_error": relative(
                        replay_wrench_delta, original_wrench_delta),
                    "FrFn_relative_error": relative(
                        replay_wrench_delta[SENS.TARGET_INDICES],
                        original_wrench_delta[SENS.TARGET_INDICES]),
                    "point_force_relative_error": relative(replay_point_delta, original_point_delta),
                    "hip_common_relative_error": abs(hip_delta - original_hip_delta) /
                                                 max(abs(original_hip_delta), 1.0e-12),
                    "contact_solver_signature_match": (
                        item["contact_solver_signature"] == contact_signature(reference)),
                }
                parity_rows.append(row)
                replays[(direction, scale, sign)] = item
                stage0_records.append({**row, "tau": tau, "replay": item})

    maximum_parity_error = max(max(baseline_parity[key] for key in
                                   ("aggregate_wrench_relative_error", "point_force_relative_error",
                                    "hip_common_relative_error")),
                               max(max(row[key] for key in
                                   ("torque_input_relative_error",
                                    "aggregate_wrench_relative_error", "FrFn_relative_error",
                                    "point_force_relative_error", "hip_common_relative_error"))
                                   for row in parity_rows))
    stage0_pass = (maximum_parity_error <= 1.0e-4 and
                   baseline_parity["contact_solver_signature_match"] and
                   all(row["contact_solver_signature_match"] for row in parity_rows))
    if not stage0_pass:
        result = {"classification": "U-UNTRUSTED", "stage0_pass": False,
                  "maximum_torque_replay_relative_error": maximum_parity_error,
                  "baseline_parity": baseline_parity, "stage0_parity": parity_rows,
                  "reason": "torque replay/input/state contract did not close"}
        P45.write_json(output / "torque-free-contact-attribution.json", result)
        P45.write_json(output / "replay-records.json", {"stage0": stage0_records})
        P45.write_json(output / "summary.json", {"pass": False, **result})
        return 2

    target_delta = np.asarray(sensitivity["target_wrench_delta_n"])
    raw: dict[tuple[str, float, int], dict[str, Any]] = {}
    stage1_records = []
    for axis, direction in enumerate(DIRECTIONS):
        for scale in SCALES:
            for sign in (-1, 1):
                item = replays[(direction, scale, sign)]
                value = attribution(item, replay_baseline, mass, jacobians, model.nv)
                denominator = sign * scale * target_delta[axis]
                normalized = {
                    "delta_tau_per_qp_target": (np.asarray(item["tau"]) - tau0) / denominator,
                    "point_acceleration_RLN_per_qp_target": {
                        name: array / denominator for name, array in value["point_acceleration_RLN"].items()},
                    "point_force_RLN_per_qp_target": value["point_force_RLN"] / denominator,
                    "aggregate_wrench_per_qp_target": value["aggregate_wrench"] / denominator,
                    "qacc_balance_closure_per_qp_target": value["qacc_balance_closure_max_abs"] /
                                                         abs(denominator),
                }
                raw[(direction, scale, sign)] = normalized
                stage1_records.append({"direction": direction, "scale": scale, "branch": sign,
                                       "normalized": normalized, "unscaled": value})

    central: dict[str, dict[str, Any]] = {}
    branch_split = scale_convergence = 0.0
    keys = ("actuator_free", "contact_reaction", "other_constraint",
            "passive_applied", "sum")
    for direction in DIRECTIONS:
        by_scale = {}
        for scale in SCALES:
            plus, minus = raw[(direction, scale, 1)], raw[(direction, scale, -1)]
            branch_split = max(branch_split, *(relative(
                plus["point_acceleration_RLN_per_qp_target"][key],
                minus["point_acceleration_RLN_per_qp_target"][key]) for key in keys),
                relative(plus["point_force_RLN_per_qp_target"],
                         minus["point_force_RLN_per_qp_target"]))
            by_scale[scale] = {
                "delta_tau": 0.5 * (plus["delta_tau_per_qp_target"] +
                                    minus["delta_tau_per_qp_target"]),
                "point_acceleration": {key: 0.5 * (
                    plus["point_acceleration_RLN_per_qp_target"][key] +
                    minus["point_acceleration_RLN_per_qp_target"][key]) for key in keys},
                "point_force": 0.5 * (plus["point_force_RLN_per_qp_target"] +
                                      minus["point_force_RLN_per_qp_target"]),
                "aggregate_wrench": 0.5 * (plus["aggregate_wrench_per_qp_target"] +
                                           minus["aggregate_wrench_per_qp_target"]),
            }
        reference = by_scale[1.0]
        for scale in (0.5, 0.25):
            scale_convergence = max(scale_convergence, *(relative(
                reference["point_acceleration"][key], by_scale[scale]["point_acceleration"][key])
                for key in keys), relative(reference["point_force"], by_scale[scale]["point_force"]))
        central[direction] = reference

    fn_summary = {}
    free_cross_ratios = []; force_cross_ratios = []; cancellations = []
    for direction, commanded_side in (("Fn_L", 0), ("Fn_R", 1)):
        value = central[direction]
        free = value["point_acceleration"]["actuator_free"][:, :, 0].mean(axis=1)
        contact = value["point_acceleration"]["contact_reaction"][:, :, 0].mean(axis=1)
        other = value["point_acceleration"]["other_constraint"][:, :, 0].mean(axis=1)
        net = value["point_acceleration"]["sum"][:, :, 0].mean(axis=1)
        force = value["point_force"][:, :, 0].sum(axis=1)
        opposite = free * force < 0.0
        cancellation = -(contact + other) / free
        offside = 1 - commanded_side
        free_cross = abs(float(free[offside] / free[commanded_side]))
        force_cross = abs(float(force[offside] / force[commanded_side]))
        free_cross_ratios.append(free_cross); force_cross_ratios.append(force_cross)
        cancellations.extend(cancellation.tolist())
        fn_summary[direction] = {
            "delta_tau_by_actuator_nm_per_n": dict(zip(ACTUATORS, value["delta_tau"])),
            "wheel_mean_free_rolling_acceleration": free,
            "wheel_mean_contact_reaction_rolling_acceleration": contact,
            "wheel_mean_other_constraint_rolling_acceleration": other,
            "wheel_mean_net_rolling_acceleration": net,
            "aggregate_Fr_response": force,
            "Fr_opposes_free_rolling_tendency": opposite,
            "solver_cancellation_fraction": cancellation,
            "free_cross_ratio": free_cross, "Fr_cross_ratio": force_cross,
        }

    regime_count = len({item["contact_solver_signature"] for item in
                        [replay_baseline, *replays.values()]})
    point_closure = max(item["point_aggregate_closure_max_abs"] for item in
                        [replay_baseline, *replays.values()])
    whole_closure = max(max(abs(item["whole_dynamics_closure"]),
                            abs(item["contact_applyft_closure"])) for item in
                        [replay_baseline, *replays.values()])
    balance_closure = max(row["normalized"]["qacc_balance_closure_per_qp_target"]
                          for row in stage1_records)
    stable = branch_split <= 0.05 and scale_convergence <= 0.05 and regime_count == 1
    free_driven = (all(0.9 <= value <= 1.1 for value in cancellations) and
                   max(abs(a - b) for a, b in zip(free_cross_ratios, force_cross_ratios)) <= 0.1)
    solver_dominant = max(free_cross_ratios) <= 0.1 and max(force_cross_ratios) >= 0.25
    trusted = (stage0_pass and point_closure <= 1.0e-10 and whole_closure <= 1.0e-8 and
               balance_closure <= 1.0e-7)
    classification = ("U-UNTRUSTED" if not trusted else
                      "D-REGIME_DEPENDENT" if not stable else
                      "A-FREE_ACCELERATION_DRIVEN" if free_driven else
                      "B-SOLVER_REACTION_COUPLING_DOMINANT" if solver_dominant else
                      "C-MIXED_FREE_MOTION_AND_SOLVER_COUPLING")
    result = {
        "classification": classification, "trusted": trusted, "stage0_pass": stage0_pass,
        "stage0_maximum_relative_error": maximum_parity_error,
        "baseline_parity": baseline_parity, "stage0_parity": parity_rows,
        "frozen_conclusion": ("QP contact wrench is not a direct plant input; actual reaction is "
                              "reproduced by the QP-solution torque increment"),
        "fn_direction_summary": fn_summary,
        "cross_interpretation": ("bilateral actuator-induced free rolling motion is already present; "
                                 "the solver reaction follows and cancels that bilateral tendency"),
        "branch_split_relative": branch_split, "scale_convergence_relative": scale_convergence,
        "contact_solver_regime_signature_count": regime_count,
        "point_aggregate_wrench_closure_max_abs": point_closure,
        "whole_dynamics_contact_closure_max_abs": whole_closure,
        "free_reaction_qacc_balance_closure_max_abs": balance_closure,
        "scope": "compatible-H0 tick0 fixed-state torque replay only; no QP re-solve or repair",
    }
    P45.write_json(output / "torque-free-contact-attribution.json", result)
    P45.write_json(output / "replay-records.json", {"stage0": stage0_records,
                                                     "stage1": stage1_records})
    compared = ["torque-free-contact-attribution.json", "replay-records.json"]
    replay_error = max(P45.semantic_error(args.replay_of / name, output / name)
                       for name in compared) if args.replay_of else None
    replay_pass = replay_error is None or replay_error <= float(base["gates"]["semantic_replay_max_abs"])
    P45.write_json(output / "summary.json", {"pass": trusted and stable and replay_pass,
        "classification": classification, "replay_max_abs_error": replay_error,
        "stage0_pass": stage0_pass, "trusted": trusted, "stable": stable})
    sources = [source / "probe-records.json", source / "contact-realization-sensitivity.json",
               config_path, ROOT / base["scene"], authority, wrench_source, Path(__file__).resolve()]
    P45.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources}})
    return 0 if trusted and stable and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
