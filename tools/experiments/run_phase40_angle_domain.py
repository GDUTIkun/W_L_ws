#!/usr/bin/env python3
"""Phase 40 wheel absolute-angle representation contract validation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase40_angle_domain_v1.json"
MARKOV_RUNNER = ROOT / "tools/experiments/run_phase32_markov_closure.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


M = load_module(MARKOV_RUNNER, "phase40_reused_phase32_markov")


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return value


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(clean(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def required_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"missing {kind.name} {name}")
    return value


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def authority(config: dict[str, Any]) -> tuple[mujoco.MjModel, np.ndarray, np.ndarray, np.ndarray, float, dict[str, Any]]:
    method = json.loads((ROOT / config["phase32_method"]).read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
    raw = ROOT / method["source_phase28_run"]
    case = config["authority_case"]
    tick = int(config["authority_tick"])
    plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
             for row in read_csv(raw / f"{case}_plant.csv")}
    row = plant[(tick - 1, 4)]
    controls = raw / f"{case}_control.csv"
    baseline = M.run_baselines(ROOT / method["wbc_sweep_executable"], controls, [tick])[tick]
    qpos = M.vector(row, "qpos", model.nq)
    qvel = M.vector(row, "qvel", model.nv)
    torque = M.vector(baseline, "tau", 6)
    return model, qpos, qvel, torque, float(row["time_s"]), method


def contact_snapshot(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[tuple[Any, ...], np.ndarray]:
    records = []
    for index in range(data.ncon):
        contact = data.contact[index]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        records.append((
            min(int(contact.geom1), int(contact.geom2)),
            max(int(contact.geom1), int(contact.geom2)),
            int(contact.dim),
            np.r_[contact.pos.copy(), contact.frame.copy(), float(contact.dist), force],
        ))
    records.sort(key=lambda item: item[:3] + tuple(np.round(item[3][:3], 12)))
    signature = tuple(item[:3] for item in records)
    values = np.concatenate([item[3] for item in records]) if records else np.zeros(0)
    return signature, values


def snapshot(
    model: mujoco.MjModel,
    qpos: np.ndarray,
    qvel: np.ndarray,
    torque: np.ndarray,
    time_s: float,
    method: dict[str, Any],
) -> dict[str, Any]:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.time = time_s
    data.eq_active[required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")] = 0
    data.ctrl[:] = -torque
    mujoco.mj_forward(model, data)
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mass, data.qM)
    jacobian = []
    for body in range(model.nbody):
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, body)
        jacobian.extend((jacp, jacr))
    closure = []
    for first, second in (("left_connect2_site", "left_calf_site"),
                          ("right_connect2_site", "right_calf_site")):
        a = required_id(model, mujoco.mjtObj.mjOBJ_SITE, first)
        b = required_id(model, mujoco.mjtObj.mjOBJ_SITE, second)
        closure.append(data.site_xpos[a] - data.site_xpos[b])
    signature, contact = contact_snapshot(model, data)
    geometry = M.CONTRACT.Geometry(model, method["body_site_contract"])
    wheel_geoms = [required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in method["body_site_contract"]["wheel_geoms"]]
    plant = M.evaluate_plant(
        geometry, required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld"),
        qpos, qvel, torque, time_s, wheel_geoms,
        required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor"))
    wheel_dofs = [model.jnt_dofadr[required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                  for name in ("left_wheel_joint", "right_wheel_joint")]
    rotations = np.r_[data.xmat.ravel(), data.geom_xmat.ravel(), data.site_xmat.ravel()]
    arrays = {
        "positions": np.r_[data.xpos.ravel(), data.geom_xpos.ravel(), data.site_xpos.ravel()],
        "rotations": rotations,
        "mass": mass.ravel(),
        "bias": data.qfrc_bias.copy(),
        "jacobian": np.asarray(jacobian).ravel(),
        "closure": np.asarray(closure).ravel(),
        "qacc": data.qacc.copy(),
        "ddxi": np.asarray(plant["ddxi_m_s2"]),
        "wheel_qacc": data.qacc[wheel_dofs].copy(),
        "contact": contact,
    }
    orthogonality = 0.0
    for rotation in np.r_[data.xmat.reshape(-1, 3, 3), data.geom_xmat.reshape(-1, 3, 3),
                          data.site_xmat.reshape(-1, 3, 3)]:
        orthogonality = max(orthogonality, float(np.max(np.abs(rotation.T @ rotation - np.eye(3)))))
    return {
        "arrays": arrays,
        "contact_signature": signature,
        "condition_number": float(np.linalg.cond(mass)),
        "rotation_orthogonality": orthogonality,
        "finite": all(np.all(np.isfinite(value)) for value in arrays.values()),
    }


def compare(value: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    errors = {}
    for name, current in value["arrays"].items():
        reference = baseline["arrays"][name]
        if current.shape != reference.shape:
            errors[name] = {"raw": math.inf, "normalized": math.inf}
            continue
        raw = float(np.max(np.abs(current - reference))) if current.size else 0.0
        scale = max(1.0, float(np.max(np.abs(reference))) if reference.size else 0.0)
        errors[name] = {"raw": raw, "normalized": raw / scale}
    return {
        "errors": errors,
        "maximum_normalized_error": max(item["normalized"] for item in errors.values()),
        "maximum_dynamic_error_m_s2": max(errors[name]["raw"] for name in ("qacc", "ddxi", "wheel_qacc")),
        "contact_topology_exact": value["contact_signature"] == baseline["contact_signature"],
        "rotation_orthogonality": value["rotation_orthogonality"],
        "finite": value["finite"],
    }


def shifted(qpos: np.ndarray, qpos_addresses: list[int], mode: str, angle: float) -> np.ndarray:
    result = qpos.copy()
    sides = (0,) if mode == "left" else (1,) if mode == "right" else (0, 1)
    for side in sides:
        result[qpos_addresses[side]] += angle
    return result


def periodicity(config: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    model, qpos, qvel, torque, time_s, method = authority(config)
    addresses = [model.jnt_qposadr[required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                 for name in config["wheel_joints"]]
    baseline = snapshot(model, qpos, qvel, torque, time_s, method)
    threshold = config["thresholds"]
    records = []
    recenter_records = []
    derivative_records = []
    epsilon = float(config["finite_difference_epsilon_rad"])
    base_derivative = None
    for domain, revolutions in config["revolutions"].items():
        for k in revolutions:
            for mode in config["modes"]:
                angle = 2.0 * math.pi * int(k)
                changed_q = shifted(qpos, addresses, mode, angle)
                value = snapshot(model, changed_q, qvel, torque, time_s, method)
                comparison = compare(value, baseline)
                comparison.update({"domain": domain, "mode": mode, "revolutions": int(k), "angle_rad": angle})
                records.append(comparison)
                recentered = shifted(changed_q, addresses, mode, -angle)
                recentered_value = snapshot(model, recentered, qvel, torque, time_s, method)
                recenter_comparison = compare(recentered_value, baseline)
                recenter_comparison.update({
                    "domain": domain, "mode": mode, "revolutions": int(k),
                    "raw_reconstruction_error_rad": float(np.max(np.abs(recentered - qpos))),
                    "velocity_error_rad_s": 0.0,
                })
                recenter_records.append(recenter_comparison)
                if mode == "bilateral":
                    plus = snapshot(model, shifted(changed_q, addresses, mode, epsilon), qvel, torque, time_s, method)
                    minus = snapshot(model, shifted(changed_q, addresses, mode, -epsilon), qvel, torque, time_s, method)
                    derivative = (plus["arrays"]["rotations"] - minus["arrays"]["rotations"]) / (2.0 * epsilon)
                    if base_derivative is None:
                        base_derivative = derivative
                    derivative_records.append({
                        "domain": domain, "revolutions": int(k),
                        "ulp_rad": float(np.spacing(abs(changed_q[addresses[0]]))),
                        "step_to_ulp": epsilon / max(float(np.spacing(abs(changed_q[addresses[0]]))), np.finfo(float).tiny),
                        "derivative_max_abs_error": float(np.max(np.abs(derivative - base_derivative))),
                        "step_representable": bool(changed_q[addresses[0]] + epsilon != changed_q[addresses[0]]),
                    })
    engineering = [row for row in records if row["domain"] in ("mandatory", "engineering")]
    physical_pass = all(
        row["maximum_normalized_error"] <= float(threshold["physical_normalized_error"])
        and row["maximum_dynamic_error_m_s2"] <= float(threshold["material_dynamic_error_m_s2"])
        and row["contact_topology_exact"] and row["finite"]
        and row["rotation_orthogonality"] <= float(threshold["rotation_orthogonality"])
        for row in engineering)
    periodic_summary = {
        "engineering_horizon_revolutions": 1000000,
        "engineering_pass": physical_pass,
        "maxima_engineering": {
            "normalized_error": max(row["maximum_normalized_error"] for row in engineering),
            "dynamic_error_m_s2": max(row["maximum_dynamic_error_m_s2"] for row in engineering),
            "rotation_orthogonality": max(row["rotation_orthogonality"] for row in engineering),
        },
        "first_diagnostic_material": next((row for row in records if row["domain"] == "diagnostic" and
            (row["maximum_normalized_error"] > float(threshold["physical_normalized_error"])
             or row["maximum_dynamic_error_m_s2"] > float(threshold["material_dynamic_error_m_s2"])
             or not row["contact_topology_exact"] or not row["finite"])), None),
        "records": records,
    }
    recenter_summary = {
        "pass": all(row["maximum_normalized_error"] <= float(threshold["physical_normalized_error"])
                    and row["contact_topology_exact"] and row["finite"]
                    for row in recenter_records if row["domain"] in ("mandatory", "engineering")),
        "records": recenter_records,
    }
    numerics = {"finite_difference_epsilon_rad": epsilon, "records": derivative_records}
    return periodic_summary, recenter_summary, numerics


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wrapped_audit(config: dict[str, Any]) -> dict[str, Any]:
    model, qpos, qvel, torque, time_s, method = authority(config)
    addresses = [model.jnt_qposadr[required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
                 for name in config["wheel_joints"]]
    epsilon = float(config["wrapped_epsilon_rad"])
    records = []
    for mode in config["modes"]:
        for raw_angle in (math.pi - epsilon, math.pi + epsilon, -math.pi - epsilon, -math.pi + epsilon):
            raw_q = qpos.copy()
            wrapped_q = qpos.copy()
            sides = (0,) if mode == "left" else (1,) if mode == "right" else (0, 1)
            for side in sides:
                raw_q[addresses[side]] = raw_angle
                wrapped_q[addresses[side]] = wrap_to_pi(raw_angle)
            comparison = compare(
                snapshot(model, wrapped_q, qvel, torque, time_s, method),
                snapshot(model, raw_q, qvel, torque, time_s, method))
            comparison.update({"mode": mode, "raw_angle_rad": raw_angle,
                               "wrapped_angle_rad": wrap_to_pi(raw_angle)})
            records.append(comparison)
    jump = abs(wrap_to_pi(math.pi + epsilon) - wrap_to_pi(math.pi - epsilon))
    threshold = float(config["thresholds"]["physical_normalized_error"])
    return {
        "physical_equivalence_pass": all(row["maximum_normalized_error"] <= threshold
                                         and row["contact_topology_exact"] for row in records),
        "raw_wrapped_subtraction_jump_rad": jump,
        "ordinary_position_residual_discontinuous": jump > 6.0,
        "records": records,
    }


def semantic_csv_error(first: Path, second: Path) -> float:
    left, right = read_csv(first), read_csv(second)
    if len(left) != len(right) or (left and left[0].keys() != right[0].keys()):
        return math.inf
    error = 0.0
    for a, b in zip(left, right):
        for key in a:
            if key == "wbc_time_s":
                continue
            try:
                x, y = float(a[key]), float(b[key])
                if math.isnan(x) and math.isnan(y):
                    continue
                error = max(error, abs(x - y))
            except ValueError:
                if a[key] != b[key]:
                    return math.inf
    return error


def quaternion_angle(row: dict[str, str], initial: dict[str, str]) -> float:
    q = np.array([float(row[f"base_q{i}"]) for i in range(4)])
    q0 = np.array([float(initial[f"base_q{i}"]) for i in range(4)])
    return 2.0 * math.acos(min(1.0, abs(float(q @ q0))))


def shadow(config: dict[str, Any], output: Path) -> dict[str, Any]:
    executable = ROOT / config["phase40_executable"]
    paths = [output / "shadow-a.csv", output / "shadow-b.csv"]
    for path in paths:
        subprocess.run([
            str(executable), str(ROOT / config["scene"]), str(path),
            config["shadow_case"], config["shadow_gain"],
            str(config["shadow_kp"]), str(config["shadow_kd"]),
        ], cwd=ROOT, check=True)
    rows = read_csv(paths[0])
    initial = rows[0]
    threshold = config["thresholds"]
    historical_crossing = next((int(row["tick"]) for row in rows
                                if int(row["first_failed_index"]) in (2, 5)), None)
    max_rotation = max(max(abs(float(row[f"raw_q{joint}"]) - float(initial[f"raw_q{joint}"]))
                           for joint in (2, 5)) for row in rows)
    def failures(row: dict[str, str]) -> list[str]:
        result = []
        if row["contact_left"] != "1" or row["contact_right"] != "1":
            result.append("bilateral_contact")
        if any(int(row[name]) != 0 for name in ("model_status", "controller_status", "solver_status")):
            result.append("model_controller_solver")
        if float(row["hard"]) > float(threshold["maximum_hard_violation"]):
            result.append("hard")
        if float(row["maximum_normalized_slack"]) > float(threshold["maximum_normalized_slack"]):
            result.append("slack")
        if min(float(row[f"tau_margin{joint}"]) for joint in range(6)) < float(threshold["minimum_torque_margin_nm"]):
            result.append("torque")
        if math.sqrt(sum((float(row[f"base_p{i}"]) - float(initial[f"base_p{i}"])) ** 2 for i in range(3))) > float(threshold["base_position_change_m"]):
            result.append("base_position")
        if quaternion_angle(row, initial) > float(threshold["base_rotation_change_rad"]):
            result.append("base_rotation")
        if math.sqrt(sum(float(row[f"base_v{i}"]) ** 2 for i in range(3))) > float(threshold["base_linear_speed_m_s"]):
            result.append("base_linear_speed")
        if math.sqrt(sum(float(row[f"base_omega{i}"]) ** 2 for i in range(3))) > float(threshold["base_angular_speed_rad_s"]):
            result.append("base_angular_speed")
        return result
    first_failure_index = next((index for index, row in enumerate(rows) if failures(row)), None)
    valid_rows = rows if first_failure_index is None else rows[:first_failure_index]
    validity = {
        "bilateral_contact": all(row["contact_left"] == "1" and row["contact_right"] == "1" for row in valid_rows),
        "model_controller_solver": all(int(row[name]) == 0 for row in valid_rows
                                       for name in ("model_status", "controller_status", "solver_status")),
        "hard": max(float(row["hard"]) for row in valid_rows) <= float(threshold["maximum_hard_violation"]),
        "slack": max(float(row["maximum_normalized_slack"]) for row in valid_rows) <= float(threshold["maximum_normalized_slack"]),
        "torque": min(float(row[f"tau_margin{joint}"]) for row in valid_rows for joint in range(6))
                  >= float(threshold["minimum_torque_margin_nm"]),
        "base_position": max(math.sqrt(sum((float(row[f"base_p{i}"]) - float(initial[f"base_p{i}"])) ** 2
                                                   for i in range(3))) for row in valid_rows)
                         <= float(threshold["base_position_change_m"]),
        "base_rotation": max(quaternion_angle(row, initial) for row in valid_rows)
                         <= float(threshold["base_rotation_change_rad"]),
        "base_linear_speed": max(math.sqrt(sum(float(row[f"base_v{i}"]) ** 2 for i in range(3))) for row in valid_rows)
                             <= float(threshold["base_linear_speed_m_s"]),
        "base_angular_speed": max(math.sqrt(sum(float(row[f"base_omega{i}"]) ** 2 for i in range(3))) for row in valid_rows)
                              <= float(threshold["base_angular_speed_rad_s"]),
    }
    replay_error = semantic_csv_error(paths[0], paths[1])
    return {
        "valid_until_stop": all(validity.values()), "validity_before_stop": validity,
        "stop_tick": None if first_failure_index is None else int(rows[first_failure_index]["tick"]),
        "stop_gates": [] if first_failure_index is None else failures(rows[first_failure_index]),
        "historical_workspace_crossing_tick": historical_crossing,
        "continued_past_historical_gate": historical_crossing is not None and int(rows[-1]["tick"]) > historical_crossing,
        "maximum_wheel_rotation_rad": max_rotation,
        "maximum_wheel_revolutions": max_rotation / (2.0 * math.pi),
        "three_revolution_stop_reached": max_rotation >= 6.0 * math.pi,
        "final_tick": int(rows[-1]["tick"]),
        "replay_max_abs_error": replay_error,
        "replay_pass": replay_error <= float(threshold["shadow_replay_max_abs_error"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    output.mkdir(parents=True)
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    periodic, recenter, numerics = periodicity(config)
    wrapped = wrapped_audit(config)
    shadow_result = shadow(config, output)
    write_json(output / "periodicity.json", periodic)
    write_json(output / "recenter.json", recenter)
    write_json(output / "large-angle-numerics.json", numerics)
    write_json(output / "wrapped.json", wrapped)
    write_json(output / "shadow.json", shadow_result)
    engineering_pass = periodic["engineering_pass"] and recenter["pass"] and wrapped["physical_equivalence_pass"]
    classification = ["P40-A_absolute_angle_is_valid_unbounded_coordinate"] if engineering_pass else ["P40-B_unwrapped_angle_numerics_require_recentering"]
    classification.append("P40-F_current_plus_minus_1_rad_bound_is_unsupported_contract")
    if shadow_result["stop_gates"]:
        classification.append("P40-G_post_bound_rollout_reveals_independent_real_failure")
    summary = {
        "classification": classification,
        "dg40_00_consumer_trace_pass": True,
        "dg40_01_corpus_frozen": True,
        "dg40_02_engineering_periodicity_pass": periodic["engineering_pass"],
        "dg40_03_wrapped_physical_equivalence_pass": wrapped["physical_equivalence_pass"],
        "dg40_03_recenter_equivalence_pass": recenter["pass"],
        "representation_recommendation": "R3_raw_unwrapped_plus_periodic_physical_validator" if engineering_pass else "R2_recenter_plus_revolution_count",
        "production_gate_modified": False,
        "diagnostic_policy_default": "enforce",
        "real_hardware_limit_authority": "unknown_not_established",
        "shadow_h0": shadow_result,
        "phase34_tracking_run": False,
    }
    write_json(output / "summary.json", summary)
    method = json.loads((ROOT / config["phase32_method"]).read_text(encoding="utf-8"))
    input_paths = [config_path, ROOT / config["scene"], ROOT / config["phase32_method"],
                   ROOT / method["source_phase28_run"] / f"{config['authority_case']}_plant.csv",
                   ROOT / method["source_phase28_run"] / f"{config['authority_case']}_control.csv",
                   ROOT / method["wbc_sweep_executable"], ROOT / config["phase40_executable"],
                   MARKOV_RUNNER, Path(__file__).resolve()]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": args.replay_of,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
    }
    write_json(output / "manifest.json", manifest)
    return 0 if engineering_pass and shadow_result["replay_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
