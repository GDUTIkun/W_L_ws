#!/usr/bin/env python3
"""Phase 44 WBC-to-plant constrained rolling realization audit."""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase44_realization_audit_v1.json"
P42_PATH = ROOT / "tools/experiments/run_phase42_causal_attribution.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P42 = load_module(P42_PATH, "phase44_phase42_oracle")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


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


def vec(row: dict[str, str], prefix: str, count: int) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{index}"]) for index in range(count)])


def matrix(row: dict[str, str], prefix: str, rows: int, columns: int) -> np.ndarray:
    return np.asarray([[float(row[f"{prefix}{r}_{c}"]) for c in range(columns)]
                       for r in range(rows)])


def profile_spec(candidate: str, bandwidth_hz: float) -> dict[str, float | str]:
    omega = 2.0 * math.pi * bandwidth_hz
    return {
        "candidate": candidate,
        "gain": f"{bandwidth_hz:g}Hz",
        "kp": omega * omega if candidate in ("C", "D") else 0.0,
        "kd": 2.0 * omega if candidate in ("C", "D") else 0.0,
        "rate_gain": omega if candidate in ("B", "D") else 0.0,
    }


def native_from_control(model: mujoco.MjModel, row: dict[str, str]) -> dict[str, str]:
    data = mujoco.MjData(model)
    qpos = np.zeros(model.nq)
    qpos[3] = 1.0
    joint_names = ("left_hip_joint", "left_knee_joint", "left_wheel_joint",
                   "right_hip_joint", "right_knee_joint", "right_wheel_joint",
                   "left_connect1_joint", "left_connect2_joint",
                   "right_connect1_joint", "right_connect2_joint")
    for index, name in enumerate(joint_names):
        joint = P42.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos[int(model.jnt_qposadr[joint])] = float(row[f"raw_q{index}"])
    desired_quaternion = vec(row, "base_q", 4)
    qpos[3:7] = desired_quaternion
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    site = P42.required_id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
    desired_site = vec(row, "base_p", 3)
    qpos[:3] += desired_site - np.asarray(data.site_xpos[site])
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    jacp = np.zeros((3, model.nv)); jacr = np.zeros_like(jacp)
    mujoco.mj_jacSite(model, data, jacp, jacr, site)
    qvel = np.zeros(model.nv)
    for index, name in enumerate(joint_names):
        joint = P42.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qvel[int(model.jnt_dofadr[joint])] = float(row[f"raw_dq{index}"])
    desired_twist = np.r_[vec(row, "base_v", 3), vec(row, "base_omega", 3)]
    jac = np.vstack((jacp, jacr))
    qvel[:6] = np.linalg.solve(jac[:, :6], desired_twist - jac[:, 6:] @ qvel[6:])
    result = {
        "record_kind": "pre_command", "control_tick": row["tick"],
        "physics_substep": "-1", "time_s": row["time_s"],
    }
    for prefix, values in (("qpos", qpos), ("qvel", qvel),
                           ("ctrl", np.zeros(model.nu)), ("qacc", np.zeros(model.nv))):
        for index, value in enumerate(values):
            result[f"{prefix}{index}"] = f"{value:.17g}"
    return result


def validate_snapshot_reconstruction(config: dict[str, Any], model: mujoco.MjModel) -> dict[str, float]:
    control = {int(row["tick"]): row for row in read_csv(ROOT / config["phase42_control_authority"])}
    native = {int(row["control_tick"]): row for row in
              read_csv(ROOT / config["phase42_native_authority"])
              if row["record_kind"] == "pre_command"}
    qpos_error = qvel_error = 0.0
    for tick in config["common_snapshot_ticks"]:
        rebuilt = native_from_control(model, control[tick])
        qpos_error = max(qpos_error, float(np.max(np.abs(
            vec(rebuilt, "qpos", model.nq) - vec(native[tick], "qpos", model.nq)))))
        qvel_error = max(qvel_error, float(np.max(np.abs(
            vec(rebuilt, "qvel", model.nv) - vec(native[tick], "qvel", model.nv)))))
    return {"qpos_max_abs": qpos_error, "qvel_max_abs": qvel_error}


def persistent_tick(values: np.ndarray, threshold: float, window: int) -> int | None:
    for start in range(0, max(0, len(values) - window + 1)):
        if np.all(np.abs(values[start:start + window]) >= threshold):
            return start
    return None


def first_tick(values: np.ndarray, threshold: float) -> int | None:
    indices = np.flatnonzero(np.abs(values) >= threshold)
    return int(indices[0]) if indices.size else None


def select_own_ticks(rows: list[dict[str, str]], candidate: str,
                     config: dict[str, Any]) -> tuple[list[int], dict[str, int | None]]:
    thresholds = config["event_thresholds"]
    radius = float(config["wheel_radius_m"])
    left_rate = -radius * np.asarray([float(row["raw_dq2"]) for row in rows])
    right_rate = -radius * np.asarray([float(row["raw_dq5"]) for row in rows])
    common_rate = 0.5 * (left_rate + right_rate)
    differential_rate = 0.5 * (right_rate - left_rate)
    if candidate == "B":
        task_residual = np.maximum(
            np.abs(np.asarray([float(row["realized_qdd_wheel_left"]) -
                               float(row["desired_qdd_wheel_left"]) for row in rows])),
            np.abs(np.asarray([float(row["realized_qdd_wheel_right"]) -
                               float(row["desired_qdd_wheel_right"]) for row in rows])))
        task_threshold = float(thresholds["task_residual_native_rad_s2"])
    else:
        task_residual = np.maximum(
            np.abs(np.asarray([float(row["physical_ddxi_left"]) -
                               float(row["desired_ddxi_left"]) for row in rows])),
            np.abs(np.asarray([float(row["physical_ddxi_right"]) -
                               float(row["desired_ddxi_right"]) for row in rows])))
        task_threshold = float(thresholds["task_residual_xi_m_s2"])
    xi_left = np.asarray([float(row["xi_left"]) for row in rows])
    xi_right = np.asarray([float(row["xi_right"]) for row in rows])
    xi_deviation = np.maximum(np.abs(xi_left - xi_left[0]), np.abs(xi_right - xi_right[0]))
    rotation = np.asarray([math.sqrt(sum(float(row[f"base_rotvec{i}"]) ** 2
                                        for i in range(3))) for row in rows])
    slack = np.asarray([float(row["maximum_normalized_slack"]) for row in rows])
    loads = np.asarray([[float(row["normal_left"]), float(row["normal_right"])] for row in rows])
    contact_bad = np.asarray([row["contact_left"] != "1" or row["contact_right"] != "1"
                              for row in rows])
    load_bad = np.min(loads, axis=1) <= float(thresholds["normal_load_fraction"]) * np.min(loads[0])
    events: dict[str, int | None] = {
        "tick0": 0,
        "persistent_common_rate": persistent_tick(common_rate,
            float(thresholds["native_common_rim_rate_m_s"]),
            int(thresholds["persistent_window_ticks"])),
        "persistent_differential_rate": persistent_tick(differential_rate,
            float(thresholds["native_differential_rim_rate_m_s"]),
            int(thresholds["persistent_window_ticks"])),
        "material_task_residual": first_tick(task_residual, task_threshold),
        "material_xi_deviation": first_tick(xi_deviation,
            float(thresholds["xi_deviation_m"])),
        "material_base_rotation": first_tick(rotation,
            float(thresholds["base_rotation_rad"])),
        "material_slack": first_tick(slack, float(thresholds["normalized_slack"])),
        "contact_or_load_deterioration": int(np.flatnonzero(contact_bad | load_bad)[0])
            if np.any(contact_bad | load_bad) else None,
        "failure_minus_5": max(0, len(rows) - 1 - 5),
        "failure_minus_1": max(0, len(rows) - 1 - 1),
    }
    return sorted({value for value in events.values() if value is not None}), events


def write_native_authority(path: Path, rows: list[dict[str, str]]) -> None:
    write_csv(path, rows)


def run_controller(config: dict[str, Any], output: Path, authority: Path, tick: int,
                   spec: dict[str, Any], delta: np.ndarray) -> dict[str, str]:
    command = [str(ROOT / config["executable"]), str(ROOT / config["scene"]), str(output),
               f"R43-{spec['candidate']}__screen", str(spec["gain"]), str(spec["kp"]),
               str(spec["kd"]), str(spec["rate_gain"]), "0", "0", "0", "0",
               str(authority), str(tick), *map(str, delta)]
    subprocess.run(command, cwd=ROOT, check=True)
    rows = read_csv(output)
    if len(rows) != 1:
        raise RuntimeError(f"expected one controller row in {output}")
    return rows[0]


def material_point_metrics(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray,
                           ctrl: np.ndarray, epsilon: float) -> dict[str, np.ndarray | float]:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos; data.qvel[:] = qvel; data.ctrl[:] = ctrl
    base_weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[base_weld] = 0
    mujoco.mj_forward(model, data)
    qacc = data.qacc.copy()
    floor = P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    wheel_geoms = [P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in ("left_wheel_collision", "right_wheel_collision")]
    wheel_bodies = [P42.required_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                    for name in ("left_wheel_body", "right_wheel_body")]
    positions: list[np.ndarray | None] = [None, None]
    normals: list[np.ndarray | None] = [None, None]
    for contact in data.contact:
        for side, geom in enumerate(wheel_geoms):
            if {int(contact.geom1), int(contact.geom2)} == {geom, floor} and positions[side] is None:
                positions[side] = np.asarray(contact.pos).copy()
                normal = np.asarray(contact.frame).reshape(3, 3)[0].copy()
                normals[side] = normal if int(contact.geom2) == geom else -normal
    slip = np.full(2, np.nan); tangential_acceleration = np.full(2, np.nan)
    lateral = np.full(2, np.nan); normal_velocity = np.full(2, np.nan)
    formula_residual = 0.0

    def body_velocity(changed_qpos: np.ndarray, changed_qvel: np.ndarray,
                      body: int) -> tuple[np.ndarray, np.ndarray]:
        probe = mujoco.MjData(model); probe.qpos[:] = changed_qpos; probe.qvel[:] = changed_qvel
        probe.eq_active[base_weld] = 0; mujoco.mj_forward(model, probe)
        jacp = np.zeros((3, model.nv)); jacr = np.zeros_like(jacp)
        mujoco.mj_jacBody(model, probe, jacp, jacr, body)
        return jacp @ changed_qvel, jacr @ changed_qvel

    for side, body in enumerate(wheel_bodies):
        if positions[side] is None or normals[side] is None:
            continue
        center = np.asarray(data.xpos[body]).copy(); r = positions[side] - center
        jacp = np.zeros((3, model.nv)); jacr = np.zeros_like(jacp)
        mujoco.mj_jacBody(model, data, jacp, jacr, body)
        velocity_center = jacp @ qvel; omega = jacr @ qvel
        point_velocity = velocity_center + np.cross(omega, r)
        normal = normals[side] / np.linalg.norm(normals[side])
        tangent = np.array([1.0, 0.0, 0.0]); tangent -= normal * np.dot(tangent, normal)
        tangent /= np.linalg.norm(tangent)
        lateral_axis = np.cross(normal, tangent)
        slip[side] = float(np.dot(tangent, point_velocity))
        lateral[side] = float(np.dot(lateral_axis, point_velocity))
        normal_velocity[side] = float(np.dot(normal, point_velocity))
        samples = []
        point_samples = []
        for sign in (-1.0, 1.0):
            changed = qpos.copy(); mujoco.mj_integratePos(model, changed, qvel, sign * epsilon)
            vc, wc = body_velocity(changed, qvel + sign * epsilon * qacc, body)
            samples.append((vc, wc))
            point_samples.append(vc + np.cross(wc, r))
        acceleration_center = (samples[1][0] - samples[0][0]) / (2.0 * epsilon)
        alpha = (samples[1][1] - samples[0][1]) / (2.0 * epsilon)
        acceleration_point = (acceleration_center + np.cross(alpha, r) +
                              np.cross(omega, np.cross(omega, r)))
        direct = (point_samples[1] - point_samples[0]) / (2.0 * epsilon)
        formula_residual = max(formula_residual, float(np.max(np.abs(acceleration_point - direct))))
        tangential_acceleration[side] = float(np.dot(tangent, acceleration_point))
    return {"slip": slip, "lateral": lateral, "normal_velocity": normal_velocity,
            "tangential_acceleration": tangential_acceleration,
            "formula_residual": formula_residual}


def basis(values: np.ndarray) -> np.ndarray:
    return np.asarray([0.5 * (values[0] + values[1]), 0.5 * (values[1] - values[0])])


def native_xi_acceleration_map(oracle: Any, qpos: np.ndarray, qvel: np.ndarray,
                               time_s: float, qacc: np.ndarray,
                               ddxi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    step = 1e-6
    jacobian = np.zeros((2, oracle.model.nv))
    for column in range(oracle.model.nv):
        values = []
        for sign in (-1.0, 1.0):
            changed = qvel.copy(); changed[column] += sign * step
            oracle.geometry.set_state(qpos, changed, time_s)
            mujoco.mj_forward(oracle.model, oracle.geometry.data)
            values.append(np.asarray(oracle.geometry.current_value()["velocity"]))
        jacobian[:, column] = (values[1] - values[0]) / (2.0 * step)
    return jacobian, ddxi - jacobian @ qacc


def semantic_error(left: Path, right: Path, ignored: set[str]) -> float:
    a, b = read_csv(left), read_csv(right)
    if len(a) != len(b) or (a and a[0].keys() != b[0].keys()):
        return math.inf
    error = 0.0
    for first, second in zip(a, b):
        for key in first:
            if key in ignored:
                continue
            try:
                x, y = float(first[key]), float(second[key])
                if math.isnan(x) and math.isnan(y):
                    continue
                error = max(error, abs(x - y))
            except ValueError:
                if first[key] != second[key]:
                    return math.inf
    return error


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    output.mkdir(parents=True)
    probes = output / "probes"; probes.mkdir()
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
    reconstruction = validate_snapshot_reconstruction(config, model)
    tolerances = config["tolerances"]
    if (reconstruction["qpos_max_abs"] > tolerances["snapshot_qpos_max_abs"] or
            reconstruction["qvel_max_abs"] > tolerances["snapshot_qvel_max_abs"]):
        raise RuntimeError(f"native snapshot reconstruction failed: {reconstruction}")
    p42_config = json.loads((ROOT / config["phase42_config"]).read_text(encoding="utf-8"))
    oracle = P42.Oracle(p42_config)
    native_common = {int(row["control_tick"]): row for row in
                     read_csv(ROOT / config["phase42_native_authority"])
                     if row["record_kind"] == "pre_command"}
    own_sources: dict[str, tuple[Path, list[int]]] = {}
    selection: dict[str, Any] = {}
    bw = float(config["representative_bandwidth_hz"])
    for candidate in ("B", "C", "D"):
        path = ROOT / config["phase43_formal"] / f"nominal-{candidate}-{config['representative_gain']}.csv"
        rows = read_csv(path)
        ticks, events = select_own_ticks(rows, candidate, config)
        authority = output / f"own-native-{candidate}.csv"
        write_native_authority(authority, [native_from_control(model, rows[tick]) for tick in ticks])
        own_sources[candidate] = (authority, ticks)
        selection[candidate] = {"source": str(path.relative_to(ROOT)), "ticks": ticks, "events": events}
    write_json(output / "snapshot-selection.json", selection)

    snapshot_rows: list[dict[str, Any]] = []
    task_rows: list[dict[str, Any]] = []
    acceleration_rows: list[dict[str, Any]] = []
    contact_rows: list[dict[str, Any]] = []
    rolling_rows: list[dict[str, Any]] = []
    xi_rows: list[dict[str, Any]] = []
    controller_rows: list[dict[str, str]] = []
    probe_data: dict[tuple[str, str, int, str, float], dict[str, Any]] = {}
    actual_contact_details: list[dict[str, Any]] = []

    for candidate in ("B", "C", "D"):
        spec = profile_spec(candidate, bw)
        sources = [("common", ROOT / config["phase42_native_authority"],
                    list(map(int, config["common_snapshot_ticks"])), native_common),
                   ("own", own_sources[candidate][0], own_sources[candidate][1],
                    {int(row["control_tick"]): row for row in read_csv(own_sources[candidate][0])})]
        channels = []
        if candidate in ("C", "D"):
            channels += [("xi_common", np.asarray([1.0, 1.0, 0.0, 0.0])),
                         ("xi_differential", np.asarray([-1.0, 1.0, 0.0, 0.0]))]
        if candidate in ("B", "D"):
            channels += [("native_common", np.asarray([0.0, 0.0, 1.0, 1.0])),
                         ("native_differential", np.asarray([0.0, 0.0, -1.0, 1.0]))]
        for source_name, authority, ticks, native_by_tick in sources:
            for tick in ticks:
                probes_to_run = [("baseline", 0.0, np.zeros(4))]
                for channel, direction in channels:
                    magnitude = (float(config["task_delta"]["xi_acceleration_m_s2"])
                                 if channel.startswith("xi") else
                                 float(config["task_delta"]["native_acceleration_rad_s2"]))
                    for scale in (1.0, float(config["task_delta"]["half_scale"])):
                        for sign in (-1.0, 1.0):
                            probes_to_run.append((channel, sign * scale,
                                                  sign * scale * magnitude * direction))
                for channel, signed_scale, delta in probes_to_run:
                    probe_path = probes / (f"{candidate}-{source_name}-t{tick}-{channel}-"
                                           f"{signed_scale:+g}.csv")
                    control = run_controller(config, probe_path, authority, tick, spec, delta)
                    controller_rows.append(control)
                    native = native_by_tick[tick]
                    tau = vec(control, "tau", 6)
                    detail: list[dict[str, Any]] = []
                    actual = oracle.evaluate(native, detail, -tau)
                    actual_contact_details.extend(detail)
                    qpos = vec(native, "qpos", model.nq); qvel = vec(native, "qvel", model.nv)
                    material = material_point_metrics(model, qpos, qvel, -tau,
                        float(config["finite_difference_epsilon_s"]))
                    nudot = vec(control, "physical_solution", 12)
                    lambdas = vec(control, "physical_solution", 30)[18:30]
                    reduction = matrix(control, "reduction_", 16, 12)
                    reduction_bias = vec(control, "reduction_bias", 16)
                    predicted_qacc = reduction @ nudot + reduction_bias
                    actual_qacc = vec(actual, "qacc", 16)
                    mass = vec(actual, "mass", 256).reshape(16, 16)
                    reduced_actual = np.linalg.solve(reduction.T @ mass @ reduction,
                        reduction.T @ mass @ (actual_qacc - reduction_bias))
                    maps = [matrix(control, f"contact_map_{side}_", 12, 6)
                            for side in range(2)]
                    qp_contact = maps[0] @ lambdas[:6] + maps[1] @ lambdas[6:]
                    mj_contact_native = (vec(actual, "qfrc_contact_left", 16) +
                                         vec(actual, "qfrc_contact_right", 16))
                    mj_contact = reduction.T @ mj_contact_native
                    mj_actuator = reduction.T @ vec(actual, "qfrc_actuator", 16)
                    xi_map = matrix(control, "xi_map_", 2, 12)
                    xi_bias = vec(control, "xi_bias", 2)
                    qp_ddxi = xi_map @ nudot + xi_bias
                    actual_ddxi = np.asarray([actual["ddxi_left_m_s2"], actual["ddxi_right_m_s2"]])
                    qp_slip_acc = np.asarray([float(control["contact_task_residual0"]),
                                              float(control["contact_task_residual3"])])
                    qp_y = np.r_[nudot[[8, 11]], qp_ddxi, qp_slip_acc,
                                 nudot[[0, 2, 4]], lambdas[[2, 8]]]
                    mj_y = np.r_[actual_qacc[oracle.wheel_dadr], actual_ddxi,
                                 material["tangential_acceleration"], reduced_actual[[0, 2, 4]],
                                 [actual["normal_load_left_n"], actual["normal_load_right_n"]]]
                    key = (candidate, source_name, tick, channel, signed_scale)
                    probe_data[key] = {"qp_y": qp_y, "mj_y": mj_y, "control": control,
                                       "actual": actual, "nudot": nudot,
                                       "reduced_actual": reduced_actual, "xi_map": xi_map,
                                       "xi_bias": xi_bias, "qp_ddxi": qp_ddxi,
                                       "actual_ddxi": actual_ddxi, "tau": tau,
                                       "qp_contact": qp_contact, "mj_contact": mj_contact,
                                       "mj_actuator": mj_actuator, "actual_qacc": actual_qacc,
                                       "mass": mass}
                    if channel != "baseline":
                        continue
                    snapshot_rows.append({"candidate": candidate, "source": source_name,
                        "tick": tick, "time_s": native["time_s"], "contact_left": control["contact_left"],
                        "contact_right": control["contact_right"]})
                    desired_xi = np.asarray([float(control["desired_ddxi_left"]),
                                             float(control["desired_ddxi_right"])])
                    desired_native = np.asarray([
                        float(control["desired_qdd_wheel_left"]),
                        float(control["desired_qdd_wheel_right"])])
                    for task, desired, realized, scale_value, task_index in (
                        ("xi", desired_xi, qp_ddxi, 1.0, 6),
                        ("native", desired_native, nudot[[8, 11]], 20.0, 7)):
                        enabled = (task == "xi" and candidate in ("C", "D")) or (
                                  task == "native" and candidate in ("B", "D"))
                        residual = realized - desired
                        map_task = xi_map if task == "xi" else np.eye(12)[[8, 11]]
                        gradient = map_task.T @ residual / (scale_value ** 2)
                        for side in range(2):
                            task_rows.append({"candidate": candidate, "source": source_name,
                                "tick": tick, "task": task, "enabled": enabled, "side": side,
                                "desired": desired[side], "realized_qp": realized[side],
                                "raw_residual": residual[side],
                                "normalized_residual": residual[side] / scale_value,
                                "weighted_squared_cost": (residual[side] / scale_value) ** 2,
                                "gradient_norm": float(np.linalg.norm(gradient)),
                                "reported_task_cost": float(control[f"task_cost{task_index}"]),
                                "wrench_task_cost": float(control["task_cost8"]),
                                "contact_task_cost": float(control["task_cost0"]),
                                "slack_task_cost": float(control["task_cost9"]),
                                "base_task_information": "unavailable_in_minimal_profile",
                                "active_torque": control["active_count0"],
                                "active_contact": control["active_count1"],
                                "active_acceleration": control["active_count2"]})
                    error = actual_qacc - predicted_qacc
                    acceleration_rows.append({"candidate": candidate, "source": source_name,
                        "tick": tick, "native_full_norm": float(np.linalg.norm(error)),
                        "base_6d_norm": float(np.linalg.norm(error[:6])),
                        "leg_active_norm": float(np.linalg.norm(error[[6, 7, 9, 10]])),
                        "wheel_left": error[oracle.wheel_dadr[0]],
                        "wheel_right": error[oracle.wheel_dadr[1]],
                        "wheel_common": basis(error[oracle.wheel_dadr])[0],
                        "wheel_differential": basis(error[oracle.wheel_dadr])[1],
                        "reduced_full_norm": float(np.linalg.norm(reduced_actual - nudot)),
                        "reduced_xi_error_left": actual_ddxi[0] - qp_ddxi[0],
                        "reduced_xi_error_right": actual_ddxi[1] - qp_ddxi[1],
                        "affine_bias_norm": float(np.linalg.norm(reduction_bias)),
                        "full_dynamics_residual": actual["full_dynamics_residual_max_abs"]})
                    cerror = mj_contact - qp_contact
                    contact_rows.append({"candidate": candidate, "source": source_name, "tick": tick,
                        "qp_norm": float(np.linalg.norm(qp_contact)),
                        "mj_norm": float(np.linalg.norm(mj_contact)),
                        "error_norm": float(np.linalg.norm(cerror)),
                        "relative_error": float(np.linalg.norm(cerror) / max(np.linalg.norm(qp_contact), 1e-12)),
                        "base_error_norm": float(np.linalg.norm(cerror[:6])),
                        "leg_error_norm": float(np.linalg.norm(cerror[[6, 7, 9, 10]])),
                        "wheel_left_error": cerror[8], "wheel_right_error": cerror[11],
                        "wheel_common_error": basis(cerror[[8, 11]])[0],
                        "wheel_differential_error": basis(cerror[[8, 11]])[1],
                        "qp_actuator_wheel_left": -tau[2], "qp_actuator_wheel_right": -tau[5],
                        "mj_actuator_wheel_left": mj_actuator[8], "mj_actuator_wheel_right": mj_actuator[11],
                        "reconstruction_residual": actual["contact_applyft_jacobian_max_abs"]})
                    rolling_rows.append({"candidate": candidate, "source": source_name, "tick": tick,
                        "native_qdot_left": actual["wheel_dq_left_rad_s"],
                        "native_qdot_right": actual["wheel_dq_right_rad_s"],
                        "rim_rate_left": -float(config["wheel_radius_m"]) * actual["wheel_dq_left_rad_s"],
                        "rim_rate_right": -float(config["wheel_radius_m"]) * actual["wheel_dq_right_rad_s"],
                        "dxi_left": actual["dxi_left_m_s"], "dxi_right": actual["dxi_right_m_s"],
                        "slip_left": material["slip"][0], "slip_right": material["slip"][1],
                        "lateral_left": material["lateral"][0], "lateral_right": material["lateral"][1],
                        "normal_velocity_left": material["normal_velocity"][0],
                        "normal_velocity_right": material["normal_velocity"][1],
                        "material_tangent_acc_left": material["tangential_acceleration"][0],
                        "material_tangent_acc_right": material["tangential_acceleration"][1],
                        "penetration_left": actual["penetration_left_m"],
                        "penetration_right": actual["penetration_right_m"],
                        "normal_load_left": actual["normal_load_left_n"],
                        "normal_load_right": actual["normal_load_right_n"],
                        "rigid_body_formula_residual": material["formula_residual"]})
                    if candidate == "C" and source_name == "own":
                        native_map, native_bias = native_xi_acceleration_map(
                            oracle, qpos, qvel, float(native["time_s"]), actual_qacc, actual_ddxi)
                        native_leg = [index for index in range(6, 16)
                                      if index not in oracle.wheel_dadr]
                        for side in range(2):
                            qp_values = {"base": float(xi_map[side, :6] @ nudot[:6]),
                                "leg": float(xi_map[side, [6, 7, 9, 10]] @ nudot[[6, 7, 9, 10]]),
                                "wheel": float(xi_map[side, [8, 11]] @ nudot[[8, 11]]),
                                "jdot_v": float(xi_bias[side])}
                            mj_values = {"base": float(native_map[side, :6] @ actual_qacc[:6]),
                                "leg": float(native_map[side, native_leg] @ actual_qacc[native_leg]),
                                "wheel": float(native_map[side, oracle.wheel_dadr] @
                                               actual_qacc[oracle.wheel_dadr]),
                                "jdot_v": float(native_bias[side])}
                            for realization, values, reported in (("qp", qp_values, qp_ddxi[side]),
                                                                  ("mj_native", mj_values,
                                                                   actual_ddxi[side])):
                                total = sum(values.values())
                                xi_rows.append({"candidate": "C", "source": "own", "tick": tick,
                                    "realization": realization, "side": side, **values, "sum": total,
                                    "reported": reported, "closure_error": total - reported})

    write_csv(output / "controller-probes.csv", controller_rows)
    write_csv(output / "snapshot-table.csv", snapshot_rows)
    write_csv(output / "task-reference-qp.csv", task_rows)
    write_csv(output / "qp-vs-mj-qacc.csv", acceleration_rows)
    write_csv(output / "contact-generalized-force.csv", contact_rows)
    write_csv(output / "rolling-kinematics.csv", rolling_rows)
    write_csv(output / "xi-decomposition.csv", xi_rows)
    if actual_contact_details:
        write_csv(output / "mujoco-contact-details.csv", actual_contact_details)

    authority: dict[str, Any] = {}
    contact_authority_rows: list[dict[str, Any]] = []
    symmetry_errors: list[float] = []
    half_errors: list[float] = []
    for candidate in ("B", "C", "D"):
        channels = [name for name in ("xi_common", "xi_differential", "native_common",
                                      "native_differential")
                    if any(key[0] == candidate and key[3] == name for key in probe_data)]
        for source_name, ticks in (("common", list(map(int, config["common_snapshot_ticks"]))),
                                   ("own", own_sources[candidate][1])):
            for tick in ticks:
                g_qp = np.zeros((11, len(channels))); g_mj = np.zeros_like(g_qp)
                for column, channel in enumerate(channels):
                    magnitude = (float(config["task_delta"]["xi_acceleration_m_s2"])
                                 if channel.startswith("xi") else
                                 float(config["task_delta"]["native_acceleration_rad_s2"]))
                    plus = probe_data[(candidate, source_name, tick, channel, 1.0)]
                    minus = probe_data[(candidate, source_name, tick, channel, -1.0)]
                    plus_half = probe_data[(candidate, source_name, tick, channel, 0.5)]
                    minus_half = probe_data[(candidate, source_name, tick, channel, -0.5)]
                    g_qp[:, column] = (plus["qp_y"] - minus["qp_y"]) / (2.0 * magnitude)
                    g_mj[:, column] = (plus["mj_y"] - minus["mj_y"]) / (2.0 * magnitude)
                    d_qp_contact = (plus["qp_contact"] - minus["qp_contact"]) / (2.0 * magnitude)
                    d_mj_contact = (plus["mj_contact"] - minus["mj_contact"]) / (2.0 * magnitude)
                    d_actuator = (plus["mj_actuator"] - minus["mj_actuator"]) / (2.0 * magnitude)
                    d_qacc = (plus["actual_qacc"] - minus["actual_qacc"]) / (2.0 * magnitude)
                    baseline = probe_data[(candidate, source_name, tick, "baseline", 0.0)]
                    for side, reduced_row, native_row in (("left", 8, oracle.wheel_dadr[0]),
                                                          ("right", 11, oracle.wheel_dadr[1])):
                        lhs = float(baseline["mass"][native_row] @ d_qacc)
                        contact_authority_rows.append({"candidate": candidate,
                            "source": source_name, "tick": tick, "channel": channel,
                            "side": side, "qp_contact_gain": d_qp_contact[reduced_row],
                            "mj_contact_gain": d_mj_contact[reduced_row],
                            "mj_actuator_gain": d_actuator[reduced_row],
                            "mj_mass_times_qacc_gain": lhs,
                            "mj_other_gain": lhs - d_actuator[reduced_row] - d_mj_contact[reduced_row],
                            "mj_wheel_qacc_gain": d_qacc[native_row]})
                    half_qp = (plus_half["qp_y"] - minus_half["qp_y"]) / magnitude
                    half_mj = (plus_half["mj_y"] - minus_half["mj_y"]) / magnitude
                    half_errors += [float(np.linalg.norm(g_qp[:, column] - half_qp) /
                                          max(np.linalg.norm(g_qp[:, column]), 1e-12)),
                                    float(np.linalg.norm(g_mj[:, column] - half_mj) /
                                          max(np.linalg.norm(g_mj[:, column]), 1e-12))]
                    symmetry_errors += [float(np.linalg.norm(plus["qp_y"] + minus["qp_y"] -
                                                               2.0 * baseline["qp_y"]) /
                                                max(np.linalg.norm(plus["qp_y"] - minus["qp_y"]), 1e-12)),
                                        float(np.linalg.norm(plus["mj_y"] + minus["mj_y"] -
                                                               2.0 * baseline["mj_y"]) /
                                                max(np.linalg.norm(plus["mj_y"] - minus["mj_y"]), 1e-12))]
                mismatch = g_mj - g_qp
                singular = np.linalg.svd(g_mj, compute_uv=False)
                authority[f"{candidate}-{source_name}-t{tick}"] = {
                    "input_channels": channels,
                    "output_rows": ["qdd_wheel_left", "qdd_wheel_right", "ddxi_left", "ddxi_right",
                                    "a_slip_left", "a_slip_right", "base_x", "base_z", "base_pitch",
                                    "normal_load_left", "normal_load_right"],
                    "g_qp": g_qp, "g_mj": g_mj, "g_mismatch": mismatch,
                    "condition_number": float(singular[0] / singular[-1]) if singular[-1] > 0 else math.inf,
                    "singular_values": singular,
                    "near_null": bool(singular[-1] <= float(tolerances["near_null_singular_value"])),
                    "mismatch_ratio": float(np.linalg.norm(mismatch) / max(np.linalg.norm(g_qp), 1e-12)),
                }
    write_json(output / "authority-matrices.json", authority)
    write_csv(output / "contact-authority-transfer.csv", contact_authority_rows)

    thresholds = config["classification_thresholds"]
    enabled_tasks = [row for row in task_rows if row["enabled"]]
    task_loss = max(abs(float(row["normalized_residual"])) for row in enabled_tasks)
    wheel_acc_loss = max(max(abs(float(row["wheel_left"])), abs(float(row["wheel_right"])))
                         for row in acceleration_rows)
    xi_acc_loss = max(max(abs(float(row["reduced_xi_error_left"])),
                          abs(float(row["reduced_xi_error_right"]))) for row in acceleration_rows)
    contact_loss = max(float(row["relative_error"]) for row in contact_rows)
    authority_loss = max(value["mismatch_ratio"] for value in authority.values())
    optimization_material = task_loss > float(thresholds["task_max_normalized_residual"])
    plant_material = (wheel_acc_loss > float(thresholds["native_wheel_acceleration_error_rad_s2"])
                      or xi_acc_loss > float(thresholds["xi_acceleration_error_m_s2"])
                      or contact_loss > float(thresholds["contact_generalized_force_relative"])
                      or authority_loss > float(thresholds["authority_mismatch_relative"]))
    c_own = [row for row in xi_rows if row["realization"] == "mj_native"]
    contribution = {name: float(np.mean([abs(float(row[name])) for row in c_own]))
                    for name in ("base", "leg", "wheel", "jdot_v")}
    dominant = max(contribution, key=contribution.get)
    coordinate_material = dominant != "wheel" and contribution[dominant] > contribution["wheel"]
    layers = [name for name, value in (("optimization", optimization_material),
                                       ("plant", plant_material),
                                       ("coordinate", coordinate_material)) if value]
    provisional_classification = ("P44-E" if len(layers) >= 2 else "P44-A" if layers == ["optimization"]
                      else "P44-B" if layers == ["plant"] else "P44-C" if layers == ["coordinate"]
                      else "P44-U")
    phase42_summary = json.loads((ROOT / config["phase42_summary"]).read_text(encoding="utf-8"))
    phase43_summary = json.loads((ROOT / config["phase43_summary"]).read_text(encoding="utf-8"))
    gates = {
        "DG44-00": (phase42_summary.get("failure_tick") == 111 and
                    phase43_summary.get("classification") == "P43-U" and
                    reconstruction["qpos_max_abs"] <= tolerances["snapshot_qpos_max_abs"] and
                    reconstruction["qvel_max_abs"] <= tolerances["snapshot_qvel_max_abs"]),
        "DG44-01": bool(enabled_tasks),
        "DG44-02": max(row["full_dynamics_residual"] for row in acceleration_rows) <=
                   tolerances["full_dynamics_max_abs"],
        "DG44-03": max(row["reconstruction_residual"] for row in contact_rows) <=
                   tolerances["contact_reconstruction_max_abs"],
        "DG44-04": bool(contact_rows), "DG44-05": bool(rolling_rows),
        "DG44-06": (max(symmetry_errors) <= tolerances["probe_odd_symmetry_relative"] and
                    max(half_errors) <= tolerances["probe_half_delta_relative"]),
        "DG44-07": max(abs(float(row["closure_error"])) for row in xi_rows) <=
                   tolerances["xi_decomposition_max_abs"],
        "DG44-08": True,
    }
    classification = provisional_classification if gates["DG44-06"] and gates["DG44-07"] else "P44-U"
    gates["DG44-08"] = classification != "P44-U"
    evidence = {"classification": classification, "material_layers": layers,
        "provisional_classification_if_oracles_pass": provisional_classification,
        "mechanism": "B-contact" if plant_material and contact_loss >
                     float(thresholds["contact_generalized_force_relative"]) else "B-multiple"
                     if plant_material else None,
        "task_max_normalized_residual": task_loss,
        "native_wheel_acceleration_error_max": wheel_acc_loss,
        "xi_acceleration_error_max": xi_acc_loss,
        "contact_relative_error_max": contact_loss,
        "authority_mismatch_ratio_max": authority_loss,
        "c_mean_absolute_contributions": contribution, "c_dominant_contribution": dominant,
        "probe_symmetry_relative_max": max(symmetry_errors),
        "probe_half_delta_relative_max": max(half_errors),
        "snapshot_reconstruction": reconstruction, "gates": gates,
        "no_repair_contract": config["no_repair_contract"]}
    write_json(output / "classification-evidence.json", evidence)

    replay_error = None
    if args.replay_of:
        comparisons = ["snapshot-table.csv", "task-reference-qp.csv", "qp-vs-mj-qacc.csv",
                       "contact-generalized-force.csv", "contact-authority-transfer.csv",
                       "rolling-kinematics.csv", "xi-decomposition.csv"]
        replay_error = max(semantic_error(args.replay_of / name, output / name, set())
                           for name in comparisons)
    gates["DG44-09"] = replay_error is None or replay_error <= tolerances["semantic_replay_max_abs"]
    summary = {"pass": all(gates.values()), "classification": classification, "gates": gates,
               "replay_max_abs_error": replay_error, **evidence}
    write_json(output / "summary.json", summary)
    sources = [config_path, ROOT / config["scene"], ROOT / config["executable"],
               ROOT / config["phase42_config"], ROOT / config["phase42_control_authority"],
               ROOT / config["phase42_native_authority"], ROOT / config["phase42_summary"],
               ROOT / config["phase43_config"], ROOT / config["phase43_manifest"],
               ROOT / config["phase43_summary"], Path(__file__).resolve(), P42_PATH,
               ROOT / "ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp",
               ROOT / "ros_ws/src/wheel_leg_mujoco/src/phase35_workspace_attribution_loop.cpp"]
    write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
        **config["no_repair_contract"]})
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
