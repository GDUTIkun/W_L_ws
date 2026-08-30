#!/usr/bin/env python3
"""Phase 42 no-repair wheel-spin/contact-loss causal attribution."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import platform
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase42_causal_attribution_v1.json"
GEOMETRY_SCRIPT = ROOT / "tools/experiments/run_phase31_wheel_state_contract.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


GEOMETRY = load_module(GEOMETRY_SCRIPT, "phase42_geometry")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or list(rows[0]), lineterminator="\n")
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


def required_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"missing {kind.name} {name}")
    return value


def vector(row: dict[str, str], prefix: str, count: int) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{index}"]) for index in range(count)])


def semantic_error(left: list[dict[str, str]], right: list[dict[str, str]], ignored: set[str]) -> float:
    if len(left) != len(right) or (left and left[0].keys() != right[0].keys()):
        return math.inf
    result = 0.0
    for first, second in zip(left, right):
        for key in first:
            if key in ignored:
                continue
            try:
                a, b = float(first[key]), float(second[key])
                if math.isnan(a) and math.isnan(b):
                    continue
                result = max(result, abs(a - b))
            except ValueError:
                if first[key] != second[key]:
                    return math.inf
    return result


def grouped(values: np.ndarray) -> dict[str, float]:
    return {"left": float(values[0]), "right": float(values[1]),
            "common": float(0.5 * (values[0] + values[1])),
            "differential": float(0.5 * (values[1] - values[0]))}


def acceleration(geometry: Any, qpos: np.ndarray, qvel: np.ndarray,
                 ctrl: np.ndarray, time_s: float, epsilon: float) -> tuple[np.ndarray, float]:
    geometry.set_state(qpos, qvel, time_s)
    geometry.data.ctrl[:] = ctrl
    mujoco.mj_forward(geometry.model, geometry.data)
    qacc = geometry.data.qacc.copy()
    values = []
    for sign in (-1.0, 1.0):
        changed = qpos.copy()
        mujoco.mj_integratePos(geometry.model, changed, qvel, sign * epsilon)
        geometry.set_state(changed, qvel + sign * epsilon * qacc, time_s + sign * epsilon)
        geometry.data.ctrl[:] = ctrl
        mujoco.mj_forward(geometry.model, geometry.data)
        values.append(geometry.current_value()["velocity"])
    full = (values[1] - values[0]) / (2.0 * epsilon)
    geometry.set_state(qpos, qvel, time_s)
    geometry.data.ctrl[:] = ctrl
    mujoco.mj_forward(geometry.model, geometry.data)
    half_values = []
    for sign in (-1.0, 1.0):
        changed = qpos.copy()
        mujoco.mj_integratePos(geometry.model, changed, qvel, sign * epsilon * 0.5)
        geometry.set_state(changed, qvel + sign * epsilon * 0.5 * qacc,
                           time_s + sign * epsilon * 0.5)
        geometry.data.ctrl[:] = ctrl
        mujoco.mj_forward(geometry.model, geometry.data)
        half_values.append(geometry.current_value()["velocity"])
    return full, float(np.max(np.abs(full - (half_values[1] - half_values[0]) / epsilon)))


class Oracle:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
        contract = json.loads((ROOT / config["phase31_contract"]).read_text(encoding="utf-8"))
        self.geometry = GEOMETRY.Geometry(self.model, contract["body_site_contract"])
        self.data = mujoco.MjData(self.model)
        self.base_weld = required_id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
        self.floor = required_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.wheel_geoms = [required_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                            for name in ("left_wheel_collision", "right_wheel_collision")]
        self.wheel_bodies = [required_id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                             for name in ("left_wheel_body", "right_wheel_body")]
        self.wheel_joints = [required_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                             for name in ("left_wheel_joint", "right_wheel_joint")]
        self.wheel_qadr = [int(self.model.jnt_qposadr[joint]) for joint in self.wheel_joints]
        self.wheel_dadr = [int(self.model.jnt_dofadr[joint]) for joint in self.wheel_joints]
        self.leg_qadr = [int(self.model.jnt_qposadr[required_id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]) for name in
            ("left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint")]
        self.base_site = required_id(self.model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")

    def evaluate(self, native: dict[str, str], contact_rows: list[dict[str, Any]],
                 ctrl_override: np.ndarray | None = None) -> dict[str, Any]:
        qpos = vector(native, "qpos", self.model.nq)
        qvel = vector(native, "qvel", self.model.nv)
        ctrl = vector(native, "ctrl", self.model.nu) if ctrl_override is None else ctrl_override
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.ctrl[:] = ctrl
        self.data.time = float(native["time_s"])
        self.data.eq_active[self.base_weld] = 0
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        mass = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, mass, self.data.qM)
        lhs = mass @ self.data.qacc + self.data.qfrc_bias
        rhs = (self.data.qfrc_actuator + self.data.qfrc_passive +
               self.data.qfrc_applied + self.data.qfrc_constraint)
        contact_qfrc = np.zeros((2, self.model.nv))
        actual_wrench = np.zeros((2, 6))
        loads = np.zeros(2)
        penetration = np.zeros(2)
        slip = np.zeros((2, 2))
        counts = np.zeros(2, dtype=int)
        applyft_error = 0.0
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            side = next((value for value, geom in enumerate(self.wheel_geoms)
                         if {int(contact.geom1), int(contact.geom2)} == {geom, self.floor}), None)
            if side is None:
                continue
            local = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, index, local)
            frame = np.asarray(contact.frame).reshape(3, 3)
            world_force = frame.T @ local[:3]
            world_torque = frame.T @ local[3:]
            if int(contact.geom2) != self.wheel_geoms[side]:
                world_force *= -1.0
                world_torque *= -1.0
            generalized = np.zeros(self.model.nv)
            mujoco.mj_applyFT(self.model, self.data, world_force, world_torque,
                              np.asarray(contact.pos), self.wheel_bodies[side], generalized)
            jacp = np.zeros((3, self.model.nv)); jacr = np.zeros_like(jacp)
            mujoco.mj_jac(self.model, self.data, jacp, jacr,
                          np.asarray(contact.pos), self.wheel_bodies[side])
            via_jacobian = jacp.T @ world_force + jacr.T @ world_torque
            applyft_error = max(applyft_error, float(np.max(np.abs(generalized - via_jacobian))))
            contact_qfrc[side] += generalized
            wheel_origin = self.data.xpos[self.wheel_bodies[side]]
            actual_wrench[side, :3] += world_force
            actual_wrench[side, 3:] += world_torque + np.cross(
                np.asarray(contact.pos) - wheel_origin, world_force)
            loads[side] += world_force[2]
            penetration[side] = max(penetration[side], max(0.0, -float(contact.dist)))
            velocity = jacp @ qvel
            contact_velocity = frame @ velocity
            slip[side] = np.maximum(slip[side], np.abs(contact_velocity[1:3]))
            counts[side] += 1
            tangential = float(np.linalg.norm(local[1:3]))
            mu = float(contact.friction[0])
            item: dict[str, Any] = {
                "record_kind": native["record_kind"], "control_tick": int(native["control_tick"]),
                "physics_substep": int(native["physics_substep"]), "time_s": float(native["time_s"]),
                "contact_index": index, "side": side, "geom1": int(contact.geom1),
                "geom2": int(contact.geom2), "dim": int(contact.dim),
                "efc_address": int(contact.efc_address), "distance_m": float(contact.dist),
                "mu": mu, "tangential_norm_n": tangential,
                "friction_margin_diagnostic_n": mu * abs(float(local[0])) - tangential,
            }
            for axis in range(3):
                item[f"position_world_{axis}"] = float(contact.pos[axis])
                item[f"world_force_{axis}"] = world_force[axis]
                item[f"world_torque_{axis}"] = world_torque[axis]
                item[f"contact_velocity_{axis}"] = contact_velocity[axis]
            for row in range(3):
                for column in range(3):
                    item[f"frame_{row}{column}"] = frame[row, column]
            for dof in range(self.model.nv):
                item[f"generalized_{dof}"] = generalized[dof]
            contact_rows.append(item)
        geometry_value = self.geometry.current_value() if False else None
        self.geometry.data.eq_active[self.base_weld] = 0
        self.geometry.set_state(qpos, qvel, float(native["time_s"]))
        self.geometry.data.ctrl[:] = ctrl
        mujoco.mj_forward(self.model, self.geometry.data)
        geometry_value = self.geometry.current_value()
        ddxi, ddxi_error = acceleration(
            self.geometry, qpos, qvel, ctrl, float(native["time_s"]),
            float(self.config["finite_difference_epsilon_s"]))
        wheel_q = qpos[self.wheel_qadr]
        wheel_dq = qvel[self.wheel_dadr]
        wheel_ddq = self.data.qacc[self.wheel_dadr]
        other_constraint = self.data.qfrc_constraint - np.sum(contact_qfrc, axis=0)
        captured = vector(native, "qacc", self.model.nv)
        row: dict[str, Any] = {
            "record_kind": native["record_kind"], "control_tick": int(native["control_tick"]),
            "physics_substep": int(native["physics_substep"]), "time_s": float(native["time_s"]),
            "contact_count_left": counts[0], "contact_count_right": counts[1],
            "normal_load_left_n": loads[0], "normal_load_right_n": loads[1],
            "penetration_left_m": penetration[0], "penetration_right_m": penetration[1],
            "rolling_slip_left_m_s": slip[0, 0], "rolling_slip_right_m_s": slip[1, 0],
            "lateral_slip_left_m_s": slip[0, 1], "lateral_slip_right_m_s": slip[1, 1],
            "full_dynamics_residual_max_abs": float(np.max(np.abs(lhs - rhs))),
            "contact_applyft_jacobian_max_abs": applyft_error,
            "captured_qacc_max_abs_error": float(np.max(np.abs(captured - self.data.qacc))),
            "ddxi_half_step_max_abs_error_m_s2": ddxi_error,
        }
        for name, values in (("qpos", qpos), ("qvel", qvel), ("ctrl", ctrl),
                             ("qacc", self.data.qacc), ("mass", mass.ravel()),
                             ("qfrc_bias", self.data.qfrc_bias),
                             ("qfrc_passive", self.data.qfrc_passive),
                             ("qfrc_actuator", self.data.qfrc_actuator),
                             ("qfrc_applied", self.data.qfrc_applied),
                             ("qfrc_constraint", self.data.qfrc_constraint),
                             ("qfrc_contact_left", contact_qfrc[0]),
                             ("qfrc_contact_right", contact_qfrc[1]),
                             ("qfrc_other_constraint", other_constraint)):
            for index, value in enumerate(np.asarray(values).ravel()):
                row[f"{name}{index}"] = value
        for side, name in enumerate(("left", "right")):
            row[f"wheel_q_{name}_rad"] = wheel_q[side]
            row[f"wheel_dq_{name}_rad_s"] = wheel_dq[side]
            row[f"wheel_ddq_{name}_rad_s2"] = wheel_ddq[side]
            row[f"xi_{name}_m"] = geometry_value["position"][side]
            row[f"dxi_{name}_m_s"] = geometry_value["velocity"][side]
            row[f"ddxi_{name}_m_s2"] = ddxi[side]
            for index in range(6):
                row[f"actual_contact_wrench_{name}_{index}"] = actual_wrench[side, index]
        for axis in range(3):
            row[f"base_position_{axis}"] = geometry_value["base_position"][axis]
            row[f"base_velocity_{axis}"] = geometry_value["base_velocity"][axis]
            row[f"base_omega_{axis}"] = geometry_value["base_omega"][axis]
        for index, address in enumerate(self.leg_qadr):
            row[f"leg_q{index}"] = qpos[address]
        return row


def control_for_wbc(rows: list[dict[str, str]]) -> list[dict[str, Any]]:
    result = []
    for source in rows:
        row: dict[str, Any] = {"tick": source["tick"],
                              "source_ns": int(round(float(source["time_s"]) * 1e9)),
                              "contact_left": source["contact_left"],
                              "contact_right": source["contact_right"]}
        for index in range(3):
            row[f"base_p{index}"] = source[f"base_p{index}"]
            row[f"base_v{index}"] = source[f"base_v{index}"]
            row[f"base_w{index}"] = source[f"base_omega{index}"]
        for index in range(4):
            row[f"quat{index}"] = source[f"base_q{index}"]
        for index in range(6):
            row[f"q{index}"] = source[f"q{index}"]
            row[f"dq{index}"] = source[f"dq{index}"]
        for index in range(9):
            row[f"reference{index}"] = 0.0
        for index in range(12):
            row[f"reference{9 + index}"] = source[f"requested_wrench{index}"]
        result.append(row)
    return result


def wbc_baselines(executable: Path, rows: list[dict[str, Any]], ticks: list[int],
                  zero_rate_tick: int | None = None) -> dict[int, dict[str, str]]:
    modified = [dict(row) for row in rows]
    if zero_rate_tick is not None:
        target = next(row for row in modified if int(row["tick"]) == zero_rate_tick)
        target["dq2"] = 0.0
        target["dq5"] = 0.0
    with tempfile.TemporaryDirectory(prefix="phase42-wbc-") as directory:
        path = Path(directory) / "control.csv"
        write_csv(path, modified)
        command = [str(executable), str(path), "0.1", *map(str, ticks)]
        completed = subprocess.run(command, text=True, capture_output=True)
        if completed.returncode:
            raise RuntimeError(completed.stderr.strip() or "WBC snapshot oracle failed")
    return {int(row["tick"]): row for row in csv.DictReader(io.StringIO(completed.stdout))
            if row["channel"] == "baseline"}


def first_persistent(values: list[float], floor: float, window: int, same: int) -> int | None:
    for start in range(len(values) - window + 1):
        sample = values[start:start + window]
        if abs(sample[0]) < floor:
            continue
        active = [value for value in sample if abs(value) >= floor]
        if len(active) >= same and (sum(value > 0 for value in active) >= same or
                                    sum(value < 0 for value in active) >= same):
            return start
    return None


def event_audit(post: list[dict[str, Any]], config: dict[str, Any]) -> tuple[dict[str, Any], list[int]]:
    initial = post[0]
    signals: dict[str, list[float]] = {
        "wheel_common_rate_m_s": [0.5 * (row["dxi_left_m_s"] + row["dxi_right_m_s"]) for row in post],
        "wheel_differential_rate_m_s": [0.5 * (row["dxi_right_m_s"] - row["dxi_left_m_s"]) for row in post],
        "wheel_common_acceleration_m_s2": [0.5 * (row["ddxi_left_m_s2"] + row["ddxi_right_m_s2"]) for row in post],
        "wheel_differential_acceleration_m_s2": [0.5 * (row["ddxi_right_m_s2"] - row["ddxi_left_m_s2"]) for row in post],
        "normal_load_difference_n": [row["normal_load_right_n"] - row["normal_load_left_n"] for row in post],
        "base_position_change_m": [math.sqrt(sum((row[f"base_position_{axis}"] - initial[f"base_position_{axis}"]) ** 2 for axis in range(3))) for row in post],
        "base_speed_m_s": [math.sqrt(sum(row[f"base_velocity_{axis}"] ** 2 for axis in range(3))) for row in post],
        "leg_position_change_rad": [max(abs(row[f"leg_q{index}"] - initial[f"leg_q{index}"]) for index in range(4)) for row in post],
        "penetration_difference_m": [row["penetration_right_m"] - row["penetration_left_m"] for row in post],
    }
    settings = config["event_detection"]
    result: dict[str, Any] = {}
    key_ticks = {0}
    for family, values in signals.items():
        rule = settings["families"][family]
        bands = {}
        for multiplier in settings["sensitivity_multipliers"]:
            onset = first_persistent(values, float(rule["material_floor"]) * multiplier,
                                     int(settings["window_ticks"]),
                                     int(settings["minimum_same_direction_steps"]))
            tick = None if onset is None else int(post[onset]["control_tick"])
            bands[str(multiplier)] = tick
        numeric_index = first_persistent(values, float(rule["numeric_floor"]),
                                         int(settings["window_ticks"]),
                                         int(settings["minimum_same_direction_steps"]))
        material_tick = bands["1.0"]
        if material_tick is not None:
            key_ticks.update((material_tick - 1, material_tick, material_tick + 1))
        derivatives = np.diff(values)
        result[family] = {
            "numeric_onset_tick": None if numeric_index is None else int(post[numeric_index]["control_tick"]),
            "material_onset_tick": material_tick, "sensitivity_onset_ticks": bands,
            "initial": values[0], "final_bilateral": values[-1],
            "peak_abs": max(map(abs, values)),
            "peak_derivative_per_tick": float(np.max(np.abs(derivatives))) if len(derivatives) else 0.0,
            "ordering_sensitivity_stable": len({tick for tick in bands.values() if tick is not None}) <= 1,
        }
    loss = int(config["expected_failure_tick"])
    key_ticks.update(loss - int(offset) for offset in config["key_snapshot_offsets_from_loss"])
    available = {int(row["control_tick"]) for row in post}
    return result, sorted(tick for tick in key_ticks if tick in available and 0 <= tick < loss)


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
    executable = ROOT / config["executable"]
    command = [str(executable), str(ROOT / config["scene"]), "OUTPUT",
               config["case"], config["gain"], str(config["kp"]), str(config["kd"])]
    controls = []
    natives = []
    for label in ("a", "b"):
        control_path = output / f"control-{label}.csv"
        actual = command.copy(); actual[2] = str(control_path)
        subprocess.run(actual, cwd=ROOT, check=True)
        controls.append(control_path)
        natives.append(output / f"control-{label}_native.csv")
    control_rows = [read_csv(path) for path in controls]
    native_rows = [read_csv(path) for path in natives]
    phase41_rows = read_csv(ROOT / config["phase41_authority"])
    control_replay = semantic_error(control_rows[0], control_rows[1], {"wbc_time_s"})
    native_replay = semantic_error(native_rows[0], native_rows[1], set())
    phase41_error = semantic_error(control_rows[0], phase41_rows, {"wbc_time_s"})
    failure_tick = next((int(row["tick"]) for row in control_rows[0]
                         if row["contact_right"] != "1"), None)
    oracle = Oracle(config)
    contact_rows: list[dict[str, Any]] = []
    plant_rows = [oracle.evaluate(row, contact_rows) for row in native_rows[0]]
    write_csv(output / "plant.csv", plant_rows)
    write_csv(output / "contacts.csv", contact_rows)
    post = [row for row in plant_rows if row["record_kind"] == "post_command"]
    events, key_ticks = event_audit(post, config)
    write_json(output / "events.json", {"families": events, "key_snapshot_ticks": key_ticks})
    wbc_rows = control_for_wbc(control_rows[0])
    wbc_executable = ROOT / config["wbc_snapshot_executable"]
    actual_wbc = wbc_baselines(wbc_executable, wbc_rows, key_ticks)
    post_by_tick = {int(row["control_tick"]): row for row in post}
    native_post = {int(row["control_tick"]): row for row in native_rows[0]
                   if row["record_kind"] == "post_command"}
    balances = []
    counterfactuals = []
    torque_replay_error = 0.0
    wheel_step = float(config["wheel_rate_jacobian_step_rad_s"])
    for tick in key_ticks:
        actual_torque = np.asarray([float(actual_wbc[tick][f"tau{index}"]) for index in range(6)])
        logged = next(row for row in control_rows[0] if int(row["tick"]) == tick)
        logged_torque = np.asarray([float(logged[f"tau{index}"]) for index in range(6)])
        torque_replay_error = max(torque_replay_error, float(np.max(np.abs(actual_torque - logged_torque))))
        actual = oracle.evaluate(native_post[tick], [], -actual_torque)
        item: dict[str, Any] = {"tick": tick}
        for side, dof in enumerate(oracle.wheel_dadr):
            name = ("left", "right")[side]
            item[f"{name}_inertia"] = sum(actual[f"mass{dof * oracle.model.nv + column}"] *
                                           actual[f"qacc{column}"] for column in range(oracle.model.nv))
            for term in ("qfrc_bias", "qfrc_actuator", "qfrc_passive", "qfrc_applied",
                         "qfrc_contact_left", "qfrc_contact_right", "qfrc_other_constraint"):
                item[f"{name}_{term}"] = actual[f"{term}{dof}"]
            item[f"{name}_ddq"] = actual[f"qacc{dof}"]
            item[f"{name}_ddxi"] = actual[f"ddxi_{name}_m_s2"]
        item["full_residual"] = actual["full_dynamics_residual_max_abs"]
        item["contact_reconstruction_residual"] = actual["contact_applyft_jacobian_max_abs"]
        balances.append(item)

        zero_wbc = wbc_baselines(wbc_executable, wbc_rows, [tick], zero_rate_tick=tick)[tick]
        zero_torque = np.asarray([float(zero_wbc[f"tau{index}"]) for index in range(6)])
        zero_native = dict(native_post[tick])
        for address in oracle.wheel_dadr:
            zero_native[f"qvel{address}"] = "0"
        zero = oracle.evaluate(zero_native, [], -zero_torque)
        changed = [index for index in range(oracle.model.nv)
                   if float(zero_native[f"qvel{index}"]) != float(native_post[tick][f"qvel{index}"])]
        record = {"tick": tick, "changed_qvel_indices": changed,
                  "expected_wheel_qvel_indices": oracle.wheel_dadr,
                  "only_wheel_qvel_changed": set(changed).issubset(set(oracle.wheel_dadr)),
                  "zero_wheel_rates_exact": all(float(zero_native[f"qvel{address}"]) == 0.0
                                                for address in oracle.wheel_dadr),
                  "actual_torque_nm": actual_torque, "zero_rate_torque_nm": zero_torque}
        for name in ("left", "right"):
            for quantity in ("ddxi", "normal_load"):
                suffix = f"{name}_m_s2" if quantity == "ddxi" else f"{name}_n"
                key = f"{quantity}_{suffix}"
                record[f"actual_{key}"] = actual[key]
                record[f"zero_rate_{key}"] = zero[key]
                record[f"delta_{key}"] = zero[key] - actual[key]
        counterfactuals.append(record)
    write_csv(output / "wheel-row-balance.csv", balances)
    write_json(output / "zero-rate-counterfactual.json", counterfactuals)
    tolerance = config["closure_tolerances"]
    maximums = {
        "full_dynamics": max(row["full_dynamics_residual_max_abs"] for row in plant_rows),
        "contact_reconstruction": max(row["contact_applyft_jacobian_max_abs"] for row in plant_rows),
        "captured_qacc": max(row["captured_qacc_max_abs_error"] for row in plant_rows),
        "ddxi_half_step": max(row["ddxi_half_step_max_abs_error_m_s2"] for row in plant_rows),
    }
    initial = post_by_tick[0]
    final = post_by_tick[max(post_by_tick)]
    initial_common = 0.5 * (initial["ddxi_left_m_s2"] + initial["ddxi_right_m_s2"])
    initial_differential = 0.5 * (initial["ddxi_right_m_s2"] - initial["ddxi_left_m_s2"])
    final_common_rate = 0.5 * (final["dxi_left_m_s"] + final["dxi_right_m_s"])
    final_differential_rate = 0.5 * (final["dxi_right_m_s"] - final["dxi_left_m_s"])
    material_acceleration = max(abs(initial_common), abs(initial_differential)) >= float(
        config["event_detection"]["families"]["wheel_common_acceleration_m_s2"]["material_floor"])
    direction_consistent = ((abs(initial_common) < 1e-12 or initial_common * final_common_rate > 0) and
                            (abs(initial_differential) < 1e-12 or
                             initial_differential * final_differential_rate > 0))
    maximum_zero_rate_ddxi_effect = max(
        abs(record[f"delta_ddxi_{name}_m_s2"])
        for record in counterfactuals for name in ("left", "right"))
    maximum_zero_rate_load_effect = max(
        abs(record[f"delta_normal_load_{name}_n"])
        for record in counterfactuals for name in ("left", "right"))
    initial_load_asymmetry = abs(initial["normal_load_right_n"] - initial["normal_load_left_n"])
    coupled_material = (material_acceleration and not direction_consistent and
                        initial_load_asymmetry >= float(config["event_detection"]["families"]
                                                       ["normal_load_difference_n"]["material_floor"]) and
                        maximum_zero_rate_ddxi_effect >= float(config["counterfactual_material_ddxi_m_s2"]) and
                        maximum_zero_rate_load_effect >= float(config["counterfactual_material_load_n"]))
    classification = ("P42-A_fixed_request_realization_not_rolling_equilibrium"
                      if material_acceleration and direction_consistent else
                      "P42-E_multiple_coupled_causes" if coupled_material else
                      "P42-U_unresolved")
    gates = {
        "phase41_failure_reproduced": failure_tick == int(config["expected_failure_tick"]),
        "control_replay": control_replay <= float(config["semantic_tolerance"]),
        "native_replay": native_replay <= float(config["semantic_tolerance"]),
        "phase41_semantic_parity": phase41_error <= float(config["semantic_tolerance"]),
        "wbc_snapshot_replay": torque_replay_error <= float(config["semantic_tolerance"]),
        "full_dynamics_closure": maximums["full_dynamics"] <= float(tolerance["full_dynamics_max_abs"]),
        "contact_reconstruction": maximums["contact_reconstruction"] <= float(tolerance["contact_reconstruction_max_abs"]),
        "captured_qacc_parity": maximums["captured_qacc"] <= float(tolerance["captured_qacc_max_abs"]),
        "ddxi_oracle": maximums["ddxi_half_step"] <= float(tolerance["ddxi_half_step_max_abs_m_s2"]),
        "counterfactual_changes_only_wheel_rate": all(
            record["only_wheel_qvel_changed"] and record["zero_wheel_rates_exact"]
            for record in counterfactuals),
        "classification_resolved": classification != "P42-U_unresolved",
    }
    summary = {
        "classification": classification, "pass": all(gates.values()), "gates": gates,
        "failure_tick": failure_tick, "key_snapshot_ticks": key_ticks,
        "maximum_residuals": maximums, "control_replay_max_abs_error": control_replay,
        "native_replay_max_abs_error": native_replay,
        "phase41_semantic_max_abs_error": phase41_error,
        "wbc_snapshot_torque_max_abs_error_nm": torque_replay_error,
        "tick0": {"common_ddxi_m_s2": initial_common,
                  "differential_ddxi_m_s2": initial_differential,
                  "normal_load_asymmetry_n": initial_load_asymmetry,
                  "direction_matches_final_rate": direction_consistent},
        "counterfactual_maximum_effect": {
            "ddxi_m_s2": maximum_zero_rate_ddxi_effect,
            "normal_load_n": maximum_zero_rate_load_effect},
        "final_bilateral": {"tick": max(post_by_tick),
                             "common_dxi_m_s": final_common_rate,
                             "differential_dxi_m_s": final_differential_rate},
        "repair": False, "phase34_run": False,
    }
    write_json(output / "summary.json", summary)
    sources = [config_path, ROOT / config["scene"], executable,
               ROOT / config["phase41_config"], ROOT / config["phase41_authority"],
               ROOT / config["phase41_summary"], ROOT / config["phase31_contract"],
               wbc_executable, Path(__file__).resolve(),
               ROOT / "ros_ws/src/wheel_leg_mujoco/src/phase35_workspace_attribution_loop.cpp"]
    write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": args.replay_of, "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
        "no_repair": True, "phase34_run": False,
    })
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
