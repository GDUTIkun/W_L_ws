#!/usr/bin/env python3
"""Run the Phase-19 full-state pre-freeze controller gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from solve_mujoco_planar_equilibrium import DEFAULT_SCENE, object_id

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase19_planar_prefreeze.json"
DEFAULT_EQUILIBRIUM = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-equilibrium/equilibrium.json"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-planar-prefreeze-v2"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pitch(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (x * x + y * y))


class Plant:
    def __init__(self, scene: Path, equilibrium: np.ndarray, config: dict[str, Any]) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.eq = equilibrium
        self.config = config
        joint_names = (
            "right_hip_joint", "right_knee_joint", "left_hip_joint",
            "left_knee_joint", "base_z_joint", "right_connect1_joint",
            "right_connect2_joint", "left_connect1_joint", "left_connect2_joint",
        )
        self.qeq = self.model.qpos0.copy()
        self.qeq[[
            self.model.jnt_qposadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
            for name in joint_names
        ]] = equilibrium[:9]
        active = ("left_hip", "left_knee", "right_hip", "right_knee")
        self.active_qpos = np.asarray([
            self.model.jnt_qposadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name + "_joint")]
            for name in active
        ])
        self.active_dofs = np.asarray([
            self.model.jnt_dofadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name + "_joint")]
            for name in active
        ])
        self.active_actuators = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_torque")
            for name in active
        ])
        self.wheel_actuators = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ("left_wheel_torque", "right_wheel_torque")
        ])
        self.base_x_qpos = self.model.jnt_qposadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_x_joint")]
        self.base_x_dof = self.model.jnt_dofadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_x_joint")]
        self.pitch_qpos = self.model.jnt_qposadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_pitch_joint")]
        self.pitch_dof = self.model.jnt_dofadr[object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_pitch_joint")]
        self.base_weld = object_id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
        self.site = object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
        self.floor = object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.wheels = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, side + "_wheel_collision")
            for side in ("left", "right")
        ]
        self.reference = equilibrium[[2, 3, 0, 1]]
        self.support = equilibrium[9:]
        self.anchor = self.observe(self.reset(np.zeros(4)))[0]

    def reset(self, state: np.ndarray) -> mujoco.MjData:
        data = mujoco.MjData(self.model)
        data.qpos[:] = self.qeq
        data.qvel[:] = 0.0
        data.qpos[self.base_x_qpos] += state[0]
        data.qvel[self.base_x_dof] = state[1]
        data.qpos[self.pitch_qpos] += state[2]
        data.qvel[self.pitch_dof] = state[3]
        data.eq_active[:] = self.model.eq_active0
        data.eq_active[self.base_weld] = 0
        mujoco.mj_forward(self.model, data)
        return data

    def observe(self, data: mujoco.MjData) -> np.ndarray:
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, data.site_xmat[self.site])
        linear = np.zeros((3, self.model.nv))
        angular = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, data, linear, angular, self.site)
        return np.asarray([
            data.site_xpos[self.site, 0] - getattr(self, "anchor", 0.0),
            linear[0] @ data.qvel,
            pitch(quaternion),
            angular[1] @ data.qvel,
        ])

    def write_control(self, data: mujoco.MjData, wheel_native: float) -> None:
        kp = float(self.config["leg_kp_nm_per_rad"])
        kd = float(self.config["leg_kd_nm_s_per_rad"])
        leg_limit = float(self.config["leg_torque_limit_nm"])
        data.ctrl[:] = 0.0
        data.ctrl[self.active_actuators] = np.clip(
            self.support
            + kp * (self.reference - data.qpos[self.active_qpos])
            - kd * data.qvel[self.active_dofs],
            -leg_limit, leg_limit,
        )
        data.ctrl[self.wheel_actuators] = wheel_native

    def step_tick(self, data: mujoco.MjData, wheel_native: float) -> None:
        self.write_control(data, wheel_native)
        for _ in range(int(self.config["physics_steps_per_control"])):
            mujoco.mj_step(self.model, data)

    def reduced_tick(self, state: np.ndarray, wheel_native: float) -> np.ndarray:
        data = self.reset(state)
        self.step_tick(data, wheel_native)
        return self.observe(data)

    def full_tick(self, state: np.ndarray, gain: np.ndarray) -> np.ndarray:
        data = mujoco.MjData(self.model)
        data.qpos[:] = self.qeq + state[: self.model.nq]
        data.qvel[:] = state[self.model.nq :]
        data.eq_active[:] = self.model.eq_active0
        data.eq_active[self.base_weld] = 0
        mujoco.mj_forward(self.model, data)
        self.step_tick(data, float(gain @ self.observe(data)))
        return np.r_[data.qpos - self.qeq, data.qvel]

    def bilateral_contact(self, data: mujoco.MjData) -> bool:
        present = set()
        for contact in data.contact[: data.ncon]:
            pair = {int(contact.geom1), int(contact.geom2)}
            for side, wheel in enumerate(self.wheels):
                if pair == {self.floor, wheel}:
                    present.add(side)
        return len(present) == 2


def lqr(a: np.ndarray, b: np.ndarray, config: dict[str, Any]) -> np.ndarray:
    q = np.diag(np.asarray(config["q_diagonal"], dtype=float))
    r = float(config["r"])
    p = q.copy()
    column = b.reshape(-1, 1)
    for _ in range(int(config["maximum_iterations"])):
        denominator = r + float((column.T @ p @ column).item())
        gain = column.T @ p @ a / denominator
        updated = q + a.T @ p @ a - a.T @ p @ column @ gain
        if np.max(np.abs(updated - p)) <= float(config["tolerance"]):
            p = updated
            break
        p = updated
    return (column.T @ p @ a / (r + float((column.T @ p @ column).item()))).reshape(4)


def run(plant: Plant) -> dict[str, Any]:
    finite = plant.config["finite_difference"]
    origin = np.zeros(4)
    affine = plant.reduced_tick(origin, 0.0)
    a = np.zeros((4, 4))
    for index, epsilon in enumerate(finite["reduced_state"]):
        plus, minus = origin.copy(), origin.copy()
        plus[index], minus[index] = epsilon, -epsilon
        a[:, index] = (
            plant.reduced_tick(plus, 0.0) - plant.reduced_tick(minus, 0.0)
        ) / (2.0 * epsilon)
    input_epsilon = float(finite["wheel_torque_nm"])
    b_native = (
        plant.reduced_tick(origin, input_epsilon)
        - plant.reduced_tick(origin, -input_epsilon)
    ) / (2.0 * input_epsilon)
    b_canonical = -b_native
    gain = lqr(a, b_canonical, plant.config["lqr"])
    controllability = np.column_stack([
        np.linalg.matrix_power(a, power) @ b_canonical for power in range(4)
    ])
    reduced_poles = np.linalg.eigvals(a - np.outer(b_canonical, gain))

    dimension = plant.model.nq + plant.model.nv
    full = np.zeros((dimension, dimension))
    zero = np.zeros(dimension)
    for index in range(dimension):
        epsilon = float(
            finite["full_qpos"] if index < plant.model.nq else finite["full_qvel"]
        )
        plus, minus = zero.copy(), zero.copy()
        plus[index], minus[index] = epsilon, -epsilon
        full[:, index] = (
            plant.full_tick(plus, gain) - plant.full_tick(minus, gain)
        ) / (2.0 * epsilon)
    full_poles = np.linalg.eigvals(full)

    cases = []
    ticks = int(round(float(plant.config["duration_s"]) / float(plant.config["control_period_s"])))
    for case in plant.config["cases"]:
        data = plant.reset(np.asarray(case["initial_state"], dtype=float))
        maximum_pitch = 0.0
        bilateral = 0
        completed = 0
        for _ in range(ticks):
            state = plant.observe(data)
            wheel = float(np.clip(
                gain @ state,
                -float(plant.config["wheel_torque_limit_nm"]),
                float(plant.config["wheel_torque_limit_nm"]),
            ))
            plant.step_tick(data, wheel)
            state = plant.observe(data)
            maximum_pitch = max(maximum_pitch, abs(float(state[2])))
            bilateral += plant.bilateral_contact(data)
            completed += 1
            if not np.all(np.isfinite(data.qpos)) or maximum_pitch > 0.5:
                break
        cases.append({
            "id": case["id"],
            "completed_ticks": completed,
            "maximum_abs_pitch_rad": maximum_pitch,
            "bilateral_contact_fraction": bilateral / completed,
            "final_state": plant.observe(data).tolist(),
        })

    gates = plant.config["gates"]
    checks = {
        "reduced_controllability": int(np.linalg.matrix_rank(controllability)) == int(gates["reduced_controllability_rank"]),
        "reduced_stability": float(np.max(np.abs(reduced_poles))) <= float(gates["reduced_spectral_radius_max"]),
        "full_stability": float(np.max(np.abs(full_poles))) <= float(gates["full_spectral_radius_max"]),
        "nonlinear_holdouts": all(
            case["completed_ticks"] == ticks
            and case["maximum_abs_pitch_rad"] <= float(gates["maximum_pitch_rad"])
            and case["bilateral_contact_fraction"] >= float(gates["minimum_bilateral_contact_fraction"])
            for case in cases
        ),
    }
    return {
        "pass": all(checks.values()),
        "decision": "IMPLEMENT_CORE" if all(checks.values()) else "REWORK",
        "checks": checks,
        "reduced_model": {
            "A": a.tolist(), "B_native": b_native.tolist(),
            "B_canonical": b_canonical.tolist(), "affine_drift": affine.tolist(),
            "gain_canonical_tau_equals_minus_Kx": gain.tolist(),
            "controllability_rank": int(np.linalg.matrix_rank(controllability)),
            "closed_loop_poles": [[float(value.real), float(value.imag)] for value in reduced_poles],
            "spectral_radius": float(np.max(np.abs(reduced_poles))),
        },
        "full_model": {
            "dimension": dimension,
            "closed_loop_poles": [[float(value.real), float(value.imag)] for value in full_poles],
            "spectral_radius": float(np.max(np.abs(full_poles))),
            "unstable_pole_count": int(np.sum(np.abs(full_poles) > 1.0 + 1e-6)),
        },
        "cases": cases,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--equilibrium", type=Path, default=DEFAULT_EQUILIBRIUM)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(arguments.config.resolve().read_text())
    equilibrium = np.asarray(json.loads(arguments.equilibrium.resolve().read_text())["candidate"])
    summary = run(Plant(arguments.scene.resolve(), equilibrium, config))
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    manifest = {
        "scene_sha256": sha256(arguments.scene.resolve()),
        "config_sha256": sha256(arguments.config.resolve()),
        "equilibrium_sha256": sha256(arguments.equilibrium.resolve()),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "summary_sha256": sha256(summary_path),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
