#!/usr/bin/env python3
"""Validate the Phase-19 planar state, sign, and rolling contract."""

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

from solve_mujoco_planar_equilibrium import DEFAULT_CONFIG, DEFAULT_SCENE, Problem

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_EQUILIBRIUM = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-equilibrium/equilibrium.json"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-state-contract"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pitch(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (x * x + y * y))


def site_state(problem: Problem) -> np.ndarray:
    model, data = problem.model, problem.data
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, data.site_xmat[site])
    linear_jacobian = np.zeros((3, model.nv))
    angular_jacobian = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, linear_jacobian, angular_jacobian, site)
    return np.asarray([
        data.site_xpos[site, 0],
        float(linear_jacobian[0] @ data.qvel),
        pitch(quaternion),
        float(angular_jacobian[1] @ data.qvel),
    ])


def validate(problem: Problem, candidate: np.ndarray) -> dict[str, Any]:
    problem.apply(candidate)
    model, data = problem.model, problem.data
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
    base_x_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_x_joint")
    pitch_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_pitch_joint")
    base_x_qpos, base_x_dof = model.jnt_qposadr[base_x_joint], model.jnt_dofadr[base_x_joint]
    pitch_qpos, pitch_dof = model.jnt_qposadr[pitch_joint], model.jnt_dofadr[pitch_joint]
    reference_qpos = data.qpos.copy()
    epsilon = 1e-6
    finite_position = np.zeros(model.nv)
    finite_pitch = np.zeros(model.nv)
    for joint in range(model.njnt):
        qpos_address = int(model.jnt_qposadr[joint])
        dof_address = int(model.jnt_dofadr[joint])
        plus, minus = reference_qpos.copy(), reference_qpos.copy()
        plus[qpos_address] += epsilon
        minus[qpos_address] -= epsilon
        data.qpos[:] = plus
        mujoco.mj_forward(model, data)
        plus_x, plus_pitch = site_state(problem)[[0, 2]]
        data.qpos[:] = minus
        mujoco.mj_forward(model, data)
        minus_x, minus_pitch = site_state(problem)[[0, 2]]
        finite_position[dof_address] = (plus_x - minus_x) / (2.0 * epsilon)
        finite_pitch[dof_address] = (plus_pitch - minus_pitch) / (2.0 * epsilon)
    data.qpos[:] = reference_qpos
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    linear_jacobian = np.zeros((3, model.nv))
    angular_jacobian = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, linear_jacobian, angular_jacobian, site)

    data.qvel[:] = 0.0
    data.qvel[base_x_dof] = 0.37
    state_x_velocity = site_state(problem)
    data.qvel[:] = 0.0
    data.qvel[pitch_dof] = -0.29
    state_pitch_velocity = site_state(problem)
    data.qvel[:] = 0.0

    rolling = {}
    for side in ("left", "right"):
        joint = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_wheel_joint"
        )
        axis = data.xaxis[joint].copy()
        native_no_slip = -np.cross(axis, np.array([0.0, 0.0, -0.05]))
        rolling[side] = native_no_slip.tolist()

    metrics = {
        "maximum_site_x_jacobian_error": float(np.max(np.abs(finite_position - linear_jacobian[0]))),
        "maximum_pitch_jacobian_error": float(np.max(np.abs(finite_pitch - angular_jacobian[1]))),
        "positive_base_x_position_derivative": float(finite_position[base_x_dof]),
        "positive_pitch_derivative": float(finite_pitch[pitch_dof]),
        "base_x_velocity_oracle": float(state_x_velocity[1]),
        "pitch_rate_oracle": float(state_pitch_velocity[3]),
        "native_positive_wheel_no_slip_displacement_m_per_rad": rolling,
        "canonical_adapter_relations": {
            "joint_position": "canonical=-native+offset",
            "joint_velocity": "canonical=-native",
            "actuator_torque": "native=-canonical",
            "canonical_positive_wheel_roll": "-X",
        },
        "finite": bool(np.all(np.isfinite(finite_position)) and np.all(np.isfinite(finite_pitch))),
    }
    gates = {
        "site_x_jacobian": metrics["maximum_site_x_jacobian_error"] <= 1e-9,
        "pitch_jacobian": metrics["maximum_pitch_jacobian_error"] <= 1e-9,
        "base_x_sign": abs(metrics["positive_base_x_position_derivative"] - 1.0) <= 1e-9,
        "pitch_sign": abs(metrics["positive_pitch_derivative"] - 1.0) <= 1e-9,
        "base_x_velocity": abs(metrics["base_x_velocity_oracle"] - 0.37) <= 1e-12,
        "pitch_rate": abs(metrics["pitch_rate_oracle"] + 0.29) <= 1e-12,
        "native_rolling": all(
            np.allclose(value, [0.05, 0.0, 0.0], rtol=0.0, atol=1e-9)
            for value in rolling.values()
        ),
        "finite": metrics["finite"],
    }
    return {"pass": all(gates.values()), "metrics": metrics, "gates": gates}


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
    equilibrium = json.loads(arguments.equilibrium.resolve().read_text())
    result = validate(Problem(arguments.scene.resolve(), config), np.asarray(equilibrium["candidate"]))
    summary = output / "summary.json"
    summary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    manifest = {
        "scene_sha256": sha256(arguments.scene.resolve()),
        "config_sha256": sha256(arguments.config.resolve()),
        "equilibrium_sha256": sha256(arguments.equilibrium.resolve()),
        "validator_sha256": sha256(Path(__file__).resolve()),
        "summary_sha256": sha256(summary),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
