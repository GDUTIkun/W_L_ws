#!/usr/bin/env python3
"""Validate the Phase-23 canonical 16D NMPC state mapping.

The production state does not expose MuJoCo.  This independent oracle uses the
compiled nominal plant only to establish golden vectors and finite-difference
checks for the frozen RobotState/base-control-site contract.
"""

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
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
EXPERIMENTS = ROOT / "tools/experiments"
sys.path.insert(0, str(EXPERIMENTS))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402

DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase23_nmpc_state_oracle_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def left_jacobian_inverse(rotation_vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotation_vector))
    hat = skew(rotation_vector)
    if theta < 1e-7:
        return np.eye(3) - 0.5 * hat + (1.0 / 12.0) * hat @ hat
    coefficient = 1.0 / (theta * theta) - (1.0 + np.cos(theta)) / (
        2.0 * theta * np.sin(theta)
    )
    return np.eye(3) - 0.5 * hat + coefficient * hat @ hat


def point_jacobian(oracle: Oracle, body: int, point: np.ndarray) -> np.ndarray:
    linear = np.zeros((3, oracle.model.nv))
    angular = np.zeros((3, oracle.model.nv))
    mujoco.mj_jac(oracle.model, oracle.data, linear, angular, point, body)
    return linear


def map_state(oracle: Oracle, qpos: np.ndarray, qvel: np.ndarray) -> tuple[np.ndarray, dict[str, Any]]:
    oracle.forward(qpos, qvel)
    base = oracle.base_control_site
    base_position = oracle.data.site_xpos[base].copy()
    rotation_n_from_b = oracle.data.site_xmat[base].reshape(3, 3).copy()
    rotation_vector = Rotation.from_matrix(rotation_n_from_b).as_rotvec()
    base_linear_jacobian, base_angular_jacobian = oracle.site_jacobian(base)
    base_linear_velocity = base_linear_jacobian @ qvel
    base_angular_velocity = base_angular_jacobian @ qvel

    relative_positions = []
    relative_speeds = []
    wheel_positions = []
    for body in oracle.wheel_bodies:
        wheel_position = oracle.data.xpos[body].copy()
        wheel_velocity = point_jacobian(oracle, body, wheel_position) @ qvel
        relative_b = rotation_n_from_b.T @ (wheel_position - base_position)
        omega_b = rotation_n_from_b.T @ base_angular_velocity
        relative_velocity_b = (
            rotation_n_from_b.T @ (wheel_velocity - base_linear_velocity)
            - np.cross(omega_b, relative_b)
        )
        wheel_positions.append(wheel_position)
        relative_positions.append(float(relative_b[0]))
        relative_speeds.append(float(relative_velocity_b[0]))

    state = np.r_[
        base_position,
        rotation_vector,
        base_linear_velocity,
        base_angular_velocity,
        relative_positions,
        relative_speeds,
    ]
    details = {
        "rotation_n_from_b": rotation_n_from_b,
        "wheel_positions_n_m": np.asarray(wheel_positions),
        "base_linear_jacobian": base_linear_jacobian,
        "base_angular_jacobian": base_angular_jacobian,
    }
    return state, details


def state_position(state: np.ndarray) -> np.ndarray:
    return np.r_[state[:6], state[12:14]]


def evaluate_sample(oracle: Oracle, sample: dict[str, Any], step: float) -> dict[str, Any]:
    qpos = oracle.sample_qpos(sample)
    reduction, _ = oracle.reduction(qpos)
    reduced_velocity = np.asarray(sample["reduced_velocity"], dtype=float)
    qvel = reduction @ reduced_velocity
    state, details = map_state(oracle, qpos, qvel)

    plus_qpos = oracle.integrate_flow(qpos, reduced_velocity, step)
    minus_qpos = oracle.integrate_flow(qpos, reduced_velocity, -step)
    plus_state, _ = map_state(oracle, plus_qpos, np.zeros(oracle.model.nv))
    minus_state, _ = map_state(oracle, minus_qpos, np.zeros(oracle.model.nv))
    position_fd = (state_position(plus_state) - state_position(minus_state)) / (2.0 * step)
    expected_position_rate = np.r_[
        state[6:9],
        left_jacobian_inverse(state[3:6]) @ state[9:12],
        state[14:16],
    ]

    base_twist = np.r_[
        details["base_linear_jacobian"] @ qvel,
        details["base_angular_jacobian"] @ qvel,
    ]
    repeated, _ = map_state(oracle, qpos, qvel)
    quaternion = qpos[3:7].copy()
    sign_flipped = qpos.copy()
    sign_flipped[3:7] = -quaternion
    sign_state, _ = map_state(oracle, sign_flipped, qvel)

    return {
        "state": state.tolist(),
        "base_twist_error": float(np.max(np.abs(base_twist - reduced_velocity[:6]))),
        "state_position_fd_error": float(np.max(np.abs(position_fd - expected_position_rate))),
        "rotation_rate_fd_error": float(np.max(np.abs(position_fd[3:6] - expected_position_rate[3:6]))),
        "wheel_relative_velocity_fd_error": float(np.max(np.abs(position_fd[6:8] - state[14:16]))),
        "quaternion_sign_invariance_error": float(np.max(np.abs(sign_state - state))),
        "repeat_error": float(np.max(np.abs(repeated - state))),
        "wheel_positions_n_m": details["wheel_positions_n_m"].tolist(),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")

    config_path = arguments.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    model_config_path = (ROOT / config["source_model_oracle"]).resolve()
    model_config, model_inputs = load_config(model_config_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_config, equilibrium)
    step = float(config["finite_difference_step_s"])
    samples = {
        sample["id"]: evaluate_sample(oracle, sample, step) for sample in config["samples"]
    }
    thresholds = config["thresholds"]
    maxima = {
        key: max(sample[key] for sample in samples.values())
        for key in (
            "base_twist_error",
            "state_position_fd_error",
            "rotation_rate_fd_error",
            "wheel_relative_velocity_fd_error",
            "quaternion_sign_invariance_error",
            "repeat_error",
        )
    }
    equilibrium_state = np.asarray(samples["equilibrium"]["state"])
    equilibrium_speed = float(np.max(np.abs(np.r_[
        equilibrium_state[6:12], equilibrium_state[14:16]
    ])))
    gates = {
        "base_twist": maxima["base_twist_error"] <= thresholds["maximum_base_twist_error"],
        "position_rate": maxima["state_position_fd_error"] <= thresholds["maximum_state_position_fd_error"],
        "rotation_chart_rate": maxima["rotation_rate_fd_error"] <= thresholds["maximum_rotation_rate_fd_error"],
        "wheel_relative_rate": maxima["wheel_relative_velocity_fd_error"] <= thresholds["maximum_wheel_relative_velocity_fd_error"],
        "equilibrium_speed": equilibrium_speed <= thresholds["maximum_equilibrium_speed"],
        "quaternion_sign": maxima["quaternion_sign_invariance_error"] <= thresholds["maximum_quaternion_sign_invariance_error"],
        "determinism": maxima["repeat_error"] <= thresholds["maximum_repeat_error"],
        "finite": all(np.all(np.isfinite(sample["state"])) for sample in samples.values()),
        "chart": all(np.linalg.norm(sample["state"][3:6]) <= config["rotation_chart_maximum_norm_rad"] for sample in samples.values()),
    }
    summary = {
        "schema_version": 1,
        "phase": 23,
        "profile": config["profile"],
        "state_order": config["state_order"],
        "state_origin": "base_control_frame site",
        "orientation": "world-axis shortest-arc rotation vector for R_N_from_B",
        "twist": "base-control-site spatial linear/angular velocity expressed in N",
        "wheel_relative": "x component in B of wheel-center minus base-control-site, with rotating-frame derivative",
        "gates": gates,
        "pass": all(gates.values()),
        "maxima": {**maxima, "equilibrium_speed": equilibrium_speed},
        "samples": samples,
    }
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": __import__("scipy").__version__,
        "mujoco": mujoco.__version__,
        "hardware_data": False,
        "config": str(config_path.relative_to(ROOT)),
        "config_sha256": sha256(config_path),
        "model_config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "equilibrium": str(equilibrium_path.relative_to(ROOT)),
        "equilibrium_sha256": sha256(equilibrium_path),
        "validator": str(Path(__file__).resolve().relative_to(ROOT)),
        "validator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {"summary.json": sha256(summary_path)},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
