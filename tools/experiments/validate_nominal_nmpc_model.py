#!/usr/bin/env python3
"""Validate the revised Phase-23 locked-composite base model."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import mujoco
import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/experiments"))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402
from validate_nominal_nmpc_state import left_jacobian_inverse  # noqa: E402

DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase23_nmpc_model_oracle_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def full_mass(oracle: Oracle) -> np.ndarray:
    result = np.zeros((oracle.model.nv, oracle.model.nv))
    mujoco.mj_fullM(oracle.model, result, oracle.data.qM)
    return result


def locked_terms(
    oracle: Oracle, qpos: np.ndarray, base_twist: np.ndarray, difference_step: float
) -> tuple[np.ndarray, np.ndarray]:
    reduction, _ = oracle.reduction(qpos)
    velocity = np.r_[base_twist, np.zeros(6)]
    qvel = reduction @ velocity
    plus = oracle.integrate_flow(qpos, velocity, difference_step)
    minus = oracle.integrate_flow(qpos, velocity, -difference_step)
    plus_reduction, _ = oracle.reduction(plus)
    minus_reduction, _ = oracle.reduction(minus)
    reduction_dot_velocity = (
        (plus_reduction - minus_reduction) / (2.0 * difference_step)
    ) @ velocity
    oracle.forward(qpos, qvel)
    mass = full_mass(oracle)
    bias = reduction.T @ (oracle.data.qfrc_bias.copy() + mass @ reduction_dot_velocity)
    return reduction.T @ mass @ reduction, bias


def composite_parameters(base_mass: np.ndarray) -> tuple[float, np.ndarray, np.ndarray, float]:
    mass = float(np.trace(base_mass[:3, :3]) / 3.0)
    cross = -base_mass[:3, 3:6] / mass
    com_b = np.array([cross[2, 1], cross[0, 2], cross[1, 0]])
    inertia_b = base_mass[3:6, 3:6]
    inertia_com_b = inertia_b - mass * (
        float(com_b @ com_b) * np.eye(3) - np.outer(com_b, com_b)
    )
    reconstructed = np.block([
        [mass * np.eye(3), -mass * np.array([
            [0.0, -com_b[2], com_b[1]],
            [com_b[2], 0.0, -com_b[0]],
            [-com_b[1], com_b[0], 0.0],
        ])],
        [mass * np.array([
            [0.0, -com_b[2], com_b[1]],
            [com_b[2], 0.0, -com_b[0]],
            [-com_b[1], com_b[0], 0.0],
        ]), inertia_b],
    ])
    return mass, com_b, inertia_com_b, float(np.max(np.abs(reconstructed - base_mass)))


def rotation_matrix(rotation_vector: np.ndarray) -> np.ndarray:
    return Rotation.from_rotvec(rotation_vector).as_matrix()


def dynamics(
    state: np.ndarray,
    wrench: np.ndarray,
    mass: float,
    com_b: np.ndarray,
    inertia_com_b: np.ndarray,
    reference_rotation: np.ndarray,
) -> np.ndarray:
    rotation_vector = state[3:6]
    linear_velocity = state[6:9]
    angular_velocity = state[9:12]
    rotation = rotation_matrix(rotation_vector) @ reference_rotation
    total_force_n = rotation @ (wrench[:3] + wrench[6:9])
    total_moment_b_n = rotation @ (wrench[3:6] + wrench[9:12])
    com_offset_n = rotation @ com_b
    inertia_com_n = rotation @ inertia_com_b @ rotation.T
    moment_com_n = total_moment_b_n - np.cross(com_offset_n, total_force_n)
    angular_acceleration = np.linalg.solve(
        inertia_com_n,
        moment_com_n - np.cross(angular_velocity, inertia_com_n @ angular_velocity),
    )
    com_acceleration = total_force_n / mass + np.array([0.0, 0.0, -9.81])
    base_acceleration = (
        com_acceleration
        - np.cross(angular_acceleration, com_offset_n)
        - np.cross(angular_velocity, np.cross(angular_velocity, com_offset_n))
    )
    return np.r_[
        linear_velocity,
        left_jacobian_inverse(rotation_vector) @ angular_velocity,
        base_acceleration,
        angular_acceleration,
    ]


def rk4(function: Callable[[np.ndarray], np.ndarray], state: np.ndarray, step: float) -> np.ndarray:
    k1 = function(state)
    k2 = function(state + 0.5 * step * k1)
    k3 = function(state + 0.5 * step * k2)
    k4 = function(state + step * k3)
    return state + step * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def central_jacobian(function: Callable[[np.ndarray], np.ndarray], value: np.ndarray, step: float) -> np.ndarray:
    result = np.zeros((function(value).size, value.size))
    for column in range(value.size):
        delta = np.zeros_like(value)
        delta[column] = step * max(1.0, abs(value[column]))
        result[:, column] = (function(value + delta) - function(value - delta)) / (2.0 * delta[column])
    return result


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
    model_path = (ROOT / config["source_model_oracle"]).resolve()
    model_config, model_inputs = load_config(model_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_config, equilibrium)
    zero_sample = {"base_rotation_vector_rad": [0.0] * 3, "canonical_joint_delta_rad": [0.0] * 6}
    equilibrium_qpos = oracle.sample_qpos(zero_sample)
    equilibrium_mass, _ = locked_terms(
        oracle, equilibrium_qpos, np.zeros(6), config["reduction_difference_step_s"]
    )
    mass, com_b, inertia_com_b, spatial_error = composite_parameters(equilibrium_mass[:6, :6])
    source_equilibrium_wrench = np.asarray(config["equilibrium_wrench_flu"], dtype=float)
    required_force_b = np.array([0.0, 0.0, mass * 9.81])
    required_moment_b = np.cross(com_b, required_force_b)
    equilibrium_wrench = source_equilibrium_wrench.copy()
    force_correction = required_force_b - (
        equilibrium_wrench[:3] + equilibrium_wrench[6:9]
    )
    moment_correction = required_moment_b - (
        equilibrium_wrench[3:6] + equilibrium_wrench[9:12]
    )
    equilibrium_wrench[:3] += 0.5 * force_correction
    equilibrium_wrench[6:9] += 0.5 * force_correction
    equilibrium_wrench[3:6] += 0.5 * moment_correction
    equilibrium_wrench[9:12] += 0.5 * moment_correction
    dt = float(config["sample_period_s"])
    fd_step = float(config["jacobian_difference_step"])
    samples: dict[str, Any] = {}
    for sample in config["samples"]:
        rotation_vector = np.asarray(sample["rotation_vector_rad"], dtype=float)
        reference_rotation_vector = np.asarray(
            sample["reference_rotation_vector_rad"], dtype=float
        )
        reference_rotation = rotation_matrix(reference_rotation_vector)
        physical_rotation = rotation_matrix(rotation_vector) @ reference_rotation
        physical_rotation_vector = Rotation.from_matrix(physical_rotation).as_rotvec()
        state = np.r_[
            np.array([0.0, 0.0, oracle.base_z]),
            rotation_vector,
            np.asarray(sample["linear_velocity_n_m_s"], dtype=float),
            np.asarray(sample["angular_velocity_n_rad_s"], dtype=float),
        ]
        wrench = equilibrium_wrench + np.asarray(sample["wrench_delta_flu"], dtype=float)
        qpos = oracle.sample_qpos({
            "base_rotation_vector_rad": physical_rotation_vector.tolist(),
            "canonical_joint_delta_rad": [0.0] * 6,
        })
        exact_mass, exact_bias = locked_terms(
            oracle, qpos, state[6:12], config["reduction_difference_step_s"]
        )
        rotation = physical_rotation
        generalized_wrench = np.r_[
            rotation @ (wrench[:3] + wrench[6:9]),
            rotation @ (wrench[3:6] + wrench[9:12]),
        ]
        exact_acceleration = np.linalg.solve(
            exact_mass[:6, :6], generalized_wrench - exact_bias[:6]
        )
        predicted = dynamics(
            state, wrench, mass, com_b, inertia_com_b, reference_rotation
        )
        flow = lambda candidate: dynamics(
            candidate, wrench, mass, com_b, inertia_com_b, reference_rotation
        )
        next_rk4 = rk4(flow, state, dt)
        half = rk4(flow, rk4(flow, state, 0.5 * dt), 0.5 * dt)
        dop853 = solve_ivp(
            lambda _time, value: flow(value), [0.0, dt], state,
            method="DOP853", rtol=1e-12, atol=1e-14
        ).y[:, -1]
        combined = np.r_[state, wrench]
        flow_combined = lambda value: dynamics(
            value[:12], value[12:], mass, com_b, inertia_com_b,
            reference_rotation
        )
        jacobian = central_jacobian(flow_combined, combined, fd_step)
        jacobian_fine = central_jacobian(flow_combined, combined, 0.5 * fd_step)
        discrete_combined = lambda value: rk4(
            lambda candidate: dynamics(
                candidate, value[12:], mass, com_b, inertia_com_b,
                reference_rotation
            ), value[:12], dt
        )
        discrete_jacobian = central_jacobian(discrete_combined, combined, fd_step)
        discrete_jacobian_fine = central_jacobian(
            discrete_combined, combined, 0.5 * fd_step
        )
        repeated = dynamics(
            state, wrench, mass, com_b, inertia_com_b, reference_rotation
        )
        samples[sample["id"]] = {
            "state": state.tolist(),
            "reference_rotation_vector_rad": reference_rotation_vector.tolist(),
            "wrench": wrench.tolist(),
            "continuous": predicted.tolist(),
            "rk4_next": next_rk4.tolist(),
            "state_jacobian": jacobian[:, :12].tolist(),
            "input_jacobian": jacobian[:, 12:].tolist(),
            "rk4_state_jacobian": discrete_jacobian[:, :12].tolist(),
            "rk4_input_jacobian": discrete_jacobian[:, 12:].tolist(),
            "continuous_model_error": float(np.max(np.abs(predicted[6:12] - exact_acceleration))),
            "rk4_vs_dop853_error": float(np.max(np.abs(next_rk4 - dop853))),
            "rk4_step_doubling_error": float(np.max(np.abs(next_rk4 - half))),
            "jacobian_step_consistency": float(np.max(np.abs(jacobian - jacobian_fine))),
            "rk4_jacobian_step_consistency": float(np.max(np.abs(
                discrete_jacobian - discrete_jacobian_fine
            ))),
            "repeat_error": float(np.max(np.abs(repeated - predicted))),
        }

    maxima = {
        key: max(sample[key] for sample in samples.values())
        for key in (
            "continuous_model_error", "rk4_vs_dop853_error",
            "rk4_step_doubling_error", "jacobian_step_consistency", "repeat_error"
            , "rk4_jacobian_step_consistency"
        )
    }
    equilibrium_derivative = float(np.max(np.abs(samples["equilibrium"]["continuous"])))
    thresholds = config["thresholds"]
    gates = {
        "spatial_inertia": spatial_error <= thresholds["maximum_spatial_inertia_reconstruction_error"],
        "continuous_model": maxima["continuous_model_error"] <= thresholds["maximum_continuous_model_error"],
        "equilibrium": equilibrium_derivative <= thresholds["maximum_equilibrium_derivative"],
        "rk4_dop853": maxima["rk4_vs_dop853_error"] <= thresholds["maximum_rk4_vs_dop853_error"],
        "rk4_step_doubling": maxima["rk4_step_doubling_error"] <= thresholds["maximum_rk4_step_doubling_error"],
        "jacobian_reference_stability": maxima["jacobian_step_consistency"] <= thresholds["maximum_jacobian_step_consistency"],
        "rk4_jacobian_reference_stability": maxima["rk4_jacobian_step_consistency"] <= thresholds["maximum_jacobian_step_consistency"],
        "determinism": maxima["repeat_error"] <= thresholds["maximum_repeat_error"],
        "positive_inertia": mass > 0.0 and float(np.min(np.linalg.eigvalsh(inertia_com_b))) > 0.0,
        "finite": all(np.all(np.isfinite(sample["continuous"])) for sample in samples.values()),
    }
    summary = {
        "schema_version": 1,
        "phase": 23,
        "profile": config["profile"],
        "model": "locked composite rigid body about canonical base-control point",
        "input": "two external contact wrenches expressed in base FLU and about base-control point",
        "parameters": {
            "mass_kg": mass,
            "com_from_base_b_m": com_b.tolist(),
            "inertia_com_b_kg_m2": inertia_com_b.tolist(),
            "spatial_inertia_reconstruction_error": spatial_error,
            "source_equilibrium_wrench_flu": source_equilibrium_wrench.tolist(),
            "projected_equilibrium_wrench_flu": equilibrium_wrench.tolist(),
            "maximum_equilibrium_wrench_projection": float(np.max(np.abs(
                equilibrium_wrench - source_equilibrium_wrench
            ))),
        },
        "gates": gates,
        "pass": all(gates.values()),
        "maxima": {**maxima, "equilibrium_derivative": equilibrium_derivative},
        "samples": samples,
    }
    output.mkdir(parents=True, exist_ok=True)
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    golden_path = output / "golden.txt"
    with golden_path.open("w", encoding="utf-8") as stream:
        stream.write(f"{len(samples)}\n")
        for sample_id, sample in samples.items():
            stream.write(sample_id + "\n")
            stream.write(" ".join(
                f"{value:.17g}" for value in sample["reference_rotation_vector_rad"]
            ) + "\n")
            for key in ("state", "wrench", "continuous", "rk4_next"):
                stream.write(" ".join(f"{value:.17g}" for value in sample[key]) + "\n")
            for key in (
                "state_jacobian", "input_jacobian",
                "rk4_state_jacobian", "rk4_input_jacobian"
            ):
                stream.write(" ".join(
                    f"{value:.17g}" for row in sample[key] for value in row
                ) + "\n")
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
        "outputs": {"summary.json": sha256(summary_path), "golden.txt": sha256(golden_path)},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"pass": summary["pass"], "gates": gates, "maxima": summary["maxima"], "parameters": summary["parameters"]}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
