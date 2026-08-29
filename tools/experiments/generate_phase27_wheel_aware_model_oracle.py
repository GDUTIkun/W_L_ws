#!/usr/bin/env python3
"""Generate the independent Phase-27 16-state model oracle."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

import mujoco
import numpy as np
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/experiments"))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402
from validate_nominal_nmpc_state import left_jacobian_inverse  # noqa: E402

DEFAULT_CONFIG = (
    ROOT / "simulation/mujoco/config/phase27_wheel_aware_model_oracle_v1.json"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def skew(vector: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0, -vector[2], vector[1]],
        [vector[2], 0.0, -vector[0]],
        [-vector[1], vector[0], 0.0],
    ])


def aggregate_body(oracle: Oracle) -> tuple[float, np.ndarray, np.ndarray, float]:
    base = oracle.data.site_xpos[oracle.base_control_site]
    rotation = oracle.data.site_xmat[oracle.base_control_site].reshape(3, 3)
    wheel_bodies = set(oracle.wheel_bodies)
    parts = []
    for body in range(1, oracle.model.nbody):
        if body in wheel_bodies:
            continue
        mass = float(oracle.model.body_mass[body])
        com_b = rotation.T @ (oracle.data.xipos[body] - base)
        inertia_rotation_b = (
            rotation.T @ oracle.data.ximat[body].reshape(3, 3)
        )
        inertia_com_b = (
            inertia_rotation_b
            @ np.diag(oracle.model.body_inertia[body])
            @ inertia_rotation_b.T
        )
        parts.append((mass, com_b, inertia_com_b))
    mass = sum(part[0] for part in parts)
    com_b = sum(part_mass * part_com for part_mass, part_com, _ in parts) / mass
    inertia_com_b = sum(
        inertia + part_mass * (
            float((part_com - com_b) @ (part_com - com_b)) * np.eye(3)
            - np.outer(part_com - com_b, part_com - com_b)
        )
        for part_mass, part_com, inertia in parts
    )
    inertia_b = inertia_com_b + mass * (
        float(com_b @ com_b) * np.eye(3) - np.outer(com_b, com_b)
    )
    spatial = np.block([
        [mass * np.eye(3), -mass * skew(com_b)],
        [mass * skew(com_b), inertia_b],
    ])
    recovered_com = np.array([
        (-spatial[:3, 3:6] / mass)[2, 1],
        (-spatial[:3, 3:6] / mass)[0, 2],
        (-spatial[:3, 3:6] / mass)[1, 0],
    ])
    recovered_inertia = inertia_b - mass * (
        float(recovered_com @ recovered_com) * np.eye(3)
        - np.outer(recovered_com, recovered_com)
    )
    error = max(
        abs(float(np.trace(spatial[:3, :3]) / 3.0) - mass),
        float(np.max(np.abs(recovered_com - com_b))),
        float(np.max(np.abs(recovered_inertia - inertia_com_b))),
    )
    return mass, com_b, inertia_com_b, error


def equilibrium_input(
    mass: float, com_b: np.ndarray, wheel_origins_b: np.ndarray
) -> np.ndarray:
    weight = mass * 9.81
    loads = np.linalg.solve(
        np.array([[1.0, 1.0], wheel_origins_b[:, 0]]),
        np.array([weight, com_b[0] * weight]),
    )
    roll_moment = com_b[1] * weight - float(wheel_origins_b[:, 1] @ loads)
    result = np.zeros(12)
    result[2] = loads[0]
    result[8] = loads[1]
    result[3] = result[9] = 0.5 * roll_moment
    return result


def dynamics(
    state: np.ndarray,
    wrench: np.ndarray,
    reference_rotation: np.ndarray,
    parameters: dict[str, np.ndarray | float],
) -> np.ndarray:
    rotation_vector = state[3:6]
    linear_velocity = state[6:9]
    angular_velocity = state[9:12]
    rotation = Rotation.from_rotvec(rotation_vector).as_matrix() @ reference_rotation
    left_force = wrench[:3]
    right_force = wrench[6:9]
    origins = np.asarray(parameters["wheel_origins_b"], dtype=float).copy()
    origins[:, 0] = state[12:14]
    force_b = left_force + right_force
    moment_b = (
        wrench[3:6] + np.cross(origins[0], left_force)
        + wrench[9:12] + np.cross(origins[1], right_force)
    )
    force_n = rotation @ force_b
    moment_b_n = rotation @ moment_b
    com_offset_n = rotation @ parameters["com_b"]
    inertia_com_n = rotation @ parameters["inertia_com_b"] @ rotation.T
    moment_com_n = moment_b_n - np.cross(com_offset_n, force_n)
    angular_acceleration = np.linalg.solve(
        inertia_com_n,
        moment_com_n - np.cross(
            angular_velocity, inertia_com_n @ angular_velocity
        ),
    )
    com_acceleration = force_n / parameters["body_mass"] + [0.0, 0.0, -9.81]
    base_acceleration = (
        com_acceleration
        - np.cross(angular_acceleration, com_offset_n)
        - np.cross(angular_velocity, np.cross(angular_velocity, com_offset_n))
    )
    base_forward = force_b[0] / parameters["body_mass"]
    wheel_acceleration = np.array([
        -base_forward - (
            parameters["wheel_radius"] * wrench[0] + wrench[4]
        ) / parameters["wheel_denominator"],
        -base_forward - (
            parameters["wheel_radius"] * wrench[6] + wrench[10]
        ) / parameters["wheel_denominator"],
    ])
    return np.r_[
        linear_velocity,
        left_jacobian_inverse(rotation_vector) @ angular_velocity,
        base_acceleration,
        angular_acceleration,
        state[14:16],
        wheel_acceleration,
    ]


def rk4(function: Callable[[np.ndarray], np.ndarray], state: np.ndarray, step: float) -> np.ndarray:
    k1 = function(state)
    k2 = function(state + 0.5 * step * k1)
    k3 = function(state + 0.5 * step * k2)
    k4 = function(state + step * k3)
    return state + step * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0


def discrete_rk4(
    function: Callable[[np.ndarray], np.ndarray], state: np.ndarray,
    step: float, substeps: int,
) -> np.ndarray:
    result = state
    for _ in range(substeps):
        result = rk4(function, result, step / substeps)
    return result


def central_jacobian(function: Callable[[np.ndarray], np.ndarray], value: np.ndarray, step: float) -> np.ndarray:
    result = np.zeros((function(value).size, value.size))
    for column in range(value.size):
        delta = np.zeros_like(value)
        delta[column] = step * max(1.0, abs(value[column]))
        result[:, column] = (
            function(value + delta) - function(value - delta)
        ) / (2.0 * delta[column])
    return result


def write_golden(path: Path, samples: dict[str, dict[str, object]]) -> None:
    lines = [f"PHASE27_WHEEL_AWARE_MODEL_GOLDEN_V1 {len(samples)}"]
    for sample_id, sample in samples.items():
        lines.append(sample_id)
        for key in (
            "reference_rotation_vector_rad", "state", "input", "continuous",
            "rk4_next", "state_jacobian", "input_jacobian",
            "rk4_state_jacobian", "rk4_input_jacobian",
        ):
            lines.append(" ".join(
                f"{value:.17g}" for value in np.ravel(sample[key])
            ))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    config_path = args.config.resolve()
    config, config_inputs = load_config(config_path)
    model_config_path = (ROOT / config["source_model_oracle"]).resolve()
    model_config, model_inputs = load_config(model_config_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_config, equilibrium)
    qpos = oracle.sample_qpos({
        "base_rotation_vector_rad": [0.0] * 3,
        "canonical_joint_delta_rad": [0.0] * 6,
    })
    oracle.forward(qpos, np.zeros(oracle.model.nv))
    body_mass, com_b, inertia_com_b, parameter_error = aggregate_body(oracle)
    base_position = oracle.data.site_xpos[oracle.base_control_site].copy()
    base_rotation = oracle.data.site_xmat[oracle.base_control_site].reshape(3, 3)
    wheel_origins_b = np.array([
        base_rotation.T @ (oracle.data.xpos[body] - base_position)
        for body in oracle.wheel_bodies
    ])
    wheel_masses = np.asarray(oracle.model.body_mass)[oracle.wheel_bodies]
    wheel_axes_n = np.array([
        oracle.data.xmat[body].reshape(3, 3)
        @ np.asarray(oracle.model.jnt_axis)[oracle.model.body_jntadr[body]]
        for body in oracle.wheel_bodies
    ])
    wheel_inertias = []
    for body, axis_n in zip(oracle.wheel_bodies, wheel_axes_n):
        inertia_rotation = oracle.data.ximat[body].reshape(3, 3)
        inertia_n = (
            inertia_rotation @ np.diag(oracle.model.body_inertia[body])
            @ inertia_rotation.T
        )
        wheel_inertias.append(float(axis_n @ inertia_n @ axis_n))
    wheel_inertias = np.asarray(wheel_inertias)
    wheel_mass = float(np.mean(wheel_masses))
    wheel_inertia = float(np.mean(wheel_inertias))
    wheel_radius = float(config["wheel_radius_m"])
    parameters = {
        "body_mass": body_mass,
        "com_b": com_b,
        "inertia_com_b": inertia_com_b,
        "wheel_origins_b": wheel_origins_b,
        "wheel_mass": wheel_mass,
        "wheel_inertia": wheel_inertia,
        "wheel_radius": wheel_radius,
        "wheel_denominator": wheel_mass * wheel_radius + wheel_inertia / wheel_radius,
    }
    equilibrium_wrench = equilibrium_input(body_mass, com_b, wheel_origins_b)
    dt = float(config["sample_period_s"])
    rk4_substeps = int(config["rk4_substeps"])
    if rk4_substeps < 1:
        raise ValueError("rk4_substeps must be positive")
    fd_step = float(config["jacobian_difference_step"])
    samples = {}
    maxima = {
        "rk4_vs_dop853_error": 0.0,
        "rk4_step_doubling_error": 0.0,
        "jacobian_step_consistency": 0.0,
        "rk4_jacobian_step_consistency": 0.0,
        "mode_identity_error": 0.0,
        "repeat_error": 0.0,
    }
    for spec in config["samples"]:
        reference_vector = np.asarray(spec["reference_rotation_vector_rad"], dtype=float)
        reference_rotation = Rotation.from_rotvec(reference_vector).as_matrix()
        wheel_delta = np.asarray(spec["wheel_state_delta"], dtype=float)
        state = np.r_[
            base_position,
            spec["rotation_vector_rad"],
            spec["linear_velocity_n_m_s"],
            spec["angular_velocity_n_rad_s"],
            wheel_origins_b[:, 0] + wheel_delta[:2],
            wheel_delta[2:],
        ].astype(float)
        wrench = equilibrium_wrench + np.asarray(spec["wrench_delta_flu"], dtype=float)
        flow = lambda candidate: dynamics(candidate, wrench, reference_rotation, parameters)
        next_rk4 = discrete_rk4(flow, state, dt, rk4_substeps)
        dop853 = solve_ivp(
            lambda _time, value: flow(value), [0.0, dt], state,
            method="DOP853", rtol=1e-12, atol=1e-14,
        ).y[:, -1]
        half = discrete_rk4(flow, state, dt, 2 * rk4_substeps)
        combined = np.r_[state, wrench]
        combined_flow = lambda value: dynamics(
            value[:16], value[16:], reference_rotation, parameters
        )
        jacobian = central_jacobian(combined_flow, combined, fd_step)
        jacobian_fine = central_jacobian(combined_flow, combined, 0.5 * fd_step)
        combined_rk4 = lambda value: discrete_rk4(
            lambda candidate: dynamics(
                candidate, value[16:], reference_rotation, parameters
            ), value[:16], dt, rk4_substeps,
        )
        rk4_jacobian = central_jacobian(combined_rk4, combined, fd_step)
        rk4_jacobian_fine = central_jacobian(
            combined_rk4, combined, 0.5 * fd_step
        )
        continuous = flow(state)
        force_sum_x = wrench[0] + wrench[6]
        expected_left = -force_sum_x / body_mass - (
            wheel_radius * wrench[0] + wrench[4]
        ) / parameters["wheel_denominator"]
        expected_right = -force_sum_x / body_mass - (
            wheel_radius * wrench[6] + wrench[10]
        ) / parameters["wheel_denominator"]
        mode_error = max(
            abs(continuous[14] - expected_left),
            abs(continuous[15] - expected_right),
            abs(0.5 * (continuous[15] - continuous[14]) - (
                wheel_radius * (wrench[0] - wrench[6])
                + wrench[4] - wrench[10]
            ) / (2.0 * parameters["wheel_denominator"])),
        )
        errors = {
            "rk4_vs_dop853_error": float(np.max(np.abs(next_rk4 - dop853))),
            "rk4_step_doubling_error": float(np.max(np.abs(next_rk4 - half))),
            "jacobian_step_consistency": float(np.max(np.abs(jacobian - jacobian_fine))),
            "rk4_jacobian_step_consistency": float(np.max(np.abs(
                rk4_jacobian - rk4_jacobian_fine
            ))),
            "mode_identity_error": float(mode_error),
            "repeat_error": float(np.max(np.abs(flow(state) - continuous))),
        }
        for key, value in errors.items():
            maxima[key] = max(maxima[key], value)
        samples[spec["id"]] = {
            "reference_rotation_vector_rad": reference_vector.tolist(),
            "state": state.tolist(),
            "input": wrench.tolist(),
            "continuous": continuous.tolist(),
            "rk4_next": next_rk4.tolist(),
            "state_jacobian": jacobian[:, :16].tolist(),
            "input_jacobian": jacobian[:, 16:].tolist(),
            "rk4_state_jacobian": rk4_jacobian[:, :16].tolist(),
            "rk4_input_jacobian": rk4_jacobian[:, 16:].tolist(),
            **errors,
        }
    equilibrium_derivative = float(np.max(np.abs(samples["equilibrium"]["continuous"])))
    thresholds = config["thresholds"]
    wheel_asymmetry = max(
        float(np.ptp(wheel_masses)), float(np.ptp(wheel_inertias))
    )
    workspace = config["wheel_workspace_m"]
    model_validity = all(
        np.linalg.norm(sample["state"][3:6]) <= config["rotation_chart_maximum_norm_rad"]
        and workspace["left"][0] <= sample["state"][12] <= workspace["left"][1]
        and workspace["right"][0] <= sample["state"][13] <= workspace["right"][1]
        for sample in samples.values()
    )
    gates = {
        "parameter_reconstruction": parameter_error <= thresholds["maximum_parameter_reconstruction_error"],
        "positive_parameters": body_mass > 0.0 and wheel_mass > 0.0
        and parameters["wheel_denominator"] > 0.0
        and float(np.min(np.linalg.eigvalsh(inertia_com_b))) > 0.0,
        "wheel_side_parameter_symmetry": wheel_asymmetry <= thresholds["maximum_wheel_side_parameter_asymmetry"],
        "equilibrium": equilibrium_derivative <= thresholds["maximum_equilibrium_derivative"],
        "one_step": maxima["rk4_vs_dop853_error"] <= thresholds["maximum_rk4_vs_dop853_error"],
        "step_doubling": maxima["rk4_step_doubling_error"] <= thresholds["maximum_rk4_step_doubling_error"],
        "fd_sensitivity": max(
            maxima["jacobian_step_consistency"],
            maxima["rk4_jacobian_step_consistency"],
        ) <= thresholds["maximum_jacobian_step_consistency"],
        "common_differential_left_right": maxima["mode_identity_error"] <= thresholds["maximum_mode_identity_error"],
        "model_validity": model_validity,
        "determinism": maxima["repeat_error"] <= thresholds["maximum_repeat_error"],
    }
    output.mkdir(parents=True, exist_ok=True)
    golden_path = output / "golden.txt"
    write_golden(golden_path, samples)
    summary = {
        "schema_version": 1,
        "phase": 27,
        "profile": config["profile"],
        "model": "current-nominal upper-body composite excluding wheel bodies plus paper Eq.(12)",
        "state_order": ["p_B_N_3", "relative_rotation_vector_3", "v_B_N_3", "omega_B_N_3", "xi_L", "xi_R", "dxi_L", "dxi_R"],
        "input": "left/right wheel-on-body interaction wrench at wheel-body origin in body FLU",
        "parameters": {
            "body_mass_kg": body_mass,
            "body_com_from_base_b_m": com_b.tolist(),
            "body_inertia_com_b_kg_m2": inertia_com_b.tolist(),
            "wheel_mass_kg": wheel_mass,
            "wheel_axle_inertia_kg_m2": wheel_inertia,
            "wheel_radius_m": wheel_radius,
            "wheel_denominator_kg_m": parameters["wheel_denominator"],
            "wheel_origins_from_base_b_m": wheel_origins_b.tolist(),
            "equilibrium_input": equilibrium_wrench.tolist(),
            "parameter_reconstruction_error": parameter_error,
            "wheel_side_parameter_asymmetry": wheel_asymmetry,
        },
        "maxima": {**maxima, "equilibrium_derivative": equilibrium_derivative},
        "gates": gates,
        "pass": all(gates.values()),
        "samples": samples,
    }
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
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "model_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "equilibrium": str(equilibrium_path.relative_to(ROOT)),
        "equilibrium_sha256": sha256(equilibrium_path),
        "generator": str(Path(__file__).resolve().relative_to(ROOT)),
        "generator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {"golden.txt": sha256(golden_path), "summary.json": sha256(summary_path)},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"pass": summary["pass"], "gates": gates, "maxima": summary["maxima"], "parameters": summary["parameters"]}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)
