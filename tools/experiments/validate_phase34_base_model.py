#!/usr/bin/env python3
"""Validate the Phase 34 12-state base model against an independent DOP853 oracle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import casadi as ca
import numpy as np
from scipy.integrate import solve_ivp

from generate_phase34_base_acados_solver import discrete_expression

ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean(item) for item in value]
    return value


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_matrix(vector: np.ndarray) -> np.ndarray:
    squared = float(vector @ vector)
    hat = skew(vector)
    if squared < 1.0e-12:
        sine = 1.0 - squared / 6.0 + squared * squared / 120.0
        cosine = 0.5 - squared / 24.0 + squared * squared / 720.0
    else:
        angle = np.sqrt(squared)
        sine = np.sin(angle) / angle
        cosine = (1.0 - np.cos(angle)) / squared
    return np.eye(3) + sine * hat + cosine * hat @ hat


def left_jacobian_inverse(vector: np.ndarray) -> np.ndarray:
    squared = float(vector @ vector)
    hat = skew(vector)
    if squared < 1.0e-12:
        coefficient = 1.0 / 12.0 + squared / 720.0
    else:
        angle = np.sqrt(squared)
        coefficient = 1.0 / squared - (1.0 + np.cos(angle)) / (
            2.0 * angle * np.sin(angle)
        )
    return np.eye(3) - 0.5 * hat + coefficient * hat @ hat


def flow(state: np.ndarray, control: np.ndarray, parameter: np.ndarray, config: dict) -> np.ndarray:
    reference_rotation = parameter[:9].reshape(3, 3)
    rotation = rotation_matrix(state[3:6]) @ reference_rotation
    left_origin = np.array([parameter[9], *config["left_wheel_origin_yz_b_m"]])
    right_origin = np.array([parameter[10], *config["right_wheel_origin_yz_b_m"]])
    force_b = control[:3] + control[6:9]
    moment_b = (
        control[3:6]
        + np.cross(left_origin, control[:3])
        + control[9:12]
        + np.cross(right_origin, control[6:9])
    )
    force_n = rotation @ force_b
    com_n = rotation @ np.asarray(config["body_com_from_base_b_m"])
    inertia_b = np.asarray(config["body_inertia_com_b_kg_m2"])
    inertia_n = rotation @ inertia_b @ rotation.T
    omega = state[9:12]
    angular = np.linalg.solve(
        inertia_n,
        rotation @ moment_b - np.cross(com_n, force_n) - np.cross(omega, inertia_n @ omega),
    )
    com_acceleration = force_n / float(config["body_mass_kg"])
    com_acceleration[2] -= float(config["gravity_m_s2"])
    linear = com_acceleration - np.cross(angular, com_n) - np.cross(
        omega, np.cross(omega, com_n)
    )
    return np.r_[state[6:9], left_jacobian_inverse(state[3:6]) @ omega, linear, angular]


def integrate(state: np.ndarray, control: np.ndarray, parameter: np.ndarray, config: dict) -> np.ndarray:
    result = solve_ivp(
        lambda _time, value: flow(value, control, parameter, config),
        (0.0, float(config["sampling_period_s"])),
        state,
        method="DOP853",
        rtol=1.0e-12,
        atol=1.0e-14,
    )
    if not result.success:
        raise RuntimeError(result.message)
    return result.y[:, -1]


def finite_jacobian(function, value: np.ndarray, step: float) -> np.ndarray:
    columns = []
    for index in range(value.size):
        minus = value.copy(); minus[index] -= step
        plus = value.copy(); plus[index] += step
        columns.append((function(plus) - function(minus)) / (2.0 * step))
    return np.column_stack(columns)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--method", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    method = json.loads(args.method.read_text(encoding="utf-8"))
    settings, gates = method["model_samples"], method["gates"]

    x_symbol, u_symbol, p_symbol, discrete = discrete_expression(config)
    function = ca.Function(
        "phase34_discrete",
        [x_symbol, u_symbol, p_symbol],
        [
            discrete,
            ca.jacobian(discrete, x_symbol),
            ca.jacobian(discrete, u_symbol),
            ca.jacobian(discrete, p_symbol[9:11]),
        ],
    )
    rng = np.random.default_rng(int(settings["seed"]))
    equilibrium_state = np.asarray(config["equilibrium_state"], dtype=float)
    equilibrium_input = np.asarray(config["equilibrium_input"], dtype=float)
    envelope = np.asarray(config["state_envelope_half_width"], dtype=float)
    lower = np.asarray(config["input_lower"], dtype=float)
    upper = np.asarray(config["input_upper"], dtype=float)
    equilibrium_xi = np.asarray(config["equilibrium_wheel_position_m"], dtype=float)
    samples = []
    for index in range(int(settings["random_count"])):
        state = equilibrium_state + float(settings["state_fraction_of_envelope"]) * envelope * rng.uniform(-1, 1, 12)
        control_span = np.minimum(equilibrium_input - lower, upper - equilibrium_input)
        control = equilibrium_input + float(settings["input_fraction_of_bounds"]) * control_span * rng.uniform(-1, 1, 12)
        xi_offset = float(settings["xi_offsets_m"][index % len(settings["xi_offsets_m"])])
        xi = equilibrium_xi + np.array([xi_offset, -xi_offset])
        parameter = np.r_[np.eye(3).reshape(-1), xi]
        candidate, jac_x, jac_u, jac_xi = [np.asarray(item, dtype=float) for item in function(state, control, parameter)]
        candidate = candidate.reshape(-1)
        reference = integrate(state, control, parameter, config)
        fd_x = finite_jacobian(lambda value: integrate(value, control, parameter, config), state, 1.0e-6)
        fd_u = finite_jacobian(lambda value: integrate(state, value, parameter, config), control, 1.0e-5)
        fd_xi = finite_jacobian(
            lambda value: integrate(state, control, np.r_[parameter[:9], value], config),
            xi,
            1.0e-6,
        )
        samples.append(
            {
                "index": index,
                "next_error": float(np.max(np.abs(candidate - reference))),
                "state_jacobian_error": float(np.max(np.abs(jac_x - fd_x))),
                "input_jacobian_error": float(np.max(np.abs(jac_u - fd_u))),
                "xi_jacobian_error": float(np.max(np.abs(jac_xi - fd_xi))),
                "xi_sensitivity_max": float(np.max(np.abs(jac_xi))),
            }
        )

    phase27 = json.loads((ROOT / method["phase27_ocp_config"]).read_text(encoding="utf-8"))
    projection = {
        "state_order": config["state_order"] == phase27["state_order"][:12],
        "equilibrium_state": config["equilibrium_state"] == phase27["equilibrium_state"][:12],
        "state_error_scale": config["state_error_scale"] == phase27["state_error_scale"][:12],
        "state_weight": config["state_weight"] == phase27["state_weight"][:12],
        "state_envelope": config["state_envelope_half_width"] == phase27["state_envelope_half_width"][:12],
        "input_contract": all(config[key] == phase27[key] for key in ("input_order", "equilibrium_input", "input_error_scale", "input_weight", "input_lower", "input_upper")),
        "base_parameters": all(config[key] == phase27[key] for key in ("body_mass_kg", "body_com_from_base_b_m", "body_inertia_com_b_kg_m2", "left_wheel_origin_yz_b_m", "right_wheel_origin_yz_b_m")),
    }
    maxima = {
        key: max(sample[key] for sample in samples)
        for key in ("next_error", "state_jacobian_error", "input_jacobian_error", "xi_jacobian_error")
    }
    pass_map = {
        "next": maxima["next_error"] <= float(gates["model_next_max_abs_error"]),
        "state_jacobian": maxima["state_jacobian_error"] <= float(gates["model_state_jacobian_max_abs_error"]),
        "input_jacobian": maxima["input_jacobian_error"] <= float(gates["model_input_jacobian_max_abs_error"]),
        "xi_jacobian": maxima["xi_jacobian_error"] <= float(gates["model_parameter_jacobian_max_abs_error"]),
        "xi_is_material": min(sample["xi_sensitivity_max"] for sample in samples) > 1.0e-8,
        "projection": all(projection.values()),
        "dimension": int(discrete.numel()) == 12 and int(p_symbol.numel()) == 11,
    }
    summary = {"pass": all(pass_map.values()), "gates": pass_map, "maxima": maxima, "projection": projection}
    output.mkdir(parents=True)
    (output / "details.json").write_text(json.dumps(clean(samples), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "profile": method["profile"] + "_base_model",
        "command": " ".join(__import__("sys").argv),
        "config": str(args.config.resolve().relative_to(ROOT)),
        "config_sha256": sha256(args.config),
        "method": str(args.method.resolve().relative_to(ROOT)),
        "method_sha256": sha256(args.method),
        "runner": str(Path(__file__).resolve().relative_to(ROOT)),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "replay_of": args.replay_of,
        "outputs": {name: sha256(output / name) for name in ("details.json", "summary.json")},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
