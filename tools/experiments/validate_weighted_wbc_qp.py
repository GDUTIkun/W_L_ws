#!/usr/bin/env python3
"""Layered Phase-21 hard-QP pre-freeze validation."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_qp_prefreeze.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def solve_admm(
    h: np.ndarray, g: np.ndarray, a: np.ndarray, lower: np.ndarray,
    upper: np.ndarray, settings: dict[str, Any], initial_x: np.ndarray | None = None
) -> dict[str, Any]:
    if not all(np.all(np.isfinite(value)) for value in (h, g, a, lower, upper)):
        return {"status": "invalid"}
    if np.any(lower > upper) or np.max(np.abs(h - h.T)) > 1e-12:
        return {"status": "invalid"}
    if np.min(np.linalg.eigvalsh(h)) < -1e-12:
        return {"status": "nonconvex"}
    rho, sigma = float(settings["rho"]), float(settings["sigma"])
    inverse = np.linalg.inv(h + sigma * np.eye(h.shape[0]) + rho * a.T @ a)
    x = np.zeros(h.shape[0]) if initial_x is None else initial_x.copy()
    z = np.clip(a @ x, lower, upper)
    y = np.zeros(a.shape[0])
    started = time.perf_counter_ns()
    for iteration in range(1, int(settings["maximum_iterations"]) + 1):
        x = inverse @ (-g + sigma * x + rho * a.T @ (z - y))
        ax = a @ x
        previous = z.copy()
        z = np.clip(ax + y, lower, upper)
        y += ax - z
        primal = float(np.linalg.norm(ax - z))
        dual = float(np.linalg.norm(rho * a.T @ (z - previous)))
        primal_tolerance = np.sqrt(a.shape[0]) * settings["absolute_tolerance"] + settings["relative_tolerance"] * max(np.linalg.norm(ax), np.linalg.norm(z))
        dual_tolerance = np.sqrt(h.shape[0]) * settings["absolute_tolerance"] + settings["relative_tolerance"] * np.linalg.norm(rho * a.T @ y)
        if not np.all(np.isfinite(x)):
            return {"status": "nonfinite", "iterations": iteration}
        if primal <= primal_tolerance and dual <= dual_tolerance:
            break
    else:
        iteration = int(settings["maximum_iterations"])
    elapsed_ms = (time.perf_counter_ns() - started) / 1e6
    dual_vector = rho * y
    stationarity = float(np.max(np.abs(h @ x + g + a.T @ dual_vector)))
    bound_violation = float(max(0.0, np.max(lower - a @ x), np.max(a @ x - upper)))
    return {
        "status": "converged" if iteration < int(settings["maximum_iterations"]) else "maximum_iterations",
        "iterations": iteration,
        "solve_time_ms": elapsed_ms,
        "primal_residual": primal,
        "dual_residual": dual,
        "stationarity_residual": stationarity,
        "bound_violation": bound_violation,
        "x": x,
    }


def add_rows(rows: list[np.ndarray], lower: list[float], upper: list[float],
             matrix: np.ndarray, low: np.ndarray, high: np.ndarray) -> None:
    rows.extend(matrix)
    lower.extend(low.tolist())
    upper.extend(high.tolist())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cpp-benchmark", type=Path)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config_path = arguments.config.resolve()
    config, config_inputs = load_config(config_path)
    model_config_path = (ROOT / config["model_profile"]).resolve()
    model_config, model_inputs = load_config(model_config_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_config, equilibrium)
    qpos = oracle.sample_qpos(model_config["samples"][0])
    reduction, _ = oracle.reduction(qpos)
    oracle.forward(qpos)
    full_mass = np.zeros((oracle.model.nv, oracle.model.nv))
    mujoco.mj_fullM(oracle.model, full_mass, oracle.data.qM)
    mass = reduction.T @ full_mass @ reduction
    bias = reduction.T @ oracle.data.qfrc_bias.copy()
    contact, points, _ = oracle.contact(qpos, reduction)
    actuator_full = np.zeros((oracle.model.nv, 6))
    for column, actuator in enumerate(oracle.actuators):
        joint = int(oracle.model.actuator_trnid[actuator, 0])
        actuator_full[int(oracle.model.jnt_dofadr[joint]), column] = -1.0
    actuator = reduction.T @ actuator_full

    static_matrix = np.hstack((actuator, contact.T))
    static = np.linalg.lstsq(static_matrix, bias, rcond=None)[0]
    reference_lambda = static[6:]
    oracle.forward(qpos)
    base_origin = oracle.data.site_xpos[oracle.base_control_site].copy()
    wrench_map = np.zeros((12, 6))
    for side in range(2):
        offset = points[side] - base_origin
        skew = np.asarray([[0, -offset[2], offset[1]], [offset[2], 0, -offset[0]], [-offset[1], offset[0], 0]])
        wrench_map[6 * side:6 * side + 3, 3 * side:3 * side + 3] = np.eye(3)
        wrench_map[6 * side + 3:6 * side + 6, 3 * side:3 * side + 3] = skew
    reference_wrench = wrench_map @ reference_lambda

    scales = config["variable_scale"]
    variable_scale = np.asarray(scales["acceleration"] + scales["torque"] + scales["contact_force"] + scales["wrench_slack"], dtype=float)
    transform = np.diag(variable_scale)
    objective = config["objective"]
    h = np.diag(
        [objective["acceleration_regularization"]] * 12 +
        [objective["torque_regularization"]] * 6 +
        [objective["contact_force_regularization"]] * 6 +
        [objective["wrench_slack_penalty"]] * 12
    )
    g = np.zeros(36)

    dynamics = np.zeros((12, 36))
    dynamics[:, :12] = mass
    dynamics[:, 12:18] = -actuator
    dynamics[:, 18:24] = -contact.T
    wrench = np.zeros((12, 36))
    wrench[:, 18:24] = wrench_map
    wrench[:, 24:36] = -np.eye(12)
    equality = np.vstack((
        dynamics / np.asarray(config["row_scale"]["dynamics"])[:, None],
        wrench / np.asarray(config["row_scale"]["wrench"])[:, None],
    )) @ transform
    equality_rhs = np.r_[
        -bias / np.asarray(config["row_scale"]["dynamics"]),
        reference_wrench / np.asarray(config["row_scale"]["wrench"]),
    ]

    layers: dict[str, Any] = {}
    rows = list(equality)
    lower = equality_rhs.tolist()
    upper = equality_rhs.tolist()
    layer_names = ["equality", "torque", "contact", "acceleration"]
    solutions: dict[str, np.ndarray] = {}
    for layer in layer_names:
        if layer == "torque":
            matrix = np.zeros((6, 36)); matrix[:, 12:18] = np.eye(6)
            limits = np.asarray(config["bounds"]["torque_nm"])
            add_rows(rows, lower, upper, (matrix @ transform) / limits[:, None], -np.ones(6), np.ones(6))
        elif layer == "contact":
            normal = np.zeros((2, 36)); normal[0, 20] = 1; normal[1, 23] = 1
            maximum_normal = float(config["bounds"]["maximum_normal_force_n"])
            add_rows(rows, lower, upper, (normal @ transform) / maximum_normal, np.zeros(2), np.ones(2))
            friction_rows = []
            mu = float(config["friction_coefficient"])
            for start in (18, 21):
                for tangent in (start, start + 1):
                    row = np.zeros(36); row[tangent] = 1; row[start + 2] = -mu; friction_rows.append(row)
                    row = np.zeros(36); row[tangent] = -1; row[start + 2] = -mu; friction_rows.append(row)
            add_rows(rows, lower, upper, (np.asarray(friction_rows) @ transform) / maximum_normal, np.full(8, -1e4), np.zeros(8))
        elif layer == "acceleration":
            matrix = np.zeros((12, 36)); matrix[:, :12] = np.eye(12)
            limits = np.asarray(config["bounds"]["acceleration"])
            add_rows(rows, lower, upper, (matrix @ transform) / limits[:, None], -np.ones(12), np.ones(12))
        a = np.asarray(rows)
        result = solve_admm(h, g, a, np.asarray(lower), np.asarray(upper), config["solver"])
        x = result.pop("x", np.zeros(36))
        physical = variable_scale * x
        equality_residual = float(np.max(np.abs(equality @ x - equality_rhs)))
        layers[layer] = {
            **result,
            "constraint_rows": int(a.shape[0]),
            "equality_residual": equality_residual,
            "maximum_abs_torque_nm": float(np.max(np.abs(physical[12:18]))),
            "minimum_torque_margin_nm": float(np.min(
                np.asarray(config["bounds"]["torque_nm"]) - np.abs(physical[12:18])
            )),
            "minimum_normal_force_n": float(min(physical[20], physical[23])),
            "minimum_friction_margin_n": float(min(
                physical[20] - abs(physical[18]), physical[20] - abs(physical[19]),
                physical[23] - abs(physical[21]), physical[23] - abs(physical[22]),
            )),
            "maximum_abs_wrench_slack": float(np.max(np.abs(physical[24:36]))),
        }
        solutions[layer] = x

    kkt = np.block([[h, equality.T], [equality, np.zeros((24, 24))]])
    kkt_rhs = np.r_[-g, equality_rhs]
    equality_oracle = np.linalg.solve(kkt, kkt_rhs)[:36]
    cross_oracle_difference = float(np.max(np.abs(solutions["equality"] - equality_oracle)))
    infeasible_a = np.vstack((equality, np.eye(1, 36, 0), np.eye(1, 36, 0)))
    infeasible_l = np.r_[equality_rhs, 0.0, 1.0]
    infeasible_u = infeasible_l.copy()
    infeasible = solve_admm(h, g, infeasible_a, infeasible_l, infeasible_u, {**config["solver"], "maximum_iterations": 200})
    infeasible.pop("x", None)

    problem_path = output / "problem.txt"
    problem_values = np.r_[h.ravel(), g, a.ravel(), np.asarray(lower), np.asarray(upper)]
    problem_path.write_text(
        f"36 {a.shape[0]} {config['solver']['rho']:.17g} {config['solver']['sigma']:.17g} "
        f"{config['solver']['absolute_tolerance']:.17g} {config['solver']['relative_tolerance']:.17g} "
        f"{int(config['solver']['maximum_iterations'])}\n" +
        " ".join(f"{value:.17g}" for value in problem_values) + "\n",
        encoding="utf-8",
    )
    cpp_benchmark = None
    if arguments.cpp_benchmark is not None:
        cpp_benchmark = json.loads(arguments.cpp_benchmark.resolve().read_text(encoding="utf-8"))

    gates_config = config["gates"]
    gates = {
        "all_layers_converged": all(value["status"] == "converged" for value in layers.values()),
        "equality_residual": max(value["equality_residual"] for value in layers.values()) <= gates_config["maximum_equality_residual"],
        "bound_violation": max(value["bound_violation"] for value in layers.values()) <= gates_config["maximum_bound_violation"],
        "stationarity": max(value["stationarity_residual"] for value in layers.values()) <= gates_config["maximum_stationarity_residual"],
        "cross_oracle": cross_oracle_difference <= gates_config["maximum_cross_oracle_difference"],
        "torque_margin": layers["acceleration"]["minimum_torque_margin_nm"] >= gates_config["minimum_torque_margin_nm"],
        "normal_margin": layers["acceleration"]["minimum_normal_force_n"] >= gates_config["minimum_normal_force_n"],
        "friction_margin": layers["acceleration"]["minimum_friction_margin_n"] >= gates_config["minimum_friction_margin_n"],
        "deadline": cpp_benchmark is not None and cpp_benchmark.get("pass", False) and cpp_benchmark["cold_max_solve_time_ms"] <= gates_config["maximum_reference_host_solve_time_ms"],
        "infeasible_rejected": infeasible["status"] != "converged" or infeasible["bound_violation"] > gates_config["maximum_bound_violation"],
    }
    summary = {
        "schema_version": 1, "phase": 21, "profile": config["profile"],
        "variable_order": ["nudot_12", "tau_6", "lambda_left_right_6", "slack_left_right_12"],
        "reference_tau_nm": static[:6].tolist(),
        "reference_lambda_n": reference_lambda.tolist(),
        "static_model_residual": float(np.max(np.abs(static_matrix @ static - bias))),
        "layers": layers, "cross_oracle_difference": cross_oracle_difference,
        "infeasible_case": infeasible, "gates": gates, "pass": all(gates.values()),
        "python_solve_time_observation_ms": max(value["solve_time_ms"] for value in layers.values()),
        "cpp_benchmark": cpp_benchmark,
    }
    write_json(output / "summary.json", summary)
    write_json(output / "manifest.json", {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "numpy": np.__version__, "mujoco": mujoco.__version__,
        "hardware_data": False, "config": str(config_path.relative_to(ROOT)),
        "config_sha256": sha256(config_path), "model_config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "equilibrium": str(equilibrium_path.relative_to(ROOT)), "equilibrium_sha256": sha256(equilibrium_path),
        "validator": str(Path(__file__).resolve().relative_to(ROOT)), "validator_sha256": sha256(Path(__file__).resolve()),
        "cpp_benchmark_input": None if arguments.cpp_benchmark is None else {
            "path": str(arguments.cpp_benchmark.resolve().relative_to(ROOT)),
            "sha256": sha256(arguments.cpp_benchmark.resolve()),
        },
        "outputs": {"summary.json": sha256(output / "summary.json"), "problem.txt": sha256(problem_path)},
    })
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
