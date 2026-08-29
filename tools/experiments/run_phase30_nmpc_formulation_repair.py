#!/usr/bin/env python3
"""Run the frozen Phase 30 direct-weight causal screens and fixed grids."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import casadi
import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase30_nmpc_formulation_repair_v1.json"
PHASE29_SCRIPT = ROOT / "tools/experiments/run_phase29_nmpc_root_cause.py"


def load_phase29() -> Any:
    spec = importlib.util.spec_from_file_location("phase29_root_cause", PHASE29_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase 29 oracle")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


P29 = load_phase29()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


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


def configure_weights(solver: Any, alpha: float, beta: float, attitude: list[int]) -> None:
    baseline_q = solver.q.copy()
    baseline_qe = solver.qe.copy()
    solver.q = baseline_q
    solver.q[attitude] *= beta
    solver.qe = baseline_qe
    solver.qe[0] *= alpha


def solve_converged(solver: Any, problem: Any, gates: dict, *, details: bool = False) -> dict:
    solver.reset()
    result = P29.converged_solve(solver, problem, gates, details=details)
    result["kkt_pass"] = bool(
        result["status"] == 0
        and result["residuals"][0] <= float(gates["converged_stationarity_max"])
        and max(result["residuals"][1:]) <= float(gates["converged_feasibility_max"])
        and result["cost_error"] <= float(gates["cost_recompute_max_abs_error"])
    )
    return result


def replay_to(solver: Any, sequence: list[Any], tick: int) -> tuple[dict, float]:
    maximum_error = 0.0
    selected = None
    for index, problem in enumerate(sequence):
        if problem.tick > tick:
            break
        selected = solver.solve(problem, cold=index == 0, details=problem.tick == tick)
        maximum_error = max(maximum_error, float(np.max(np.abs(selected["u0"] - problem.logged_wrench))))
    if selected is None:
        raise RuntimeError(f"no update at or before tick {tick}")
    return selected, maximum_error


def copy_with_delta(problem: Any, index: int, delta: float) -> Any:
    state = problem.state.copy()
    state[index] += delta
    return P29.Problem(
        state, problem.reference, problem.center, problem.rotation, problem.tick, problem.logged_wrench
    )


def branch_signature(solution: dict, threshold: float) -> tuple[bool, bool]:
    stage = solution["stage"]
    return (
        float(stage["minimum_input_bound_distance"]) <= threshold,
        float(stage["minimum_state_bound_distance"]) <= threshold,
    )


def metric(
    solver: Any,
    problem: Any,
    state_index: int,
    acceleration_index: int,
    delta: float,
    gates: dict,
) -> dict:
    base = solve_converged(solver, problem, gates, details=True)
    minus = solve_converged(solver, copy_with_delta(problem, state_index, -delta), gates, details=True)
    plus = solve_converged(solver, copy_with_delta(problem, state_index, delta), gates, details=True)
    derivative = float(
        (plus["acceleration"][acceleration_index] - minus["acceleration"][acceleration_index])
        / (2.0 * delta)
    )
    error = float(problem.state[state_index] - problem.reference[state_index])
    acceleration = float(base["acceleration"][acceleration_index])
    signatures = [branch_signature(item, float(gates["active_bound_distance"])) for item in (minus, base, plus)]
    branch_pass = len(set(signatures)) == 1 and all(item["kkt_pass"] for item in (minus, base, plus))
    return {
        "state_index": state_index,
        "acceleration_index": acceleration_index,
        "error": error,
        "acceleration": acceleration,
        "corrective_product": -error * acceleration,
        "acceleration_derivative": derivative,
        "corrective_derivative": -derivative,
        "delta": delta,
        "branch_signatures": signatures,
        "branch_pass": branch_pass,
        "base_residuals": base["residuals"],
        "minus_residuals": minus["residuals"],
        "plus_residuals": plus["residuals"],
        "u0": base["u0"],
    }


def update_neighborhood(sequence: list[Any], action_tick: int) -> list[Any]:
    position = next(index for index, problem in enumerate(sequence) if problem.tick == action_tick)
    if position == 0 or position + 1 >= len(sequence):
        raise RuntimeError("authority action lacks a two-sided update neighborhood")
    return sequence[position - 1 : position + 2]


def evaluate_profile(
    branch: str,
    alpha: float,
    beta: float,
    problems: list[Any],
    sqp_dir: Path,
    config: dict,
    method: dict,
) -> dict:
    gates = method["gates"]
    attitude = [int(value) for value in method["running_attitude_indices"]]
    solver = P29.OfflineSolver(sqp_dir, config)
    configure_weights(solver, alpha, beta, attitude)
    entries: dict[str, dict] = {}
    if branch == "T0":
        targets = [(4, 10, float(method["perturbation"]["rotation_rad"])),
                   (10, 10, float(method["perturbation"]["angular_velocity_rad_s"]))]
        guards = [(0, 6), (6, 6)]
    else:
        targets = [(0, 6, float(method["perturbation"]["position_m"])),
                   (6, 6, float(method["perturbation"]["linear_velocity_m_s"]))]
        targets += [
            (index, index + 6 if index < 6 else index, float(method["perturbation"]["rotation_rad"] if index < 6 else method["perturbation"]["angular_velocity_rad_s"]))
            for index in attitude
        ]
        guards = []
    for problem in problems:
        label = f"tick_{problem.tick}"
        entries[label] = {
            str(index): metric(solver, problem, index, acceleration, delta, gates)
            for index, acceleration, delta in targets
        }
        if guards:
            base = solve_converged(solver, problem, gates)
            entries[label]["longitudinal_guards"] = {
                str(index): {
                    "error": float(problem.state[index] - problem.reference[index]),
                    "acceleration": float(base["acceleration"][acceleration]),
                    "corrective_product": -float(problem.state[index] - problem.reference[index])
                    * float(base["acceleration"][acceleration]),
                }
                for index, acceleration in guards
            }
    return {
        "alpha": alpha,
        "beta": beta,
        "running_state_weight_scale": [beta if index in attitude else 1.0 for index in range(16)],
        "terminal_state_weight_scale": [alpha if index == 0 else 1.0 for index in range(16)],
        "samples": entries,
    }


def required_metrics(profile: dict) -> list[dict]:
    return [
        value
        for sample in profile["samples"].values()
        for key, value in sample.items()
        if key != "longitudinal_guards"
    ]


def assess(profile: dict, zero: dict, method: dict, branch: str) -> dict:
    gates = method["gates"]
    product_floor = float(gates["corrective_product_abs_floor"])
    derivative_floor = float(gates["corrective_derivative_abs_floor_s2"])
    relative = float(gates["relative_zero_scale_margin"])
    actual = required_metrics(profile)
    reference = required_metrics(zero)
    checks = []
    for item, zero_item in zip(actual, reference, strict=True):
        product_threshold = max(product_floor, relative * float(zero_item["corrective_product"]))
        derivative_threshold = max(derivative_floor, relative * float(zero_item["corrective_derivative"]))
        checks.append({
            "state_index": item["state_index"],
            "product": item["corrective_product"],
            "product_threshold": product_threshold,
            "derivative": item["corrective_derivative"],
            "derivative_threshold": derivative_threshold,
            "pass": bool(item["branch_pass"] and item["corrective_product"] > product_threshold and item["corrective_derivative"] > derivative_threshold),
        })
    guard_pass = True
    if branch == "T0":
        guard_pass = all(
            guard["corrective_product"] >= -product_floor
            for sample in profile["samples"].values()
            for guard in sample["longitudinal_guards"].values()
        )
    return {"checks": checks, "guard_pass": guard_pass, "pass": bool(guard_pass and all(item["pass"] for item in checks))}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    parser.add_argument("--supersedes")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")

    method = json.loads(args.method.read_text(encoding="utf-8"))
    phase29_method_path = ROOT / method["source_phase29_method"]
    phase29_method = json.loads(phase29_method_path.read_text(encoding="utf-8"))
    config_path = ROOT / method["source_ocp_config"]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    source_run = ROOT / method["source_phase28_run"]
    authority = ROOT / method["source_phase29_authority"]
    rti_dir = ROOT / method["source_generated_dir"]
    sqp_dir = ROOT / method["offline_sqp_generated_dir"]
    for directory in (rti_dir, sqp_dir):
        if not (directory / "acados_ocp.json").is_file() or not list(directory.glob("libacados_ocp_solver_*.so")):
            raise RuntimeError(f"generated oracle unavailable: {directory}")

    cases: dict[str, dict] = {}
    all_one: dict[str, dict] = {}
    neighborhoods: dict[str, list[Any]] = {}
    for case in phase29_method["cases"]:
        rows = P29.csv_rows(source_run / f"{case['id']}_control.csv")
        sequence = P29.problem_sequence(rows, float(config["equilibrium_state"][2]))
        action = max((problem for problem in sequence if problem.tick <= int(case["snapshot_tick"])), key=lambda item: item.tick)
        neighborhoods[case["id"]] = update_neighborhood(sequence, action.tick)
        expected = json.loads((authority / f"{case['id']}_root_cause.json").read_text(encoding="utf-8"))
        rti = P29.OfflineSolver(rti_dir, config)
        configure_weights(rti, 1.0, 1.0, method["running_attitude_indices"])
        production, prefix_error = replay_to(rti, sequence, action.tick)
        sqp = P29.OfflineSolver(sqp_dir, config)
        configure_weights(sqp, 1.0, 1.0, method["running_attitude_indices"])
        converged = solve_converged(sqp, action, method["gates"])
        production_u_error = float(np.max(np.abs(production["u0"] - expected["solutions"]["production"]["u0"])))
        converged_u_error = float(np.max(np.abs(converged["u0"] - expected["solutions"]["converged"]["u0"])))
        objective_error = abs(float(converged["recomputed_cost"]) - float(expected["solutions"]["converged"]["objective"]))
        parity_pass = bool(
            production_u_error <= float(method["gates"]["baseline_u0_max_abs_error"])
            and converged_u_error <= float(method["gates"]["baseline_u0_max_abs_error"])
            and objective_error <= float(method["gates"]["baseline_objective_max_abs_error"])
            and converged["kkt_pass"]
        )
        all_one[case["id"]] = {
            "production_u0_max_abs_error": production_u_error,
            "converged_u0_max_abs_error": converged_u_error,
            "converged_objective_abs_error": objective_error,
            "source_prefix_request_max_abs_error": prefix_error,
            "pass": parity_pass,
        }
        cases[case["id"]] = case

    if not all(item["pass"] for item in all_one.values()):
        raise RuntimeError("all-one baseline does not reproduce Phase 29 authority")

    branch_specs = {
        "T0": ("T0_static", 0.0, 1.0),
        "T1": ("T1_straight_start_cruise_brake", 1.0, 0.0),
    }
    branches: dict[str, dict] = {}
    for branch, (case_id, zero_alpha, zero_beta) in branch_specs.items():
        zero = evaluate_profile(branch, zero_alpha, zero_beta, neighborhoods[case_id], sqp_dir, config, method)
        zero_assessment = assess(zero, zero, method, branch)
        zero_pass = bool(zero_assessment["pass"])
        grid_results = []
        selected = None
        for scale in method["grid"]:
            alpha, beta = (float(scale), 1.0) if branch == "T0" else (1.0, float(scale))
            profile = evaluate_profile(branch, alpha, beta, neighborhoods[case_id], sqp_dir, config, method)
            assessment = assess(profile, zero, method, branch)
            eligible = bool(assessment["pass"] and (branch != "T1" or float(scale) > 0.0))
            grid_results.append({"scale": scale, "profile": profile, "assessment": assessment, "eligible": eligible})
            if zero_pass and eligible:
                selected = float(scale)
        production_check = None
        if selected is not None:
            action = neighborhoods[case_id][1]
            rows = P29.csv_rows(source_run / f"{case_id}_control.csv")
            sequence = P29.problem_sequence(rows, float(config["equilibrium_state"][2]))
            rti = P29.OfflineSolver(rti_dir, config)
            configure_weights(rti, selected if branch == "T0" else 1.0, selected if branch == "T1" else 1.0, method["running_attitude_indices"])
            solution, _ = replay_to(rti, sequence, action.tick)
            error_index = int(cases[case_id]["error_indices"][-1])
            acceleration_index = int(cases[case_id]["acceleration_index"])
            score = float((action.state[error_index] - action.reference[error_index]) * solution["acceleration"][acceleration_index])
            production_check = {"reinforcing_score": score, "pass": score < -float(method["gates"]["production_direction_deadband"]), "u0": solution["u0"]}
            if not production_check["pass"]:
                selected = None
        branches[branch] = {
            "zero_profile": zero,
            "zero_assessment": zero_assessment,
            "zero_screen_pass": zero_pass,
            "grid": grid_results,
            "selected_scale": selected,
            "production_lifecycle": production_check,
            "failure": None if selected is not None else ("R30-A_terminal_scalar_insufficient" if branch == "T0" else "R30-B_attitude_scalar_insufficient"),
        }

    output.mkdir(parents=True)
    result = {
        "schema_version": 1,
        "phase": 30,
        "all_one_parity": all_one,
        "branches": branches,
        "pass": all(value["selected_scale"] is not None for value in branches.values()),
    }
    (output / "direct_weight_sweep.json").write_text(json.dumps(clean(result), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    summary = {
        "pass": result["pass"],
        "all_one_parity_pass": all(item["pass"] for item in all_one.values()),
        "branches": {name: {"zero_screen_pass": value["zero_screen_pass"], "selected_scale": value["selected_scale"], "failure": value["failure"]} for name, value in branches.items()},
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "supersedes": args.supersedes,
        "python": platform.python_version(),
        "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__, "casadi": casadi.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in (args.method.resolve(), phase29_method_path, config_path, PHASE29_SCRIPT)},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
