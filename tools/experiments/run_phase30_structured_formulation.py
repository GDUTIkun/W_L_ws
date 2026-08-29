#!/usr/bin/env python3
"""Evaluate the frozen Phase 30 v2 structured NMPC cost candidates."""

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
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase30_structured_formulation_v2.json"
PHASE30_V1_SCRIPT = ROOT / "tools/experiments/run_phase30_nmpc_formulation_repair.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P30 = load_module(PHASE30_V1_SCRIPT, "phase30_v1")
P29 = P30.P29


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


class StructuredSolver(P29.OfflineSolver):
    def __init__(self, generated: Path, config: dict, q_run: np.ndarray, q_terminal: np.ndarray):
        super().__init__(generated, config)
        self.q_run = np.asarray(q_run, dtype=float)
        self.q_terminal = np.asarray(q_terminal, dtype=float)
        self.q = np.diag(self.q_run).copy()
        self.qe = np.diag(self.q_terminal).copy()

    def configure(self, problem: Any, **options: Any) -> list[np.ndarray]:
        references = super().configure(problem, **options)
        stage_w = np.zeros((28, 28))
        stage_w[:16, :16] = self.q_run
        stage_w[16:, 16:] = np.diag(self.r)
        for stage in range(self.n):
            self.solver.cost_set(stage, "W", stage_w)
        self.solver.cost_set(self.n, "W", self.q_terminal)
        return references

    def solve(self, problem: Any, *, cold: bool, details: bool = False, **options: Any) -> dict:
        result = super().solve(problem, cold=cold, details=True, **options)
        stage = result["stage"]
        objective = 0.0
        for index in range(self.n):
            state_error = np.asarray(stage["x"][index]) - np.asarray(stage["reference"][index])
            input_error = np.asarray(stage["u"][index]) - self.eq
            objective += 0.5 * self.dt * (
                float(state_error @ self.q_run @ state_error)
                + float(np.sum(self.r * input_error**2))
            )
        terminal_error = np.asarray(stage["x"][-1]) - np.asarray(stage["reference"][-1])
        objective += 0.5 * float(terminal_error @ self.q_terminal @ terminal_error)
        result["recomputed_cost"] = objective
        result["cost_error"] = abs(float(result["solver_cost"]) - objective)
        if not details:
            result.pop("stage")
        return result


def baseline_cost(config: dict) -> tuple[np.ndarray, np.ndarray]:
    scale = np.asarray(config["state_error_scale"], dtype=float)
    q = np.asarray(config["state_weight"], dtype=float) / scale**2
    return np.diag(q), np.diag(float(config["terminal_weight_multiplier"]) * q)


def candidate_cost(config: dict, candidate: dict) -> tuple[np.ndarray, np.ndarray]:
    q_run, q_terminal = baseline_cost(config)
    kind = candidate["kind"]
    q = np.diag(q_run).copy()
    if kind == "terminal_diagonal":
        q_terminal[0, 0] = float(candidate["terminal_x_multiplier"]) * q[0]
        q_terminal[6, 6] = float(candidate["terminal_vx_multiplier"]) * q[6]
    elif kind == "longitudinal_pitch_correlation":
        rho = float(candidate["rho"])
        for left, right in ((0, 4), (6, 10)):
            value = rho * np.sqrt(q[left] * q[right])
            q_run[left, right] = value
            q_run[right, left] = value
    elif kind == "wheel_rate_differential_only":
        if abs(q[14] - q[15]) > 1e-12:
            raise RuntimeError("B4 requires symmetric baseline wheel-rate weights")
        q_run[14:16, 14:16] = 0.5 * q[14] * np.array([[1.0, -1.0], [-1.0, 1.0]])
    elif kind != "baseline":
        raise RuntimeError(f"unknown candidate kind: {kind}")
    return q_run, q_terminal


def spectral_report(matrix: np.ndarray, floor: float) -> dict:
    eigenvalues = np.linalg.eigvalsh(matrix)
    positive = eigenvalues[eigenvalues > floor]
    return {
        "minimum_eigenvalue": float(eigenvalues[0]),
        "maximum_eigenvalue": float(eigenvalues[-1]),
        "positive_subspace_condition": float(positive[-1] / positive[0]) if positive.size else float("inf"),
        "nullity": int(np.sum(eigenvalues <= floor)),
    }


def converged(solver: StructuredSolver, problem: Any, gates: dict, *, details: bool = False) -> dict:
    solver.reset()
    result = P29.converged_solve(solver, problem, gates, details=details)
    result["kkt_pass"] = bool(
        result["status"] == 0
        and result["residuals"][0] <= float(gates["converged_stationarity_max"])
        and max(result["residuals"][1:]) <= float(gates["converged_feasibility_max"])
        and result["cost_error"] <= float(gates["cost_recompute_max_abs_error"])
    )
    return result


def replay(solver: StructuredSolver, sequence: list[Any], tick: int) -> dict:
    result = None
    for index, problem in enumerate(sequence):
        if problem.tick > tick:
            break
        result = solver.solve(problem, cold=index == 0, details=problem.tick == tick)
    if result is None:
        raise RuntimeError(f"no problem through tick {tick}")
    return result


def shifted(problem: Any, index: int, delta: float) -> Any:
    state = problem.state.copy()
    state[index] += delta
    return P29.Problem(state, problem.reference, problem.center, problem.rotation, problem.tick, problem.logged_wrench)


def branch_signature(solution: dict, threshold: float) -> tuple[bool, bool]:
    stage = solution["stage"]
    return (
        float(stage["minimum_input_bound_distance"]) <= threshold,
        float(stage["minimum_state_bound_distance"]) <= threshold,
    )


def derivative(
    solver: StructuredSolver,
    problem: Any,
    state_index: int,
    acceleration_index: int,
    delta: float,
    gates: dict,
) -> dict:
    minus = converged(solver, shifted(problem, state_index, -delta), gates, details=True)
    plus = converged(solver, shifted(problem, state_index, delta), gates, details=True)
    value = float(
        (plus["acceleration"][acceleration_index] - minus["acceleration"][acceleration_index])
        / (2.0 * delta)
    )
    signatures = [branch_signature(item, float(gates["active_bound_distance"])) for item in (minus, plus)]
    return {
        "state_index": state_index,
        "acceleration_index": acceleration_index,
        "delta": delta,
        "acceleration_derivative": value,
        "corrective_derivative": -value,
        "branch_signatures": signatures,
        "branch_pass": bool(signatures[0] == signatures[1] and minus["kkt_pass"] and plus["kkt_pass"]),
        "minus_residuals": minus["residuals"],
        "plus_residuals": plus["residuals"],
    }


def neighborhood(sequence: list[Any], action_tick: int) -> list[Any]:
    position = next(index for index, problem in enumerate(sequence) if problem.tick == action_tick)
    return sequence[position - 1 : position + 2]


def decomposition(solution: dict, solver: StructuredSolver) -> dict:
    stage = solution["stage"]
    groups = {
        "longitudinal": [0, 6],
        "attitude": [3, 4, 5, 9, 10, 11],
        "wheel_position": [12, 13],
        "other": [1, 2, 7, 8],
    }
    values = {name: 0.0 for name in groups}
    values.update({"longitudinal_attitude_cross": 0.0, "wheel_common": 0.0, "wheel_differential": 0.0, "input": 0.0, "terminal": 0.0})
    for index in range(solver.n):
        error = np.asarray(stage["x"][index]) - np.asarray(stage["reference"][index])
        for name, indices in groups.items():
            block = solver.q_run[np.ix_(indices, indices)]
            values[name] += 0.5 * solver.dt * float(error[indices] @ block @ error[indices])
        values["longitudinal_attitude_cross"] += solver.dt * float(
            error[0] * solver.q_run[0, 4] * error[4]
            + error[6] * solver.q_run[6, 10] * error[10]
        )
        common = 0.5 * (error[14] + error[15])
        differential = 0.5 * (error[15] - error[14])
        wheel_block = solver.q_run[14:16, 14:16]
        common_vector = np.array([common, common])
        differential_vector = np.array([-differential, differential])
        values["wheel_common"] += 0.5 * solver.dt * float(common_vector @ wheel_block @ common_vector)
        values["wheel_differential"] += 0.5 * solver.dt * float(differential_vector @ wheel_block @ differential_vector)
        input_error = np.asarray(stage["u"][index]) - solver.eq
        values["input"] += 0.5 * solver.dt * float(np.sum(solver.r * input_error**2))
    terminal_error = np.asarray(stage["x"][-1]) - np.asarray(stage["reference"][-1])
    values["terminal"] = 0.5 * float(terminal_error @ solver.q_terminal @ terminal_error)
    values["sum"] = sum(value for key, value in values.items() if key != "sum")
    return values


def evaluate_candidate(
    branch: str,
    candidate: dict,
    problems: list[Any],
    full_sequence: list[Any],
    action_tick: int,
    rti_dir: Path,
    sqp_dir: Path,
    config: dict,
    method: dict,
) -> dict:
    q_run, q_terminal = candidate_cost(config, candidate)
    gates = method["gates"]
    sqp = StructuredSolver(sqp_dir, config, q_run, q_terminal)
    samples = {}
    sample_passes = []
    for problem in problems:
        base = converged(sqp, problem, gates, details=True)
        if branch == "T0":
            derivatives = {
                "4": derivative(sqp, problem, 4, 10, float(method["perturbation"]["rotation_rad"]), gates),
                "10": derivative(sqp, problem, 10, 10, float(method["perturbation"]["angular_velocity_rad_s"]), gates),
            }
            ax = float(base["acceleration"][6])
            guards = {
                str(index): -float(problem.state[index] - problem.reference[index]) * ax
                for index in (0, 6)
            }
            passed = bool(
                base["kkt_pass"]
                and all(value["branch_pass"] and value["corrective_derivative"] > float(gates["corrective_derivative_min_s2"]) for value in derivatives.values())
                and all(value >= float(gates["corrective_product_guard"]) for value in guards.values())
            )
        else:
            derivatives = {}
            for index in (3, 4, 5, 9, 10, 11):
                delta = float(method["perturbation"]["rotation_rad"] if index < 6 else method["perturbation"]["angular_velocity_rad_s"])
                acceleration_index = index + 6 if index < 6 else index
                derivatives[str(index)] = derivative(sqp, problem, index, acceleration_index, delta, gates)
            ax = float(base["acceleration"][6])
            passed = bool(
                base["kkt_pass"]
                and ax > 0.0
                and all(value["branch_pass"] and value["corrective_derivative"] > float(gates["corrective_derivative_min_s2"]) for value in derivatives.values())
            )
            guards = {}
        sample_passes.append(passed)
        samples[str(problem.tick)] = {
            "pass": passed,
            "state": problem.state,
            "reference": problem.reference,
            "u0": base["u0"],
            "acceleration": base["acceleration"],
            "residuals": base["residuals"],
            "cost_error": base["cost_error"],
            "derivatives": derivatives,
            "longitudinal_guards": guards,
            "objective_decomposition": decomposition(base, sqp),
        }
    rti = StructuredSolver(rti_dir, config, q_run, q_terminal)
    production = replay(rti, full_sequence, action_tick)
    action = next(problem for problem in full_sequence if problem.tick == action_tick)
    error_index = 10 if branch == "T0" else 6
    acceleration_index = 10 if branch == "T0" else 6
    reinforcing_score = float(
        (action.state[error_index] - action.reference[error_index])
        * production["acceleration"][acceleration_index]
    )
    production_pass = reinforcing_score < -float(gates["production_direction_deadband"])
    stage_spectrum = spectral_report(q_run, abs(float(gates["psd_min_eigenvalue"])))
    terminal_spectrum = spectral_report(q_terminal, abs(float(gates["psd_min_eigenvalue"])))
    psd_pass = bool(
        stage_spectrum["minimum_eigenvalue"] >= float(gates["psd_min_eigenvalue"])
        and terminal_spectrum["minimum_eigenvalue"] >= float(gates["psd_min_eigenvalue"])
    )
    return {
        "candidate": candidate,
        "q_run": q_run,
        "q_terminal": q_terminal,
        "stage_spectrum": stage_spectrum,
        "terminal_spectrum": terminal_spectrum,
        "psd_pass": psd_pass,
        "samples": samples,
        "production_lifecycle": {"reinforcing_score": reinforcing_score, "pass": production_pass, "u0": production["u0"]},
        "pass": bool(psd_pass and production_pass and all(sample_passes)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    method_path = args.method.resolve()
    method = json.loads(method_path.read_text(encoding="utf-8"))
    config_path = ROOT / method["source_ocp_config"]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    phase29_method_path = ROOT / method["source_phase29_method"]
    phase29_method = json.loads(phase29_method_path.read_text(encoding="utf-8"))
    authority = ROOT / method["source_phase29_authority"]
    source_run = ROOT / method["source_phase28_run"]
    rti_dir = ROOT / method["source_generated_dir"]
    sqp_dir = ROOT / method["offline_sqp_generated_dir"]

    prepared = {}
    parity = {}
    for case in phase29_method["cases"]:
        rows = P29.csv_rows(source_run / f"{case['id']}_control.csv")
        sequence = P29.problem_sequence(rows, float(config["equilibrium_state"][2]))
        action = max((problem for problem in sequence if problem.tick <= int(case["snapshot_tick"])), key=lambda item: item.tick)
        prepared[case["id"]] = (sequence, action, neighborhood(sequence, action.tick))
        q_run, q_terminal = baseline_cost(config)
        production = replay(StructuredSolver(rti_dir, config, q_run, q_terminal), sequence, action.tick)
        baseline = converged(StructuredSolver(sqp_dir, config, q_run, q_terminal), action, method["gates"])
        expected = json.loads((authority / f"{case['id']}_root_cause.json").read_text(encoding="utf-8"))
        production_error = float(np.max(np.abs(production["u0"] - expected["solutions"]["production"]["u0"])))
        converged_error = float(np.max(np.abs(baseline["u0"] - expected["solutions"]["converged"]["u0"])))
        objective_error = abs(float(baseline["recomputed_cost"]) - float(expected["solutions"]["converged"]["objective"]))
        parity[case["id"]] = {
            "production_u0_max_abs_error": production_error,
            "converged_u0_max_abs_error": converged_error,
            "converged_objective_abs_error": objective_error,
            "pass": bool(production_error <= float(method["gates"]["baseline_u0_max_abs_error"])
                         and converged_error <= float(method["gates"]["baseline_u0_max_abs_error"])
                         and objective_error <= float(method["gates"]["baseline_objective_max_abs_error"])
                         and baseline["kkt_pass"]),
        }
    if not all(item["pass"] for item in parity.values()):
        raise RuntimeError("structured all-one baseline parity failed")

    branch_cases = {"T0": "T0_static", "T1": "T1_straight_start_cruise_brake"}
    results = {}
    selected = {}
    for branch, case_id in branch_cases.items():
        sequence, action, problems = prepared[case_id]
        candidates = method["t0_candidates" if branch == "T0" else "t1_candidates"]
        branch_results = [
            evaluate_candidate(branch, candidate, problems, sequence, action.tick, rti_dir, sqp_dir, config, method)
            for candidate in candidates
        ]
        results[branch] = branch_results
        by_id = {item["candidate"]["id"]: item for item in branch_results}
        priority = method["t0_selection_priority" if branch == "T0" else "t1_selection_priority"]
        selected[branch] = next((identifier for identifier in priority if by_id[identifier]["pass"]), None)

    summary = {
        "all_one_parity_pass": all(item["pass"] for item in parity.values()),
        "selected": selected,
        "branches": {
            branch: {
                "candidates": {item["candidate"]["id"]: item["pass"] for item in values},
                "failure": None if selected[branch] else ("R31-A_terminal_structural_candidates_fail" if branch == "T0" else "R31-B_cross_state_structural_candidates_fail"),
            }
            for branch, values in results.items()
        },
        "pass": all(selected.values()),
    }
    output.mkdir(parents=True)
    (output / "structured_causal_matrix.json").write_text(json.dumps(clean({"parity": parity, "results": results}), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__, "casadi": casadi.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in (method_path, config_path, phase29_method_path, PHASE30_V1_SCRIPT, Path(__file__).resolve())},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
