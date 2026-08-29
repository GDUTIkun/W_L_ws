#!/usr/bin/env python3
"""Audit Phase 30 horizon-reference feasibility before any further NMPC cost changes."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import casadi as ca
import mujoco
import numpy as np
import scipy
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase30_reference_consistency_v3.json"
P29_PATH = ROOT / "tools/experiments/run_phase29_nmpc_root_cause.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P29 = load_module(P29_PATH, "phase30_v3_phase29")


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


def exact_discrete_function(config: dict, generator_path: Path) -> ca.Function:
    generator = load_module(generator_path, "phase30_v3_phase27_generator")
    ocp = generator.create_ocp(config, ROOT / ".phase30-v3-unused-codegen")
    return ca.Function(
        "phase30_v3_exact_discrete",
        [ocp.model.x, ocp.model.u, ocp.model.p],
        [ocp.model.disc_dyn_expr],
    )


def stage_references(problem: Any, horizon: int, step: float) -> list[np.ndarray]:
    return [P29.advance(problem.reference, stage, step) for stage in range(horizon + 1)]


def grouped(defect: np.ndarray, scale: np.ndarray, groups: dict[str, list[int]]) -> dict[str, Any]:
    normalized = defect / scale
    report: dict[str, Any] = {}
    for name, indices in groups.items():
        values = defect[indices]
        scaled = normalized[indices]
        report[name] = {
            "raw": values,
            "normalized": scaled,
            "normalized_max_abs": float(np.max(np.abs(scaled))),
        }
    wheel = {
        "wheel_common_position": (0.5 * (defect[12] + defect[13]), 0.5 * (scale[12] + scale[13])),
        "wheel_differential_position": (0.5 * (defect[13] - defect[12]), 0.5 * (scale[12] + scale[13])),
        "wheel_common_rate": (0.5 * (defect[14] + defect[15]), 0.5 * (scale[14] + scale[15])),
        "wheel_differential_rate": (0.5 * (defect[15] - defect[14]), 0.5 * (scale[14] + scale[15])),
    }
    for name, (value, denominator) in wheel.items():
        report[name] = {
            "raw": float(value),
            "normalized": float(value / denominator),
            "normalized_max_abs": abs(float(value / denominator)),
        }
    return report


def baseline_authority(
    case: dict,
    sequence: list[Any],
    authority: dict,
    rti_dir: Path,
    sqp_dir: Path,
    config: dict,
    gates: dict,
    solver_gates: dict,
) -> dict[str, Any]:
    action_tick = int(case["expected_action_tick"])
    action = next(problem for problem in sequence if problem.tick == action_tick)
    expected_problem = authority["problem"]
    semantic_errors = {
        "state": float(np.max(np.abs(action.state - np.asarray(expected_problem["state"])) )),
        "reference": float(np.max(np.abs(action.reference - np.asarray(expected_problem["reference"])) )),
        "rotation": float(np.max(np.abs(action.rotation - np.asarray(expected_problem["rotation"])) )),
        "logged_wrench": float(np.max(np.abs(action.logged_wrench - np.asarray(expected_problem["logged_wrench"])) )),
    }
    rti = P29.OfflineSolver(rti_dir, config)
    prefix_errors = []
    production = None
    for index, problem in enumerate(sequence):
        if problem.tick > action_tick:
            break
        solved = rti.solve(problem, cold=index == 0, details=problem.tick == action_tick)
        prefix_errors.append(float(np.max(np.abs(solved["u0"] - problem.logged_wrench))))
        if problem.tick == action_tick:
            production = solved
    if production is None:
        raise RuntimeError(f"missing action tick {action_tick}")
    sqp = P29.OfflineSolver(sqp_dir, config)
    converged = P29.converged_solve(sqp, action, solver_gates, details=False)
    production_error = float(np.max(np.abs(production["u0"] - np.asarray(authority["solutions"]["production"]["u0"]))))
    converged_error = float(np.max(np.abs(converged["u0"] - np.asarray(authority["solutions"]["converged"]["u0"]))))
    semantic_max = max(semantic_errors.values())
    request_max = max(prefix_errors + [production_error, converged_error])
    return {
        "action_tick": action_tick,
        "semantic_errors": semantic_errors,
        "semantic_max_abs_error": semantic_max,
        "prefix_and_solution_max_abs_error": request_max,
        "production_u0_max_abs_error": production_error,
        "converged_u0_max_abs_error": converged_error,
        "pass": bool(
            semantic_max <= float(gates["baseline_semantics_max_abs_error"])
            and request_max <= float(gates["baseline_request_max_abs_error"])
        ),
    }


def classify(current_max: float, best_max: float, gates: dict) -> str:
    large = float(gates["current_large_normalized_max"])
    small = float(gates["defect_small_normalized_max"])
    ratio = best_max / current_max if current_max > 0.0 else 0.0
    if current_max <= small:
        return "P31-C_current_reference_already_consistent"
    if current_max >= large and best_max <= small and ratio <= float(gates["case_a_best_to_current_ratio_max"]):
        return "P31-A_stage_input_feedforward_inconsistent"
    if current_max >= large and best_max > small:
        return "P31-B_state_reference_dynamically_inconsistent"
    return "unresolved_transition_band"


def audit_problem(
    problem: Any,
    discrete: ca.Function,
    config: dict,
    method: dict,
) -> dict[str, Any]:
    horizon = int(config["horizon_steps"])
    references = stage_references(problem, horizon, float(config["sampling_period_s"]))
    equilibrium = np.asarray(config["equilibrium_input"], dtype=float)
    lower = np.asarray(config["input_lower"], dtype=float)
    upper = np.asarray(config["input_upper"], dtype=float)
    scale = np.asarray(config["state_error_scale"], dtype=float)
    parameter = problem.rotation.reshape(-1)
    options = method["best_input_solver"]
    stages = []
    current_max = 0.0
    best_max = 0.0
    initial = equilibrium.copy()
    all_success = True
    for stage in range(horizon):
        state = references[stage]
        target = references[stage + 1]

        def defect(control: np.ndarray) -> np.ndarray:
            predicted = np.asarray(discrete(state, control, parameter)).reshape(-1)
            return target - predicted

        current_defect = defect(equilibrium)
        solved = least_squares(
            lambda control: defect(control) / scale,
            initial,
            bounds=(lower, upper),
            method="trf",
            xtol=float(options["xtol"]),
            ftol=float(options["ftol"]),
            gtol=float(options["gtol"]),
            max_nfev=int(options["max_nfev"]),
            x_scale="jac",
        )
        best_defect = defect(solved.x)
        current_stage_max = float(np.max(np.abs(current_defect / scale)))
        best_stage_max = float(np.max(np.abs(best_defect / scale)))
        current_max = max(current_max, current_stage_max)
        best_max = max(best_max, best_stage_max)
        bound_distance = np.minimum(solved.x - lower, upper - solved.x)
        stages.append({
            "stage": stage,
            "x_ref": state,
            "x_ref_next": target,
            "u_ref_current": equilibrium,
            "u_ref_best": solved.x,
            "current_defect": current_defect,
            "best_defect": best_defect,
            "current_normalized_max_abs": current_stage_max,
            "best_normalized_max_abs": best_stage_max,
            "current_groups": grouped(current_defect, scale, method["state_groups"]),
            "best_groups": grouped(best_defect, scale, method["state_groups"]),
            "optimizer": {
                "success": solved.success,
                "status": solved.status,
                "message": solved.message,
                "nfev": solved.nfev,
                "cost": solved.cost,
                "optimality": solved.optimality,
                "active_bounds": np.flatnonzero(bound_distance <= float(method["gates"]["active_bound_distance"])),
            },
        })
        all_success = all_success and bool(solved.success)
        initial = solved.x
    return {
        "tick": problem.tick,
        "rotation": problem.rotation,
        "reference_anchor": problem.reference,
        "current_normalized_max_abs": current_max,
        "best_normalized_max_abs": best_max,
        "best_to_current_ratio": best_max / current_max if current_max > 0.0 else 0.0,
        "optimizer_all_success": all_success,
        "classification": classify(current_max, best_max, method["gates"]),
        "stages": stages,
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
    generator_path = ROOT / method["source_generator"]
    authority_dir = ROOT / method["source_phase29_authority"]
    raw_dir = ROOT / method["source_phase28_run"]
    rti_dir = ROOT / method["source_generated_dir"]
    sqp_dir = ROOT / method["offline_sqp_generated_dir"]
    discrete = exact_discrete_function(config, generator_path)

    authorities = {}
    results = {}
    cases = {case["id"]: case for case in phase29_method["cases"]}
    for case_id in method["case_ids"]:
        case = cases[case_id]
        rows = P29.csv_rows(raw_dir / f"{case_id}_control.csv")
        sequence = P29.problem_sequence(rows, float(config["equilibrium_state"][2]))
        authority = json.loads((authority_dir / f"{case_id}_root_cause.json").read_text(encoding="utf-8"))
        authorities[case_id] = baseline_authority(
            case,
            sequence,
            authority,
            rti_dir,
            sqp_dir,
            config,
            method["gates"],
            phase29_method["gates"],
        )
        action_index = next(index for index, problem in enumerate(sequence) if problem.tick == int(case["expected_action_tick"]))
        selected = [sequence[action_index + offset] for offset in method["neighbor_update_offsets"]]
        results[case_id] = [audit_problem(problem, discrete, config, method) for problem in selected]

    if not all(item["pass"] for item in authorities.values()):
        raise RuntimeError("Gate 0 baseline authority failed")
    classifications = {
        case_id: sorted({item["classification"] for item in items})
        for case_id, items in results.items()
    }
    optimizer_pass = all(item["optimizer_all_success"] for items in results.values() for item in items)
    summary = {
        "gate0_baseline_authority_pass": True,
        "gate1_current_defect_export_pass": True,
        "gate2_optimizer_pass": optimizer_pass,
        "classifications": classifications,
        "single_classification_per_case": all(len(values) == 1 for values in classifications.values()),
        "pass_for_branch_selection": bool(optimizer_pass and all(len(values) == 1 for values in classifications.values())),
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "reference_defect_audit.json").write_text(
        json.dumps(clean({"authority": authorities, "results": results}), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest_inputs = [method_path, config_path, phase29_method_path, generator_path, P29_PATH, Path(__file__).resolve()]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "mujoco": mujoco.__version__,
            "casadi": ca.__version__,
            "acados_template": importlib.metadata.version("acados_template"),
        },
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in manifest_inputs},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if summary["pass_for_branch_selection"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
