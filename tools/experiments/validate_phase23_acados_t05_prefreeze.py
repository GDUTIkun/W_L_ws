#!/usr/bin/env python3
"""Re-solve the P23-T05 reduced-model tuning and cost ablations."""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import os
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

import acados_template
import casadi
import numpy as np
import scipy
from acados_template import AcadosOcpSolver

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PROFILE = ROOT / "simulation/mujoco/config/phase23_acados_t05_profile_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def root_relative(path: Path) -> str:
    return str(path.resolve().relative_to(ROOT))


def load_generator():
    path = ROOT / "tools/experiments/generate_phase23_acados_solver.py"
    spec = importlib.util.spec_from_file_location("phase23_generator", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase 23 generator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, path


def variant_config(base: dict, change: dict) -> dict:
    result = copy.deepcopy(base)
    if "state_weight_index" in change:
        result["state_weight"][int(change["state_weight_index"])] = float(change["value"])
    if "terminal_weight_multiplier" in change:
        result["terminal_weight_multiplier"] = float(change["terminal_weight_multiplier"])
    if "input_weight_all" in change:
        result["input_weight"] = [float(change["input_weight_all"])] * 12
    return result


def reference(profile: dict, equilibrium: np.ndarray, case: str, time_s: float) -> np.ndarray:
    result = equilibrium.copy()
    decision = profile["reference_decision"]
    if time_s < float(decision["step_start_s"]):
        return result
    amplitude = float(decision["amplitude_m"])
    if case == "positive":
        result[0] += amplitude
    elif case == "negative":
        result[0] -= amplitude
    elif case == "return" and time_s < float(decision["return_start_s"]):
        result[0] += amplitude
    return result


def simulate(solver: AcadosOcpSolver, config: dict, profile: dict, case: str) -> dict:
    horizon = int(config["horizon_steps"])
    period = float(config["sampling_period_s"])
    equilibrium_state = np.asarray(config["equilibrium_state"], dtype=float)
    equilibrium_input = np.asarray(config["equilibrium_input"], dtype=float)
    input_scale = np.asarray(config["input_error_scale"], dtype=float)
    input_lower = np.asarray(config["input_lower"], dtype=float)
    input_upper = np.asarray(config["input_upper"], dtype=float)
    state_envelope = np.asarray(
        profile["constraint_decision"]["relative_state_envelope"], dtype=float
    )
    state = equilibrium_state.copy()
    previous_input = equilibrium_input.copy()
    positions: list[float] = []
    maximum_delta = 0.0
    maximum_input_bound_violation = 0.0
    maximum_state_envelope_violation = 0.0
    maximum_protected_input = 0.0
    statuses: list[int] = []
    solver.reset()
    for tick in range(int(round(10.0 / period))):
        target = reference(profile, equilibrium_state, case, tick * period)
        yref = np.concatenate((target, equilibrium_input))
        for stage in range(horizon):
            solver.set(stage, "yref", yref)
        solver.set(horizon, "yref", target)
        solver.set(0, "lbx", state)
        solver.set(0, "ubx", state)
        status = int(solver.solve())
        statuses.append(status)
        if status != 0:
            break
        current_input = solver.get(0, "u")
        maximum_delta = max(
            maximum_delta,
            float(np.max(np.abs((current_input - previous_input) / input_scale))),
        )
        protected = np.r_[1:6, 7:12]
        maximum_protected_input = max(
            maximum_protected_input,
            float(np.max(np.abs((current_input - equilibrium_input)[protected] / input_scale[protected]))),
        )
        maximum_input_bound_violation = max(
            maximum_input_bound_violation,
            float(np.max(np.maximum(input_lower - current_input, current_input - input_upper))),
        )
        previous_input = current_input
        state = solver.get(1, "x")
        deviation = state - equilibrium_state
        maximum_state_envelope_violation = max(
            maximum_state_envelope_violation,
            float(np.max(np.abs(deviation) - state_envelope)),
        )
        positions.append(float(deviation[0]))
    tail = positions[-100:]
    return {
        "statuses": sorted(set(statuses)),
        "ticks_completed": len(positions),
        "final_mean_x_m": float(np.mean(tail)) if tail else None,
        "maximum_x_m": max(positions) if positions else None,
        "minimum_x_m": min(positions) if positions else None,
        "maximum_normalized_delta_wrench": maximum_delta,
        "maximum_normalized_protected_wrench_deviation": maximum_protected_input,
        "maximum_input_bound_violation": max(0.0, maximum_input_bound_violation),
        "maximum_relative_state_envelope_violation": max(0.0, maximum_state_envelope_violation),
    }


def evaluate(results: dict, gates: dict) -> dict:
    baseline = results["baseline"]["cases"]
    longitudinal = results["no_longitudinal_state_cost"]["cases"]
    no_terminal = results["no_terminal_cost"]["cases"]
    unselective = results["unselective_wrench_cost"]["cases"]
    all_cases = [case for variant in results.values() for case in variant["cases"].values()]
    baseline_return = abs(float(baseline["return"]["final_mean_x_m"]))
    terminal_return = abs(float(no_terminal["return"]["final_mean_x_m"]))
    baseline_protected = max(
        float(case["maximum_normalized_protected_wrench_deviation"])
        for case in baseline.values()
    )
    unselective_protected = max(
        float(case["maximum_normalized_protected_wrench_deviation"])
        for case in unselective.values()
    )
    checks = {
        "all_variants_solve": all(case["statuses"] == [0] for case in all_cases),
        "baseline_hold": abs(float(baseline["hold"]["final_mean_x_m"])) <= gates["maximum_model_hold_error_m"],
        "baseline_positive_tracking": float(baseline["positive"]["final_mean_x_m"]) >= gates["minimum_model_step_tracking_m"],
        "baseline_negative_tracking": float(baseline["negative"]["final_mean_x_m"]) <= -gates["minimum_model_step_tracking_m"],
        "baseline_return_excursion": float(baseline["return"]["maximum_x_m"]) >= gates["minimum_model_return_excursion_m"],
        "baseline_return_recovery": baseline_return <= gates["maximum_model_return_error_m"],
        "baseline_delta_wrench": max(float(case["maximum_normalized_delta_wrench"]) for case in baseline.values()) <= gates["maximum_normalized_delta_wrench"],
        "all_input_bounds": max(float(case["maximum_input_bound_violation"]) for case in all_cases) <= gates["maximum_input_bound_violation"],
        "baseline_state_envelope": max(float(case["maximum_relative_state_envelope_violation"]) for case in baseline.values()) <= gates["maximum_relative_state_envelope_violation"],
        "longitudinal_cost_attribution": (
            abs(float(baseline["positive"]["final_mean_x_m"]) - float(longitudinal["positive"]["final_mean_x_m"]))
            >= gates["minimum_longitudinal_ablation_loss_m"]
        ),
        "terminal_cost_attribution": terminal_return >= gates["minimum_terminal_return_error_ratio"] * max(baseline_return, 1e-12),
        "selective_wrench_attribution": unselective_protected >= gates["minimum_unselective_protected_wrench_ratio"] * max(baseline_protected, 1e-12),
    }
    return {
        "pass": all(checks.values()),
        "checks": checks,
        "comparison": {
            "baseline_return_error_m": baseline_return,
            "no_terminal_return_error_m": terminal_return,
            "terminal_return_error_ratio": terminal_return / max(baseline_return, 1e-12),
            "baseline_protected_wrench": baseline_protected,
            "unselective_protected_wrench": unselective_protected,
            "unselective_protected_wrench_ratio": unselective_protected / max(baseline_protected, 1e-12),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--supersedes", action="append", default=[])
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output must be absent or empty: {output}")
    profile_path = args.profile.resolve()
    profile = json.loads(profile_path.read_text(encoding="utf-8"))
    ocp_path = (ROOT / profile["parent_ocp"]).resolve()
    base = json.loads(ocp_path.read_text(encoding="utf-8"))
    generator, generator_path = load_generator()
    results = {}
    with tempfile.TemporaryDirectory(prefix="phase23-t05-ablation.") as temporary:
        for name, change in profile["ablation_variants"].items():
            config = variant_config(base, change)
            export = Path(temporary) / name
            ocp = generator.create_ocp(config, export)
            solver = AcadosOcpSolver(ocp, build=True, generate=True, verbose=False)
            results[name] = {
                "change": change,
                "cases": {
                    case: simulate(solver, config, profile, case)
                    for case in ("hold", "positive", "negative", "return")
                },
            }
    evaluation = evaluate(results, profile["prefreeze_gates"])
    output.mkdir(parents=True, exist_ok=True)
    summary = {
        "schema_version": 1,
        "phase": 23,
        "task": "P23-T05",
        "evidence_class": "reduced-model pre-freeze tuning and true solver ablation",
        "pass": evaluation["pass"],
        "evaluation": evaluation,
        "variants": results,
    }
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "phase": 23,
        "task": "P23-T05",
        "profile": root_relative(profile_path),
        "profile_sha256": sha256(profile_path),
        "ocp": root_relative(ocp_path),
        "ocp_sha256": sha256(ocp_path),
        "generator": root_relative(generator_path),
        "generator_sha256": sha256(generator_path),
        "validator": root_relative(Path(__file__)),
        "validator_sha256": sha256(Path(__file__)),
        "summary_sha256": sha256(summary_path),
        "python": sys.executable,
        "python_version": platform.python_version(),
        "dependencies": {
            "numpy": np.__version__,
            "scipy": scipy.__version__,
            "casadi": casadi.__version__,
            "acados_template": str(Path(acados_template.__file__).resolve()),
        },
        "acados_source_dir": os.environ.get("ACADOS_SOURCE_DIR"),
        "acados_commit": subprocess.check_output(
            ["git", "-C", os.environ["ACADOS_SOURCE_DIR"], "rev-parse", "HEAD"], text=True
        ).strip(),
        "supersedes": args.supersedes,
        "replay_of": None,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"pass": summary["pass"], "checks": evaluation["checks"]}, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
