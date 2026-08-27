#!/usr/bin/env python3
"""Run the fixed Phase-21 failure-attribution matrix without retuning the QP."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_tasks import ControllerOracle, Plant  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_attribution.json"
TASKS = ("contact", "base_x", "height", "orientation", "leg")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def finite_or_none(value: float) -> float | None:
    return float(value) if np.isfinite(value) else None


def vector_json(value: np.ndarray | list[float]) -> str:
    return json.dumps(np.asarray(value, dtype=float).tolist(), separators=(",", ":"))


def enabled_tasks(case: dict[str, Any]) -> set[str]:
    if "single_task" in case:
        return {case["single_task"]}
    result = set(TASKS)
    if "disable_task" in case:
        result.remove(case["disable_task"])
    return result


def correlation(left: list[float], right: list[float]) -> float | None:
    x = np.asarray(left, dtype=float); y = np.asarray(right, dtype=float)
    valid = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(valid) < 3 or np.std(x[valid]) <= 1e-14 or np.std(y[valid]) <= 1e-14:
        return None
    return float(np.corrcoef(x[valid], y[valid])[0, 1])


def gate_result(result: dict[str, Any], gates: dict[str, Any]) -> dict[str, bool]:
    return {
        "position": bool(result["abs_x_m"] <= gates["maximum_abs_x_m"] and
                         result["abs_y_m"] <= gates["maximum_abs_y_m"] and
                         result["height_error_m"] <= gates["maximum_height_error_m"]),
        "orientation": bool(result["abs_roll_rad"] <= gates["maximum_abs_roll_rad"] and
                            result["abs_pitch_rad"] <= gates["maximum_abs_pitch_rad"] and
                            result["abs_yaw_rad"] <= gates["maximum_abs_yaw_rad"]),
        "settling": bool(result["final_linear_speed_m_s"] <= gates["maximum_final_linear_speed_m_s"] and
                         result["final_angular_speed_rad_s"] <= gates["maximum_final_angular_speed_rad_s"]),
        "contact": bool(result["bilateral_contact_fraction"] >= gates["minimum_bilateral_contact_fraction"] and
                        result["minimum_final_normal_force_n"] >= gates["minimum_normal_force_n"]),
        "plant": bool(result["penetration_m"] <= gates["maximum_penetration_m"] and
                      result["rolling_slip_m_s"] <= gates["maximum_abs_rolling_slip_m_s"] and
                      result["lateral_slip_m_s"] <= gates["maximum_abs_lateral_slip_m_s"] and
                      result["closure_residual_m"] <= gates["maximum_closure_residual_m"]),
        "qp": bool(result["hard_residual"] <= gates["maximum_hard_residual"] and
                   result["bound_violation"] <= gates["maximum_bound_violation"] and
                   result["wrench_slack"] <= gates["maximum_abs_wrench_slack"] and
                   result["solver_failure_count"] <= gates["maximum_solver_failure_count"] and
                   result["saturation_count"] <= gates["maximum_saturation_count"]),
    }


def stack_window(records: list[dict[str, Any]], path: Path) -> None:
    if not records:
        raise RuntimeError("Failure-window capture is empty")
    keys = records[0].keys()
    arrays = {key: np.stack([record[key] for record in records]) for key in keys}
    np.savez_compressed(path, **arrays)


def run_case(controller: ControllerOracle, plant: Plant, case: dict[str, Any], ticks: int,
             failure_window: tuple[int, int], directory: Path,
             direction_threshold: float) -> dict[str, Any]:
    directory.mkdir(parents=True, exist_ok=False)
    controller.reset(); plant.reset()
    enabled = enabled_tasks(case)
    wrench_enabled = not bool(case.get("disable_wrench_fidelity", False))
    reference_height = float(controller.reference_position[2])
    maxima = {"abs_x_m": 0.0, "abs_y_m": 0.0, "height_error_m": 0.0,
              "abs_roll_rad": 0.0, "abs_pitch_rad": 0.0, "abs_yaw_rad": 0.0,
              "penetration_m": 0.0, "rolling_slip_m_s": 0.0, "lateral_slip_m_s": 0.0,
              "closure_residual_m": 0.0, "hard_residual": 0.0, "bound_violation": 0.0,
              "wrench_slack": 0.0, "contact_generalized_mismatch": 0.0,
              "cop_error_m": 0.0}
    failures = saturation = bilateral = maximum_iterations = 0
    first_failure_tick: int | None = None
    histories: dict[str, list[float]] = {name: [] for name in
        ("mismatch", "rolling_slip", "base_x_error", "cop_error", "iterations", "slack")}
    for task in TASKS:
        histories[f"task_{task}_residual"] = []
        histories[f"task_{task}_cost"] = []
    direction: dict[str, list[float]] = {task: [] for task in TASKS}
    window_records: list[dict[str, Any]] = []
    fields = ["tick", "time_s", "solver_status", "iterations", "solve_time_ms",
              "primal_residual", "dual_residual", "stationarity_residual", "hard_residual",
              "bound_violation", "x_m", "y_m", "height_m", "roll_rad", "pitch_rad", "yaw_rad",
              "linear_speed_m_s", "angular_speed_rad_s", "left_normal_n", "right_normal_n",
              "penetration_m", "rolling_slip_m_s", "lateral_slip_m_s", "closure_residual_m",
              "wheel_angle_left_rad", "wheel_angle_right_rad", "wheel_roll_distance_left_m",
              "wheel_roll_distance_right_m", "wrench_slack", "wrench_fidelity_enabled",
              "wrench_slack_penalty", "contact_generalized_mismatch_norm", "contact_generalized_mismatch",
              "truth_contact_force", "truth_contact_moment_about_wheel", "truth_cop", "model_contact_points",
              "cop_error_m", "minimum_torque_margin_nm", "minimum_normal_force_n",
              "minimum_friction_margin_n", "minimum_acceleration_margin", "active_torque_count",
              "active_normal_count", "active_friction_count", "active_acceleration_count"]
    for task in TASKS:
        fields.extend((f"{task}_enabled", f"{task}_target", f"{task}_achieved",
                       f"{task}_normalized_residual", f"{task}_residual_norm",
                       f"{task}_weighted_cost", f"{task}_direction_dot", f"{task}_direction_cosine"))
    fields.extend([f"nudot_{i}" for i in range(12)] + [f"tau_{i}" for i in range(6)] +
                  [f"lambda_{i}" for i in range(6)] + [f"slack_{i}" for i in range(12)])
    with (directory / "ticks.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for tick in range(ticks):
            capture = failure_window[0] <= tick <= failure_window[1]
            solved = controller.solve(plant.data.qpos.copy(), plant.data.qvel.copy(), enabled,
                                      wrench_fidelity_enabled=wrench_enabled, capture_problem=capture)
            truth = plant.contact_truth(solved["audit"]["reduction"])
            model_force = solved["audit"]["model_contact_generalized_force"]
            mismatch_vector = truth["reduced_generalized_force"] - model_force
            mismatch = float(np.linalg.norm(mismatch_vector))
            contact_points = solved["audit"]["contact_points"]
            cop_side_error = np.linalg.norm(truth["cop"] - contact_points, axis=1)
            finite_cop = cop_side_error[np.isfinite(cop_side_error)]
            cop_error = float(np.max(finite_cop)) if finite_cop.size else float("nan")
            valid = solved["status"] == "converged" and np.all(np.isfinite(solved["physical"]))
            if not valid and first_failure_tick is None:
                first_failure_tick = tick
            failures += int(not valid)
            physical = solved["physical"]
            torque = physical[12:18] if valid else np.zeros(6)
            limits = np.asarray(controller.qp["bounds"]["torque_nm"])
            saturation += int(np.any(np.abs(torque) >= limits - 1e-6))
            plant.data.ctrl[:] = 0.0
            for actuator, value in zip(plant.actuators, torque):
                plant.data.ctrl[actuator] = -value
            for _ in range(5):
                mujoco.mj_step(plant.model, plant.data)
            metrics = plant.metrics()
            bilateral += int(metrics["left_normal_n"] > 0.0 and metrics["right_normal_n"] > 0.0)
            x_error = abs(metrics["x_m"] - controller.reference_position[0])
            maxima["abs_x_m"] = max(maxima["abs_x_m"], x_error)
            maxima["abs_y_m"] = max(maxima["abs_y_m"], abs(metrics["y_m"] - controller.reference_position[1]))
            maxima["height_error_m"] = max(maxima["height_error_m"], abs(metrics["height_m"] - reference_height))
            for name in ("roll", "pitch", "yaw"):
                maxima[f"abs_{name}_rad"] = max(maxima[f"abs_{name}_rad"], abs(metrics[f"{name}_rad"]))
            for name in ("penetration_m", "rolling_slip_m_s", "lateral_slip_m_s", "closure_residual_m"):
                maxima[name] = max(maxima[name], metrics[name])
            maxima["hard_residual"] = max(maxima["hard_residual"], solved["hard_residual"])
            maxima["bound_violation"] = max(maxima["bound_violation"], solved.get("bound_violation", float("inf")))
            maxima["wrench_slack"] = max(maxima["wrench_slack"], solved["maximum_abs_slack"])
            maxima["contact_generalized_mismatch"] = max(maxima["contact_generalized_mismatch"], mismatch)
            if np.isfinite(cop_error): maxima["cop_error_m"] = max(maxima["cop_error_m"], cop_error)
            maximum_iterations = max(maximum_iterations, int(solved.get("iterations", 0)))
            histories["mismatch"].append(mismatch); histories["rolling_slip"].append(metrics["rolling_slip_m_s"])
            histories["base_x_error"].append(x_error); histories["cop_error"].append(cop_error)
            histories["iterations"].append(float(solved.get("iterations", 0)))
            histories["slack"].append(solved["maximum_abs_slack"])
            row: dict[str, Any] = {"tick": tick, "time_s": plant.data.time,
                "solver_status": solved["status"], "iterations": solved.get("iterations", 0),
                "solve_time_ms": solved.get("solve_time_ms", float("nan")),
                "primal_residual": solved.get("primal_residual", float("nan")),
                "dual_residual": solved.get("dual_residual", float("nan")),
                "stationarity_residual": solved.get("stationarity_residual", float("nan")),
                "hard_residual": solved["hard_residual"], "bound_violation": solved.get("bound_violation", float("inf")),
                **metrics, "wheel_angle_left_rad": plant.data.qpos[plant.model.jnt_qposadr[plant.active_joints[2]]],
                "wheel_angle_right_rad": plant.data.qpos[plant.model.jnt_qposadr[plant.active_joints[5]]],
                "wheel_roll_distance_left_m": 0.05 * plant.data.qpos[plant.model.jnt_qposadr[plant.active_joints[2]]],
                "wheel_roll_distance_right_m": 0.05 * plant.data.qpos[plant.model.jnt_qposadr[plant.active_joints[5]]],
                "wrench_slack": solved["maximum_abs_slack"],
                "wrench_fidelity_enabled": solved["wrench_fidelity_enabled"],
                "wrench_slack_penalty": solved["wrench_slack_penalty"],
                "contact_generalized_mismatch_norm": mismatch,
                "contact_generalized_mismatch": vector_json(mismatch_vector),
                "truth_contact_force": vector_json(truth["forces"]),
                "truth_contact_moment_about_wheel": vector_json(truth["moments_about_wheel"]),
                "truth_cop": vector_json(truth["cop"]), "model_contact_points": vector_json(contact_points),
                "cop_error_m": cop_error, **solved["bound_diagnostics"]}
            for task, values in solved["task_diagnostics"].items():
                histories[f"task_{task}_residual"].append(values["residual_norm"])
                histories[f"task_{task}_cost"].append(values["weighted_cost"])
                if values["enabled"] and values["target_norm"] > direction_threshold and np.isfinite(values["direction_cosine"]):
                    direction[task].append(values["direction_cosine"])
                row.update({f"{task}_enabled": values["enabled"], f"{task}_target": vector_json(values["target"]),
                            f"{task}_achieved": vector_json(values["achieved"]),
                            f"{task}_normalized_residual": vector_json(values["normalized_residual"]),
                            f"{task}_residual_norm": values["residual_norm"],
                            f"{task}_weighted_cost": values["weighted_cost"],
                            f"{task}_direction_dot": values["direction_dot"],
                            f"{task}_direction_cosine": values["direction_cosine"]})
            row.update({f"nudot_{i}": physical[i] for i in range(12)})
            row.update({f"tau_{i}": physical[12 + i] for i in range(6)})
            row.update({f"lambda_{i}": physical[18 + i] for i in range(6)})
            row.update({f"slack_{i}": physical[24 + i] for i in range(12)})
            writer.writerow(row)
            if capture:
                audit = solved["audit"]
                window_records.append({"tick": np.asarray(tick), "H": audit["H"], "g": audit["g"],
                    "A": audit["A"], "l": audit["l"], "u": audit["u"],
                    "variable_scale": audit["variable_scale"], "row_scale_dynamics": audit["row_scale_dynamics"],
                    "row_scale_wrench": audit["row_scale_wrench"], "warm_start_before": audit["warm_start_before"],
                    "scaled_solution": audit["scaled_solution"], "reference_wrench": audit["reference_wrench"],
                    "solver_status": np.asarray(1 if solved["status"] == "converged" else 0),
                    "iterations": np.asarray(solved.get("iterations", 0)),
                    "primal_residual": np.asarray(solved.get("primal_residual", np.nan)),
                    "dual_residual": np.asarray(solved.get("dual_residual", np.nan)),
                    "stationarity_residual": np.asarray(solved.get("stationarity_residual", np.nan)),
                    "bound_violation": np.asarray(solved.get("bound_violation", np.nan))})
    stack_window(window_records, directory / "failure_window.npz")
    final = plant.metrics()
    correlations = {key: correlation(histories["mismatch"], values)
                    for key, values in histories.items() if key != "mismatch"}
    direction_summary = {task: {"sample_count": len(values),
                                "minimum_cosine": finite_or_none(min(values)) if values else None,
                                "median_cosine": finite_or_none(float(np.median(values))) if values else None,
                                "positive_fraction": float(np.mean(np.asarray(values) > 0.0)) if values else None}
                         for task, values in direction.items()}
    return {**maxima, "enabled_tasks": sorted(enabled), "wrench_fidelity_enabled": wrench_enabled,
            "first_solver_failure_tick": first_failure_tick, "solver_failure_count": failures,
            "saturation_count": saturation, "maximum_iterations": maximum_iterations,
            "bilateral_contact_fraction": bilateral / ticks,
            "minimum_final_normal_force_n": min(final["left_normal_n"], final["right_normal_n"]),
            "final_linear_speed_m_s": final["linear_speed_m_s"],
            "final_angular_speed_rad_s": final["angular_speed_rad_s"],
            "mismatch_correlations": correlations, "direction_audit": direction_summary}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--case", action="append")
    parser.add_argument("--ticks", type=int)
    args = parser.parse_args(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_path = args.config.resolve(); config, config_inputs = load_config(config_path)
    model_path = (ROOT / config["model_profile"]).resolve(); model_config, model_inputs = load_config(model_path)
    qp_path = (ROOT / config["qp_profile"]).resolve(); qp_config, qp_inputs = load_config(qp_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    controller = ControllerOracle(config, model_config, equilibrium, qp_config)
    plant = Plant(model_config, equilibrium)
    ticks = args.ticks or int(round(config["duration_s"] /
        (plant.model.opt.timestep * config["physics_steps_per_control"])))
    cases = config["attribution"]["cases"]
    if args.case:
        requested = set(args.case); cases = [case for case in cases if case["id"] in requested]
        missing = requested - {case["id"] for case in cases}
        if missing: raise RuntimeError(f"Unknown attribution cases: {sorted(missing)}")
    window = tuple(int(value) for value in config["attribution"]["failure_window_ticks"])
    results = {}
    for case in cases:
        print(f"running {case['id']} ({ticks} ticks)", flush=True)
        results[case["id"]] = run_case(controller, plant, case, ticks, window,
            output / case["id"], float(config["attribution"]["direction_target_threshold"]))
    case_gates = {name: gate_result(result, config["gates"]) for name, result in results.items()}
    summary = {"schema_version": 1, "phase": 21, "purpose": "failure_attribution_only",
               "profile": config["profile"], "ticks_per_case": ticks,
               "failure_window_ticks": list(window),
               "reference_wrench": {"generation": "once_at_controller_initialization",
                                     "source": "static least-squares equilibrium lambda mapped at reference contact points",
                                     "value": controller.reference_wrench.tolist()},
               "results": results, "case_gates": case_gates,
               "baseline_pass": bool(case_gates.get("baseline")) and all(case_gates["baseline"].values()),
               "phase_gate_changed": False}
    write_json(output / "summary.json", summary)
    output_files = [path for path in output.rglob("*") if path.is_file()]
    manifest = {"schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "numpy": np.__version__, "mujoco": mujoco.__version__,
        "hardware_data": False, "config": str(config_path.relative_to(ROOT)),
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "model_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "qp_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in qp_inputs},
        "validator": str(Path(__file__).resolve().relative_to(ROOT)),
        "validator_sha256": sha256(Path(__file__).resolve()),
        "source_dependencies": {str(path.relative_to(ROOT)): sha256(path) for path in (
            ROOT / "tools/experiments/validate_weighted_wbc_tasks.py",
            ROOT / "tools/experiments/validate_weighted_wbc_qp.py",
            ROOT / "tools/experiments/validate_mujoco_weighted_wbc_model.py",
        )},
        "outputs": {str(path.relative_to(output)): sha256(path) for path in output_files}}
    write_json(output / "manifest.json", manifest)
    print(json.dumps({"baseline_pass": summary["baseline_pass"],
                      "first_failure_ticks": {name: value["first_solver_failure_tick"] for name, value in results.items()}}, indent=2))
    return 0


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
