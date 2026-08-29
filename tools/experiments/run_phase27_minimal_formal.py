#!/usr/bin/env python3
"""Run and evaluate the frozen Phase 27 T0--T3 Minimal controller matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase27_minimal_formal_v1.json"
DEFAULT_RUNNER = ROOT / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/weighted_wbc_loop"
GENERATED = ROOT / "ros_ws/src/wheel_leg_core/acados_generated/phase27_wheel_aware_nmpc_v2"
WALL_CLOCK_COLUMNS = {"core_step_ns", "phase27_solve_s", "phase27_wbc_total_s"}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def vector(values: list[float]) -> str:
    return ",".join(format(float(value), ".17g") for value in values)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite_rows(rows: list[dict[str, str]], text: set[str]) -> bool:
    return all(
        value in ("True", "False") or math.isfinite(float(value))
        for row in rows for key, value in row.items() if key not in text
    )


def quaternion_rpy(row: dict[str, str]) -> tuple[float, float, float]:
    w, x, y, z = (float(row[f"quat{i}"]) for i in range(4))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(math.ceil(fraction * len(ordered))) - 1)]


def classify_failure(flags: dict[str, bool]) -> str:
    for name in (
        "nonfinite", "contact_or_plant", "deadline", "nmpc", "interface",
        "wbc", "safety_envelope", "planner",
    ):
        if flags.get(name, False):
            return name
    return "none"


def synthetic_self_test() -> None:
    assert classify_failure({}) == "none"
    names = (
        "nonfinite", "contact_or_plant", "deadline", "nmpc", "interface",
        "wbc", "safety_envelope", "planner",
    )
    for name in names:
        assert classify_failure({name: True}) == name
    assert classify_failure({"nmpc": True, "wbc": True}) == "nmpc"
    assert 0.01 <= 0.01  # threshold equality is inclusive
    rows = [{"tick": "0"}, {"tick": "1"}, {"tick": "2"}]
    first = next(row for row in rows if int(row["tick"]) >= 1)
    assert first["tick"] == "1"


def integrate_reference(control: list[dict[str, str]]) -> list[tuple[float, float, float]]:
    x = float(control[0]["base_p0"])
    y = float(control[0]["base_p1"])
    yaw = quaternion_rpy(control[0])[2]
    result = [(x, y, yaw)]
    for row in control[1:]:
        dt = 0.01
        speed = float(row["phase27_v_ref"])
        x += dt * speed * math.cos(yaw)
        y += dt * speed * math.sin(yaw)
        yaw += dt * float(row["phase27_yaw_rate_ref"])
        result.append((x, y, yaw))
    return result


def evaluate_case(
    case: dict[str, Any], control: list[dict[str, str]],
    plant: list[dict[str, str]], config: dict[str, Any], cone: list[list[float]],
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    hard = config["hard_gates"]
    performance = config["performance_gates"]
    ticks = int(case["ticks"])
    refs = integrate_reference(control)
    rpy = [quaternion_rpy(row) for row in control]
    x_error = [float(row["base_p0"]) - ref[0] for row, ref in zip(control, refs)]
    y_error = [float(row["base_p1"]) - ref[1] for row, ref in zip(control, refs)]
    yaw_error = [wrap(attitude[2] - ref[2]) for attitude, ref in zip(rpy, refs)]
    xi_delta = [
        0.5 * (float(row["phase27_xi_right"]) - float(row["phase27_xi_left"]))
        for row in control
    ]
    xi_common = [
        0.5 * (float(row["phase27_xi_right"]) + float(row["phase27_xi_left"]))
        for row in control
    ]
    velocity_error = [
        float(row["base_v0"]) - float(row["phase27_v_ref"]) for row in control
    ]
    yaw_rate_error = [
        float(row["base_w2"]) - float(row["phase27_yaw_rate_ref"]) for row in control
    ]
    combined = [float(row["phase27_wbc_total_s"]) for row in control]
    nmpc_updates = [row for row in control if row["phase27_update"] == "1"]
    force_residual = max(
        abs(float(row[f"phase27_wrench_residual{side * 6 + axis}"]))
        for row in control for side in range(2) for axis in range(3)
    )
    moment_residual = max(
        abs(float(row[f"phase27_wrench_residual{side * 6 + axis}"]))
        for row in control for side in range(2) for axis in range(3, 6)
    )
    force_rate = 0.0
    moment_rate = 0.0
    for before, after in zip(control, control[1:]):
        for side in range(2):
            for axis in range(6):
                rate = abs(float(after[f"phase27_requested_wrench{6 * side + axis}"]) -
                           float(before[f"phase27_requested_wrench{6 * side + axis}"])) / 0.01
                if axis < 3:
                    force_rate = max(force_rate, rate)
                else:
                    moment_rate = max(moment_rate, rate)
    cone_margin = math.inf
    for row in control:
        for side in range(2):
            wrench = [float(row[f"z{18 + 6 * side + i}"]) for i in range(6)]
            for cone_row in cone:
                cone_margin = min(cone_margin, -sum(a * b for a, b in zip(cone_row, wrench)))
    torque_violation = max(
        max(0.0, abs(float(row[f"command_tau{joint}"])) - config["torque_limit_nm"][joint])
        for row in control for joint in range(6)
    )
    contact_fraction = sum(
        row["contact_left"] == "2" and row["contact_right"] == "2" for row in control
    ) / max(1, len(control))
    feasibility = max(
        max(float(row[name]) for name in (
            "phase27_dynamics", "phase27_inequality", "phase27_complementarity",
            "phase27_first_step_defect",
        )) for row in nmpc_updates
    )
    metrics = {
        "control_rows": len(control),
        "plant_rows": len(plant),
        "maximum_abs_x_tracking_error_m": max(map(abs, x_error)),
        "maximum_abs_y_tracking_error_m": max(map(abs, y_error)),
        "maximum_height_error_m": max(abs(float(row["base_p2"]) - 0.31543998403249462) for row in control),
        "maximum_abs_roll_rad": max(abs(value[0]) for value in rpy),
        "maximum_abs_pitch_rad": max(abs(value[1]) for value in rpy),
        "maximum_abs_yaw_tracking_error_rad": max(map(abs, yaw_error)),
        "minimum_wheel_normal_load_n": min(float(row[name]) for row in plant for name in ("left_normal_load_n", "right_normal_load_n")),
        "bilateral_contact_fraction": contact_fraction,
        "maximum_combined_time_s": max(combined),
        "p99_combined_time_s": percentile(combined, 0.99),
        "maximum_hard_violation": max(float(row["hard"]) for row in control),
        "maximum_nmpc_feasibility_residual": feasibility,
        "maximum_nmpc_independent_dynamics_defect": max(float(row["phase27_maximum_dynamics_defect"]) for row in nmpc_updates),
        "maximum_nmpc_bound_violation": max(max(float(row["phase27_input_bound_violation"]), float(row["phase27_state_bound_violation"])) for row in nmpc_updates),
        "maximum_nmpc_projected_stationarity": max(float(row["phase27_projected_stationarity"]) for row in nmpc_updates),
        "maximum_torque_limit_violation_nm": torque_violation,
        "minimum_contact_cone_margin": cone_margin,
        "maximum_zoh_difference": max(float(row["zoh_diff"]) for row in control),
        "velocity_rmse_m_s": math.sqrt(sum(value * value for value in velocity_error) / len(velocity_error)),
        "final_velocity_error_m_s": abs(velocity_error[-1]),
        "yaw_rate_rmse_rad_s": math.sqrt(sum(value * value for value in yaw_rate_error) / len(yaw_rate_error)),
        "maximum_force_wrench_residual_n": force_residual,
        "maximum_moment_wrench_residual_nm": moment_residual,
        "maximum_force_wrench_rate_n_s": force_rate,
        "maximum_moment_wrench_rate_nm_s": moment_rate,
        "maximum_normalized_slack": max(float(row["max_normalized_slack"]) for row in control),
        "initial_xi_delta_m": xi_delta[0],
        "maximum_abs_xi_delta_m": max(map(abs, xi_delta)),
        "tail_abs_xi_delta_m": max(map(abs, xi_delta[-int(performance["t3_tail_window_ticks"]):])),
        "maximum_abs_xi_common_tracking_error_m": max(abs(value - float(row["planner_xi_c"])) for value, row in zip(xi_common, control)),
        "final_linear_speed_m_s": math.sqrt(sum(float(control[-1][name]) ** 2 for name in ("base_v0", "base_v1", "base_v2"))),
        "final_angular_speed_rad_s": math.sqrt(sum(float(control[-1][name]) ** 2 for name in ("base_w0", "base_w1", "base_w2"))),
    }
    hard_checks = {
        "completed": len(control) == ticks and len(plant) == ticks * config["schedule"]["physics_substeps_per_control"],
        "finite": finite_rows(control, {"scenario", "episode"}) and finite_rows(plant, {"scenario", "episode", "disturbance"}),
        "controller_ok": all(row["status"] == "0" and row["latch"] == "0" for row in control),
        "wbc_ok": all(row["weighted_status"] == "0" and row["model_status"] == "0" and row["solver_status"] == "0" for row in control),
        "nmpc_ok": all(row["phase27_status"] == "0" and row["phase27_acados_status"] == "0" for row in control),
        "schedule": all((row["phase27_update"] == "1") == (int(row["tick"]) % 2 == 0) and int(row["phase27_age"]) == int(row["tick"]) % 2 for row in control),
        "deadline": metrics["maximum_combined_time_s"] <= hard["maximum_combined_time_s"],
        "hard_violation": metrics["maximum_hard_violation"] <= hard["maximum_hard_violation"],
        "nmpc_feasibility": feasibility <= hard["maximum_nmpc_feasibility_residual"],
        "nmpc_dynamics": metrics["maximum_nmpc_independent_dynamics_defect"] <= hard["maximum_nmpc_independent_dynamics_defect"],
        "nmpc_bounds": metrics["maximum_nmpc_bound_violation"] <= hard["maximum_nmpc_bound_violation"],
        "nmpc_stationarity": metrics["maximum_nmpc_projected_stationarity"] <= hard["maximum_nmpc_projected_stationarity"],
        "contact": contact_fraction >= hard["minimum_bilateral_contact_fraction"],
        "normal_load": metrics["minimum_wheel_normal_load_n"] >= hard["minimum_wheel_normal_load_n"],
        "x": metrics["maximum_abs_x_tracking_error_m"] <= hard["maximum_abs_x_tracking_error_m"],
        "y": metrics["maximum_abs_y_tracking_error_m"] <= hard["maximum_abs_y_tracking_error_m"],
        "height": metrics["maximum_height_error_m"] <= hard["maximum_height_error_m"],
        "roll_pitch": max(metrics["maximum_abs_roll_rad"], metrics["maximum_abs_pitch_rad"]) <= hard["maximum_abs_roll_pitch_rad"],
        "yaw": metrics["maximum_abs_yaw_tracking_error_rad"] <= hard["maximum_abs_yaw_tracking_error_rad"],
        "torque": torque_violation <= hard["maximum_torque_limit_violation_nm"],
        "zoh": metrics["maximum_zoh_difference"] <= hard["maximum_zoh_difference"],
    }
    performance_checks = {
        "wrench_force_fidelity": force_residual <= performance["maximum_force_wrench_residual_n"],
        "wrench_moment_fidelity": moment_residual <= performance["maximum_moment_wrench_residual_nm"],
        "wrench_force_rate": force_rate <= performance["maximum_force_wrench_rate_n_s"],
        "wrench_moment_rate": moment_rate <= performance["maximum_moment_wrench_rate_nm_s"],
        "slack": metrics["maximum_normalized_slack"] <= performance["maximum_normalized_slack"],
    }
    if case["id"].startswith("T0"):
        performance_checks.update({
            "static_linear_speed": metrics["final_linear_speed_m_s"] <= performance["maximum_static_linear_speed_m_s"],
            "static_angular_speed": metrics["final_angular_speed_rad_s"] <= performance["maximum_static_angular_speed_rad_s"],
        })
    if case["id"].startswith("T1"):
        performance_checks.update({
            "velocity_rmse": metrics["velocity_rmse_m_s"] <= performance["maximum_velocity_rmse_m_s"],
            "final_velocity": metrics["final_velocity_error_m_s"] <= performance["maximum_final_velocity_error_m_s"],
        })
    if case["id"].startswith("T2"):
        performance_checks["yaw_rate_rmse"] = metrics["yaw_rate_rmse_rad_s"] <= performance["maximum_yaw_rate_rmse_rad_s"]
    if case["id"].startswith("T3"):
        performance_checks.update({
            "initial_xi_delta": abs(metrics["initial_xi_delta_m"] - case["target_initial_xi_delta_m"]) <= performance["initial_xi_delta_tolerance_m"],
            "xi_delta_recovery": metrics["tail_abs_xi_delta_m"] <= performance["maximum_t3_tail_abs_xi_delta_m"],
        })
    performance_checks["hard_gate_precondition"] = all(hard_checks.values())

    first_failure = None
    for index, row in enumerate(control):
        attitude = rpy[index]
        interface_bad = any(
            abs(float(row[f"phase27_wrench_residual{side * 6 + axis}"])) >
            (performance["maximum_force_wrench_residual_n"] if axis < 3 else performance["maximum_moment_wrench_residual_nm"])
            for side in range(2) for axis in range(6)
        )
        envelope_bad = (
            abs(x_error[index]) > hard["maximum_abs_x_tracking_error_m"] or
            abs(y_error[index]) > hard["maximum_abs_y_tracking_error_m"] or
            abs(float(row["base_p2"]) - 0.31543998403249462) > hard["maximum_height_error_m"] or
            abs(attitude[0]) > hard["maximum_abs_roll_pitch_rad"] or
            abs(attitude[1]) > hard["maximum_abs_roll_pitch_rad"] or
            abs(yaw_error[index]) > hard["maximum_abs_yaw_tracking_error_rad"]
        )
        flags = {
            "nonfinite": not finite_rows([row], {"scenario", "episode"}),
            "contact_or_plant": row["contact_left"] != "2" or row["contact_right"] != "2",
            "deadline": float(row["phase27_wbc_total_s"]) > hard["maximum_combined_time_s"],
            "nmpc": row["phase27_status"] != "0" or row["phase27_acados_status"] != "0",
            "interface": interface_bad,
            "wbc": row["weighted_status"] != "0" and not envelope_bad,
            "safety_envelope": envelope_bad,
            "planner": not (-0.3303432354 <= float(row["planner_xi_c"]) <= 0.1659029424),
        }
        layer = classify_failure(flags)
        if layer != "none" or row["status"] != "0":
            first_failure = {
                "case": case["id"], "tick": int(row["tick"]),
                "time_s": float(row["pre_step_plant_time_s"]), "layer": layer,
                "flags": flags,
                "control": {key: value for key, value in row.items() if key not in WALL_CLOCK_COLUMNS},
                "plant": [item for item in plant if item["control_tick"] == row["tick"]],
            }
            break
    passed = all(hard_checks.values()) and all(performance_checks.values())
    return {
        "id": case["id"], "pass": passed,
        "hard_pass": all(hard_checks.values()), "hard_checks": hard_checks,
        "performance_pass": all(performance_checks.values()),
        "performance_checks": performance_checks, "metrics": metrics,
        "first_failure_layer": "none" if first_failure is None else first_failure["layer"],
    }, first_failure


def run(arguments: argparse.Namespace) -> int:
    config_path = arguments.config.resolve()
    runner = arguments.runner.resolve()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    runtime_profile = json.loads((ROOT / config["source_profiles"]["runtime_model"]).read_text(encoding="utf-8"))
    cone = runtime_profile["contact"]["h_cone_37x6"]
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for case in config["cases"]:
        control = output / f"{case['id']}_control.csv"
        plant = output / f"{case['id']}_plant.csv"
        command = [
            str(runner), "--model", str(ROOT / config["scene"]),
            "--control-output", str(control), "--plant-output", str(plant),
            "--scenario", "hold", "--controller-mode", config["controller_mode"],
            "--phase27-reference-profile", case["reference_profile"],
            "--episodes", "1", "--ticks", str(case["ticks"]),
            "--equilibrium", vector(case.get("equilibrium", config["equilibrium"])),
            "--torque-limit", vector(config["torque_limit_nm"]),
        ]
        subprocess.run(command, check=True)
        result, failure = evaluate_case(case, read_rows(control), read_rows(plant), config, cone)
        results.append(result)
        if failure is not None:
            (output / f"{case['id']}_first_failure.json").write_text(
                json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
    summary = {
        "schema_version": 1, "phase": 27, "profile": config["profile"],
        "pass": all(item["pass"] for item in results), "cases": results,
        "conclusion": "Minimal PASS" if all(item["pass"] for item in results) else "Minimal FAIL",
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    source_paths = [
        config_path, Path(__file__).resolve(), ROOT / config["scene"],
        *(ROOT / value for value in config["source_profiles"].values()),
        ROOT / "ros_ws/src/wheel_leg_core/src/controller_core.cpp",
        ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp",
    ]
    manifest = {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "hardware_data": False, "python": platform.python_version(),
        "dependencies": {name: importlib.metadata.version(name) for name in ("mujoco", "numpy", "scipy", "casadi")},
        "config": relative(config_path), "config_sha256": sha256(config_path),
        "runner": relative(runner), "runner_sha256": sha256(runner),
        "inputs": {relative(path): sha256(path) for path in source_paths},
        "generated_inputs": {relative(path): sha256(path) for path in sorted(GENERATED.rglob("*")) if path.is_file()},
        "outputs": {path.name: sha256(path) for path in sorted(output.iterdir()) if path.is_file()},
        "command": [str(item) for item in sys.argv], "seed": config["seed"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"pass": summary["pass"], "conclusion": summary["conclusion"], "cases": [{item["id"]: item["pass"]} for item in results]}))
    return 0 if summary["pass"] else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--self-test", action="store_true")
    arguments = parser.parse_args()
    synthetic_self_test()
    if arguments.self_test:
        print("phase27 formal evaluator synthetic oracle: PASS")
        return 0
    if arguments.output_dir is None:
        raise RuntimeError("--output-dir is required unless --self-test is used")
    return run(arguments)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
