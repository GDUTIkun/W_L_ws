#!/usr/bin/env python3
"""Run the frozen Phase 28 diagnostic-continuation attribution matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase28_drift_attribution_v1.json"
DEFAULT_RUNNER = ROOT / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/weighted_wbc_loop"

WALL_CLOCK_COLUMNS = {"core_step_ns", "phase27_solve_s", "phase27_wbc_total_s"}
BODY_MASS_KG = 5.7482000000000006
COM_B = np.array([-0.011186360321930223, 0.00010351112192572815, -0.050073820064730427])
INERTIA_COM_B = np.array([
    [0.14032539391425894, -0.00027482615417932365, -0.0055301280789718825],
    [-0.00027482615417932365, 0.075346414957965305, 0.00019749711948992591],
    [-0.0055301280789718825, 0.00019749711948992591, 0.094068657813524068],
])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def vector(values: list[float]) -> str:
    return ",".join(format(float(value), ".17g") for value in values)


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def rpy(row: dict[str, str]) -> np.ndarray:
    quaternion = [float(row[f"quat{i}"]) for i in range(4)]
    return Rotation.from_quat(
        [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    ).as_euler("xyz")


def references(control: list[dict[str, str]]) -> list[np.ndarray]:
    position = np.array([float(control[0]["base_p0"]), float(control[0]["base_p1"])])
    yaw = rpy(control[0])[2]
    result = [np.array([position[0], position[1], yaw])]
    for row in control[1:]:
        speed = float(row["phase27_v_ref"])
        position += 0.01 * speed * np.array([math.cos(yaw), math.sin(yaw)])
        yaw += 0.01 * float(row["phase27_yaw_rate_ref"])
        result.append(np.array([position[0], position[1], yaw]))
    return result


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def nmpc_acceleration(row: dict[str, str]) -> np.ndarray:
    quaternion = [float(row[f"quat{i}"]) for i in range(4)]
    rotation = Rotation.from_quat(
        [quaternion[1], quaternion[2], quaternion[3], quaternion[0]]
    ).as_matrix()
    angular_velocity = np.array([float(row[f"base_w{i}"]) for i in range(3)])
    wrench = np.array([float(row[f"phase27_requested_wrench{i}"]) for i in range(12)])
    left_origin = np.array([
        float(row["phase27_xi_left"]), 0.21229919000000008, -0.26587051502608744
    ])
    right_origin = np.array([
        float(row["phase27_xi_right"]), -0.21230080999999992, -0.26574406892872388
    ])
    force_b = wrench[:3] + wrench[6:9]
    moment_b = (
        wrench[3:6] + np.cross(left_origin, wrench[:3])
        + wrench[9:12] + np.cross(right_origin, wrench[6:9])
    )
    force_n = rotation @ force_b
    com_n = rotation @ COM_B
    inertia_n = rotation @ INERTIA_COM_B @ rotation.T
    moment_com_n = rotation @ moment_b - np.cross(com_n, force_n)
    angular = np.linalg.solve(
        inertia_n,
        moment_com_n - np.cross(angular_velocity, inertia_n @ angular_velocity),
    )
    com_acceleration = force_n / BODY_MASS_KG + np.array([0.0, 0.0, -9.81])
    linear = (
        com_acceleration - np.cross(angular, com_n)
        - np.cross(angular_velocity, np.cross(angular_velocity, com_n))
    )
    return np.concatenate((linear, angular))


def rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def nominal_outside(
    row: dict[str, str], attitude: np.ndarray, reference: np.ndarray,
    envelope: dict[str, float],
) -> bool:
    return (
        abs(float(row["base_p0"]) - reference[0]) > envelope["maximum_abs_x_tracking_error_m"]
        or abs(float(row["base_p1"]) - reference[1]) > envelope["maximum_abs_y_tracking_error_m"]
        or abs(float(row["base_p2"]) - 0.31543998403249462) > envelope["maximum_height_error_m"]
        or abs(attitude[0]) > envelope["maximum_abs_roll_pitch_rad"]
        or abs(attitude[1]) > envelope["maximum_abs_roll_pitch_rad"]
        or abs(wrap(attitude[2] - reference[2])) > envelope["maximum_abs_yaw_tracking_error_rad"]
    )


def shared_prefix_error(
    current: list[dict[str, str]], baseline: list[dict[str, str]], stop: int,
) -> tuple[float, list[str]]:
    maximum = 0.0
    mismatched: set[str] = set()
    common = set(current[0]).intersection(baseline[0]) - WALL_CLOCK_COLUMNS
    for left, right in zip(current[:stop], baseline[:stop]):
        for name in common:
            if name in {"scenario", "episode"}:
                if left[name] != right[name]:
                    mismatched.add(name)
                continue
            error = abs(float(left[name]) - float(right[name]))
            maximum = max(maximum, error)
            if error > 1.0e-12:
                mismatched.add(name)
    return maximum, sorted(mismatched)


def classify(
    recovered: bool | None, corrective: bool | None,
    realization: bool | None, upper_match: bool | None,
    plant_match: bool | None,
) -> str:
    if recovered:
        return "A_threshold_only"
    if recovered is None:
        return "unresolved"
    if not corrective:
        return "unresolved" if corrective is None else "B_nmpc_corrective_failure"
    if not realization:
        return "unresolved" if realization is None else "C_wbc_realization_or_resource"
    if upper_match is None:
        return "unresolved"
    if not upper_match:
        return "D_model_to_plant_mismatch"
    if plant_match is None:
        return "unresolved"
    if not plant_match:
        return "D_model_to_plant_mismatch"
    return "E_fast_stabilization_or_bandwidth_gap"


def synthetic_self_test() -> None:
    assert classify(True, False, False, False, False) == "A_threshold_only"
    assert classify(False, False, True, True, True) == "B_nmpc_corrective_failure"
    assert classify(False, True, False, True, True) == "C_wbc_realization_or_resource"
    assert classify(False, True, True, False, True) == "D_model_to_plant_mismatch"
    assert classify(False, True, True, True, False) == "D_model_to_plant_mismatch"
    assert classify(False, True, True, True, True) == "E_fast_stabilization_or_bandwidth_gap"
    assert classify(False, None, True, True, True) == "unresolved"


def evaluate_case(
    case: dict[str, Any], control: list[dict[str, str]],
    plant: list[dict[str, str]], baseline: list[dict[str, str]],
    config: dict[str, Any],
) -> dict[str, Any]:
    attitude = [rpy(row) for row in control]
    reference = references(control)
    outside = [
        nominal_outside(row, angles, target, config["nominal_envelope"])
        for row, angles, target in zip(control, attitude, reference)
    ]
    nominal_tick = next(index for index, value in enumerate(outside) if value)
    diagnostic_tick = next(
        (index for index, row in enumerate(control) if row["status"] != "0"),
        len(control),
    )
    recovery_hold = int(config["gates"]["recovery_hold_ticks"])
    recovered = any(
        not any(outside[start:start + recovery_hold])
        for start in range(nominal_tick + 1, len(outside) - recovery_hold + 1)
    )
    plant_by_tick: dict[int, list[dict[str, str]]] = {}
    for row in plant:
        plant_by_tick.setdefault(int(row["control_tick"]), []).append(row)
    first = max(2, nominal_tick - int(config["gates"]["analysis_tail_ticks"]))
    last = min(diagnostic_tick, len(control) - 1)
    indices = list(range(first, last))
    if not indices:
        raise RuntimeError(f"{case['id']} has no attribution window")
    nmpc = np.array([nmpc_acceleration(control[index]) for index in indices])
    wbc = np.array([
        [float(control[index][f"z{axis}"]) for axis in range(6)]
        for index in indices
    ])
    direct = np.array([
        np.mean([
            [float(row[f"base_control_linear_acceleration_n{axis}"]) for axis in range(3)]
            + [float(row[f"base_control_angular_acceleration_n{axis}"]) for axis in range(3)]
            for row in plant_by_tick[index]
        ], axis=0)
        for index in indices
    ])
    finite_difference = np.array([
        [
            (float(control[index + 1][f"base_v{axis}"])
             - float(control[index - 1][f"base_v{axis}"])) / 0.02
            for axis in range(3)
        ] + [
            (float(control[index + 1][f"base_w{axis}"])
             - float(control[index - 1][f"base_w{axis}"])) / 0.02
            for axis in range(3)
        ]
        for index in indices
    ])
    direct_for_finite_difference = np.array([
        np.mean([
            [float(row[f"base_control_linear_acceleration_n{axis}"]) for axis in range(3)]
            + [float(row[f"base_control_angular_acceleration_n{axis}"]) for axis in range(3)]
            for tick in (index - 1, index) for row in plant_by_tick[tick]
        ], axis=0)
        for index in indices
    ])
    force_residual = max(
        abs(float(control[index][f"phase27_wrench_residual{side * 6 + axis}"]))
        for index in indices for side in range(2) for axis in range(3)
    )
    moment_residual = max(
        abs(float(control[index][f"phase27_wrench_residual{side * 6 + axis}"]))
        for index in indices for side in range(2) for axis in range(3, 6)
    )
    gates = config["gates"]
    if case["id"].startswith("T0"):
        score = np.array([
            -attitude[index][1] * nmpc[row, 4]
            for row, index in enumerate(indices)
        ])
    else:
        score = np.array([
            -(float(control[index]["base_v0"]) - float(control[index]["phase27_v_ref"]))
            * nmpc[row, 0]
            for row, index in enumerate(indices)
        ])
    corrective_fraction = float(np.mean(score > gates["minimum_corrective_score"]))
    corrective = corrective_fraction >= 0.7
    realization = (
        force_residual <= gates["maximum_force_wrench_residual_n"]
        and moment_residual <= gates["maximum_moment_wrench_residual_nm"]
    )
    upper_linear = rms(wbc[:, :3] - nmpc[:, :3])
    upper_angular = rms(wbc[:, 3:] - nmpc[:, 3:])
    plant_linear = rms(direct[:, :3] - wbc[:, :3])
    plant_angular = rms(direct[:, 3:] - wbc[:, 3:])
    direct_fd_linear = rms(direct_for_finite_difference[:, :3] - finite_difference[:, :3])
    direct_fd_angular = rms(direct_for_finite_difference[:, 3:] - finite_difference[:, 3:])
    acceleration_oracle = (
        direct_fd_linear <= gates["maximum_direct_vs_fd_linear_acceleration_rms_m_s2"]
        and direct_fd_angular <= gates["maximum_direct_vs_fd_angular_acceleration_rms_rad_s2"]
    )
    upper_match = (
        upper_linear <= gates["maximum_upper_linear_acceleration_rms_m_s2"]
        and upper_angular <= gates["maximum_upper_angular_acceleration_rms_rad_s2"]
    )
    plant_match = acceleration_oracle and (
        plant_linear <= gates["maximum_plant_linear_acceleration_rms_m_s2"]
        and plant_angular <= gates["maximum_plant_angular_acceleration_rms_rad_s2"]
    )
    attribution = classify(recovered, corrective, realization, upper_match, plant_match)
    primary_case = case["id"].startswith(("T0", "T1"))
    resource_rows = [
        row for index in indices for row in plant_by_tick[index]
    ]
    prefix_error, prefix_mismatches = shared_prefix_error(
        control, baseline, case["phase27_first_failure_tick"]
    )
    stop_row = control[diagnostic_tick] if diagnostic_tick < len(control) else None
    maximum_excursion = {
        "x_tracking_error_m": max(abs(float(row["base_p0"]) - target[0]) for row, target in zip(control, reference)),
        "pitch_rad": max(abs(value[1]) for value in attitude),
        "linear_speed_m_s": max(math.sqrt(sum(float(row[f"base_v{i}"]) ** 2 for i in range(3))) for row in control),
        "angular_speed_rad_s": max(math.sqrt(sum(float(row[f"base_w{i}"]) ** 2 for i in range(3))) for row in control),
    }
    return {
        "id": case["id"],
        "phase27_expected_first_failure_tick": case["phase27_first_failure_tick"],
        "nominal_failure_tick": nominal_tick,
        "nominal_failure_time_s": 0.01 * nominal_tick,
        "baseline_prefix_maximum_error": prefix_error,
        "baseline_prefix_mismatched_columns": prefix_mismatches,
        "diagnostic_stop_tick": diagnostic_tick if stop_row else None,
        "diagnostic_stop_time_s": 0.01 * diagnostic_tick if stop_row else None,
        "diagnostic_stop": None if stop_row is None else {
            "status": int(stop_row["status"]),
            "weighted_status": int(stop_row["weighted_status"]),
            "model_status": int(stop_row["model_status"]),
            "solver_status": int(stop_row["solver_status"]),
            "nmpc_status": int(stop_row["phase27_status"]),
            "nmpc_projected_stationarity": float(stop_row["phase27_projected_stationarity"]),
        },
        "trajectory_class": "recovery" if recovered else (
            "divergence" if stop_row is not None else "drift"
        ),
        "maximum_excursion": maximum_excursion,
        "window": {"first_tick": first, "last_tick_exclusive": last},
        "corrective": {
            "pass": corrective,
            "restorative_fraction": corrective_fraction,
            "median_score": float(np.median(score)),
        },
        "wbc_realization": {
            "pass": realization,
            "maximum_force_residual_n": force_residual,
            "maximum_moment_residual_nm": moment_residual,
            "maximum_hard_violation": max(float(control[index]["hard"]) for index in indices),
            "maximum_abs_torque_nm": max(abs(float(control[index][f"command_tau{joint}"])) for index in indices for joint in range(6)),
            "minimum_torque_margin_nm": min(
                config["torque_limit_nm"][joint]
                - abs(float(control[index][f"command_tau{joint}"]))
                for index in indices for joint in range(6)
            ),
            "maximum_normalized_slack": max(
                float(control[index]["max_normalized_slack"])
                for index in indices
            ),
            "minimum_wheel_normal_load_n": min(
                float(row[name]) for row in resource_rows
                for name in ("left_normal_load_n", "right_normal_load_n")
            ),
            "maximum_penetration_m": max(
                float(row["penetration_m"]) for row in resource_rows
            ),
            "maximum_closure_residual_m": max(
                float(row["closure_residual_m"]) for row in resource_rows
            ),
        },
        "acceleration": {
            "oracle_pass": acceleration_oracle,
            "upper_match": upper_match,
            "plant_match": plant_match,
            "upper_linear_rms_m_s2": upper_linear,
            "upper_angular_rms_rad_s2": upper_angular,
            "plant_linear_rms_m_s2": plant_linear,
            "plant_angular_rms_rad_s2": plant_angular,
            "direct_vs_fd_linear_rms_m_s2": direct_fd_linear,
            "direct_vs_fd_angular_rms_rad_s2": direct_fd_angular,
            "tail_nmpc_mean": np.mean(nmpc[-10:], axis=0).tolist(),
            "tail_wbc_mean": np.mean(wbc[-10:], axis=0).tolist(),
            "tail_plant_mean": np.mean(direct[-10:], axis=0).tolist(),
        },
        "attribution_path_result": attribution,
        "primary_attribution": attribution if primary_case else None,
        "pass": (
            nominal_tick == case["phase27_first_failure_tick"]
            and prefix_error <= 1.0e-12 and not prefix_mismatches
            and acceleration_oracle and attribution != "unresolved"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    synthetic_self_test()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    source = ROOT / config["source_phase27_formal"]
    if sha256(ROOT / config["source_phase27_config"]) != config["source_phase27_config_sha256"]:
        raise RuntimeError("Phase 27 config authority hash mismatch")
    if args.output_dir.exists():
        raise RuntimeError("refusing to overwrite output directory")
    args.output_dir.mkdir(parents=True)
    results = []
    outputs: dict[str, str] = {}
    for case in config["cases"]:
        control_path = args.output_dir / f"{case['id']}_control.csv"
        plant_path = args.output_dir / f"{case['id']}_plant.csv"
        command = [
            str(args.runner), "--model", str(ROOT / config["scene"]),
            "--control-output", str(control_path), "--plant-output", str(plant_path),
            "--scenario", "hold", "--controller-mode", config["controller_mode"],
            "--phase27-reference-profile", case["reference_profile"],
            "--phase28-diagnostic-continuation", "true",
            "--episodes", "1", "--ticks", str(case["ticks"]),
            "--equilibrium", vector(config["equilibrium"]),
            "--torque-limit", vector(config["torque_limit_nm"]),
        ]
        subprocess.run(command, cwd=ROOT, check=True)
        result = evaluate_case(
            case, rows(control_path), rows(plant_path),
            rows(source / f"{case['id']}_control.csv"), config,
        )
        report = args.output_dir / f"{case['id']}_attribution.json"
        report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        results.append(result)
        for path in (control_path, plant_path, report):
            outputs[path.name] = sha256(path)
    summary = {
        "schema_version": 1,
        "phase": 28,
        "profile": config["profile"],
        "cases": results,
        "t0_primary_attribution": results[0]["primary_attribution"],
        "t1_primary_attribution": results[1]["primary_attribution"],
        "t2_mechanism_consistency_with_t1": {
            result["id"]: result["attribution_path_result"]
            == results[1]["primary_attribution"] for result in results[2:]
        },
        "t2_symmetry": "consistent" if all(
            result["attribution_path_result"] == results[1]["primary_attribution"]
            for result in results[2:]
        ) else "not_consistent",
        "pass": all(result["pass"] for result in results),
    }
    summary_path = args.output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    outputs[summary_path.name] = sha256(summary_path)
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "dependencies": {
            name: importlib.metadata.version(name) for name in ("mujoco", "numpy", "scipy")
        },
        "config": relative(args.config),
        "config_sha256": sha256(args.config),
        "runner": relative(args.runner),
        "runner_sha256": sha256(args.runner),
        "source_phase27_formal": config["source_phase27_formal"],
        "source_run": "phase27-minimal-formal-v2",
        "inputs": {
            relative(path): sha256(path) for path in (
                args.config, ROOT / config["source_phase27_config"],
                ROOT / config["scene"], Path(__file__),
                ROOT / "ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp",
                ROOT / "ros_ws/src/wheel_leg_core/src/wheel_aware_nmpc_model.cpp",
            )
        },
        "outputs": outputs,
        "hardware_data": False,
    }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "pass": summary["pass"],
        "t0": summary["t0_primary_attribution"],
        "t1": summary["t1_primary_attribution"],
        "t2_symmetry": summary["t2_symmetry"],
    }, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
