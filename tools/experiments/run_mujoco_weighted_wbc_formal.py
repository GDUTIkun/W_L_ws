#!/usr/bin/env python3
"""Run and evaluate a frozen weighted-WBC C++ formal matrix.

Drives the `weighted_wbc_loop` runner case by case with a frozen config,
evaluates every control tick
(Core/Adapter status, solver/model/task diagnostics, deadline) and every
physics substep (plant truth), and writes summary.json and manifest.json.

Refuses to write into a non-empty output directory; never overwrites runner
CSVs (the runner itself also refuses existing paths).
"""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_weighted_wbc_formal_v1.json"
DEFAULT_RUNNER = (
    ROOT / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/weighted_wbc_loop"
)

LEG_JOINTS = (0, 1, 3, 4)
CONTROL_EXACT_IGNORE = (
    "core_step_ns",
    "nmpc_preparation_s",
    "nmpc_feedback_s",
    "nmpc_wbc_total_s",
)  # wall-clock only


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def root_relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def merge_config(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if key == "base_config":
            continue
        if (key != "solver" and isinstance(value, dict) and
                isinstance(merged.get(key), dict)):
            merged[key] = merge_config(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_config(path: Path, seen: tuple[Path, ...] = ()) -> tuple[dict[str, Any], list[Path]]:
    resolved = path.resolve()
    if resolved in seen:
        raise RuntimeError(f"Config inheritance cycle: {resolved}")
    current = json.loads(resolved.read_text(encoding="utf-8"))
    base_value = current.get("base_config")
    if base_value is None:
        return current, [resolved]
    base_path = Path(base_value)
    if not base_path.is_absolute():
        base_path = (ROOT / base_path).resolve()
    base, chain = load_config(base_path, (*seen, resolved))
    return merge_config(base, current), [*chain, resolved]


def vector(values: list[float]) -> str:
    return ",".join(format(float(value), ".17g") for value in values)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def finite_row(row: dict[str, str], skip: tuple[str, ...] = ()) -> bool:
    return all(
        value in ("True", "False") or math.isfinite(float(value))
        for key, value in row.items()
        if key not in skip
    )


def quaternion_log(q: list[float]) -> list[float]:
    """World-axis shortest-arc Log components [x, y, z] of a unit quaternion."""
    norm = math.sqrt(sum(component * component for component in q))
    w = q[0] / norm
    vector_part = [component / norm for component in q[1:]]
    vector_norm = math.sqrt(sum(component * component for component in vector_part))
    if vector_norm < 1e-15:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(vector_norm, w)
    return [angle * component / vector_norm for component in vector_part]


def quaternion_conjugate(q: list[float]) -> list[float]:
    return [q[0], -q[1], -q[2], -q[3]]


def quaternion_multiply(a: list[float], b: list[float]) -> list[float]:
    return [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]


def orientation_log_relative(q: list[float], anchor: list[float]) -> list[float]:
    return quaternion_log(quaternion_multiply(q, quaternion_conjugate(anchor)))


def episodes_of(rows: list[dict[str, str]]) -> dict[str, list[dict[str, str]]]:
    grouped: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        grouped.setdefault(row["episode"], []).append(row)
    return grouped


def replay_exact(rows: list[dict[str, str]], ignore: tuple[str, ...] = ()) -> bool:
    sequences = [
        [{key: value for key, value in row.items()
          if key not in (*ignore, "episode")}
         for row in episode]
        for episode in episodes_of(rows).values()
    ]
    return len(sequences) <= 1 or all(item == sequences[0] for item in sequences[1:])


def normal_checks(
    control: list[dict[str, str]],
    plant: list[dict[str, str]],
    config: dict[str, Any],
    episodes: int,
) -> tuple[dict[str, Any], dict[str, bool]]:
    gates = config["gates"]
    reference = config["reference"]
    torque_limit = config["torque_limit_nm"]
    grouped = episodes_of(control)
    anchors: dict[str, list[float]] = {}
    for episode, rows in grouped.items():
        anchors[episode] = [float(rows[0][f"quat{index}"]) for index in range(4)]
    by_tick = {
        (row["episode"], row["tick"]): row for row in control
    }

    orientation_errors = [
        component
        for row in control
        for component in orientation_log_relative(
            [float(row[f"quat{index}"]) for index in range(4)],
            anchors[row["episode"]],
        )
    ]
    final_rows = [
        row for row in control
        if int(row["tick"]) == int(config["normal_ticks"]) - 1
    ]
    task_residuals = [
        abs(float(row[f"task_residual{task}"]))
        for row in control for task in range(7)
    ]
    task_costs = [
        abs(float(row[f"task_cost{task}"])) for row in control for task in range(7)
    ]
    adapter_sign_error = 0.0
    for row in plant:
        command = by_tick[(row["episode"], row["control_tick"])]
        for joint in range(6):
            adapter_sign_error = max(
                adapter_sign_error,
                abs(float(row[f"ctrl{joint}"]) +
                    float(command[f"command_tau{joint}"])),
            )
    saturation_count = sum(
        abs(float(row[f"command_tau{joint}"])) > torque_limit[joint] + 1e-12
        for row in control for joint in range(6)
    )
    period_errors = [
        abs(float(row["dt_s"]) - config["timing"]["control_period_s"])
        for row in control if int(row["tick"]) > 0
    ] + [abs(float(row["dt_s"])) for row in control if int(row["tick"]) == 0]
    metrics = {
        "control_row_count": len(control),
        "plant_row_count": len(plant),
        "maximum_core_step_ms": max(float(row["core_step_ns"]) for row in control)
        / 1.0e6,
        "maximum_abs_x_m": max(
            abs(float(row["base_p0"]) -
                float(grouped[row["episode"]][0]["base_p0"]))
            for row in control
        ),
        "maximum_abs_y_m": max(
            abs(float(row["base_p1"]) -
                float(grouped[row["episode"]][0]["base_p1"]))
            for row in control
        ),
        "maximum_height_error_m": max(
            abs(float(row["base_p2"]) -
                float(grouped[row["episode"]][0]["base_p2"]))
            for row in control
        ),
        "maximum_abs_roll_rad": max(abs(v) for v in orientation_errors[0::3]),
        "maximum_abs_pitch_rad": max(abs(v) for v in orientation_errors[1::3]),
        "maximum_abs_yaw_rad": max(abs(v) for v in orientation_errors[2::3]),
        "maximum_leg_error_rad": max(
            abs(float(row[f"q{joint}"]) - reference[joint])
            for row in control for joint in LEG_JOINTS
        ),
        "bilateral_contact_fraction": sum(
            row["contact_left"] == "2" and row["contact_right"] == "2"
            for row in control) / len(control),
        "maximum_final_linear_speed_m_s": max(math.sqrt(sum(
            float(row[name]) ** 2
            for name in ("base_v0", "base_v1", "base_v2")
        )) for row in final_rows),
        "maximum_final_angular_speed_rad_s": max(math.sqrt(sum(
            float(row[name]) ** 2
            for name in ("base_w0", "base_w1", "base_w2")
        )) for row in final_rows),
        "maximum_zoh_difference": max(float(row["zoh_diff"]) for row in control),
        "maximum_adapter_sign_error": adapter_sign_error,
        "minimum_wheel_normal_load_n": min(
            float(row[name]) for row in plant
            for name in ("left_normal_load_n", "right_normal_load_n")
        ),
        "maximum_penetration_m": max(float(row["penetration_m"]) for row in plant),
        "maximum_abs_rolling_slip_m_s": max(
            float(row["rolling_slip_m_s"]) for row in plant),
        "maximum_abs_lateral_slip_m_s": max(
            float(row["lateral_slip_m_s"]) for row in plant),
        "maximum_closure_residual_m": max(
            max(float(row["closure_residual_m"]) for row in plant),
            max(float(row["closure_residual"]) for row in control),
        ),
        "maximum_hard_violation": max(
            float(row["hard"]) for row in control),
        "maximum_primal_residual": max(float(row["primal"]) for row in control),
        "maximum_dual_residual": max(float(row["dual"]) for row in control),
        "maximum_stationarity_residual": max(
            float(row["stationarity"]) for row in control),
        "maximum_solver_iterations": max(
            float(row["iterations"]) for row in control),
        "maximum_normalized_slack": max(
            float(row["max_normalized_slack"]) for row in control),
        "maximum_task_residual": max(task_residuals),
        "maximum_task_cost": max(task_costs),
        "saturation_count": saturation_count,
        "maximum_control_period_error_s": max(period_errors),
        "episode_replay_exact": replay_exact(control, CONTROL_EXACT_IGNORE),
        "plant_episode_replay_exact": replay_exact(plant),
    }
    checks = {
        "completed": len(control) == int(config["normal_ticks"]) * episodes
        and len(plant) == int(config["normal_ticks"])
        * int(config["timing"]["physics_substeps_per_control"]) * episodes,
        "finite": all(
            finite_row(row, ("scenario", "episode")) for row in control
        ) and all(
            finite_row(row, ("scenario", "episode", "disturbance")) for row in plant
        ),
        "controller_ok": all(
            row["status"] == "0" and row["latch"] == "0" for row in control
        ),
        "wbc_statuses": all(
            row["weighted_status"] == "0" and row["model_status"] == "0"
            and row["solver_status"] == "0"
            for row in control
        ),
        "command_accepted": all(row["accepted"] == "1" for row in control),
        "x": metrics["maximum_abs_x_m"] <= gates["maximum_abs_x_m"],
        "y": metrics["maximum_abs_y_m"] <= gates["maximum_abs_y_m"],
        "height": metrics["maximum_height_error_m"] <= gates["maximum_height_error_m"],
        "roll": metrics["maximum_abs_roll_rad"] <= gates["maximum_abs_roll_rad"],
        "pitch": metrics["maximum_abs_pitch_rad"] <= gates["maximum_abs_pitch_rad"],
        "yaw": metrics["maximum_abs_yaw_rad"] <= gates["maximum_abs_yaw_rad"],
        "leg": metrics["maximum_leg_error_rad"] <= gates["maximum_leg_error_rad"],
        "contact": metrics["bilateral_contact_fraction"]
        >= gates["minimum_bilateral_contact_fraction"],
        "final_linear": metrics["maximum_final_linear_speed_m_s"]
        <= gates["maximum_final_linear_speed_m_s"],
        "final_angular": metrics["maximum_final_angular_speed_rad_s"]
        <= gates["maximum_final_angular_speed_rad_s"],
        "zoh": metrics["maximum_zoh_difference"] <= gates["maximum_zoh_difference"],
        "adapter_sign": metrics["maximum_adapter_sign_error"]
        <= gates["maximum_adapter_sign_error"],
        "normal_load": metrics["minimum_wheel_normal_load_n"]
        >= gates["minimum_wheel_normal_load_n"],
        "penetration": metrics["maximum_penetration_m"] <= gates["maximum_penetration_m"],
        "rolling_slip": metrics["maximum_abs_rolling_slip_m_s"]
        <= gates["maximum_abs_rolling_slip_m_s"],
        "lateral_slip": metrics["maximum_abs_lateral_slip_m_s"]
        <= gates["maximum_abs_lateral_slip_m_s"],
        "closure": metrics["maximum_closure_residual_m"]
        <= gates["maximum_closure_residual_m"],
        "hard_violation": metrics["maximum_hard_violation"]
        <= gates["maximum_hard_violation"],
        "primal": metrics["maximum_primal_residual"] <= gates["maximum_primal_residual"],
        "dual": metrics["maximum_dual_residual"] <= gates["maximum_dual_residual"],
        "stationarity": metrics["maximum_stationarity_residual"]
        <= gates["maximum_stationarity_residual"],
        "solver_failures": metrics["maximum_solver_iterations"] >= 0
        and not any(row["solver_status"] != "0" for row in control),
        "slack": metrics["maximum_normalized_slack"] <= gates["maximum_normalized_slack"],
        "task_residual": metrics["maximum_task_residual"]
        <= gates["maximum_task_residual"],
        "task_cost": metrics["maximum_task_cost"] <= gates["maximum_task_cost"],
        "no_saturation": metrics["saturation_count"]
        <= gates["maximum_saturation_count"],
        "deadline": metrics["maximum_core_step_ms"] <= gates["maximum_core_step_ms"],
        "control_period": metrics["maximum_control_period_error_s"]
        <= gates["maximum_control_period_error_s"],
        "episode_replay": metrics["episode_replay_exact"]
        and metrics["plant_episode_replay_exact"],
    }
    return metrics, checks


FAULT_STATUS = {
    "invalid": "2",
    "nonmonotonic": "3",
    "contact_loss_left": "4",
    "contact_loss_right": "4",
    "timing": "4",
    "saturation": "4",
    "nmpc_solver_failure": "4",
    "nmpc_late": "4",
    "nmpc_stale": "4",
    "nmpc_nonfinite": "4",
}


def fault_checks(
    control: list[dict[str, str]],
    plant: list[dict[str, str]],
    scenario: str,
    config: dict[str, Any],
) -> dict[str, Any]:
    fault_tick = int(config["fault_tick"])
    grouped = episodes_of(control)
    checks: dict[str, Any] = {
        "two_episodes": len(grouped) == 2,
        # No `finite` check here: the invalid scenario injects a non-finite
        # quaternion by construction; the contract under test is fail-zero.
        "latched_zero": all(
            row["latch"] == "1"
            and all(float(row[f"command_tau{joint}"]) == 0.0 for joint in range(6))
            for episode in grouped.values() for row in episode[fault_tick:]
        ),
        "reset_replay_exact": replay_exact(control, CONTROL_EXACT_IGNORE),
        "plant_reset_replay_exact": replay_exact(plant),
    }
    if scenario == "saturation":
        checks["pre_fault_ok"] = True
        checks["fault_status"] = all(
            row["status"] == FAULT_STATUS[scenario]
            for episode in grouped.values() for row in episode
        )
    else:
        checks["pre_fault_ok"] = all(
            row["status"] == "0" for episode in grouped.values()
            for row in episode[:fault_tick]
        )
        checks["fault_status"] = all(
            episode[fault_tick]["status"] == FAULT_STATUS[scenario]
            for episode in grouped.values()
        )
    return checks


def base_command(
    runner: Path, scene: Path, control: Path, plant: Path,
    config: dict[str, Any],
) -> list[str]:
    return [
        str(runner), "--model", str(scene),
        "--control-output", str(control), "--plant-output", str(plant),
        "--equilibrium", vector(config["equilibrium"]),
        "--torque-limit", vector(config["torque_limit_nm"]),
    ]


def run(config: dict[str, Any], runner: Path, output: Path) -> dict[str, Any]:
    scene = (ROOT / config["scene"]).resolve()
    cases = []
    for case in config["cases"]:
        control_path = output / f"{case['id']}_control.csv"
        plant_path = output / f"{case['id']}_plant.csv"
        episodes = int(case.get("episodes", 1))
        command = base_command(runner, scene, control_path, plant_path, config) + [
            "--scenario", "hold",
            "--controller-mode", config.get("controller_mode", "weighted_wbc"),
            "--nmpc-reference", case.get("nmpc_reference", "hold"),
            "--episodes", str(episodes),
            "--ticks", str(config["normal_ticks"]),
            "--initial-state", vector(case.get("initial_state", [0.0] * 8)),
            "--leg-perturbation", vector(case.get("leg_perturbation", [0.0] * 4)),
            "--disturbance-start-tick", str(config["disturbance_start_tick"]),
            "--disturbance-ticks", str(config["disturbance_ticks"]),
            "--force", vector(case.get("force_n", [0.0] * 3)),
            "--moment", vector(case.get("moment_nm", [0.0] * 3)),
        ]
        subprocess.run(command, check=True)
        metrics, checks = normal_checks(
            read_rows(control_path), read_rows(plant_path), config, episodes
        )
        if config.get("controller_mode") == "nominal_nmpc":
            rows = read_rows(control_path)
            checks.update({
                "nmpc_status": all(row["nmpc_status"] == "0" for row in rows),
                "nmpc_schedule": all(
                    (row["nmpc_update"] == "1") == (int(row["tick"]) % 2 == 0)
                    and int(row["nmpc_age"]) == int(row["tick"]) % 2
                    for row in rows
                ),
                "nmpc_deadline": max(
                    float(row["nmpc_wbc_total_s"]) for row in rows
                ) <= config["nmpc_gates"]["maximum_combined_time_s"],
                "nmpc_feasibility": max(
                    max(float(row[name]) for name in (
                        "nmpc_dynamics", "nmpc_inequality",
                        "nmpc_complementarity", "nmpc_first_step_defect"))
                    for row in rows
                ) <= config["nmpc_gates"]["maximum_feasibility_residual"],
                "nmpc_stationarity": max(
                    float(row["nmpc_stationarity"]) for row in rows
                ) <= config["nmpc_gates"]["maximum_stationarity_residual"],
                "nmpc_independent_dynamics": max(
                    float(row["nmpc_maximum_dynamics_defect"]) for row in rows
                ) <= config["nmpc_gates"]["maximum_independent_dynamics_defect"],
                "nmpc_independent_stationarity": max(
                    float(row["nmpc_projected_stationarity"]) for row in rows
                ) <= config["nmpc_gates"]["maximum_projected_stationarity"],
                "nmpc_independent_objective": all(
                    math.isfinite(float(row["nmpc_objective"])) for row in rows
                ),
            })
            reference = case.get("nmpc_reference", "hold")
            positions = [float(row["base_p0"]) for row in rows]
            origin = positions[0]
            final_delta = sum(positions[-100:]) / min(100, len(positions)) - origin
            if reference == "positive":
                checks["nmpc_tracking"] = (
                    final_delta >= config["nmpc_gates"]["minimum_step_tracking_m"]
                )
            elif reference == "negative":
                checks["nmpc_tracking"] = (
                    final_delta <= -config["nmpc_gates"]["minimum_step_tracking_m"]
                )
            elif reference == "return":
                checks["nmpc_tracking"] = (
                    max(positions) - origin >=
                    config["nmpc_gates"]["minimum_return_excursion_m"]
                    and abs(final_delta) <=
                    config["nmpc_gates"]["maximum_return_error_m"]
                )
        cases.append({"id": case["id"], "pass": all(checks.values()),
                      "checks": checks, "metrics": metrics})
    faults = []
    for scenario in config["fault_cases"]:
        control_path = output / f"fault_{scenario}_control.csv"
        plant_path = output / f"fault_{scenario}_plant.csv"
        saturation = config["saturation"] if scenario == "saturation" else None
        torque_limit = (
            saturation["torque_limit_nm"] if saturation
            else config["torque_limit_nm"]
        )
        command = [
            str(runner), "--model", str(scene),
            "--control-output", str(control_path), "--plant-output", str(plant_path),
            "--equilibrium", vector(config["equilibrium"]),
            "--torque-limit", vector(torque_limit),
            "--scenario", scenario if scenario != "saturation" else "hold",
            "--controller-mode", config.get("controller_mode", "weighted_wbc"),
            "--nmpc-reference", "hold",
            "--episodes", "2",
            "--ticks", str(config["fault_ticks"]),
            "--fault-tick", str(config["fault_tick"]),
        ]
        subprocess.run(command, check=True)
        checks = fault_checks(
            read_rows(control_path), read_rows(plant_path), scenario, config
        )
        faults.append({"id": scenario, "pass": all(checks.values()),
                       "checks": checks})
    return {
        "schema_version": 1,
        "phase": config.get("phase", 21),
        "profile": config["profile"],
        "evidence_class": config["evidence_class"],
        "pass": all(case["pass"] for case in cases + faults),
        "cases": cases,
        "fault_cases": faults,
    }


SOURCE_INPUTS = (
    "ros_ws/src/wheel_leg_core/include/wheel_leg_core/dense_qp_solver.hpp",
    "ros_ws/src/wheel_leg_core/include/wheel_leg_core/controller_core.hpp",
    "ros_ws/src/wheel_leg_core/include/wheel_leg_core/nominal_nmpc_model.hpp",
    "ros_ws/src/wheel_leg_core/include/wheel_leg_core/nominal_nmpc_solver.hpp",
    "ros_ws/src/wheel_leg_core/include/wheel_leg_core/weighted_wbc_controller.hpp",
    "ros_ws/src/wheel_leg_core/src/controller_core.cpp",
    "ros_ws/src/wheel_leg_core/src/nominal_nmpc_model.cpp",
    "ros_ws/src/wheel_leg_core/src/nominal_nmpc_solver.cpp",
    "ros_ws/src/wheel_leg_core/src/dense_qp_solver.cpp",
    "ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp",
    "ros_ws/src/wheel_leg_mujoco/src/adapter.cpp",
    "ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp",
    "ros_ws/src/wheel_leg_mujoco/CMakeLists.txt",
)

PROFILE_INPUTS = (
    "simulation/mujoco/config/phase21_task_prefreeze_42d_nonlinear_frozen_v2.json",
    "simulation/mujoco/config/phase21_task_prefreeze_42d_runtime_v2.json",
    "simulation/mujoco/config/phase21_hard_qp_42d_runtime_v2.json",
    "simulation/mujoco/config/phase21_runtime_model_profile_v1.json",
    "simulation/mujoco/config/phase20_equilibrium.json",
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    config_path = arguments.config.resolve()
    runner = arguments.runner.resolve()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    config, config_chain = load_config(config_path)
    if not config["cases"] or not config["fault_cases"]:
        raise RuntimeError("Frozen case matrix must not be empty")
    output.mkdir(parents=True, exist_ok=True)
    summary = run(config, runner, output)
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    dynamic_profiles = tuple(config.get("source_profiles", {}).values())
    source_inputs = {
        str(path): sha256(ROOT / path)
        for path in dict.fromkeys((*SOURCE_INPUTS, *PROFILE_INPUTS, *dynamic_profiles))
    }
    outputs = {
        path.name: sha256(path) for path in sorted(output.iterdir())
        if path.is_file()
    }
    acados_provenance: dict[str, Any] | None = None
    generated_inputs: dict[str, str] = {}
    if config.get("controller_mode") == "nominal_nmpc":
        acados_root = Path("/home/t/opt/acados")
        generated_root = (
            ROOT / "ros_ws/src/wheel_leg_core/acados_generated/"
            "phase23_nominal_nmpc_v1"
        )
        generated_inputs = {
            root_relative(path): sha256(path)
            for path in sorted(generated_root.rglob("*")) if path.is_file()
        }
        libraries = {
            name: acados_root / "lib" / name for name in (
                "libacados.so", "libhpipm.so", "libblasfeo.so.0.1.4.2"
            )
        }
        renderer = ROOT / ".cache/acados/t_renderer"
        acados_provenance = {
            "root": str(acados_root),
            "commit": subprocess.check_output(
                ["git", "-C", str(acados_root), "rev-parse", "HEAD"],
                text=True,
            ).strip(),
            "libraries": {
                name: sha256(path) for name, path in libraries.items()
            },
            "renderer_path": str(renderer),
            "renderer_sha256": sha256(renderer),
            "casadi_version": importlib.metadata.version("casadi"),
            "acados_template_path": str(
                acados_root / "interfaces/acados_template/acados_template"
            ),
            "runner_ldd": [
                line.rsplit(" (0x", 1)[0].strip()
                for line in subprocess.check_output(
                    ["ldd", str(runner)], text=True
                ).splitlines()
            ],
        }
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "hardware_data": False,
        "config_path": root_relative(config_path),
        "config_sha256": sha256(config_path),
        "config_chain": {
            root_relative(path): sha256(path) for path in config_chain
        },
        "runner_path": root_relative(runner),
        "runner_sha256": sha256(runner),
        "wrapper_sha256": sha256(Path(__file__).resolve()),
        "scene": config["scene"],
        "scene_sha256": sha256((ROOT / config["scene"]).resolve()),
        "solver": config["solver"],
        "source_inputs": source_inputs,
        "generated_inputs": generated_inputs,
        "acados_provenance": acados_provenance,
        "outputs": outputs,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"pass": summary["pass"],
                      "cases": [case["pass"] for case in summary["cases"]],
                      "fault_cases": [f["pass"] for f in summary["fault_cases"]]}))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
