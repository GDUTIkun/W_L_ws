#!/usr/bin/env python3
"""Run and evaluate the Phase-20 C++ full-3D standing matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase20_formal.json"
DEFAULT_RUNNER = (
    ROOT / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/standing_3d_loop"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values: list[float]) -> str:
    return ",".join(format(float(value), ".17g") for value in values)


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def replay_exact(values: list[dict[str, str]]) -> bool:
    episodes: dict[str, list[dict[str, str]]] = {}
    for row in values:
        normalized = dict(row)
        episode = normalized.pop("episode")
        episodes.setdefault(episode, []).append(normalized)
    sequences = list(episodes.values())
    return len(sequences) <= 1 or all(item == sequences[0] for item in sequences[1:])


def normal_checks(
    values: list[dict[str, str]], config: dict[str, Any], episodes: int
) -> tuple[dict[str, Any], dict[str, bool]]:
    active = (0, 1, 3, 4)
    reference = config["reference"]
    gates = config["gates"]
    first_y: dict[str, float] = {}
    first_z: dict[str, float] = {}
    for row in values:
        first_y.setdefault(row["episode"], float(row["y_m"]))
        first_z.setdefault(row["episode"], float(row["height_m"]))
    virtual_errors = []
    for row in values:
        virtual = [float(row[f"u{i}"]) for i in range(3)]
        for joint in range(6):
            balance = config["roll_direction"][joint] * virtual[1]
            if joint == 2:
                balance += virtual[0] + virtual[2]
            elif joint == 5:
                balance += virtual[0] - virtual[2]
            expected = (
                float(row[f"support{joint}"])
                + float(row[f"pd{joint}"]) + balance
            )
            virtual_errors.append(abs(float(row[f"raw{joint}"]) - expected))
    finals = [
        row for row in values
        if int(row["tick"]) == int(config["normal_ticks"]) - 1
    ]
    metrics = {
        "row_count": len(values),
        "maximum_abs_x_m": max(abs(float(row["x_m"])) for row in values),
        "maximum_abs_y_m": max(
            abs(float(row["y_m"]) - first_y[row["episode"]]) for row in values
        ),
        "maximum_height_error_m": max(
            abs(float(row["height_m"]) - first_z[row["episode"]])
            for row in values
        ),
        "maximum_abs_pitch_rad": max(abs(float(row["pitch_rad"])) for row in values),
        "maximum_abs_roll_rad": max(abs(float(row["roll_rad"])) for row in values),
        "maximum_abs_yaw_rad": max(abs(float(row["yaw_rad"])) for row in values),
        "maximum_leg_error_rad": max(
            abs(float(row[f"q{joint}"]) - reference[joint])
            for row in values for joint in active
        ),
        "bilateral_contact_fraction": sum(
            row["contact_left"] == "2" and row["contact_right"] == "2"
            for row in values
        ) / len(values),
        "maximum_final_linear_speed_m_s": max(math.sqrt(sum(
            float(row[name]) ** 2 for name in ("vx_m_s", "vy_m_s", "vz_m_s")
        )) for row in finals),
        "maximum_final_angular_speed_rad_s": max(math.sqrt(sum(
            float(row[name]) ** 2 for name in ("wx_rad_s", "wy_rad_s", "wz_rad_s")
        )) for row in finals),
        "maximum_zoh_difference": max(
            float(row["zoh_ctrl_max_difference"]) for row in values
        ),
        "maximum_adapter_sign_error": max(
            abs(float(row[f"ctrl{joint}"]) + float(row[f"tau{joint}"]))
            for row in values for joint in range(6)
        ),
        "maximum_virtual_mapping_error": max(virtual_errors),
        "minimum_wheel_normal_load_n": min(
            float(row[name]) for row in values
            for name in ("left_normal_load_n", "right_normal_load_n")
        ),
        "maximum_penetration_m": max(
            float(row["maximum_penetration_m"]) for row in values
        ),
        "maximum_abs_rolling_slip_m_s": max(
            float(row["maximum_abs_rolling_slip_m_s"]) for row in values
        ),
        "maximum_abs_lateral_slip_m_s": max(
            float(row["maximum_abs_lateral_slip_m_s"]) for row in values
        ),
        "maximum_closure_residual_m": max(
            float(row["closure_residual_m"]) for row in values
        ),
        "episode_replay_exact": replay_exact(values),
    }
    checks = {
        "completed": len(values) == int(config["normal_ticks"]) * episodes,
        "finite": all(
            math.isfinite(float(value))
            for row in values for key, value in row.items()
            if key not in ("scenario", "episode")
        ),
        "controller_ok": all(
            row["status"] == "0" and row["safety_latched"] == "0"
            for row in values
        ),
        "command_accepted": all(row["command_accepted"] == "1" for row in values),
        "x": metrics["maximum_abs_x_m"] <= gates["maximum_abs_x_m"],
        "y": metrics["maximum_abs_y_m"] <= gates["maximum_abs_y_m"],
        "height": metrics["maximum_height_error_m"] <= gates["maximum_height_error_m"],
        "pitch": metrics["maximum_abs_pitch_rad"] <= gates["maximum_abs_pitch_rad"],
        "roll": metrics["maximum_abs_roll_rad"] <= gates["maximum_abs_roll_rad"],
        "yaw": metrics["maximum_abs_yaw_rad"] <= gates["maximum_abs_yaw_rad"],
        "leg": metrics["maximum_leg_error_rad"] <= gates["maximum_leg_error_rad"],
        "contact": metrics["bilateral_contact_fraction"] >= gates["minimum_bilateral_contact_fraction"],
        "final_linear": metrics["maximum_final_linear_speed_m_s"] <= gates["maximum_final_linear_speed_m_s"],
        "final_angular": metrics["maximum_final_angular_speed_rad_s"] <= gates["maximum_final_angular_speed_rad_s"],
        "zoh": metrics["maximum_zoh_difference"] <= gates["maximum_zoh_difference"],
        "adapter_sign": metrics["maximum_adapter_sign_error"] <= gates["maximum_adapter_sign_error"],
        "virtual_mapping": metrics["maximum_virtual_mapping_error"] <= gates["maximum_virtual_mapping_error"],
        "normal_load": metrics["minimum_wheel_normal_load_n"] >= gates["minimum_wheel_normal_load_n"],
        "penetration": metrics["maximum_penetration_m"] <= gates["maximum_penetration_m"],
        "rolling_slip": metrics["maximum_abs_rolling_slip_m_s"] <= gates["maximum_abs_rolling_slip_m_s"],
        "lateral_slip": metrics["maximum_abs_lateral_slip_m_s"] <= gates["maximum_abs_lateral_slip_m_s"],
        "closure": metrics["maximum_closure_residual_m"] <= gates["maximum_closure_residual_m"],
        "episode_replay": metrics["episode_replay_exact"],
    }
    return metrics, checks


def fault_checks(
    values: list[dict[str, str]], scenario: str, fault_tick: int
) -> dict[str, bool]:
    expected = {
        "invalid": "2", "nonmonotonic": "3",
        "contact_loss_left": "4", "contact_loss_right": "4",
        "timing": "4", "saturation": "4",
    }[scenario]
    episodes: dict[str, list[dict[str, str]]] = {}
    for row in values:
        episodes.setdefault(row["episode"], []).append(row)
    return {
        "two_episodes": len(episodes) == 2,
        "pre_fault_ok": all(
            row["status"] == "0" for episode in episodes.values()
            for row in episode if int(row["tick"]) < fault_tick
        ),
        "fault_status": all(
            episode[fault_tick]["status"] == expected for episode in episodes.values()
        ),
        "latched_zero": all(
            row["safety_latched"] == "1"
            and all(float(row[f"tau{joint}"]) == 0.0 for joint in range(6))
            for episode in episodes.values() for row in episode[fault_tick:]
        ),
        "reset_replay_exact": replay_exact(values),
    }


def base_command(
    runner: Path, scene: Path, output: Path, config: dict[str, Any]
) -> list[str]:
    return [
        str(runner), "--model", str(scene), "--output", str(output),
        "--equilibrium", vector(config["equilibrium"]),
        "--reference", vector(config["reference"]),
        "--support", vector(config["support_torque_nm"]),
        "--kp", vector(config["kp_nm_per_rad"]),
        "--kd", vector(config["kd_nm_s_per_rad"]),
        "--torque-limit", vector(config["torque_limit_nm"]),
        "--gain", vector(config["standing_gain"]),
        "--roll-direction", vector(config["roll_direction"]),
        "--safety", vector(config["safety"]),
    ]


def run(
    config: dict[str, Any], runner: Path, output: Path
) -> dict[str, Any]:
    scene = (ROOT / config["scene"]).resolve()
    cases = []
    for case in config["cases"]:
        raw = output / f"{case['id']}.csv"
        episodes = int(case.get("episodes", 1))
        command = base_command(runner, scene, raw, config) + [
            "--ticks", str(config["normal_ticks"]), "--episodes", str(episodes),
            "--initial-state", vector(case.get("initial_state", [0.0] * 8)),
            "--leg-perturbation", vector(case.get("leg_perturbation", [0.0] * 4)),
            "--disturbance-start-tick", str(config["disturbance_start_tick"]),
            "--disturbance-ticks", str(config["disturbance_ticks"]),
            "--force", vector(case.get("force_n", [0.0] * 3)),
            "--moment", vector(case.get("moment_nm", [0.0] * 3)),
        ]
        subprocess.run(command, check=True)
        metrics, checks = normal_checks(rows(raw), config, episodes)
        cases.append({"id": case["id"], "pass": all(checks.values()),
                      "checks": checks, "metrics": metrics})
    faults = []
    for scenario in config["fault_cases"]:
        raw = output / f"fault_{scenario}.csv"
        command = base_command(runner, scene, raw, config) + [
            "--scenario", scenario, "--ticks", str(config["fault_ticks"]),
            "--episodes", "2", "--fault-tick", str(config["fault_tick"]),
        ]
        subprocess.run(command, check=True)
        checks = fault_checks(rows(raw), scenario, int(config["fault_tick"]))
        faults.append({"id": scenario, "pass": all(checks.values()), "checks": checks})
    return {
        "schema_version": 1,
        "phase": 20,
        "evidence_class": "formal current-nominal simulation-only",
        "pass": all(case["pass"] for case in cases + faults),
        "cases": cases,
        "fault_cases": faults,
    }


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
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    summary = run(config, runner, output)
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    source_inputs = [
        ROOT / "ros_ws/src/wheel_leg_core/include/wheel_leg_core/controller_core.hpp",
        ROOT / "ros_ws/src/wheel_leg_core/src/controller_core.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/adapter.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/standing_3d_loop.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/CMakeLists.txt",
    ]
    outputs = sorted(path for path in output.iterdir() if path.is_file())
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "hardware_data": False,
        "config_sha256": sha256(config_path),
        "runner_sha256": sha256(runner),
        "wrapper_sha256": sha256(Path(__file__).resolve()),
        "scene_sha256": sha256((ROOT / config["scene"]).resolve()),
        "source_inputs": {
            str(path.relative_to(ROOT)): sha256(path) for path in source_inputs
        },
        "outputs": {path.name: sha256(path) for path in outputs},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
