#!/usr/bin/env python3
"""Run and evaluate the Phase-19 C++ exact-planar standing matrix."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase19_planar_formal.json"
DEFAULT_RUNNER = ROOT / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/planar_standing_loop"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-formal-v4"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values: list[float]) -> str:
    return ",".join(format(float(value), ".17g") for value in values)


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def episode_replay_exact(rows: list[dict[str, str]]) -> bool:
    episodes: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        normalized = dict(row)
        episode = normalized.pop("episode")
        episodes.setdefault(episode, []).append(normalized)
    values = list(episodes.values())
    return len(values) <= 1 or all(value == values[0] for value in values[1:])


def normal_metrics(
    rows: list[dict[str, str]], config: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, bool]]:
    reference = config["reference"]
    active = (0, 1, 3, 4)
    first_height: dict[str, float] = {}
    for row in rows:
        first_height.setdefault(row["episode"], float(row["height"]))
    metrics = {
        "row_count": len(rows),
        "maximum_abs_pitch_rad": max(abs(float(row["pitch"])) for row in rows),
        "maximum_abs_x_m": max(abs(float(row["x"])) for row in rows),
        "maximum_height_error_m": max(
            abs(float(row["height"]) - first_height[row["episode"]]) for row in rows
        ),
        "maximum_leg_error_rad": max(
            abs(float(row[f"q{joint}"]) - float(reference[joint]))
            for row in rows for joint in active
        ),
        "bilateral_contact_fraction": sum(
            row["contact_left"] == "2" and row["contact_right"] == "2"
            for row in rows
        ) / len(rows),
        "maximum_zoh_difference": max(float(row["zoh_ctrl_max_difference"]) for row in rows),
        "maximum_equal_wheel_torque_difference": max(
            abs(float(row["tau2"]) - float(row["tau5"])) for row in rows
        ),
        "maximum_adapter_sign_error": max(
            abs(float(row[f"ctrl{joint}"]) + float(row[f"tau{joint}"]))
            for row in rows for joint in range(6)
        ),
        "episode_replay_exact": episode_replay_exact(rows),
    }
    finals = [row for row in rows if int(row["tick"]) == config["normal_ticks"] - 1]
    metrics["maximum_final_abs_x_m"] = max(abs(float(row["x"])) for row in finals)
    metrics["maximum_final_abs_dx_m_s"] = max(abs(float(row["dx"])) for row in finals)
    metrics["maximum_final_abs_pitch_rad"] = max(abs(float(row["pitch"])) for row in finals)
    metrics["maximum_final_abs_pitch_rate_rad_s"] = max(abs(float(row["dtheta"])) for row in finals)
    gates = config["gates"]
    checks = {
        "completed": len(rows) == config["normal_ticks"] * len({row["episode"] for row in rows}),
        "controller_ok": all(row["status"] == "0" and row["safety_latched"] == "0" for row in rows),
        "command_accepted": all(row["command_accepted"] == "1" for row in rows),
        "pitch": metrics["maximum_abs_pitch_rad"] <= gates["maximum_abs_pitch_rad"],
        "x": metrics["maximum_abs_x_m"] <= gates["maximum_abs_x_m"],
        "height": metrics["maximum_height_error_m"] <= gates["maximum_height_error_m"],
        "leg_error": metrics["maximum_leg_error_rad"] <= gates["maximum_leg_error_rad"],
        "contact": metrics["bilateral_contact_fraction"] >= gates["minimum_bilateral_contact_fraction"],
        "final_x": metrics["maximum_final_abs_x_m"] <= gates["maximum_final_abs_x_m"],
        "final_dx": metrics["maximum_final_abs_dx_m_s"] <= gates["maximum_final_abs_dx_m_s"],
        "final_pitch": metrics["maximum_final_abs_pitch_rad"] <= gates["maximum_final_abs_pitch_rad"],
        "final_pitch_rate": metrics["maximum_final_abs_pitch_rate_rad_s"] <= gates["maximum_final_abs_pitch_rate_rad_s"],
        "zoh": metrics["maximum_zoh_difference"] <= gates["maximum_zoh_difference"],
        "equal_wheels": metrics["maximum_equal_wheel_torque_difference"] <= gates["maximum_equal_wheel_torque_difference"],
        "adapter_sign": metrics["maximum_adapter_sign_error"] <= gates["maximum_adapter_sign_error"],
        "episode_replay": metrics["episode_replay_exact"],
    }
    return metrics, checks


def fault_checks(rows: list[dict[str, str]], scenario: str, fault_tick: int) -> dict[str, bool]:
    expected = {"contact_loss": "4", "invalid": "2", "nonmonotonic": "3", "saturation": "4"}[scenario]
    by_episode: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_episode.setdefault(row["episode"], []).append(row)
    return {
        "two_episodes": len(by_episode) == 2,
        "pre_fault_ok": all(
            row["status"] == "0" for episode in by_episode.values()
            for row in episode if int(row["tick"]) < fault_tick
        ),
        "fault_status": all(
            episode[fault_tick]["status"] == expected for episode in by_episode.values()
        ),
        "latched_after_fault": all(
            row["safety_latched"] == "1" and all(float(row[f"tau{i}"]) == 0.0 for i in range(6))
            for episode in by_episode.values() for row in episode[fault_tick:]
        ),
        "reset_replay_exact": episode_replay_exact(rows),
    }


def runner_command(
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
        "--safety", vector(config["safety"]),
    ]


def run(config: dict[str, Any], runner: Path, output: Path) -> dict[str, Any]:
    scene = (ROOT / config["scene"]).resolve()
    cases = []
    for case in config["cases"]:
        raw = output / f"{case['id']}.csv"
        command = runner_command(runner, scene, raw, config) + [
            "--ticks", str(config["normal_ticks"]),
            "--episodes", str(case.get("episodes", 1)),
            "--initial-state", vector(case.get("initial_state", [0, 0, 0, 0])),
            "--leg-perturbation", vector(case.get("leg_perturbation", [0, 0, 0, 0])),
            "--disturbance-start-tick", str(case.get("disturbance_start_tick", -1)),
            "--disturbance-ticks", str(case.get("disturbance_ticks", 0)),
            "--force-x", str(case.get("force_x_n", 0.0)),
            "--pitch-moment", str(case.get("pitch_moment_nm", 0.0)),
        ]
        subprocess.run(command, check=True)
        rows = load_rows(raw)
        metrics, checks = normal_metrics(rows, config)
        cases.append({"id": case["id"], "pass": all(checks.values()), "checks": checks, "metrics": metrics})
    faults = []
    for scenario in config["fault_cases"]:
        raw = output / f"fault_{scenario}.csv"
        command = runner_command(runner, scene, raw, config) + [
            "--scenario", scenario, "--ticks", str(config["fault_ticks"]),
            "--episodes", "2", "--fault-tick", str(config["fault_tick"]),
        ]
        subprocess.run(command, check=True)
        checks = fault_checks(load_rows(raw), scenario, config["fault_tick"])
        faults.append({"id": scenario, "pass": all(checks.values()), "checks": checks})
    return {
        "pass": all(case["pass"] for case in cases) and all(case["pass"] for case in faults),
        "cases": cases,
        "fault_cases": faults,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    config_path, runner, output = arguments.config.resolve(), arguments.runner.resolve(), arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text())
    summary = run(config, runner, output)
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    outputs = sorted(path for path in output.iterdir() if path.is_file())
    source_inputs = [
        ROOT / "ros_ws/src/wheel_leg_core/include/wheel_leg_core/controller_core.hpp",
        ROOT / "ros_ws/src/wheel_leg_core/src/controller_core.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/adapter.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/planar_standing_loop.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/CMakeLists.txt",
    ]
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "config_sha256": sha256(config_path),
        "runner_sha256": sha256(runner),
        "wrapper_sha256": sha256(Path(__file__).resolve()),
        "scene_sha256": sha256((ROOT / config["scene"]).resolve()),
        "source_inputs": {
            str(path.relative_to(ROOT)): sha256(path) for path in source_inputs
        },
        "outputs": {path.name: sha256(path) for path in outputs},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
