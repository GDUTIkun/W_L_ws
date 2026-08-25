#!/usr/bin/env python3
"""Run the Phase 16 deterministic Controller↔MuJoCo validation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase16_nominal.json"
DEFAULT_RUNNER = (
    ROOT
    / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/deterministic_loop"
)
DEFAULT_OUTPUT = (
    ROOT
    / "docs/workflow/phases/16-controller-mujoco-deterministic-loop/evidence"
    / "automated/2026-08-25-nominal"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def run_cpp(
    runner: Path,
    scene: Path,
    output: Path,
    scenario: str,
    episodes: int,
    ticks: int,
    physics_steps: int,
) -> None:
    subprocess.run(
        [
            str(runner),
            "--model",
            str(scene),
            "--output",
            str(output),
            "--scenario",
            scenario,
            "--episodes",
            str(episodes),
            "--ticks",
            str(ticks),
            "--physics-steps-per-control",
            str(physics_steps),
        ],
        cwd=ROOT,
        check=True,
    )


def numeric_columns(rows: list[dict[str, str]]) -> list[str]:
    return [
        name
        for name in rows[0]
        if name not in {"scenario", "fault_event"}
    ]


def max_numeric_difference(
    first: list[dict[str, str]], second: list[dict[str, str]], ignore: set[str]
) -> float:
    maximum = 0.0
    for left, right in zip(first, second, strict=True):
        for name in numeric_columns(first):
            if name not in ignore:
                maximum = max(maximum, abs(float(left[name]) - float(right[name])))
    return maximum


def all_finite(rows: list[dict[str, str]]) -> bool:
    return all(
        math.isfinite(float(row[name]))
        for row in rows
        for name in numeric_columns(rows)
    )


def evaluate_nominal(
    rows: list[dict[str, str]], config: dict[str, Any]
) -> tuple[dict[str, bool], dict[str, float | int | str]]:
    episodes = int(config["nominal"]["episodes"])
    ticks = int(config["nominal"]["ticks_per_episode"])
    physics_steps = int(config["physics_steps_per_control"])
    period_ns = int(round(float(config["control_period_s"]) * 1.0e9))
    grouped = [
        [row for row in rows if int(row["episode"]) == episode]
        for episode in range(episodes)
    ]
    replay_diff = max_numeric_difference(grouped[0], grouped[1], {"episode"})
    tau_names = [f"tau_{index}" for index in range(6)]
    ctrl_names = [f"ctrl_{index}" for index in range(6)]
    max_torque = max(abs(float(row[name])) for row in rows for name in tau_names)
    max_ctrl = max(abs(float(row[name])) for row in rows for name in ctrl_names)
    max_zoh = max(float(row["zoh_ctrl_max_difference"]) for row in rows)
    thresholds = config["thresholds"]
    checks = {
        "row_count": len(rows) == episodes * ticks,
        "finite": all_finite(rows),
        "all_core_steps_accepted": all(row["core_status"] == "0" for row in rows),
        "all_commands_accepted": all(
            row["command_attempted"] == "1" and row["command_accepted"] == "1"
            for row in rows
        ),
        "source_and_receipt_time": all(
            int(row["source_time_ns"]) == int(row["tick"]) * period_ns
            and int(row["receipt_time_ns"]) == int(row["source_time_ns"])
            for row in rows
        ),
        "controller_dt": all(
            float(row["dt_s"]) == (0.0 if int(row["tick"]) == 0 else 0.01)
            for row in rows
        ),
        "physics_tick_accounting": all(
            int(row["physics_begin"]) == int(row["tick"]) * physics_steps
            and int(row["physics_end"])
            == (int(row["tick"]) + 1) * physics_steps
            for row in rows
        ),
        "contact_disabled": all(
            row["contact_left"] == "1" and row["contact_right"] == "1"
            for row in rows
        ),
        "zero_core_torque": max_torque
        <= float(thresholds["max_abs_nominal_torque_nm"]),
        "zero_native_ctrl": max_ctrl
        <= float(thresholds["max_abs_nominal_ctrl_nm"]),
        "zoh_exact": max_zoh
        <= float(thresholds["max_zoh_ctrl_difference_nm"]),
        "reset_replay_exact": replay_diff
        <= float(thresholds["max_replay_numeric_difference"]),
    }
    metrics: dict[str, float | int | str] = {
        "rows": len(rows),
        "episodes": episodes,
        "ticks_per_episode": ticks,
        "max_abs_core_torque_nm": max_torque,
        "max_abs_native_ctrl_nm": max_ctrl,
        "max_zoh_ctrl_difference_nm": max_zoh,
        "reset_replay_max_numeric_difference": replay_diff,
    }
    return checks, metrics


def evaluate_faults(
    rows: list[dict[str, str]], config: dict[str, Any]
) -> dict[str, bool]:
    by_key = {(int(row["episode"]), int(row["tick"])): row for row in rows}
    faults = config["faults"]
    duplicate_tick = int(faults["duplicate_state_tick"])
    future_tick = int(faults["future_command_tick"])
    stale_tick = int(faults["stale_command_tick"])
    seed_tick = int(faults["timeout_seed_tick"])
    timeout_tick = int(faults["receipt_timeout_tick"])
    recovery_tick = int(faults["recovery_tick"])
    reset_key = (
        int(faults["reset_old_command_episode"]),
        int(faults["reset_old_command_tick"]),
    )
    ctrl = lambda row: float(row["ctrl_0"])
    checks = {
        "duplicate_state_rejected": by_key[(0, duplicate_tick)]["probe_status"]
        == "3",
        "future_command_rejected": (
            by_key[(0, future_tick)]["fault_event"] == "future_command"
            and by_key[(0, future_tick)]["command_accepted"] == "0"
            and ctrl(by_key[(0, future_tick)]) == 0.0
        ),
        "stale_command_rejected": (
            by_key[(0, stale_tick)]["fault_event"] == "stale_command"
            and by_key[(0, stale_tick)]["command_accepted"] == "0"
            and ctrl(by_key[(0, stale_tick)]) == 0.0
        ),
        "timeout_seed_applied": (
            by_key[(0, seed_tick)]["fault_event"] == "timeout_seed"
            and by_key[(0, seed_tick)]["injected_torque_nm"] == "1"
            and ctrl(by_key[(0, seed_tick)]) == -1.0
        ),
        "receipt_timeout_fails_to_zero": (
            by_key[(0, timeout_tick)]["fault_event"] == "receipt_timeout"
            and by_key[(0, timeout_tick)]["command_attempted"] == "0"
            and ctrl(by_key[(0, timeout_tick)]) == 0.0
        ),
        "post_timeout_recovery": (
            by_key[(0, recovery_tick)]["fault_event"] == "recovery"
            and by_key[(0, recovery_tick)]["command_accepted"] == "1"
            and ctrl(by_key[(0, recovery_tick)]) == 0.0
        ),
        "reset_old_command_rejected_and_new_epoch_recovers": (
            by_key[reset_key]["fault_event"] == "reset_old_recovery"
            and by_key[reset_key]["probe_command_attempted"] == "1"
            and by_key[reset_key]["probe_command_accepted"] == "0"
            and by_key[reset_key]["command_accepted"] == "1"
            and ctrl(by_key[reset_key]) == 0.0
        ),
        "fault_core_output_remains_zero": all(
            float(row[f"tau_{joint}"]) == 0.0
            for row in rows
            for joint in range(6)
        ),
        "fault_log_finite": all_finite(rows),
        "fault_zoh_exact": all(
            float(row["zoh_ctrl_max_difference"]) == 0.0 for row in rows
        ),
    }
    return checks


def command_first_line(command: list[str]) -> str:
    result = subprocess.run(command, check=True, text=True, capture_output=True)
    return result.stdout.splitlines()[0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="nominal")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = args.config.resolve()
    runner = args.runner.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config["profile"] != args.profile:
        raise SystemExit("--profile does not match config profile")
    scene = (ROOT / config["scene"]).resolve()
    ratio = float(config["control_period_s"]) / float(config["physics_timestep_s"])
    if ratio != int(ratio) or int(ratio) != int(config["physics_steps_per_control"]):
        raise SystemExit("control_period_s must be an integer multiple of physics_timestep_s")
    for path in (runner, scene):
        if not path.is_file():
            raise SystemExit(f"Required input is missing: {path}")

    nominal = config["nominal"]
    faults = config["faults"]
    nominal_a = output_dir / "nominal_a.csv"
    nominal_b = output_dir / "nominal_b.csv"
    fault_path = output_dir / "faults.csv"
    for path in (nominal_a, nominal_b):
        run_cpp(
            runner,
            scene,
            path,
            "nominal",
            int(nominal["episodes"]),
            int(nominal["ticks_per_episode"]),
            int(config["physics_steps_per_control"]),
        )
    run_cpp(
        runner,
        scene,
        fault_path,
        "faults",
        int(faults["episodes"]),
        int(faults["ticks_per_episode"]),
        int(config["physics_steps_per_control"]),
    )

    rows_a = read_rows(nominal_a)
    rows_b = read_rows(nominal_b)
    nominal_checks, nominal_metrics = evaluate_nominal(rows_a, config)
    fault_checks = evaluate_faults(read_rows(fault_path), config)
    cross_process_diff = max_numeric_difference(rows_a, rows_b, set())
    checks = {
        **nominal_checks,
        **fault_checks,
        "fresh_process_csv_exact": nominal_a.read_bytes() == nominal_b.read_bytes(),
        "fresh_process_numeric_exact": cross_process_diff
        <= float(config["thresholds"]["max_replay_numeric_difference"]),
    }
    nominal_metrics["fresh_process_max_numeric_difference"] = cross_process_diff
    nominal_metrics["fresh_process_sha256"] = sha256(nominal_a)

    source_inputs = [
        config_path,
        scene,
        ROOT / "simulation/mujoco/model/wheel_leg.xml",
        ROOT / "ros_ws/src/wheel_leg_core/src/controller_core.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/adapter.cpp",
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/deterministic_loop.cpp",
        Path(__file__).resolve(),
        runner,
    ]
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "profile": config["profile"],
        "model_revision": config["model_revision"],
        "hardware_data_used": False,
        "versions": {
            "runner": command_first_line([str(runner), "--version"]),
            "python": platform.python_version(),
            "compiler": command_first_line([os.environ.get("CXX", "c++"), "--version"]),
            "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        },
        "timing": {
            "physics_timestep_s": config["physics_timestep_s"],
            "control_period_s": config["control_period_s"],
            "physics_steps_per_control": config["physics_steps_per_control"],
        },
        "inputs": {
            str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path): sha256(path)
            for path in source_inputs
        },
        "outputs": {
            path.name: sha256(path) for path in (nominal_a, nominal_b, fault_path)
        },
    }
    summary = {
        "schema_version": 1,
        "overall_pass": all(checks.values()),
        "hardware_data_used": False,
        "checks": checks,
        "metrics": nominal_metrics,
        "interpretation_limit": (
            "Simulation-only execution, clock, reset, fail-safe, logging, and replay "
            "evidence. No PD, contact-fidelity, realtime, or real-hardware claim."
        ),
    }
    (output_dir / "run_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output_dir / "phase16_validation.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
