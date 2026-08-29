#!/usr/bin/env python3
"""Evaluate the Phase-27 2/10/20 versus 1/5/20 timing corpus."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = (
    ROOT / "docs/workflow/phases/27-theory-restored-minimal-wbc/evidence/"
    "automated/timing-comparison-v1"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def exact_replay(left: Path, right: Path, ignored: set[str]) -> bool:
    left_rows, right_rows = rows(left), rows(right)
    return len(left_rows) == len(right_rows) and all(
        all(a[key] == b[key] for key in a if key not in ignored)
        for a, b in zip(left_rows, right_rows)
    )


def metrics(control_path: Path, plant_path: Path, period_s: float) -> dict[str, object]:
    control, plant = rows(control_path), rows(plant_path)
    core_ms = np.array([float(row["core_step_ns"]) * 1e-6 for row in control])
    dt = np.array([float(row["dt_s"]) for row in control[1:]])
    result = {
        "control_rows": len(control),
        "plant_rows": len(plant),
        "maximum_core_step_ms": float(np.max(core_ms)),
        "p99_core_step_ms": float(np.percentile(core_ms, 99)),
        "deadline_miss_ratio": float(np.mean(core_ms > 1000.0 * period_s)),
        "maximum_period_error_s": float(np.max(np.abs(dt - period_s))),
        "minimum_wheel_normal_load_n": min(
            min(float(row["left_normal_load_n"]), float(row["right_normal_load_n"]))
            for row in plant
        ),
        "maximum_penetration_m": max(float(row["penetration_m"]) for row in plant),
        "maximum_abs_rolling_slip_m_s": max(abs(float(row["rolling_slip_m_s"])) for row in plant),
        "maximum_abs_lateral_slip_m_s": max(abs(float(row["lateral_slip_m_s"])) for row in plant),
        "maximum_closure_residual_m": max(float(row["closure_residual_m"]) for row in plant),
        "maximum_abs_base_x_m": max(abs(float(row["qpos0"])) for row in plant),
        "all_control_ok": all(
            row["status"] == "0" and row["latch"] == "0"
            and row["accepted"] == "1" and row["weighted_status"] == "0"
            and row["model_status"] == "0" and row["solver_status"] == "0"
            and row["contact_left"] == "2" and row["contact_right"] == "2"
            for row in control
        ),
        "zoh_exact": all(float(row["zoh_diff"]) == 0.0 for row in control),
        "finite": all(
            np.isfinite(float(value))
            for row in control + plant for value in row.values()
            if value not in ("hold", "")
        ),
    }
    result["pass"] = (
        result["all_control_ok"] and result["zoh_exact"] and result["finite"]
        and result["maximum_period_error_s"] <= 1e-6
        and result["deadline_miss_ratio"] == 0.0
        and result["minimum_wheel_normal_load_n"] >= 1.0
        and result["maximum_penetration_m"] <= 0.004
        and result["maximum_abs_rolling_slip_m_s"] <= 0.05
        and result["maximum_abs_lateral_slip_m_s"] <= 0.05
        and result["maximum_closure_residual_m"] <= 0.0002
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", type=Path, default=DEFAULT_INPUT)
    args = parser.parse_args()
    input_dir = args.input_dir.resolve()
    summary_path = input_dir / "summary.json"
    manifest_path = input_dir / "manifest.json"
    if summary_path.exists() or manifest_path.exists():
        raise RuntimeError("Refusing to overwrite timing summary/manifest")
    profiles = {
        "baseline_2_10_20": {"prefix": "baseline", "period_s": 0.01},
        "candidate_1_5_20": {"prefix": "candidate", "period_s": 0.005},
    }
    results = {}
    for profile, spec in profiles.items():
        prefix = spec["prefix"]
        results[profile] = {
            "hold": metrics(
                input_dir / f"{prefix}_hold_control.csv",
                input_dir / f"{prefix}_hold_plant.csv",
                spec["period_s"],
            ),
            "disturbance": metrics(
                input_dir / f"{prefix}_disturbance_control.csv",
                input_dir / f"{prefix}_disturbance_plant.csv",
                spec["period_s"],
            ),
            "replay_control_exact_except_wall_clock": exact_replay(
                input_dir / f"{prefix}_hold_control.csv",
                input_dir / f"{prefix}_hold_replay_control.csv",
                {"core_step_ns"},
            ),
            "replay_plant_exact": exact_replay(
                input_dir / f"{prefix}_hold_plant.csv",
                input_dir / f"{prefix}_hold_replay_plant.csv",
                set(),
            ),
        }
        results[profile]["pass"] = (
            results[profile]["hold"]["pass"]
            and results[profile]["disturbance"]["pass"]
            and results[profile]["replay_control_exact_except_wall_clock"]
            and results[profile]["replay_plant_exact"]
        )
    all_profiles_pass = all(result["pass"] for result in results.values())
    summary = {
        "schema_version": 1,
        "phase": 27,
        "profile": f"phase27_{input_dir.name.replace('-', '_')}",
        "supersedes": (
            "timing-comparison-v1" if input_dir.name == "timing-comparison-v2"
            else None
        ),
        "profiles": results,
        "decision": "baseline_2_10_20" if all_profiles_pass else "undecided",
        "decision_reason": (
            "Both profiles pass identical plant/control gates; 1/5/20 doubles "
            "physics and WBC load without a demonstrated gate improvement. "
            "Retain the approved 2/10/20 schedule and audit the new 16-state "
            "NMPC against its 20 ms update and 10 ms combined deadline in T06."
            if all_profiles_pass else
            "At least one timing profile failed; no schedule decision is authorized."
        ),
        "pass": all_profiles_pass,
    }
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    source_paths = [
        Path(__file__).resolve(),
        ROOT / "ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp",
        ROOT / "simulation/mujoco/model/phase18_floating_contact.xml",
        ROOT / "simulation/mujoco/config/phase21_weighted_wbc_formal_v1.json",
    ]
    output_paths = sorted(path for path in input_dir.iterdir() if path.is_file())
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "hardware_data": False,
        "sources": {str(path.relative_to(ROOT)): sha256(path) for path in source_paths},
        "outputs": {path.name: sha256(path) for path in output_paths},
    }
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}")
        raise SystemExit(2)
