#!/usr/bin/env python3
"""Evaluate the frozen Phase 34 step/ramp tracking screen."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GAINS = ROOT / "simulation/mujoco/config/phase34_xi_tracking_gains_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gains", type=Path, default=DEFAULT_GAINS)
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    gains_path = args.gains.resolve()
    gains = json.loads(gains_path.read_text(encoding="utf-8"))
    screen = gains["screen"]
    raw = args.raw.resolve()
    statuses = list(csv.DictReader((raw / "run-status.csv").open()))
    expected = {(gain["id"], profile)
                for gain in gains["candidate_sets"] for profile in ("step", "ramp")}
    observed = {(row["gain_id"], row["profile"]) for row in statuses}
    details = []
    gain_passes: dict[str, bool] = {}
    source_paths = [gains_path, Path(__file__).resolve(), raw / "run-status.csv"]
    for row in statuses:
        gain_id = row["gain_id"]
        profile = row["profile"]
        csv_path = raw / f"{gain_id}-{profile}.csv"
        stderr_path = raw / f"{gain_id}-{profile}.stderr.txt"
        source_paths.extend([csv_path, stderr_path])
        samples = list(csv.DictReader(csv_path.open()))
        target_samples = [sample for sample in samples if int(sample["tick"]) >= 50]
        final_error = abs(float(samples[-1]["common_error"]))
        maximum_differential_error = max(
            abs(float(sample["differential_error"])) for sample in target_samples)
        settled_at = None
        for index, sample in enumerate(target_samples):
            if all(abs(float(tail["common_error"])) <=
                   float(screen["maximum_final_common_error_m"])
                   for tail in target_samples[index:]):
                settled_at = float(sample["time_s"]) - 0.5
                break
        torque_limits = np.asarray(screen["torque_limit_nm"])
        torque_violation = max(0.0, max(
            float(np.max(np.abs([float(sample[f"tau{joint}"]) for joint in range(6)]) -
                         torque_limits)) for sample in samples))
        gates = {
            "completed": int(row["exit_code"]) == 0 and len(samples) == 150,
            "final_common_error": final_error <= float(
                screen["maximum_final_common_error_m"]),
            "settling_time": settled_at is not None and settled_at <= float(
                screen["maximum_settling_time_s"]),
            "differential_hold": maximum_differential_error <= float(
                screen["maximum_differential_drift_m"]),
            "bilateral_contact": all(
                sample["contact_left"] == "1" and sample["contact_right"] == "1"
                for sample in samples),
            "hard_constraints": max(float(sample["hard"]) for sample in samples) <=
                float(screen["maximum_hard_violation"]),
            "deadline": max(float(sample["wbc_time_s"]) for sample in samples) <=
                float(screen["maximum_wbc_time_s"]),
            "slack": max(float(sample["max_normalized_slack"]) for sample in samples) <=
                float(screen["maximum_normalized_slack"]),
            "torque": torque_violation <= 1e-10,
        }
        passed = all(gates.values())
        gain_passes[gain_id] = gain_passes.get(gain_id, True) and passed
        details.append({
            "gain_id": gain_id,
            "profile": profile,
            "exit_code": int(row["exit_code"]),
            "stderr": stderr_path.read_text(encoding="utf-8").strip(),
            "sample_count": len(samples),
            "last_tick": int(samples[-1]["tick"]),
            "final_available_common_error_m": final_error,
            "maximum_differential_error_m": maximum_differential_error,
            "settling_time_s": settled_at,
            "maximum_hard_violation": max(float(sample["hard"]) for sample in samples),
            "maximum_wbc_time_s": max(float(sample["wbc_time_s"]) for sample in samples),
            "maximum_normalized_slack": max(
                float(sample["max_normalized_slack"]) for sample in samples),
            "maximum_torque_limit_violation_nm": torque_violation,
            "gates": gates,
            "pass": passed,
        })
    any_gain_pass = observed == expected and any(gain_passes.values())
    summary = {
        "classification": "P34-E_wheel_tracking_failure",
        "tracking_pass": any_gain_pass,
        "blocking_finding": None if any_gain_pass else "P34-E",
        "expected_run_count": len(expected),
        "observed_run_count": len(observed),
        "gain_passes": gain_passes,
        "all_runs_rejected_outside_workspace": all(
            item["stderr"] == "measurement WBC model failed: status=2"
            for item in details),
        "earliest_rejection_tick": min(item["last_tick"] + 1 for item in details),
        "latest_rejection_tick": max(item["last_tick"] + 1 for item in details),
        "minimum_final_available_common_error_m": min(
            item["final_available_common_error_m"] for item in details),
        "maximum_final_available_common_error_m": max(
            item["final_available_common_error_m"] for item in details),
    }
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "inputs": {str(path.relative_to(ROOT)): sha256(path)
                   for path in sorted(set(source_paths))},
    }
    output.mkdir(parents=True)
    (output / "details.json").write_text(json.dumps(details, indent=2) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return 0 if any_gain_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
