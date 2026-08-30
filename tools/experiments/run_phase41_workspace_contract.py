#!/usr/bin/env python3
"""Phase 41 production R3 workspace-contract H0 revalidation."""

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

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase41_workspace_contract_v1.json"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quaternion_angle(row: dict[str, str], initial: dict[str, str]) -> float:
    q = [float(row[f"base_q{i}"]) for i in range(4)]
    q0 = [float(initial[f"base_q{i}"]) for i in range(4)]
    return 2.0 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(q, q0)))))


def row_failures(
    row: dict[str, str], initial: dict[str, str], thresholds: dict[str, float]
) -> list[str]:
    failures = []
    if row["contact_left"] != "1":
        failures.append("left_contact_loss")
    if row["contact_right"] != "1":
        failures.append("right_contact_loss")
    if any(int(row[name]) != 0 for name in
           ("model_status", "controller_status", "solver_status")):
        failures.append("model_controller_solver")
    if float(row["hard"]) > thresholds["maximum_hard_violation"]:
        failures.append("hard")
    if float(row["maximum_normalized_slack"]) > thresholds["maximum_normalized_slack"]:
        failures.append("slack")
    if min(float(row[f"tau_margin{joint}"]) for joint in range(6)) < thresholds["minimum_torque_margin_nm"]:
        failures.append("torque")
    position_change = math.sqrt(sum(
        (float(row[f"base_p{i}"]) - float(initial[f"base_p{i}"])) ** 2
        for i in range(3)))
    if position_change > thresholds["base_position_change_m"]:
        failures.append("base_position")
    if quaternion_angle(row, initial) > thresholds["base_rotation_change_rad"]:
        failures.append("base_rotation")
    if math.sqrt(sum(float(row[f"base_v{i}"]) ** 2 for i in range(3))) > thresholds["base_linear_speed_m_s"]:
        failures.append("base_linear_speed")
    if math.sqrt(sum(float(row[f"base_omega{i}"]) ** 2 for i in range(3))) > thresholds["base_angular_speed_rad_s"]:
        failures.append("base_angular_speed")
    return failures


def semantic_error(
    first: list[dict[str, str]], second: list[dict[str, str]], ignored: set[str]
) -> float:
    if len(first) != len(second) or (first and first[0].keys() != second[0].keys()):
        return math.inf
    maximum = 0.0
    for left, right in zip(first, second):
        for key in left:
            if key in ignored:
                continue
            try:
                a, b = float(left[key]), float(right[key])
                if math.isnan(a) and math.isnan(b):
                    continue
                maximum = max(maximum, abs(a - b))
            except ValueError:
                if left[key] != right[key]:
                    return math.inf
    return maximum


def analyze(rows: list[dict[str, str]], thresholds: dict[str, float]) -> dict[str, Any]:
    if not rows:
        raise RuntimeError("empty H0 output")
    initial = rows[0]
    crossing = next((int(row["tick"]) for row in rows
                     if abs(float(row["delta2"])) > 1.0 or
                     abs(float(row["delta5"])) > 1.0), None)
    failure_index = next((index for index, row in enumerate(rows)
                          if row_failures(row, initial, thresholds)), None)
    valid_rows = rows if failure_index is None else rows[:failure_index]
    maximum_rotation = max(
        max(abs(float(row[f"raw_q{joint}"]) - float(initial[f"raw_q{joint}"]))
            for joint in (2, 5)) for row in rows)
    return {
        "old_bound_crossing_tick": crossing,
        "crossing_model_status": None if crossing is None else
            int(next(row for row in rows if int(row["tick"]) == crossing)["model_status"]),
        "crossing_continued": crossing is not None and int(rows[-1]["tick"]) > crossing,
        "first_independent_failure_tick": None if failure_index is None else
            int(rows[failure_index]["tick"]),
        "first_independent_failures": [] if failure_index is None else
            row_failures(rows[failure_index], initial, thresholds),
        "valid_before_failure": all(not row_failures(row, initial, thresholds)
                                    for row in valid_rows),
        "final_tick": int(rows[-1]["tick"]),
        "maximum_wheel_rotation_rad": maximum_rotation,
        "maximum_wheel_revolutions": maximum_rotation / (2.0 * math.pi),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    output.mkdir(parents=True)
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    phase40_config = json.loads((ROOT / config["phase40_config"]).read_text(encoding="utf-8"))
    thresholds = phase40_config["thresholds"]
    executable = ROOT / config["executable"]
    outputs = [output / "H0-production-a.csv", output / "H0-production-b.csv"]
    command = [str(executable), str(ROOT / config["scene"]), "OUTPUT",
               config["case"], config["gain"], str(config["kp"]), str(config["kd"])]
    for path in outputs:
        actual = command.copy()
        actual[2] = str(path)
        subprocess.run(actual, cwd=ROOT, check=True)
    rows_a, rows_b = read_csv(outputs[0]), read_csv(outputs[1])
    analysis = analyze(rows_a, thresholds)
    replay_error = semantic_error(rows_a, rows_b, {"wbc_time_s"})
    shadow_rows = read_csv(ROOT / config["phase40_shadow_csv"])
    shadow_error = semantic_error(
        rows_a, shadow_rows,
        {"wbc_time_s", "gain_id", "minimum_margin_index", "first_failed_index"})
    phase40 = json.loads((ROOT / config["phase40_summary"]).read_text(encoding="utf-8"))
    expected = {
        "old_bound_crossing": analysis["old_bound_crossing_tick"] ==
            int(config["expected_old_bound_crossing_tick"]),
        "crossing_not_rejected": analysis["crossing_model_status"] == 0 and
            analysis["crossing_continued"],
        "failure_tick": analysis["first_independent_failure_tick"] ==
            int(config["expected_failure_tick"]),
        "failure_kind": analysis["first_independent_failures"] ==
            [config["expected_failure"]],
        "valid_before_failure": analysis["valid_before_failure"],
        "fresh_replay": replay_error <= float(config["semantic_tolerance"]),
        "phase40_shadow_semantics": shadow_error <= float(config["semantic_tolerance"]),
        "phase40_shadow_decision":
            phase40["shadow_h0"]["stop_tick"] == analysis["first_independent_failure_tick"],
    }
    passed = all(expected.values())
    summary = {
        "classification": "P41-A_workspace_contract_corrected_contact_loss_reproduced"
        if passed else "P41-B_production_shadow_semantic_mismatch",
        "pass": passed,
        "contract": "leg_workspace_enforced_wheel_q_finite_only",
        "analysis": analysis,
        "gates": expected,
        "formal_replay_max_abs_error": replay_error,
        "phase40_shadow_physical_control_max_abs_error": shadow_error,
        "contact_loss_repair": False,
        "phase34_tracking_run": False,
    }
    write_json(output / "summary.json", summary)
    write_json(output / "details.json", {"command": command, "thresholds": thresholds})
    inputs = [config_path, ROOT / config["scene"], executable,
              ROOT / config["phase40_config"], ROOT / config["phase40_summary"],
              ROOT / config["phase40_shadow_csv"], Path(__file__).resolve()]
    write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": args.replay_of,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
    })
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
