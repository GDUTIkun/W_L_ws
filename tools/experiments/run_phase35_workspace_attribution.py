#!/usr/bin/env python3
"""Run and analyze the frozen Phase 35 workspace-attribution corpus."""

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

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase35_workspace_attribution_v1.json"
DEFAULT_EXECUTABLE = ROOT / "ros_ws/build/wheel_leg_mujoco/phase35_workspace_attribution_loop"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def first_sustained(values: list[float], window: int, negative: int,
                    minimum_loss: float) -> int | None:
    for start in range(len(values) - window + 1):
        sample = values[start:start + window]
        decreases = sum(sample[index + 1] < sample[index]
                        for index in range(window - 1))
        if decreases >= negative and sample[0] - sample[-1] >= minimum_loss:
            return start
    return None


def analyze_case(case_rows: list[dict[str, str]], config: dict) -> dict:
    widths = config["workspace_half_width_rad"]
    trend = config["trend"]
    failure = next((row for row in case_rows if int(row["model_status"]) == 2), None)
    trends = []
    for joint, width in enumerate(widths):
        normalized = [float(row[f"signed_margin{joint}"]) / width for row in case_rows]
        onset = first_sustained(normalized, trend["window_ticks"],
                                trend["minimum_negative_steps"],
                                trend["minimum_normalized_loss"])
        near = next((index for index, value in enumerate(normalized)
                     if value <= trend["near_boundary_normalized_margin"]), None)
        trends.append({"joint": joint, "trend_tick": onset, "near_boundary_tick": near,
                       "reset_normalized_margin": normalized[0],
                       "last_normalized_margin": normalized[-1]})
    finite_rejecting_envelope = failure is not None and all(
        math.isfinite(float(failure[name]))
        for joint in range(6)
        for name in (f"q{joint}", f"dq{joint}", f"delta{joint}",
                     f"lower_margin{joint}", f"upper_margin{joint}",
                     f"signed_margin{joint}"))
    finite_rejecting_geometry = failure is not None and all(
        math.isfinite(float(failure[f"raw_wheel_{kind}{side}_{axis}"]))
        for kind in ("p", "v") for side in range(2) for axis in range(3))
    valid_rows = [row for row in case_rows if int(row["model_status"]) == 0]
    return {
        "sample_count": len(case_rows),
        "failure_tick": int(failure["tick"]) if failure else None,
        "first_failed_index": int(failure["first_failed_index"]) if failure else None,
        "minimum_margin_index": int(failure["minimum_margin_index"]) if failure else None,
        "rejecting_signed_margin_rad": (
            float(failure[f"signed_margin{failure['first_failed_index']}"])
            if failure else None),
        "finite_rejecting_envelope": finite_rejecting_envelope,
        "finite_rejecting_geometry": finite_rejecting_geometry,
        "workspace_inspector_parity": all(
            (int(row["model_status"]) == 2) == (int(row["first_failed_index"]) >= 0)
            for row in case_rows),
        "trends": trends,
        "maximum_hard_violation_before_failure": max(
            (float(row["hard"]) for row in valid_rows), default=0.0),
        "maximum_normalized_slack_before_failure": max(
            (float(row["maximum_normalized_slack"]) for row in valid_rows), default=0.0),
        "minimum_torque_margin_before_failure_nm": min(
            (float(row[f"tau_margin{joint}"])
             for row in valid_rows for joint in range(6)), default=math.inf),
        "bilateral_contact_before_failure": all(
            row["contact_left"] == "1" and row["contact_right"] == "1"
            for row in valid_rows),
    }


def replay_error(first: list[dict[str, str]], second: list[dict[str, str]]) -> float:
    ignored = {"wbc_time_s"}
    maximum = 0.0
    if len(first) != len(second):
        return math.inf
    for left, right in zip(first, second):
        for key in left:
            if key in ignored:
                continue
            try:
                a, b = float(left[key]), float(right[key])
            except ValueError:
                if left[key] != right[key]:
                    return math.inf
                continue
            if math.isnan(a) and math.isnan(b):
                continue
            maximum = max(maximum, abs(a - b))
    return maximum


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--executable", type=Path, default=DEFAULT_EXECUTABLE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--supersedes")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    config_path = args.config.resolve()
    executable = args.executable.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    gains_path = (ROOT / config["source_gains"]).resolve()
    gains = json.loads(gains_path.read_text(encoding="utf-8"))
    scene = (ROOT / config["scene"]).resolve()
    gain_by_id = {item["id"]: item for item in gains["candidate_sets"]}
    output.mkdir(parents=True)

    run_specs = [
        ("H0_minimal_hold", "none", 0.0, 0.0, "H0-a"),
        ("H0_minimal_hold", "none", 0.0, 0.0, "H0-b"),
        ("H1_zero_ddxi_row", "none", 0.0, 0.0, "H1-a"),
        ("H1_zero_ddxi_row", "none", 0.0, 0.0, "H1-b"),
    ]
    for gain_id, gain in gain_by_id.items():
        for profile in ("step", "ramp"):
            run_specs.append((f"tracking_{profile}_{gain_id}", gain_id,
                              gain["kp_s2"], gain["kd_s"],
                              f"tracking-{gain_id}-{profile}"))
    commands = []
    for case_id, gain_id, kp, kd, stem in run_specs:
        csv_path = output / f"{stem}.csv"
        command = [str(executable), str(scene), str(csv_path), case_id, gain_id,
                   repr(kp), repr(kd)]
        subprocess.run(command, cwd=ROOT, check=True)
        commands.append(" ".join(command))

    analyses = {stem: analyze_case(rows(output / f"{stem}.csv"), config)
                for *_, stem in run_specs}
    h0_replay = replay_error(rows(output / "H0-a.csv"), rows(output / "H0-b.csv"))
    h1_replay = replay_error(rows(output / "H1-a.csv"), rows(output / "H1-b.csv"))
    tracking = [value for key, value in analyses.items() if key.startswith("tracking-")]
    tracking_ticks = [item["failure_tick"] for item in tracking]
    h0 = analyses["H0-a"]
    gates = config["gates"]
    summary = {
        "classification": "P35-A_pre_target_minimal_wbc_workspace_drift"
        if h0["failure_tick"] is not None else "P35-U_unresolved",
        "limiting_joint": "right_wheel" if h0["first_failed_index"] == 5 else None,
        "activation_branch": "Phase27 Minimal fixed equilibrium-wrench hold",
        "h0_failure_tick": h0["failure_tick"],
        "h0_first_failed_index": h0["first_failed_index"],
        "h0_replay_max_abs_error": h0_replay,
        "h1_replay_max_abs_error": h1_replay,
        "tracking_failure_ticks": tracking_ticks,
        "phase34_timing_reproduced": all(
            gates["phase34_first_failure_tick_min"] <= tick <=
            gates["phase34_first_failure_tick_max"] for tick in tracking_ticks),
        "inspector_parity": all(item["workspace_inspector_parity"]
                                for item in analyses.values()),
        "rejecting_envelope_complete": all(item["finite_rejecting_envelope"]
                                             for item in analyses.values()),
        "rejecting_raw_geometry_complete": all(item["finite_rejecting_geometry"]
                                                 for item in analyses.values()),
        "upstream_contact_loss": not h0["bilateral_contact_before_failure"],
        "upstream_hard_violation": h0["maximum_hard_violation_before_failure"] >
            gates["maximum_hard_violation"],
        "upstream_slack_violation": h0["maximum_normalized_slack_before_failure"] >
            gates["maximum_normalized_slack"],
        "upstream_torque_limit": h0["minimum_torque_margin_before_failure_nm"] <
            gates["minimum_torque_margin_nm"],
        "direct_and_h2_causally_ineligible": h0["failure_tick"] is not None,
    }
    source_paths = [config_path, gains_path, scene,
                    ROOT / "simulation/mujoco/model/wheel_leg.xml", executable,
                    Path(__file__).resolve(),
                    ROOT / "ros_ws/src/wheel_leg_core/include/wheel_leg_core/nominal_wbc_model.hpp",
                    ROOT / "ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp",
                    ROOT / "ros_ws/src/wheel_leg_mujoco/src/phase35_workspace_attribution_loop.cpp"]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "python": platform.python_version(),
        "commands": commands,
        "inputs": {str(path.relative_to(ROOT)): sha256(path)
                   if path.is_relative_to(ROOT) else sha256(path)
                   for path in source_paths},
        "supersedes": args.supersedes,
        "source_phase34": "xi-tracking-formal-v3",
    }
    (output / "details.json").write_text(json.dumps(analyses, indent=2) + "\n")
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    passed = (summary["classification"].startswith("P35-A") and
              summary["phase34_timing_reproduced"] and summary["inspector_parity"] and
              summary["rejecting_envelope_complete"] and
              summary["rejecting_raw_geometry_complete"] and
              h0_replay <= gates["replay_max_abs_error"] and
              h1_replay <= gates["replay_max_abs_error"])
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
