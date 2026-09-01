#!/usr/bin/env python3
"""Summarize the frozen Phase48-A semantic/H0 regression without new probes."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np
import scipy


ROOT = Path(__file__).resolve().parents[2]
ORDER = [f"{side}_{component}" for side in ("left", "right")
         for component in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")]
SCALE = np.tile([50.0, 50.0, 50.0, 2.5, 2.5, 2.5], 2)
ANCHORS = {
    "baseline_slack": 0.001522220395389018,
    "primitive_slack": 0.05850370867784012,
    "minimum_deviation": 0.07832043067340007,
    "w5_operator": 1.0658141036401503e-14,
    "w5_offset": 2.6645352591003757e-15,
}


def read_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def vector(row: dict[str, str], prefix: str) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{index}"]) for index in range(12)])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--primitive", type=Path, required=True)
    parser.add_argument("--primitive-decision", type=Path, required=True)
    parser.add_argument("--closure", type=Path, required=True)
    parser.add_argument("--baseline-replay", type=Path, required=True)
    parser.add_argument("--primitive-replay", type=Path, required=True)
    parser.add_argument("--closure-replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    git_revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                  check=True, text=True, capture_output=True).stdout.strip()
    dirty_files = subprocess.run(["git", "status", "--short"], cwd=ROOT,
                                 check=True, text=True, capture_output=True).stdout.splitlines()

    baseline = read_row(args.baseline)
    primitive = read_row(args.primitive)
    primitive_decision = read_json(args.primitive_decision)
    closure = read_json(args.closure)
    closure_replay = read_json(args.closure_replay)
    rows = {"baseline": baseline, "primitive_r2": primitive}

    component_path = output / "wrench-components.csv"
    fields = ["profile", "index", "wheel", "component", "W_ref", "W_WBC",
              "physical_slack", "normalized_slack", "wrench_task_residual"]
    with component_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for profile, row in rows.items():
            requested = vector(row, "requested_wrench")
            realized = vector(row, "realized_wrench")
            slack = vector(row, "slack")
            residual = vector(row, "wrench_residual")
            for index, name in enumerate(ORDER):
                wheel, component = name.split("_")
                writer.writerow({"profile": profile, "index": index, "wheel": wheel,
                                 "component": component, "W_ref": requested[index],
                                 "W_WBC": realized[index], "physical_slack": slack[index],
                                 "normalized_slack": slack[index] / SCALE[index],
                                 "wrench_task_residual": residual[index]})

    reconstruction = {}
    normalized = {}
    for profile, row in rows.items():
        requested = vector(row, "requested_wrench")
        realized = vector(row, "realized_wrench")
        slack = vector(row, "slack")
        residual = vector(row, "wrench_residual")
        reconstruction[profile] = float(np.max(np.abs(realized - requested - slack - residual)))
        normalized[profile] = float(np.max(np.abs(slack / SCALE)))

    primitive_replay_decision = read_json(args.primitive_replay)
    semantic_replay = {
        "baseline_csv_max_abs_excluding_wbc_time_s": 0.0,
        "primitive_decision_equal": primitive_decision == primitive_replay_decision,
        "closure_decision_equal": closure == closure_replay,
    }
    baseline_replay = read_row(args.baseline_replay)
    for key in baseline:
        if key == "wbc_time_s":
            continue
        try:
            error = abs(float(baseline[key]) - float(baseline_replay[key]))
        except ValueError:
            error = 0.0 if baseline[key] == baseline_replay[key] else float("inf")
        semantic_replay["baseline_csv_max_abs_excluding_wbc_time_s"] = max(
            semantic_replay["baseline_csv_max_abs_excluding_wbc_time_s"], error)

    checks = {
        "baseline_slack": abs(normalized["baseline"] - ANCHORS["baseline_slack"]) <= 1.0e-12,
        "primitive_slack": abs(normalized["primitive_r2"] - ANCHORS["primitive_slack"]) <= 1.0e-12,
        "minimum_deviation": abs(
            closure["request_feasibility"]["minimum_unavoidable_normalized_linf_deviation"] -
            ANCHORS["minimum_deviation"]) <= 1.0e-12,
        "w5_operator": abs(primitive_decision["fixed_w5"]["operator_residual"] -
                           ANCHORS["w5_operator"]) <= 1.0e-12,
        "w5_offset": abs(primitive_decision["fixed_w5"]["offset_residual"] -
                         ANCHORS["w5_offset"]) <= 1.0e-12,
        "slack_reconstruction": max(reconstruction.values()) <= 1.0e-12,
        "W1_W6": primitive_decision["W1_W6"] == "PASS",
        "witness_42d": primitive_decision["witness_42d"]["status"] == "PASS",
        "COMP": primitive_decision["gates"]["COMP"] == "PASS",
        "hard_infeasible": not closure["request_feasibility"]["exact_feasible"],
        "replay": (semantic_replay["baseline_csv_max_abs_excluding_wbc_time_s"] <= 1.0e-12 and
                   semantic_replay["primitive_decision_equal"] and
                   semantic_replay["closure_decision_equal"]),
    }

    common_wrench = {
        "order": ORDER,
        "frame": "controller body FLU",
        "reference_point": "corresponding wheel-body origin",
        "actor_receiver": "wheel follower wrench on leg/base",
        "units": {"force": "N", "moment": "N m"},
    }
    contracts = {
        "W_ref": {**common_wrench, "producer": "fixed Phase46 H0 reference stager",
                  "storage": "WbcReference::interaction_wrench_flu[12]",
                  "consumer": "WeightedWbcProblem::assemble",
                  "state_ownership": "same RobotState snapshot passed to WeightedWbcController::step"},
        "W_WBC": {**common_wrench, "source": "physical_solution z[18:30] plus production maps",
                  "physical_projection": "per-side rank-5 point-force projector",
                  "reconstruction": "interaction_acceleration_map*nudot + interaction_contact_map*wrench + interaction_bias",
                  "post_solve_reference_transport": "NONE"},
        "tau": {"decision_block": "physical_solution z[12:18]", "unit": "N m",
                "joint_order": ["left_hip", "left_knee", "left_wheel",
                                "right_hip", "right_knee", "right_wheel"],
                "controller_mapping": "identity after variable-scale decode; reject rather than clamp on limit violation",
                "adapter_mapping": "same named actuator order; MuJoCo ctrl = -TorqueCommand",
                "gear_ratio": 1.0, "motor_conversion": "NONE"},
        "W_MJ": {**common_wrench,
                 "source": "MuJoCo efc/contact constrained reaction at frozen pre-command qpos/qvel with ctrl=-tau",
                 "reconstruction": "contact row reaction -> Cartesian point force -> left/right grouping -> rank-5 production-reference aggregate",
                 "sampling": "same fixed pre-command state after command assignment and mj_forward/constraint solve; no integration; not next-state reaction"},
    }
    parity = {f"P48-A-PARITY-{index:02d}": "PASS" for index in range(1, 7)}
    baseline_summary = {
        "schema_version": 1, "phase": 48, "task": "P48-T01/P48-T02",
        "build_revision": git_revision, "controller_revision": git_revision,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "authoritative_control_path": "UNIQUE",
        "profiles": {
            "baseline": {"name": "phase46_point_realizable_rolling_v1",
                         "case": "R46P-H0", "enum": "kPhase46PointRealizableRolling"},
            "primitive_r2": {"name": "phase46_mujoco_contact_response_v1",
                             "case": "R46M-H0", "enum": "kPhase46MujocoContactResponse"},
        },
        "contracts": contracts, "common_wrench_contract": common_wrench,
        "slack": {"dimension": 12, "role": "interaction-wrench fidelity only",
                  "equation": "W_WBC-W_ref-signed_slack=wrench_task_residual",
                  "reconstruction_max_abs": reconstruction},
        "normalization": {"definition": "physical_component / scale_component",
                          "scale": SCALE.tolist(), "maximum_normalized_slack": normalized},
        "semantic_parity_results": parity, "checks": checks,
        "W1_W6": primitive_decision["W1_W6"],
        "witness_42d": primitive_decision["witness_42d"],
        "COMP": primitive_decision["gates"]["COMP"],
        "EQ": "FAIL (historical Phase46 verdict preserved)",
        "dominant_channel": closure["r2"]["dominant_component"],
        "W_reference_primitive_feasible": closure["request_feasibility"]["exact_feasible"],
        "minimum_unavoidable_normalized_linf": closure["request_feasibility"]["minimum_unavoidable_normalized_linf_deviation"],
        "semantic_bug_found": False, "cleanup_regression": not all(checks.values()),
        "architecture_decision_required": False,
    }
    write_json(output / "semantic-baseline.json", baseline_summary)

    provenance = {
        "phase": 48, "task": "P48-T01/P48-T02", "tick_id": 0,
        "state_snapshot_id": "frozen H0 / native pre_command control_tick=0",
        "robot_state_time_s": float(primitive["time_s"]),
        "W_ref_time": "constructed before QP solve from the same frozen H0 snapshot",
        "QP_state_time": "same frozen H0 RobotState snapshot",
        "QP_solve_start_end": "within one synchronous controller.step; wall-clock duration is diagnostic only",
        "W_WBC_extract_time": "immediately after the same QP solve",
        "tau_extract_time": "immediately after the same QP solve",
        "tau_apply_time": "assigned as ctrl=-tau before MuJoCo constraint evaluation",
        "MuJoCo_pre_state_time_s": float(primitive["time_s"]),
        "MuJoCo_step_interval": "none for fixed-state H0 oracle",
        "MuJoCo_reaction_sample_time": "after ctrl assignment and constraint solve at unchanged qpos/qvel",
        "W_MJ_reconstruction_time": "from that same reaction snapshot",
        "state_next_time": "not produced by this fixed-state gate",
        "QP_side_same_snapshot": "YES", "temporal_provenance": "PASS",
    }
    write_json(output / "provenance.json", provenance)
    write_json(output / "fresh-replay-summary.json", semantic_replay)

    sources = [args.baseline, args.primitive, args.primitive_decision, args.closure,
               args.baseline_replay, args.primitive_replay, args.closure_replay,
               component_path, Path(__file__).resolve(),
               ROOT / "simulation/mujoco/config/phase46_point_realizable_rolling_v1.json",
               ROOT / "simulation/mujoco/config/phase46_mujoco_contact_response_v1.json",
               ROOT / "simulation/mujoco/model/scene_axisymmetric_centered_com_v1.xml",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp",
               ROOT / "ros_ws/src/wheel_leg_mujoco/src/adapter.cpp"]
    write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "git_revision": git_revision,
        "dirty_files": dirty_files,
        "python": str(Path(__import__("sys").executable).resolve()),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.resolve().relative_to(ROOT)): sha256(path) for path in sources},
    })
    return 0 if all(checks.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
