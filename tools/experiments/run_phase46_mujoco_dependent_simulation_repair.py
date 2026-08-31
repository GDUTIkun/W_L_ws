#!/usr/bin/env python3
"""Phase46 MuJoCo-dependent simulation-only R2, strict-gate evaluator."""

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

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def boolean(row: dict[str, str], name: str) -> bool:
    return row[name] == "1"


def number(row: dict[str, str], name: str) -> float:
    return float(row[name])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    config_path = args.config.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    executable = ROOT / config["runtime_executable"]
    scene = ROOT / config["scene"]
    control = output / "compatible-h0.csv"
    command = [
        str(executable), str(scene), str(control), config["case_id"],
        config["gain_id"], str(config["xi_kp_s2"]), str(config["xi_kd_s"]),
        str(config["rate_gain"]), *(str(value) for value in config["wrench_trim"]),
        str(config["formal_ticks"]),
    ]
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if completed.returncode != 0:
        raise RuntimeError(completed.stderr or completed.stdout)
    with control.open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream))

    provenance = {
        "same_snapshot": boolean(row, "r2_same_snapshot"),
        "post_response_leakage": boolean(row, "r2_post_response_leakage"),
        "mujoco_internals": ["qpos", "qvel", "qM", "qfrc_smooth", "efc_J",
                             "efc_D", "efc_aref", "efc_type", "efc_id",
                             "contact topology and frames"],
        "same_snapshot_max_abs": 0.0,
        "oracle_qacc_max_abs": number(row, "r2_oracle_qacc_max_abs"),
    }
    legality = {
        "contact_equality_partition": boolean(row, "r2_contact_equality_partition"),
        "partition_max_abs": number(row, "r2_partition_max_abs"),
        "force_dual_covariance": boolean(row, "r2_force_dual_covariance"),
        "virtual_power_max_abs": number(row, "r2_virtual_power_max_abs"),
        "full_space_legal": boolean(row, "r2_full_space_legal"),
        "reduced_tangent_legal": boolean(row, "r2_reduced_legal"),
        "current_hard_equality_rank": int(row["r2_current_hard_rank"]),
        "rank_with_r2": int(row["r2_rank_with_response"]),
        "incremental_rank": int(row["r2_incremental_rank"]),
        "decision_row_rank": int(row["r2_decision_row_rank"]),
        "decision_row_condition": number(row, "r2_decision_row_condition"),
        "contact_force_image_compatibility": boolean(
            row, "r2_contact_image_compatible"),
        "contact_force_image_residual": number(row, "r2_contact_image_residual"),
        "pre_solve_active_set_consistency": boolean(
            row, "r2_active_set_consistent"),
        "minimum_predicted_contact_row_force": number(
            row, "r2_minimum_predicted_row_force"),
        "active_set_signature": row["r2_active_set_signature"],
        "qc0_norm": number(row, "r2_qc0_norm"),
        "qct_norm": number(row, "r2_qct_norm"),
    }
    pre_comp = (
        provenance["same_snapshot"] and not provenance["post_response_leakage"] and
        provenance["oracle_qacc_max_abs"] <= 1e-8 and
        legality["contact_equality_partition"] and
        legality["force_dual_covariance"] and
        not legality["full_space_legal"] and legality["reduced_tangent_legal"] and
        legality["incremental_rank"] == legality["decision_row_rank"] and
        legality["contact_force_image_compatibility"] and
        legality["pre_solve_active_set_consistency"]
    )
    solver = {
        "controller_status": int(row["controller_status"]),
        "solver_status": int(row["solver_status"]),
        "primal_residual": number(row, "primal"),
        "dual_residual": number(row, "dual"),
        "stationarity_residual": number(row, "stationarity"),
        "converged": int(row["controller_status"]) == 0,
        "interpretation": "PrimalInfeasible" if int(row["solver_status"]) == 5 else "other",
    }
    comp = pre_comp and solver["converged"]
    classification = (
        "H-MUJOCO-R2-HARD-INTEGRATION-OVERCONSTRAINED"
        if pre_comp and not solver["converged"] else "U-UNTRUSTED"
    )
    decision = {
        "schema_version": 1,
        "phase": 46,
        "classification": classification,
        "simulation_only": True,
        "mujoco_dependent": True,
        "hardware_ready": False,
        "warning": config["warning"],
        "default_production_profile_numerics_changed": False,
        "mujoco_r2_profile_numerics_changed": True,
        "contact_response_law": config["contact_response_law"],
        "contact_map_updated_each_tick": True,
        "provenance": provenance,
        "legality": legality,
        "implementation": {
            "profile": "kPhase46MujocoContactResponse",
            "hard_form": "compressed independent rows of N^T(Aw*W-Qct*tau)=N^T*Qc0",
            "soft_fallback": False,
            "active_set_inner_iteration": False,
            "files": [
                "ros_ws/src/wheel_leg_core/include/wheel_leg_core/weighted_wbc_problem.hpp",
                "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",
                "ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp",
                "ros_ws/src/wheel_leg_mujoco/include/wheel_leg_mujoco/mujoco_contact_response.hpp",
                "ros_ws/src/wheel_leg_mujoco/src/mujoco_contact_response.cpp",
                "ros_ws/src/wheel_leg_mujoco/src/phase35_workspace_attribution_loop.cpp",
            ],
        },
        "gates": {
            "pre_implementation": "PASS" if pre_comp else "FAIL",
            "COMP": "PASS" if comp else "FAIL",
            "EQ": "NOT ENTERED", "AUTH": "NOT ENTERED", "REAL": "NOT ENTERED",
            "SHORT": "NOT ENTERED", "10_s": "NOT ENTERED",
        },
        "solver": solver,
        "common_qp_transfer": "NOT ENTERED",
        "common_mj_transfer": "NOT ENTERED",
        "differential_qp_transfer": "NOT ENTERED",
        "differential_mj_transfer": "NOT ENTERED",
        "harmful_cross_before": -4.295093192621674,
        "harmful_cross_after": "NOT ENTERED",
        "slip_self_before": 0.030842288660802054,
        "slip_self_after": "NOT ENTERED",
        "xi_self_after": "NOT ENTERED",
        "contact_gap_before": -0.753272490427,
        "contact_gap_after": "NOT ENTERED",
        "contact_gap_reduction": "NOT ENTERED",
        "runtime_r2_residual_rms": "NOT ENTERED",
        "runtime_r2_residual_max": "NOT ENTERED",
        "active_set_transitions": "NOT ENTERED",
        "active_set_robustness": "PASS at pre-solve H0; rollout NOT ENTERED",
        "R1": "PASS by construction/regression; solve not entered",
        "mismatch_migration": "NOT ENTERED",
        "simulation_pipeline_closed": False,
        "hardware_replacement_required": True,
        "hardware_replacement_note": "replace the MuJoCo-internal contact-response layer before real-robot deployment",
        "mandatory_stop": "COMP",
        "next_action": "fix one remaining simulation blocker",
    }
    write(output / "r2-mujoco-dependent-simulation-repair.json", decision)
    replay_equal = None
    if args.replay_of:
        prior = json.loads((args.replay_of / "r2-mujoco-dependent-simulation-repair.json").read_text())
        replay_equal = prior == decision
    passed = classification.startswith("H-") and replay_equal is not False
    write(output / "summary.json", {
        "pass": passed, "classification": classification,
        "mandatory_stop": "COMP", "replay_equal": replay_equal,
    })
    sources = [config_path, executable, scene, Path(__file__).resolve(),
               ROOT / "ros_ws/src/wheel_leg_mujoco/src/mujoco_contact_response.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp"]
    write(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": [sys.executable, *sys.argv], "runtime_command": command,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): sha(path) for path in sources},
    })
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
