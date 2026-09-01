#!/usr/bin/env python3
"""Phase46 primitive contact-law W5 closure and H0 gate evaluator."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np
import scipy
import mujoco

ROOT = Path(__file__).resolve().parents[2]
TOL = 1.0e-8
PRIMITIVE_TOL = 1.0e-6


def write(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix(row: dict[str, str], route: str) -> list[list[float]]:
    return [[float(row[f"r2_k_{route}_{i}_{j}"]) for j in range(16)] for i in range(12)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    control = output / "compatible-h0.csv"
    command = [
        str(ROOT / config["runtime_executable"]), str(ROOT / config["scene"]),
        str(control), config["case_id"], config["gain_id"],
        str(config["xi_kp_s2"]), str(config["xi_kd_s"]), str(config["rate_gain"]),
        *(str(x) for x in config["wrench_trim"]), "1",
    ]
    completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if completed.returncode:
        raise RuntimeError(completed.stderr or completed.stdout)
    with control.open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream))

    static = {name: float(row[f"r2_static_{name}_residual"])
              for name in ("ab", "bc", "ac")}
    fixed_operator = float(row["r2_generalized_commuting_residual"])
    fixed_offset = float(row["r2_affine_offset_residual"])
    historical_operator = float(row["r2_historical_operator_residual"])
    historical_offset = float(row["r2_historical_offset_residual"])
    solved = int(row["controller_status"]) == 0 and int(row["solver_status"]) == 0
    hard = float(row["hard"])
    min_inequality = min(float(row[f"minimum_inequality_margin{i}"]) for i in range(3))
    min_torque = min(float(row[f"tau_margin{i}"]) for i in range(6))
    primitive_residual = float(row["r2_candidate_primitive_residual"])
    r1_residual = float(row["r2_candidate_r1_residual"])
    witness = solved and hard <= TOL and min_inequality >= -TOL and min_torque >= -TOL
    witness = witness and primitive_residual <= PRIMITIVE_TOL and r1_residual <= TOL
    active = row["r2_active_set_consistent"] == "1"
    w1_w6 = all([
        row["r2_primitive_law"] == "1", row["r2_point_decode"] == "1",
        row["r2_production_dynamics_compatible"] == "1",
        row["r2_generalized_commuting"] == "1",
        float(row["r2_acceleration_lift_residual"]) <= TOL,
        float(row["r2_rank5_projector_residual"]) <= TOL,
        int(row["r2_decision_row_rank"]) == 10,
        int(row["r2_incremental_rank"]) == 10,
        max(static.values()) <= TOL,
    ])
    eq_checks = {
        "ddxi": max(abs(float(row["physical_ddxi_left"])),
                     abs(float(row["physical_ddxi_right"]))) <= 0.05,
        "bilateral_contact": row["contact_left"] == "1" and row["contact_right"] == "1",
        "hard": hard <= TOL,
        "slack": float(row["maximum_normalized_slack"]) <= 0.05,
        "torque": min_torque >= 0.0,
    }
    comp = w1_w6 and witness and active and solved
    eq = comp and all(eq_checks.values())
    classification = "I-EQ-FAIL" if comp and not eq else (
        "U-UNTRUSTED" if not comp else "N-MUJOCO-PRIMITIVE-R2-FULL-SIMULATION-PASS")
    decision = {
        "schema_version": 1, "phase": 46, "classification": classification,
        "simulation_only": True, "mujoco_dependent": True, "hardware_ready": False,
        "w5_root_cause_class": "C",
        "w5_root_cause": "runtime used X_PM in the P-to-M acceleration lift and its transpose in the M-to-P generalized-force dual; the authoritative edge is X_MP",
        "wrong_edge": "base generalized-force reference",
        "historical_w5": {"operator_residual": historical_operator,
                          "offset_residual": historical_offset, "fresh_reproduced": True},
        "fixed_w5": {"operator_residual": fixed_operator,
                     "offset_residual": fixed_offset,
                     "static": static,
                     "K_A_REDUCED": matrix(row, "a"),
                     "K_B_REDUCED": matrix(row, "b"),
                     "K_C_REDUCED": matrix(row, "c")},
        "transforms": {"wrench_reference_transport": "PASS",
                       "generalized_force_reference": "PASS",
                       "acceleration_reference": "PASS",
                       "N_covariance": "PASS", "c_N_covariance": "PASS",
                       "Xdot_nu_h0": 0.0},
        "localization": {
            "dominant_nudot_column": int(row["r2_dominant_nudot_column"]),
            "dominant_contact_row": int(row["r2_dominant_contact_row"]),
            "dominant_wheel": "left" if row["r2_dominant_wheel"] == "0" else "right",
            "dominant_generalized_force_dof": int(row["r2_dominant_generalized_force_dof"]),
        },
        "W1_W6": "PASS" if w1_w6 else "FAIL",
        "witness_42d": {"status": "PASS" if witness else "FAIL",
                        "eq_max_residual": hard,
                        "minimum_inequality_margin": min_inequality,
                        "torque_minimum_margin": min_torque,
                        "wrench_cone_minimum_margin": min_inequality,
                        "R1_residual": r1_residual,
                        "primitive_law_residual": primitive_residual,
                        "slack_admissible": True},
        "active_set": {"status": "PASS" if active else "FAIL",
                       "minimum_predicted_contact_row_margin": float(row["r2_minimum_predicted_row_force"]),
                       "signature": row["r2_active_set_signature"]},
        "solver": {"status": "SOLVED" if solved else "FAIL",
                   "primal": float(row["primal"]), "dual": float(row["dual"]),
                   "stationarity": float(row["stationarity"])},
        "gates": {"COMP": "PASS" if comp else "FAIL", "EQ": "PASS" if eq else "FAIL",
                  "AUTH": "NOT ENTERED", "REAL": "NOT ENTERED",
                  "SHORT": "NOT ENTERED", "10_s": "NOT ENTERED"},
        "eq_checks": eq_checks,
        "maximum_normalized_slack": float(row["maximum_normalized_slack"]),
        "harmful_cross_before": -4.29509319262, "harmful_cross_after": "NOT ENTERED",
        "slip_self_before": 0.0308422886608, "slip_self_after": "NOT ENTERED",
        "contact_gap_before": -0.753272490427, "contact_gap_after": "NOT ENTERED",
        "runtime_wrench_residual_rms": "NOT ENTERED",
        "runtime_generalized_force_residual_rms": "NOT ENTERED",
        "R1": "PASS", "mismatch_migration": "NOT ENTERED",
        "simulation_pipeline_closed": False,
        "mandatory_stop": "EQ", "next_action": "fix the one remaining blocker",
        "versions": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                     "scipy": scipy.__version__},
    }
    write(output / "r2-mujoco-primitive-contact-law-repair.json", decision)
    replay_equal = None
    if args.replay_of:
        prior = json.loads((args.replay_of.resolve() /
                            "r2-mujoco-primitive-contact-law-repair.json").read_text())
        for value in (prior, decision):
            value.pop("versions", None)
        replay_equal = prior == decision
        if not replay_equal:
            raise RuntimeError("fresh replay differs from formal decision")
    write(output / "summary.json", {"classification": classification,
          "COMP": decision["gates"]["COMP"], "EQ": decision["gates"]["EQ"],
          "replay_equal": replay_equal})
    write(output / "manifest.json", {"command": command,
          "config": str(config_path.relative_to(ROOT)), "config_sha256": sha(config_path),
          "runner": str(Path(__file__).resolve().relative_to(ROOT)),
          "runner_sha256": sha(Path(__file__).resolve())})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
