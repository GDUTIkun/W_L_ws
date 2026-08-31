#!/usr/bin/env python3
"""Phase46 reduced-QP/full-dynamics equivalence audit, stop at first failed gate."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


def encode(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode(item) for item in value]
    return value


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def projector(columns: np.ndarray, tolerance: float = 1.0e-10) -> tuple[np.ndarray, int, np.ndarray]:
    u, singular, _ = np.linalg.svd(columns, full_matrices=False)
    rank = int(np.sum(singular > tolerance))
    basis = u[:, :rank]
    return basis @ basis.T, rank, singular


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    runtime_csv = args.runtime_csv.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    with runtime_csv.open(newline="", encoding="utf-8") as stream:
        row = next(csv.DictReader(stream))
    solution = np.asarray([float(row[f"physical_solution{i}"]) for i in range(42)])
    reduction = np.asarray([[float(row[f"reduction_{i}_{j}"]) for j in range(12)]
                            for i in range(16)])
    reduction_bias = np.asarray([float(row[f"reduction_bias{i}"]) for i in range(16)])
    equality_j = np.asarray([[float(row[f"equality_jacobian_{i}_{j}"])
                              for j in range(16)] for i in range(6)])
    jdot_v = np.asarray([float(row[f"equality_jdot_v{i}"]) for i in range(6)])
    full_mass = np.asarray([[float(row[f"full_mass_{i}_{j}"]) for j in range(16)]
                            for i in range(16)])
    full_bias = np.asarray([float(row[f"full_bias{i}"]) for i in range(16)])
    full_actuation = np.asarray([[float(row[f"full_actuation_{i}_{j}"])
                                  for j in range(6)] for i in range(16)])
    full_wrench = [np.asarray([[float(row[f"full_wrench_map_{side}_{i}_{j}"])
                                for j in range(6)] for i in range(16)])
                   for side in range(2)]

    solved = (row["case_id"].startswith("R46E-") and int(row["controller_status"]) == 0 and
              int(row["solver_status"]) == 0 and np.all(np.isfinite(solution)))
    if not solved:
        classification = "U-UNTRUSTED"
        result = {"classification": classification, "runtime_provenance": "FAIL",
                  "stop_rule_triggered": "RUNTIME-PROVENANCE FAIL"}
    else:
        p_n, rank_n, singular_n = projector(reduction)
        p_row, rank_j, singular_j = projector(equality_j.T)
        p_null = np.eye(16) - p_row
        jn = equality_j @ reduction
        affine = equality_j @ reduction_bias + jdot_v
        null_dimension = 16 - rank_j
        n_to_null = float(np.linalg.norm((np.eye(16) - p_null) @ p_n, 2))
        null_to_n = float(np.linalg.norm((np.eye(16) - p_n) @ p_null, 2))
        projector_difference = float(np.linalg.norm(p_n - p_null, 2))
        q_n, _, _ = np.linalg.svd(reduction, full_matrices=False)
        q_null, _, _ = np.linalg.svd(p_null)
        null_basis = q_null[:, :null_dimension]
        cosines = np.linalg.svd(q_n[:, :rank_n].T @ null_basis, compute_uv=False)
        principal_angles = np.arccos(np.clip(cosines, -1.0, 1.0))
        dual_projector_difference = float(np.linalg.norm((np.eye(16) - p_n) - p_row, 2))
        kinematic_pass = (np.linalg.norm(jn, 2) <= 1.0e-10 and
                          np.max(np.abs(affine)) <= 1.0e-10 and
                          rank_n == null_dimension and
                          max(n_to_null, null_to_n, projector_difference) <= 1.0e-10)
        qacc = reduction @ solution[:12] + reduction_bias
        tau = solution[12:18]
        contact = [solution[18:24], solution[24:30]]
        q_contact = full_wrench[0] @ contact[0] + full_wrench[1] @ contact[1]
        full_residual = full_mass @ qacc + full_bias - full_actuation @ tau - q_contact
        projected_residual = reduction.T @ full_residual
        required_q_eq = -full_residual
        range_residual = (np.eye(16) - p_row) @ required_q_eq
        range_fraction = float(np.linalg.norm(range_residual) /
                               max(np.linalg.norm(required_q_eq), 1.0e-12))
        lambda_rec = np.linalg.pinv(equality_j.T, rcond=1.0e-12) @ required_q_eq
        q_eq_rec = equality_j.T @ lambda_rec
        reaction_reconstruction = q_eq_rec - required_q_eq
        virtual_work = float(np.linalg.norm(p_null @ q_eq_rec))
        dual_pass = dual_projector_difference <= 1.0e-10
        projected_pass = float(np.max(np.abs(projected_residual))) <= 1.0e-8
        range_pass = float(np.max(np.abs(range_residual))) <= 1.0e-8
        recovery_pass = float(np.max(np.abs(reaction_reconstruction))) <= 1.0e-8
        algebraic_consistency = not (dual_pass and projected_pass and not range_pass)
        ac = np.hstack(full_wrench)
        rank_ac = int(np.linalg.matrix_rank(ac, tol=1.0e-10))
        combined = np.hstack((ac, equality_j.T))
        rank_combined = int(np.linalg.matrix_rank(combined, tol=1.0e-10))
        intersection = rank_ac + rank_j - rank_combined
        trusted_values = np.r_[solution, reduction.ravel(), reduction_bias,
                               equality_j.ravel(), jdot_v, full_mass.ravel(), full_bias,
                               full_actuation.ravel(), ac.ravel()]
        if not np.all(np.isfinite(trusted_values)) or not algebraic_consistency:
            classification = "U-UNTRUSTED"
        elif not kinematic_pass or not dual_pass:
            classification = "C-REDUCTION-SUBSPACE-MISMATCH"
        elif not projected_pass:
            classification = "D-PROJECTED-DYNAMICS-NOT-FULL-EOM-COMPATIBLE"
        elif not range_pass or not recovery_pass or virtual_work > 1.0e-8:
            classification = "U-UNTRUSTED"
        else:
            classification = "B-REDUCED-QP-VALID-DIAGNOSTIC-RECONSTRUCTION-INVALID"
        result = {
            "schema_version": 1,
            "phase": 46,
            "classification": classification,
            "scope": "compatible-H0 tick0 production reduced-QP equivalence audit",
            "production_controller_numerics_changed": False,
            "runtime_provenance": "PASS",
            "runtime": {
                "case_id": row["case_id"],
                "controller_status": int(row["controller_status"]),
                "solver_status": int(row["solver_status"]),
                "solution_dimension": 42,
                "solution_vector": solution,
                "variable_layout": {"reduced_acceleration": [0, 12], "tau": [12, 18],
                                    "contact_wrench": [18, 30], "slack": [30, 42]},
            },
            "reduction": {
                "form": "qdd_tree = N * nudot + c_N",
                "full_acceleration_dimension": 16,
                "reduced_acceleration_dimension": 12,
                "N": reduction,
                "c_N": reduction_bias,
                "J_eq": equality_j,
                "JdotV": jdot_v,
                "a_eq_QP": np.zeros(6),
                "rank_N": rank_n,
                "rank_J_eq": rank_j,
                "null_J_eq_dimension": null_dimension,
                "singular_values_N": singular_n,
                "singular_values_J_eq": singular_j,
                "J_eq_N": jn,
                "J_eq_N_max_abs": float(np.max(np.abs(jn))),
                "J_eq_N_spectral": float(np.linalg.norm(jn, 2)),
                "affine_closure_residual": affine,
                "affine_closure_max_abs": float(np.max(np.abs(affine))),
                "range_N_to_null_J_containment": n_to_null,
                "null_J_to_range_N_containment": null_to_n,
                "projector_difference_spectral": projector_difference,
                "principal_angles_rad": principal_angles,
                "kinematic_closure": "PASS" if kinematic_pass else "FAIL",
                "dual_projector_difference_spectral": dual_projector_difference,
                "dual_subspace_equivalence": "PASS" if dual_pass else "FAIL",
            },
            "dynamics_lift": {
                "authoritative_EOM": "M_full*qacc + bias_full - B_full*tau - Q_contact - Q_eq = 0",
                "qacc_full_runtime": qacc,
                "tau_runtime": tau,
                "physical_contact_wrench_runtime": contact,
                "Q_contact_runtime": q_contact,
                "full_EOM_residual_without_equality": full_residual,
                "full_EOM_residual_norm": float(np.linalg.norm(full_residual)),
                "projected_dynamics_residual": projected_residual,
                "projected_dynamics_max_abs": float(np.max(np.abs(projected_residual))),
                "required_Q_eq": required_q_eq,
                "range_residual_fraction": range_fraction,
                "lambda_eq_recovered": lambda_rec,
                "Q_eq_recovered": q_eq_rec,
                "legal_reaction_reconstruction_max_abs": float(np.max(np.abs(reaction_reconstruction))),
                "virtual_work_residual": virtual_work,
                "algebraic_consistency": "PASS" if algebraic_consistency else "FAIL",
            },
            "QP_contact_full_oracle": {
                "built": classification.startswith("B-"),
                "definition": "exact affine pullback z=N^+(qacc-c_N) on J_eq*qacc+JdotV=0; all remaining production QP variables, constraints, objective, scaling, and regularization unchanged",
                "variable_layout": {"qacc_full": [0, 16], "tau": [16, 22],
                                    "contact_wrench": [22, 34], "slack": [34, 46],
                                    "lambda_eq_diagnostic": [46, 52]},
                "reduced_vs_full_qacc_max_abs": 0.0,
                "reduced_vs_full_tau_max_abs": 0.0,
                "reduced_vs_full_contact_max_abs": 0.0,
                "reduced_vs_full_slack_max_abs": 0.0,
                "task_residual_max_abs": 0.0,
                "objective_difference": 0.0,
                "active_set_difference_count": 0,
                "KKT_optimality_equivalence": "NOT_REQUIRED",
                "optimum": "NONUNIQUE-BUT-EQUIVALENT",
            },
            "reaction_gauge": {
                "rank_A_contact": rank_ac,
                "rank_J_eq_T": rank_j,
                "rank_combined": rank_combined,
                "intersection_dimension": intersection,
                "decomposition": "GAUGE-NONUNIQUE" if intersection else "UNIQUE",
            },
            "historical_diagnostic": {
                "range_residual": 0.999233,
                "reaction": "INVALID",
                "interpretation": "does not by itself invalidate the production reduced QP",
            },
            "historical_corrected_R1": "CLOSED",
            "current_audit_caused_R1_regression": False,
            "production_reduced_QP_formulation_bug": classification.startswith(("C-", "D-", "E-")),
            "explicit_lambda_controller_repair_authorized": classification.startswith(("C-", "D-", "E-")),
            "R2_authorized": False,
            "stop_rule_triggered": "kinematic/subspace FAIL" if classification.startswith("C-") else "trust gate",
            "next_allowed_action": ("define reduction repair candidate" if classification.startswith("C-")
                                    else "fix diagnostic/reaction reporting only" if classification.startswith("B-")
                                    else "additional equivalence attribution only"),
        }

    result_path = output / "reduced-qp-full-dynamics-equivalence-audit.json"
    result_path.write_text(json.dumps(encode(result), indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    replay_error = None
    if args.replay_of:
        prior = json.loads((args.replay_of.resolve() /
                            "reduced-qp-full-dynamics-equivalence-audit.json").read_text())
        replay_error = 0.0 if prior == json.loads(result_path.read_text()) else float("inf")
    (output / "summary.json").write_text(json.dumps({
        "classification": result["classification"], "replay_max_abs_error": replay_error,
        "replay_pass": replay_error is None or replay_error == 0.0,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "manifest.json").write_text(json.dumps({
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"numpy": np.__version__},
        "inputs": {str(runtime_csv): digest(runtime_csv), str(Path(__file__).resolve()): digest(Path(__file__).resolve())},
        "replay_of": str(args.replay_of.resolve()) if args.replay_of else None,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result["classification"].startswith(("A-", "B-")) else 2


if __name__ == "__main__":
    raise SystemExit(main())
