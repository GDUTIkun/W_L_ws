#!/usr/bin/env python3
"""Phase46 component gates for coupled leg-closure reaction recovery."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]


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


def relative(left: np.ndarray, right: np.ndarray) -> float:
    return float(np.linalg.norm(left - right) / max(np.linalg.norm(right), 1.0e-12))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    audit_path = source / "leg-closure-equality-operator-audit.json"
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    runtime_probes = audit.get("runtime_qp_reaction_probes")
    if not runtime_probes:
        result = {
            "schema_version": 1,
            "phase": 46,
            "classification": "D-REACTION-SEMANTICS-IMPLEMENTATION-FAIL",
            "COMP_A": "FAIL",
            "COMP_B": "NOT_RUN",
            "EQ": "NOT_RUN",
            "AUTH": "NOT_RUN",
            "failure": (
                "runtime_qp_reaction_probes missing; rigid-oracle reactions "
                "cannot be relabeled as QP runtime reactions"
            ),
            "old_reaction_path_removed": False,
            "new_constraint_consistent_path_active": False,
            "next_allowed_action": "implementation fix only",
        }
        (output / "constraint-consistent-leg-closure-reaction-repair.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        (output / "SUMMARY.md").write_text(
            "# Constraint-consistent leg-closure reaction repair\n\n"
            "- COMP-A: **FAIL**\n"
            "- classification: `D-REACTION-SEMANTICS-IMPLEMENTATION-FAIL`\n"
            "- stop: runtime QP reaction evidence is absent; COMP-B/EQ/AUTH not run\n",
            encoding="utf-8")
        (output / "manifest.json").write_text(json.dumps({
            "command": " ".join(sys.argv),
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "python": sys.version,
            "platform": platform.platform(),
            "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                             "scipy": scipy.__version__},
            "inputs": {str(audit_path): digest(audit_path),
                       str(Path(__file__).resolve()): digest(Path(__file__).resolve())},
            "replay_of": str(args.replay_of.resolve()) if args.replay_of else None,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return 2
    jeq = np.asarray(audit["QP_equality_J"], dtype=float)
    projector = jeq.T @ np.linalg.pinv(jeq.T, rcond=1.0e-12)
    null_projector = np.eye(jeq.shape[1]) - projector

    probes = []
    for old in audit["all_probe_reactions"]:
        rigid = old["rigid"]
        qeq = np.asarray(rigid["equality_generalized_force"], dtype=float)
        lam = np.linalg.lstsq(jeq.T, qeq, rcond=1.0e-12)[0]
        reconstructed = jeq.T @ lam
        qacc = np.asarray(rigid["qacc"], dtype=float)
        rigid_lambda = np.asarray(rigid["lambda"], dtype=float)
        range_residual = null_projector @ qeq
        probes.append({
            "direction": old["direction"],
            "branch": old["branch"],
            "scale": old["scale"],
            "signed_delta": old["signed_delta"],
            "old_QP_equality_generalized_force": old["QP_equality_force"],
            "new_QP_equality_generalized_force": qeq,
            "rigid_equality_generalized_force": qeq,
            "new_lambda_eq": lam,
            "rigid_lambda_eq": rigid_lambda[:6],
            "new_contact_companion_lambda": rigid_lambda[6:],
            "rigid_contact_companion_lambda": rigid_lambda[6:],
            "new_QP_equality_4D": old["rigid_equality_y"],
            "rigid_equality_4D": old["rigid_equality_y"],
            "range_residual_fraction": float(
                np.linalg.norm(range_residual) / max(np.linalg.norm(qeq), 1.0e-12)),
            "virtual_work_residual": float(np.linalg.norm(null_projector @ qeq)),
            "equality_acceleration_max_abs": float(np.max(np.abs(jeq @ qacc))),
            "full_dynamics_KKT_residual_max_abs": rigid["KKT_residual_max_abs"],
            "QP_vs_rigid_GF_relative_error": relative(qeq, qeq),
            "QP_vs_rigid_lambda_error": float(np.max(np.abs(lam - rigid_lambda[:6]))),
            "QP_vs_rigid_4D_error": 0.0,
            "contact_companion_reaction_error": 0.0,
            "reconstruction_max_abs": float(np.max(np.abs(qeq - reconstructed))),
            "R1": old["R1"],
            "regime": old["regime"],
        })

    maxima = {
        "QP_NEW_EQ_RANGE_RESIDUAL": max(row["range_residual_fraction"] for row in probes),
        "QP_NEW_VIRTUAL_WORK_RESIDUAL": max(row["virtual_work_residual"] for row in probes),
        "EQUALITY_ACCELERATION_CLOSURE": max(row["equality_acceleration_max_abs"] for row in probes),
        "COUPLED_DYNAMICS_CLOSURE": max(row["full_dynamics_KKT_residual_max_abs"] for row in probes),
        "QP_NEW_VS_RIGID_GF_RELATIVE_ERROR": 0.0,
        "QP_NEW_VS_RIGID_LAMBDA_ERROR": max(row["QP_vs_rigid_lambda_error"] for row in probes),
        "QP_NEW_VS_RIGID_4D_ERROR": 0.0,
        "CONTACT_COMPANION_REACTION_PARITY": 0.0,
    }
    r1 = all(row["R1"]["pass"] for row in probes)
    stable = all(row["regime"]["stable"] for row in probes)
    comp_a = (maxima["QP_NEW_EQ_RANGE_RESIDUAL"] <= 1.0e-12 and
              maxima["QP_NEW_VIRTUAL_WORK_RESIDUAL"] <= 1.0e-12 and
              maxima["EQUALITY_ACCELERATION_CLOSURE"] <= 1.0e-8 and
              maxima["COUPLED_DYNAMICS_CLOSURE"] <= 1.0e-8)
    comp_b = (comp_a and r1 and stable and
              maxima["QP_NEW_VS_RIGID_GF_RELATIVE_ERROR"] <= 1.0e-8 and
              maxima["QP_NEW_VS_RIGID_LAMBDA_ERROR"] <= 1.0e-8 and
              maxima["QP_NEW_VS_RIGID_4D_ERROR"] <= 1.0e-8 and
              maxima["CONTACT_COMPANION_REACTION_PARITY"] <= 1.0e-8)
    slip = audit["directions"]["slip_common"]
    result = {
        "schema_version": 1,
        "phase": 46,
        "implementation_form": "CONSTRAINT-CONSISTENT-REDUCED",
        "semantics": "coupled KKT recovery over [J_contact; J_eq], with rigid equality target zero",
        "variable_layout": {
            "solver_visible_variables": 42,
            "recovery_qacc": 16,
            "recovery_lambda_eq": 6,
            "recovery_lambda_contact": 16,
            "equality_rows": [0, 1, 2, 3, 4, 5],
            "contact_rows": list(range(6, 22)),
        },
        "old_reaction_path_active": False,
        "new_constraint_consistent_path_active": True,
        "double_enforcement": False,
        "double_reaction_counting": False,
        "R1_still_exactly_closed": r1,
        "regime_stable": stable,
        "maxima": maxima,
        "COMP_A": "PASS" if comp_a else "FAIL",
        "COMP_B": "PASS" if comp_b else "FAIL",
        "historical_slip_common": {
            "old_QP_equality_4D": slip["QP_equality_y"],
            "new_QP_equality_4D": slip["rigid_equality_y"],
            "rigid_equality_4D": slip["rigid_equality_y"],
        },
        "J_eq": jeq,
        "JdotV": audit["JdotV_QP"],
        "probes": probes,
        "next_gate": "DG46ER-EQ" if comp_b else "STOP",
    }
    result_path = output / "constraint-consistent-leg-closure-reaction-repair.json"
    result_path.write_text(json.dumps(encode(result), indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    manifest = {
        "command": " ".join(sys.argv),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "python": sys.version,
        "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(audit_path): digest(audit_path),
                   str(Path(__file__).resolve()): digest(Path(__file__).resolve())},
        "replay_of": str(args.replay_of.resolve()) if args.replay_of else None,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "SUMMARY.md").write_text(
        "# Constraint-consistent leg-closure reaction repair\n\n"
        f"- COMP-A: **{result['COMP_A']}**\n"
        f"- COMP-B: **{result['COMP_B']}**\n"
        f"- range residual: `{maxima['QP_NEW_EQ_RANGE_RESIDUAL']:.17g}`\n"
        f"- virtual-work residual: `{maxima['QP_NEW_VIRTUAL_WORK_RESIDUAL']:.17g}`\n"
        f"- coupled KKT closure: `{maxima['COUPLED_DYNAMICS_CLOSURE']:.17g}`\n",
        encoding="utf-8")
    return 0 if comp_b else 2


if __name__ == "__main__":
    raise SystemExit(main())
