#!/usr/bin/env python3
"""Attribute Phase46 MuJoCo-only closure modes from frozen evidence; no model repair."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import scipy
from scipy.linalg import null_space, subspace_angles

ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
DEFAULT_CONDITIONED = PHASE / "evidence/automated/closure-conditioned-effective-inertia-formal-v2/closure-conditioned-effective-inertia-audit.json"
DEFAULT_EQUALITY = PHASE / "evidence/automated/leg-closure-equality-operator-audit-formal-v4/leg-closure-equality-operator-audit.json"
OUTPUT_NAMES = ("ddxi_common", "slip_common", "ddxi_differential", "slip_differential")


def encode(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode(item) for item in value]
    return value


def write(path: Path, value) -> None:
    path.write_text(json.dumps(encode(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def conditioned(mass: np.ndarray, jacobian: np.ndarray) -> np.ndarray:
    inverse = np.linalg.inv(mass)
    schur = jacobian @ inverse @ jacobian.T
    return inverse - inverse @ jacobian.T @ np.linalg.pinv(schur, rcond=1e-12) @ jacobian @ inverse


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--conditioned", type=Path, default=DEFAULT_CONDITIONED)
    parser.add_argument("--equality", type=Path, default=DEFAULT_EQUALITY)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    conditioned_evidence = json.loads(args.conditioned.read_text())
    equality = json.loads(args.equality.read_text())
    operators = conditioned_evidence["operator_provenance"]
    mass = np.asarray(operators["M_MJ"])
    common = np.asarray(operators["J_MJ_common4"])
    native = np.asarray(equality["MJ_equality_J"])
    probes = [item for item in conditioned_evidence["all_probes"]
              if item["direction"] == "slip_common" and item["scale"] == 1.0]
    frozen_force = np.mean([item["Delta_Q_smooth"] for item in probes], axis=0)

    geometry = equality["constraint_geometry"]
    residuals = np.asarray([item["relative_position"] for item in geometry])
    row_norms = np.linalg.norm(native, axis=1)
    singular_values = np.linalg.svd(native, compute_uv=False)
    common_projector = common.T @ common
    y_rows = native[[1, 4]]
    native_only = y_rows / np.linalg.norm(y_rows, axis=1, keepdims=True)
    weak_singular_values = np.linalg.svd(y_rows, compute_uv=False)
    cartesian_coefficients = np.eye(2)
    angles = subspace_angles(native_only.T, null_space(common))

    # At the frozen planar geometry, each Cartesian-y row is exactly the base angular
    # cross-product of the nonzero x/z site residual; it vanishes on the exact manifold.
    predicted_y_rows = np.zeros_like(y_rows)
    for index, (dx, _dy, dz) in enumerate(residuals):
        predicted_y_rows[index, 3] = dz
        predicted_y_rows[index, 5] = -dx
    sign_errors = [min(np.max(np.abs(row-pred)), np.max(np.abs(row+pred)))
                   for row, pred in zip(y_rows, predicted_y_rows)]

    # Stay above the frozen SVD cutoff; crossing it is the discontinuous zero-rank limit.
    scales = (1.0, 0.5, 0.25, 0.1)
    scaled = []
    base_k = conditioned(mass, native)
    for scale in scales:
        scaled_native = native.copy()
        scaled_native[[1, 4]] *= scale
        operator = conditioned(mass, scaled_native)
        scaled.append({"scale": scale,
                       "operator_gap_from_scale_1": np.linalg.norm(operator-base_k, 2),
                       "qacc_gap_from_scale_1": np.linalg.norm((operator-base_k)@frozen_force)})
    k_null = conditioned(mass, native[[0, 2, 3, 5]])
    hard_gap = (base_k-k_null) @ frozen_force
    stored_gap = np.asarray(conditioned_evidence["directions"]["slip_common"]["MJ_only_gap"]["qacc_gap"])
    stored_operator_gap = ((np.asarray(operators["K_MJ6"])-np.asarray(operators["K_MJ4"]))
                           @ frozen_force)

    diagnostics = equality["MJ_equality_diagnostics"]
    weak_aref = np.asarray(diagnostics["efc_aref"])[[1, 4]]
    weak_pos = np.asarray(diagnostics["efc_pos"])[[1, 4]]
    physical = {
        "classification": "BOOKKEEPING-HARD-RANK-ARTIFACT-NOT-INDEPENDENT",
        "reason": "the two rows are out-of-plane Cartesian connect rows induced only by finite in-plane site residual; their Jacobians vanish on the exact closure manifold, while hard conditioning normalizes every nonzero row",
        "independent_physical_mismatch": False,
        "material_counterfactual_is_discontinuous": True,
        "contact_unique_remaining_mismatch": True,
        "R2_candidate_for_next_reauthorization": True,
        "R2_authorized": False,
    }
    trust = {
        "rank_common": int(np.linalg.matrix_rank(common, 1e-10)),
        "rank_native_raw_at_1e-10": int(np.linalg.matrix_rank(native, 1e-10)),
        "native_singular_values": singular_values,
        "native_only_singular_values": weak_singular_values[:2],
        "maximum_y_row_residual_cross_product_error": max(sign_errors),
        "maximum_weak_position_residual": np.max(np.abs(weak_pos)),
        "maximum_weak_stabilization_target": np.max(np.abs(weak_aref)),
        "scaled_hard_operator_max_gap": max(item["operator_gap_from_scale_1"] for item in scaled),
        "stored_response_qacc_closure": np.linalg.norm(stored_operator_gap-stored_gap),
    }
    passed = bool(trust["rank_common"] == 4 and trust["rank_native_raw_at_1e-10"] == 6
              and trust["maximum_y_row_residual_cross_product_error"] <= 1e-12
              and trust["maximum_weak_position_residual"] <= 1e-12
              and trust["maximum_weak_stabilization_target"] <= 1e-10
              and trust["scaled_hard_operator_max_gap"] <= 1e-5
              and trust["stored_response_qacc_closure"] <= 1e-8)
    result = {
        "schema_version": 1,
        "phase": 46,
        "scope": "fixed-state MuJoCo-only closure-model attribution",
        "site_pairs": geometry,
        "raw_rows": {"jacobian": native, "row_norms": row_norms, "singular_values": singular_values},
        "common4": common,
        "native_only_2d": {"basis": native_only, "cartesian_row_coefficients": cartesian_coefficients,
                           "angles_to_common_tangent_rad": angles,
                           "interpretation": "left/right out-of-plane site-relative y rows; common/differential combinations constrain base roll/yaw through the finite x/z closure residual"},
        "exact_manifold_test": {"site_residuals": residuals, "measured_y_rows": y_rows,
                                "predicted_from_base_rotation_cross_residual": predicted_y_rows,
                                "max_abs_error_allowing_row_sign": max(sign_errors),
                                "rank_on_exact_planar_closure": 4},
        "solver_semantics": {"weak_rows": [1, 4], "efc_pos": weak_pos, "efc_aref": weak_aref,
                             "efc_D": np.asarray(diagnostics["efc_D"])[[1, 4]],
                             "efc_R": np.asarray(diagnostics["efc_R"])[[1, 4]],
                             "hard_row_scaling": scaled,
                             "zero_limit_qacc_jump": hard_gap,
                             "stored_operator_qacc_gap": stored_operator_gap,
                             "stored_MJ_only_qacc_gap": stored_gap,
                             "stored_outputs": conditioned_evidence["directions"]["slip_common"]["MJ_only_gap"]["output_gap"]},
        "decision": physical,
        "trust": {**trust, "pass": passed},
    }
    write(output / "closure-model-attribution.json", result)

    replay_error = None
    if args.replay_of:
        prior = json.loads((args.replay_of / "closure-model-attribution.json").read_text())
        def numeric_leaves(value):
            if isinstance(value, dict):
                for key in sorted(value):
                    if key not in {"created_utc", "command"}:
                        yield from numeric_leaves(value[key])
            elif isinstance(value, list):
                for item in value:
                    yield from numeric_leaves(item)
            elif isinstance(value, (int, float)) and not isinstance(value, bool):
                yield float(value)
        replay_error = max((abs(a-b) for a, b in zip(numeric_leaves(prior), numeric_leaves(encode(result)))), default=0.0)
        passed = passed and replay_error <= 1e-11
    write(output / "summary.json", {"pass": passed, "classification": physical["classification"],
                                     "replay_max_abs_error": replay_error, **{k: physical[k] for k in ("independent_physical_mismatch", "contact_unique_remaining_mismatch", "R2_candidate_for_next_reauthorization", "R2_authorized")}})
    sources = (args.conditioned.resolve(), args.equality.resolve(), Path(__file__).resolve())
    write(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
          "command": " ".join(sys.argv), "python": sys.version, "platform": platform.platform(),
          "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__},
          "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
