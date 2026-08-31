#!/usr/bin/env python3
"""Phase46 algebra-only audit of the current rank-5 wrench projector."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy
from scipy.linalg import subspace_angles


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
DEFAULT_SOURCE = PHASE / "evidence/automated/contact-realization-sensitivity-formal-v4"
DEFAULT_ORACLE = PHASE / "evidence/automated/root-cause-closure-formal-v3/point-realizability.json"
DEFAULT_CANDIDATE = PHASE / "evidence/automated/point-realizable-repair-equilibrium-formal-v1/baseline.csv"
TOLERANCE = 1.0e-10


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ROOT_CAUSE = load(ROOT / "tools/experiments/run_phase46_root_cause_closure.py",
                  "p46_subspace_root")


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, default=lambda item: item.tolist()) + "\n",
                    encoding="utf-8")


def first_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def orthogonal_projector(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    u, singular, vh = np.linalg.svd(matrix, full_matrices=True)
    svd_tolerance = max(matrix.shape) * np.finfo(float).eps * singular[0]
    rank = int(np.sum(singular > svd_tolerance))
    return u[:, :rank] @ u[:, :rank].T, u, vh, rank


def matrix_norms(matrix: np.ndarray) -> dict[str, float]:
    return {"spectral": float(np.linalg.norm(matrix, 2)),
            "frobenius": float(np.linalg.norm(matrix, "fro")),
            "max_abs": float(np.max(np.abs(matrix)))}


def point_map(points: list[dict[str, Any]], frame: np.ndarray,
              reference_world: np.ndarray) -> np.ndarray:
    blocks = []
    for point in sorted(points, key=lambda item: item["point_index"]):
        lever_contact = frame.T @ (np.asarray(point["position_world_m"]) - reference_world)
        blocks.append(np.vstack((np.eye(3), skew(lever_contact))))
    return np.hstack(blocks)


def transport(old_reference: np.ndarray, new_reference: np.ndarray,
              frame: np.ndarray) -> np.ndarray:
    old_minus_new = frame.T @ (old_reference - new_reference)
    result = np.eye(6)
    result[3:, :3] = skew(old_minus_new)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--oracle", type=Path, default=DEFAULT_ORACLE)
    parser.add_argument("--candidate", type=Path, default=DEFAULT_CANDIDATE)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    source, oracle_path, candidate_path = (args.source.resolve(), args.oracle.resolve(),
                                            args.candidate.resolve())
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    records = read_json(source / "probe-records.json")
    baseline = records["baseline"]
    setup = ROOT_CAUSE.actual_setup({}, records)
    prior = read_json(oracle_path)
    candidate = first_row(candidate_path)
    data = mujoco.MjData(setup["model"])
    data.qpos[:] = setup["actual"]["qpos"] if "actual" in setup else ROOT_CAUSE.P44.vec(
        setup["native"], "qpos", setup["model"].nq)
    data.qvel[:] = ROOT_CAUSE.P44.vec(setup["native"], "qvel", setup["model"].nv)
    mujoco.mj_forward(setup["model"], data)

    sides: dict[str, Any] = {}
    classifications = []
    for side, side_name in enumerate(("left", "right")):
        geometry = setup["geometry"][side]
        frame = np.asarray(geometry["frame"])
        reference = np.asarray(geometry["point"])
        side_points = [row for row in baseline["points"] if row["side"] == side_name]
        gp = point_map(side_points, frame, reference)
        pg, u, vh, rank = orthogonal_projector(gp)
        singular = np.linalg.svd(gp, compute_uv=False)
        null_g = u[:, rank]

        axis_world = data.ximat[geometry["body"]].reshape(3, 3)[:, 0]
        axis = frame.T @ axis_world
        axis /= np.linalg.norm(axis)
        null_p = np.r_[np.zeros(3), axis]
        pw = np.eye(6) - np.outer(null_p, null_p)

        basis_p = np.linalg.eigh(pw)[1][:, -5:]
        basis_g = u[:, :rank]
        angles = subspace_angles(basis_p, basis_g)
        ep = pw - pg
        p_to_g = (np.eye(6) - pg) @ pw
        g_to_p = (np.eye(6) - pw) @ pg

        qp_wrench = np.asarray(baseline["qp_wrench"])[6 * side:6 * side + 6]
        physical_wrench = np.asarray([float(candidate[f"physical_solution{18 + 6 * side + i}"])
                                      for i in range(6)])
        test_vectors = {f"canonical_{i}": pw[:, i] for i in range(6)}
        test_vectors["compatible_h0_wrench"] = pw @ qp_wrench
        test_vectors["point_realizable_candidate_nominal_physical_solution"] = physical_wrench
        reconstructions = {}
        for name, wrench in test_vectors.items():
            force = np.linalg.pinv(gp, rcond=TOLERANCE) @ wrench
            residual = wrench - gp @ force
            reconstructions[name] = {"wrench": wrench, "minimum_norm_point_force": force,
                                     "residual": residual, "residual_norm": float(np.linalg.norm(residual))}
        reverse = pw @ basis_g - basis_g

        midpoint = np.mean([np.asarray(row["position_world_m"]) for row in side_points], axis=0)
        wheel_center = data.xpos[geometry["body"]].copy()
        reference_checks = {}
        for name, new_reference in (("contact_point_midpoint", midpoint),
                                    ("wheel_center", wheel_center)):
            transported = transport(reference, new_reference, frame)
            transported_back = transport(new_reference, reference, frame)
            gp_direct = point_map(side_points, frame, new_reference)
            pg_new, u_new, _, rank_new = orthogonal_projector(gp_direct)
            null_new = u_new[:, rank_new]
            reference_checks[name] = {
                "reference_world_m": new_reference,
                "transported_gp_parity_max_abs": float(np.max(np.abs(transported @ gp - gp_direct))),
                "round_trip_max_abs": float(np.max(np.abs(transported_back @ transported - np.eye(6)))),
                "rank": rank_new, "left_null_direction": null_new,
                "pure_axis_collinearity": abs(float(null_new @ null_p)),
                "projector": pg_new,
            }

        prior_side = prior["sides"][side_name]
        oracle_parity = {
            "rank_equal": rank == prior_side["rank"],
            "singular_values_max_abs": float(np.max(np.abs(singular - prior_side["singular_values"]))),
            "left_null_collinearity": abs(float(null_g @ np.asarray(prior_side["wrench_left_nullspace"]).reshape(6))),
        }
        exact = (matrix_norms(p_to_g)["spectral"] <= TOLERANCE and
                 matrix_norms(g_to_p)["spectral"] <= TOLERANCE and
                 float(np.max(angles)) <= TOLERANCE and
                 max(value["residual_norm"] for value in reconstructions.values()) <= TOLERANCE)
        reference_specific = (reference_checks["contact_point_midpoint"]["pure_axis_collinearity"] >=
                              1.0 - TOLERANCE)
        classifications.append("A-EXACT_SUBSPACE_EQUIVALENCE" if exact else
                               "C-REFERENCE_POINT_MISMATCH" if reference_specific else
                               "B-APPROXIMATE_NOT_EXACT")
        sides[side_name] = {
            "contact_frame": frame, "force_ordering_per_point": ["Fr", "Fl", "Fn"],
            "wrench_ordering": ["Fr", "Fl", "Fn", "Mr", "Ml", "Mn"],
            "production_reference_world_m": reference,
            "contact_points_world_m": [row["position_world_m"] for row in sorted(side_points, key=lambda x: x["point_index"])],
            "Gp": gp, "Pg": pg, "Pw": pw, "rank_Gp": rank,
            "rank_Pg": int(np.linalg.matrix_rank(pg, tol=TOLERANCE)),
            "rank_Pw": int(np.linalg.matrix_rank(pw, tol=TOLERANCE)),
            "singular_values_Gp": singular,
            "nonzero_condition_number_Gp": float(singular[0] / singular[rank - 1]),
            "point_force_nullspace_dimension": int(gp.shape[1] - rank),
            "point_force_nullspace_basis": vh[rank:].T,
            "actual_missing_wrench_direction": null_g,
            "current_Pw_removed_direction": null_p,
            "missing_direction_collinearity": abs(float(null_g @ null_p)),
            "actual_missing_direction_is_exactly_pure_Ml": abs(float(null_g @ null_p)) >= 1.0 - TOLERANCE,
            "projector_checks": {
                "Pg_symmetry": matrix_norms(pg - pg.T),
                "Pg_idempotence": matrix_norms(pg @ pg - pg),
                "Pw_symmetry": matrix_norms(pw - pw.T),
                "Pw_idempotence": matrix_norms(pw @ pw - pw),
            },
            "projector_difference": matrix_norms(ep),
            "containment_P_to_G": matrix_norms(p_to_g),
            "containment_G_to_P": matrix_norms(g_to_p),
            "principal_angles_rad": angles,
            "maximum_principal_angle_rad": float(np.max(angles)),
            "reconstruction": reconstructions,
            "maximum_reconstruction_residual_norm": max(value["residual_norm"] for value in reconstructions.values()),
            "reverse_reconstruction_residuals": reverse,
            "maximum_reverse_reconstruction_residual_norm": float(max(np.linalg.norm(reverse[:, i]) for i in range(rank))),
            "reference_point_sensitivity": reference_checks,
            "existing_oracle_parity": oracle_parity,
            "classification": classifications[-1],
        }

    classification = (classifications[0] if len(set(classifications)) == 1 else "U-UNTRUSTED")
    result = {
        "scope": "compatible-H0 tick0 frozen Model B actual two-point contacts; algebra only",
        "svd_rcond": TOLERANCE, "equality_tolerance": TOLERANCE,
        "sides": sides, "classification": classification,
        "range_Pw_equals_range_Gp": classification == "A-EXACT_SUBSPACE_EQUIVALENCE",
        "current_R1_repair_exact": classification == "A-EXACT_SUBSPACE_EQUIVALENCE",
        "DG46P_EQ_authoritative_for_exact_point_realizable_repair": classification == "A-EXACT_SUBSPACE_EQUIVALENCE",
        "R2_CONTACT_RESPONSE_MISMATCH_AFTER_R1_authoritative": classification == "A-EXACT_SUBSPACE_EQUIVALENCE",
        "authority_consequence": ("exact R1 repair fails equilibrium" if classification == "A-EXACT_SUBSPACE_EQUIVALENCE"
                                  else "approximate Ml-deletion candidate fails equilibrium"),
    }
    if not all(np.isfinite(value) for side in sides.values() for value in
               [side["maximum_principal_angle_rad"], side["maximum_reconstruction_residual_norm"]]):
        raise RuntimeError("non-finite audit result")
    write_json(output / "point-subspace-equivalence.json", result)

    replay_error = None
    if args.replay_of:
        old = read_json(args.replay_of / "point-subspace-equivalence.json")
        replay_error = ROOT_CAUSE.P45.semantic_error(
            args.replay_of / "point-subspace-equivalence.json",
            output / "point-subspace-equivalence.json")
        if old["classification"] != classification:
            replay_error = float("inf")
    write_json(output / "summary.json", {"pass": classification != "U-UNTRUSTED",
                                           "classification": classification,
                                           "replay_max_abs_error": replay_error,
                                           "replay_pass": replay_error is None or replay_error <= 1.0e-12})
    sources = [source / "probe-records.json", oracle_path, candidate_path, Path(__file__).resolve()]
    write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
    })
    return 0 if classification != "U-UNTRUSTED" and (replay_error is None or replay_error <= 1.0e-12) else 2


if __name__ == "__main__":
    raise SystemExit(main())
