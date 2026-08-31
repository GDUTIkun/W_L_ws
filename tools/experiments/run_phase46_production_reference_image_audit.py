#!/usr/bin/env python3
"""Build the Phase46 two-point image at the production wrench reference."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
OPERATOR = (PHASE / "evidence/automated/wrench-generalized-force-operator-audit-formal-v2/"
            "wrench-generalized-force-operator-audit.json")
TOL = 1.0e-10


def read(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               default=lambda item: item.tolist()) + "\n", encoding="utf-8")


def norms(value: np.ndarray) -> dict[str, float]:
    return {"spectral": float(np.linalg.norm(value, 2)),
            "frobenius": float(np.linalg.norm(value, "fro")),
            "max_abs": float(np.max(np.abs(value)))}


def semantic_error(left: Path, right: Path) -> float:
    def numbers(value: Any) -> list[float]:
        if isinstance(value, dict):
            return sum((numbers(value[key]) for key in sorted(value)), [])
        if isinstance(value, list):
            return sum((numbers(item) for item in value), [])
        return [float(value)] if isinstance(value, (int, float)) and not isinstance(value, bool) else []
    first, second = numbers(read(left)), numbers(read(right))
    return float("inf") if len(first) != len(second) else max(
        (abs(a - b) for a, b in zip(first, second)), default=0.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    source = read(OPERATOR)
    sides = {}
    all_closed = True
    for side_name in ("left", "right"):
        old = source["sides"][side_name]
        gp_point = np.asarray(old["Gp"])
        transport = np.asarray(old["reference_transport"]["wrench_actual_to_production"])
        gp_prod = transport @ gp_point
        u, singular, vh = np.linalg.svd(gp_prod, full_matrices=True)
        rank = int(np.sum(singular > max(gp_prod.shape) * np.finfo(float).eps * singular[0]))
        pg_prod = u[:, :rank] @ u[:, :rank].T
        pg_current = np.asarray(old["exact_R1_Gp"])
        current_u, _, _ = np.linalg.svd(pg_current, full_matrices=True)
        current_rank = int(np.linalg.matrix_rank(pg_current, tol=TOL))
        p_current = current_u[:, :current_rank] @ current_u[:, :current_rank].T
        difference = p_current - pg_prod
        reconstruction = []
        for index, name in enumerate(old["wrench_order"]):
            wrench = pg_prod[:, index]
            force = np.linalg.pinv(gp_prod, rcond=1.0e-12) @ wrench
            residual = gp_prod @ force - wrench
            reconstruction.append({"component": name, "wrench": wrench,
                                   "point_force": force,
                                   "residual_max_abs": float(np.max(np.abs(residual)))})
        projector_checks = {
            "symmetry_max_abs": float(np.max(np.abs(pg_prod - pg_prod.T))),
            "idempotence_max_abs": float(np.max(np.abs(pg_prod @ pg_prod - pg_prod))),
            "range_containment_max_abs": float(np.max(np.abs((np.eye(6) - pg_prod) @ gp_prod))),
            "maximum_reconstruction_max_abs": max(row["residual_max_abs"] for row in reconstruction),
        }
        operator_checks = {
            "full_max_abs": old["reference_transport"]["transported_full_max_abs"],
            "reduced_max_abs": old["reference_transport"]["transported_reduced_max_abs"],
            "virtual_work_pass": old["virtual_work"]["pass"],
        }
        side_closed = (max(projector_checks.values()) <= TOL and
                       operator_checks["full_max_abs"] <= TOL and
                       operator_checks["reduced_max_abs"] <= TOL and
                       operator_checks["virtual_work_pass"])
        all_closed &= side_closed
        component_difference = np.linalg.norm(difference, axis=0)
        sides[side_name] = {
            "frame": old["contact_frame_world"],
            "wrench_order": old["wrench_order"],
            "force_order": old["point_force_order"],
            "force_sign": old["force_sign"],
            "point_reference_world_m": old["aggregate_reference_world_m"],
            "production_reference_world_m": old["production_twist_reference_world_m"],
            "wrench_transport_point_to_production": transport,
            "twist_dual_transport_production_to_point": transport.T,
            "Gp_point": gp_point, "Gp_production": gp_prod,
            "Pg_production": pg_prod, "rank": rank,
            "singular_values": singular,
            "missing_wrench_direction": u[:, rank],
            "point_force_nullspace": vh[rank:].T,
            "projector_checks": projector_checks,
            "operator_checks": operator_checks,
            "reconstruction": reconstruction,
            "current_projector": p_current,
            "current_projector_rank": current_rank,
            "current_minus_production_projector": difference,
            "projector_difference_norms": norms(difference),
            "dominant_difference_component": old["wrench_order"][int(np.argmax(component_difference))],
            "side_pass": side_closed,
        }

    classification = "A-PRODUCTION-REFERENCE-IMAGE-CLOSED" if all_closed else "U-UNTRUSTED"
    result = {
        "classification": classification,
        "scope": "compatible-H0 tick0 frozen Model B actual two-point geometry; algebra only",
        "production_reference_Gp_pass": all_closed,
        "full_operator_parity": all(side["operator_checks"]["full_max_abs"] <= TOL
                                    for side in sides.values()),
        "reduced_operator_parity": all(side["operator_checks"]["reduced_max_abs"] <= TOL
                                       for side in sides.values()),
        "virtual_work_parity": all(side["operator_checks"]["virtual_work_pass"]
                                   for side in sides.values()),
        "true_production_reference_projector_known": all_closed,
        "current_projector_equals_true_projector": all(
            side["projector_difference_norms"]["spectral"] <= TOL for side in sides.values()),
        "R1_exactly_closed_in_current_controller": all(
            side["projector_difference_norms"]["spectral"] <= TOL for side in sides.values()),
        "sides": sides,
        "next_allowed_action": ("implement one corrected exact-R1 candidate" if all_closed else
                                "attribution only"),
    }
    if not all(np.isfinite(value) for side in sides.values() for value in
               [*side["singular_values"], *side["missing_wrench_direction"],
                *side["projector_difference_norms"].values()]):
        raise RuntimeError("non-finite production-reference image audit")
    decision = output / "production-reference-image-audit.json"
    write(decision, result)
    replay_error = None if args.replay_of is None else semantic_error(
        args.replay_of / decision.name, decision)
    write(output / "summary.json", {"pass": all_closed, "classification": classification,
                                     "replay_max_abs_error": replay_error,
                                     "replay_pass": replay_error is None or replay_error <= 1.0e-12})
    write(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"numpy": np.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in (OPERATOR, Path(__file__).resolve())},
    })
    return 0 if all_closed else 2


if __name__ == "__main__":
    raise SystemExit(main())
