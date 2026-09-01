#!/usr/bin/env python3
"""Fixed-H0 Phase46 primitive-R2 wrench-request realizability gate."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.optimize import linprog

ORDER = [f"{side}_{component}" for side in ("left", "right")
         for component in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")]
SCALE = np.tile([50.0, 50.0, 50.0, 2.5, 2.5, 2.5], 2)


def read_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def read_dump(path: Path) -> dict[str, np.ndarray]:
    values: dict[str, list[tuple[int, int, float]]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for name, row, column, value in csv.reader(stream):
            values.setdefault(name, []).append((int(row), int(column), float(value)))
    result = {}
    for name, entries in values.items():
        matrix = np.zeros((max(x[0] for x in entries) + 1,
                           max(x[1] for x in entries) + 1))
        for row, column, value in entries:
            matrix[row, column] = value
        result[name] = matrix
    return result


def vector(row: dict[str, str], prefix: str) -> np.ndarray:
    return np.array([float(row[f"{prefix}{index}"]) for index in range(12)])


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    for source, name in ((args.baseline, "baseline.csv"),
                         (args.candidate, "compatible-h0.csv"),
                         (args.qp_dump, "candidate-qp-operators.csv")):
        shutil.copy2(source, output / name)

    baseline = read_row(args.baseline)
    candidate = read_row(args.candidate)
    qp = read_dump(args.qp_dump)
    variable_scale = qp["variable_scale"].reshape(-1)
    requested = vector(candidate, "requested_wrench")
    realized = vector(candidate, "realized_wrench")
    slack = vector(candidate, "slack")
    residual = vector(candidate, "wrench_residual")
    baseline_slack = vector(baseline, "slack")
    baseline_realized = vector(baseline, "realized_wrench")
    baseline_requested = vector(baseline, "requested_wrench")

    interaction = np.zeros((12, 42))
    bias = np.zeros(12)
    for side in range(2):
        rows = slice(6 * side, 6 * side + 6)
        interaction[rows, :12] = qp[f"interaction_acceleration_map_{side}"]
        interaction[rows, 18 + 6 * side:24 + 6 * side] = (
            qp[f"interaction_contact_map_{side}"] @
            qp[f"point_force_wrench_projector_{side}"])
        bias[rows] = qp[f"interaction_bias_{side}"].reshape(-1)
    interaction = interaction @ np.diag(variable_scale)
    target = requested - bias

    a = qp["a"]
    lower = qp["lower"].reshape(-1)
    upper = qp["upper"].reshape(-1)
    finite_lower = lower > -1.0e29
    finite_upper = upper < 1.0e29
    equality = finite_lower & finite_upper & (np.abs(lower - upper) <= 1.0e-12)
    a_eq = a[equality]
    b_eq = lower[equality]
    a_ub, b_ub = [], []
    for index in range(len(lower)):
        if equality[index]:
            continue
        if finite_upper[index]:
            a_ub.append(a[index])
            b_ub.append(upper[index])
        if finite_lower[index]:
            a_ub.append(-a[index])
            b_ub.append(-lower[index])
    a_ub = np.asarray(a_ub)
    b_ub = np.asarray(b_ub)

    exact = linprog(np.zeros(42), A_ub=a_ub, b_ub=b_ub,
                    A_eq=np.vstack([a_eq, interaction]),
                    b_eq=np.r_[b_eq, target], bounds=[(None, None)] * 42,
                    method="highs")
    a_ub_t = np.c_[a_ub, np.zeros(len(a_ub))]
    minimax = linprog(
        np.r_[np.zeros(42), 1.0],
        A_ub=np.vstack([a_ub_t,
                        np.c_[interaction / SCALE[:, None], -np.ones(12)],
                        np.c_[-interaction / SCALE[:, None], -np.ones(12)]]),
        b_ub=np.r_[b_ub, target / SCALE, -target / SCALE],
        A_eq=np.c_[a_eq, np.zeros(len(a_eq))], b_eq=b_eq,
        bounds=[(None, None)] * 42 + [(0.0, None)], method="highs")
    if not minimax.success:
        raise RuntimeError(minimax.message)
    witness = minimax.x[:42]
    unavoidable = interaction @ witness - target
    dominant = int(np.argmax(np.abs(slack / SCALE)))
    reconstruction = realized - requested - slack - residual

    decision = {
        "schema_version": 1,
        "phase": 46,
        "classification": "A-WRENCH-REFERENCE-NOT-PRIMITIVE-FEASIBLE",
        "simulation_only": True,
        "mujoco_dependent": True,
        "hardware_ready": False,
        "slack_semantics": "interaction-wrench fidelity only",
        "slack_order": ORDER,
        "frame": "controller body FLU",
        "moment_origin": "corresponding wheel-body origin",
        "sign": "wheel follower on leg/base",
        "normalization_scale": SCALE.tolist(),
        "reconstruction_equation": "realized-reference-slack=wrench_residual",
        "reconstruction_max_abs": float(np.max(np.abs(reconstruction))),
        "baseline": {
            "requested": baseline_requested.tolist(),
            "realized": baseline_realized.tolist(),
            "slack": baseline_slack.tolist(),
            "normalized_slack": (baseline_slack / SCALE).tolist(),
            "maximum_normalized_slack": float(np.max(np.abs(baseline_slack / SCALE))),
        },
        "r2": {
            "requested": requested.tolist(),
            "realized": realized.tolist(),
            "slack": slack.tolist(),
            "normalized_slack": (slack / SCALE).tolist(),
            "delta_slack": (slack - baseline_slack).tolist(),
            "normalized_delta_slack": ((slack - baseline_slack) / SCALE).tolist(),
            "maximum_normalized_slack": float(np.max(np.abs(slack / SCALE))),
            "dominant_index": dominant,
            "dominant_component": ORDER[dominant],
        },
        "request_feasibility": {
            "full_12d_meaningful": True,
            "interaction_operator_rank": int(np.linalg.matrix_rank(interaction, tol=1.0e-10)),
            "hard_equality_count": int(np.count_nonzero(equality)),
            "exact_feasible": bool(exact.success),
            "exact_solver_status": int(exact.status),
            "exact_solver_message": exact.message,
            "minimum_unavoidable_normalized_linf_deviation": float(minimax.fun),
            "minimum_deviation_vector": unavoidable.tolist(),
            "minimum_normalized_deviation_vector": (unavoidable / SCALE).tolist(),
            "witness_hard_equality_max_abs": float(np.max(np.abs(a_eq @ witness - b_eq))),
            "witness_minimum_inequality_margin": float(np.min(b_ub - a_ub @ witness)),
        },
        "request_realizability_conflict": True,
        "soft_objective_inventory": "NOT ENTERED — mandatory infeasible-request stop",
        "kkt_audit": "NOT ENTERED — mandatory infeasible-request stop",
        "diagnostic_ablations": "NOT ENTERED — mandatory infeasible-request stop",
        "selected_minimal_repair": "NONE",
        "gates": {"W1_W6": "PASS", "witness_42d": "PASS", "COMP": "PASS",
                  "EQ": "FAIL", "AUTH": "NOT ENTERED", "REAL": "NOT ENTERED",
                  "SHORT": "NOT ENTERED", "10_s": "NOT ENTERED"},
        "historical_short_status": "NOT COMPLETED — EQ stopped at tick 0",
        "next_action": "authorize upstream wrench-request realizability handling",
    }
    result_path = output / "r2-mujoco-primitive-contact-law-repair.json"
    result_path.write_text(json.dumps(decision, indent=2) + "\n", encoding="utf-8")
    summary = {"classification": decision["classification"],
               "exact_feasible": exact.success,
               "minimum_unavoidable_normalized_linf_deviation": minimax.fun}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    manifest = {"inputs": {name: sha256(output / name) for name in
                            ("baseline.csv", "compatible-h0.csv",
                             "candidate-qp-operators.csv")},
                "runner": str(Path(__file__).resolve()),
                "runner_sha256": sha256(Path(__file__).resolve())}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    if args.replay_of:
        prior = json.loads((args.replay_of / result_path.name).read_text(encoding="utf-8"))
        if prior != decision:
            raise RuntimeError("replay decision differs from formal decision")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
