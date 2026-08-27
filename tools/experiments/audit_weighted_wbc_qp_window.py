#!/usr/bin/env python3
"""Independently audit saved Phase-21 QPs with SciPy HiGHS and SLSQP."""

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
import scipy
from scipy.optimize import linprog, minimize


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def constraints(a: np.ndarray, lower: np.ndarray, upper: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    equality = np.isfinite(lower) & np.isfinite(upper) & (np.abs(lower - upper) <= 1e-12)
    a_eq = a[equality]; b_eq = 0.5 * (lower[equality] + upper[equality])
    a_ub = []; b_ub = []
    for index in np.flatnonzero(~equality):
        if np.isfinite(upper[index]):
            a_ub.append(a[index]); b_ub.append(upper[index])
        if np.isfinite(lower[index]):
            a_ub.append(-a[index]); b_ub.append(-lower[index])
    return a_eq, b_eq, np.asarray(a_ub), np.asarray(b_ub)


def violation(a: np.ndarray, lower: np.ndarray, upper: np.ndarray, x: np.ndarray) -> float:
    ax = a @ x
    return float(max(0.0, np.max(lower - ax), np.max(ax - upper)))


def objective(h: np.ndarray, g: np.ndarray, x: np.ndarray) -> float:
    return float(0.5 * x @ h @ x + g @ x)


def audit_problem(h: np.ndarray, g: np.ndarray, a: np.ndarray, lower: np.ndarray,
                  upper: np.ndarray, admm_x: np.ndarray) -> dict[str, Any]:
    a_eq, b_eq, a_ub, b_ub = constraints(a, lower, upper)
    feasibility = linprog(np.zeros(h.shape[0]), A_ub=a_ub, b_ub=b_ub,
                          A_eq=a_eq, b_eq=b_eq, bounds=[(None, None)] * h.shape[0],
                          method="highs", options={"dual_feasibility_tolerance": 1e-9,
                                                   "primal_feasibility_tolerance": 1e-9})
    lp_x = feasibility.x if feasibility.success else np.full(h.shape[0], np.nan)
    lp_violation = violation(a, lower, upper, lp_x) if feasibility.success else None
    qp = None
    if feasibility.success:
        scipy_constraints = []
        if a_eq.size:
            scipy_constraints.append({"type": "eq", "fun": lambda x: a_eq @ x - b_eq,
                                      "jac": lambda x: a_eq})
        if a_ub.size:
            scipy_constraints.append({"type": "ineq", "fun": lambda x: b_ub - a_ub @ x,
                                      "jac": lambda x: -a_ub})
        qp = minimize(lambda x: objective(h, g, x), lp_x,
                      jac=lambda x: h @ x + g, constraints=scipy_constraints,
                      method="SLSQP", options={"ftol": 1e-12, "maxiter": 2000, "disp": False})
    qp_x = qp.x if qp is not None else np.full(h.shape[0], np.nan)
    qp_violation = violation(a, lower, upper, qp_x) if qp is not None else None
    admm_violation = violation(a, lower, upper, admm_x)
    regularized = h + 1e-6 * np.eye(h.shape[0]) + 10.0 * a.T @ a
    return {
        "lp_feasible": bool(feasibility.success), "lp_status": int(feasibility.status),
        "lp_message": str(feasibility.message), "lp_bound_violation": lp_violation,
        "qp_success": bool(qp.success) if qp is not None else False,
        "qp_status": int(qp.status) if qp is not None else None,
        "qp_message": str(qp.message) if qp is not None else "not_run",
        "qp_iterations": int(qp.nit) if qp is not None else 0,
        "qp_bound_violation": qp_violation,
        "qp_objective": objective(h, g, qp_x) if qp is not None else None,
        "admm_bound_violation_recomputed": admm_violation,
        "admm_objective": objective(h, g, admm_x),
        "admm_qp_solution_inf_difference": float(np.max(np.abs(admm_x - qp_x))) if qp is not None else None,
        "normal_matrix_condition_number": float(np.linalg.cond(regularized)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--attribution-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(); source = args.attribution_dir.resolve(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    summary_path = source / "summary.json"
    attribution = json.loads(summary_path.read_text(encoding="utf-8"))
    rows = []; cases: dict[str, Any] = {}
    for case in attribution["results"]:
        capture_path = source / case / "failure_window.npz"
        capture = np.load(capture_path)
        case_rows = []
        for index, tick in enumerate(capture["tick"]):
            result = audit_problem(capture["H"][index], capture["g"][index], capture["A"][index],
                                   capture["l"][index], capture["u"][index],
                                   capture["scaled_solution"][index])
            admm_converged = bool(capture["solver_status"][index])
            if not admm_converged and result["lp_feasible"]:
                classification = "admm_numerical_limit_on_feasible_problem"
            elif not admm_converged:
                classification = "mathematical_infeasibility_supported"
            elif result["qp_success"]:
                classification = "admm_and_independent_qp_converged"
            else:
                classification = "admm_converged_independent_qp_inconclusive"
            row = {"case": case, "tick": int(tick), "admm_converged": admm_converged,
                   "admm_iterations": int(capture["iterations"][index]),
                   "admm_primal_residual": float(capture["primal_residual"][index]),
                   "admm_dual_residual": float(capture["dual_residual"][index]),
                   "admm_stationarity_residual": float(capture["stationarity_residual"][index]),
                   "admm_reported_bound_violation": float(capture["bound_violation"][index]),
                   "classification": classification, **result}
            rows.append(row); case_rows.append(row)
        failures = [row for row in case_rows if not row["admm_converged"]]
        cases[case] = {
            "ticks_audited": len(case_rows), "admm_failure_count": len(failures),
            "all_lp_feasible": all(row["lp_feasible"] for row in case_rows),
            "independent_qp_success_count": sum(row["qp_success"] for row in case_rows),
            "failure_classifications": {name: sum(row["classification"] == name for row in failures)
                                        for name in sorted({row["classification"] for row in failures})},
            "maximum_lp_bound_violation": max((row["lp_bound_violation"] or 0.0) for row in case_rows),
            "maximum_qp_bound_violation": max((row["qp_bound_violation"] or 0.0) for row in case_rows),
            "maximum_normal_matrix_condition_number": max(row["normal_matrix_condition_number"] for row in case_rows),
        }
    fields = list(rows[0])
    with (output / "ticks.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    summary = {"schema_version": 1, "phase": 21, "purpose": "independent_failure_window_qp_oracle",
               "source": str(source), "oracle": {"feasibility": "SciPy HiGHS double precision",
               "convex_qp": "SciPy SLSQP with analytic objective gradient and linear Jacobians",
               "objective_and_constraints_unchanged": True}, "cases": cases}
    write_json(output / "summary.json", summary)
    write_json(output / "manifest.json", {"schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
        "numpy": np.__version__, "scipy": scipy.__version__,
        "source_summary": str(summary_path), "source_summary_sha256": sha256(summary_path),
        "source_manifest_sha256": sha256(source / "manifest.json"),
        "validator": str(Path(__file__).resolve()), "validator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {"summary.json": sha256(output / "summary.json"),
                    "ticks.csv": sha256(output / "ticks.csv")}})
    print(json.dumps({case: {"failures": value["admm_failure_count"],
                            "classifications": value["failure_classifications"],
                            "qp_success": value["independent_qp_success_count"]}
                      for case, value in cases.items()}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
