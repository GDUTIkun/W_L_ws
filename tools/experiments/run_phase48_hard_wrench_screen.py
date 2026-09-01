#!/usr/bin/env python3
"""Phase48-B fixed-H0, hard-only wrench realizability screen."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import scipy
from scipy.linalg import null_space
from scipy.optimize import linprog, minimize

ORDER = [f"{side}_{component}" for side in ("left", "right")
         for component in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")]
COMPONENTS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
SCALE6 = np.array([50.0, 50.0, 50.0, 2.5, 2.5, 2.5])
SCALE = np.tile(SCALE6, 2)
RANK_TOL = 1.0e-10
RESIDUAL_TOL = 1.0e-7
MARGIN_TOL = 1.0e-8
PROJECTION_TOL = 1.0e-9
ACTIVE_TOL = 1.0e-7
NEAR_ACTIVE_TOL = 1.0e-5
REPLAY_TOL = 1.0e-9
TIE_TOL = 1.0e-9
SMALL_MAGNITUDE = 0.05
NOMINAL_ANCHOR = 0.07832043067340007
NOMINAL_ANCHOR_TOL = 1.0e-9


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def read_dump(path: Path) -> dict[str, np.ndarray]:
    entries: dict[str, list[tuple[int, int, float]]] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for name, row, column, value in csv.reader(stream):
            entries.setdefault(name, []).append((int(row), int(column), float(value)))
    result: dict[str, np.ndarray] = {}
    for name, values in entries.items():
        matrix = np.zeros((max(v[0] for v in values) + 1,
                           max(v[1] for v in values) + 1))
        for row, column, value in values:
            matrix[row, column] = value
        result[name] = matrix
    return result


def csv_vector(row: dict[str, str], prefix: str) -> np.ndarray:
    return np.array([float(row[f"{prefix}{index}"]) for index in range(12)])


def git_value(*args: str) -> str:
    completed = subprocess.run(["git", *args], check=True, text=True,
                               capture_output=True)
    return completed.stdout.strip()


def independent_columns(matrix: np.ndarray) -> list[int]:
    selected: list[int] = []
    rank = 0
    for column in range(matrix.shape[1]):
        candidate = selected + [column]
        new_rank = int(np.linalg.matrix_rank(matrix[:, candidate], tol=RANK_TOL))
        if new_rank > rank:
            selected.append(column)
            rank = new_rank
    return selected


class HardProblem:
    def __init__(self, qp: dict[str, np.ndarray]) -> None:
        self.qp = qp
        self.variable_scale = qp["variable_scale"].reshape(-1)
        self.projector = np.zeros((12, 12))
        self.interaction = np.zeros((12, 42))
        self.bias = np.zeros(12)
        for side in range(2):
            rows = slice(6 * side, 6 * side + 6)
            pg = qp[f"point_force_wrench_projector_{side}"]
            self.projector[rows, rows] = pg
            self.interaction[rows, :12] = qp[f"interaction_acceleration_map_{side}"]
            self.interaction[rows, 18 + 6 * side:24 + 6 * side] = (
                qp[f"interaction_contact_map_{side}"] @ pg)
            self.bias[rows] = qp[f"interaction_bias_{side}"].reshape(-1)
        self.interaction = self.interaction @ np.diag(self.variable_scale)

        matrix = qp["a"]
        lower = qp["lower"].reshape(-1)
        upper = qp["upper"].reshape(-1)
        finite_lower = lower > -1.0e29
        finite_upper = upper < 1.0e29
        self.equality_mask = (finite_lower & finite_upper &
                              (np.abs(lower - upper) <= 1.0e-12))
        self.a_eq = matrix[self.equality_mask]
        self.b_eq = lower[self.equality_mask]
        inequality_rows: list[np.ndarray] = []
        inequality_bounds: list[float] = []
        inequality_sources: list[int] = []
        for row in range(len(lower)):
            if self.equality_mask[row]:
                continue
            if finite_upper[row]:
                inequality_rows.append(matrix[row])
                inequality_bounds.append(upper[row])
                inequality_sources.append(row)
            if finite_lower[row]:
                inequality_rows.append(-matrix[row])
                inequality_bounds.append(-lower[row])
                inequality_sources.append(row)
        self.a_ub = np.asarray(inequality_rows)
        self.b_ub = np.asarray(inequality_bounds)
        self.inequality_sources = np.asarray(inequality_sources)

    def wrench(self, x: np.ndarray) -> np.ndarray:
        return self.interaction @ x + self.bias

    def exact(self, request: np.ndarray):
        target = request - self.bias
        return linprog(np.zeros(42), A_ub=self.a_ub, b_ub=self.b_ub,
                       A_eq=np.vstack([self.a_eq, self.interaction]),
                       b_eq=np.r_[self.b_eq, target],
                       bounds=[(None, None)] * 42, method="highs")

    def minimax(self, request: np.ndarray):
        target = request - self.bias
        return linprog(
            np.r_[np.zeros(42), 1.0],
            A_ub=np.vstack([
                np.c_[self.a_ub, np.zeros(len(self.a_ub))],
                np.c_[self.interaction / SCALE[:, None], -np.ones(12)],
                np.c_[-self.interaction / SCALE[:, None], -np.ones(12)],
            ]),
            b_ub=np.r_[self.b_ub, target / SCALE, -target / SCALE],
            A_eq=np.c_[self.a_eq, np.zeros(len(self.a_eq))], b_eq=self.b_eq,
            bounds=[(None, None)] * 42 + [(0.0, None)], method="highs")

    def r1_interior_anchor(self) -> tuple[np.ndarray, np.ndarray, float]:
        outside = np.eye(12) - self.projector
        equalities = np.vstack([self.a_eq, outside @ self.interaction])
        rhs = np.r_[self.b_eq, -outside @ self.bias]
        result = linprog(
            np.r_[np.zeros(42), -1.0],
            A_ub=np.c_[self.a_ub, np.ones(len(self.a_ub))], b_ub=self.b_ub,
            A_eq=np.c_[equalities, np.zeros(len(equalities))], b_eq=rhs,
            bounds=[(None, None)] * 42 + [(0.0, None)], method="highs")
        if not result.success or result.x[-1] <= NEAR_ACTIVE_TOL:
            raise RuntimeError(f"no comfortable R1 hard-feasible anchor: {result.message}")
        return self.wrench(result.x[:42]), result.x[:42], float(result.x[-1])

    def tie_break(self, request: np.ndarray, minimax_result):
        x_start = minimax_result.x[:42]
        t_star = float(minimax_result.x[-1])
        x_particular = np.linalg.lstsq(self.a_eq, self.b_eq, rcond=RANK_TOL)[0]
        basis = null_space(self.a_eq, rcond=RANK_TOL)
        y_start = np.linalg.lstsq(basis, x_start - x_particular,
                                 rcond=RANK_TOL)[0]
        a_poly = np.vstack([
            self.a_ub,
            self.interaction / SCALE[:, None],
            -self.interaction / SCALE[:, None],
        ])
        b_poly = np.r_[
            self.b_ub,
            (request - self.bias) / SCALE + t_star + TIE_TOL,
            -(request - self.bias) / SCALE + t_star + TIE_TOL,
        ]
        reduced_a = a_poly @ basis
        reduced_b = b_poly - a_poly @ x_particular

        def objective(y: np.ndarray) -> float:
            deviation = (self.wrench(x_particular + basis @ y) - request) / SCALE
            return float(deviation @ deviation)

        result = minimize(objective, y_start, method="SLSQP",
                          constraints={"type": "ineq",
                                       "fun": lambda y: reduced_b - reduced_a @ y},
                          options={"ftol": 1.0e-13, "maxiter": 2000,
                                   "disp": False})
        return result, x_particular + basis @ result.x, t_star


def direction_record(problem: HardProblem, mode: str, component: str) -> dict:
    index = COMPONENTS.index(component)
    sides = []
    side_meta = []
    for side in range(2):
        pg = problem.projector[6 * side:6 * side + 6,
                               6 * side:6 * side + 6]
        raw = np.eye(6)[:, index]
        projected = pg @ raw
        divisor = float(np.max(np.abs(projected) / SCALE6))
        normalized = projected / divisor
        sides.append(normalized)
        side_meta.append({
            "side": ("left", "right")[side],
            "canonical": raw.tolist(),
            "projected": projected.tolist(),
            "normalized_projected": normalized.tolist(),
            "projection_residual": float(np.max(np.abs((np.eye(6) - pg) @ normalized))),
        })
    right_sign = 1.0 if mode == "common" else -1.0
    return {"source_canonical_direction": component,
            "construction": mode, "side_metadata": side_meta,
            "direction_12d": np.r_[sides[0], right_sign * sides[1]].tolist()}


def row_family(source: int) -> str:
    if 12 <= source < 18:
        return "torque"
    if 18 <= source < 92:
        return "cone_unilateral"
    if 92 <= source < 104:
        return "acceleration"
    return "other"


def metrics(problem: HardProblem, x: np.ndarray, request: np.ndarray) -> dict:
    wrench = problem.wrench(x)
    deviation = wrench - request
    margins = problem.b_ub - problem.a_ub @ x
    hard_eq = problem.a_eq @ x - problem.b_eq
    # The primitive profile owns rows 105:117; select by original row identity.
    original_equalities = np.flatnonzero(problem.equality_mask)
    primitive_mask = original_equalities >= 105
    primitive_residual = (hard_eq[primitive_mask] if np.any(primitive_mask)
                          else np.zeros(1))
    r1_residual = 0.0
    for side in range(2):
        rows = slice(6 * side, 6 * side + 6)
        pg = problem.projector[rows, rows]
        physical_decision = (problem.variable_scale[18 + 6 * side:24 + 6 * side] *
                             x[18 + 6 * side:24 + 6 * side])
        projected_decision = pg @ physical_decision
        r1_residual = max(r1_residual, float(np.max(np.abs(
            (np.eye(6) - pg) @ projected_decision))))
    active = np.flatnonzero(margins <= ACTIVE_TOL)
    near = np.flatnonzero((margins > ACTIVE_TOL) &
                          (margins <= NEAR_ACTIVE_TOL))
    family_margins = {}
    for family in ("torque", "cone_unilateral", "acceleration", "other"):
        selected = [margins[i] for i, source in enumerate(problem.inequality_sources)
                    if row_family(int(source)) == family]
        family_margins[family] = float(min(selected)) if selected else None
    dominant = int(np.argmax(np.abs(deviation / SCALE)))
    return {
        "wrench": wrench.tolist(), "physical_deviation": deviation.tolist(),
        "normalized_deviation": (deviation / SCALE).tolist(),
        "dominant_index": dominant, "dominant_channel": ORDER[dominant],
        "dominant_wheel": ORDER[dominant].split("_")[0],
        "dominant_component": ORDER[dominant].split("_")[1],
        "hard_equality_residual": float(np.max(np.abs(hard_eq))),
        "wrench_equality_residual": float(np.max(np.abs(deviation))),
        "primitive_residual": float(np.max(np.abs(primitive_residual))),
        "r1_residual": float(r1_residual),
        "minimum_inequality_margin": float(np.min(margins)),
        "minimum_torque_margin": family_margins["torque"],
        "minimum_cone_unilateral_margin": family_margins["cone_unilateral"],
        "minimum_acceleration_margin": family_margins["acceleration"],
        "minimum_other_margin": family_margins["other"],
        "active_hard_constraints": [int(problem.inequality_sources[i]) for i in active],
        "near_active_hard_constraints": [int(problem.inequality_sources[i]) for i in near],
        "active_constraint_signature": ",".join(
            str(int(problem.inequality_sources[i])) for i in active),
    }


def solve_case(problem: HardProblem, catalogue: dict) -> dict:
    request = np.asarray(catalogue["w_ref"])
    started = time.perf_counter()
    exact = problem.exact(request)
    if exact.success:
        witness = exact.x
        minimum = 0.0
        tie_status = "NOT REQUIRED — exact feasible"
        verdict = "EXACT HARD-FEASIBLE"
        solver_iterations = exact.nit
        solver_message = exact.message
    elif exact.status == 2:
        minimax = problem.minimax(request)
        if not minimax.success or not np.isfinite(minimax.fun):
            return {"case_id": catalogue["case_id"], "classification": "UNTRUSTED",
                    "exact_hard_feasible": "UNTRUSTED",
                    "feasibility_solver_status": exact.message,
                    "closest_solver_status": minimax.message}
        tie, witness, minimum = problem.tie_break(request, minimax)
        if not tie.success:
            return {"case_id": catalogue["case_id"], "classification": "UNTRUSTED",
                    "exact_hard_feasible": "UNTRUSTED",
                    "feasibility_solver_status": exact.message,
                    "closest_solver_status": tie.message}
        tie_status = tie.message
        verdict = "HARD-INFEASIBLE"
        solver_iterations = minimax.nit + tie.nit
        solver_message = exact.message
    else:
        return {"case_id": catalogue["case_id"], "classification": "UNTRUSTED",
                "exact_hard_feasible": "UNTRUSTED",
                "feasibility_solver_status": exact.message}
    values = metrics(problem, witness, request)
    achieved_minimum = float(np.max(np.abs(np.asarray(values["normalized_deviation"]))))
    trusted = (np.isfinite(witness).all() and
               values["hard_equality_residual"] <= RESIDUAL_TOL and
               values["minimum_inequality_margin"] >= -MARGIN_TOL and
               values["primitive_residual"] <= RESIDUAL_TOL and
               values["r1_residual"] <= RESIDUAL_TOL)
    if verdict == "EXACT HARD-FEASIBLE":
        trusted &= values["wrench_equality_residual"] <= RESIDUAL_TOL
    else:
        trusted &= achieved_minimum <= minimum + TIE_TOL + RESIDUAL_TOL
    if not trusted:
        verdict = "UNTRUSTED"
    return {
        "case_id": catalogue["case_id"], "request_label": catalogue["request_label"],
        "state_id": "H0", "request_mode": catalogue["request_mode"],
        "branch": catalogue["branch"], "magnitude": catalogue["magnitude"],
        "r1_physical": catalogue["r1_physical"],
        "r1_image_residual": catalogue["r1_image_residual"],
        "w_ref": request.tolist(), "classification": verdict,
        "exact_hard_feasible": "YES" if verdict == "EXACT HARD-FEASIBLE" else
                               ("NO" if verdict == "HARD-INFEASIBLE" else "UNTRUSTED"),
        "feasibility_solver_status": solver_message,
        "minimum_normalized_linf": float(minimum),
        "tie_break_achieved_normalized_linf": achieved_minimum,
        "closest_hard_feasible_wrench": values.pop("wrench"),
        **values, "tie_break_status": tie_status,
        "solver_iterations": int(solver_iterations),
        "solver_time_s": time.perf_counter() - started,
        "duals_if_available": "HiGHS exact/minimax marginals retained only by solver; not used",
        "deep_hard_attribution": "NOT ENTERED",
        "task_competition_attribution": "NOT ENTERED",
        "plant_realization": "NOT ENTERED",
    }


def flatten_case(case: dict) -> dict:
    row = {key: value for key, value in case.items()
           if not isinstance(value, (list, dict))}
    vector_fields = {
        "w_ref": "w_ref", "closest_hard_feasible_wrench": "closest_w",
        "physical_deviation": "physical_deviation",
        "normalized_deviation": "normalized_deviation",
    }
    for field, prefix in vector_fields.items():
        for index, value in enumerate(case[field]):
            row[f"{prefix}_{ORDER[index]}"] = value
    row["active_hard_constraints"] = ",".join(map(str, case["active_hard_constraints"]))
    row["near_active_hard_constraints"] = ",".join(
        map(str, case["near_active_hard_constraints"]))
    return row


def stable_case(case: dict) -> dict:
    return {key: value for key, value in case.items()
            if key not in ("solver_time_s",)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")

    qp = read_dump(args.qp_dump)
    problem = HardProblem(qp)
    ranks = [int(np.linalg.matrix_rank(
        qp[f"point_force_wrench_projector_{side}"], tol=RANK_TOL))
             for side in range(2)]
    bases = [independent_columns(qp[f"point_force_wrench_projector_{side}"])
             for side in range(2)]
    if ranks != [5, 5] or any(len(basis) != 5 for basis in bases):
        raise RuntimeError(f"STOP-B projector rank/basis regression: {ranks}, {bases}")

    anchor, _, anchor_margin = problem.r1_interior_anchor()
    nominal = csv_vector(read_row(args.candidate), "requested_wrench")
    catalogue: list[dict] = [{
        "case_id": "P48B-H0-NOMINAL", "request_label": "authoritative_nominal_h0",
        "state_id": "H0", "request_mode": "nominal", "branch": "nominal",
        "magnitude": 0.0, "primary": False, "regression_anchor": True,
        "source_canonical_direction": None, "projected_direction": None,
        "basis_rank_metadata": {"left_rank": ranks[0], "right_rank": ranks[1]},
        "construction": "authoritative Phase48-A request; R1 exception for regression only",
        "r1_physical": "NOT APPLICABLE — frozen nominal regression anchor",
        "r1_image_residual": float(np.max(np.abs((np.eye(12) - problem.projector) @ nominal))),
        "w_ref": nominal.tolist(),
    }, {
        "case_id": "P48B-H0-R1-INTERIOR", "request_label": "r1_hard_interior_anchor",
        "state_id": "H0", "request_mode": "nominal", "branch": "anchor",
        "magnitude": 0.0, "primary": True, "regression_anchor": False,
        "source_canonical_direction": None, "projected_direction": None,
        "basis_rank_metadata": {"left_rank": ranks[0], "right_rank": ranks[1]},
        "construction": "hard-only maximum-minimum-row-margin witness in corrected-R1 image",
        "anchor_minimum_row_margin": anchor_margin, "r1_physical": "PASS",
        "r1_image_residual": float(np.max(np.abs((np.eye(12) - problem.projector) @ anchor))),
        "w_ref": anchor.tolist(),
    }]
    specifications = (("common", "Fx"), ("common", "Fz"), ("common", "Ty"),
                      ("differential", "Fy"), ("differential", "Tx"),
                      ("differential", "Tz"))
    for mode, component in specifications:
        direction = direction_record(problem, mode, component)
        vector = np.asarray(direction["direction_12d"])
        for sign, branch in ((1.0, "positive"), (-1.0, "negative")):
            request = anchor + sign * SMALL_MAGNITUDE * vector
            residual = float(np.max(np.abs((np.eye(12) - problem.projector) @ request)))
            catalogue.append({
                "case_id": f"P48B-H0-{mode.upper()}-{component.upper()}-{branch.upper()}",
                "request_label": f"{mode}_{component}", "state_id": "H0",
                "request_mode": mode, "branch": branch,
                "magnitude": SMALL_MAGNITUDE, "primary": True,
                "regression_anchor": False, **direction,
                "projected_direction": vector.tolist(),
                "basis_rank_metadata": {
                    "left_rank": ranks[0], "right_rank": ranks[1],
                    "left_independent_canonical_columns": [COMPONENTS[i] for i in bases[0]],
                    "right_independent_canonical_columns": [COMPONENTS[i] for i in bases[1]],
                    "direction_is_independent_basis_member": component in
                        [COMPONENTS[i] for i in bases[0]],
                },
                "sign": sign, "r1_physical": "PASS" if residual <= PROJECTION_TOL else "FAIL",
                "r1_image_residual": residual, "w_ref": request.tolist(),
            })
    if any(item["primary"] and item["r1_physical"] != "PASS" for item in catalogue):
        raise RuntimeError("STOP-B primary request outside corrected-R1 image")

    cases = [solve_case(problem, item) for item in catalogue]
    nominal_case = cases[0]
    nominal_pass = (nominal_case["classification"] == "HARD-INFEASIBLE" and
                    abs(nominal_case["minimum_normalized_linf"] - NOMINAL_ANCHOR)
                    <= NOMINAL_ANCHOR_TOL)
    trusted = all(case["classification"] != "UNTRUSTED" for case in cases)
    classifications = {name: sum(case["classification"] == name for case in cases[1:])
                       for name in ("EXACT HARD-FEASIBLE", "HARD-INFEASIBLE", "UNTRUSTED")}
    gate_pass = (nominal_pass and trusted and classifications["EXACT HARD-FEASIBLE"] >= 1 and
                 classifications["HARD-INFEASIBLE"] >= 1)
    summary = {
        "schema_version": 1, "phase": 48, "task_id": "P48-T03",
        "verdict": "PASS" if gate_pass else "BLOCKED",
        "state": "authoritative fixed H0", "nominal_h0_regression": nominal_pass,
        "primary_request_basis": "R1-PHYSICAL", "left_physical_rank": ranks[0],
        "right_physical_rank": ranks[1], "number_of_primary_cases": len(cases) - 1,
        "counts_excluding_nominal_regression_anchor": classifications,
        "representative_feasible_cases": [c["case_id"] for c in cases
                                           if c["classification"] == "EXACT HARD-FEASIBLE"],
        "representative_infeasible_cases": [c["case_id"] for c in cases[1:]
                                             if c["classification"] == "HARD-INFEASIBLE"],
        "nominal_minimum_normalized_linf": nominal_case["minimum_normalized_linf"],
        "worst_minimum_normalized_linf": max(c["minimum_normalized_linf"] for c in cases),
        "worst_case": max(cases, key=lambda c: c["minimum_normalized_linf"])["case_id"],
        "dominant_deviation_channels": sorted({c["dominant_channel"] for c in cases
                                                if c["classification"] == "HARD-INFEASIBLE"}),
        "soft_objectives_used_in_feasibility_verdict": False,
        "closest_wrench_solve": trusted, "deterministic_tie_break": trusted,
        "semantic_regression": False, "r1_w5_w1_w6_regression": False,
        "p48_t04": "NOT ENTERED", "p48_t05": "NOT ENTERED",
        "p48_t06_plus": "NOT ENTERED", "p48_t03_closed": gate_pass,
    }

    output.mkdir(parents=True)
    shutil.copy2(args.candidate, output / "compatible-h0.csv")
    shutil.copy2(args.qp_dump, output / "candidate-qp-operators.csv")
    catalogue_doc = {
        "schema_version": 1, "task_id": "P48-T03", "state_id": "H0",
        "normalization_scale": SCALE.tolist(), "small_magnitude": SMALL_MAGNITUDE,
        "rank_tolerance": RANK_TOL, "projection_tolerance": PROJECTION_TOL,
        "left_projector_rank": ranks[0], "right_projector_rank": ranks[1],
        "left_independent_columns": [COMPONENTS[i] for i in bases[0]],
        "right_independent_columns": [COMPONENTS[i] for i in bases[1]],
        "requests": catalogue,
    }
    catalogue_text = json.dumps(catalogue_doc, indent=2) + "\n"
    (output / "request-catalogue.json").write_text(catalogue_text, encoding="utf-8")
    (output / "phase48-b-request-catalogue.json").write_text(catalogue_text, encoding="utf-8")
    (output / "hard-realizability-summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (output / "closest-feasible-wrenches.csv").write_text("", encoding="utf-8")
    rows = [flatten_case(case) for case in cases]
    for filename in ("hard-realizability-probes.csv", "closest-feasible-wrenches.csv"):
        with (output / filename).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    manifest = {
        "schema_version": 1, "phase": 48, "task_id": "P48-T03",
        "command": " ".join(sys.argv), "interpreter": sys.executable,
        "python": platform.python_version(), "numpy": np.__version__,
        "scipy": scipy.__version__, "solver": "SciPy HiGHS LP + SLSQP convex L2 tie-break",
        "solver_settings": {"linprog_method": "highs", "slsqp_ftol": 1.0e-13,
                            "slsqp_maxiter": 2000},
        "thresholds": {"rank": RANK_TOL, "residual": RESIDUAL_TOL,
                       "margin": MARGIN_TOL, "projection": PROJECTION_TOL,
                       "active": ACTIVE_TOL, "near_active": NEAR_ACTIVE_TOL,
                       "replay": REPLAY_TOL, "tie": TIE_TOL,
                       "nominal_anchor": NOMINAL_ANCHOR_TOL},
        "normalization_scale": SCALE.tolist(), "seed": None,
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_dirty_files": git_value("status", "--short").splitlines(),
        "inputs": {"compatible-h0.csv": sha256(output / "compatible-h0.csv"),
                   "candidate-qp-operators.csv": sha256(output / "candidate-qp-operators.csv")},
        "request_catalogue_sha256": sha256(output / "request-catalogue.json"),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "soft_objectives_used_in_verdict": False,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n",
                                           encoding="utf-8")

    replay = {"replay_of": None, "case_count_exact": None, "request_hash_exact": None,
              "classification_exact": None, "numeric_within_tolerance": None,
              "closest_wrench_tie_break_deterministic": None, "verdict": "FORMAL"}
    if args.replay_of:
        prior_catalogue = args.replay_of / "request-catalogue.json"
        prior_cases_path = args.replay_of / "hard-realizability-probes.csv"
        with prior_cases_path.open(newline="", encoding="utf-8") as stream:
            prior_rows = list(csv.DictReader(stream))
        current_stable = [stable_case(case) for case in cases]
        # JSON round-trip gives a type-stable prior representation for numeric comparison.
        prior_summary = json.loads((args.replay_of / "hard-realizability-summary.json").read_text())
        request_hash_exact = sha256(prior_catalogue) == sha256(output / "request-catalogue.json")
        classifications_exact = [row["classification"] for row in prior_rows] == [
            case["classification"] for case in cases]
        numeric_ok = True
        vector_columns = [f"{prefix}_{name}" for prefix in
                          ("w_ref", "closest_w", "physical_deviation", "normalized_deviation")
                          for name in ORDER]
        scalar_columns = ["minimum_normalized_linf", "hard_equality_residual",
                          "primitive_residual", "r1_residual", "minimum_inequality_margin"]
        for prior_row, current_row in zip(prior_rows, rows):
            for column in vector_columns + scalar_columns:
                numeric_ok &= abs(float(prior_row[column]) - float(current_row[column])) <= REPLAY_TOL
        replay = {
            "replay_of": str(args.replay_of.resolve()),
            "case_count_exact": len(prior_rows) == len(cases),
            "request_hash_exact": request_hash_exact,
            "classification_exact": classifications_exact,
            "numeric_within_tolerance": bool(numeric_ok),
            "closest_wrench_tie_break_deterministic": bool(numeric_ok),
            "formal_verdict": prior_summary["verdict"],
        }
        replay["verdict"] = "PASS" if all(value is True for key, value in replay.items()
                                              if key not in ("replay_of", "formal_verdict")) else "FAIL"
        if replay["verdict"] != "PASS":
            summary["verdict"] = "BLOCKED"
            summary["p48_t03_closed"] = False
            (output / "hard-realizability-summary.json").write_text(
                json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    (output / "fresh-replay-summary.json").write_text(json.dumps(replay, indent=2) + "\n",
                                                       encoding="utf-8")
    return 0 if summary["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
