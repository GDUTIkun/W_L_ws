#!/usr/bin/env python3
"""Freeze and independently audit the Phase-21 42D hard-QP candidate."""
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
from scipy.optimize import linprog, lsq_linear, minimize

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402
from validate_weighted_wbc_continuous_contact import ContinuousPatch  # noqa: E402
from validate_weighted_wbc_contact_centered_wrench import (  # noqa: E402
    actuator_map,
    build_h,
    geometry_map,
    rays,
    wrench_generalized_map,
)

ROOT = Path(__file__).resolve().parents[2]
NVAR = 42
INF = 1.0e30


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False,
                               default=lambda x: x.item() if isinstance(x, np.generic) else x) + "\n")


def max_abs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value))) if value.size else 0.0


def bound_violation(a: np.ndarray, lower: np.ndarray, upper: np.ndarray,
                    x: np.ndarray) -> float:
    ax = a @ x
    return float(max(0.0, np.max(lower - ax), np.max(ax - upper)))


def split_constraints(a: np.ndarray, lower: np.ndarray, upper: np.ndarray
                      ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    equality = np.abs(lower - upper) <= 1.0e-12
    aeq, beq = a[equality], 0.5 * (lower[equality] + upper[equality])
    rows, rhs = [], []
    for index in np.flatnonzero(~equality):
        if upper[index] < INF:
            rows.append(a[index]); rhs.append(upper[index])
        if lower[index] > -INF:
            rows.append(-a[index]); rhs.append(-lower[index])
    return aeq, beq, np.asarray(rows), np.asarray(rhs)


def independent_oracle(h: np.ndarray, g: np.ndarray, a: np.ndarray,
                       lower: np.ndarray, upper: np.ndarray,
                       settings: dict[str, Any]) -> dict[str, Any]:
    aeq, beq, aub, bub = split_constraints(a, lower, upper)
    lp = linprog(np.zeros(len(g)), A_ub=aub, b_ub=bub, A_eq=aeq, b_eq=beq,
                 bounds=[(None, None)] * len(g), method="highs",
                 options={"primal_feasibility_tolerance": settings["feasibility_tolerance"],
                          "dual_feasibility_tolerance": settings["feasibility_tolerance"]})
    if not lp.success:
        return {"feasible": False, "lp_status": int(lp.status),
                "lp_message": str(lp.message), "qp_success": False}
    constraints = []
    if len(aeq):
        constraints.append({"type": "eq", "fun": lambda x: aeq @ x - beq,
                            "jac": lambda x: aeq})
    if len(aub):
        constraints.append({"type": "ineq", "fun": lambda x: bub - aub @ x,
                            "jac": lambda x: -aub})
    qp = minimize(lambda x: 0.5 * x @ h @ x + g @ x, lp.x,
                  jac=lambda x: h @ x + g, constraints=constraints,
                  method="SLSQP", options={"ftol": settings["qp_ftol"],
                                            "maxiter": settings["maximum_iterations"]})
    x = np.asarray(qp.x)
    slack = bub - aub @ x
    active = slack <= settings["active_tolerance"]
    # Equality multipliers are free; active Cx<=d multipliers are nonnegative.
    kkt = np.column_stack((aeq.T, aub[active].T))
    if kkt.shape[1]:
        multiplier = lsq_linear(kkt, -(h @ x + g),
                                bounds=(np.r_[np.full(len(aeq), -np.inf),
                                              np.zeros(np.count_nonzero(active))],
                                        np.full(kkt.shape[1], np.inf)),
                                lsmr_tol="auto").x
        stationarity = max_abs(h @ x + g + kkt @ multiplier)
        inequality_multiplier = multiplier[len(aeq):]
        complementarity = max_abs(inequality_multiplier * slack[active])
        minimum_dual = float(np.min(inequality_multiplier)) if len(inequality_multiplier) else 0.0
    else:
        stationarity = max_abs(h @ x + g)
        complementarity, minimum_dual = 0.0, 0.0
    return {
        "feasible": True, "lp_status": int(lp.status), "lp_message": str(lp.message),
        "lp_bound_violation": bound_violation(a, lower, upper, lp.x),
        "qp_success": bool(qp.success), "qp_status": int(qp.status),
        "qp_message": str(qp.message), "qp_iterations": int(qp.nit),
        "qp_bound_violation": bound_violation(a, lower, upper, x),
        "qp_stationarity_residual": stationarity,
        "qp_complementarity_residual": complementarity,
        "qp_minimum_inequality_multiplier": minimum_dual,
        "objective": float(0.5 * x @ h @ x + g @ x),
        "x": x,
    }


class HardQpBuilder:
    """Build the hard contract in scaled solver coordinates x, z_physical=D*x."""

    def __init__(self, config: dict[str, Any], model: dict[str, Any],
                 contact: dict[str, Any], equilibrium: dict[str, Any]) -> None:
        self.config, self.model_config = config, model
        self.oracle = Oracle(model, equilibrium)
        continuous, _ = load_config((ROOT / contact["continuous_contact_config"]).resolve())
        self.patch = ContinuousPatch(self.oracle, continuous["continuous_contact_oracle"])
        equilibrium_q = self.oracle.sample_qpos(model["samples"][0])
        _, offsets, _ = geometry_map(self.patch, equilibrium_q, 0)
        self.h_cone, self.hull = build_h(
            rays(offsets, float(contact["friction_coefficient"])),
            contact["hull_qhull_options"])
        scales = config["variable_scale"]
        self.scale = np.asarray(scales["acceleration"] + scales["torque"] +
                                2 * scales["wrench_per_side"] +
                                2 * scales["slack_per_side"], dtype=float)
        self.transform = np.diag(self.scale)
        self.row_map = [
            {"name": "dynamics", "start": 0, "count": 12, "sense": "equality"},
            {"name": "torque_box", "start": 12, "count": 6, "sense": "two_sided"},
            {"name": "left_wrench_cone", "start": 18, "count": len(self.h_cone), "sense": "upper"},
            {"name": "right_wrench_cone", "start": 18 + len(self.h_cone), "count": len(self.h_cone), "sense": "upper"},
            {"name": "acceleration_box", "start": 18 + 2 * len(self.h_cone), "count": 12, "sense": "two_sided"},
        ]

    def canonical_qpos(self, captured: np.ndarray) -> np.ndarray:
        q = captured.copy(); q[self.oracle.passive_qpos] = self.oracle.equilibrium_passive
        return self.oracle.solve_passive(q)[0]

    def reduced_velocity(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        self.oracle.forward(qpos)
        linear, angular = self.oracle.site_jacobian(self.oracle.base_control_site)
        return np.r_[linear @ qvel, angular @ qvel, -qvel[self.oracle.active_dofs]]

    def dynamics(self, qpos: np.ndarray, velocity: np.ndarray
                 ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
        reduction, _ = self.oracle.reduction(qpos)
        step = float(self.oracle.config["solver"]["second_difference_step"])
        plus = self.oracle.integrate_flow(qpos, velocity, step)
        minus = self.oracle.integrate_flow(qpos, velocity, -step)
        plus_n, _ = self.oracle.reduction(plus); minus_n, _ = self.oracle.reduction(minus)
        ndot_velocity = ((plus_n - minus_n) / (2.0 * step)) @ velocity
        self.oracle.forward(qpos, reduction @ velocity)
        full_mass = np.zeros((self.oracle.model.nv, self.oracle.model.nv))
        mujoco.mj_fullM(self.oracle.model, full_mass, self.oracle.data.qM)
        mass = reduction.T @ full_mass @ reduction
        bias = reduction.T @ (self.oracle.data.qfrc_bias.copy() + full_mass @ ndot_velocity)
        actuation = actuator_map(self.oracle, reduction)
        wrenches = [wrench_generalized_map(self.oracle, self.patch, qpos, side)[0]
                    for side in range(2)]
        return mass, bias, actuation, wrenches

    def build(self, qpos: np.ndarray, velocity: np.ndarray) -> dict[str, Any]:
        mass, bias, actuation, wrenches = self.dynamics(qpos, velocity)
        physical = np.zeros((12, NVAR)); physical[:, :12] = mass
        physical[:, 12:18] = -actuation
        physical[:, 18:24] = -wrenches[0]; physical[:, 24:30] = -wrenches[1]
        dynamics_scale = np.asarray(self.config["row_scale"]["dynamics"])
        rows = [(physical @ self.transform) / dynamics_scale[:, None]]
        lower = [-bias / dynamics_scale]; upper = [-bias / dynamics_scale]

        torque = np.zeros((6, NVAR)); torque[:, 12:18] = np.eye(6)
        torque_limit = np.asarray(self.config["bounds"]["torque_nm"])
        rows.append((torque @ self.transform) / torque_limit[:, None])
        lower.append(-np.ones(6)); upper.append(np.ones(6))

        cone_norms = []
        wrench_scale = self.scale[18:24]
        for start in (18, 24):
            cone = np.zeros((len(self.h_cone), NVAR)); cone[:, start:start + 6] = self.h_cone
            scaled = cone @ self.transform
            norms = np.linalg.norm(scaled, axis=1); cone_norms.extend(norms.tolist())
            normalized = scaled / norms[:, None]
            rows.append(normalized)
            lower.append(np.full(len(self.h_cone), -INF)); upper.append(np.zeros(len(self.h_cone)))

        acceleration = np.zeros((12, NVAR)); acceleration[:, :12] = np.eye(12)
        acceleration_limit = np.asarray(self.config["bounds"]["acceleration"])
        rows.append((acceleration @ self.transform) / acceleration_limit[:, None])
        lower.append(-np.ones(12)); upper.append(np.ones(12))
        a, lower_v, upper_v = np.vstack(rows), np.concatenate(lower), np.concatenate(upper)
        h = np.eye(NVAR) * float(self.config["minimum_scaled_norm_regularization"])
        regularized = h + self.config["solver"]["sigma"] * np.eye(NVAR) + \
            self.config["solver"]["rho"] * a.T @ a
        return {"H": h, "g": np.zeros(NVAR), "A": a, "l": lower_v, "u": upper_v,
                "physical_dynamics": physical, "physical_rhs": -bias,
                "cone_scaled_norms": np.asarray(cone_norms),
                "cone_normalized_row_norm_error": max_abs(
                    np.linalg.norm(np.vstack(rows[2:4]), axis=1) - 1.0),
                "normal_matrix_condition_number": float(np.linalg.cond(regularized)),
                "wrench_variable_scale": wrench_scale}


def corpus(builder: HardQpBuilder, capture: Any) -> list[tuple[str, np.ndarray, np.ndarray]]:
    values = []
    for sample in builder.model_config["samples"]:
        values.append(("workspace_" + sample["id"], builder.oracle.sample_qpos(sample), np.zeros(12)))
    for tick in builder.config["dynamic_ticks"]:
        q = builder.canonical_qpos(capture["qpos"][tick])
        values.append((f"dynamic_tick_{tick}", q,
                       builder.reduced_velocity(q, capture["qvel"][tick])))
    return values


def write_problem_corpus(path: Path, cases: list[dict[str, Any]], config: dict[str, Any]) -> None:
    # C++ benchmark corpus: each problem carries the independent QP oracle x.
    settings = config["solver"]
    with path.open("w", encoding="utf-8") as stream:
        stream.write(f"DENSE_QP_CORPUS_V1 {len(cases)}\n")
        for case in cases:
            problem = case["problem"]
            stream.write(f"{NVAR} {problem['A'].shape[0]} 12 "
                         f"{settings['rho']:.17g} {settings['sigma']:.17g} "
                         f"{settings['absolute_tolerance']:.17g} "
                         f"{settings['relative_tolerance']:.17g} "
                         f"{int(settings['maximum_iterations'])}\n")
            values = np.r_[problem["H"].ravel(), problem["g"], problem["A"].ravel(),
                           problem["l"], problem["u"]]
            stream.write(" ".join(f"{value:.17g}" for value in values) + "\n")
            stream.write("oracle\n")
            stream.write(" ".join(f"{value:.17g}" for value in case["audit"]["x"]) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config, config_inputs = load_config(args.config.resolve())
    model, model_inputs = load_config((ROOT / config["model_profile"]).resolve())
    contact, contact_inputs = load_config((ROOT / config["contact_profile"]).resolve())
    equilibrium_path = ROOT / model["equilibrium"]
    builder = HardQpBuilder(config, model, contact,
                            json.loads(equilibrium_path.read_text(encoding="utf-8")))
    capture_path = ROOT / config["dynamic_capture"]; capture = np.load(capture_path)
    cases = []
    for case_id, qpos, velocity in corpus(builder, capture):
        problem = builder.build(qpos, velocity)
        audit = independent_oracle(problem["H"], problem["g"], problem["A"],
                                   problem["l"], problem["u"], config["oracle"])
        cases.append({"id": case_id, "kind": "nominal", "audit": audit, "problem": problem})

    # Mathematical failure corpus is deliberately separate from plant-state coverage.
    base = cases[0]["problem"]
    contradictory_a = np.vstack((base["A"], np.eye(1, NVAR, 0), np.eye(1, NVAR, 0)))
    contradictory_l = np.r_[base["l"], 0.0, 1.0]
    contradictory_u = np.r_[base["u"], 0.0, 1.0]
    contradictory = independent_oracle(base["H"], base["g"], contradictory_a,
                                       contradictory_l, contradictory_u, config["oracle"])
    faults = {
        "contradictory_equalities": {k: v for k, v in contradictory.items() if k != "x"},
        "nonfinite_input": {"expected_solver_status": "invalid_input"},
        "indefinite_hessian": {"expected_solver_status": "invalid_input"},
        "inconsistent_bounds": {"expected_solver_status": "invalid_input"},
        "iteration_limit": {"expected_solver_status": "maximum_iterations"},
    }
    gates_cfg = config["gates"]
    row_count = int(base["A"].shape[0])
    cone_norm_error = max(case["problem"]["cone_normalized_row_norm_error"]
                          for case in cases)
    gates = {
        "dimensions": NVAR == gates_cfg["required_variable_count"] and
                      row_count == gates_cfg["required_row_count"],
        "cone_rows": len(builder.h_cone) == gates_cfg["required_cone_rows_per_side"],
        "oracle_feasible": all(case["audit"]["feasible"] and case["audit"]["qp_success"]
                               for case in cases),
        "oracle_bounds": max(case["audit"].get("qp_bound_violation", INF) for case in cases)
                         <= gates_cfg["maximum_oracle_bound_violation"],
        "oracle_stationarity": max(case["audit"].get("qp_stationarity_residual", INF)
                                    for case in cases)
                               <= gates_cfg["maximum_oracle_stationarity_residual"],
        "cone_row_scaling": cone_norm_error <= gates_cfg["maximum_cone_row_norm_error"],
        "conditioning": max(case["problem"]["normal_matrix_condition_number"] for case in cases)
                        <= gates_cfg["maximum_scaling_condition_number"],
        "infeasible_rejected": not contradictory["feasible"],
    }
    serial_cases = []
    for case in cases:
        audit = {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                 for k, v in case["audit"].items() if k != "x"}
        serial_cases.append({"id": case["id"], "kind": case["kind"], "audit": audit,
                             "normal_matrix_condition_number": case["problem"]["normal_matrix_condition_number"]})
    summary = {
        "schema_version": 1, "phase": 21, "profile": config["profile"],
        "scope": "P21-T04 hard-QP mathematics and independent solver oracle only",
        "variable_order": ["nudot_12", "tau_6", "wrench_left_C_6", "wrench_right_C_6",
                           "slack_left_controller_FLU_6", "slack_right_controller_FLU_6"],
        "variable_count": NVAR, "row_count": row_count, "row_map": builder.row_map,
        "slack_semantics": "per-side controller-FLU [Fx,Fy,Fz,Tx,Ty,Tz], with W_feasible = W_reference + slack; reserved for future soft wrench fidelity and absent from every hard row",
        "joint_protection_audit": "No state-dependent joint position/velocity protection is frozen. The retained 12D acceleration box is the only P21-T04 numerical protection; position-aware protection remains a P21-T05 decision gate.",
        "objective_semantics": "unit minimum-scaled-norm tie-breaker only; no standing/contact/wrench-fidelity task weights",
        "scaling": {"variable_scale": builder.scale.tolist(),
                    "dynamics_row_scale": config["row_scale"]["dynamics"],
                    "cone_rule": config["row_scale"]["cone_rule"]},
        "contact_cone_H_physical": builder.h_cone.tolist(),
        "contact_cone_inequality": "H_C * w_C <= 0; H_C rows are unit Euclidean norm in mixed physical coordinates before solver scaling",
        "hull": builder.hull, "corpus": serial_cases, "faults": faults,
        "gates": gates, "pass": all(gates.values()),
        "limits": "This result cannot close DG21-03 until the C++ 42D ADMM corpus/benchmark passes, and cannot close DG21-04 or authorize task tuning/Core integration."
    }
    write_json(output / "summary.json", summary)
    write_problem_corpus(output / "problem_corpus.txt", cases, config)
    script = Path(__file__).resolve(); inputs = config_inputs + model_inputs + contact_inputs
    write_json(output / "manifest.json", {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "numpy": np.__version__,
        "scipy": scipy.__version__, "mujoco": mujoco.__version__,
        "config_inputs": {str(p.relative_to(ROOT)): sha256(p) for p in inputs},
        "equilibrium": {str(equilibrium_path.relative_to(ROOT)): sha256(equilibrium_path)},
        "dynamic_capture": {str(capture_path.relative_to(ROOT)): sha256(capture_path)},
        "validator": str(script.relative_to(ROOT)), "validator_sha256": sha256(script),
        "outputs": {name: sha256(output / name) for name in ("summary.json", "problem_corpus.txt")},
    })
    print(json.dumps({"cases": len(cases), "rows": row_count, "gates": gates,
                      "pass": summary["pass"]}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
