#!/usr/bin/env python3
"""Run the frozen Phase 29 offline NMPC root-cause audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import warnings
from dataclasses import dataclass
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
from typing import Any

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase29_nmpc_root_cause_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return value


@dataclass
class Problem:
    state: np.ndarray
    reference: np.ndarray
    center: np.ndarray
    rotation: np.ndarray
    tick: int
    logged_wrench: np.ndarray


def csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def heading(quaternion_wxyz: np.ndarray) -> float:
    return float(Rotation.from_quat(quaternion_wxyz[[1, 2, 3, 0]]).as_euler("xyz")[2])


def problem_sequence(rows: list[dict[str, str]], height: float) -> list[Problem]:
    initial_q = np.array([float(rows[0][f"quat{i}"]) for i in range(4)])
    position = np.array([float(rows[0]["base_p0"]), float(rows[0]["base_p1"])])
    yaw = heading(initial_q)
    result: list[Problem] = []
    for tick, row in enumerate(rows):
        if tick > 0:
            speed = float(row["phase27_v_ref"])
            position += 0.01 * speed * np.array([math.cos(yaw), math.sin(yaw)])
            yaw += 0.01 * float(row["phase27_yaw_rate_ref"])
        if int(row["phase27_update"]) != 1:
            continue
        quaternion = np.array([float(row[f"quat{i}"]) for i in range(4)])
        actual = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
        reference_rotation = Rotation.from_euler("z", yaw)
        rotation_error = (actual * reference_rotation.inv()).as_rotvec()
        state = np.array(
            [float(row[f"base_p{i}"]) for i in range(3)]
            + rotation_error.tolist()
            + [float(row[f"base_v{i}"]) for i in range(3)]
            + [float(row[f"base_w{i}"]) for i in range(3)]
            + [
                float(row["phase27_xi_left"]),
                float(row["phase27_xi_right"]),
                float(row["phase27_dxi_left"]),
                float(row["phase27_dxi_right"]),
            ]
        )
        reference = np.zeros(16)
        reference[:2] = position
        reference[2] = height
        speed = float(row["phase27_v_ref"])
        reference[6:8] = speed * np.array([math.cos(yaw), math.sin(yaw)])
        reference[11] = float(row["phase27_yaw_rate_ref"])
        reference[12:14] = float(row["planner_xi_c"])
        reference[14:16] = float(row["planner_dxi_c"])
        result.append(
            Problem(
                state=state,
                reference=reference,
                center=reference.copy(),
                rotation=reference_rotation.as_matrix(),
                tick=tick,
                logged_wrench=np.array(
                    [float(row[f"phase27_requested_wrench{i}"]) for i in range(12)]
                ),
            )
        )
    return result


def advance(reference: np.ndarray, stage: int, step: float) -> np.ndarray:
    value = reference.copy()
    time = step * stage
    value[:3] += time * reference[6:9]
    value[3:6] += time * reference[9:12]
    value[12:14] += time * reference[14:16]
    return value


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rotation_matrix(vector: np.ndarray) -> np.ndarray:
    squared = float(vector @ vector)
    hat = skew(vector)
    if squared < 1e-12:
        sine = 1.0 - squared / 6.0 + squared * squared / 120.0
        cosine = 0.5 - squared / 24.0 + squared * squared / 720.0
    else:
        angle = math.sqrt(squared)
        sine = math.sin(angle) / angle
        cosine = (1.0 - math.cos(angle)) / squared
    return np.eye(3) + sine * hat + cosine * hat @ hat


def flow(state: np.ndarray, control: np.ndarray, rotation_reference: np.ndarray, config: dict) -> np.ndarray:
    rotation = rotation_matrix(state[3:6]) @ rotation_reference
    left_force, left_torque = control[:3], control[3:6]
    right_force, right_torque = control[6:9], control[9:12]
    left_origin = np.array([state[12], *config["left_wheel_origin_yz_b_m"]])
    right_origin = np.array([state[13], *config["right_wheel_origin_yz_b_m"]])
    force_b = left_force + right_force
    moment_b = (
        left_torque
        + np.cross(left_origin, left_force)
        + right_torque
        + np.cross(right_origin, right_force)
    )
    force_n = rotation @ force_b
    moment_n = rotation @ moment_b
    com_n = rotation @ np.asarray(config["body_com_from_base_b_m"], dtype=float)
    inertia_b = np.asarray(config["body_inertia_com_b_kg_m2"], dtype=float)
    inertia_n = rotation @ inertia_b @ rotation.T
    angular = np.linalg.solve(
        inertia_n,
        moment_n
        - np.cross(com_n, force_n)
        - np.cross(state[9:12], inertia_n @ state[9:12]),
    )
    com_acceleration = force_n / float(config["body_mass_kg"])
    com_acceleration[2] -= float(config["gravity_m_s2"])
    linear = (
        com_acceleration
        - np.cross(angular, com_n)
        - np.cross(state[9:12], np.cross(state[9:12], com_n))
    )
    denominator = (
        float(config["wheel_mass_kg"]) * float(config["wheel_radius_m"])
        + float(config["wheel_axle_inertia_kg_m2"]) / float(config["wheel_radius_m"])
    )
    base_forward = force_b[0] / float(config["body_mass_kg"])
    wheel_left = -base_forward - (
        float(config["wheel_radius_m"]) * left_force[0] + left_torque[1]
    ) / denominator
    wheel_right = -base_forward - (
        float(config["wheel_radius_m"]) * right_force[0] + right_torque[1]
    ) / denominator
    output = np.zeros(16)
    output[:3] = state[6:9]
    output[3:6] = state[9:12]
    output[6:9] = linear
    output[9:12] = angular
    output[12:14] = state[14:16]
    output[14:16] = [wheel_left, wheel_right]
    return output


class OfflineSolver:
    def __init__(self, generated: Path, config: dict):
        self.generated = generated
        self.config = config
        json_path = generated / "acados_ocp.json"
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            ocp = AcadosOcp.from_json(str(json_path))
            self.solver = AcadosOcpSolver(ocp, generate=False, build=False)
        self.n = int(config["horizon_steps"])
        self.dt = float(config["sampling_period_s"])
        state_scale = np.asarray(config["state_error_scale"], dtype=float)
        input_scale = np.asarray(config["input_error_scale"], dtype=float)
        self.q = np.asarray(config["state_weight"], dtype=float) / state_scale**2
        self.r = np.asarray(config["input_weight"], dtype=float) / input_scale**2
        self.qe = float(config["terminal_weight_multiplier"]) * self.q
        self.eq = np.asarray(config["equilibrium_input"], dtype=float)
        self.envelope = np.asarray(config["state_envelope_half_width"], dtype=float)
        self.input_lower = np.asarray(config["input_lower"], dtype=float)
        self.input_upper = np.asarray(config["input_upper"], dtype=float)
        self.wheel_lower = np.asarray(config["wheel_workspace_lower"], dtype=float)
        self.wheel_upper = np.asarray(config["wheel_workspace_upper"], dtype=float)

    def reset(self) -> None:
        self.solver.reset(reset_qp_solver_mem=True, reset_numerical_values=True)

    def configure(
        self,
        problem: Problem,
        *,
        cold: bool,
        hold_reference: bool = False,
        zero_terminal: bool = False,
        terminal_zero_indices: list[int] | None = None,
        relax_state: bool = False,
        relax_input: bool = False,
        bound_scale: float = 100.0,
    ) -> list[np.ndarray]:
        references: list[np.ndarray] = []
        parameter = problem.rotation.reshape(-1)
        step_w = np.diag(np.concatenate((self.q, self.r)))
        terminal_weights = np.zeros(16) if zero_terminal else self.qe.copy()
        if terminal_zero_indices:
            terminal_weights[terminal_zero_indices] = 0.0
        terminal_w = np.diag(terminal_weights)
        for stage in range(self.n + 1):
            reference = problem.reference.copy() if hold_reference else advance(problem.reference, stage, self.dt)
            center = advance(problem.center, stage, self.dt)
            references.append(reference)
            lower = center - self.envelope
            upper = center + self.envelope
            lower[12:14] = np.maximum(lower[12:14], self.wheel_lower)
            upper[12:14] = np.minimum(upper[12:14], self.wheel_upper)
            if relax_state:
                lower = center - bound_scale * self.envelope
                upper = center + bound_scale * self.envelope
            self.solver.set(stage, "p", parameter)
            self.solver.cost_set(stage, "yref", np.r_[reference, self.eq] if stage < self.n else reference)
            self.solver.cost_set(stage, "W", step_w if stage < self.n else terminal_w)
            if stage > 0:
                self.solver.constraints_set(stage, "lbx", lower)
                self.solver.constraints_set(stage, "ubx", upper)
            if cold:
                self.solver.set(stage, "x", reference)
                if stage < self.n:
                    self.solver.set(stage, "u", self.eq)
            if stage < self.n:
                if relax_input:
                    span = bound_scale * np.maximum(1.0, self.input_upper - self.input_lower)
                    self.solver.constraints_set(stage, "lbu", -span)
                    self.solver.constraints_set(stage, "ubu", span)
                else:
                    self.solver.constraints_set(stage, "lbu", self.input_lower)
                    self.solver.constraints_set(stage, "ubu", self.input_upper)
        self.solver.constraints_set(0, "lbx", problem.state)
        self.solver.constraints_set(0, "ubx", problem.state)
        return references

    def solve(self, problem: Problem, *, cold: bool, details: bool = False, **shadow: Any) -> dict[str, Any]:
        references = self.configure(problem, cold=cold, **shadow)
        status = int(self.solver.solve())
        residuals = np.asarray(self.solver.get_residuals(recompute=True), dtype=float)
        states = [np.asarray(self.solver.get(stage, "x"), dtype=float) for stage in range(self.n + 1)]
        inputs = [np.asarray(self.solver.get(stage, "u"), dtype=float) for stage in range(self.n)]
        objective = 0.0
        stage_costs: list[float] = []
        for stage in range(self.n):
            state_error = states[stage] - references[stage]
            input_error = inputs[stage] - self.eq
            value = 0.5 * self.dt * (
                float(np.sum(self.q * state_error**2))
                + float(np.sum(self.r * input_error**2))
            )
            stage_costs.append(value)
            objective += value
        terminal_weights = np.zeros(16) if shadow.get("zero_terminal", False) else self.qe.copy()
        if shadow.get("terminal_zero_indices"):
            terminal_weights[shadow["terminal_zero_indices"]] = 0.0
        terminal_error = states[-1] - references[-1]
        terminal_components = 0.5 * terminal_weights * terminal_error**2
        terminal_cost = float(np.sum(terminal_components))
        objective += terminal_cost
        acceleration = flow(problem.state, inputs[0], problem.rotation, self.config)
        result: dict[str, Any] = {
            "status": status,
            "u0": inputs[0],
            "acceleration": acceleration,
            "solver_cost": float(self.solver.get_cost()),
            "recomputed_cost": objective,
            "cost_error": abs(float(self.solver.get_cost()) - objective),
            "residuals": residuals,
            "sqp_iter": int(self.solver.get_stats("sqp_iter")),
        }
        if details:
            input_distance = min(
                float(np.min(np.vstack(inputs) - self.input_lower)),
                float(np.min(self.input_upper - np.vstack(inputs))),
            )
            state_distances = []
            active_multipliers = []
            for stage in range(1, self.n + 1):
                center = advance(problem.center, stage, self.dt)
                lower = center - self.envelope
                upper = center + self.envelope
                lower[12:14] = np.maximum(lower[12:14], self.wheel_lower)
                upper[12:14] = np.minimum(upper[12:14], self.wheel_upper)
                state_distances.extend((states[stage] - lower).tolist())
                state_distances.extend((upper - states[stage]).tolist())
                lam = np.asarray(self.solver.get(stage, "lam"), dtype=float)
                active_multipliers.extend(np.abs(lam).tolist())
            result["stage"] = {
                "x": states,
                "u": inputs,
                "reference": references,
                "cost": stage_costs,
                "terminal_cost": terminal_cost,
                "terminal_cost_by_state": terminal_components,
                "minimum_input_bound_distance": input_distance,
                "minimum_state_bound_distance": float(min(state_distances)),
                "maximum_abs_multiplier": float(max(active_multipliers)),
            }
        return result


def converged_solve(
    solver: OfflineSolver,
    problem: Problem,
    gates: dict,
    *,
    details: bool = False,
    **shadow: Any,
) -> dict[str, Any]:
    result = solver.solve(problem, cold=True, details=details, **shadow)
    passes = 1
    while (
        (
            result["residuals"][0]
            > float(gates["converged_stationarity_max"])
            or max(result["residuals"][1:])
            > float(gates["converged_feasibility_max"])
        )
        and passes < 5
    ):
        result = solver.solve(problem, cold=False, details=details, **shadow)
        passes += 1
    result["oracle_passes"] = passes
    return result


def perturbation_size(index: int, method: dict) -> float:
    p = method["perturbation"]
    if index < 3:
        return float(p["position_m"])
    if index < 6:
        return float(p["rotation_rad"])
    if index < 9:
        return float(p["linear_velocity_m_s"])
    if index < 12:
        return float(p["angular_velocity_rad_s"])
    if index < 14:
        return float(p["wheel_position_m"])
    return float(p["wheel_velocity_m_s"])


def counterfactual_problem(problem: Problem, indices: list[int]) -> Problem:
    state = problem.state.copy()
    state[indices] = problem.reference[indices]
    return Problem(state, problem.reference, problem.center, problem.rotation, problem.tick, problem.logged_wrench)


def action_summary(solution: dict[str, Any], case: dict, authority_error: float) -> dict[str, Any]:
    acceleration_index = int(case["acceleration_index"])
    acceleration = float(solution["acceleration"][acceleration_index])
    return {
        "u0": solution["u0"],
        "acceleration": acceleration,
        "authority_error": authority_error,
        "reinforcing_score": authority_error * acceleration,
        "objective": solution["recomputed_cost"],
        "residuals": solution["residuals"],
        "sqp_iter": solution["sqp_iter"],
    }


def attach_state(solution: dict[str, Any], problem: Problem) -> dict[str, Any]:
    solution["problem_state"] = problem.state
    return solution


def classify(case_result: dict, gates: dict) -> str:
    deadband = float(gates["corrective_score_deadband"])
    if not case_result["semantics_pass"]:
        return "P29-A_state_reference_semantics"
    if not case_result["cost_direction_pass"]:
        return "P29-B_cost_direction"
    if not case_result["model_authority_pass"]:
        return "P29-C_dynamics_control_authority"
    production = case_result["solutions"]["production"]["reinforcing_score"]
    converged = case_result["solutions"]["converged"]["reinforcing_score"]
    if production > deadband and converged < -deadband:
        return "P29-G_solver_rti_lifecycle"
    if production <= deadband:
        return "unresolved_not_reproduced"
    shadows = case_result["shadows"]
    restored_bounds = [name for name in ("relax_state", "relax_input") if shadows[name]["reinforcing_score"] < -deadband]
    if restored_bounds:
        return "P29-F_constraint_driven" if len(restored_bounds) == 1 else "unresolved_multiple_constraints"
    restored_horizon = [name for name in ("held_reference", "zero_terminal") if shadows[name]["reinforcing_score"] < -deadband]
    restored_groups = [name for name, value in case_result["single_removal"].items() if value["reinforcing_score"] < -deadband]
    if "zero_terminal" in restored_horizon:
        terminal_groups = [
            name for name, value in shadows.items()
            if name.startswith("terminal_without_")
            and value["reinforcing_score"] < -deadband
        ]
        return (
            "P29-E_horizon_reference_propagation"
            if len(terminal_groups) == 1
            else "unresolved_horizon_interaction"
        )
    if restored_horizon:
        return "P29-E_horizon_reference_propagation" if len(restored_horizon) == 1 else "unresolved_horizon_interaction"
    if restored_groups:
        return "P29-D_cross_state_coupling"
    return "unresolved_no_causal_flip"


def synthetic_self_test() -> None:
    gates = {"corrective_score_deadband": 1e-7}
    base = {
        "semantics_pass": True,
        "cost_direction_pass": True,
        "model_authority_pass": True,
        "solutions": {"production": {"reinforcing_score": 1.0}, "converged": {"reinforcing_score": -1.0}},
        "shadows": {},
        "single_removal": {},
    }
    assert classify(base, gates) == "P29-G_solver_rti_lifecycle"
    base["solutions"]["converged"]["reinforcing_score"] = 1.0
    base["shadows"] = {
        "relax_state": {"reinforcing_score": -1.0},
        "relax_input": {"reinforcing_score": 1.0},
        "held_reference": {"reinforcing_score": 1.0},
        "zero_terminal": {"reinforcing_score": 1.0},
    }
    assert classify(base, gates) == "P29-F_constraint_driven"
    base["shadows"]["relax_state"]["reinforcing_score"] = 1.0
    base["single_removal"] = {"wheel": {"reinforcing_score": -1.0}}
    assert classify(base, gates) == "P29-D_cross_state_coupling"
    base["single_removal"] = {}
    base["shadows"]["zero_terminal"]["reinforcing_score"] = -1.0
    base["shadows"]["terminal_without_base"] = {"reinforcing_score": -1.0}
    assert classify(base, gates) == "P29-E_horizon_reference_propagation"
    base["semantics_pass"] = False
    assert classify(base, gates) == "P29-A_state_reference_semantics"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    parser.add_argument("--supersedes")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    synthetic_self_test()

    method = json.loads(args.method.read_text(encoding="utf-8"))
    ocp_config_path = ROOT / method["source_ocp_config"]
    ocp_config = json.loads(ocp_config_path.read_text(encoding="utf-8"))
    source_run = ROOT / method["source_phase28_run"]
    rti_dir = ROOT / method["source_generated_dir"]
    sqp_dir = ROOT / method["offline_sqp_generated_dir"]
    for directory in (rti_dir, sqp_dir):
        if not (directory / "acados_ocp.json").is_file():
            raise RuntimeError(f"missing generated oracle: {directory}")
        if not list(directory.glob("libacados_ocp_solver_*.so")):
            raise RuntimeError(f"missing shared library; run make shared_lib in {directory}")

    output.mkdir(parents=True)
    cases_summary = []
    for case in method["cases"]:
        rows = csv_rows(source_run / f"{case['id']}_control.csv")
        sequence = problem_sequence(rows, float(ocp_config["equilibrium_state"][2]))
        snapshot_tick = int(case["snapshot_tick"])
        snapshot_row = rows[snapshot_tick]
        snapshot_sequence = problem_sequence(rows[: snapshot_tick + 1], float(ocp_config["equilibrium_state"][2]))
        action_problem = max((problem for problem in sequence if problem.tick <= snapshot_tick), key=lambda item: item.tick)
        if action_problem.tick != int(case["expected_action_tick"]):
            raise RuntimeError(f"{case['id']} action tick changed: {action_problem.tick}")
        error_index = int(case["error_indices"][-1])
        authority_error = float(
            action_problem.state[error_index] - action_problem.reference[error_index]
        )

        expected_state = np.asarray(case["expected_snapshot_state"], dtype=float)
        expected_reference = np.asarray(case["expected_snapshot_reference"], dtype=float)
        # The snapshot can be a held-action tick; reconstruct its semantics independently of action_problem.
        q = np.array([float(snapshot_row[f"quat{i}"]) for i in range(4)])
        all_refs = problem_sequence(
            [dict(row, phase27_update="1") for row in rows[: snapshot_tick + 1]],
            float(ocp_config["equilibrium_state"][2]),
        )
        snapshot_problem = all_refs[-1]
        semantics_error = max(
            float(np.max(np.abs(snapshot_problem.state - expected_state))),
            float(np.max(np.abs(snapshot_problem.reference - expected_reference))),
        )
        semantics_pass = semantics_error <= float(method["gates"]["snapshot_semantics_max_abs_error"])

        production_solver = OfflineSolver(rti_dir, ocp_config)
        production_errors = []
        production_solution: dict[str, Any] | None = None
        for index, problem in enumerate(sequence):
            if problem.tick > action_problem.tick:
                break
            solved = production_solver.solve(problem, cold=index == 0, details=problem.tick == action_problem.tick)
            production_errors.append(float(np.max(np.abs(solved["u0"] - problem.logged_wrench))))
            if problem.tick == action_problem.tick:
                production_solution = attach_state(solved, problem)
        assert production_solution is not None

        cold_solver = OfflineSolver(rti_dir, ocp_config)
        cold_solution = attach_state(cold_solver.solve(action_problem, cold=True, details=True), action_problem)
        repeated_solution = cold_solution
        for iteration in range(int(method["repeated_rti_iterations"])):
            repeated_solution = attach_state(
                cold_solver.solve(action_problem, cold=False, details=iteration + 1 == int(method["repeated_rti_iterations"])),
                action_problem,
            )

        sqp_solver = OfflineSolver(sqp_dir, ocp_config)
        converged_solution = attach_state(
            converged_solve(
                sqp_solver, action_problem, method["gates"], details=True
            ),
            action_problem,
        )

        single: dict[str, Any] = {}
        pairs: dict[str, Any] = {}
        group_items = list(method["state_groups"].items())
        for name, indices in group_items:
            sqp_solver.reset()
            problem = counterfactual_problem(action_problem, indices)
            single[name] = action_summary(
                attach_state(
                    converged_solve(sqp_solver, problem, method["gates"]),
                    problem,
                ),
                case,
                authority_error,
            )
        for (left_name, left_indices), (right_name, right_indices) in combinations(group_items, 2):
            sqp_solver.reset()
            problem = counterfactual_problem(action_problem, sorted(set(left_indices + right_indices)))
            pairs[f"{left_name}+{right_name}"] = action_summary(
                attach_state(
                    converged_solve(sqp_solver, problem, method["gates"]),
                    problem,
                ),
                case,
                authority_error,
            )

        sensitivities: dict[str, Any] = {}
        for index in case["error_indices"]:
            delta = perturbation_size(int(index), method)
            minus = Problem(action_problem.state.copy(), action_problem.reference, action_problem.center, action_problem.rotation, action_problem.tick, action_problem.logged_wrench)
            plus = Problem(action_problem.state.copy(), action_problem.reference, action_problem.center, action_problem.rotation, action_problem.tick, action_problem.logged_wrench)
            minus.state[int(index)] -= delta
            plus.state[int(index)] += delta
            sqp_solver.reset(); negative = converged_solve(sqp_solver, minus, method["gates"])
            sqp_solver.reset(); positive = converged_solve(sqp_solver, plus, method["gates"])
            sensitivities[str(index)] = {
                "delta": delta,
                "objective_derivative": (positive["recomputed_cost"] - negative["recomputed_cost"]) / (2.0 * delta),
                "acceleration_derivative": (
                    positive["acceleration"][int(case["acceleration_index"])]
                    - negative["acceleration"][int(case["acceleration_index"])]
                ) / (2.0 * delta),
                "u0_derivative": (positive["u0"] - negative["u0"]) / (2.0 * delta),
                "minus_residuals": negative["residuals"],
                "plus_residuals": positive["residuals"],
            }

        shadows: dict[str, Any] = {}
        for name, options in {
            "relax_state": {"relax_state": True},
            "relax_input": {"relax_input": True},
            "held_reference": {"hold_reference": True},
            "zero_terminal": {"zero_terminal": True},
        }.items():
            sqp_solver.reset()
            solution = attach_state(
                converged_solve(
                    sqp_solver,
                    action_problem,
                    method["gates"],
                    bound_scale=float(method["shadow_bound_scale"]),
                    **options,
                ),
                action_problem,
            )
            shadows[name] = action_summary(solution, case, authority_error)
        for name, indices in group_items:
            sqp_solver.reset()
            solution = attach_state(
                converged_solve(
                    sqp_solver,
                    action_problem,
                    method["gates"],
                    terminal_zero_indices=indices,
                ),
                action_problem,
            )
            shadows[f"terminal_without_{name}"] = action_summary(
                solution, case, authority_error
            )

        axis = int(case["acceleration_index"])
        input_jacobian = np.zeros(12)
        for index in range(12):
            delta = 1e-5
            minus_u = converged_solution["u0"].copy(); minus_u[index] -= delta
            plus_u = converged_solution["u0"].copy(); plus_u[index] += delta
            input_jacobian[index] = (
                flow(action_problem.state, plus_u, action_problem.rotation, ocp_config)[axis]
                - flow(action_problem.state, minus_u, action_problem.rotation, ocp_config)[axis]
            ) / (2.0 * delta)
        eq = np.asarray(ocp_config["equilibrium_input"], dtype=float)
        base_acceleration = flow(action_problem.state, eq, action_problem.rotation, ocp_config)[axis]
        wrench_groups = {
            "common_Fx": [0, 6],
            "vertical_Fz": [2, 8],
            "pitch_Ty": [4, 10],
            "other_wrench": [1, 3, 5, 7, 9, 11],
        }
        wrench_decomposition = {"equilibrium": base_acceleration}
        for name, indices in wrench_groups.items():
            isolated = eq.copy()
            isolated[indices] = converged_solution["u0"][indices]
            wrench_decomposition[name] = (
                flow(action_problem.state, isolated, action_problem.rotation, ocp_config)[axis]
                - base_acceleration
            )
        wrench_decomposition["reconstruction_error"] = (
            float(converged_solution["acceleration"][axis])
            - sum(float(value) for key, value in wrench_decomposition.items() if key != "reconstruction_error")
        )

        cost_direction_pass = all(
            float(value["objective_derivative"])
            * float(action_problem.state[int(index)] - action_problem.reference[int(index)])
            > 0.0
            for index, value in sensitivities.items()
        )
        model_authority_pass = bool(
            np.all(np.isfinite(input_jacobian))
            and abs(wrench_decomposition["reconstruction_error"]) <= 1e-8
        )

        case_result = {
            "id": case["id"],
            "snapshot_tick": snapshot_tick,
            "action_tick": action_problem.tick,
            "snapshot_is_update": bool(int(snapshot_row["phase27_update"])),
            "semantics_max_abs_error": semantics_error,
            "semantics_pass": semantics_pass,
            "cost_direction_pass": cost_direction_pass,
            "model_authority_pass": model_authority_pass,
            "production_prefix_max_request_error": max(production_errors),
            "production_prefix_pass": max(production_errors) <= float(method["gates"]["production_request_max_abs_error"]),
            "problem": {
                "state": action_problem.state,
                "reference": action_problem.reference,
                "rotation": action_problem.rotation,
                "logged_wrench": action_problem.logged_wrench,
            },
            "solutions": {
                "production": action_summary(production_solution, case, authority_error),
                "cold": action_summary(cold_solution, case, authority_error),
                "repeated_rti": action_summary(repeated_solution, case, authority_error),
                "converged": action_summary(converged_solution, case, authority_error),
            },
            "solution_details": {
                "production": production_solution["stage"],
                "cold": cold_solution["stage"],
                "repeated_rti": repeated_solution["stage"],
                "converged": converged_solution["stage"],
            },
            "single_removal": single,
            "pair_removal": pairs,
            "sensitivities": sensitivities,
            "shadows": shadows,
            "model_control_authority_row": input_jacobian,
            "wrench_acceleration_decomposition": wrench_decomposition,
        }
        case_result["classification"] = classify(case_result, method["gates"])
        case_result["pass"] = (
            case_result["semantics_pass"]
            and case_result["production_prefix_pass"]
            and case_result["solutions"]["converged"]["residuals"][0]
            <= float(method["gates"]["converged_stationarity_max"])
            and max(case_result["solutions"]["converged"]["residuals"][1:])
            <= float(method["gates"]["converged_feasibility_max"])
            and not case_result["classification"].startswith("unresolved")
        )
        case_path = output / f"{case['id']}_root_cause.json"
        case_path.write_text(json.dumps(clean(case_result), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        cases_summary.append({
            "id": case["id"],
            "snapshot_tick": snapshot_tick,
            "action_tick": action_problem.tick,
            "classification": case_result["classification"],
            "pass": case_result["pass"],
            "production_prefix_max_request_error": case_result["production_prefix_max_request_error"],
            "scores": {name: value["reinforcing_score"] for name, value in case_result["solutions"].items()},
            "single_removal_scores": {name: value["reinforcing_score"] for name, value in single.items()},
            "shadow_scores": {name: value["reinforcing_score"] for name, value in shadows.items()},
        })

    holdout_summary = []
    if all(case["pass"] for case in cases_summary):
        for holdout in method.get("holdouts", []):
            rows = csv_rows(source_run / f"{holdout['id']}_control.csv")
            sequence = problem_sequence(
                rows, float(ocp_config["equilibrium_state"][2])
            )
            action_problem = max(
                (
                    problem
                    for problem in sequence
                    if problem.tick <= int(holdout["snapshot_tick"])
                ),
                key=lambda item: item.tick,
            )
            if action_problem.tick != int(holdout["expected_action_tick"]):
                raise RuntimeError(
                    f"{holdout['id']} action tick changed: {action_problem.tick}"
                )
            error_index = int(holdout["error_index"])
            authority_error = float(
                action_problem.state[error_index]
                - action_problem.reference[error_index]
            )
            production_solver = OfflineSolver(rti_dir, ocp_config)
            prefix_errors = []
            production_solution = None
            for index, problem in enumerate(sequence):
                if problem.tick > action_problem.tick:
                    break
                solved = production_solver.solve(
                    problem,
                    cold=index == 0,
                    details=problem.tick == action_problem.tick,
                )
                prefix_errors.append(
                    float(np.max(np.abs(solved["u0"] - problem.logged_wrench)))
                )
                if problem.tick == action_problem.tick:
                    production_solution = attach_state(solved, problem)
            assert production_solution is not None
            sqp_solver = OfflineSolver(sqp_dir, ocp_config)
            converged = attach_state(
                converged_solve(
                    sqp_solver,
                    action_problem,
                    method["gates"],
                    details=True,
                ),
                action_problem,
            )
            removals = {}
            for group in holdout["diagnostic_groups"]:
                sqp_solver.reset()
                problem = counterfactual_problem(
                    action_problem, method["state_groups"][group]
                )
                removals[group] = action_summary(
                    attach_state(
                        converged_solve(
                            sqp_solver, problem, method["gates"]
                        ),
                        problem,
                    ),
                    holdout,
                    authority_error,
                )
            production_summary = action_summary(
                production_solution, holdout, authority_error
            )
            converged_summary = action_summary(
                converged, holdout, authority_error
            )
            deadband = float(method["gates"]["corrective_score_deadband"])
            same_mechanism = (
                production_summary["reinforcing_score"] > deadband
                and converged_summary["reinforcing_score"] > deadband
                and removals[holdout["required_group"]]["reinforcing_score"]
                < -deadband
            )
            valid = (
                max(prefix_errors)
                <= float(method["gates"]["production_request_max_abs_error"])
                and converged_summary["residuals"][0]
                <= float(method["gates"]["converged_stationarity_max"])
                and max(converged_summary["residuals"][1:])
                <= float(method["gates"]["converged_feasibility_max"])
            )
            result = {
                "id": holdout["id"],
                "source_mechanism": holdout["source_mechanism"],
                "snapshot_tick": int(holdout["snapshot_tick"]),
                "action_tick": action_problem.tick,
                "production_prefix_max_request_error": max(prefix_errors),
                "production": production_summary,
                "converged": converged_summary,
                "removals": removals,
                "mechanism_consistency": (
                    "same" if same_mechanism else "not_same"
                ),
                "valid": valid,
            }
            (output / f"{holdout['id']}_holdout.json").write_text(
                json.dumps(clean(result), indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            holdout_summary.append(
                {
                    "id": holdout["id"],
                    "mechanism_consistency": result["mechanism_consistency"],
                    "valid": valid,
                    "production_score": production_summary["reinforcing_score"],
                    "converged_score": converged_summary["reinforcing_score"],
                    "removal_scores": {
                        name: value["reinforcing_score"]
                        for name, value in removals.items()
                    },
                }
            )

    summary = {
        "schema_version": 1,
        "phase": 29,
        "profile": method["profile"],
        "cases": cases_summary,
        "holdouts": holdout_summary,
        "pass": (
            all(case["pass"] for case in cases_summary)
            and all(holdout["valid"] for holdout in holdout_summary)
        ),
    }
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [
        args.method,
        ocp_config_path,
        Path(__file__),
        rti_dir / "acados_ocp.json",
        rti_dir / "phase27_generation_manifest.json",
        sqp_dir / "acados_ocp.json",
        sqp_dir / "phase29_generation_manifest.json",
    ] + [
        source_run / f"{case['id']}_control.csv"
        for case in method["cases"] + method.get("holdouts", [])
    ]
    outputs = sorted(path for path in output.iterdir() if path.is_file())
    manifest = {
        "schema_version": 1,
        "phase": 29,
        "profile": method["profile"],
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "dependencies": {
            "numpy": np.__version__,
            "scipy": importlib.metadata.version("scipy"),
            "casadi": importlib.metadata.version("casadi"),
            "acados_template": str(Path(importlib.import_module("acados_template").__file__).resolve()),
        },
        "source_run": method["source_phase28_run"],
        "replay_of": args.replay_of,
        "supersedes": args.supersedes,
        "hardware_data": False,
        "inputs": {relative(path): sha256(path) for path in inputs},
        "outputs": {relative(path): sha256(path) for path in outputs},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(clean(summary), indent=2, sort_keys=True))
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
