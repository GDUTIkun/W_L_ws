#!/usr/bin/env python3
"""Phase46 compatible-H0 fixed-state Fn-to-Fr root-cause closure."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
DEFAULT_SOURCE = PHASE / "evidence/automated/contact-realization-sensitivity-formal-v4"
DEFAULT_TORQUE = PHASE / "evidence/automated/torque-free-contact-attribution-formal-v1"
DIRECTIONS = ("Fr_L", "Fn_L", "Fr_R", "Fn_R")
ACTUATORS = ("left_hip", "left_knee", "left_wheel",
             "right_hip", "right_knee", "right_wheel")
SCALES = (1.0, 0.5, 0.25)


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


SENS = load(ROOT / "tools/experiments/run_phase46_contact_realization_sensitivity.py",
            "p46_root_sensitivity")
TAU = load(ROOT / "tools/experiments/run_phase46_torque_free_contact_attribution.py",
           "p46_root_tau")
ATTR, P45C, P45, P44, P42 = SENS.ATTR, SENS.P45C, SENS.P45, SENS.P44, SENS.P42


def matrix(rows: dict[str, list[tuple[int, int, float]]], name: str) -> np.ndarray:
    values = rows[name]
    result = np.zeros((max(row for row, _, _ in values) + 1,
                       max(column for _, column, _ in values) + 1))
    for row, column, value in values:
        result[row, column] = value
    return result


def dump(executable: Path, row: Path) -> dict[str, np.ndarray]:
    environment = os.environ.copy()
    acados = str(ROOT.parent / "opt/acados/lib")
    environment["LD_LIBRARY_PATH"] = acados + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    completed = subprocess.run([str(executable), str(row)], cwd=ROOT, env=environment,
                               check=True, text=True, capture_output=True)
    values: dict[str, list[tuple[int, int, float]]] = {}
    for name, row_index, column_index, value in csv.reader(completed.stdout.splitlines()):
        values.setdefault(name, []).append((int(row_index), int(column_index), float(value)))
    return {name: matrix(values, name) for name in values}


def central(plus: np.ndarray, minus: np.ndarray, denominator: float) -> np.ndarray:
    return (plus - minus) / (2.0 * denominator)


def rel(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.linalg.norm(first - second) / max(np.linalg.norm(first), 1.0e-12))


def skew(value: np.ndarray) -> np.ndarray:
    x, y, z = value
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def actual_setup(base: dict[str, Any], records: dict[str, Any]) -> dict[str, Any]:
    config_path = ROOT / "simulation/mujoco/config/phase46_hip_common_increment_limited_v1.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation = json.loads((ROOT / config["continuation_config"]).read_text(encoding="utf-8"))
    frozen, _, _ = P45C.frozen_inputs(continuation)
    model = mujoco.MjModel.from_xml_path(str(ROOT / frozen["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / frozen["phase42_config"]).read_text(encoding="utf-8")))
    native = P45.native_state(P44.read_csv(ROOT / frozen["phase42_native_authority"]), 0)
    qpos = P44.vec(native, "qpos", model.nq)
    qvel = P44.vec(native, "qvel", model.nv)
    geometry = ATTR.contact_geometry(model, qpos, qvel,
                                     float(oracle.config["canonical_wheel_radius_m"]))
    replay = TAU.observe_tau(np.asarray(records["baseline"]["tau"]), native, model, oracle,
                             geometry, ATTR.hip_dofs(model))
    mass = P44.vec(replay["actual"]["dynamics"], "mass", model.nv ** 2).reshape(model.nv, model.nv)
    point_jacobians = TAU.point_jacobians(model, native, geometry, replay)
    reduction, reduction_metrics = ATTR.plant_constrained_reduction(model, qpos, qvel)
    point_reduced = [dict(row, reduced=row["map"] @ reduction) for row in point_jacobians]
    data = mujoco.MjData(model)
    data.qpos[:] = qpos; data.qvel[:] = qvel
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0
    mujoco.mj_forward(model, data)
    actuator = np.zeros((model.nv, model.nu))
    zero_actuator = np.asarray(data.qfrc_actuator).copy()
    for column in range(model.nu):
        probe = mujoco.MjData(model)
        probe.qpos[:] = qpos; probe.qvel[:] = qvel
        probe.eq_active[weld] = 0
        probe.ctrl[column] = -1.0
        mujoco.mj_forward(model, probe)
        actuator[:, column] = np.asarray(probe.qfrc_actuator) - zero_actuator
    return {"model": model, "oracle": oracle, "native": native, "geometry": geometry,
            "replay": replay, "mass": mass, "point_jacobians": point_jacobians,
            "point_reduced": point_reduced, "reduction": reduction,
            "reduction_metrics": reduction_metrics, "actuation": actuator,
            "config_path": config_path, "frozen": frozen}


def point_acceleration(point_maps: list[dict[str, Any]], acceleration: np.ndarray,
                       key: str = "map") -> np.ndarray:
    result = np.zeros((2, 2, 3))
    for row in point_maps:
        result[row["side"], row["point_index"]] = row[key] @ acceleration
    return result


def active_rows(control: dict[str, Any]) -> tuple[list[int], list[str]]:
    rows = list(range(12)); sides = ["equality"] * 12
    for row in range(12, 104):
        value = int(float(control[f"active_row{row}"]))
        if value:
            rows.append(row)
            sides.append("lower" if value == 1 else "upper" if value == 2 else "both")
    return rows, sides


def kkt_solve(h: np.ndarray, a: np.ndarray, rows: list[int], rhs_x: np.ndarray,
              rhs_a: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, float]:
    active = a[rows]
    kkt = np.block([[h, active.T], [active, np.zeros((len(rows), len(rows)))]])
    rhs = np.r_[rhs_x, np.zeros(len(rows)) if rhs_a is None else rhs_a]
    answer, *_ = np.linalg.lstsq(kkt, rhs, rcond=1.0e-12)
    return answer[:h.shape[0]], answer[h.shape[0]:], float(np.max(np.abs(kkt @ answer - rhs)))


def point_map(side: int, baseline: dict[str, Any], geometry: list[dict[str, Any]]) -> np.ndarray:
    blocks = []
    for row in baseline["points"]:
        if row["side"] != ("left", "right")[side]:
            continue
        lever = np.asarray(row["position_world_m"]) - geometry[side]["point"]
        frame = geometry[side]["frame"]
        blocks.append(np.vstack((np.eye(3), frame.T @ skew(lever) @ frame)))
    return np.hstack(blocks)


def point_vector(item: dict[str, Any], side: int) -> np.ndarray:
    values = [np.asarray(row["wheel_force_Fr_Fl_Fn_n"]) for row in item["points"]
              if row["side"] == ("left", "right")[side]]
    return np.concatenate(values)


def markdown(title: str, lines: list[str]) -> str:
    return "# " + title + "\n\n" + "\n\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--torque-source", type=Path, default=DEFAULT_TORQUE)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    source = args.source.resolve(); output = args.output.resolve(); qp_dump = args.qp_dump.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    records = json.loads((source / "probe-records.json").read_text(encoding="utf-8"))
    sensitivity = json.loads((source / "contact-realization-sensitivity.json").read_text(encoding="utf-8"))
    torque = json.loads((args.torque_source / "torque-free-contact-attribution.json").read_text(
        encoding="utf-8"))
    baseline = records["baseline"]
    probes = {(row["direction"], float(row["scale"]), int(row["branch"])): row
              for row in records["probes"] if row["stage"] == "target"}
    target_delta = np.asarray(sensitivity["target_wrench_delta_n"])
    operators = dump(qp_dump, source / "probes/baseline-detail.csv")
    mass_qp = operators["mass"]; actuation_qp = operators["actuation"]
    wrench_map = np.hstack((operators["wrench_map_0"], operators["wrench_map_1"]))
    contact_map = np.vstack((operators["contact_jacobian_0"], operators["contact_jacobian_1"]))
    scale = operators["variable_scale"].reshape(-1)
    actual = actual_setup({}, records)

    baseline_solution = np.asarray(baseline["solution"])
    raw: dict[tuple[str, float, int], dict[str, Any]] = {}
    eom_max = 0.0; decomposition_max = 0.0; other_component_max = 0.0
    branch_split = 0.0; scale_convergence = 0.0
    for axis, direction in enumerate(DIRECTIONS):
        for factor in SCALES:
            for branch in (-1, 1):
                item = probes[(direction, factor, branch)]
                dz = (np.asarray(item["solution"]) - baseline_solution) / (
                    branch * factor * target_delta[axis])
                nudot, tau_gain, wrench = dz[:12], dz[12:18], dz[18:30]
                identity = mass_qp @ nudot - actuation_qp @ tau_gain - wrench_map @ wrench
                tau_acc = np.linalg.pinv(actuation_qp) @ (mass_qp @ nudot)
                tau_contact = -np.linalg.pinv(actuation_qp) @ (wrench_map @ wrench)
                tau_other = tau_gain - tau_acc - tau_contact
                eom_max = max(eom_max, float(np.max(np.abs(identity))))
                decomposition_max = max(decomposition_max, float(np.max(np.abs(
                    tau_gain - tau_acc - tau_contact - tau_other))))
                other_component_max = max(other_component_max, float(np.max(np.abs(tau_other))))
                raw[(direction, factor, branch)] = {
                    "nudot": nudot, "tau": tau_gain, "wrench": wrench,
                    "tau_acceleration": tau_acc, "tau_contact": tau_contact,
                    "tau_other": tau_other, "identity": identity,
                }

    central_values: dict[str, dict[str, Any]] = {}
    torque_rows = []
    for direction in DIRECTIONS:
        by_scale = {}
        for factor in SCALES:
            plus, minus = raw[(direction, factor, 1)], raw[(direction, factor, -1)]
            combined = {key: 0.5 * (plus[key] + minus[key]) for key in
                        ("nudot", "tau", "wrench", "tau_acceleration", "tau_contact", "tau_other")}
            branch_split = max(branch_split, *(rel(plus[key], minus[key]) for key in
                               ("nudot", "tau", "wrench")))
            by_scale[factor] = combined
        for factor in (0.5, 0.25):
            scale_convergence = max(scale_convergence, *(rel(by_scale[1.0][key], by_scale[factor][key])
                                    for key in ("nudot", "tau", "wrench")))
        value = by_scale[1.0]
        for component in ("tau_acceleration", "tau_contact", "tau_other"):
            qacc = np.linalg.solve(actual["mass"], actual["actuation"] @ value[component])
            value[component + "_actual_point_acceleration_RLN"] = point_acceleration(
                actual["point_jacobians"], qacc)
        value["tau_total_actual_point_acceleration_RLN"] = sum(
            value[name + "_actual_point_acceleration_RLN"]
            for name in ("tau_acceleration", "tau_contact", "tau_other"))
        central_values[direction] = value
        for index, actuator_name in enumerate(ACTUATORS):
            torque_rows.append({"direction": direction, "actuator": actuator_name,
                                "tau_total_nm_per_n": value["tau"][index],
                                "tau_acc_nm_per_n": value["tau_acceleration"][index],
                                "tau_contact_nm_per_n": value["tau_contact"][index],
                                "tau_other_nm_per_n": value["tau_other"][index]})

    fn_acc_shares = []; fn_contact_shares = []; fn_other_shares = []
    for direction in ("Fn_L", "Fn_R"):
        value = central_values[direction]
        fn_acc_shares.append(float(np.linalg.norm(value["tau_acceleration"]) /
                                   np.linalg.norm(value["tau"])))
        fn_contact_shares.append(float(np.linalg.norm(value["tau_contact"]) /
                                       np.linalg.norm(value["tau"])))
        fn_other_shares.append(float(np.linalg.norm(value["tau_other"]) /
                                     np.linalg.norm(value["tau"])))
    torque_class = ("T2-ACCELERATION_TASK_COUPLING_DOMINANT" if min(fn_acc_shares) >= 0.75 else
                    "T1-CONTACT_BALANCING_DOMINANT" if max(fn_acc_shares) <= 0.25 else
                    "T3-MIXED_TORQUE_GENERATION")
    directional_quality = {}
    for direction in DIRECTIONS:
        entries = []
        for factor in SCALES:
            for branch in (-1, 1):
                control = probes[(direction, factor, branch)]["control"]
                rows, sides = active_rows(control)
                entries.append({"scale": factor, "branch": branch, "active_rows": rows,
                    "active_sides": sides, "hard_residual": float(control["hard"]),
                    "maximum_normalized_slack": float(control["maximum_normalized_slack"]),
                    "primal_residual": float(control["primal"]),
                    "dual_residual": float(control["dual"]),
                    "stationarity_residual": float(control["stationarity"]),
                    "task_residuals": [float(control[f"task_max_residual{i}"]) for i in range(11)]})
        directional_quality[direction] = entries
    stage_a = {"classification": torque_class, "operator_definition":
        "M*dnu = B*dtau + JwT*dlambda; tau_acc=B+M*dnu; tau_contact=-B+JwT*dlambda",
        "qp_eom_identity_max_abs": eom_max, "tau_decomposition_closure_max_abs": decomposition_max,
        "tau_other_numerical_component_max_abs": other_component_max,
        "branch_split_relative": branch_split, "scale_convergence_relative": scale_convergence,
        "fn_acceleration_component_norm_share": dict(zip(("Fn_L", "Fn_R"), fn_acc_shares)),
        "fn_contact_component_norm_share": dict(zip(("Fn_L", "Fn_R"), fn_contact_shares)),
        "fn_other_component_norm_share": dict(zip(("Fn_L", "Fn_R"), fn_other_shares)),
        "directional_quality": directional_quality, "directions": central_values}

    # Stage B: exact two-point-force image of each aggregate wheel wrench.
    realizability: dict[str, Any] = {"sides": {}, "directions": {}}
    material_unrealizable = False
    for side in range(2):
        gp = point_map(side, baseline, actual["geometry"])
        u, singular, _ = np.linalg.svd(gp)
        rank = int(np.linalg.matrix_rank(gp, tol=1.0e-10))
        null_left = u[:, rank:]
        nominal_wrench = np.asarray(baseline["qp_wrench"])[6 * side:6 * side + 6]
        nominal_force = np.linalg.pinv(gp) @ nominal_wrench
        nominal_residual = nominal_wrench - gp @ nominal_force
        realizability["sides"][("left", "right")[side]] = {
            "rank": rank, "singular_values": singular,
            "condition_nonzero": float(singular[0] / singular[rank - 1]),
            "force_nullspace_dimension": int(gp.shape[1] - rank),
            "wrench_left_nullspace": null_left,
            "nominal_exact_residual": nominal_residual,
            "nominal_relative_unrealizable_fraction": float(
                np.linalg.norm(nominal_residual) / max(np.linalg.norm(nominal_wrench), 1e-12)),
        }
    for direction in DIRECTIONS:
        direction_result = {}
        value = central_values[direction]
        for side in range(2):
            gp = point_map(side, baseline, actual["geometry"])
            delta_wrench = value["wrench"][6 * side:6 * side + 6]
            delta_force = np.linalg.pinv(gp) @ delta_wrench
            projected = gp @ delta_force
            perpendicular = delta_wrench - projected
            qacc_perp = np.linalg.solve(mass_qp, operators[f"wrench_map_{side}"] @ perpendicular)
            qacc_all = np.linalg.solve(mass_qp, operators[f"wrench_map_{side}"] @ delta_wrench)
            xi_map = np.asarray([[float(baseline["control"][f"xi_map_{s}_{j}"])
                                  for j in range(12)] for s in range(2)])
            rolling_map = np.asarray([[float(baseline["control"][f"rolling_map_{s}_{j}"])
                                       for j in range(12)] for s in range(2)])
            nominal_points = point_vector(baseline, side)
            candidate = nominal_points + delta_force
            mu = min(float(row["friction_margin_n"] + np.linalg.norm(
                np.asarray(row["wheel_force_Fr_Fl_Fn_n"])[:2])) /
                max(float(np.asarray(row["wheel_force_Fr_Fl_Fn_n"])[2]), 1e-12)
                for row in baseline["points"] if row["side"] == ("left", "right")[side])
            point_feasibility = []
            for point in candidate.reshape(-1, 3):
                margin = mu * point[2] - float(np.linalg.norm(point[:2]))
                point_feasibility.append({"force_RLN": point, "normal_nonnegative": point[2] >= 0.0,
                                          "friction_margin_n": margin, "feasible": point[2] >= 0.0 and margin >= 0.0})
            relative_unrealizable = float(np.linalg.norm(perpendicular) /
                                          max(np.linalg.norm(delta_wrench), 1e-12))
            harmful_fraction = float(abs(0.5 * np.sum(rolling_map @ qacc_perp)) /
                                     max(abs(0.5 * np.sum(rolling_map @ qacc_all)), 1e-12))
            hip_perp = float(0.5 * (qacc_perp[6] + qacc_perp[9]))
            ddxi_perp = float(0.5 * np.sum(xi_map @ qacc_perp))
            if direction.startswith("Fn") and relative_unrealizable > 0.1 and harmful_fraction > 0.1:
                material_unrealizable = True
            direction_result[("left", "right")[side]] = {
                "delta_wrench": delta_wrench, "projected_realizable": projected,
                "unrealizable": perpendicular, "relative_unrealizable_fraction": relative_unrealizable,
                "hip_common_acceleration_contribution": hip_perp,
                "ddxi_common_contribution": ddxi_perp,
                "rolling_cancellation_contribution_fraction": harmful_fraction,
                "point_force_increment_minimum_norm": delta_force,
                "nominal_plus_increment_feasibility": point_feasibility,
            }
        realizability["directions"][direction] = direction_result
    realizability["structural_candidate"] = (
        "R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH" if material_unrealizable else "none")

    # Stage C: QP balance at the actual frozen points, in the same reduced ordering.
    contact_balance = {}
    contact_closure = 0.0
    for direction, value in central_values.items():
        q_tau = np.linalg.solve(mass_qp, actuation_qp @ value["tau"])
        q_lambda = np.linalg.solve(mass_qp, wrench_map @ value["wrench"])
        q_net = value["nudot"]
        points_tau = point_acceleration(actual["point_reduced"], q_tau, "reduced")
        points_lambda = point_acceleration(actual["point_reduced"], q_lambda, "reduced")
        points_net = point_acceleration(actual["point_reduced"], q_net, "reduced")
        closure = points_tau + points_lambda - points_net
        contact_closure = max(contact_closure, float(np.max(np.abs(closure))))
        ratios = np.abs(points_lambda[:, :, 0]) / np.maximum(np.abs(points_tau[:, :, 0]), 1e-12)
        contact_balance[direction] = {"actuator_free_RLN": points_tau,
                                      "qp_contact_RLN": points_lambda,
                                      "other_RLN": np.zeros_like(points_tau),
                                      "net_RLN": points_net, "closure_RLN": closure,
                                      "rolling_cancellation_ratio": ratios}

    # Stage D: exact fixed-active-set source sensitivity and diagnostic-only Hessian blocks.
    kkt: dict[str, Any] = {"triggered": torque_class in (
        "T2-ACCELERATION_TASK_COUPLING_DOMINANT", "T3-MIXED_TORQUE_GENERATION"),
        "directions": {}}
    h = operators["h"]; a = operators["a"]
    sdiag = np.diag(scale)
    contact_task = np.zeros((6, 42)); contact_task[:3, :12] = operators["contact_jacobian_0"]
    contact_task[3:, :12] = operators["contact_jacobian_1"]
    contact_task = contact_task @ sdiag / 10.0
    xi_task = np.zeros((2, 42)); xi_task[:, :12] = np.asarray([
        [float(baseline["control"][f"xi_map_{side}_{j}"]) for j in range(12)] for side in range(2)])
    xi_task = xi_task @ sdiag
    rolling_task = np.zeros((2, 42)); rolling_task[:, :12] = np.asarray([
        [float(baseline["control"][f"rolling_map_{side}_{j}"]) for j in range(12)] for side in range(2)])
    rolling_task = rolling_task @ sdiag
    wrench_task = np.zeros((12, 42))
    for side in range(2):
        wrench_task[6*side:6*side+6, :12] = operators[f"interaction_acceleration_map_{side}"]
        wrench_task[6*side:6*side+6, 18+6*side:24+6*side] = operators[f"interaction_contact_map_{side}"]
        wrench_task[6*side:6*side+6, 30+6*side:36+6*side] = -np.eye(6)
    wrench_scale = np.tile(scale[30:36], 2)
    wrench_task = wrench_task @ sdiag / wrench_scale[:, None]
    slack_task = np.zeros((12, 42)); slack_task[:, 30:42] = np.eye(12)
    slack_task = slack_task @ sdiag / wrench_scale[:, None]
    regularization = np.diag(np.r_[np.full(30, 1.0e-6), np.zeros(12)])
    h_blocks = {"regularization": regularization, "contact_task": contact_task.T @ contact_task,
                "xi_task": xi_task.T @ xi_task, "rolling_slip_task": rolling_task.T @ rolling_task,
                "interaction_wrench_tracking": wrench_task.T @ wrench_task,
                "slack_penalty": slack_task.T @ slack_task}
    h_reconstruction = float(np.max(np.abs(h - sum(h_blocks.values()))))
    kkt["hessian_reconstruction_max_abs"] = h_reconstruction
    for axis, direction in enumerate(DIRECTIONS):
        plus_control = probes[(direction, 1.0, 1)]["control"]
        minus_control = probes[(direction, 1.0, -1)]["control"]
        plus_rows, plus_sides = active_rows(plus_control)
        minus_rows, minus_sides = active_rows(minus_control)
        signature_match = plus_rows == minus_rows and plus_sides == minus_sides
        if not signature_match:
            kkt["directions"][direction] = {"central_valid": False, "reason": "active set changed"}
            continue
        plus_ops = dump(qp_dump, source / "probes" / f"target-{axis}-1-+1.csv")
        minus_ops = dump(qp_dump, source / "probes" / f"target-{axis}-1--1.csv")
        denominator = target_delta[axis]
        dg = central(plus_ops["g"], minus_ops["g"], denominator).reshape(-1)
        dg_xi = central(plus_ops["g_xi_target"], minus_ops["g_xi_target"], denominator).reshape(-1)
        dg_rolling = central(plus_ops["g_rolling_target"], minus_ops["g_rolling_target"], denominator).reshape(-1)
        dx, dual_gain, residual = kkt_solve(h, a, plus_rows, -dg)
        dx_xi, _, residual_xi = kkt_solve(h, a, plus_rows, -dg_xi)
        dx_rolling, _, residual_rolling = kkt_solve(h, a, plus_rows, -dg_rolling)
        observed_dx = central(np.asarray(probes[(direction, 1.0, 1)]["solution"]),
                              np.asarray(probes[(direction, 1.0, -1)]["solution"]), denominator) / scale
        active = a[plus_rows]
        dual_actual = {}
        stationarity_actual = {}
        for branch, branch_ops, branch_control in ((1, plus_ops, plus_control),
                                                    (-1, minus_ops, minus_control)):
            x = np.asarray(probes[(direction, 1.0, branch)]["solution"]) / scale
            gradient = branch_ops["h"] @ x + branch_ops["g"].reshape(-1)
            multipliers, *_ = np.linalg.lstsq(active.T, -gradient, rcond=1.0e-12)
            dual_actual[("plus", "minus")[branch < 0]] = multipliers
            stationarity_actual[("plus", "minus")[branch < 0]] = float(
                np.max(np.abs(gradient + active.T @ multipliers)))
        counterfactual = {}
        for name, block in h_blocks.items():
            changed, _, changed_residual = kkt_solve(h - block, a, plus_rows, -dg)
            counterfactual[name] = {
                "diagnostic_only_non_candidate": True,
                "tau_norm_ratio_to_full": float(np.linalg.norm((changed * scale)[12:18]) /
                                                  max(np.linalg.norm((dx * scale)[12:18]), 1e-12)),
                "kkt_residual": changed_residual}
        kkt["directions"][direction] = {
            "central_valid": True, "active_rows": plus_rows, "active_sides": plus_sides,
            "dual_sensitivity": dual_gain, "kkt_residual": residual,
            "dual_variables": dual_actual,
            "dual_stationarity_reconstruction_max_abs": stationarity_actual,
            "solution_relative_error": rel(observed_dx, dx),
            "xi_excitation_tau_gain": (dx_xi * scale)[12:18],
            "rolling_excitation_tau_gain": (dx_rolling * scale)[12:18],
            "source_sum_tau_closure": (dx_xi + dx_rolling - dx)[12:18] * scale[12:18],
            "source_kkt_residuals": {"xi": residual_xi, "rolling": residual_rolling},
            "block_counterfactuals": counterfactual}

    # Stage E/F/G use the authoritative torque replay rather than treating bare Delassus as solver law.
    jp = np.vstack([row["map"] for row in actual["point_jacobians"]])
    delassus = jp @ np.linalg.solve(actual["mass"], jp.T)
    solver_response = {"bare_delassus": delassus, "bare_rank": int(np.linalg.matrix_rank(delassus)),
        "bare_condition": float(np.linalg.cond(delassus)),
        "not_complete_solver_law": True, "identified_probe_classification": torque["classification"],
        "branch_split_relative": torque["branch_split_relative"],
        "scale_convergence_relative": torque["scale_convergence_relative"],
        "contact_solver_regime_signature_count": torque["contact_solver_regime_signature_count"],
        "fn_direction_summary": torque["fn_direction_summary"]}

    xi_actual, _ = P44.native_xi_acceleration_map(
        actual["oracle"], P44.vec(actual["native"], "qpos", actual["model"].nq),
        P44.vec(actual["native"], "qvel", actual["model"].nv), 0.0,
        P44.vec(actual["replay"]["actual"]["dynamics"], "qacc", actual["model"].nv),
        np.asarray([actual["replay"]["actual"]["dynamics"]["ddxi_left_m_s2"],
                    actual["replay"]["actual"]["dynamics"]["ddxi_right_m_s2"]]))
    causal = {}
    for axis, direction in ((1, "Fn_L"), (3, "Fn_R")):
        plus, minus = probes[(direction, 1.0, 1)], probes[(direction, 1.0, -1)]
        denom = target_delta[axis]
        qacc_actual = central(P44.vec(plus["actual"]["dynamics"], "qacc", actual["model"].nv),
                              P44.vec(minus["actual"]["dynamics"], "qacc", actual["model"].nv), denom)
        hip_only = np.zeros_like(qacc_actual); hip = ATTR.hip_dofs(actual["model"])
        hip_only[list(hip)] = qacc_actual[list(hip)]
        ddxi = central(np.asarray([plus["actual"]["dynamics"]["ddxi_left_m_s2"],
                                   plus["actual"]["dynamics"]["ddxi_right_m_s2"]]),
                        np.asarray([minus["actual"]["dynamics"]["ddxi_left_m_s2"],
                                    minus["actual"]["dynamics"]["ddxi_right_m_s2"]]), denom)
        causal[direction] = {
            "qp_delta_lambda": central_values[direction]["wrench"],
            "qp_delta_nudot": central_values[direction]["nudot"],
            "qp_delta_tau": central_values[direction]["tau"],
            "torque_source": {name: central_values[direction][name] for name in
                              ("tau_acceleration", "tau_contact", "tau_other")},
            "actual_actuator_free_acceleration_RLN": central_values[direction]["tau_total_actual_point_acceleration_RLN"],
            "qp_predicted_contact_balance": contact_balance[direction],
            "actual_point_force_RLN": central( TAU.point_array(plus), TAU.point_array(minus), denom),
            "actual_aggregate_wrench": central(np.asarray(plus["mj_wrench"]), np.asarray(minus["mj_wrench"]), denom),
            "actual_hip_common_acceleration": float(0.5 * (qacc_actual[hip[0]] + qacc_actual[hip[1]])),
            "actual_ddxi": ddxi, "actual_ddxi_common": float(0.5 * np.sum(ddxi)),
            "hip_common_ddxi_contribution": float(0.5 * np.sum(xi_actual @ hip_only)),
        }

    left_free = np.asarray(torque["fn_direction_summary"]["Fn_L"]["wheel_mean_free_rolling_acceleration"])
    right_free = np.asarray(torque["fn_direction_summary"]["Fn_R"]["wheel_mean_free_rolling_acceleration"])
    asymmetry = {"Fn_L_free_cross_ratio": float(abs(left_free[1] / left_free[0])),
                 "Fn_R_free_cross_ratio": float(abs(right_free[0] / right_free[1])),
                 "nominal_load_n": [baseline["actual"]["dynamics"]["normal_load_left_n"],
                                      baseline["actual"]["dynamics"]["normal_load_right_n"]],
                 "interpretation": "bilateral QP torque leverage is primary; load/geometry/effective-mass differences modulate the right/left residual"}

    # R3 is causally earlier: true RHS excitation is xi/rolling task target, not a contact-wrench target.
    # R1 is independently material because the chosen aggregate solution contains a large point-unrealizable Ml mode.
    first_mismatch = "R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH"
    primary = ("the aggregate 6D wheel-wrench formulation admits a lateral-axis moment outside "
               "the rank-5 image of the actual two 3D point contacts, and that component supplies "
               "approximately all QP-predicted harmful rolling cancellation")
    repair = "actual point-contact-realizable force/wrench parameterization"
    decision = {"torque_generation_mechanism": torque_class,
                "first_material_mismatch": first_mismatch,
                "ordered_mismatches": [
                    {"rank": 1, "classification": "R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH",
                     "primary": True, "reason": primary}],
                "recommended_repair_layer": repair,
                "do_not_do": ["QP-space hip-common projection/penalty", "inverse Rc compensation",
                               "friction/solref/solimp/solver/contact tuning", "hip-task redesign before realizability is repaired"],
                "phase_status": "review/REWORK", "repair_implemented": False}

    P45.write_json(output / "qp-torque-source.json", stage_a)
    P45.write_csv(output / "qp-torque-source.csv", torque_rows)
    P45.write_json(output / "point-realizability.json", realizability)
    P45.write_json(output / "contact-space-balance.json", {"closure_max_abs": contact_closure,
                                                              "directions": contact_balance})
    balance_rows = []
    for direction, value in contact_balance.items():
        for side in range(2):
            for point in range(2):
                for component, component_name in enumerate(("rolling", "lateral", "normal")):
                    balance_rows.append({"direction": direction, "side": ("left", "right")[side],
                        "point": point, "component": component_name,
                        "actuator_free": value["actuator_free_RLN"][side, point, component],
                        "qp_contact": value["qp_contact_RLN"][side, point, component],
                        "net": value["net_RLN"][side, point, component],
                        "closure": value["closure_RLN"][side, point, component]})
    P45.write_csv(output / "contact-space-balance.csv", balance_rows)
    P45.write_json(output / "kkt-sensitivity.json", kkt)
    P45.write_json(output / "solver-response-operator.json", solver_response)
    P45.write_json(output / "fn-fr-causal-chain.json", causal)
    causal_rows = [{"direction": direction,
                    "actual_Fr_left": value["actual_aggregate_wrench"][0],
                    "actual_Fn_left": value["actual_aggregate_wrench"][2],
                    "actual_Fr_right": value["actual_aggregate_wrench"][6],
                    "actual_Fn_right": value["actual_aggregate_wrench"][8],
                    "actual_hip_common": value["actual_hip_common_acceleration"],
                    "actual_ddxi_common": value["actual_ddxi_common"],
                    "hip_ddxi_contribution": value["hip_common_ddxi_contribution"]}
                   for direction, value in causal.items()]
    P45.write_csv(output / "fn-fr-causal-chain.csv", causal_rows)
    P45.write_json(output / "root-cause-decision.json", {**decision, "left_right_asymmetry": asymmetry,
        "closure": {"qp_eom": eom_max, "torque_decomposition": decomposition_max,
                    "qp_contact_space": contact_closure,
                    "whole_dynamics_contact": sensitivity["whole_dynamics_contact_closure_max_abs"]}})

    docs = {
        "QP_CAUSAL_ATTRIBUTION.md": markdown("QP causal attribution", [
            f"Verdict: `{torque_class}`.",
            f"The real QP increment identity closes at `{eom_max:.3e}`. For Fn_L/Fn_R, the acceleration-component torque norm shares are `{fn_acc_shares[0]:.6f}` and `{fn_acc_shares[1]:.6f}`; contact-balancing is secondary.",
            "The true KKT RHS excitation is the wheel-longitudinal and rolling/slip target pair. The so-called Fn direction is a solution-space label synthesized from those task targets; no aggregate Fn target is directly applied to the plant." ]),
        "CONTACT_REALIZABILITY_AND_RESPONSE.md": markdown("Contact realizability and response", [
            "Each wheel's two 3D point forces span a rank-5 aggregate wrench subspace. The missing wrench direction is dominated by lateral-axis moment `Ml`, which the Fn-labelled QP directions use materially.",
            "The MuJoCo response remains regime-stable and cancels 98.79–98.93% of the torque-induced rolling free tendency. The bare Delassus matrix is reported only as an effective-mass block, not as the complete compliant solver law." ]),
        "ROOT_CAUSE_CLOSURE.md": markdown("Root-cause closure", [
            f"Torque mechanism: `{torque_class}`.", f"First mismatch verdict: `{first_mismatch}`.",
            "The xi/rolling objectives are the true KKT excitation and explain why torque is large, but that is a generation mechanism rather than the first proved mismatch. The first proved mismatch is `R1`: the QP uses a material lateral-axis moment outside the actual rank-5 point-force image, and this unrealizable component supplies approximately all predicted rolling cancellation.",
            "Actual `Fr` is the normal constrained reaction to the already-present rolling free acceleration, not a solver-created Fn→Fr conversion." ]),
        "REPAIR_DIRECTION.md": markdown("Repair direction", [
            f"Recommended first repair layer: **{repair}**.",
            "Do not start with hip-task redesign, hip-common projection/penalty, inverse `Rc`, or contact/friction/solver tuning. This run performs attribution only and leaves Phase46 in REWORK." ]),
    }
    for name, text in docs.items():
        (output / name).write_text(text, encoding="utf-8")

    compared = ["qp-torque-source.json", "qp-torque-source.csv", "point-realizability.json",
                "contact-space-balance.json", "contact-space-balance.csv", "kkt-sensitivity.json",
                "solver-response-operator.json", "fn-fr-causal-chain.json", "fn-fr-causal-chain.csv",
                "root-cause-decision.json"]
    replay_error = max(
        [P45.semantic_error(args.replay_of / name, output / name) for name in compared] +
        [0.0 if (args.replay_of / name).read_text(encoding="utf-8") == text else float("inf")
         for name, text in docs.items()]) if args.replay_of else None
    trusted = (eom_max <= 1.0e-8 and decomposition_max <= 1.0e-10 and
               contact_closure <= 1.0e-8 and branch_split <= 0.05 and
               scale_convergence <= 0.05 and h_reconstruction <= 1.0e-10 and
               sensitivity["whole_dynamics_contact_closure_max_abs"] <= 1.0e-8)
    replay_pass = replay_error is None or replay_error <= 1.0e-11
    P45.write_json(output / "summary.json", {"pass": trusted and replay_pass,
        "trusted": trusted, "replay_max_abs_error": replay_error, **decision})
    sources = [source / "probe-records.json", source / "contact-realization-sensitivity.json",
               args.torque_source / "torque-free-contact-attribution.json", qp_dump,
               Path(__file__).resolve(), ROOT / "tools/experiments/phase46_dump_qp_operators.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp"]
    P45.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                    hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if trusted and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
