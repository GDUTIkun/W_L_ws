#!/usr/bin/env python3
"""Run Phase46 corrected production-reference exact-R1 COMP, then EQ."""

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
PRODUCTION_AUDIT = (PHASE / "evidence/automated/production-reference-image-audit-formal-v1/"
                    "production-reference-image-audit.json")
OPERATOR_AUDIT = (PHASE / "evidence/automated/wrench-generalized-force-operator-audit-formal-v2/"
                  "wrench-generalized-force-operator-audit.json")
TOL = 1.0e-10


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


EVAL = load(ROOT / "tools/experiments/run_phase46_point_realizable_repair.py",
            "p46_corrected_eval")
ATTR, P45C, P45, P44, P42 = EVAL.ATTR, EVAL.P45C, EVAL.P45, EVAL.P44, EVAL.P42


def read(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


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
    completed = subprocess.run(
        [str(executable), str(row), "point-realizable"], cwd=ROOT,
        env=environment, check=True, text=True, capture_output=True)
    values: dict[str, list[tuple[int, int, float]]] = {}
    for name, row_index, column_index, value in csv.reader(completed.stdout.splitlines()):
        values.setdefault(name, []).append((int(row_index), int(column_index), float(value)))
    return {name: matrix(values, name) for name in values}


def max_abs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    config_path = args.config.resolve()
    qp_dump = args.qp_dump.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    config = read(config_path)
    continuation_path = ROOT / config["continuation_config"]
    continuation = read(continuation_path)
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    control = P45.run(base, output / "baseline.csv", config["case_id"],
                      authority=authority, tick=0, delta=np.zeros(4),
                      wrench_trim=trim)[0]
    operators = dump(qp_dump, output / "baseline.csv")
    production = read(PRODUCTION_AUDIT)
    operator_source = read(OPERATOR_AUDIT)

    side_rows = []
    full_operator_error = 0.0
    reduced_operator_error = 0.0
    point_reconstruction_error = 0.0
    controller_semantic_error = 0.0
    for side, name in enumerate(("left", "right")):
        expected = production["sides"][name]
        source = operator_source["sides"][name]
        gp = np.asarray(expected["Gp_production"])
        pg = np.asarray(expected["Pg_production"])
        controller = operators[f"point_force_wrench_projector_{side}"]
        singular = np.linalg.svd(gp, compute_uv=False)
        rank = int(np.linalg.matrix_rank(gp, tol=TOL))
        missing = np.linalg.svd(gp, full_matrices=True)[0][:, rank]
        projector_difference = controller - pg
        reconstruction = gp @ np.linalg.pinv(gp, rcond=1.0e-12) @ controller - controller

        aw_full = np.asarray(source["Aw_full"])
        aw_reduced = np.asarray(source["Aw_reduced"])
        jp_transpose = np.asarray(source["Jp"]).T
        full_residual = aw_full @ gp - jp_transpose
        reduced_residual = aw_reduced @ gp - np.asarray(source["reduction"]).T @ jp_transpose
        full_operator_error = max(full_operator_error, max_abs(full_residual))
        reduced_operator_error = max(reduced_operator_error, max_abs(reduced_residual))

        wrench = np.asarray([
            float(control[f"physical_solution{18 + 6 * side + index}"])
            for index in range(6)])
        point_force = np.linalg.pinv(gp, rcond=1.0e-12) @ wrench
        wrench_reconstruction = gp @ point_force - wrench
        point_reconstruction_error = max(point_reconstruction_error,
                                         max_abs(wrench_reconstruction))

        normalized_missing = np.zeros(42)
        variable_scale = operators["variable_scale"].reshape(-1)
        normalized_missing[18 + 6 * side:24 + 6 * side] = (
            missing / variable_scale[18 + 6 * side:24 + 6 * side])
        constraint_null = max_abs(operators["a"] @ normalized_missing)
        raw_task_null = max_abs(operators["h"] @ normalized_missing)
        task_null = max_abs(
            operators["h"] @ normalized_missing - 1.0e-6 * normalized_missing)
        gradient_null = abs(float(operators["g"].reshape(-1) @ normalized_missing))
        output_range = max_abs((np.eye(6) - controller) @ wrench)
        semantics = max(constraint_null, task_null, gradient_null, output_range)
        controller_semantic_error = max(controller_semantic_error, semantics)

        checks = {
            "rank": rank == 5,
            "symmetry": max_abs(controller - controller.T) <= TOL,
            "idempotence": max_abs(controller @ controller - controller) <= TOL,
            "controller_equals_Pg_prod": max_abs(projector_difference) <= TOL,
            "range_contains_Gp_prod": max_abs((np.eye(6) - controller) @ gp) <= TOL,
            "controller_range_in_Gp_prod": max_abs((np.eye(6) - pg) @ controller) <= TOL,
            "missing_direction_annihilated": max_abs(controller @ missing) <= TOL,
            "projected_basis_reconstructs": max_abs(reconstruction) <= TOL,
            "physical_wrench_reconstructs": max_abs(wrench_reconstruction) <= TOL,
            "qp_operator_uses_physical_range": max(constraint_null, task_null,
                                                     gradient_null) <= TOL,
            "controller_output_in_physical_range": output_range <= TOL,
        }
        side_rows.append({
            "side": name,
            "rank": rank,
            "singular_values": singular,
            "controller_projector": controller,
            "Pg_production": pg,
            "projector_difference_max_abs": max_abs(projector_difference),
            "projector_difference_spectral": float(np.linalg.norm(projector_difference, 2)),
            "symmetry_max_abs": max_abs(controller - controller.T),
            "idempotence_max_abs": max_abs(controller @ controller - controller),
            "Gp_to_controller_containment_max_abs": max_abs((np.eye(6) - controller) @ gp),
            "controller_to_Gp_containment_max_abs": max_abs((np.eye(6) - pg) @ controller),
            "missing_direction": missing,
            "missing_direction_annihilation_max_abs": max_abs(controller @ missing),
            "projected_basis_reconstruction_max_abs": max_abs(reconstruction),
            "physical_wrench": wrench,
            "point_force": point_force,
            "point_force_reconstruction_max_abs": max_abs(wrench_reconstruction),
            "full_operator_parity_max_abs": max_abs(full_residual),
            "reduced_operator_parity_max_abs": max_abs(reduced_residual),
            "constraint_missing_direction_max_abs": constraint_null,
            "raw_h_missing_direction_max_abs_including_numerical_regularization":
                raw_task_null,
            "task_missing_direction_max_abs": task_null,
            "gradient_missing_direction_abs": gradient_null,
            "controller_output_range_residual_max_abs": output_range,
            "checks": checks,
            "pass": all(checks.values()),
        })

    comp_pass = (all(row["pass"] for row in side_rows) and
                 full_operator_error <= TOL and reduced_operator_error <= TOL and
                 point_reconstruction_error <= TOL and controller_semantic_error <= TOL)
    comp = {
        "pass": comp_pass,
        "classification": "CORRECTED-R1-COMP-PASS" if comp_pass else
                          "CORRECTED-R1-COMP-FAIL",
        "scope": "Phase46 compatible-H0 tick0 frozen production-reference exact-R1",
        "controller_projector_equals_Pg_prod": all(
            row["checks"]["controller_equals_Pg_prod"] for row in side_rows),
        "production_reference_image_closure": all(row["pass"] for row in side_rows),
        "full_operator_parity": full_operator_error <= TOL,
        "reduced_operator_parity": reduced_operator_error <= TOL,
        "point_force_reconstruction": point_reconstruction_error <= TOL,
        "controller_semantics": controller_semantic_error <= TOL,
        "maximum_full_operator_parity_max_abs": full_operator_error,
        "maximum_reduced_operator_parity_max_abs": reduced_operator_error,
        "maximum_point_force_reconstruction_max_abs": point_reconstruction_error,
        "maximum_controller_semantic_error": controller_semantic_error,
        "sides": side_rows,
        "old_candidate_results": "superseded by production-reference corrected candidate",
        "R2_authorized": False,
    }
    P45.write_json(output / "corrected-exact-r1-comp.json", comp)

    if comp_pass:
        model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
        oracle = P42.Oracle(read(ROOT / base["phase42_config"]))
        actual = P45.actual(base, model, oracle, native, control)
        qp_output, mj_output = P45C.task_output(control, actual)
        contact_count = [sum(int(row["side"]) == side for row in actual["details"])
                         for side in range(2)]
        equilibrium_metrics = {
            "qp_ddxi_per_side_m_s2": np.asarray(qp_output[:2]),
            "qp_material_tangent_acceleration_per_side_m_s2": np.asarray(qp_output[2:]),
            "mujoco_ddxi_per_side_m_s2": np.asarray(mj_output[:2]),
            "mujoco_material_tangent_acceleration_per_side_m_s2": np.asarray(mj_output[2:]),
            "contact_count_per_side": contact_count,
            "normal_load_per_side_n": [float(control["normal_left"]),
                                        float(control["normal_right"])],
            "hard_violation": float(control["hard"]),
            "maximum_normalized_slack": float(control["maximum_normalized_slack"]),
            "minimum_torque_margin_nm": min(float(control[f"tau_margin{i}"])
                                              for i in range(6)),
            "whole_dynamics_closure": float(
                actual["dynamics"]["full_dynamics_residual_max_abs"]),
            "contact_reconstruction_closure": float(
                actual["dynamics"]["contact_applyft_jacobian_max_abs"]),
        }
        equilibrium_pass = (
            max(abs(value) for value in equilibrium_metrics["mujoco_ddxi_per_side_m_s2"]) <= 0.05 and
            max(abs(value) for value in equilibrium_metrics[
                "mujoco_material_tangent_acceleration_per_side_m_s2"]) <= 0.01 and
            contact_count == [2, 2] and
            min(equilibrium_metrics["normal_load_per_side_n"]) > 0.0 and
            equilibrium_metrics["hard_violation"] <= 1.0e-7 and
            equilibrium_metrics["maximum_normalized_slack"] <= 0.05 and
            equilibrium_metrics["minimum_torque_margin_nm"] >= -1.0e-10 and
            equilibrium_metrics["whole_dynamics_closure"] <= 1.0e-8 and
            equilibrium_metrics["contact_reconstruction_closure"] <= 1.0e-8)
        equilibrium = {
            "pass": equilibrium_pass,
            "entered": True,
            "classification": ("CORRECTED_EXACT_R1_EQ_PASS" if equilibrium_pass else
                               "CORRECTED_EXACT_R1_EQ_FAIL"),
            "metrics": equilibrium_metrics,
            "R1_exactly_closed_at_frozen_H0": True,
            "R1_repair_preserves_equilibrium": equilibrium_pass,
            "R2_authorized": False,
            "next_allowed_action": ("fixed-state authority audit" if equilibrium_pass else
                                    "post-corrected-exact-R1 attribution"),
        }
    else:
        equilibrium = {
            "pass": False, "entered": False, "classification": "NOT-RUN",
            "metrics": {}, "R1_exactly_closed_at_frozen_H0": False,
            "R1_repair_preserves_equilibrium": False, "R2_authorized": False,
            "next_allowed_action": "implementation fix only",
        }
    P45.write_json(output / "corrected-exact-r1-equilibrium.json", equilibrium)

    compared = ("corrected-exact-r1-comp.json", "corrected-exact-r1-equilibrium.json")
    replay_error = max((P45.semantic_error(args.replay_of / name, output / name)
                        for name in compared), default=0.0) if args.replay_of else None
    replay_pass = replay_error is None or replay_error <= 1.0e-11
    P45.write_json(output / "summary.json", {
        "pass": comp_pass and equilibrium.get("pass", False) and replay_pass,
        "comp_pass": comp_pass,
        "equilibrium_status": ("PASS" if equilibrium.get("pass") else
                               "FAIL" if equilibrium.get("entered") else "NOT RUN"),
        "replay_max_abs_error": replay_error,
        "replay_pass": replay_pass,
        "R2_authorized": False,
        "next_allowed_action": equilibrium["next_allowed_action"],
    })
    sources = [config_path, continuation_path, ROOT / base["scene"],
               ROOT / base["executable"], authority, wrench_source, qp_dump,
               PRODUCTION_AUDIT, OPERATOR_AUDIT, Path(__file__).resolve(),
               ROOT / "tools/experiments/phase46_dump_qp_operators.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp"]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                    hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
    })
    return 0 if comp_pass and equilibrium.get("pass", False) and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
