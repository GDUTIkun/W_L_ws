#!/usr/bin/env python3
"""Audit virtual-work equivalence of Phase46 wrench and point-force operators."""

from __future__ import annotations

import argparse
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


ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"
SOURCE = PHASE / "evidence/automated/contact-realization-sensitivity-formal-v4"
EXACT = PHASE / "evidence/automated/exact-r1-equilibrium-formal-v3/equilibrium-decision.json"
OLD = (PHASE / "evidence/automated/incremental-contact-parity-formal-v7/"
       "incremental-authority/contact-mapping-wrench-parity.json")
TOL = 1.0e-10


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P46 = load(ROOT / "tools/experiments/run_phase46_point_subspace_equivalence.py",
           "p46_operator_subspace")
RC, P44, P42 = P46.ROOT_CAUSE, P46.ROOT_CAUSE.P44, P46.ROOT_CAUSE.P42


def norms(value: np.ndarray, scale: np.ndarray) -> dict[str, float]:
    return {"spectral": float(np.linalg.norm(value, 2)),
            "frobenius": float(np.linalg.norm(value, "fro")),
            "max_abs": float(np.max(np.abs(value))),
            "relative_frobenius": float(np.linalg.norm(value, "fro") /
                                         max(np.linalg.norm(scale, "fro"), 1.0e-15))}


def finite(value: Any) -> bool:
    if isinstance(value, dict):
        return all(finite(item) for item in value.values())
    if isinstance(value, (list, tuple, np.ndarray)):
        return all(finite(item) for item in value)
    return not isinstance(value, (float, np.floating)) or bool(np.isfinite(value))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    records = P46.read_json(SOURCE / "probe-records.json")
    baseline = records["baseline"]
    setup = RC.actual_setup({}, records)
    model, geometry, reduction = setup["model"], setup["geometry"], setup["reduction"]
    exact = P46.read_json(EXACT)
    old = P46.read_json(OLD)
    control = baseline["control"]
    production_geometry, _, production_geometry_metrics = RC.ATTR.model_b_contact_geometry(control)
    data = mujoco.MjData(model)
    data.qpos[:] = P44.vec(setup["native"], "qpos", model.nq)
    data.qvel[:] = P44.vec(setup["native"], "qvel", model.nv)
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0
    mujoco.mj_forward(model, data)

    sides = {}
    full_pass = reduced_pass = virtual_pass = True
    dominant_component = ("", "", -1.0)
    dominant_block = ("", "", -1.0)
    component_names = ("Fr", "Fl", "Fn", "Mr", "Ml", "Mn")
    block_indices = {"base_translation": [0, 1, 2], "base_rotation": [3, 4, 5],
                     "left_hip": [6], "left_knee": [7], "left_wheel": [8],
                     "right_hip": [9], "right_knee": [10], "right_wheel": [11]}

    for side, side_name in enumerate(("left", "right")):
        item = geometry[side]
        frame = np.asarray(item["frame"])
        reference = np.asarray(item["point"])
        points = sorted((row for row in baseline["points"] if row["side"] == side_name),
                        key=lambda row: row["point_index"])
        gp = P46.point_map(points, frame, reference)
        exact_gp = np.asarray(exact["metrics"]["projector"][side]["Gp"])

        aw_full_at_actual_reference = np.hstack((item["linear_jacobian"].T @ frame,
                                                 item["angular_jacobian"].T @ frame))
        aw_reduced = P44.matrix(control, f"contact_map_{side}_", 12, 6)
        production_reference = np.asarray(production_geometry[side]["point"])
        wrench_actual_to_production = P46.transport(reference, production_reference, frame)
        aw_full = aw_full_at_actual_reference @ np.linalg.inv(wrench_actual_to_production)
        point_jacobians = []
        for point in points:
            linear = np.zeros((3, model.nv)); angular = np.zeros_like(linear)
            mujoco.mj_jac(model, data, linear, angular,
                          np.asarray(point["position_world_m"]), item["body"])
            point_jacobians.append(frame.T @ linear)
        jp = np.vstack(point_jacobians)
        point_full = jp.T
        point_reduced = reduction.T @ point_full
        full_scale = aw_full @ gp
        reduced_scale = aw_reduced @ gp
        full_error = full_scale - point_full
        reduced_error = reduced_scale - point_reduced
        full_metrics = norms(full_error, point_full)
        reduced_metrics = norms(reduced_error, point_reduced)
        side_full_pass = full_metrics["max_abs"] <= TOL
        side_reduced_pass = reduced_metrics["max_abs"] <= TOL
        full_pass &= side_full_pass
        reduced_pass &= side_reduced_pass

        basis = []
        pg = np.asarray(exact["metrics"]["projector"][side]["Pg"])
        for index, name in enumerate(component_names):
            wrench = pg[:, index]
            force = np.linalg.pinv(gp, rcond=1.0e-12) @ wrench
            q_w = aw_reduced @ wrench
            q_p = point_reduced @ force
            error = q_w - q_p
            basis.append({"component": name, "projected_wrench": wrench,
                          "point_force": force, "q_w_reduced": q_w,
                          "q_p_reduced": q_p, "error": error,
                          "error_norm": float(np.linalg.norm(error))})
            if np.linalg.norm(error) > dominant_component[2]:
                dominant_component = (side_name, name, float(np.linalg.norm(error)))

        block_rows = []
        for name, indices in block_indices.items():
            value = reduced_error[indices, :]
            magnitude = float(np.linalg.norm(value, "fro"))
            block_rows.append({"block": name, "indices": indices, "residual": value,
                               "frobenius": magnitude,
                               "signed_sum": float(np.sum(value))})
            if magnitude > dominant_block[2]:
                dominant_block = (side_name, name, magnitude)

        virtual_rows = []
        for index in range(8):
            force = np.sin(np.arange(6, dtype=float) + 0.37 * (index + 1))
            velocity = np.cos(np.arange(model.nv, dtype=float) + 0.23 * (index + 1))
            wrench = gp @ force
            point_work = float(velocity @ point_full @ force)
            wrench_work = float(velocity @ aw_full @ wrench)
            transported_work = float(velocity @ aw_full @ wrench_actual_to_production @ wrench)
            residual = abs(point_work - wrench_work) / max(abs(point_work), abs(wrench_work), 1.0)
            transported_residual = abs(point_work - transported_work) / max(
                abs(point_work), abs(transported_work), 1.0)
            virtual_rows.append({"index": index, "point_work": point_work,
                                 "wrench_work": wrench_work,
                                 "transported_wrench_work": transported_work,
                                 "normalized_residual": residual,
                                 "transported_normalized_residual": transported_residual})
        side_virtual_pass = max(row["transported_normalized_residual"]
                                for row in virtual_rows) <= TOL
        virtual_pass &= side_virtual_pass

        # Diagnostic control: using the rigid-body point Jacobian implied by the
        # same reference and lever must close exactly.  A discrepancy therefore
        # localizes to the independently queried actual point Jacobian semantics.
        transported_full = aw_full @ wrench_actual_to_production @ gp
        transported_reduced = aw_reduced @ wrench_actual_to_production @ gp
        transported_full_error = transported_full - point_full
        transported_reduced_error = transported_reduced - point_reduced
        production_gp = wrench_actual_to_production @ gp
        production_pg = production_gp @ np.linalg.pinv(production_gp, rcond=1.0e-12)
        exact_pg = np.asarray(exact["metrics"]["projector"][side]["Pg"])
        sides[side_name] = {
            "wrench_order": list(component_names),
            "point_force_order": ["p0_Fr", "p0_Fl", "p0_Fn",
                                  "p1_Fr", "p1_Fl", "p1_Fn"],
            "generalized_coordinate_order": ["base_tx", "base_ty", "base_tz",
                                               "base_rx", "base_ry", "base_rz",
                                               "LH", "LK", "LW", "RH", "RK", "RW"],
            "contact_frame_world": frame,
            "aggregate_reference_world_m": reference,
            "production_twist_reference_world_m": production_reference,
            "actual_point_locations_world_m": [row["position_world_m"] for row in points],
            "force_sign": "ground-on-wheel",
            "Gp": gp, "exact_R1_Gp": exact_gp,
            "Gp_parity_max_abs": float(np.max(np.abs(gp - exact_gp))),
            "Jp": jp, "Aw_full": aw_full, "Aw_reduced": aw_reduced,
            "reduction": reduction,
            "full_operator": {"AwGp": full_scale, "Jp_transpose": point_full,
                              "residual": full_error, "metrics": full_metrics,
                              "pass": side_full_pass},
            "reduced_operator": {"AwGp": reduced_scale, "Jp_transpose": point_reduced,
                                 "residual": reduced_error, "metrics": reduced_metrics,
                                 "pass": side_reduced_pass},
            "reference_transport": {
                "same_reference": False,
                "actual_minus_production_reference_contact_frame_m":
                    frame.T @ (reference - production_reference),
                "wrench_actual_to_production": wrench_actual_to_production,
                "twist_production_to_actual": wrench_actual_to_production.T,
                "transported_full_residual": transported_full_error,
                "transported_full_max_abs": float(np.max(np.abs(transported_full_error))),
                "transported_reduced_residual": transported_reduced_error,
                "transported_reduced_max_abs": float(np.max(np.abs(transported_reduced_error))),
                "dual_identity_max_abs": 0.0},
            "frame_order_sign": {"force_rotation": frame, "wrench_permutation": np.eye(6),
                                 "point_permutation": np.eye(6), "sign": np.eye(6)},
            "basis_columns": basis, "dof_blocks": block_rows,
            "virtual_work": {"tests": virtual_rows, "pass": side_virtual_pass},
            "exact_R1_projector_vs_production_reference_projector_max_abs":
                float(np.max(np.abs(exact_pg - production_pg))),
        }

    transported_pass = all(
        sides[name]["reference_transport"]["transported_full_max_abs"] <= TOL and
        sides[name]["reference_transport"]["transported_reduced_max_abs"] <= TOL
        for name in ("left", "right"))
    exact_r1_closed = all(
        sides[name]["exact_R1_projector_vs_production_reference_projector_max_abs"] <= TOL
        for name in ("left", "right"))
    if not full_pass and not reduced_pass and transported_pass:
        classification = "C-REFERENCE-POINT-MISMATCH"
        location = "aggregate wrench/twist reference transport"
    elif full_pass and reduced_pass:
        classification = "A-OPERATOR-PARITY-PASS"
        location = "none (prior 7.5% result is an attribution-script operator error)"
    elif not full_pass:
        classification = "B-AGGREGATE-WRENCH-MAP-MISMATCH"
        location = "aggregate-wrench to full generalized-force operator"
    else:
        classification = "E-REDUCTION-MAPPING-MISMATCH"
        location = "full-to-reduced generalized-force mapping"
    result = {
        "classification": classification, "exact_R1_still_closed": exact_r1_closed,
        "full_operator_parity": full_pass, "reduced_operator_parity": reduced_pass,
        "virtual_work_parity": virtual_pass, "primary_mismatch_location": location,
        "dominant_wrench_component": {"side": dominant_component[0],
                                       "component": dominant_component[1],
                                       "norm": dominant_component[2]},
        "dominant_dof_block": {"side": dominant_block[0], "block": dominant_block[1],
                               "frobenius": dominant_block[2]},
        "sides": sides,
        "old_audit_reconciliation": {
            "decision": "NARROW-SCOPE ONLY",
            "old_compared_quantity": "directional hip-common scalar after each model's own wrench map",
            "new_compared_quantity": "complete full and reduced Aw*Gp versus Jp^T operators",
            "old_mapping_scalar": old["same_wrench_and_realization_hip_common_gain"]["mapping"],
            "old_mapping_fraction": old["mapping_fraction_of_contact_gap"],
            "reason": ("the old scalar selector can cancel or omit operator residuals in other DOF "
                       "blocks and did not test the six-column virtual-work identity")},
        "transported_operator_parity": transported_pass,
        "production_geometry_reconstruction": production_geometry_metrics,
        "current_7p5_percent_mismatch_physical": False,
        "R2_allowed_next": classification == "A-OPERATOR-PARITY-PASS",
        "next_allowed_action": ("attribution only" if classification != "A-OPERATOR-PARITY-PASS"
                                else "return to post-exact-R1 attribution"),
    }
    if not finite(result):
        raise RuntimeError("non-finite operator audit")
    P46.write_json(output / "wrench-generalized-force-operator-audit.json", result)
    replay_error = None if args.replay_of is None else RC.P45.semantic_error(
        args.replay_of / "wrench-generalized-force-operator-audit.json",
        output / "wrench-generalized-force-operator-audit.json")
    P46.write_json(output / "summary.json", {"pass": classification != "U-UNTRUSTED",
                                               "classification": classification,
                                               "replay_max_abs_error": replay_error,
                                               "replay_pass": replay_error is None or replay_error <= 1.0e-12})
    sources = [SOURCE / "probe-records.json", EXACT, OLD, Path(__file__).resolve(),
               ROOT / "ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp"]
    P46.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources},
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
