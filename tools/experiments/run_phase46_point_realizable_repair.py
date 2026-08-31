#!/usr/bin/env python3
"""Evaluate the frozen Phase46 point-realizable repair gates."""

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


ROOT = Path(__file__).resolve().parents[2]


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ATTR = load(ROOT / "tools/experiments/run_phase45_rework_authority_attribution.py",
            "p46_point_attr")
P45C, P45, P44, P42 = ATTR.P45C, ATTR.P45, ATTR.P44, ATTR.P42


def read_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def read_row(path: Path) -> dict[str, str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return next(csv.DictReader(stream))


def finite_tree(value: Any) -> bool:
    if isinstance(value, dict):
        return all(finite_tree(item) for item in value.values())
    if isinstance(value, list):
        return all(finite_tree(item) for item in value)
    return not isinstance(value, (int, float)) or np.isfinite(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    config_path = args.config.resolve()
    source = args.source.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    config = read_json(config_path)
    gates = config["repair_gates"]
    matrices = read_json(source / "common-transfer-matrices.json")
    raw_summary = read_json(source / "summary.json")
    baseline_control = read_row(source / "probes/baseline-detail.csv")

    continuation_path = ROOT / config["continuation_config"]
    continuation = read_json(continuation_path)
    base, _, _ = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(read_json(ROOT / base["phase42_config"]))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    actual = P45.actual(base, model, oracle, native, baseline_control)
    qp_output, mj_output = P45C.task_output(baseline_control, actual)

    geometry, _, geometry_metrics = ATTR.model_b_contact_geometry(baseline_control)
    model_data = mujoco.MjData(model)
    model_data.qpos[:] = actual["qpos"]
    model_data.qvel[:] = actual["qvel"]
    mujoco.mj_forward(model, model_data)
    projector_metrics = []
    maximum_axial_moment = 0.0
    for side, item in enumerate(geometry):
        axis_world = model_data.ximat[item["body"]].reshape(3, 3)[:, 0]
        axis = item["frame"].T @ axis_world
        axis /= np.linalg.norm(axis)
        projector = np.eye(6)
        projector[3:, 3:] -= np.outer(axis, axis)
        projector_metrics.append({
            "side": ("left", "right")[side],
            "rank": int(np.linalg.matrix_rank(projector, tol=1.0e-12)),
            "symmetry_max_abs": float(np.max(np.abs(projector - projector.T))),
            "idempotence_max_abs": float(np.max(np.abs(projector @ projector - projector))),
            "axis_null_max_abs": float(np.max(np.abs(projector @ np.r_[np.zeros(3), axis]))),
            "axis_contact_frame": axis,
        })
        for path in sorted((source / "probes").glob("*.csv")):
            if path.stem.endswith("_native"):
                continue
            row = read_row(path)
            moment = np.asarray([float(row[f"physical_solution{18 + 6 * side + index}"])
                                 for index in range(3, 6)])
            maximum_axial_moment = max(maximum_axial_moment, abs(float(axis @ moment)))

    component_pass = (
        all(item["rank"] == 5 and item["symmetry_max_abs"] <= 1.0e-12 and
            item["idempotence_max_abs"] <= 1.0e-12 and
            item["axis_null_max_abs"] <= 1.0e-12 for item in projector_metrics) and
        maximum_axial_moment <= float(gates["maximum_abs_axial_moment_nm"]))

    contact_count = [sum(int(row["side"]) == side for row in actual["details"])
                     for side in range(2)]
    baseline = {
        "qp_ddxi_per_side_m_s2": np.asarray(qp_output[:2]),
        "qp_material_tangent_acceleration_per_side_m_s2": np.asarray(qp_output[2:]),
        "mujoco_ddxi_per_side_m_s2": np.asarray(mj_output[:2]),
        "mujoco_material_tangent_acceleration_per_side_m_s2": np.asarray(mj_output[2:]),
        "contact_count_per_side": contact_count,
        "hard_violation": float(baseline_control["hard"]),
        "maximum_normalized_slack": float(baseline_control["maximum_normalized_slack"]),
        "minimum_torque_margin_nm": min(float(baseline_control[f"tau_margin{i}"])
                                         for i in range(6)),
        "whole_dynamics_closure": float(actual["dynamics"]["full_dynamics_residual_max_abs"]),
        "contact_applyft_closure": float(actual["dynamics"]["contact_applyft_jacobian_max_abs"]),
    }
    equilibrium_pass = (
        max(abs(value) for value in baseline["mujoco_ddxi_per_side_m_s2"]) <= 0.05 and
        max(abs(value) for value in baseline["mujoco_material_tangent_acceleration_per_side_m_s2"]) <= 0.01 and
        contact_count == [2, 2] and baseline["hard_violation"] <= 1.0e-7 and
        baseline["maximum_normalized_slack"] <= 0.05 and
        baseline["minimum_torque_margin_nm"] >= -1.0e-10 and
        baseline["whole_dynamics_closure"] <= 1.0e-8 and
        baseline["contact_applyft_closure"] <= 1.0e-8)

    xi = np.asarray(matrices["xi_common_only"]["g_mj"])
    slip = np.asarray(matrices["slip_common_only"]["g_mj"])
    old_cross = float(gates["phase45_harmful_cross_gain"])
    old_slip = float(gates["phase45_slip_self_gain"])
    cross_reduction = 1.0 - abs(float(slip[0])) / abs(old_cross)
    branch_scale_pass = bool(raw_summary["all_directional_scales_trusted"])
    authority_checks = {
        "cross_abs": abs(float(slip[0])) <= float(gates["maximum_abs_actual_cross_gain"]),
        "cross_reduction": cross_reduction >= float(gates["minimum_cross_reduction_fraction"]),
        "slip_self_positive": float(slip[1]) > 0.0,
        "slip_self_retained": float(slip[1]) >=
            float(gates["minimum_slip_self_retention_fraction"]) * old_slip,
        "xi_self": float(xi[0]) >= float(gates["minimum_abs_xi_self_gain"]),
        "branch_scale": branch_scale_pass,
    }
    authority_pass = all(authority_checks.values())
    entered = {"component": True, "equilibrium": component_pass,
               "authority": component_pass and equilibrium_pass,
               "realization": component_pass and equilibrium_pass and authority_pass}
    mandatory_stop = ("DG46P-COMP" if not component_pass else
                      "DG46P-EQ" if not equilibrium_pass else
                      "DG46P-AUTH" if not authority_pass else None)
    classification = ("P46P-PASS" if mandatory_stop is None else
                      "P46P-EQ-FAIL" if mandatory_stop == "DG46P-EQ" else
                      "P46-D-SLIP_AUTHORITY_DESTROYED" if
                      mandatory_stop == "DG46P-AUTH" and
                      not authority_checks["slip_self_positive"] else
                      "P46-B-CROSS_REDUCED_BUT_INSUFFICIENT" if
                      mandatory_stop == "DG46P-AUTH" else "P46-U-COMPONENT-FAIL")
    decision = {
        "pass": component_pass and equilibrium_pass and authority_pass,
        "classification": classification,
        "mandatory_gate_stop": mandatory_stop,
        "entered_gates": entered,
        "component_pass": component_pass,
        "equilibrium_pass": equilibrium_pass,
        "authority_pass": authority_pass,
        "projector": {"sides": projector_metrics,
                      "maximum_abs_axial_moment_nm": maximum_axial_moment},
        "baseline": baseline,
        "authority": {
            "status": ("gate_evidence" if entered["authority"] else
                       "diagnostic_only_not_entered_after_equilibrium_failure"),
            "phase45_harmful_cross_gain": old_cross,
            "actual_slip_to_ddxi_common_gain": float(slip[0]),
            "cross_reduction_fraction": cross_reduction,
            "phase45_slip_self_gain": old_slip,
            "actual_slip_self_gain": float(slip[1]),
            "actual_xi_self_gain": float(xi[0]),
            "checks": authority_checks,
        },
        "realization_gate": "not entered after mandatory AUTH failure" if not authority_pass else "entered",
        "geometry_reconstruction": geometry_metrics,
        "scope_contract": config["scope_contract"],
    }
    if not finite_tree(decision):
        raise RuntimeError("non-finite decision evidence")
    P45.write_json(output / "repair-decision.json", decision)
    (output / "SUMMARY.md").write_text(
        "# Phase46 point-realizable repair\n\n"
        f"- verdict: `{decision['classification']}`\n"
        f"- harmful cross: `{slip[0]:.12g}` (reduction `{cross_reduction:.6%}`)\n"
        f"- actual slip self: `{slip[1]:.12g}`\n"
        f"- mandatory stop: `{decision['mandatory_gate_stop']}`\n",
        encoding="utf-8")
    replay_error = None
    if args.replay_of:
        replay_error = P45.semantic_error(
            args.replay_of / "repair-decision.json", output / "repair-decision.json")
    P45.write_json(output / "summary.json", {
        "pass": decision["pass"], "classification": decision["classification"],
        "replay_max_abs_error": replay_error,
        "replay_pass": replay_error is None or replay_error <= 1.0e-11,
    })
    sources = [config_path, continuation_path, source / "common-transfer-matrices.json",
               source / "summary.json", Path(__file__).resolve()]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources},
    })
    return 0 if decision["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
