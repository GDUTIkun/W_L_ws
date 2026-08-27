#!/usr/bin/env python3
"""Layered independent feasibility audit for the frozen Phase-21 42D hard QP."""
from __future__ import annotations

import argparse
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_weighted_wbc_hard_qp_42d import (  # noqa: E402
    INF, NVAR, ROOT, HardQpBuilder, corpus, independent_oracle, load_config,
    sha256, write_json,
)


DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_hard_layers_42d.json"
DEFAULT_OUTPUT = ROOT / "data/experiments/2026-08-27-phase21-hard-layers-42d-v1"


def scalar(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: scalar(item) for key, item in value.items()}
    if isinstance(value, list):
        return [scalar(item) for item in value]
    return value


def layer_metrics(builder: HardQpBuilder, problem: dict[str, Any], x: np.ndarray,
                  row_stop: int, active_tolerance: float) -> dict[str, Any]:
    physical = builder.transform @ x
    metric: dict[str, Any] = {
        "physical_x": physical,
        "physical_dynamics_residual": float(np.max(np.abs(
            problem["physical_dynamics"] @ physical - problem["physical_rhs"]))),
        "maximum_slack_magnitude": float(np.max(np.abs(physical[30:42]))),
    }
    if row_stop >= 18:
        torque_limit = np.asarray(builder.config["bounds"]["torque_nm"], dtype=float)
        torque_margin = torque_limit - np.abs(physical[12:18])
        metric.update({"torque_nm": physical[12:18], "torque_margin_nm": torque_margin,
                       "maximum_torque_violation": float(max(0.0, -np.min(torque_margin)))})
    if row_stop >= 92:
        cone_rows: list[dict[str, Any]] = []
        cone_margins: list[float] = []
        for side, start in (("left", 18), ("right", 24)):
            values = builder.h_cone @ physical[start:start + 6]
            margins = -values
            cone_margins.extend(margins.tolist())
            scaled_activity = problem["A"][18 + (0 if side == "left" else len(builder.h_cone)):
                                                   18 + (1 if side == "left" else 2) * len(builder.h_cone)] @ x
            cone_rows.extend({"row": f"{side}_cone_{index}", "margin": float(margin)}
                              for index, margin in enumerate(margins)
                              if abs(scaled_activity[index]) <= active_tolerance)
        metric.update({"minimum_cone_margin": float(min(cone_margins)),
                       "maximum_cone_violation": float(max(0.0, -min(cone_margins))),
                       "left_normal_force": float(physical[20]),
                       "right_normal_force": float(physical[26]),
                       "active_cone_rows": cone_rows})
    if row_stop >= 104:
        acceleration_limit = np.asarray(builder.config["bounds"]["acceleration"], dtype=float)
        acceleration_margin = acceleration_limit - np.abs(physical[:12])
        metric.update({"acceleration": physical[:12], "acceleration_margin": acceleration_margin,
                       "maximum_acceleration_violation": float(max(0.0, -np.min(acceleration_margin)))})
    return metric


def joint_protection_audit(builder: HardQpBuilder, required_count: int) -> dict[str, Any]:
    model, oracle = builder.oracle.model, builder.oracle
    joints = []
    for joint in oracle.active_joints:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(joint))
        joints.append({"name": name, "limited": bool(model.jnt_limited[joint]),
                       "range": model.jnt_range[joint].tolist()})
    no_authoritative_limits = (len(joints) == required_count and
                               all(not joint["limited"] and joint["range"] == [0.0, 0.0]
                                   for joint in joints))
    return {"active_joints": joints,
            "decision": "excluded_no_authoritative_limits" if no_authoritative_limits else "unresolved",
            "retained_protection": "frozen_12D_acceleration_box",
            "pass": no_authoritative_limits}


def audited_oracle(problem: dict[str, Any], row_stop: int,
                   settings: dict[str, Any]) -> tuple[dict[str, Any], bool]:
    """Call the frozen oracle, padding equality-only layers with a null inequality."""
    a, lower, upper = (problem["A"][:row_stop], problem["l"][:row_stop],
                       problem["u"][:row_stop])
    inequality = np.abs(lower - upper) > 1.0e-12
    has_inequality = bool(np.any((upper < INF) & inequality) or
                          np.any((lower > -INF) & inequality))
    if not has_inequality:
        # scipy.linprog requires a 2D A_ub.  0*x <= 0 preserves this layer exactly.
        a, lower, upper = (np.vstack((a, np.zeros((1, NVAR)))),
                           np.r_[lower, -INF], np.r_[upper, 0.0])
    return independent_oracle(problem["H"], problem["g"], a, lower, upper, settings), not has_inequality


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config, config_inputs = load_config(args.config.resolve())
    model, model_inputs = load_config((ROOT / config["model_profile"]).resolve())
    contact, contact_inputs = load_config((ROOT / config["contact_profile"]).resolve())
    equilibrium_path = ROOT / model["equilibrium"]
    builder = HardQpBuilder(config, model, contact,
                            json.loads(equilibrium_path.read_text(encoding="utf-8")))
    capture_path = ROOT / config["dynamic_capture"]
    capture = np.load(capture_path)
    gates_cfg, settings = config["gates"], config["oracle"]
    layers = config["layer_boundaries"]
    if [layer["row_stop"] for layer in layers] != [12, 18, 92, 104]:
        raise RuntimeError("Layer boundaries must preserve the frozen 42D row order")

    serial_cases = []
    all_audits: list[dict[str, Any]] = []
    all_metrics: list[dict[str, Any]] = []
    equilibrium_problem: dict[str, Any] | None = None
    for case_id, qpos, velocity in corpus(builder, capture):
        problem = builder.build(qpos, velocity)
        case_layers = []
        for layer in layers:
            stop = int(layer["row_stop"])
            audit, synthetic_null_inequality = audited_oracle(problem, stop, settings)
            metrics = layer_metrics(builder, problem, audit["x"], stop,
                                    gates_cfg["active_cone_tolerance"]) if audit.get("qp_success") else {}
            case_layers.append({"name": layer["name"], "row_count": stop,
                                "oracle_synthetic_null_inequality": synthetic_null_inequality,
                                "audit": {key: value for key, value in audit.items() if key != "x"},
                                "metrics": metrics})
            all_audits.append(audit)
            all_metrics.append(metrics)
        serial_cases.append({"id": case_id,
                             "kind": "workspace" if case_id.startswith("workspace_") else "dynamic",
                             "layers": case_layers})
        if case_id == "workspace_equilibrium":
            equilibrium_problem = problem

    if equilibrium_problem is None:
        raise RuntimeError("Missing workspace_equilibrium from frozen corpus")
    zero_acceleration = np.zeros((12, NVAR)); zero_acceleration[:, :12] = np.eye(12)
    static_a = np.vstack((equilibrium_problem["A"], zero_acceleration))
    static_l = np.r_[equilibrium_problem["l"], np.zeros(12)]
    static_u = np.r_[equilibrium_problem["u"], np.zeros(12)]
    static_audit = independent_oracle(equilibrium_problem["H"], equilibrium_problem["g"],
                                      static_a, static_l, static_u, settings)
    static_metrics = layer_metrics(builder, equilibrium_problem, static_audit["x"], 104,
                                   gates_cfg["active_cone_tolerance"]) if static_audit.get("qp_success") else {}
    joint_audit = joint_protection_audit(builder, gates_cfg["required_active_joint_count"])

    all_audits.append(static_audit)
    if static_metrics:
        all_metrics.append(static_metrics)

    finite_metrics = [metric for metric in all_metrics if metric]
    gates = {
        "layer_dimensions": all(layer["row_stop"] <= 104 for layer in layers),
        "all_layers_oracle_feasible": all(audit.get("feasible") and audit.get("qp_success") for audit in all_audits),
        "oracle_bounds": max(audit.get("qp_bound_violation", INF) for audit in all_audits)
                         <= gates_cfg["maximum_oracle_bound_violation"],
        "oracle_stationarity": max(audit.get("qp_stationarity_residual", INF) for audit in all_audits)
                               <= gates_cfg["maximum_oracle_stationarity_residual"],
        "oracle_complementarity": max(audit.get("qp_complementarity_residual", INF) for audit in all_audits)
                                  <= gates_cfg["maximum_oracle_complementarity_residual"],
        "physical_dynamics": max(metric["physical_dynamics_residual"] for metric in finite_metrics)
                             <= gates_cfg["maximum_physical_dynamics_residual"],
        "torque_limits": max(metric.get("maximum_torque_violation", 0.0) for metric in finite_metrics)
                         <= gates_cfg["maximum_physical_torque_violation"],
        "cone_limits": max(metric.get("maximum_cone_violation", 0.0) for metric in finite_metrics)
                       <= gates_cfg["maximum_physical_cone_violation"],
        "acceleration_limits": max(metric.get("maximum_acceleration_violation", 0.0) for metric in finite_metrics)
                               <= gates_cfg["maximum_physical_acceleration_violation"],
        "slack_zero": max(metric["maximum_slack_magnitude"] for metric in finite_metrics)
                      <= gates_cfg["maximum_slack_magnitude"],
        "static_equilibrium_dimensions": static_a.shape[0] == gates_cfg["required_static_equilibrium_rows"],
        "static_equilibrium_feasible": bool(static_audit.get("feasible") and static_audit.get("qp_success")),
        "static_equilibrium_margins": bool(static_metrics) and min(
            static_metrics["minimum_cone_margin"], np.min(static_metrics["torque_margin_nm"]),
            np.min(static_metrics["acceleration_margin"])) >= gates_cfg["minimum_static_equilibrium_margin"],
        "joint_protection_decision": joint_audit["pass"],
    }
    summary = {
        "schema_version": 1, "phase": 21, "profile": config["profile"],
        "scope": "P21-T05 layered hard-feasibility and frozen joint-protection decision",
        "variable_order": ["nudot_12", "tau_6", "wrench_left_C_6", "wrench_right_C_6",
                           "slack_left_controller_FLU_6", "slack_right_controller_FLU_6"],
        "layers": layers, "case_count": len(serial_cases), "row_map": builder.row_map,
        "static_equilibrium": {"row_count": int(static_a.shape[0]),
                                 "audit": {key: value for key, value in static_audit.items() if key != "x"},
                                 "metrics": static_metrics},
        "max_metrics": {
            "oracle_bound_violation": max(audit.get("qp_bound_violation", INF) for audit in all_audits),
            "oracle_stationarity_residual": max(audit.get("qp_stationarity_residual", INF) for audit in all_audits),
            "oracle_complementarity_residual": max(audit.get("qp_complementarity_residual", INF) for audit in all_audits),
            "physical_dynamics_residual": max(metric["physical_dynamics_residual"] for metric in finite_metrics),
            "physical_torque_violation": max(metric.get("maximum_torque_violation", 0.0) for metric in finite_metrics),
            "physical_cone_violation": max(metric.get("maximum_cone_violation", 0.0) for metric in finite_metrics),
            "physical_acceleration_violation": max(metric.get("maximum_acceleration_violation", 0.0) for metric in finite_metrics),
            "slack_magnitude": max(metric["maximum_slack_magnitude"] for metric in finite_metrics),
        },
        "joint_protection": joint_audit, "gates": gates, "pass": all(gates.values()),
    }
    write_json(output / "cases.json", scalar(serial_cases))
    write_json(output / "summary.json", scalar(summary))
    script = Path(__file__).resolve()
    inputs = config_inputs + model_inputs + contact_inputs
    write_json(output / "manifest.json", {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
        "mujoco": mujoco.__version__,
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
        "equilibrium": {str(equilibrium_path.relative_to(ROOT)): sha256(equilibrium_path)},
        "dynamic_capture": {str(capture_path.relative_to(ROOT)): sha256(capture_path)},
        "validator": str(script.relative_to(ROOT)), "validator_sha256": sha256(script),
        "outputs": {name: sha256(output / name) for name in ("summary.json", "cases.json")},
    })
    print(json.dumps({"cases": len(serial_cases), "layers": len(layers), "gates": gates,
                      "pass": summary["pass"]}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
