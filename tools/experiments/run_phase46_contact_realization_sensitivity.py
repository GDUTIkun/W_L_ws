#!/usr/bin/env python3
"""Phase46 compatible-H0 tick0 local QP-to-plant contact sensitivity audit."""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase46_hip_common_increment_limited_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ATTR = load(ROOT / "tools/experiments/run_phase45_rework_authority_attribution.py", "p46_rc_attr")
P45C, P45, P44, P42 = ATTR.P45C, ATTR.P45, ATTR.P44, ATTR.P42
SCALES = (1.0, 0.5, 0.25)
COMPONENTS = ("Fr", "Fl", "Fn", "Mr", "Ml", "Mn")
TARGET_INDICES = np.asarray([0, 2, 6, 8])
TARGET_NAMES = ("Fr_L", "Fn_L", "Fr_R", "Fn_R")


def vec(row: dict[str, Any], prefix: str, count: int) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{index}"]) for index in range(count)])


def rel(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1.0e-12))


def point_forces(actual: dict[str, Any], geometry: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], np.ndarray]:
    records: list[dict[str, Any]] = []
    aggregate = np.zeros((2, 6))
    for side, item in enumerate(geometry):
        rows = [row for row in actual["details"] if int(row["side"]) == side]
        for point_index, row in enumerate(rows):
            force = np.asarray([row[f"world_force_{axis}"] for axis in range(3)], dtype=float)
            torque = np.asarray([row[f"world_torque_{axis}"] for axis in range(3)], dtype=float)
            point = np.asarray([row[f"position_world_{axis}"] for axis in range(3)], dtype=float)
            contact_frame = np.asarray(
                [[row[f"frame_{r}{c}"] for c in range(3)] for r in range(3)], dtype=float)
            wheel_components = item["frame"].T @ force
            contact_components = contact_frame @ force
            moment = torque + np.cross(point - item["point"], force)
            aggregate[side, :3] += wheel_components
            aggregate[side, 3:] += item["frame"].T @ moment
            records.append({
                "side": ("left", "right")[side], "point_index": point_index,
                "position_world_m": point, "wheel_force_Fr_Fl_Fn_n": wheel_components,
                "contact_normal_tangent12_force_n": contact_components,
                "world_torque_nm": torque, "distance_m": float(row["distance_m"]),
                "penetration_m": max(0.0, -float(row["distance_m"])),
                "friction_margin_n": float(row["friction_margin_diagnostic_n"]),
                "dimension": int(row["dim"]), "geom1": int(row["geom1"]),
                "geom2": int(row["geom2"]), "efc_address": int(row["efc_address"]),
            })
    return records, aggregate


def observe(base: dict[str, Any], authority: Path, trim: np.ndarray, native: dict[str, str],
            model: mujoco.MjModel, oracle: Any, geometry: list[dict[str, Any]],
            regime_limits: dict[str, float], delta: np.ndarray, path: Path,
            case_id: str) -> dict[str, Any]:
    control = P45.run(base, path, case_id, authority=authority, tick=0,
                      delta=delta, wrench_trim=trim)[0]
    control.pop("wbc_time_s", None)
    actual = P45.actual(base, model, oracle, native, control)
    qp_wrench = vec(control, "physical_solution", 30)[18:30]
    mj_wrench, _ = ATTR.actual_contact_wrench(actual, geometry)
    points, reconstructed = point_forces(actual, geometry)
    qacc = P44.vec(actual["dynamics"], "qacc", model.nv)
    return {
        "delta_command": delta, "control": control, "actual": actual,
        "qp_wrench": qp_wrench, "mj_wrench": mj_wrench.reshape(-1),
        "tau": vec(control, "tau", 6), "solution": vec(control, "physical_solution", 42),
        "points": points,
        "point_to_aggregate_closure_max_abs": float(np.max(np.abs(reconstructed - mj_wrench))),
        "contact_count": [sum(int(row["side"]) == side for row in actual["details"])
                          for side in range(2)],
        "contact_dimensions": sorted({int(row["dim"]) for row in actual["details"]}),
        "penetration_m": [float(actual["dynamics"][f"penetration_{side}_m"])
                          for side in ("left", "right")],
        "minimum_friction_margin_n": min(float(row["friction_margin_diagnostic_n"])
                                          for row in actual["details"]),
        "regime_signature": P45C.signature(control, P45.clean(actual), regime_limits),
        "actual_hip_common_acceleration_rad_s2": float(0.5 * (qacc[6] + qacc[11])),
        "whole_dynamics_closure": float(actual["dynamics"]["full_dynamics_residual_max_abs"]),
        "contact_applyft_closure": float(actual["dynamics"]["contact_applyft_jacobian_max_abs"]),
    }


def increments(item: dict[str, Any], baseline: dict[str, Any], mass: np.ndarray,
               reduction: np.ndarray, hip: tuple[int, int]) -> dict[str, Any]:
    dq = item["qp_wrench"] - baseline["qp_wrench"]
    dy = item["mj_wrench"] - baseline["mj_wrench"]
    actual = item["actual"]["dynamics"]
    zero = baseline["actual"]["dynamics"]
    qcontact = lambda row: (P44.vec(row, "qfrc_contact_left", 16) +
                            P44.vec(row, "qfrc_contact_right", 16))
    contact_force = reduction.T @ (qcontact(actual) - qcontact(zero))
    return {
        "qp_wrench_12d": dq, "u_4d": dq[TARGET_INDICES], "mj_wrench_12d": dy,
        "y_4d": dy[TARGET_INDICES], "tau_6d": item["tau"] - baseline["tau"],
        "optimization_variables_42d": item["solution"] - baseline["solution"],
        "actual_hip_common_acceleration": (item["actual_hip_common_acceleration_rad_s2"] -
                                            baseline["actual_hip_common_acceleration_rad_s2"]),
        "contact_hip_common_contribution": ATTR.reduced_hip_map(
            mass, reduction, hip, contact_force),
    }


def point_target_forces(item: dict[str, Any]) -> np.ndarray:
    result = np.zeros((2, 2, 2))
    for row in item["points"]:
        side = 0 if row["side"] == "left" else 1
        force = np.asarray(row["wheel_force_Fr_Fl_Fn_n"])
        result[side, int(row["point_index"])] = force[[0, 2]]
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    probes = output / "probes"; probes.mkdir()

    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation_path = ROOT / config["continuation_config"]
    continuation = json.loads(continuation_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    case_id = config.get("case_id", "R46I-H0")
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)

    baseline_control = P45.run(base, probes / "baseline.csv", case_id, authority=authority,
                               tick=0, delta=np.zeros(4), wrench_trim=trim)[0]
    baseline_actual = P45.actual(base, model, oracle, native, baseline_control)
    geometry = ATTR.contact_geometry(model, baseline_actual["qpos"], baseline_actual["qvel"],
                                     float(oracle.config["canonical_wheel_radius_m"]))
    baseline = observe(base, authority, trim, native, model, oracle, geometry,
                       continuation["post_reaudit"]["regime_signature"], np.zeros(4),
                       probes / "baseline-detail.csv", case_id)
    mass = P44.vec(baseline_actual["dynamics"], "mass", model.nv ** 2).reshape(model.nv, model.nv)
    reduction, reduction_metrics = ATTR.plant_constrained_reduction(
        model, baseline_actual["qpos"], baseline_actual["qvel"])
    hip = ATTR.hip_dofs(model)

    command_delta = float(config.get("delta_m_s2", 0.01))
    calibration: dict[tuple[int, float, int], dict[str, Any]] = {}
    probe_records: list[dict[str, Any]] = []
    for axis in range(4):
        for scale in SCALES:
            for sign in (-1, 1):
                delta = np.zeros(4); delta[axis] = sign * scale * command_delta
                item = observe(base, authority, trim, native, model, oracle, geometry,
                               continuation["post_reaudit"]["regime_signature"], delta,
                               probes / f"calibration-{axis}-{scale:g}-{sign:+d}.csv", case_id)
                item["increment"] = increments(item, baseline, mass, reduction, hip)
                calibration[(axis, scale, sign)] = item
                probe_records.append({"stage": "calibration", "direction": axis,
                                      "scale": scale, "branch": sign, **item})

    command_to_u = np.column_stack([
        (calibration[(axis, 1.0, 1)]["increment"]["u_4d"] -
         calibration[(axis, 1.0, -1)]["increment"]["u_4d"]) / (2.0 * command_delta)
        for axis in range(4)])
    singular = np.linalg.svd(command_to_u, compute_uv=False)
    condition = float(singular[0] / singular[-1]) if singular[-1] > 0.0 else float("inf")
    design_directions = np.linalg.solve(command_to_u, np.eye(4))
    target_delta = command_delta / np.max(np.abs(design_directions), axis=0)

    targeted: dict[tuple[int, float, int], dict[str, Any]] = {}
    for axis in range(4):
        for scale in SCALES:
            for sign in (-1, 1):
                delta = sign * scale * target_delta[axis] * design_directions[:, axis]
                item = observe(base, authority, trim, native, model, oracle, geometry,
                               continuation["post_reaudit"]["regime_signature"], delta,
                               probes / f"target-{axis}-{scale:g}-{sign:+d}.csv", case_id)
                item["increment"] = increments(item, baseline, mass, reduction, hip)
                targeted[(axis, scale, sign)] = item
                probe_records.append({"stage": "target", "direction": TARGET_NAMES[axis],
                                      "scale": scale, "branch": sign, **item})

    matrices: dict[tuple[float, int], dict[str, Any]] = {}
    for scale in SCALES:
        for sign in (-1, 1):
            u = np.column_stack([targeted[(axis, scale, sign)]["increment"]["u_4d"] /
                                 (sign * scale * target_delta[axis]) for axis in range(4)])
            y = np.column_stack([targeted[(axis, scale, sign)]["increment"]["y_4d"] /
                                 (sign * scale * target_delta[axis]) for axis in range(4)])
            matrices[(scale, sign)] = {"u_gain": u, "y_gain": y,
                                       "R_c": y @ np.linalg.inv(u),
                                       "u_condition_number": float(np.linalg.cond(u))}
    central = {scale: 0.5 * (matrices[(scale, 1)]["R_c"] + matrices[(scale, -1)]["R_c"])
               for scale in SCALES}
    reference = central[1.0]
    branch_split = max(rel(matrices[(scale, 1)]["R_c"], matrices[(scale, -1)]["R_c"])
                       for scale in SCALES)
    scale_convergence = max(rel(reference, central[scale]) for scale in (0.5, 0.25))

    purity_rows = []
    radius = float(oracle.config["canonical_wheel_radius_m"])
    for axis in range(4):
        plus = targeted[(axis, 1.0, 1)]["increment"]
        minus = targeted[(axis, 1.0, -1)]["increment"]
        gain = {key: 0.5 * (plus[key] - minus[key]) / target_delta[axis]
                for key in ("qp_wrench_12d", "u_4d", "tau_6d", "optimization_variables_42d")}
        target = abs(float(gain["u_4d"][axis]))
        off_target = np.delete(gain["u_4d"], axis)
        lateral = gain["qp_wrench_12d"][[1, 7]]
        moments = gain["qp_wrench_12d"][[3, 4, 5, 9, 10, 11]]
        purity_rows.append({
            "direction": TARGET_NAMES[axis], "target_gain": target,
            "off_target_u_norm_ratio": float(np.linalg.norm(off_target) / max(target, 1e-12)),
            "lateral_force_norm_ratio": float(np.linalg.norm(lateral) / max(target, 1e-12)),
            "moment_equivalent_norm_ratio": float(np.linalg.norm(moments) /
                                                  max(radius * target, 1e-12)),
            "torque_equivalent_norm_ratio": float(np.linalg.norm(gain["tau_6d"]) /
                                                  max(radius * target, 1e-12)),
            "qp_wrench_gain_12d": gain["qp_wrench_12d"],
            "tau_gain_6d": gain["tau_6d"],
            "optimization_gain_42d": gain["optimization_variables_42d"],
        })

    slip_plus = calibration[(2, 1.0, 1)]["increment"]
    slip_plus_r = calibration[(3, 1.0, 1)]["increment"]
    slip_minus = calibration[(2, 1.0, -1)]["increment"]
    slip_minus_r = calibration[(3, 1.0, -1)]["increment"]
    slip_u = 0.5 * ((slip_plus["u_4d"] + slip_plus_r["u_4d"]) -
                    (slip_minus["u_4d"] + slip_minus_r["u_4d"])) / command_delta
    slip_y = 0.5 * ((slip_plus["y_4d"] + slip_plus_r["y_4d"]) -
                    (slip_minus["y_4d"] + slip_minus_r["y_4d"])) / command_delta
    slip_gap = slip_y - slip_u
    slip_point_gain = 0.5 * (
        point_target_forces(calibration[(2, 1.0, 1)]) +
        point_target_forces(calibration[(3, 1.0, 1)]) -
        point_target_forces(calibration[(2, 1.0, -1)]) -
        point_target_forces(calibration[(3, 1.0, -1)])) / command_delta
    point_resultant = np.sum(slip_point_gain, axis=1)
    point_redistribution = 0.5 * (slip_point_gain[:, 0] - slip_point_gain[:, 1])
    point_resultant_closure = float(np.max(np.abs(
        point_resultant.reshape(-1) - slip_y)))
    redistribution_ratio = float(
        np.linalg.norm(point_redistribution) /
        max(np.linalg.norm(point_resultant) / np.sqrt(2.0), 1.0e-12))

    all_items = [baseline, *calibration.values(), *targeted.values()]
    signatures = {item["regime_signature"] for item in all_items}
    topology_stable = all(item["contact_count"] == [2, 2] and
                          item["contact_dimensions"] == [3] for item in all_items)
    point_closure = max(item["point_to_aggregate_closure_max_abs"] for item in all_items)
    dynamics_closure = max(max(abs(item["whole_dynamics_closure"]),
                               abs(item["contact_applyft_closure"])) for item in all_items)
    stable = branch_split <= 0.05 and scale_convergence <= 0.05 and len(signatures) == 1
    trusted = (singular[-1] >= 1.0e-6 and condition <= 1.0e6 and topology_stable and
               point_closure <= 1.0e-10 and dynamics_closure <= 1.0e-8)
    offdiag = reference - np.diag(np.diag(reference))
    cross_ratio = float(np.linalg.norm(offdiag) / max(np.linalg.norm(reference), 1e-12))
    purity_coupled = any(max(row["off_target_u_norm_ratio"], row["lateral_force_norm_ratio"],
                             row["moment_equivalent_norm_ratio"],
                             row["torque_equivalent_norm_ratio"]) > 0.1 for row in purity_rows)
    strongly_coupled = cross_ratio > 0.25 or purity_coupled
    aggregate_change = float(np.linalg.norm(slip_gap)) > 0.1 * max(float(np.linalg.norm(slip_u)), 1e-12)
    if not trusted:
        classification = "U-UNTRUSTED"
    elif not stable:
        classification = "D-NONLINEAR_OR_REGIME_DEPENDENT"
    elif not aggregate_change:
        classification = "C-POINT_REDISTRIBUTION_DOMINANT"
    elif strongly_coupled:
        classification = "B-STABLE_BUT_STRONGLY_COUPLED"
    else:
        classification = "A-LOCAL_MAP_STABLE"

    records_path = output / "probe-records.json"
    P45.write_json(records_path, {"baseline": baseline, "probes": probe_records})
    result = {
        "classification": classification, "trusted": trusted, "locally_stable": stable,
        "definition": "local QP-solution-to-plant realization sensitivity",
        "u_order": TARGET_NAMES, "y_order": TARGET_NAMES, "R_c": reference,
        "branch_matrices": {f"{scale:g}_{'plus' if sign > 0 else 'minus'}": value
                            for (scale, sign), value in matrices.items()},
        "central_scale_matrices": {f"{scale:g}": value for scale, value in central.items()},
        "branch_split_relative": branch_split, "scale_convergence_relative": scale_convergence,
        "command_to_u": command_to_u, "command_to_u_singular_values": singular,
        "command_to_u_condition_number": condition, "target_command_directions": design_directions,
        "target_wrench_delta_n": target_delta,
        "probe_purity": purity_rows, "cross_realization_frobenius_ratio": cross_ratio,
        "strongly_coupled": strongly_coupled, "regime_signature_count": len(signatures),
        "topology_stable": topology_stable, "minimum_friction_margin_n": min(
            item["minimum_friction_margin_n"] for item in all_items),
        "maximum_penetration_m": max(max(item["penetration_m"]) for item in all_items),
        "point_to_aggregate_wrench_closure_max_abs": point_closure,
        "whole_dynamics_contact_closure_max_abs": dynamics_closure,
        "slip_common_u_gain": slip_u, "slip_common_y_gain": slip_y,
        "slip_common_y_minus_u": slip_gap,
        "slip_common_point_force_gain_side_point_FrFn": slip_point_gain,
        "slip_common_point_resultant_gain": point_resultant,
        "slip_common_point_redistribution_mode": point_redistribution,
        "point_resultant_closure_max_abs": point_resultant_closure,
        "point_redistribution_to_resultant_mode_ratio": redistribution_ratio,
        "right_wheel_Fr_gap_is_largest": int(np.argmax(np.abs(slip_gap))) == 2,
        "aggregate_resultant_change": aggregate_change,
        "point_redistribution_only": not aggregate_change,
        "realization_aware_correction_qualified": bool(
            trusted and stable and not strongly_coupled and len(signatures) == 1),
        "required_next_layer": ("none before a realization-aware correction audit" if
                                trusted and stable and not strongly_coupled else
                                "continue compliant multi-contact / solver-reaction mechanism attribution"),
        "plant_reduction_metrics": reduction_metrics,
        "scope": "compatible-H0 tick0 fixed-state only; no repair or trajectory",
    }
    P45.write_json(output / "contact-realization-sensitivity.json", result)
    compared = ["contact-realization-sensitivity.json", "probe-records.json"]
    replay_error = max(P45.semantic_error(args.replay_of / name, output / name)
                       for name in compared) if args.replay_of else None
    replay_pass = replay_error is None or replay_error <= float(base["gates"]["semantic_replay_max_abs"])
    P45.write_json(output / "summary.json", {
        "pass": trusted and replay_pass, "classification": classification,
        "replay_max_abs_error": replay_error, "trusted": trusted,
        "locally_stable": stable, "scope": result["scope"],
    })
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"],
               authority, wrench_source, Path(__file__).resolve(),
               ROOT / "tools/experiments/run_phase45_rework_authority_attribution.py"]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources},
    })
    return 0 if trusted and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
