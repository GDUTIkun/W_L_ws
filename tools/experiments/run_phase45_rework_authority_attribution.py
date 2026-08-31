#!/usr/bin/env python3
"""Phase45 fixed-state xi/slip common-channel authority attribution only."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase45_rework_authority_attribution_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P45C = load(ROOT / "tools/experiments/run_phase45_h0_continuation.py", "p45_attr_cont")
P45 = P45C.P45
P44, P42, EQ = P45C.P44, P45C.P42, P45C.EQ
P21 = load(ROOT / "tools/experiments/validate_mujoco_weighted_wbc_model.py", "p45_attr_p21")


def vector(row: dict[str, Any], prefix: str, count: int) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{index}"]) for index in range(count)])


def common(values: np.ndarray) -> float:
    return float(0.5 * (values[0] + values[1]))


def flatten(prefix: str, values: np.ndarray) -> dict[str, float]:
    return {f"{prefix}{index}": float(value) for index, value in enumerate(values)}


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1e-12))


def force_terms(actual: dict[str, Any], reduction: np.ndarray) -> dict[str, np.ndarray]:
    q = lambda name: P44.vec(actual["dynamics"], name, reduction.shape[0])
    contact = q("qfrc_contact_left") + q("qfrc_contact_right")
    actuator = q("qfrc_actuator")
    remaining = q("qfrc_passive") + q("qfrc_applied") + q("qfrc_other_constraint")
    mass = P44.vec(actual["dynamics"], "mass", reduction.shape[0] ** 2).reshape(reduction.shape[0], -1)
    lhs = mass @ q("qacc") + q("qfrc_bias")
    return {name: reduction.T @ values for name, values in
            (("contact", contact), ("actuator", actuator), ("remaining", remaining), ("lhs", lhs))}


def full_force_terms(actual: dict[str, Any], reduction: np.ndarray) -> dict[str, np.ndarray]:
    q = lambda name: P44.vec(actual["dynamics"], name, reduction.shape[0])
    contact = q("qfrc_contact_left") + q("qfrc_contact_right")
    actuator = q("qfrc_actuator")
    remaining = q("qfrc_passive") + q("qfrc_applied") + q("qfrc_other_constraint")
    mass = P44.vec(actual["dynamics"], "mass", reduction.shape[0] ** 2).reshape(
        reduction.shape[0], reduction.shape[0])
    lhs = mass @ q("qacc") + q("qfrc_bias")
    return {"contact": contact, "actuator": actuator, "remaining": remaining, "lhs": lhs}


def hip_dofs(model: mujoco.MjModel) -> tuple[int, int]:
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
           for name in ("left_hip_joint", "right_hip_joint")]
    if min(ids) < 0:
        raise RuntimeError("missing bilateral hip joints")
    return tuple(int(model.jnt_dofadr[index]) for index in ids)


def constrained_hip_map(mass: np.ndarray, dofs: tuple[int, int], force: np.ndarray) -> float:
    return common(np.linalg.solve(mass, force)[list(dofs)])


def reduced_hip_map(mass: np.ndarray, reduction: np.ndarray,
                    dofs: tuple[int, int], force: np.ndarray) -> float:
    acceleration = reduction @ np.linalg.solve(reduction.T @ mass @ reduction, force)
    return common(acceleration[list(dofs)])


def plant_constrained_reduction(model: mujoco.MjModel, qpos: np.ndarray,
                                qvel: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0
    mujoco.mj_forward(model, data)
    base_site = P42.required_id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
    base_linear = np.zeros((3, model.nv)); base_angular = np.zeros_like(base_linear)
    mujoco.mj_jacSite(model, data, base_linear, base_angular, base_site)
    base_twist = np.vstack((base_linear, base_angular))[:, :6]
    reduction = np.zeros((model.nv, 12))
    reduction[:6, :6] = np.linalg.solve(base_twist, np.eye(6))
    active_names = ("left_hip_joint", "left_knee_joint", "left_wheel_joint",
                    "right_hip_joint", "right_knee_joint", "right_wheel_joint")
    passive_names = ("left_connect1_joint", "left_connect2_joint",
                     "right_connect1_joint", "right_connect2_joint")
    active = [int(model.jnt_dofadr[P42.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
              for name in active_names]
    passive = [int(model.jnt_dofadr[P42.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)])
               for name in passive_names]
    reduction[active, 6:] = -np.eye(6)
    closure_rows = []
    for first_name, second_name in (("left_connect2_site", "left_calf_site"),
                                    ("right_connect2_site", "right_calf_site")):
        first = P42.required_id(model, mujoco.mjtObj.mjOBJ_SITE, first_name)
        second = P42.required_id(model, mujoco.mjtObj.mjOBJ_SITE, second_name)
        first_jac = np.zeros((3, model.nv)); second_jac = np.zeros_like(first_jac)
        scratch = np.zeros_like(first_jac)
        mujoco.mj_jacSite(model, data, first_jac, scratch, first)
        mujoco.mj_jacSite(model, data, second_jac, scratch, second)
        closure_rows.append(first_jac - second_jac)
    closure = np.vstack(closure_rows)
    reduction[passive, :] = np.linalg.lstsq(
        closure[:, passive], -(closure @ reduction), rcond=1.0e-12)[0]
    return reduction, {
        "constraint_tangent_max_abs": float(np.max(np.abs(closure @ reduction))),
        "base_twist_mapping_max_abs": float(np.max(np.abs(
            base_twist @ reduction[:6, :6] - np.eye(6)))),
    }


def contact_geometry(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray,
                     radius: float) -> list[dict[str, Any]]:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0
    mujoco.mj_forward(model, data)
    result = []
    for side, name in enumerate(("left", "right")):
        geom = P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{name}_wheel_collision")
        body = P42.required_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{name}_wheel_body")
        mesh = int(model.geom_dataid[geom])
        start = int(model.mesh_vertadr[mesh])
        vertices = model.mesh_vert[start:start + int(model.mesh_vertnum[mesh])]
        midpoint = 0.5 * (float(vertices[:, 0].min()) + float(vertices[:, 0].max()))
        rotation = data.ximat[body].reshape(3, 3).copy()
        axis = rotation[:, 0]
        normal = np.array([0.0, 0.0, 1.0])
        dot = float(axis @ normal)
        projection = float(np.sqrt(max(0.0, 1.0 - dot * dot)))
        rolling = np.cross(axis, normal) / projection
        lateral = np.cross(normal, rolling)
        radial = (normal - dot * axis) / projection
        point = data.xipos[body].copy() + midpoint * axis - radius * radial
        frame = np.column_stack((rolling, lateral, normal))
        linear = np.zeros((3, model.nv))
        angular = np.zeros_like(linear)
        mujoco.mj_jac(model, data, linear, angular, point, body)
        result.append({"side": side, "geom": geom, "body": body, "point": point,
                       "frame": frame, "lever": point - data.xpos[body],
                       "linear_jacobian": linear, "angular_jacobian": angular})
    return result


def model_b_contact_geometry(control: dict[str, Any]) -> tuple[list[dict[str, Any]], list[np.ndarray], dict[str, Any]]:
    config_path = ROOT / "simulation/mujoco/config/phase21_model_oracle_v8.json"
    config, _ = P21.load_config(config_path)
    equilibrium = json.loads((ROOT / config["equilibrium"]).read_text(encoding="utf-8"))
    oracle = P21.Oracle(config, equilibrium)
    qpos = oracle.model.qpos0.copy()
    base_quaternion = vector(control, "base_q", 4)
    rotation_flat = np.empty(9)
    mujoco.mju_quat2Mat(rotation_flat, base_quaternion)
    base_rotation = rotation_flat.reshape(3, 3)
    base_control_position = vector(control, "base_p", 3)
    base_control_local = oracle.model.site_pos[oracle.base_control_site]
    qpos[:3] = base_control_position - base_rotation @ base_control_local
    qpos[3:7] = base_quaternion
    canonical = np.asarray([float(control[f"q{index}"]) for index in range(6)])
    qpos[oracle.active_qpos] = np.asarray(config["canonical_joint_offsets_rad"]) - canonical
    qpos[oracle.passive_qpos] = oracle.equilibrium_passive
    qpos, closure = oracle.solve_passive(qpos)
    reduction, reduction_metrics = oracle.reduction(qpos)
    profile = json.loads((ROOT / "simulation/mujoco/config/phase21_runtime_model_profile_v1.json").read_text(
        encoding="utf-8"))
    geometry = contact_geometry(oracle.model, qpos, np.zeros(oracle.model.nv),
                                float(profile["contact"]["radius_m"]))
    maps = [np.hstack(((item["linear_jacobian"] @ reduction).T @ item["frame"],
                       (item["angular_jacobian"] @ reduction).T @ item["frame"]))
            for item in geometry]
    logged = [P44.matrix(control, f"contact_map_{side}_", 12, 6) for side in range(2)]
    metrics = {
        "source": str(config_path.relative_to(ROOT)),
        "closure_residual_m": closure["closure_residual_m"],
        "constraint_tangent_max_abs": reduction_metrics["constraint_tangent"],
        "logged_map_reconstruction_max_abs": max(
            float(np.max(np.abs(maps[side] - logged[side]))) for side in range(2)),
    }
    return geometry, maps, metrics


def actual_contact_wrench(actual: dict[str, Any], geometry: list[dict[str, Any]]) -> tuple[np.ndarray, list[dict[str, Any]]]:
    wrenches = np.zeros((2, 6))
    parity: list[dict[str, Any]] = []
    for side, item in enumerate(geometry):
        rows = [row for row in actual["details"] if int(row["side"]) == side]
        force = np.zeros(3)
        moment = np.zeros(3)
        weighted_point = np.zeros(3)
        load = 0.0
        normal_errors = []
        for row in rows:
            world_force = np.asarray([row[f"world_force_{axis}"] for axis in range(3)], dtype=float)
            world_torque = np.asarray([row[f"world_torque_{axis}"] for axis in range(3)], dtype=float)
            point = np.asarray([row[f"position_world_{axis}"] for axis in range(3)], dtype=float)
            frame = np.asarray([[row[f"frame_{r}{c}"] for c in range(3)] for r in range(3)], dtype=float)
            normal = frame[0] if int(row["geom2"]) == item["geom"] else -frame[0]
            normal_errors.append(float(np.linalg.norm(normal - item["frame"][:, 2])))
            force += world_force
            moment += world_torque + np.cross(point - item["point"], world_force)
            weight = max(0.0, float(world_force @ item["frame"][:, 2]))
            weighted_point += weight * point
            load += weight
        wrenches[side, :3] = item["frame"].T @ force
        wrenches[side, 3:] = item["frame"].T @ moment
        cop = weighted_point / load if load > 0.0 else np.full(3, np.nan)
        parity.append({"side": side, "contact_count": len(rows),
                       "cop_minus_qp_center": cop - item["point"],
                       "lever_arm_difference": cop - item["point"],
                       "normal_max_vector_error": max(normal_errors, default=np.nan)})
    return wrenches, parity


def contact_realization_sample(sample: dict[str, Any], geometry: list[dict[str, Any]],
                               reduction: np.ndarray) -> dict[str, Any]:
    control, actual = sample["control"], sample["actual"]
    qp_wrench = P44.vec(control, "physical_solution", 30)[18:30].reshape(2, 6)
    mj_wrench, parity = actual_contact_wrench(actual, geometry)
    qp_maps = [P44.matrix(control, f"contact_map_{side}_", 12, 6) for side in range(2)]
    mj_maps = [np.hstack(((item["linear_jacobian"] @ reduction).T @ item["frame"],
                          (item["angular_jacobian"] @ reduction).T @ item["frame"]))
               for item in geometry]
    qp_force = sum((qp_maps[side] @ qp_wrench[side] for side in range(2)), np.zeros(12))
    mj_same_force = sum((mj_maps[side] @ qp_wrench[side] for side in range(2)), np.zeros(12))
    mj_force = sum((mj_maps[side] @ mj_wrench[side] for side in range(2)), np.zeros(12))
    actual_force = reduction.T @ (P44.vec(actual["dynamics"], "qfrc_contact_left", 16) +
                                  P44.vec(actual["dynamics"], "qfrc_contact_right", 16))
    return {"qp_wrench": qp_wrench, "mj_wrench": mj_wrench,
            "qp_maps": qp_maps, "mj_maps": mj_maps,
            "qp_force": qp_force, "mj_same_force": mj_same_force,
            "mj_force": mj_force, "actual_force": actual_force, "geometry_parity": parity}


def contact_realization_gain(probe: dict[str, Any], baseline: dict[str, Any],
                             denominator: float, mass: np.ndarray, reduction: np.ndarray,
                             hip: tuple[int, int]) -> dict[str, Any]:
    first, zero = probe["contact_realization"], baseline["contact_realization"]
    dqp = (first["qp_wrench"] - zero["qp_wrench"]) / denominator
    dmj = (first["mj_wrench"] - zero["mj_wrench"]) / denominator
    mapping_force = np.zeros(12)
    wrench_force = np.zeros(12)
    component = np.zeros((2, 6))
    side_mapping = np.zeros(2)
    side_wrench = np.zeros(2)
    qp_same = np.zeros(12)
    mj_same = np.zeros(12)
    mj_actual = np.zeros(12)
    for side in range(2):
        bqp, bmj = zero["qp_maps"][side], zero["mj_maps"][side]
        map_side = (bmj - bqp) @ dqp[side]
        wrench_side_force = bmj @ (dmj[side] - dqp[side])
        mapping_force += map_side
        wrench_force += wrench_side_force
        qp_same += bqp @ dqp[side]
        mj_same += bmj @ dqp[side]
        mj_actual += bmj @ dmj[side]
        side_mapping[side] = reduced_hip_map(mass, reduction, hip, map_side)
        side_wrench[side] = reduced_hip_map(mass, reduction, hip, wrench_side_force)
        for axis in range(6):
            component[side, axis] = reduced_hip_map(
                mass, reduction, hip, bmj[:, axis] * (dmj[side, axis] - dqp[side, axis]))
    gap = mj_actual - qp_same
    reconstruction = ((first["mj_force"] - zero["mj_force"]) -
                      (first["actual_force"] - zero["actual_force"])) / denominator
    return {"qp_same": reduced_hip_map(mass, reduction, hip, qp_same),
            "mj_same": reduced_hip_map(mass, reduction, hip, mj_same),
            "mj_actual": reduced_hip_map(mass, reduction, hip, mj_actual),
            "mapping": reduced_hip_map(mass, reduction, hip, mapping_force),
            "wrench": reduced_hip_map(mass, reduction, hip, wrench_force),
            "gap": reduced_hip_map(mass, reduction, hip, gap),
            "closure": reduced_hip_map(mass, reduction, hip, gap - mapping_force - wrench_force),
            "generalized_force_closure_max_abs": float(np.max(np.abs(
                gap - mapping_force - wrench_force))),
            "actual_contact_reconstruction_max_abs": float(np.max(np.abs(reconstruction))),
            "side_mapping": side_mapping, "side_wrench": side_wrench,
            "wrench_component": component, "dqp_wrench": dqp, "dmj_wrench": dmj}


def leg_dofs(model: mujoco.MjModel, wheel_dofs: list[int]) -> list[tuple[int, str]]:
    excluded = {*range(6), *wheel_dofs}
    result: list[tuple[int, str]] = []
    for dof in range(model.nv):
        if dof in excluded:
            continue
        joint = int(model.dof_jntid[dof])
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if not name:
            raise RuntimeError(f"leg/non-wheel dof {dof} has no joint name")
        result.append((dof, name))
    return result


def leg_dof_gain(probe: dict[str, Any], baseline: dict[str, Any], denominator: float,
                 dofs: list[tuple[int, str]], qacc_key: str) -> dict[str, float]:
    return {
        name: common(probe["xi_map"][:, dof] * probe[qacc_key][dof] -
                     baseline["xi_map"][:, dof] * baseline[qacc_key][dof]) / denominator
        for dof, name in dofs
    }


def leg_modes(map_: np.ndarray, qacc_gain: dict[str, float], dofs: list[tuple[int, str]]) -> list[dict[str, float | str]]:
    by_name = {name: dof for dof, name in dofs}
    modes: list[dict[str, float | str]] = []
    for name in sorted(by_name):
        if not name.startswith("left_"):
            continue
        right_name = "right_" + name.removeprefix("left_")
        if right_name not in by_name:
            raise RuntimeError(f"missing right counterpart for {name}")
        left, right = by_name[name], by_name[right_name]
        sensitivity_left = common(map_[:, left])
        sensitivity_right = common(map_[:, right])
        acceleration_common = 0.5 * (qacc_gain[name] + qacc_gain[right_name])
        acceleration_differential = 0.5 * (qacc_gain[right_name] - qacc_gain[name])
        modes.append({
            "joint_family": name.removeprefix("left_").removesuffix("_joint"),
            "common_qacc_gain_rad_s2_per_m_s2": acceleration_common,
            "differential_qacc_gain_rad_s2_per_m_s2": acceleration_differential,
            "common_mode_ddxi_contribution": (sensitivity_left + sensitivity_right) * acceleration_common,
            "differential_mode_ddxi_contribution": (-sensitivity_left + sensitivity_right) * acceleration_differential,
        })
    return modes


def classify_qp_plant_hip_modes(summaries: dict[str, dict[str, Any]]) -> dict[str, Any]:
    def common(summary: dict[str, Any], family: str) -> float:
        return float(next(row["common_mode_ddxi_contribution"] for row in summary["modes"]
                          if row["joint_family"] == family))

    qp_hip, qp_knee = common(summaries["qp"], "hip"), common(summaries["qp"], "knee")
    mj_hip, mj_knee = common(summaries["mujoco"], "hip"), common(summaries["mujoco"], "knee")
    material = 0.05  # Existing DG45-AUTH minimum authority magnitude.
    qp_ratio = abs(qp_hip + qp_knee) / max(abs(qp_hip), 1e-12)
    mj_ratio = abs(mj_hip + mj_knee) / max(abs(mj_hip), 1e-12)
    cancellation = abs(qp_hip) >= material and qp_hip * qp_knee < 0.0 and qp_ratio <= 0.05
    broken = cancellation and mj_hip * mj_knee > 0.0 and mj_ratio >= 0.5
    created = abs(qp_hip) < material and abs(mj_hip) >= material
    qp_dominant = abs(qp_hip) >= material and qp_hip * mj_hip > 0.0 and not cancellation
    flags = {"A-QP_HIP_MODE_DOMINANT": qp_dominant,
             "B-QP_CANCELLATION_BROKEN_IN_PLANT": broken,
             "C-HIP_MODE_CREATED_BY_PLANT_REALIZATION": created}
    active = [name for name, value in flags.items() if value]
    classification = "E-MULTIPLE" if len(active) > 1 else active[0] if active else "U-UNTRUSTED"
    return {"classification": classification, "flags": flags, "material_threshold": material,
            "qp": {"hip_common": qp_hip, "knee_common": qp_knee,
                   "common_leg_cancellation_ratio": qp_ratio},
            "mujoco": {"hip_common": mj_hip, "knee_common": mj_knee,
                       "common_leg_cancellation_ratio": mj_ratio}}


def sample(base: dict[str, Any], authority: Path, trim: np.ndarray, native: dict[str, str],
           model: mujoco.MjModel, oracle: Any, reduction0: np.ndarray, delta: np.ndarray,
           path: Path, case_id: str = "R45-H0") -> dict[str, Any]:
    control = P45.run(base, path, case_id, authority=authority, tick=0,
                      delta=delta, wrench_trim=trim)[0]
    actual = P45.actual(base, model, oracle, native, control)
    material = actual["material"]
    qp, mj = P45C.task_output(control, actual)
    qacc = P44.vec(actual["dynamics"], "qacc", model.nv)
    reduction = P44.matrix(control, "reduction_", model.nv, 12)
    qacc_qp = reduction @ P44.vec(control, "physical_solution", 12) + P44.vec(control, "reduction_bias", model.nv)
    ddxi = np.asarray([actual["dynamics"]["ddxi_left_m_s2"], actual["dynamics"]["ddxi_right_m_s2"]])
    xi_map, xi_bias = P44.native_xi_acceleration_map(
        oracle, actual["qpos"], actual["qvel"], float(native["time_s"]), qacc, ddxi)
    qp_parts = EQ.decompose_xi(xi_map, xi_bias, qacc_qp, oracle.wheel_dadr)
    mj_parts = EQ.decompose_xi(xi_map, xi_bias, qacc, oracle.wheel_dadr)
    lambdas = P44.vec(control, "physical_solution", 30)[18:30]
    maps = [P44.matrix(control, f"contact_map_{side}_", 12, 6) for side in range(2)]
    qp_contact = maps[0] @ lambdas[:6] + maps[1] @ lambdas[6:]
    return {
        "control": control, "actual": actual, "qp_y": np.asarray([common(qp[:2]), common(qp[2:])]),
        "mj_y": np.asarray([common(mj[:2]), common(mj[2:])]),
        "forces": {"qp_contact": qp_contact, "qp_contact_full": reduction0 @ qp_contact,
                   **force_terms(actual, reduction0), **full_force_terms(actual, reduction0)},
        "qacc_qp": qacc_qp, "qacc_mj": qacc, "xi_map": xi_map,
        "xi_qp": qp_parts, "xi_mj": mj_parts,
        "reduction_delta_max_abs": float(np.max(np.abs(reduction - reduction0))),
        "native_wheel_qacc_qp": qacc_qp[oracle.wheel_dadr],
        "native_wheel_qacc_mj": qacc[oracle.wheel_dadr],
        "hard": float(control["hard"]), "slack": float(control["maximum_normalized_slack"]),
        "torque_margin": min(float(control[f"tau_margin{i}"]) for i in range(6)),
        "dynamics_closure": float(actual["dynamics"]["full_dynamics_residual_max_abs"]),
        "contact_closure": float(actual["dynamics"]["contact_applyft_jacobian_max_abs"]),
    }


def branch_gain(probe: dict[str, Any], baseline: dict[str, Any], denominator: float,
                dofs: list[tuple[int, str]], mass: np.ndarray,
                hip: tuple[int, int]) -> dict[str, Any]:
    gains = {"qp_y": (probe["qp_y"] - baseline["qp_y"]) / denominator,
             "mj_y": (probe["mj_y"] - baseline["mj_y"]) / denominator}
    for name in ("qp_contact", "contact", "actuator", "remaining", "lhs"):
        gains[name] = (probe["forces"][name] - baseline["forces"][name]) / denominator
    gains["native_wheel_qacc_qp"] = (probe["native_wheel_qacc_qp"] - baseline["native_wheel_qacc_qp"]) / denominator
    gains["native_wheel_qacc_mj"] = (probe["native_wheel_qacc_mj"] - baseline["native_wheel_qacc_mj"]) / denominator
    for model_name in ("qp", "mj"):
        for part in ("base", "leg_nonwheel", "wheel", "jdot_v"):
            gains[f"xi_{model_name}_{part}"] = common(np.asarray([
                probe[f"xi_{model_name}"][side][part] - baseline[f"xi_{model_name}"][side][part]
                for side in range(2)])) / denominator
    gains["xi_qp_leg_dof"] = leg_dof_gain(probe, baseline, denominator, dofs, "qacc_qp")
    gains["xi_mj_leg_dof"] = leg_dof_gain(probe, baseline, denominator, dofs, "qacc_mj")
    full = {name: (probe["forces"][name] - baseline["forces"][name]) / denominator
            for name in ("qp_contact_full", "contact", "actuator", "remaining", "lhs")}
    contributions = {name: constrained_hip_map(mass, hip, values)
                     for name, values in full.items()}
    contributions["contact_realization_difference"] = (
        contributions["contact"] - contributions["qp_contact_full"])
    contributions["actual_sum"] = (contributions["actuator"] + contributions["contact"] +
                                   contributions["remaining"])
    contributions["actual_hip_common"] = common(probe["qacc_mj"][list(hip)] -
                                                 baseline["qacc_mj"][list(hip)]) / denominator
    contributions["qp_hip_common"] = common(probe["qacc_qp"][list(hip)] -
                                             baseline["qacc_qp"][list(hip)]) / denominator
    contributions["force_balance_max_abs"] = float(np.max(np.abs(
        full["lhs"] - full["actuator"] - full["contact"] - full["remaining"])))
    contributions["contribution_closure"] = (
        contributions["actual_sum"] - contributions["actual_hip_common"])
    contributions["lhs_mapping_closure"] = (
        contributions["lhs"] - contributions["actual_hip_common"])
    gains["constrained_hip"] = contributions
    return gains


def redirection(gain: dict[str, Any], cfg: dict[str, Any]) -> dict[str, Any]:
    contact, actuator, remaining = gain["contact"], gain["actuator"], gain["remaining"]
    cn, an, rn = map(float, (np.linalg.norm(contact), np.linalg.norm(actuator), np.linalg.norm(remaining)))
    cosine = float(np.dot(contact, actuator) / max(cn * an, 1e-12))
    share = cn / max(cn + an + rn, 1e-12)
    return {"contact_to_actuator_norm_ratio": cn / max(an, 1e-12),
            "contact_actuator_cosine": cosine, "contact_force_share": share,
            "quantitatively_dominant": (cn / max(an, 1e-12) >= cfg["minimum_contact_to_actuator_norm_ratio"] and
                                         cosine <= cfg["maximum_contact_actuator_cosine"] and
                                         share >= cfg["minimum_contact_force_share"])}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args(); output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config_path = args.config.resolve(); config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation_path = ROOT / config["continuation_config"]
    continuation = json.loads(continuation_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    if "runtime_executable" in config:
        base["executable"] = config["runtime_executable"]
    case_id = config.get("case_id", "R45-H0")
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    baseline_control = P45.run(base, probes / "baseline.csv", case_id, authority=authority,
                               tick=0, wrench_trim=trim)[0]
    reduction0 = P44.matrix(baseline_control, "reduction_", model.nv, 12)
    baseline = sample(base, authority, trim, native, model, oracle, reduction0,
                      np.zeros(4), probes / "baseline-detail.csv", case_id)
    model_leg_dofs = leg_dofs(model, oracle.wheel_dadr)
    model_hip_dofs = hip_dofs(model)
    frozen_mass = P44.vec(baseline["actual"]["dynamics"], "mass", model.nv ** 2).reshape(model.nv, model.nv)
    plant_reduction, plant_reduction_metrics = plant_constrained_reduction(
        model, baseline["actual"]["qpos"], baseline["actual"]["qvel"])
    frozen_geometry = contact_geometry(model, baseline["actual"]["qpos"], baseline["actual"]["qvel"],
                                       float(oracle.config["canonical_wheel_radius_m"]))
    model_b_geometry, model_b_maps, model_b_metrics = model_b_contact_geometry(baseline_control)
    baseline["contact_realization"] = contact_realization_sample(
        baseline, frozen_geometry, plant_reduction)
    delta = float(config["delta_m_s2"])
    data: dict[tuple[str, float, int], dict[str, Any]] = {}
    probe_rows: list[dict[str, Any]] = []
    for channel, direction in config["input_channels"].items():
        vector_direction = np.asarray(direction, dtype=float)
        for scale in map(float, config["delta_scales"]):
            for sign in (-1, 1):
                values = sign * scale * delta * vector_direction
                item = sample(base, authority, trim, native, model, oracle, reduction0, values,
                              probes / f"{channel}-{scale:g}-{sign:+d}.csv", case_id)
                item["contact_realization"] = contact_realization_sample(
                    item, frozen_geometry, plant_reduction)
                data[(channel, scale, sign)] = item
                probe_rows.append({"channel": channel, "scale": scale, "sign": sign,
                    "delta_ddxi_common_m_s2": values[0], "delta_a_t_common_m_s2": values[2],
                    "qp_ddxi_common_m_s2": item["qp_y"][0], "qp_a_t_common_m_s2": item["qp_y"][1],
                    "mj_ddxi_common_m_s2": item["mj_y"][0], "mj_a_t_common_m_s2": item["mj_y"][1],
                    "native_wheel_qacc_qp_common_rad_s2": common(item["native_wheel_qacc_qp"]),
                    "native_wheel_qacc_mj_common_rad_s2": common(item["native_wheel_qacc_mj"]),
                    "xi_qp_base_common": common(np.asarray([x["base"] for x in item["xi_qp"]])),
                    "xi_qp_leg_nonwheel_common": common(np.asarray([x["leg_nonwheel"] for x in item["xi_qp"]])),
                    "xi_qp_wheel_common": common(np.asarray([x["wheel"] for x in item["xi_qp"]])),
                    "xi_qp_jdot_v_common": common(np.asarray([x["jdot_v"] for x in item["xi_qp"]])),
                    "xi_mj_base_common": common(np.asarray([x["base"] for x in item["xi_mj"]])),
                    "xi_mj_leg_nonwheel_common": common(np.asarray([x["leg_nonwheel"] for x in item["xi_mj"]])),
                    "xi_mj_wheel_common": common(np.asarray([x["wheel"] for x in item["xi_mj"]])),
                    "xi_mj_jdot_v_common": common(np.asarray([x["jdot_v"] for x in item["xi_mj"]])),
                    "hard": item["hard"], "maximum_normalized_slack": item["slack"],
                    "minimum_torque_margin_nm": item["torque_margin"], "dynamics_closure": item["dynamics_closure"],
                    "contact_closure": item["contact_closure"], "reduction_delta_max_abs": item["reduction_delta_max_abs"],
                    **flatten("qp_contact_", item["forces"]["qp_contact"]),
                    **flatten("mj_contact_", item["forces"]["contact"]),
                    **flatten("mj_actuator_", item["forces"]["actuator"]),
                    **flatten("mj_remaining_", item["forces"]["remaining"]),
                    **flatten("mj_lhs_", item["forces"]["lhs"])})

    directional: list[dict[str, Any]] = []
    leg_dof_rows: list[dict[str, Any]] = []
    matrices: dict[str, dict[str, Any]] = {}
    channel_branches: dict[str, dict[int, dict[float, dict[str, Any]]]] = {}
    all_trusted = True
    for channel in config["input_channels"]:
        branches: dict[int, dict[float, dict[str, Any]]] = {sign: {} for sign in (-1, 1)}
        for sign in (-1, 1):
            for scale in map(float, config["delta_scales"]):
                gain = branch_gain(data[(channel, scale, sign)], baseline, sign * scale * delta,
                                   model_leg_dofs, frozen_mass, model_hip_dofs)
                gain["contact_realization"] = contact_realization_gain(
                    data[(channel, scale, sign)], baseline, sign * scale * delta,
                    frozen_mass, plant_reduction, model_hip_dofs)
                branches[sign][scale] = gain
                reference = branches[sign][1.0] if scale != 1.0 else gain
                conv = max(relative(reference["qp_y"], gain["qp_y"]), relative(reference["mj_y"], gain["mj_y"]))
                trusted = conv <= float(config["maximum_directional_convergence_relative"])
                all_trusted &= trusted
                route = redirection(gain, config["contact_redirection"])
                directional.append({"channel": channel, "branch": "+" if sign > 0 else "-", "scale": scale,
                    "trusted": trusted, "convergence_relative": conv,
                    "g_qp_ddxi_common": gain["qp_y"][0], "g_qp_a_t_common": gain["qp_y"][1],
                    "g_mj_ddxi_common": gain["mj_y"][0], "g_mj_a_t_common": gain["mj_y"][1],
                    "qp_contact_norm": float(np.linalg.norm(gain["qp_contact"])),
                    "mj_contact_norm": float(np.linalg.norm(gain["contact"])),
                    "mj_actuator_norm": float(np.linalg.norm(gain["actuator"])),
                    "mj_remaining_norm": float(np.linalg.norm(gain["remaining"])),
                    **route, **flatten("g_qp_contact_", gain["qp_contact"]),
                    **flatten("g_mj_contact_", gain["contact"]), **flatten("g_mj_actuator_", gain["actuator"]),
                    **flatten("g_mj_remaining_", gain["remaining"]), **flatten("g_mj_lhs_", gain["lhs"]),
                    "g_native_wheel_qacc_qp_common": common(gain["native_wheel_qacc_qp"]),
                    "g_native_wheel_qacc_mj_common": common(gain["native_wheel_qacc_mj"]),
                    **{f"g_hip_common_{name}": float(value)
                       for name, value in gain["constrained_hip"].items()},
                    **{f"g_contact_parity_{name}": float(value)
                       for name, value in gain["contact_realization"].items()
                       if np.isscalar(value)},
                    **{f"g_contact_parity_side_{side}_{kind}":
                       float(gain["contact_realization"][f"side_{kind}"][side])
                       for side in range(2) for kind in ("mapping", "wrench")},
                    **{f"g_contact_parity_side_{side}_wrench_{axis}":
                       float(gain["contact_realization"]["wrench_component"][side, axis])
                       for side in range(2) for axis in range(6)},
                    **{f"g_{name}": gain[name] for name in gain
                       if name.startswith("xi_") and name not in {"xi_qp_leg_dof", "xi_mj_leg_dof"}}})
                for dof, name in model_leg_dofs:
                    for model_name in ("qp", "mj"):
                        leg_dof_rows.append({"channel": channel, "branch": "+" if sign > 0 else "-", "scale": scale,
                                             "model": model_name, "dof": dof, "joint": name,
                                             "g_ddxi_common_contribution": gain[f"xi_{model_name}_leg_dof"][name]})
        plus, minus = branches[1][1.0], branches[-1][1.0]
        split = max(relative(plus["qp_y"], minus["qp_y"]), relative(plus["mj_y"], minus["mj_y"]))
        central_allowed = split <= float(config["maximum_directional_split_relative"])
        matrices[channel] = {"input": channel, "output_rows": config["output_rows"],
            "g_qp_plus": plus["qp_y"], "g_qp_minus": minus["qp_y"],
            "g_mj_plus": plus["mj_y"], "g_mj_minus": minus["mj_y"],
            "directional_split_relative": split, "central_average_allowed": central_allowed,
            "g_qp": 0.5 * (plus["qp_y"] + minus["qp_y"]) if central_allowed else None,
            "g_mj": 0.5 * (plus["mj_y"] + minus["mj_y"]) if central_allowed else None}
        channel_branches[channel] = branches

    if all(value["central_average_allowed"] for value in matrices.values()):
        g_qp = np.column_stack([matrices[name]["g_qp"] for name in config["input_channels"]])
        g_mj = np.column_stack([matrices[name]["g_mj"] for name in config["input_channels"]])
        self_xi = g_qp[0, 0] * g_mj[0, 0] < 0.0
        self_slip = g_qp[1, 1] * g_mj[1, 1] < 0.0
        unified_qp, unified_mj = float(0.5 * np.sum(g_qp)), float(0.5 * np.sum(g_mj))
        cross = (not self_xi and not self_slip and unified_qp * unified_mj < 0.0)
        common_routes = [row["quantitatively_dominant"] for row in directional if row["scale"] == 1.0]
        contact = unified_qp * unified_mj < 0.0 and all(common_routes)
        flags = {"A-XI_SELF_REVERSAL": self_xi, "B-SLIP_SELF_REVERSAL": self_slip,
                 "C-CROSS_COUPLING_REVERSAL": cross, "D-CONTACT_REDIRECTION_DOMINANT": contact}
        active = [name for name, value in flags.items() if value]
        classification = "E-MULTIPLE" if len(active) > 1 else active[0] if active else "U-INSUFFICIENT_OR_UNTRUSTED"
        matrices["unified_reconstructed"] = {"g_qp": g_qp, "g_mj": g_mj,
            "unified_projected_qp": unified_qp, "unified_projected_mj": unified_mj}
    else:
        flags = {}; classification = "U-INSUFFICIENT_OR_UNTRUSTED"

    slip_branches = {sign: channel_branches["slip_common_only"][sign][1.0] for sign in (-1, 1)}
    mode_summaries: dict[str, dict[str, Any]] = {}
    for model_name, qacc_key in (("qp", "qacc_qp"), ("mujoco", "qacc_mj")):
        dof_key = f"xi_{'mj' if model_name == 'mujoco' else 'qp'}_leg_dof"
        slip_dof = {name: float(0.5 * (slip_branches[1][dof_key][name] + slip_branches[-1][dof_key][name]))
                    for _, name in model_leg_dofs}
        slip_qacc_gain = {
            name: float(0.5 * ((data[("slip_common_only", 1.0, 1)][qacc_key][dof] - baseline[qacc_key][dof]) / delta +
                               (data[("slip_common_only", 1.0, -1)][qacc_key][dof] - baseline[qacc_key][dof]) / -delta))
            for dof, name in model_leg_dofs
        }
        leg_sum = float(sum(slip_dof.values()))
        leg_target = float(0.5 * (slip_branches[1][f"xi_{'mj' if model_name == 'mujoco' else 'qp'}_leg_nonwheel"] +
                                  slip_branches[-1][f"xi_{'mj' if model_name == 'mujoco' else 'qp'}_leg_nonwheel"]))
        dof_closure = leg_sum - leg_target
        modes = leg_modes(baseline["xi_map"], slip_qacc_gain, model_leg_dofs)
        mode_closure = float(sum(row["common_mode_ddxi_contribution"] + row["differential_mode_ddxi_contribution"]
                                 for row in modes) - leg_sum)
        if abs(dof_closure) > 1e-10 or abs(mode_closure) > 1e-10:
            raise RuntimeError(f"{model_name} leg-dof decomposition did not close: dof={dof_closure}, mode={mode_closure}")
        mode_summaries[model_name] = {
            "ddxi_cross_gain": float(matrices["slip_common_only"]["g_mj" if model_name == "mujoco" else "g_qp"][0]),
            "leg_nonwheel_contribution": leg_target, "dof_contributions": slip_dof,
            "dof_sum": leg_sum, "dof_closure": dof_closure,
            "modes": modes, "mode_closure": mode_closure,
        }
    hip_mode_classification = classify_qp_plant_hip_modes(mode_summaries)

    contribution_names = ("actuator", "qp_contact_full", "contact",
                          "contact_realization_difference", "remaining", "actual_sum",
                          "actual_hip_common", "qp_hip_common", "force_balance_max_abs",
                          "contribution_closure", "lhs_mapping_closure")
    directional_names = contribution_names[:7]
    hip_rows = [row for row in directional if row["channel"] == "slip_common_only"]
    reference_rows = {(row["branch"], float(row["scale"])): row for row in hip_rows}
    scale_errors = []
    for branch in ("+", "-"):
        reference = reference_rows[(branch, 1.0)]
        for scale in (0.5, 0.25):
            candidate = reference_rows[(branch, scale)]
            scale_errors.append(max(abs(candidate[f"g_hip_common_{name}"] -
                                        reference[f"g_hip_common_{name}"]) /
                                    max(abs(reference[f"g_hip_common_{name}"]), 1e-12)
                                    for name in directional_names))
    plus, minus = reference_rows[("+", 1.0)], reference_rows[("-", 1.0)]
    split = max(abs(plus[f"g_hip_common_{name}"] - minus[f"g_hip_common_{name}"]) /
                max(abs(0.5 * (plus[f"g_hip_common_{name}"] + minus[f"g_hip_common_{name}"])), 1e-12)
                for name in directional_names)
    central = {name: 0.5 * (plus[f"g_hip_common_{name}"] + minus[f"g_hip_common_{name}"])
               for name in contribution_names}
    closure = max(abs(central[name]) for name in
                  ("force_balance_max_abs", "contribution_closure", "lhs_mapping_closure"))
    attribution = {
        "channel": "slip_common_only", "fixed_state": True,
        "mapping": "frozen-state realized constrained-dynamics balance: bilateral hip-common selector times M^-1, with contact and other constraint forces explicit",
        "branch_split_relative": split,
        "scale_convergence_relative": max(scale_errors),
        "contribution_closure_abs": closure,
        "maximum_abs_qp_hip_common_gain": max(abs(row["g_hip_common_qp_hip_common"])
                                               for row in hip_rows),
        "branch_and_scale_pass": (split <= float(config["maximum_directional_split_relative"]) and
                                   max(scale_errors) <= float(config["maximum_directional_convergence_relative"]) and
                                   max(abs(row["g_hip_common_qp_hip_common"]) for row in hip_rows) <= 1.0e-8),
        "closure_pass": closure <= 1.0e-8,
        "central_gain_rad_s2_per_m_s2": central,
        "whole_dynamics_contact_closure_max_abs": max(
            max(abs(float(row["dynamics_closure"])), abs(float(row["contact_closure"])))
            for row in probe_rows),
    }

    parity_names = ("qp_same", "mj_same", "mj_actual", "mapping", "wrench", "gap")
    parity_central = {name: 0.5 * (plus[f"g_contact_parity_{name}"] +
                                  minus[f"g_contact_parity_{name}"])
                      for name in parity_names}
    parity_scale_errors = []
    for branch in ("+", "-"):
        reference = reference_rows[(branch, 1.0)]
        for scale in (0.5, 0.25):
            candidate = reference_rows[(branch, scale)]
            parity_scale_errors.append(max(
                abs(candidate[f"g_contact_parity_{name}"] -
                    reference[f"g_contact_parity_{name}"]) /
                max(abs(reference[f"g_contact_parity_{name}"]), 1.0e-8)
                for name in parity_names))
    parity_split = max(abs(plus[f"g_contact_parity_{name}"] -
                               minus[f"g_contact_parity_{name}"]) /
                       max(abs(parity_central[name]), 1.0e-8)
                       for name in parity_names)
    side_wrench = [0.5 * (plus[f"g_contact_parity_side_{side}_wrench"] +
                          minus[f"g_contact_parity_side_{side}_wrench"])
                   for side in range(2)]
    component_names = ("Fr", "Fl", "Fn", "Mr", "Ml", "Mn")
    wrench_components = {
        ("left", "right")[side]: {
            component_names[axis]: 0.5 * (
                plus[f"g_contact_parity_side_{side}_wrench_{axis}"] +
                minus[f"g_contact_parity_side_{side}_wrench_{axis}"])
            for axis in range(6)}
        for side in range(2)}
    contact_plus = slip_branches[1]["contact_realization"]
    contact_minus = slip_branches[-1]["contact_realization"]
    dqp_wrench = 0.5 * (contact_plus["dqp_wrench"] + contact_minus["dqp_wrench"])
    dmj_wrench = 0.5 * (contact_plus["dmj_wrench"] + contact_minus["dmj_wrench"])
    slip_samples = [baseline, *(data[("slip_common_only", scale, sign)]
                                 for scale in map(float, config["delta_scales"])
                                 for sign in (-1, 1))]
    constraint_mechanism = {
        "contact_count_by_side_observed": {
            ("left", "right")[side]: sorted({
                sum(int(row["side"]) == side for row in item["actual"]["details"])
                for item in slip_samples}) for side in range(2)},
        "contact_dimension_observed": sorted({
            int(row["dim"]) for item in slip_samples for row in item["actual"]["details"]}),
        "minimum_friction_margin_diagnostic_n": min(
            float(row["friction_margin_diagnostic_n"])
            for item in slip_samples for row in item["actual"]["details"]),
        "normal_frame_max_vector_error": max(
            float(side["normal_max_vector_error"])
            for item in slip_samples for side in item["contact_realization"]["geometry_parity"]),
        "interpretation": ("fixed two-point-per-wheel MuJoCo constraint reaction; the directional "
                           "probe does not switch contact count, contact dimension, or normal frame"),
    }
    contact_closure = max(
        max(abs(row["g_contact_parity_closure"]),
            abs(row["g_contact_parity_generalized_force_closure_max_abs"]),
            abs(row["g_contact_parity_actual_contact_reconstruction_max_abs"]))
        for row in hip_rows)
    mapping_fraction = abs(parity_central["mapping"]) / max(abs(parity_central["gap"]), 1.0e-12)
    wrench_fraction = abs(parity_central["wrench"]) / max(abs(parity_central["gap"]), 1.0e-12)
    contact_classification = ("B-WRENCH_REALIZATION_DOMINANT" if mapping_fraction <= 0.1 else
                              "A-GEOMETRY_MAPPING_DOMINANT" if wrench_fraction <= 0.1 else
                              "C-MIXED_CONTACT_GAP")
    geometry_sides = []
    for side in range(2):
        qp_geometry, mj_geometry = model_b_geometry[side], frozen_geometry[side]
        actual_contact = baseline["contact_realization"]["geometry_parity"][side]
        geometry_sides.append({
            "side": ("left", "right")[side],
            "qp_contact_point_world_m": qp_geometry["point"],
            "mj_contact_point_world_m": mj_geometry["point"],
            "mj_minus_qp_contact_point_world_m": mj_geometry["point"] - qp_geometry["point"],
            "contact_frame_qp_world": qp_geometry["frame"],
            "contact_frame_mj_world": mj_geometry["frame"],
            "frame_max_abs_difference": float(np.max(np.abs(
                mj_geometry["frame"] - qp_geometry["frame"]))),
            "lever_arm_qp_world_m": qp_geometry["lever"],
            "lever_arm_mj_world_m": mj_geometry["lever"],
            "mj_minus_qp_lever_arm_world_m": mj_geometry["lever"] - qp_geometry["lever"],
            "actual_contact_count": actual_contact["contact_count"],
            "actual_cop_minus_mj_reference_world_m": actual_contact["cop_minus_qp_center"],
            "actual_contact_normal_max_vector_error": actual_contact["normal_max_vector_error"],
        })
    geometry_parity = {
        "qp_model_b_reconstruction": model_b_metrics,
        "mujoco_plant_reduction": plant_reduction_metrics,
        "qp_mj_wrench_map_max_abs": max(
            float(np.max(np.abs(baseline["contact_realization"]["qp_maps"][side] -
                                    baseline["contact_realization"]["mj_maps"][side])))
            for side in range(2)),
        "qp_reconstructed_vs_logged_map_max_abs": max(
            float(np.max(np.abs(model_b_maps[side] -
                                baseline["contact_realization"]["qp_maps"][side])))
            for side in range(2)),
        "frame_orthonormal_max_abs": max(
            float(np.max(np.abs(item["frame"].T @ item["frame"] - np.eye(3))))
            for item in (*model_b_geometry, *frozen_geometry)),
        "sides": geometry_sides,
        "wrench_reference": {
            "qp": "Model-B analytic contact center and Model-B contact frame",
            "mujoco": "current-plant analytic contact center and frame; multi-contact wrench transported to this reference",
            "mapping_term": "same numeric QP wrench components through each model's own verified frame/reference mapping",
            "wrench_term": "MuJoCo-minus-QP wrench components through the frozen MuJoCo mapping",
        },
        "left_right_order": ["left", "right"],
    }
    contact_parity = {
        "classification": contact_classification,
        "geometry_frame_parity": geometry_parity,
        "same_wrench_and_realization_hip_common_gain": parity_central,
        "mapping_fraction_of_contact_gap": mapping_fraction,
        "wrench_fraction_of_contact_gap": wrench_fraction,
        "wrench_realization_side_contribution": {"left": side_wrench[0], "right": side_wrench[1]},
        "wrench_realization_component_contribution": wrench_components,
        "qp_wrench_directional_gain": dqp_wrench,
        "mujoco_wrench_directional_gain": dmj_wrench,
        "mujoco_minus_qp_wrench_directional_gain": dmj_wrench - dqp_wrench,
        "constraint_realization_mechanism_check": constraint_mechanism,
        "branch_split_relative": parity_split,
        "scale_convergence_relative": max(parity_scale_errors),
        "decomposition_closure_max_abs": contact_closure,
        "trusted": (parity_split <= float(config["maximum_directional_split_relative"]) and
                    max(parity_scale_errors) <= float(config["maximum_directional_convergence_relative"]) and
                    contact_closure <= 1.0e-8),
        "scope": "compatible-H0 tick0 fixed-state slip-common only; no trajectory or controller change",
    }

    P45.write_csv(output / "probe-observables.csv", probe_rows)
    P45.write_csv(output / "directional-transfer.csv", directional)
    P45.write_csv(output / "leg-dof-transfer.csv", leg_dof_rows)
    P45.write_json(output / "leg-mode-summary.json", {
        "channel": "slip_common_only", "scale": 1.0,
        **mode_summaries,
    })
    P45.write_json(output / "qp-plant-hip-mode-classification.json", hip_mode_classification)
    P45.write_json(output / "constrained-hip-common-attribution.json", attribution)
    P45.write_json(output / "contact-mapping-wrench-parity.json", contact_parity)
    P45.write_json(output / "common-transfer-matrices.json", matrices)
    P45.write_json(output / "classification.json", {"classification": classification,
        "flags": flags, "central_averages_allowed": all(value["central_average_allowed"] for value in matrices.values() if isinstance(value, dict) and "central_average_allowed" in value),
        "scope_contract": config["scope_contract"]})
    compared = ["probe-observables.csv", "directional-transfer.csv", "leg-dof-transfer.csv", "leg-mode-summary.json",
                "qp-plant-hip-mode-classification.json", "common-transfer-matrices.json", "classification.json",
                "constrained-hip-common-attribution.json", "contact-mapping-wrench-parity.json"]
    replay_error = max(P45.semantic_error(args.replay_of / name, output / name) for name in compared) if args.replay_of else None
    replay_pass = replay_error is None or replay_error <= float(base["gates"]["semantic_replay_max_abs"])
    P45.write_json(output / "summary.json", {"pass": all_trusted and attribution["branch_and_scale_pass"] and
        attribution["closure_pass"] and contact_parity["trusted"] and replay_pass,
        "classification": classification,
        "all_directional_scales_trusted": all_trusted, "replay_max_abs_error": replay_error,
        "qp_plant_hip_mode_classification": hip_mode_classification["classification"],
        "slip_qp_leg_dof_closure": mode_summaries["qp"]["dof_closure"],
        "slip_qp_leg_mode_closure": mode_summaries["qp"]["mode_closure"],
        "slip_mj_leg_dof_closure": mode_summaries["mujoco"]["dof_closure"],
        "slip_mj_leg_mode_closure": mode_summaries["mujoco"]["mode_closure"],
        "constrained_hip_attribution": attribution, "contact_parity": contact_parity,
        "scope_contract": config["scope_contract"]})
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"], authority, wrench_source, Path(__file__).resolve()]
    P45.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(), "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if (all_trusted and attribution["branch_and_scale_pass"] and
                 attribution["closure_pass"] and contact_parity["trusted"] and replay_pass) else 2


if __name__ == "__main__":
    raise SystemExit(main())
