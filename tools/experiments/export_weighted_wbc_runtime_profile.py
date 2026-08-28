#!/usr/bin/env python3
"""Export the deterministic, MuJoCo-compiled Phase-21 runtime model profile."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT / "simulation/mujoco/config/phase21_runtime_model_profile_v1.json"
MODEL_CONFIG = ROOT / "simulation/mujoco/config/phase21_model_oracle_v8.json"
CONTACT_CONFIG = ROOT / "simulation/mujoco/config/phase21_contact_centered_wrench_oracle.json"
HARD_CONFIG = ROOT / "simulation/mujoco/config/phase21_hard_qp_42d.json"
TASK_CONFIG = ROOT / "simulation/mujoco/config/phase21_task_prefreeze_42d.json"
NONLINEAR_CONFIG = ROOT / "simulation/mujoco/config/phase21_task_prefreeze_42d_nonlinear_frozen_v1.json"
PHASE14_MANIFEST = ROOT / "data/experiments/2026-08-25-mujoco-internal-dynamics/raw/parameter_manifest.json"
PHASE15_MANIFEST = ROOT / "docs/workflow/phases/15-mujoco-closed-chain-kinematics/evidence/automated/2026-08-25-nominal/geometry_manifest.json"
PHASE15_CONFIG = ROOT / "simulation/mujoco/config/phase15_nominal.json"

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402
from validate_weighted_wbc_continuous_contact import ContinuousPatch  # noqa: E402
from validate_weighted_wbc_contact_centered_wrench import (  # noqa: E402
    build_h,
    geometry_map,
    rays,
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rel(path: Path) -> str:
    return path.resolve().relative_to(ROOT).as_posix()


def named_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"missing compiled {kind.name} {name!r}")
    return int(value)


def finite(value: Any, path: str = "profile") -> None:
    if isinstance(value, dict):
        for key, nested in value.items():
            finite(nested, f"{path}.{key}")
    elif isinstance(value, list):
        for index, nested in enumerate(value):
            finite(nested, f"{path}[{index}]")
    elif isinstance(value, float) and not np.isfinite(value):
        raise RuntimeError(f"non-finite value at {path}")


def body_record(model: mujoco.MjModel, body: int) -> dict[str, Any]:
    return {
        "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body),
        "id": body,
        "parent_id": int(model.body_parentid[body]),
        "position_parent_m": model.body_pos[body].tolist(),
        "quaternion_parent_wxyz": model.body_quat[body].tolist(),
        "mass_kg": float(model.body_mass[body]),
        "com_local_m": model.body_ipos[body].tolist(),
        "inertial_quaternion_wxyz": model.body_iquat[body].tolist(),
        "principal_inertia_kg_m2": model.body_inertia[body].tolist(),
    }


def joint_record(model: mujoco.MjModel, joint: int) -> dict[str, Any]:
    return {
        "name": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint),
        "id": joint,
        "body_id": int(model.jnt_bodyid[joint]),
        "type": int(model.jnt_type[joint]),
        "qpos_address": int(model.jnt_qposadr[joint]),
        "dof_address": int(model.jnt_dofadr[joint]),
        "position_body_m": model.jnt_pos[joint].tolist(),
        "axis_body": model.jnt_axis[joint].tolist(),
    }


def site_record(model: mujoco.MjModel, name: str) -> dict[str, Any]:
    site = named_id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    return {
        "name": name,
        "id": site,
        "body_id": int(model.site_bodyid[site]),
        "position_body_m": model.site_pos[site].tolist(),
        "quaternion_body_wxyz": model.site_quat[site].tolist(),
    }


def compare_phase14(model: mujoco.MjModel, manifest: dict[str, Any]) -> float:
    maximum = 0.0
    for expected in manifest["bodies"]:
        body = named_id(model, mujoco.mjtObj.mjOBJ_BODY, expected["name"])
        fields = (
            (np.asarray([model.body_mass[body]]), np.asarray([expected["mass_kg"]])),
            (model.body_ipos[body], np.asarray(expected["local_com_m"])),
            (model.body_inertia[body], np.asarray(expected["principal_inertia_kg_m2"])),
            (model.body_iquat[body], np.asarray(expected["inertial_quaternion_wxyz"])),
        )
        maximum = max(maximum, *(float(np.max(np.abs(actual - reference)))
                                  for actual, reference in fields))
    if maximum > 1.0e-12:
        raise RuntimeError(f"Phase-14 compiled inertial mismatch: {maximum:.17g}")
    return maximum


def build_profile() -> dict[str, Any]:
    model_cfg, model_inputs = load_config(MODEL_CONFIG)
    contact_cfg, contact_inputs = load_config(CONTACT_CONFIG)
    hard_cfg, hard_inputs = load_config(HARD_CONFIG)
    task_cfg, task_inputs = load_config(TASK_CONFIG)
    nonlinear_cfg, nonlinear_inputs = load_config(NONLINEAR_CONFIG)
    equilibrium_path = ROOT / model_cfg["equilibrium"]
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_cfg, equilibrium)
    model = oracle.model
    if (model.nbody, model.njnt, model.nq, model.nv, model.nu) != (12, 11, 17, 16, 6):
        raise RuntimeError("unexpected compiled current-nominal dimensions")

    continuous_cfg, continuous_inputs = load_config(
        (ROOT / contact_cfg["continuous_contact_config"]).resolve())
    patch = ContinuousPatch(oracle, continuous_cfg["continuous_contact_oracle"])
    equilibrium_q = oracle.sample_qpos(model_cfg["samples"][0])
    _, offsets, _ = geometry_map(patch, equilibrium_q, 0)
    h_cone, hull = build_h(
        rays(offsets, float(contact_cfg["friction_coefficient"])),
        contact_cfg["hull_qhull_options"])
    if h_cone.shape != (37, 6):
        raise RuntimeError(f"unexpected H-cone shape {h_cone.shape}")

    phase14 = json.loads(PHASE14_MANIFEST.read_text(encoding="utf-8"))
    phase15 = json.loads(PHASE15_MANIFEST.read_text(encoding="utf-8"))
    phase15_config = json.loads(PHASE15_CONFIG.read_text(encoding="utf-8"))
    inertial_error = compare_phase14(model, phase14)
    included_model = ROOT / phase14["included_model"]
    scene = ROOT / model_cfg["scene"]
    included_model_hash = sha256(included_model)

    joints = [joint_record(model, joint) for joint in range(1, model.njnt)]
    actuator_order = []
    for name in model_cfg["actuators"]:
        actuator = named_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        joint = int(model.actuator_trnid[actuator, 0])
        actuator_order.append({
            "name": name,
            "id": actuator,
            "joint": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint),
            "native_gear": float(model.actuator_gear[actuator, 0]),
            "canonical_torque_sign": -1.0,
        })

    all_inputs = {
        path.resolve() for path in [
            *model_inputs, *contact_inputs, *continuous_inputs, *hard_inputs,
            *task_inputs, *nonlinear_inputs, equilibrium_path, PHASE14_MANIFEST,
            PHASE15_MANIFEST, scene, included_model,
            PHASE15_CONFIG,
        ]
    }
    profile = {
        "schema_version": 1,
        "profile": "phase21_runtime_model_profile_v1",
        "claim": "current nominal MuJoCo simulation only",
        "compiled_with": {"mujoco": mujoco.__version__, "numpy": np.__version__},
        "provenance": [{"path": rel(path), "sha256": sha256(path)}
                       for path in sorted(all_inputs)],
        "compiled_dimensions": {"nbody_including_world": model.nbody,
                                "njnt": model.njnt, "nq": model.nq,
                                "nv": model.nv, "nu": model.nu},
        "gravity_world_m_s2": model.opt.gravity.tolist(),
        "bodies": [body_record(model, body) for body in range(1, model.nbody)],
        "joints": joints,
        "sites": [site_record(model, name) for name in [
            model_cfg["base_control_site"],
            *(name for pair in model_cfg["closure_pairs"] for name in pair),
        ]],
        "orders": {
            "canonical_active_joints": model_cfg["canonical_active_joints"],
            "passive_joints": model_cfg["passive_joints"],
            "actuators": actuator_order,
            "reduced_velocity": ["base_linear_world_x", "base_linear_world_y",
                                 "base_linear_world_z", "base_angular_world_x",
                                 "base_angular_world_y", "base_angular_world_z",
                                 *[f"canonical_{name}" for name in model_cfg["canonical_active_joints"]]],
            "decision": ["nudot_12", "tau_6", "left_wrench_C_6",
                         "right_wrench_C_6", "left_slack_FLU_6", "right_slack_FLU_6"],
        },
        "closure": {
            "site_pairs": model_cfg["closure_pairs"],
            "equilibrium_active_native_rad": oracle.equilibrium_active.tolist(),
            "equilibrium_passive_native_rad": oracle.equilibrium_passive.tolist(),
            "canonical_joint_offsets_rad": model_cfg["canonical_joint_offsets_rad"],
            "solver": model_cfg["solver"],
            "thresholds": {key: value for key, value in model_cfg["thresholds"].items()
                           if key in ("maximum_closure_residual_m",
                                      "maximum_passive_condition_number",
                                      "minimum_passive_singular_value")},
            "workspace_rad": phase15_config["workspace_rad"],
        },
        "contact": {
            "wheel_bodies": model_cfg["wheel_bodies"],
            "ground_normal_world": continuous_cfg["continuous_contact_oracle"]["ground_normal_world"],
            "radius_m": continuous_cfg["continuous_contact_oracle"]["radius_m"],
            "support_band_m": continuous_cfg["continuous_contact_oracle"]["support_band_m"],
            "mesh_axis_bounds_m": patch.bounds,
            "contact_center_offsets_C_m": offsets.tolist(),
            "friction_coefficient": contact_cfg["friction_coefficient"],
            "h_cone_37x6": h_cone.tolist(),
            "hull": hull,
        },
        "wbc": {
            "hard": {key: hard_cfg[key] for key in (
                "variable_scale", "row_scale", "bounds",
                "minimum_scaled_norm_regularization", "solver", "gates")},
            "task": task_cfg["task"],
            "nonlinear": {key: nonlinear_cfg[key] for key in (
                "duration_s", "physics_steps_per_control", "nonlinear_gains", "gates")},
        },
        "cross_checks": {
            "phase14_maximum_compiled_inertial_error": inertial_error,
            "phase14_body_count": len(phase14["bodies"]),
            "phase14_included_model_sha256": phase14["included_model_sha256"],
            "current_included_model_sha256": included_model_hash,
            "included_model_hash_matches_phase14":
                included_model_hash == phase14["included_model_sha256"],
            "phase15_profile": phase15["profile"],
            "h_cone_rows": int(h_cone.shape[0]),
        },
    }
    finite(profile)
    return profile


def cpp_number(value: float) -> str:
    return format(float(value), ".17g")


def cpp_array(values: Any) -> str:
    return "{" + ", ".join(
        cpp_array(value) if isinstance(value, (list, tuple)) else cpp_number(value)
        for value in values) + "}"


def cpp_nested_array(values: Any) -> str:
    return "{{" + ", ".join(cpp_array(value) for value in values) + "}}"


def render_cpp(profile: dict[str, Any]) -> str:
    bodies = profile["bodies"]
    body_rows = [[*body["position_parent_m"], *body["quaternion_parent_wxyz"],
                  body["mass_kg"], *body["com_local_m"],
                  *body["inertial_quaternion_wxyz"],
                  *body["principal_inertia_kg_m2"]] for body in bodies]
    hard = profile["wbc"]["hard"]
    scales = hard["variable_scale"]
    variable_scale = (scales["acceleration"] + scales["torque"] +
                      2 * scales["wrench_per_side"] + 2 * scales["slack_per_side"])
    source_hash = hashlib.sha256(
        (json.dumps(profile, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    ).hexdigest()
    return f"""// Generated by export_weighted_wbc_runtime_profile.py; do not edit.
#pragma once

#include <array>

namespace wheel_leg::phase21_profile {{
inline constexpr char kProfileSha256[] = \"{source_hash}\";
inline constexpr std::array<int, 11> kBodyParent{cpp_array([b['parent_id'] for b in bodies])};
inline constexpr std::array<std::array<double, 18>, 11> kBody{cpp_nested_array(body_rows)};
inline constexpr std::array<double, 3> kGravity{cpp_array(profile['gravity_world_m_s2'])};
inline constexpr std::array<double, 3> kBaseControlPosition{cpp_array(profile['sites'][0]['position_body_m'])};
inline constexpr std::array<int, 10> kJointBody{cpp_array([j['body_id'] for j in profile['joints']])};
inline constexpr std::array<int, 6> kActiveNative{{5, 6, 7, 0, 1, 2}};
inline constexpr std::array<int, 4> kPassiveNative{{8, 9, 3, 4}};
inline constexpr std::array<double, 6> kCanonicalOffset{cpp_array(profile['closure']['canonical_joint_offsets_rad'])};
inline constexpr std::array<double, 6> kEquilibriumActiveNative{cpp_array(profile['closure']['equilibrium_active_native_rad'])};
inline constexpr std::array<double, 4> kEquilibriumPassive{cpp_array(profile['closure']['equilibrium_passive_native_rad'])};
inline constexpr std::array<std::array<double, 2>, 3> kWorkspaceBounds{cpp_nested_array([[-0.65, 0.65], [-0.75, 0.75], [-1.0, 1.0]])};
inline constexpr std::array<int, 4> kClosureBody{{11, 8, 6, 3}};
inline constexpr std::array<std::array<double, 3>, 4> kClosurePosition{cpp_nested_array([[-0.0435, -0.17467, 0.0], [0.0318, -0.03859, 0.0105], [-0.0435, -0.17467, 0.0], [0.0318, -0.03859, -0.0105]])};
inline constexpr std::array<int, 2> kWheelBody{{9, 4}};
inline constexpr std::array<std::array<double, 2>, 2> kWheelAxisBounds{cpp_nested_array(profile['contact']['mesh_axis_bounds_m'])};
inline constexpr double kWheelRadius = {cpp_number(profile['contact']['radius_m'])};
inline constexpr double kSupportBand = {cpp_number(profile['contact']['support_band_m'])};
inline constexpr std::array<std::array<double, 6>, 37> kWrenchCone{cpp_nested_array(profile['contact']['h_cone_37x6'])};
inline constexpr std::array<double, 42> kVariableScale{cpp_array(variable_scale)};
inline constexpr std::array<double, 12> kDynamicsRowScale{cpp_array(hard['row_scale']['dynamics'])};
inline constexpr std::array<double, 6> kTorqueLimit{cpp_array(hard['bounds']['torque_nm'])};
inline constexpr std::array<double, 12> kAccelerationLimit{cpp_array(hard['bounds']['acceleration'])};
}}  // namespace wheel_leg::phase21_profile
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--cpp-output", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite existing profile: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    profile = build_profile()
    payload = json.dumps(profile, indent=2, sort_keys=True,
                         allow_nan=False) + "\n"
    output.write_text(payload, encoding="utf-8")
    print(f"wrote {output} ({hashlib.sha256(payload.encode()).hexdigest()})")
    if args.cpp_output:
        cpp_output = args.cpp_output.resolve()
        if cpp_output.exists():
            raise RuntimeError(f"refusing to overwrite existing C++ profile: {cpp_output}")
        cpp_output.parent.mkdir(parents=True, exist_ok=True)
        cpp_output.write_text(render_cpp(profile), encoding="utf-8")
        print(f"wrote {cpp_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
