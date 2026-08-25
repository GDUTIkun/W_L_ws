"""Run the Phase-15 MuJoCo closed-chain kinematics validation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


DEFAULT_CONFIG = "simulation/mujoco/config/phase15_nominal.json"
DEFAULT_OUTPUT = (
    "docs/workflow/phases/15-mujoco-closed-chain-kinematics/"
    "evidence/automated/2026-08-25-nominal"
)
JOINT_KEYS = ("hip", "knee", "wheel", "connect1", "connect2")


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise AssertionError(f"Missing {kind.name} named {name!r}")
    return result


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values: np.ndarray | list[float]) -> list[float]:
    return [float(value) for value in np.asarray(values).ravel()]


def max_abs(*values: np.ndarray) -> float:
    return max(float(np.max(np.abs(value))) for value in values)


def rx(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def ry(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rz(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def euler_xyz(values: list[float]) -> np.ndarray:
    return rx(values[0]) @ ry(values[1]) @ rz(values[2])


def compose(
    position: np.ndarray,
    rotation: np.ndarray,
    local_position: list[float],
    local_rotation: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    return position + rotation @ np.asarray(local_position), rotation @ local_rotation


def context(model: mujoco.MjModel, side: str, config: dict[str, Any]) -> dict[str, Any]:
    side_config = config["sides"][side]
    joint_names = side_config["active_joints"] + side_config["passive_joints"]
    joint_ids = [
        object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in joint_names
    ]
    return {
        "side": side,
        "config": side_config,
        "joint_names": joint_names,
        "joint_ids": joint_ids,
        "qpos": np.array([model.jnt_qposadr[joint] for joint in joint_ids]),
        "dofs": np.array([model.jnt_dofadr[joint] for joint in joint_ids]),
        "wheel_body": object_id(
            model, mujoco.mjtObj.mjOBJ_BODY, side_config["wheel_body"]
        ),
        "wheel_geom": object_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, side_config["wheel_geom"]
        ),
        "calf_site": object_id(
            model, mujoco.mjtObj.mjOBJ_SITE, side_config["calf_site"]
        ),
        "connect_site": object_id(
            model, mujoco.mjtObj.mjOBJ_SITE, side_config["connect_site"]
        ),
    }


def set_state(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    leg: dict[str, Any],
    active: np.ndarray,
    passive: np.ndarray,
) -> None:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.eq_active[:] = 0
    data.qpos[leg["qpos"][:3]] = active
    data.qpos[leg["qpos"][3:]] = passive
    mujoco.mj_forward(model, data)


def site_jacobian(
    model: mujoco.MjModel, data: mujoco.MjData, site: int
) -> tuple[np.ndarray, np.ndarray]:
    linear = np.zeros((3, model.nv))
    angular = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, linear, angular, site)
    return linear, angular


def closure_state(
    model: mujoco.MjModel, data: mujoco.MjData, leg: dict[str, Any]
) -> tuple[np.ndarray, np.ndarray]:
    connect_linear, _ = site_jacobian(model, data, leg["connect_site"])
    calf_linear, _ = site_jacobian(model, data, leg["calf_site"])
    residual = (
        data.site_xpos[leg["connect_site"]]
        - data.site_xpos[leg["calf_site"]]
    )
    return residual, (connect_linear - calf_linear)[:, leg["dofs"]]


def solve_passive(
    model: mujoco.MjModel,
    leg: dict[str, Any],
    active: np.ndarray,
    seed: np.ndarray,
    solver: dict[str, Any],
) -> tuple[np.ndarray, dict[str, Any], mujoco.MjData]:
    data = mujoco.MjData(model)
    passive = np.asarray(seed, dtype=float).copy()
    converged = False
    iterations = 0
    step_norm = 0.0
    for iterations in range(solver["max_iterations"] + 1):
        set_state(model, data, leg, active, passive)
        residual, jacobian = closure_state(model, data, leg)
        residual_norm = float(np.max(np.abs(residual)))
        if residual_norm <= solver["position_tolerance_m"]:
            converged = True
            break
        step = np.linalg.lstsq(
            jacobian[:, 3:], -residual, rcond=solver["rcond"]
        )[0]
        step_norm = float(np.max(np.abs(step)))
        largest = float(np.linalg.norm(step))
        if largest > solver["max_step_rad"]:
            step *= solver["max_step_rad"] / largest
        passive += step
        if step_norm <= solver["step_tolerance_rad"]:
            break
    set_state(model, data, leg, active, passive)
    residual, jacobian = closure_state(model, data, leg)
    singular_values = np.linalg.svd(jacobian[:, 3:], compute_uv=False)
    return passive, {
        "converged": converged,
        "iterations": iterations,
        "max_residual_m": float(np.max(np.abs(residual))),
        "last_step_rad": step_norm,
        "passive_min_singular_value": float(singular_values[-1]),
        "passive_condition_number": float(singular_values[0] / singular_values[-1]),
    }, data


def point_jacobian(
    point: np.ndarray,
    indices: tuple[int, ...],
    origins: list[np.ndarray],
    axes: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    linear = np.zeros((3, 5))
    angular = np.zeros((3, 5))
    for index in indices:
        linear[:, index] = np.cross(axes[index], point - origins[index])
        angular[:, index] = axes[index]
    return linear, angular


def profile_kinematics(
    leg: dict[str, Any],
    active: np.ndarray,
    passive: np.ndarray,
    contact: dict[str, Any],
) -> dict[str, np.ndarray]:
    geometry = leg["config"]["geometry"]
    base_position = np.asarray(geometry["base_position_m"])
    identity = np.eye(3)
    thigh_position, thigh_fixed = compose(
        base_position,
        identity,
        geometry["thigh_position_m"],
        euler_xyz(geometry["thigh_euler_xyz_rad"]),
    )
    hip_axis = thigh_fixed @ np.array([0.0, 0.0, 1.0])
    thigh_rotation = thigh_fixed @ rz(active[0])

    calf_position, calf_fixed = compose(
        thigh_position,
        thigh_rotation,
        geometry["calf_position_m"],
        euler_xyz(geometry["calf_euler_xyz_rad"]),
    )
    knee_axis = calf_fixed @ np.array([0.0, 0.0, 1.0])
    calf_rotation = calf_fixed @ rz(active[1])

    wheel_position, wheel_fixed = compose(
        calf_position,
        calf_rotation,
        geometry["wheel_position_m"],
        identity,
    )
    wheel_axis = wheel_fixed @ np.array([0.0, 0.0, 1.0])
    wheel_rotation = wheel_fixed @ rz(active[2])

    connect1_position, connect1_fixed = compose(
        thigh_position,
        thigh_rotation,
        geometry["connect1_position_m"],
        euler_xyz(geometry["connect1_euler_xyz_rad"]),
    )
    connect1_axis = connect1_fixed @ np.array([0.0, 0.0, 1.0])
    connect1_rotation = connect1_fixed @ rz(passive[0])
    connect2_position, connect2_fixed = compose(
        connect1_position,
        connect1_rotation,
        geometry["connect2_position_m"],
        euler_xyz(geometry["connect2_euler_xyz_rad"]),
    )
    connect2_axis = connect2_fixed @ np.array([0.0, 0.0, 1.0])
    connect2_rotation = connect2_fixed @ rz(passive[1])

    calf_site = calf_position + calf_rotation @ np.asarray(
        geometry["calf_site_position_m"]
    )
    connect_site = connect2_position + connect2_rotation @ np.asarray(
        geometry["connect_site_position_m"]
    )
    contact_point = wheel_position + wheel_rotation @ np.asarray(
        contact["contact_point_at_zero_local_m"]
    )

    origins = [
        thigh_position,
        calf_position,
        wheel_position,
        connect1_position,
        connect2_position,
    ]
    axes = [hip_axis, knee_axis, wheel_axis, connect1_axis, connect2_axis]
    center_linear, center_angular = point_jacobian(
        wheel_position, (0, 1, 2), origins, axes
    )
    contact_linear, contact_angular = point_jacobian(
        contact_point, (0, 1, 2), origins, axes
    )
    calf_linear, _ = point_jacobian(calf_site, (0, 1), origins, axes)
    connect_linear, _ = point_jacobian(connect_site, (0, 3, 4), origins, axes)
    return {
        "wheel_position": wheel_position,
        "wheel_rotation": wheel_rotation,
        "contact_point": contact_point,
        "calf_site": calf_site,
        "connect_site": connect_site,
        "closure_jacobian": connect_linear - calf_linear,
        "center_linear_jacobian": center_linear,
        "center_angular_jacobian": center_angular,
        "contact_linear_jacobian": contact_linear,
        "contact_angular_jacobian": contact_angular,
    }


def mujoco_kinematics(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    leg: dict[str, Any],
    contact: dict[str, Any],
) -> dict[str, np.ndarray]:
    wheel_position = data.xpos[leg["wheel_body"]].copy()
    wheel_rotation = data.xmat[leg["wheel_body"]].reshape(3, 3).copy()
    contact_point = wheel_position + wheel_rotation @ np.asarray(
        contact["contact_point_at_zero_local_m"]
    )
    center_linear = np.zeros((3, model.nv))
    center_angular = np.zeros((3, model.nv))
    mujoco.mj_jacBody(
        model,
        data,
        center_linear,
        center_angular,
        leg["wheel_body"],
    )
    contact_linear = np.zeros((3, model.nv))
    contact_angular = np.zeros((3, model.nv))
    mujoco.mj_jac(
        model,
        data,
        contact_linear,
        contact_angular,
        contact_point,
        leg["wheel_body"],
    )
    _, closure_jacobian = closure_state(model, data, leg)
    columns = leg["dofs"]
    return {
        "wheel_position": wheel_position,
        "wheel_rotation": wheel_rotation,
        "contact_point": contact_point,
        "calf_site": data.site_xpos[leg["calf_site"]].copy(),
        "connect_site": data.site_xpos[leg["connect_site"]].copy(),
        "closure_jacobian": closure_jacobian,
        "center_linear_jacobian": center_linear[:, columns],
        "center_angular_jacobian": center_angular[:, columns],
        "contact_linear_jacobian": contact_linear[:, columns],
        "contact_angular_jacobian": contact_angular[:, columns],
    }


def reduction(
    closure_jacobian: np.ndarray, rcond: float
) -> tuple[np.ndarray, dict[str, float]]:
    active = closure_jacobian[:, :3]
    passive = closure_jacobian[:, 3:]
    passive_map = np.linalg.lstsq(passive, -active, rcond=rcond)[0]
    mapping = np.vstack((np.eye(3), passive_map))
    singular_values = np.linalg.svd(passive, compute_uv=False)
    return mapping, {
        "tangent_residual": float(
            np.max(np.abs(closure_jacobian @ mapping))
        ),
        "passive_min_singular_value": float(singular_values[-1]),
        "passive_condition_number": float(
            singular_values[0] / singular_values[-1]
        ),
    }


def reduced_task_jacobians(
    state: dict[str, np.ndarray], mapping: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    center = np.vstack(
        (state["center_linear_jacobian"], state["center_angular_jacobian"])
    ) @ mapping
    contact = np.vstack(
        (state["contact_linear_jacobian"], state["contact_angular_jacobian"])
    ) @ mapping
    return center, contact


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(cosine))
    skew = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    if angle < 1.0e-8:
        return 0.5 * skew
    return angle / (2.0 * np.sin(angle)) * skew


def finite_difference(
    model: mujoco.MjModel,
    leg: dict[str, Any],
    active: np.ndarray,
    passive: np.ndarray,
    epsilon: float,
    config: dict[str, Any],
) -> dict[str, np.ndarray]:
    center_linear = np.zeros((3, 3))
    center_angular = np.zeros((3, 3))
    contact_linear = np.zeros((3, 3))
    for column in range(3):
        plus = active.copy()
        minus = active.copy()
        plus[column] += epsilon
        minus[column] -= epsilon
        plus_passive, plus_info, plus_data = solve_passive(
            model, leg, plus, passive, config["solver"]
        )
        minus_passive, minus_info, minus_data = solve_passive(
            model, leg, minus, passive, config["solver"]
        )
        if not plus_info["converged"] or not minus_info["converged"]:
            raise AssertionError("Finite-difference passive solve did not converge")
        plus_state = mujoco_kinematics(
            model, plus_data, leg, config["contact_point"]
        )
        minus_state = mujoco_kinematics(
            model, minus_data, leg, config["contact_point"]
        )
        center_linear[:, column] = (
            plus_state["wheel_position"] - minus_state["wheel_position"]
        ) / (2.0 * epsilon)
        contact_linear[:, column] = (
            plus_state["contact_point"] - minus_state["contact_point"]
        ) / (2.0 * epsilon)
        center_angular[:, column] = rotation_vector(
            plus_state["wheel_rotation"]
            @ minus_state["wheel_rotation"].T
        ) / (2.0 * epsilon)
        del plus_passive, minus_passive
    return {
        "center_linear": center_linear,
        "center_angular": center_angular,
        "contact_linear": contact_linear,
    }


def mesh_geometry(
    model: mujoco.MjModel, leg: dict[str, Any], contact: dict[str, Any]
) -> dict[str, Any]:
    geom = leg["wheel_geom"]
    mesh = int(model.geom_dataid[geom])
    start = int(model.mesh_vertadr[mesh])
    count = int(model.mesh_vertnum[mesh])
    vertices = model.mesh_vert[start : start + count]
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
    local_vertices = vertices @ rotation.reshape(3, 3).T + model.geom_pos[geom]
    radial = np.linalg.norm(local_vertices[:, :2], axis=1)
    radius = float(contact["nominal_radius_m"])
    return {
        "geom": leg["config"]["wheel_geom"],
        "joint_axis_local": vector(contact["wheel_axis_local"]),
        "vertex_count": count,
        "body_local_bounds_min_m": vector(np.min(local_vertices, axis=0)),
        "body_local_bounds_max_m": vector(np.max(local_vertices, axis=0)),
        "compiled_max_radial_m": float(np.max(radial)),
        "nominal_radius_m": radius,
        "nominal_radius_deviation_m": abs(float(np.max(radial)) - radius),
        "axial_width_m": float(np.ptp(local_vertices[:, 2])),
    }


def geometry_manifest(
    model: mujoco.MjModel,
    legs: dict[str, dict[str, Any]],
    config: dict[str, Any],
    scene_path: Path,
    model_path: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    thresholds = config["thresholds"]
    max_position = 0.0
    max_rotation = 0.0
    max_mesh_radius_deviation = 0.0
    max_rolling_direction_error = 0.0
    side_entries = {}
    for side, leg in legs.items():
        active = np.zeros(3)
        passive = np.zeros(2)
        data = mujoco.MjData(model)
        set_state(model, data, leg, active, passive)
        profile = profile_kinematics(
            leg, active, passive, config["contact_point"]
        )
        compiled = mujoco_kinematics(
            model, data, leg, config["contact_point"]
        )
        position_error = max_abs(
            profile["wheel_position"] - compiled["wheel_position"],
            profile["calf_site"] - compiled["calf_site"],
            profile["connect_site"] - compiled["connect_site"],
        )
        rotation_error = max_abs(
            profile["wheel_rotation"] - compiled["wheel_rotation"]
        )
        mesh = mesh_geometry(model, leg, config["contact_point"])
        axis_world = compiled["wheel_rotation"] @ np.asarray(
            config["contact_point"]["wheel_axis_local"]
        )
        radial_world = compiled["wheel_rotation"] @ np.asarray(
            config["contact_point"]["contact_point_at_zero_local_m"]
        )
        positive_material_velocity = np.cross(axis_world, radial_world)
        positive_no_slip_rolling = -positive_material_velocity
        expected_material_velocity = np.array(
            [-config["contact_point"]["nominal_radius_m"], 0.0, 0.0]
        )
        rolling_direction_error = max_abs(
            positive_material_velocity - expected_material_velocity
        )
        max_position = max(max_position, position_error)
        max_rotation = max(max_rotation, rotation_error)
        max_mesh_radius_deviation = max(
            max_mesh_radius_deviation, mesh["nominal_radius_deviation_m"]
        )
        max_rolling_direction_error = max(
            max_rolling_direction_error, rolling_direction_error
        )
        side_entries[side] = {
            "joint_names": leg["joint_names"],
            "joint_ids": leg["joint_ids"],
            "qpos_addresses": vector(leg["qpos"]),
            "dof_addresses": vector(leg["dofs"]),
            "local_joint_axes": [
                vector(model.jnt_axis[joint]) for joint in leg["joint_ids"]
            ],
            "calf_site": leg["config"]["calf_site"],
            "connect_site": leg["config"]["connect_site"],
            "wheel_body": leg["config"]["wheel_body"],
            "mesh": mesh,
            "positive_wheel_material_velocity_at_nominal_contact_m_per_rad": vector(
                positive_material_velocity
            ),
            "positive_no_slip_rolling_displacement_m_per_rad": vector(
                positive_no_slip_rolling
            ),
        }
    manifest = {
        "schema_version": 1,
        "profile": config["profile"],
        "mujoco_version": mujoco.__version__,
        "scene": config["scene"],
        "scene_sha256": sha256(scene_path),
        "included_model": config["included_model"],
        "included_model_sha256": sha256(model_path),
        "coordinate_space": "MuJoCo native SI; canonical conversion is not applied",
        "closure": (
            "site-to-site connect; three residual rows with numerical rank two "
            "in the planar leg"
        ),
        "contact_point": config["contact_point"],
        "sides": side_entries,
        "interpretation_limit": (
            "Nominal MuJoCo geometry only; no real-hardware dimension or "
            "contact calibration is claimed."
        ),
    }
    check = {
        "pass": (
            max_position <= thresholds["profile_model_position_m"]
            and max_rotation <= thresholds["profile_model_rotation_matrix"]
            and max_mesh_radius_deviation
            <= thresholds["mesh_nominal_radius_deviation_m"]
            and max_rolling_direction_error <= thresholds["rolling_direction"]
        ),
        "max_profile_model_position_error_m": max_position,
        "max_profile_model_rotation_matrix_error": max_rotation,
        "max_mesh_nominal_radius_deviation_m": max_mesh_radius_deviation,
        "max_rolling_direction_error": max_rolling_direction_error,
        "simulink_radius_is_not_used": (
            config["contact_point"]["nominal_radius_m"]
            != config["contact_point"]["simulink_simplified_radius_m"]
        ),
    }
    return manifest, check


def branch_solutions(
    model: mujoco.MjModel,
    leg: dict[str, Any],
    config: dict[str, Any],
) -> tuple[dict[float, np.ndarray], dict[str, float | bool | int]]:
    knees = sorted(float(value) for value in config["workspace_rad"]["knee"])
    positive = [value for value in knees if value >= 0.0]
    negative = sorted((value for value in knees if value <= 0.0), reverse=True)
    solutions: dict[float, np.ndarray] = {}
    max_residual = 0.0
    max_reference = 0.0
    max_iterations = 0
    converged = True
    for path in (positive, negative):
        seed = np.zeros(2)
        for knee in path:
            passive, info, _ = solve_passive(
                model,
                leg,
                np.array([0.0, knee, 0.0]),
                seed,
                config["solver"],
            )
            solutions[knee] = passive
            seed = passive
            expected = knee * np.asarray(
                leg["config"]["expected_passive_from_knee"]
            )
            max_residual = max(max_residual, info["max_residual_m"])
            max_reference = max(
                max_reference, float(np.max(np.abs(passive - expected)))
            )
            max_iterations = max(max_iterations, int(info["iterations"]))
            converged = converged and bool(info["converged"])
    max_reverse = 0.0
    for path in (positive, negative):
        seed = solutions[path[-1]].copy()
        for knee in reversed(path):
            passive, info, _ = solve_passive(
                model,
                leg,
                np.array([0.0, knee, 0.0]),
                seed,
                config["solver"],
            )
            max_reverse = max(
                max_reverse,
                float(np.max(np.abs(passive - solutions[knee]))),
            )
            seed = passive
            converged = converged and bool(info["converged"])
    thresholds = config["thresholds"]
    return solutions, {
        "pass": (
            converged
            and max_residual <= thresholds["closure_position_m"]
            and max_reference <= thresholds["passive_reference_rad"]
            and max_reverse <= thresholds["branch_reverse_rad"]
        ),
        "converged": converged,
        "knee_samples": len(knees),
        "max_solver_iterations": max_iterations,
        "max_closure_residual_m": max_residual,
        "max_passive_reference_error_rad": max_reference,
        "max_reverse_path_error_rad": max_reverse,
    }


def evaluate(
    model: mujoco.MjModel,
    legs: dict[str, dict[str, Any]],
    config: dict[str, Any],
    geometry_check: dict[str, Any],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    thresholds = config["thresholds"]
    branch = {}
    solutions = {}
    for side, leg in legs.items():
        solutions[side], branch[side] = branch_solutions(model, leg, config)

    maxima = {
        "closure_residual_m": 0.0,
        "fk_position_m": 0.0,
        "fk_rotation_matrix": 0.0,
        "full_jacobian": 0.0,
        "constraint_tangent": 0.0,
        "reduced_jacobian": 0.0,
        "passive_condition_number": 0.0,
        "velocity": 0.0,
        "virtual_work_nm": 0.0,
        "power_w": 0.0,
    }
    min_passive_singular = float("inf")
    fd_maxima = {
        str(epsilon): {"linear": 0.0, "angular": 0.0}
        for epsilon in config["finite_difference_eps_rad"]
    }
    rows: list[dict[str, Any]] = []
    states: dict[tuple[str, float, float, float], dict[str, np.ndarray]] = {}
    velocity = np.asarray(config["active_velocity_rad_s"])
    force = np.asarray(config["test_wrench_world"]["force_n"])
    torque = np.asarray(config["test_wrench_world"]["torque_nm"])

    workspace = itertools.product(
        config["workspace_rad"]["hip"],
        config["workspace_rad"]["knee"],
        config["workspace_rad"]["wheel"],
    )
    active_samples = [np.asarray(values, dtype=float) for values in workspace]
    for side, leg in legs.items():
        for active in active_samples:
            passive, solve_info, data = solve_passive(
                model,
                leg,
                active,
                solutions[side][float(active[1])],
                config["solver"],
            )
            if not solve_info["converged"]:
                raise AssertionError(f"{side} passive solve failed at {active}")
            profile = profile_kinematics(
                leg, active, passive, config["contact_point"]
            )
            compiled = mujoco_kinematics(
                model, data, leg, config["contact_point"]
            )
            profile_mapping, profile_reduction = reduction(
                profile["closure_jacobian"], config["solver"]["rcond"]
            )
            compiled_mapping, compiled_reduction = reduction(
                compiled["closure_jacobian"], config["solver"]["rcond"]
            )
            profile_center, profile_contact = reduced_task_jacobians(
                profile, profile_mapping
            )
            compiled_center, compiled_contact = reduced_task_jacobians(
                compiled, compiled_mapping
            )

            fk_position = max_abs(
                profile["wheel_position"] - compiled["wheel_position"],
                profile["contact_point"] - compiled["contact_point"],
                profile["calf_site"] - compiled["calf_site"],
                profile["connect_site"] - compiled["connect_site"],
            )
            fk_rotation = max_abs(
                profile["wheel_rotation"] - compiled["wheel_rotation"]
            )
            full_jacobian = max_abs(
                profile["closure_jacobian"] - compiled["closure_jacobian"],
                profile["center_linear_jacobian"]
                - compiled["center_linear_jacobian"],
                profile["center_angular_jacobian"]
                - compiled["center_angular_jacobian"],
                profile["contact_linear_jacobian"]
                - compiled["contact_linear_jacobian"],
            )
            reduced_jacobian = max_abs(
                profile_center - compiled_center,
                profile_contact - compiled_contact,
            )
            velocity_error = max_abs(
                profile_center @ velocity - compiled_center @ velocity,
                profile_contact @ velocity - compiled_contact @ velocity,
            )

            applied = np.zeros(model.nv)
            mujoco.mj_applyFT(
                model,
                data,
                force,
                torque,
                compiled["contact_point"],
                leg["wheel_body"],
                applied,
            )
            active_generalized = compiled_mapping.T @ applied[leg["dofs"]]
            reduced_generalized = compiled_contact.T @ np.r_[force, torque]
            virtual_work_error = max_abs(
                active_generalized - reduced_generalized
            )
            task_velocity = compiled_contact @ velocity
            joint_power = float(np.dot(active_generalized, velocity))
            task_power = float(
                np.dot(force, task_velocity[:3])
                + np.dot(torque, task_velocity[3:])
            )
            power_error = abs(joint_power - task_power)

            formal_linear = 0.0
            formal_angular = 0.0
            for epsilon in config["finite_difference_eps_rad"]:
                finite = finite_difference(
                    model, leg, active, passive, epsilon, config
                )
                linear_error = max_abs(
                    finite["center_linear"] - compiled_center[:3],
                    finite["contact_linear"] - compiled_contact[:3],
                )
                angular_error = max_abs(
                    finite["center_angular"] - compiled_center[3:]
                )
                fd_maxima[str(epsilon)]["linear"] = max(
                    fd_maxima[str(epsilon)]["linear"], linear_error
                )
                fd_maxima[str(epsilon)]["angular"] = max(
                    fd_maxima[str(epsilon)]["angular"], angular_error
                )
                if epsilon == config["formal_finite_difference_eps_rad"]:
                    formal_linear = linear_error
                    formal_angular = angular_error

            maxima["closure_residual_m"] = max(
                maxima["closure_residual_m"], solve_info["max_residual_m"]
            )
            maxima["fk_position_m"] = max(maxima["fk_position_m"], fk_position)
            maxima["fk_rotation_matrix"] = max(
                maxima["fk_rotation_matrix"], fk_rotation
            )
            maxima["full_jacobian"] = max(
                maxima["full_jacobian"], full_jacobian
            )
            maxima["constraint_tangent"] = max(
                maxima["constraint_tangent"],
                profile_reduction["tangent_residual"],
                compiled_reduction["tangent_residual"],
            )
            maxima["reduced_jacobian"] = max(
                maxima["reduced_jacobian"], reduced_jacobian
            )
            maxima["passive_condition_number"] = max(
                maxima["passive_condition_number"],
                compiled_reduction["passive_condition_number"],
            )
            min_passive_singular = min(
                min_passive_singular,
                compiled_reduction["passive_min_singular_value"],
            )
            maxima["velocity"] = max(maxima["velocity"], velocity_error)
            maxima["virtual_work_nm"] = max(
                maxima["virtual_work_nm"], virtual_work_error
            )
            maxima["power_w"] = max(maxima["power_w"], power_error)
            states[(side, *active)] = compiled
            rows.append(
                {
                    "side": side,
                    "hip_rad": float(active[0]),
                    "knee_rad": float(active[1]),
                    "wheel_rad": float(active[2]),
                    "connect1_rad": float(passive[0]),
                    "connect2_rad": float(passive[1]),
                    "solver_iterations": int(solve_info["iterations"]),
                    "closure_residual_m": solve_info["max_residual_m"],
                    "passive_min_singular_value": compiled_reduction[
                        "passive_min_singular_value"
                    ],
                    "passive_condition_number": compiled_reduction[
                        "passive_condition_number"
                    ],
                    "fk_position_error_m": fk_position,
                    "fk_rotation_matrix_error": fk_rotation,
                    "full_jacobian_error": full_jacobian,
                    "constraint_tangent_residual": max(
                        profile_reduction["tangent_residual"],
                        compiled_reduction["tangent_residual"],
                    ),
                    "reduced_jacobian_error": reduced_jacobian,
                    "finite_difference_linear_error": formal_linear,
                    "finite_difference_angular_error": formal_angular,
                    "velocity_error": velocity_error,
                    "virtual_work_error_nm": virtual_work_error,
                    "power_error_w": power_error,
                }
            )

    mirror_position = 0.0
    shared_frame_rotation = 0.0
    for active in active_samples:
        left = states[("left", *active)]
        right = states[("right", *active)]
        mirror = np.diag([1.0, -1.0, 1.0])
        mirror_position = max(
            mirror_position,
            max_abs(
                right["wheel_position"] - mirror @ left["wheel_position"]
            ),
            max_abs(
                right["contact_point"] - mirror @ left["contact_point"]
            ),
        )
        shared_frame_rotation = max(
            shared_frame_rotation,
            max_abs(
                right["wheel_rotation"] - left["wheel_rotation"]
            ),
        )

    formal = fd_maxima[str(config["formal_finite_difference_eps_rad"])]
    checks = {
        "geometry_profile": geometry_check,
        "assembly_branch": {
            "pass": all(result["pass"] for result in branch.values()),
            "sides": branch,
        },
        "workspace": {
            "pass": (
                maxima["closure_residual_m"] <= thresholds["closure_position_m"]
                and min_passive_singular
                >= thresholds["passive_block_min_singular_value"]
                and maxima["passive_condition_number"]
                <= thresholds["passive_block_condition_number"]
            ),
            "samples_per_side": len(active_samples),
            "total_samples": len(rows),
            "max_closure_residual_m": maxima["closure_residual_m"],
            "min_passive_block_singular_value": min_passive_singular,
            "max_passive_block_condition_number": maxima[
                "passive_condition_number"
            ],
        },
        "fk_and_full_jacobian": {
            "pass": (
                maxima["fk_position_m"] <= thresholds["fk_position_m"]
                and maxima["fk_rotation_matrix"]
                <= thresholds["fk_rotation_matrix"]
                and maxima["full_jacobian"] <= thresholds["full_jacobian"]
            ),
            "max_position_error_m": maxima["fk_position_m"],
            "max_rotation_matrix_error": maxima["fk_rotation_matrix"],
            "max_full_jacobian_error": maxima["full_jacobian"],
        },
        "reduced_jacobian": {
            "pass": (
                maxima["constraint_tangent"]
                <= thresholds["constraint_tangent"]
                and maxima["reduced_jacobian"]
                <= thresholds["reduced_jacobian"]
                and formal["linear"]
                <= thresholds["finite_difference_linear"]
                and formal["angular"]
                <= thresholds["finite_difference_angular"]
            ),
            "max_constraint_tangent_residual": maxima[
                "constraint_tangent"
            ],
            "max_analytic_mujoco_reduced_jacobian_error": maxima[
                "reduced_jacobian"
            ],
            "finite_difference_by_epsilon": fd_maxima,
        },
        "velocity_and_virtual_work": {
            "pass": (
                maxima["velocity"] <= thresholds["velocity"]
                and maxima["virtual_work_nm"]
                <= thresholds["virtual_work_nm"]
                and maxima["power_w"] <= thresholds["power_w"]
            ),
            "max_velocity_error": maxima["velocity"],
            "max_virtual_work_error_nm": maxima["virtual_work_nm"],
            "max_power_error_w": maxima["power_w"],
        },
        "left_right_symmetry": {
            "pass": (
                mirror_position <= thresholds["mirror_position_m"]
                and shared_frame_rotation
                <= thresholds["left_right_frame_rotation_matrix"]
            ),
            "max_mirrored_position_error_m": mirror_position,
            "frame_contract": (
                "positions mirror across the XZ plane; left/right wheel body "
                "frames share the same right-handed axes"
            ),
            "max_shared_frame_rotation_matrix_error": shared_frame_rotation,
        },
    }
    return checks, rows


def write_workspace(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", default="nominal")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    config_path = (repo_root / args.config).resolve()
    output_dir = (repo_root / args.output_dir).resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise AssertionError(
            f"Refusing to overwrite non-empty output directory: {output_dir}"
        )
    config = json.loads(config_path.read_text(encoding="utf-8"))
    if config["profile"] != args.profile:
        raise AssertionError(
            f"Profile {args.profile!r} does not match config {config['profile']!r}"
        )
    if mujoco.__version__ != config["mujoco_version"]:
        raise AssertionError(
            f"Expected MuJoCo {config['mujoco_version']}, got {mujoco.__version__}"
        )
    scene_path = (repo_root / config["scene"]).resolve()
    included_model_path = (repo_root / config["included_model"]).resolve()
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    legs = {side: context(model, side, config) for side in config["sides"]}
    manifest, geometry_check = geometry_manifest(
        model, legs, config, scene_path, included_model_path
    )
    checks, rows = evaluate(model, legs, config, geometry_check)
    repeat_checks, repeat_rows = evaluate(
        model, legs, config, geometry_check
    )
    first_numeric = np.asarray(
        [
            [value for value in row.values() if isinstance(value, (int, float))]
            for row in rows
        ]
    )
    repeat_numeric = np.asarray(
        [
            [value for value in row.values() if isinstance(value, (int, float))]
            for row in repeat_rows
        ]
    )
    determinism_difference = float(
        np.max(np.abs(first_numeric - repeat_numeric))
    )
    checks["determinism"] = {
        "pass": (
            determinism_difference
            <= config["thresholds"]["determinism_absolute"]
            and checks == repeat_checks
        ),
        "max_absolute_numeric_difference": determinism_difference,
        "summary_exact_match": checks == repeat_checks,
    }
    overall_pass = all(check["pass"] for check in checks.values())
    runner_path = Path(__file__).resolve()
    run_manifest = {
        "schema_version": 1,
        "run_id": output_dir.name,
        "profile": config["profile"],
        "mujoco_version": mujoco.__version__,
        "seed": config["seed"],
        "config_path": str(args.config),
        "config_sha256": sha256(config_path),
        "scene_path": config["scene"],
        "scene_sha256": sha256(scene_path),
        "included_model_path": config["included_model"],
        "included_model_sha256": sha256(included_model_path),
        "runner_path": str(runner_path.relative_to(repo_root)),
        "runner_sha256": sha256(runner_path),
        "solver": config["solver"],
        "workspace_rad": config["workspace_rad"],
        "thresholds": config["thresholds"],
        "supersedes": None,
        "hardware_data_used": False,
    }
    result = {
        "schema_version": 1,
        "overall_pass": overall_pass,
        "run_manifest": run_manifest,
        "checks": checks,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "phase15_validation.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "geometry_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "run_manifest.json").write_text(
        json.dumps(run_manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    write_workspace(output_dir / "workspace.csv", rows)

    print(
        f"Phase 15 MuJoCo closed-chain kinematics: "
        f"{'PASS' if overall_pass else 'FAIL'}"
    )
    for name, check in checks.items():
        print(f"  {name}: {'PASS' if check['pass'] else 'FAIL'}")
    print(f"  evidence: {output_dir.relative_to(repo_root)}")
    if not overall_pass:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
