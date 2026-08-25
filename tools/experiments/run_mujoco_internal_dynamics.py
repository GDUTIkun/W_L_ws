"""Run the frozen Phase-14 MuJoCo-only validation sweep."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


DEFAULT_CONFIG = "simulation/mujoco/config/phase14_validation.json"
DEFAULT_OUTPUT = (
    "docs/workflow/phases/14-mujoco-internal-dynamics-validation/"
    "evidence/automated"
)
SIDES = ("left", "right")


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise AssertionError(f"Missing {kind.name} named {name!r}")
    return result


def object_name(model: mujoco.MjModel, kind: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, kind, index) or f"<{kind.name}:{index}>"


def rx(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rz(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def vector(values: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(values).ravel()]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def set_equalities(
    model: mujoco.MjModel, data: mujoco.MjData, active_names: set[str]
) -> None:
    for index in range(model.neq):
        name = object_name(model, mujoco.mjtObj.mjOBJ_EQUALITY, index)
        data.eq_active[index] = int(name in active_names)


def set_joint_values(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_order: list[str],
    values: list[float],
) -> None:
    data.qpos[:] = model.qpos0
    for name, value in zip(joint_order, values, strict=True):
        joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[model.jnt_qposadr[joint]] = value


def driven_dofs(model: mujoco.MjModel, joint_order: list[str]) -> np.ndarray:
    return np.array(
        [
            model.jnt_dofadr[
                object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in joint_order
        ],
        dtype=int,
    )


def driven_qpos(model: mujoco.MjModel, joint_order: list[str]) -> np.ndarray:
    return np.array(
        [
            model.jnt_qposadr[
                object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in joint_order
        ],
        dtype=int,
    )


def reference_leg(side: str, hip: float, knee: float, wheel: float) -> dict[str, np.ndarray]:
    sign = 1.0 if side == "left" else -1.0
    hip_position = np.array([-0.0775, sign * 0.04425, 0.58])
    thigh_rotation = rx(-np.pi / 2.0) @ rz(-np.pi / 2.0) @ rz(hip)
    knee_position = hip_position + thigh_rotation @ np.array(
        [-0.04350, -0.17467, sign * 0.12525]
    )
    calf_rotation = thigh_rotation @ rz(np.pi) @ rz(knee)
    wheel_position = knee_position + calf_rotation @ np.array(
        [0.14297, -0.17368, sign * 0.04280]
    )
    wheel_rotation = calf_rotation @ rz(wheel)
    axis = np.array([0.0, 1.0, 0.0])
    linear_jacobian = np.column_stack(
        (
            np.cross(axis, wheel_position - hip_position),
            np.cross(axis, wheel_position - knee_position),
            np.zeros(3),
        )
    )
    angular_jacobian = np.column_stack((axis, axis, axis))
    return {
        "hip_position": hip_position,
        "knee_position": knee_position,
        "wheel_position": wheel_position,
        "wheel_rotation": wheel_rotation,
        "linear_jacobian": linear_jacobian,
        "angular_jacobian": angular_jacobian,
    }


def compiled_potential(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    gravity = -float(model.opt.gravity[2])
    return float(np.sum(model.body_mass * data.xipos[:, 2]) * gravity)


def full_mass_matrix(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    result = np.empty((model.nv, model.nv))
    mujoco.mj_fullM(model, result, data.qM)
    return result


def equality_residual(data: mujoco.MjData) -> tuple[float, float]:
    if data.nefc == 0:
        return 0.0, 0.0
    return (
        float(np.max(np.abs(data.efc_pos[: data.nefc]))),
        float(np.max(np.abs(data.efc_vel[: data.nefc]))),
    )


def check_fixture(
    model: mujoco.MjModel,
    single_leg_model: mujoco.MjModel,
    config: dict[str, Any],
) -> dict[str, Any]:
    equality_names = [
        object_name(model, mujoco.mjtObj.mjOBJ_EQUALITY, index)
        for index in range(model.neq)
    ]
    actuator_joints = []
    for actuator in range(model.nu):
        joint = int(model.actuator_trnid[actuator, 0])
        actuator_joints.append(
            object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        )
    contact_disabled = bool(
        int(model.opt.disableflags) & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    )
    inertial_difference = 0.0
    for name in (
        "left_thigh_body",
        "left_calf_body",
        "left_wheel_body",
        "left_connect1_body",
        "left_connect2_body",
    ):
        full_body = object_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        fixture_body = object_id(
            single_leg_model, mujoco.mjtObj.mjOBJ_BODY, name
        )
        inertial_difference = max(
            inertial_difference,
            abs(
                float(model.body_mass[full_body])
                - float(single_leg_model.body_mass[fixture_body])
            ),
            float(
                np.max(
                    np.abs(
                        model.body_ipos[full_body]
                        - single_leg_model.body_ipos[fixture_body]
                    )
                )
            ),
            float(
                np.max(
                    np.abs(
                        model.body_inertia[full_body]
                        - single_leg_model.body_inertia[fixture_body]
                    )
                )
            ),
            1.0
            - abs(
                float(
                    np.dot(
                        model.body_iquat[full_body],
                        single_leg_model.body_iquat[fixture_body],
                    )
                )
            ),
        )
    single_leg_data = mujoco.MjData(single_leg_model)
    mujoco.mj_forward(single_leg_model, single_leg_data)
    single_leg_jacobian = np.asarray(single_leg_data.efc_J).reshape(
        single_leg_data.nefc, single_leg_model.nv
    )
    single_leg_constraint_rank = int(np.linalg.matrix_rank(single_leg_jacobian))
    passed = (
        model.nq == 17
        and model.nv == 16
        and model.nu == 6
        and equality_names
        == ["base_weld", "left_leg_closure", "right_leg_closure"]
        and actuator_joints == config["joint_order"]
        and contact_disabled
        and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor") < 0
        and single_leg_model.nq == 5
        and single_leg_model.nv == 5
        and single_leg_model.nu == 3
        and single_leg_model.neq == 1
        and object_name(
            single_leg_model, mujoco.mjtObj.mjOBJ_EQUALITY, 0
        )
        == "left_leg_closure"
        and single_leg_constraint_rank == 2
        and inertial_difference <= 1.0e-14
    )
    return {
        "pass": passed,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "equalities": equality_names,
        "actuator_joints": actuator_joints,
        "contact_disabled": contact_disabled,
        "floor_absent": True,
        "single_leg_max_compiled_inertial_difference": inertial_difference,
        "single_leg_dimensions": {
            "nq": single_leg_model.nq,
            "nv": single_leg_model.nv,
            "nu": single_leg_model.nu,
            "neq": single_leg_model.neq,
        },
        "single_leg_constraint_rank": single_leg_constraint_rank,
        "single_leg_independent_dofs": single_leg_model.nv
        - single_leg_constraint_rank,
        "single_leg_mode": "formal fixed-base/contact-free five-body closed leg with three actuators and compiled imported inertials",
        "full_constraint_mode": "base_weld and both named leg closures active",
    }


def check_kinematics(
    model: mujoco.MjModel, config: dict[str, Any]
) -> dict[str, Any]:
    max_position = 0.0
    max_rotation = 0.0
    max_jacobian = 0.0
    worst = ""
    for sample_name, values in config["samples_rad"].items():
        data = mujoco.MjData(model)
        set_joint_values(model, data, config["joint_order"], values)
        set_equalities(model, data, {"base_weld"})
        mujoco.mj_forward(model, data)
        for side_index, side in enumerate(SIDES):
            hip, knee, wheel = values[3 * side_index : 3 * side_index + 3]
            reference = reference_leg(side, hip, knee, wheel)
            knee_joint = object_id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_knee_joint"
            )
            wheel_body = object_id(
                model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel_body"
            )
            position_error = max(
                float(
                    np.max(
                        np.abs(
                            data.xanchor[knee_joint]
                            - reference["knee_position"]
                        )
                    )
                ),
                float(
                    np.max(
                        np.abs(
                            data.xpos[wheel_body]
                            - reference["wheel_position"]
                        )
                    )
                ),
            )
            rotation_error = float(
                np.max(
                    np.abs(
                        data.xmat[wheel_body].reshape(3, 3)
                        - reference["wheel_rotation"]
                    )
                )
            )
            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacBody(model, data, jacp, jacr, wheel_body)
            side_names = config["joint_order"][
                3 * side_index : 3 * side_index + 3
            ]
            columns = driven_dofs(model, side_names)
            jacobian_error = max(
                float(
                    np.max(
                        np.abs(
                            jacp[:, columns]
                            - reference["linear_jacobian"]
                        )
                    )
                ),
                float(
                    np.max(
                        np.abs(
                            jacr[:, columns]
                            - reference["angular_jacobian"]
                        )
                    )
                ),
            )
            local_worst = max(position_error, rotation_error, jacobian_error)
            if local_worst >= max(max_position, max_rotation, max_jacobian):
                worst = f"{sample_name}:{side}"
            max_position = max(max_position, position_error)
            max_rotation = max(max_rotation, rotation_error)
            max_jacobian = max(max_jacobian, jacobian_error)
    thresholds = config["thresholds"]
    rolling_direction = vector(
        -np.cross(np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, -1.0]))
    )
    return {
        "pass": (
            max_position <= thresholds["fk_position_m"]
            and max_rotation <= thresholds["fk_rotation_matrix"]
            and max_jacobian <= thresholds["jacobian"]
            and rolling_direction == [1.0, -0.0, -0.0]
        ),
        "samples": len(config["samples_rad"]),
        "max_position_error_m": max_position,
        "max_rotation_matrix_error": max_rotation,
        "max_jacobian_error": max_jacobian,
        "worst_sample": worst,
        "positive_wheel_rolling_direction_flu": rolling_direction,
    }


def check_gravity(model: mujoco.MjModel, config: dict[str, Any]) -> dict[str, Any]:
    indices = driven_dofs(model, config["joint_order"])
    max_force_error = 0.0
    max_static_acceleration = 0.0
    worst = ""
    epsilon = 1.0e-6
    freefall_z = None
    for sample_name, values in config["samples_rad"].items():
        data = mujoco.MjData(model)
        set_joint_values(model, data, config["joint_order"], values)
        set_equalities(model, data, set())
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        bias = data.qfrc_bias.copy()
        numerical = np.zeros(len(indices))
        for output_index, dof in enumerate(indices):
            qpos_index = int(
                model.jnt_qposadr[
                    object_id(
                        model,
                        mujoco.mjtObj.mjOBJ_JOINT,
                        config["joint_order"][output_index],
                    )
                ]
            )
            potentials = []
            for delta in (-epsilon, epsilon):
                probe = mujoco.MjData(model)
                probe.qpos[:] = data.qpos
                probe.qpos[qpos_index] += delta
                set_equalities(model, probe, set())
                mujoco.mj_forward(model, probe)
                potentials.append(compiled_potential(model, probe))
            numerical[output_index] = (
                potentials[1] - potentials[0]
            ) / (2.0 * epsilon)
        force_error = float(np.max(np.abs(bias[indices] - numerical)))
        equilibrium = mujoco.MjData(model)
        equilibrium.qpos[:] = data.qpos
        equilibrium.qvel[:] = 0.0
        equilibrium.qfrc_applied[:] = bias
        set_equalities(model, equilibrium, set())
        mujoco.mj_forward(model, equilibrium)
        static_error = float(np.max(np.abs(equilibrium.qacc)))
        if force_error >= max_force_error:
            worst = sample_name
        max_force_error = max(max_force_error, force_error)
        max_static_acceleration = max(max_static_acceleration, static_error)
        if sample_name == "zero":
            falling = mujoco.MjData(model)
            falling.qpos[:] = data.qpos
            set_equalities(model, falling, set())
            mujoco.mj_forward(model, falling)
            freefall_z = float(falling.qacc[2])
    thresholds = config["thresholds"]
    return {
        "pass": (
            max_force_error <= thresholds["gravity_generalized_force_nm"]
            and max_static_acceleration <= thresholds["static_acceleration"]
            and freefall_z is not None
            and abs(freefall_z + 9.81) <= 1.0e-10
        ),
        "max_generalized_force_error_nm": max_force_error,
        "max_static_acceleration": max_static_acceleration,
        "unconstrained_base_freefall_z_m_s2": freefall_z,
        "worst_sample": worst,
    }


def check_mass(model: mujoco.MjModel, config: dict[str, Any]) -> dict[str, Any]:
    driven = driven_dofs(model, config["joint_order"])
    max_symmetry = 0.0
    min_eigenvalue = float("inf")
    max_condition = 0.0
    min_driven_eigenvalue = float("inf")
    worst = ""
    for sample_name, values in config["samples_rad"].items():
        data = mujoco.MjData(model)
        set_joint_values(model, data, config["joint_order"], values)
        set_equalities(model, data, set())
        mujoco.mj_forward(model, data)
        mass = full_mass_matrix(model, data)
        symmetry = float(np.max(np.abs(mass - mass.T)))
        eigenvalues = np.linalg.eigvalsh(mass)
        condition = float(np.linalg.cond(mass))
        driven_eigenvalue = float(
            np.min(np.linalg.eigvalsh(mass[np.ix_(driven, driven)]))
        )
        if condition >= max_condition:
            worst = sample_name
        max_symmetry = max(max_symmetry, symmetry)
        min_eigenvalue = min(min_eigenvalue, float(eigenvalues[0]))
        min_driven_eigenvalue = min(min_driven_eigenvalue, driven_eigenvalue)
        max_condition = max(max_condition, condition)

    constrained = mujoco.MjData(model)
    set_equalities(
        model,
        constrained,
        {"base_weld", "left_leg_closure", "right_leg_closure"},
    )
    mujoco.mj_forward(model, constrained)
    jacobian = np.asarray(constrained.efc_J).reshape(constrained.nefc, model.nv)
    _, singular_values, vh = np.linalg.svd(jacobian, full_matrices=True)
    rank_tolerance = max(jacobian.shape) * np.finfo(float).eps * singular_values[0]
    rank = int(np.sum(singular_values > rank_tolerance))
    nullspace = vh[rank:].T
    constrained_mass = nullspace.T @ full_mass_matrix(model, constrained) @ nullspace
    constrained_min_eigenvalue = float(np.min(np.linalg.eigvalsh(constrained_mass)))
    thresholds = config["thresholds"]
    return {
        "pass": (
            max_symmetry <= thresholds["mass_symmetry"]
            and min_eigenvalue >= thresholds["mass_min_eigenvalue"]
            and min_driven_eigenvalue >= thresholds["mass_min_eigenvalue"]
            and max_condition <= thresholds["mass_condition_number"]
            and nullspace.shape[1] == 6
            and constrained_min_eigenvalue > 0.0
        ),
        "unconstrained_dimension": model.nv,
        "max_symmetry_error": max_symmetry,
        "min_full_eigenvalue": min_eigenvalue,
        "min_driven_submatrix_eigenvalue": min_driven_eigenvalue,
        "max_condition_number": max_condition,
        "worst_sample": worst,
        "constraint_jacobian_rank": rank,
        "constrained_nullspace_dimension": int(nullspace.shape[1]),
        "constrained_min_eigenvalue": constrained_min_eigenvalue,
    }


def check_forward_inverse(
    model: mujoco.MjModel, config: dict[str, Any]
) -> dict[str, Any]:
    driven = driven_dofs(model, config["joint_order"])
    max_acceleration_error = 0.0
    max_equation_error = 0.0
    worst = ""
    rng = np.random.default_rng(config["seed"])
    for sample_name, values in config["samples_rad"].items():
        inverse = mujoco.MjData(model)
        set_joint_values(model, inverse, config["joint_order"], values)
        set_equalities(model, inverse, set())
        inverse.qvel[:] = rng.uniform(-0.2, 0.2, model.nv)
        inverse.qvel[driven] = config["inverse_velocity_rad_s"]
        desired_acceleration = rng.uniform(-0.5, 0.5, model.nv)
        desired_acceleration[driven] = config["inverse_acceleration_rad_s2"]
        inverse.qacc[:] = desired_acceleration
        mujoco.mj_inverse(model, inverse)
        generalized_force = inverse.qfrc_inverse.copy()
        mass = full_mass_matrix(model, inverse)
        equation_error = float(
            np.max(
                np.abs(
                    generalized_force
                    - (
                        mass @ desired_acceleration
                        + inverse.qfrc_bias
                        - inverse.qfrc_passive
                    )
                )
            )
        )

        forward = mujoco.MjData(model)
        forward.qpos[:] = inverse.qpos
        forward.qvel[:] = inverse.qvel
        forward.qfrc_applied[:] = generalized_force
        set_equalities(model, forward, set())
        mujoco.mj_forward(model, forward)
        acceleration_error = float(
            np.max(np.abs(forward.qacc - desired_acceleration))
        )
        if acceleration_error >= max_acceleration_error:
            worst = sample_name
        max_acceleration_error = max(max_acceleration_error, acceleration_error)
        max_equation_error = max(max_equation_error, equation_error)
    threshold = config["thresholds"]["forward_inverse_acceleration"]
    return {
        "pass": max(max_acceleration_error, max_equation_error) <= threshold,
        "max_acceleration_round_trip_error": max_acceleration_error,
        "max_mass_equation_error": max_equation_error,
        "worst_sample": worst,
    }


def check_coupling(model: mujoco.MjModel, config: dict[str, Any]) -> dict[str, Any]:
    driven = driven_dofs(model, config["joint_order"][:3])
    original_gravity = model.opt.gravity.copy()
    model.opt.gravity[:] = 0.0
    matrix = np.zeros((len(driven), model.nu))
    mapping_error = 0.0
    for actuator in range(model.nu):
        data = mujoco.MjData(model)
        set_equalities(model, data, set())
        data.ctrl[actuator] = 1.0
        mujoco.mj_forward(model, data)
        matrix[:, actuator] = data.qacc[driven]
        expected_force = np.zeros(model.nv)
        expected_force[driven[actuator]] = 1.0
        mapping_error = max(
            mapping_error,
            float(np.max(np.abs(data.qfrc_actuator - expected_force))),
        )
    model.opt.gravity[:] = original_gravity
    symmetry_error = float(np.max(np.abs(matrix - matrix.T)))
    minimum_diagonal = float(np.min(np.diag(matrix)))
    within_side_off_diagonal = float(
        np.max(np.abs(matrix - np.diag(np.diag(matrix))))
    )
    thresholds = config["thresholds"]
    return {
        "pass": (
            mapping_error <= 1.0e-12
            and minimum_diagonal > 0.0
            and within_side_off_diagonal > 1.0e-4
            and symmetry_error <= thresholds["coupling_reciprocity_rad_s2"]
        ),
        "native_unit_ctrl_to_driven_qacc_rad_s2": matrix.tolist(),
        "actuator_generalized_force_mapping_error": mapping_error,
        "minimum_diagonal_response_rad_s2": minimum_diagonal,
        "max_within_side_off_diagonal_rad_s2": within_side_off_diagonal,
        "reciprocity_error_rad_s2": symmetry_error,
        "sign_interpretation": "positive native ctrl produces positive native joint acceleration; canonical TorqueCommand is negated by Adapter",
    }


def check_constraints(
    model: mujoco.MjModel, config: dict[str, Any]
) -> dict[str, Any]:
    original_gravity = model.opt.gravity.copy()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    set_equalities(
        model,
        data,
        {"base_weld", "left_leg_closure", "right_leg_closure"},
    )
    mujoco.mj_forward(model, data)
    max_position = 0.0
    max_velocity = 0.0
    max_solver_iterations = 0
    finite = True
    for _ in range(100):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        position, velocity = equality_residual(data)
        max_position = max(max_position, position)
        max_velocity = max(max_velocity, velocity)
        max_solver_iterations = max(
            max_solver_iterations, int(np.max(data.solver_niter))
        )
        finite = finite and bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and np.all(np.isfinite(data.qacc))
        )
    model.opt.gravity[:] = original_gravity
    thresholds = config["thresholds"]
    return {
        "pass": (
            finite
            and max_position <= thresholds["constraint_position_m"]
            and max_velocity <= thresholds["constraint_velocity_m_s"]
        ),
        "mode": "complete double-leg model, base weld and both loop closures active, gravity/contact/input disabled",
        "steps": 100,
        "finite": finite,
        "max_constraint_position_m": max_position,
        "max_constraint_velocity_m_s": max_velocity,
        "max_solver_iterations": max_solver_iterations,
    }


def check_energy(model: mujoco.MjModel, config: dict[str, Any]) -> dict[str, Any]:
    original_gravity = model.opt.gravity.copy()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    set_joint_values(
        model,
        data,
        config["joint_order"][:3],
        config["samples_rad"]["typical_a"][:3],
    )
    set_equalities(model, data, set())
    mujoco.mj_forward(model, data)
    initial_energy = float(np.sum(data.energy))
    work = 0.0
    max_absolute_balance = 0.0
    max_scale = 1.0e-9
    steps = config["replay"]["steps"]
    amplitude = config["replay"]["torque_amplitude_nm"]
    frequency = config["replay"]["frequency_hz"]
    timestep = float(model.opt.timestep)
    for step in range(steps):
        phase = 2.0 * np.pi * frequency * step * timestep
        data.ctrl[:] = amplitude * np.sin(
            phase + np.arange(model.nu) * np.pi / 5.0
        )
        power_before = float(np.dot(data.ctrl, data.actuator_velocity))
        mujoco.mj_step(model, data)
        power_after = float(np.dot(data.ctrl, data.actuator_velocity))
        work += 0.5 * (power_before + power_after) * timestep
        energy_change = float(np.sum(data.energy)) - initial_energy
        balance = energy_change - work
        max_absolute_balance = max(max_absolute_balance, abs(balance))
        max_scale = max(max_scale, abs(work), abs(energy_change))
    model.opt.gravity[:] = original_gravity
    relative = max_absolute_balance / max_scale
    return {
        "pass": (
            np.isfinite(relative)
            and relative <= config["thresholds"]["energy_balance_relative"]
        ),
        "steps": steps,
        "gravity_disabled": True,
        "initial_mechanical_energy_j": initial_energy,
        "final_mechanical_energy_j": float(np.sum(data.energy)),
        "integrated_actuator_work_j": work,
        "max_absolute_balance_error_j": max_absolute_balance,
        "max_relative_balance_error": relative,
    }


def run_replay(
    model: mujoco.MjModel, config: dict[str, Any]
) -> tuple[np.ndarray, list[list[float]], dict[str, float]]:
    original_gravity = model.opt.gravity.copy()
    model.opt.gravity[:] = 0.0
    data = mujoco.MjData(model)
    set_equalities(model, data, set())
    mujoco.mj_forward(model, data)
    joint_order = config["joint_order"][:3]
    qpos_indices = driven_qpos(model, joint_order)
    dof_indices = driven_dofs(model, joint_order)
    amplitude = config["replay"]["torque_amplitude_nm"]
    frequency = config["replay"]["frequency_hz"]
    timestep = float(model.opt.timestep)
    rows: list[list[float]] = []
    samples = []
    max_constraint_position = 0.0
    max_constraint_velocity = 0.0
    max_solver_iterations = 0
    for step in range(config["replay"]["steps"]):
        phase = 2.0 * np.pi * frequency * step * timestep
        data.ctrl[:] = amplitude * np.sin(
            phase + np.arange(model.nu) * np.pi / 5.0
        )
        mujoco.mj_step(model, data)
        position_residual, velocity_residual = 0.0, 0.0
        max_constraint_position = max(max_constraint_position, position_residual)
        max_constraint_velocity = max(max_constraint_velocity, velocity_residual)
        max_solver_iterations = max(
            max_solver_iterations, int(np.max(data.solver_niter))
        )
        q = data.qpos[qpos_indices].copy()
        dq = data.qvel[dof_indices].copy()
        qdd = data.qacc[dof_indices].copy()
        sample = np.r_[
            data.time,
            q,
            dq,
            qdd,
            data.ctrl.copy(),
            position_residual,
            velocity_residual,
            np.sum(data.energy),
        ]
        samples.append(sample)
        rows.append(vector(sample))
    metrics = {
        "max_constraint_position_m": max_constraint_position,
        "max_constraint_velocity_m_s": max_constraint_velocity,
        "max_solver_iterations": float(max_solver_iterations),
        "max_abs_joint_position_rad": float(
            np.max(np.abs(np.asarray(samples)[:, 1:4]))
        ),
        "max_abs_joint_velocity_rad_s": float(
            np.max(np.abs(np.asarray(samples)[:, 4:7]))
        ),
        "max_abs_joint_acceleration_rad_s2": float(
            np.max(np.abs(np.asarray(samples)[:, 7:10]))
        ),
    }
    model.opt.gravity[:] = original_gravity
    return np.asarray(samples), rows, metrics


def check_replay(
    model: mujoco.MjModel, config: dict[str, Any]
) -> tuple[dict[str, Any], list[list[float]]]:
    first, rows, metrics = run_replay(model, config)
    second, _, _ = run_replay(model, config)
    difference = float(np.max(np.abs(first - second)))
    thresholds = config["thresholds"]
    finite = bool(np.all(np.isfinite(first)))
    passed = (
        finite
        and difference <= thresholds["determinism_absolute"]
        and metrics["max_constraint_position_m"]
        <= thresholds["constraint_position_m"]
        and metrics["max_constraint_velocity_m_s"]
        <= thresholds["constraint_velocity_m_s"]
        and metrics["max_abs_joint_position_rad"]
        <= thresholds["bounded_joint_position_rad"]
        and metrics["max_abs_joint_velocity_rad_s"]
        <= thresholds["bounded_joint_velocity_rad_s"]
        and metrics["max_abs_joint_acceleration_rad_s2"]
        <= thresholds["bounded_joint_acceleration_rad_s2"]
    )
    return {
        "pass": passed,
        "finite": finite,
        "determinism_max_absolute_difference": difference,
        **metrics,
    }, rows


def parameter_manifest(
    model: mujoco.MjModel,
    scene_path: Path,
    included_model_path: Path,
    scene_label: str,
    included_model_label: str,
) -> dict[str, Any]:
    bodies = []
    for body in range(1, model.nbody):
        bodies.append(
            {
                "name": object_name(model, mujoco.mjtObj.mjOBJ_BODY, body),
                "mass_kg": float(model.body_mass[body]),
                "local_com_m": vector(model.body_ipos[body]),
                "principal_inertia_kg_m2": vector(model.body_inertia[body]),
                "inertial_quaternion_wxyz": vector(model.body_iquat[body]),
                "mass_source": "imported nominal geom mass",
                "com_inertia_source": "derived by MuJoCo 3.7.0 from mesh geometry and imported nominal geom masses",
                "hardware_status": "unverified",
            }
        )
    joints = []
    for joint in range(model.njnt):
        dof = int(model.jnt_dofadr[joint])
        if dof < 0:
            continue
        joints.append(
            {
                "name": object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint),
                "damping": float(model.dof_damping[dof]),
                "frictionloss": float(model.dof_frictionloss[dof]),
                "armature": float(model.dof_armature[dof]),
                "source": "MuJoCo default because MJCF does not specify a value",
                "hardware_status": "unknown; requires later identification",
            }
        )
    actuators = []
    for actuator in range(model.nu):
        joint = int(model.actuator_trnid[actuator, 0])
        actuators.append(
            {
                "name": object_name(
                    model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator
                ),
                "joint": object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint),
                "gear": vector(model.actuator_gear[actuator]),
                "source": "Phase 04 unit-gear interface actuator",
                "hardware_status": "not a calibrated motor model",
            }
        )
    return {
        "schema_version": 1,
        "mujoco_version": mujoco.__version__,
        "source_scene": scene_label,
        "source_scene_sha256": sha256(scene_path),
        "included_model": included_model_label,
        "included_model_sha256": sha256(included_model_path),
        "compiled_dimensions": {
            "nq": model.nq,
            "nv": model.nv,
            "nu": model.nu,
            "neq": model.neq,
            "nbody": model.nbody,
        },
        "bodies": bodies,
        "joints": joints,
        "actuators": actuators,
        "solver": {
            "timestep_s": float(model.opt.timestep),
            "gravity_m_s2": vector(model.opt.gravity),
            "integrator": int(model.opt.integrator),
            "solver": int(model.opt.solver),
            "iterations": int(model.opt.iterations),
            "tolerance": float(model.opt.tolerance),
            "source": "versioned Phase 14 fixture; nominal numerical configuration",
        },
        "interpretation_limit": "Internal-consistency evidence only; no value is validated against real hardware.",
    }


def write_csv(
    path: Path, rows: list[list[float]], joint_order: list[str]
) -> None:
    header = ["time_s"]
    for prefix, unit in (
        ("q", "rad"),
        ("dq", "rad_s"),
        ("qdd", "rad_s2"),
        ("ctrl_native", "nm"),
    ):
        header.extend(f"{prefix}_{name}_{unit}" for name in joint_order)
    header.extend(
        [
            "constraint_position_max_m",
            "constraint_velocity_max_m_s",
            "mechanical_energy_j",
        ]
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    config_path = (repo_root / args.config).resolve()
    output_dir = (repo_root / args.output_dir).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    scene_path = (repo_root / config["scene"]).resolve()
    included_model_path = (repo_root / config["included_model"]).resolve()
    single_leg_scene_path = (repo_root / config["single_leg_scene"]).resolve()
    if mujoco.__version__ != config["mujoco_version"]:
        raise AssertionError(
            f"Expected MuJoCo {config['mujoco_version']}, got {mujoco.__version__}"
        )
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    single_leg_model = mujoco.MjModel.from_xml_path(str(single_leg_scene_path))

    checks: dict[str, Any] = {
        "fixture": check_fixture(model, single_leg_model, config),
        "kinematics": check_kinematics(model, config),
        "gravity": check_gravity(model, config),
        "mass_matrix": check_mass(model, config),
        "forward_inverse": check_forward_inverse(model, config),
        "constraints": check_constraints(model, config),
        "coupling": check_coupling(single_leg_model, config),
        "energy": check_energy(single_leg_model, config),
    }
    checks["replay"], replay_rows = check_replay(single_leg_model, config)
    overall_pass = all(check["pass"] for check in checks.values())
    result = {
        "schema_version": 1,
        "overall_pass": overall_pass,
        "config_path": str(args.config),
        "config_sha256": sha256(config_path),
        "scene_path": config["scene"],
        "scene_sha256": sha256(scene_path),
        "included_model_path": config["included_model"],
        "included_model_sha256": sha256(included_model_path),
        "single_leg_scene_path": config["single_leg_scene"],
        "single_leg_scene_sha256": sha256(single_leg_scene_path),
        "mujoco_version": mujoco.__version__,
        "checks": checks,
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "phase14_validation.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    (output_dir / "parameter_manifest.json").write_text(
        json.dumps(
            parameter_manifest(
                model,
                scene_path,
                included_model_path,
                config["scene"],
                config["included_model"],
            ),
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    write_csv(
        output_dir / "open_loop_replay.csv",
        replay_rows,
        config["joint_order"][:3],
    )

    print(f"Phase 14 MuJoCo internal dynamics: {'PASS' if overall_pass else 'FAIL'}")
    for name, check in checks.items():
        print(f"  {name}: {'PASS' if check['pass'] else 'FAIL'}")
    print(f"  evidence: {output_dir.relative_to(repo_root)}")
    if not overall_pass:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
