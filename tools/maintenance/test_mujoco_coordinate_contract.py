"""Verify the approved Phase-02 MuJoCo coordinate contract."""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np


DRIVEN_JOINTS = (
    "left_hip_joint",
    "left_knee_joint",
    "left_wheel_joint",
    "right_hip_joint",
    "right_knee_joint",
    "right_wheel_joint",
)

WHEEL_BY_SIDE = {
    "left": "left_wheel_body",
    "right": "right_wheel_body",
}

R_N_FROM_S = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ]
)


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise AssertionError(f"Missing {kind.name} named {name!r}.")
    return result


def assert_close(
    actual: np.ndarray,
    expected: np.ndarray,
    *,
    atol: float = 1e-9,
    message: str,
) -> None:
    if not np.allclose(actual, expected, rtol=0.0, atol=atol):
        raise AssertionError(
            f"{message}\nactual={np.asarray(actual)}\nexpected={np.asarray(expected)}"
        )


def axis_angle_quaternion(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    return np.r_[np.cos(angle / 2.0), axis * np.sin(angle / 2.0)]


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, np.asarray(quaternion, dtype=float))
    return result.reshape(3, 3)


def sensor_value(
    model: mujoco.MjModel, data: mujoco.MjData, name: str
) -> np.ndarray:
    sensor_id = object_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return data.sensordata[address : address + dimension].copy()


def body_position_after_joint_delta(
    model: mujoco.MjModel, joint_name: str, body_name: str, delta: float
) -> np.ndarray:
    joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    body_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    data.qpos[model.jnt_qposadr[joint_id]] += delta
    mujoco.mj_forward(model, data)
    return data.xpos[body_id].copy()


def test_base_control_frame(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    body_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_body")
    site_id = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")

    assert int(model.site_bodyid[site_id]) == body_id
    assert_close(
        model.site_pos[site_id],
        model.body_ipos[body_id],
        atol=1e-9,
        message="base_control_frame is stale relative to the compiled torso COM.",
    )
    assert_close(
        data.site_xpos[site_id],
        data.xipos[body_id],
        atol=1e-9,
        message="base_control_frame world position must equal torso COM position.",
    )
    assert_close(
        data.site_xmat[site_id],
        data.xmat[body_id],
        message="base_control_frame axes must remain parallel to base_body axes.",
    )


def test_joint_axes_and_signs(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    simscape_positive_axis_n = R_N_FROM_S @ np.array([0.0, 0.0, 1.0])
    expected_mujoco_axis_n = np.array([0.0, 1.0, 0.0])

    for name in DRIVEN_JOINTS:
        joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        axis_n = data.xaxis[joint_id]
        assert_close(
            axis_n,
            expected_mujoco_axis_n,
            atol=5e-6,
            message=f"{name} is no longer aligned with canonical +N_y.",
        )
        if float(np.dot(axis_n, simscape_positive_axis_n)) > -0.99999:
            raise AssertionError(
                f"{name} must remain opposite the Simscape +S_z joint axis."
            )

    epsilon = 1e-6
    for joint_kind in ("hip", "knee"):
        derivatives = []
        for side in ("left", "right"):
            joint_name = f"{side}_{joint_kind}_joint"
            wheel_name = WHEEL_BY_SIDE[side]
            plus = body_position_after_joint_delta(
                model, joint_name, wheel_name, epsilon
            )
            minus = body_position_after_joint_delta(
                model, joint_name, wheel_name, -epsilon
            )
            derivatives.append((plus - minus) / (2.0 * epsilon))
        assert_close(
            derivatives[0],
            derivatives[1],
            atol=2e-8,
            message=f"Left/right {joint_kind} positive perturbations lost symmetry.",
        )

    # For a wheel rotating about +N_y with bottom radius along -N_z,
    # no-slip center velocity is -omega x radius = +N_x (forward).
    bottom_radius_n = np.array([0.0, 0.0, -1.0])
    rolling_center_velocity_n = -np.cross(
        expected_mujoco_axis_n, bottom_radius_n
    )
    assert_close(
        rolling_center_velocity_n,
        np.array([1.0, 0.0, 0.0]),
        message="Positive MuJoCo wheel q must retain forward rolling semantics.",
    )


def test_quaternion_contract(scene_path: Path) -> None:
    probe_model = mujoco.MjModel.from_xml_path(str(scene_path))
    probe_model.eq_active0[:] = 0

    cases = (
        axis_angle_quaternion(np.array([1.0, 0.0, 0.0]), np.pi / 2.0),
        axis_angle_quaternion(np.array([0.0, 1.0, 0.0]), -np.pi / 2.0),
        axis_angle_quaternion(np.array([0.0, 0.0, 1.0]), np.pi / 2.0),
        axis_angle_quaternion(np.array([1.0, 2.0, -3.0]), 0.73),
    )
    for quaternion in cases:
        data = mujoco.MjData(probe_model)
        data.qpos[:] = probe_model.qpos0
        data.qpos[3:7] = quaternion
        mujoco.mj_forward(probe_model, data)
        reported = sensor_value(probe_model, data, "base_quat")
        if np.dot(reported, quaternion) < 0.0:
            reported = -reported
        assert_close(
            reported,
            quaternion,
            atol=2e-9,
            message="MuJoCo framequat changed its active wxyz convention.",
        )
        assert_close(
            quaternion_matrix(reported),
            quaternion_matrix(quaternion),
            atol=2e-9,
            message="Reported quaternion rotation matrix mismatch.",
        )

    # A baseline positive yaw is about +S_y. Conjugating both world and body
    # axes into FLU must produce the same positive yaw about +N_z.
    yaw_samples = np.deg2rad(np.array([170.0, 179.0, 181.0, 190.0]))
    wrapped = []
    for yaw in yaw_samples:
        c, s = np.cos(yaw), np.sin(yaw)
        rotation_s = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        rotation_n = R_N_FROM_S @ rotation_s @ R_N_FROM_S.T
        expected_n = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        assert_close(
            rotation_n,
            expected_n,
            message="Simscape positive yaw did not map to canonical positive yaw.",
        )
        wrapped.append(np.arctan2(rotation_n[1, 0], rotation_n[0, 0]))
    assert_close(
        np.unwrap(wrapped),
        yaw_samples,
        atol=2e-15,
        message="Canonical yaw continuity failed across +pi.",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene", default="simulation/mujoco/model/scence.xml", type=Path
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    scene_path = (repo_root / args.scene).resolve()

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    assert_close(
        R_N_FROM_S.T @ R_N_FROM_S,
        np.eye(3),
        message="R_N_from_S is not orthonormal.",
    )
    if not np.isclose(np.linalg.det(R_N_FROM_S), 1.0):
        raise AssertionError("R_N_from_S must be a proper rotation.")

    test_base_control_frame(model, data)
    test_joint_axes_and_signs(model, data)
    test_quaternion_contract(scene_path)

    print("MuJoCo coordinate contract: PASS")
    print("  world: native MuJoCo = canonical FLU")
    print("  base_control_frame: torso COM position + base_body axes")
    print("  driven joints: MuJoCo +N_y = Simscape -S_z")
    print("  quaternion: active wxyz; positive yaw continuity verified")
    print("  joint zero offsets are validated by wheel_leg_mujoco adapter tests")
    print("  real IMU installation remains an external gate")


if __name__ == "__main__":
    main()
