#!/usr/bin/env python3
"""Audit Phase-20 full-3D state and realizable virtual-input signs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase20_contract.json"
ACTIVE_JOINTS = (
    "left_hip_joint", "left_knee_joint", "left_wheel_joint",
    "right_hip_joint", "right_knee_joint", "right_wheel_joint",
)
PASSIVE_JOINTS = (
    "right_connect1_joint", "right_connect2_joint",
    "left_connect1_joint", "left_connect2_joint",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"Missing MuJoCo object {name!r}")
    return int(result)


def rotation_vector(matrix: np.ndarray) -> np.ndarray:
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, matrix.reshape(9))
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    norm = float(np.linalg.norm(quaternion[1:]))
    if norm < 1e-15:
        return 2.0 * quaternion[1:]
    return (2.0 * math.atan2(norm, float(quaternion[0])) / norm) * quaternion[1:]


class Contract:
    def __init__(self, config: dict[str, Any], equilibrium: list[float]) -> None:
        self.config = config
        self.equilibrium = np.asarray(equilibrium, dtype=float)
        scene = ROOT / config["scene"]
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.base_weld = object_id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld"
        )
        self.base_site = object_id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame"
        )
        self.active_qpos = np.asarray([
            self.model.jnt_qposadr[
                object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ] for name in ACTIVE_JOINTS
        ], dtype=int)
        self.passive_qpos = np.asarray([
            self.model.jnt_qposadr[
                object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ] for name in PASSIVE_JOINTS
        ], dtype=int)
        self.actuator_ids = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in config["actuator_order"]
        ], dtype=int)
        # Equilibrium order is right leg, left leg, z, passive right/left, support left/right.
        self.native_reference = np.asarray([
            equilibrium[2], equilibrium[3], 0.0,
            equilibrium[0], equilibrium[1], 0.0,
        ])
        self.native_passive = np.asarray(equilibrium[5:9], dtype=float)
        self.native_support = np.asarray([
            equilibrium[9], equilibrium[10], 0.0,
            equilibrium[11], equilibrium[12], 0.0,
        ])

    def reset(self, quaternion: tuple[float, float, float, float] = (1, 0, 0, 0)) -> mujoco.MjData:
        data = mujoco.MjData(self.model)
        data.eq_active[self.base_weld] = 0
        data.qpos[:7] = (0.0, 0.0, self.equilibrium[4], *quaternion)
        data.qpos[self.active_qpos] = self.native_reference
        data.qpos[self.passive_qpos] = self.native_passive
        data.qvel[:] = 0.0
        data.ctrl[self.actuator_ids] = self.native_support
        mujoco.mj_forward(self.model, data)
        return data

    def actuator_acceleration_map(self) -> np.ndarray:
        epsilon = float(self.config["finite_difference_torque_nm"])
        columns = []
        for actuator in self.actuator_ids:
            plus, minus = self.reset(), self.reset()
            plus.ctrl[actuator] += epsilon
            minus.ctrl[actuator] -= epsilon
            mujoco.mj_forward(self.model, plus)
            mujoco.mj_forward(self.model, minus)
            columns.append((plus.qacc - minus.qacc) / (2.0 * epsilon))
        return np.column_stack(columns)

    def orientation_axis_error(self) -> float:
        step = float(self.config["orientation_step_rad"])
        errors = []
        for axis in range(3):
            vector = np.zeros(3)
            vector[axis] = math.sin(step / 2.0)
            data = self.reset((math.cos(step / 2.0), *vector))
            rotation = data.site_xmat[self.base_site].reshape(3, 3)
            expected = np.zeros(3)
            expected[axis] = step
            errors.append(float(np.max(np.abs(rotation_vector(rotation) - expected))))
        return max(errors)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--equilibrium", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config_path = args.config.resolve()
    equilibrium_path = args.equilibrium.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))["candidate"]
    contract = Contract(config, equilibrium)
    acceleration = contract.actuator_acceleration_map()

    # MuJoCo native torque is the negative of canonical TorqueCommand.
    native_common = acceleration[:, 2] + acceleration[:, 5]
    native_difference = acceleration[:, 2] - acceleration[:, 5]
    leg_angular = acceleration[3:6][:, [0, 1, 3, 4]]
    native_roll = np.linalg.lstsq(
        leg_angular, np.asarray([1.0, 0.0, 0.0]), rcond=None
    )[0]
    native_roll /= np.linalg.norm(native_roll)
    canonical_roll = -native_roll

    canonical_response = np.column_stack([
        -native_common[3:6],
        leg_angular @ native_roll,
        -native_difference[3:6],
    ])
    condition = float(np.linalg.cond(canonical_response))
    target = np.diag(canonical_response[[1, 0, 2], :])
    roll_response = canonical_response[:, 1]
    cross_ratio = float(
        max(abs(roll_response[1]), abs(roll_response[2]))
        / abs(roll_response[0])
    )
    orientation_error = contract.orientation_axis_error()
    thresholds = config["thresholds"]
    gates = {
        "compiled_full_3d": bool(
            contract.model.nq == 17 and contract.model.nv == 16
            and contract.model.nu == 6 and contract.model.neq == 3
            and contract.model.opt.timestep == config["physics_timestep_s"]
        ),
        "base_freejoint": bool(contract.model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE),
        "input_rank": int(np.linalg.matrix_rank(canonical_response)) == 3,
        "input_condition": condition <= thresholds["maximum_input_basis_condition_number"],
        "target_authority": bool(np.min(np.abs(target)) >= thresholds["minimum_target_angular_acceleration_per_nm"]),
        "roll_cross_coupling": cross_ratio <= thresholds["maximum_roll_pitch_yaw_cross_ratio"],
        "orientation_log": orientation_error <= thresholds["maximum_orientation_log_axis_error"],
    }
    summary = {
        "schema_version": 1,
        "phase": 20,
        "pass": all(gates.values()),
        "gates": gates,
        "metrics": {
            "compiled": {
                "nq": contract.model.nq, "nv": contract.model.nv,
                "nu": contract.model.nu, "neq": contract.model.neq,
                "ngeom": contract.model.ngeom,
                "timestep_s": contract.model.opt.timestep,
            },
            "canonical_angular_acceleration_per_virtual_nm": canonical_response.tolist(),
            "input_basis_rank": int(np.linalg.matrix_rank(canonical_response)),
            "input_basis_condition_number": condition,
            "roll_pitch_yaw_cross_ratio": cross_ratio,
            "orientation_log_axis_error": orientation_error,
        },
        "s_roll_canonical_active_leg_order": canonical_roll.tolist(),
        "s_roll_canonical_joint_order": [
            float(canonical_roll[0]), float(canonical_roll[1]), 0.0,
            float(canonical_roll[2]), float(canonical_roll[3]), 0.0,
        ],
        "virtual_input_order": ["common_wheel", "roll_leg", "yaw_wheel"],
        "state_order": [
            "x_error_m", "vx_m_s", "pitch_error_rad", "omega_y_rad_s",
            "roll_error_rad", "omega_x_rad_s", "yaw_error_rad", "omega_z_rad_s",
        ],
        "canonical_to_native_torque_sign": -1,
    }
    (output / "contract.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest = {
        "config": str(config_path.relative_to(ROOT)),
        "config_sha256": sha256(config_path),
        "equilibrium": str(equilibrium_path.relative_to(ROOT)),
        "equilibrium_sha256": sha256(equilibrium_path),
        "scene": config["scene"],
        "scene_sha256": sha256(ROOT / config["scene"]),
        "script_sha256": sha256(Path(__file__).resolve()),
        "output_sha256": sha256(output / "contract.json"),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "hardware_data": False,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
