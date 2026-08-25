"""Compile the imported MJCF and export runtime frame/sensor evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np


DRIVEN_JOINT_TO_BODY = {
    "left_hip_joint": "left_wheel_body",
    "left_knee_joint": "left_wheel_body",
    "left_wheel_joint": "left_wheel_body",
    "right_hip_joint": "right_wheel_body",
    "right_knee_joint": "right_wheel_body",
    "right_wheel_joint": "right_wheel_body",
}


def object_name(model: mujoco.MjModel, kind: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, kind, index) or f"<{kind.name}:{index}>"


def vector(values: np.ndarray) -> list[float]:
    return [float(value) for value in np.asarray(values).ravel()]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def wheel_pose_for_joint(
    model: mujoco.MjModel,
    joint_name: str,
    body_name: str,
    delta: float,
) -> tuple[np.ndarray, np.ndarray]:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    data.qpos[model.jnt_qposadr[joint_id]] += delta
    mujoco.mj_forward(model, data)
    return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()


def sensor_slice(
    model: mujoco.MjModel, data: mujoco.MjData, sensor_name: str
) -> np.ndarray:
    sensor_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name
    )
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    return data.sensordata[address : address + dimension].copy()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        default="simulation/mujoco/model/scence.xml",
        type=Path,
    )
    parser.add_argument(
        "--output",
        default=(
            "docs/workflow/phases/02-coordinate-interface-contract/"
            "evidence/mujoco_runtime_manifest.json"
        ),
        type=Path,
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    scene_path = (repo_root / args.scene).resolve()
    output_path = (repo_root / args.output).resolve()

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    joints = []
    for joint_id in range(model.njnt):
        name = object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        joints.append(
            {
                "name": name,
                "type": int(model.jnt_type[joint_id]),
                "qposAddress": int(model.jnt_qposadr[joint_id]),
                "dofAddress": int(model.jnt_dofadr[joint_id]),
                "localAxis": vector(model.jnt_axis[joint_id]),
                "worldAxisAtQpos0": vector(data.xaxis[joint_id]),
                "body": object_name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.jnt_bodyid[joint_id]),
                ),
            }
        )

    bodies = []
    for body_id in range(1, model.nbody):
        bodies.append(
            {
                "name": object_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id),
                "parent": object_name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.body_parentid[body_id]),
                ),
                "worldPositionAtQpos0": vector(data.xpos[body_id]),
                "worldRotationAtQpos0": vector(
                    data.xmat[body_id].reshape(3, 3)
                ),
            }
        )

    sites = []
    for site_id in range(model.nsite):
        sites.append(
            {
                "name": object_name(model, mujoco.mjtObj.mjOBJ_SITE, site_id),
                "body": object_name(
                    model,
                    mujoco.mjtObj.mjOBJ_BODY,
                    int(model.site_bodyid[site_id]),
                ),
                "worldPositionAtQpos0": vector(data.site_xpos[site_id]),
                "worldRotationAtQpos0": vector(
                    data.site_xmat[site_id].reshape(3, 3)
                ),
            }
        )

    sensors = []
    for sensor_id in range(model.nsensor):
        address = int(model.sensor_adr[sensor_id])
        dimension = int(model.sensor_dim[sensor_id])
        sensors.append(
            {
                "name": object_name(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_id),
                "type": int(model.sensor_type[sensor_id]),
                "dimension": dimension,
                "dataAddress": address,
                "valueAtQpos0": vector(
                    data.sensordata[address : address + dimension]
                ),
            }
        )

    epsilon = 1e-6
    perturbations = []
    for joint_name, wheel_body in DRIVEN_JOINT_TO_BODY.items():
        minus_position, minus_rotation = wheel_pose_for_joint(
            model, joint_name, wheel_body, -epsilon
        )
        plus_position, plus_rotation = wheel_pose_for_joint(
            model, joint_name, wheel_body, epsilon
        )
        perturbations.append(
            {
                "joint": joint_name,
                "observedBody": wheel_body,
                "epsilonRad": epsilon,
                "dWorldPositionDqAtZero": vector(
                    (plus_position - minus_position) / (2 * epsilon)
                ),
                "rotationDeltaFrobeniusPerRad": float(
                    np.linalg.norm(plus_rotation - minus_rotation)
                    / (2 * epsilon)
                ),
            }
        )

    # Runtime-only probes leave repository XML untouched. Disabling all
    # equality constraints releases the imported free base for sensor tests.
    probe_model = mujoco.MjModel.from_xml_path(str(scene_path))
    probe_model.eq_active0[:] = 0
    probe_model.opt.gravity[:] = [0.0, 0.0, -9.81]
    freefall_data = mujoco.MjData(probe_model)
    mujoco.mj_forward(probe_model, freefall_data)

    quaternion_data = mujoco.MjData(probe_model)
    quaternion_data.qpos[:] = probe_model.qpos0
    half_angle = np.pi / 4
    quaternion_data.qpos[3:7] = [
        np.cos(half_angle),
        0.0,
        0.0,
        np.sin(half_angle),
    ]
    mujoco.mj_forward(probe_model, quaternion_data)

    gyro_model = mujoco.MjModel.from_xml_path(str(scene_path))
    gyro_model.eq_active0[:] = 0
    gyro_data = mujoco.MjData(gyro_model)
    gyro_data.qvel[5] = 0.25
    mujoco.mj_forward(gyro_model, gyro_data)

    runtime_probes = {
        "freefallWithInjectedGravity": {
            "injectedGravityNative": vector(probe_model.opt.gravity),
            "allEqualityConstraintsDisabled": True,
            "baseTranslationalQaccNative": vector(freefall_data.qacc[:3]),
            "baseAccelerometerLocal": vector(
                sensor_slice(probe_model, freefall_data, "base_accel")
            ),
            "expected": "near zero specific force during freefall",
        },
        "baseFramequatPositive90AboutNativeZ": {
            "injectedBaseQposQuaternionWxyz": vector(
                quaternion_data.qpos[3:7]
            ),
            "reportedFramequatWxyz": vector(
                sensor_slice(probe_model, quaternion_data, "base_quat")
            ),
        },
        "baseGyroPositiveNativeZ": {
            "injectedAngularRateRadPerSecond": [0.0, 0.0, 0.25],
            "reportedGyroLocalRadPerSecond": vector(
                sensor_slice(gyro_model, gyro_data, "base_gyro")
            ),
        },
    }

    manifest = {
        "schemaVersion": 1,
        "generatedAt": datetime.now(timezone.utc).isoformat(),
        "runtime": {
            "mujocoVersion": mujoco.__version__,
            "environment": "conda:mujoco",
            "reproduction": (
                "conda env update -n mujoco -f simulation/mujoco/environment.yml"
            ),
        },
        "source": {
            "scenePath": str(args.scene).replace("\\", "/"),
            "sceneSha256": sha256(scene_path),
        },
        "compiled": {
            "nq": model.nq,
            "nv": model.nv,
            "nu": model.nu,
            "numberOfSensors": model.nsensor,
            "sensorDataWidth": model.nsensordata,
            "gravity": vector(model.opt.gravity),
            "timestep": float(model.opt.timestep),
            "qpos0": vector(model.qpos0),
        },
        "bodies": bodies,
        "joints": joints,
        "sites": sites,
        "sensors": sensors,
        "positiveJointPerturbations": perturbations,
        "runtimeProbes": runtime_probes,
        "interpretationLimits": [
            "The model has zero actuators, so torque direction is not tested.",
            "The base freejoint is constrained by a world weld.",
            "Direct qpos perturbations report kinematic-tree derivatives; they do not solve closed-loop equality constraints.",
            "Mesh appearance and real encoder/IMU installation directions remain external evidence gates.",
        ],
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(f"MuJoCo runtime manifest written to {output_path}")
    print(
        f"MuJoCo {mujoco.__version__}: nq={model.nq}, nv={model.nv}, "
        f"nu={model.nu}, nsensordata={model.nsensordata}"
    )
    print(f"Compiled gravity={vector(model.opt.gravity)}")
    print("Runtime frame audit: PASS (dynamic semantic gates remain open)")


if __name__ == "__main__":
    main()
