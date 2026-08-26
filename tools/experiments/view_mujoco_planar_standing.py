#!/usr/bin/env python3
"""Visualize the frozen Phase-19 exact-planar standing profile.

This is a viewer-only reproduction aid. Formal PASS evidence continues to come
from run_mujoco_planar_standing_formal.py and the C++ Controller/Adapter loop.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from run_mujoco_planar_prefreeze import Plant
from solve_mujoco_planar_equilibrium import object_id


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase19_planar_formal.json"
JOINT_NAMES = (
    "left_hip_joint",
    "left_knee_joint",
    "left_wheel_joint",
    "right_hip_joint",
    "right_knee_joint",
    "right_wheel_joint",
)
ACTUATOR_NAMES = tuple(name.removesuffix("_joint") + "_torque" for name in JOINT_NAMES)
JOINT_OFFSETS = np.asarray(
    [-1.3267204093873923, 2.2088002548867229, 0.0,
     -1.3267204093873923, 2.2088002548867229, 0.0]
)
LEG_INDICES = np.asarray([0, 1, 3, 4])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open a live MuJoCo viewer for the frozen Phase-19 standing profile."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--case",
        default="pitch_positive",
        help="case id from the formal profile (default: pitch_positive)",
    )
    parser.add_argument(
        "--duration", type=float, default=10.0,
        help="simulation seconds for --headless smoke test (default: 10)",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="run without a window; intended only for smoke-testing this entrypoint",
    )
    return parser.parse_args()


def load_profile(path: Path) -> tuple[dict[str, Any], Path]:
    path = path.resolve()
    profile = json.loads(path.read_text())
    scene = (ROOT / profile["scene"]).resolve()
    if not scene.is_file():
        raise FileNotFoundError(f"Phase-19 scene does not exist: {scene}")
    return profile, scene


def select_case(profile: dict[str, Any], case_id: str) -> dict[str, Any]:
    if case_id == "nominal":
        return {"id": "nominal", "initial_state": [0.0, 0.0, 0.0, 0.0]}
    for case in profile["cases"]:
        if case["id"] == case_id:
            return case
    available = ["nominal", *(case["id"] for case in profile["cases"])]
    raise ValueError(f"unknown case {case_id!r}; choose one of: {', '.join(available)}")


def build_plant(profile: dict[str, Any], scene: Path) -> Plant:
    # Plant expects the nine equilibrium coordinates followed by four native
    # support torques. Control below is evaluated in canonical coordinates, so
    # these appended values are used only to satisfy that reusable constructor.
    equilibrium = np.r_[np.asarray(profile["equilibrium"], dtype=float), np.zeros(4)]
    config = {
        "leg_kp_nm_per_rad": 0.0,
        "leg_kd_nm_s_per_rad": 0.0,
        "leg_torque_limit_nm": 10.0,
        "physics_steps_per_control": 5,
    }
    plant = Plant(scene, equilibrium, config)
    if not math.isclose(plant.model.opt.timestep, 0.002, abs_tol=1.0e-12):
        raise RuntimeError("Phase-19 viewer requires the frozen 2 ms physics step")
    return plant


def initialize(plant: Plant, case: dict[str, Any]) -> mujoco.MjData:
    state = np.asarray(case.get("initial_state", [0.0, 0.0, 0.0, 0.0]), dtype=float)
    data = plant.reset(state)
    perturbation = np.asarray(case.get("leg_perturbation", np.zeros(4)), dtype=float)
    data.qpos[plant.active_qpos] -= perturbation
    mujoco.mj_forward(plant.model, data)

    base_z = object_id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, "base_z_joint")
    base_z_qpos = plant.model.jnt_qposadr[base_z]
    for _ in range(500):
        if plant.bilateral_contact(data):
            break
        data.qpos[base_z_qpos] -= 1.0e-5
        mujoco.mj_forward(plant.model, data)
    else:
        raise RuntimeError("could not project reset onto bilateral wheel contact")
    return data


class CanonicalController:
    def __init__(self, plant: Plant, profile: dict[str, Any], data: mujoco.MjData) -> None:
        self.plant = plant
        self.reference = np.asarray(profile["reference"], dtype=float)
        self.support = np.asarray(profile["support_torque_nm"], dtype=float)
        self.kp = np.asarray(profile["kp_nm_per_rad"], dtype=float)
        self.kd = np.asarray(profile["kd_nm_s_per_rad"], dtype=float)
        self.limit = np.asarray(profile["torque_limit_nm"], dtype=float)
        self.gain = np.asarray(profile["standing_gain"], dtype=float)
        self.safety = np.asarray(profile["safety"], dtype=float)
        self.joints = np.asarray([
            object_id(plant.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in JOINT_NAMES
        ])
        self.qpos = plant.model.jnt_qposadr[self.joints]
        self.dofs = plant.model.jnt_dofadr[self.joints]
        self.actuators = np.asarray([
            object_id(plant.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in ACTUATOR_NAMES
        ])
        self.anchor_x = float(data.site_xpos[plant.site, 0])
        self.anchor_height = float(data.site_xpos[plant.site, 2])
        self.latched = False

    def step(self, data: mujoco.MjData, *, refresh: bool = True) -> None:
        # Match Adapter::extractState(): refresh kinematics/contact at each
        # 10 ms controller sample before reading the canonical state.
        if refresh:
            mujoco.mj_forward(self.plant.model, data)
        canonical_q = -data.qpos[self.qpos] + JOINT_OFFSETS
        canonical_dq = -data.qvel[self.dofs]
        state = self.plant.observe(data)
        state[0] = data.site_xpos[self.plant.site, 0] - self.anchor_x
        height_error = data.site_xpos[self.plant.site, 2] - self.anchor_height
        leg_error = canonical_q[LEG_INDICES] - self.reference[LEG_INDICES]

        self.latched = self.latched or (
            not self.plant.bilateral_contact(data)
            or abs(state[2]) > self.safety[0]
            or abs(state[0]) > self.safety[1]
            or abs(height_error) > self.safety[2]
            or np.max(np.abs(leg_error)) > self.safety[3]
            or np.max(np.abs(canonical_dq)) > self.safety[4]
        )

        canonical_torque = np.zeros(6)
        if not self.latched:
            canonical_torque[LEG_INDICES] = (
                self.support[LEG_INDICES]
                + self.kp[LEG_INDICES]
                * (self.reference[LEG_INDICES] - canonical_q[LEG_INDICES])
                - self.kd[LEG_INDICES] * canonical_dq[LEG_INDICES]
            )
            wheel_torque = -float(self.gain @ state)
            canonical_torque[[2, 5]] = wheel_torque
            self.latched = bool(np.any(np.abs(canonical_torque) > self.limit))

        data.ctrl[:] = 0.0
        if not self.latched:
            # The Adapter contract is native MuJoCo ctrl = -canonical torque.
            data.ctrl[self.actuators] = -canonical_torque


class ViewerControlCallback:
    """Apply the 10 ms controller and formal disturbance from MuJoCo's loop."""

    def __init__(
        self,
        plant: Plant,
        data: mujoco.MjData,
        controller: CanonicalController,
        case: dict[str, Any],
    ) -> None:
        self.plant = plant
        self.data = data
        self.controller = controller
        self.case = case
        self.base_body = object_id(
            plant.model, mujoco.mjtObj.mjOBJ_BODY, "base_body"
        )
        self.last_control_step = -1
        self.latch_reported = False

    def __call__(self, _model: mujoco.MjModel, data: mujoco.MjData) -> None:
        physics_step = int(round(data.time / self.plant.model.opt.timestep))
        if physics_step < self.last_control_step:
            self.last_control_step = -1
        if physics_step % 5 == 0 and physics_step != self.last_control_step:
            # The callback runs inside mj_forward, where kinematics/contact are
            # already current; recursively calling mj_forward is invalid.
            self.controller.step(data, refresh=False)
            self.last_control_step = physics_step
            if self.controller.latched and not self.latch_reported:
                print(f"safety latch at t={data.time:.3f} s; commands forced to zero")
                self.latch_reported = True

        tick = physics_step // 5
        force_start = int(self.case.get("disturbance_start_tick", -1))
        force_stop = force_start + int(self.case.get("disturbance_ticks", 0))
        disturbance_active = force_start <= tick < force_stop
        data.xfrc_applied[6 * self.base_body] = (
            float(self.case.get("force_x_n", 0.0)) if disturbance_active else 0.0
        )
        data.xfrc_applied[6 * self.base_body + 4] = (
            float(self.case.get("pitch_moment_nm", 0.0))
            if disturbance_active else 0.0
        )


def run(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise ValueError("duration must be positive")
    profile, scene = load_profile(args.config)
    case = select_case(profile, args.case)
    plant = build_plant(profile, scene)
    data = initialize(plant, case)
    controller = CanonicalController(plant, profile, data)
    callback = ViewerControlCallback(plant, data, controller, case)

    if not args.headless:
        from mujoco import viewer as mujoco_viewer
        print("MuJoCo Simulate 已打开：按 Space 开始/暂停，关闭窗口退出。")
        mujoco.set_mjcb_control(callback)
        try:
            mujoco_viewer.launch(plant.model, data)
        finally:
            mujoco.set_mjcb_control(None)
    else:
        steps = int(round(args.duration / plant.model.opt.timestep))
        mujoco.set_mjcb_control(callback)
        try:
            for _ in range(steps):
                mujoco.mj_step(plant.model, data)
        finally:
            mujoco.set_mjcb_control(None)

    mujoco.mj_forward(plant.model, data)
    final_state = plant.observe(data)
    final_state[0] = data.site_xpos[plant.site, 0] - controller.anchor_x
    print(
        f"case={case['id']} simulated={data.time:.3f}s "
        f"latched={controller.latched} "
        f"final=[x={final_state[0]:+.6f}m, dx={final_state[1]:+.6f}m/s, "
        f"pitch={final_state[2]:+.6f}rad, dtheta={final_state[3]:+.6f}rad/s]"
    )


if __name__ == "__main__":
    run(parse_args())
