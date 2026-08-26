#!/usr/bin/env python3
"""Reproduce the Phase-19 pre-freeze standing-controller decision gate.

This deliberately does not implement a second production controller loop.  It
tests whether the proposed four-state/common-wheel structure is good enough to
justify implementing it in Controller Core.  A failed gate is valid evidence.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase19_exploration.json"
DEFAULT_OUTPUT = (
    ROOT
    / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-prefreeze"
)
ACTIVE_NAMES = (
    "left_hip_joint", "left_knee_joint", "left_wheel_joint",
    "right_hip_joint", "right_knee_joint", "right_wheel_joint",
)
PASSIVE_NAMES = (
    "left_connect1_joint", "left_connect2_joint",
    "right_connect1_joint", "right_connect2_joint",
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"Missing MuJoCo object {name!r}")
    return result


def pitch_from_quaternion(quaternion: np.ndarray) -> float:
    w, x, y, z = quaternion
    return math.atan2(2.0 * (w * y - z * x), 1.0 - 2.0 * (x * x + y * y))


class Exploration:
    def __init__(self, config: dict[str, Any], scene: Path) -> None:
        self.config = config
        self.candidate = config["candidate"]
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.model.opt.timestep = float(config["physics_timestep_s"])
        self.active_joint_ids = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ACTIVE_NAMES
        ])
        self.passive_joint_ids = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in PASSIVE_NAMES
        ])
        self.active_qpos = self.model.jnt_qposadr[self.active_joint_ids]
        self.active_dofs = self.model.jnt_dofadr[self.active_joint_ids]
        self.passive_qpos = self.model.jnt_qposadr[self.passive_joint_ids]
        self.base_weld = object_id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld"
        )
        self.base_site = object_id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame"
        )
        self.floor = object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.wheels = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("left_wheel_collision", "right_wheel_collision")
        ]
        self.reference = np.asarray(
            self.candidate["native_active_reference_rad"], dtype=float
        )
        self.support = np.asarray(
            self.candidate["native_support_torque_nm"], dtype=float
        )
        self.feedback = np.asarray(
            self.candidate["native_common_wheel_feedback"], dtype=float
        )

    def reset(self, initial_x: float, initial_pitch: float) -> mujoco.MjData:
        data = mujoco.MjData(self.model)
        mujoco.mj_resetData(self.model, data)
        data.eq_active[self.base_weld] = 0
        x, y, z = self.candidate["base_qpos_xyz_m"]
        data.qpos[:7] = (
            x + initial_x, y, z,
            math.cos(initial_pitch / 2.0), 0.0,
            math.sin(initial_pitch / 2.0), 0.0,
        )
        data.qpos[self.active_qpos] = self.reference
        data.qpos[self.passive_qpos] = np.asarray(
            self.candidate["native_passive_reference_rad"], dtype=float
        )
        data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, data)
        return data

    def site_state(self, data: mujoco.MjData) -> np.ndarray:
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(quaternion, data.site_xmat[self.base_site])
        jacobian_position = np.zeros((3, self.model.nv))
        jacobian_rotation = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(
            self.model, data, jacobian_position, jacobian_rotation,
            self.base_site,
        )
        linear = jacobian_position @ data.qvel
        angular = jacobian_rotation @ data.qvel
        return np.asarray([
            data.site_xpos[self.base_site, 0], linear[0],
            pitch_from_quaternion(quaternion), angular[1],
        ])

    def rpy(self, data: mujoco.MjData) -> np.ndarray:
        matrix = data.site_xmat[self.base_site].reshape(3, 3)
        return np.asarray([
            math.atan2(matrix[2, 1], matrix[2, 2]),
            math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0))),
            math.atan2(matrix[1, 0], matrix[0, 0]),
        ])

    def native_control(self, data: mujoco.MjData, state_error: np.ndarray) -> np.ndarray:
        control = self.support.copy()
        leg_indices = np.asarray([0, 1, 3, 4])
        control[leg_indices] += (
            float(self.candidate["leg_kp_nm_per_rad"])
            * (self.reference[leg_indices] - data.qpos[self.active_qpos[leg_indices]])
            - float(self.candidate["leg_kd_nm_s_per_rad"])
            * data.qvel[self.active_dofs[leg_indices]]
        )
        wheel = float(self.feedback @ state_error)
        wheel = float(np.clip(
            wheel,
            -float(self.candidate["wheel_torque_limit_nm"]),
            float(self.candidate["wheel_torque_limit_nm"]),
        ))
        control[[2, 5]] = wheel
        return np.clip(
            control,
            -float(self.candidate["leg_torque_limit_nm"]),
            float(self.candidate["leg_torque_limit_nm"]),
        )

    def apply_planar_probe(self, data: mujoco.MjData) -> None:
        probe = self.config["planar_stabilization_probe"]
        data.qfrc_applied[1] = (
            -float(probe["lateral_position_stiffness_n_per_m"]) * data.qpos[1]
            -float(probe["lateral_velocity_damping_n_s_per_m"]) * data.qvel[1]
        )
        data.qfrc_applied[3] = (
            -float(probe["roll_rate_damping_nm_s_per_rad"]) * data.qvel[3]
        )
        data.qfrc_applied[5] = (
            -float(probe["yaw_rate_damping_nm_s_per_rad"]) * data.qvel[5]
        )

    def contact_bits(self, data: mujoco.MjData) -> tuple[int, int]:
        bits = [0, 0]
        for index in range(data.ncon):
            contact = data.contact[index]
            for side, wheel in enumerate(self.wheels):
                if {contact.geom1, contact.geom2} == {wheel, self.floor}:
                    bits[side] = 1
        return bits[0], bits[1]

    def one_tick(self, state: np.ndarray, wheel_torque: float) -> np.ndarray:
        data = self.reset(float(state[0]), float(state[2]))
        data.qvel[0] = state[1]
        data.qvel[4] = state[3]
        mujoco.mj_forward(self.model, data)
        zero_error = np.zeros(4)
        control = self.native_control(data, zero_error)
        control[[2, 5]] = wheel_torque
        data.ctrl[:] = control
        for _ in range(int(self.config["physics_steps_per_control"])):
            data.qfrc_applied[:] = 0.0
            self.apply_planar_probe(data)
            mujoco.mj_step(self.model, data)
        result = self.site_state(data)
        result[0] += float(self.candidate["base_qpos_xyz_m"][0])
        return result

    def local_model(self) -> dict[str, Any]:
        origin_data = self.reset(0.0, 0.0)
        site_origin = self.site_state(origin_data)
        origin = np.zeros(4)
        nominal = self.one_tick(origin, 0.0)
        nominal[0] -= site_origin[0]
        a = np.zeros((4, 4))
        eps = (1.0e-5, 1.0e-4, 1.0e-5, 1.0e-4)
        for index, delta in enumerate(eps):
            positive = origin.copy()
            negative = origin.copy()
            positive[index] += delta
            negative[index] -= delta
            plus = self.one_tick(positive, 0.0)
            minus = self.one_tick(negative, 0.0)
            a[:, index] = (plus - minus) / (2.0 * delta)
        delta_u = 1.0e-4
        b = (
            self.one_tick(origin, delta_u)
            - self.one_tick(origin, -delta_u)
        ) / (2.0 * delta_u)
        controllability = np.column_stack([
            np.linalg.matrix_power(a, power) @ b for power in range(4)
        ])
        # Native wheel input is the negative of canonical TorqueCommand.  The
        # proposed canonical law tau=-Kx therefore closes as A+B_native*K.
        poles = np.linalg.eigvals(a + np.outer(b, self.feedback))
        return {
            "state_order": ["base_site_x_error_m", "base_site_vx_m_s", "pitch_rad", "pitch_rate_rad_s"],
            "input": "equal native wheel torque; canonical TorqueCommand has opposite sign",
            "A": a.tolist(),
            "B_native": b.tolist(),
            "affine_drift_one_tick": nominal.tolist(),
            "K_canonical_tau_equals_minus_Kx": self.feedback.tolist(),
            "controllability_rank": int(np.linalg.matrix_rank(controllability)),
            "closed_loop_poles": [[float(value.real), float(value.imag)] for value in poles],
            "closed_loop_spectral_radius": float(np.max(np.abs(poles))),
        }

    def run_case(self, case: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        data = self.reset(case["initial_x_m"], case["initial_pitch_rad"])
        anchor = self.site_state(data)[0]
        rows: list[dict[str, Any]] = []
        steps = int(round(
            float(self.config["duration_s"]) / float(self.config["control_period_s"])
        ))
        for tick in range(steps):
            state = self.site_state(data)
            state[0] -= anchor
            native_control = self.native_control(data, state)
            data.ctrl[:] = native_control
            for _ in range(int(self.config["physics_steps_per_control"])):
                data.qfrc_applied[:] = 0.0
                if case["planar_stabilization"]:
                    self.apply_planar_probe(data)
                mujoco.mj_step(self.model, data)
            state_after = self.site_state(data)
            state_after[0] -= anchor
            roll, pitch, yaw = self.rpy(data)
            left_contact, right_contact = self.contact_bits(data)
            rows.append({
                "case": case["id"], "tick": tick,
                "time_s": float(data.time),
                "x_error_m": float(state_after[0]),
                "vx_m_s": float(state_after[1]),
                "pitch_rad": float(pitch),
                "pitch_rate_rad_s": float(state_after[3]),
                "base_z_m": float(data.qpos[2]),
                "base_y_m": float(data.qpos[1]),
                "roll_rad": float(roll), "yaw_rad": float(yaw),
                "left_contact": left_contact, "right_contact": right_contact,
                "native_left_wheel_torque_nm": float(native_control[2]),
                "native_right_wheel_torque_nm": float(native_control[5]),
            })
        nominal_z = float(self.candidate["base_qpos_xyz_m"][2])
        metrics = {
            "case": case["id"],
            "planar_stabilization": bool(case["planar_stabilization"]),
            "finite": all(
                math.isfinite(float(value))
                for row in rows for value in row.values()
                if not isinstance(value, str)
            ),
            "final_abs_x_error_m": abs(rows[-1]["x_error_m"]),
            "final_abs_pitch_rad": abs(rows[-1]["pitch_rad"]),
            "max_abs_pitch_rad": max(abs(row["pitch_rad"]) for row in rows),
            "max_abs_height_error_m": max(abs(row["base_z_m"] - nominal_z) for row in rows),
            "max_abs_lateral_position_m": max(abs(row["base_y_m"]) for row in rows),
            "max_abs_roll_or_yaw_rad": max(
                max(abs(row["roll_rad"]), abs(row["yaw_rad"])) for row in rows
            ),
            "both_wheels_contact_fraction": sum(
                row["left_contact"] and row["right_contact"] for row in rows
            ) / len(rows),
            "equal_wheel_torque_max_error_nm": max(
                abs(row["native_left_wheel_torque_nm"] - row["native_right_wheel_torque_nm"])
                for row in rows
            ),
        }
        return rows, metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config_path = args.config.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    scene = (ROOT / config["scene"]).resolve()
    model = (ROOT / config["included_model"]).resolve()
    exploration = Exploration(config, scene)
    local_model = exploration.local_model()
    all_rows: list[dict[str, Any]] = []
    cases: list[dict[str, Any]] = []
    for case in config["cases"]:
        rows, metrics = exploration.run_case(case)
        all_rows.extend(rows)
        cases.append(metrics)
    gates = config["prefreeze_gates"]
    local_model["gate_pass"] = (
        local_model["controllability_rank"] == 4
        and local_model["closed_loop_spectral_radius"]
        <= gates["closed_loop_spectral_radius_max"]
    )
    for metrics in cases:
        metrics["gate_pass"] = (
            metrics["finite"]
            and metrics["final_abs_x_error_m"] <= gates["final_abs_x_error_m"]
            and metrics["final_abs_pitch_rad"] <= gates["final_abs_pitch_rad"]
            and metrics["max_abs_pitch_rad"] <= gates["max_abs_pitch_rad"]
            and metrics["max_abs_height_error_m"] <= gates["max_abs_height_error_m"]
            and metrics["max_abs_lateral_position_m"] <= gates["max_abs_lateral_position_m"]
            and metrics["max_abs_roll_or_yaw_rad"] <= gates["max_abs_roll_or_yaw_rad"]
            and metrics["both_wheels_contact_fraction"]
            >= gates["minimum_both_wheels_contact_fraction"]
            and metrics["equal_wheel_torque_max_error_nm"] == 0.0
        )
    summary = {
        "schema_version": 1,
        "phase": 19,
        "evidence_class": "exploratory pre-freeze decision gate",
        "hardware_data": False,
        "local_model": local_model,
        "cases": cases,
        "overall_pass": bool(local_model["gate_pass"] and all(case["gate_pass"] for case in cases)),
        "decision": "IMPLEMENT_CORE" if local_model["gate_pass"] and all(case["gate_pass"] for case in cases) else "REWORK",
    }
    with (output_dir / "timeseries.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    manifest = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": sys.version,
        "platform": platform.platform(),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "config": str(config_path.relative_to(ROOT)),
        "scene": str(scene.relative_to(ROOT)),
        "included_model": str(model.relative_to(ROOT)),
        "sha256": {
            "config": sha256(config_path), "scene": sha256(scene),
            "included_model": sha256(model), "script": sha256(Path(__file__)),
        },
        "hardware_data": False,
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))
    return 0 if summary["overall_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
