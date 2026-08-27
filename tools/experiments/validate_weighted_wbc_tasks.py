#!/usr/bin/env python3
"""Phase-21 weighted-task ablation and nonlinear pre-freeze validation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402
from validate_weighted_wbc_qp import solve_admm  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_task_prefreeze.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"Missing {kind.name} named {name!r}")
    return result


def rotation_vector(quaternion: np.ndarray) -> np.ndarray:
    q = quaternion.copy()
    if q[0] < 0.0:
        q = -q
    norm = float(np.linalg.norm(q[1:]))
    if norm < 1e-14:
        return 2.0 * q[1:]
    return 2.0 * np.arctan2(norm, q[0]) * q[1:] / norm


class ControllerOracle:
    def __init__(self, config: dict[str, Any], model_config: dict[str, Any],
                 equilibrium: dict[str, Any], qp_config: dict[str, Any]) -> None:
        self.config = config
        self.qp = qp_config
        self.oracle = Oracle(model_config, equilibrium)
        self.reference_qpos = self.oracle.sample_qpos(model_config["samples"][0])
        self.reference_active = -self.reference_qpos[self.oracle.active_qpos]
        self.reference_position = self._base_position(self.reference_qpos)
        model = self.oracle.model
        self.full_actuator = np.zeros((model.nv, 6))
        for column, actuator in enumerate(self.oracle.actuators):
            joint = int(model.actuator_trnid[actuator, 0])
            self.full_actuator[int(model.jnt_dofadr[joint]), column] = -1.0
        reference = self.model(self.reference_qpos, np.zeros(12))
        static_matrix = np.hstack((reference["actuator"], reference["contact"].T))
        static = np.linalg.lstsq(static_matrix, reference["bias"], rcond=None)[0]
        self.reference_wrench = reference["wrench_map"] @ static[6:]
        scales = qp_config["variable_scale"]
        self.variable_scale = np.asarray(
            scales["acceleration"] + scales["torque"] +
            scales["contact_force"] + scales["wrench_slack"], dtype=float
        )
        self.transform = np.diag(self.variable_scale)
        self.warm_start = np.zeros(36)

    def reset(self) -> None:
        self.warm_start[:] = 0.0

    def _base_position(self, qpos: np.ndarray) -> np.ndarray:
        self.oracle.forward(qpos)
        return self.oracle.data.site_xpos[self.oracle.base_control_site].copy()

    def canonical_qpos(self, plant_qpos: np.ndarray) -> np.ndarray:
        qpos = plant_qpos.copy()
        qpos[self.oracle.passive_qpos] = self.oracle.equilibrium_passive
        return self.oracle.solve_passive(qpos)[0]

    def reduced_velocity(self, qpos: np.ndarray, plant_qvel: np.ndarray) -> np.ndarray:
        self.oracle.forward(qpos)
        linear, angular = self.oracle.site_jacobian(self.oracle.base_control_site)
        return np.r_[linear @ plant_qvel, angular @ plant_qvel,
                     -plant_qvel[self.oracle.active_dofs]]

    def model(self, qpos: np.ndarray, velocity: np.ndarray) -> dict[str, np.ndarray]:
        reduction, metrics = self.oracle.reduction(qpos)
        step = float(self.oracle.config["solver"]["second_difference_step"])
        plus = self.oracle.integrate_flow(qpos, velocity, step)
        minus = self.oracle.integrate_flow(qpos, velocity, -step)
        plus_reduction, _ = self.oracle.reduction(plus)
        minus_reduction, _ = self.oracle.reduction(minus)
        ndot_velocity = ((plus_reduction - minus_reduction) / (2.0 * step)) @ velocity
        plus_contact, _, _ = self.oracle.contact(plus, plus_reduction)
        minus_contact, _, _ = self.oracle.contact(minus, minus_reduction)
        jdot_velocity = ((plus_contact - minus_contact) / (2.0 * step)) @ velocity
        self.oracle.forward(qpos, reduction @ velocity)
        full_mass = np.zeros((self.oracle.model.nv, self.oracle.model.nv))
        mujoco.mj_fullM(self.oracle.model, full_mass, self.oracle.data.qM)
        mass = reduction.T @ full_mass @ reduction
        bias = reduction.T @ (self.oracle.data.qfrc_bias.copy() + full_mass @ ndot_velocity)
        contact, points, _ = self.oracle.contact(qpos, reduction)
        actuator = reduction.T @ self.full_actuator
        self.oracle.forward(qpos)
        base = self.oracle.data.site_xpos[self.oracle.base_control_site].copy()
        wrench_map = np.zeros((12, 6))
        for side in range(2):
            offset = points[side] - base
            skew = np.asarray([[0.0, -offset[2], offset[1]],
                               [offset[2], 0.0, -offset[0]],
                               [-offset[1], offset[0], 0.0]])
            wrench_map[6 * side:6 * side + 3, 3 * side:3 * side + 3] = np.eye(3)
            wrench_map[6 * side + 3:6 * side + 6, 3 * side:3 * side + 3] = skew
        return {"mass": mass, "bias": bias, "contact": contact,
                "contact_bias": jdot_velocity, "actuator": actuator,
                "wrench_map": wrench_map, "closure": np.asarray([metrics["closure_residual_m"]]),
                "reduction": reduction, "contact_points": points}

    @staticmethod
    def add_objective(h: np.ndarray, g: np.ndarray, matrix: np.ndarray,
                      target: np.ndarray, scale: np.ndarray, weight: float,
                      transform: np.ndarray) -> None:
        if weight == 0.0:
            return
        normalized = (matrix @ transform) / scale[:, None]
        rhs = target / scale
        h += weight * normalized.T @ normalized
        g -= weight * normalized.T @ rhs

    def solve(self, plant_qpos: np.ndarray, plant_qvel: np.ndarray,
              enabled: set[str] | None = None, wrench_fidelity_enabled: bool = True,
              wrench_slack_penalty_override: float | None = None,
              capture_problem: bool = False) -> dict[str, Any]:
        if enabled is None:
            enabled = {"contact", "base_x", "height", "orientation", "leg"}
        qpos = self.canonical_qpos(plant_qpos)
        velocity = self.reduced_velocity(qpos, plant_qvel)
        model = self.model(qpos, velocity)
        qpc = self.qp
        objective = {**qpc["objective"], **self.config.get("objective", {})}
        wrench_slack_penalty = objective["wrench_slack_penalty"]
        if wrench_slack_penalty_override is not None:
            wrench_slack_penalty = wrench_slack_penalty_override
        if not wrench_fidelity_enabled:
            wrench_slack_penalty = 0.0
        h = np.diag(
            [objective["acceleration_regularization"]] * 12 +
            [objective["torque_regularization"]] * 6 +
            [objective["contact_force_regularization"]] * 6 +
            [wrench_slack_penalty] * 12
        )
        g = np.zeros(36)
        task = self.config["task"]
        task_specs: list[dict[str, Any]] = []
        contact_task = np.zeros((6, 36)); contact_task[:, :12] = model["contact"]
        task_specs.append({"name": "contact", "matrix": contact_task,
                           "target": -model["contact_bias"],
                           "scale": np.full(6, task["contact_acceleration_scale_m_s2"]),
                           "weight": task["contact_acceleration_weight"],
                           "enabled": "contact" in enabled})
        self.add_objective(h, g, contact_task, -model["contact_bias"],
                           np.full(6, task["contact_acceleration_scale_m_s2"]),
                           task["contact_acceleration_weight"] if "contact" in enabled else 0.0,
                           self.transform)
        self.oracle.forward(qpos)
        position = self.oracle.data.site_xpos[self.oracle.base_control_site].copy()
        orientation = rotation_vector(qpos[3:7])
        desired_linear = -np.asarray(task["translation_kp_s2"]) * (position - self.reference_position)
        desired_linear -= np.asarray(task["translation_kd_s"]) * velocity[:3]
        desired_angular = -np.asarray(task["orientation_kp_s2"]) * orientation
        desired_angular -= np.asarray(task["orientation_kd_s"]) * velocity[3:6]
        for name, indices, target, scale, weight in (
            ("base_x", [0], desired_linear[[0]], task["base_acceleration_scale_m_s2"], task["base_x_weight"]),
            ("height", [2], desired_linear[[2]], task["base_acceleration_scale_m_s2"], task["base_height_weight"]),
            ("orientation", [3, 4, 5], desired_angular, task["angular_acceleration_scale_rad_s2"], task["base_orientation_weight"]),
        ):
            matrix = np.zeros((len(indices), 36))
            matrix[np.arange(len(indices)), indices] = 1.0
            task_specs.append({"name": name, "matrix": matrix, "target": target,
                               "scale": np.full(len(indices), scale), "weight": weight,
                               "enabled": name in enabled})
            self.add_objective(h, g, matrix, target, np.full(len(indices), scale),
                               weight if name in enabled else 0.0, self.transform)
        canonical_active = -qpos[self.oracle.active_qpos]
        leg_indices = np.asarray([0, 1, 3, 4])
        desired_leg = task["leg_kp_s2"] * (self.reference_active[leg_indices] - canonical_active[leg_indices])
        desired_leg -= task["leg_kd_s"] * velocity[6 + leg_indices]
        leg_task = np.zeros((4, 36))
        leg_task[np.arange(4), 6 + leg_indices] = 1.0
        task_specs.append({"name": "leg", "matrix": leg_task, "target": desired_leg,
                           "scale": np.full(4, task["leg_acceleration_scale_rad_s2"]),
                           "weight": task["leg_posture_weight"], "enabled": "leg" in enabled})
        self.add_objective(h, g, leg_task, desired_leg,
                           np.full(4, task["leg_acceleration_scale_rad_s2"]),
                           task["leg_posture_weight"] if "leg" in enabled else 0.0,
                           self.transform)

        dynamics = np.zeros((12, 36)); dynamics[:, :12] = model["mass"]
        dynamics[:, 12:18] = -model["actuator"]; dynamics[:, 18:24] = -model["contact"].T
        wrench = np.zeros((12, 36)); wrench[:, 18:24] = model["wrench_map"]
        wrench[:, 24:36] = -np.eye(12)
        row_scale = qpc["row_scale"]
        equality = np.vstack((
            dynamics / np.asarray(row_scale["dynamics"])[:, None],
            wrench / np.asarray(row_scale["wrench"])[:, None],
        )) @ self.transform
        rhs = np.r_[-model["bias"] / np.asarray(row_scale["dynamics"]),
                    self.reference_wrench / np.asarray(row_scale["wrench"])]
        rows = list(equality); lower = rhs.tolist(); upper = rhs.tolist()
        limits = np.asarray(qpc["bounds"]["torque_nm"])
        torque = np.zeros((6, 36)); torque[:, 12:18] = np.eye(6)
        rows.extend((torque @ self.transform) / limits[:, None]); lower.extend((-np.ones(6)).tolist()); upper.extend(np.ones(6).tolist())
        maximum_normal = float(qpc["bounds"]["maximum_normal_force_n"])
        normal = np.zeros((2, 36)); normal[0, 20] = 1.0; normal[1, 23] = 1.0
        rows.extend((normal @ self.transform) / maximum_normal); lower.extend([0.0, 0.0]); upper.extend([1.0, 1.0])
        friction = []
        mu = float(qpc["friction_coefficient"])
        for start in (18, 21):
            for tangent in (start, start + 1):
                row = np.zeros(36); row[tangent] = 1.0; row[start + 2] = -mu; friction.append(row)
                row = np.zeros(36); row[tangent] = -1.0; row[start + 2] = -mu; friction.append(row)
        rows.extend((np.asarray(friction) @ self.transform) / maximum_normal); lower.extend([-1e4] * 8); upper.extend([0.0] * 8)
        acceleration_limits = np.asarray(qpc["bounds"]["acceleration"])
        acceleration = np.zeros((12, 36)); acceleration[:, :12] = np.eye(12)
        rows.extend((acceleration @ self.transform) / acceleration_limits[:, None]); lower.extend((-np.ones(12)).tolist()); upper.extend(np.ones(12).tolist())
        a = np.asarray(rows); low = np.asarray(lower); high = np.asarray(upper)
        warm_start_before = self.warm_start.copy()
        result = solve_admm(h, g, a, low, high, self.config["solver"], warm_start_before)
        scaled = result.pop("x", np.zeros(36)); physical = self.variable_scale * scaled
        if result["status"] == "converged":
            self.warm_start = scaled.copy()
        task_diagnostics: dict[str, Any] = {}
        for spec in task_specs:
            achieved = spec["matrix"] @ physical
            target = spec["target"]
            normalized_residual = (achieved - target) / spec["scale"]
            achieved_norm = float(np.linalg.norm(achieved))
            target_norm = float(np.linalg.norm(target))
            dot = float(achieved @ target)
            cosine = dot / (achieved_norm * target_norm) if achieved_norm > 1e-12 and target_norm > 1e-12 else float("nan")
            task_diagnostics[spec["name"]] = {
                "enabled": bool(spec["enabled"]), "target": target.tolist(),
                "achieved": achieved.tolist(), "normalized_residual": normalized_residual.tolist(),
                "residual_norm": float(np.linalg.norm(normalized_residual)),
                "weighted_cost": float(0.5 * spec["weight"] * (normalized_residual @ normalized_residual)) if spec["enabled"] else 0.0,
                "target_norm": target_norm, "achieved_norm": achieved_norm,
                "direction_dot": dot, "direction_cosine": cosine,
            }
        torque_margin = np.asarray(qpc["bounds"]["torque_nm"]) - np.abs(physical[12:18])
        normal_forces = physical[[20, 23]]
        friction_margins = []
        for start in (18, 21):
            friction_margins.extend((mu * physical[start + 2] - abs(physical[start]),
                                     mu * physical[start + 2] - abs(physical[start + 1])))
        acceleration_margin = np.asarray(qpc["bounds"]["acceleration"]) - np.abs(physical[:12])
        active_tolerance = float(self.config.get("attribution", {}).get("active_tolerance", 1e-5))
        bounds = {
            "minimum_torque_margin_nm": float(np.min(torque_margin)),
            "minimum_normal_force_n": float(np.min(normal_forces)),
            "minimum_friction_margin_n": float(np.min(friction_margins)),
            "minimum_acceleration_margin": float(np.min(acceleration_margin)),
            "active_torque_count": int(np.count_nonzero(torque_margin <= active_tolerance)),
            "active_normal_count": int(np.count_nonzero((normal_forces <= active_tolerance) | (maximum_normal - normal_forces <= active_tolerance))),
            "active_friction_count": int(np.count_nonzero(np.asarray(friction_margins) <= active_tolerance)),
            "active_acceleration_count": int(np.count_nonzero(acceleration_margin <= active_tolerance)),
        }
        audit = {
            "model_contact_generalized_force": model["contact"].T @ physical[18:24],
            "reduction": model["reduction"], "contact_points": model["contact_points"],
        }
        if capture_problem:
            audit.update({"H": h.copy(), "g": g.copy(), "A": a.copy(), "l": low.copy(),
                          "u": high.copy(), "variable_scale": self.variable_scale.copy(),
                          "row_scale_dynamics": np.asarray(row_scale["dynamics"], dtype=float),
                          "row_scale_wrench": np.asarray(row_scale["wrench"], dtype=float),
                          "warm_start_before": warm_start_before, "scaled_solution": scaled.copy(),
                          "reference_wrench": self.reference_wrench.copy()})
        result.update({
            "physical": physical,
            "hard_residual": float(np.max(np.abs(equality @ scaled - rhs))),
            "closure_residual_m": float(model["closure"][0]),
            "maximum_abs_slack": float(np.max(np.abs(physical[24:36]))),
            "position": position,
            "orientation": orientation,
            "velocity": velocity,
            "task_diagnostics": task_diagnostics,
            "bound_diagnostics": bounds,
            "audit": audit,
            "wrench_fidelity_enabled": bool(wrench_fidelity_enabled),
            "wrench_slack_penalty": float(wrench_slack_penalty),
        })
        return result


class Plant:
    def __init__(self, model_config: dict[str, Any], equilibrium: dict[str, Any]) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / model_config["scene"]))
        self.data = mujoco.MjData(self.model)
        self.base_body = object_id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_body")
        self.base_site = object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, model_config["base_control_site"])
        self.floor = object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.wheel_geoms = [object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in ("left_wheel_collision", "right_wheel_collision")]
        self.wheel_bodies = [object_id(self.model, mujoco.mjtObj.mjOBJ_BODY, name) for name in model_config["wheel_bodies"]]
        self.active_joints = [object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in model_config["canonical_active_joints"]]
        self.passive_joints = [object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in model_config["passive_joints"]]
        self.actuators = [object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in model_config["actuators"]]
        self.base_weld = object_id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
        self.equilibrium = np.asarray(equilibrium["candidate"], dtype=float)
        self.reset()

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.eq_active[self.base_weld] = 0
        c = self.equilibrium
        self.data.qpos[:3] = (0.0, 0.0, c[4]); self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        active = np.asarray([c[2], c[3], 0.0, c[0], c[1], 0.0])
        passive = np.asarray([c[7], c[8], c[5], c[6]])
        for joint, value in zip(self.active_joints, active): self.data.qpos[self.model.jnt_qposadr[joint]] = value
        for joint, value in zip(self.passive_joints, passive): self.data.qpos[self.model.jnt_qposadr[joint]] = value
        mujoco.mj_forward(self.model, self.data)

    def metrics(self) -> dict[str, float]:
        loads = [0.0, 0.0]; penetration = 0.0
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            for side, geom in enumerate(self.wheel_geoms):
                if {contact.geom1, contact.geom2} != {self.floor, geom}: continue
                local = np.zeros(6); mujoco.mj_contactForce(self.model, self.data, index, local)
                world = contact.frame.reshape(3, 3).T @ local[:3]
                if contact.geom2 != geom: world = -world
                loads[side] += world[2]; penetration = max(penetration, max(0.0, -contact.dist))
        rolling = 0.0; lateral = 0.0
        for body, joint in zip(self.wheel_bodies, (self.active_joints[2], self.active_joints[5])):
            velocity = np.zeros(6); mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
            rolling = max(rolling, abs(velocity[3] - 0.05 * self.data.qvel[self.model.jnt_dofadr[joint]]))
            lateral = max(lateral, abs(velocity[4]))
        closure = 0.0
        for side in ("left", "right"):
            first = object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_connect2_site")
            second = object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_calf_site")
            closure = max(closure, float(np.linalg.norm(self.data.site_xpos[first] - self.data.site_xpos[second])))
        linear = np.zeros((3, self.model.nv)); angular = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, linear, angular, self.base_site)
        q = self.data.qpos[3:7]
        return {"x_m": float(self.data.site_xpos[self.base_site, 0]), "y_m": float(self.data.site_xpos[self.base_site, 1]),
                "height_m": float(self.data.site_xpos[self.base_site, 2]), "roll_rad": float(rotation_vector(q)[0]),
                "pitch_rad": float(rotation_vector(q)[1]), "yaw_rad": float(rotation_vector(q)[2]),
                "linear_speed_m_s": float(np.linalg.norm(linear @ self.data.qvel)),
                "angular_speed_rad_s": float(np.linalg.norm(angular @ self.data.qvel)),
                "left_normal_n": loads[0], "right_normal_n": loads[1], "penetration_m": penetration,
                "rolling_slip_m_s": rolling, "lateral_slip_m_s": lateral, "closure_residual_m": closure}

    def contact_truth(self, reduction: np.ndarray) -> dict[str, Any]:
        """Resolve mesh-contact forces and their reduced generalized force offline."""
        forces = np.zeros((2, 3)); moments = np.zeros((2, 3))
        cop_weighted = np.zeros((2, 3)); normal_load = np.zeros(2)
        generalized = np.zeros(self.model.nv)
        contact_counts = np.zeros(2, dtype=int)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            for side, geom in enumerate(self.wheel_geoms):
                if {contact.geom1, contact.geom2} != {self.floor, geom}:
                    continue
                local = np.zeros(6); mujoco.mj_contactForce(self.model, self.data, index, local)
                frame = contact.frame.reshape(3, 3).T
                force = frame @ local[:3]; torque = frame @ local[3:]
                if contact.geom2 != geom:
                    force = -force; torque = -torque
                point = contact.pos.copy()
                center = self.data.xpos[self.wheel_bodies[side]].copy()
                forces[side] += force
                moments[side] += torque + np.cross(point - center, force)
                load = max(0.0, float(force[2]))
                cop_weighted[side] += load * point; normal_load[side] += load
                contact_counts[side] += 1
                contribution = np.zeros(self.model.nv)
                mujoco.mj_applyFT(self.model, self.data, force, torque, point,
                                  self.wheel_bodies[side], contribution)
                generalized += contribution
        cop = np.full((2, 3), np.nan)
        valid = normal_load > 1e-12
        cop[valid] = cop_weighted[valid] / normal_load[valid, None]
        reduced = reduction.T @ generalized
        return {"forces": forces, "moments_about_wheel": moments, "cop": cop,
                "contact_counts": contact_counts, "full_generalized_force": generalized,
                "reduced_generalized_force": reduced}


def run_case(controller: ControllerOracle, plant: Plant, case: dict[str, Any], ticks: int,
             disturbance_start: int, disturbance_ticks: int, writer: csv.DictWriter) -> dict[str, Any]:
    plant.reset(); controller.reset(); reference_height = controller.reference_position[2]
    maxima = {"abs_x_m": 0.0, "abs_y_m": 0.0, "height_error_m": 0.0, "abs_roll_rad": 0.0,
              "abs_pitch_rad": 0.0, "abs_yaw_rad": 0.0, "penetration_m": 0.0,
              "rolling_slip_m_s": 0.0, "lateral_slip_m_s": 0.0, "closure_residual_m": 0.0,
              "hard_residual": 0.0, "bound_violation": 0.0, "wrench_slack": 0.0}
    bilateral = 0; failures = 0; saturation = 0; iteration_max = 0
    for tick in range(ticks):
        solved = controller.solve(plant.data.qpos.copy(), plant.data.qvel.copy())
        valid = solved["status"] == "converged" and np.all(np.isfinite(solved["physical"]))
        failures += int(not valid)
        torque = solved["physical"][12:18] if valid else np.zeros(6)
        limits = np.asarray(controller.qp["bounds"]["torque_nm"])
        saturation += int(np.any(np.abs(torque) >= limits - 1e-6))
        plant.data.ctrl[:] = 0.0
        for actuator, value in zip(plant.actuators, torque): plant.data.ctrl[actuator] = -value
        for _ in range(5):
            plant.data.xfrc_applied[plant.base_body, :] = 0.0
            if disturbance_start <= tick < disturbance_start + disturbance_ticks:
                plant.data.xfrc_applied[plant.base_body, :3] = np.asarray(case.get("force_n", [0.0, 0.0, 0.0]))
                plant.data.xfrc_applied[plant.base_body, 3:] = np.asarray(case.get("moment_nm", [0.0, 0.0, 0.0]))
            mujoco.mj_step(plant.model, plant.data)
        metrics = plant.metrics(); bilateral += int(metrics["left_normal_n"] > 0.0 and metrics["right_normal_n"] > 0.0)
        maxima["abs_x_m"] = max(maxima["abs_x_m"], abs(metrics["x_m"] - controller.reference_position[0]))
        maxima["abs_y_m"] = max(maxima["abs_y_m"], abs(metrics["y_m"] - controller.reference_position[1]))
        maxima["height_error_m"] = max(maxima["height_error_m"], abs(metrics["height_m"] - reference_height))
        for name in ("roll", "pitch", "yaw"): maxima[f"abs_{name}_rad"] = max(maxima[f"abs_{name}_rad"], abs(metrics[f"{name}_rad"]))
        for name in ("penetration_m", "rolling_slip_m_s", "lateral_slip_m_s", "closure_residual_m"):
            maxima[name] = max(maxima[name], metrics[name])
        maxima["hard_residual"] = max(maxima["hard_residual"], solved["hard_residual"])
        maxima["bound_violation"] = max(maxima["bound_violation"], solved.get("bound_violation", float("inf")))
        maxima["wrench_slack"] = max(maxima["wrench_slack"], solved["maximum_abs_slack"])
        iteration_max = max(iteration_max, int(solved.get("iterations", 0)))
        writer.writerow({"case": case["id"], "tick": tick, "time_s": plant.data.time,
                         "solver_status": solved["status"], "iterations": solved.get("iterations", 0),
                         **metrics, "hard_residual": solved["hard_residual"],
                         "bound_violation": solved.get("bound_violation", float("inf")),
                         "wrench_slack": solved["maximum_abs_slack"],
                         **{f"tau{index}": value for index, value in enumerate(torque)}})
    final = plant.metrics()
    return {**maxima, "bilateral_contact_fraction": bilateral / ticks,
            "minimum_normal_force_n": min(final["left_normal_n"], final["right_normal_n"]),
            "final_linear_speed_m_s": final["linear_speed_m_s"],
            "final_angular_speed_rad_s": final["angular_speed_rad_s"],
            "solver_failure_count": failures, "saturation_count": saturation,
            "maximum_iterations": iteration_max}


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True); parser.add_argument("--case", action="append")
    parser.add_argument("--ticks", type=int)
    args = parser.parse_args(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_path = args.config.resolve(); config, config_inputs = load_config(config_path)
    model_path = (ROOT / config["model_profile"]).resolve(); model_config, model_inputs = load_config(model_path)
    qp_path = (ROOT / config["qp_profile"]).resolve(); qp_config, qp_inputs = load_config(qp_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve(); equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    controller = ControllerOracle(config, model_config, equilibrium, qp_config); plant = Plant(model_config, equilibrium)
    cases = [("tuning", case) for case in config["tuning_cases"]] + [("holdout", case) for case in config["holdout_cases"]]
    if args.case: cases = [(group, case) for group, case in cases if case["id"] in set(args.case)]
    fields = ["case", "tick", "time_s", "solver_status", "iterations", "x_m", "y_m", "height_m", "roll_rad", "pitch_rad", "yaw_rad", "linear_speed_m_s", "angular_speed_rad_s", "left_normal_n", "right_normal_n", "penetration_m", "rolling_slip_m_s", "lateral_slip_m_s", "closure_residual_m", "hard_residual", "bound_violation", "wrench_slack"] + [f"tau{i}" for i in range(6)]
    results: dict[str, Any] = {}; ticks = args.ticks or int(round(config["duration_s"] / (plant.model.opt.timestep * config["physics_steps_per_control"])))
    with (output / "ticks.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for group, case in cases:
            results[case["id"]] = {"group": group, **run_case(controller, plant, case, ticks, config["disturbance_start_tick"], config["disturbance_ticks"], writer)}
    gates_config = config["gates"]
    case_gates = {}
    for case, value in results.items():
        case_gates[case] = {
            "position": bool(value["abs_x_m"] <= gates_config["maximum_abs_x_m"] and value["abs_y_m"] <= gates_config["maximum_abs_y_m"] and value["height_error_m"] <= gates_config["maximum_height_error_m"]),
            "orientation": bool(value["abs_roll_rad"] <= gates_config["maximum_abs_roll_rad"] and value["abs_pitch_rad"] <= gates_config["maximum_abs_pitch_rad"] and value["abs_yaw_rad"] <= gates_config["maximum_abs_yaw_rad"]),
            "settling": bool(value["final_linear_speed_m_s"] <= gates_config["maximum_final_linear_speed_m_s"] and value["final_angular_speed_rad_s"] <= gates_config["maximum_final_angular_speed_rad_s"]),
            "contact": bool(value["bilateral_contact_fraction"] >= gates_config["minimum_bilateral_contact_fraction"] and value["minimum_normal_force_n"] >= gates_config["minimum_normal_force_n"]),
            "plant": bool(value["penetration_m"] <= gates_config["maximum_penetration_m"] and value["rolling_slip_m_s"] <= gates_config["maximum_abs_rolling_slip_m_s"] and value["lateral_slip_m_s"] <= gates_config["maximum_abs_lateral_slip_m_s"] and value["closure_residual_m"] <= gates_config["maximum_closure_residual_m"]),
            "qp": bool(value["hard_residual"] <= gates_config["maximum_hard_residual"] and value["bound_violation"] <= gates_config["maximum_bound_violation"] and value["wrench_slack"] <= gates_config["maximum_abs_wrench_slack"] and value["solver_failure_count"] <= gates_config["maximum_solver_failure_count"] and value["saturation_count"] <= gates_config["maximum_saturation_count"]),
        }
    summary = {"schema_version": 1, "phase": 21, "profile": config["profile"], "ticks_per_case": ticks,
               "results": results, "case_gates": case_gates,
               "pass": bool(case_gates) and all(all(g.values()) for g in case_gates.values())}
    write_json(output / "summary.json", summary)
    write_json(output / "manifest.json", {"schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "numpy": np.__version__, "mujoco": mujoco.__version__, "hardware_data": False,
        "config": str(config_path.relative_to(ROOT)), "config_sha256": sha256(config_path),
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "model_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "qp_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in qp_inputs},
        "validator": str(Path(__file__).resolve().relative_to(ROOT)), "validator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {name: sha256(output / name) for name in ("summary.json", "ticks.csv")}})
    print(json.dumps(summary, indent=2, sort_keys=True)); return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
