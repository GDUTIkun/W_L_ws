#!/usr/bin/env python3
"""Validate the Phase-21 12D reduced model against the nominal MuJoCo plant."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_model_oracle.json"


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"Missing {kind.name} named {name!r}")
    return result


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def load_config(path: Path) -> tuple[dict[str, Any], list[Path]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if "extends" not in raw:
        return raw, [path]
    base_path = (ROOT / raw["extends"]).resolve()
    base, inputs = load_config(base_path)
    merged = {**base, **{key: value for key, value in raw.items() if key != "extends"}}
    for nested in ("solver", "thresholds", "task", "gates", "objective", "bounds",
                   "row_scale", "variable_scale"):
        merged[nested] = {**base.get(nested, {}), **raw.get(nested, {})}
    return merged, inputs + [path]


def rotation_quaternion(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.r_[np.cos(0.5 * angle), np.sin(0.5 * angle) * vector / angle]


class Oracle:
    def __init__(self, config: dict[str, Any], equilibrium: dict[str, Any]) -> None:
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
        self.data = mujoco.MjData(self.model)
        if (self.model.nq, self.model.nv, self.model.nu) != (17, 16, 6):
            raise RuntimeError("Unexpected nominal full-3D model dimensions")
        self.active_joints = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in config["canonical_active_joints"]
        ]
        self.passive_joints = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in config["passive_joints"]
        ]
        self.active_qpos = np.asarray(
            [self.model.jnt_qposadr[joint] for joint in self.active_joints], dtype=int
        )
        self.active_dofs = np.asarray(
            [self.model.jnt_dofadr[joint] for joint in self.active_joints], dtype=int
        )
        self.passive_qpos = np.asarray(
            [self.model.jnt_qposadr[joint] for joint in self.passive_joints], dtype=int
        )
        self.passive_dofs = np.asarray(
            [self.model.jnt_dofadr[joint] for joint in self.passive_joints], dtype=int
        )
        self.closure_sites = [
            tuple(object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, name) for name in pair)
            for pair in config["closure_pairs"]
        ]
        self.wheel_bodies = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in config["wheel_bodies"]
        ]
        self.base_control_site = object_id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, config["base_control_site"]
        )
        self.actuators = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in config["actuators"]
        ]
        candidate = np.asarray(equilibrium["candidate"], dtype=float)
        self.base_z = float(candidate[4])
        # Candidate order is right active, left active, z, right passive, left passive.
        self.equilibrium_active = np.asarray(
            [candidate[2], candidate[3], 0.0, candidate[0], candidate[1], 0.0]
        )
        self.equilibrium_passive = np.asarray(
            [candidate[7], candidate[8], candidate[5], candidate[6]]
        )

    def site_jacobian(self, site: int) -> tuple[np.ndarray, np.ndarray]:
        linear = np.zeros((3, self.model.nv))
        angular = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, linear, angular, site)
        return linear, angular

    def closure(self) -> tuple[np.ndarray, np.ndarray]:
        residuals, rows = [], []
        for first, second in self.closure_sites:
            first_linear, _ = self.site_jacobian(first)
            second_linear, _ = self.site_jacobian(second)
            residuals.append(self.data.site_xpos[first] - self.data.site_xpos[second])
            rows.append(first_linear - second_linear)
        return np.concatenate(residuals), np.vstack(rows)

    def forward(self, qpos: np.ndarray, qvel: np.ndarray | None = None) -> None:
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0 if qvel is None else qvel
        self.data.ctrl[:] = 0.0
        self.data.eq_active[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def solve_passive(self, qpos: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
        solver = self.config["solver"]
        qpos = qpos.copy()
        step_norm = 0.0
        for iteration in range(solver["maximum_iterations"] + 1):
            self.forward(qpos)
            residual, jacobian = self.closure()
            if np.max(np.abs(residual)) <= solver["position_tolerance_m"]:
                break
            step = np.linalg.lstsq(
                jacobian[:, self.passive_dofs], -residual, rcond=solver["rcond"]
            )[0]
            step_norm = float(np.max(np.abs(step)))
            norm = float(np.linalg.norm(step))
            if norm > solver["maximum_step_rad"]:
                step *= solver["maximum_step_rad"] / norm
            qpos[self.passive_qpos] += step
            if step_norm <= solver["step_tolerance_rad"]:
                break
        else:
            iteration = solver["maximum_iterations"] + 1
        self.forward(qpos)
        residual, jacobian = self.closure()
        passive_block = jacobian[:, self.passive_dofs]
        singular = np.linalg.svd(passive_block, compute_uv=False)
        metrics = {
            "iterations": int(iteration),
            "closure_residual_m": float(np.max(np.abs(residual))),
            "last_step_rad": step_norm,
            "passive_min_singular_value": float(singular[-1]),
            "passive_condition_number": float(singular[0] / singular[-1]),
        }
        if metrics["closure_residual_m"] > solver["position_tolerance_m"]:
            raise RuntimeError(f"Passive reconstruction failed: {metrics}")
        return qpos, metrics

    def sample_qpos(self, sample: dict[str, Any]) -> np.ndarray:
        qpos = self.model.qpos0.copy()
        qpos[:3] = (0.0, 0.0, self.base_z)
        qpos[3:7] = rotation_quaternion(np.asarray(sample["base_rotation_vector_rad"]))
        canonical_delta = np.asarray(sample["canonical_joint_delta_rad"])
        qpos[self.active_qpos] = self.equilibrium_active - canonical_delta
        qpos[self.passive_qpos] = self.equilibrium_passive
        return self.solve_passive(qpos)[0]

    def reduction(self, qpos: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
        self.forward(qpos)
        residual, closure_jacobian = self.closure()
        base_linear, base_angular = self.site_jacobian(self.base_control_site)
        base_twist = np.vstack((base_linear, base_angular))[:, :6]
        reduction = np.zeros((self.model.nv, 12))
        reduction[:6, :6] = np.linalg.solve(base_twist, np.eye(6))
        reduction[self.active_dofs, 6:] = -np.eye(6)
        passive_block = closure_jacobian[:, self.passive_dofs]
        active_block = closure_jacobian[:, self.active_dofs]
        reduction[self.passive_dofs, 6:] = np.linalg.lstsq(
            passive_block, active_block, rcond=self.config["solver"]["rcond"]
        )[0]
        singular = np.linalg.svd(passive_block, compute_uv=False)
        metrics = {
            "closure_residual_m": float(np.max(np.abs(residual))),
            "constraint_tangent": float(np.max(np.abs(closure_jacobian @ reduction))),
            "base_twist_mapping": float(
                np.max(np.abs(base_twist @ reduction[:6, :6] - np.eye(6)))
            ),
            "passive_min_singular_value": float(singular[-1]),
            "passive_condition_number": float(singular[0] / singular[-1]),
        }
        return reduction, metrics

    def contact(
        self, qpos: np.ndarray, reduction: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        self.forward(qpos)
        rows, points = [], []
        configured_points = self.config.get("contact_points_local_m")
        configured_world_offsets = self.config.get("contact_points_world_offset_m")
        for side, body in enumerate(self.wheel_bodies):
            if configured_world_offsets is not None:
                point = self.data.xpos[body] + np.asarray(configured_world_offsets[side])
            else:
                local_point = np.asarray(
                    configured_points[side] if configured_points is not None else
                    self.config.get(
                        "contact_point_local_m",
                        [0.0, 0.0, -float(self.config["wheel_radius_m"])],
                    )
                )
                point = self.data.xpos[body] + self.data.xmat[body].reshape(3, 3) @ local_point
            linear = np.zeros((3, self.model.nv))
            angular = np.zeros((3, self.model.nv))
            mujoco.mj_jac(self.model, self.data, linear, angular, point, body)
            rows.append(linear)
            points.append(point)
        full = np.vstack(rows)
        return full @ reduction, np.asarray(points), full

    def integrate(self, qpos: np.ndarray, reduced_velocity: np.ndarray, step: float) -> np.ndarray:
        reduction, _ = self.reduction(qpos)
        result = qpos.copy()
        mujoco.mj_integratePos(self.model, result, reduction @ reduced_velocity, step)
        return self.solve_passive(result)[0]

    def integrate_flow(
        self, qpos: np.ndarray, reduced_velocity: np.ndarray, step: float
    ) -> np.ndarray:
        substeps = int(self.config["solver"].get("flow_integration_substeps", 1))
        result = qpos.copy()
        dt = step / substeps
        for _ in range(substeps):
            start_reduction, _ = self.reduction(result)
            midpoint = result.copy()
            mujoco.mj_integratePos(
                self.model, midpoint, start_reduction @ reduced_velocity, 0.5 * dt
            )
            midpoint = self.solve_passive(midpoint)[0]
            midpoint_reduction, _ = self.reduction(midpoint)
            next_qpos = result.copy()
            mujoco.mj_integratePos(
                self.model, next_qpos, midpoint_reduction @ reduced_velocity, dt
            )
            result = self.solve_passive(next_qpos)[0]
        return result

    def potential(self, qpos: np.ndarray) -> float:
        self.forward(qpos)
        mujoco.mj_energyPos(self.model, self.data)
        return float(self.data.energy[0])

    def evaluate(self, sample: dict[str, Any]) -> dict[str, Any]:
        qpos = self.sample_qpos(sample)
        reduction, result = self.reduction(qpos)
        self.forward(qpos)
        full_mass = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, full_mass, self.data.qM)
        reduced_mass = reduction.T @ full_mass @ reduction
        contact, points, full_contact = self.contact(qpos, reduction)

        actuator_map = np.zeros((self.model.nv, 6))
        for column, actuator in enumerate(self.actuators):
            joint = int(self.model.actuator_trnid[actuator, 0])
            actuator_map[int(self.model.jnt_dofadr[joint]), column] = -float(
                self.model.actuator_gear[actuator, 0]
            )
        reduced_actuator = reduction.T @ actuator_map
        expected_actuator = np.vstack((np.zeros((6, 6)), np.eye(6)))
        static_matrix = np.hstack((reduced_actuator, contact.T))
        static_solution = np.linalg.lstsq(static_matrix, reduction.T @ self.data.qfrc_bias, rcond=None)[0]
        static_equilibrium_residual = float(np.max(np.abs(
            static_matrix @ static_solution - reduction.T @ self.data.qfrc_bias
        )))

        step = float(self.config["solver"]["finite_difference_step"])
        passive_fd_error = 0.0
        for index in range(6):
            velocity = np.zeros(12)
            velocity[6 + index] = 1.0
            plus = self.integrate(qpos, velocity, step)
            minus = self.integrate(qpos, velocity, -step)
            derivative = (plus[self.passive_qpos] - minus[self.passive_qpos]) / (2.0 * step)
            passive_fd_error = max(
                passive_fd_error,
                float(np.max(np.abs(derivative - reduction[self.passive_dofs, 6 + index]))),
            )

        self.forward(qpos)
        gravity = reduction.T @ self.data.qfrc_bias.copy()
        gravity_energy_error = 0.0
        for index in range(12):
            velocity = np.zeros(12)
            velocity[index] = 1.0
            plus = self.integrate(qpos, velocity, step)
            minus = self.integrate(qpos, velocity, -step)
            derivative = (self.potential(plus) - self.potential(minus)) / (2.0 * step)
            gravity_energy_error = max(gravity_energy_error, abs(derivative - gravity[index]))

        test_velocity = np.asarray(
            [0.07, -0.05, 0.03, 0.11, -0.09, 0.08, 0.13, -0.10, 0.17, -0.12, 0.09, -0.16]
        )
        test_lambda = np.asarray([0.8, -0.3, 2.1, -0.6, 0.4, 1.7])
        plus = self.integrate(qpos, test_velocity, step)
        minus = self.integrate(qpos, test_velocity, -step)
        if self.config.get("contact_points_world_offset_m") is None:
            _, plus_points, _ = self.contact(plus, self.reduction(plus)[0])
            _, minus_points, _ = self.contact(minus, self.reduction(minus)[0])
        else:
            self.forward(qpos)
            local_points = [
                self.data.xmat[body].reshape(3, 3).T @ (points[side] - self.data.xpos[body])
                for side, body in enumerate(self.wheel_bodies)
            ]
            def fixed_material_points(configuration: np.ndarray) -> np.ndarray:
                self.forward(configuration)
                return np.asarray([
                    self.data.xpos[body] + self.data.xmat[body].reshape(3, 3) @ local_points[side]
                    for side, body in enumerate(self.wheel_bodies)
                ])
            plus_points = fixed_material_points(plus)
            minus_points = fixed_material_points(minus)
        point_velocity_fd = ((plus_points - minus_points) / (2.0 * step)).ravel()
        contact_velocity_error = float(np.max(np.abs(point_velocity_fd - contact @ test_velocity)))

        second_step = float(self.config["solver"].get("second_difference_step", 1e-4))
        second_plus = self.integrate_flow(qpos, test_velocity, second_step)
        second_minus = self.integrate_flow(qpos, test_velocity, -second_step)
        plus_reduction, _ = self.reduction(second_plus)
        minus_reduction, _ = self.reduction(second_minus)
        plus_contact, plus_points, _ = self.contact(second_plus, plus_reduction)
        minus_contact, minus_points, _ = self.contact(second_minus, minus_reduction)
        ndot_nu = ((plus_reduction - minus_reduction) / (2.0 * second_step)) @ test_velocity
        jdot_nu = ((plus_contact - minus_contact) / (2.0 * second_step)) @ test_velocity
        if self.config.get("contact_points_world_offset_m") is None:
            point_acceleration_fd = (
                plus_points.ravel() - 2.0 * points.ravel() + minus_points.ravel()
            ) / (second_step * second_step)
        else:
            point_acceleration_fd = (
                plus_contact @ test_velocity - minus_contact @ test_velocity
            ) / (2.0 * second_step)
        contact_bias_error = float(np.max(np.abs(point_acceleration_fd - jdot_nu)))

        plus_mass_full = np.zeros_like(full_mass)
        self.forward(second_plus)
        mujoco.mj_fullM(self.model, plus_mass_full, self.data.qM)
        plus_mass = plus_reduction.T @ plus_mass_full @ plus_reduction
        minus_mass_full = np.zeros_like(full_mass)
        self.forward(second_minus)
        mujoco.mj_fullM(self.model, minus_mass_full, self.data.qM)
        minus_mass = minus_reduction.T @ minus_mass_full @ minus_reduction
        mass_dot = (plus_mass - minus_mass) / (2.0 * second_step)
        self.forward(qpos, reduction @ test_velocity)
        velocity_bias = reduction.T @ (self.data.qfrc_bias.copy() + full_mass @ ndot_nu)
        coriolis_power_error = abs(
            float(test_velocity @ (velocity_bias - gravity))
            - 0.5 * float(test_velocity @ mass_dot @ test_velocity)
        )

        self.forward(qpos)
        base_origin = self.data.site_xpos[self.base_control_site].copy()
        wrench_map = np.zeros((12, 6))
        for side in range(2):
            offset = points[side] - base_origin
            skew = np.asarray([
                [0.0, -offset[2], offset[1]],
                [offset[2], 0.0, -offset[0]],
                [-offset[1], offset[0], 0.0],
            ])
            wrench_map[6 * side:6 * side + 3, 3 * side:3 * side + 3] = np.eye(3)
            wrench_map[6 * side + 3:6 * side + 6, 3 * side:3 * side + 3] = skew
        mapped_wrench = wrench_map @ test_lambda
        expected_wrench = np.concatenate([
            np.r_[test_lambda[3 * side:3 * side + 3],
                  np.cross(points[side] - base_origin, test_lambda[3 * side:3 * side + 3])]
            for side in range(2)
        ])
        wrench_map_error = float(np.max(np.abs(mapped_wrench - expected_wrench)))

        full_velocity = reduction @ test_velocity
        virtual_work_error = abs(
            float(full_velocity @ (full_contact.T @ test_lambda))
            - float(test_velocity @ (contact.T @ test_lambda))
        )
        force = reduced_actuator @ np.asarray([0.2, -0.1, 0.05, -0.15, 0.12, -0.04])
        force += contact.T @ test_lambda - gravity
        acceleration = np.linalg.solve(reduced_mass, force)
        forward_inverse_residual = float(
            np.max(np.abs(reduced_mass @ acceleration + gravity - reduced_actuator @ np.asarray(
                [0.2, -0.1, 0.05, -0.15, 0.12, -0.04]
            ) - contact.T @ test_lambda))
        )
        delassus = contact @ np.linalg.solve(reduced_mass, contact.T)

        result.update({
            "passive_velocity_fd_rad_s": passive_fd_error,
            "mass_symmetry": float(np.max(np.abs(reduced_mass - reduced_mass.T))),
            "mass_minimum_eigenvalue": float(np.min(np.linalg.eigvalsh(reduced_mass))),
            "actuator_map_error": float(np.max(np.abs(reduced_actuator - expected_actuator))),
            "static_equilibrium_residual": static_equilibrium_residual,
            "static_actuation_contact_rank": int(np.linalg.matrix_rank(static_matrix)),
            "gravity_energy_error_nm": float(gravity_energy_error),
            "contact_velocity_fd_m_s": contact_velocity_error,
            "contact_bias_fd_m_s2": contact_bias_error,
            "coriolis_power_error_w": float(coriolis_power_error),
            "wrench_map_error": wrench_map_error,
            "virtual_work_error_w": float(virtual_work_error),
            "forward_inverse_residual": forward_inverse_residual,
            "contact_delassus_minimum_eigenvalue": float(np.min(np.linalg.eigvalsh(delassus))),
            "left_contact_point_world_m": points[0].tolist(),
            "right_contact_point_world_m": points[1].tolist(),
            "normal_force_order_check": bool(
                contact.shape == (6, 12) and points[0, 1] > points[1, 1]
                and wrench_map[2, 2] == 1.0 and wrench_map[8, 5] == 1.0
            ),
        })
        return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config_path = arguments.config.resolve()
    config, config_inputs = load_config(config_path)
    equilibrium_path = (ROOT / config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(config, equilibrium)
    samples = {sample["id"]: oracle.evaluate(sample) for sample in config["samples"]}
    thresholds = config["thresholds"]
    gates = {
        "closure": max(value["closure_residual_m"] for value in samples.values()) <= thresholds["maximum_closure_residual_m"],
        "constraint_tangent": max(value["constraint_tangent"] for value in samples.values()) <= thresholds["maximum_constraint_tangent"],
        "base_twist": max(value["base_twist_mapping"] for value in samples.values()) <= thresholds["maximum_base_twist_mapping"],
        "passive_velocity": max(value["passive_velocity_fd_rad_s"] for value in samples.values()) <= thresholds["maximum_passive_velocity_fd_rad_s"],
        "conditioning": max(value["passive_condition_number"] for value in samples.values()) <= thresholds["maximum_passive_condition_number"] and min(value["passive_min_singular_value"] for value in samples.values()) >= thresholds["minimum_passive_singular_value"],
        "mass": max(value["mass_symmetry"] for value in samples.values()) <= thresholds["maximum_mass_symmetry"] and min(value["mass_minimum_eigenvalue"] for value in samples.values()) >= thresholds["minimum_mass_eigenvalue"],
        "actuator_map": max(value["actuator_map_error"] for value in samples.values()) <= thresholds["maximum_actuator_map_error"],
        "static_equilibrium": samples["equilibrium"]["static_equilibrium_residual"] <= thresholds["maximum_static_equilibrium_residual"],
        "gravity_energy": max(value["gravity_energy_error_nm"] for value in samples.values()) <= thresholds["maximum_gravity_energy_error_nm"],
        "contact_velocity": max(value["contact_velocity_fd_m_s"] for value in samples.values()) <= thresholds["maximum_contact_velocity_fd_m_s"],
        "contact_bias": max(value["contact_bias_fd_m_s2"] for value in samples.values()) <= thresholds["maximum_contact_bias_fd_m_s2"],
        "coriolis_power": max(value["coriolis_power_error_w"] for value in samples.values()) <= thresholds["maximum_coriolis_power_error_w"],
        "wrench_map": max(value["wrench_map_error"] for value in samples.values()) <= thresholds["maximum_wrench_map_error"],
        "virtual_work": max(value["virtual_work_error_w"] for value in samples.values()) <= thresholds["maximum_virtual_work_error_w"],
        "forward_inverse": max(value["forward_inverse_residual"] for value in samples.values()) <= thresholds["maximum_forward_inverse_residual"],
        "contact_authority": min(value["contact_delassus_minimum_eigenvalue"] for value in samples.values()) >= thresholds["minimum_contact_delassus_eigenvalue"] and all(value["normal_force_order_check"] for value in samples.values()),
    }
    summary = {
        "schema_version": 1,
        "phase": 21,
        "profile": config["profile"],
        "coordinate_order": ["base_vx_world", "base_vy_world", "base_vz_world", "base_wx_world", "base_wy_world", "base_wz_world", "left_hip", "left_knee", "left_wheel", "right_hip", "right_knee", "right_wheel"],
        "contact_force_order": ["left_rolling", "left_lateral", "left_normal", "right_rolling", "right_lateral", "right_normal"],
        "gates": gates,
        "pass": all(gates.values()),
        "samples": samples,
    }
    write_json(output / "summary.json", summary)
    write_json(output / "manifest.json", {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "mujoco": mujoco.__version__,
        "hardware_data": False,
        "config": str(config_path.relative_to(ROOT)),
        "config_sha256": sha256(config_path),
        "config_inputs": {
            str(path.relative_to(ROOT)): sha256(path) for path in config_inputs
        },
        "scene": config["scene"],
        "scene_sha256": sha256(ROOT / config["scene"]),
        "equilibrium": config["equilibrium"],
        "equilibrium_sha256": sha256(equilibrium_path),
        "validator": str(Path(__file__).resolve().relative_to(ROOT)),
        "validator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {"summary.json": sha256(output / "summary.json")},
    })
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
