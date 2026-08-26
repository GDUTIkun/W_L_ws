#!/usr/bin/env python3
"""Solve and audit the Phase-19 zero-wheel-torque planar equilibrium."""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase19_planar_equilibrium.json"
DEFAULT_SCENE = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-model/phase19_planar_scene.xml"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-equilibrium"
STEP_FACTORS = (1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125, 0.015625, 0.0078125)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"Missing required MuJoCo object: {name}")
    return int(result)


class Problem:
    def __init__(self, scene: Path, config: dict[str, Any]) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.config = config
        expected_timestep = float(config["physics_timestep_s"])
        if self.model.opt.timestep != expected_timestep:
            raise RuntimeError(
                f"Expected timestep {expected_timestep}, got {self.model.opt.timestep}"
            )
        self.base_weld = object_id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, config["disabled_equality"]
        )
        self.joint_qpos = np.asarray([
            self.model.jnt_qposadr[
                object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in config["joint_order"]
        ], dtype=int)
        self.driven = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in config["driven_actuators"]
        ], dtype=int)
        self.zero = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in config["zero_actuators"]
        ], dtype=int)
        self.base_x = self.model.jnt_qposadr[
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_x_joint")
        ]
        self.base_pitch = self.model.jnt_qposadr[
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "base_pitch_joint")
        ]
        self.left_wheel_geom = object_id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_collision"
        )
        self.right_wheel_geom = object_id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_wheel_collision"
        )

    def apply(self, candidate: np.ndarray) -> tuple[np.ndarray, int]:
        data = self.data
        data.qpos[:] = self.model.qpos0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[self.base_x] = 0.0
        data.qpos[self.base_pitch] = 0.0
        data.qpos[self.joint_qpos] = candidate[:9]
        data.ctrl[self.driven] = candidate[9:]
        data.ctrl[self.zero] = 0.0
        data.eq_active[:] = self.model.eq_active0
        data.eq_active[self.base_weld] = 0
        mujoco.mj_forward(self.model, data)
        return data.qacc.copy(), int(data.ncon)


def solve(problem: Problem) -> tuple[np.ndarray, list[dict[str, Any]]]:
    settings = problem.config["solver"]
    candidate = np.asarray(problem.config["seed"], dtype=float)
    scales = np.asarray(settings["scales"], dtype=float)
    if candidate.shape != (13,) or scales.shape != (13,):
        raise RuntimeError("Equilibrium seed and scales must each contain 13 values")
    damping = float(settings["initial_damping"])
    epsilon = float(settings["finite_difference_relative_step"])
    target = float(settings["acceleration_target"])
    trace: list[dict[str, Any]] = []

    for iteration in range(int(settings["maximum_iterations"])):
        residual, contacts = problem.apply(candidate)
        maximum = float(np.max(np.abs(residual)))
        trace.append({
            "iteration": iteration,
            "cost": float(0.5 * residual @ residual),
            "maximum_acceleration": maximum,
            "contacts": contacts,
            "damping": damping,
        })
        if maximum <= target:
            return candidate, trace
        jacobian = np.empty((problem.model.nv, candidate.size))
        for column in range(candidate.size):
            step = epsilon * scales[column]
            plus, minus = candidate.copy(), candidate.copy()
            plus[column] += step
            minus[column] -= step
            plus_residual, plus_contacts = problem.apply(plus)
            minus_residual, minus_contacts = problem.apply(minus)
            if plus_contacts != contacts or minus_contacts != contacts:
                raise RuntimeError("Contact topology changed during finite difference")
            jacobian[:, column] = (plus_residual - minus_residual) / (2.0 * step)
        scaled = jacobian * scales
        normal = scaled.T @ scaled + damping * np.eye(candidate.size)
        step = scales * np.linalg.solve(normal, -scaled.T @ residual)
        cost = float(0.5 * residual @ residual)
        accepted = False
        for factor in STEP_FACTORS:
            proposal = candidate + factor * step
            proposal_residual, proposal_contacts = problem.apply(proposal)
            proposal_cost = float(0.5 * proposal_residual @ proposal_residual)
            if proposal_contacts == contacts and proposal_cost < cost:
                candidate = proposal
                damping = max(damping / 3.0, 1e-14)
                accepted = True
                break
        if not accepted:
            damping = min(damping * 10.0, 1e14)
    residual, _ = problem.apply(candidate)
    raise RuntimeError(
        f"Equilibrium solve did not converge: max acceleration {np.max(np.abs(residual)):.9g}"
    )


def wheel_loads(problem: Problem) -> dict[str, float]:
    loads = {"left": 0.0, "right": 0.0}
    wrench = np.zeros(6)
    for index, contact in enumerate(problem.data.contact[: problem.data.ncon]):
        mujoco.mj_contactForce(problem.model, problem.data, index, wrench)
        geoms = {int(contact.geom1), int(contact.geom2)}
        if problem.left_wheel_geom in geoms:
            loads["left"] += float(wrench[0])
        if problem.right_wheel_geom in geoms:
            loads["right"] += float(wrench[0])
    return loads


def audit(problem: Problem, candidate: np.ndarray) -> dict[str, Any]:
    acceleration, contacts = problem.apply(candidate)
    data, thresholds = problem.data, problem.config["thresholds"]
    force_residual = data.qfrc_bias - data.qfrc_actuator - data.qfrc_constraint
    loads = wheel_loads(problem)
    closure = np.asarray(data.efc_pos[:6]).copy()
    initial_qpos, initial_qvel = data.qpos.copy(), data.qvel.copy()
    mujoco.mj_step(problem.model, data)
    qpos_drift = float(np.max(np.abs(data.qpos - initial_qpos)))
    qvel_drift = float(np.max(np.abs(data.qvel - initial_qvel)))
    metrics = {
        "maximum_acceleration_rad_m_s2": float(np.max(np.abs(acceleration))),
        "maximum_generalized_force_residual": float(np.max(np.abs(force_residual))),
        "maximum_closure_residual_m": float(np.max(np.abs(closure))),
        "wheel_normal_load_n": loads,
        "maximum_support_torque_nm": float(np.max(np.abs(candidate[9:]))),
        "active_side_difference_rad": {
            "hip": float(abs(candidate[0] - candidate[2])),
            "knee": float(abs(candidate[1] - candidate[3])),
        },
        "one_step_qpos_drift": qpos_drift,
        "one_step_qvel_drift": qvel_drift,
        "contact_count": contacts,
        "finite": bool(np.all(np.isfinite(candidate)) and np.all(np.isfinite(acceleration))),
        "wheel_torque_exact_zero": bool(np.all(data.ctrl[problem.zero] == 0.0)),
    }
    gates = {
        "acceleration": metrics["maximum_acceleration_rad_m_s2"] <= thresholds["maximum_acceleration_rad_m_s2"],
        "generalized_force": metrics["maximum_generalized_force_residual"] <= thresholds["maximum_generalized_force_residual"],
        "closure": metrics["maximum_closure_residual_m"] <= thresholds["maximum_closure_residual_m"],
        "bilateral_load": min(loads.values()) >= thresholds["minimum_wheel_normal_load_n"],
        "support_torque": metrics["maximum_support_torque_nm"] <= thresholds["maximum_support_torque_nm"],
        "side_difference": max(metrics["active_side_difference_rad"].values()) <= thresholds["maximum_active_side_difference_rad"],
        "one_step_qpos": qpos_drift <= thresholds["one_step_qpos_drift"],
        "one_step_qvel": qvel_drift <= thresholds["one_step_qvel_drift"],
        "finite": metrics["finite"],
        "zero_wheel_torque": metrics["wheel_torque_exact_zero"],
    }
    return {"pass": all(gates.values()), "metrics": metrics, "gates": gates}


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    scene, config_path = arguments.scene.resolve(), arguments.config.resolve()
    config = json.loads(config_path.read_text())
    problem = Problem(scene, config)
    candidate, trace = solve(problem)
    validation = audit(problem, candidate)
    write_json(output / "equilibrium.json", {
        "schema_version": 1,
        "candidate_order": config["joint_order"] + config["driven_actuators"],
        "candidate": candidate.tolist(),
        "iterations": len(trace),
    })
    write_json(output / "solver_trace.json", trace)
    write_json(output / "summary.json", validation)
    write_json(output / "manifest.json", {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "scene": str(scene), "scene_sha256": sha256(scene),
        "config": str(config_path), "config_sha256": sha256(config_path),
        "solver_sha256": sha256(Path(__file__).resolve()),
        "outputs": {
            name: sha256(output / name)
            for name in ("equilibrium.json", "solver_trace.json", "summary.json")
        },
    })
    print(json.dumps(validation, indent=2, sort_keys=True))
    return 0 if validation["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
