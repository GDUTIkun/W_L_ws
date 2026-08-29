#!/usr/bin/env python3
"""Generate independent MuJoCo goldens for Phase-27 wheel contracts."""

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
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/experiments"))
from validate_mujoco_weighted_wbc_model import Oracle, load_config, object_id  # noqa: E402

DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase27_wheel_interaction_oracle_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def jacobian(oracle: Oracle, body: int, point: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    linear = np.zeros((3, oracle.model.nv))
    angular = np.zeros((3, oracle.model.nv))
    mujoco.mj_jac(oracle.model, oracle.data, linear, angular, point, body)
    return linear, angular


def snapshot(oracle: Oracle, qpos: np.ndarray, reduced_velocity: np.ndarray) -> dict[str, Any]:
    reduction, _ = oracle.reduction(qpos)
    qvel = reduction @ reduced_velocity
    oracle.forward(qpos, qvel)
    bodies = []
    for body in oracle.wheel_bodies:
        origin = oracle.data.xpos[body].copy()
        com = oracle.data.xipos[body].copy()
        origin_jacobian, angular_jacobian = jacobian(oracle, body, origin)
        bodies.append({
            "origin": origin,
            "com": com,
            "origin_velocity": origin_jacobian @ qvel,
            "com_jacobian": jacobian(oracle, body, com)[0],
            "angular_jacobian": angular_jacobian,
            "omega": angular_jacobian @ qvel,
        })
    base = oracle.base_control_site
    base_linear, base_angular = oracle.site_jacobian(base)
    return {
        "reduction": reduction,
        "qvel": qvel,
        "base_position": oracle.data.site_xpos[base].copy(),
        "base_rotation": oracle.data.site_xmat[base].reshape(3, 3).copy(),
        "base_linear_velocity": base_linear @ qvel,
        "base_angular_velocity": base_angular @ qvel,
        "bodies": bodies,
    }


def evaluate_sample(
    oracle: Oracle,
    sample: dict[str, Any],
    geom_ids: list[int],
    midpoint: np.ndarray,
    radius: float,
    step: float,
) -> dict[str, Any]:
    qpos = oracle.sample_qpos(sample)
    reduced_velocity = np.asarray(sample["reduced_velocity"], dtype=float)
    current = snapshot(oracle, qpos, reduced_velocity)
    plus_qpos = oracle.integrate_flow(qpos, reduced_velocity, step)
    minus_qpos = oracle.integrate_flow(qpos, reduced_velocity, -step)
    plus = snapshot(oracle, plus_qpos, reduced_velocity)
    minus = snapshot(oracle, minus_qpos, reduced_velocity)
    rotation = current["base_rotation"]
    rotation_b_from_n = rotation.T
    omega_b = rotation_b_from_n @ current["base_angular_velocity"]

    quaternion_xyzw = Rotation.from_matrix(rotation).as_quat()
    robot_state = np.r_[
        current["base_position"],
        quaternion_xyzw[3], quaternion_xyzw[:3],
        current["base_linear_velocity"],
        current["base_angular_velocity"],
        np.asarray(oracle.config["canonical_joint_offsets_rad"])[None, :].ravel()
        - qpos[oracle.active_qpos],
        -current["qvel"][oracle.active_dofs],
    ]
    sides = []
    for side, body in enumerate(oracle.wheel_bodies):
        body_now = current["bodies"][side]
        relative_b = rotation_b_from_n @ (
            body_now["origin"] - current["base_position"]
        )
        relative_velocity_b = rotation_b_from_n @ (
            body_now["origin_velocity"] - current["base_linear_velocity"]
        ) - np.cross(omega_b, relative_b)

        oracle.forward(qpos, current["qvel"])
        geom_rotation = oracle.data.geom_xmat[geom_ids[side]].reshape(3, 3).copy()
        geom_position = oracle.data.geom_xpos[geom_ids[side]].copy()
        axis = geom_rotation[:, 0]
        normal = np.array([0.0, 0.0, 1.0])
        projection = np.sqrt(1.0 - float(axis @ normal) ** 2)
        radial = (normal - float(axis @ normal) * axis) / projection
        contact_position = geom_position + midpoint[side] * axis - radius * radial
        rolling = np.cross(axis, normal) / projection
        lateral = np.cross(normal, rolling)
        contact_frame = np.column_stack((rolling, lateral, normal))
        frame_in_base = rotation_b_from_n @ contact_frame
        contact_map = np.zeros((6, 6))
        contact_map[:3, :3] = frame_in_base
        contact_map[3:, :3] = (
            rotation_b_from_n @ skew(contact_position - body_now["origin"]) @ contact_frame
        )
        contact_map[3:, 3:] = frame_in_base

        mass = float(oracle.model.body_mass[body])
        com_jacobian = body_now["com_jacobian"]
        angular_jacobian = body_now["angular_jacobian"]
        inertia_rotation = oracle.data.ximat[body].reshape(3, 3).copy()
        inertia_world = inertia_rotation @ np.diag(oracle.model.body_inertia[body]) @ inertia_rotation.T
        com_offset = body_now["com"] - body_now["origin"]
        force_map_world = mass * com_jacobian @ current["reduction"]
        moment_map_world = (
            inertia_world @ angular_jacobian @ current["reduction"]
            + skew(com_offset) @ force_map_world
        )
        acceleration_map = -np.vstack((
            rotation_b_from_n @ force_map_world,
            rotation_b_from_n @ moment_map_world,
        ))

        com_acceleration = (
            plus["bodies"][side]["com"] - 2.0 * body_now["com"]
            + minus["bodies"][side]["com"]
        ) / (step * step)
        angular_acceleration = (
            plus["bodies"][side]["omega"] - minus["bodies"][side]["omega"]
        ) / (2.0 * step)
        gravity = np.asarray(oracle.model.opt.gravity)
        inertial_force = mass * (com_acceleration - gravity)
        inertial_moment = (
            inertia_world @ angular_acceleration
            + np.cross(body_now["omega"], inertia_world @ body_now["omega"])
            + np.cross(com_offset, inertial_force)
        )
        bias = -np.r_[
            rotation_b_from_n @ inertial_force,
            rotation_b_from_n @ inertial_moment,
        ]
        sides.append({
            "xi": float(relative_b[0]),
            "dxi": float(relative_velocity_b[0]),
            "acceleration_map": acceleration_map,
            "contact_map": contact_map,
            "bias": bias,
        })
    return {"robot_state": robot_state, "sides": sides}


def write_golden(path: Path, samples: dict[str, Any]) -> None:
    lines = [f"PHASE27_WHEEL_INTERACTION_GOLDEN_V1 {len(samples)}"]
    for sample_id, result in samples.items():
        lines.append(sample_id)
        lines.append(" ".join(f"{value:.17g}" for value in result["robot_state"]))
        lines.append(" ".join(
            f"{value:.17g}" for side in result["sides"] for value in (side["xi"], side["dxi"])
        ))
        for side in result["sides"]:
            for key in ("acceleration_map", "contact_map", "bias"):
                lines.append(" ".join(f"{value:.17g}" for value in np.ravel(side[key])))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    state_config_path = (ROOT / config["source_state_oracle"]).resolve()
    state_config = json.loads(state_config_path.read_text(encoding="utf-8"))
    model_config_path = (ROOT / state_config["source_model_oracle"]).resolve()
    model_config, model_inputs = load_config(model_config_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_config, equilibrium)
    geom_ids = [
        object_id(oracle.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in config["wheel_geoms"]
    ]
    samples = {
        sample["id"]: evaluate_sample(
            oracle, sample, geom_ids,
            np.asarray(config["wheel_axis_midpoint_m"]),
            float(config["wheel_radius_m"]),
            float(config["second_difference_step_s"]),
        )
        for sample in state_config["samples"]
    }
    output.mkdir(parents=True, exist_ok=True)
    golden_path = output / "golden.txt"
    write_golden(golden_path, samples)
    summary = {
        "schema_version": 1,
        "phase": 27,
        "profile": config["profile"],
        "sample_ids": list(samples),
        "quantity": "wheel follower on leg/base at wheel-body origin in body FLU",
        "component_order": ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"],
        "side_order": ["left", "right"],
        "thresholds": config["thresholds"],
    }
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "scipy": __import__("scipy").__version__,
        "config": str(config_path.relative_to(ROOT)),
        "config_sha256": sha256(config_path),
        "state_config_sha256": sha256(state_config_path),
        "model_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "generator": str(Path(__file__).resolve().relative_to(ROOT)),
        "generator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {"golden.txt": sha256(golden_path), "summary.json": sha256(summary_path)},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
