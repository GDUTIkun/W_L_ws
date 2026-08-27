#!/usr/bin/env python3
"""Gate the frozen lowest-eight force patch before any 78D QP work."""

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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config, object_id  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
                    encoding="utf-8")


def mesh_vertices(model: mujoco.MjModel, geom: int) -> np.ndarray:
    mesh = int(model.geom_dataid[geom])
    start = int(model.mesh_vertadr[mesh])
    count = int(model.mesh_vertnum[mesh])
    return model.mesh_vert[start:start + count].copy()


def select(vertices: np.ndarray, position: np.ndarray, rotation: np.ndarray,
           count: int) -> tuple[np.ndarray, np.ndarray]:
    world = position + vertices @ rotation.T
    indices = np.lexsort((np.arange(len(vertices)), world[:, 2]))[:count]
    return indices, world


def point_jacobians(oracle: Oracle, reduction: np.ndarray, body: int,
                    points: np.ndarray) -> np.ndarray:
    rows = []
    for point in points:
        linear = np.zeros((3, oracle.model.nv))
        angular = np.zeros((3, oracle.model.nv))
        mujoco.mj_jac(oracle.model, oracle.data, linear, angular, point, body)
        rows.append(linear @ reduction)
    return np.asarray(rows)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    source = args.capture_dir.resolve()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config_path = args.config.resolve()
    config, config_inputs = load_config(config_path)
    settings = config["patch_local_oracle"]
    count = int(settings["selector_count_per_wheel"])
    model_path = (ROOT / config["model_profile"]).resolve()
    model_config, model_inputs = load_config(model_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    oracle = Oracle(model_config, equilibrium)
    geoms = [object_id(oracle.model, mujoco.mjtObj.mjOBJ_GEOM, name)
             for name in ("left_wheel_collision", "right_wheel_collision")]
    vertices = [mesh_vertices(oracle.model, geom) for geom in geoms]
    wheel_joints = [oracle.active_joints[2], oracle.active_joints[5]]
    wheel_qpos = [int(oracle.model.jnt_qposadr[joint]) for joint in wheel_joints]

    capture_path = source / "capture.npz"
    capture = np.load(capture_path)
    if "qpos" not in capture or "qvel" not in capture:
        raise RuntimeError("Capture lacks qpos/qvel required for the canonical Jacobian audit")
    ticks = np.asarray(capture["tick"], dtype=int)
    start = int(config["contact_representation_audit"]["analysis_start_tick"])
    end = int(config["contact_representation_audit"]["analysis_end_tick"])
    dt = float(settings["control_period_s"])

    sample_rows = []
    for sample in model_config["samples"]:
        qpos = oracle.sample_qpos(sample)
        oracle.forward(qpos)
        side_rows = []
        for side, geom in enumerate(geoms):
            rotation = oracle.data.geom_xmat[geom].reshape(3, 3).copy()
            indices, world = select(vertices[side], oracle.data.geom_xpos[geom].copy(),
                                    rotation, count)
            repeat, _ = select(vertices[side], oracle.data.geom_xpos[geom].copy(),
                               rotation, count)
            side_rows.append({"side": "left" if side == 0 else "right",
                              "indices": indices.tolist(),
                              "points_world_m": world[indices].tolist(),
                              "deterministic_repeat": bool(np.array_equal(indices, repeat)),
                              "finite": bool(np.all(np.isfinite(world[indices])))})
        sample_rows.append({"sample": sample["id"], "sides": side_rows})

    ordered_indices: list[list[np.ndarray]] = [[], []]
    world_points: list[list[np.ndarray]] = [[], []]
    jacobians: list[list[np.ndarray]] = [[], []]
    reduced_velocities = []
    rolling_angles: list[list[float]] = [[], []]
    pose_errors = []
    for tick in ticks:
        qpos = capture["qpos"][tick].copy()
        qvel = capture["qvel"][tick].copy()
        reduction, _ = oracle.reduction(qpos)
        reduced_velocities.append(np.linalg.lstsq(reduction, qvel, rcond=None)[0])
        oracle.forward(qpos)
        for side, (geom, body) in enumerate(zip(geoms, oracle.wheel_bodies)):
            position = oracle.data.geom_xpos[geom].copy()
            rotation = oracle.data.geom_xmat[geom].reshape(3, 3).copy()
            pose_errors.append(float(np.max(np.abs(
                position - capture["geom_position"][tick, side]))))
            pose_errors.append(float(np.max(np.abs(
                rotation - capture["geom_rotation"][tick, side]))))
            indices, world = select(vertices[side], position, rotation, count)
            points = world[indices]
            ordered_indices[side].append(indices)
            world_points[side].append(points)
            jacobians[side].append(point_jacobians(oracle, reduction, body, points))
            rolling_angles[side].append(float(qpos[wheel_qpos[side]]))

    reduced_velocities = np.asarray(reduced_velocities)
    events = []
    set_switch_count = [0, 0]
    order_switch_count = [0, 0]
    for side, body in enumerate(oracle.wheel_bodies):
        for tick in range(max(1, start), min(end, len(ticks) - 1) + 1):
            old = ordered_indices[side][tick - 1]
            new = ordered_indices[side][tick]
            if np.array_equal(old, new):
                continue
            order_switch_count[side] += 1
            set_changed = set(old.tolist()) != set(new.tolist())
            set_switch_count[side] += int(set_changed)
            qpos = capture["qpos"][tick].copy()
            reduction, _ = oracle.reduction(qpos)
            oracle.forward(qpos)
            geom = geoms[side]
            position = oracle.data.geom_xpos[geom].copy()
            rotation = oracle.data.geom_xmat[geom].reshape(3, 3).copy()
            _, world = select(vertices[side], position, rotation, count)
            old_points_same_pose = world[old]
            new_points_same_pose = world[new]
            old_j_same_pose = point_jacobians(oracle, reduction, body, old_points_same_pose)
            new_j_same_pose = point_jacobians(oracle, reduction, body, new_points_same_pose)
            point_jump = np.linalg.norm(new_points_same_pose - old_points_same_pose, axis=1)
            jacobian_jump = np.max(np.abs(new_j_same_pose - old_j_same_pose), axis=(1, 2))
            bias_jump = ((new_j_same_pose - old_j_same_pose) / dt) @ reduced_velocities[tick]
            events.append({
                "tick": int(tick), "side": "left" if side == 0 else "right",
                "wheel_angle_rad": rolling_angles[side][tick],
                "set_changed": bool(set_changed),
                "old_indices": old.tolist(), "new_indices": new.tolist(),
                "old_points_at_new_pose_m": old_points_same_pose.tolist(),
                "new_points_at_new_pose_m": new_points_same_pose.tolist(),
                "old_reduced_jacobians_at_new_pose": old_j_same_pose.tolist(),
                "new_reduced_jacobians_at_new_pose": new_j_same_pose.tolist(),
                "slot_position_jump_m": point_jump.tolist(),
                "slot_jacobian_jump": jacobian_jump.tolist(),
                "finite_tick_contact_bias_jump_m_s2": bias_jump.tolist(),
            })

    set_events = [event for event in events if event["set_changed"]]
    maximum_position_jump = max(
        (max(event["slot_position_jump_m"]) for event in set_events), default=0.0)
    maximum_jacobian_jump = max(
        (max(event["slot_jacobian_jump"]) for event in set_events), default=0.0)
    maximum_bias_jump = max(
        (float(np.max(np.abs(event["finite_tick_contact_bias_jump_m_s2"])))
         for event in set_events), default=0.0)
    gates = {
        "pose_reconstruction": max(pose_errors, default=0.0)
        <= float(settings["maximum_pose_reconstruction_error_m"]),
        "sample_geometry_finite_deterministic": all(
            side["finite"] and side["deterministic_repeat"]
            for sample in sample_rows for side in sample["sides"]),
        "selector_position_continuity": maximum_position_jump
        <= float(settings["maximum_selector_switch_position_discontinuity_m"]),
        "selector_jacobian_continuity": maximum_jacobian_jump
        <= float(settings["maximum_selector_switch_jacobian_discontinuity"]),
    }
    passed = all(gates.values())
    summary = {
        "schema_version": 1, "phase": 21,
        "profile": config["profile"],
        "purpose": "frozen_lowest_eight_patch_local_oracle_gate_1",
        "selector": {"count_per_wheel": count,
                     "order": settings["selector_order"]},
        "analysis_ticks": [start, end],
        "order_switch_count": {"left": order_switch_count[0],
                               "right": order_switch_count[1]},
        "set_switch_count": {"left": set_switch_count[0],
                             "right": set_switch_count[1]},
        "maximum_same_pose_slot_position_jump_m": maximum_position_jump,
        "maximum_same_pose_reduced_jacobian_jump": maximum_jacobian_jump,
        "maximum_finite_tick_contact_bias_jump_m_s2": maximum_bias_jump,
        "maximum_pose_reconstruction_error": max(pose_errors, default=0.0),
        "gates": gates, "pass": passed,
        "downstream": {
            "patch_dynamics_pfaffian_oracle": "not_run_selector_continuity_gate_failed" if not passed else "authorized",
            "hard_qp_78d": "not_run_local_oracle_not_passed" if not passed else "pending",
            "dense_admm_78d_benchmark": "not_run_local_oracle_not_passed" if not passed else "pending",
        },
        "interpretation": (
            "A deterministic ordered selector is not differentially continuous when a set "
            "switch replaces a slot with a spatially distinct compiled material vertex. "
            "Jdot_nu is then undefined at the switch; the reported finite-tick value is only "
            "a lower-resolution jump proxy, not a continuous contact-bias definition."
        ),
    }
    write_json(output / "summary.json", summary)
    write_json(output / "workspace_samples.json", sample_rows)
    write_json(output / "switches.json", events)
    script_path = Path(__file__).resolve()
    write_json(output / "manifest.json", {
        "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(), "numpy": np.__version__,
        "mujoco": mujoco.__version__, "hardware_data": False,
        "config": str(config_path.relative_to(ROOT)),
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "model_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "equilibrium": str(equilibrium_path.relative_to(ROOT)),
        "equilibrium_sha256": sha256(equilibrium_path),
        "capture": str(capture_path), "capture_sha256": sha256(capture_path),
        "capture_manifest_sha256": sha256(source / "manifest.json"),
        "validator": str(script_path.relative_to(ROOT)),
        "validator_sha256": sha256(script_path),
        "outputs": {name: sha256(output / name) for name in
                    ("summary.json", "workspace_samples.json", "switches.json")},
    })
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
