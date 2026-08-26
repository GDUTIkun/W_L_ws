#!/usr/bin/env python3
"""Solve the Phase-20 upright full-3D zero-wheel-torque equilibrium."""

from __future__ import annotations

import argparse
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from solve_mujoco_planar_equilibrium import (
    audit,
    object_id,
    sha256,
    solve,
    write_json,
)

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase20_equilibrium.json"


class Problem:
    """Full freejoint variant of the shared standing-equilibrium problem."""

    def __init__(self, scene: Path, config: dict[str, Any]) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.config = config
        if self.model.opt.timestep != float(config["physics_timestep_s"]):
            raise RuntimeError("Unexpected physics timestep")
        if self.model.nq != 17 or self.model.nv != 16 or self.model.nu != 6:
            raise RuntimeError("Unexpected full-3D compiled dimensions")

        self.base_weld = object_id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY,
            config["disabled_equality"],
        )
        if self.model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            raise RuntimeError("base joint is not a freejoint")

        joint_names = [name for name in config["joint_order"] if name != "base_z_freejoint"]
        self.joint_qpos = np.asarray([
            self.model.jnt_qposadr[
                object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ]
            for name in joint_names
        ], dtype=int)
        self.driven = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in config["driven_actuators"]
        ], dtype=int)
        self.zero = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in config["zero_actuators"]
        ], dtype=int)
        self.left_wheel_geom = object_id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "left_wheel_collision"
        )
        self.right_wheel_geom = object_id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "right_wheel_collision"
        )

    def apply(self, candidate: np.ndarray) -> tuple[np.ndarray, int]:
        if candidate.shape != (13,):
            raise RuntimeError("Full-3D equilibrium candidate must contain 13 values")
        data = self.data
        data.qpos[:] = self.model.qpos0
        data.qvel[:] = 0.0
        data.ctrl[:] = 0.0
        data.qpos[:7] = (0.0, 0.0, candidate[4], 1.0, 0.0, 0.0, 0.0)
        data.qpos[self.joint_qpos] = candidate[[0, 1, 2, 3, 5, 6, 7, 8]]
        data.ctrl[self.driven] = candidate[9:]
        data.ctrl[self.zero] = 0.0
        data.eq_active[:] = self.model.eq_active0
        data.eq_active[self.base_weld] = 0
        mujoco.mj_forward(self.model, data)
        return data.qacc.copy(), int(data.ncon)


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
    config = json.loads(config_path.read_text(encoding="utf-8"))
    scene = (ROOT / config["scene"]).resolve()
    problem = Problem(scene, config)
    candidate, trace = solve(problem)
    validation = audit(problem, candidate)

    write_json(output / "equilibrium.json", {
        "schema_version": 1,
        "candidate_order": config["joint_order"] + config["driven_actuators"],
        "candidate": candidate.tolist(),
        "iterations": len(trace),
        "base_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "base_weld_active": False,
    })
    write_json(output / "solver_trace.json", trace)
    write_json(output / "summary.json", validation)
    write_json(output / "manifest.json", {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "scene": str(scene.relative_to(ROOT)),
        "scene_sha256": sha256(scene),
        "included_model": "simulation/mujoco/model/wheel_leg.xml",
        "included_model_sha256": sha256(ROOT / "simulation/mujoco/model/wheel_leg.xml"),
        "config": str(config_path.relative_to(ROOT)),
        "config_sha256": sha256(config_path),
        "solver_sha256": sha256(Path(__file__).resolve()),
        "compiled": {
            "nq": problem.model.nq, "nv": problem.model.nv,
            "nu": problem.model.nu, "neq": problem.model.neq,
            "ngeom": problem.model.ngeom,
            "timestep_s": problem.model.opt.timestep,
        },
        "outputs": {
            name: sha256(output / name)
            for name in ("equilibrium.json", "solver_trace.json", "summary.json")
        },
        "hardware_data": False,
    })
    print(json.dumps(validation, indent=2, sort_keys=True))
    return 0 if validation["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
