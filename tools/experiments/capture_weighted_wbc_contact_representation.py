#!/usr/bin/env python3
"""Capture canonical-state wheel geometry and plant contact truth before Phase-21 failure."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_tasks import ControllerOracle, Plant, ROOT  # noqa: E402


DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase21_contact_representation_audit.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def mesh_vertices(model: mujoco.MjModel, geom: int) -> np.ndarray:
    mesh = int(model.geom_dataid[geom])
    start = int(model.mesh_vertadr[mesh])
    count = int(model.mesh_vertnum[mesh])
    return model.mesh_vert[start:start + count].copy()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_path = args.config.resolve(); config, config_inputs = load_config(config_path)
    model_path = (ROOT / config["model_profile"]).resolve(); model_config, model_inputs = load_config(model_path)
    qp_path = (ROOT / config["qp_profile"]).resolve(); qp_config, qp_inputs = load_config(qp_path)
    equilibrium_path = (ROOT / model_config["equilibrium"]).resolve()
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))
    controller = ControllerOracle(config, model_config, equilibrium, qp_config)
    plant = Plant(model_config, equilibrium)
    audit_config = config["contact_representation_audit"]
    ticks = int(audit_config["ticks"])
    arrays: dict[str, list[np.ndarray]] = {name: [] for name in (
        "qpos", "qvel", "geom_position", "geom_rotation", "wheel_center", "truth_force",
        "truth_moment_about_wheel", "truth_cop", "truth_contact_counts",
        "model_contact_points", "model_contact_generalized_force",
        "truth_contact_generalized_force", "physical_solution")}
    scalar: dict[str, list[float]] = {name: [] for name in
        ("solver_converged", "solver_iterations", "solver_bound_violation")}
    for tick in range(ticks):
        # mj_step integrates qpos after its final forward pass; refresh all pose/contact
        # fields so the captured canonical state and derived geometry share one tick.
        mujoco.mj_forward(plant.model, plant.data)
        solved = controller.solve(plant.data.qpos.copy(), plant.data.qvel.copy())
        truth = plant.contact_truth(solved["audit"]["reduction"])
        arrays["qpos"].append(plant.data.qpos.copy())
        arrays["qvel"].append(plant.data.qvel.copy())
        arrays["geom_position"].append(plant.data.geom_xpos[plant.wheel_geoms].copy())
        arrays["geom_rotation"].append(plant.data.geom_xmat[plant.wheel_geoms].reshape(2, 3, 3).copy())
        arrays["wheel_center"].append(plant.data.xpos[plant.wheel_bodies].copy())
        arrays["truth_force"].append(truth["forces"])
        arrays["truth_moment_about_wheel"].append(truth["moments_about_wheel"])
        arrays["truth_cop"].append(truth["cop"])
        arrays["truth_contact_counts"].append(truth["contact_counts"])
        arrays["model_contact_points"].append(solved["audit"]["contact_points"])
        arrays["model_contact_generalized_force"].append(solved["audit"]["model_contact_generalized_force"])
        arrays["truth_contact_generalized_force"].append(truth["reduced_generalized_force"])
        arrays["physical_solution"].append(solved["physical"])
        scalar["solver_converged"].append(float(solved["status"] == "converged"))
        scalar["solver_iterations"].append(float(solved.get("iterations", 0)))
        scalar["solver_bound_violation"].append(float(solved.get("bound_violation", np.nan)))
        valid = solved["status"] == "converged" and np.all(np.isfinite(solved["physical"]))
        torque = solved["physical"][12:18] if valid else np.zeros(6)
        plant.data.ctrl[:] = 0.0
        for actuator, value in zip(plant.actuators, torque):
            plant.data.ctrl[actuator] = -value
        for _ in range(int(config["physics_steps_per_control"])):
            mujoco.mj_step(plant.model, plant.data)
    capture_path = output / "capture.npz"
    np.savez_compressed(capture_path, tick=np.arange(ticks),
        mesh_vertices_left=mesh_vertices(plant.model, plant.wheel_geoms[0]),
        mesh_vertices_right=mesh_vertices(plant.model, plant.wheel_geoms[1]),
        **{name: np.asarray(values) for name, values in arrays.items()},
        **{name: np.asarray(values) for name, values in scalar.items()})
    first_failure = next((index for index, value in enumerate(scalar["solver_converged"]) if not value), None)
    summary = {"schema_version": 1, "phase": 21, "profile": config["profile"],
               "purpose": "contact_representation_decision_capture_only",
               "ticks": ticks, "first_solver_failure_tick": first_failure,
               "controller_contact_truth_feedback": False,
               "mesh_vertex_semantics": "compiled MuJoCo wheel mesh vertices transformed by geom pose for offline validation only"}
    write_json(output / "summary.json", summary)
    sources = [Path(__file__).resolve(), ROOT / "tools/experiments/validate_weighted_wbc_tasks.py",
               ROOT / "tools/experiments/validate_weighted_wbc_qp.py",
               ROOT / "tools/experiments/validate_mujoco_weighted_wbc_model.py"]
    write_json(output / "manifest.json", {"schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
        "numpy": np.__version__, "mujoco": mujoco.__version__, "hardware_data": False,
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "model_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in model_inputs},
        "qp_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in qp_inputs},
        "sources": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
        "outputs": {"capture.npz": sha256(capture_path),
                    "summary.json": sha256(output / "summary.json")}})
    print(json.dumps(summary, indent=2, sort_keys=True)); return 0


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
