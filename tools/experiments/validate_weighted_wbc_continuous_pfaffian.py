#!/usr/bin/env python3
"""Validate continuous three-row-per-wheel Pfaffian soft-task kinematics."""
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
from validate_weighted_wbc_continuous_contact import ContinuousPatch
from validate_mujoco_weighted_wbc_model import Oracle, load_config

ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()
def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False,
                               default=lambda x: x.item() if isinstance(x, np.generic) else x) + "\n")
def maxabs(x: np.ndarray) -> float: return float(np.max(np.abs(x)))
def reduced_velocity(oracle: Oracle, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
    reduction, _ = oracle.reduction(qpos)
    return np.linalg.lstsq(reduction, qvel, rcond=None)[0]


def material_point(oracle: Oracle, qpos: np.ndarray, side: int, local: np.ndarray) -> np.ndarray:
    oracle.forward(qpos); body = oracle.wheel_bodies[side]
    return oracle.data.xpos[body] + oracle.data.xmat[body].reshape(3, 3) @ local


def pc_local(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray, side: int) -> tuple[np.ndarray, dict[str, Any]]:
    g = patch.geometry(qpos, side); body = oracle.wheel_bodies[side]
    oracle.forward(qpos)
    local = oracle.data.xmat[body].reshape(3, 3).T @ (np.asarray(g["contact_center"]) - oracle.data.xpos[body])
    return local, g


def A_matrix(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray, side: int) -> np.ndarray:
    reduction, _ = oracle.reduction(qpos); local, g = pc_local(oracle, patch, qpos, side)
    point = material_point(oracle, qpos, side, local); linear = np.zeros((3, oracle.model.nv)); angular = np.zeros_like(linear)
    mujoco.mj_jac(oracle.model, oracle.data, linear, angular, point, oracle.wheel_bodies[side])
    Rc = np.column_stack((g["rolling"], g["lateral"], patch.n))
    return Rc.T @ linear @ reduction


def material_velocity_fd(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray, velocity: np.ndarray,
                         side: int, step: float) -> np.ndarray:
    local, g = pc_local(oracle, patch, qpos, side)
    plus, minus = (oracle.integrate_flow(qpos, velocity, sign * step) for sign in (1., -1.))
    world_velocity = (material_point(oracle, plus, side, local) - material_point(oracle, minus, side, local)) / (2 * step)
    Rc = np.column_stack((g["rolling"], g["lateral"], patch.n))
    return Rc.T @ world_velocity


def evaluate(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray, velocity: np.ndarray, side: int,
             settings: dict[str, Any]) -> dict[str, Any]:
    A = A_matrix(oracle, patch, qpos, side); predicted = A @ velocity
    velocity_rows = []
    for step in settings["velocity_fd_steps_s"]:
        measured = material_velocity_fd(oracle, patch, qpos, velocity, side, float(step))
        error = maxabs(measured - predicted)
        velocity_rows.append({"step_s": step, "absolute_error_m_s": error,
                              "relative_error": error / max(1., maxabs(measured))})
    bias_rows = []
    for outer in settings["bias_outer_steps_s"]:
        plus, minus = (oracle.integrate_flow(qpos, velocity, sign * float(outer)) for sign in (1., -1.))
        analytic = (A_matrix(oracle, patch, plus, side) - A_matrix(oracle, patch, minus, side)) @ velocity / (2 * float(outer))
        # Independent outer target: freeze each outer state's material pc coordinate during an inner FD.
        target = (material_velocity_fd(oracle, patch, plus, velocity, side, float(settings["bias_inner_step_s"])) -
                  material_velocity_fd(oracle, patch, minus, velocity, side, float(settings["bias_inner_step_s"]))) / (2 * float(outer))
        error = maxabs(target - analytic)
        bias_rows.append({"outer_step_s": outer, "analytic_adot_nu": analytic.tolist(), "independent_bias": target.tolist(),
                          "absolute_error_m_s2": error, "relative_error": error / max(1., maxabs(target))})
    return {"velocity_fd": velocity_rows, "bias": bias_rows, "A": A.tolist(), "A_shape": list(A.shape)}


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", type=Path, required=True); parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_path = args.config.resolve(); config, inputs = load_config(config_path); settings = config["continuous_pfaffian_oracle"]
    contact_config, contact_inputs = load_config((ROOT / settings["continuous_profile"]).resolve())
    model_config, model_inputs = load_config((ROOT / contact_config["continuous_contact_oracle"]["model_profile"]).resolve())
    equilibrium_path = ROOT / model_config["equilibrium"]; oracle = Oracle(model_config, json.loads(equilibrium_path.read_text()))
    patch = ContinuousPatch(oracle, contact_config["continuous_contact_oracle"]); capture_dir = ROOT / settings["capture_v2"]
    capture = np.load(capture_dir / "capture.npz"); switches = json.loads((ROOT / settings["old_switches"]).read_text())
    ticks = set(int(x) for x in settings["representative_rolling_ticks"])
    ticks.update(max(1, min(271, int(event["tick"]) + offset)) for event in switches for offset in (-1, 0, 1))
    velocity = np.array([.07, -.05, .03, .11, -.09, .08, .13, -.10, .17, -.12, .09, -.16])
    rows = []
    captured_velocities = []
    for tick in sorted(ticks):
        rolling_velocity = reduced_velocity(oracle, capture["qpos"][tick], capture["qvel"][tick])
        captured_velocities.append(rolling_velocity)
        for side in range(2): rows.append({"source": "rolling", "state_id": f"tick_{tick}", "side": side,
                                            "reduced_velocity": rolling_velocity.tolist(),
                                            **evaluate(oracle, patch, capture["qpos"][tick], rolling_velocity, side, settings)})
    envelope_rotation = np.max(np.abs([x["base_rotation_vector_rad"] for x in model_config["samples"]]), axis=0)
    envelope_delta = np.max(np.abs([x["canonical_joint_delta_rad"] for x in model_config["samples"]]), axis=0)
    rng = np.random.default_rng(int(settings["random_seed"]))
    samples = [*model_config["samples"], *[{"id": f"random_{i:02d}", "base_rotation_vector_rad": rng.uniform(-envelope_rotation, envelope_rotation).tolist(), "canonical_joint_delta_rad": rng.uniform(-envelope_delta, envelope_delta).tolist()} for i in range(int(settings["random_count"]))]]
    workspace = []
    for sample in samples:
        qpos = oracle.sample_qpos(sample)
        for side in range(2): workspace.append({"source": "workspace", "state_id": sample["id"], "side": side,
                                                  **evaluate(oracle, patch, qpos, velocity, side, settings)})
    all_rows = rows + workspace
    velocity_error = max(item["absolute_error_m_s"] for row in all_rows for item in row["velocity_fd"])
    authority_bias = max(item["absolute_error_m_s2"] for row in all_rows for item in row["bias"] if item["outer_step_s"] == 0.0001)
    near = [row for row in rows if int(row["state_id"].split("_")[1]) in {max(1, min(271, int(e["tick"]) + d)) for e in switches for d in (-1, 0, 1)}]
    switch_actual = max(maxabs(np.asarray(item["analytic_adot_nu"])) for row in near for item in row["bias"] if item["outer_step_s"] == 0.0001)
    other_actual = max(maxabs(np.asarray(item["analytic_adot_nu"])) for row in rows if row not in near for item in row["bias"] if item["outer_step_s"] == 0.0001)
    qeq = oracle.sample_qpos(model_config["samples"][0]); sign_rows = []
    for side, column in enumerate((8, 11)):
        unit = np.zeros(12); unit[column] = 1.; value = A_matrix(oracle, patch, qeq, side) @ unit
        sign_rows.append({"side": side, "wheel_column": column, "contact_frame_velocity_m_s": value.tolist()})
    mirror_error = abs(sign_rows[0]["contact_frame_velocity_m_s"][0] - sign_rows[1]["contact_frame_velocity_m_s"][0])
    gates = {"finite_fixed_dimension_order": all(row["A_shape"] == [3, 12] and np.all(np.isfinite(row["A"])) for row in all_rows),
             "velocity_fd": velocity_error <= float(settings["maximum_velocity_fd_m_s"]),
             "adot_cross_oracle": authority_bias <= float(settings["maximum_bias_cross_error_m_s2"]),
             "no_artificial_switch_spike": max(item["absolute_error_m_s2"] for row in near for item in row["bias"] if item["outer_step_s"] == 0.0001) <= float(settings["maximum_bias_cross_error_m_s2"]),
             "wheel_rolling_sign_radius": all(row["contact_frame_velocity_m_s"][0] >= float(settings["wheel_rolling_sign_minimum_m_s"]) and abs(row["contact_frame_velocity_m_s"][0] - float(contact_config["continuous_contact_oracle"]["radius_m"])) <= float(settings["maximum_wheel_rolling_radius_error_m"]) for row in sign_rows)}
    captured_velocities = np.asarray(captured_velocities)
    summary = {"schema_version": 1, "phase": 21, "profile": config["profile"], "candidate": {"rows_per_wheel": 3, "total_rows": 6, "order": ["rolling", "lateral", "normal"], "definition": "A_side=R_c^T J_material(pc(q), body) N(q)"}, "semantics": "instantaneous wheel material point located at analytic continuous contact center; validated only for compliant-contact SOFT acceleration-task kinematics", "coverage": {"rolling_ticks": sorted(ticks), "old_switch_event_count": len(switches), "rolling_velocity_source": "synchronized capture qvel projected by lstsq(N(q), qvel)", "maximum_captured_reduced_velocity": float(np.max(np.abs(captured_velocities))), "maximum_captured_wheel_velocity": float(np.max(np.abs(captured_velocities[:, [8, 11]]))), "workspace_samples": [x["id"] for x in samples], "workspace_test_velocity": velocity.tolist()}, "maximum_velocity_fd_error_m_s": velocity_error, "maximum_adot_cross_error_m_s2": authority_bias, "actual_bias_magnitude": {"near_old_switches": switch_actual, "elsewhere": other_actual}, "wheel_rolling_sign": {"rows": sign_rows, "left_right_rolling_difference": mirror_error, "nonrolling_components_report_only": "lateral/normal components are reported without a zero gate because fixed geometric offsets produce physical cross-components"}, "gates": gates, "pass": all(gates.values()), "limits": "This validation does not authorize a hard rigid 3D constraint or any QP. Hard/soft physical enforcement remains SOFT."}
    write_json(output / "summary.json", summary); write_json(output / "detailed.json", all_rows)
    script = Path(__file__).resolve(); outputs = ["summary.json", "detailed.json"]
    write_json(output / "manifest.json", {"schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(), "numpy": np.__version__, "mujoco": mujoco.__version__, "config_inputs": {str(p.relative_to(ROOT)): sha256(p) for p in inputs}, "continuous_inputs": {str(p.relative_to(ROOT)): sha256(p) for p in contact_inputs}, "model_inputs": {str(p.relative_to(ROOT)): sha256(p) for p in model_inputs}, "capture": sha256(capture_dir / "capture.npz"), "switches": sha256(ROOT / settings["old_switches"]), "validator": str(script.relative_to(ROOT)), "validator_sha256": sha256(script), "outputs": {name: sha256(output / name) for name in outputs}})
    print(json.dumps(summary, indent=2, sort_keys=True)); return 0 if summary["pass"] else 1

if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error: print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
