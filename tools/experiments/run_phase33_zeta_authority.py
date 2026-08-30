#!/usr/bin/env python3
"""Phase 33 gain-free wheel-height acceleration authority screen."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import platform
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase33_zeta_manifold_v1.json"
LEG_SCRIPT = ROOT / "tools/experiments/run_phase32_leg_nullspace.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None: raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


LEG = load_module(LEG_SCRIPT, "phase33_leg_authority")
M = LEG.M


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean(item) for item in value]
    return value


def zeta_value(geometry: Any) -> tuple[np.ndarray, np.ndarray]:
    rotation = geometry.data.site_xmat[geometry.base_site].reshape(3, 3)
    base_position = geometry.data.site_xpos[geometry.base_site].copy()
    jacp_base = np.zeros((3, geometry.model.nv))
    jacr_base = np.zeros((3, geometry.model.nv))
    mujoco.mj_jacSite(
        geometry.model, geometry.data, jacp_base, jacr_base, geometry.base_site)
    base_velocity = jacp_base @ geometry.data.qvel
    omega_b = rotation.T @ (jacr_base @ geometry.data.qvel)
    position = np.zeros(2)
    velocity = np.zeros(2)
    for side, body in enumerate(geometry.wheel_bodies):
        jacp_wheel = np.zeros((3, geometry.model.nv))
        mujoco.mj_jacBody(geometry.model, geometry.data, jacp_wheel, None, body)
        relative = rotation.T @ (geometry.data.xpos[body] - base_position)
        relative_velocity = rotation.T @ (
            jacp_wheel @ geometry.data.qvel - base_velocity) - np.cross(omega_b, relative)
        position[side] = relative[2]
        velocity[side] = relative_velocity[2]
    return position, velocity


def physical_ddzeta(
    geometry: Any, base_weld: int, qpos: np.ndarray, qvel: np.ndarray,
    torque: np.ndarray, time_s: float, epsilon: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    geometry.data.eq_active[base_weld] = 0
    geometry.set_state(qpos, qvel, time_s)
    geometry.data.ctrl[:] = -torque
    geometry.data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(geometry.model, geometry.data)
    qacc = geometry.data.qacc.copy()
    zeta, dzeta = zeta_value(geometry)
    values = []
    for sign in (-1.0, 1.0):
        perturbed = qpos.copy()
        mujoco.mj_integratePos(geometry.model, perturbed, qvel, sign * epsilon)
        geometry.data.eq_active[base_weld] = 0
        geometry.set_state(
            perturbed, qvel + sign * epsilon * qacc, time_s + sign * epsilon)
        values.append(zeta_value(geometry)[1])
    return zeta, dzeta, (values[1] - values[0]) / (2.0 * epsilon)


def run_sweep(executable: Path, control: Path, delta: float, tick: int) -> list[dict[str, str]]:
    completed = subprocess.run(
        [str(executable), str(control), repr(delta), str(tick)],
        check=True, text=True, capture_output=True)
    return list(csv.DictReader(io.StringIO(completed.stdout)))


def vector(row: dict[str, str], prefix: str, count: int) -> np.ndarray:
    return np.array([float(row[f"{prefix}{index}"]) for index in range(count)])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists(): raise RuntimeError(f"output already exists: {output}")
    method_path = args.method.resolve()
    method = json.loads(method_path.read_text(encoding="utf-8"))
    phase32_path = ROOT / method["phase32_method"]
    phase32 = json.loads(phase32_path.read_text(encoding="utf-8"))
    authority_path = ROOT / method["phase32_leg_authority"]
    authority = json.loads(authority_path.read_text(encoding="utf-8"))
    executable = ROOT / method["zeta_sweep_executable"]
    scene = ROOT / phase32["scene"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = M.CONTRACT.Geometry(model, phase32["body_site_contract"])
    base_weld = M.CONTRACT.required_id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    sides = [
        {"name": "left", "index": 0, "qpos": [12, 13, 15, 16],
         "dofs": [11, 12, 14, 15], "equality_id": M.CONTRACT.required_id(
             model, mujoco.mjtObj.mjOBJ_EQUALITY, "left_leg_closure")},
        {"name": "right", "index": 1, "qpos": [7, 8, 10, 11],
         "dofs": [6, 7, 9, 10], "equality_id": M.CONTRACT.required_id(
             model, mujoco.mjtObj.mjOBJ_EQUALITY, "right_leg_closure")},
    ]
    raw_root = ROOT / method["source_phase28_run"]
    delta = float(method["task"]["authority_delta_m_s2"])
    samples = []
    self_gains = []
    cross_ratios = []
    wrench_changes = []
    consistency_errors = []
    hard_violations = []
    statuses = []
    source_paths = [method_path, phase32_path, authority_path, executable, scene,
                    Path(__file__).resolve(), LEG_SCRIPT]
    with tempfile.TemporaryDirectory(prefix="phase33-zeta-authority-") as temp_name:
        temp = Path(temp_name)
        for authority_sample in authority["samples"]:
            case_id = authority_sample["case"]
            tick = int(authority_sample["tick"])
            control_path = raw_root / f"{case_id}_control.csv"
            plant_path = raw_root / f"{case_id}_plant.csv"
            source_paths.extend([control_path, plant_path])
            controls = M.read_csv(control_path)
            fields = list(controls[0].keys())
            plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
                     for row in M.read_csv(plant_path)}
            plant_row = plant[(tick - 1, 4)]
            qpos = M.vector(plant_row, "qpos", model.nq)
            qvel = M.vector(plant_row, "qvel", model.nv)
            time_s = float(plant_row["time_s"])
            for family in ("C1", "C2"):
                for state_sign in (-1, 1):
                    scale = 1.0
                    if family == "C1":
                        changed_qpos, changed_qvel, _ = LEG.c1_variant(
                            geometry, base_weld, qpos, qvel, time_s, sides,
                            state_sign * scale * float(
                                phase32["leg_pairs"]["configuration_hip_delta_rad"]))
                    else:
                        changed_qpos, changed_qvel, _ = LEG.c2_variant(
                            geometry, base_weld, qpos, qvel, time_s, sides,
                            state_sign * scale * float(
                                phase32["leg_pairs"]["velocity_active_max_delta_rad_s"]))
                    patches = LEG.control_patches(
                        qpos, qvel, changed_qpos, changed_qvel)
                    changed_controls = [dict(row) for row in controls]
                    target = next(row for row in changed_controls
                                  if int(row["tick"]) == tick)
                    for field, value in patches.items():
                        target[field] = repr(float(target[field]) + value)
                    patched_path = temp / f"{case_id}-{tick}-{family}-{state_sign}.csv"
                    M.write_csv(patched_path, changed_controls, fields)
                    rows = run_sweep(executable, patched_path, delta, tick)
                    indexed = {(row["channel"], float(row["step_scale"]),
                                int(float(row["sign"]))): row for row in rows}
                    baseline = indexed[("baseline", 0.0, 0)]
                    baseline_realized = vector(baseline, "realized", 12)
                    state_result: dict[str, Any] = {
                        "case": case_id, "tick": tick, "family": family,
                        "state_sign": state_sign, "control_patches": patches,
                        "baseline_zeta_m": vector(baseline, "zeta", 2),
                        "baseline_dzeta_m_s": vector(baseline, "dzeta", 2),
                        "channels": {},
                    }
                    for channel, side in (("left", 0), ("right", 1)):
                        scale_results = {}
                        responses = {}
                        for request_scale in (1.0, 0.5):
                            negative = indexed[(channel, request_scale, -1)]
                            positive = indexed[(channel, request_scale, 1)]
                            signed_physical = []
                            signed_predicted = []
                            local_wrench_changes = []
                            for row in (negative, positive):
                                torque = vector(row, "tau", 6)
                                zeta, dzeta, ddzeta = physical_ddzeta(
                                    geometry, base_weld, changed_qpos,
                                    changed_qvel, torque, time_s)
                                signed_physical.append(ddzeta)
                                signed_predicted.append(vector(row, "ddzeta", 2))
                                realized = vector(row, "realized", 12)
                                change = float(np.max(np.abs(realized - baseline_realized)) /
                                               max(np.max(np.abs(baseline_realized)), 1e-12))
                                local_wrench_changes.append(change)
                                wrench_changes.append(change)
                                hard_violations.append(float(row["hard_violation"]))
                                statuses.append(int(row["status"]))
                            response = (signed_physical[1] - signed_physical[0]) / (
                                2.0 * request_scale * delta)
                            predicted_response = (
                                signed_predicted[1] - signed_predicted[0]) / (
                                2.0 * request_scale * delta)
                            responses[request_scale] = response
                            self_gain = float(response[side])
                            cross_ratio = float(abs(response[1 - side]) /
                                                max(abs(self_gain), 1e-12))
                            self_gains.append(self_gain)
                            cross_ratios.append(cross_ratio)
                            scale_results[str(request_scale)] = {
                                "physical_response": response,
                                "wbc_model_response": predicted_response,
                                "self_gain": self_gain,
                                "cross_ratio": cross_ratio,
                                "realized_wrench_relative_change_max": max(local_wrench_changes),
                                "physical_zeta_m": zeta,
                                "physical_dzeta_m_s": dzeta,
                            }
                        consistency = float(np.max(np.abs(
                            responses[1.0] - responses[0.5])) /
                            max(np.max(np.abs(responses[0.5])), 1e-12))
                        consistency_errors.append(consistency)
                        state_result["channels"][channel] = {
                            "scales": scale_results,
                            "full_half_response_relative_error": consistency,
                        }
                    samples.append(state_result)
    gates = method["gates"]
    gate_results = {
        "self_authority": min(self_gains) >= float(gates["self_authority_gain_min"]),
        "cross_side_isolation": max(cross_ratios) <= float(gates["cross_side_to_self_max"]),
        "wrench_preservation": max(wrench_changes) <= float(
            gates["realized_wrench_relative_change_max"]),
        "full_half_consistency": max(consistency_errors) <= float(
            gates["full_half_authority_relative_error_max"]),
        "hard_constraints": max(hard_violations) <= float(gates["hard_violation_max"]),
        "solver_status": all(status == int(gates["controller_ok_status"])
                             for status in statuses),
    }
    authority_pass = all(gate_results.values())
    summary = {
        "classification": "gain_free_zeta_authority_pass" if authority_pass else "unresolved",
        "authority_pass": authority_pass,
        "gate_results": gate_results,
        "blocking_finding": None if authority_pass else
            "gain_free_cross_side_isolation_gate_failure",
        "sample_count": len(samples),
        "minimum_physical_self_gain": min(self_gains),
        "maximum_cross_side_to_self_ratio": max(cross_ratios),
        "maximum_realized_wrench_relative_change": max(wrench_changes),
        "maximum_full_half_response_relative_error": max(consistency_errors),
        "maximum_hard_violation": max(hard_violations),
    }
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "python": platform.python_version(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path)
                   for path in sorted(set(source_paths))},
        "replay_of": args.replay_of,
    }
    output.mkdir(parents=True)
    (output / "details.json").write_text(
        json.dumps(clean({"samples": samples}), indent=2, sort_keys=True) + "\n")
    (output / "summary.json").write_text(
        json.dumps(clean(summary), indent=2, sort_keys=True) + "\n")
    (output / "manifest.json").write_text(
        json.dumps(clean(manifest), indent=2, sort_keys=True) + "\n")
    return 0 if authority_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
