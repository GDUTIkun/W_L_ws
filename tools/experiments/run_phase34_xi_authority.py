#!/usr/bin/env python3
"""Phase 34 gain-free longitudinal wheel authority screen at Phase 29 states."""

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
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase34_feasibility_v1.json"
PHASE33_RUNNER = ROOT / "tools/experiments/run_phase33_zeta_authority.py"
PHASE32_METHOD = ROOT / "simulation/mujoco/config/phase32_markov_closure_v2.json"
SWEEP = ROOT / "ros_ws/build/wheel_leg_core/phase34_wbc_xi_sweep"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P33 = load_module(PHASE33_RUNNER, "phase34_phase33_authority")
M = P33.M


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    return value


def xi_value(geometry: Any) -> tuple[np.ndarray, np.ndarray]:
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
        position[side] = relative[0]
        velocity[side] = relative_velocity[0]
    return position, velocity


def physical_ddxi(
    geometry: Any, base_weld: int, qpos: np.ndarray, qvel: np.ndarray,
    torque: np.ndarray, time_s: float, epsilon: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    geometry.data.eq_active[base_weld] = 0
    geometry.set_state(qpos, qvel, time_s)
    geometry.data.ctrl[:] = -torque
    geometry.data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(geometry.model, geometry.data)
    qacc = geometry.data.qacc.copy()
    xi, dxi = xi_value(geometry)
    values = []
    for sign in (-1.0, 1.0):
        perturbed = qpos.copy()
        mujoco.mj_integratePos(geometry.model, perturbed, qvel, sign * epsilon)
        geometry.data.eq_active[base_weld] = 0
        geometry.set_state(
            perturbed, qvel + sign * epsilon * qacc, time_s + sign * epsilon)
        values.append(xi_value(geometry)[1])
    return xi, dxi, (values[1] - values[0]) / (2.0 * epsilon)


def run_sweep(control: Path, delta: float, tick: int) -> list[dict[str, str]]:
    completed = subprocess.run(
        [str(SWEEP), str(control), repr(delta), str(tick)],
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
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")

    method_path = args.method.resolve()
    method = json.loads(method_path.read_text(encoding="utf-8"))
    phase29_path = ROOT / method["phase29_method"]
    phase29 = json.loads(phase29_path.read_text(encoding="utf-8"))
    phase32 = json.loads(PHASE32_METHOD.read_text(encoding="utf-8"))
    scene = ROOT / phase32["scene"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = M.CONTRACT.Geometry(model, phase32["body_site_contract"])
    base_weld = M.CONTRACT.required_id(
        model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    raw_root = ROOT / method["source_phase28_run"]
    delta = float(method["wbc_authority"]["direct_acceleration_delta_m_s2"])

    samples: list[dict[str, Any]] = []
    self_gains: list[float] = []
    cross_ratios: list[float] = []
    condition_numbers: list[float] = []
    wrench_changes: list[float] = []
    hard_violations: list[float] = []
    statuses: list[int] = []
    model_errors: list[float] = []
    source_paths = [method_path, phase29_path, PHASE32_METHOD, PHASE33_RUNNER,
                    Path(__file__).resolve(), SWEEP, scene]

    for case in phase29["cases"]:
        case_id = case["id"]
        tick = int(case["expected_action_tick"])
        control_path = raw_root / f"{case_id}_control.csv"
        plant_path = raw_root / f"{case_id}_plant.csv"
        source_paths.extend([control_path, plant_path])
        controls = M.read_csv(control_path)
        plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
                 for row in M.read_csv(plant_path)}
        plant_row = plant[(tick - 1, 4)]
        qpos = M.vector(plant_row, "qpos", model.nq)
        qvel = M.vector(plant_row, "qvel", model.nv)
        time_s = float(plant_row["time_s"])

        rows = run_sweep(control_path, delta, tick)
        indexed = {(row["channel"], float(row["step_scale"]),
                    int(float(row["sign"]))): row for row in rows}
        baseline = indexed[("baseline", 0.0, 0)]
        baseline_realized = vector(baseline, "realized", 12)
        case_result: dict[str, Any] = {
            "case": case_id,
            "tick": tick,
            "baseline_xi_m": vector(baseline, "xi", 2),
            "baseline_dxi_m_s": vector(baseline, "dxi", 2),
            "scales": {},
        }
        matrices: dict[float, np.ndarray] = {}
        for scale in (1.0, 0.5):
            response_matrix = np.zeros((2, 2))
            scale_result: dict[str, Any] = {}
            for column, channel in enumerate(("common", "differential")):
                physical_values = []
                predicted_values = []
                local_changes = []
                for sign in (-1, 1):
                    row = indexed[(channel, scale, sign)]
                    torque = vector(row, "tau", 6)
                    xi, dxi, ddxi = physical_ddxi(
                        geometry, base_weld, qpos, qvel, torque, time_s)
                    predicted = vector(row, "ddxi", 2)
                    physical_values.append(ddxi)
                    predicted_values.append(predicted)
                    model_errors.append(float(np.max(np.abs(ddxi - predicted))))
                    realized = vector(row, "realized", 12)
                    change = float(np.max(np.abs(realized - baseline_realized)) /
                                   max(np.max(np.abs(baseline_realized)), 1e-12))
                    local_changes.append(change)
                    wrench_changes.append(change)
                    hard_violations.append(float(row["hard_violation"]))
                    statuses.append(int(row["status"]))
                side_response = (physical_values[1] - physical_values[0]) / (
                    2.0 * scale * delta)
                predicted_response = (predicted_values[1] - predicted_values[0]) / (
                    2.0 * scale * delta)
                channel_response = np.array([
                    0.5 * (side_response[0] + side_response[1]),
                    0.5 * (side_response[1] - side_response[0]),
                ])
                response_matrix[:, column] = channel_response
                scale_result[channel] = {
                    "physical_side_response": side_response,
                    "physical_common_differential_response": channel_response,
                    "wbc_model_side_response": predicted_response,
                    "maximum_realized_wrench_relative_change": max(local_changes),
                    "physical_xi_m": xi,
                    "physical_dxi_m_s": dxi,
                }
            matrices[scale] = response_matrix
            diagonal = np.diag(response_matrix)
            local_cross = max(
                abs(response_matrix[1, 0]) / max(abs(diagonal[0]), 1e-12),
                abs(response_matrix[0, 1]) / max(abs(diagonal[1]), 1e-12))
            condition = float(np.linalg.cond(response_matrix))
            self_gains.extend(diagonal.tolist())
            cross_ratios.append(float(local_cross))
            condition_numbers.append(condition)
            scale_result["response_matrix"] = response_matrix
            scale_result["condition_number"] = condition
            scale_result["maximum_cross_to_self_ratio"] = local_cross
            case_result["scales"][str(scale)] = scale_result
        case_result["full_half_matrix_relative_error"] = float(
            np.max(np.abs(matrices[1.0] - matrices[0.5])) /
            max(np.max(np.abs(matrices[0.5])), 1e-12))
        samples.append(case_result)

    gates = method["wbc_authority"]
    gate_results = {
        "positive_self_authority": min(self_gains) >= float(gates["minimum_self_gain"]),
        "cross_channel_isolation": max(cross_ratios) <= float(
            gates["maximum_cross_to_self_ratio"]),
        "conditioning": max(condition_numbers) <= float(
            gates["maximum_condition_number"]),
        "wrench_preservation": max(wrench_changes) <= float(
            gates["maximum_realized_wrench_relative_change"]),
        "hard_constraints": max(hard_violations) <= float(
            gates["maximum_hard_violation"]),
        "solver_status": all(status == 0 for status in statuses),
    }
    authority_pass = all(gate_results.values())
    summary = {
        "classification": (
            "gain_free_longitudinal_authority_pass" if authority_pass
            else "longitudinal_authority_gate_failure"),
        "authority_pass": authority_pass,
        "gate_results": gate_results,
        "blocking_finding": None if authority_pass else "P34-D",
        "sample_count": len(samples),
        "minimum_physical_self_gain": min(self_gains),
        "maximum_cross_to_self_ratio": max(cross_ratios),
        "maximum_condition_number": max(condition_numbers),
        "maximum_realized_wrench_relative_change": max(wrench_changes),
        "maximum_hard_violation": max(hard_violations),
        "maximum_wbc_to_physical_ddxi_error_m_s2": max(model_errors),
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
