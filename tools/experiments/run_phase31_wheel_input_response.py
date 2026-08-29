#!/usr/bin/env python3
"""Phase 31 Gate 6: same-state common/differential wheel-wrench sensitivities."""

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

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase31_wheel_state_contract_v1.json"
CONTRACT_SCRIPT = ROOT / "tools/experiments/run_phase31_wheel_state_contract.py"
SWEEP_SOURCE = ROOT / "tools/experiments/phase31_wbc_wrench_sweep.cpp"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CONTRACT = load_module(CONTRACT_SCRIPT, "phase31_input_contract")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean(item) for item in value]
    return value


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def acceleration_from_current_qacc(geometry: Any, epsilon: float = 1e-6) -> np.ndarray:
    qpos = geometry.data.qpos.copy()
    qvel = geometry.data.qvel.copy()
    qacc = geometry.data.qacc.copy()
    time_s = float(geometry.data.time)
    values = []
    for sign in (-1.0, 1.0):
        perturbed = qpos.copy()
        mujoco.mj_integratePos(geometry.model, perturbed, qvel, sign * epsilon)
        geometry.set_state(perturbed, qvel + sign * epsilon * qacc, time_s + sign * epsilon)
        values.append(geometry.current_value()["velocity"])
    return (values[1] - values[0]) / (2.0 * epsilon)


def response(
    geometry: Any,
    row: dict[str, str],
    sweep: dict[tuple[int, str, float, int], dict[str, str]],
    tick: int,
    channel: str,
    delta: float,
    scale: float,
) -> dict[str, Any]:
    accelerations = []
    qacc_values = []
    realized = []
    for sign in (-1, 1):
        sweep_row = sweep[(tick, channel, scale, sign)]
        if int(sweep_row["status"]) != 0:
            raise RuntimeError(f"WBC solve failed at tick {tick}, {channel}")
        torque = np.array([float(sweep_row[f"tau{index}"]) for index in range(6)])
        geometry.set_row(row)
        geometry.data.ctrl[:] = -torque
        geometry.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(geometry.model, geometry.data)
        qacc_values.append(geometry.data.qacc.copy())
        accelerations.append(acceleration_from_current_qacc(geometry))
        component = 0 if channel.endswith("fx") else 4
        realized.append(np.array([
            float(sweep_row[f"realized{component}"]),
            float(sweep_row[f"realized{6 + component}"]),
        ]))
    step = scale * delta
    derivative = (accelerations[1] - accelerations[0]) / (2.0 * step)
    realized_derivative = (realized[1] - realized[0]) / (2.0 * step)
    geometry.set_row(row)
    nv = geometry.model.nv
    base_jacobian = np.zeros((3, nv))
    base_angular_jacobian = np.zeros((3, nv))
    mujoco.mj_jacSite(
        geometry.model, geometry.data, base_jacobian,
        base_angular_jacobian, geometry.base_site,
    )
    delta_qacc = (qacc_values[1] - qacc_values[0]) / (2.0 * step)
    rotation_n_from_b = geometry.data.site_xmat[geometry.base_site].reshape(3, 3)
    delta_alpha_b = rotation_n_from_b.T @ (base_angular_jacobian @ delta_qacc)
    components = {"wheel_translation": [], "base_translation": [], "base_angular": []}
    base_origin = geometry.data.site_xpos[geometry.base_site]
    for wheel in geometry.wheel_bodies:
        wheel_jacobian = np.zeros((3, nv))
        mujoco.mj_jacBody(geometry.model, geometry.data, wheel_jacobian, None, wheel)
        wheel_term = rotation_n_from_b.T @ (wheel_jacobian @ delta_qacc)
        base_term = -rotation_n_from_b.T @ (base_jacobian @ delta_qacc)
        relative_b = rotation_n_from_b.T @ (geometry.data.xpos[wheel] - base_origin)
        angular_term = -np.cross(delta_alpha_b, relative_b)
        components["wheel_translation"].append(wheel_term[0])
        components["base_translation"].append(base_term[0])
        components["base_angular"].append(angular_term[0])
    component_sum = sum(np.asarray(values) for values in components.values())
    return {
        "individual": derivative,
        "grouped": CONTRACT.grouped(derivative),
        "realized_wrench_grouped": CONTRACT.grouped(realized_derivative),
        "acceleration_sensitivity_components": {
            name: {"individual": values, "grouped": CONTRACT.grouped(np.asarray(values))}
            for name, values in components.items()
        },
        "component_sum_max_abs_error": float(np.max(np.abs(component_sum - derivative))),
    }


def run_sweep(executable: Path, control_path: Path, force_delta: float,
              ticks: list[int]) -> dict[tuple[int, str, float, int], dict[str, str]]:
    command = [str(executable), str(control_path), str(force_delta), *map(str, ticks)]
    completed = subprocess.run(command, check=True, text=True, capture_output=True)
    rows = list(csv.DictReader(io.StringIO(completed.stdout)))
    return {
        (int(row["tick"]), row["channel"], float(row["step_scale"]), int(float(row["sign"]))): row
        for row in rows
    }


def expected(config: dict) -> dict[str, dict[str, float]]:
    mass = float(config["body_mass_kg"])
    radius = float(config["wheel_radius_m"])
    denominator = float(config["wheel_mass_kg"]) * radius + float(config["wheel_axle_inertia_kg_m2"]) / radius
    return {
        "common_fx": {"common": -2.0 / mass - radius / denominator, "differential": 0.0},
        "differential_fx": {"common": 0.0, "differential": -radius / denominator},
        "common_ty": {"common": -1.0 / denominator, "differential": 0.0},
        "differential_ty": {"common": 0.0, "differential": -1.0 / denominator},
    }


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
    ocp_path = ROOT / method["source_ocp_config"]
    ocp = json.loads(ocp_path.read_text(encoding="utf-8"))
    scene = ROOT / method["scene"]
    geometry = CONTRACT.Geometry(mujoco.MjModel.from_xml_path(str(scene)), method["body_site_contract"])
    sweep_executable = ROOT / method["input_response"]["wbc_sweep_executable"]
    if not sweep_executable.is_file():
        raise RuntimeError(f"missing WBC sweep executable: {sweep_executable}")
    raw_root = ROOT / method["source_phase28_run"]
    spec = method["input_response"]
    channels = {
        "common_fx": float(spec["force_delta_n"]),
        "differential_fx": float(spec["force_delta_n"]),
        "common_ty": float(spec["moment_delta_nm"]),
        "differential_ty": float(spec["moment_delta_nm"]),
    }
    eq = expected(ocp)
    results = {}
    raw_paths = []
    primary_errors = []
    cross_ratios = []
    sign_passes = []
    step_errors = []
    baseline_torque_errors = []
    realization_primary_errors = []
    realization_cross_ratios = []
    component_errors = []
    for case in method["cases"]:
        control_path = raw_root / f"{case['id']}_control.csv"
        plant_path = raw_root / f"{case['id']}_plant.csv"
        raw_paths.extend([control_path, plant_path])
        control = {int(row["tick"]): row for row in read_csv(control_path)}
        plant = {(int(row["control_tick"]), int(row["physics_substep"])): row for row in read_csv(plant_path)}
        sweep = run_sweep(sweep_executable, control_path, float(spec["force_delta_n"]), case["authority_ticks"])
        case_results = []
        for tick in case["authority_ticks"]:
            state_row = plant[(tick - 1, 4)]
            baseline = sweep[(tick, "baseline", 0.0, 0)]
            baseline_error = max(
                abs(float(baseline[f"tau{index}"]) - float(control[tick][f"raw_tau{index}"]))
                for index in range(6)
            )
            baseline_torque_errors.append(baseline_error)
            channel_results = {}
            for channel, delta in channels.items():
                measured = response(geometry, state_row, sweep, tick, channel, delta, 1.0)
                half_step = response(geometry, state_row, sweep, tick, channel, delta, 0.5)
                primary_name = "common" if channel.startswith("common") else "differential"
                cross_name = "differential" if primary_name == "common" else "common"
                primary = float(measured["grouped"][primary_name])
                target = float(eq[channel][primary_name])
                relative_error = abs(primary - target) / abs(target)
                cross_ratio = abs(float(measured["grouped"][cross_name])) / max(abs(primary), 1e-12)
                sign_pass = bool(abs(primary) <= float(spec["sensitivity_sign_deadband"]) or np.sign(primary) == np.sign(target))
                half_primary = float(half_step["grouped"][primary_name])
                step_error = abs(primary - half_primary) / max(abs(half_primary), 1e-12)
                realized_primary = float(measured["realized_wrench_grouped"][primary_name])
                realized_cross = float(measured["realized_wrench_grouped"][cross_name])
                realization_primary_error = abs(realized_primary - 1.0)
                realization_cross_ratio = abs(realized_cross) / max(abs(realized_primary), 1e-12)
                primary_errors.append(relative_error)
                cross_ratios.append(cross_ratio)
                sign_passes.append(sign_pass)
                step_errors.append(step_error)
                realization_primary_errors.append(realization_primary_error)
                realization_cross_ratios.append(realization_cross_ratio)
                component_errors.extend([
                    float(measured["component_sum_max_abs_error"]),
                    float(half_step["component_sum_max_abs_error"]),
                ])
                channel_results[channel] = {
                    "measured": measured,
                    "half_step": half_step,
                    "eq12": eq[channel],
                    "primary_relative_error": relative_error,
                    "cross_to_primary_ratio": cross_ratio,
                    "sign_pass": sign_pass,
                    "finite_difference_step_relative_error": step_error,
                    "realization_primary_gain_error": realization_primary_error,
                    "realization_cross_to_primary_ratio": realization_cross_ratio,
                }
            case_results.append({"tick": tick, "baseline_torque_max_abs_error_nm": baseline_error, "channels": channel_results})
        results[case["id"]] = case_results
    sign_pass = all(sign_passes)
    gain_pass = max(primary_errors) <= float(spec["primary_relative_gain_error_max"])
    cross_pass = max(cross_ratios) <= float(spec["cross_to_primary_gain_ratio_max"])
    step_pass = max(step_errors) <= float(spec["finite_difference_step_relative_error_max"])
    baseline_pass = max(baseline_torque_errors) <= float(spec["wbc_baseline_torque_max_abs_error_nm"])
    realization_pass = (
        max(realization_primary_errors) <= float(spec["wbc_realization_primary_gain_error_max"])
        and max(realization_cross_ratios) <= float(spec["wbc_realization_cross_gain_ratio_max"])
    )
    decomposition_pass = max(component_errors) <= float(spec["acceleration_decomposition_max_abs_error"])
    summary = {
        "sign_pass": sign_pass, "gain_pass": gain_pass, "cross_coupling_pass": cross_pass,
        "finite_difference_step_pass": step_pass,
        "wbc_baseline_reproduction_pass": baseline_pass,
        "wbc_realization_sensitivity_pass": realization_pass,
        "acceleration_decomposition_pass": decomposition_pass,
        "max_primary_relative_gain_error": max(primary_errors),
        "max_cross_to_primary_gain_ratio": max(cross_ratios),
        "max_finite_difference_step_relative_error": max(step_errors),
        "max_wbc_baseline_torque_error_nm": max(baseline_torque_errors),
        "max_wbc_realization_primary_gain_error": max(realization_primary_errors),
        "max_wbc_realization_cross_gain_ratio": max(realization_cross_ratios),
        "max_acceleration_decomposition_error": max(component_errors),
        "classification": (
            "invalid_input_response_oracle" if not step_pass or not baseline_pass or not decomposition_pass
            else "P31-C_wbc_realization_mapping" if not realization_pass
            else "P31-D_eq12_sign_or_frame_error" if not sign_pass
            else "eq12_gain_or_coupling_mismatch" if not gain_pass or not cross_pass
            else "eq12_input_response_pass"
        ),
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "wheel_input_response.json").write_text(json.dumps(clean({"eq12": eq, "results": results}), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [method_path, ocp_path, scene, CONTRACT_SCRIPT, SWEEP_SOURCE,
              Path(__file__).resolve(), sweep_executable, *raw_paths]
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv), "replay_of": args.replay_of, "python": platform.python_version(), "dependencies": {"numpy": np.__version__, "mujoco": mujoco.__version__}, "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if sign_pass and gain_pass and cross_pass and step_pass and baseline_pass and realization_pass and decomposition_pass else 2


if __name__ == "__main__": raise SystemExit(main())
