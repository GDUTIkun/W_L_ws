#!/usr/bin/env python3
"""Phase 32: floating-base same-x16 wheel-spin Markov closure audit."""

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
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase32_markov_closure_v1.json"
CONTRACT_SCRIPT = ROOT / "tools/experiments/run_phase31_wheel_state_contract.py"
INPUT_SCRIPT = ROOT / "tools/experiments/run_phase31_wheel_input_response.py"
SWEEP_SOURCE = ROOT / "tools/experiments/phase31_wbc_wrench_sweep.cpp"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CONTRACT = load_module(CONTRACT_SCRIPT, "phase32_contract")
INPUT = load_module(INPUT_SCRIPT, "phase32_input")


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


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, str]], fieldnames: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_baselines(executable: Path, control_path: Path, ticks: list[int]) -> dict[int, dict[str, str]]:
    command = [str(executable), str(control_path), "0.1", *map(str, ticks)]
    completed = subprocess.run(command, check=True, text=True, capture_output=True)
    rows = csv.DictReader(io.StringIO(completed.stdout))
    return {int(row["tick"]): row for row in rows if row["channel"] == "baseline"}


def perturbed_wbc(
    executable: Path,
    original_rows: list[dict[str, str]],
    fieldnames: list[str],
    tick: int,
    wheel_delta: np.ndarray,
    directory: Path,
) -> dict[str, str]:
    rows = [dict(row) for row in original_rows]
    target = next(row for row in rows if int(row["tick"]) == tick)
    target["dq2"] = repr(float(target["dq2"]) + float(wheel_delta[0]))
    target["dq5"] = repr(float(target["dq5"]) + float(wheel_delta[1]))
    path = directory / f"control-{tick}-{wheel_delta[0]:+.6f}-{wheel_delta[1]:+.6f}.csv"
    write_csv(path, rows, fieldnames)
    return run_baselines(executable, path, [tick])[tick]


def vector(row: dict[str, str], prefix: str, count: int) -> np.ndarray:
    return np.array([float(row[f"{prefix}{index}"]) for index in range(count)])


def requested_wrench(row: dict[str, str]) -> np.ndarray:
    return np.array([float(row[f"reference{9 + index}"]) for index in range(12)])


def contact_diagnostics(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    wheel_geoms: list[int],
    floor_geom: int,
) -> dict[str, Any]:
    contacts = []
    normal_load = np.zeros(2)
    max_penetration = 0.0
    max_constraint_speed = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        side = next((s for s, geom in enumerate(wheel_geoms)
                     if {int(contact.geom1), int(contact.geom2)} == {geom, floor_geom}), None)
        if side is None:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        address = int(contact.efc_address)
        velocity = [] if address < 0 else data.efc_vel[address:address + int(contact.dim)].copy()
        normal_load[side] += max(0.0, float(force[0]))
        max_penetration = max(max_penetration, max(0.0, -float(contact.dist)))
        if len(velocity):
            max_constraint_speed = max(max_constraint_speed, float(np.max(np.abs(velocity))))
        contacts.append({
            "side": side,
            "geom1": int(contact.geom1),
            "geom2": int(contact.geom2),
            "dim": int(contact.dim),
            "dist_m": float(contact.dist),
            "force_contact": force,
            "constraint_velocity": velocity,
        })
    return {
        "count": len(contacts),
        "normal_load_n": normal_load,
        "max_penetration_m": max_penetration,
        "max_constraint_speed": max_constraint_speed,
        "contacts": contacts,
    }


def evaluate_plant(
    geometry: Any,
    base_weld: int,
    qpos: np.ndarray,
    qvel: np.ndarray,
    torque: np.ndarray,
    time_s: float,
    wheel_geoms: list[int],
    floor_geom: int,
) -> dict[str, Any]:
    geometry.data.eq_active[base_weld] = 0
    geometry.set_state(qpos, qvel, time_s)
    geometry.data.ctrl[:] = -torque
    geometry.data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(geometry.model, geometry.data)
    reduced = geometry.current_value()
    diagnostics = contact_diagnostics(
        geometry.model, geometry.data, wheel_geoms, floor_geom)
    qacc = geometry.data.qacc.copy()
    first = INPUT.acceleration_from_current_qacc(geometry, 1e-6)
    geometry.data.eq_active[base_weld] = 0
    geometry.set_state(qpos, qvel, time_s)
    geometry.data.ctrl[:] = -torque
    geometry.data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(geometry.model, geometry.data)
    second = INPUT.acceleration_from_current_qacc(geometry, 5e-7)
    return {
        "wheel_position_m": reduced["position"],
        "wheel_velocity_m_s": reduced["velocity"],
        "ddxi_m_s2": first,
        "ddxi_half_epsilon_m_s2": second,
        "oracle_max_abs_error_m_s2": float(np.max(np.abs(first - second))),
        "qacc": qacc,
        "finite": bool(np.all(np.isfinite(qacc)) and np.all(np.isfinite(first))),
        "contact": diagnostics,
    }


def authority_status(method: dict[str, Any]) -> dict[str, Any]:
    sources = {
        name: json.loads((ROOT / path).read_text(encoding="utf-8"))
        for name, path in method["phase31_authority"].items()
    }
    facts = {
        "measurement_contract_pass": sources["wheel_state_contract"]["classification"] == "measurement_contract_pass_proceed_to_dynamics",
        "acceleration_oracle_pass": bool(sources["wheel_acceleration"]["acceleration_oracle_pass"]),
        "eq12_residual_reproduced": bool(sources["wheel_acceleration"]["significant_eq12_residual"]),
        "wbc_realization_pass": bool(sources["wheel_input_response"]["wbc_realization_sensitivity_pass"]),
        "eq12_gain_failure_reproduced": not bool(sources["wheel_input_response"]["gain_pass"]),
        "scalar_inertia_rejected": bool(sources["effective_inertia"]["scalar_effective_inertia_rejected"]),
    }
    return {
        "facts": facts,
        "semantic_reproduction_pass": all(facts.values()),
        "controlled_dynamics_authority_valid": False,
        "controlled_dynamics_limitation": "Phase31 input-response replay left XML base_weld active; Phase32 uses floating-base eq_active=0",
    }


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
    phase31_method_path = ROOT / method["source_phase31_method"]
    phase31 = json.loads(phase31_method_path.read_text(encoding="utf-8"))
    scene = ROOT / method["scene"]
    executable = ROOT / method["wbc_sweep_executable"]
    if not executable.is_file():
        raise RuntimeError(f"missing WBC sweep executable: {executable}")
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = CONTRACT.Geometry(model, method["body_site_contract"])
    base_weld = CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    wheel_geoms = [CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in method["body_site_contract"]["wheel_geoms"]]
    floor_geom = CONTRACT.required_id(
        model, mujoco.mjtObj.mjOBJ_GEOM, method["body_site_contract"]["floor_geom"])
    wheel_dofs = [
        int(model.jnt_dofadr[CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint")]),
        int(model.jnt_dofadr[CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint")]),
    ]
    raw_root = ROOT / method["source_phase28_run"]
    raw_paths: list[Path] = []
    all_pairs = []
    oracle_errors = []
    direct_errors = []
    wheel_errors = []
    requested_errors = []
    closure_differences = []
    consistency_errors = []
    realized_errors = []
    finite = []
    bilateral = []
    with tempfile.TemporaryDirectory(prefix="phase32-") as temp_name:
        temp = Path(temp_name)
        for case in method["cases"]:
            control_path = raw_root / f"{case['id']}_control.csv"
            plant_path = raw_root / f"{case['id']}_plant.csv"
            raw_paths.extend([control_path, plant_path])
            control_rows = read_csv(control_path)
            control_by_tick = {int(row["tick"]): row for row in control_rows}
            plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
                     for row in read_csv(plant_path)}
            fieldnames = list(control_rows[0].keys())
            baseline_wbc = run_baselines(executable, control_path, case["authority_ticks"])
            for tick in case["authority_ticks"]:
                state_row = plant[(tick - 1, 4)]
                control_row = control_by_tick[tick]
                qpos = vector(state_row, "qpos", model.nq)
                qvel = vector(state_row, "qvel", model.nv)
                baseline_sweep = baseline_wbc[tick]
                baseline_torque = vector(baseline_sweep, "tau", 6)
                baseline_realized = vector(baseline_sweep, "realized", 12)
                requested = requested_wrench(control_row)
                baseline_plant = evaluate_plant(
                    geometry, base_weld, qpos, qvel, baseline_torque,
                    float(state_row["time_s"]), wheel_geoms, floor_geom)
                mode_results: dict[str, Any] = {}
                for mode, direction_raw in method["wheel_rate_pairs"]["modes"].items():
                    direction = np.asarray(direction_raw, dtype=float)
                    scale_results: dict[str, Any] = {}
                    sensitivities: dict[float, np.ndarray] = {}
                    for scale_raw in method["wheel_rate_pairs"]["step_scales"]:
                        scale = float(scale_raw)
                        delta = float(method["wheel_rate_pairs"]["delta_rad_s"]) * scale
                        signed = {}
                        for sign in (-1, 1):
                            canonical_delta = sign * delta * direction
                            changed_qvel = qvel.copy()
                            for side, dof in enumerate(wheel_dofs):
                                changed_qvel[dof] -= canonical_delta[side]
                            sweep = perturbed_wbc(
                                executable, control_rows, fieldnames, tick,
                                canonical_delta, temp)
                            torque = vector(sweep, "tau", 6)
                            realized = vector(sweep, "realized", 12)
                            plant_composed = evaluate_plant(
                                geometry, base_weld, qpos, changed_qvel, torque,
                                float(state_row["time_s"]), wheel_geoms, floor_geom)
                            plant_fixed_torque = evaluate_plant(
                                geometry, base_weld, qpos, changed_qvel, baseline_torque,
                                float(state_row["time_s"]), wheel_geoms, floor_geom)
                            direct_error = max(
                                float(np.max(np.abs(qpos - qpos))),
                                float(np.max(np.abs(qvel[:6] - changed_qvel[:6]))),
                            )
                            wheel_error = max(
                                float(np.max(np.abs(plant_composed["wheel_position_m"] - baseline_plant["wheel_position_m"]))),
                                float(np.max(np.abs(plant_composed["wheel_velocity_m_s"] - baseline_plant["wheel_velocity_m_s"]))),
                            )
                            requested_error = float(np.max(np.abs(requested_wrench(control_row) - requested)))
                            realized_relative = float(np.max(np.abs(realized - baseline_realized)) /
                                                      max(np.max(np.abs(baseline_realized)), 1e-12))
                            oracle_errors.extend([
                                plant_composed["oracle_max_abs_error_m_s2"],
                                plant_fixed_torque["oracle_max_abs_error_m_s2"],
                            ])
                            direct_errors.append(direct_error)
                            wheel_errors.append(wheel_error)
                            requested_errors.append(requested_error)
                            realized_errors.append(realized_relative)
                            finite.extend([plant_composed["finite"], plant_fixed_torque["finite"]])
                            bilateral.append(bool(
                                np.all(np.asarray(plant_composed["contact"]["normal_load_n"]) > 0.0)))
                            signed[str(sign)] = {
                                "canonical_wheel_rate_delta_rad_s": canonical_delta,
                                "requested_wrench_flu": requested,
                                "wbc_torque_nm": torque,
                                "realized_wrench_flu": realized,
                                "realized_wrench_relative_difference": realized_relative,
                                "projection": {
                                    "direct_max_abs_error": direct_error,
                                    "xi_dxi_max_abs_error": wheel_error,
                                },
                                "composed_wbc_plant": plant_composed,
                                "fixed_baseline_torque_plant": plant_fixed_torque,
                            }
                        plus = np.asarray(signed["1"]["composed_wbc_plant"]["ddxi_m_s2"])
                        minus = np.asarray(signed["-1"]["composed_wbc_plant"]["ddxi_m_s2"])
                        fixed_plus = np.asarray(signed["1"]["fixed_baseline_torque_plant"]["ddxi_m_s2"])
                        fixed_minus = np.asarray(signed["-1"]["fixed_baseline_torque_plant"]["ddxi_m_s2"])
                        sensitivity = (plus - minus) / (2.0 * delta)
                        fixed_sensitivity = (fixed_plus - fixed_minus) / (2.0 * delta)
                        sensitivities[scale] = sensitivity
                        difference = plus - minus
                        closure_differences.append(float(np.max(np.abs(difference))))
                        scale_results[str(scale)] = {
                            "signed": signed,
                            "composed_sensitivity_per_rad_s": sensitivity,
                            "fixed_torque_sensitivity_per_rad_s": fixed_sensitivity,
                            "symmetric_ddxi_difference_m_s2": difference,
                            "grouped_difference": CONTRACT.grouped(difference),
                        }
                    full = sensitivities[1.0]
                    half = sensitivities[0.5]
                    consistency = float(np.max(np.abs(full - half)) /
                                        max(np.max(np.abs(half)), 1e-12))
                    consistency_errors.append(consistency)
                    mode_results[mode] = {
                        "scales": scale_results,
                        "full_half_sensitivity_relative_error": consistency,
                    }
                all_pairs.append({
                    "case": case["id"],
                    "tick": tick,
                    "time_s": float(state_row["time_s"]),
                    "baseline": {
                        "requested_wrench_flu": requested,
                        "wbc_torque_nm": baseline_torque,
                        "realized_wrench_flu": baseline_realized,
                        "plant": baseline_plant,
                    },
                    "modes": mode_results,
                })
    gates = method["gates"]
    projection_pass = (
        max(direct_errors) <= float(gates["direct_reduced_component_max_abs_error"])
        and max(wheel_errors) <= float(gates["reconstructed_wheel_state_max_abs_error"])
        and max(requested_errors) <= float(gates["requested_wrench_max_abs_error"])
    )
    oracle_pass = max(oracle_errors) <= float(gates["full_ddxi_oracle_max_abs_error_m_s2"])
    perturbation_pass = max(consistency_errors) <= float(gates["full_half_sensitivity_relative_error_max"])
    closure_failure = (
        max(closure_differences) > float(gates["closure_difference_max_abs_m_s2"])
        and max(closure_differences) / float(gates["closure_normalization_m_s2"])
            > float(gates["closure_normalized_max"])
    )
    valid = projection_pass and oracle_pass and perturbation_pass and all(finite) and all(bilateral)
    authority = authority_status(method)
    summary = {
        "authority": authority,
        "floating_base_oracle": True,
        "base_weld_active": False,
        "projection_pass": projection_pass,
        "full_ddxi_oracle_pass": oracle_pass,
        "perturbation_consistency_pass": perturbation_pass,
        "finite_pass": all(finite),
        "bilateral_contact_pass": all(bilateral),
        "closure_failure": closure_failure if valid else False,
        "maxima": {
            "direct_reduced_component_error": max(direct_errors),
            "xi_dxi_error": max(wheel_errors),
            "requested_wrench_error": max(requested_errors),
            "ddxi_oracle_error_m_s2": max(oracle_errors),
            "symmetric_ddxi_difference_m_s2": max(closure_differences),
            "normalized_symmetric_ddxi_difference": max(closure_differences) / float(gates["closure_normalization_m_s2"]),
            "full_half_sensitivity_relative_error": max(consistency_errors),
            "realized_wrench_relative_difference": max(realized_errors),
        },
        "wbc_realized_wrench_parity_pass": max(realized_errors) <= float(gates["physical_realized_wrench_relative_error_max"]),
        "physical_interaction_wrench_parity_established": False,
        "classification": (
            "P32-A_full_dynamics_oracle_incomplete" if not valid
            else "P32-C_16state_markov_closure_failure" if closure_failure
            else "P32-B_16state_markov_closure_confirmed"
        ),
        "hidden_family": "P32-F_contact_regime_hidden_state" if valid and closure_failure else "unresolved",
        "root_cause_model_class": "M5" if valid and closure_failure else "unresolved",
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "markov_closure.json").write_text(
        json.dumps(clean({"pairs": all_pairs}), indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    (output / "summary.json").write_text(
        json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    input_paths = [method_path, phase31_method_path, scene, CONTRACT_SCRIPT,
                   INPUT_SCRIPT, SWEEP_SOURCE, Path(__file__).resolve(), executable,
                   *[ROOT / path for path in method["phase31_authority"].values()], *raw_paths]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {
            "numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if valid else 2


if __name__ == "__main__":
    raise SystemExit(main())
