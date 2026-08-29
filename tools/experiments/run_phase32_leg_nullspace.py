#!/usr/bin/env python3
"""Phase 32 C1/C2 same-x16 leg configuration and velocity pairs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import platform
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase32_markov_closure_v1.json"
MARKOV_SCRIPT = ROOT / "tools/experiments/run_phase32_markov_closure.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None: raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


M = load_module(MARKOV_SCRIPT, "phase32_leg_markov")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean(item) for item in value]
    return value


def patched_wbc(
    executable: Path, rows: list[dict[str, str]], fieldnames: list[str], tick: int,
    patches: dict[str, float], path: Path,
) -> dict[str, str]:
    changed = [dict(row) for row in rows]
    target = next(row for row in changed if int(row["tick"]) == tick)
    for field, delta in patches.items():
        target[field] = repr(float(target[field]) + delta)
    M.write_csv(path, changed, fieldnames)
    return M.run_baselines(executable, path, [tick])[tick]


def equality_data(
    geometry: Any, base_weld: int, equality_id: int,
    qpos: np.ndarray, qvel: np.ndarray, time_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    geometry.data.eq_active[base_weld] = 0
    geometry.set_state(qpos, qvel, time_s)
    rows = np.flatnonzero(
        (np.asarray(geometry.data.efc_type) == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
        & (np.asarray(geometry.data.efc_id) == equality_id))
    jacobian = geometry.data.efc_J.reshape(geometry.data.nefc, geometry.model.nv)
    return geometry.data.efc_pos[rows].copy(), geometry.data.efc_vel[rows].copy(), jacobian[rows].copy()


def xi_velocity_row(
    geometry: Any, base_weld: int, qpos: np.ndarray, qvel: np.ndarray,
    dofs: list[int], side: int, time_s: float,
) -> np.ndarray:
    zero = qvel.copy()
    zero[dofs] = 0.0
    geometry.data.eq_active[base_weld] = 0
    geometry.set_state(qpos, zero, time_s)
    offset = float(geometry.current_value()["velocity"][side])
    result = np.zeros(len(dofs))
    for index, dof in enumerate(dofs):
        unit = zero.copy()
        unit[dof] = 1.0
        geometry.data.eq_active[base_weld] = 0
        geometry.set_state(qpos, unit, time_s)
        result[index] = float(geometry.current_value()["velocity"][side]) - offset
    return result


def c1_variant(
    geometry: Any, base_weld: int, qpos: np.ndarray, qvel: np.ndarray,
    time_s: float, sides: list[dict[str, Any]], hip_delta: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    changed_qpos = qpos.copy()
    changed_qvel = qvel.copy()
    baseline_geometry = []
    for side in sides:
        equality, equality_velocity, _ = equality_data(
            geometry, base_weld, side["equality_id"], qpos, qvel, time_s)
        geometry.data.eq_active[base_weld] = 0
        geometry.set_state(qpos, qvel, time_s)
        baseline_geometry.append((
            equality, equality_velocity,
            float(geometry.current_value()["position"][side["index"]]),
            float(geometry.current_value()["velocity"][side["index"]]),
        ))
    diagnostics = {}
    for side, baseline in zip(sides, baseline_geometry):
        qindices = side["qpos"]
        fixed, unknown = qindices[0], qindices[1:]
        changed_qpos[fixed] = qpos[fixed] + hip_delta

        def residual(values: np.ndarray) -> np.ndarray:
            trial = changed_qpos.copy()
            trial[unknown] = values
            equality, _, _ = equality_data(
                geometry, base_weld, side["equality_id"], trial, qvel, time_s)
            geometry.data.eq_active[base_weld] = 0
            geometry.set_state(trial, qvel, time_s)
            xi = float(geometry.current_value()["position"][side["index"]])
            return np.r_[
                (equality[[0, 2]] - baseline[0][[0, 2]]) / 1e-3,
                (xi - baseline[2]) / 1e-3,
            ]

        solved = least_squares(
            residual, qpos[unknown], xtol=1e-14, ftol=1e-14,
            gtol=1e-14, max_nfev=200)
        changed_qpos[unknown] = solved.x
        equality, current_equality_velocity, jacobian = equality_data(
            geometry, base_weld, side["equality_id"], changed_qpos,
            changed_qvel, time_s)
        geometry.data.eq_active[base_weld] = 0
        geometry.set_state(changed_qpos, changed_qvel, time_s)
        current_dxi = float(geometry.current_value()["velocity"][side["index"]])
        dxi_row = xi_velocity_row(
            geometry, base_weld, changed_qpos, changed_qvel,
            side["dofs"], side["index"], time_s)
        matrix = np.vstack((jacobian[[0, 2]][:, side["dofs"]], dxi_row))
        target = np.r_[baseline[1][[0, 2]], baseline[3]]
        current = np.r_[current_equality_velocity[[0, 2]], current_dxi]
        changed_qvel[side["dofs"]] += np.linalg.lstsq(
            matrix, target - current, rcond=None)[0]
        final_equality, final_velocity, _ = equality_data(
            geometry, base_weld, side["equality_id"], changed_qpos,
            changed_qvel, time_s)
        diagnostics[side["name"]] = {
            "solve_cost": float(solved.cost),
            "closure_position_max_abs_change_m": float(np.max(np.abs(final_equality - baseline[0]))),
            "closure_velocity_max_abs_change_m_s": float(np.max(np.abs(final_velocity - baseline[1]))),
            "native_joint_position_delta_rad": changed_qpos[qindices] - qpos[qindices],
            "native_joint_velocity_delta_rad_s": changed_qvel[side["dofs"]] - qvel[side["dofs"]],
        }
    return changed_qpos, changed_qvel, diagnostics


def c2_variant(
    geometry: Any, base_weld: int, qpos: np.ndarray, qvel: np.ndarray,
    time_s: float, sides: list[dict[str, Any]], active_scale: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    changed = qvel.copy()
    diagnostics = {}
    for side in sides:
        _, _, jacobian = equality_data(
            geometry, base_weld, side["equality_id"], qpos, qvel, time_s)
        dxi_row = xi_velocity_row(
            geometry, base_weld, qpos, qvel, side["dofs"], side["index"], time_s)
        matrix = np.vstack((jacobian[[0, 2]][:, side["dofs"]], dxi_row))
        _, _, right = np.linalg.svd(matrix)
        null = right[-1]
        null *= active_scale / max(float(np.max(np.abs(null[:2]))), 1e-12)
        changed[side["dofs"]] += null
        diagnostics[side["name"]] = {
            "constraint_null_residual": float(np.max(np.abs(matrix @ null))),
            "native_joint_velocity_delta_rad_s": null,
        }
    return qpos.copy(), changed, diagnostics


def control_patches(qpos: np.ndarray, qvel: np.ndarray,
                    changed_qpos: np.ndarray, changed_qvel: np.ndarray) -> dict[str, float]:
    mapping = [
        ("q0", 12), ("q1", 13), ("q3", 7), ("q4", 8),
        ("dq0", 11), ("dq1", 12), ("dq3", 6), ("dq4", 7),
    ]
    patches = {}
    for field, native in mapping:
        source = qvel if field.startswith("dq") else qpos
        changed = changed_qvel if field.startswith("dq") else changed_qpos
        patches[field] = -float(changed[native] - source[native])
    return patches


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
    scene = ROOT / method["scene"]
    executable = ROOT / method["wbc_sweep_executable"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = M.CONTRACT.Geometry(model, method["body_site_contract"])
    base_weld = M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    wheel_geoms = [M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in method["body_site_contract"]["wheel_geoms"]]
    floor = M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    sides = [
        {"name": "left", "index": 0, "qpos": [12, 13, 15, 16], "dofs": [11, 12, 14, 15],
         "equality_id": M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "left_leg_closure")},
        {"name": "right", "index": 1, "qpos": [7, 8, 10, 11], "dofs": [6, 7, 9, 10],
         "equality_id": M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "right_leg_closure")},
    ]
    raw_root = ROOT / method["source_phase28_run"]
    raw_paths = []
    samples = []
    oracle_errors = []
    projection_errors = []
    closure_differences = {"C1": [], "C2": []}
    consistency_errors = {"C1": [], "C2": []}
    contact_passes = []
    finite_passes = []
    realized_errors = []
    with tempfile.TemporaryDirectory(prefix="phase32-leg-") as temp_name:
        temp = Path(temp_name)
        for case in method["cases"]:
            control_path = raw_root / f"{case['id']}_control.csv"
            plant_path = raw_root / f"{case['id']}_plant.csv"
            raw_paths.extend([control_path, plant_path])
            controls = M.read_csv(control_path)
            control_by_tick = {int(row["tick"]): row for row in controls}
            fieldnames = list(controls[0].keys())
            plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
                     for row in M.read_csv(plant_path)}
            baselines = M.run_baselines(executable, control_path, case["authority_ticks"])
            for tick in case["authority_ticks"]:
                row = plant[(tick - 1, 4)]
                qpos = M.vector(row, "qpos", model.nq)
                qvel = M.vector(row, "qvel", model.nv)
                time_s = float(row["time_s"])
                baseline_sweep = baselines[tick]
                baseline_torque = M.vector(baseline_sweep, "tau", 6)
                baseline_realized = M.vector(baseline_sweep, "realized", 12)
                baseline = M.evaluate_plant(
                    geometry, base_weld, qpos, qvel, baseline_torque,
                    time_s, wheel_geoms, floor)
                families = {}
                for family in ("C1", "C2"):
                    scales = {}
                    sensitivities = {}
                    for scale in (1.0, 0.5):
                        signed = {}
                        for sign in (-1, 1):
                            if family == "C1":
                                changed_qpos, changed_qvel, construction = c1_variant(
                                    geometry, base_weld, qpos, qvel, time_s,
                                sides, sign * scale * float(
                                    method.get("leg_pairs", {}).get(
                                        "configuration_hip_delta_rad", 0.001)))
                                denominator = scale * float(
                                    method.get("leg_pairs", {}).get(
                                        "configuration_hip_delta_rad", 0.001))
                            else:
                                changed_qpos, changed_qvel, construction = c2_variant(
                                    geometry, base_weld, qpos, qvel, time_s,
                                    sides, sign * scale * float(
                                        method.get("leg_pairs", {}).get(
                                            "velocity_active_max_delta_rad_s", 0.5)))
                                denominator = scale * float(
                                    method.get("leg_pairs", {}).get(
                                        "velocity_active_max_delta_rad_s", 0.5))
                            patches = control_patches(
                                qpos, qvel, changed_qpos, changed_qvel)
                            sweep = patched_wbc(
                                executable, controls, fieldnames, tick, patches,
                                temp / f"{case['id']}-{tick}-{family}-{scale}-{sign}.csv")
                            torque = M.vector(sweep, "tau", 6)
                            realized = M.vector(sweep, "realized", 12)
                            composed = M.evaluate_plant(
                                geometry, base_weld, changed_qpos, changed_qvel,
                                torque, time_s, wheel_geoms, floor)
                            fixed = M.evaluate_plant(
                                geometry, base_weld, changed_qpos, changed_qvel,
                                baseline_torque, time_s, wheel_geoms, floor)
                            projection_error = max(
                                float(np.max(np.abs(composed["wheel_position_m"] - baseline["wheel_position_m"]))),
                                float(np.max(np.abs(composed["wheel_velocity_m_s"] - baseline["wheel_velocity_m_s"]))),
                            )
                            realized_error = float(np.max(np.abs(realized - baseline_realized)) /
                                                   max(np.max(np.abs(baseline_realized)), 1e-12))
                            oracle_errors.extend([composed["oracle_max_abs_error_m_s2"], fixed["oracle_max_abs_error_m_s2"]])
                            projection_errors.append(projection_error)
                            realized_errors.append(realized_error)
                            contact_passes.append(bool(np.all(np.asarray(composed["contact"]["normal_load_n"]) > 0.0)))
                            finite_passes.extend([composed["finite"], fixed["finite"]])
                            signed[str(sign)] = {
                                "construction": construction,
                                "control_patches": patches,
                                "projection_xi_dxi_max_abs_error": projection_error,
                                "wbc_torque_nm": torque,
                                "realized_wrench_flu": realized,
                                "realized_wrench_relative_difference": realized_error,
                                "composed_wbc_plant": composed,
                                "fixed_baseline_torque_plant": fixed,
                            }
                        plus = np.asarray(signed["1"]["composed_wbc_plant"]["ddxi_m_s2"])
                        minus = np.asarray(signed["-1"]["composed_wbc_plant"]["ddxi_m_s2"])
                        sensitivity = (plus - minus) / (2.0 * denominator)
                        sensitivities[scale] = sensitivity
                        difference = plus - minus
                        closure_differences[family].append(float(np.max(np.abs(difference))))
                        scales[str(scale)] = {
                            "signed": signed,
                            "symmetric_ddxi_difference_m_s2": difference,
                            "grouped_difference": M.CONTRACT.grouped(difference),
                            "sensitivity": sensitivity,
                        }
                    consistency = float(np.max(np.abs(sensitivities[1.0] - sensitivities[0.5])) /
                                        max(np.max(np.abs(sensitivities[0.5])), 1e-12))
                    consistency_errors[family].append(consistency)
                    families[family] = {"scales": scales, "full_half_sensitivity_relative_error": consistency}
                samples.append({"case": case["id"], "tick": tick, "baseline": baseline, "families": families})
    gates = method["gates"]
    valid = (
        max(oracle_errors) <= float(gates["full_ddxi_oracle_max_abs_error_m_s2"])
        and max(projection_errors) <= float(gates["reconstructed_wheel_state_max_abs_error"])
        and max(max(values) for values in consistency_errors.values()) <= float(gates["full_half_sensitivity_relative_error_max"])
        and all(contact_passes) and all(finite_passes)
    )
    failures = {
        family: max(values) > float(gates["closure_difference_max_abs_m_s2"])
        for family, values in closure_differences.items()
    }
    summary = {
        "valid": valid,
        "projection_pass": max(projection_errors) <= float(gates["reconstructed_wheel_state_max_abs_error"]),
        "full_ddxi_oracle_pass": max(oracle_errors) <= float(gates["full_ddxi_oracle_max_abs_error_m_s2"]),
        "perturbation_consistency_pass": max(max(values) for values in consistency_errors.values()) <= float(gates["full_half_sensitivity_relative_error_max"]),
        "bilateral_contact_pass": all(contact_passes), "finite_pass": all(finite_passes),
        "C1_closure_failure": failures["C1"] if valid else False,
        "C2_closure_failure": failures["C2"] if valid else False,
        "maxima": {
            "xi_dxi_projection_error": max(projection_errors),
            "ddxi_oracle_error_m_s2": max(oracle_errors),
            "C1_symmetric_ddxi_difference_m_s2": max(closure_differences["C1"]),
            "C2_symmetric_ddxi_difference_m_s2": max(closure_differences["C2"]),
            "C1_full_half_sensitivity_relative_error": max(consistency_errors["C1"]),
            "C2_full_half_sensitivity_relative_error": max(consistency_errors["C2"]),
            "realized_wrench_relative_difference": max(realized_errors),
        },
        "wbc_realized_wrench_parity_pass": max(realized_errors) <= float(gates["physical_realized_wrench_relative_error_max"]),
        "physical_interaction_wrench_parity_established": False,
        "classification": "P32-C_16state_markov_closure_failure" if valid and any(failures.values()) else "P32-A_full_dynamics_oracle_incomplete" if not valid else "P32-B_16state_markov_closure_confirmed",
        "hidden_families": [name for name, failed in (("P32-D_leg_configuration_hidden_state", failures["C1"]), ("P32-E_leg_velocity_hidden_state", failures["C2"])) if valid and failed],
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "leg_nullspace.json").write_text(json.dumps(clean({"samples": samples}), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [method_path, scene, MARKOV_SCRIPT, Path(__file__).resolve(), executable, *raw_paths]
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv), "replay_of": args.replay_of, "python": platform.python_version(), "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__}, "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if valid else 2


if __name__ == "__main__": raise SystemExit(main())
