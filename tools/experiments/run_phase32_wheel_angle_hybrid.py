#!/usr/bin/env python3
"""Phase 32 hybrid wheel-angle/contact-patch same-x16 audit."""

from __future__ import annotations

import argparse
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

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase32_wheel_angle_hybrid_v1.json"
MARKOV_SCRIPT = ROOT / "tools/experiments/run_phase32_markov_closure.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None: raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


M = load_module(MARKOV_SCRIPT, "phase32_angle_markov")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean(item) for item in value]
    return value


def patched_wbc(executable: Path, rows: list[dict[str, str]], fields: list[str],
                tick: int, delta: float, path: Path) -> dict[str, str]:
    changed = [dict(row) for row in rows]
    target = next(row for row in changed if int(row["tick"]) == tick)
    target["q2"] = repr(float(target["q2"]) - delta)
    target["q5"] = repr(float(target["q5"]) - delta)
    M.write_csv(path, changed, fields)
    return M.run_baselines(executable, path, [tick])[tick]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists(): raise RuntimeError(f"output already exists: {output}")
    method_path = args.method.resolve()
    spec = json.loads(method_path.read_text(encoding="utf-8"))
    base_path = ROOT / spec["base_method"]
    method = json.loads(base_path.read_text(encoding="utf-8"))
    scene = ROOT / method["scene"]
    executable = ROOT / method["wbc_sweep_executable"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = M.CONTRACT.Geometry(model, method["body_site_contract"])
    base_weld = M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    wheel_geoms = [M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in method["body_site_contract"]["wheel_geoms"]]
    floor = M.CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    wheel_qpos = [14, 9]
    raw_root = ROOT / method["source_phase28_run"]
    samples = []
    raw_paths = []
    projection_errors = []
    oracle_errors = []
    full_differences = []
    smooth_consistency = []
    contact_passes = []
    finite_passes = []
    realized_errors = []
    with tempfile.TemporaryDirectory(prefix="phase32-angle-") as temp_name:
        temp = Path(temp_name)
        for case in method["cases"]:
            control_path = raw_root / f"{case['id']}_control.csv"
            plant_path = raw_root / f"{case['id']}_plant.csv"
            raw_paths.extend([control_path, plant_path])
            controls = M.read_csv(control_path)
            fields = list(controls[0].keys())
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
                baseline = M.evaluate_plant(geometry, base_weld, qpos, qvel,
                                            baseline_torque, time_s, wheel_geoms, floor)
                scales = {}
                sensitivities = {}
                for scale in (1.0, float(spec["half_scale"])):
                    signed = {}
                    delta = scale * float(spec["wheel_angle_delta_rad"])
                    for sign in (-1, 1):
                        signed_delta = sign * delta
                        changed_qpos = qpos.copy()
                        changed_qpos[wheel_qpos] += signed_delta
                        sweep = patched_wbc(executable, controls, fields, tick,
                                            signed_delta,
                                            temp / f"{case['id']}-{tick}-{scale}-{sign}.csv")
                        torque = M.vector(sweep, "tau", 6)
                        realized = M.vector(sweep, "realized", 12)
                        composed = M.evaluate_plant(
                            geometry, base_weld, changed_qpos, qvel, torque,
                            time_s, wheel_geoms, floor)
                        projection_error = max(
                            float(np.max(np.abs(composed["wheel_position_m"] - baseline["wheel_position_m"]))),
                            float(np.max(np.abs(composed["wheel_velocity_m_s"] - baseline["wheel_velocity_m_s"]))),
                        )
                        realized_error = float(np.max(np.abs(realized - baseline_realized)) /
                                               max(np.max(np.abs(baseline_realized)), 1e-12))
                        projection_errors.append(projection_error)
                        oracle_errors.append(composed["oracle_max_abs_error_m_s2"])
                        realized_errors.append(realized_error)
                        contact_passes.append(bool(np.all(np.asarray(composed["contact"]["normal_load_n"]) > 0.0)))
                        finite_passes.append(composed["finite"])
                        signed[str(sign)] = {
                            "wheel_angle_delta_rad": signed_delta,
                            "projection_xi_dxi_max_abs_error": projection_error,
                            "wbc_torque_nm": torque,
                            "realized_wrench_flu": realized,
                            "realized_wrench_relative_difference": realized_error,
                            "plant": composed,
                        }
                    plus = np.asarray(signed["1"]["plant"]["ddxi_m_s2"])
                    minus = np.asarray(signed["-1"]["plant"]["ddxi_m_s2"])
                    difference = plus - minus
                    sensitivities[scale] = difference / (2.0 * delta)
                    scales[str(scale)] = {
                        "signed": signed,
                        "symmetric_ddxi_difference_m_s2": difference,
                        "grouped_difference": M.CONTRACT.grouped(difference),
                    }
                    if scale == 1.0: full_differences.append(float(np.max(np.abs(difference))))
                consistency = float(np.max(np.abs(sensitivities[1.0] - sensitivities[float(spec["half_scale"])])) /
                                    max(np.max(np.abs(sensitivities[float(spec["half_scale"])])), 1e-12))
                smooth_consistency.append(consistency)
                samples.append({"case": case["id"], "tick": tick,
                                "scales": scales,
                                "smooth_full_half_relative_error_diagnostic_only": consistency})
    gate = spec["hybrid_gate"]
    valid = (max(projection_errors) <= float(gate["projection_max_abs_error"])
             and max(oracle_errors) <= float(gate["oracle_max_abs_error_m_s2"])
             and all(contact_passes) and all(finite_passes))
    discrete_failure = min(full_differences) > float(gate["pair_min_ddxi_difference_m_s2"])
    summary = {
        "valid": valid, "bilateral_contact_pass": all(contact_passes),
        "finite_pass": all(finite_passes),
        "projection_pass": max(projection_errors) <= float(gate["projection_max_abs_error"]),
        "full_ddxi_oracle_pass": max(oracle_errors) <= float(gate["oracle_max_abs_error_m_s2"]),
        "smooth_full_half_gate_applicable": False,
        "smooth_full_half_limitation": gate["reason"],
        "fresh_replay_required_for_decision": bool(gate["requires_byte_identical_fresh_replay"]),
        "discrete_contact_patch_closure_failure_candidate": valid and discrete_failure,
        "maxima": {"xi_dxi_projection_error": max(projection_errors),
                   "ddxi_oracle_error_m_s2": max(oracle_errors),
                   "minimum_symmetric_ddxi_difference_m_s2": min(full_differences),
                   "maximum_symmetric_ddxi_difference_m_s2": max(full_differences),
                   "smooth_full_half_relative_error_diagnostic_only": max(smooth_consistency),
                   "realized_wrench_relative_difference": max(realized_errors)},
        "wbc_realized_wrench_parity_pass": max(realized_errors) <= 0.02,
        "physical_interaction_wrench_parity_established": False,
        "classification": "P32-F_discrete_contact_patch_hidden_state_candidate" if valid and discrete_failure else "P32-A_full_dynamics_oracle_incomplete",
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "wheel_angle_hybrid.json").write_text(json.dumps(clean({"samples": samples}), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [method_path, base_path, scene, MARKOV_SCRIPT, Path(__file__).resolve(), executable, *raw_paths]
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv), "replay_of": args.replay_of, "python": platform.python_version(), "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__}, "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if valid and discrete_failure else 2


if __name__ == "__main__": raise SystemExit(main())
