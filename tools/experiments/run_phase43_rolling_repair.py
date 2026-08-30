#!/usr/bin/env python3
"""Phase 43 minimal rolling-stabilization repair selection."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
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
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase43_rolling_repair_v1.json"
PHASE42_RUNNER = ROOT / "tools/experiments/run_phase42_causal_attribution.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P42 = load_module(PHASE42_RUNNER, "phase43_phase42_oracle")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


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


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(clean(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def semantic_error(left: Path, right: Path, ignored: set[str]) -> float:
    return P42.semantic_error(read_csv(left), read_csv(right), ignored)


def candidate_specs(config: dict[str, Any]) -> list[dict[str, Any]]:
    result = [{"candidate": "A", "gain": "trim", "kp": 0.0, "kd": 0.0,
               "rate_gain": 0.0}]
    for bandwidth in config["bandwidth_hz"]:
        omega = 2.0 * math.pi * float(bandwidth)
        for candidate in ("B", "C", "D"):
            result.append({
                "candidate": candidate,
                "gain": f"{bandwidth:g}Hz",
                "bandwidth_hz": bandwidth,
                "kp": omega * omega if candidate in ("C", "D") else 0.0,
                "kd": 2.0 * float(config["damping_ratio"]) * omega
                if candidate in ("C", "D") else 0.0,
                "rate_gain": omega if candidate in ("B", "D") else 0.0,
            })
    return result


def command(config: dict[str, Any], output: Path, case: str, spec: dict[str, Any],
            trim: np.ndarray, snapshot_tick: int | None = None) -> list[str]:
    values = [
        str(ROOT / config["executable"]), str(ROOT / config["scene"]), str(output),
        case, spec["gain"], str(spec["kp"]), str(spec["kd"]),
        str(spec["rate_gain"]), *map(str, trim),
    ]
    if snapshot_tick is not None:
        values += [str(ROOT / config["phase42_native_authority"]), str(snapshot_tick)]
    return values


def run_case(config: dict[str, Any], output: Path, case: str, spec: dict[str, Any],
             trim: np.ndarray, snapshot_tick: int | None = None) -> list[dict[str, str]]:
    subprocess.run(command(config, output, case, spec, trim, snapshot_tick), cwd=ROOT, check=True)
    return read_csv(output)


def optimize_trim(config: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    settings = config["trim"]
    baseline = {"gain": "trim-search", "kp": 0.0, "kd": 0.0, "rate_gain": 0.0}
    evaluations = 0
    with tempfile.TemporaryDirectory(prefix="phase43-trim-") as directory:
        root = Path(directory)

        def residual(trim: np.ndarray) -> np.ndarray:
            nonlocal evaluations
            path = root / f"eval-{evaluations:03d}.csv"
            rows = run_case(config, path, "R43-A__screen", baseline, trim, 0)
            evaluations += 1
            row = rows[0]
            ddxi = np.asarray([float(row["physical_ddxi_left"]),
                               float(row["physical_ddxi_right"])])
            qdd = np.asarray([float(row["realized_qdd_wheel_left"]),
                              float(row["realized_qdd_wheel_right"])])
            regularization = math.sqrt(float(settings["regularization"])) * trim
            return np.r_[ddxi, 0.05 * qdd, regularization]

        solved = least_squares(
            residual, np.zeros(4), bounds=(settings["lower"], settings["upper"]),
            max_nfev=int(settings["maximum_evaluations"]), xtol=1e-10, ftol=1e-10,
            gtol=1e-10, verbose=0,
        )
    return solved.x, {"success": solved.success, "status": solved.status,
                      "message": solved.message, "evaluations": evaluations,
                      "solver_nfev": solved.nfev,
                      "cost": solved.cost, "optimality": solved.optimality}


def candidate_control(case: str, row: dict[str, str]) -> np.ndarray:
    del case
    return -np.asarray([float(row[f"tau{index}"]) for index in range(6)])


def snapshot_audit(config: dict[str, Any], output: Path, specs: list[dict[str, Any]],
                   trim: np.ndarray) -> tuple[list[dict[str, Any]], dict[str, bool]]:
    native = read_csv(ROOT / config["phase42_native_authority"])
    native_by_tick = {int(row["control_tick"]): row for row in native
                      if row["record_kind"] == "pre_command"}
    p42_config = json.loads((ROOT / config["phase42_config"]).read_text(encoding="utf-8"))
    oracle = P42.Oracle(p42_config)
    rows: list[dict[str, Any]] = []
    tick0_baseline: dict[str, float] | None = None
    all_specs = [{"candidate": "0", "gain": "baseline", "kp": 0.0, "kd": 0.0,
                  "rate_gain": 0.0}, *specs]
    for spec in all_specs:
        candidate = spec["candidate"]
        for tick in config["snapshot_ticks"]:
            path = output / f"snapshot-{candidate}-{spec['gain']}-t{tick}.csv"
            control = run_case(config, path, f"R43-{candidate}__screen", spec,
                               trim if candidate == "A" else np.zeros(4), tick)[0]
            actual = oracle.evaluate(native_by_tick[tick], [], candidate_control(candidate, control))
            ddxi_left, ddxi_right = actual["ddxi_left_m_s2"], actual["ddxi_right_m_s2"]
            item = {
                "candidate": candidate, "gain": spec["gain"], "tick": tick,
                "ddxi_left_m_s2": ddxi_left, "ddxi_right_m_s2": ddxi_right,
                "ddxi_common_m_s2": 0.5 * (ddxi_left + ddxi_right),
                "ddxi_differential_m_s2": 0.5 * (ddxi_right - ddxi_left),
                "wheel_qdd_left_rad_s2": actual["wheel_ddq_left_rad_s2"],
                "wheel_qdd_right_rad_s2": actual["wheel_ddq_right_rad_s2"],
                "normal_load_left_n": actual["normal_load_left_n"],
                "normal_load_right_n": actual["normal_load_right_n"],
                "hard": float(control["hard"]),
                "slack": float(control["maximum_normalized_slack"]),
                "max_torque_nm": max(abs(float(control[f"tau{i}"])) for i in range(6)),
                "max_wrench_error": max(abs(float(control[f"realized_wrench{i}"]) -
                                            float(control[f"requested_wrench{i}"]))
                                        for i in range(12)),
                "full_dynamics_residual": actual["full_dynamics_residual_max_abs"],
                "contact_reconstruction_residual": actual["contact_applyft_jacobian_max_abs"],
            }
            rows.append(item)
            if candidate == "0" and tick == 0:
                tick0_baseline = item
    assert tick0_baseline is not None
    gates: dict[str, bool] = {}
    limits = config["gates"]
    baseline_acceleration = max(abs(tick0_baseline["ddxi_common_m_s2"]),
                                abs(tick0_baseline["ddxi_differential_m_s2"]))
    for spec in specs:
        item = next(row for row in rows if row["candidate"] == spec["candidate"] and
                    row["gain"] == spec["gain"] and row["tick"] == 0)
        acceleration = max(abs(item["ddxi_common_m_s2"]),
                           abs(item["ddxi_differential_m_s2"]))
        eq = ((acceleration <= float(limits["equilibrium_ddxi_abs_m_s2"]) or
               acceleration <= baseline_acceleration *
               float(limits["equilibrium_ddxi_relative_to_baseline"])) and
              max(abs(item["wheel_qdd_left_rad_s2"]),
                  abs(item["wheel_qdd_right_rad_s2"])) <=
              float(limits["equilibrium_wheel_qdd_abs_rad_s2"]))
        gates[f"{spec['candidate']}-{spec['gain']}"] = eq
    return rows, gates


def rollout_metrics(rows: list[dict[str, str]], config: dict[str, Any],
                    baseline_wrench_error: float) -> dict[str, Any]:
    limits = config["gates"]
    first = rows[0]
    rim_left = np.asarray([-0.05 * float(row["raw_dq2"]) for row in rows])
    rim_right = np.asarray([-0.05 * float(row["raw_dq5"]) for row in rows])
    rim_common = 0.5 * (rim_left + rim_right)
    rim_differential = 0.5 * (rim_left - rim_right)
    xi_left = np.asarray([float(row["xi_left"]) for row in rows])
    xi_right = np.asarray([float(row["xi_right"]) for row in rows])
    xi_common = 0.5 * (xi_left + xi_right)
    xi_differential = 0.5 * (xi_right - xi_left)
    base = np.asarray([[float(row[f"base_p{i}"]) for i in range(3)] for row in rows])
    base_speed = np.asarray([math.sqrt(sum(float(row[f"base_v{i}"]) ** 2 for i in range(3)))
                             for row in rows])
    angular_speed = np.asarray([math.sqrt(sum(float(row[f"base_omega{i}"]) ** 2
                                               for i in range(3))) for row in rows])
    rotation = np.asarray([math.sqrt(sum(float(row[f"base_rotvec{i}"]) ** 2
                                         for i in range(3))) for row in rows])
    wrench_errors = [abs(float(row[f"realized_wrench{i}"]) -
                         float(row[f"requested_wrench{i}"]))
                     for row in rows for i in range(12)]
    max_torque_margin = min(float(row[f"tau_margin{i}"]) for row in rows for i in range(6))
    gates = {
        "full_horizon": len(rows) == 1000 and int(rows[-1]["tick"]) == 999,
        "bilateral_contact": all(row["contact_left"] == "1" and row["contact_right"] == "1"
                                 for row in rows),
        "rate": max(np.max(np.abs(rim_common)), np.max(np.abs(rim_differential))) <=
                float(limits["maximum_direction_normalized_rim_rate_m_s"]),
        "xi": max(np.max(np.abs(xi_common - xi_common[0])),
                  np.max(np.abs(xi_differential - xi_differential[0]))) <=
              float(limits["maximum_xi_error_m"]),
        "base": (np.max(np.linalg.norm(base - base[0], axis=1)) <=
                 float(limits["maximum_base_position_change_m"]) and
                 np.max(np.abs(rotation - rotation[0])) <=
                 float(limits["maximum_base_rotation_change_rad"]) and
                 np.max(base_speed) <= float(limits["maximum_base_linear_speed_m_s"]) and
                 np.max(angular_speed) <= float(limits["maximum_base_angular_speed_rad_s"])),
        "wbc": (max(float(row["hard"]) for row in rows) <=
                float(limits["maximum_hard_violation"]) and
                max(float(row["maximum_normalized_slack"]) for row in rows) <=
                float(limits["maximum_normalized_slack"]) and
                max_torque_margin >= float(limits["minimum_torque_margin_nm"]) and
                all(row["model_status"] == "0" and row["controller_status"] == "0"
                    for row in rows)),
        "wrench_realization": max(wrench_errors) <= baseline_wrench_error +
                              float(limits["maximum_wrench_residual_degradation"]),
    }
    return {"pass": all(gates.values()), "gates": gates, "ticks": len(rows),
            "last_tick": int(rows[-1]["tick"]),
            "max_rim_common_m_s": float(np.max(np.abs(rim_common))),
            "max_rim_differential_m_s": float(np.max(np.abs(rim_differential))),
            "max_xi_common_error_m": float(np.max(np.abs(xi_common - xi_common[0]))),
            "max_xi_differential_error_m": float(np.max(np.abs(xi_differential - xi_differential[0]))),
            "max_base_position_change_m": float(np.max(np.linalg.norm(base - base[0], axis=1))),
            "max_wrench_error": max(wrench_errors),
            "rms_wrench_error": float(np.sqrt(np.mean(np.square(wrench_errors)))),
            "maximum_slack": max(float(row["maximum_normalized_slack"]) for row in rows),
            "minimum_torque_margin_nm": max_torque_margin}


def perturbation_metrics(rows: list[dict[str, str]], mode: str,
                         config: dict[str, Any], baseline_wrench_error: float) -> dict[str, Any]:
    start = int(round(float(config["perturbations"]["assessment_start_s"]) /
                      float(config["control_period_s"])))
    final_count = int(round(float(config["gates"]["final_window_s"]) /
                            float(config["control_period_s"])))
    rim_left = np.asarray([-0.05 * float(row["raw_dq2"]) for row in rows])
    rim_right = np.asarray([-0.05 * float(row["raw_dq5"]) for row in rows])
    xi_left = np.asarray([float(row["xi_left"]) for row in rows])
    xi_right = np.asarray([float(row["xi_right"]) for row in rows])
    signals = {
        "rate_common": 0.5 * (rim_left + rim_right),
        "rate_differential": 0.5 * (rim_left - rim_right),
        "xi_common": 0.5 * (xi_left + xi_right) - 0.5 * (xi_left[0] + xi_right[0]),
        "xi_differential": 0.5 * (xi_right - xi_left) - 0.5 * (xi_right[0] - xi_left[0]),
    }
    signal = np.abs(signals[mode])
    reference = max(float(signal[start]), float(np.max(signal[:start + 1])), 1e-9)
    growth = float(np.max(signal[start:]) / reference)
    final = float(np.max(signal[-final_count:]) / reference)
    base = rollout_metrics(rows, config, baseline_wrench_error)
    gates = dict(base["gates"])
    gates["bounded_growth"] = growth <= float(config["gates"]["maximum_perturbation_growth_ratio"])
    gates["bounded_final"] = final <= float(config["gates"]["maximum_perturbation_final_ratio"])
    classification = "returning" if final <= 0.5 else "bounded" if all(gates.values()) else "amplifying"
    return {"pass": all(gates.values()), "classification": classification,
            "growth_ratio": growth, "final_ratio": final, "gates": gates}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    output.mkdir(parents=True)
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    specs = candidate_specs(config)

    baseline_spec = {"candidate": "0", "gain": "baseline", "kp": 0.0, "kd": 0.0,
                     "rate_gain": 0.0}
    baseline_a = output / "baseline-a.csv"
    baseline_b = output / "baseline-b.csv"
    baseline_rows = run_case(config, baseline_a, "R43-0_nominal", baseline_spec, np.zeros(4))
    run_case(config, baseline_b, "R43-0_nominal", baseline_spec, np.zeros(4))
    failure_tick = next((int(row["tick"]) for row in baseline_rows
                         if row["contact_right"] != "1"), None)
    baseline_replay = semantic_error(baseline_a, baseline_b, {"wbc_time_s"})
    baseline_wrench_error = max(
        abs(float(row[f"realized_wrench{i}"]) - float(row[f"requested_wrench{i}"]))
        for row in baseline_rows for i in range(12))

    trim, trim_solver = optimize_trim(config)
    snapshots, equilibrium_gates = snapshot_audit(config, output, specs, trim)
    write_csv(output / "snapshot-audit.csv", snapshots)

    nominal: dict[str, Any] = {}
    nominal_paths: dict[str, Path] = {}
    for spec in specs:
        key = f"{spec['candidate']}-{spec['gain']}"
        path = output / f"nominal-{key}.csv"
        rows = run_case(config, path, f"R43-{spec['candidate']}__nominal", spec,
                        trim if spec["candidate"] == "A" else np.zeros(4))
        nominal[key] = rollout_metrics(rows, config, baseline_wrench_error)
        nominal[key]["equilibrium_gate"] = equilibrium_gates[key]
        nominal[key]["pass"] = nominal[key]["pass"] and equilibrium_gates[key]
        nominal_paths[key] = path

    perturbations: dict[str, Any] = {}
    modes = ("rate_common", "rate_differential", "xi_common", "xi_differential")
    for spec in specs:
        key = f"{spec['candidate']}-{spec['gain']}"
        if not nominal[key]["pass"]:
            continue
        perturbations[key] = {}
        for mode in modes:
            path = output / f"perturb-{key}-{mode}.csv"
            rows = run_case(config, path, f"R43-{spec['candidate']}__{mode}", spec,
                            trim if spec["candidate"] == "A" else np.zeros(4))
            perturbations[key][mode] = perturbation_metrics(
                rows, mode, config, baseline_wrench_error)

    passing: list[dict[str, Any]] = []
    for spec in specs:
        key = f"{spec['candidate']}-{spec['gain']}"
        perturb_pass = key in perturbations and all(value["pass"]
                                                    for value in perturbations[key].values())
        if nominal[key]["pass"] and perturb_pass:
            passing.append(spec)
    candidate_rank = {"A": 0, "B": 1, "C": 1, "D": 2}
    preference = {"C": 0, "B": 1, "A": 2, "D": 3}
    passing.sort(key=lambda spec: (candidate_rank[spec["candidate"]],
                                   preference[spec["candidate"]],
                                   float(spec.get("bandwidth_hz", 0.0))))
    selected = passing[0] if passing else None
    classification = f"P43-{selected['candidate']}" if selected else "P43-U"
    gates = {
        "baseline_reproduced": failure_tick == int(config["baseline_failure_tick"]),
        "baseline_replay": baseline_replay <= float(config["semantic_tolerance"]),
        "trim_solver": bool(trim_solver["success"]),
        "selection_resolved": selected is not None,
    }
    summary = {"pass": all(gates.values()), "classification": classification,
               "selected": selected, "gates": gates, "failure_tick": failure_tick,
               "baseline_replay_max_abs_error": baseline_replay,
               "baseline_max_wrench_error": baseline_wrench_error,
               "trim": {"delta_wrench_components": trim, "solver": trim_solver},
               "equilibrium_gates": equilibrium_gates,
               "nominal": nominal, "perturbations": perturbations,
               "phase34_run": False, "nmpc_12d_run": False,
               "repair_16d": False, "plant_modification": False,
               "contact_modification": False}
    write_json(output / "candidate-summary.json", {"specs": specs, "passing": passing,
                                                    "selected": selected})
    write_json(output / "gate-results.json", gates)
    write_json(output / "summary.json", summary)
    sources = [config_path, ROOT / config["scene"], ROOT / config["executable"],
               ROOT / config["phase42_config"], ROOT / config["phase42_native_authority"],
               ROOT / config["phase42_summary"], Path(__file__).resolve(), PHASE42_RUNNER,
               ROOT / "ros_ws/src/wheel_leg_core/include/wheel_leg_core/weighted_wbc_problem.hpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp",
               ROOT / "ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp",
               ROOT / "ros_ws/src/wheel_leg_mujoco/src/phase35_workspace_attribution_loop.cpp"]
    write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": args.replay_of, "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
        "phase34_run": False, "nmpc_12d_run": False, "repair_16d": False,
        "plant_modification": False, "contact_modification": False,
    })
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
