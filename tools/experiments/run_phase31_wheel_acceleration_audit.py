#!/usr/bin/env python3
"""Phase 31 Gates 4-5: wheel acceleration oracle and direct Eq.(12) audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase31_wheel_state_contract_v1.json"
CONTRACT_SCRIPT = ROOT / "tools/experiments/run_phase31_wheel_state_contract.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CONTRACT = load_module(CONTRACT_SCRIPT, "phase31_contract_oracle")


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


def wrench(row: dict[str, str], kind: str) -> np.ndarray:
    return np.array([float(row[f"phase27_{kind}_wrench{i}"]) for i in range(12)])


def eq12(control: np.ndarray, config: dict) -> np.ndarray:
    body_mass = float(config["body_mass_kg"])
    radius = float(config["wheel_radius_m"])
    denominator = float(config["wheel_mass_kg"]) * radius + float(config["wheel_axle_inertia_kg_m2"]) / radius
    base_forward = (control[0] + control[6]) / body_mass
    return np.array([
        -base_forward - (radius * control[0] + control[4]) / denominator,
        -base_forward - (radius * control[6] + control[10]) / denominator,
    ])


def sample(
    tick: int,
    control: dict[int, dict[str, str]],
    plant: dict[tuple[int, int], dict[str, str]],
    geometry: Any,
    ocp: dict,
) -> dict[str, Any]:
    row = control[tick]
    center_row = plant[(tick - 1, 4)]
    interior_rows = [plant[(tick, substep)] for substep in range(5)]
    interior_values = [geometry.value(item) for item in interior_rows]
    times = np.array([float(item["time_s"]) for item in interior_rows])
    dt = float(times[1] - times[0])
    if not np.allclose(np.diff(times), dt, rtol=0.0, atol=1e-14):
        raise RuntimeError(f"nonuniform plant dt at tick {tick}: {times}")
    velocities = [item["velocity"] for item in interior_values]
    second_order = (velocities[3] - velocities[1]) / (2.0 * dt)
    fourth_order = (velocities[0] - 8.0 * velocities[1] + 8.0 * velocities[3] - velocities[4]) / (12.0 * dt)
    plant_acceleration = fourth_order
    requested = wrench(row, "requested")
    realized = wrench(row, "realized")
    requested_eq12 = eq12(requested, ocp)
    realized_eq12 = eq12(realized, ocp)
    oracle_delta = fourth_order - second_order
    requested_residual = plant_acceleration - requested_eq12
    realized_residual = plant_acceleration - realized_eq12
    return {
        "tick": tick,
        "time_s": float(center_row["time_s"]),
        "dt_s": dt,
        "latch": int(row["latch"]),
        "plant_ddxi_fourth_order_velocity_fd": CONTRACT.grouped(fourth_order),
        "plant_ddxi_second_order_velocity_fd": CONTRACT.grouped(second_order),
        "acceleration_oracle_delta": CONTRACT.grouped(oracle_delta),
        "acceleration_oracle_max_abs_error_m_s2": float(np.max(np.abs(oracle_delta))),
        "requested_eq12_ddxi": CONTRACT.grouped(requested_eq12),
        "realized_eq12_ddxi": CONTRACT.grouped(realized_eq12),
        "plant_minus_requested_eq12": CONTRACT.grouped(requested_residual),
        "plant_minus_realized_eq12": CONTRACT.grouped(realized_residual),
        "requested_residual_max_abs_m_s2": float(np.max(np.abs(requested_residual))),
        "realized_residual_max_abs_m_s2": float(np.max(np.abs(realized_residual))),
        "requested_vs_realized_eq12_max_abs_m_s2": float(np.max(np.abs(requested_eq12 - realized_eq12))),
        "requested_wrench": requested,
        "realized_wrench": realized,
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
    ocp_path = ROOT / method["source_ocp_config"]
    ocp = json.loads(ocp_path.read_text(encoding="utf-8"))
    scene = ROOT / method["scene"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = CONTRACT.Geometry(model, method["body_site_contract"])
    raw_root = ROOT / method["source_phase28_run"]
    results = {}
    raw_paths = []
    for case in method["cases"]:
        control_path = raw_root / f"{case['id']}_control.csv"
        plant_path = raw_root / f"{case['id']}_plant.csv"
        raw_paths.extend([control_path, plant_path])
        control = {int(row["tick"]): row for row in read_csv(control_path)}
        plant = {(int(row["control_tick"]), int(row["physics_substep"])): row for row in read_csv(plant_path)}
        results[case["id"]] = [sample(tick, control, plant, geometry, ocp) for tick in case["authority_ticks"]]
    samples = [item for values in results.values() for item in values]
    oracle_max = max(item["acceleration_oracle_max_abs_error_m_s2"] for item in samples)
    requested_max = max(item["requested_residual_max_abs_m_s2"] for item in samples)
    realized_max = max(item["realized_residual_max_abs_m_s2"] for item in samples)
    oracle_pass = oracle_max <= float(method["gates"]["acceleration_position_vs_velocity_fd_max_abs_error_m_s2"])
    significant = max(requested_max, realized_max) > float(method["gates"]["significant_eq12_residual_m_s2"])
    summary = {
        "acceleration_oracle_pass": oracle_pass,
        "maxima": {
            "position_vs_velocity_fd_m_s2": oracle_max,
            "plant_minus_requested_eq12_m_s2": requested_max,
            "plant_minus_realized_eq12_m_s2": realized_max,
            "requested_vs_realized_eq12_m_s2": max(item["requested_vs_realized_eq12_max_abs_m_s2"] for item in samples),
        },
        "significant_eq12_residual": significant,
        "classification": (
            "acceleration_oracle_inconclusive" if not oracle_pass
            else "eq12_residual_present_needs_controlled_input_response" if significant
            else "eq12_direct_corpus_pass"
        ),
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "wheel_acceleration_audit.json").write_text(json.dumps(clean(results), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [method_path, ocp_path, scene, CONTRACT_SCRIPT, Path(__file__).resolve(), *raw_paths]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {"numpy": np.__version__, "mujoco": mujoco.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if oracle_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
