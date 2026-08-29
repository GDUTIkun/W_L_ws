#!/usr/bin/env python3
"""Branch-M recorded-MuJoCo rollout audit for the Phase 30 v3 model."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import importlib.util
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import casadi as ca
import mujoco
import numpy as np
import scipy
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase30_reference_consistency_v3.json"
REFERENCE_SCRIPT = ROOT / "tools/experiments/run_phase30_reference_consistency.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


REF = load_module(REFERENCE_SCRIPT, "phase30_v3_reference_oracle")
P29 = REF.P29


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


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def state_from_row(row: dict[str, str], anchor: Rotation) -> np.ndarray:
    quaternion = np.array([float(row[f"quat{i}"]) for i in range(4)])
    actual = Rotation.from_quat(quaternion[[1, 2, 3, 0]])
    rotation_error = (actual * anchor.inv()).as_rotvec()
    return np.array(
        [float(row[f"base_p{i}"]) for i in range(3)]
        + rotation_error.tolist()
        + [float(row[f"base_v{i}"]) for i in range(3)]
        + [float(row[f"base_w{i}"]) for i in range(3)]
        + [
            float(row["phase27_xi_left"]),
            float(row["phase27_xi_right"]),
            float(row["phase27_dxi_left"]),
            float(row["phase27_dxi_right"]),
        ]
    )


def wrench(row: dict[str, str], kind: str) -> np.ndarray:
    return np.array([float(row[f"phase27_{kind}_wrench{i}"]) for i in range(12)])


def valid_interval(table: dict[int, dict[str, str]], start: int, target: int) -> tuple[bool, str | None]:
    for tick in range(start, target + 1):
        row = table.get(tick)
        if row is None:
            return False, f"missing_tick_{tick}"
        if float(row["dt_s"]) <= 0.0:
            return False, f"nonpositive_dt_tick_{tick}"
        if int(row["latch"]) != 0:
            return False, f"latched_tick_{tick}"
    return True, None


def rollout(
    problem: Any,
    table: dict[int, dict[str, str]],
    discrete: ca.Function,
    config: dict,
    method: dict,
) -> dict[str, Any]:
    anchor = Rotation.from_matrix(problem.rotation)
    parameter = problem.rotation.reshape(-1)
    scale = np.asarray(config["state_error_scale"], dtype=float)
    start = int(problem.tick)
    start_state = state_from_row(table[start], anchor)
    start_error = float(np.max(np.abs(start_state - problem.state)))
    requested_state = start_state.copy()
    realized_state = start_state.copy()
    horizons = set(int(value) for value in method["model_adequacy"]["horizons_ms"])
    maximum_steps = max(horizons) // 20
    samples = {}
    stopped = None
    for step in range(1, maximum_steps + 1):
        tick = start + 2 * (step - 1)
        target_tick = start + 2 * step
        valid, reason = valid_interval(table, tick, target_tick)
        if not valid:
            stopped = reason
            break
        requested = wrench(table[tick], "requested")
        realized = 0.5 * (wrench(table[tick], "realized") + wrench(table[tick + 1], "realized"))
        requested_state = np.asarray(discrete(requested_state, requested, parameter)).reshape(-1)
        realized_state = np.asarray(discrete(realized_state, realized, parameter)).reshape(-1)
        horizon_ms = step * 20
        if horizon_ms not in horizons:
            continue
        actual = state_from_row(table[target_tick], anchor)
        requested_error = requested_state - actual
        realized_error = realized_state - actual
        input_error = requested - realized
        force_indices = [0, 1, 2, 6, 7, 8]
        moment_indices = [3, 4, 5, 9, 10, 11]
        limit = float(method["model_adequacy"]["realized_input_normalized_max_limits"][str(horizon_ms)])
        realized_max = float(np.max(np.abs(realized_error / scale)))
        samples[str(horizon_ms)] = {
            "target_tick": target_tick,
            "actual_state": actual,
            "requested_input_model_state": requested_state.copy(),
            "realized_input_model_state": realized_state.copy(),
            "requested_input_error": requested_error,
            "realized_input_error": realized_error,
            "requested_normalized_max_abs": float(np.max(np.abs(requested_error / scale))),
            "realized_normalized_max_abs": realized_max,
            "realized_limit": limit,
            "realized_model_pass": realized_max <= limit,
            "last_interval_requested_minus_realized_force_max_n": float(np.max(np.abs(input_error[force_indices]))),
            "last_interval_requested_minus_realized_moment_max_nm": float(np.max(np.abs(input_error[moment_indices]))),
            "requested_groups": REF.grouped(requested_error, scale, method["state_groups"]),
            "realized_groups": REF.grouped(realized_error, scale, method["state_groups"]),
        }
    missing = sorted(horizons - {int(value) for value in samples})
    return {
        "start_tick": start,
        "start_state_max_abs_error": start_error,
        "samples": samples,
        "missing_horizons_ms": missing,
        "stopped_reason": stopped,
        "available_horizons_pass": all(sample["realized_model_pass"] for sample in samples.values()),
        "complete_horizon_set": not missing,
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
    config_path = ROOT / method["source_ocp_config"]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    phase29_path = ROOT / method["source_phase29_method"]
    phase29 = json.loads(phase29_path.read_text(encoding="utf-8"))
    raw_dir = ROOT / method["source_phase28_run"]
    generator_path = ROOT / method["source_generator"]
    discrete = REF.exact_discrete_function(config, generator_path)
    cases = {case["id"]: case for case in phase29["cases"]}
    results = {}
    raw_paths = []
    for case_id in method["case_ids"]:
        raw_path = raw_dir / f"{case_id}_control.csv"
        raw_paths.append(raw_path)
        raw_rows = rows(raw_path)
        table = {int(row["tick"]): row for row in raw_rows}
        sequence = P29.problem_sequence(raw_rows, float(config["equilibrium_state"][2]))
        action_index = next(index for index, problem in enumerate(sequence) if problem.tick == int(cases[case_id]["expected_action_tick"]))
        selected = [sequence[action_index + offset] for offset in method["neighbor_update_offsets"]]
        results[case_id] = [rollout(problem, table, discrete, config, method) for problem in selected]
    start_semantics_pass = all(item["start_state_max_abs_error"] <= 1e-12 for values in results.values() for item in values)
    available_pass = all(item["available_horizons_pass"] for values in results.values() for item in values)
    complete = all(item["complete_horizon_set"] for values in results.values() for item in values)
    summary = {
        "start_semantics_pass": start_semantics_pass,
        "available_horizons_pass": available_pass,
        "complete_horizon_set": complete,
        "classification": (
            "P31-F_local_model_adequacy_failure" if not available_pass
            else "inconclusive_recorded_trace_truncated" if not complete
            else "P31-C_model_adequate_seek_control_architecture"
        ),
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "model_adequacy.json").write_text(json.dumps(clean(results), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [method_path, config_path, phase29_path, generator_path, REFERENCE_SCRIPT, Path(__file__).resolve(), *raw_paths]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {
            "numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__,
            "casadi": ca.__version__, "acados_template": importlib.metadata.version("acados_template")
        },
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if start_semantics_pass and available_pass and complete else 2


if __name__ == "__main__":
    raise SystemExit(main())
