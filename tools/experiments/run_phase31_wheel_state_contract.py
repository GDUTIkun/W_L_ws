#!/usr/bin/env python3
"""Phase 31 Gates 0-3: direct MuJoCo wheel-state contract audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
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


def required_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"missing {kind.name} {name}")
    return value


class Geometry:
    def __init__(self, model: mujoco.MjModel, contract: dict):
        self.model = model
        self.data = mujoco.MjData(model)
        self.base_site = required_id(model, mujoco.mjtObj.mjOBJ_SITE, contract["base_site"])
        self.wheel_bodies = [
            required_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in contract["wheel_bodies"]
        ]

    def set_row(self, row: dict[str, str]) -> None:
        self.set_state(
            np.array([float(row[f"qpos{i}"]) for i in range(self.model.nq)]),
            np.array([float(row[f"qvel{i}"]) for i in range(self.model.nv)]),
            float(row["time_s"]),
        )

    def set_state(self, qpos: np.ndarray, qvel: np.ndarray, time_s: float = 0.0) -> None:
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.time = time_s
        mujoco.mj_forward(self.model, self.data)

    def value(self, row: dict[str, str]) -> dict[str, np.ndarray]:
        self.set_row(row)
        return self.current_value()

    def current_value(self) -> dict[str, np.ndarray]:
        rotation = self.data.site_xmat[self.base_site].reshape(3, 3)
        base_position = self.data.site_xpos[self.base_site].copy()
        jacp_base = np.zeros((3, self.model.nv))
        jacr_base = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jacp_base, jacr_base, self.base_site)
        base_velocity = jacp_base @ self.data.qvel
        base_omega = jacr_base @ self.data.qvel
        position = np.zeros(2)
        velocity = np.zeros(2)
        for side, body in enumerate(self.wheel_bodies):
            wheel_position = self.data.xpos[body].copy()
            jacp_wheel = np.zeros((3, self.model.nv))
            jacr_unused = np.zeros((3, self.model.nv))
            mujoco.mj_jacBody(self.model, self.data, jacp_wheel, jacr_unused, body)
            wheel_velocity = jacp_wheel @ self.data.qvel
            relative_b = rotation.T @ (wheel_position - base_position)
            omega_b = rotation.T @ base_omega
            relative_velocity_b = rotation.T @ (wheel_velocity - base_velocity) - np.cross(omega_b, relative_b)
            position[side] = relative_b[0]
            velocity[side] = relative_velocity_b[0]
        return {
            "position": position,
            "velocity": velocity,
            "base_position": base_position,
            "base_velocity": base_velocity,
            "base_omega": base_omega,
            "base_rotation": rotation,
        }


def grouped(values: np.ndarray) -> dict[str, float]:
    return {
        "left": float(values[0]),
        "right": float(values[1]),
        "common": float(0.5 * (values[0] + values[1])),
        "differential": float(0.5 * (values[1] - values[0])),
    }


def signs_match(left: np.ndarray, right: np.ndarray, deadband: float) -> bool:
    active = (np.abs(left) > deadband) & (np.abs(right) > deadband)
    return bool(np.all(np.sign(left[active]) == np.sign(right[active])))


def baseline_phase30(method: dict) -> dict[str, Any]:
    source = ROOT / method["source_phase30_model_run"] / "model_adequacy.json"
    data = json.loads(source.read_text(encoding="utf-8"))
    observed = {}
    values = []
    for case in method["cases"]:
        case_values = []
        by_tick = {int(item["start_tick"]): item for item in data[case["id"]]}
        for tick in case["authority_ticks"]:
            metric = float(by_tick[tick]["samples"]["20"]["realized_normalized_max_abs"])
            case_values.append(metric)
            values.append(metric)
        observed[case["id"]] = case_values
    expected = [
        0.20255077278239703, 0.20647133198381057, 0.22577229364498672,
        0.1296009020123884, 0.15977243812722228, 0.1959647572307204,
    ]
    error = float(np.max(np.abs(np.asarray(values) - np.asarray(expected))))
    return {"observed": observed, "expected": expected, "max_abs_error": error}


def sample_at_tick(
    tick: int,
    control: dict[int, dict[str, str]],
    plant: dict[tuple[int, int], dict[str, str]],
    geometry: Geometry,
    method: dict,
) -> dict[str, Any]:
    current = control[tick]
    center_row = plant[(tick - 1, 4)]
    minus_row = plant[(tick - 1, 3)]
    plus_row = plant[(tick, 0)]
    center = geometry.value(center_row)
    minus = geometry.value(minus_row)
    plus = geometry.value(plus_row)
    adapter_position = np.array([float(current["phase27_xi_left"]), float(current["phase27_xi_right"])])
    adapter_velocity = np.array([float(current["phase27_dxi_left"]), float(current["phase27_dxi_right"])])
    dt = float(plus_row["time_s"]) - float(minus_row["time_s"])
    fd_velocity = (plus["position"] - minus["position"]) / dt
    position_error = adapter_position - center["position"]
    adapter_velocity_error = adapter_velocity - center["velocity"]
    fd_velocity_error = fd_velocity - center["velocity"]
    logged_base = np.array([float(current[f"base_p{i}"]) for i in range(3)])
    logged_velocity = np.array([float(current[f"base_v{i}"]) for i in range(3)])
    logged_omega = np.array([float(current[f"base_w{i}"]) for i in range(3)])
    alignment = max(
        float(np.max(np.abs(logged_base - center["base_position"]))),
        float(np.max(np.abs(logged_velocity - center["base_velocity"]))),
        float(np.max(np.abs(logged_omega - center["base_omega"]))),
        abs(float(current["pre_step_plant_time_s"]) - float(center_row["time_s"])),
    )
    deadband = float(method["gates"]["sign_deadband"])
    return {
        "tick": tick,
        "time_s": float(center_row["time_s"]),
        "safety": {"latch": int(current["latch"]), "dt_s": float(current["dt_s"])},
        "alignment_max_abs_error": alignment,
        "fd_dt_s": dt,
        "adapter_position": grouped(adapter_position),
        "geometry_position": grouped(center["position"]),
        "position_error": grouped(position_error),
        "adapter_velocity": grouped(adapter_velocity),
        "kinematic_velocity": grouped(center["velocity"]),
        "fd_velocity": grouped(fd_velocity),
        "adapter_minus_kinematic_velocity": grouped(adapter_velocity_error),
        "fd_minus_kinematic_velocity": grouped(fd_velocity_error),
        "position_max_abs_error_m": float(np.max(np.abs(position_error))),
        "adapter_velocity_max_abs_error_m_s": float(np.max(np.abs(adapter_velocity_error))),
        "fd_velocity_max_abs_error_m_s": float(np.max(np.abs(fd_velocity_error))),
        "position_sign_parity": signs_match(adapter_position, center["position"], deadband),
        "velocity_sign_parity": signs_match(adapter_velocity, center["velocity"], deadband),
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
    scene = ROOT / method["scene"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    geometry = Geometry(model, method["body_site_contract"])
    raw_root = ROOT / method["source_phase28_run"]
    baseline = baseline_phase30(method)
    results = {}
    raw_paths = []
    for case in method["cases"]:
        control_path = raw_root / f"{case['id']}_control.csv"
        plant_path = raw_root / f"{case['id']}_plant.csv"
        raw_paths.extend([control_path, plant_path])
        control_rows = read_csv(control_path)
        plant_rows = read_csv(plant_path)
        control = {int(row["tick"]): row for row in control_rows}
        plant = {(int(row["control_tick"]), int(row["physics_substep"])): row for row in plant_rows}
        results[case["id"]] = [
            sample_at_tick(tick, control, plant, geometry, method)
            for tick in case["authority_ticks"]
        ]
    samples = [item for values in results.values() for item in values]
    gates = method["gates"]
    summary = {
        "gate0_phase30_baseline_pass": baseline["max_abs_error"] <= float(gates["phase30_metric_max_abs_error"]),
        "alignment_pass": max(item["alignment_max_abs_error"] for item in samples) <= float(gates["control_vs_plant_state_max_abs_error"]),
        "position_contract_pass": bool(
            max(item["position_max_abs_error_m"] for item in samples) <= float(gates["position_contract_max_abs_error_m"])
            and all(item["position_sign_parity"] for item in samples)
        ),
        "velocity_adapter_vs_kin_pass": bool(
            max(item["adapter_velocity_max_abs_error_m_s"] for item in samples) <= float(gates["velocity_adapter_vs_kin_max_abs_error_m_s"])
            and all(item["velocity_sign_parity"] for item in samples)
        ),
        "velocity_fd_vs_kin_pass": max(item["fd_velocity_max_abs_error_m_s"] for item in samples) <= float(gates["velocity_fd_vs_kin_max_abs_error_m_s"]),
        "maxima": {
            "alignment": max(item["alignment_max_abs_error"] for item in samples),
            "position_error_m": max(item["position_max_abs_error_m"] for item in samples),
            "adapter_vs_kin_velocity_error_m_s": max(item["adapter_velocity_max_abs_error_m_s"] for item in samples),
            "fd_vs_kin_velocity_error_m_s": max(item["fd_velocity_max_abs_error_m_s"] for item in samples),
        },
        "production_modified": False,
    }
    if not summary["gate0_phase30_baseline_pass"] or not summary["alignment_pass"]:
        classification = "authority_alignment_failure"
    elif not summary["position_contract_pass"]:
        classification = "P31-A_wheel_position_semantics_mismatch"
    elif not summary["velocity_adapter_vs_kin_pass"] or not summary["velocity_fd_vs_kin_pass"]:
        classification = "P31-B_wheel_rate_measurement_semantics_mismatch"
    else:
        classification = "measurement_contract_pass_proceed_to_dynamics"
    summary["classification"] = classification
    output.mkdir(parents=True)
    (output / "wheel_state_contract.json").write_text(
        json.dumps(clean({"baseline": baseline, "ids": {"base_site": geometry.base_site, "wheel_bodies": geometry.wheel_bodies}, "results": results}), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    input_paths = [method_path, scene, ROOT / "ros_ws/src/wheel_leg_mujoco/src/adapter.cpp", ROOT / "ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp", ROOT / "ros_ws/src/wheel_leg_core/src/controller_core.cpp", Path(__file__).resolve(), *raw_paths]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {"numpy": np.__version__, "mujoco": mujoco.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if classification == "measurement_contract_pass_proceed_to_dynamics" else 2


if __name__ == "__main__":
    raise SystemExit(main())
