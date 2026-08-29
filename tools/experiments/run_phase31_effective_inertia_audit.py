#!/usr/bin/env python3
"""Phase 31 Gates 7-9: residual, scalar-inertia, and structural audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase31_wheel_state_contract_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def pitch_from_quaternion(row: dict[str, str]) -> float:
    w, x, y, z = (float(row[f"quat{index}"]) for index in range(4))
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def correlation(x: list[float], y: list[float]) -> float | None:
    if len(x) < 2 or np.std(x) <= 1e-14 or np.std(y) <= 1e-14:
        return None
    return float(np.corrcoef(x, y)[0, 1])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--input-response", type=Path, required=True)
    parser.add_argument("--acceleration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    method_path = args.method.resolve()
    method = json.loads(method_path.read_text(encoding="utf-8"))
    response_path = args.input_response.resolve()
    acceleration_path = args.acceleration.resolve()
    response = json.loads(response_path.read_text(encoding="utf-8"))
    acceleration = json.loads(acceleration_path.read_text(encoding="utf-8"))
    ocp_path = ROOT / method["source_ocp_config"]
    ocp = json.loads(ocp_path.read_text(encoding="utf-8"))
    radius = float(ocp["wheel_radius_m"])
    body_mass = float(ocp["body_mass_kg"])
    current_denominator = (
        float(ocp["wheel_mass_kg"]) * radius
        + float(ocp["wheel_axle_inertia_kg_m2"]) / radius
    )
    audit = method["effective_inertia_audit"]
    estimates: list[dict[str, Any]] = []
    common_fx_feasible = []
    component_fractions: list[dict[str, Any]] = []
    for case, rows in response["results"].items():
        for row in rows:
            tick = int(row["tick"])
            for channel, channel_result in row["channels"].items():
                mode = "common" if channel.startswith("common") else "differential"
                gain = float(channel_result["measured"]["grouped"][mode])
                if channel.endswith("ty"):
                    denominator = -1.0 / gain
                elif channel == "differential_fx":
                    denominator = -radius / gain
                else:
                    remainder = -gain - 2.0 / body_mass
                    denominator = radius / remainder if remainder > 0.0 else None
                    common_fx_feasible.append(denominator is not None)
                estimates.append({
                    "case": case, "tick": tick, "channel": channel,
                    "plant_gain": gain, "effective_denominator": denominator,
                })
                components = channel_result["measured"]["acceleration_sensitivity_components"]
                component_fractions.append({
                    "case": case, "tick": tick, "channel": channel,
                    **{
                        name: float(value["grouped"][mode]) / gain
                        for name, value in components.items()
                    },
                })
    positive = [row for row in estimates if row["effective_denominator"] is not None]
    by_channel: dict[str, dict[str, float]] = {}
    for channel in ("common_ty", "differential_fx", "differential_ty"):
        values = np.array([
            row["effective_denominator"] for row in positive if row["channel"] == channel
        ], dtype=float)
        by_channel[channel] = {
            "minimum": float(np.min(values)), "maximum": float(np.max(values)),
            "mean": float(np.mean(values)),
            "coefficient_of_variation": float(np.std(values) / np.mean(values)),
            "relative_to_current_mean": float(np.mean(values) / current_denominator),
        }
    channel_means = [value["mean"] for value in by_channel.values()]
    cross_channel_ratio = max(channel_means) / min(channel_means)

    raw_root = ROOT / method["source_phase28_run"]
    samples: list[dict[str, float | str | int]] = []
    raw_paths: list[Path] = []
    for case_spec in method["cases"]:
        case = case_spec["id"]
        control_path = raw_root / f"{case}_control.csv"
        plant_path = raw_root / f"{case}_plant.csv"
        raw_paths.extend([control_path, plant_path])
        controls = {int(row["tick"]): row for row in read_csv(control_path)}
        plants = {
            (int(row["control_tick"]), int(row["physics_substep"])): row
            for row in read_csv(plant_path)
        }
        acceleration_rows = {int(row["tick"]): row for row in acceleration[case]}
        for tick in case_spec["authority_ticks"]:
            control = controls[tick]
            plant = plants[(tick - 1, 4)]
            residual = acceleration_rows[tick]["plant_minus_realized_eq12"]
            samples.append({
                "case": case, "tick": tick,
                "residual_common": float(residual["common"]),
                "residual_differential": float(residual["differential"]),
                "pitch": pitch_from_quaternion(control),
                "pitch_rate": float(control["base_w1"]),
                "base_vx": float(control["base_v0"]),
                "base_ax": float(plant["base_control_linear_acceleration_n0"]),
                "base_pitch_acceleration": float(plant["base_control_angular_acceleration_n1"]),
                "leg_q_norm": float(np.linalg.norm([
                    float(control[f"q{index}"]) for index in (0, 1, 3, 4)
                ])),
                "leg_dq_norm": float(np.linalg.norm([
                    float(control[f"dq{index}"]) for index in (0, 1, 3, 4)
                ])),
                "wheel_rate_common": 0.5 * (
                    float(control["dq2"]) + float(control["dq5"])
                ),
                "normal_load_common": 0.5 * (
                    float(plant["left_normal_load_n"]) + float(plant["right_normal_load_n"])
                ),
                "normal_load_differential": 0.5 * (
                    float(plant["right_normal_load_n"]) - float(plant["left_normal_load_n"])
                ),
            })
    variables = [
        key for key in samples[0]
        if key not in {"case", "tick", "residual_common", "residual_differential"}
    ]
    correlations = {
        residual_name: {
            variable: correlation(
                [float(row[variable]) for row in samples],
                [float(row[residual_name]) for row in samples],
            )
            for variable in variables
        }
        for residual_name in ("residual_common", "residual_differential")
    }
    scalar_inertia_rejected = (
        not any(common_fx_feasible)
        and cross_channel_ratio > float(audit["effective_denominator_cross_channel_ratio_max"])
    )
    state_dependence_supported = max(
        value["coefficient_of_variation"] for value in by_channel.values()
    ) > float(audit["within_channel_coefficient_of_variation_max"])
    classification = (
        "P31-E_missing_wheel_kinematic_dynamic_coupling"
        if scalar_inertia_rejected and not state_dependence_supported
        else "P31-C_configuration_dependent_effective_inertia"
        if state_dependence_supported
        else "unresolved"
    )
    report = {
        "current_denominator": current_denominator,
        "effective_denominator_estimates": estimates,
        "effective_denominator_by_channel": by_channel,
        "effective_denominator_cross_channel_ratio": cross_channel_ratio,
        "common_fx_positive_scalar_feasible": any(common_fx_feasible),
        "component_fractions": component_fractions,
        "closed_loop_samples": samples,
        "correlations": correlations,
        "correlation_interpretation": (
            f"descriptive_only_n={len(samples)}; controlled input response and exact "
            "acceleration decomposition, not correlation, carry the causal decision"
        ),
    }
    summary = {
        "scalar_effective_inertia_rejected": scalar_inertia_rejected,
        "configuration_dependence_supported": state_dependence_supported,
        "classification": classification,
        "root_cause_model_class": "M4" if classification.startswith("P31-E") else "unresolved",
        "candidate_authorized": classification.startswith("P31-E"),
        "production_modified": False,
    }
    output.mkdir(parents=True)
    (output / "effective_inertia_audit.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    inputs = [method_path, response_path, acceleration_path, ocp_path,
              Path(__file__).resolve(), *raw_paths]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": args.replay_of,
        "python": platform.python_version(), "dependencies": {"numpy": np.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if classification.startswith("P31-E") else 2


if __name__ == "__main__":
    raise SystemExit(main())
