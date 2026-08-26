#!/usr/bin/env python3
"""Run the Phase-19 v3 sampled-leg attribution and nonlinear pre-freeze gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from run_mujoco_planar_prefreeze import Plant, lqr

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase19_planar_prefreeze_v3.json"
DEFAULT_SCENE = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-model/phase19_planar_scene.xml"
DEFAULT_EQUILIBRIUM = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-equilibrium/equilibrium.json"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-planar-prefreeze-v3"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def local_model(plant: Plant, scale: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    finite = plant.config["finite_difference"]
    origin = np.zeros(4)
    affine = plant.reduced_tick(origin, 0.0)
    a = np.empty((4, 4))
    for index, base_epsilon in enumerate(finite["reduced_state"]):
        epsilon = scale * float(base_epsilon)
        plus, minus = origin.copy(), origin.copy()
        plus[index], minus[index] = epsilon, -epsilon
        a[:, index] = (
            plant.reduced_tick(plus, 0.0) - plant.reduced_tick(minus, 0.0)
        ) / (2.0 * epsilon)
    epsilon = scale * float(finite["wheel_torque_nm"])
    b_native = (
        plant.reduced_tick(origin, epsilon) - plant.reduced_tick(origin, -epsilon)
    ) / (2.0 * epsilon)
    return a, b_native, -b_native, affine


def model_evidence(plant: Plant) -> tuple[dict[str, Any], np.ndarray]:
    scales = [float(value) for value in plant.config["finite_difference"]["convergence_scales"]]
    models = []
    nominal_a = nominal_b = nominal_affine = None
    for scale in scales:
        a, b_native, b_canonical, affine = local_model(plant, scale)
        models.append((scale, a, b_native, b_canonical, affine))
        if scale == 1.0:
            nominal_a, nominal_b, nominal_affine = a, b_canonical, affine
    if nominal_a is None or nominal_b is None or nominal_affine is None:
        raise RuntimeError("finite_difference.convergence_scales must contain 1.0")
    gain = lqr(nominal_a, nominal_b, plant.config["lqr"])
    controllability = np.column_stack([
        np.linalg.matrix_power(nominal_a, power) @ nominal_b for power in range(4)
    ])
    poles = np.linalg.eigvals(nominal_a - np.outer(nominal_b, gain))
    convergence = []
    maximum_difference = 0.0
    for scale, a, b_native, _, affine in models:
        difference = max(
            float(np.max(np.abs(a - nominal_a))),
            float(np.max(np.abs(-b_native - nominal_b))),
        )
        maximum_difference = max(maximum_difference, difference)
        convergence.append({
            "scale": scale,
            "maximum_matrix_difference_from_scale_1": difference,
            "maximum_abs_affine_drift": float(np.max(np.abs(affine))),
        })
    return ({
        "A": nominal_a.tolist(),
        "B_canonical": nominal_b.tolist(),
        "affine_drift": nominal_affine.tolist(),
        "gain_canonical_tau_equals_minus_Kx": gain.tolist(),
        "controllability_rank": int(np.linalg.matrix_rank(controllability)),
        "closed_loop_poles": [[float(value.real), float(value.imag)] for value in poles],
        "spectral_radius": float(np.max(np.abs(poles))),
        "finite_difference_convergence": convergence,
        "maximum_matrix_convergence_difference": maximum_difference,
    }, gain)


def raw_coordinate_diagnostic(plant: Plant, gain: np.ndarray) -> dict[str, Any]:
    """Show why unconstrained qpos/qvel finite differences are not a release gate."""
    dimension = plant.model.nq + plant.model.nv
    zero = np.zeros(dimension)
    results = []
    for qpos_epsilon, qvel_epsilon in plant.config["finite_difference"]["raw_full_coordinate_steps"]:
        matrix = np.empty((dimension, dimension))
        for index in range(dimension):
            epsilon = float(qpos_epsilon if index < plant.model.nq else qvel_epsilon)
            plus, minus = zero.copy(), zero.copy()
            plus[index], minus[index] = epsilon, -epsilon
            matrix[:, index] = (
                plant.full_tick(plus, gain) - plant.full_tick(minus, gain)
            ) / (2.0 * epsilon)
        poles = np.linalg.eigvals(matrix)
        results.append({
            "qpos_epsilon": float(qpos_epsilon),
            "qvel_epsilon": float(qvel_epsilon),
            "spectral_radius": float(np.max(np.abs(poles))),
            "unstable_pole_count": int(np.sum(np.abs(poles) > 1.0 + 1e-6)),
        })
    radii = [item["spectral_radius"] for item in results]
    return {
        "authoritative_release_gate": False,
        "reason": "Raw generalized-coordinate perturbations leave the equality/contact constraint manifold.",
        "step_sweep": results,
        "spectral_radius_range": [min(radii), max(radii)],
    }


def run_case(plant: Plant, gain: np.ndarray, initial_state: list[float], duration_s: float) -> dict[str, Any]:
    data = plant.reset(np.asarray(initial_state, dtype=float))
    height_reference = float(data.site_xpos[plant.site, 2])
    ticks = int(round(duration_s / float(plant.config["control_period_s"])))
    metrics = {
        "maximum_abs_pitch_rad": 0.0,
        "maximum_abs_x_m": 0.0,
        "maximum_leg_error_rad": 0.0,
        "maximum_height_error_m": 0.0,
        "maximum_wheel_torque_nm": 0.0,
        "maximum_leg_torque_nm": 0.0,
    }
    bilateral = 0
    completed = 0
    for _ in range(ticks):
        state = plant.observe(data)
        wheel = float(np.clip(
            gain @ state,
            -float(plant.config["wheel_torque_limit_nm"]),
            float(plant.config["wheel_torque_limit_nm"]),
        ))
        plant.step_tick(data, wheel)
        state = plant.observe(data)
        metrics["maximum_abs_pitch_rad"] = max(metrics["maximum_abs_pitch_rad"], abs(float(state[2])))
        metrics["maximum_abs_x_m"] = max(metrics["maximum_abs_x_m"], abs(float(state[0])))
        metrics["maximum_leg_error_rad"] = max(
            metrics["maximum_leg_error_rad"],
            float(np.max(np.abs(data.qpos[plant.active_qpos] - plant.reference))),
        )
        metrics["maximum_height_error_m"] = max(
            metrics["maximum_height_error_m"],
            abs(float(data.site_xpos[plant.site, 2]) - height_reference),
        )
        metrics["maximum_wheel_torque_nm"] = max(metrics["maximum_wheel_torque_nm"], abs(wheel))
        metrics["maximum_leg_torque_nm"] = max(
            metrics["maximum_leg_torque_nm"],
            float(np.max(np.abs(data.ctrl[plant.active_actuators]))),
        )
        bilateral += plant.bilateral_contact(data)
        completed += 1
        if not np.all(np.isfinite(data.qpos)) or abs(float(state[2])) > 0.55:
            break
    final_state = plant.observe(data)
    return {
        "completed_ticks": completed,
        "requested_ticks": ticks,
        "bilateral_contact_fraction": bilateral / completed,
        "final_state": final_state.tolist(),
        "finite": bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))),
        **metrics,
    }


def run(config: dict[str, Any], scene: Path, equilibrium: np.ndarray) -> dict[str, Any]:
    plant = Plant(scene, equilibrium, config)
    model, gain = model_evidence(plant)
    raw_diagnostic = raw_coordinate_diagnostic(plant, gain)
    cases = []
    for case in config["cases"]:
        cases.append({
            "id": case["id"],
            **run_case(plant, gain, case["initial_state"], float(config["duration_s"])),
        })

    rejected_config = json.loads(json.dumps(config))
    rejected_config["leg_kp_nm_per_rad"] = float(config["diagnostic_rejected_leg_kp_nm_per_rad"])
    rejected_config["leg_kd_nm_s_per_rad"] = float(config["diagnostic_rejected_leg_kd_nm_s_per_rad"])
    rejected = run_case(
        Plant(scene, equilibrium, rejected_config), gain, [0.0, 0.0, 0.0, 0.0],
        float(config["diagnostic_rejected_duration_s"]),
    )

    gates = config["gates"]
    checks = {
        "reduced_controllability": model["controllability_rank"] == int(gates["reduced_controllability_rank"]),
        "reduced_stability": model["spectral_radius"] <= float(gates["reduced_spectral_radius_max"]),
        "reduced_fd_convergence": model["maximum_matrix_convergence_difference"] <= float(gates["reduced_matrix_convergence_max_abs"]),
        "rejected_sampled_leg_gain_reproduces_failure": rejected["maximum_abs_pitch_rad"] > float(gates["rejected_gain_must_fail_pitch_rad"]),
        "nonlinear_full_plant_envelope": all(
            case["completed_ticks"] == case["requested_ticks"]
            and case["finite"]
            and case["maximum_abs_pitch_rad"] <= float(gates["maximum_pitch_rad"])
            and case["maximum_abs_x_m"] <= float(gates["maximum_abs_x_m"])
            and case["maximum_leg_error_rad"] <= float(gates["maximum_leg_error_rad"])
            and case["maximum_height_error_m"] <= float(gates["maximum_height_error_m"])
            and case["bilateral_contact_fraction"] >= float(gates["minimum_bilateral_contact_fraction"])
            and abs(case["final_state"][0]) <= float(gates["maximum_final_abs_x_m"])
            and abs(case["final_state"][1]) <= float(gates["maximum_final_abs_dx_m_s"])
            and abs(case["final_state"][2]) <= float(gates["maximum_final_abs_pitch_rad"])
            and abs(case["final_state"][3]) <= float(gates["maximum_final_abs_pitch_rate_rad_s"])
            for case in cases
        ),
    }
    return {
        "pass": all(checks.values()),
        "decision": "ALLOW_CORE_IMPLEMENTATION" if all(checks.values()) else "REWORK",
        "checks": checks,
        "local_model": model,
        "raw_full_coordinate_diagnostic": raw_diagnostic,
        "sampled_leg_gain_attribution": {
            "rejected_gain": [rejected_config["leg_kp_nm_per_rad"], rejected_config["leg_kd_nm_s_per_rad"]],
            "accepted_candidate_gain": [config["leg_kp_nm_per_rad"], config["leg_kd_nm_s_per_rad"]],
            "rejected_nominal_case": rejected,
        },
        "cases": cases,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--equilibrium", type=Path, default=DEFAULT_EQUILIBRIUM)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    scene, config_path, equilibrium_path = (
        arguments.scene.resolve(), arguments.config.resolve(), arguments.equilibrium.resolve()
    )
    config = json.loads(config_path.read_text())
    equilibrium = np.asarray(json.loads(equilibrium_path.read_text())["candidate"], dtype=float)
    summary = run(config, scene, equilibrium)
    summary_path = output / "summary.json"
    write_json(summary_path, summary)
    write_json(output / "manifest.json", {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "scene": str(scene),
        "scene_sha256": sha256(scene),
        "config": str(config_path),
        "config_sha256": sha256(config_path),
        "equilibrium": str(equilibrium_path),
        "equilibrium_sha256": sha256(equilibrium_path),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "summary_sha256": sha256(summary_path),
    })
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
