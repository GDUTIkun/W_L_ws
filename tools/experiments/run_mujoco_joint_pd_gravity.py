#!/usr/bin/env python3
"""Validate the Phase 17 fixed-base Joint PD and nominal gravity profile."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase17_nominal.json"
DEFAULT_RUNNER = ROOT / "ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/deterministic_loop"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/17-nominal-joint-pd-gravity-compensation/evidence/automated/2026-08-25-formal-v3"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def vector_arg(values: list[float] | np.ndarray) -> str:
    return ",".join(format(float(value), ".17g") for value in values)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def run_case(
    runner: Path,
    scene: Path,
    output: Path,
    config: dict[str, Any],
    *,
    ticks: int,
    episodes: int = 1,
    enable_pd: bool = True,
    enable_gravity: bool = True,
    reference_step: np.ndarray | None = None,
    reference_tick: int = -1,
    disturbance: np.ndarray | None = None,
    disturbance_tick: int = -1,
) -> None:
    controller = config["controller"]
    gravity = config["gravity_profile"]
    command = [
        str(runner), "--model", str(scene), "--output", str(output),
        "--scenario", "control", "--episodes", str(episodes),
        "--ticks", str(ticks), "--physics-steps-per-control",
        str(config["physics_steps_per_control"]), "--enable-pd",
        str(int(enable_pd)), "--enable-gravity", str(int(enable_gravity)),
        "--reference", vector_arg(config["canonical_offset_rad"]),
        "--kp", vector_arg(controller["kp_nm_per_rad"]),
        "--kd", vector_arg(controller["kd_nm_s_per_rad"]),
        "--torque-limit", vector_arg(controller["torque_limit_nm"]),
        "--gravity-offset", vector_arg(config["canonical_offset_rad"]),
        "--gravity-left-sin", vector_arg(gravity["left_sin_torque_nm"]),
        "--gravity-left-cos", vector_arg(gravity["left_cos_torque_nm"]),
        "--gravity-right-sin", vector_arg(gravity["right_sin_torque_nm"]),
        "--gravity-right-cos", vector_arg(gravity["right_cos_torque_nm"]),
    ]
    if reference_step is not None:
        command += ["--reference-tick", str(reference_tick),
                    "--reference-step", vector_arg(reference_step)]
    if disturbance is not None:
        command += ["--disturbance-tick", str(disturbance_tick),
                    "--disturbance", vector_arg(disturbance)]
    subprocess.run(command, cwd=ROOT, check=True)


def load_phase15_module() -> Any:
    path = ROOT / "tools/experiments/run_mujoco_closed_chain_kinematics.py"
    spec = importlib.util.spec_from_file_location("phase15_kinematics", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load Phase 15 kinematics evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def analytic_gravity(
    native: np.ndarray, side: str, profile: dict[str, Any]
) -> np.ndarray:
    result = np.zeros(3)
    sin_values = profile[f"{side}_sin_torque_nm"]
    cos_values = profile[f"{side}_cos_torque_nm"]
    for wave, sin_coefficient, cos_coefficient in zip(
        profile["native_wave_numbers"], sin_values, cos_values, strict=True
    ):
        wave_vector = np.asarray(wave, dtype=float)
        phase = float(wave_vector @ native)
        result += wave_vector * (
            float(sin_coefficient) * math.sin(phase)
            + float(cos_coefficient) * math.cos(phase)
        )
    return result


def compiled_potential(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    return float(np.sum(model.body_mass * data.xipos[:, 2]) * -model.opt.gravity[2])


def validate_gravity(
    model: mujoco.MjModel, config: dict[str, Any]
) -> dict[str, float | int]:
    phase15 = load_phase15_module()
    phase15_config = json.loads(
        (ROOT / config["phase15_config"]).read_text(encoding="utf-8")
    )
    workspace = (
        np.linspace(-0.65, 0.65, 5),
        np.linspace(-0.75, 0.75, 5),
        np.linspace(-1.0, 1.0, 3),
    )
    epsilon = 1.0e-6
    max_bias_error = 0.0
    max_potential_error = 0.0
    wheel_gravity = 0.0
    samples = 0
    for side in ("left", "right"):
        leg = phase15.context(model, side, phase15_config)
        seed = np.zeros(2)
        for hip in workspace[0]:
            for knee in workspace[1]:
                for wheel in workspace[2]:
                    native = np.array([hip, knee, wheel])
                    passive, info, data = phase15.solve_passive(
                        model, leg, native, seed, phase15_config["solver"]
                    )
                    if not info["converged"]:
                        raise RuntimeError("Gravity sweep passive solve failed")
                    seed = passive
                    _, closure_jacobian = phase15.closure_state(model, data, leg)
                    reduction, _ = phase15.reduction(
                        closure_jacobian, phase15_config["solver"]["rcond"]
                    )
                    expected = -(reduction.T @ data.qfrc_bias[leg["dofs"]])
                    actual = analytic_gravity(native, side, config["gravity_profile"])
                    numerical = np.zeros(3)
                    for joint in range(3):
                        potentials = []
                        for delta in (-epsilon, epsilon):
                            probe = native.copy()
                            probe[joint] += delta
                            _, probe_info, probe_data = phase15.solve_passive(
                                model, leg, probe, passive,
                                phase15_config["solver"],
                            )
                            if not probe_info["converged"]:
                                raise RuntimeError("Potential oracle passive solve failed")
                            potentials.append(compiled_potential(model, probe_data))
                        numerical[joint] = -(potentials[1] - potentials[0]) / (2 * epsilon)
                    max_bias_error = max(max_bias_error, float(np.max(np.abs(actual - expected))))
                    max_potential_error = max(
                        max_potential_error, float(np.max(np.abs(actual - numerical)))
                    )
                    wheel_gravity = max(wheel_gravity, abs(float(actual[2])))
                    samples += 1
    return {
        "samples": samples,
        "max_reduced_bias_error_nm": max_bias_error,
        "max_potential_gradient_error_nm": max_potential_error,
        "max_abs_wheel_gravity_nm": wheel_gravity,
    }


def arrays(rows: list[dict[str, str]]) -> tuple[np.ndarray, ...]:
    def values(prefix: str) -> np.ndarray:
        return np.asarray([[float(row[f"{prefix}_{joint}"]) for joint in range(6)] for row in rows])
    return values("q"), values("dq"), values("reference"), values("tau"), values("ctrl")


def finite_and_zoh(rows: list[dict[str, str]]) -> bool:
    ignored = {"scenario", "fault_event"}
    return all(
        math.isfinite(float(value))
        for row in rows for name, value in row.items() if name not in ignored
    ) and all(float(row["zoh_ctrl_max_difference"]) == 0.0 for row in rows)


def settling_time(error: np.ndarray, tolerance: float, period: float) -> float:
    for index in range(len(error)):
        if np.all(np.abs(error[index:]) <= tolerance):
            return index * period
    return math.inf


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--runner", type=Path, default=DEFAULT_RUNNER)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config_path = args.config.resolve()
    runner = args.runner.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    scene = (ROOT / config["scene"]).resolve()
    for required in (runner, scene, ROOT / config["included_model"]):
        if not required.is_file():
            raise SystemExit(f"Required input is missing: {required}")
    if config["control_period_s"] / config["physics_timestep_s"] != config["physics_steps_per_control"]:
        raise SystemExit("Invalid fixed-step ratio")

    model = mujoco.MjModel.from_xml_path(str(scene))
    gravity_metrics = validate_gravity(model, config)
    formal = config["formal_cases"]
    ticks = int(formal["ticks"])
    reference_tick = int(formal["reference_tick"])
    outputs: list[Path] = []
    logged_gravity_error = 0.0
    offsets = np.asarray(config["canonical_offset_rad"])

    def execute(name: str, **kwargs: Any) -> list[dict[str, str]]:
        nonlocal logged_gravity_error
        path = output_dir / f"{name}.csv"
        run_case(runner, scene, path, config, **kwargs)
        outputs.append(path)
        rows = read_rows(path)
        if kwargs.get("enable_gravity", True):
            for row in rows:
                position = np.asarray([float(row[f"q_{joint}"]) for joint in range(6)])
                logged = np.asarray([float(row[f"tau_gravity_{joint}"]) for joint in range(6)])
                expected = np.r_[
                    analytic_gravity(offsets[:3] - position[:3], "left", config["gravity_profile"]),
                    analytic_gravity(offsets[3:] - position[3:], "right", config["gravity_profile"]),
                ]
                logged_gravity_error = max(
                    logged_gravity_error, float(np.max(np.abs(logged - expected)))
                )
        return rows

    zero_rows = execute("zero_short", ticks=30, enable_pd=False, enable_gravity=False)
    gravity_rows = execute("gravity_short", ticks=30, enable_pd=False, enable_gravity=True)
    pd_rows = execute("pd_only_hold", ticks=ticks, enable_gravity=False)
    hold_rows = execute("pd_gravity_hold", ticks=ticks)

    checks: dict[str, bool] = {}
    metrics: dict[str, Any] = {"gravity": gravity_metrics, "steps": {}, "disturbances": {}}
    thresholds = config["thresholds"]
    checks["gravity_matches_reduced_bias"] = gravity_metrics["max_reduced_bias_error_nm"] <= thresholds["gravity_oracle_max_error_nm"]
    checks["gravity_matches_potential_gradient"] = gravity_metrics["max_potential_gradient_error_nm"] <= thresholds["gravity_potential_max_error_nm"]
    checks["wheel_com_eccentricity_retained"] = gravity_metrics["max_abs_wheel_gravity_nm"] > 1.0e-5

    zero_q, *_ = arrays(zero_rows)
    gravity_q, *_ = arrays(gravity_rows)
    pd_q, *_ = arrays(pd_rows)
    hold_q, hold_dq, _, _, _ = arrays(hold_rows)
    zero_drift = float(np.max(np.abs(zero_q[-1] - offsets)))
    gravity_drift = float(np.max(np.abs(gravity_q[-1] - offsets)))
    pd_error = float(np.max(np.abs(pd_q[-1] - offsets)))
    hold_error = float(np.max(np.abs(hold_q[-1] - offsets)))
    metrics["holds"] = {
        "zero_short_final_drift_rad": zero_drift,
        "gravity_short_final_drift_rad": gravity_drift,
        "pd_only_final_error_rad": pd_error,
        "pd_gravity_final_error_rad": hold_error,
        "pd_gravity_max_velocity_rad_s": float(np.max(np.abs(hold_dq))),
    }
    checks["gravity_short_improves_zero"] = gravity_drift < zero_drift
    checks["pd_gravity_improves_pd_only"] = hold_error < pd_error
    checks["pd_gravity_hold_bounded"] = hold_error <= thresholds["hold_error_rad"]

    amplitudes = formal["step_amplitude_rad"]
    all_step_pass = True
    for joint in range(6):
        amplitude = float(amplitudes[joint % 3])
        for direction in (-1.0, 1.0):
            step = np.zeros(6)
            step[joint] = direction * amplitude
            name = f"step_j{joint}_{'pos' if direction > 0 else 'neg'}"
            rows = execute(name, ticks=ticks, reference_step=step, reference_tick=reference_tick)
            q, dq, reference, _, _ = arrays(rows)
            post_error = reference[reference_tick:, joint] - q[reference_tick:, joint]
            tolerance = thresholds["final_error_wheel_rad"] if joint % 3 == 2 else thresholds["final_error_hip_knee_rad"]
            final_error = abs(float(post_error[-1]))
            settle = settling_time(post_error, tolerance, config["control_period_s"])
            progress = direction * (q[reference_tick:, joint] - offsets[joint])
            overshoot = max(0.0, (float(np.max(progress)) - amplitude) / amplitude)
            max_velocity = float(np.max(np.abs(dq[:, joint])))
            velocity_limit = thresholds["max_velocity_wheel_rad_s"] if joint % 3 == 2 else thresholds["max_velocity_hip_knee_rad_s"]
            passed = final_error <= tolerance and settle <= thresholds["settling_time_s"] and overshoot <= thresholds["overshoot_ratio"] and max_velocity <= velocity_limit and finite_and_zoh(rows)
            all_step_pass &= passed
            metrics["steps"][name] = {"pass": passed, "final_error_rad": final_error, "settling_time_s": settle, "overshoot_ratio": overshoot, "max_velocity_rad_s": max_velocity}
    checks["all_single_joint_steps"] = all_step_pass

    symmetry_pass = True
    for joint_class in range(3):
        step = np.zeros(6)
        step[joint_class] = amplitudes[joint_class]
        step[joint_class + 3] = amplitudes[joint_class]
        rows = execute(f"symmetric_{joint_class}", ticks=ticks, reference_step=step, reference_tick=reference_tick)
        q, *_ = arrays(rows)
        symmetry_error = float(np.max(np.abs((q[:, joint_class] - offsets[joint_class]) - (q[:, joint_class + 3] - offsets[joint_class + 3]))))
        symmetry_pass &= symmetry_error <= thresholds["symmetry_error_rad"] and finite_and_zoh(rows)
        metrics.setdefault("symmetry", {})[str(joint_class)] = symmetry_error
    checks["left_right_symmetry"] = symmetry_pass

    disturbance_pass = True
    for joint_class in range(3):
        disturbance = np.zeros(6)
        disturbance[joint_class] = formal["disturbance_torque_nm"][joint_class]
        disturbance[joint_class + 3] = formal["disturbance_torque_nm"][joint_class]
        rows = execute(f"disturbance_{joint_class}", ticks=ticks, disturbance=disturbance, disturbance_tick=formal["disturbance_tick"])
        q, *_ = arrays(rows)
        recovery = float(np.max(np.abs(q[-1] - offsets)))
        passed = recovery <= thresholds["recovery_error_rad"] and finite_and_zoh(rows)
        disturbance_pass &= passed
        metrics["disturbances"][str(joint_class)] = {"pass": passed, "final_recovery_error_rad": recovery}
    checks["all_disturbance_recovery"] = disturbance_pass

    saturation_step = np.zeros(6)
    saturation_step[0] = 1.0
    saturation_rows = execute("saturation", ticks=120, reference_step=saturation_step, reference_tick=20)
    saturation_flags = [float(row[f"saturated_{joint}"]) for row in saturation_rows for joint in range(6)]
    _, _, _, saturation_tau, _ = arrays(saturation_rows)
    limits = np.asarray(config["controller"]["torque_limit_nm"])
    checks["post_sum_saturation"] = max(saturation_flags) == 1.0 and np.all(np.abs(saturation_tau) <= limits + 1e-15) and finite_and_zoh(saturation_rows)

    replay_step = np.zeros(6)
    replay_step[1] = 0.1
    replay_a = execute("replay_a", ticks=250, episodes=2, reference_step=replay_step, reference_tick=40)
    replay_b = execute("replay_b", ticks=250, episodes=2, reference_step=replay_step, reference_tick=40)
    normalized_a = [{**row, "episode": "0"} for row in replay_a[:250]]
    normalized_episode = [{**row, "episode": "0"} for row in replay_a[250:]]
    checks["reset_replay_exact"] = normalized_a == normalized_episode
    checks["fresh_process_exact"] = (output_dir / "replay_a.csv").read_bytes() == (output_dir / "replay_b.csv").read_bytes()
    checks["all_logs_finite_and_zoh"] = all(finite_and_zoh(read_rows(path)) for path in outputs)
    checks["cpp_gravity_matches_profile"] = logged_gravity_error <= thresholds["gravity_oracle_max_error_nm"]
    metrics["gravity"]["max_cpp_profile_error_nm"] = logged_gravity_error

    input_paths = [config_path, scene, ROOT / config["included_model"], ROOT / config["phase15_config"], ROOT / "ros_ws/src/wheel_leg_core/include/wheel_leg_core/controller_core.hpp", ROOT / "ros_ws/src/wheel_leg_core/src/controller_core.cpp", ROOT / "ros_ws/src/wheel_leg_mujoco/src/deterministic_loop.cpp", Path(__file__).resolve(), runner]
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "profile": config["profile"],
        "model_revision": config["model_revision"],
        "hardware_data_used": False,
        "versions": {"mujoco": mujoco.__version__, "python": platform.python_version(), "runner": subprocess.run([str(runner), "--version"], check=True, text=True, capture_output=True).stdout.strip(), "ros_distro": os.environ.get("ROS_DISTRO", "unknown")},
        "timing": {name: config[name] for name in ("physics_timestep_s", "control_period_s", "physics_steps_per_control")},
        "inputs": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path): sha256(path) for path in input_paths},
        "outputs": {path.name: sha256(path) for path in outputs},
    }
    summary = {
        "schema_version": 1,
        "overall_pass": all(checks.values()),
        "hardware_data_used": False,
        "checks": checks,
        "metrics": metrics,
        "interpretation_limit": "Simulation-only current nominal fixed-base/contact-disabled Joint PD and gravity evidence; no contact, standing, actuator, realtime, or real-hardware claim.",
    }
    (output_dir / "run_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output_dir / "phase17_validation.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
