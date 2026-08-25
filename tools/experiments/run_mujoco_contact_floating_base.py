#!/usr/bin/env python3
"""Phase 18 nominal wheel-contact and zero-command floating-base validation."""

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

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase18_nominal.json"
DEFAULT_OUTPUT = ROOT / "docs/workflow/phases/18-mujoco-contact-floating-base-plant-validation/evidence/automated/2026-08-25-formal-v5"
SIDES = ("left", "right")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"Required MuJoCo object is missing: {name}")
    return value


def name(model: mujoco.MjModel, kind: mujoco.mjtObj, value: int) -> str:
    return mujoco.mj_id2name(model, kind, value) or f"unnamed_{value}"


def body_velocity(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> np.ndarray:
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(
        model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, velocity, 0
    )
    return velocity


def contact_summary(
    model: mujoco.MjModel, data: mujoco.MjData
) -> tuple[dict[str, dict[str, Any]], set[tuple[str, str]]]:
    floor = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    wheels = {
        side: object_id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_wheel_collision")
        for side in SIDES
    }
    summary = {
        side: {"count": 0, "force": np.zeros(3), "min_dist": math.inf}
        for side in SIDES
    }
    unexpected: set[tuple[str, str]] = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        pair = {contact.geom1, contact.geom2}
        side = next(
            (candidate for candidate, wheel in wheels.items() if pair == {floor, wheel}),
            None,
        )
        if side is None:
            unexpected.add(
                tuple(sorted((
                    name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
                    name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
                )))
            )
            continue
        local = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, local)
        world = np.zeros(3)
        mujoco.mju_mulMatTVec(world, contact.frame.reshape(3, 3), local[:3])
        # mj_contactForce is the force on geom2 by geom1.
        if contact.geom2 != wheels[side]:
            world *= -1.0
        summary[side]["count"] += 1
        summary[side]["force"] += world
        summary[side]["min_dist"] = min(summary[side]["min_dist"], float(contact.dist))
    return summary, unexpected


def configure_friction(model: mujoco.MjModel, coefficient: float) -> None:
    for geom_name in ("floor", "left_wheel_collision", "right_wheel_collision"):
        geom = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        model.geom_friction[geom, 0] = coefficient


def probe_case(
    scene: Path,
    *,
    steps: int,
    settle_steps: int,
    friction: float,
    wheel_radius: float,
    initial_vx: float = 0.0,
    initial_vy: float = 0.0,
    torque: float = 0.0,
) -> tuple[list[dict[str, float | int]], dict[str, float]]:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    configure_friction(model, friction)
    masses: dict[str, float] = {}
    body_ids: dict[str, int] = {}
    wheel_dofs: dict[str, int] = {}
    for side in SIDES:
        carriage = object_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_probe_carriage")
        body_ids[side] = object_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel_body")
        masses[side] = float(model.body_subtreemass[carriage])
        for axis, speed in (("x", initial_vx), ("y", initial_vy)):
            joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_probe_{axis}")
            data.qvel[model.jnt_dofadr[joint]] = speed
        wheel_joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_wheel_joint")
        wheel_dofs[side] = int(model.jnt_dofadr[wheel_joint])
    mujoco.mj_forward(model, data)
    initial_vz = {
        side: float(body_velocity(model, data, body_ids[side])[5]) for side in SIDES
    }
    rows: list[dict[str, float | int]] = []
    unexpected: set[tuple[str, str]] = set()
    impulse = {side: 0.0 for side in SIDES}
    for step in range(steps):
        data.ctrl[:] = torque if step >= settle_steps else 0.0
        mujoco.mj_step(model, data)
        contacts, bad = contact_summary(model, data)
        unexpected |= bad
        row: dict[str, float | int] = {"step": step, "time_s": float(data.time)}
        for side in SIDES:
            velocity = body_velocity(model, data, body_ids[side])
            position = data.xpos[body_ids[side]]
            omega = float(data.qvel[wheel_dofs[side]])
            force = contacts[side]["force"]
            impulse[side] += float(force[2]) * model.opt.timestep
            row.update({
                f"{side}_x_m": float(position[0]),
                f"{side}_y_m": float(position[1]),
                f"{side}_z_m": float(position[2]),
                f"{side}_vx_m_s": float(velocity[3]),
                f"{side}_vy_m_s": float(velocity[4]),
                f"{side}_vz_m_s": float(velocity[5]),
                f"{side}_omega_rad_s": omega,
                f"{side}_slip_x_m_s": float(velocity[3] - wheel_radius * omega),
                f"{side}_contact_count": int(contacts[side]["count"]),
                f"{side}_fx_n": float(force[0]),
                f"{side}_fy_n": float(force[1]),
                f"{side}_fz_n": float(force[2]),
                f"{side}_min_dist_m": float(contacts[side]["min_dist"] if contacts[side]["count"] else 0.0),
            })
        rows.append(row)
    metrics: dict[str, float] = {"unexpected_pairs": float(len(unexpected))}
    duration = steps * model.opt.timestep
    for side in SIDES:
        final_vz = float(rows[-1][f"{side}_vz_m_s"])
        expected_impulse = masses[side] * (final_vz - initial_vz[side] + 9.81 * duration)
        metrics[f"{side}_mass_kg"] = masses[side]
        metrics[f"{side}_vertical_impulse_n_s"] = impulse[side]
        metrics[f"{side}_vertical_impulse_error_n_s"] = abs(impulse[side] - expected_impulse)
    return rows, metrics


def floating_case(scene: Path, steps: int, base_z: float) -> tuple[list[dict[str, float | int]], set[tuple[str, str]]]:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    weld = object_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    freejoint = model.body_jntadr[object_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_body")]
    data.eq_active[weld] = 0
    data.qpos[model.jnt_qposadr[freejoint] + 2] = base_z
    mujoco.mj_forward(model, data)
    base_site = object_id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
    site_pairs = [
        (object_id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_connect2_site"),
         object_id(model, mujoco.mjtObj.mjOBJ_SITE, f"{side}_calf_site"))
        for side in SIDES
    ]
    total_mass = float(np.sum(model.body_mass))
    rows: list[dict[str, float | int]] = []
    unexpected: set[tuple[str, str]] = set()
    for step in range(steps):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        contacts, bad = contact_summary(model, data)
        unexpected |= bad
        quaternion = np.zeros(4)
        mujoco.mju_mat2Quat(quaternion, data.site_xmat[base_site])
        system_com = np.sum(model.body_mass[:, None] * data.xipos, axis=0) / total_mass
        closure = max(float(np.linalg.norm(data.site_xpos[a] - data.site_xpos[b])) for a, b in site_pairs)
        jacobian_position = np.zeros((3, model.nv))
        jacobian_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacobian_position, jacobian_rotation, base_site)
        base_linear = jacobian_position @ data.qvel
        base_angular = jacobian_rotation @ data.qvel
        rows.append({
            "step": step,
            "time_s": float(data.time),
            "system_com_x_m": float(system_com[0]),
            "system_com_y_m": float(system_com[1]),
            "system_com_z_m": float(system_com[2]),
            "base_x_m": float(data.site_xpos[base_site, 0]),
            "base_y_m": float(data.site_xpos[base_site, 1]),
            "base_z_m": float(data.site_xpos[base_site, 2]),
            "base_vx_m_s": float(base_linear[0]),
            "base_vy_m_s": float(base_linear[1]),
            "base_vz_m_s": float(base_linear[2]),
            "base_wx_rad_s": float(base_angular[0]),
            "base_wy_rad_s": float(base_angular[1]),
            "base_wz_rad_s": float(base_angular[2]),
            "quat_w": float(quaternion[0]),
            "quat_x": float(quaternion[1]),
            "quat_y": float(quaternion[2]),
            "quat_z": float(quaternion[3]),
            "closure_residual_m": closure,
            "left_contact_count": int(contacts["left"]["count"]),
            "right_contact_count": int(contacts["right"]["count"]),
            "left_fz_n": float(contacts["left"]["force"][2]),
            "right_fz_n": float(contacts["right"]["force"][2]),
            "left_min_dist_m": float(contacts["left"]["min_dist"] if contacts["left"]["count"] else 0.0),
            "right_min_dist_m": float(contacts["right"]["min_dist"] if contacts["right"]["count"] else 0.0),
        })
    return rows, unexpected


def write_rows(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def finite_rows(rows: list[dict[str, Any]]) -> bool:
    return all(math.isfinite(float(value)) for row in rows for value in row.values())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config_path = args.config.resolve()
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise SystemExit(f"Refusing to overwrite non-empty output directory: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    config = json.loads(config_path.read_text(encoding="utf-8"))
    probe_scene = (ROOT / config["probe_scene"]).resolve()
    floating_scene = (ROOT / config["floating_scene"]).resolve()
    included_model = (ROOT / config["included_model"]).resolve()
    for path in (probe_scene, floating_scene, included_model):
        if not path.is_file():
            raise SystemExit(f"Missing input: {path}")

    compiled = mujoco.MjModel.from_xml_path(str(floating_scene))
    compiled_data = mujoco.MjData(compiled)
    mujoco.mj_forward(compiled, compiled_data)
    active = {
        name(compiled, mujoco.mjtObj.mjOBJ_GEOM, index):
        (int(compiled.geom_contype[index]), int(compiled.geom_conaffinity[index]))
        for index in range(compiled.ngeom)
        if compiled.geom_contype[index] or compiled.geom_conaffinity[index]
    }
    expected_active = {
        "floor": (1, 0), "left_wheel_collision": (0, 1),
        "right_wheel_collision": (0, 1),
    }

    cases = config["cases"]
    thresholds = config["thresholds"]
    outputs: list[Path] = []
    all_rows: dict[str, list[dict[str, Any]]] = {}
    metrics: dict[str, Any] = {}

    def run_probe(case_name: str, **kwargs: Any) -> tuple[list[dict[str, Any]], dict[str, float]]:
        rows, case_metrics = probe_case(
            probe_scene, steps=int(cases["probe_steps"]),
            settle_steps=int(cases["probe_settle_steps"]),
            wheel_radius=float(config["wheel_radius_m"]), **kwargs
        )
        path = output_dir / f"{case_name}.csv"
        write_rows(path, rows)
        outputs.append(path)
        all_rows[case_name] = rows
        metrics[case_name] = case_metrics
        return rows, case_metrics

    normal, normal_metrics = run_probe("probe_normal", friction=1.0)
    rolling: dict[str, list[dict[str, Any]]] = {}
    for friction in cases["friction_sweep"]:
        key = f"rolling_mu_{float(friction):g}".replace(".", "p")
        rolling[key], _ = run_probe(
            key, friction=float(friction), torque=float(cases["wheel_torque_nm"])
        )
    rolling_neg, _ = run_probe(
        "rolling_negative", friction=1.0, torque=-float(cases["wheel_torque_nm"])
    )
    lateral: dict[str, list[dict[str, Any]]] = {}
    for friction in cases["friction_sweep"]:
        key = f"lateral_mu_{float(friction):g}".replace(".", "p")
        lateral[key], _ = run_probe(
            key, friction=float(friction), initial_vy=float(cases["initial_speed_m_s"])
        )
    lateral_negative, _ = run_probe(
        "lateral_negative", friction=1.0,
        initial_vy=-float(cases["initial_speed_m_s"]),
    )

    floating_a, unexpected_a = floating_case(
        floating_scene, int(cases["floating_steps"]), float(cases["floating_base_z_m"])
    )
    floating_b, unexpected_b = floating_case(
        floating_scene, int(cases["floating_steps"]), float(cases["floating_base_z_m"])
    )
    for filename, rows in (("floating_a.csv", floating_a), ("floating_b.csv", floating_b)):
        path = output_dir / filename
        write_rows(path, rows)
        outputs.append(path)
    all_rows["floating_a"] = floating_a
    all_rows["floating_b"] = floating_b

    checks: dict[str, bool] = {}
    checks["wheel_only_collision_mask"] = active == expected_active and compiled_data.ncon == 0
    checks["all_rows_finite"] = all(finite_rows(rows) for rows in all_rows.values())
    checks["no_unexpected_contact_pairs"] = not unexpected_a and not unexpected_b and normal_metrics["unexpected_pairs"] == 0

    tail = normal[-100:]
    load_errors = []
    impulse_errors = []
    penetrations = []
    for side in SIDES:
        expected_load = normal_metrics[f"{side}_mass_kg"] * 9.81
        measured_load = float(np.mean([row[f"{side}_fz_n"] for row in tail]))
        load_errors.append(abs(measured_load - expected_load) / expected_load)
        impulse_errors.append(normal_metrics[f"{side}_vertical_impulse_error_n_s"])
        penetrations.append(max(0.0, -min(float(row[f"{side}_min_dist_m"]) for row in normal)))
    metrics["normal_summary"] = {
        "max_static_load_relative_error": max(load_errors),
        "max_vertical_impulse_error_n_s": max(impulse_errors),
        "max_penetration_m": max(penetrations),
    }
    checks["normal_static_load"] = max(load_errors) <= thresholds["static_load_relative_error"]
    checks["normal_impulse_balance"] = max(impulse_errors) <= thresholds["vertical_impulse_error_n_s"]
    checks["penetration_bounded"] = max(penetrations) <= thresholds["max_penetration_m"]
    left_load = float(np.mean([row["left_fz_n"] for row in tail]))
    right_load = float(np.mean([row["right_fz_n"] for row in tail]))
    checks["normal_left_right_symmetry"] = (
        abs(left_load - right_load) / max(left_load, right_load)
        <= thresholds["left_right_relative_error"]
    )

    def displacement(rows: list[dict[str, Any]], side: str) -> float:
        return float(rows[-1][f"{side}_x_m"] - rows[int(cases["probe_settle_steps"])][f"{side}_x_m"])

    positive = rolling["rolling_mu_1"]
    frictionless = rolling["rolling_mu_0"]
    positive_dx = [displacement(positive, side) for side in SIDES]
    negative_dx = [displacement(rolling_neg, side) for side in SIDES]
    zero_dx = [displacement(frictionless, side) for side in SIDES]
    metrics["rolling_summary"] = {
        "positive_displacement_m": positive_dx,
        "negative_displacement_m": negative_dx,
        "frictionless_displacement_m": zero_dx,
    }
    checks["rolling_direction"] = all(
        value >= thresholds["rolling_min_forward_displacement_m"] for value in positive_dx
    ) and all(value <= -thresholds["rolling_min_forward_displacement_m"] for value in negative_dx)
    checks["rolling_requires_friction"] = max(abs(value) for value in zero_dx) <= thresholds["rolling_frictionless_max_displacement_m"]
    checks["rolling_left_right_symmetry"] = max(
        abs(positive_dx[0] - positive_dx[1]) / max(abs(positive_dx[0]), abs(positive_dx[1])),
        abs(negative_dx[0] - negative_dx[1]) / max(abs(negative_dx[0]), abs(negative_dx[1])),
    ) <= thresholds["left_right_relative_error"]

    free_vy = max(abs(float(row[f"{side}_vy_m_s"])) for side in SIDES for row in lateral["lateral_mu_0"][-20:])
    nominal_vy = max(abs(float(row[f"{side}_vy_m_s"])) for side in SIDES for row in lateral["lateral_mu_1"][-20:])
    high_vy = max(abs(float(row[f"{side}_vy_m_s"])) for side in SIDES for row in lateral["lateral_mu_2"][-20:])
    metrics["lateral_summary"] = {"mu0_final_speed_m_s": free_vy, "mu1_final_speed_m_s": nominal_vy, "mu2_final_speed_m_s": high_vy}
    checks["lateral_friction_trend"] = nominal_vy <= thresholds["lateral_velocity_ratio"] * free_vy and high_vy <= nominal_vy + 1e-9
    checks["lateral_bidirectional"] = max(
        abs(float(row[f"{side}_vy_m_s"]))
        for row in lateral_negative[-20:] for side in SIDES
    ) <= thresholds["lateral_velocity_ratio"] * free_vy

    cone_ok = True
    for case_name, rows in all_rows.items():
        if not case_name.startswith(("rolling_mu_1", "rolling_mu_2", "lateral_mu_1", "lateral_mu_2", "probe_normal")):
            continue
        mu = 2.0 if "mu_2" in case_name else 1.0
        for row in rows:
            for side in SIDES:
                tangential = math.hypot(float(row[f"{side}_fx_n"]), float(row[f"{side}_fy_n"]))
                cone_ok &= tangential <= mu * max(0.0, float(row[f"{side}_fz_n"])) + thresholds["friction_cone_tolerance_n"]
    checks["friction_force_bounded"] = cone_ok
    max_friction_power = max(
        float(row[f"{side}_fx_n"]) * float(row[f"{side}_slip_x_m_s"])
        + float(row[f"{side}_fy_n"]) * float(row[f"{side}_vy_m_s"])
        for case_name, rows in all_rows.items()
        if not case_name.startswith("floating")
        for row in rows
        for side in SIDES
    )
    metrics["max_friction_power_w"] = max_friction_power
    checks["friction_does_not_add_energy"] = (
        max_friction_power <= thresholds["friction_power_tolerance_w"]
    )

    contact_indices = [index for index, row in enumerate(floating_a) if row["left_contact_count"] or row["right_contact_count"]]
    first_contact = min(contact_indices) if contact_indices else len(floating_a)
    com_z = np.asarray([row["system_com_z_m"] for row in floating_a], dtype=float)
    acceleration = np.diff(com_z, 2) / config["physics_timestep_s"] ** 2
    precontact_end = max(1, min(len(acceleration), first_contact - 2))
    free_fall_error = abs(float(np.mean(acceleration[:precontact_end])) + 9.81)
    quat_error = max(abs(math.sqrt(sum(float(row[key]) ** 2 for key in ("quat_w", "quat_x", "quat_y", "quat_z"))) - 1.0) for row in floating_a)
    closure = max(float(row["closure_residual_m"]) for row in floating_a)
    penetration = max(0.0, -min(min(float(row["left_min_dist_m"]), float(row["right_min_dist_m"])) for row in floating_a))
    metrics["floating_summary"] = {
        "first_contact_step": first_contact,
        "free_fall_acceleration_error_m_s2": free_fall_error,
        "max_quaternion_norm_error": quat_error,
        "max_closure_residual_m": closure,
        "max_penetration_m": penetration,
    }
    checks["floating_contacts_both_wheels"] = any(row["left_contact_count"] for row in floating_a) and any(row["right_contact_count"] for row in floating_a)
    checks["floating_free_fall_gravity"] = free_fall_error <= thresholds["free_fall_acceleration_error_m_s2"]
    checks["floating_quaternion_normalized"] = quat_error <= thresholds["quaternion_norm_error"]
    checks["floating_closure_bounded"] = closure <= thresholds["closure_residual_m"]
    checks["floating_penetration_bounded"] = penetration <= thresholds["max_penetration_m"]
    checks["reset_replay_exact"] = floating_a == floating_b

    input_paths = [config_path, probe_scene, floating_scene, included_model, Path(__file__).resolve()]
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "profile": config["profile"],
        "model_revision": config["model_revision"],
        "hardware_data_used": False,
        "versions": {"mujoco": mujoco.__version__, "python": platform.python_version()},
        "timing": {key: config[key] for key in ("physics_timestep_s", "control_period_s", "physics_steps_per_control")},
        "contact": config["contact"],
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
        "outputs": {path.name: sha256(path) for path in outputs},
    }
    summary = {
        "schema_version": 1,
        "overall_pass": all(checks.values()),
        "hardware_data_used": False,
        "checks": checks,
        "metrics": metrics,
        "interpretation_limit": "Current nominal MuJoCo internal wheel-contact and zero-command floating-base evidence only; no standing, calibrated tire/ground, actuator, or real-hardware claim.",
    }
    manifest_path = output_dir / "run_manifest.json"
    summary_path = output_dir / "phase18_validation.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["overall_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
