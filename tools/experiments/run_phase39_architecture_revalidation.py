#!/usr/bin/env python3
"""Phase 39 centered-wheel nominal-plant architecture revalidation."""

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
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase39_architecture_revalidation_v1.json"
PHASE37_RUNNER = ROOT / "tools/experiments/run_phase37_causal_revalidation.py"
PHASE32_MARKOV = ROOT / "tools/experiments/run_phase32_markov_closure.py"
PHASE32_LEG = ROOT / "tools/experiments/run_phase32_leg_nullspace.py"
PHASE32_ANGLE = ROOT / "tools/experiments/run_phase32_wheel_angle_hybrid.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P37 = load_module(PHASE37_RUNNER, "phase39_reused_phase37")
M = load_module(PHASE32_MARKOV, "phase39_reused_phase32_markov")


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


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(clean(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def max_abs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value))) if value.size else 0.0


def required_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"missing {kind.name} {name}")
    return value


def object_names(model: mujoco.MjModel, kind: mujoco.mjtObj, count: int) -> list[str | None]:
    return [mujoco.mj_id2name(model, kind, index) for index in range(count)]


def model_parity(config: dict[str, Any]) -> dict[str, Any]:
    model_a = mujoco.MjModel.from_xml_path(str(ROOT / config["model_a_scene"]))
    model_b = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
    dimension_fields = ("nq", "nv", "nu", "nbody", "njnt", "ngeom", "nsite", "neq")
    dimensions = {field: [int(getattr(model_a, field)), int(getattr(model_b, field))]
                  for field in dimension_fields}
    names = {
        "body": object_names(model_a, mujoco.mjtObj.mjOBJ_BODY, model_a.nbody)
        == object_names(model_b, mujoco.mjtObj.mjOBJ_BODY, model_b.nbody),
        "joint": object_names(model_a, mujoco.mjtObj.mjOBJ_JOINT, model_a.njnt)
        == object_names(model_b, mujoco.mjtObj.mjOBJ_JOINT, model_b.njnt),
        "geom": object_names(model_a, mujoco.mjtObj.mjOBJ_GEOM, model_a.ngeom)
        == object_names(model_b, mujoco.mjtObj.mjOBJ_GEOM, model_b.ngeom),
        "site": object_names(model_a, mujoco.mjtObj.mjOBJ_SITE, model_a.nsite)
        == object_names(model_b, mujoco.mjtObj.mjOBJ_SITE, model_b.nsite),
        "actuator": object_names(model_a, mujoco.mjtObj.mjOBJ_ACTUATOR, model_a.nu)
        == object_names(model_b, mujoco.mjtObj.mjOBJ_ACTUATOR, model_b.nu),
        "equality": object_names(model_a, mujoco.mjtObj.mjOBJ_EQUALITY, model_a.neq)
        == object_names(model_b, mujoco.mjtObj.mjOBJ_EQUALITY, model_b.neq),
    }
    parameter_fields = (
        "body_pos", "body_quat", "body_mass", "body_iquat", "body_inertia",
        "jnt_type", "jnt_bodyid", "jnt_pos", "jnt_axis", "jnt_range", "jnt_margin",
        "dof_frictionloss", "dof_armature", "dof_damping",
        "geom_type", "geom_contype", "geom_conaffinity", "geom_condim", "geom_bodyid",
        "geom_dataid", "geom_pos", "geom_quat", "geom_size", "geom_friction",
        "geom_solref", "geom_solimp", "geom_margin", "geom_gap",
        "site_type", "site_bodyid", "site_pos", "site_quat", "site_size",
        "actuator_trntype", "actuator_dyntype", "actuator_gaintype", "actuator_biastype",
        "actuator_trnid", "actuator_gear", "actuator_ctrlrange", "actuator_forcerange",
        "actuator_dynprm", "actuator_gainprm", "actuator_biasprm",
        "eq_type", "eq_obj1id", "eq_obj2id", "eq_data", "eq_solref", "eq_solimp",
    )
    errors = {field: max_abs(np.asarray(getattr(model_a, field), dtype=float)
                             - np.asarray(getattr(model_b, field), dtype=float))
              for field in parameter_fields}
    errors.update({
        "gravity": max_abs(model_a.opt.gravity - model_b.opt.gravity),
        "timestep": abs(float(model_a.opt.timestep - model_b.opt.timestep)),
        "tolerance": abs(float(model_a.opt.tolerance - model_b.opt.tolerance)),
        "integrator": abs(int(model_a.opt.integrator) - int(model_b.opt.integrator)),
    })
    ipos_difference = np.asarray(model_b.body_ipos - model_a.body_ipos)
    allowed = np.zeros_like(ipos_difference, dtype=bool)
    wheels: dict[str, Any] = {}
    for side in ("left", "right"):
        body = required_id(model_a, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel_body")
        allowed[body, :2] = True
        wheels[side] = {
            "body_id": body,
            "model_a_ipos_b_m": model_a.body_ipos[body].copy(),
            "model_b_ipos_b_m": model_b.body_ipos[body].copy(),
            "radial_com_m": float(np.linalg.norm(model_b.body_ipos[body, :2])),
            "axial_com_error_m": abs(float(model_b.body_ipos[body, 2] - model_a.body_ipos[body, 2])),
            "mass_error_kg": abs(float(model_b.body_mass[body] - model_a.body_mass[body])),
            "principal_inertia_error_kg_m2": max_abs(model_b.body_inertia[body] - model_a.body_inertia[body]),
            "inertial_orientation_error": max_abs(model_b.body_iquat[body] - model_a.body_iquat[body]),
        }
    forbidden_ipos_error = max_abs(ipos_difference[~allowed])
    threshold = float(config["thresholds"]["compiled_parameter_parity"])
    centered = float(config["thresholds"]["centered_radial_com_m"])
    pass_value = bool(
        all(left == right for left, right in dimensions.values())
        and all(names.values()) and max(errors.values()) <= threshold
        and forbidden_ipos_error <= threshold
        and all(item["radial_com_m"] <= centered
                and max(item[key] for key in (
                    "axial_com_error_m", "mass_error_kg", "principal_inertia_error_kg_m2",
                    "inertial_orientation_error")) <= threshold
                for item in wheels.values())
    )
    return {
        "pass": pass_value,
        "classification": "DG39-00_PASS" if pass_value else "P39-A_nominal_model_parity_failure",
        "dimensions": dimensions,
        "name_parity": names,
        "unchanged_parameter_max_abs_errors": errors,
        "forbidden_body_ipos_error_m": forbidden_ipos_error,
        "allowed_difference": "left/right wheel body_ipos body-X/Y only",
        "wheels": wheels,
    }


def phase_isolation(config_path: Path, config: dict[str, Any], output: Path) -> dict[str, Any]:
    periodic_output = output / "periodicity"
    result = P37.run_periodicity(config_path, periodic_output)
    if result != 0:
        raise RuntimeError(f"Phase36 periodic runner returned {result}")
    summary = json.loads((periodic_output / "summary.json").read_text(encoding="utf-8"))
    details = json.loads((periodic_output / "details.json").read_text(encoding="utf-8"))
    contact = P37.contact_phase_metrics(details)
    old = json.loads((ROOT / config["source_phase36_summary"]).read_text(encoding="utf-8"))
    old_effect = float(old["maxima"]["physical_ddxi_change_m_s2"])
    on_effect = float(summary["maxima"]["physical_ddxi_change_m_s2"])
    off_effect = float(summary["maxima"]["contact_off_ddxi_change_m_s2"])
    threshold = config["thresholds"]
    geometry_pass = bool(
        contact["centroid_m"] <= float(threshold["contact_phase_position_m"])
        and contact["normal"] <= float(threshold["contact_phase_normal"])
        and contact["depth_m"] <= float(threshold["contact_phase_depth_m"])
        and contact["contact_count_difference"] == 0)
    isolation_pass = bool(on_effect <= max(
        float(threshold["contact_on_off_absolute_m_s2"]),
        float(threshold["contact_on_off_ratio"]) * off_effect))
    improvement_pass = on_effect / old_effect <= float(threshold["phase36_ddxi_improvement_ratio"])
    pass_value = bool(
        geometry_pass and isolation_pass and improvement_pass
        and summary["dg36_02_core_model_periodicity_pass"]
        and summary["periodic_dynamic_response_equivalence_pass"])
    result_summary = {
        "pass": pass_value,
        "classification": "DG39-01_PASS" if pass_value else "P39-A_nominal_phase_isolation_not_closed",
        "geometry_pass": geometry_pass,
        "isolation_pass": isolation_pass,
        "improvement_pass": improvement_pass,
        "contact_phase_metrics": contact,
        "contact_on_ddxi_effect_m_s2": on_effect,
        "contact_off_ddxi_effect_m_s2": off_effect,
        "contact_on_off_ratio": on_effect / max(off_effect, 1e-30),
        "phase36_improvement_ratio": on_effect / old_effect,
        "core_periodicity_pass": summary["dg36_02_core_model_periodicity_pass"],
        "response_periodicity_pass": summary["periodic_dynamic_response_equivalence_pass"],
        "retained_core_phase_variation": P37.core_phase_variation(details),
    }
    write_json(output / "summary.json", result_summary)
    return result_summary


def run_phase32(script: Path, method: Path, output: Path, replay_of: str | None) -> tuple[int, dict[str, Any]]:
    command = [sys.executable, str(script), "--method", str(method), "--output", str(output)]
    if replay_of:
        command.extend(["--replay-of", replay_of])
    completed = subprocess.run(command, cwd=ROOT)
    summary_path = output / "summary.json"
    if not summary_path.is_file():
        raise RuntimeError(f"missing Phase32 summary after return code {completed.returncode}: {summary_path}")
    return completed.returncode, json.loads(summary_path.read_text(encoding="utf-8"))


def angle_fixed_torque_details(angle_method: Path) -> dict[str, Any]:
    spec = json.loads(angle_method.read_text(encoding="utf-8"))
    method = json.loads((ROOT / spec["base_method"]).read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(ROOT / method["scene"]))
    geometry = M.CONTRACT.Geometry(model, method["body_site_contract"])
    base_weld = required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    wheel_geoms = [required_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
                   for name in method["body_site_contract"]["wheel_geoms"]]
    floor = required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    wheel_qpos = [14, 9]
    raw_root = ROOT / method["source_phase28_run"]
    executable = ROOT / method["wbc_sweep_executable"]
    pairs = []
    maxima = {"fixed_torque_ddxi_difference_m_s2": 0.0, "requested_wrench_error": 0.0}
    for case in method["cases"]:
        control_path = raw_root / f"{case['id']}_control.csv"
        plant_path = raw_root / f"{case['id']}_plant.csv"
        controls = M.read_csv(control_path)
        control_by_tick = {int(row["tick"]): row for row in controls}
        plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
                 for row in M.read_csv(plant_path)}
        baselines = M.run_baselines(executable, control_path, case["authority_ticks"])
        for tick in case["authority_ticks"]:
            row = plant[(tick - 1, 4)]
            qpos = M.vector(row, "qpos", model.nq)
            qvel = M.vector(row, "qvel", model.nv)
            baseline_torque = M.vector(baselines[tick], "tau", 6)
            requested = M.requested_wrench(control_by_tick[tick])
            scales: dict[str, Any] = {}
            for scale in (1.0, float(spec["half_scale"])):
                signed: dict[str, Any] = {}
                for sign in (-1, 1):
                    delta = sign * scale * float(spec["wheel_angle_delta_rad"])
                    changed = qpos.copy()
                    changed[wheel_qpos] += delta
                    plant_value = M.evaluate_plant(
                        geometry, base_weld, changed, qvel, baseline_torque,
                        float(row["time_s"]), wheel_geoms, floor)
                    signed[str(sign)] = {
                        "wheel_angle_delta_rad": delta,
                        "requested_wrench_flu": requested,
                        "fixed_baseline_torque_plant": plant_value,
                    }
                plus = np.asarray(signed["1"]["fixed_baseline_torque_plant"]["ddxi_m_s2"])
                minus = np.asarray(signed["-1"]["fixed_baseline_torque_plant"]["ddxi_m_s2"])
                difference = plus - minus
                value = max_abs(difference)
                maxima["fixed_torque_ddxi_difference_m_s2"] = max(
                    maxima["fixed_torque_ddxi_difference_m_s2"], value)
                scales[str(scale)] = {
                    "signed": signed,
                    "fixed_torque_symmetric_ddxi_difference_m_s2": difference,
                    "maximum_abs_difference_m_s2": value,
                }
            pairs.append({"case": case["id"], "tick": tick, "scales": scales})
    return {"pairs": pairs, "maxima": maxima}


def phase32_revalidation(config: dict[str, Any], output: Path, replay_of: str | None) -> dict[str, Any]:
    method = ROOT / config["phase32_method"]
    angle_method = ROOT / config["phase32_angle_method"]
    leg_code, leg = run_phase32(PHASE32_LEG, method, output / "leg-nullspace", replay_of)
    rate_code, rate = run_phase32(PHASE32_MARKOV, method, output / "wheel-rate", replay_of)
    angle_code, angle = run_phase32(PHASE32_ANGLE, angle_method, output / "wheel-angle", replay_of)
    fixed_angle = angle_fixed_torque_details(angle_method)
    write_json(output / "wheel-angle-fixed-torque.json", fixed_angle)
    gate = float(config["thresholds"]["closure_difference_max_abs_m_s2"])
    leg_valid = bool(leg.get("valid")) and leg_code == 0
    rate_valid = bool(
        rate.get("projection_pass") and rate.get("full_ddxi_oracle_pass")
        and rate.get("perturbation_consistency_pass") and rate.get("finite_pass")
        and rate.get("bilateral_contact_pass")) and rate_code == 0
    angle_valid = bool(angle.get("valid")) and angle_code in (0, 2)
    families = {
        "C1_leg_configuration": {
            "valid": leg_valid,
            "maximum_ddxi_difference_m_s2": float(leg["maxima"]["C1_symmetric_ddxi_difference_m_s2"]),
        },
        "C2_leg_velocity": {
            "valid": leg_valid,
            "maximum_ddxi_difference_m_s2": float(leg["maxima"]["C2_symmetric_ddxi_difference_m_s2"]),
        },
        "C3_wheel_spin_rate": {
            "valid": rate_valid,
            "maximum_ddxi_difference_m_s2": float(rate["maxima"]["symmetric_ddxi_difference_m_s2"]),
        },
        "wheel_absolute_angle": {
            "valid": angle_valid,
            "maximum_ddxi_difference_m_s2": float(angle["maxima"]["maximum_symmetric_ddxi_difference_m_s2"]),
            "fixed_torque_maximum_ddxi_difference_m_s2": float(
                fixed_angle["maxima"]["fixed_torque_ddxi_difference_m_s2"]),
        },
    }
    for value in families.values():
        value["pass"] = bool(value["valid"] and value["maximum_ddxi_difference_m_s2"] <= gate)
    all_valid = all(value["valid"] for value in families.values())
    angle_pass = families["wheel_absolute_angle"]["pass"]
    smooth_passes = [families[key]["pass"] for key in (
        "C1_leg_configuration", "C2_leg_velocity", "C3_wheel_spin_rate")]
    if all_valid and angle_pass and all(smooth_passes):
        classification = "P39-B_x16_closure_restored"
    elif all_valid and angle_pass and any(smooth_passes) and not all(smooth_passes):
        classification = "P39-C_x16_closure_improved_but_not_restored"
    elif all_valid and angle_pass and not any(smooth_passes):
        classification = "P39-D_x16_nonclosure_structurally_persists"
    else:
        classification = "P39-U_closure_replay_inconclusive"
    baseline_paths = {
        "C1_C2": ROOT / config["source_phase32_leg_summary"],
        "C3": ROOT / config["source_phase32_rate_summary"],
        "angle": ROOT / config["source_phase32_angle_summary"],
    }
    baselines = {key: json.loads(path.read_text(encoding="utf-8"))
                 for key, path in baseline_paths.items()}
    comparison = {
        "C1_ratio_to_phase32": families["C1_leg_configuration"]["maximum_ddxi_difference_m_s2"]
        / float(baselines["C1_C2"]["maxima"]["C1_symmetric_ddxi_difference_m_s2"]),
        "C2_ratio_to_phase32": families["C2_leg_velocity"]["maximum_ddxi_difference_m_s2"]
        / float(baselines["C1_C2"]["maxima"]["C2_symmetric_ddxi_difference_m_s2"]),
        "C3_ratio_to_phase32": families["C3_wheel_spin_rate"]["maximum_ddxi_difference_m_s2"]
        / float(baselines["C3"]["maxima"]["symmetric_ddxi_difference_m_s2"]),
        "angle_ratio_to_phase32": families["wheel_absolute_angle"]["maximum_ddxi_difference_m_s2"]
        / float(baselines["angle"]["maxima"]["maximum_symmetric_ddxi_difference_m_s2"]),
    }
    realized_max = max(
        float(leg["maxima"]["realized_wrench_relative_difference"]),
        float(rate["maxima"]["realized_wrench_relative_difference"]),
        float(angle["maxima"]["realized_wrench_relative_difference"]),
    )
    result = {
        "classification": classification,
        "valid": all_valid,
        "families": families,
        "comparison_to_phase32": comparison,
        "runner_return_codes": {"leg": leg_code, "rate": rate_code, "angle": angle_code},
        "requested_wrench_parity_pass": bool(
            float(rate["maxima"]["requested_wrench_error"]) <= 1e-12),
        "maximum_realized_wrench_relative_difference": realized_max,
        "realized_wrench_parity_pass": realized_max <= float(
            config["thresholds"]["realized_wrench_relative_max"]),
        "fixed_torque_angle_maximum_ddxi_difference_m_s2": fixed_angle["maxima"][
            "fixed_torque_ddxi_difference_m_s2"],
    }
    write_json(output / "summary.json", result)
    return result


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def h0_revalidation(config: dict[str, Any], output: Path) -> dict[str, Any]:
    raw = P37.run_h0(config, output)
    rows = read_csv(output / "H0-a.csv")
    initial = [float(rows[0]["delta2"]), float(rows[0]["delta5"])]
    drift = [max(abs(float(row[f"delta{joint}"]) - initial[index]) for row in rows)
             for index, joint in enumerate((2, 5))]
    valid_rows = [row for row in rows if int(row["model_status"]) == 0]
    threshold = config["thresholds"]
    validity = {
        "replay": float(raw["replay_max_abs_error"]) <= float(threshold["h0_replay_max_abs_error"]),
        "bilateral_contact": all(row["contact_left"] == "1" and row["contact_right"] == "1"
                                 for row in valid_rows),
        "hard": max((float(row["hard"]) for row in valid_rows), default=0.0)
        <= float(threshold["maximum_hard_violation"]),
        "slack": max((float(row["maximum_normalized_slack"]) for row in valid_rows), default=0.0)
        <= float(threshold["maximum_normalized_slack"]),
        "torque": min((float(row[f"tau_margin{joint}"]) for row in valid_rows for joint in range(6)),
                      default=math.inf) >= float(threshold["minimum_torque_margin_nm"]),
        "finite": all(math.isfinite(float(row[name])) for row in valid_rows
                      for name in ("xi_left", "xi_right", "zeta_left", "zeta_right",
                                   "physical_ddxi_left", "physical_ddxi_right")),
    }
    valid = all(validity.values())
    failure_tick = raw["analysis"]["failure_tick"]
    if not valid:
        classification = "P39-U_H0_inconclusive"
    elif failure_tick is None and max(drift) <= float(threshold["h0_spin_drift_rad"]):
        classification = "P39-E_H0_spin_drift_removed"
    else:
        classification = "P39-F_H0_spin_drift_persists"
    result = {
        "classification": classification,
        "valid": valid,
        "validity": validity,
        "failure_tick": failure_tick,
        "first_failed_index": raw["analysis"]["first_failed_index"],
        "maximum_wheel_canonical_delta_change_rad": {"left": drift[0], "right": drift[1]},
        "maximum_absolute_wheel_spin_rad": raw["maximum_absolute_wheel_spin_rad"],
        "replay_max_abs_error": raw["replay_max_abs_error"],
        "trend": raw["analysis"]["trends"],
        "upstream_contact_loss": not raw["analysis"]["bilateral_contact_before_failure"],
        "upstream_hard_violation": not validity["hard"],
        "upstream_slack_violation": not validity["slack"],
        "upstream_torque_limit": not validity["torque"],
        "final_tick": raw["final_tick"],
        "final_xi_m": raw["final_xi_m"],
        "final_zeta_m": raw["final_zeta_m"],
    }
    write_json(output / "phase39_summary.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    config_path = args.config.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    config = json.loads(config_path.read_text(encoding="utf-8"))
    output.mkdir(parents=True)
    parity = model_parity(config)
    write_json(output / "model_parity.json", parity)
    if not parity["pass"]:
        summary = {"classification": parity["classification"], "dg39_00_parity_pass": False}
        write_json(output / "summary.json", summary)
        return 2
    isolation = phase_isolation(config_path, config, output / "phase-isolation")
    if not isolation["pass"]:
        summary = {"classification": isolation["classification"], "dg39_00_parity_pass": True,
                   "dg39_01_phase_isolation_pass": False}
        write_json(output / "summary.json", summary)
        return 2
    phase32 = phase32_revalidation(config, output / "phase32-closure", args.replay_of)
    if phase32["classification"] == "P39-U_closure_replay_inconclusive":
        summary = {"classification": phase32["classification"], "dg39_00_parity_pass": True,
                   "dg39_01_phase_isolation_pass": True, "dg39_02_phase32_valid": False}
        write_json(output / "summary.json", summary)
        return 2
    h0 = h0_revalidation(config, output / "phase35-h0")
    summary = {
        "classification": phase32["classification"],
        "h0_classification": h0["classification"],
        "dg39_00_parity_pass": True,
        "dg39_01_phase_isolation_pass": True,
        "dg39_02_phase32_valid": phase32["valid"],
        "dg39_03_h0_valid": h0["valid"],
        "families": phase32["families"],
        "comparison_to_phase32": phase32["comparison_to_phase32"],
        "requested_wrench_parity_pass": phase32["requested_wrench_parity_pass"],
        "realized_wrench_parity_pass": phase32["realized_wrench_parity_pass"],
        "phase_isolation": isolation,
        "h0": h0,
        "workspace_absolute_angle_model_validity_basis": False,
        "workspace_safety_domain_basis_resolved": False,
        "phase34_tracking_run": False,
        "production_modified": False,
    }
    write_json(output / "summary.json", summary)
    input_paths = [
        config_path, ROOT / config["scene"], ROOT / config["model_a_scene"],
        ROOT / config["model_a"], ROOT / config["model_b"],
        ROOT / config["phase32_method"], ROOT / config["phase32_angle_method"],
        ROOT / config["source_phase35_hold"], ROOT / config["source_phase36_summary"],
        ROOT / config["source_phase37_config"], ROOT / config["source_phase38_summary"],
        ROOT / config["source_phase32_leg_summary"], ROOT / config["source_phase32_rate_summary"],
        ROOT / config["source_phase32_angle_summary"], ROOT / config["phase35_executable"],
        ROOT / config["phase35_config"],
        PHASE37_RUNNER, PHASE32_MARKOV, PHASE32_LEG, PHASE32_ANGLE, Path(__file__).resolve(),
    ]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv),
        "replay_of": args.replay_of,
        "python": sys.version,
        "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
        "model_a_b_compiled_diff": parity,
    }
    write_json(output / "manifest.json", manifest)
    return 0 if h0["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
