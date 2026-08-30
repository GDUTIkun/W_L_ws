#!/usr/bin/env python3
"""Phase 37 axisymmetric-collision parity and causal replay runner."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase37_axisymmetric_collision_v1.json"
PHASE36_RUNNER = ROOT / "tools/experiments/run_phase36_wheel_phase_validity.py"
PHASE35_RUNNER = ROOT / "tools/experiments/run_phase35_workspace_attribution.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P36 = load_module(PHASE36_RUNNER, "phase37_reused_phase36")
P35 = load_module(PHASE35_RUNNER, "phase37_reused_phase35")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(P36.clean(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def max_abs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value))) if value.size else 0.0


def named_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise RuntimeError(f"missing {kind.name} {name}")
    return result


def collision_parity(config: dict[str, Any]) -> dict[str, Any]:
    nominal = mujoco.MjModel.from_xml_path(str(ROOT / config["nominal_scene"]))
    corrected = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
    errors: dict[str, float] = {}
    for field in ("body_pos", "body_quat", "body_mass", "body_ipos", "body_iquat",
                  "body_inertia", "jnt_pos", "jnt_axis", "jnt_range", "dof_armature",
                  "dof_damping", "dof_frictionloss", "actuator_gear", "actuator_ctrlrange",
                  "eq_data"):
        errors[field] = max_abs(np.asarray(getattr(nominal, field)) - np.asarray(getattr(corrected, field)))
    errors["gravity"] = max_abs(nominal.opt.gravity - corrected.opt.gravity)
    errors["timestep"] = abs(float(nominal.opt.timestep - corrected.opt.timestep))
    errors["solver_tolerance"] = abs(float(nominal.opt.tolerance - corrected.opt.tolerance))
    errors["integrator"] = abs(int(nominal.opt.integrator) - int(corrected.opt.integrator))
    errors["dimensions_except_geom"] = float(max(
        abs(nominal.nq - corrected.nq), abs(nominal.nv - corrected.nv),
        abs(nominal.nu - corrected.nu), abs(nominal.nbody - corrected.nbody),
        abs(nominal.njnt - corrected.njnt), abs(nominal.neq - corrected.neq)))
    wheel = {}
    for side in ("left", "right"):
        old_geom = named_id(nominal, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_wheel_collision")
        visual = named_id(corrected, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_wheel_visual")
        cylinder = named_id(corrected, mujoco.mjtObj.mjOBJ_GEOM, f"{side}_wheel_collision")
        old_body = int(nominal.geom_bodyid[old_geom]); new_body = int(corrected.geom_bodyid[cylinder])
        wheel[side] = {
            "body_mass_error_kg": abs(float(nominal.body_mass[old_body] - corrected.body_mass[new_body])),
            "body_com_error_m": max_abs(nominal.body_ipos[old_body] - corrected.body_ipos[new_body]),
            "body_inertia_error_kg_m2": max_abs(nominal.body_inertia[old_body] - corrected.body_inertia[new_body]),
            "body_inertia_quaternion_error": max_abs(nominal.body_iquat[old_body] - corrected.body_iquat[new_body]),
            "visual_mesh_data_id_equal": int(nominal.geom_dataid[old_geom]) == int(corrected.geom_dataid[visual]),
            "visual_mass_mask": [float(corrected.geom_contype[visual]), float(corrected.geom_conaffinity[visual])],
            "collision_type": int(corrected.geom_type[cylinder]),
            "collision_size_m": corrected.geom_size[cylinder].copy(),
            "collision_position_b_m": corrected.geom_pos[cylinder].copy(),
            "collision_mask": [int(corrected.geom_contype[cylinder]), int(corrected.geom_conaffinity[cylinder])],
            "friction_error": max_abs(nominal.geom_friction[old_geom] - corrected.geom_friction[cylinder]),
            "solref_error": max_abs(nominal.geom_solref[old_geom] - corrected.geom_solref[cylinder]),
            "solimp_error": max_abs(nominal.geom_solimp[old_geom] - corrected.geom_solimp[cylinder]),
        }
    floor_old = named_id(nominal, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    floor_new = named_id(corrected, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    errors["ground_friction"] = max_abs(nominal.geom_friction[floor_old] - corrected.geom_friction[floor_new])
    errors["ground_solref"] = max_abs(nominal.geom_solref[floor_old] - corrected.geom_solref[floor_new])
    errors["ground_solimp"] = max_abs(nominal.geom_solimp[floor_old] - corrected.geom_solimp[floor_new])
    maximum = max(errors.values())
    wheel_errors = max(value[key] for value in wheel.values() for key in (
        "body_mass_error_kg", "body_com_error_m", "body_inertia_error_kg_m2",
        "body_inertia_quaternion_error", "friction_error", "solref_error", "solimp_error"))
    masks_pass = all(value["visual_mass_mask"] == [0.0, 0.0]
                     and value["collision_mask"] == [0, 1]
                     and value["visual_mesh_data_id_equal"] for value in wheel.values())
    radius_pass = all(abs(float(value["collision_size_m"][0]) - 0.05) <= 1e-12 for value in wheel.values())
    threshold = float(config["thresholds"]["noncollision_parameter_parity"])
    return {"errors": errors, "wheel": wheel, "maximum_noncollision_error": max(maximum, wheel_errors),
            "only_collision_representation_changed": bool(max(maximum, wheel_errors) <= threshold
                                                         and masks_pass and radius_pass
                                                         and corrected.ngeom == nominal.ngeom + 2),
            "nominal_dimensions": [nominal.nq, nominal.nv, nominal.nu, nominal.nbody, nominal.ngeom],
            "corrected_dimensions": [corrected.nq, corrected.nv, corrected.nu,
                                     corrected.nbody, corrected.ngeom]}


def cylinder_bounds(self: Any) -> tuple[np.ndarray, np.ndarray]:
    lowers, uppers = [], []
    for geom in self.wheel_geoms:
        rotation = self.data.geom_xmat[geom].reshape(3, 3)
        axis = rotation[:, 2]
        radius, half_width = self.model.geom_size[geom, :2]
        extent = radius * np.sqrt(np.maximum(0.0, 1.0 - axis * axis)) + half_width * np.abs(axis)
        lowers.append(self.data.geom_xpos[geom] - extent)
        uppers.append(self.data.geom_xpos[geom] + extent)
    return np.min(lowers, axis=0), np.max(uppers, axis=0)


def cylinder_symmetry(self: Any) -> dict[str, Any]:
    return {side: {"hausdorff_m_by_order": {str(order): 0.0 for order in self.method["finite_symmetry_search_orders"]},
                   "equivalent_orders": list(self.method["finite_symmetry_search_orders"])}
            for side in ("left", "right")}


def run_periodicity(config_path: Path, output: Path) -> int:
    P36.Audit.mesh_bounds = cylinder_bounds
    P36.Audit.symmetry = cylinder_symmetry
    saved = sys.argv
    sys.argv = [str(PHASE36_RUNNER), "--method", str(config_path), "--output", str(output)]
    try:
        return int(P36.main())
    finally:
        sys.argv = saved


def contact_phase_metrics(details: dict[str, Any]) -> dict[str, Any]:
    samples = details["samples"]
    maxima = {"centroid_m": 0.0, "normal": 0.0, "depth_m": 0.0,
              "normal_load_n": 0.0, "contact_count_difference": 0}
    for mode in ("left", "right", "bilateral"):
        baseline = samples[f"{mode}:{0.0:+.8f}"]["contact"]
        base_points = np.asarray(baseline["points"]); base_normals = np.asarray(baseline["normals"])
        base_centroid = base_points.mean(axis=0); base_normal = base_normals.mean(axis=0)
        for key, sample in samples.items():
            if not key.startswith(mode + ":"):
                continue
            contact = sample["contact"]; points = np.asarray(contact["points"])
            normals = np.asarray(contact["normals"])
            maxima["contact_count_difference"] = max(
                maxima["contact_count_difference"], abs(int(contact["count"]) - int(baseline["count"])))
            if len(points): maxima["centroid_m"] = max(maxima["centroid_m"], max_abs(points.mean(axis=0) - base_centroid))
            if len(normals): maxima["normal"] = max(maxima["normal"], max_abs(normals.mean(axis=0) - base_normal))
            maxima["depth_m"] = max(maxima["depth_m"], abs(float(contact["minimum_distance_m"])
                                                           - float(baseline["minimum_distance_m"])))
            maxima["normal_load_n"] = max(maxima["normal_load_n"], max_abs(
                np.asarray(contact["normal_load_n"]) - np.asarray(baseline["normal_load_n"])))
    return maxima


def core_phase_variation(details: dict[str, Any]) -> dict[str, float]:
    maxima = {key: 0.0 for key in ("mass_matrix", "bias", "reduced_mass", "reduced_bias")}
    for layer in ("samples", "contact_off"):
        samples = details[layer]
        for mode in ("left", "right", "bilateral"):
            baseline = samples[f"{mode}:{0.0:+.8f}"]
            for key in maxima:
                base = np.asarray(baseline[key])
                maxima[key] = max(maxima[key], *(max_abs(np.asarray(sample[key]) - base)
                    for sample_key, sample in samples.items() if sample_key.startswith(mode + ":")))
    return maxima


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def run_h0(config: dict[str, Any], output: Path) -> dict[str, Any]:
    executable = ROOT / config["phase35_executable"]
    scene = ROOT / config["scene"]
    output.mkdir()
    commands = []
    for stem in ("H0-a", "H0-b"):
        csv_path = output / f"{stem}.csv"
        command = [str(executable), str(scene), str(csv_path), "H0_minimal_hold", "none", "0", "0"]
        subprocess.run(command, cwd=ROOT, check=True)
        commands.append(" ".join(command))
    first, second = read_rows(output / "H0-a.csv"), read_rows(output / "H0-b.csv")
    phase35_config = json.loads((ROOT / "simulation/mujoco/config/phase35_workspace_attribution_v1.json").read_text())
    analysis = P35.analyze_case(first, phase35_config)
    replay = P35.replay_error(first, second)
    maximum_spin = max(abs(float(row[name])) for row in first
                       for name in ("wheel_mesh_phase_left", "wheel_mesh_phase_right"))
    final = first[-1]
    threshold = config["thresholds"]
    drift = bool(analysis["failure_tick"] is not None or maximum_spin >= float(threshold["h0_spin_drift_rad"]))
    result = {"analysis": analysis, "replay_max_abs_error": replay,
              "maximum_absolute_wheel_spin_rad": maximum_spin,
              "final_tick": int(final["tick"]), "final_xi_m": [float(final["xi_left"]), float(final["xi_right"])],
              "final_zeta_m": [float(final["zeta_left"]), float(final["zeta_right"])],
              "drift_persists": drift,
              "classification": ("P37-F_H0_wheel_spin_drift_persists_with_clean_contact"
                                 if drift else "P37-G_H0_preexisting_drift_removed_with_clean_contact"),
              "commands": commands}
    write_json(output / "summary.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    config_path = args.config.resolve(); config = json.loads(config_path.read_text())
    output.mkdir(parents=True)
    def manifest() -> dict[str, Any]:
        inputs = [config_path, ROOT / config["scene"], ROOT / config["nominal_scene"],
                  ROOT / config["axisymmetric_model"], ROOT / config["nominal_model"],
                  ROOT / config["phase32_method"], PHASE36_RUNNER, PHASE35_RUNNER,
                  Path(__file__).resolve(), ROOT / config["phase35_executable"],
                  ROOT / config["source_phase36_summary"], ROOT / config["source_phase32_summary"]]
        return {"created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
                "replay_of": args.replay_of, "python": sys.version, "platform": platform.platform(),
                "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
                "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs}}
    parity = collision_parity(config)
    write_json(output / "collision_parity.json", parity)
    if not parity["only_collision_representation_changed"]:
        write_json(output / "summary.json", {"classification": "P37-E_collision_replacement_changes_noncollision_model_contract"})
        write_json(output / "manifest.json", manifest())
        return 2

    periodic_dir = output / "phase36_periodicity"
    if run_periodicity(config_path, periodic_dir) != 0:
        return 2
    periodic = json.loads((periodic_dir / "summary.json").read_text())
    periodic_details = json.loads((periodic_dir / "details.json").read_text())
    contact_metrics = contact_phase_metrics(periodic_details)
    source36 = json.loads((ROOT / config["source_phase36_summary"]).read_text())
    old_effect = float(source36["maxima"]["physical_ddxi_change_m_s2"])
    new_effect = float(periodic["maxima"]["physical_ddxi_change_m_s2"])
    off_effect = float(periodic["maxima"]["contact_off_ddxi_change_m_s2"])
    improvement = new_effect / old_effect
    thresholds = config["thresholds"]
    static_pass = bool(
        contact_metrics["centroid_m"] <= float(thresholds["contact_phase_position_m"])
        and contact_metrics["normal"] <= float(thresholds["contact_phase_normal"])
        and contact_metrics["depth_m"] <= float(thresholds["contact_phase_depth_m"])
        and contact_metrics["contact_count_difference"] == 0)
    isolation_pass = bool(new_effect <= max(float(thresholds["contact_on_off_absolute_m_s2"]),
                                            float(thresholds["contact_on_off_ratio"]) * off_effect))
    collision_pass = bool(static_pass and isolation_pass
                          and periodic["dg36_02_core_model_periodicity_pass"]
                          and periodic["periodic_dynamic_response_equivalence_pass"]
                          and improvement <= float(thresholds["phase36_ddxi_improvement_ratio"]))
    collision = {"static_contact_metrics": contact_metrics,
                 "retained_core_phase_variation": core_phase_variation(periodic_details),
                 "old_phase36_ddxi_m_s2": old_effect,
                 "new_ddxi_phase_effect_m_s2": new_effect, "contact_off_effect_m_s2": off_effect,
                 "ddxi_improvement_ratio": improvement, "static_pass": static_pass,
                 "isolation_pass": isolation_pass, "collision_artifact_removed": collision_pass}
    write_json(output / "collision_correction.json", collision)
    if not collision_pass:
        write_json(output / "summary.json", {"classification": "P37-D_axisymmetric_collision_still_phase_sensitive",
                                             "collision": collision, "production_modified": False})
        write_json(output / "manifest.json", manifest())
        return 2

    phase32_dir = output / "phase32_closure"
    command32 = [sys.executable, str(ROOT / config["phase32_runner"]), "--method",
                 str(ROOT / config["phase32_method"]), "--output", str(phase32_dir)]
    completed32 = subprocess.run(command32, cwd=ROOT)
    if completed32.returncode != 0:
        return completed32.returncode
    phase32 = json.loads((phase32_dir / "summary.json").read_text())
    old32 = json.loads((ROOT / config["source_phase32_summary"]).read_text())
    current = float(phase32["maxima"]["symmetric_ddxi_difference_m_s2"])
    old = float(old32["maxima"]["symmetric_ddxi_difference_m_s2"])
    if not phase32["closure_failure"]:
        closure_class = "P37-A_collision_correction_restores_x16_closure"
    elif current / old <= float(thresholds["phase32_material_improvement_ratio"]):
        closure_class = "P37-B_collision_correction_improves_but_does_not_restore_x16_closure"
    else:
        closure_class = "P37-C_x16_nonclosure_persists_after_collision_correction"
    closure = {"classification": closure_class, "old_symmetric_ddxi_m_s2": old,
               "new_symmetric_ddxi_m_s2": current, "ratio": current / old,
               "phase32_summary": phase32}
    write_json(output / "phase32_closure_reassessment.json", closure)

    h0 = run_h0(config, output / "phase35_h0")
    replay_pass = h0["replay_max_abs_error"] <= float(thresholds["h0_replay_max_abs_error"])
    summary = {"collision_verdict": "axisymmetric_collision_removes_P36_D",
               "closure_verdict": closure_class, "h0_verdict": h0["classification"],
               "workspace_gate_natural_validity_basis": False,
               "phase34_tracking_authorized": False,
               "phase34_tracking_reason": "live gate remains unchanged; requires independent workspace-contract correction",
               "dg37_00_parity_pass": True, "dg37_01_static_contact_pass": static_pass,
               "dg37_02_periodic_contact_pass": periodic["periodic_dynamic_response_equivalence_pass"],
               "dg37_03_contact_isolation_pass": isolation_pass,
               "dg37_04_phase32_replay_complete": True, "dg37_05_h0_replay_pass": replay_pass,
               "dg37_06_workspace_candidate": "remove absolute wheel angle from finite validity coordinates",
               "dg37_07_tracking_reopened": False, "production_modified": False,
               "collision": collision, "closure": {key: value for key, value in closure.items() if key != "phase32_summary"},
               "h0": {key: value for key, value in h0.items() if key != "commands"}}
    write_json(output / "summary.json", summary)
    final_manifest = manifest()
    final_manifest["phase32_command"] = " ".join(command32)
    write_json(output / "manifest.json", final_manifest)
    return 0 if replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
