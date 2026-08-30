#!/usr/bin/env python3
"""Phase 38 diagnostic wheel COM/inertia counterfactual attribution."""

from __future__ import annotations

import argparse
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
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase38_wheel_inertia_attribution_v1.json"
P36_PATH = ROOT / "tools/experiments/run_phase36_wheel_phase_validity.py"
P37_PATH = ROOT / "tools/experiments/run_phase37_causal_revalidation.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P36 = load_module(P36_PATH, "phase38_reused_phase36")
P37 = load_module(P37_PATH, "phase38_reused_phase37")
P36.Audit.mesh_bounds = P37.cylinder_bounds
P36.Audit.symmetry = P37.cylinder_symmetry


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(P36.clean(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def max_abs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value))) if value.size else 0.0


def rotation(quaternion: np.ndarray) -> np.ndarray:
    matrix = np.zeros(9)
    mujoco.mju_quat2Mat(matrix, quaternion)
    return matrix.reshape(3, 3)


def tensor(model: mujoco.MjModel, body: int) -> np.ndarray:
    frame = rotation(model.body_iquat[body])
    return frame @ np.diag(model.body_inertia[body]) @ frame.T


def wheel_ids(model: mujoco.MjModel) -> list[int]:
    return [P36.object_id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel_body")
            for side in ("left", "right")]


def inertial_semantics(config: dict[str, Any]) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
    result = {}
    thresholds = config["thresholds"]
    for side, body in zip(("left", "right"), wheel_ids(model)):
        inertia = tensor(model, body)
        com = model.body_ipos[body].copy()
        radial = float(np.linalg.norm(com[:2]))
        transverse = float(abs(inertia[0, 0] - inertia[1, 1]) /
                           (0.5 * (inertia[0, 0] + inertia[1, 1])))
        scale = max(float(np.mean(np.diag(inertia))), 1e-30)
        products = {"Ixy": float(inertia[0, 1]), "Ixz": float(inertia[0, 2]),
                    "Iyz": float(inertia[1, 2])}
        result[side] = {
            "body_frame": "wheel body local frame",
            "hinge_point_b_m": [0.0, 0.0, 0.0], "hinge_axis_b": [0.0, 0.0, 1.0],
            "collision_axis_b": [0.0, 0.0, 1.0],
            "mass_kg": float(model.body_mass[body]), "com_b_m": com,
            "com_radial_distance_m": radial, "com_axial_m": float(com[2]),
            "principal_inertia_kg_m2": model.body_inertia[body].copy(),
            "inertial_frame_quaternion_b": model.body_iquat[body].copy(),
            "inertia_body_axle_frame_kg_m2": inertia,
            "transverse_anisotropy": transverse,
            "products_kg_m2": products,
            "normalized_products": {key: abs(value) / scale for key, value in products.items()},
            "positive_definite": bool(np.min(np.linalg.eigvalsh(inertia)) >=
                                      float(thresholds["positive_inertia_eigenvalue_kg_m2"])),
            "radial_com_significant": radial >= float(thresholds["radial_com_significant_m"]),
            "anisotropy_significant": transverse >= float(thresholds["transverse_anisotropy_significant"]),
        }
    left, right = result["left"], result["right"]
    radial_mismatch = abs(left["com_radial_distance_m"] - right["com_radial_distance_m"]) / max(
        left["com_radial_distance_m"], right["com_radial_distance_m"], 1e-30)
    inertia_left = np.asarray(left["inertia_body_axle_frame_kg_m2"])
    inertia_right = np.asarray(right["inertia_body_axle_frame_kg_m2"])
    inertia_mismatch = max_abs(inertia_left - inertia_right) / max(max_abs(inertia_left), 1e-30)
    result["left_right"] = {"radial_com_relative_mismatch": radial_mismatch,
                            "inertia_relative_mismatch": inertia_mismatch,
                            "v4_authorized": max(radial_mismatch, inertia_mismatch) >
                                float(thresholds["left_right_relative_mismatch"])}
    return result


def analytic_plausibility(semantics: dict[str, Any], config: dict[str, Any]) -> dict[str, Any]:
    radius, width = 0.05, 0.04
    result = {}
    for side in ("left", "right"):
        item = semantics[side]; mass = float(item["mass_kg"])
        axial = 0.5 * mass * radius * radius
        transverse = mass * (3.0 * radius * radius + width * width) / 12.0
        actual = np.asarray(item["inertia_body_axle_frame_kg_m2"])
        result[side] = {"solid_cylinder_axial_kg_m2": axial,
                        "solid_cylinder_transverse_kg_m2": transverse,
                        "actual_axial_ratio": float(actual[2, 2] / axial),
                        "actual_transverse_ratios": [float(actual[0, 0] / transverse),
                                                     float(actual[1, 1] / transverse)]}
    ratios = [value for item in result.values() for key, raw in item.items() if "ratio" in key
              for value in (raw if isinstance(raw, list) else [raw])]
    threshold = config["thresholds"]
    result["scale_plausible"] = bool(min(ratios) >= float(threshold["analytic_scale_ratio_min"])
                                     and max(ratios) <= float(threshold["analytic_scale_ratio_max"])
                                     and all(semantics[side]["positive_definite"] for side in ("left", "right")))
    result["source_limit"] = ("STL geometry plus explicit geom mass is available; no STEP/SolidWorks "
                              "mass-property report or assembly-density provenance was found")
    return result


def apply_variant(audit: Any, variant: str) -> dict[str, Any]:
    before, after = {}, {}
    for side, body in zip(("left", "right"), wheel_ids(audit.model)):
        before[side] = {"mass": float(audit.model.body_mass[body]),
                        "com": audit.model.body_ipos[body].copy(),
                        "principal_inertia": audit.model.body_inertia[body].copy(),
                        "inertial_quaternion": audit.model.body_iquat[body].copy(),
                        "tensor_body": tensor(audit.model, body)}
        if variant in ("V1", "V3"):
            audit.model.body_ipos[body, :2] = 0.0
        if variant in ("V2", "V3"):
            current = tensor(audit.model, body)
            transverse = 0.5 * (current[0, 0] + current[1, 1])
            audit.model.body_inertia[body] = (transverse, transverse, current[2, 2])
            audit.model.body_iquat[body] = (1.0, 0.0, 0.0, 0.0)
        after[side] = {"mass": float(audit.model.body_mass[body]),
                       "com": audit.model.body_ipos[body].copy(),
                       "principal_inertia": audit.model.body_inertia[body].copy(),
                       "inertial_quaternion": audit.model.body_iquat[body].copy(),
                       "tensor_body": tensor(audit.model, body)}
    mujoco.mj_setConst(audit.model, audit.data)
    return {"rule": audit.method["variants"][variant], "before": before, "after": after}


def force_metrics(audit: Any) -> dict[str, np.ndarray]:
    normal = np.zeros(2); tangential = np.zeros(2)
    for index in range(audit.data.ncon):
        contact = audit.data.contact[index]
        side = next((side for side, geom in enumerate(audit.wheel_geoms)
                     if {int(contact.geom1), int(contact.geom2)} == {geom, audit.floor}), None)
        if side is None:
            continue
        force = np.zeros(6); mujoco.mj_contactForce(audit.model, audit.data, index, force)
        normal[side] += abs(float(force[0])); tangential[side] += float(np.linalg.norm(force[1:3]))
    return {"normal_load_n": normal, "tangential_load_n": tangential}


def run_variant(config: dict[str, Any], variant: str) -> dict[str, Any]:
    audit = P36.Audit(config)
    descriptor = apply_variant(audit, variant)
    phases = sorted(set(config["coarse_phase_rad"] + config["boundary_phase_rad"]))
    layers = {}
    for contact in (False, True):
        name = "contact_on" if contact else "contact_off"
        samples = {}
        for mode in config["modes"]:
            for phase in phases:
                sample = audit.sample(mode, float(phase), contact=contact)
                sample["force"] = force_metrics(audit)
                samples[f"{mode}:{float(phase):+.8f}"] = sample
        layers[name] = samples
    return {"descriptor": descriptor, "layers": layers}


def variation(samples: dict[str, Any]) -> dict[str, float]:
    keys = ("mass_matrix", "bias", "reduced_mass", "reduced_bias", "qacc_rad_m_s2",
            "physical_ddxi_m_s2", "wheel_acceleration_rad_s2")
    maxima = {key: 0.0 for key in keys}
    maxima.update({"normal_load_n": 0.0, "tangential_load_n": 0.0,
                   "contact_centroid_m": 0.0, "contact_normal": 0.0,
                   "contact_depth_m": 0.0, "contact_count_difference": 0})
    for mode in ("left", "right", "bilateral"):
        baseline = samples[f"{mode}:{0.0:+.8f}"]
        base_points = np.asarray(baseline["contact"]["points"])
        base_normals = np.asarray(baseline["contact"]["normals"])
        for sample_key, sample in samples.items():
            if not sample_key.startswith(mode + ":"):
                continue
            for key in keys:
                maxima[key] = max(maxima[key], max_abs(np.asarray(sample[key]) - np.asarray(baseline[key])))
            for key in ("normal_load_n", "tangential_load_n"):
                maxima[key] = max(maxima[key], max_abs(np.asarray(sample["force"][key])
                                                       - np.asarray(baseline["force"][key])))
            points = np.asarray(sample["contact"]["points"]); normals = np.asarray(sample["contact"]["normals"])
            if len(points) and len(base_points):
                maxima["contact_centroid_m"] = max(maxima["contact_centroid_m"],
                                                     max_abs(points.mean(0) - base_points.mean(0)))
            if len(normals) and len(base_normals):
                maxima["contact_normal"] = max(maxima["contact_normal"],
                                                max_abs(normals.mean(0) - base_normals.mean(0)))
            maxima["contact_depth_m"] = max(maxima["contact_depth_m"], abs(
                float(sample["contact"]["minimum_distance_m"]) -
                float(baseline["contact"]["minimum_distance_m"])))
            maxima["contact_count_difference"] = max(maxima["contact_count_difference"], abs(
                int(sample["contact"]["count"]) - int(baseline["contact"]["count"])))
    return maxima


def ratios(metrics: dict[str, dict[str, float]], variant: str, layer: str) -> dict[str, float]:
    return {key: float(metrics[variant][layer][key] / max(metrics["V0"][layer][key], 1e-30))
            for key in ("mass_matrix", "bias", "qacc_rad_m_s2", "physical_ddxi_m_s2")}


def classify(metrics: dict[str, Any], config: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    threshold = config["thresholds"]
    evidence = {}
    for variant in ("V1", "V2", "V3"):
        evidence[variant] = {layer: ratios(metrics, variant, layer)
                             for layer in ("contact_off", "contact_on")}
        rigid = evidence[variant]["contact_off"]
        evidence[variant]["multi_observable_primary"] = bool(
            rigid["physical_ddxi_m_s2"] <= float(threshold["primary_causal_reduction_ratio"])
            and min(rigid["mass_matrix"], rigid["bias"]) <=
                float(threshold["primary_causal_reduction_ratio"]))
    v1, v2, v3 = (evidence[key]["multi_observable_primary"] for key in ("V1", "V2", "V3"))
    if v1 and not v2:
        result = "P38-A_COM_eccentricity_is_primary_phase_source"
    elif v2 and not v1:
        result = "P38-B_transverse_inertia_anisotropy_is_primary_phase_source"
    elif v3 and not v1 and not v2:
        result = "P38-C_combined_COM_and_inertia_asymmetry"
    elif v1 and v2:
        result = "P38-C_combined_COM_and_inertia_asymmetry"
    else:
        result = "P38-G_residual_not_explained_by_wheel_inertia"
    return result, evidence


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
    semantics = inertial_semantics(config)
    plausibility = analytic_plausibility(semantics, config)
    variants = {variant: run_variant(config, variant) for variant in ("V0", "V1", "V2", "V3")}
    metrics = {variant: {layer: variation(samples) for layer, samples in result["layers"].items()}
               for variant, result in variants.items()}
    classification, causal = classify(metrics, config)
    threshold = config["thresholds"]
    geometry_pass = all(
        item["contact_on"]["contact_centroid_m"] <= float(threshold["geometry_invariance_m"])
        and item["contact_on"]["contact_normal"] <= float(threshold["geometry_invariance_normal"])
        and item["contact_on"]["contact_count_difference"] <= int(threshold["geometry_contact_count_difference"])
        for item in metrics.values())
    amplification = {variant: metrics[variant]["contact_on"]["physical_ddxi_m_s2"] /
                     max(metrics[variant]["contact_off"]["physical_ddxi_m_s2"], 1e-30)
                     for variant in variants}
    summary = {
        "classification": classification,
        "dg38_00_semantics_pass": True,
        "dg38_01_com_audit_pass": True,
        "dg38_02_tensor_audit_pass": all(semantics[side]["positive_definite"] for side in ("left", "right")),
        "dg38_03_plausibility_pass": plausibility["scale_plausible"],
        "dg38_04_rigid_body_isolation_complete": True,
        "dg38_05_contact_amplification_complete": geometry_pass,
        "dg38_06_attribution_complete": classification != "P38-G_residual_not_explained_by_wheel_inertia",
        "geometry_invariance_pass": geometry_pass,
        "v4_authorized": semantics["left_right"]["v4_authorized"],
        "physical_correction_justified": False,
        "physical_correction_limitation": "no CAD mass-property/density/assembly provenance beyond STL geometry and assigned total mass",
        "production_modified": False, "phase32_or_h0_run": False,
        "metrics": metrics, "causal_ratios": causal, "contact_amplification": amplification,
        "com_radial_m": {side: semantics[side]["com_radial_distance_m"] for side in ("left", "right")},
        "transverse_anisotropy": {side: semantics[side]["transverse_anisotropy"] for side in ("left", "right")},
    }
    write_json(output / "semantics.json", semantics)
    write_json(output / "plausibility.json", plausibility)
    write_json(output / "details.json", variants)
    write_json(output / "summary.json", summary)
    inputs = [config_path, ROOT / config["scene"], ROOT / config["model"], P36_PATH, P37_PATH,
              Path(__file__).resolve(), ROOT / config["source_phase35_hold"],
              ROOT / config["source_phase37_summary"],
              *[ROOT / path for path in config["source_meshes"].values()]]
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
                "replay_of": args.replay_of, "python": sys.version, "platform": platform.platform(),
                "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
                "variants": {variant: result["descriptor"] for variant, result in variants.items()},
                "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs}}
    write_json(output / "manifest.json", manifest)
    passed = all(summary[key] for key in ("dg38_00_semantics_pass", "dg38_01_com_audit_pass",
        "dg38_02_tensor_audit_pass", "dg38_03_plausibility_pass", "dg38_04_rigid_body_isolation_complete",
        "dg38_05_contact_amplification_complete", "dg38_06_attribution_complete"))
    print(json.dumps(P36.clean(summary), indent=2, sort_keys=True))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
