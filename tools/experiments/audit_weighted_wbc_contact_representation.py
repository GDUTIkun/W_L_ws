#!/usr/bin/env python3
"""Audit single-point and mesh-support contact representations from a saved capture."""

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

import numpy as np
import scipy
from scipy.optimize import linprog


ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def load_config(path: Path) -> tuple[dict[str, Any], list[Path]]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if "extends" not in raw:
        return raw, [path]
    base, inputs = load_config((ROOT / raw["extends"]).resolve())
    merged = {**base, **{key: value for key, value in raw.items() if key != "extends"}}
    if "contact_representation_audit" in base or "contact_representation_audit" in raw:
        merged["contact_representation_audit"] = {
            **base.get("contact_representation_audit", {}),
            **raw.get("contact_representation_audit", {})}
    return merged, inputs + [path]


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def extreme_four(points: np.ndarray) -> np.ndarray:
    xy = points[:, :2]
    scores = (xy[:, 0] + xy[:, 1], xy[:, 0] - xy[:, 1],
              -xy[:, 0] + xy[:, 1], -xy[:, 0] - xy[:, 1])
    indices = list(dict.fromkeys(int(np.argmax(score)) for score in scores))
    return points[indices]


def wrench_residual(points: np.ndarray, center: np.ndarray, wrench: np.ndarray,
                    friction: float) -> tuple[float, bool]:
    count = len(points); mapping = np.zeros((6, 3 * count))
    for index, point in enumerate(points):
        mapping[:3, 3 * index:3 * index + 3] = np.eye(3)
        mapping[3:, 3 * index:3 * index + 3] = skew(point - center)
    # Minimize the infinity-norm wrench residual with pyramidal point-force friction.
    variables = 3 * count + 1; objective = np.zeros(variables); objective[-1] = 1.0
    inequalities = []; bounds = []
    for sign in (1.0, -1.0):
        block = np.zeros((6, variables)); block[:, :-1] = sign * mapping; block[:, -1] = -1.0
        inequalities.append(block); bounds.append(sign * wrench)
    friction_rows = []
    for index in range(count):
        for tangent in (0, 1):
            for sign in (1.0, -1.0):
                row = np.zeros(variables); row[3 * index + tangent] = sign
                row[3 * index + 2] = -friction; friction_rows.append(row)
    inequalities.append(np.asarray(friction_rows)); bounds.append(np.zeros(len(friction_rows)))
    variable_bounds = []
    for _ in range(count): variable_bounds.extend(((None, None), (None, None), (0.0, None)))
    variable_bounds.append((0.0, None))
    result = linprog(objective, A_ub=np.vstack(inequalities), b_ub=np.concatenate(bounds),
                     bounds=variable_bounds, method="highs",
                     options={"primal_feasibility_tolerance": 1e-9,
                              "dual_feasibility_tolerance": 1e-9})
    return (float(result.fun) if result.success else float("inf"), bool(result.success))


def summarize(values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=float)
    return {"median": float(np.median(data)), "p95": float(np.quantile(data, 0.95)),
            "maximum": float(np.max(data))}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(); source = args.capture_dir.resolve(); output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    config_path = args.config.resolve(); config, config_inputs = load_config(config_path)
    settings = config["contact_representation_audit"]
    capture_path = source / "capture.npz"; capture = np.load(capture_path)
    start, end = int(settings["analysis_start_tick"]), int(settings["analysis_end_tick"])
    band = float(settings["frozen_support_band_m"]); friction = float(settings["friction_coefficient"])
    minimum_load = float(settings["minimum_normal_load_n"])
    rows = []; metrics: dict[str, list[float]] = {name: [] for name in (
        "fixed_point_moment_residual", "support_centroid_moment_residual",
        "arbitrary_single_force_irreducible_moment", "support_centroid_cop_error",
        "full_patch_wrench_residual", "four_extreme_wrench_residual",
        "lowest_four_wrench_residual", "lowest_eight_wrench_residual")}
    feasible = {name: [] for name in ("full_patch", "four_extreme", "lowest_four", "lowest_eight")}
    containment = []; sensitivity = {float(value): {"error": [], "containment": []}
                                     for value in settings["support_band_sensitivity_m"]}
    for tick in range(start, end + 1):
        for side in range(2):
            force = capture["truth_force"][tick, side]
            moment = capture["truth_moment_about_wheel"][tick, side]
            if force[2] < minimum_load:
                continue
            center = capture["wheel_center"][tick, side]
            vertices = capture["mesh_vertices_left" if side == 0 else "mesh_vertices_right"]
            world = capture["geom_position"][tick, side] + vertices @ capture["geom_rotation"][tick, side].T
            minimum_z = float(np.min(world[:, 2])); support = world[world[:, 2] <= minimum_z + band]
            four = extreme_four(support); order = np.argsort(world[:, 2])
            lowest = world[order[:4]]; lowest_eight = world[order[:8]]
            cop = capture["truth_cop"][tick, side]
            centroid = np.mean(support, axis=0)
            fixed_offset = capture["model_contact_points"][tick, side] - center
            fixed_residual = float(np.linalg.norm(moment - np.cross(fixed_offset, force)))
            centroid_residual = float(np.linalg.norm(moment - np.cross(centroid - center, force)))
            irreducible = float(abs(force @ moment) / np.linalg.norm(force))
            cop_error = float(np.linalg.norm(centroid[:2] - cop[:2]))
            low, high = np.min(support[:, :2], axis=0), np.max(support[:, :2], axis=0)
            inside = bool(np.all(cop[:2] >= low - 1e-12) and np.all(cop[:2] <= high + 1e-12))
            wrench = np.r_[force, moment]
            candidate_results = {}
            for name, points in (("full_patch", support), ("four_extreme", four),
                                 ("lowest_four", lowest), ("lowest_eight", lowest_eight)):
                residual, solved = wrench_residual(points, center, wrench, friction)
                candidate_results[name] = residual
                feasible[name].append(bool(solved and residual <= settings["maximum_wrench_equality_residual"]))
                metrics[f"{name}_wrench_residual"].append(residual)
            for sensitivity_band, values in sensitivity.items():
                selected = world[world[:, 2] <= minimum_z + sensitivity_band]
                selected_centroid = np.mean(selected, axis=0)
                selected_low, selected_high = np.min(selected[:, :2], axis=0), np.max(selected[:, :2], axis=0)
                values["error"].append(float(np.linalg.norm(selected_centroid[:2] - cop[:2])))
                values["containment"].append(bool(np.all(cop[:2] >= selected_low - 1e-12) and
                                                  np.all(cop[:2] <= selected_high + 1e-12)))
            metrics["fixed_point_moment_residual"].append(fixed_residual)
            metrics["support_centroid_moment_residual"].append(centroid_residual)
            metrics["arbitrary_single_force_irreducible_moment"].append(irreducible)
            metrics["support_centroid_cop_error"].append(cop_error); containment.append(inside)
            rows.append({"tick": tick, "side": "left" if side == 0 else "right",
                         "normal_load_n": force[2], "support_vertex_count": len(support),
                         "support_centroid_cop_error_m": cop_error,
                         "truth_cop_inside_support_bounds": inside,
                         "fixed_point_moment_residual_nm": fixed_residual,
                         "support_centroid_moment_residual_nm": centroid_residual,
                         "arbitrary_single_force_irreducible_moment_nm": irreducible,
                         **{f"{name}_wrench_residual": value for name, value in candidate_results.items()}})
    with (output / "ticks.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    metric_summary = {name: summarize(values) for name, values in metrics.items()}
    feasible_fraction = {name: float(np.mean(values)) for name, values in feasible.items()}
    sensitivity_summary = {str(value): {"centroid_cop_error_m": summarize(values["error"]),
                                        "cop_containment_fraction": float(np.mean(values["containment"]))}
                           for value, values in sensitivity.items()}
    gates = {"lowest_eight_representability": feasible_fraction["lowest_eight"] >= settings["minimum_lowest_eight_feasible_fraction"],
             "full_patch_representability": feasible_fraction["full_patch"] >= settings["minimum_full_patch_feasible_fraction"],
             "four_extreme_representability": feasible_fraction["four_extreme"] >= settings["minimum_four_extreme_feasible_fraction"]}
    summary = {"schema_version": 1, "phase": 21, "purpose": "contact_representation_decision_audit",
               "source_capture": str(source), "analysis_ticks": [start, end], "sample_count": len(rows),
               "frozen_support_band_m": band, "support_cop_containment_fraction": float(np.mean(containment)),
               "metrics": metric_summary, "candidate_feasible_fraction": feasible_fraction,
               "support_band_sensitivity": sensitivity_summary, "gates": gates,
               "pass": all(gates.values()),
               "interpretation_limit": "MuJoCo contact truth is validation-only and is not a controller input or online fallback"}
    write_json(output / "summary.json", summary)
    write_json(output / "manifest.json", {"schema_version": 1,
        "created_at": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(),
        "numpy": np.__version__, "scipy": scipy.__version__,
        "config": str(config_path),
        "config_inputs": {str(path.relative_to(ROOT)): sha256(path) for path in config_inputs},
        "capture": str(capture_path), "capture_sha256": sha256(capture_path),
        "capture_manifest_sha256": sha256(source / "manifest.json"),
        "validator": str(Path(__file__).resolve()), "validator_sha256": sha256(Path(__file__).resolve()),
        "outputs": {"summary.json": sha256(output / "summary.json"),
                    "ticks.csv": sha256(output / "ticks.csv")}})
    print(json.dumps(summary, indent=2, sort_keys=True)); return 0


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
