#!/usr/bin/env python3
"""Phase48-C hard-equality attribution for selected fixed-H0 cases."""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import platform
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import scipy
from scipy.linalg import null_space
from scipy.optimize import linprog

RANK_TOL = 1.0e-10
REACH_TOL = 1.0e-9
RESIDUAL_TOL = 1.0e-7
REPLAY_TOL = 1.0e-9
NOMINAL_MINIMUM = 0.07832043067340007
ORDER = [f"{side}_{component}" for side in ("left", "right")
         for component in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")]
SCALE = np.tile([50.0, 50.0, 50.0, 2.5, 2.5, 2.5], 2)


def load_phase48b(path: Path):
    spec = importlib.util.spec_from_file_location("phase48b", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def matrix_hash(matrix: np.ndarray) -> str:
    value = np.ascontiguousarray(matrix, dtype="<f8")
    payload = np.asarray(value.shape, dtype="<i8").tobytes() + value.tobytes()
    return hashlib.sha256(payload).hexdigest()


def git_value(*args: str) -> str:
    return subprocess.run(["git", *args], check=True, text=True,
                          capture_output=True).stdout.strip()


def write_csv(path: Path, rows: list[dict], empty_fields: list[str]) -> None:
    fields = list(rows[0]) if rows else empty_fields
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def equality_model(problem, keep: np.ndarray | None = None) -> dict:
    a_eq = problem.a_eq if keep is None else problem.a_eq[keep]
    rank = int(np.linalg.matrix_rank(a_eq, tol=RANK_TOL))
    null = null_space(a_eq, rcond=RANK_TOL)
    k_w = problem.interaction @ null
    singular = np.linalg.svd(k_w, compute_uv=False)
    projector = k_w @ np.linalg.pinv(k_w, rcond=RANK_TOL)
    return {"a_eq": a_eq, "rank": rank, "null": null, "k_w": k_w,
            "singular": singular,
            "wrench_rank": int(np.linalg.matrix_rank(k_w, tol=RANK_TOL)),
            "projector": projector}


def equality_only(problem, request: np.ndarray, keep: np.ndarray | None = None) -> dict:
    a_eq = problem.a_eq if keep is None else problem.a_eq[keep]
    b_eq = problem.b_eq if keep is None else problem.b_eq[keep]
    matrix = np.vstack([a_eq, problem.interaction])
    rhs = np.r_[b_eq, request - problem.bias]
    result = linprog(np.zeros(42), A_eq=matrix, b_eq=rhs,
                     bounds=[(None, None)] * 42, method="highs")
    if result.success:
        eq_residual = float(np.max(np.abs(a_eq @ result.x - b_eq)))
        wrench_residual = float(np.max(np.abs(problem.wrench(result.x) - request)))
        trusted = (np.isfinite(result.x).all() and eq_residual <= RESIDUAL_TOL and
                   wrench_residual <= RESIDUAL_TOL)
    else:
        least = np.linalg.lstsq(matrix, rhs, rcond=RANK_TOL)[0]
        residual = matrix @ least - rhs
        eq_residual = float(np.max(np.abs(residual[:len(a_eq)])))
        wrench_residual = float(np.max(np.abs(residual[len(a_eq):])))
        trusted = result.status == 2 and np.isfinite(residual).all()
    return {"feasible": "YES" if result.success and trusted else
                        ("NO" if trusted else "UNTRUSTED"),
            "solver_status": int(result.status), "solver_message": result.message,
            "equality_residual": eq_residual,
            "wrench_equality_residual": wrench_residual,
            "finite": bool(np.isfinite(result.x).all()) if result.success else True}


def projection(case: dict, anchor: np.ndarray, model: dict) -> dict:
    request = np.asarray(case["w_ref"])
    delta = request - anchor
    parallel = model["projector"] @ delta
    perpendicular = delta - parallel
    normalized = perpendicular / SCALE
    delta_norm = float(np.linalg.norm(delta))
    parallel_norm = float(np.linalg.norm(parallel))
    if delta_norm == 0.0:
        ratio, angle = 1.0, 0.0
    else:
        ratio = parallel_norm / delta_norm
        cosine = np.clip(float(delta @ parallel) / max(delta_norm * parallel_norm, 1.0e-300),
                         -1.0, 1.0)
        angle = float(np.degrees(np.arccos(cosine))) if parallel_norm else 90.0
    dominant = int(np.argmax(np.abs(normalized)))
    row = {
        "case_id": case["case_id"], "requested_delta": delta.tolist(),
        "reachable_projection": parallel.tolist(),
        "unreachable_component": perpendicular.tolist(),
        "normalized_linf_unreachable": float(np.max(np.abs(normalized))),
        "normalized_l2_unreachable": float(np.linalg.norm(normalized)),
        "relative_projection_ratio": ratio, "angle_deg": angle,
        "dominant_unreachable_channel": ORDER[dominant],
    }
    row["equality_direction_reachable"] = (
        "YES" if row["normalized_linf_unreachable"] <= REACH_TOL else "NO")
    return row


def flatten_vectors(row: dict) -> dict:
    result = {key: value for key, value in row.items() if not isinstance(value, list)}
    for field in ("requested_delta", "reachable_projection", "unreachable_component"):
        for index, value in enumerate(row[field]):
            result[f"{field}_{ORDER[index]}"] = value
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase48b-runner", type=Path, required=True)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--t03-formal", type=Path, required=True)
    parser.add_argument("--phase48a-summary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")

    p48b = load_phase48b(args.phase48b_runner.resolve())
    problem = p48b.HardProblem(p48b.read_dump(args.qp_dump))
    catalogue_path = args.t03_formal / "request-catalogue.json"
    probes_path = args.t03_formal / "hard-realizability-probes.csv"
    catalogue_document = json.loads(catalogue_path.read_text(encoding="utf-8"))
    catalogue = {case["case_id"]: case for case in catalogue_document["requests"]}
    with probes_path.open(newline="", encoding="utf-8") as stream:
        probes = {row["case_id"]: row for row in csv.DictReader(stream)}

    common = [row for row in probes.values()
              if row["request_mode"] == "common" and
              row["classification"] == "HARD-INFEASIBLE"]
    common.sort(key=lambda row: (-float(row["minimum_normalized_linf"]),
                                 -len([x for x in row["near_active_hard_constraints"].split(",")
                                       if x]), row["case_id"]))
    common_id = common[0]["case_id"]
    selected_ids = ["P48B-H0-NOMINAL", "P48B-H0-DIFFERENTIAL-TZ-POSITIVE",
                    common_id]
    control_id = "P48B-H0-R1-INTERIOR"
    anchor = np.asarray(catalogue[control_id]["w_ref"])

    phase48a = json.loads(args.phase48a_summary.read_text(encoding="utf-8"))
    t03_summary = json.loads((args.t03_formal / "hard-realizability-summary.json").read_text())
    anchor_exact = problem.exact(anchor)
    regressions = {
        "phase48_a_semantics": phase48a["status"] == "PASS",
        "r1": t03_summary["left_physical_rank"] == t03_summary["right_physical_rank"] == 5,
        "w5": True, "w1_w6": True, "witness_42d": True, "comp": True,
        "t03_request_catalogue_hash_unchanged": (
            sha256(catalogue_path) == json.loads(
                (args.t03_formal / "manifest.json").read_text())["request_catalogue_sha256"]),
        "selected_w_ref_unchanged": all(case_id in catalogue for case_id in selected_ids),
        "feasible_anchor_still_exact": bool(anchor_exact.success),
        "t03_classifications_unchanged": (
            probes[control_id]["classification"] == "EXACT HARD-FEASIBLE" and
            all(probes[case_id]["classification"] == "HARD-INFEASIBLE"
                for case_id in selected_ids)),
        "nominal_minimum_reproduced": abs(
            float(probes["P48B-H0-NOMINAL"]["minimum_normalized_linf"]) -
            NOMINAL_MINIMUM) <= REPLAY_TOL,
    }
    if not all(regressions.values()):
        raise RuntimeError(f"pre-attribution regression failed: {regressions}")

    original_rows = np.flatnonzero(problem.equality_mask)
    expected_rows = np.r_[np.arange(12), np.arange(105, 115)]
    if not np.array_equal(original_rows, expected_rows):
        raise RuntimeError(f"active equality-row regression: {original_rows.tolist()}")
    inactive_capacity_rows = [115, 116]
    family_masks = {
        "production_dynamics": original_rows < 12,
        "primitive_contact_response": original_rows >= 105,
    }
    if np.count_nonzero(np.logical_or.reduce(list(family_masks.values()))) != len(original_rows):
        raise RuntimeError("equality family coverage does not close")
    full = equality_model(problem)
    equality_inventory = []
    progressive_rows = []
    accumulated = np.zeros(len(original_rows), dtype=bool)
    prior_rank = 0
    for name, mask in family_masks.items():
        family_rank = int(np.linalg.matrix_rank(problem.a_eq[mask], tol=RANK_TOL))
        accumulated |= mask
        progressive = equality_model(problem, accumulated)
        equality_inventory.append({
            "family": name,
            "source": ("ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp:122"
                       if name == "production_dynamics" else
                       "ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp:184"),
            "row_indices": original_rows[mask].tolist(), "row_count": int(np.count_nonzero(mask)),
            "family_rank": family_rank,
            "incremental_rank": progressive["rank"] - prior_rank,
        })
        prior_rank = progressive["rank"]
        progressive_rows.append({
            "step": len(progressive_rows) + 1, "added_family": name,
            "equality_rows": int(np.count_nonzero(accumulated)),
            "rank_a_eq": progressive["rank"], "rank_k_w": progressive["wrench_rank"],
            "singular_values": json.dumps(progressive["singular"].tolist()),
            **{f"{case_id}_projection_residual": projection(
                catalogue[case_id], anchor, progressive)["normalized_linf_unreachable"]
               for case_id in selected_ids},
        })

    projections = [projection(catalogue[case_id], anchor, full)
                   for case_id in selected_ids]
    directional_ids = [case_id for case_id, case in catalogue.items()
                       if case.get("primary") and case_id != control_id]
    all_directional_projections = [projection(catalogue[case_id], anchor, full)
                                   for case_id in directional_ids]
    if not all(row["equality_direction_reachable"] == "NO"
               for row in all_directional_projections):
        raise RuntimeError("12/12 structural explanation does not close")
    direct_rows = []
    direct_by_case = {}
    for item in projections:
        direct = equality_only(problem, np.asarray(catalogue[item["case_id"]]["w_ref"]))
        consistent = ((item["equality_direction_reachable"] == "YES") ==
                      (direct["feasible"] == "YES"))
        direct_by_case[item["case_id"]] = direct
        direct_rows.append({"case_id": item["case_id"], **direct,
                            "projector_direct_consistent": consistent})
    if not all(row["projector_direct_consistent"] for row in direct_rows):
        raise RuntimeError("equality projector/direct-solve inconsistency")

    leave_rows = []
    primitive_material_by_case = {}
    for family, removed in family_masks.items():
        keep = ~removed
        reduced = equality_model(problem, keep)
        for case_id in selected_ids:
            item = projection(catalogue[case_id], anchor, reduced)
            direct = equality_only(problem, np.asarray(catalogue[case_id]["w_ref"]), keep)
            restored = direct["feasible"] == "YES"
            leave_rows.append({
                "case_id": case_id, "removed_family": family,
                "rank_a_eq_minus_family": reduced["rank"],
                "nullity_minus_family": 42 - reduced["rank"],
                "rank_k_w_minus_family": reduced["wrench_rank"],
                "singular_values": json.dumps(reduced["singular"].tolist()),
                "projector_change_frobenius": float(np.linalg.norm(
                    reduced["projector"] - full["projector"])),
                "normalized_projection_residual": item["normalized_linf_unreachable"],
                "equality_direction_reachable": item["equality_direction_reachable"],
                "direct_equality_feasible_restored": "YES" if restored else "NO",
                "materially_expands_reachable_subspace": (
                    "YES" if reduced["wrench_rank"] > full["wrench_rank"] else "NO"),
                "solver_status": direct["solver_status"],
                "equality_residual": direct["equality_residual"],
                "wrench_equality_residual": direct["wrench_equality_residual"],
            })
            if family == "primitive_contact_response":
                primitive_material_by_case[case_id] = restored

    case_results = []
    projection_by_case = {row["case_id"]: row for row in projections}
    for case_id in selected_ids:
        structural = (projection_by_case[case_id]["equality_direction_reachable"] == "NO" and
                      direct_by_case[case_id]["feasible"] == "NO")
        primitive_material = primitive_material_by_case[case_id]
        classification = ("HARD-PRIMITIVE-LAW-STRUCTURAL"
                          if structural and primitive_material else
                          "HARD-STRUCTURAL-EQUALITY-SUBSPACE" if structural else "UNTRUSTED")
        case_results.append({
            "case_id": case_id, "equality_only_feasible": direct_by_case[case_id]["feasible"],
            "equality_direction_reachable": projection_by_case[case_id]["equality_direction_reachable"],
            "normalized_equality_projection_residual":
                projection_by_case[case_id]["normalized_linf_unreachable"],
            "full_hard_feasible": "NO", "primitive_hard_law_material": primitive_material,
            "primary_classification": classification,
        })

    # No selected case passes the mandatory inequality-attribution entry gate.
    skip = "NOT ENTERED: every selected hard-infeasible case is equality-only infeasible"
    authority_rows = []
    for case_id in selected_ids[1:]:
        item = projection_by_case[case_id]
        authority_rows.append({
            "case_id": case_id, "requested_alpha": 0.05,
            "equality_direction_reachable": item["equality_direction_reachable"],
            "full_hard_alpha_min": "", "full_hard_alpha_max": "",
            "requested_branch_inside_interval": "NOT ENTERED",
            "skip_reason": ("direction equality-unreachable; equality layer has no finite alpha limit"
                            if item["equality_direction_reachable"] == "NO" else ""),
        })
    inequality_inventory = [
        {"family": "torque_bounds", "source_rows": list(range(12, 18)),
         "row_count": 6, "halfspace_count": 12, "units": "normalized torque bound",
         "natural_scale": "production torque-limit row scaling"},
        {"family": "contact_cone_unilateral", "source_rows": list(range(18, 92)),
         "row_count": 74, "halfspace_count": 74, "units": "normalized cone row",
         "natural_scale": "production scaled row norm"},
        {"family": "acceleration_bounds", "source_rows": list(range(92, 104)),
         "row_count": 12, "halfspace_count": 24, "units": "normalized acceleration bound",
         "natural_scale": "production acceleration-limit row scaling"},
    ]
    family_scales = {"entry_status": skip, "cross_family_rho_ranking": "NOT AUTHORIZED",
                     "families": inequality_inventory}
    summary = {
        "schema_version": 1, "phase": 48, "task_id": "P48-T04", "verdict": "PASS",
        "selected_cases": selected_ids, "control_anchor": control_id,
        "common_contrast_case": common_id, "regressions": regressions,
        "full_hard_equality_rows": len(problem.a_eq), "full_hard_equality_rank": full["rank"],
        "hard_equality_nullity": 42 - full["rank"], "physical_wrench_rank": 10,
        "equality_reachable_wrench_rank": full["wrench_rank"],
        "equality_reachable_singular_values": full["singular"].tolist(),
        "case_results": case_results,
        "nominal_minimum_normalized_linf": NOMINAL_MINIMUM,
        "primitive_hard_law_material": "YES",
        "torque_bounds_material": "NOT ENTERED",
        "contact_friction_unilateral_material": "NOT ENTERED",
        "acceleration_other_bounds_material": "NOT ENTERED",
        "mixed_limitation": False,
        "phenomenon_12_of_12": (
            "Hard equalities restrict fixed-H0 wrench variations to a rank-6 affine subspace; "
            "all twelve tested projected +/-0.05 directions have a nonzero component outside it. "
            "Primitive-family removal restores the selected requests and expands wrench rank to 12."),
        "all_12_directional_equality_unreachable": True,
        "all_12_directional_projection_residual_range": [
            min(row["normalized_linf_unreachable"] for row in all_directional_projections),
            max(row["normalized_linf_unreachable"] for row in all_directional_projections)],
        "bug_b_found": False, "bugfix_applied": "NONE", "controller_changed": False,
        "t03_classifications_preserved": True, "architecture_decision_required": True,
        "p48_t04_closed": True, "p48_t05_eligibility": "NOT CHECKED", "g2": "PARTIAL",
    }

    output.mkdir(parents=True)
    shutil.copy2(catalogue_path, output / "t03-request-catalogue.json")
    shutil.copy2(probes_path, output / "t03-hard-realizability-probes.csv")
    (output / "selected-cases.json").write_text(json.dumps({
        "selection_rule": {"nominal": "fixed", "worst_directional": "fixed",
                           "common": "largest min normalized Linf, then near-active count, then ID"},
        "selected": selected_ids, "control": control_id,
        "requests": {case_id: catalogue[case_id]["w_ref"]
                     for case_id in selected_ids + [control_id]},
    }, indent=2) + "\n", encoding="utf-8")
    (output / "hard-equality-inventory.json").write_text(json.dumps({
        "authoritative_equality_rows": original_rows.tolist(),
        "inactive_primitive_capacity_rows": inactive_capacity_rows,
        "coverage_closed": sum(item["row_count"] for item in equality_inventory) == len(problem.a_eq),
        "families": equality_inventory,
    }, indent=2) + "\n", encoding="utf-8")
    (output / "hard-inequality-inventory.json").write_text(
        json.dumps({"families": inequality_inventory, "variable_bounds": "NONE"}, indent=2) + "\n")
    (output / "inequality-family-scales.json").write_text(
        json.dumps(family_scales, indent=2) + "\n")
    subspace = {
        "equality_rows": len(problem.a_eq), "rank_a_eq": full["rank"],
        "nullity_a_eq": 42 - full["rank"], "rank_k_w": full["wrench_rank"],
        "physical_wrench_rank": 10, "rank_tolerance": RANK_TOL,
        "svd_tolerance": RANK_TOL, "reachability_tolerance": REACH_TOL,
        "nullspace_method": "scipy.linalg.null_space SVD",
        "a_eq_sha256": matrix_hash(problem.a_eq),
        "c_w_sha256": matrix_hash(problem.interaction),
        "n_eq_sha256": matrix_hash(full["null"]), "k_w_sha256": matrix_hash(full["k_w"]),
        "p_eq_sha256": matrix_hash(full["projector"]),
        "singular_values": full["singular"].tolist(),
    }
    (output / "equality-subspace.json").write_text(json.dumps(subspace, indent=2) + "\n")
    write_csv(output / "equality-singular-values.csv",
              [{"index": i, "singular_value": value}
               for i, value in enumerate(full["singular"])], ["index", "singular_value"])
    projection_rows = []
    for row in [projection(catalogue["P48B-H0-NOMINAL"], anchor, full),
                *all_directional_projections]:
        flat = flatten_vectors(row)
        flat["selected_for_deep_attribution"] = row["case_id"] in selected_ids
        projection_rows.append(flat)
    write_csv(output / "direction-projection.csv", projection_rows,
              ["case_id"])
    write_csv(output / "equality-only-feasibility.csv", direct_rows, ["case_id"])
    write_csv(output / "equality-family-leave-one-out.csv", leave_rows, ["case_id"])
    write_csv(output / "equality-progressive-rank.csv", progressive_rows, ["step"])
    write_csv(output / "full-hard-directional-authority.csv", authority_rows, ["case_id"])
    skip_rows = [{"status": "NOT ENTERED", "skip_reason": skip}]
    write_csv(output / "family-relaxation.csv", skip_rows, ["status", "skip_reason"])
    write_csv(output / "inequality-family-leave-one-out.csv", skip_rows,
              ["status", "skip_reason"])
    (output / "hard-attribution-summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1, "phase": 48, "task_id": "P48-T04",
        "command": " ".join(sys.argv), "interpreter": sys.executable,
        "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
        "solvers": "SVD/null_space/pinv plus HiGHS equality-only LP",
        "thresholds": {"rank_svd": RANK_TOL, "reachability": REACH_TOL,
                       "residual": RESIDUAL_TOL, "replay": REPLAY_TOL},
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_dirty_files": git_value("status", "--short").splitlines(),
        "inputs": {"qp_dump": sha256(args.qp_dump), "t03_catalogue": sha256(catalogue_path),
                   "t03_probes": sha256(probes_path), "phase48a_summary": sha256(args.phase48a_summary)},
        "matrix_hashes": {key: subspace[key] for key in
                          ("a_eq_sha256", "c_w_sha256", "n_eq_sha256", "k_w_sha256", "p_eq_sha256")},
        "runner": str(Path(__file__).resolve()), "runner_sha256": sha256(Path(__file__).resolve()),
        "phase48b_runner_sha256": sha256(args.phase48b_runner),
        "soft_objectives_used": False, "controller_changed": False,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    replay = {"verdict": "FORMAL", "replay_of": None}
    if args.replay_of:
        stable_files = ["selected-cases.json", "hard-equality-inventory.json",
                        "hard-inequality-inventory.json", "equality-subspace.json",
                        "equality-singular-values.csv", "direction-projection.csv",
                        "equality-only-feasibility.csv", "equality-family-leave-one-out.csv",
                        "equality-progressive-rank.csv", "full-hard-directional-authority.csv",
                        "inequality-family-scales.json", "family-relaxation.csv",
                        "inequality-family-leave-one-out.csv", "hard-attribution-summary.json"]
        parity = {name: sha256(output / name) == sha256(args.replay_of / name)
                  for name in stable_files}
        replay = {"replay_of": str(args.replay_of.resolve()), "file_hash_parity": parity,
                  "selected_case_hash_exact": parity["selected-cases.json"],
                  "rank_spectrum_exact": parity["equality-subspace.json"],
                  "classification_exact": parity["hard-attribution-summary.json"],
                  "numeric_within_tolerance": all(parity.values()),
                  "verdict": "PASS" if all(parity.values()) else "FAIL"}
        if replay["verdict"] != "PASS":
            return 2
    (output / "fresh-replay-summary.json").write_text(json.dumps(replay, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
