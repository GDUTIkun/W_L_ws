#!/usr/bin/env python3
"""Run only the Phase46 point-realizable COMP/EQ gates."""

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
from scipy.linalg import subspace_angles


ROOT = Path(__file__).resolve().parents[2]


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


EVAL = load(ROOT / "tools/experiments/run_phase46_point_realizable_repair.py",
            "p46_point_eval")
ATTR, P45C, P45, P44, P42 = EVAL.ATTR, EVAL.P45C, EVAL.P45, EVAL.P44, EVAL.P42


def skew(value: np.ndarray) -> np.ndarray:
    x, y, z = value
    return np.asarray([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def point_map(details: list[dict[str, Any]], side: int, geometry: dict[str, Any]) -> np.ndarray:
    blocks = []
    frame = np.asarray(geometry["frame"])
    reference = np.asarray(geometry["point"])
    points = sorted((row for row in details if int(row["side"]) == side),
                    key=lambda row: int(row["contact_index"]))
    for row in points:
        position = np.asarray([row[f"position_world_{index}"] for index in range(3)])
        lever = frame.T @ (position - reference)
        blocks.append(np.vstack((np.eye(3), skew(lever))))
    if len(blocks) != 2:
        raise RuntimeError(f"side {side} does not have exactly two actual points")
    return np.hstack(blocks)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    config_path = args.config.resolve()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)

    config = EVAL.read_json(config_path)
    continuation_path = ROOT / config["continuation_config"]
    continuation = EVAL.read_json(continuation_path)
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    control = P45.run(base, output / "baseline.csv", config["case_id"],
                      authority=authority, tick=0, delta=np.zeros(4),
                      wrench_trim=trim)[0]
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(EVAL.read_json(ROOT / base["phase42_config"]))
    actual = P45.actual(base, model, oracle, native, control)
    qp_output, mj_output = P45C.task_output(control, actual)
    geometry = ATTR.contact_geometry(
        model, actual["qpos"], actual["qvel"],
        float(oracle.config["canonical_wheel_radius_m"]))
    geometry_metrics = {"source": "frozen actual Model-B state",
                        "production_reference_and_frame": True}
    data = mujoco.MjData(model)
    data.qpos[:] = actual["qpos"]
    data.qvel[:] = actual["qvel"]
    mujoco.mj_forward(model, data)

    projector = []
    for side, item in enumerate(geometry):
        axis_world = data.ximat[item["body"]].reshape(3, 3)[:, 0]
        axis = item["frame"].T @ axis_world
        axis /= np.linalg.norm(axis)
        gp = point_map(actual["details"], side, item)
        u, singular, _ = np.linalg.svd(gp)
        rank = int(np.linalg.matrix_rank(gp, tol=1.0e-10))
        pg = u[:, :rank] @ u[:, :rank].T
        line_offset = item["frame"].T @ np.asarray([0.0, 0.0, -0.5 * item["point"][2]])
        production_gp = np.hstack([
            np.vstack((np.eye(3), skew(line_offset - 0.5 * axis))),
            np.vstack((np.eye(3), skew(line_offset + 0.5 * axis))),
        ])
        production_u, _, _ = np.linalg.svd(production_gp)
        production_pg = production_u[:, :5] @ production_u[:, :5].T
        p_to_g = (np.eye(6) - pg) @ production_pg
        g_to_p = (np.eye(6) - production_pg) @ pg
        angles = subspace_angles(production_u[:, :5], u[:, :rank])
        missing = u[:, rank]
        wrench = P44.vec(control, "physical_solution", 30)[18 + 6 * side:24 + 6 * side]
        force = np.linalg.pinv(gp, rcond=1.0e-10) @ wrench
        reconstruction = wrench - gp @ force
        projector.append({
            "side": ("left", "right")[side],
            "Gp": gp, "Pg": pg, "production_Pg": production_pg,
            "rank": rank, "singular_values": singular,
            "condition_nonzero": float(singular[0] / singular[rank - 1]),
            "symmetry_max_abs": float(np.max(np.abs(production_pg - production_pg.T))),
            "idempotence_max_abs": float(np.max(np.abs(production_pg @ production_pg - production_pg))),
            "projector_parity_max_abs": float(np.max(np.abs(production_pg - pg))),
            "containment_P_to_G_spectral": float(np.linalg.norm(p_to_g, 2)),
            "containment_G_to_P_spectral": float(np.linalg.norm(g_to_p, 2)),
            "maximum_principal_angle_rad": float(np.max(angles)),
            "actual_missing_direction": missing,
            "missing_direction_annihilation_max_abs": float(np.max(np.abs(production_pg @ missing))),
            "physical_wrench": wrench,
            "point_force_reconstruction": force,
            "reconstruction_residual": reconstruction,
            "reconstruction_residual_norm": float(np.linalg.norm(reconstruction)),
        })
    component_pass = (
        all(row["rank"] == 5 and row["symmetry_max_abs"] <= 1.0e-12 and
            row["idempotence_max_abs"] <= 1.0e-12 and
            row["projector_parity_max_abs"] <= 1.0e-10 and
            row["containment_P_to_G_spectral"] <= 1.0e-10 and
            row["containment_G_to_P_spectral"] <= 1.0e-10 and
            row["maximum_principal_angle_rad"] <= 1.0e-10 and
            row["missing_direction_annihilation_max_abs"] <= 1.0e-10 and
            row["reconstruction_residual_norm"] <= 1.0e-10 for row in projector))
    contact_count = [sum(int(row["side"]) == side for row in actual["details"])
                     for side in range(2)]
    metrics = {
        "component_pass": component_pass,
        "projector": projector,
        "qp_ddxi_per_side_m_s2": np.asarray(qp_output[:2]),
        "qp_material_tangent_acceleration_per_side_m_s2": np.asarray(qp_output[2:]),
        "mujoco_ddxi_per_side_m_s2": np.asarray(mj_output[:2]),
        "mujoco_material_tangent_acceleration_per_side_m_s2": np.asarray(mj_output[2:]),
        "contact_count_per_side": contact_count,
        "hard_violation": float(control["hard"]),
        "maximum_normalized_slack": float(control["maximum_normalized_slack"]),
        "minimum_torque_margin_nm": min(float(control[f"tau_margin{i}"]) for i in range(6)),
        "whole_dynamics_closure": float(actual["dynamics"]["full_dynamics_residual_max_abs"]),
        "contact_applyft_closure": float(actual["dynamics"]["contact_applyft_jacobian_max_abs"]),
        "geometry_reconstruction": geometry_metrics,
    }
    equilibrium_pass = (
        max(abs(value) for value in metrics["mujoco_ddxi_per_side_m_s2"]) <= 0.05 and
        max(abs(value) for value in metrics["mujoco_material_tangent_acceleration_per_side_m_s2"]) <= 0.01 and
        contact_count == [2, 2] and metrics["hard_violation"] <= 1.0e-7 and
        metrics["maximum_normalized_slack"] <= 0.05 and
        metrics["minimum_torque_margin_nm"] >= -1.0e-10 and
        metrics["whole_dynamics_closure"] <= 1.0e-8 and
        metrics["contact_applyft_closure"] <= 1.0e-8)
    decision = {
        "pass": component_pass and equilibrium_pass,
        "classification": "EXACT_R1_EQ_PASS" if equilibrium_pass else "EXACT_R1_EQ_FAIL",
        "mandatory_gate_stop": None if equilibrium_pass else "EXACT-R1-EQ",
        "exact_r1_implementation_pass": component_pass,
        "range_decision_equals_range_Gp": component_pass,
        "equilibrium_pass": equilibrium_pass,
        "metrics": metrics,
        "scope_contract": config["scope_contract"],
    }
    P45.write_json(output / "equilibrium-decision.json", decision)
    replay_error = None if args.replay_of is None else P45.semantic_error(
        args.replay_of / "equilibrium-decision.json", output / "equilibrium-decision.json")
    P45.write_json(output / "summary.json", {
        "pass": decision["pass"], "classification": decision["classification"],
        "replay_max_abs_error": replay_error,
        "replay_pass": replay_error is None or replay_error <= 1.0e-11,
    })
    sources = [config_path, continuation_path, ROOT / base["scene"],
               ROOT / base["executable"], authority, wrench_source,
               Path(__file__).resolve(), EVAL.__file__]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "sources": {str(Path(path).relative_to(ROOT)): hashlib.sha256(Path(path).read_bytes()).hexdigest()
                    for path in sources},
    })
    return 0 if decision["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
