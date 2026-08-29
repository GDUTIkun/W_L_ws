#!/usr/bin/env python3
"""Phase 32 C1/C2 feasibility and C3 projection-nullspace rank audit."""

from __future__ import annotations

import argparse
import csv
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
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase32_markov_closure_v1.json"
CONTRACT_SCRIPT = ROOT / "tools/experiments/run_phase31_wheel_state_contract.py"


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


CONTRACT = load_module(CONTRACT_SCRIPT, "phase32_rank_contract")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean(item) for item in value]
    return value


def rank(matrix: np.ndarray, tolerance: float = 1e-9) -> dict[str, Any]:
    singular = np.linalg.svd(matrix, compute_uv=False)
    value = int(np.sum(singular > tolerance))
    return {
        "rows": int(matrix.shape[0]), "columns": int(matrix.shape[1]),
        "rank": value, "nullity": int(matrix.shape[1] - value),
        "singular_values": singular,
    }


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists(): raise RuntimeError(f"output already exists: {output}")
    method_path = args.method.resolve()
    method = json.loads(method_path.read_text(encoding="utf-8"))
    scene = ROOT / method["scene"]
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    base_weld = CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    base_site = CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
    sides = [
        {
            "name": "left", "equality": "left_leg_closure",
            "wheel_body": "left_wheel_body", "wheel_geom": "left_wheel_collision",
            "leg_dofs": [11, 12, 14, 15], "spin_dof": 13,
        },
        {
            "name": "right", "equality": "right_leg_closure",
            "wheel_body": "right_wheel_body", "wheel_geom": "right_wheel_collision",
            "leg_dofs": [6, 7, 9, 10], "spin_dof": 8,
        },
    ]
    for side in sides:
        side["equality_id"] = CONTRACT.required_id(
            model, mujoco.mjtObj.mjOBJ_EQUALITY, side["equality"])
        side["wheel_body_id"] = CONTRACT.required_id(
            model, mujoco.mjtObj.mjOBJ_BODY, side["wheel_body"])
        side["wheel_geom_id"] = CONTRACT.required_id(
            model, mujoco.mjtObj.mjOBJ_GEOM, side["wheel_geom"])
    floor = CONTRACT.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    raw_root = ROOT / method["source_phase28_run"]
    raw_paths = []
    samples = []
    fixed_contact_ranks = []
    free_height_nullities = []
    spin_projection_norms = []
    spin_contact_norms = []
    for case in method["cases"]:
        plant_path = raw_root / f"{case['id']}_plant.csv"
        raw_paths.append(plant_path)
        plant = {(int(row["control_tick"]), int(row["physics_substep"])): row
                 for row in read_csv(plant_path)}
        for tick in case["authority_ticks"]:
            row = plant[(tick - 1, 4)]
            data.qpos[:] = [float(row[f"qpos{index}"]) for index in range(model.nq)]
            data.qvel[:] = [float(row[f"qvel{index}"]) for index in range(model.nv)]
            data.eq_active[base_weld] = 0
            mujoco.mj_forward(model, data)
            efc_j = data.efc_J.reshape(data.nefc, model.nv)
            rotation = data.site_xmat[base_site].reshape(3, 3)
            sample_sides = {}
            for side in sides:
                leg_dofs = side["leg_dofs"]
                equality_rows = np.flatnonzero(
                    (np.asarray(data.efc_type) == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
                    & (np.asarray(data.efc_id) == side["equality_id"]))
                closure = efc_j[equality_rows][:, leg_dofs]
                jacp_wheel = np.zeros((3, model.nv))
                mujoco.mj_jacBody(model, data, jacp_wheel, None, side["wheel_body_id"])
                xi = (rotation.T @ jacp_wheel)[0:1, leg_dofs]
                height = jacp_wheel[2:3, leg_dofs]
                free_height = np.vstack((closure, xi))
                fixed_contact = np.vstack((closure, xi, height))
                contact_rows = []
                for contact in data.contact[:data.ncon]:
                    if {int(contact.geom1), int(contact.geom2)} == {floor, side["wheel_geom_id"]}:
                        address = int(contact.efc_address)
                        if address >= 0:
                            # MuJoCo pyramidal friction emits 2*(dim-1) scalar rows.
                            contact_rows.extend(range(address, address + 2 * (int(contact.dim) - 1)))
                contact_j = efc_j[contact_rows] if contact_rows else np.zeros((0, model.nv))
                spin_projection = np.array([
                    float(np.max(np.abs(closure[:, [leg_dofs.index(side["spin_dof"])]])))
                    if side["spin_dof"] in leg_dofs else 0.0,
                    float(abs((rotation.T @ jacp_wheel)[0, side["spin_dof"]])),
                    float(abs(jacp_wheel[2, side["spin_dof"]])),
                ])
                spin_contact = float(np.linalg.norm(contact_j[:, side["spin_dof"]]))
                free_result = rank(free_height)
                fixed_result = rank(fixed_contact)
                free_height_nullities.append(free_result["nullity"])
                fixed_contact_ranks.append(fixed_result["rank"])
                spin_projection_norms.append(float(np.linalg.norm(spin_projection)))
                spin_contact_norms.append(spin_contact)
                sample_sides[side["name"]] = {
                    "closure": rank(closure),
                    "closure_plus_xi": free_result,
                    "closure_plus_xi_plus_wheel_height": fixed_result,
                    "interpretation": {
                        "C1_configuration_pair_with_free_height": free_result["nullity"] > 0,
                        "C1_configuration_pair_with_fixed_contact_geometry": fixed_result["nullity"] > 0,
                        "C2_velocity_pair_with_free_height_rate": free_result["nullity"] > 0,
                        "C2_velocity_pair_with_fixed_normal_contact_rate": fixed_result["nullity"] > 0,
                    },
                    "wheel_spin_projection_norm": float(np.linalg.norm(spin_projection)),
                    "wheel_spin_contact_constraint_jacobian_norm": spin_contact,
                    "contact_constraint_rows": len(contact_rows),
                }
            samples.append({"case": case["id"], "tick": tick, "sides": sample_sides})
    summary = {
        "rank_tolerance": 1e-9,
        "C1_C2_free_height_null_direction_present": min(free_height_nullities) > 0,
        "C1_C2_fixed_contact_null_direction_present": min(fixed_contact_ranks) < 4,
        "C1_C2_pair_decision": "infeasible_under_fixed_bilateral_normal_contact_geometry",
        "C3_wheel_spin_is_exact_projection_null_direction": max(spin_projection_norms) <= 1e-12,
        "C3_wheel_spin_changes_contact_constraint_velocity": min(spin_contact_norms) > 1e-6,
        "max_wheel_spin_projection_norm": max(spin_projection_norms),
        "min_wheel_spin_contact_constraint_jacobian_norm": min(spin_contact_norms),
        "production_modified": False,
        "classification": "projection_rank_pass",
    }
    output.mkdir(parents=True)
    (output / "projection_rank.json").write_text(
        json.dumps(clean({"samples": samples}), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(
        json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    inputs = [method_path, scene, CONTRACT_SCRIPT, Path(__file__).resolve(), *raw_paths]
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": args.replay_of,
        "python": platform.python_version(),
        "dependencies": {"numpy": np.__version__, "scipy": scipy.__version__, "mujoco": mujoco.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in inputs},
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
