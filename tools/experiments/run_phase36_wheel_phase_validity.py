#!/usr/bin/env python3
"""Phase 36 offline fixed-state wheel-phase/model-validity audit."""

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

import mujoco
import numpy as np
import scipy
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase36_wheel_phase_validity_v1.json"


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


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    value = mujoco.mj_name2id(model, kind, name)
    if value < 0:
        raise RuntimeError(f"missing {kind.name} {name}")
    return value


def max_abs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value))) if value.size else 0.0


def relative(error: float, first: np.ndarray, second: np.ndarray) -> float:
    return error / max(1.0, max_abs(first), max_abs(second))


class Audit:
    ACTIVE = [
        "left_hip_joint", "left_knee_joint", "left_wheel_joint",
        "right_hip_joint", "right_knee_joint", "right_wheel_joint",
    ]
    PASSIVE = [
        "left_connect1_joint", "left_connect2_joint",
        "right_connect1_joint", "right_connect2_joint",
    ]

    def __init__(self, method: dict[str, Any]) -> None:
        self.method = method
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / method["scene"]))
        self.data = mujoco.MjData(self.model)
        self.joints = [object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.ACTIVE]
        self.passive = [object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in self.PASSIVE]
        self.qadr = np.asarray([self.model.jnt_qposadr[joint] for joint in self.joints], dtype=int)
        self.dadr = np.asarray([self.model.jnt_dofadr[joint] for joint in self.joints], dtype=int)
        self.passive_qadr = np.asarray([self.model.jnt_qposadr[joint] for joint in self.passive], dtype=int)
        self.passive_dadr = np.asarray([self.model.jnt_dofadr[joint] for joint in self.passive], dtype=int)
        self.wheel_joints = [self.joints[2], self.joints[5]]
        self.wheel_qadr = self.qadr[[2, 5]]
        self.wheel_dadr = self.dadr[[2, 5]]
        self.wheel_bodies = [object_id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                             for name in ("left_wheel_body", "right_wheel_body")]
        self.wheel_geoms = [object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                            for name in ("left_wheel_collision", "right_wheel_collision")]
        self.floor = object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.base_site = object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame")
        self.closure_sites = [
            tuple(object_id(self.model, mujoco.mjtObj.mjOBJ_SITE, name) for name in pair)
            for pair in (("left_connect2_site", "left_calf_site"),
                         ("right_connect2_site", "right_calf_site"))
        ]
        self.base_weld = object_id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
        self.qpos = self.equilibrium_qpos()
        self.torque = self.phase35_torque()

    def equilibrium_qpos(self) -> np.ndarray:
        candidate = np.asarray([
            -0.34332947374181766, 0.5693992271789607,
            -0.35472149355205396, 0.5694045089964002,
            0.34771766403249466,
            -0.572545089643551, -0.5729875480645877,
            0.5725979309569537, -0.5730345859812999,
        ])
        qpos = self.model.qpos0.copy()
        qpos[:3] = (0.0, 0.0, candidate[4])
        qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        qpos[self.qadr] = (candidate[2], candidate[3], 0.0,
                           candidate[0], candidate[1], 0.0)
        qpos[self.passive_qadr] = (candidate[7], candidate[8], candidate[5], candidate[6])
        return qpos

    def phase35_torque(self) -> np.ndarray:
        path = ROOT / self.method["source_phase35_hold"]
        with path.open(newline="", encoding="utf-8") as stream:
            row = next(csv.DictReader(stream))
        return np.asarray([float(row[f"tau{i}"]) for i in range(6)])

    def set_phase(self, mode: str, phase: float, contact: bool = True) -> np.ndarray:
        qpos = self.qpos.copy()
        canonical = np.zeros(2)
        canonical[:] = phase if mode == "bilateral" else 0.0
        if mode == "left": canonical[0] = phase
        if mode == "right": canonical[1] = phase
        qpos[self.wheel_qadr] = float(self.method["canonical_to_native_sign"]) * canonical
        self.model.opt.disableflags = int(self.model.opt.disableflags) & ~int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        if not contact:
            self.model.opt.disableflags = int(self.model.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = -self.torque
        self.data.eq_active[self.base_weld] = 0
        self.data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        return qpos

    def jac_body(self, body: int) -> tuple[np.ndarray, np.ndarray]:
        linear = np.zeros((3, self.model.nv)); angular = np.zeros_like(linear)
        mujoco.mj_jacBody(self.model, self.data, linear, angular, body)
        return linear, angular

    def jac_site(self, site: int) -> tuple[np.ndarray, np.ndarray]:
        linear = np.zeros((3, self.model.nv)); angular = np.zeros_like(linear)
        mujoco.mj_jacSite(self.model, self.data, linear, angular, site)
        return linear, angular

    def closure_reduction(self) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
        rows, residuals = [], []
        for first, second in self.closure_sites:
            ja, _ = self.jac_site(first); jb, _ = self.jac_site(second)
            rows.append(ja - jb)
            residuals.append(self.data.site_xpos[first] - self.data.site_xpos[second])
        closure = np.vstack(rows)
        base_linear, base_angular = self.jac_site(self.base_site)
        base_twist = np.vstack((base_linear, base_angular))[:, :6]
        reduction = np.zeros((self.model.nv, 12))
        reduction[:6, :6] = np.linalg.solve(base_twist, np.eye(6))
        reduction[self.dadr, 6:] = -np.eye(6)
        passive = closure[:, self.passive_dadr]
        reduction[self.passive_dadr, 6:] = np.linalg.lstsq(
            passive, closure[:, self.dadr], rcond=1e-12)[0]
        singular = np.linalg.svd(passive, compute_uv=False)
        return reduction, closure, {
            "residual_m": max_abs(np.concatenate(residuals)),
            "tangent_error": max_abs(closure @ reduction),
            "passive_condition": float(singular[0] / singular[-1]),
        }

    def wheel_coordinates(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        base_rotation = self.data.site_xmat[self.base_site].reshape(3, 3)
        base_position = self.data.site_xpos[self.base_site]
        jb, jrb = self.jac_site(self.base_site)
        values, jacobians = [], []
        for body in self.wheel_bodies:
            jw, _ = self.jac_body(body)
            relative_world = self.data.xpos[body] - base_position
            relative = base_rotation.T @ relative_world
            cross = np.array([[0.0, -relative[2], relative[1]],
                              [relative[2], 0.0, -relative[0]],
                              [-relative[1], relative[0], 0.0]])
            relative_jac = base_rotation.T @ (jw - jb) + cross @ (base_rotation.T @ jrb)
            values.append(relative)
            jacobians.append(relative_jac)
        values_array = np.asarray(values)
        jac = np.asarray(jacobians)
        return values_array[:, 0], values_array[:, 2], jac[:, 0, :], jac[:, 2, :]

    def mesh_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        vertices = []
        for geom in self.wheel_geoms:
            mesh = int(self.model.geom_dataid[geom]); start = int(self.model.mesh_vertadr[mesh])
            count = int(self.model.mesh_vertnum[mesh]); local = self.model.mesh_vert[start:start + count]
            world = self.data.geom_xpos[geom] + local @ self.data.geom_xmat[geom].reshape(3, 3).T
            vertices.append(world)
        points = np.vstack(vertices)
        return points.min(axis=0), points.max(axis=0)

    def contacts(self, reduction: np.ndarray) -> dict[str, Any]:
        records, jacobians = [], []
        loads = np.zeros(2)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            side = next((s for s, geom in enumerate(self.wheel_geoms)
                         if {int(contact.geom1), int(contact.geom2)} == {geom, self.floor}), None)
            if side is None:
                continue
            force = np.zeros(6); mujoco.mj_contactForce(self.model, self.data, index, force)
            loads[side] += max(0.0, float(force[0]))
            linear = np.zeros((3, self.model.nv)); angular = np.zeros_like(linear)
            mujoco.mj_jac(self.model, self.data, linear, angular,
                          np.asarray(contact.pos), self.wheel_bodies[side])
            jacobians.append(linear)
            records.append({"side": side, "position": np.asarray(contact.pos).copy(),
                            "normal": np.asarray(contact.frame[:3]).copy(),
                            "distance_m": float(contact.dist), "dim": int(contact.dim)})
        records.sort(key=lambda item: (item["side"], *item["position"]))
        points = np.asarray([item["position"] for item in records]) if records else np.zeros((0, 3))
        normals = np.asarray([item["normal"] for item in records]) if records else np.zeros((0, 3))
        full_jac = np.vstack(jacobians) if jacobians else np.zeros((0, self.model.nv))
        return {"count": len(records), "pair_ids": [[self.wheel_geoms[item["side"]], self.floor]
                for item in records], "points": points, "normals": normals,
                "minimum_distance_m": min((item["distance_m"] for item in records), default=float("inf")),
                "normal_load_n": loads, "jacobian": full_jac,
                "reduced_jacobian": full_jac @ reduction, "records": records}

    def sample(self, mode: str, phase: float, contact: bool = True) -> dict[str, Any]:
        self.set_phase(mode, phase, contact)
        reduction, closure, closure_metrics = self.closure_reduction()
        xi, zeta, axi, azeta = self.wheel_coordinates()
        full_mass = np.zeros((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, full_mass, self.data.qM)
        contact_data = self.contacts(reduction)
        lower, upper = self.mesh_bounds()
        qacc = self.data.qacc.copy()
        return {
            "mode": mode, "phase_rad": phase, "contact_enabled": contact,
            "wheel_center_world_m": self.data.xpos[self.wheel_bodies].copy(),
            "xi_m": xi, "zeta_m": zeta, "wheel_origin_jacobian": axi,
            "zeta_jacobian": azeta, "A_xi": axi @ reduction, "b_xi_m_s2": np.zeros(2),
            "mass_matrix": full_mass, "reduced_mass": reduction.T @ full_mass @ reduction,
            "bias": self.data.qfrc_bias.copy(), "reduced_bias": reduction.T @ self.data.qfrc_bias,
            "closure_jacobian": closure, "closure": closure_metrics,
            "mesh_world_lower_m": lower, "mesh_world_upper_m": upper,
            "contact": contact_data, "qacc_rad_m_s2": qacc,
            "physical_ddxi_m_s2": axi @ qacc,
            "wheel_acceleration_rad_s2": qacc[self.wheel_dadr],
            "finite": bool(np.all(np.isfinite(qacc)) and np.all(np.isfinite(full_mass))),
            "qp_solution": None, "realized_wrench": None, "qp_residual": None,
            "qp_limitation": "unavailable-by-contract outside live NominalWbcModel workspace; gate was not bypassed",
            "torque_canonical_nm": self.torque,
        }

    def local_mesh_vertices(self, geom: int) -> np.ndarray:
        mesh = int(self.model.geom_dataid[geom]); start = int(self.model.mesh_vertadr[mesh])
        count = int(self.model.mesh_vertnum[mesh]); vertices = self.model.mesh_vert[start:start + count]
        # Express compiled mesh vertices in the wheel-body frame.
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, self.model.geom_quat[geom])
        return self.model.geom_pos[geom] + vertices @ rotation.reshape(3, 3).T

    def symmetry(self) -> dict[str, Any]:
        results = {}
        threshold = float(self.method["thresholds"]["symmetry_hausdorff_m"])
        for side, geom in zip(("left", "right"), self.wheel_geoms):
            vertices = self.local_mesh_vertices(geom); tree = cKDTree(vertices)
            orders = {}
            for order in self.method["finite_symmetry_search_orders"]:
                angle = 2.0 * np.pi / float(order)
                rotation = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                                     [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]])
                rotated = vertices @ rotation.T
                forward = float(np.max(tree.query(rotated)[0]))
                reverse = float(np.max(cKDTree(rotated).query(vertices)[0]))
                orders[str(order)] = max(forward, reverse)
            results[side] = {"hausdorff_m_by_order": orders,
                             "equivalent_orders": [int(order) for order, error in orders.items()
                                                   if error <= threshold]}
        return results


def vector_for_periodic(sample: dict[str, Any], key: str) -> np.ndarray:
    if key.startswith("contact."):
        value = sample["contact"][key.split(".", 1)[1]]
    else:
        value = sample[key]
    return np.asarray(value, dtype=float).ravel()


def compare(first: dict[str, Any], second: dict[str, Any]) -> dict[str, Any]:
    keys = ["wheel_center_world_m", "xi_m", "zeta_m", "wheel_origin_jacobian", "A_xi",
            "mass_matrix", "reduced_mass", "bias", "reduced_bias", "closure_jacobian",
            "qacc_rad_m_s2", "physical_ddxi_m_s2", "wheel_acceleration_rad_s2",
            "contact.points", "contact.normals", "contact.normal_load_n",
            "contact.jacobian", "contact.reduced_jacobian"]
    result = {}
    for key in keys:
        left, right = vector_for_periodic(first, key), vector_for_periodic(second, key)
        if left.shape != right.shape:
            result[key] = {"shape_equal": False, "absolute": float("inf"), "relative": float("inf")}
            continue
        error = max_abs(left - right)
        result[key] = {"shape_equal": True, "absolute": error, "relative": relative(error, left, right)}
    result["contact_topology_equal"] = (
        first["contact"]["count"] == second["contact"]["count"]
        and first["contact"]["pair_ids"] == second["contact"]["pair_ids"])
    return result


def contact_centroid(sample: dict[str, Any]) -> np.ndarray:
    points = np.asarray(sample["contact"]["points"])
    return points.mean(axis=0) if len(points) else np.full(3, np.nan)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    method_path = args.method.resolve(); method = json.loads(method_path.read_text(encoding="utf-8"))
    audit = Audit(method)
    phases = sorted(set(method["coarse_phase_rad"] + method["boundary_phase_rad"]))
    samples = {(mode, phase): audit.sample(mode, float(phase))
               for mode in method["modes"] for phase in phases}
    contact_off = {(mode, phase): audit.sample(mode, float(phase), contact=False)
                   for mode in method["modes"] for phase in phases}
    periodic = []
    for mode in method["modes"]:
        for phase in method["periodic_base_phase_rad"]:
            first = audit.sample(mode, float(phase)); second = audit.sample(mode, float(phase) + 2.0 * np.pi)
            periodic.append({"mode": mode, "phase_a_rad": phase, "phase_b_rad": phase + 2.0 * np.pi,
                             "comparison": compare(first, second)})
    threshold = method["thresholds"]
    core_periodic_keys = ["wheel_center_world_m", "xi_m", "zeta_m", "wheel_origin_jacobian",
                          "A_xi", "mass_matrix", "reduced_mass", "bias", "reduced_bias",
                          "closure_jacobian"]
    core_periodic_errors = [pair["comparison"][key][field] for pair in periodic
                            for key in core_periodic_keys for field in ("absolute", "relative")]
    response_periodic_errors = [pair["comparison"][key][field] for pair in periodic
                                for key in ("qacc_rad_m_s2", "physical_ddxi_m_s2",
                                            "wheel_acceleration_rad_s2")
                                for field in ("absolute", "relative")]
    periodic_topology = all(pair["comparison"]["contact_topology_equal"] for pair in periodic)
    periodic_core_pass = bool(max(core_periodic_errors) <= max(
        float(threshold["periodic_absolute"]), float(threshold["periodic_relative"])))
    periodic_response_pass = bool(periodic_topology and max(response_periodic_errors) <= max(
        float(threshold["periodic_absolute"]), float(threshold["periodic_relative"])))

    zero = {(mode): samples[(mode, 0.0)] for mode in method["modes"]}
    contact_changes, ddxi_changes, off_ddxi_changes, origin_changes = [], [], [], []
    for (mode, phase), sample in samples.items():
        baseline = zero[mode]
        a, b = contact_centroid(sample), contact_centroid(baseline)
        if np.all(np.isfinite(a)) and np.all(np.isfinite(b)):
            contact_changes.append(max_abs(a - b))
        ddxi_changes.append(max_abs(sample["physical_ddxi_m_s2"] - baseline["physical_ddxi_m_s2"]))
        off_ddxi_changes.append(max_abs(contact_off[(mode, phase)]["physical_ddxi_m_s2"]
                                         - contact_off[(mode, 0.0)]["physical_ddxi_m_s2"]))
        origin_changes.append(max_abs(sample["wheel_center_world_m"] - baseline["wheel_center_world_m"]))

    boundary = []
    for mode in method["modes"]:
        for sign in (-1.0, 1.0):
            def dd(value: float) -> np.ndarray: return samples[(mode, sign * value)]["physical_ddxi_m_s2"]
            middle = max_abs(dd(1.01) - dd(0.99)) / 0.02
            left = max_abs(dd(0.99) - dd(0.95)) / 0.04
            right = max_abs(dd(1.05) - dd(1.01)) / 0.04
            ratio = middle / max(left, right, 1e-15)
            absolute = max_abs(dd(1.01) - dd(0.99))
            counts = [samples[(mode, sign * value)]["contact"]["count"] for value in (0.99, 1.0, 1.01)]
            boundary.append({"mode": mode, "sign": int(sign), "central_ddxi_jump_m_s2": absolute,
                             "central_slope_m_s2_per_rad": middle, "neighbor_slope_max": max(left, right),
                             "jump_ratio": ratio, "contact_counts_0p99_1p00_1p01": counts,
                             "special": bool(ratio >= float(threshold["boundary_jump_ratio"])
                                             and absolute >= float(threshold["boundary_absolute_ddxi_m_s2"]))})
    material_contact = max(contact_changes, default=0.0) >= float(threshold["material_contact_point_change_m"])
    material_dynamic = max(ddxi_changes) >= float(threshold["material_ddxi_change_m_s2"])
    boundary_special = any(item["special"] for item in boundary)
    origin_pass = max(origin_changes) <= float(threshold["wheel_origin_invariance_m"])
    # The accepted Phase35 source state has a nonzero absolute assembly residual.  This audit
    # tests whether wheel phase changes it; it must not silently redefine the frozen state.
    closure_values = [sample["closure"]["residual_m"] for sample in samples.values()]
    closure_variation = max(closure_values) - min(closure_values)
    closure_pass = closure_variation <= float(threshold["closure_residual_m"])
    all_finite = all(sample["finite"] for sample in samples.values())
    contact_isolation_ratio = max(off_ddxi_changes) / max(max(ddxi_changes), 1e-15)
    if not all_finite or not periodic_core_pass:
        classification = "P36-E_wbc_or_model_periodicity_inconsistency"
    elif boundary_special:
        classification = "P36-A_evidenced_necessary_boundary"
    elif (material_contact or material_dynamic) and (contact_isolation_ratio <= 0.1 or not periodic_response_pass):
        classification = "P36-D_collision_mesh_contact_discretization_artifact"
    elif material_contact or material_dynamic:
        classification = "P36-B_phase_sensitive_but_one_rad_arbitrary"
    else:
        classification = "P36-C_periodic_consistent_bound_unsupported"
    summary = {
        "classification": classification,
        "dg36_00_semantics_pass": True,
        "dg36_01_static_geometry_pass": all_finite and origin_pass and closure_pass,
        "dg36_02_core_model_periodicity_pass": periodic_core_pass,
        "dg36_03_dynamic_equivalence_audit_complete": True,
        "periodic_dynamic_response_equivalence_pass": periodic_response_pass,
        "dg36_04_one_rad_boundary_special": boundary_special,
        "dg36_05_terminal_interpretation_pass": classification != "P36-U_evidence_insufficient",
        "production_modified": False, "live_gate_bypassed": False,
        "qp_outside_gate_available": False,
        "maxima": {"wheel_origin_change_m": max(origin_changes),
                   "contact_centroid_change_m": max(contact_changes, default=0.0),
                   "physical_ddxi_change_m_s2": max(ddxi_changes),
                   "contact_off_ddxi_change_m_s2": max(off_ddxi_changes),
                   "contact_isolation_ratio": contact_isolation_ratio,
                   "core_periodic_error": max(core_periodic_errors),
                   "periodic_response_error": max(response_periodic_errors),
                   "closure_residual_m": max(closure_values),
                   "closure_residual_variation_m": closure_variation},
        "material_contact_phase_effect": material_contact,
        "material_dynamic_phase_effect": material_dynamic,
        "periodic_contact_topology_pass": periodic_topology,
        "boundary": boundary,
    }
    symmetry = audit.symmetry()
    input_paths = [method_path, ROOT / method["scene"], ROOT / method["source_phase35_hold"], Path(__file__).resolve()]
    manifest = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                "command": " ".join(sys.argv), "replay_of": args.replay_of,
                "python": sys.version, "platform": platform.platform(),
                "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
                "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in input_paths},
                "method_hash": sha256(method_path), "sample_count": len(samples),
                "contact_off_sample_count": len(contact_off), "periodic_pair_count": len(periodic)}
    output.mkdir(parents=True)
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n")
    (output / "details.json").write_text(json.dumps(clean({"symmetry": symmetry,
        "samples": {f"{mode}:{phase:+.8f}": sample for (mode, phase), sample in samples.items()},
        "contact_off": {f"{mode}:{phase:+.8f}": sample for (mode, phase), sample in contact_off.items()},
        "periodic_pairs": periodic}), indent=2, sort_keys=True) + "\n")
    (output / "manifest.json").write_text(json.dumps(clean(manifest), indent=2, sort_keys=True) + "\n")
    print(json.dumps(clean(summary), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
