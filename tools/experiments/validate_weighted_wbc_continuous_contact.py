#!/usr/bin/env python3
"""Validate the frozen continuous six-point Phase-21 contact representation."""

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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_weighted_wbc_contact_representation import wrench_residual  # noqa: E402
from validate_mujoco_weighted_wbc_model import Oracle, load_config, object_id  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False,
                               default=lambda item: item.item() if isinstance(item, np.generic) else item) + "\n", encoding="utf-8")


def skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


class ContinuousPatch:
    def __init__(self, oracle: Oracle, settings: dict[str, Any]) -> None:
        self.oracle, self.settings = oracle, settings
        self.n = np.asarray(settings["ground_normal_world"], dtype=float)
        self.n /= np.linalg.norm(self.n)
        self.geoms = [object_id(oracle.model, mujoco.mjtObj.mjOBJ_GEOM, name)
                      for name in ("left_wheel_collision", "right_wheel_collision")]
        self.bounds = []
        for geom in self.geoms:
            mesh = int(oracle.model.geom_dataid[geom]); start = int(oracle.model.mesh_vertadr[mesh])
            vertices = oracle.model.mesh_vert[start:start + int(oracle.model.mesh_vertnum[mesh])]
            self.bounds.append((float(vertices[:, 0].min()), float(vertices[:, 0].max())))

    def geometry(self, qpos: np.ndarray, side: int) -> dict[str, np.ndarray | float]:
        self.oracle.forward(qpos)
        geom, body = self.geoms[side], self.oracle.wheel_bodies[side]
        R = self.oracle.data.geom_xmat[geom].reshape(3, 3).copy()
        c = self.oracle.data.geom_xpos[geom].copy(); a = R[:, 0]
        dot = float(a @ self.n); s = float(np.sqrt(max(0., 1. - dot * dot)))
        if s < float(self.settings["minimum_axis_ground_projection"]):
            raise RuntimeError(f"wheel {side} axis nearly parallel to ground normal: {s}")
        tr = np.cross(a, self.n) / s; tl = np.cross(self.n, tr); radial = (self.n - dot * a) / s
        xmin, xmax = self.bounds[side]; mid, half_lateral = .5 * (xmin + xmax), .5 * (xmax - xmin)
        r, d = float(self.settings["radius_m"]), float(self.settings["support_band_m"])
        half_roll = float(np.sqrt(2 * r * d - d * d)); pc = c + mid * a - r * radial
        # Frozen order: bottom lateral endpoints, then negative/positive rolling band edges.
        points = np.array([pc - half_lateral * tl, pc + half_lateral * tl,
                           *[pc + x * tr + y * tl + d * self.n
                             for x in (-half_roll, half_roll) for y in (-half_lateral, half_lateral)]])
        return {"center": c, "body_center": self.oracle.data.xpos[body].copy(), "axis": a, "rolling": tr, "lateral": tl, "radial_up": radial,
                "contact_center": pc, "points": points, "axis_ground_projection": s, "body": body}

    def geometric_jacobian(self, qpos: np.ndarray, reduction: np.ndarray, side: int) -> np.ndarray:
        g = self.geometry(qpos, side); body = int(g["body"]); c = np.asarray(g["center"]); a = np.asarray(g["axis"])
        tr, tl, radial = (np.asarray(g[k]) for k in ("rolling", "lateral", "radial_up"))
        dot, s = float(a @ self.n), float(g["axis_ground_projection"])
        linear = np.zeros((3, self.oracle.model.nv)); angular = np.zeros_like(linear)
        mujoco.mj_jac(self.oracle.model, self.oracle.data, linear, angular, c, body)
        jc, jw = linear @ reduction, angular @ reduction
        xmin, xmax = self.bounds[side]; mid, half_lateral = .5 * (xmin + xmax), .5 * (xmax - xmin)
        r, d = float(self.settings["radius_m"]), float(self.settings["support_band_m"])
        half_roll = float(np.sqrt(2 * r * d - d * d)); rows = []
        for x, y, z in ((0., -half_lateral, 0.), (0., half_lateral, 0.),
                        *[(x, y, d) for x in (-half_roll, half_roll) for y in (-half_lateral, half_lateral)]):
            out = np.zeros((3, 12))
            for k in range(12):
                da = np.cross(jw[:, k], a); ddot = float(da @ self.n); ds = -dot * ddot / s
                dradial = (-ddot * a - dot * da) / s - radial * ds / s
                dtr = np.cross(da, self.n) / s - tr * ds / s
                dtl = np.cross(self.n, dtr)
                out[:, k] = jc[:, k] + mid * da - r * dradial + x * dtr + y * dtl
            rows.append(out)
        return np.asarray(rows)

    def force_jacobian(self, qpos: np.ndarray, reduction: np.ndarray, side: int) -> np.ndarray:
        g = self.geometry(qpos, side); rows = []
        for point in np.asarray(g["points"]):
            linear = np.zeros((3, self.oracle.model.nv)); angular = np.zeros_like(linear)
            mujoco.mj_jac(self.oracle.model, self.oracle.data, linear, angular, point, int(g["body"]))
            rows.append(linear @ reduction)
        return np.asarray(rows)


def maxabs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value)))


def state_velocity(oracle: Oracle, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
    reduction, _ = oracle.reduction(qpos)
    return np.linalg.lstsq(reduction, qvel, rcond=None)[0]


def differential(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray, velocity: np.ndarray,
                 side: int, steps: list[float], bias_step: float) -> dict[str, Any]:
    reduction, _ = oracle.reduction(qpos); analytic = patch.geometric_jacobian(qpos, reduction, side)
    point = np.asarray(patch.geometry(qpos, side)["points"])
    velocity_errors = []
    for step in steps:
        plus, minus = (oracle.integrate_flow(qpos, velocity, sign * step) for sign in (1., -1.))
        fd = (np.asarray(patch.geometry(plus, side)["points"]) - np.asarray(patch.geometry(minus, side)["points"])) / (2 * step)
        error = maxabs(fd - np.einsum("pij,j->pi", analytic, velocity))
        velocity_errors.append({"step_s": step, "absolute_error_m_s": error,
                                "relative_error": error / max(1., maxabs(fd))})
    plus, minus = (oracle.integrate_flow(qpos, velocity, sign * bias_step) for sign in (1., -1.))
    rp, _ = oracle.reduction(plus); rm, _ = oracle.reduction(minus)
    jp, jm = patch.geometric_jacobian(plus, rp, side), patch.geometric_jacobian(minus, rm, side)
    jdot_nu = np.einsum("pij,j->pi", (jp - jm) / (2 * bias_step), velocity)
    acceleration_fd = (np.asarray(patch.geometry(plus, side)["points"]) - 2 * point +
                       np.asarray(patch.geometry(minus, side)["points"])) / (bias_step * bias_step)
    bias_error = maxabs(acceleration_fd - jdot_nu)
    return {"velocity_fd": velocity_errors, "bias_step_s": bias_step,
            "bias_absolute_error_m_s2": bias_error,
            "bias_relative_error": bias_error / max(1., maxabs(acceleration_fd))}


def force_map_cases(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray,
                    velocity: np.ndarray) -> list[dict[str, Any]]:
    """Apply frozen local-force distributions through both independent force maps."""
    rng = np.random.default_rng(2106); zeros = np.zeros((2, 6, 3)); cases: dict[str, np.ndarray] = {}
    for side in range(2):
        for point in range(6):
            for axis in range(3):
                value = zeros.copy(); value[side, point, axis] = 1.; cases[f"basis_s{side}_p{point}_{axis}"] = value
    value = zeros.copy(); value[0] = rng.normal(size=(6, 3)); value[0, :, 2] = rng.uniform(.2, 1.2, 6); cases["random_single_wheel"] = value
    value = zeros.copy(); value[:, :, 2] = 1.; cases["symmetric_two_wheel_normal"] = value
    value = zeros.copy(); value[0, :, 2] = .8; value[1, :, 2] = 1.3; cases["asymmetric_two_wheel_normal"] = value
    value = zeros.copy(); value[0, :, 0] = .7; value[1, :, 1] = -.4; cases["pure_tangential"] = value
    value = zeros.copy(); value[:, :, 2] = .9; cases["pure_normal"] = value
    value = zeros.copy(); value[0, 0, 2] = 1.; value[0, 5, 2] = -1.; cases["moment_producing"] = value
    reduction, _ = oracle.reduction(qpos); full_velocity = reduction @ velocity; results = []
    for case_id, local in cases.items():
        full = np.zeros(oracle.model.nv); expected = np.zeros(12)
        for side in range(2):
            g = patch.geometry(qpos, side); Rc = np.column_stack((g["rolling"], g["lateral"], patch.n))
            J = patch.force_jacobian(qpos, reduction, side)
            for point, force_local in enumerate(local[side]):
                force = Rc @ force_local
                mujoco.mj_applyFT(oracle.model, oracle.data, force, np.zeros(3), np.asarray(g["points"])[point], int(oracle.wheel_bodies[side]), full)
                expected += J[point].T @ force
        results.append({"case_id": case_id, "applyft_projection_error": maxabs(reduction.T @ full - expected),
                        "virtual_work_error_w": abs(float(full_velocity @ full - velocity @ expected))})
    return results


def representability(capture_dir: Path, settings: dict[str, Any], candidate: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    capture = np.load(capture_dir / "capture.npz"); band = float(settings["support_band_m"])
    start, end = 1, min(271, len(capture["tick"]) - 2); rows = []
    for tick in range(start, end + 1):
        for side in range(2):
            force, moment = capture["truth_force"][tick, side], capture["truth_moment_about_wheel"][tick, side]
            if force[2] < 1.: continue
            R, c = capture["geom_rotation"][tick, side], capture["geom_position"][tick, side]
            a, n = R[:, 0], np.array([0., 0., 1.]); dot = a @ n; s = np.sqrt(1 - dot * dot)
            tr, tl, radial = np.cross(a, n) / s, np.cross(n, np.cross(a, n) / s), (n - dot * a) / s
            vertices = capture["mesh_vertices_left" if side == 0 else "mesh_vertices_right"]; xmin, xmax = vertices[:, 0].min(), vertices[:, 0].max()
            d, r = band, float(settings["radius_m"]); pc = c + .5 * (xmin + xmax) * a - r * radial
            hr, hl = np.sqrt(2 * r * d - d * d), .5 * (xmax - xmin)
            corners = [pc + x * tr + y * tl + d * n for x in (-hr, hr) for y in (-hl, hl)]
            points = np.array(corners if candidate == "rejected_four_band_corners" else [pc - hl * tl, pc + hl * tl, *corners])
            Rc = np.column_stack((tr, tl, n))
            local_points = (points - capture["wheel_center"][tick, side]) @ Rc
            local_wrench = np.r_[Rc.T @ force, Rc.T @ moment]
            residual, solved = wrench_residual(local_points, np.zeros(3), local_wrench, float(settings["friction_coefficient"]))
            rows.append({"capture": capture_dir.name, "candidate": candidate, "tick": tick, "side": side, "normal_force_n": float(force[2]),
                         "wrench_residual": residual, "feasible": bool(solved and residual <= float(settings["maximum_wrench_equality_residual"])),
                         "contact_center_to_truth_cop_xy_error_m": float(np.linalg.norm(pc[:2] - capture["truth_cop"][tick, side, :2]))})
    feasible = [row["feasible"] for row in rows]
    return {"valid_sides": len(rows), "feasible_fraction": float(np.mean(feasible)),
            "maximum_wrench_residual": max(row["wrench_residual"] for row in rows),
            "maximum_contact_center_to_truth_cop_xy_error_m": max(row["contact_center_to_truth_cop_xy_error_m"] for row in rows)}, rows


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True); args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True); config_path = args.config.resolve(); config, inputs = load_config(config_path)
    settings = config["continuous_contact_oracle"]; model_path = (ROOT / settings["model_profile"]).resolve()
    model_config, model_inputs = load_config(model_path); equilibrium_path = ROOT / model_config["equilibrium"]
    oracle = Oracle(model_config, json.loads(equilibrium_path.read_text())); patch = ContinuousPatch(oracle, settings)
    cap2 = (ROOT / settings["capture_v2"]).resolve(); capture = np.load(cap2 / "capture.npz"); switches = json.loads((ROOT / settings["old_switches"]).read_text())
    ticks = np.asarray(capture["tick"], dtype=int); rolling = []
    for tick in ticks[1:272]:
        qpos = capture["qpos"][tick]
        for side in range(2):
            g = patch.geometry(qpos, side); rolling.append({"tick": int(tick), "side": side, "points": np.asarray(g["points"]).tolist(),
                "frame": {k: np.asarray(g[k]).tolist() for k in ("axis", "rolling", "lateral", "radial_up")}, "projection": float(g["axis_ground_projection"])})
    diffticks = set(int(v) for v in settings["representative_rolling_ticks"])
    diffticks.update(max(1, min(271, int(event["tick"]) + offset)) for event in switches for offset in (-1, 0, 1))
    differential_rows = []
    for tick in sorted(diffticks):
        qpos, velocity = capture["qpos"][tick], state_velocity(oracle, capture["qpos"][tick], capture["qvel"][tick])
        for side in range(2): differential_rows.append({"tick": tick, "side": side, **differential(oracle, patch, qpos, velocity, side, list(settings["velocity_fd_steps_s"]), float(settings["bias_fd_step_s"]))})
    envelope_rotation = np.max(np.abs([sample["base_rotation_vector_rad"] for sample in model_config["samples"]]), axis=0)
    envelope_delta = np.max(np.abs([sample["canonical_joint_delta_rad"] for sample in model_config["samples"]]), axis=0)
    rng = np.random.default_rng(int(settings["workspace_random_seed"]))
    random_samples = [{"id": f"random_{index:02d}",
                       "base_rotation_vector_rad": rng.uniform(-envelope_rotation, envelope_rotation).tolist(),
                       "canonical_joint_delta_rad": rng.uniform(-envelope_delta, envelope_delta).tolist()}
                      for index in range(int(settings["workspace_random_count"]))]
    workspace = []
    workspace_qpos: list[tuple[str, np.ndarray]] = []
    for sample in [*model_config["samples"], *random_samples]:
        qpos = oracle.sample_qpos(sample); velocity = np.array([.07, -.05, .03, .11, -.09, .08, .13, -.10, .17, -.12, .09, -.16])
        workspace_qpos.append((sample["id"], qpos))
        for side in range(2): workspace.append({"sample": sample["id"], "side": side, "geometry": {k: np.asarray(v).tolist() if isinstance(v, np.ndarray) else v for k, v in patch.geometry(qpos, side).items() if k != "body"}, **differential(oracle, patch, qpos, velocity, side, list(settings["velocity_fd_steps_s"]), float(settings["bias_fd_step_s"]))})
    force_rows = []
    for row in differential_rows:
        qpos, velocity = capture["qpos"][row["tick"]], state_velocity(oracle, capture["qpos"][row["tick"]], capture["qvel"][row["tick"]])
        force_rows.extend({"source": "rolling", "tick": row["tick"], **result} for result in force_map_cases(oracle, patch, qpos, velocity))
    for sample_id, qpos in workspace_qpos:
        force_rows.extend({"source": "workspace", "sample": sample_id, **result}
                          for result in force_map_cases(oracle, patch, qpos, np.array([.07, -.05, .03, .11, -.09, .08, .13, -.10, .17, -.12, .09, -.16])))
    represent = {}; repr_rows = []
    for key in ("capture_v1", "capture_v2"):
        represent[key] = {}
        for candidate in ("rejected_four_band_corners", "selected_six_point_patch"):
            candidate_summary, rows = representability((ROOT / settings[key]).resolve(), settings, candidate)
            represent[key][candidate] = candidate_summary; repr_rows.extend(rows)
    fine = []
    for event in switches:
        tick, side = int(event["tick"]), 0 if event["side"] == "left" else 1
        q0, q1 = capture["qpos"][tick - 1], capture["qpos"][tick]; delta = np.zeros(oracle.model.nv); mujoco.mj_differentiatePos(oracle.model, delta, 1., q0, q1)
        frames = []
        for alpha in np.linspace(0., 1., int(settings["fine_switch_points"])):
            q = q0.copy(); mujoco.mj_integratePos(oracle.model, q, delta, float(alpha)); q = oracle.solve_passive(q)[0]; g = patch.geometry(q, side)
            frames.append({k: np.asarray(g[k]).tolist() for k in ("points", "axis", "rolling", "lateral", "radial_up")})
        fine.append({"tick": tick, "side": side, "frames": frames})
    def jumps(series: list[dict[str, Any]], key: str) -> float:
        return max((maxabs(np.asarray(b["frame"][key]) - np.asarray(a["frame"][key])) for a, b in zip(series, series[1:])), default=0.)
    geom_jumps = {key: max(jumps([r for r in rolling if r["side"] == side], key) for side in range(2))
                  for key in ("axis", "rolling", "lateral", "radial_up")}
    fine_jump = max((maxabs(np.diff(np.asarray([f["points"] for f in e["frames"]]), axis=0)) for e in fine), default=0.)
    frame_dots = [float(np.dot(np.asarray(b["frame"][key]), np.asarray(a["frame"][key])))
                  for side in range(2) for key in ("axis", "rolling", "lateral", "radial_up")
                  for a, b in zip([r for r in rolling if r["side"] == side], [r for r in rolling if r["side"] == side][1:])]
    geometry_finite = all(np.all(np.isfinite(np.asarray(row["points"]))) and
                          all(np.all(np.isfinite(np.asarray(value))) for value in row["frame"].values())
                          for row in rolling)
    frame_errors = [maxabs(np.column_stack((row["frame"]["rolling"], row["frame"]["lateral"], patch.n)).T @
                           np.column_stack((row["frame"]["rolling"], row["frame"]["lateral"], patch.n)) - np.eye(3)) for row in rolling]
    frame_det_errors = [abs(np.linalg.det(np.column_stack((row["frame"]["rolling"], row["frame"]["lateral"], patch.n))) - 1.) for row in rolling]
    fine_finite = all(np.all(np.isfinite(np.asarray(frame["points"]))) and
                      all(np.all(np.isfinite(np.asarray(frame[key]))) for key in ("axis", "rolling", "lateral", "radial_up"))
                      for event in fine for frame in event["frames"])
    fine_frame_dots = [float(np.dot(np.asarray(b[key]), np.asarray(a[key]))) for event in fine
                       for key in ("axis", "rolling", "lateral", "radial_up")
                       for a, b in zip(event["frames"], event["frames"][1:])]
    velocity_error = max(v["absolute_error_m_s"] for row in differential_rows + workspace for v in row["velocity_fd"])
    bias_error = max(row["bias_absolute_error_m_s2"] for row in differential_rows + workspace)
    left_span, right_span = (bound[1] - bound[0] for bound in patch.bounds); left_mid, right_mid = (.5 * sum(bound) for bound in patch.bounds)
    mesh_mirror_error = max(abs(left_span - right_span), abs(left_mid + right_mid))
    gates = {"finite_fixed_six_point_shape_order_nonsingular": geometry_finite and all(np.asarray(row["points"]).shape == (6, 3) for row in rolling) and min(row["projection"] for row in rolling) >= float(settings["minimum_axis_ground_projection"]),
             "frame_orthonormal_right_handed": max(frame_errors) <= float(settings["maximum_frame_orthonormality_error"]) and max(frame_det_errors) <= float(settings["maximum_frame_determinant_error"]),
             "frame_no_sign_flip": min(frame_dots, default=1.) > 0.,
             "fine_switch_continuity": fine_finite and min(fine_frame_dots, default=1.) > 0. and fine_jump <= float(settings["maximum_fine_point_increment_m"]),
             "compiled_mesh_mirror": mesh_mirror_error <= float(settings["maximum_mesh_mirror_error_m"]),
             "velocity_fd": velocity_error <= float(settings["maximum_velocity_fd_m_s"]), "contact_bias": bias_error <= float(settings["maximum_contact_bias_fd_m_s2"]),
             "force_applyft": max(row["applyft_projection_error"] for row in force_rows) <= float(settings["maximum_applyft_error"]),
             "force_virtual_work": max(row["virtual_work_error_w"] for row in force_rows) <= float(settings["maximum_virtual_work_error_w"]),
             "representability_v1": represent["capture_v1"]["selected_six_point_patch"]["feasible_fraction"] >= float(settings["minimum_representable_fraction"]),
             "representability_v2": represent["capture_v2"]["selected_six_point_patch"]["feasible_fraction"] >= float(settings["minimum_representable_fraction"])}
    rank_rows = []; G_by_side: list[list[np.ndarray]] = [[], []]
    rank_states = [(f"rolling_{tick}", capture["qpos"][tick]) for tick in range(1, 272)] + workspace_qpos
    for state_id, qstate in rank_states:
        for side in range(2):
            g = patch.geometry(qstate, side); Rc = np.column_stack((g["rolling"], g["lateral"], patch.n))
            G = np.hstack([np.vstack((Rc, skew(p - np.asarray(g["body_center"])) @ Rc)) for p in np.asarray(g["points"])])
            G_by_side[side].append(G)
            s = np.linalg.svd(G, compute_uv=False); rank_rows.append({"state_id": state_id, "side": side, "rank": int(np.linalg.matrix_rank(G)), "singular_values": s.tolist(), "condition_nonzero": float(s[0] / s[-1]), "nullspace_dimension": 18 - int(np.linalg.matrix_rank(G))})
    g_variation = max((maxabs(G - values[0]) for values in G_by_side for G in values[1:]), default=0.)
    rank_summary = {"row_count": len(rank_rows), "rank_min": min(r["rank"] for r in rank_rows), "rank_max": max(r["rank"] for r in rank_rows), "condition_nonzero_min": min(r["condition_nonzero"] for r in rank_rows), "condition_nonzero_max": max(r["condition_nonzero"] for r in rank_rows), "nullspace_dimension_min": min(r["nullspace_dimension"] for r in rank_rows), "nullspace_dimension_max": max(r["nullspace_dimension"] for r in rank_rows), "minimum_singular_value": min(r["singular_values"][-1] for r in rank_rows), "maximum_sampled_G_variation": g_variation}
    summary = {"schema_version": 1, "phase": 21, "profile": config["profile"], "representation": "selected six-point analytic virtual surface patch: two bottom lateral endpoints then four band-edge corners; prior four-band-corner candidate is rejected by independent corpus representability", "geometry": {"radius_m": settings["radius_m"], "support_band_m": settings["support_band_m"], "points_per_wheel": settings["points_per_wheel"], "point_order": settings["point_order"], "rolling_frame": "normalize(axis cross ground_normal)", "lateral_frame": "ground_normal cross rolling", "mesh_axial_span_m": {"left": left_span, "right": right_span}, "mesh_axial_midpoint_m": {"left": left_mid, "right": right_mid}, "mesh_mirror_error_m": mesh_mirror_error}, "force_coordinates": {"per_point_order": ["rolling", "lateral", "normal"], "world_force": "R_c f_local, R_c=[rolling lateral normal]", "pyramid": "|f_rolling|, |f_lateral| <= mu f_normal; f_normal >= 0", "controller_wrench": "world/FLU wrench; LP rotates truth force, moment, and point offsets into R_c before applying the local pyramid"}, "coverage": {"rolling_ticks": [1, 271], "differential_ticks": sorted(diffticks), "old_switch_event_count": len(switches), "workspace_base_samples": len(model_config["samples"]), "workspace_random_seed": settings["workspace_random_seed"], "workspace_random_count": len(random_samples)}, "maximum_frame_increment": geom_jumps, "minimum_consecutive_frame_dot": min(frame_dots, default=1.), "maximum_frame_orthonormality_error": max(frame_errors), "maximum_frame_determinant_error": max(frame_det_errors), "maximum_fine_point_increment_m": fine_jump, "minimum_fine_consecutive_frame_dot": min(fine_frame_dots, default=1.), "maximum_velocity_fd_error_m_s": velocity_error, "maximum_contact_bias_error_m_s2": bias_error, "force_map": {"case_ids": sorted({r["case_id"] for r in force_rows}), "case_count": len(force_rows), "maximum_applyft_projection_error": max(r["applyft_projection_error"] for r in force_rows), "maximum_virtual_work_error_w": max(r["virtual_work_error_w"] for r in force_rows), "semantic_limit": "J_force is the instantaneous material-point Jacobian. J_geom is the virtual-point geometric Jacobian; virtual points are not asserted material trajectories."}, "representability": represent, "rank_condensation": {"wrench_reference": "actual wheel/body center", **rank_summary, "conclusion": "At every sampled state the 6x18 G(q) has an exact per-state polyhedral 6D wrench-cone projection; internal point-force variables are not mathematically required. G(q) varies with pose and a single fixed local H-representation remains OPEN."}, "cop_metric_limit": "contact-center-to-truth-COP is a geometric comparison only; solved-force COP is nonunique because the point-force distribution has a nullspace.", "pfaffian": {"status": settings["pfaffian_status"], "candidate": "three rows per side at analytic contact center: rolling, lateral, normal", "claim": "not validated by this force/geometry oracle"}, "gates": gates, "pass": all(gates.values())}
    write_json(output / "summary.json", summary); write_json(output / "rolling_geometry.json", rolling); write_json(output / "differential.json", differential_rows + workspace); write_json(output / "fine_switch_sweeps.json", fine); write_json(output / "rank_condensation.json", rank_rows); write_json(output / "force_map.json", force_rows)
    with (output / "representability.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(repr_rows[0])); writer.writeheader(); writer.writerows(repr_rows)
    script = Path(__file__).resolve(); outputs = ["summary.json", "rolling_geometry.json", "differential.json", "fine_switch_sweeps.json", "rank_condensation.json", "force_map.json", "representability.csv"]
    write_json(output / "manifest.json", {"schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(), "python": platform.python_version(), "numpy": np.__version__, "mujoco": mujoco.__version__, "config_inputs": {str(p.relative_to(ROOT)): sha256(p) for p in inputs}, "model_inputs": {str(p.relative_to(ROOT)): sha256(p) for p in model_inputs}, "captures": {key: {"capture": sha256((ROOT / settings[key]) / "capture.npz"), "manifest": sha256((ROOT / settings[key]) / "manifest.json")} for key in ("capture_v1", "capture_v2")}, "switches_sha256": sha256(ROOT / settings["old_switches"]), "validator": str(script.relative_to(ROOT)), "validator_sha256": sha256(script), "outputs": {name: sha256(output / name) for name in outputs}})
    print(json.dumps(summary, indent=2, sort_keys=True, default=lambda item: item.item() if isinstance(item, np.generic) else item)); return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as error: print(f"ERROR: {error}", file=sys.stderr); sys.exit(2)
