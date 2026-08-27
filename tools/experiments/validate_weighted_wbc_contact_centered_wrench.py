#!/usr/bin/env python3
"""Offline oracle for Phase-21 fixed contact-centred six-dimensional wrench cone.

This is deliberately an LP/ConvexHull evidence tool, not a WBC QP implementation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy import __version__ as scipy_version
from scipy.optimize import linprog
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import Oracle, load_config  # noqa: E402
from validate_weighted_wbc_continuous_contact import ContinuousPatch, skew  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False,
                               default=lambda x: x.item() if isinstance(x, np.generic) else x) + "\n", encoding="utf-8")


def mabs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value)))


def rc(geom: dict[str, Any], normal: np.ndarray) -> np.ndarray:
    return np.column_stack((geom["rolling"], geom["lateral"], normal))


def geometry_map(patch: ContinuousPatch, qpos: np.ndarray, side: int) -> tuple[dict[str, Any], np.ndarray, np.ndarray]:
    g = patch.geometry(qpos, side); frame = rc(g, patch.n); center = np.asarray(g["contact_center"])
    offsets = (np.asarray(g["points"]) - center) @ frame
    G = np.hstack([np.vstack((np.eye(3), skew(offset) @ np.eye(3))) for offset in offsets])
    return g, offsets, G


def rays(offsets: np.ndarray, mu: float) -> np.ndarray:
    return np.asarray([np.r_[d, np.cross(offset, d)]
                       for offset in offsets for sr in (-1., 1.) for sl in (-1., 1.)
                       for d in (np.array([sr * mu, sl * mu, 1.]),)]) .T


def point_inequalities(mu: float) -> np.ndarray:
    rows = []
    for i in range(6):
        for axis in (0, 1):
            for sign in (-1., 1.):
                row = np.zeros(18); row[3 * i + axis] = sign; row[3 * i + 2] = -mu; rows.append(row)
        row = np.zeros(18); row[3 * i + 2] = -1.; rows.append(row)
    return np.asarray(rows)


def lp_member(A: np.ndarray, target: np.ndarray, inequalities: np.ndarray, tol: float) -> tuple[bool, np.ndarray | None, float]:
    result = linprog(np.zeros(A.shape[1]), A_ub=inequalities, b_ub=np.zeros(len(inequalities)),
                     A_eq=A, b_eq=target, bounds=[(None, None)] * A.shape[1], method="highs")
    if not result.success:
        return False, None, 1e300
    residual = mabs(A @ result.x - target)
    violation = max(0., float(np.max(inequalities @ result.x)))
    return residual <= tol and violation <= tol, result.x, max(residual, violation)


def v_member(R: np.ndarray, target: np.ndarray, tol: float) -> tuple[bool, np.ndarray | None, float]:
    result = linprog(np.zeros(R.shape[1]), A_eq=R, b_eq=target, bounds=[(0., None)] * R.shape[1], method="highs")
    if not result.success:
        return False, None, 1e300
    residual = mabs(R @ result.x - target)
    return residual <= tol, result.x, residual


def build_h(R: np.ndarray, options: str) -> tuple[np.ndarray, dict[str, Any]]:
    # Every nonzero ray has Fn>0, so Fn=1 is a complete, bounded 5D section.
    if np.min(R[2]) <= 0.: raise RuntimeError("Cannot normalize rays by Fn")
    idx = [0, 1, 3, 4, 5]; section = (R[idx] / R[2]).T
    hull = ConvexHull(section, qhull_options=options)
    raw = []
    for eq in hull.equations:
        h = np.zeros(6); h[idx] = eq[:-1]; h[2] = eq[-1]
        norm = np.linalg.norm(h)
        raw.append(h / norm)
    unique: list[np.ndarray] = []
    for h in raw:
        if not any(np.max(np.abs(h - old)) <= 1e-9 for old in unique): unique.append(h)
    # The slice does not itself encode its positive homogenizing coordinate.
    h = np.vstack((np.asarray(unique), np.array([0., 0., -1., 0., 0., 0.])))
    return h, {"slice": "Fn=1; coordinates [Fr,Fl,Mr,Ml,Mn]", "raw_simplicial_facets": len(raw),
               "unique_facets_before_fn": len(unique), "facets_including_fn_nonnegative": len(h),
               "normalization": "each inequality row has Euclidean norm one", "qhull_options": options,
               "section_rank": int(np.linalg.matrix_rank(section - section[0])),
               "section_condition": float(np.linalg.cond(section - section.mean(axis=0)))}


def h_member(H: np.ndarray, target: np.ndarray, tol: float) -> tuple[bool, float]:
    return bool(np.max(H @ target) <= tol), float(np.max(H @ target))


def local_truth(capture: Any, tick: int, side: int, patch: ContinuousPatch) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if "qpos" in capture.files:
        qpos = capture["qpos"][tick]; g, _, _ = geometry_map(patch, qpos, side); frame = rc(g, patch.n); center = np.asarray(g["contact_center"])
    else:
        Rg, center_geom = capture["geom_rotation"][tick, side], capture["geom_position"][tick, side]
        axis = Rg[:, 0]; n = patch.n; dot = float(axis @ n); s = float(np.sqrt(1. - dot * dot))
        rolling = np.cross(axis, n) / s; lateral = np.cross(n, rolling); radial = (n - dot * axis) / s
        center = center_geom + .5 * sum(patch.bounds[side]) * axis - float(patch.settings["radius_m"]) * radial
        frame = np.column_stack((rolling, lateral, n))
    force = capture["truth_force"][tick, side]; moment_o = capture["truth_moment_about_wheel"][tick, side]
    origin = capture["wheel_center"][tick, side]
    local = np.r_[frame.T @ force, frame.T @ (moment_o - np.cross(center - origin, force))]
    # Independent inverse transform catches the moment-translation sign/reference convention.
    recovered = frame @ local[3:] + np.cross(center - origin, frame @ local[:3])
    return local, np.r_[force, moment_o], np.r_[frame @ local[:3], recovered]


def actuator_map(oracle: Oracle, reduction: np.ndarray) -> np.ndarray:
    full = np.zeros((oracle.model.nv, 6))
    for col, actuator in enumerate(oracle.actuators):
        joint = int(oracle.model.actuator_trnid[actuator, 0]); full[int(oracle.model.jnt_dofadr[joint]), col] = -float(oracle.model.actuator_gear[actuator, 0])
    return reduction.T @ full


def wrench_generalized_map(oracle: Oracle, patch: ContinuousPatch, qpos: np.ndarray, side: int) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    reduction, _ = oracle.reduction(qpos); g, _, _ = geometry_map(patch, qpos, side); frame = rc(g, patch.n); center = np.asarray(g["contact_center"])
    linear = np.zeros((3, oracle.model.nv)); angular = np.zeros_like(linear)
    mujoco.mj_jac(oracle.model, oracle.data, linear, angular, center, int(g["body"]))
    B = np.hstack(((linear @ reduction).T @ frame, (angular @ reduction).T @ frame))
    return B, reduction, g


def canonical_reconstruct(oracle: Oracle, captured: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
    # Preserve base pose and active coordinates; recompute the ideal-closure passive state.
    q = captured.copy(); q[oracle.passive_qpos] = oracle.equilibrium_passive
    return oracle.solve_passive(q)


def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("--config", type=Path, required=True); ap.add_argument("--output-dir", type=Path, required=True); args = ap.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)
    cfg_path = args.config.resolve(); cfg, cfg_inputs = load_config(cfg_path)
    model_cfg, model_inputs = load_config((ROOT / cfg["model_profile"]).resolve()); equilibrium = json.loads((ROOT / model_cfg["equilibrium"]).read_text())
    cont_cfg, cont_inputs = load_config((ROOT / cfg["continuous_contact_config"]).resolve()); oracle = Oracle(model_cfg, equilibrium); patch = ContinuousPatch(oracle, cont_cfg["continuous_contact_oracle"])
    mu, tol = float(cfg["friction_coefficient"]), float(cfg["membership_tolerance"])
    capture_paths = [ROOT / cfg["capture_v1"], ROOT / cfg["capture_v2"]]; captures = [np.load(x / "capture.npz") for x in capture_paths]
    switches = json.loads((ROOT / cfg["old_switches"]).read_text())
    eq = oracle.sample_qpos(model_cfg["samples"][0]); base_g, base_offsets, base_G = geometry_map(patch, eq, 0); R = rays(base_offsets, mu); Cf = point_inequalities(mu); H, hull = build_h(R, cfg["hull_qhull_options"])

    # Truth-pose geometry verifies constancy; static pose reconstruction is reported separately below.
    states: list[tuple[str, np.ndarray]] = [("equilibrium", eq)]
    states += [(f"phase15_{s['id']}", oracle.sample_qpos(s)) for s in model_cfg["samples"]]
    rng = np.random.default_rng(int(cfg["random_seed"])); er = np.max(np.abs([s["base_rotation_vector_rad"] for s in model_cfg["samples"]]), axis=0); ej = np.max(np.abs([s["canonical_joint_delta_rad"] for s in model_cfg["samples"]]), axis=0)
    states += [(f"random_{i:02d}", oracle.sample_qpos({"id": str(i), "base_rotation_vector_rad": rng.uniform(-er, er).tolist(), "canonical_joint_delta_rad": rng.uniform(-ej, ej).tolist()})) for i in range(int(cfg["workspace_random_count"]))]
    for tick in range(1, 272): states.append((f"rolling_{tick}", captures[1]["qpos"][tick]))
    for event in switches:
        for offset in (-1, 0, 1):
            tick = max(1, min(271, int(event["tick"]) + offset)); states.append((f"switch_{event['side']}_{tick}", captures[1]["qpos"][tick]))
    geom_rows = []
    for sid, q in states:
        for side in range(2):
            _, off, G = geometry_map(patch, q, side); geom_rows.append({"state": sid, "side": side, "offsets_C_m": off.tolist(), "G_C": G.tolist(), "rank": int(np.linalg.matrix_rank(G)), "singular_values": np.linalg.svd(G, compute_uv=False).tolist()})
    variation = {"offset_elementwise": max(mabs(np.asarray(x["offsets_C_m"]) - base_offsets) for x in geom_rows), "G_elementwise": max(mabs(np.asarray(x["G_C"]) - base_G) for x in geom_rows), "G_frobenius": max(float(np.linalg.norm(np.asarray(x["G_C"]) - base_G)) for x in geom_rows)}

    # Independent A(point LP), B(rays), C(H) corpus.  Truth includes both frozen capture sources.
    cases: list[tuple[str, np.ndarray]] = [("zero", np.zeros(6))]
    test_f = np.zeros(18); test_f[2::3] = 1.; cases += [("pure_normal", base_G @ test_f)]
    edge = np.zeros((6,3)); edge[:,2] = 1.; edge[:,0] = .999999; cases.append(("near_friction_edge", base_G @ edge.ravel()))
    tangent = np.zeros((6,3)); tangent[:,2] = 1.; tangent[:,0] = .6; tangent[:,1] = -.4; cases.append(("pure_tangential_support", base_G @ tangent.ravel()))
    rolling_moment = np.zeros((6,3)); rolling_moment[2,2] = 1.; rolling_moment[3,2] = -1.; cases.append(("rolling_lateral_moment", base_G @ rolling_moment.ravel()))
    torsion = R @ np.eye(24)[:, [0, 7, 12, 19]].sum(axis=1); cases.append(("feasible_torsional_combination", torsion))
    asymmetric = np.zeros((6,3)); asymmetric[0] = [.5,-.2,2.]; asymmetric[5] = [-.4,.6,3.]; cases.append(("asymmetric_distribution", base_G @ asymmetric.ravel()))
    for i in range(40):
        f = rng.uniform(0., 3., 18).reshape(6, 3); f[:, :2] = rng.uniform(-1., 1., (6, 2)) * f[:, 2:3]; cases.append((f"point_random_{i:02d}", base_G @ f.ravel()))
        alpha = rng.uniform(0., 3., 24); cases.append((f"ray_random_{i:02d}", R @ alpha))
    # Explicitly infeasible: negative normal and oversized torsion at unit normal.
    cases += [("infeasible_negative_normal", np.array([0., 0., -1., 0., 0., 0.])), ("infeasible_torsion", np.array([0., 0., 1., 0., 0., 1.]))]
    for i, facet in enumerate(H):
        # Average all generating rays lying on this facet to make a reproducible boundary point.
        on_facet = np.flatnonzero(np.abs(facet @ R) <= 1e-8)
        candidate = np.zeros(6) if not len(on_facet) else np.mean(R[:, on_facet], axis=1)
        cases.append((f"facet_{i:02d}_boundary", candidate)); cases.append((f"facet_{i:02d}_outward", candidate + 1e-5 * facet))
    truth_rows, sign_error = [], 0.
    for cp, capture in zip(capture_paths, captures):
        for tick in range(1, min(271, len(capture["tick"]) - 2) + 1):
            for side in range(2):
                if capture["truth_force"][tick, side, 2] < 1.: continue
                w, original, recovered = local_truth(capture, tick, side, patch); sign_error = max(sign_error, mabs(original - recovered)); cases.append((f"truth_{cp.name}_{tick}_{side}", w)); truth_rows.append({"capture": cp.name, "tick": tick, "side": side, "w_C": w.tolist()})
    members = []
    for cid, w in cases:
        a, _, ea = lp_member(base_G, w, Cf, tol); b, _, eb = v_member(R, w, tol); c, ec = h_member(H, w, tol)
        members.append({"case": cid, "point_lp": a, "ray_v": b, "facet_h": c, "point_error": ea, "ray_error": eb, "facet_maximum": ec, "consistent": a == b == c})

    # Transform/generalized force and virtual work using arbitrary distributions, including same wrench + nullspace differences.
    transform_rows = []
    selected = [("equilibrium", eq), *[(f"rolling_{t}", captures[1]["qpos"][t]) for t in (1, 68, 136, 204, 240, 255, 271)]]
    for event in switches:
        for dt in (-1, 0, 1): selected.append((f"switch_{event['side']}_{int(event['tick'])+dt}", captures[1]["qpos"][max(1, min(271, int(event["tick"]) + dt))]))
    for sid, q in selected:
        for side in range(2):
            B, red, g = wrench_generalized_map(oracle, patch, q, side); frame = rc(g, patch.n); _, _, G = geometry_map(patch, q, side); J = patch.force_jacobian(q, red, side)
            for k in range(3):
                f = rng.normal(size=18); w = G @ f; qpoint = sum(J[i].T @ frame @ f[3*i:3*i+3] for i in range(6)); qcond = B @ w
                delta = rng.normal(size=12); transform_rows.append({"state": sid, "side": side, "generalized_error": mabs(qpoint-qcond), "virtual_work_error": abs(float(delta@qpoint-w@(B.T@delta)))})
                null = np.linalg.svd(G, full_matrices=True)[2][6:].T @ rng.normal(size=12)
                f2 = f + null; q2 = sum(J[i].T @ frame @ f2[3*i:3*i+3] for i in range(6))
                transform_rows[-1].update({"same_wrench_internal_error": mabs(G@f2-w), "same_wrench_generalized_error": mabs(q2-qpoint)})

    # Static/local feasibility uses the primary condensed variables [tau,wL,wR],
    # with fixed H_C directly.  Point forces are witnesses, never primary variables.
    static_inputs = [("equilibrium", eq, {"closure_residual_m": 0.})]
    static_inputs += [(f"phase15_{s['id']}", oracle.sample_qpos(s), {"closure_residual_m": 0.}) for s in model_cfg["samples"]]
    for i in range(8):
        static_inputs.append((f"random_{i:02d}", states[5+i][1], {"closure_residual_m": 0.}))
    static_ticks = sorted(set([1, 68, 136, 204, 240, 255, 271] + [max(1,min(271,int(x["tick"])+d)) for x in switches for d in (-1,0,1)]))
    for tick in static_ticks:
        q, metrics = canonical_reconstruct(oracle, captures[1]["qpos"][tick]); static_inputs.append((f"rolling_reconstructed_{tick}", q, metrics))
    pre_static = (variation["offset_elementwise"] <= cfg["geometry_variation_tolerance"] and variation["G_elementwise"] <= cfg["map_variation_tolerance"] and all(x["rank"] == 6 for x in geom_rows) and all(x["consistent"] for x in members) and sign_error <= cfg["transform_tolerance"] and max(x["generalized_error"] for x in transform_rows) <= cfg["transform_tolerance"] and max(x["virtual_work_error"] for x in transform_rows) <= cfg["virtual_work_tolerance"])
    static_rows = []
    bounds = np.asarray(cfg["torque_bounds_nm"])
    for sid, q, recon in static_inputs:
        if not pre_static:
            static_rows.append({"state": sid, "skipped": True, "reason": "fixed-cone prerequisite failed", "reconstruction": recon}); continue
        oracle.forward(q, np.zeros(oracle.model.nv)); red, _ = oracle.reduction(q); oracle.forward(q, np.zeros(oracle.model.nv)); bias = red.T @ oracle.data.qfrc_bias.copy(); A = actuator_map(oracle, red)
        Bs = [wrench_generalized_map(oracle, patch, q, side)[0] for side in range(2)]
        M = np.hstack((A, Bs[0], Bs[1])); A_ub = np.zeros((2 * len(H), 18)); A_ub[:len(H), 6:12] = H; A_ub[len(H):, 12:18] = H
        result = linprog(np.zeros(18), A_ub=A_ub, b_ub=np.zeros(2 * len(H)), A_eq=M, b_eq=bias, bounds=[(-x, x) for x in bounds] + [(None, None)]*12, method="highs")
        mv = np.hstack((A, Bs[0] @ R, Bs[1] @ R)); rv = linprog(np.zeros(54), A_eq=mv, b_eq=bias, bounds=[(-x, x) for x in bounds] + [(0., None)] * 48, method="highs")
        mp = np.hstack((A, Bs[0] @ base_G, Bs[1] @ base_G)); cp = np.zeros((2 * len(Cf), 42)); cp[:len(Cf), 6:24] = Cf; cp[len(Cf):, 24:42] = Cf
        rp = linprog(np.zeros(42), A_ub=cp, b_ub=np.zeros(2 * len(Cf)), A_eq=mp, b_eq=bias, bounds=[(-x, x) for x in bounds] + [(None, None)] * 36, method="highs")
        representations = {"H_18": {"success": bool(result.success), "status": result.message, "residual": None if not result.success else mabs(M @ result.x - bias)}, "V_54": {"success": bool(rv.success), "status": rv.message, "residual": None if not rv.success else mabs(mv @ rv.x - bias)}, "point_42": {"success": bool(rp.success), "status": rp.message, "residual": None if not rp.success else mabs(mp @ rp.x - bias)}}
        if result.success:
            tau, wl, wr = result.x[:6], result.x[6:12], result.x[12:18]
            residual = mabs(M@result.x-bias); cone_l, _, _ = v_member(R, wl, tol); cone_r, _, _ = v_member(R, wr, tol)
            lo, lf, le = lp_member(base_G, wl, Cf, tol); ro, rf, re = lp_member(base_G, wr, Cf, tol)
            static_rows.append({"state": sid, "feasible": True, "hard_residual": residual, "tau_nm": tau.tolist(), "minimum_torque_margin_nm": float(np.min(bounds-np.abs(tau))), "wrench_left_C": wl.tolist(), "wrench_right_C": wr.tolist(), "cone_membership": bool(cone_l and cone_r), "cone_margin": float(min(-np.max(H@wl), -np.max(H@wr))), "active_facets": {"left": np.flatnonzero(np.abs(H@wl) <= tol).tolist(), "right": np.flatnonzero(np.abs(H@wr) <= tol).tolist()}, "point_force_witness": [None if lf is None else lf.tolist(), None if rf is None else rf.tolist()], "point_witness_errors": [le,re], "witness_feasible": bool(lo and ro), "representations": representations, "representation_consistent": len({x["success"] for x in representations.values()}) == 1, "reconstruction": recon})
        else: static_rows.append({"state": sid, "feasible": False, "linprog_status": result.message, "representations": representations, "representation_consistent": len({x["success"] for x in representations.values()}) == 1, "reconstruction": recon})

    gates = {"fixed_geometry": variation["offset_elementwise"] <= cfg["geometry_variation_tolerance"] and variation["G_elementwise"] <= cfg["map_variation_tolerance"], "rank_nullspace": all(x["rank"] == 6 for x in geom_rows), "point_v_h_membership": all(x["consistent"] for x in members), "truth_membership": all(x["consistent"] and x["point_lp"] for x in members if x["case"].startswith("truth_")), "moment_shift_sign": sign_error <= cfg["transform_tolerance"], "transform": max(x["generalized_error"] for x in transform_rows) <= cfg["transform_tolerance"], "virtual_work": max(x["virtual_work_error"] for x in transform_rows) <= cfg["virtual_work_tolerance"], "static": pre_static and all(x.get("feasible",False) and x.get("representation_consistent",False) and x["hard_residual"] <= cfg["static_residual_tolerance"] and x["minimum_torque_margin_nm"] >= cfg["static_margin_tolerance"] and x["cone_membership"] and x["witness_feasible"] for x in static_rows)}
    summary = {"schema_version": 1, "phase": 21, "profile": cfg["profile"], "wrench_order": ["Fr","Fl","Fn","Mr","Ml","Mn"], "reference": "analytic contact center; continuous contact frame", "fixed_point_cone": "six points, mu=1, no total normal-force upper bound", "coverage": {"geometry_states": len(geom_rows), "truth_samples": len(truth_rows), "static_cases": len(static_rows)}, "geometry_variation": variation, "rank": {"rank": 6, "nullspace_dimension": 12, "singular_values": np.linalg.svd(base_G, compute_uv=False).tolist(), "condition": float(np.linalg.cond(base_G))}, "rays": {"raw": 24, "unique": int(np.unique(R.T, axis=0).shape[0])}, "h_representation": hull, "moment_shift_sign_error": sign_error, "membership": {"count": len(members), "inconsistent": sum(not x["consistent"] for x in members), "truth_count": len(truth_rows)}, "transform_maximum_generalized_error": max(x["generalized_error"] for x in transform_rows), "transform_maximum_virtual_work_error": max(x["virtual_work_error"] for x in transform_rows), "static": {"count": len(static_rows), "feasible": sum(x["feasible"] for x in static_rows)}, "hard_constraint_dependency_audit": "No current hard constraint/task was assembled or modified by this offline oracle; no claim beyond the frozen Phase-21 documents and the inspected legacy QP validator is made.", "soft_pfaffian": "unchanged; outside this oracle", "gates": gates, "pass": all(gates.values()), "42d_candidate": "mathematically authorized only if all gates pass; this oracle does not authorize production QP/Core integration."}
    dump(output/"summary.json", summary); dump(output/"geometry.json", geom_rows); dump(output/"membership.json", members); dump(output/"truth_wrenches.json", truth_rows); dump(output/"transform.json", transform_rows); dump(output/"static.json", static_rows); dump(output/"H_C.json", {"H": H.tolist(), **hull})
    script = Path(__file__).resolve(); outs = ["summary.json","geometry.json","membership.json","truth_wrenches.json","transform.json","static.json","H_C.json"]
    dump(output/"manifest.json", {"schema_version":1,"created_at":datetime.now(timezone.utc).isoformat(),"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy_version,"mujoco":mujoco.__version__,"config_inputs":{str(p.relative_to(ROOT)):digest(p) for p in cfg_inputs+cont_inputs},"model_inputs":{str(p.relative_to(ROOT)):digest(p) for p in model_inputs},"captures":{str(p.relative_to(ROOT)): {"capture":digest(p/"capture.npz"),"manifest":digest(p/"manifest.json")} for p in capture_paths},"switches":digest(ROOT/cfg["old_switches"]),"validator":str(script.relative_to(ROOT)),"validator_sha256":digest(script),"outputs":{x:digest(output/x) for x in outs}})
    print(json.dumps(summary, indent=2, sort_keys=True)); return 0 if summary["pass"] else 1


if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as e: print(f"ERROR: {e}", file=sys.stderr); sys.exit(2)
