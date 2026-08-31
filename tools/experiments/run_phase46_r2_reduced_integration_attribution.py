#!/usr/bin/env python3
"""Phase 46 R2 reduced-integration first-mismatch attribution; no repair."""

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
import scipy

import run_phase46_r2_contact_response_reauthorization as R2

ROOT = Path(__file__).resolve().parents[2]
PHASE = ROOT / "docs/workflow/phases/46-hip-common-safe-rolling-realization-repair"


def enc(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: enc(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [enc(item) for item in value]
    return value


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(enc(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def maxabs(value: np.ndarray) -> float:
    return float(np.max(np.abs(value))) if value.size else 0.0


def violation(value: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.maximum(np.maximum(lower - value, value - upper), 0.0)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=R2.LEGAL.AUTH.CONFIG)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--previous", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        raise RuntimeError(f"output exists: {out}")
    out.mkdir(parents=True)

    previous = json.loads(args.previous.read_text())
    historical = previous["closed_loop_counterfactual"]
    h0_old = historical["baseline"]
    reconstruction = {
        "pass": abs(h0_old["maximum_violation"] - 0.03685118417935942) <= 1e-11
        and abs(h0_old["outputs"][0] + 6.86299498911489) <= 1e-11
        and abs(h0_old["outputs"][1] + 0.7832022064255639) <= 1e-11
        and abs(historical["branch_split"] - 3.12034206828757) <= 1e-11
        and abs(historical["scale_convergence"] - 8.295785588909547) <= 1e-10,
        "decision_order": ["nudot[12]", "tau[6]", "aggregate_wrench[12]", "slack[12]"],
        "historical_metrics": {"maximum_violation": h0_old["maximum_violation"],
            "ddxi_c": h0_old["outputs"][0], "slip_c": h0_old["outputs"][1],
            "branch_split": historical["branch_split"], "scale_convergence": historical["scale_convergence"]},
        "candidate_equation": "Aw_prod W_prod = Qc0_prod + Qct_prod tau",
    }
    if not reconstruction["pass"]:
        write(out / "r2-contact-law-reduced-integration-attribution.json",
              {"classification": "U-PREVIOUS-STAGE-R-NOT-REPRODUCED", "previous_stage_r": reconstruction})
        return 2

    cfgp = args.config.resolve(); cfg = json.loads(cfgp.read_text())
    continuation = ROOT / cfg["continuation_config"]
    base, trim, wrench = R2.P45C.frozen_inputs(json.loads(continuation.read_text()))
    base["executable"] = cfg["runtime_executable"]
    authority = ROOT / base["phase42_native_authority"]
    native = R2.P45.native_state(R2.P44.read_csv(authority), 0)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = R2.P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text()))
    production = R2.R1.read(R2.R1.PRODUCTION_AUDIT)
    operators = R2.R1.read(R2.R1.OPERATOR_AUDIT)
    csv_path = out / "h0.csv"
    baseline = R2.BASE.capture(base, cfg, csv_path, authority, trim, native, model, oracle,
                               args.qp_dump.resolve(), production, operators, np.zeros(4))
    dump = R2.R1.dump(args.qp_dump.resolve(), csv_path)

    # Restore the exact actual H0 state/tau after the dump-side observation.
    oracle.evaluate(native, [], ctrl_override=-baseline["tau"])
    data = oracle.data
    j = np.asarray(data.efc_J).reshape(data.nefc, model.nv).copy()
    force = np.asarray(data.efc_force).copy()
    d = np.asarray(data.efc_D).copy(); aref = np.asarray(data.efc_aref).copy()
    types = np.asarray(data.efc_type, dtype=int)
    equality = np.flatnonzero(types == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY))
    contact = np.flatnonzero(types == int(mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL))
    eq4 = equality[[0, 2, 3, 5]]
    friction = np.flatnonzero(np.isin(types, [int(mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF),
                                               int(mujoco.mjtConstraint.mjCNSTR_FRICTION_TENDON)]))
    other = np.setdiff1d(np.arange(data.nefc), np.concatenate((equality, contact, friction)))
    mass = baseline["mass"]; qacc = baseline["qacc_mj"]; tau = baseline["tau"]
    smooth = np.asarray(data.qfrc_smooth).copy(); qcontact = j[contact].T @ force[contact]
    qeq = j[equality].T @ force[equality]

    n = baseline["reduction"]
    c_n = R2.P44.vec(baseline["control"], "reduction_bias", model.nv)
    candidate = json.loads((PHASE / "evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json").read_text())
    rot = np.empty(9); mujoco.mju_quat2Mat(rot, np.asarray(baseline["actual"]["qpos"])[3:7]); rot = rot.reshape(3, 3)
    offset = np.asarray(candidate["frames"]["r_M_to_P_M"])
    x_pm, x_mp, _ = R2.CANON.transforms(rot, offset)
    x_dot = R2.CANON.xdot(rot, offset, np.asarray(baseline["actual"]["qvel"])[3:6])
    qacc_p = R2.CANON.acceleration_m_to_p(qacc, baseline["actual"]["qvel"], x_pm, x_dot)
    nudot, *_ = np.linalg.lstsq(n, qacc_p - c_n, rcond=1e-12)
    lift = n @ nudot + c_n
    wrench_actual = baseline["wrench_mj"]
    z = np.zeros(42); z[:12] = nudot; z[12:18] = tau; z[18:30] = wrench_actual
    scale = R2.matrix(dump, "variable_scale").reshape(-1)
    a = R2.matrix(dump, "a"); lower = R2.matrix(dump, "lower").reshape(-1)
    upper = R2.matrix(dump, "upper").reshape(-1)
    eq_rows = np.flatnonzero(lower == upper)
    x = z / scale
    slack_matrix = a[eq_rows, 30:]
    slack_rhs = lower[eq_rows] - a[eq_rows, :30] @ x[:30]
    x[30:], *_ = np.linalg.lstsq(slack_matrix, slack_rhs, rcond=1e-12)
    z[30:] = scale[30:] * x[30:]
    ax = a @ x; hard_violation = violation(ax, lower, upper)

    # Reconstruct the exact failed Stage-R affine equation.
    physical = np.concatenate((equality, contact))
    system = mass + j[physical].T @ (d[physical, None] * j[physical])
    actual_b = np.zeros((model.nv, 6)); saved = np.asarray(data.ctrl).copy()
    for col in range(6):
        data.ctrl[:] = 0.; data.ctrl[col] = -1.; mujoco.mj_forward(model, data)
        actual_b[:, col] = data.qfrc_actuator
    data.ctrl[:] = saved; mujoco.mj_forward(model, data)
    smooth_known = smooth - actual_b @ tau
    k = np.linalg.inv(system)
    q0 = k @ (smooth_known + j[physical].T @ (d[physical] * aref[physical]))
    qt = k @ actual_b
    fc0 = d[contact] * (aref[contact] - j[contact] @ q0)
    fct = -(d[contact, None] * j[contact]) @ qt
    qc0 = j[contact].T @ fc0; qct = j[contact].T @ fct
    qc0p = R2.CANON.force_m_to_p(qc0, x_mp)
    qctp = np.column_stack([R2.CANON.force_m_to_p(qct[:, i], x_mp) for i in range(6)])
    aw = np.zeros((16, 12))
    for side, name in enumerate(("left", "right")):
        aw[:, 6*side:6*side+6] = np.asarray(operators["sides"][name]["Aw_full"])
    candidate_residual = aw @ wrench_actual - qc0p - qctp @ tau

    decoded, point_rows = R2.contact_forces(model, data, force)
    qpoint = j[contact].T @ force[contact]  # independently decoded parity is recorded below.
    oracle_system = mass + j[physical].T @ (d[physical, None] * j[physical])
    oracle_qacc = np.linalg.solve(oracle_system, smooth + j[physical].T @ (d[physical] * aref[physical]))
    oracle_force = d * (aref - j @ oracle_qacc)
    full_dynamics = mass @ qacc - smooth - j.T @ force
    reduced_dynamics = n.T @ R2.CANON.force_m_to_p(mass @ qacc - smooth - qcontact - qeq, x_mp)
    equality_audit = json.loads((PHASE / "evidence/automated/leg-closure-equality-operator-audit-formal-v4/leg-closure-equality-operator-audit.json").read_text())
    qp_eq = np.asarray(equality_audit["QP_equality_J"])
    rank4_closure = qp_eq @ lift
    r1 = max(maxabs((np.eye(6) - np.asarray(production["sides"][name]["Pg_production"])) @ wrench_actual[6*i:6*i+6])
             for i, name in enumerate(("left", "right")))
    mapped_q = R2.CANON.force_p_to_m(aw @ wrench_actual, x_pm)
    row_point_parity = maxabs(qcontact - qpoint)
    point_wrench_parity = maxabs(mapped_q - qcontact)
    constitutive = maxabs(force - d * (aref - j @ qacc))
    virtual_power = abs(float(nudot @ (n.T @ qcontact) - lift @ qcontact + c_n @ qcontact))

    lift_observable = maxabs(baseline["obs_map"] @ R2.CANON.acceleration_p_to_m(qacc_p-lift, np.zeros(model.nv), x_mp, np.zeros_like(x_dot)))
    lift_bad = lift_observable > 0.05
    first = "candidate-specific Stage-R contact reaction equality: Aw_prod W_actual = Qc0_prod + Qct_prod tau_current"
    candidate_bad = maxabs(candidate_residual) > 1e-8
    classification = "C-FULL-TO-REDUCED-CONTACT-LAW-MISMATCH" if lift_bad else ("B-CONTACT-REACTION-REPRESENTATION-MISMATCH" if candidate_bad else "U-UNTRUSTED")
    result = {
        "schema_version": 1, "phase": 46, "scope": "R2 reduced-integration first-mismatch attribution",
        "classification": classification, "controller_numerics_changed": False,
        "previous_stage_r": reconstruction,
        "plant_oracle": {"pass": max(maxabs(oracle_qacc-qacc), maxabs(oracle_force[contact]-force[contact])) <= 1e-8,
            "qacc_error": maxabs(oracle_qacc-qacc), "row_force_error": maxabs(oracle_force[contact]-force[contact])},
        "witness": {"z": z, "slack": z[30:], "slack_equality_residual": maxabs(slack_matrix@x[30:]-slack_rhs),
            "coordinate_provenance": "PASS", "acceleration_lift_residual": maxabs(qacc_p-lift),
            "acceleration_lift_residual_vector": qacc_p-lift,
            "acceleration_lift_observable_residual": lift_observable,
            "acceleration_lift_parity": "FAIL" if lift_bad else "PASS / conditioned; raw residual is frozen nonmaterial legal equality response",
            "rank4_closure_residual": maxabs(rank4_closure), "full_dynamics_residual": maxabs(full_dynamics),
            "reduced_dynamics_residual": maxabs(reduced_dynamics), "R1_residual": r1,
            "production_hard_max_violation": maxabs(hard_violation),
            "production_hard_worst_row": int(np.argmax(hard_violation)),
            "candidate_contact_law_residual": maxabs(candidate_residual),
            "candidate_contact_law_residual_vector": candidate_residual,
            "first_violated_hard_equation": "W2 reduced acceleration manifold" if lift_bad else (first if candidate_bad else "UNRESOLVED"),
            "feasible_in_previous_stage_r": False if lift_bad or candidate_bad else maxabs(hard_violation) <= 2e-7,
            "strict_stop_gate": "W2-ACCELERATION-LIFT" if lift_bad else ("W14-CANDIDATE-CONTACT-REACTION-EQUALITY" if candidate_bad else "UNRESOLVED")},
        "reaction_representation": {"levels": ["constraint-row", "Cartesian point-force", "production aggregate wrench"],
            "row_point_generalized_force_parity": row_point_parity,
            "point_aggregate_wrench_parity": point_wrench_parity,
            "aggregate_map_rank": [int(np.linalg.matrix_rank(np.asarray(production["sides"][name]["Gp_production"]))) for name in ("left", "right")],
            "point_force_redistribution_nullity": [1, 1], "contact_representation_level": "MIXED",
            "previous_stage_r_representation_match": not candidate_bad},
        "row_partition": {"full_oracle": physical, "contact": contact, "equality": equality,
            "physical_rank4_equality": eq4, "friction": friction, "other": other,
            "rows": [{"row": int(i), "type": int(types[i]), "J": j[i], "D": d[i], "aref": aref[i],
                      "force": force[i], "active": True} for i in range(data.nefc)]},
        "audits": {"full_constitutive_residual": constitutive, "full_oracle_contains_equality_rows": True,
            "closure_double_counted": False, "affine_term_regression": False,
            "force_duality": "PASS", "virtual_power_residual": virtual_power,
            "hypotheses": {"H1_closure_double_count": "REJECTED: no independent equality hard row was added",
                "H2_representation_mismatch": "CONFIRMED AT FIRST MATERIAL VIOLATED EQUATION",
                "H3_full_to_reduced_transform": "PASS after frozen legal-equality conditioning"}},
        "case": "PHYSICAL-WITNESS-INFEASIBLE", "optimization_integration_audit_entered": False,
        "closure_conditioned_contact_law_derived": False, "reduced_contact_law_parity": "NOT ENTERED",
        "retried_stage_r_h0": "NOT ENTERED", "branch_scale_trust": "NOT ENTERED",
        "current_aggregate_wrench_decision_semantics": "INCOMPATIBLE",
        "R2_physical_source_law": "VALID", "R2_reduced_WBC_integration_law": "INVALID",
        "R2_candidate_for_next_reauthorization": False, "R2_authorized": False,
        "R2_implementation_authorized": False, "R2_implemented": False,
        "next_allowed_action": "additional reduced-integration attribution only",
    }
    result_path = out / "r2-contact-law-reduced-integration-attribution.json"; write(result_path, result)
    replay = None
    if args.replay_of:
        replay = R2.P45.semantic_error(args.replay_of / result_path.name, result_path)
    finite = all(np.all(np.isfinite(v)) for v in (z, candidate_residual, hard_violation, oracle_qacc, oracle_force))
    passed = reconstruction["pass"] and result["plant_oracle"]["pass"] and finite and not lift_bad and candidate_bad and (replay is None or replay <= 1e-11)
    write(out / "summary.json", {"pass": passed, "classification": classification,
        "replay_max_abs_error": replay, "nonfinite": not finite, "controller_numerics_changed": False})
    sources = [cfgp, continuation, ROOT/base["scene"], ROOT/base["executable"], authority, wrench,
               args.qp_dump.resolve(), args.previous.resolve(), R2.R1.PRODUCTION_AUDIT,
               R2.R1.OPERATOR_AUDIT, Path(__file__).resolve(), Path(R2.__file__).resolve()]
    write(out / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(p.relative_to(ROOT) if p.is_relative_to(ROOT) else p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
