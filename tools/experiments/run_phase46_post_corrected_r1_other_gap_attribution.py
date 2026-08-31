#!/usr/bin/env python3
"""Phase46 row-wise closure of the post-corrected-R1 authority other gap."""

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

ROOT = Path(__file__).resolve().parents[2]


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


BASE = load(ROOT / "tools/experiments/run_phase46_post_corrected_r1_authority_attribution.py",
            "p46_other_base")
AUTH, R1, P45C, P45, P44, P42 = BASE.AUTH, BASE.R1, BASE.P45C, BASE.P45, BASE.P44, BASE.P42
CHANNELS = ("contact", "equality", "joint_limit", "friction_loss", "other_constraint",
            "passive", "applied", "external_body_applied", "bias", "numerical_remainder")


def encode(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: encode(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [encode(item) for item in value]
    return value


def write(path: Path, value: Any) -> None:
    path.write_text(json.dumps(encode(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def delta(probe: np.ndarray, baseline: np.ndarray, denominator: float) -> np.ndarray:
    return (np.asarray(probe) - np.asarray(baseline)) / denominator


def qforce(item: dict[str, Any], name: str, nv: int) -> np.ndarray:
    return P44.vec(item["actual"]["dynamics"], name, nv)


def channel_force(item: dict[str, Any], baseline: dict[str, Any], name: str,
                  denominator: float, nv: int) -> np.ndarray:
    if name in ("equality", "joint_limit", "friction_loss", "other_constraint"):
        key = "other" if name == "other_constraint" else name
        return delta(item["solver_force_channels"]["generalized"][key],
                     baseline["solver_force_channels"]["generalized"][key], denominator)
    if name == "contact":
        return delta(item["solver_force_channels"]["generalized"]["contact"],
                     baseline["solver_force_channels"]["generalized"]["contact"], denominator)
    if name == "passive":
        return delta(qforce(item, "qfrc_passive", nv), qforce(baseline, "qfrc_passive", nv), denominator)
    if name == "applied":
        return delta(qforce(item, "qfrc_applied", nv), qforce(baseline, "qfrc_applied", nv), denominator)
    if name == "external_body_applied":
        # Oracle explicitly zeros xfrc_applied before every evaluate; preserve this separate branch.
        assert np.max(np.abs(item["solver_force_channels"]["xfrc_applied"])) == 0.0
        return np.zeros(nv)
    if name == "bias":
        return -delta(item["bias"], baseline["bias"], denominator)
    raise KeyError(name)


def branch(item: dict[str, Any], baseline: dict[str, Any], denominator: float,
           production: dict[str, Any], operator_source: dict[str, Any], model: mujoco.MjModel) -> dict[str, Any]:
    nv, obs, mass = model.nv, baseline["obs_map"], baseline["mass"]
    actuator = delta(item["forces"]["actuator"], baseline["forces"]["actuator"], denominator)
    delta_w = delta(item["wrench_qp"], baseline["wrench_qp"], denominator)
    qp_contact = sum(np.asarray(operator_source["sides"][side]["Aw_full"]) @
                     delta_w[6 * index:6 * index + 6]
                     for index, side in enumerate(("left", "right")))
    qp_qacc = delta(item["qacc_qp"], baseline["qacc_qp"], denominator)
    qp_bias = channel_force(item, baseline, "bias", denominator, nv)
    qp_equality = mass @ qp_qacc - actuator - qp_contact - qp_bias
    mj_forces = {name: channel_force(item, baseline, name, denominator, nv)
                 for name in CHANNELS if name != "numerical_remainder"}
    mj_qacc = delta(item["qacc_mj"], baseline["qacc_mj"], denominator)
    mj_sum = actuator + sum(mj_forces.values())
    mj_forces["numerical_remainder"] = mass @ mj_qacc - mj_sum
    zeros = np.zeros(nv)
    qp_forces = {name: zeros.copy() for name in CHANNELS}
    qp_forces.update({"contact": qp_contact, "equality": qp_equality, "bias": qp_bias})
    mj_y = {name: obs @ np.linalg.solve(mass, force) for name, force in mj_forces.items()}
    qp_y = {name: obs @ np.linalg.solve(mass, force) for name, force in qp_forces.items()}
    gaps = {name: mj_y[name] - qp_y[name] for name in CHANNELS}
    previous = (BASE.mode4(delta(item["mj"], baseline["mj"], denominator)) -
                BASE.mode4(delta(item["qp"], baseline["qp"], denominator)))
    contact_gap = gaps["contact"]
    other_gap = sum(gaps[name] for name in CHANNELS if name != "contact")
    point_contact = delta(item["forces"]["contact"], baseline["forces"]["contact"], denominator)
    row_contact = mj_forces["contact"]
    return {
        "signed_delta": denominator,
        "force_channels": {name: {"QP": qp_forces[name], "MJ": mj_forces[name],
                                   "MJ_minus_QP": mj_forces[name] - qp_forces[name]}
                           for name in CHANNELS},
        "observable_channels": {name: {"QP": qp_y[name], "MJ": mj_y[name],
                                        "MJ_minus_QP": gaps[name]} for name in CHANNELS},
        "actuator_free": obs @ np.linalg.solve(mass, actuator),
        "QP_output": BASE.mode4(delta(item["qp"], baseline["qp"], denominator)),
        "MJ_output": BASE.mode4(delta(item["mj"], baseline["mj"], denominator)),
        "contact_gap": contact_gap, "other_gap": other_gap,
        "total_discrepancy": previous,
        "other_gap_closure_max_abs": float(np.max(np.abs(previous - contact_gap - other_gap))),
        "QP_dynamics_closure_max_abs": float(np.max(np.abs(
            mass @ qp_qacc - actuator - sum(qp_forces.values())))),
        "MJ_dynamics_closure_max_abs": float(np.max(np.abs(mass @ mj_qacc - actuator - sum(mj_forces.values())))),
        "constraint_row_reconstruction_max_abs": item["solver_force_channels"]["row_reconstruction_max_abs"],
        "contact_row_vs_point_force_max_abs": float(np.max(np.abs(row_contact - point_contact))),
        "smooth_nonactuator_force": delta(item["solver_force_channels"]["qfrc_smooth"],
                                           baseline["solver_force_channels"]["qfrc_smooth"], denominator) - actuator,
        "efc_type_counts": {str(kind): int(np.count_nonzero(item["solver_force_channels"]["efc_type"] == kind))
                            for kind in np.unique(item["solver_force_channels"]["efc_type"])},
        "R1": item["r1"], "regime": item["regime"],
    }


def central(rows: dict[tuple[str, int, float], dict[str, Any]], direction: str,
            key: str) -> np.ndarray:
    return 0.5 * (np.asarray(rows[(direction, -1, 1.0)][key]) +
                  np.asarray(rows[(direction, 1, 1.0)][key]))


def central_channels(rows: dict[tuple[str, int, float], dict[str, Any]], direction: str,
                     family: str) -> dict[str, Any]:
    result = {}
    for name in CHANNELS:
        result[name] = {}
        for side in ("QP", "MJ", "MJ_minus_QP"):
            result[name][side] = 0.5 * (
                np.asarray(rows[(direction, -1, 1.0)][family][name][side]) +
                np.asarray(rows[(direction, 1, 1.0)][family][name][side]))
    return result


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1.0e-12))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=AUTH.CONFIG)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args(); output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config_path, qp_dump = args.config.resolve(), args.qp_dump.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation_path = ROOT / config["continuation_config"]
    continuation = json.loads(continuation_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation); base["executable"] = config["runtime_executable"]
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    production, operators = R1.read(R1.PRODUCTION_AUDIT), R1.read(R1.OPERATOR_AUDIT)
    baseline = BASE.capture(base, config, probes / "baseline.csv", authority, trim, native,
                            model, oracle, qp_dump, production, operators, np.zeros(4))
    specs = (("slip_common", 2, np.ones(2)),
             ("slip_differential", 2, np.asarray([-1.0, 1.0])),
             ("xi_common", 0, np.ones(2)))
    amount, scales = float(config["delta_m_s2"]), list(map(float, config["delta_scales"]))
    rows: dict[tuple[str, int, float], dict[str, Any]] = {}
    for name, start, vector in specs:
        for sign in (-1, 1):
            for scale in scales:
                task = np.zeros(4); task[start:start+2] = sign * scale * amount * vector
                item = BASE.capture(base, config, probes / f"{name}-{scale:g}-{sign:+d}.csv",
                                    authority, trim, native, model, oracle, qp_dump,
                                    production, operators, task)
                rows[(name, sign, scale)] = branch(item, baseline, sign * scale * amount,
                                                   production, operators, model)
    directions = {}
    for name, _, _ in specs:
        directions[name] = {
            "force_channels": central_channels(rows, name, "force_channels"),
            "observable_channels": central_channels(rows, name, "observable_channels"),
            **{key: central(rows, name, key) for key in
               ("actuator_free", "QP_output", "MJ_output", "contact_gap", "other_gap", "total_discrepancy")},
        }
    branch_split = max(relative(np.asarray(rows[(name, -1, 1.0)]["other_gap"]),
                                np.asarray(rows[(name, 1, 1.0)]["other_gap"]))
                       for name, _, _ in specs)
    scale_error = 0.0
    for name, _, _ in specs:
        for sign in (-1, 1):
            reference = np.asarray(rows[(name, sign, 1.0)]["other_gap"])
            scale_error = max(scale_error, *(relative(reference, np.asarray(rows[(name, sign, scale)]["other_gap"]))
                                             for scale in scales))
    max_force_closure = max(max(row[key] for key in
                                ("QP_dynamics_closure_max_abs", "MJ_dynamics_closure_max_abs",
                                 "constraint_row_reconstruction_max_abs", "contact_row_vs_point_force_max_abs"))
                            for row in rows.values())
    max_other_closure = max(row["other_gap_closure_max_abs"] for row in rows.values())
    sc = directions["slip_common"]["observable_channels"]
    contributions = {name: float(sc[name]["MJ_minus_QP"][1]) for name in CHANNELS}
    other_channels = [name for name in CHANNELS if name != "contact"]
    primary = max(other_channels, key=lambda name: abs(contributions[name]))
    target = float(directions["slip_common"]["other_gap"][1])
    fraction = contributions[primary] / target
    trust = (all(row["R1"]["pass"] and row["regime"]["stable"] for row in rows.values()) and
             max_force_closure <= 1.0e-10 and max_other_closure <= 1.0e-10 and
             branch_split <= config["maximum_directional_split_relative"] and
             scale_error <= config["maximum_directional_convergence_relative"])
    classification = "D-NONCONTACT-CONSTRAINT-GAP" if trust and primary == "equality" else "U-UNTRUSTED"
    result = {
        "schema_version": 1, "phase": 46,
        "scope": "post-corrected-R1 compatible-H0 tick0 fixed-state other-gap attribution only",
        "derivative": "(probe-baseline)/signed_delta", "outputs": BASE.OUTPUTS,
        "mujoco_dynamics_convention": "M*qacc + qfrc_bias = qfrc_actuator + qfrc_passive + qfrc_applied + qfrc_constraint",
        "qp_dynamics_convention": "M*qacc_QP + bias = actuator + production-contact + modeled equality reaction",
        "baseline_force_channels": {name: qforce(baseline, name, model.nv) for name in
                                    ("qfrc_actuator", "qfrc_passive", "qfrc_applied", "qfrc_constraint", "qfrc_bias")},
        "baseline_constraint_rows": baseline["solver_force_channels"],
        "directions": directions, "all_probe_channels": [dict(direction=key[0], branch=key[1], scale=key[2], **value)
                                                           for key, value in rows.items()],
        "trust": {"pass": trust, "maximum_force_closure": max_force_closure,
                  "maximum_other_gap_closure": max_other_closure,
                  "branch_split_relative": branch_split, "scale_convergence_relative": scale_error,
                  "fresh_state_R1_regime": all(row["R1"]["pass"] and row["regime"]["stable"] for row in rows.values())},
        "slip_common_other_gap": directions["slip_common"]["other_gap"],
        "primary_side": "CONSTRAINT",
        "primary_concrete_source": "bilateral leg-closure equality response" if primary == "equality" else primary,
        "constraint_source_definition": {
            "type": "equality", "efc_type": int(mujoco.mjtConstraint.mjCNSTR_EQUALITY),
            "equality_ids": [1, 2],
            "equality_names": ["left_leg_closure", "right_leg_closure"],
            "QP_semantics": "modeled implicitly by the frozen plant-constrained reduction",
            "MJ_semantics": "row-wise efc_J.T @ efc_force",
        },
        "primary_signed_contribution_to_slip_common": contributions[primary],
        "primary_fraction_of_other_gap": fraction,
        "is_other_gap_independent_of_contact": True,
        "contact_bookkeeping_overlap": False,
        "classification": classification,
        "contact_response_now_unique_first_mismatch": False,
        "R2_candidate_for_reauthorization": False, "R2_authorized": False,
        "next_repair_layer": "non-contact equality response model gap",
        "next_allowed_action": "define one Phase46 REWORK repair candidate" if trust else "implementation fix only",
    }
    write(output / "post-corrected-r1-other-gap-attribution.json", result)
    replay_error = None if args.replay_of is None else P45.semantic_error(
        args.replay_of / "post-corrected-r1-other-gap-attribution.json",
        output / "post-corrected-r1-other-gap-attribution.json")
    replay_pass = replay_error is None or replay_error <= 1.0e-11
    write(output / "summary.json", {"pass": trust and replay_pass, "classification": classification,
          "replay_max_abs_error": replay_error, "replay_pass": replay_pass,
          "R2_candidate_for_reauthorization": False, "R2_authorized": False,
          "next_allowed_action": result["next_allowed_action"]})
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"],
               authority, wrench_source, qp_dump, R1.PRODUCTION_AUDIT, R1.OPERATOR_AUDIT,
               Path(__file__).resolve(), Path(BASE.__file__), Path(P42.__file__)]
    write(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
          "command": " ".join(sys.argv), "python": sys.version, "platform": platform.platform(),
          "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
          "sources": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                      hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if trust and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
