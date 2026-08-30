#!/usr/bin/env python3
"""Phase45 fixed-state xi/slip common-channel authority attribution only."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import platform
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase45_rework_authority_attribution_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P45C = load(ROOT / "tools/experiments/run_phase45_h0_continuation.py", "p45_attr_cont")
P45 = P45C.P45
P44, P42, EQ = P45C.P44, P45C.P42, P45C.EQ


def vector(row: dict[str, Any], prefix: str, count: int) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{index}"]) for index in range(count)])


def common(values: np.ndarray) -> float:
    return float(0.5 * (values[0] + values[1]))


def flatten(prefix: str, values: np.ndarray) -> dict[str, float]:
    return {f"{prefix}{index}": float(value) for index, value in enumerate(values)}


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1e-12))


def force_terms(actual: dict[str, Any], reduction: np.ndarray) -> dict[str, np.ndarray]:
    q = lambda name: P44.vec(actual["dynamics"], name, reduction.shape[0])
    contact = q("qfrc_contact_left") + q("qfrc_contact_right")
    actuator = q("qfrc_actuator")
    remaining = q("qfrc_passive") + q("qfrc_applied") + q("qfrc_other_constraint")
    mass = P44.vec(actual["dynamics"], "mass", reduction.shape[0] ** 2).reshape(reduction.shape[0], -1)
    lhs = mass @ q("qacc") + q("qfrc_bias")
    return {name: reduction.T @ values for name, values in
            (("contact", contact), ("actuator", actuator), ("remaining", remaining), ("lhs", lhs))}


def sample(base: dict[str, Any], authority: Path, trim: np.ndarray, native: dict[str, str],
           model: mujoco.MjModel, oracle: Any, reduction0: np.ndarray, delta: np.ndarray,
           path: Path) -> dict[str, Any]:
    control = P45.run(base, path, "R45-H0", authority=authority, tick=0,
                      delta=delta, wrench_trim=trim)[0]
    actual = P45.actual(base, model, oracle, native, control)
    material = actual["material"]
    qp, mj = P45C.task_output(control, actual)
    qacc = P44.vec(actual["dynamics"], "qacc", model.nv)
    reduction = P44.matrix(control, "reduction_", model.nv, 12)
    qacc_qp = reduction @ P44.vec(control, "physical_solution", 12) + P44.vec(control, "reduction_bias", model.nv)
    ddxi = np.asarray([actual["dynamics"]["ddxi_left_m_s2"], actual["dynamics"]["ddxi_right_m_s2"]])
    xi_map, xi_bias = P44.native_xi_acceleration_map(
        oracle, actual["qpos"], actual["qvel"], float(native["time_s"]), qacc, ddxi)
    qp_parts = EQ.decompose_xi(xi_map, xi_bias, qacc_qp, oracle.wheel_dadr)
    mj_parts = EQ.decompose_xi(xi_map, xi_bias, qacc, oracle.wheel_dadr)
    lambdas = P44.vec(control, "physical_solution", 30)[18:30]
    maps = [P44.matrix(control, f"contact_map_{side}_", 12, 6) for side in range(2)]
    qp_contact = maps[0] @ lambdas[:6] + maps[1] @ lambdas[6:]
    return {
        "control": control, "actual": actual, "qp_y": np.asarray([common(qp[:2]), common(qp[2:])]),
        "mj_y": np.asarray([common(mj[:2]), common(mj[2:])]),
        "forces": {"qp_contact": qp_contact, **force_terms(actual, reduction0)},
        "qacc_qp": qacc_qp, "qacc_mj": qacc, "xi_qp": qp_parts, "xi_mj": mj_parts,
        "reduction_delta_max_abs": float(np.max(np.abs(reduction - reduction0))),
        "native_wheel_qacc_qp": qacc_qp[oracle.wheel_dadr],
        "native_wheel_qacc_mj": qacc[oracle.wheel_dadr],
        "hard": float(control["hard"]), "slack": float(control["maximum_normalized_slack"]),
        "torque_margin": min(float(control[f"tau_margin{i}"]) for i in range(6)),
        "dynamics_closure": float(actual["dynamics"]["full_dynamics_residual_max_abs"]),
        "contact_closure": float(actual["dynamics"]["contact_applyft_jacobian_max_abs"]),
    }


def branch_gain(probe: dict[str, Any], baseline: dict[str, Any], denominator: float) -> dict[str, Any]:
    gains = {"qp_y": (probe["qp_y"] - baseline["qp_y"]) / denominator,
             "mj_y": (probe["mj_y"] - baseline["mj_y"]) / denominator}
    for name in ("qp_contact", "contact", "actuator", "remaining", "lhs"):
        gains[name] = (probe["forces"][name] - baseline["forces"][name]) / denominator
    gains["native_wheel_qacc_qp"] = (probe["native_wheel_qacc_qp"] - baseline["native_wheel_qacc_qp"]) / denominator
    gains["native_wheel_qacc_mj"] = (probe["native_wheel_qacc_mj"] - baseline["native_wheel_qacc_mj"]) / denominator
    for model in ("qp", "mj"):
        for part in ("base", "leg_nonwheel", "wheel", "jdot_v"):
            gains[f"xi_{model}_{part}"] = common(np.asarray([
                probe[f"xi_{model}"][side][part] - baseline[f"xi_{model}"][side][part]
                for side in range(2)])) / denominator
    return gains


def redirection(gain: dict[str, Any], cfg: dict[str, Any]) -> dict[str, Any]:
    contact, actuator, remaining = gain["contact"], gain["actuator"], gain["remaining"]
    cn, an, rn = map(float, (np.linalg.norm(contact), np.linalg.norm(actuator), np.linalg.norm(remaining)))
    cosine = float(np.dot(contact, actuator) / max(cn * an, 1e-12))
    share = cn / max(cn + an + rn, 1e-12)
    return {"contact_to_actuator_norm_ratio": cn / max(an, 1e-12),
            "contact_actuator_cosine": cosine, "contact_force_share": share,
            "quantitatively_dominant": (cn / max(an, 1e-12) >= cfg["minimum_contact_to_actuator_norm_ratio"] and
                                         cosine <= cfg["maximum_contact_actuator_cosine"] and
                                         share >= cfg["minimum_contact_force_share"])}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args(); output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config_path = args.config.resolve(); config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation_path = ROOT / config["continuation_config"]
    continuation = json.loads(continuation_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    baseline_control = P45.run(base, probes / "baseline.csv", "R45-H0", authority=authority,
                               tick=0, wrench_trim=trim)[0]
    reduction0 = P44.matrix(baseline_control, "reduction_", model.nv, 12)
    baseline = sample(base, authority, trim, native, model, oracle, reduction0, np.zeros(4), probes / "baseline-detail.csv")
    delta = float(config["delta_m_s2"])
    data: dict[tuple[str, float, int], dict[str, Any]] = {}
    probe_rows: list[dict[str, Any]] = []
    for channel, direction in config["input_channels"].items():
        vector_direction = np.asarray(direction, dtype=float)
        for scale in map(float, config["delta_scales"]):
            for sign in (-1, 1):
                values = sign * scale * delta * vector_direction
                item = sample(base, authority, trim, native, model, oracle, reduction0, values,
                              probes / f"{channel}-{scale:g}-{sign:+d}.csv")
                data[(channel, scale, sign)] = item
                probe_rows.append({"channel": channel, "scale": scale, "sign": sign,
                    "delta_ddxi_common_m_s2": values[0], "delta_a_t_common_m_s2": values[2],
                    "qp_ddxi_common_m_s2": item["qp_y"][0], "qp_a_t_common_m_s2": item["qp_y"][1],
                    "mj_ddxi_common_m_s2": item["mj_y"][0], "mj_a_t_common_m_s2": item["mj_y"][1],
                    "native_wheel_qacc_qp_common_rad_s2": common(item["native_wheel_qacc_qp"]),
                    "native_wheel_qacc_mj_common_rad_s2": common(item["native_wheel_qacc_mj"]),
                    "xi_qp_base_common": common(np.asarray([x["base"] for x in item["xi_qp"]])),
                    "xi_qp_leg_nonwheel_common": common(np.asarray([x["leg_nonwheel"] for x in item["xi_qp"]])),
                    "xi_qp_wheel_common": common(np.asarray([x["wheel"] for x in item["xi_qp"]])),
                    "xi_qp_jdot_v_common": common(np.asarray([x["jdot_v"] for x in item["xi_qp"]])),
                    "xi_mj_base_common": common(np.asarray([x["base"] for x in item["xi_mj"]])),
                    "xi_mj_leg_nonwheel_common": common(np.asarray([x["leg_nonwheel"] for x in item["xi_mj"]])),
                    "xi_mj_wheel_common": common(np.asarray([x["wheel"] for x in item["xi_mj"]])),
                    "xi_mj_jdot_v_common": common(np.asarray([x["jdot_v"] for x in item["xi_mj"]])),
                    "hard": item["hard"], "maximum_normalized_slack": item["slack"],
                    "minimum_torque_margin_nm": item["torque_margin"], "dynamics_closure": item["dynamics_closure"],
                    "contact_closure": item["contact_closure"], "reduction_delta_max_abs": item["reduction_delta_max_abs"],
                    **flatten("qp_contact_", item["forces"]["qp_contact"]),
                    **flatten("mj_contact_", item["forces"]["contact"]),
                    **flatten("mj_actuator_", item["forces"]["actuator"]),
                    **flatten("mj_remaining_", item["forces"]["remaining"]),
                    **flatten("mj_lhs_", item["forces"]["lhs"])})

    directional: list[dict[str, Any]] = []
    matrices: dict[str, dict[str, Any]] = {}
    all_trusted = True
    for channel in config["input_channels"]:
        branches: dict[int, dict[float, dict[str, Any]]] = {sign: {} for sign in (-1, 1)}
        for sign in (-1, 1):
            for scale in map(float, config["delta_scales"]):
                gain = branch_gain(data[(channel, scale, sign)], baseline, sign * scale * delta)
                branches[sign][scale] = gain
                reference = branches[sign][1.0] if scale != 1.0 else gain
                conv = max(relative(reference["qp_y"], gain["qp_y"]), relative(reference["mj_y"], gain["mj_y"]))
                trusted = conv <= float(config["maximum_directional_convergence_relative"])
                all_trusted &= trusted
                route = redirection(gain, config["contact_redirection"])
                directional.append({"channel": channel, "branch": "+" if sign > 0 else "-", "scale": scale,
                    "trusted": trusted, "convergence_relative": conv,
                    "g_qp_ddxi_common": gain["qp_y"][0], "g_qp_a_t_common": gain["qp_y"][1],
                    "g_mj_ddxi_common": gain["mj_y"][0], "g_mj_a_t_common": gain["mj_y"][1],
                    "qp_contact_norm": float(np.linalg.norm(gain["qp_contact"])),
                    "mj_contact_norm": float(np.linalg.norm(gain["contact"])),
                    "mj_actuator_norm": float(np.linalg.norm(gain["actuator"])),
                    "mj_remaining_norm": float(np.linalg.norm(gain["remaining"])),
                    **route, **flatten("g_qp_contact_", gain["qp_contact"]),
                    **flatten("g_mj_contact_", gain["contact"]), **flatten("g_mj_actuator_", gain["actuator"]),
                    **flatten("g_mj_remaining_", gain["remaining"]), **flatten("g_mj_lhs_", gain["lhs"]),
                    "g_native_wheel_qacc_qp_common": common(gain["native_wheel_qacc_qp"]),
                    "g_native_wheel_qacc_mj_common": common(gain["native_wheel_qacc_mj"]),
                    **{f"g_{name}": gain[name] for name in gain if name.startswith("xi_")}})
        plus, minus = branches[1][1.0], branches[-1][1.0]
        split = max(relative(plus["qp_y"], minus["qp_y"]), relative(plus["mj_y"], minus["mj_y"]))
        central_allowed = split <= float(config["maximum_directional_split_relative"])
        matrices[channel] = {"input": channel, "output_rows": config["output_rows"],
            "g_qp_plus": plus["qp_y"], "g_qp_minus": minus["qp_y"],
            "g_mj_plus": plus["mj_y"], "g_mj_minus": minus["mj_y"],
            "directional_split_relative": split, "central_average_allowed": central_allowed,
            "g_qp": 0.5 * (plus["qp_y"] + minus["qp_y"]) if central_allowed else None,
            "g_mj": 0.5 * (plus["mj_y"] + minus["mj_y"]) if central_allowed else None}

    if all(value["central_average_allowed"] for value in matrices.values()):
        g_qp = np.column_stack([matrices[name]["g_qp"] for name in config["input_channels"]])
        g_mj = np.column_stack([matrices[name]["g_mj"] for name in config["input_channels"]])
        self_xi = g_qp[0, 0] * g_mj[0, 0] < 0.0
        self_slip = g_qp[1, 1] * g_mj[1, 1] < 0.0
        unified_qp, unified_mj = float(0.5 * np.sum(g_qp)), float(0.5 * np.sum(g_mj))
        cross = (not self_xi and not self_slip and unified_qp * unified_mj < 0.0)
        common_routes = [row["quantitatively_dominant"] for row in directional if row["scale"] == 1.0]
        contact = unified_qp * unified_mj < 0.0 and all(common_routes)
        flags = {"A-XI_SELF_REVERSAL": self_xi, "B-SLIP_SELF_REVERSAL": self_slip,
                 "C-CROSS_COUPLING_REVERSAL": cross, "D-CONTACT_REDIRECTION_DOMINANT": contact}
        active = [name for name, value in flags.items() if value]
        classification = "E-MULTIPLE" if len(active) > 1 else active[0] if active else "U-INSUFFICIENT_OR_UNTRUSTED"
        matrices["unified_reconstructed"] = {"g_qp": g_qp, "g_mj": g_mj,
            "unified_projected_qp": unified_qp, "unified_projected_mj": unified_mj}
    else:
        flags = {}; classification = "U-INSUFFICIENT_OR_UNTRUSTED"

    P45.write_csv(output / "probe-observables.csv", probe_rows)
    P45.write_csv(output / "directional-transfer.csv", directional)
    P45.write_json(output / "common-transfer-matrices.json", matrices)
    P45.write_json(output / "classification.json", {"classification": classification,
        "flags": flags, "central_averages_allowed": all(value["central_average_allowed"] for value in matrices.values() if isinstance(value, dict) and "central_average_allowed" in value),
        "scope_contract": config["scope_contract"]})
    compared = ["probe-observables.csv", "directional-transfer.csv", "common-transfer-matrices.json", "classification.json"]
    replay_error = max(P45.semantic_error(args.replay_of / name, output / name) for name in compared) if args.replay_of else None
    replay_pass = replay_error is None or replay_error <= float(base["gates"]["semantic_replay_max_abs"])
    P45.write_json(output / "summary.json", {"pass": all_trusted and replay_pass, "classification": classification,
        "all_directional_scales_trusted": all_trusted, "replay_max_abs_error": replay_error,
        "scope_contract": config["scope_contract"]})
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"], authority, wrench_source, Path(__file__).resolve()]
    P45.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(), "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if all_trusted and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
