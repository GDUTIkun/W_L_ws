#!/usr/bin/env python3
"""Phase45 compatible-wrench H0 continuation with strict ordered gates."""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase45_contact_consistent_rolling_v2.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P45 = load(ROOT / "tools/experiments/run_phase45_contact_consistent_rolling.py", "p45_cont_base")
EQ = load(ROOT / "tools/experiments/run_phase45_equilibrium_compatibility.py", "p45_cont_eq")
P44, P42 = P45.P44, P45.P42


def frozen_inputs(config: dict[str, Any]) -> tuple[dict[str, Any], np.ndarray, Path]:
    base_path = ROOT / config["base_config"]
    base = json.loads(base_path.read_text(encoding="utf-8"))
    source = ROOT / config["equilibrium_reference"]["source"]
    authority = json.loads(source.read_text(encoding="utf-8"))
    trim = np.asarray(config["equilibrium_reference"]["delta"], dtype=float)
    error = float(np.max(np.abs(trim - np.asarray(authority["trim"], dtype=float))))
    if error > float(config["equilibrium_reference"]["maximum_source_error"]):
        raise RuntimeError(f"frozen compatible wrench differs from authority by {error}")
    return base, trim, source


def signature(control: dict[str, str], observed: dict[str, Any], limits: dict[str, float]) -> str:
    details = observed["details"]
    topology = [sorted((min(int(row["geom1"]), int(row["geom2"])),
                        max(int(row["geom1"]), int(row["geom2"])), int(row["dim"]))
                       for row in details if int(row["side"]) == side) for side in range(2)]
    loads = [observed["dynamics"][name] for name in
             ("normal_load_left_n", "normal_load_right_n")]
    slips = observed["material"]["slip"]
    slack = float(control["maximum_normalized_slack"])
    discrete = {
        "topology": topology,
        "load_positive": [value > limits["positive_normal_load_n"] for value in loads],
        "slip_sign": [0 if abs(value) <= limits["slip_deadband_m_s"] else
                      (1 if value > 0 else -1) for value in slips],
        "statuses": [control[name] for name in
                     ("model_status", "controller_status", "solver_status")],
        "rolling_active": [control["rolling_active_left"], control["rolling_active_right"]],
        "active_counts": [control[f"active_count{i}"] for i in range(3)],
        "active_rows": [control[f"active_row{i}"] for i in range(12, 104)],
        "torque_active": [float(control[f"tau_margin{i}"]) <=
                          limits["inequality_active_distance"] for i in range(6)],
        "slack_state": ("inactive" if slack <= limits["slack_inactive"] else
                        "material" if slack >= limits["slack_material"] else "nonmaterial"),
    }
    return json.dumps(discrete, sort_keys=True, separators=(",", ":"))


def task_output(control: dict[str, str], observed: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    qp = np.asarray([float(control["physical_ddxi_left"]),
                     float(control["physical_ddxi_right"]),
                     float(control["qp_rolling_acceleration_left"]),
                     float(control["qp_rolling_acceleration_right"])])
    actual = np.r_[[observed["dynamics"]["ddxi_left_m_s2"],
                    observed["dynamics"]["ddxi_right_m_s2"]],
                   observed["material"]["tangential_acceleration"]]
    return qp, actual


def projected(values: np.ndarray, mode: str, xi_delta: float, slip_delta: float) -> float:
    xi = 0.5 * (values[0] + values[1]) if mode == "common" else 0.5 * (values[1] - values[0])
    slip = 0.5 * (values[2] + values[3]) if mode == "common" else 0.5 * (values[3] - values[2])
    return 0.5 * (xi / xi_delta + slip / slip_delta)


def initial_authority(base: dict[str, Any], output: Path, authority: Path, trim: np.ndarray,
                      native: dict[str, str], model: mujoco.MjModel, oracle: Any) -> tuple[list[dict[str, Any]], bool]:
    cfg = base["authority"]
    xi_delta, slip_delta = float(cfg["xi_delta_m_s2"]), float(cfg["slip_delta_m_s2"])
    modes = {"common": np.array([1., 1.]), "differential": np.array([-1., 1.])}
    cache: dict[tuple[str, float, int], tuple[np.ndarray, np.ndarray]] = {}
    rows: list[dict[str, Any]] = []
    for mode, signs in modes.items():
        for scale in map(float, cfg["delta_scales"]):
            for sign in (-1, 1):
                delta = np.r_[sign * scale * xi_delta * signs,
                              sign * scale * slip_delta * signs]
                control = P45.run(base, output / f"auth-{mode}-{scale:g}-{sign:+d}.csv",
                                  "R45-H0", authority=authority, tick=0, delta=delta,
                                  wrench_trim=trim)[0]
                cache[(mode, scale, sign)] = task_output(
                    control, P45.actual(base, model, oracle, native, control))
            minus, plus = cache[(mode, scale, -1)], cache[(mode, scale, 1)]
            qp_gain = (plus[0] - minus[0]) / (2.0 * scale)
            mj_gain = (plus[1] - minus[1]) / (2.0 * scale)
            rows.append({"mode": mode, "scale": scale,
                         "g_qp_projected": projected(qp_gain, mode, xi_delta, slip_delta),
                         "g_mj_projected": projected(mj_gain, mode, xi_delta, slip_delta)})
    references = {row["mode"]: row for row in rows if row["scale"] == 1.0}
    for row in rows:
        reference = references[row["mode"]]
        row["qp_convergence_relative"] = abs(row["g_qp_projected"] - reference["g_qp_projected"]) / max(abs(reference["g_qp_projected"]), 1e-12)
        row["mj_convergence_relative"] = abs(row["g_mj_projected"] - reference["g_mj_projected"]) / max(abs(reference["g_mj_projected"]), 1e-12)
        row["sign_match"] = row["g_qp_projected"] * row["g_mj_projected"] > 0.0
    passed = all(row["sign_match"] and
                 abs(row["g_mj_projected"]) >= cfg["minimum_abs_actual_projected_gain"] and
                 row["qp_convergence_relative"] <= cfg["maximum_directional_convergence_relative"] and
                 row["mj_convergence_relative"] <= cfg["maximum_directional_convergence_relative"]
                 for row in rows)
    return rows, passed


def real_audit(base: dict[str, Any], control: dict[str, str], observed: dict[str, Any],
               oracle: Any) -> tuple[list[dict[str, Any]], dict[str, Any], bool]:
    dynamics = observed["dynamics"]
    actual_qacc = P44.vec(dynamics, "qacc", 16)
    reduction = P44.matrix(control, "reduction_", 16, 12)
    qp_qacc = reduction @ P44.vec(control, "physical_solution", 12) + P44.vec(control, "reduction_bias", 16)
    actual_ddxi = np.asarray([dynamics["ddxi_left_m_s2"], dynamics["ddxi_right_m_s2"]])
    xi_map, xi_bias = P44.native_xi_acceleration_map(
        oracle, observed["qpos"], observed["qvel"], float(dynamics["time_s"]),
        actual_qacc, actual_ddxi)
    qp_ddxi = np.asarray([float(control["physical_ddxi_left"]),
                          float(control["physical_ddxi_right"])])
    decompositions = []
    for name, qacc, target in (("qp", qp_qacc, qp_ddxi),
                                ("mujoco", actual_qacc, actual_ddxi)):
        for row in EQ.decompose_xi(xi_map, xi_bias, qacc, oracle.wheel_dadr):
            row.update({"model": name, "target": float(target[row["side"]]),
                        "closure_error": row["sum"] - float(target[row["side"]])})
            decompositions.append(row)
    metrics = {
        "decomposition_closure": max(abs(row["closure_error"]) for row in decompositions),
        "full_dynamics_residual": dynamics["full_dynamics_residual_max_abs"],
        "contact_reconstruction_residual": dynamics["contact_applyft_jacobian_max_abs"],
        "material_formula_residual": observed["material"]["formula_residual"],
        "native_wheel_qdd": [dynamics["wheel_ddq_left_rad_s2"],
                             dynamics["wheel_ddq_right_rad_s2"]],
    }
    gates = base["gates"]
    passed = (metrics["decomposition_closure"] <= gates["maximum_oracle_affine_acceleration_error_m_s2"] and
              metrics["full_dynamics_residual"] <= gates["full_dynamics_max_abs"] and
              metrics["contact_reconstruction_residual"] <= gates["contact_reconstruction_max_abs"] and
              metrics["material_formula_residual"] <= gates["maximum_oracle_affine_acceleration_error_m_s2"])
    return decompositions, metrics, passed


def rollout_metrics(rows: list[dict[str, str]], base: dict[str, Any], expected: int,
                    tick0_wrench_error: float) -> dict[str, Any]:
    gates = base["gates"]
    first = rows[0]
    wheel_rate = np.asarray([[float(row["raw_dq2"]), float(row["raw_dq5"])] for row in rows])
    slip = np.asarray([[float(row["rolling_velocity_left"]),
                        float(row["rolling_velocity_right"])] for row in rows])
    xi = np.asarray([[float(row["xi_left"]), float(row["xi_right"])] for row in rows])
    position = np.asarray([[float(row[f"base_p{i}"]) for i in range(3)] for row in rows])
    rotation = np.asarray([[float(row[f"base_rotvec{i}"]) for i in range(3)] for row in rows])
    linear_speed = np.asarray([math.sqrt(sum(float(row[f"base_v{i}"]) ** 2 for i in range(3))) for row in rows])
    angular_speed = np.asarray([math.sqrt(sum(float(row[f"base_omega{i}"]) ** 2 for i in range(3))) for row in rows])
    wrench_error = max(abs(float(row[f"realized_wrench{i}"]) - float(row[f"requested_wrench{i}"]))
                       for row in rows for i in range(12))
    checks = {
        "full_horizon": len(rows) == expected and int(rows[-1]["tick"]) == expected - 1,
        "bilateral_contact": all(row["contact_left"] == "1" and row["contact_right"] == "1" for row in rows),
        "rolling_active": all(row["rolling_active_left"] == "1" and row["rolling_active_right"] == "1" for row in rows),
        "rate": float(np.max(np.abs(wheel_rate))) <= gates["maximum_abs_wheel_rate_rad_s"],
        "slip": float(np.max(np.abs(slip))) <= gates["maximum_abs_slip_m_s"],
        "xi": float(np.max(np.abs(xi - xi[0]))) <= gates["maximum_abs_xi_error_m"],
        "full_body": (float(np.max(np.linalg.norm(position - position[0], axis=1))) <= gates["maximum_base_position_change_m"] and
                      float(np.max(np.linalg.norm(rotation - rotation[0], axis=1))) <= gates["maximum_base_rotation_change_rad"] and
                      float(np.max(linear_speed)) <= gates["maximum_base_linear_speed_m_s"] and
                      float(np.max(angular_speed)) <= gates["maximum_base_angular_speed_rad_s"]),
        "wbc": (max(float(row["hard"]) for row in rows) <= gates["maximum_hard_violation"] and
                max(float(row["maximum_normalized_slack"]) for row in rows) <= gates["maximum_normalized_slack"] and
                min(float(row[f"tau_margin{i}"]) for row in rows for i in range(6)) >= gates["minimum_torque_margin_nm"] and
                all(row["model_status"] == row["controller_status"] == row["solver_status"] == "0" for row in rows)),
        "wrench": wrench_error <= tick0_wrench_error + gates["maximum_wrench_residual_degradation"],
    }
    return {"pass": all(checks.values()), "checks": checks, "ticks": len(rows),
            "last_tick": int(rows[-1]["tick"]),
            "max_abs_wheel_rate_rad_s": float(np.max(np.abs(wheel_rate))),
            "max_abs_slip_m_s": float(np.max(np.abs(slip))),
            "max_abs_xi_error_m": float(np.max(np.abs(xi - xi[0]))),
            "max_base_position_change_m": float(np.max(np.linalg.norm(position - position[0], axis=1))),
            "max_base_rotation_change_rad": float(np.max(np.linalg.norm(rotation - rotation[0], axis=1))),
            "max_base_linear_speed_m_s": float(np.max(linear_speed)),
            "max_base_angular_speed_rad_s": float(np.max(angular_speed)),
            "maximum_slack": max(float(row["maximum_normalized_slack"]) for row in rows),
            "minimum_torque_margin_nm": min(float(row[f"tau_margin{i}"]) for row in rows for i in range(6)),
            "max_wrench_error": wrench_error,
            "initial_case": first["case_id"]}


def post_reaudit(config: dict[str, Any], base: dict[str, Any], output: Path,
                 authority: Path, trim: np.ndarray, model: mujoco.MjModel,
                 oracle: Any) -> tuple[list[dict[str, Any]], bool]:
    native_rows = P44.read_csv(authority)
    native = {int(row["control_tick"]): row for row in native_rows if row["record_kind"] == "pre_command"}
    cfg, regime = base["authority"], config["post_reaudit"]["regime_signature"]
    xi_delta, slip_delta = float(cfg["xi_delta_m_s2"]), float(cfg["slip_delta_m_s2"])
    modes = {"common": np.array([1., 1.]), "differential": np.array([-1., 1.])}
    rows: list[dict[str, Any]] = []
    for tick in config["post_reaudit"]["snapshot_ticks"]:
        baseline_control = P45.run(base, output / f"reaudit-t{tick}-base.csv", "R45-H0",
                                   authority=authority, tick=tick, wrench_trim=trim)[0]
        baseline_actual = P45.actual(base, model, oracle, native[tick], baseline_control)
        baseline_qp, baseline_mj = task_output(baseline_control, baseline_actual)
        baseline_signature = signature(baseline_control, baseline_actual, regime)
        for mode, signs in modes.items():
            probes: dict[tuple[int, float], tuple[np.ndarray, np.ndarray, str]] = {}
            for scale in map(float, cfg["delta_scales"]):
                for sign in (-1, 1):
                    delta = np.r_[sign * scale * xi_delta * signs,
                                  sign * scale * slip_delta * signs]
                    control = P45.run(base, output / f"reaudit-t{tick}-{mode}-{scale:g}-{sign:+d}.csv",
                                      "R45-H0", authority=authority, tick=tick, delta=delta,
                                      wrench_trim=trim)[0]
                    actual = P45.actual(base, model, oracle, native[tick], control)
                    qp, mj = task_output(control, actual)
                    probes[(sign, scale)] = qp, mj, signature(control, actual, regime)
            scales = list(map(float, cfg["delta_scales"]))
            for sign in (-1, 1):
                valid = [scale for scale in scales if probes[(sign, scale)][2] == baseline_signature]
                selected = next((scale for scale in scales
                                 if scale in valid and any(smaller < scale for smaller in valid)), None)
                check = next((scale for scale in scales if selected is not None and scale < selected and scale in valid), None)
                if selected is None or check is None:
                    rows.append({"tick": tick, "mode": mode, "sign": sign,
                                 "trusted": False, "reason": "no two same-regime scales"})
                    continue
                derivatives = []
                for scale in (selected, check):
                    qp, mj, _ = probes[(sign, scale)]
                    denominator = sign * scale
                    derivatives.append(((qp - baseline_qp) / denominator,
                                        (mj - baseline_mj) / denominator))
                qp_main = projected(derivatives[0][0], mode, xi_delta, slip_delta)
                mj_main = projected(derivatives[0][1], mode, xi_delta, slip_delta)
                qp_check = projected(derivatives[1][0], mode, xi_delta, slip_delta)
                mj_check = projected(derivatives[1][1], mode, xi_delta, slip_delta)
                qp_conv = abs(qp_main - qp_check) / max(abs(qp_main), 1e-12)
                mj_conv = abs(mj_main - mj_check) / max(abs(mj_main), 1e-12)
                trusted = (qp_conv <= cfg["maximum_directional_convergence_relative"] and
                           mj_conv <= cfg["maximum_directional_convergence_relative"])
                rows.append({"tick": tick, "mode": mode, "sign": sign,
                             "selected_scale": selected, "check_scale": check,
                             "g_qp_projected": qp_main, "g_mj_projected": mj_main,
                             "qp_convergence_relative": qp_conv,
                             "mj_convergence_relative": mj_conv, "trusted": trusted,
                             "sign_match": qp_main * mj_main > 0.0})
    passed = bool(rows) and all(row.get("trusted", False) and row.get("sign_match", False) and
                                abs(row["g_mj_projected"]) >= cfg["minimum_abs_actual_projected_gain"]
                                for row in rows)
    return rows, passed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True)
    probes = output / "probes"; probes.mkdir()
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = frozen_inputs(config)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle_config = json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8"))
    oracle = P42.Oracle(oracle_config)
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    entered = {name: False for name in ("EQ", "AUTH", "REAL", "SHORT", "ROLL", "REAUDIT")}
    gates = {name: False for name in ("DG45-EQ", "DG45-AUTH", "DG45-REAL", "DG45-SHORT",
                                      "DG45-ROLL", "DG45-CONTACT", "DG45-FULLBODY",
                                      "DG45-WR", "DG45-WBC", "DG45-REAUDIT")}

    entered["EQ"] = True
    base_control = P45.run(base, probes / "tick0-compatible.csv", "R45-H0",
                           authority=authority, tick=0, wrench_trim=trim)[0]
    base_actual = P45.actual(base, model, oracle, native, base_control)
    equilibrium = [{"side": side,
        "ddxi_m_s2": base_actual["dynamics"][("ddxi_left_m_s2", "ddxi_right_m_s2")[side]],
        "material_tangent_acceleration_m_s2": base_actual["material"]["tangential_acceleration"][side],
        "native_wheel_qdd_rad_s2": base_actual["dynamics"][("wheel_ddq_left_rad_s2", "wheel_ddq_right_rad_s2")[side]],
        "normal_load_n": base_actual["dynamics"][("normal_load_left_n", "normal_load_right_n")[side]],
        "rolling_active": int(base_control[("rolling_active_left", "rolling_active_right")[side]])}
        for side in range(2)]
    P45.write_csv(output / "tick0-equilibrium.csv", equilibrium)
    gates["DG45-EQ"] = all(abs(row["ddxi_m_s2"]) <= base["gates"]["maximum_equilibrium_ddxi_abs_m_s2"] and
                             abs(row["material_tangent_acceleration_m_s2"]) <= base["gates"]["maximum_equilibrium_slip_acceleration_abs_m_s2"] and
                             row["rolling_active"] == 1 for row in equilibrium)

    authority_rows: list[dict[str, Any]] = []
    if gates["DG45-EQ"]:
        entered["AUTH"] = True
        authority_rows, gates["DG45-AUTH"] = initial_authority(
            base, probes, authority, trim, native, model, oracle)
    P45.write_csv(output / "directional-authority.csv", authority_rows or
                  [{"entered": False, "reason": "DG45-EQ must PASS first"}])

    decomposition: list[dict[str, Any]] = []
    real_metrics: dict[str, Any] = {"entered": False}
    if gates["DG45-AUTH"]:
        entered["REAL"] = True
        decomposition, real_metrics, gates["DG45-REAL"] = real_audit(
            base, base_control, base_actual, oracle)
    P45.write_csv(output / "decomposition.csv", decomposition or
                  [{"entered": False, "reason": "DG45-AUTH must PASS first"}])
    P45.write_json(output / "decomposition.json", real_metrics)

    tick0_wrench_error = max(abs(float(base_control[f"realized_wrench{i}"]) -
                                  float(base_control[f"requested_wrench{i}"])) for i in range(12))
    short_metrics: dict[str, Any] = {"entered": False}
    if gates["DG45-REAL"]:
        entered["SHORT"] = True
        short_rows = P45.run(base, output / "short-rollout.csv", "R45-H0",
                             ticks=int(base["short_horizon_ticks"]), wrench_trim=trim)
        short_metrics = rollout_metrics(short_rows, base, int(base["short_horizon_ticks"]),
                                        tick0_wrench_error)
        gates["DG45-SHORT"] = short_metrics["pass"]
    else:
        P45.write_csv(output / "short-rollout.csv", [{"entered": False,
                      "reason": "DG45-REAL must PASS first"}])

    roll_metrics: dict[str, Any] = {"entered": False}
    nominal_native: Path | None = None
    if gates["DG45-SHORT"]:
        entered["ROLL"] = True
        nominal_path = output / "nominal-rollout.csv"
        nominal_rows = P45.run(base, nominal_path, "R45-H0",
                               ticks=int(base["nominal_horizon_ticks"]), wrench_trim=trim)
        nominal_native = nominal_path.with_name("nominal-rollout_native.csv")
        roll_metrics = rollout_metrics(nominal_rows, base, int(base["nominal_horizon_ticks"]),
                                       tick0_wrench_error)
        gates["DG45-ROLL"] = roll_metrics["pass"]
        gates["DG45-CONTACT"] = (roll_metrics["checks"]["bilateral_contact"] and
                                  roll_metrics["checks"]["rolling_active"])
        gates["DG45-FULLBODY"] = roll_metrics["checks"]["full_body"]
        gates["DG45-WR"] = roll_metrics["checks"]["wrench"]
        gates["DG45-WBC"] = roll_metrics["checks"]["wbc"]
    else:
        P45.write_csv(output / "nominal-rollout.csv", [{"entered": False,
                      "reason": "DG45-SHORT must PASS first"}])
    P45.write_json(output / "rollout-metrics.json", {"short": short_metrics,
                                                      "nominal": roll_metrics})

    reaudit_rows: list[dict[str, Any]] = []
    if gates["DG45-ROLL"] and all(gates[name] for name in
            ("DG45-CONTACT", "DG45-FULLBODY", "DG45-WR", "DG45-WBC")):
        entered["REAUDIT"] = True
        assert nominal_native is not None
        reaudit_rows, gates["DG45-REAUDIT"] = post_reaudit(
            config, base, probes, nominal_native, trim, model, oracle)
    P45.write_csv(output / "post-repair-authority.csv", reaudit_rows or
                  [{"entered": False, "reason": "10 s nominal gates must PASS first"}])

    ordered = ["DG45-EQ", "DG45-AUTH", "DG45-REAL", "DG45-SHORT", "DG45-ROLL",
               "DG45-CONTACT", "DG45-FULLBODY", "DG45-WR", "DG45-WBC", "DG45-REAUDIT"]
    first_failure = next((name for name in ordered if not gates[name]), None)
    compared = ["tick0-equilibrium.csv", "directional-authority.csv", "decomposition.csv",
                "decomposition.json", "short-rollout.csv", "nominal-rollout.csv",
                "rollout-metrics.json", "post-repair-authority.csv"]
    replay_error = None
    if args.replay_of:
        replay_error = max(P45.semantic_error(args.replay_of / name, output / name) for name in compared)
    replay_pass = replay_error is None or replay_error <= base["gates"]["semantic_replay_max_abs"]
    passed = all(gates.values()) and replay_pass
    summary = {"pass": passed, "classification": "P45-H0-PASS" if passed else "P45-REWORK",
               "first_failure": first_failure, "gates": gates, "entered": entered,
               "frozen_equilibrium_wrench_delta": trim, "equilibrium": equilibrium,
               "replay_max_abs_error": replay_error, "scope_contract": config["scope_contract"]}
    P45.write_json(output / "gate-results.json", gates)
    P45.write_json(output / "summary.json", summary)
    sources = [config_path, ROOT / config["base_config"], wrench_source, Path(__file__).resolve(),
               ROOT / base["scene"], ROOT / base["executable"], authority]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in sources}, **config["scope_contract"]})
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
