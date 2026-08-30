#!/usr/bin/env python3
"""Phase46 static hip-common projection with strict ordered gates."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase46_hip_common_safe_rolling_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P45C = load(ROOT / "tools/experiments/run_phase45_h0_continuation.py", "p46_cont")
P45, P44, P42 = P45C.P45, P45C.P44, P45C.P42


def project(values: np.ndarray, mode: str) -> np.ndarray:
    if mode == "common":
        return np.asarray([0.5 * (values[0] + values[1]),
                           0.5 * (values[2] + values[3])])
    return np.asarray([0.5 * (values[1] - values[0]),
                       0.5 * (values[3] - values[2])])


def relative(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(a - b) / max(np.linalg.norm(a), 1e-12))


def runtime(config: dict[str, Any]) -> tuple[dict[str, Any], np.ndarray, Path]:
    continuation = json.loads((ROOT / config["continuation_config"]).read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    return base, trim, wrench_source


def observe(base: dict[str, Any], path: Path, case_id: str, authority: Path,
            trim: np.ndarray, native: dict[str, str], model: mujoco.MjModel,
            oracle: Any, delta: np.ndarray | None = None) -> tuple[dict[str, str], dict[str, Any], np.ndarray, np.ndarray]:
    control = P45.run(base, path, case_id, authority=authority, tick=0,
                      wrench_trim=trim, delta=delta)[0]
    actual = P45.actual(base, model, oracle, native, control)
    qp, mj = P45C.task_output(control, actual)
    return control, actual, qp, mj


def authority_audit(base: dict[str, Any], config: dict[str, Any], probes: Path,
                    authority: Path, trim: np.ndarray, native: dict[str, str],
                    model: mujoco.MjModel, oracle: Any,
                    baseline: tuple[np.ndarray, np.ndarray]) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    delta = float(config["delta_m_s2"])
    scales = list(map(float, config["delta_scales"]))
    directions = {"common": np.asarray([1.0, 1.0]),
                  "differential": np.asarray([-1.0, 1.0])}
    rows: list[dict[str, Any]] = []
    matrices: dict[str, Any] = {}
    trusted = True
    for mode, side_direction in directions.items():
        branch: dict[tuple[str, int, float], tuple[np.ndarray, np.ndarray]] = {}
        for channel in ("xi", "slip"):
            for sign in (-1, 1):
                reference: tuple[np.ndarray, np.ndarray] | None = None
                for scale in scales:
                    task_delta = np.zeros(4)
                    start = 0 if channel == "xi" else 2
                    task_delta[start:start + 2] = sign * scale * delta * side_direction
                    _, _, qp, mj = observe(
                        base, probes / f"auth-{mode}-{channel}-{scale:g}-{sign:+d}.csv",
                        config["case_id"], authority, trim, native, model, oracle, task_delta)
                    gains = ((project(qp, mode) - project(baseline[0], mode)) /
                             (sign * scale * delta),
                             (project(mj, mode) - project(baseline[1], mode)) /
                             (sign * scale * delta))
                    branch[(channel, sign, scale)] = gains
                    if scale == 1.0:
                        reference = gains
                    assert reference is not None
                    convergence = max(relative(reference[0], gains[0]),
                                      relative(reference[1], gains[1]))
                    scale_trusted = convergence <= config["maximum_directional_convergence_relative"]
                    trusted &= scale_trusted
                    rows.append({"mode": mode, "input": channel, "branch": sign,
                                 "scale": scale, "trusted": scale_trusted,
                                 "convergence_relative": convergence,
                                 "g_qp_ddxi": gains[0][0], "g_qp_a_t": gains[0][1],
                                 "g_mj_ddxi": gains[1][0], "g_mj_a_t": gains[1][1]})
        qp_matrix = np.zeros((2, 2)); mj_matrix = np.zeros((2, 2))
        split: dict[str, float] = {}
        for column, channel in enumerate(("xi", "slip")):
            minus = branch[(channel, -1, 1.0)]
            plus = branch[(channel, 1, 1.0)]
            split[channel] = max(relative(plus[0], minus[0]), relative(plus[1], minus[1]))
            trusted &= split[channel] <= config["maximum_directional_split_relative"]
            qp_matrix[:, column] = 0.5 * (minus[0] + plus[0])
            mj_matrix[:, column] = 0.5 * (minus[1] + plus[1])
        matrices[mode] = {"rows": ["ddxi", "a_t"], "columns": ["xi", "slip"],
                          "g_qp": qp_matrix, "g_mj": mj_matrix,
                          "branch_split_relative": split}

    common = matrices["common"]
    qp = np.asarray(common["g_qp"]); mj = np.asarray(common["g_mj"])
    limits = config["authority_gates"]
    harmful = abs(float(limits["phase45_harmful_cross_gain"]))
    cross = float(mj[0, 1]); slip_self = float(mj[1, 1]); xi_self = float(mj[0, 0])
    unified_qp = float(0.5 * np.sum(qp)); unified_mj = float(0.5 * np.sum(mj))
    checks = {
        "branches_and_scales_trusted": trusted,
        "cross_near_zero": abs(cross) <= limits["maximum_abs_actual_cross_gain"],
        "cross_reduction": 1.0 - abs(cross) / harmful >= limits["minimum_cross_reduction_fraction"],
        "slip_self_positive": slip_self > 0.0,
        "slip_self_retained": slip_self >= (limits["minimum_slip_self_retention_fraction"] *
                                              limits["phase45_slip_self_gain"]),
        "xi_self_positive_and_material": xi_self >= limits["minimum_abs_actual_xi_self_gain"],
        "common_unified_sign_match": unified_qp * unified_mj > 0.0,
        "common_unified_actual_material": abs(unified_mj) >=
            limits["minimum_abs_actual_unified_projected_gain"],
    }
    metrics = {"pass": all(checks.values()), "checks": checks,
               "actual_cross_gain": cross,
               "cross_reduction_fraction": 1.0 - abs(cross) / harmful,
               "actual_slip_self_gain": slip_self, "actual_xi_self_gain": xi_self,
               "unified_projected_qp": unified_qp, "unified_projected_mj": unified_mj}
    return rows, matrices, metrics


def placeholder(path: Path, reason: str) -> None:
    P45.write_json(path, {"entered": False, "reason": reason})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = runtime(config)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    gate_names = ["DG46-EQ", "DG46-AUTH", "DG46-REAL", "DG46-SHORT", "DG46-ROLL",
                  "DG46-CONTACT", "DG46-FULLBODY", "DG46-WR", "DG46-WBC", "DG46-REAUDIT"]
    gates = {name: False for name in gate_names}
    entered = {name: False for name in ("EQ", "AUTH", "REAL", "SHORT", "ROLL", "REAUDIT")}

    entered["EQ"] = True
    control, actual, baseline_qp, baseline_mj = observe(
        base, probes / "tick0-compatible.csv", config["case_id"], authority,
        trim, native, model, oracle)
    dyn = actual["dynamics"]; eq_limits = base["gates"]
    equilibrium = {
        "ddxi": [dyn["ddxi_left_m_s2"], dyn["ddxi_right_m_s2"]],
        "material_tangent_acceleration": actual["material"]["tangential_acceleration"],
        "normal_load": [dyn["normal_load_left_n"], dyn["normal_load_right_n"]],
        "rolling_active": [int(control["rolling_active_left"]), int(control["rolling_active_right"])],
        "hard": float(control["hard"]), "slack": float(control["maximum_normalized_slack"]),
        "minimum_torque_margin": min(float(control[f"tau_margin{i}"]) for i in range(6)),
        "full_dynamics_residual": dyn["full_dynamics_residual_max_abs"],
        "contact_reconstruction_residual": dyn["contact_applyft_jacobian_max_abs"],
    }
    gates["DG46-EQ"] = (
        max(abs(x) for x in equilibrium["ddxi"]) <= eq_limits["maximum_equilibrium_ddxi_abs_m_s2"] and
        max(abs(x) for x in equilibrium["material_tangent_acceleration"]) <=
            eq_limits["maximum_equilibrium_slip_acceleration_abs_m_s2"] and
        all(x > 0.0 for x in equilibrium["normal_load"]) and
        equilibrium["rolling_active"] == [1, 1] and
        equilibrium["hard"] <= eq_limits["maximum_hard_violation"] and
        equilibrium["slack"] <= eq_limits["maximum_normalized_slack"] and
        equilibrium["minimum_torque_margin"] >= eq_limits["minimum_torque_margin_nm"] and
        equilibrium["full_dynamics_residual"] <= eq_limits["full_dynamics_max_abs"] and
        equilibrium["contact_reconstruction_residual"] <= eq_limits["contact_reconstruction_max_abs"])
    P45.write_json(output / "equilibrium.json", equilibrium)

    authority_rows: list[dict[str, Any]] = []
    matrices: dict[str, Any] = {"entered": False}
    auth_metrics: dict[str, Any] = {"entered": False}
    if gates["DG46-EQ"]:
        entered["AUTH"] = True
        authority_rows, matrices, auth_metrics = authority_audit(
            base, config, probes, authority, trim, native, model, oracle,
            (baseline_qp, baseline_mj))
        gates["DG46-AUTH"] = auth_metrics["pass"]
    P45.write_csv(output / "directional-authority.csv", authority_rows or
                  [{"entered": False, "reason": "DG46-EQ must PASS first"}])
    P45.write_json(output / "transfer-matrices.json", matrices)
    P45.write_json(output / "authority-gate.json", auth_metrics)

    # Later gates remain executable but are never entered after an AUTH failure.
    if gates["DG46-AUTH"]:
        entered["REAL"] = True
        real_dir = output / "real-audit"
        command = [sys.executable, str(ROOT / "tools/experiments/run_phase45_rework_authority_attribution.py"),
                   "--config", str(config_path), "--output", str(real_dir)]
        if args.replay_of:
            command += ["--replay-of", str(args.replay_of / "real-audit")]
        completed = subprocess.run(command, check=False)
        real_summary = json.loads((real_dir / "summary.json").read_text(encoding="utf-8"))
        modes = json.loads((real_dir / "leg-mode-summary.json").read_text(encoding="utf-8"))
        realization = config["realization_gates"]
        closure = max(abs(modes[name][key]) for name in ("qp", "mujoco")
                      for key in ("dof_closure", "mode_closure"))
        alternatives = [abs(float(row[key])) for row in modes["mujoco"]["modes"]
                        for key in ("common_mode_ddxi_contribution", "differential_mode_ddxi_contribution")
                        if not (row["joint_family"] == "hip" and key == "common_mode_ddxi_contribution")]
        real_metrics = {"pass": completed.returncode == 0 and real_summary["pass"] and
                        closure <= realization["maximum_decomposition_closure"] and
                        max(alternatives) <= realization["maximum_abs_migrated_mode_contribution"],
                        "decomposition_closure": closure,
                        "maximum_abs_alternative_mode_contribution": max(alternatives)}
        gates["DG46-REAL"] = real_metrics["pass"]
        P45.write_json(output / "real-gate.json", real_metrics)
    else:
        placeholder(output / "real-gate.json", "DG46-AUTH must PASS first")

    tick0_wrench_error = max(abs(float(control[f"realized_wrench{i}"]) -
                                  float(control[f"requested_wrench{i}"])) for i in range(12))
    rollout = {"short": {"entered": False}, "nominal": {"entered": False}}
    nominal_native: Path | None = None
    if gates["DG46-REAL"]:
        entered["SHORT"] = True
        rows = P45.run(base, output / "short-rollout.csv", config["case_id"],
                       ticks=int(base["short_horizon_ticks"]), wrench_trim=trim)
        rollout["short"] = P45C.rollout_metrics(
            rows, base, int(base["short_horizon_ticks"]), tick0_wrench_error)
        gates["DG46-SHORT"] = rollout["short"]["pass"]
    else:
        P45.write_csv(output / "short-rollout.csv", [{"entered": False,
                      "reason": "DG46-REAL must PASS first"}])
    if gates["DG46-SHORT"]:
        entered["ROLL"] = True
        nominal_path = output / "nominal-rollout.csv"
        rows = P45.run(base, nominal_path, config["case_id"],
                       ticks=int(base["nominal_horizon_ticks"]), wrench_trim=trim)
        nominal_native = nominal_path.with_name("nominal-rollout_native.csv")
        rollout["nominal"] = P45C.rollout_metrics(
            rows, base, int(base["nominal_horizon_ticks"]), tick0_wrench_error)
        checks = rollout["nominal"]["checks"]
        gates["DG46-ROLL"] = rollout["nominal"]["pass"]
        gates["DG46-CONTACT"] = checks["bilateral_contact"] and checks["rolling_active"]
        gates["DG46-FULLBODY"] = checks["full_body"]
        gates["DG46-WR"] = checks["wrench"]
        gates["DG46-WBC"] = checks["wbc"]
    else:
        P45.write_csv(output / "nominal-rollout.csv", [{"entered": False,
                      "reason": "DG46-SHORT must PASS first"}])
    P45.write_json(output / "rollout-metrics.json", rollout)

    reaudit_rows: list[dict[str, Any]] = []
    if gates["DG46-ROLL"] and all(gates[name] for name in
            ("DG46-CONTACT", "DG46-FULLBODY", "DG46-WR", "DG46-WBC")):
        entered["REAUDIT"] = True
        assert nominal_native is not None
        continuation = json.loads((ROOT / config["continuation_config"]).read_text(encoding="utf-8"))
        continuation["post_reaudit"] = config["post_reaudit"]
        reaudit_rows, gates["DG46-REAUDIT"] = P45C.post_reaudit(
            continuation, base, probes, nominal_native, trim, model, oracle, config["case_id"])
    P45.write_csv(output / "post-repair-authority.csv", reaudit_rows or
                  [{"entered": False, "reason": "10 s mandatory gates must PASS first"}])

    first_failure = next((name for name in gate_names if not gates[name]), None)
    checks = auth_metrics.get("checks", {})
    cross_bad = not checks.get("cross_near_zero", True)
    slip_bad = not checks.get("slip_self_retained", True)
    if not gates["DG46-EQ"]:
        classification = "P46-E — multiple remaining mechanisms"
    elif first_failure == "DG46-AUTH":
        classification = ("P46-E — multiple remaining mechanisms" if cross_bad and slip_bad else
                          "P46-B — cross-coupling reduced but insufficient" if cross_bad else
                          "P46-D — slip authority destroyed by projection" if slip_bad else
                          "P46-U — evidence unreliable")
    elif first_failure == "DG46-REAL":
        classification = "P46-C — harmful mode migrated to another DOF/mode"
    elif first_failure is None:
        classification = "P46-A — static hip-common projection sufficient"
    else:
        classification = "P46-U — evidence unreliable"

    compared = ["equilibrium.json", "directional-authority.csv", "transfer-matrices.json",
                "authority-gate.json", "real-gate.json", "short-rollout.csv",
                "nominal-rollout.csv", "rollout-metrics.json", "post-repair-authority.csv"]
    replay_error = (max(P45.semantic_error(args.replay_of / name, output / name)
                        for name in compared) if args.replay_of else None)
    replay_pass = replay_error is None or replay_error <= base["gates"]["semantic_replay_max_abs"]
    passed = all(gates.values()) and replay_pass
    P45.write_json(output / "gate-results.json", gates)
    P45.write_json(output / "summary.json", {"pass": passed, "classification": classification,
        "first_failure": first_failure, "entered": entered, "gates": gates,
        "replay_max_abs_error": replay_error, "scope_contract": config["scope_contract"]})
    sources = [config_path, ROOT / config["continuation_config"], ROOT / base["scene"],
               ROOT / base["executable"], authority, wrench_source, Path(__file__).resolve()]
    P45.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources}, **config["scope_contract"]})
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
