#!/usr/bin/env python3
"""Phase46 REWORK: frozen-H0 nominal, slip-common incremental hip constraint."""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase46_hip_common_increment_limited_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P45C = load(ROOT / "tools/experiments/run_phase45_h0_continuation.py", "p46_increment")
P45, P44, P42 = P45C.P45, P45C.P44, P45C.P42


def runtime(config: dict[str, Any]) -> tuple[dict[str, Any], np.ndarray, Path]:
    continuation = json.loads((ROOT / config["continuation_config"]).read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    return base, trim, wrench_source


def eq_metrics(control: dict[str, str], actual: dict[str, Any], gates: dict[str, float]) -> dict[str, Any]:
    dynamics = actual["dynamics"]
    values = {
        "ddxi": [dynamics["ddxi_left_m_s2"], dynamics["ddxi_right_m_s2"]],
        "material_tangent_acceleration": actual["material"]["tangential_acceleration"],
        "normal_load": [dynamics["normal_load_left_n"], dynamics["normal_load_right_n"]],
        "rolling_active": [int(control["rolling_active_left"]), int(control["rolling_active_right"])],
        "hard": float(control["hard"]),
        "slack": float(control["maximum_normalized_slack"]),
        "minimum_torque_margin": min(float(control[f"tau_margin{i}"]) for i in range(6)),
        "full_dynamics_residual": dynamics["full_dynamics_residual_max_abs"],
        "contact_reconstruction_residual": dynamics["contact_applyft_jacobian_max_abs"],
    }
    values["pass"] = (
        max(abs(x) for x in values["ddxi"]) <= gates["maximum_equilibrium_ddxi_abs_m_s2"] and
        max(abs(x) for x in values["material_tangent_acceleration"]) <=
            gates["maximum_equilibrium_slip_acceleration_abs_m_s2"] and
        all(x > 0.0 for x in values["normal_load"]) and values["rolling_active"] == [1, 1] and
        values["hard"] <= gates["maximum_hard_violation"] and
        values["slack"] <= gates["maximum_normalized_slack"] and
        values["minimum_torque_margin"] >= gates["minimum_torque_margin_nm"] and
        values["full_dynamics_residual"] <= gates["full_dynamics_max_abs"] and
        values["contact_reconstruction_residual"] <= gates["contact_reconstruction_max_abs"])
    return values


def central(rows: list[dict[str, str]], key: str) -> float:
    selected = [float(row[key]) for row in rows if row["channel"] == "slip_common_only" and
                float(row["scale"]) == 1.0]
    if len(selected) != 2:
        raise RuntimeError(f"expected two slip-common branches for {key}")
    return 0.5 * sum(selected)


def authority_metrics(config: dict[str, Any], audit: Path) -> dict[str, Any]:
    matrices = json.loads((audit / "common-transfer-matrices.json").read_text(encoding="utf-8"))
    modes = json.loads((audit / "leg-mode-summary.json").read_text(encoding="utf-8"))
    directional = P44.read_csv(audit / "directional-transfer.csv")
    probes = P44.read_csv(audit / "probe-observables.csv")
    qp = np.asarray(matrices["unified_reconstructed"]["g_qp"], dtype=float)
    mj = np.asarray(matrices["unified_reconstructed"]["g_mj"], dtype=float)
    hip = {model: next(row for row in modes[model]["modes"] if row["joint_family"] == "hip")
           for model in ("qp", "mujoco")}
    knee = {model: next(row for row in modes[model]["modes"] if row["joint_family"] == "knee")
            for model in ("qp", "mujoco")}
    closure = max(abs(float(modes[model][key])) for model in ("qp", "mujoco")
                  for key in ("dof_closure", "mode_closure"))
    limits = config["authority_gates"]
    cross = float(mj[0, 1]); slip_self = float(mj[1, 1])
    alternative = max(
        abs(float(knee["mujoco"]["common_mode_ddxi_contribution"])),
        abs(float(hip["mujoco"]["differential_mode_ddxi_contribution"])),
        abs(central(directional, "g_xi_mj_base")),
        abs(central(directional, "g_xi_mj_wheel")),
        abs(central(directional, "g_xi_mj_jdot_v")))
    checks = {
        "branch_and_scale": all(row["trusted"] == "True" for row in directional),
        "cross_reduced": abs(cross) <= limits["maximum_abs_actual_cross_gain"] and
            1.0 - abs(cross) / abs(limits["phase45_harmful_cross_gain"]) >=
            limits["minimum_cross_reduction_fraction"],
        "slip_self_positive_retained": slip_self > 0.0 and
            slip_self >= limits["minimum_slip_self_retention_fraction"] * limits["phase45_slip_self_gain"],
        "qp_hip_increment_limited": abs(float(hip["qp"]["common_qacc_gain_rad_s2_per_m_s2"])) <=
            limits["maximum_abs_hip_common_increment_gain"],
        "plant_hip_increment_limited": abs(float(hip["mujoco"]["common_qacc_gain_rad_s2_per_m_s2"])) <=
            limits["maximum_abs_hip_common_increment_gain"],
        "no_harmful_migration": alternative <= limits["maximum_abs_migrated_mode_contribution"],
        "decomposition_closure": closure <= 1.0e-10,
        "dynamics_contact_closure": max(max(abs(float(row["dynamics_closure"])),
                                             abs(float(row["contact_closure"]))) for row in probes) <= 1.0e-8,
    }
    return {
        "pass": all(checks.values()), "checks": checks,
        "g_qp": qp, "g_mj": mj, "actual_cross_gain": cross,
        "cross_reduction_fraction": 1.0 - abs(cross) / abs(limits["phase45_harmful_cross_gain"]),
        "actual_slip_self_gain": slip_self, "hip_common": hip, "knee_common": knee,
        "other_actual_contributions": {
            "base": central(directional, "g_xi_mj_base"),
            "native_wheel": central(directional, "g_xi_mj_wheel"),
            "jdot_v": central(directional, "g_xi_mj_jdot_v"),
            "hip_differential": hip["mujoco"]["differential_mode_ddxi_contribution"],
        },
        "maximum_abs_alternative_contribution": alternative,
        "decomposition_closure": closure,
    }


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
    base, trim, wrench_source = runtime(config)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    control = P45.run(base, probes / "tick0-zero-delta.csv", config["case_id"], authority=authority,
                      tick=0, wrench_trim=trim)[0]
    actual = P45.actual(base, model, oracle, native, control)
    equilibrium = eq_metrics(control, actual, base["gates"])
    P45.write_json(output / "equilibrium.json", equilibrium)
    entered = {"EQ": True, "AUTH": False}
    gate_eq, gate_auth = equilibrium["pass"], False
    metrics: dict[str, Any] = {"entered": False}
    if gate_eq:
        entered["AUTH"] = True
        audit = output / "incremental-authority"
        command = [sys.executable, str(ROOT / "tools/experiments/run_phase45_rework_authority_attribution.py"),
                   "--config", str(config_path), "--output", str(audit)]
        if args.replay_of:
            command += ["--replay-of", str(args.replay_of / "incremental-authority")]
        completed = subprocess.run(command, check=False)
        metrics = authority_metrics(config, audit)
        metrics["attribution_exit_code"] = completed.returncode
        gate_auth = completed.returncode == 0 and metrics["pass"]
    else:
        P45.write_json(output / "incremental-authority.json", {"entered": False,
                       "reason": "DG46I-EQ must PASS first"})
    if gate_eq:
        P45.write_json(output / "incremental-authority.json", metrics)
    gates = {"DG46I-EQ": gate_eq, "DG46I-AUTH": gate_auth}
    if not gate_eq:
        classification = "C-EQUILIBRIUM_BROKEN"
    elif gate_auth:
        classification = "A-EQUILIBRIUM_PRESERVED_AND_CROSS_COUPLING_REDUCED"
    elif not metrics["checks"]["slip_self_positive_retained"]:
        classification = "D-SLIP_AUTHORITY_DESTROYED"
    elif not metrics["checks"]["cross_reduced"]:
        classification = "B-EQUILIBRIUM_PRESERVED_BUT_COUPLING_REMAINS"
    elif not metrics["checks"]["no_harmful_migration"]:
        classification = "E-MULTIPLE"
    else:
        classification = "U-UNTRUSTED"
    compared = ["equilibrium.json", "incremental-authority.json"]
    replay_error = max(P45.semantic_error(args.replay_of / name, output / name)
                       for name in compared) if args.replay_of else None
    replay_pass = replay_error is None or replay_error <= base["gates"]["semantic_replay_max_abs"]
    P45.write_json(output / "summary.json", {"pass": gate_eq and gate_auth and replay_pass,
        "classification": classification, "gates": gates, "entered": entered,
        "replay_max_abs_error": replay_error, "scope_contract": config["scope_contract"]})
    sources = [config_path, ROOT / config["continuation_config"], ROOT / base["scene"],
               ROOT / base["executable"], authority, wrench_source, Path(__file__).resolve(),
               ROOT / "tools/experiments/run_phase45_rework_authority_attribution.py"]
    P45.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "sources": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources}, **config["scope_contract"]})
    return 0 if gate_eq and gate_auth and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
