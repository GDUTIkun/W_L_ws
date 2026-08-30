#!/usr/bin/env python3
"""Phase45 tick0 fixed-wrench equilibrium compatibility audit."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import platform
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase45_equilibrium_compatibility_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P45 = load(ROOT / "tools/experiments/run_phase45_contact_consistent_rolling.py", "p45_eq_p45")
P44 = P45.P44
P42 = P45.P42


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def residual_vector(observed: dict[str, Any]) -> np.ndarray:
    dynamics = observed["dynamics"]
    return np.r_[[dynamics["ddxi_left_m_s2"], dynamics["ddxi_right_m_s2"]],
                 observed["material"]["tangential_acceleration"]]


def decompose_xi(map_: np.ndarray, bias: np.ndarray, qacc: np.ndarray,
                 wheel_dofs: list[int]) -> list[dict[str, float]]:
    base = np.arange(6)
    wheels = np.asarray(wheel_dofs)
    legs = np.asarray([i for i in range(qacc.size) if i not in {*base, *wheels}])
    parts = {
        "base": map_[:, base] @ qacc[base],
        "leg_nonwheel": map_[:, legs] @ qacc[legs],
        "wheel": map_[:, wheels] @ qacc[wheels],
        "jdot_v": bias,
    }
    total = sum(parts.values())
    return [{"side": side, **{name: float(value[side]) for name, value in parts.items()},
             "sum": float(total[side])} for side in range(2)]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")

    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    base_path = ROOT / config["base_config"]
    base = json.loads(base_path.read_text(encoding="utf-8"))
    authority = ROOT / base["phase42_native_authority"]
    tick = int(config["snapshot_tick"])
    native = P45.native_state(P44.read_csv(authority), tick)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle_config = json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8"))
    oracle = P42.Oracle(oracle_config)
    trim_config = config["wrench_counterfactual"]
    evaluations = 0

    with tempfile.TemporaryDirectory(prefix="phase45-equilibrium-") as temporary:
        temporary_path = Path(temporary)

        def evaluate(trim: np.ndarray, name: str) -> tuple[dict[str, str], dict[str, Any]]:
            nonlocal evaluations
            path = temporary_path / f"{evaluations:03d}-{name}.csv"
            control = P45.run(base, path, "R45-EQCF", authority=authority, tick=tick,
                              wrench_trim=trim)[0]
            evaluations += 1
            return control, P45.actual(base, model, oracle, native, control)

        base_control, base_actual = evaluate(np.zeros(4), "fixed")

        def residual(trim: np.ndarray) -> np.ndarray:
            return residual_vector(evaluate(trim, "solve")[1])

        solved = least_squares(
            residual, np.zeros(4),
            bounds=(trim_config["lower_delta"], trim_config["upper_delta"]),
            max_nfev=int(trim_config["maximum_evaluations"]),
            xtol=1e-12, ftol=1e-12, gtol=1e-12,
        )
        compatible_control, compatible_actual = evaluate(solved.x, "compatible")

        output.mkdir(parents=True)
        probes = output / "probes"
        probes.mkdir()
        base_control = P45.run(base, probes / "fixed.csv", "R45-EQCF",
                               authority=authority, tick=tick,
                               wrench_trim=np.zeros(4))[0]
        base_actual = P45.actual(base, model, oracle, native, base_control)
        compatible_control = P45.run(base, probes / "compatible.csv", "R45-EQCF",
                                     authority=authority, tick=tick,
                                     wrench_trim=solved.x)[0]
        compatible_actual = P45.actual(
            base, model, oracle, native, compatible_control)
        evaluations += 2

    fixed_residual = residual_vector(base_actual)
    compatible_residual = residual_vector(compatible_actual)
    desired = np.r_[[float(base_control["desired_ddxi_left"]),
                     float(base_control["desired_ddxi_right"])],
                    [float(base_control["desired_rolling_acceleration_left"]),
                     float(base_control["desired_rolling_acceleration_right"])]]
    qp = np.r_[[float(base_control["physical_ddxi_left"]),
                float(base_control["physical_ddxi_right"])],
               [float(base_control["qp_rolling_acceleration_left"]),
                float(base_control["qp_rolling_acceleration_right"])]]
    desired_qp_error = qp - desired

    actual_qacc = P44.vec(base_actual["dynamics"], "qacc", model.nv)
    reduction = P44.matrix(base_control, "reduction_", model.nv, 12)
    reduction_bias = P44.vec(base_control, "reduction_bias", model.nv)
    qp_qacc = reduction @ P44.vec(base_control, "physical_solution", 12) + reduction_bias
    actual_ddxi = fixed_residual[:2]
    xi_map, xi_bias = P44.native_xi_acceleration_map(
        oracle, base_actual["qpos"], base_actual["qvel"], float(native["time_s"]),
        actual_qacc, actual_ddxi)
    actual_decomposition = decompose_xi(xi_map, xi_bias, actual_qacc, oracle.wheel_dadr)
    qp_decomposition = decompose_xi(xi_map, xi_bias, qp_qacc, oracle.wheel_dadr)
    for side in range(2):
        actual_decomposition[side]["target"] = float(actual_ddxi[side])
        actual_decomposition[side]["closure_error"] = (
            actual_decomposition[side]["sum"] - float(actual_ddxi[side]))
        qp_decomposition[side]["target"] = float(qp[side])
        qp_decomposition[side]["closure_error"] = (
            qp_decomposition[side]["sum"] - float(qp[side]))

    indices = trim_config["requested_wrench_indices"]
    fixed_wrench = P44.vec(base_control, "requested_wrench", 12)
    compatible_wrench = P44.vec(compatible_control, "requested_wrench", 12)
    wrench_rows = [{
        "component": component, "index": index,
        "fixed": float(fixed_wrench[index]),
        "compatible": float(compatible_wrench[index]),
        "delta": float(compatible_wrench[index] - fixed_wrench[index]),
    } for component, index in zip(trim_config["components"], indices)]

    task_rows = []
    labels = ("ddxi_left", "ddxi_right", "a_t_left", "a_t_right")
    compatible_desired = np.r_[[float(compatible_control["desired_ddxi_left"]),
                                float(compatible_control["desired_ddxi_right"])],
                               [float(compatible_control["desired_rolling_acceleration_left"]),
                                float(compatible_control["desired_rolling_acceleration_right"])]]
    compatible_qp = np.r_[[float(compatible_control["physical_ddxi_left"]),
                           float(compatible_control["physical_ddxi_right"])],
                          [float(compatible_control["qp_rolling_acceleration_left"]),
                           float(compatible_control["qp_rolling_acceleration_right"])]]
    for index, label in enumerate(labels):
        task_rows.append({
            "quantity": label,
            "fixed_desired": float(desired[index]), "fixed_qp_realized": float(qp[index]),
            "fixed_mujoco_actual": float(fixed_residual[index]),
            "compatible_desired": float(compatible_desired[index]),
            "compatible_qp_realized": float(compatible_qp[index]),
            "compatible_mujoco_actual": float(compatible_residual[index]),
        })

    gates = config["gates"]
    qp_realization_pass = float(np.max(np.abs(desired_qp_error))) <= gates["maximum_qp_task_realization_error_m_s2"]
    equality_pass = float(np.max(np.abs(compatible_residual))) <= gates["maximum_counterfactual_equality_error_m_s2"]
    closure = max(abs(row["closure_error"]) for row in actual_decomposition + qp_decomposition)
    feasibility = {
        "solver_success": bool(solved.success),
        "hard_violation": float(compatible_control["hard"]),
        "maximum_normalized_slack": float(compatible_control["maximum_normalized_slack"]),
        "minimum_torque_margin_nm": min(float(compatible_control[f"tau_margin{i}"]) for i in range(6)),
        "full_dynamics_residual": compatible_actual["dynamics"]["full_dynamics_residual_max_abs"],
        "contact_reconstruction_residual": compatible_actual["dynamics"]["contact_applyft_jacobian_max_abs"],
        "normal_load_left_n": compatible_actual["dynamics"]["normal_load_left_n"],
        "normal_load_right_n": compatible_actual["dynamics"]["normal_load_right_n"],
    }
    feasibility_pass = (feasibility["solver_success"] and
        feasibility["hard_violation"] <= gates["maximum_hard_violation"] and
        feasibility["maximum_normalized_slack"] <= gates["maximum_normalized_slack"] and
        feasibility["minimum_torque_margin_nm"] >= gates["minimum_torque_margin_nm"] and
        feasibility["full_dynamics_residual"] <= gates["full_dynamics_max_abs"] and
        feasibility["contact_reconstruction_residual"] <= gates["contact_reconstruction_max_abs"])
    decomposition_pass = closure <= gates["maximum_decomposition_closure_error_m_s2"]
    classification = ("FIXED_WRENCH_EQUILIBRIUM_MISMATCH" if
                      qp_realization_pass and equality_pass and feasibility_pass and decomposition_pass
                      else "INCONCLUSIVE")
    removal = 1.0 - abs(compatible_residual[1]) / max(abs(fixed_residual[1]), 1e-15)

    P45.write_csv(output / "desired-qp-actual.csv", task_rows)
    P45.write_csv(output / "xi-decomposition.csv", [
        {"model": model_name, **row}
        for model_name, rows in (("qp", qp_decomposition), ("mujoco", actual_decomposition))
        for row in rows])
    P45.write_csv(output / "wrench-counterfactual.csv", wrench_rows)
    counterfactual = {
        "trim": solved.x, "fixed_residual": fixed_residual,
        "compatible_residual": compatible_residual,
        "right_ddxi_fraction_removed": removal,
        "solver": {"success": solved.success, "status": solved.status,
                   "message": solved.message, "nfev": solved.nfev,
                   "controller_evaluations_including_archived_probes": evaluations,
                   "cost": solved.cost, "optimality": solved.optimality},
        "feasibility": feasibility,
    }
    P45.write_json(output / "wrench-counterfactual.json", counterfactual)
    P45.write_json(output / "classification.json", {
        "classification": classification,
        "fixed_wrench_equilibrium_mismatch": classification == "FIXED_WRENCH_EQUILIBRIUM_MISMATCH",
        "qp_xi_realization_mismatch": not qp_realization_pass,
        "qp_to_mujoco_realization_gap_at_fixed_wrench": float(np.max(np.abs(fixed_residual - qp))),
        "counterfactual_closes_qp_to_mujoco_gap": equality_pass and feasibility_pass,
        "gates": {"qp_realization": qp_realization_pass, "counterfactual_equality": equality_pass,
                  "feasibility": feasibility_pass, "decomposition": decomposition_pass},
    })

    compared = ["desired-qp-actual.csv", "xi-decomposition.csv",
                "wrench-counterfactual.csv", "wrench-counterfactual.json", "classification.json"]
    replay_error = None
    if args.replay_of:
        replay_error = max(P45.semantic_error(args.replay_of / name, output / name) for name in compared)
    replay_pass = replay_error is None or replay_error <= gates["semantic_replay_max_abs"]
    summary = {"pass": classification != "INCONCLUSIVE" and replay_pass,
               "classification": classification, "replay_max_abs_error": replay_error,
               "right_fixed_ddxi_m_s2": fixed_residual[1],
               "right_compatible_ddxi_m_s2": compatible_residual[1],
               "right_ddxi_fraction_removed": removal, "scope_contract": config["scope_contract"]}
    P45.write_json(output / "summary.json", summary)
    sources = [config_path, base_path, Path(__file__).resolve(),
               ROOT / base["scene"], ROOT / base["executable"], authority]
    P45.write_json(output / "manifest.json", {
        "created_utc": datetime.now(timezone.utc).isoformat(), "command": " ".join(sys.argv),
        "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                         "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): sha256(path) for path in sources},
        **config["scope_contract"],
    })
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
