#!/usr/bin/env python3
"""Phase46 corrected production-reference exact-R1 fixed-state AUTH only."""

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
CONFIG = ROOT / "simulation/mujoco/config/phase46_point_realizable_rolling_v1.json"


def load(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


R1 = load(ROOT / "tools/experiments/run_phase46_corrected_exact_r1.py", "p46_auth_r1")
P45C, P45, P44, P42 = R1.P45C, R1.P45, R1.P44, R1.P42
TOL = R1.TOL


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


def projection(values: np.ndarray, mode: str) -> np.ndarray:
    if mode == "common":
        return np.array([0.5 * (values[0] + values[1]),
                         0.5 * (values[2] + values[3])])
    return np.array([0.5 * (values[1] - values[0]),
                     0.5 * (values[3] - values[2])])


def relative(reference: np.ndarray, candidate: np.ndarray) -> float:
    return float(np.linalg.norm(candidate - reference) /
                 max(np.linalg.norm(reference), 1.0e-12))


def topology(actual: dict[str, Any]) -> list[list[list[int]]]:
    return [sorted([[min(int(row["geom1"]), int(row["geom2"])),
                     max(int(row["geom1"]), int(row["geom2"])), int(row["dim"])]
                    for row in actual["details"] if int(row["side"]) == side])
            for side in range(2)]


def frames(actual: dict[str, Any]) -> list[list[list[float]]]:
    ordered = sorted(actual["details"], key=lambda row: (int(row["side"]), int(row["contact_index"])))
    return [[[float(row[f"frame_{i}{j}"]) for j in range(3)] for i in range(3)]
            for row in ordered]


def regime(control: dict[str, str], actual: dict[str, Any], baseline_frames: np.ndarray,
           baseline_topology: list[list[list[int]]]) -> dict[str, Any]:
    current_frames = np.asarray(frames(actual))
    statuses = [control[name] for name in ("model_status", "controller_status", "solver_status")]
    active = {
        "counts": [int(control[f"active_count{i}"]) for i in range(3)],
        "rows": [int(control[f"active_row{i}"]) for i in range(12, 104)],
    }
    details = actual["details"]
    result = {
        "bilateral_contact": all(any(int(row["side"]) == side for row in details)
                                 for side in range(2)),
        "contact_count_per_side": [sum(int(row["side"]) == side for row in details)
                                   for side in range(2)],
        "contact_topology": topology(actual),
        "contact_dimension": sorted({int(row["dim"]) for row in details}),
        "normal_frame_max_abs_delta_from_baseline": float(np.max(np.abs(
            current_frames - baseline_frames))),
        "minimum_friction_margin_diagnostic_n": min(
            float(row["friction_margin_diagnostic_n"]) for row in details),
        "active_constraints": active,
        "minimum_torque_margin_nm": min(float(control[f"tau_margin{i}"]) for i in range(6)),
        "hard_violation": float(control["hard"]),
        "maximum_normalized_slack": float(control["maximum_normalized_slack"]),
        "statuses": statuses,
        "rolling_active": [int(control["rolling_active_left"]), int(control["rolling_active_right"])],
    }
    signature = {
        "topology": result["contact_topology"], "statuses": statuses,
        "active_constraints": active, "rolling_active": result["rolling_active"],
    }
    result["solver_contact_signature"] = json.dumps(signature, sort_keys=True, separators=(",", ":"))
    result["stable"] = (
        result["bilateral_contact"] and result["contact_count_per_side"] == [2, 2] and
        result["contact_topology"] == baseline_topology and
        result["contact_dimension"] == [3] and
        result["normal_frame_max_abs_delta_from_baseline"] <= 1.0e-12 and
        result["minimum_friction_margin_diagnostic_n"] > 0.0 and
        statuses == ["0", "0", "0"] and result["rolling_active"] == [1, 1] and
        result["minimum_torque_margin_nm"] >= -1.0e-10 and
        result["hard_violation"] <= 1.0e-7 and
        result["maximum_normalized_slack"] <= 0.05)
    return result


def r1_check(control: dict[str, str], operators: dict[str, np.ndarray],
             production: dict[str, Any], operator_source: dict[str, Any]) -> dict[str, Any]:
    sides = []
    for side, name in enumerate(("left", "right")):
        expected, source = production["sides"][name], operator_source["sides"][name]
        gp, pg = np.asarray(expected["Gp_production"]), np.asarray(expected["Pg_production"])
        controller = operators[f"point_force_wrench_projector_{side}"]
        wrench = np.array([float(control[f"physical_solution{18 + 6 * side + i}"])
                           for i in range(6)])
        reconstructed = gp @ np.linalg.pinv(gp, rcond=1.0e-12) @ wrench
        full = np.asarray(source["Aw_full"]) @ gp - np.asarray(source["Jp"]).T
        reduced = (np.asarray(source["Aw_reduced"]) @ gp -
                   np.asarray(source["reduction"]).T @ np.asarray(source["Jp"]).T)
        metrics = {
            "side": name,
            "rank": int(np.linalg.matrix_rank(gp, tol=TOL)),
            "projector_parity_max_abs": R1.max_abs(controller - pg),
            "physical_range_residual_max_abs": R1.max_abs((np.eye(6) - pg) @ wrench),
            "point_force_reconstruction_max_abs": R1.max_abs(reconstructed - wrench),
            "full_operator_closure_max_abs": R1.max_abs(full),
            "reduced_operator_closure_max_abs": R1.max_abs(reduced),
            "reference_semantics": "production aggregate-wrench reference with transported actual two-point force image",
        }
        metrics["pass"] = (metrics["rank"] == 5 and
                           max(value for key, value in metrics.items()
                               if key.endswith("max_abs")) <= TOL)
        sides.append(metrics)
    return {"pass": all(side["pass"] for side in sides), "sides": sides}


def observe(base: dict[str, Any], config: dict[str, Any], path: Path, authority: Path,
            trim: np.ndarray, native: dict[str, str], model: mujoco.MjModel, oracle: Any,
            qp_dump: Path, production: dict[str, Any], operator_source: dict[str, Any],
            baseline_frames: np.ndarray | None, baseline_topology: list[list[list[int]]] | None,
            delta: np.ndarray | None = None) -> dict[str, Any]:
    control = P45.run(base, path, config["case_id"], authority=authority, tick=0,
                      wrench_trim=trim, delta=delta)[0]
    actual = P45.actual(base, model, oracle, native, control)
    qp, mj = P45C.task_output(control, actual)
    operators = R1.dump(qp_dump, path)
    if baseline_frames is None:
        baseline_frames = np.asarray(frames(actual))
        baseline_topology = topology(actual)
    assert baseline_topology is not None
    return {"qp": qp, "mj": mj, "control": control, "actual": actual,
            "regime": regime(control, actual, baseline_frames, baseline_topology),
            "r1": r1_check(control, operators, production, operator_source)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--qp-dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output, config_path, qp_dump = args.output.resolve(), args.config.resolve(), args.qp_dump.resolve()
    if output.exists():
        raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    continuation_path = ROOT / config["continuation_config"]
    continuation = json.loads(continuation_path.read_text(encoding="utf-8"))
    base, trim, wrench_source = P45C.frozen_inputs(continuation)
    base["executable"] = config["runtime_executable"]
    authority = ROOT / base["phase42_native_authority"]
    native = P45.native_state(P44.read_csv(authority), 0)
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    oracle = P42.Oracle(json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8")))
    production, operator_source = R1.read(R1.PRODUCTION_AUDIT), R1.read(R1.OPERATOR_AUDIT)

    baseline = observe(base, config, probes / "baseline.csv", authority, trim, native, model,
                       oracle, qp_dump, production, operator_source, None, None, np.zeros(4))
    baseline_frames, baseline_topology = np.asarray(frames(baseline["actual"])), topology(baseline["actual"])
    delta, scales = float(config["delta_m_s2"]), list(map(float, config["delta_scales"]))
    directions = {"common": np.array([1.0, 1.0]), "differential": np.array([-1.0, 1.0])}
    branch: dict[tuple[str, str, int, float], dict[str, Any]] = {}
    rows = []
    all_r1, all_regime = baseline["r1"]["pass"], baseline["regime"]["stable"]
    for mode, direction in directions.items():
        for channel in ("xi", "slip"):
            for sign in (-1, 1):
                reference: dict[str, np.ndarray] | None = None
                for scale in scales:
                    task_delta = np.zeros(4); start = 0 if channel == "xi" else 2
                    task_delta[start:start + 2] = sign * scale * delta * direction
                    item = observe(base, config,
                                   probes / f"auth-{mode}-{channel}-{scale:g}-{sign:+d}.csv",
                                   authority, trim, native, model, oracle, qp_dump, production,
                                   operator_source, baseline_frames, baseline_topology, task_delta)
                    denominator = sign * scale * delta
                    gains = {name: (item[name] - baseline[name]) / denominator
                             for name in ("qp", "mj")}
                    if scale == 1.0:
                        reference = gains
                    assert reference is not None
                    convergence = max(relative(reference[name], gains[name]) for name in ("qp", "mj"))
                    row = {"mode": mode, "input": channel, "branch": sign, "scale": scale,
                           "signed_delta_m_s2": denominator,
                           "baseline_subtracted": True,
                           "baseline_qp": baseline["qp"], "probe_qp": item["qp"],
                           "baseline_mj": baseline["mj"], "probe_mj": item["mj"],
                           "gain_qp_per_side": gains["qp"], "gain_mj_per_side": gains["mj"],
                           "gain_qp_common": projection(gains["qp"], "common"),
                           "gain_mj_common": projection(gains["mj"], "common"),
                           "gain_qp_differential": projection(gains["qp"], "differential"),
                           "gain_mj_differential": projection(gains["mj"], "differential"),
                           "scale_convergence_relative": convergence,
                           "scale_convergence_pass": convergence <= config["maximum_directional_convergence_relative"],
                           "r1": item["r1"], "regime": item["regime"]}
                    rows.append(row); branch[(mode, channel, sign, scale)] = row
                    all_r1 &= item["r1"]["pass"]; all_regime &= item["regime"]["stable"]

    matrices, branch_pass, scale_pass = {}, True, all(row["scale_convergence_pass"] for row in rows)
    contamination = []
    for mode in directions:
        qp_matrix, mj_matrix = np.zeros((2, 2)), np.zeros((2, 2))
        splits = {}
        for column, channel in enumerate(("xi", "slip")):
            minus, plus = branch[(mode, channel, -1, 1.0)], branch[(mode, channel, 1, 1.0)]
            output_key = mode
            qp_minus, qp_plus = minus[f"gain_qp_{output_key}"], plus[f"gain_qp_{output_key}"]
            mj_minus, mj_plus = minus[f"gain_mj_{output_key}"], plus[f"gain_mj_{output_key}"]
            split = max(relative(qp_plus, qp_minus), relative(mj_plus, mj_minus))
            splits[channel] = split; branch_pass &= split <= config["maximum_directional_split_relative"]
            qp_matrix[:, column] = 0.5 * (qp_minus + qp_plus)
            mj_matrix[:, column] = 0.5 * (mj_minus + mj_plus)
            other = "differential" if mode == "common" else "common"
            contamination.append({"input_mode": mode, "input_channel": channel,
                                  "output_mode": other,
                                  "g_qp": 0.5 * (minus[f"gain_qp_{other}"] + plus[f"gain_qp_{other}"]),
                                  "g_mj": 0.5 * (minus[f"gain_mj_{other}"] + plus[f"gain_mj_{other}"])})
        matrices[mode] = {"rows": ["ddxi", "slip_acceleration"],
                          "columns": ["xi", "slip"], "G_QP": qp_matrix,
                          "G_MJ": mj_matrix, "branch_split_relative": splits}

    gates = config["repair_gates"]; common = matrices["common"]
    qp, mj = np.asarray(common["G_QP"]), np.asarray(common["G_MJ"])
    cross, xi_self, slip_self = float(mj[0, 1]), float(mj[0, 0]), float(mj[1, 1])
    harmful, slip_old = abs(float(gates["phase45_harmful_cross_gain"])), float(gates["phase45_slip_self_gain"])
    reduction = 1.0 - abs(cross) / harmful
    checks = {
        "baseline_subtracted_all_probes": all(row["baseline_subtracted"] for row in rows),
        "r1_exact_all_probes": all_r1, "regime_stable_all_probes": all_regime,
        "branch_convergence": branch_pass, "scale_convergence": scale_pass,
        "harmful_cross_abs": abs(cross) <= gates["maximum_abs_actual_cross_gain"],
        "harmful_cross_reduction": reduction >= gates["minimum_cross_reduction_fraction"],
        "xi_self_positive_material": xi_self >= gates["minimum_abs_xi_self_gain"],
        "slip_self_positive_retained": slip_self > 0.0 and
            slip_self >= gates["minimum_slip_self_retention_fraction"] * slip_old,
        "qp_mj_xi_self_sign_consistent": qp[0, 0] * xi_self > 0.0,
        "qp_mj_slip_self_sign_consistent": qp[1, 1] * slip_self > 0.0,
    }
    auth_pass = all(checks.values())
    migrated = max((float(np.max(np.abs(item["g_mj"]))) for item in contamination), default=0.0) > 0.1
    if not all_r1:
        classification, next_action = "U-UNTRUSTED", "implementation fix only"
    elif not all_regime:
        classification, next_action = "E-REGIME-DEPENDENT", "implementation fix only"
    elif not checks["harmful_cross_abs"] or not checks["harmful_cross_reduction"]:
        classification, next_action = "B-HARMFUL-CROSS-REMAINS", "post-corrected-R1 authority attribution"
    elif not (checks["xi_self_positive_material"] and checks["slip_self_positive_retained"] and
              checks["qp_mj_xi_self_sign_consistent"] and checks["qp_mj_slip_self_sign_consistent"]):
        classification, next_action = "C-SELF-AUTHORITY-LOST", "post-corrected-R1 authority attribution"
    elif migrated:
        classification, next_action = "D-MODE-MIGRATION", "post-corrected-R1 authority attribution"
    elif not branch_pass or not scale_pass:
        classification, next_action = "U-UNTRUSTED", "implementation fix only"
    else:
        classification, next_action = "A-AUTH-PASS", "REAL fixed-state audit"

    result = {"schema_version": 1, "phase": 46,
              "scope": "corrected production-reference exact-R1 compatible-H0 tick0 AUTH only",
              "baseline_subtraction_definition": "(probe_output - baseline_output) / signed_input_delta",
              "baseline": {"qp": baseline["qp"], "mj": baseline["mj"],
                           "r1": baseline["r1"], "regime": baseline["regime"]},
              "probes": rows, "transfer": matrices, "common_differential_contamination": contamination,
              "metrics": {"actual_slip_to_ddxi_cross": cross,
                          "reduction_vs_phase45": reduction, "actual_xi_self": xi_self,
                          "actual_slip_self": slip_self,
                          "maximum_common_differential_contamination_mj":
                              max((float(np.max(np.abs(item["g_mj"]))) for item in contamination), default=0.0)},
              "checks": checks, "R1_still_exactly_closed": all_r1,
              "state_contact_regime": "STABLE" if all_regime else "CHANGED",
              "differential_authority": "FAIL" if migrated else "PASS",
              "AUTH": "PASS" if auth_pass else "FAIL", "classification": classification,
              "next_allowed_action": next_action, "stop_after_AUTH": True, "R2_authorized": False}
    write(output / "corrected-exact-r1-auth.json", result)
    replay_error = (P45.semantic_error(args.replay_of / "corrected-exact-r1-auth.json",
                                       output / "corrected-exact-r1-auth.json")
                    if args.replay_of else None)
    replay_pass = replay_error is None or replay_error <= 1.0e-11
    write(output / "summary.json", {"pass": auth_pass and replay_pass,
          "classification": classification, "replay_max_abs_error": replay_error,
          "replay_pass": replay_pass, "next_allowed_action": next_action,
          "stop_after_AUTH": True, "R2_authorized": False})
    sources = [config_path, continuation_path, ROOT / base["scene"], ROOT / base["executable"],
               authority, wrench_source, qp_dump, R1.PRODUCTION_AUDIT, R1.OPERATOR_AUDIT,
               Path(__file__).resolve(), ROOT / "tools/experiments/run_phase46_corrected_exact_r1.py"]
    write(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
          "command": " ".join(sys.argv), "python": sys.version, "platform": platform.platform(),
          "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__,
                           "scipy": scipy.__version__},
          "sources": {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                      hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}})
    return 0 if auth_pass and replay_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
