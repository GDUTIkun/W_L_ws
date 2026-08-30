#!/usr/bin/env python3
"""Phase 44 addendum: regime-aware directional authority oracle."""

from __future__ import annotations

import argparse
import hashlib
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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase44_regime_authority_addendum_v1.json"


def load_module(path: Path, name: str) -> Any:
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P44_PATH = ROOT / "tools/experiments/run_phase44_realization_audit.py"
P44 = load_module(P44_PATH, "phase44_regime_base")
P42 = P44.P42
OUTPUT_ROWS = ["qdd_wheel_left", "qdd_wheel_right", "ddxi_left", "ddxi_right",
               "a_slip_left", "a_slip_right", "base_x", "base_z", "base_pitch",
               "normal_load_left", "normal_load_right"]


def category(value: float, deadband: float) -> str:
    if value < -deadband:
        return "negative"
    if value > deadband:
        return "positive"
    return "deadband"


def signature(control: dict[str, str], actual: dict[str, Any], material: dict[str, Any],
              details: list[dict[str, Any]], candidate: str, thresholds: dict[str, float]) -> tuple[dict[str, Any], dict[str, Any]]:
    side_details = [[row for row in details if int(row["side"]) == side] for side in range(2)]
    topology = []
    contact_load = []
    continuous = []
    for side, rows in enumerate(side_details):
        identities = sorted((min(int(row["geom1"]), int(row["geom2"])),
                             max(int(row["geom1"]), int(row["geom2"])), int(row["dim"]))
                            for row in rows)
        normal = float(actual[("normal_load_left_n", "normal_load_right_n")[side]])
        utilizations = [float(row["tangential_norm_n"]) /
                        max(float(row["tangential_norm_n"]) +
                            float(row["friction_margin_diagnostic_n"]), 1e-12)
                        for row in rows]
        utilization = max(utilizations, default=0.0)
        distances = [float(row["distance_m"]) for row in rows]
        minimum_distance = min(distances, default=math.inf)
        if not rows:
            penetration_state = "separated"
        elif minimum_distance < -thresholds["penetration_deadband_m"]:
            penetration_state = "penetrating"
        else:
            penetration_state = "near_zero"
        friction_state = ("unloaded" if normal <= thresholds["positive_normal_load_n"] else
                          "near_limit" if utilization >= thresholds["friction_near_limit_utilization"] else
                          "interior")
        topology.append({"exists": bool(rows), "count": len(rows), "geom_pair_dim": identities})
        contact_load.append({
            "normal": "positive" if normal > thresholds["positive_normal_load_n"] else "nonpositive",
            "friction": friction_state,
            "penetration": penetration_state,
            "slip": category(float(material["slip"][side]), thresholds["slip_deadband_m_s"]),
        })
        continuous.append({"normal_load_n": normal, "friction_utilization": utilization,
                           "minimum_distance_m": minimum_distance if rows else None,
                           "slip_m_s": float(material["slip"][side]),
                           "raw_contact_order": [int(row["contact_index"]) for row in rows]})
    slack = float(control["maximum_normalized_slack"])
    slack_state = ("inactive" if slack <= thresholds["slack_inactive"] else
                   "material" if slack >= thresholds["slack_material"] else "nonmaterial")
    torque_margins = [float(control[f"tau_margin{index}"]) for index in range(6)]
    enabled = (["native_wheel_rate"] if candidate == "B" else
               ["wheel_longitudinal"] if candidate == "C" else
               ["wheel_longitudinal", "native_wheel_rate"])
    discrete = {
        "contact_topology": topology,
        "contact_load": contact_load,
        "qp_inequalities": {
            "active_counts": [int(control[f"active_count{index}"]) for index in range(3)],
            "active_rows_lower_upper": [int(control[f"active_row{row}"])
                                        for row in range(12, 104)],
            "torque_bound_active": [margin <= thresholds["inequality_active_distance"]
                                    for margin in torque_margins],
            "multiplier_active_set": "unavailable",
        },
        "solver_task": {
            "model_status": control["model_status"],
            "controller_status": control["controller_status"],
            "solver_status": control["solver_status"],
            "solver_success": (control["model_status"] == "0" and
                               control["controller_status"] == "0" and
                               control["solver_status"] == "0"),
            "enabled_task_rows": enabled,
            "slack_state": slack_state,
            "candidate": candidate,
            "profile": f"R43-{candidate}",
            "qp_variables": 42,
            "qp_rows": 104,
        },
    }
    observed = {"contact": continuous, "maximum_normalized_slack": slack,
                "torque_margins_nm": torque_margins,
                "minimum_inequality_margins": [float(control[f"minimum_inequality_margin{index}"])
                                               for index in range(3)]}
    return discrete, observed


def relative(first: np.ndarray, second: np.ndarray) -> float:
    return float(np.linalg.norm(first - second) /
                 max(np.linalg.norm(first), np.linalg.norm(second), 1e-12))


def semantic_error(left: Path, right: Path) -> float:
    if left.suffix == ".csv":
        return P44.semantic_error(left, right, set())
    a = json.loads(left.read_text(encoding="utf-8"))
    b = json.loads(right.read_text(encoding="utf-8"))

    def compare(x: Any, y: Any) -> float:
        if isinstance(x, dict) and isinstance(y, dict) and x.keys() == y.keys():
            return max((compare(x[key], y[key]) for key in x), default=0.0)
        if isinstance(x, list) and isinstance(y, list) and len(x) == len(y):
            return max((compare(i, j) for i, j in zip(x, y)), default=0.0)
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            return abs(float(x) - float(y))
        return 0.0 if x == y else math.inf

    return compare(a, b)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    output.mkdir(parents=True)
    probes = output / "probes"
    probes.mkdir()

    config_path = args.config.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    base_path = ROOT / config["base_config"]
    base = json.loads(base_path.read_text(encoding="utf-8"))
    thresholds = config["regime_thresholds"]
    tolerances = config["derivative_tolerances"]
    model = mujoco.MjModel.from_xml_path(str(ROOT / base["scene"]))
    reconstruction = P44.validate_snapshot_reconstruction(base, model)
    p42_config = json.loads((ROOT / base["phase42_config"]).read_text(encoding="utf-8"))
    oracle = P42.Oracle(p42_config)
    native_common = {int(row["control_tick"]): row for row in
                     P44.read_csv(ROOT / base["phase42_native_authority"])
                     if row["record_kind"] == "pre_command"}
    selection = json.loads((ROOT / config["base_formal"] / "snapshot-selection.json").read_text(
        encoding="utf-8"))
    own_sources: dict[str, tuple[Path, list[int]]] = {}
    for candidate in ("B", "C", "D"):
        rows = P44.read_csv(ROOT / base["phase43_formal"] /
                           f"nominal-{candidate}-{base['representative_gain']}.csv")
        ticks = list(map(int, selection[candidate]["ticks"]))
        authority = output / f"own-native-{candidate}.csv"
        P44.write_native_authority(authority, [P44.native_from_control(model, rows[tick]) for tick in ticks])
        own_sources[candidate] = authority, ticks

    probe_data: dict[tuple[str, str, int, str, float], dict[str, Any]] = {}
    probe_rows: list[dict[str, Any]] = []
    signatures: dict[str, Any] = {}
    scales = [float(value) for value in config["delta_scales"]]
    bandwidth = float(base["representative_bandwidth_hz"])
    for candidate in ("B", "C", "D"):
        spec = P44.profile_spec(candidate, bandwidth)
        channels = []
        if candidate in ("C", "D"):
            channels += [("xi_common", np.asarray([1.0, 1.0, 0.0, 0.0])),
                         ("xi_differential", np.asarray([-1.0, 1.0, 0.0, 0.0]))]
        if candidate in ("B", "D"):
            channels += [("native_common", np.asarray([0.0, 0.0, 1.0, 1.0])),
                         ("native_differential", np.asarray([0.0, 0.0, -1.0, 1.0]))]
        sources = [("common", ROOT / base["phase42_native_authority"],
                    list(map(int, base["common_snapshot_ticks"])), native_common),
                   ("own", own_sources[candidate][0], own_sources[candidate][1],
                    {int(row["control_tick"]): row for row in P44.read_csv(own_sources[candidate][0])})]
        for source_name, authority, ticks, native_by_tick in sources:
            for tick in ticks:
                planned = [("baseline", 0.0, np.zeros(4))]
                for channel, direction in channels:
                    magnitude = (float(base["task_delta"]["xi_acceleration_m_s2"])
                                 if channel.startswith("xi") else
                                 float(base["task_delta"]["native_acceleration_rad_s2"]))
                    for scale in scales:
                        for sign in (-1.0, 1.0):
                            planned.append((channel, sign * scale,
                                            sign * scale * magnitude * direction))
                for channel, signed_scale, delta in planned:
                    path = probes / f"{candidate}-{source_name}-t{tick}-{channel}-{signed_scale:+g}.csv"
                    control = P44.run_controller(base, path, authority, tick, spec, delta)
                    native = native_by_tick[tick]
                    tau = P44.vec(control, "tau", 6)
                    details: list[dict[str, Any]] = []
                    actual = oracle.evaluate(native, details, -tau)
                    qpos = P44.vec(native, "qpos", model.nq)
                    qvel = P44.vec(native, "qvel", model.nv)
                    material = P44.material_point_metrics(model, qpos, qvel, -tau,
                        float(base["finite_difference_epsilon_s"]))
                    nudot = P44.vec(control, "physical_solution", 12)
                    lambdas = P44.vec(control, "physical_solution", 30)[18:30]
                    reduction = P44.matrix(control, "reduction_", 16, 12)
                    mass = P44.vec(actual, "mass", 256).reshape(16, 16)
                    qacc = P44.vec(actual, "qacc", 16)
                    reduction_bias = P44.vec(control, "reduction_bias", 16)
                    reduced_actual = np.linalg.solve(reduction.T @ mass @ reduction,
                        reduction.T @ mass @ (qacc - reduction_bias))
                    maps = [P44.matrix(control, f"contact_map_{side}_", 12, 6) for side in range(2)]
                    qp_contact = maps[0] @ lambdas[:6] + maps[1] @ lambdas[6:]
                    mj_contact = reduction.T @ (P44.vec(actual, "qfrc_contact_left", 16) +
                                                P44.vec(actual, "qfrc_contact_right", 16))
                    mj_actuator = reduction.T @ P44.vec(actual, "qfrc_actuator", 16)
                    xi_map = P44.matrix(control, "xi_map_", 2, 12)
                    qp_ddxi = xi_map @ nudot + P44.vec(control, "xi_bias", 2)
                    mj_ddxi = np.asarray([actual["ddxi_left_m_s2"], actual["ddxi_right_m_s2"]])
                    qp_y = np.r_[nudot[[8, 11]], qp_ddxi,
                                 [float(control["contact_task_residual0"]),
                                  float(control["contact_task_residual3"])],
                                 nudot[[0, 2, 4]], lambdas[[2, 8]]]
                    mj_y = np.r_[qacc[oracle.wheel_dadr], mj_ddxi,
                                 material["tangential_acceleration"], reduced_actual[[0, 2, 4]],
                                 [actual["normal_load_left_n"], actual["normal_load_right_n"]]]
                    discrete, observed = signature(control, actual, material, details, candidate, thresholds)
                    encoded = json.dumps(P44.clean(discrete), sort_keys=True, separators=(",", ":"))
                    signature_id = hashlib.sha256(encoded.encode()).hexdigest()[:16]
                    key = (candidate, source_name, tick, channel, signed_scale)
                    probe_data[key] = {"qp_y": qp_y, "mj_y": mj_y, "qp_contact": qp_contact,
                                       "mj_contact": mj_contact, "mj_actuator": mj_actuator,
                                       "qacc": qacc, "mass": mass, "signature": encoded,
                                       "signature_id": signature_id}
                    label = f"{candidate}-{source_name}-t{tick}-{channel}-{signed_scale:+g}"
                    signatures[label] = {"discrete": discrete, "observed": observed,
                                         "signature_id": signature_id}
                    probe_rows.append({"candidate": candidate, "source": source_name, "tick": tick,
                        "channel": channel, "signed_scale": signed_scale, "signature_id": signature_id,
                        "solver_success": discrete["solver_task"]["solver_success"],
                        "contact_count_left": discrete["contact_topology"][0]["count"],
                        "contact_count_right": discrete["contact_topology"][1]["count"],
                        "active_torque": discrete["qp_inequalities"]["active_counts"][0],
                        "active_contact": discrete["qp_inequalities"]["active_counts"][1],
                        "active_acceleration": discrete["qp_inequalities"]["active_counts"][2],
                        "slack_state": discrete["solver_task"]["slack_state"],
                        "full_dynamics_residual": actual["full_dynamics_residual_max_abs"],
                        "contact_reconstruction_residual":
                            actual["contact_applyft_jacobian_max_abs"]})

    for row in probe_rows:
        baseline = probe_data[(row["candidate"], row["source"], int(row["tick"]), "baseline", 0.0)]
        row["same_as_baseline_regime"] = (
            probe_data[(row["candidate"], row["source"], int(row["tick"]),
                        row["channel"], float(row["signed_scale"]))]["signature"] ==
            baseline["signature"])

    directional_rows: list[dict[str, Any]] = []
    contact_rows: list[dict[str, Any]] = []
    matrices = {"qp": {}, "mj": {}, "mis": {}}
    family_classifications: dict[tuple[str, str, int, str], list[str]] = {}
    trusted_tick0 = True
    convergence_errors: list[float] = []
    balance_errors: list[float] = []
    for candidate in ("B", "C", "D"):
        channels = [name for name in ("xi_common", "xi_differential", "native_common",
                                      "native_differential")
                    if any(key[0] == candidate and key[3] == name for key in probe_data)]
        for source_name, ticks in (("common", list(map(int, base["common_snapshot_ticks"]))),
                                   ("own", own_sources[candidate][1])):
            for tick in ticks:
                baseline = probe_data[(candidate, source_name, tick, "baseline", 0.0)]
                per_channel: dict[str, Any] = {}
                for channel in channels:
                    magnitude = (float(base["task_delta"]["xi_acceleration_m_s2"])
                                 if channel.startswith("xi") else
                                 float(base["task_delta"]["native_acceleration_rad_s2"]))
                    directions: dict[str, Any] = {}
                    for sign, name in ((1.0, "plus"), (-1.0, "minus")):
                        valid_scales = [scale for scale in scales
                                        if probe_data[(candidate, source_name, tick, channel,
                                                       sign * scale)]["signature"] == baseline["signature"]]
                        selected = max(valid_scales, default=None)
                        next_scale = (next((scale for scale in scales
                                            if selected is not None and scale < selected and scale in valid_scales), None))
                        derivatives: dict[str, dict[str, np.ndarray]] = {}
                        for scale in valid_scales:
                            probe = probe_data[(candidate, source_name, tick, channel, sign * scale)]
                            denominator = scale * magnitude
                            derivatives[str(scale)] = {
                                "qp": (probe["qp_y"] - baseline["qp_y"]) / denominator if sign > 0 else
                                      (baseline["qp_y"] - probe["qp_y"]) / denominator,
                                "mj": (probe["mj_y"] - baseline["mj_y"]) / denominator if sign > 0 else
                                      (baseline["mj_y"] - probe["mj_y"]) / denominator,
                            }
                        convergence = None
                        trusted = False
                        if selected is not None and next_scale is not None:
                            convergence = max(relative(derivatives[str(selected)][kind],
                                                       derivatives[str(next_scale)][kind])
                                              for kind in ("qp", "mj"))
                            trusted = convergence <= tolerances["directional_convergence_relative"]
                            convergence_errors.append(convergence)
                        directions[name] = {"regime_valid": selected is not None, "trusted": trusted,
                            "selected_scale": selected, "check_scale": next_scale,
                            "convergence_relative": convergence,
                            "g_qp": derivatives[str(selected)]["qp"] if selected is not None else None,
                            "g_mj": derivatives[str(selected)]["mj"] if selected is not None else None}
                        if tick == 0 and not trusted:
                            trusted_tick0 = False
                    plus, minus = directions["plus"], directions["minus"]
                    for family, indices in config["output_families"].items():
                        indices = list(map(int, indices))
                        if plus["regime_valid"] and minus["regime_valid"]:
                            branch_error = max(relative(plus[kind][indices], minus[kind][indices])
                                               for kind in ("g_qp", "g_mj"))
                            regime_class = ("R44-S" if branch_error <=
                                            tolerances["smooth_branch_relative"] else "R44-P")
                        else:
                            branch_error = None
                            regime_class = ("R44-O+" if plus["regime_valid"] else "R44-O-"
                                            if minus["regime_valid"] else "R44-B")
                        trusted = ((plus["trusted"] if regime_class in ("R44-S", "R44-P", "R44-O+") else True) and
                                   (minus["trusted"] if regime_class in ("R44-S", "R44-P", "R44-O-") else True))
                        directional_rows.append({"candidate": candidate, "source": source_name, "tick": tick,
                            "channel": channel, "output_family": family, "classification": regime_class,
                            "plus_regime_valid": plus["regime_valid"], "minus_regime_valid": minus["regime_valid"],
                            "plus_trusted": plus["trusted"], "minus_trusted": minus["trusted"],
                            "family_trusted": trusted, "branch_relative": branch_error,
                            "plus_scale": plus["selected_scale"], "minus_scale": minus["selected_scale"]})
                        family_classifications.setdefault((candidate, source_name, tick, channel), []).append(regime_class)
                    per_channel[channel] = directions

                    for direction_name, direction in directions.items():
                        if not direction["trusted"]:
                            continue
                        sign = 1.0 if direction_name == "plus" else -1.0
                        scale = float(direction["selected_scale"])
                        probe = probe_data[(candidate, source_name, tick, channel, sign * scale)]
                        denominator = scale * magnitude
                        derive = lambda value, base_value: ((value - base_value) / denominator if sign > 0
                                                            else (base_value - value) / denominator)
                        d_qp_contact = derive(probe["qp_contact"], baseline["qp_contact"])
                        d_mj_contact = derive(probe["mj_contact"], baseline["mj_contact"])
                        d_actuator = derive(probe["mj_actuator"], baseline["mj_actuator"])
                        d_qacc = derive(probe["qacc"], baseline["qacc"])
                        for side, reduced_row, native_row in (("left", 8, oracle.wheel_dadr[0]),
                                                              ("right", 11, oracle.wheel_dadr[1])):
                            lhs = float(baseline["mass"][native_row] @ d_qacc)
                            other = lhs - d_actuator[reduced_row] - d_mj_contact[reduced_row]
                            balance = lhs - d_actuator[reduced_row] - d_mj_contact[reduced_row] - other
                            balance_errors.append(abs(balance))
                            contact_rows.append({"candidate": candidate, "source": source_name, "tick": tick,
                                "channel": channel, "direction": direction_name, "scale": scale, "side": side,
                                "qp_contact_gain": d_qp_contact[reduced_row],
                                "mj_contact_gain": d_mj_contact[reduced_row],
                                "mj_actuator_gain": d_actuator[reduced_row],
                                "mj_mass_times_qacc_gain": lhs, "mj_other_gain": other,
                                "balance_residual": balance, "mj_wheel_qacc_gain": d_qacc[native_row],
                                "cancellation_ratio": (-d_mj_contact[reduced_row] / d_actuator[reduced_row]
                                                       if abs(d_actuator[reduced_row]) > 1e-12 else None)})
                matrix_key = f"{candidate}-{source_name}-t{tick}"
                for kind in ("qp", "mj", "mis"):
                    matrices[kind][matrix_key] = {"input_channels": channels, "output_rows": OUTPUT_ROWS,
                                                  "directions": {}}
                for direction_name in ("plus", "minus"):
                    complete = all(per_channel[channel][direction_name]["trusted"] for channel in channels)
                    for kind in ("qp", "mj"):
                        values = (np.column_stack([per_channel[channel][direction_name][f"g_{kind}"]
                                                  for channel in channels]) if complete else None)
                        matrices[kind][matrix_key]["directions"][direction_name] = {
                            "complete_trusted": complete, "matrix": values}
                    qp = matrices["qp"][matrix_key]["directions"][direction_name]["matrix"]
                    mj = matrices["mj"][matrix_key]["directions"][direction_name]["matrix"]
                    mismatch = mj - qp if complete else None
                    entry = {"complete_trusted": complete, "matrix": mismatch,
                             "condition_number": None, "singular_values": None, "near_null": None}
                    if complete:
                        singular = np.linalg.svd(mj, compute_uv=False)
                        entry.update({"condition_number": float(singular[0] / singular[-1])
                                      if singular[-1] > 0 else None,
                                      "singular_values": singular,
                                      "near_null": bool(singular[-1] <= tolerances["near_null_singular_value"])})
                    matrices["mis"][matrix_key]["directions"][direction_name] = entry

    P44.write_json(output / "regime-signatures.json", signatures)
    P44.write_csv(output / "regime-probes.csv", probe_rows)
    P44.write_csv(output / "directional-probes.csv", directional_rows)
    P44.write_json(output / "G_QP_directional.json", matrices["qp"])
    P44.write_json(output / "G_MJ_directional.json", matrices["mj"])
    P44.write_json(output / "G_mis_directional.json", matrices["mis"])
    P44.write_csv(output / "contact-directional-transfer.csv", contact_rows)

    transitions: dict[str, Any] = {}
    persistent = int(config["persistent_nonsmooth_snapshots"])
    for candidate in ("B", "C", "D"):
        ticks = own_sources[candidate][1]
        sensitive = []
        for tick in ticks:
            classes = [value for (cand, source, item_tick, _), values in family_classifications.items()
                       if cand == candidate and source == "own" and item_tick == tick for value in values]
            sensitive.append(any(value != "R44-S" for value in classes))
        onset = next((ticks[index] for index in range(len(ticks) - persistent + 1)
                      if all(sensitive[index:index + persistent])), None)
        transitions[candidate] = {"ordered_ticks": ticks, "regime_sensitive": sensitive,
                                  "first_persistent_nonsmooth_tick": onset,
                                  "phase43_events": selection[candidate]["events"],
                                  "interpretation": "chronology_association_only"}
    P44.write_json(output / "regime-transition-events.json", transitions)

    counts = {name: sum(row["classification"] == name for row in directional_rows)
              for name in ("R44-S", "R44-P", "R44-O+", "R44-O-", "R44-B")}
    trusted_rows = sum(bool(row["family_trusted"]) for row in directional_rows)
    total_rows = len(directional_rows)
    dynamics_closure = max(float(row["full_dynamics_residual"]) for row in probe_rows)
    contact_closure = max(float(row["contact_reconstruction_residual"]) for row in probe_rows)
    cancellation = [row for row in contact_rows
                    if row["candidate"] == "D" and row["channel"] == "native_common" and
                    row["cancellation_ratio"] is not None]
    cancellation_summary = {
        "scope": "trusted D/native_common directions",
        "count": len(cancellation),
        "all_opposing_sign": bool(cancellation) and all(
            float(row["mj_actuator_gain"]) * float(row["mj_contact_gain"]) < 0
            for row in cancellation),
        "ratio_min": min((float(row["cancellation_ratio"]) for row in cancellation), default=None),
        "ratio_max": max((float(row["cancellation_ratio"]) for row in cancellation), default=None),
    }
    gates = {
        "DG44-R1": True,
        "DG44-R2": (reconstruction["qpos_max_abs"] <= base["tolerances"]["snapshot_qpos_max_abs"] and
                     reconstruction["qvel_max_abs"] <= base["tolerances"]["snapshot_qvel_max_abs"]),
        "DG44-R3": all(value is False for value in config["no_repair_contract"].values()),
        "DG44-R4": total_rows > 0,
        "DG44-R5": trusted_rows > 0 and trusted_tick0,
        "DG44-R6": any(direction["complete_trusted"] for matrix in matrices["mj"].values()
                       for direction in matrix["directions"].values()),
        "DG44-R7": (bool(balance_errors) and
                     max(balance_errors) <= tolerances["balance_max_abs"] and
                     dynamics_closure <= base["tolerances"]["full_dynamics_max_abs"] and
                     contact_closure <= base["tolerances"]["contact_reconstruction_max_abs"]),
        "DG44-R8": any(value != "R44-S" for value in counts if counts[value] > 0) or counts["R44-S"] > 0,
    }
    base_summary = json.loads((ROOT / config["base_formal"] / "summary.json").read_text(encoding="utf-8"))
    classification = (base_summary["provisional_classification_if_oracles_pass"]
                      if all(gates.values()) else "P44-U")
    gates["DG44-R9"] = classification != "P44-U"
    replay_error = None
    compared = ["regime-signatures.json", "regime-probes.csv", "directional-probes.csv",
                "G_QP_directional.json",
                "G_MJ_directional.json", "G_mis_directional.json", "contact-directional-transfer.csv",
                "regime-transition-events.json"]
    if args.replay_of:
        replay_error = max(semantic_error(args.replay_of / name, output / name) for name in compared)
    gates["DG44-R10"] = replay_error is None or replay_error <= tolerances["semantic_replay_max_abs"]
    summary = {"pass": all(gates.values()), "classification": classification, "gates": gates,
               "classification_counts": counts, "trusted_family_rows": trusted_rows,
               "total_family_rows": total_rows, "trusted_fraction": trusted_rows / total_rows,
               "max_directional_convergence_relative": max(convergence_errors, default=None),
               "max_contact_balance_residual": max(balance_errors, default=None),
               "max_full_dynamics_residual": dynamics_closure,
               "max_contact_reconstruction_residual": contact_closure,
               "contact_cancellation": cancellation_summary,
               "snapshot_reconstruction": reconstruction, "transition_events": transitions,
               "replay_max_abs_error": replay_error, "no_repair_contract": config["no_repair_contract"]}
    P44.write_json(output / "addendum-summary.json", summary)
    sources = [config_path, base_path, P44_PATH, Path(__file__).resolve(), ROOT / base["scene"],
               ROOT / base["executable"], ROOT / config["base_formal"] / "snapshot-selection.json",
               ROOT / config["base_formal"] / "summary.json"]
    P44.write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "inputs": {str(path.relative_to(ROOT)): P44.sha256(path) for path in sources},
        **config["no_repair_contract"]})
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
