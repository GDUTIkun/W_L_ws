#!/usr/bin/env python3
"""Phase45 unified contact-consistent rolling repair formal audit."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import json
import math
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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase45_contact_consistent_rolling_v1.json"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


P44 = load(ROOT / "tools/experiments/run_phase44_realization_audit.py", "phase45_p44")
P42 = P44.P42


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)): return value.item()
    if isinstance(value, dict): return {key: clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [clean(item) for item in value]
    return value


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(clean(value), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows: raise RuntimeError(f"empty CSV: {path}")
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(clean(rows))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(config: dict[str, Any], output: Path, case: str, *, ticks: int | None = None,
            authority: Path | None = None, tick: int = 0,
            delta: np.ndarray | None = None,
            wrench_trim: np.ndarray | None = None) -> list[str]:
    gain = config["gain"]
    trim = np.zeros(4) if wrench_trim is None else wrench_trim
    result = [str(ROOT / config["executable"]), str(ROOT / config["scene"]), str(output),
              case, "nominal", str(gain["xi_kp_s2"]), str(gain["xi_kd_s"]),
              "0", *map(str, trim)]
    if ticks is not None:
        result.append(str(ticks))
    if authority is not None:
        values = np.zeros(4) if delta is None else delta
        result += [str(authority), str(tick), *map(str, values)]
    return result


def run(config: dict[str, Any], output: Path, case: str, **kwargs: Any) -> list[dict[str, str]]:
    subprocess.run(command(config, output, case, **kwargs), cwd=ROOT, check=True)
    return P44.read_csv(output)


def semantic_error(left: Path, right: Path) -> float:
    if left.suffix == ".csv": return P44.semantic_error(left, right, {"wbc_time_s"})
    a = json.loads(left.read_text()); b = json.loads(right.read_text())
    def compare(x: Any, y: Any) -> float:
        if isinstance(x, dict) and isinstance(y, dict) and x.keys() == y.keys():
            return max((compare(x[k], y[k]) for k in x), default=0.0)
        if isinstance(x, list) and isinstance(y, list) and len(x) == len(y):
            return max((compare(i, j) for i, j in zip(x, y)), default=0.0)
        if isinstance(x, (int, float)) and isinstance(y, (int, float)):
            if math.isnan(float(x)) and math.isnan(float(y)): return 0.0
            return abs(float(x) - float(y))
        return 0.0 if x == y else math.inf
    return compare(a, b)


def native_state(rows: list[dict[str, str]], tick: int = 0) -> dict[str, str]:
    return next(row for row in rows if row["record_kind"] == "pre_command" and
                int(row["control_tick"]) == tick)


def actual(config: dict[str, Any], model: mujoco.MjModel, oracle: Any,
           native: dict[str, str], control: dict[str, str]) -> dict[str, Any]:
    torque = -P44.vec(control, "tau", 6)
    details: list[dict[str, Any]] = []
    dynamics = oracle.evaluate(native, details, torque)
    qpos = P44.vec(native, "qpos", model.nq); qvel = P44.vec(native, "qvel", model.nv)
    material = P44.material_point_metrics(model, qpos, qvel, torque, 1.0e-6)
    return {"dynamics": dynamics, "details": details, "material": material,
            "torque": torque, "qpos": qpos, "qvel": qvel}


def independent_material_row(model: mujoco.MjModel, qpos: np.ndarray, qvel: np.ndarray,
                             reduction: np.ndarray, reduction_bias: np.ndarray) -> dict[str, Any]:
    data = mujoco.MjData(model); data.qpos[:] = qpos; data.qvel[:] = qvel
    weld = P42.required_id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld")
    data.eq_active[weld] = 0; mujoco.mj_forward(model, data)
    floor = P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    geoms = [P42.required_id(model, mujoco.mjtObj.mjOBJ_GEOM, n)
             for n in ("left_wheel_collision", "right_wheel_collision")]
    bodies = [P42.required_id(model, mujoco.mjtObj.mjOBJ_BODY, n)
              for n in ("left_wheel_body", "right_wheel_body")]
    points: list[np.ndarray | None] = [None, None]; normals: list[np.ndarray | None] = [None, None]
    for contact in data.contact:
        for side, geom in enumerate(geoms):
            if {int(contact.geom1), int(contact.geom2)} == {geom, floor} and points[side] is None:
                points[side] = np.asarray(contact.pos).copy()
                normal = np.asarray(contact.frame).reshape(3, 3)[0].copy()
                normals[side] = normal if int(contact.geom2) == geom else -normal
    maps = np.zeros((2, 12)); bias = np.zeros(2); slip = np.zeros(2)
    for side, body in enumerate(bodies):
        if points[side] is None or normals[side] is None: raise RuntimeError("missing contact")
        normal = normals[side] / np.linalg.norm(normals[side]); tangent = np.array([1., 0., 0.])
        tangent -= normal * tangent.dot(normal); tangent /= np.linalg.norm(tangent)
        radius = points[side] - np.asarray(data.xpos[body])
        jp = np.zeros((3, model.nv)); jr = np.zeros_like(jp); mujoco.mj_jacBody(model, data, jp, jr, body)
        tree = tangent @ (jp - np.cross(np.eye(3), radius) @ jr)
        maps[side] = tree @ reduction; slip[side] = tree @ qvel
        velocities = []
        for sign in (-1., 1.):
            probe = mujoco.MjData(model); probe.qpos[:] = qpos
            mujoco.mj_integratePos(model, probe.qpos, qvel, sign * 1.0e-6)
            probe.qvel[:] = qvel; probe.eq_active[weld] = 0; mujoco.mj_forward(model, probe)
            pjp = np.zeros((3, model.nv)); pjr = np.zeros_like(pjp)
            mujoco.mj_jacBody(model, probe, pjp, pjr, body)
            velocities.append(pjp @ qvel + np.cross(pjr @ qvel, radius))
        jdot_v = tangent @ ((velocities[1] - velocities[0]) / 2.0e-6)
        bias[side] = tree @ reduction_bias + jdot_v
    return {"map": maps, "bias": bias, "slip": slip}


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True); parser.add_argument("--replay-of", type=Path)
    args = parser.parse_args(); output = args.output.resolve()
    if output.exists(): raise RuntimeError(f"output exists: {output}")
    output.mkdir(parents=True); probes = output / "probes"; probes.mkdir()
    config_path = args.config.resolve(); config = json.loads(config_path.read_text())
    gates_cfg = config["gates"]; authority_cfg = config["authority"]
    model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
    p42_config = json.loads((ROOT / config["phase42_config"]).read_text())
    oracle = P42.Oracle(p42_config)

    baseline_a = run(config, output / "baseline-a.csv", "R43-0_nominal", ticks=150)
    baseline_b = run(config, output / "baseline-b.csv", "R43-0_nominal", ticks=150)
    failure_a = next((int(r["tick"]) for r in baseline_a if r["contact_right"] != "1"), None)
    failure_b = next((int(r["tick"]) for r in baseline_b if r["contact_right"] != "1"), None)
    baseline_semantic = semantic_error(output / "baseline-a.csv", output / "baseline-b.csv")

    authority_path = ROOT / config["phase42_native_authority"]
    native = native_state(P44.read_csv(authority_path))
    base_control = run(config, probes / "tick0-base.csv", "R45-H0",
                       authority=authority_path)[0]
    base_actual = actual(config, model, oracle, native, base_control)
    reduction = P44.matrix(base_control, "reduction_", 16, 12)
    reduction_bias = P44.vec(base_control, "reduction_bias", 16)
    independent = independent_material_row(model, base_actual["qpos"], base_actual["qvel"],
                                           reduction, reduction_bias)
    controller_map = P44.matrix(base_control, "rolling_map_", 2, 12)
    controller_bias = np.asarray([float(base_control["rolling_bias_left"]),
                                  float(base_control["rolling_bias_right"])])
    controller_slip = np.asarray([float(base_control["rolling_velocity_left"]),
                                  float(base_control["rolling_velocity_right"])])
    oracle_row = {
        "velocity_max_abs_error_m_s": float(np.max(np.abs(controller_slip-independent["slip"]))),
        "map_max_abs_error": float(np.max(np.abs(controller_map-independent["map"]))),
        "bias_max_abs_error_m_s2": float(np.max(np.abs(controller_bias-independent["bias"]))),
        "controller_slip_m_s": controller_slip, "oracle_slip_m_s": independent["slip"],
        "controller_map": controller_map, "oracle_map": independent["map"],
        "controller_bias_m_s2": controller_bias, "oracle_bias_m_s2": independent["bias"],
    }
    write_json(output / "rolling-task-oracle.json", oracle_row)
    write_csv(output / "rolling-task-oracle.csv", [
        {"side": side, "controller_slip_m_s": controller_slip[side],
         "oracle_slip_m_s": independent["slip"][side],
         "controller_bias_m_s2": controller_bias[side], "oracle_bias_m_s2": independent["bias"][side]}
        for side in range(2)])

    dyn = base_actual["dynamics"]; mat = base_actual["material"]
    equilibrium = [{"side": side, "ddxi_m_s2": dyn[("ddxi_left_m_s2", "ddxi_right_m_s2")[side]],
                    "material_tangent_acceleration_m_s2": mat["tangential_acceleration"][side],
                    "native_wheel_qdd_rad_s2": dyn[("wheel_ddq_left_rad_s2", "wheel_ddq_right_rad_s2")[side]],
                    "normal_load_n": dyn[("normal_load_left_n", "normal_load_right_n")[side]],
                    "rolling_active": int(base_control[("rolling_active_left", "rolling_active_right")[side]])}
                   for side in range(2)]
    write_csv(output / "tick0-equilibrium.csv", equilibrium)
    oracle_pass = (oracle_row["velocity_max_abs_error_m_s"] <= gates_cfg["maximum_oracle_velocity_error_m_s"] and
                   oracle_row["map_max_abs_error"] <= gates_cfg["maximum_oracle_affine_acceleration_error_m_s2"] and
                   oracle_row["bias_max_abs_error_m_s2"] <= gates_cfg["maximum_oracle_affine_acceleration_error_m_s2"])
    eq_pass = all(abs(r["ddxi_m_s2"]) <= gates_cfg["maximum_equilibrium_ddxi_abs_m_s2"] and
                  abs(r["material_tangent_acceleration_m_s2"]) <= gates_cfg["maximum_equilibrium_slip_acceleration_abs_m_s2"] and
                  r["rolling_active"] == 1 for r in equilibrium)

    directional: list[dict[str, Any]] = []; probe_cache: dict[tuple[str, float, int], tuple[np.ndarray, np.ndarray, dict[str, Any]]] = {}
    mode_signs = {"common": np.array([1., 1.]), "differential": np.array([-1., 1.])}
    xi_delta = float(authority_cfg["xi_delta_m_s2"]); slip_delta = float(authority_cfg["slip_delta_m_s2"])
    for mode, signs in (mode_signs.items() if eq_pass else ()):
        for scale in map(float, authority_cfg["delta_scales"]):
            for sign in (-1, 1):
                delta = np.r_[sign*scale*xi_delta*signs, sign*scale*slip_delta*signs]
                path = probes / f"authority-{mode}-{scale:g}-{sign:+d}.csv"
                control = run(config, path, "R45-H0", authority=authority_path, delta=delta)[0]
                observed = actual(config, model, oracle, native, control)
                qp_side = np.asarray([float(control["physical_ddxi_left"]),
                                      float(control["physical_ddxi_right"]),
                                      float(control["qp_rolling_acceleration_left"]),
                                      float(control["qp_rolling_acceleration_right"])])
                mj_side = np.r_[[observed["dynamics"]["ddxi_left_m_s2"], observed["dynamics"]["ddxi_right_m_s2"]],
                                observed["material"]["tangential_acceleration"]]
                probe_cache[(mode, scale, sign)] = qp_side, mj_side, observed
            minus = probe_cache[(mode, scale, -1)]; plus = probe_cache[(mode, scale, 1)]
            denominator = 2.0 * scale
            qp_gain = (plus[0] - minus[0]) / denominator
            mj_gain = (plus[1] - minus[1]) / denominator
            qp_mode = np.array([0.5*(qp_gain[0]+qp_gain[1]), 0.5*(qp_gain[2]+qp_gain[3])]) if mode == "common" else np.array([0.5*(qp_gain[1]-qp_gain[0]), 0.5*(qp_gain[3]-qp_gain[2])])
            mj_mode = np.array([0.5*(mj_gain[0]+mj_gain[1]), 0.5*(mj_gain[2]+mj_gain[3])]) if mode == "common" else np.array([0.5*(mj_gain[1]-mj_gain[0]), 0.5*(mj_gain[3]-mj_gain[2])])
            directional.append({"mode": mode, "scale": scale,
                "g_qp_xi": qp_mode[0]/xi_delta, "g_qp_slip": qp_mode[1]/slip_delta,
                "g_mj_xi": mj_mode[0]/xi_delta, "g_mj_slip": mj_mode[1]/slip_delta,
                "g_qp_projected": 0.5*(qp_mode[0]/xi_delta+qp_mode[1]/slip_delta),
                "g_mj_projected": 0.5*(mj_mode[0]/xi_delta+mj_mode[1]/slip_delta)})
    reference = {(r["mode"]): r for r in directional if r["scale"] == 1.0}
    for row in directional:
        base = reference[row["mode"]]
        row["qp_convergence_relative"] = abs(row["g_qp_projected"]-base["g_qp_projected"])/max(abs(base["g_qp_projected"]), 1e-12)
        row["mj_convergence_relative"] = abs(row["g_mj_projected"]-base["g_mj_projected"])/max(abs(base["g_mj_projected"]), 1e-12)
        row["sign_match"] = row["g_qp_projected"] * row["g_mj_projected"] > 0.0
    if directional:
        write_csv(output / "directional-authority.csv", directional)
        write_json(output / "directional-authority.json", reference)
    else:
        write_csv(output / "directional-authority.csv", [{"entered": False, "reason": "DG45-EQ must PASS first"}])
        write_json(output / "directional-authority.json", {"entered": False, "reason": "DG45-EQ must PASS first"})

    transfer = []
    for mode in (mode_signs if eq_pass else ()):
        minus = probe_cache[(mode, 1.0, -1)][2]["dynamics"]
        plus = probe_cache[(mode, 1.0, 1)][2]["dynamics"]
        for name in ("qfrc_actuator", "qfrc_contact_left", "qfrc_contact_right"):
            change = (P44.vec(plus, name, 16)-P44.vec(minus, name, 16))/2.0
            transfer.append({"mode": mode, "quantity": name, "norm": np.linalg.norm(change),
                             "wheel_left": change[8], "wheel_right": change[11]})
    write_csv(output / "contact-transfer.csv", transfer if transfer else
              [{"entered": False, "reason": "DG45-EQ must PASS first"}])
    write_csv(output / "decomposition.csv", [{"side": side, "xi_acceleration": equilibrium[side]["ddxi_m_s2"],
        "material_acceleration": equilibrium[side]["material_tangent_acceleration_m_s2"],
        "native_wheel_acceleration": equilibrium[side]["native_wheel_qdd_rad_s2"]} for side in range(2)])
    write_json(output / "decomposition.json", {"scope": "tick0; rollout decomposition not entered before AUTH PASS",
        "equilibrium": equilibrium})

    auth_pass = bool(directional) and all(
        r["sign_match"] and abs(r["g_mj_projected"]) >= authority_cfg["minimum_abs_actual_projected_gain"] and
        r["qp_convergence_relative"] <= authority_cfg["maximum_directional_convergence_relative"] and
        r["mj_convergence_relative"] <= authority_cfg["maximum_directional_convergence_relative"] for r in directional)
    gates = {"DG45-BASE": failure_a == failure_b == config["baseline_failure_tick"] and baseline_semantic == 0.0,
             "DG45-ORACLE": oracle_pass, "DG45-EQ": eq_pass, "DG45-AUTH": auth_pass,
             "DG45-REAL": oracle_pass and eq_pass, "DG45-SHORT": False, "DG45-ROLL": False,
             "DG45-CONTACT": False, "DG45-FULLBODY": False, "DG45-WR": False,
             "DG45-WBC": False, "DG45-REAUDIT": False}
    entered = {"short": False, "nominal_10s": False, "post_repair_authority": False}
    classification = "P45-STRUCTURE-FAIL" if gates["DG45-BASE"] and oracle_pass and (not eq_pass or not auth_pass) else "P45-U"
    replay_error = None
    compared = ["rolling-task-oracle.json", "rolling-task-oracle.csv", "tick0-equilibrium.csv",
                "directional-authority.csv", "directional-authority.json", "contact-transfer.csv",
                "decomposition.csv", "decomposition.json"]
    if args.replay_of:
        replay_error = max(semantic_error(args.replay_of/name, output/name) for name in compared)
    replay_pass = replay_error is None or replay_error <= gates_cfg["semantic_replay_max_abs"]
    summary = {"pass": all(gates.values()) and replay_pass, "classification": classification,
               "gates": gates, "entered": entered, "baseline_failure_ticks": [failure_a, failure_b],
               "baseline_semantic_error": baseline_semantic, "oracle": oracle_row,
               "equilibrium": equilibrium, "authority_scale1": reference,
               "replay_max_abs_error": replay_error, "scope_contract": config["scope_contract"]}
    write_json(output / "gate-results.json", gates); write_json(output / "summary.json", summary)
    for name in ("short-rollout.csv", "nominal-rollout.csv", "wrench-audit.csv"):
        write_csv(output / name, [{"entered": False, "reason": "DG45-AUTH must PASS first"}])
    write_json(output / "post-repair-authority.json", {"entered": False, "reason": "10 s nominal not authorized"})
    sources = [config_path, Path(__file__).resolve(), ROOT/config["scene"], ROOT/config["executable"], authority_path]
    write_json(output / "manifest.json", {"created_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(sys.argv), "replay_of": str(args.replay_of) if args.replay_of else None,
        "python": sys.version, "platform": platform.platform(),
        "dependencies": {"mujoco": mujoco.__version__, "numpy": np.__version__, "scipy": scipy.__version__},
        "inputs": {str(p.relative_to(ROOT)): sha256(p) for p in sources}, **config["scope_contract"]})
    return 0 if summary["pass"] else 2


if __name__ == "__main__": raise SystemExit(main())
