#!/usr/bin/env python3
"""P21-T06 2 s nonlinear smoke runner for the frozen 42D local candidate."""
from __future__ import annotations

import argparse, csv, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_hard_qp_42d import HardQpBuilder, independent_oracle  # noqa: E402
from validate_weighted_wbc_tasks import Plant  # noqa: E402
from validate_weighted_wbc_tasks_42d import result, wrench_flu  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NVAR = 42

def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()
def dump(path: Path, value: Any) -> None:
    def native(x: Any) -> Any:
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False, default=native) + "\n")
def rotation_error(quat: np.ndarray, reference: np.ndarray) -> np.ndarray:
    q = quat.copy(); r = reference.copy()
    if q[0] < 0: q = -q
    if r[0] < 0: r = -r
    inverse = np.r_[r[0], -r[1:]]; relative = np.empty(4)
    mujoco.mju_mulQuat(relative, q, inverse)
    if relative[0] < 0: relative = -relative
    norm = float(np.linalg.norm(relative[1:]))
    return 2 * relative[1:] if norm < 1e-14 else 2 * np.arctan2(norm, relative[0]) * relative[1:] / norm

def reference_wrench(builder: HardQpBuilder, cfg: dict[str, Any], qeq: np.ndarray) -> np.ndarray:
    base = builder.build(qeq, np.zeros(12)); a = np.vstack((base["A"], np.eye(12, NVAR)))
    static = independent_oracle(base["H"], base["g"], a, np.r_[base["l"], np.zeros(12)], np.r_[base["u"], np.zeros(12)], cfg["oracle"])
    if not static.get("qp_success"): raise RuntimeError("Frozen zero-nudot static reference solve failed")
    z = builder.transform @ static["x"]
    return np.r_[wrench_flu(builder, qeq, 0, z[18:24]), wrench_flu(builder, qeq, 1, z[24:30])]

def solve_tick(builder: HardQpBuilder, cfg: dict[str, Any], plant: Plant, ref: dict[str, np.ndarray], wrench: np.ndarray, gains: dict[str, Any]) -> dict[str, Any]:
    q = builder.canonical_qpos(plant.data.qpos.copy()); nu = builder.reduced_velocity(q, plant.data.qvel.copy())
    builder.oracle.forward(q); position = builder.oracle.data.site_xpos[builder.oracle.base_control_site].copy()
    canonical = -q[builder.oracle.active_qpos]; indices = np.asarray(cfg["task"]["leg_canonical_indices"])
    overrides = {
        "base_x": np.array([-gains["base_x"][0] * (position[0] - ref["position"][0]) - gains["base_x"][1] * nu[0]]),
        "height": np.array([-gains["height"][0] * (position[2] - ref["position"][2]) - gains["height"][1] * nu[2]]),
        "orientation": -np.asarray(gains["orientation_kp"]) * rotation_error(q[3:7], ref["quaternion"]) - np.asarray(gains["orientation_kd"]) * nu[3:6],
        "leg": gains["leg"][0] * (ref["active"][indices] - canonical[indices]) - gains["leg"][1] * nu[6 + indices],
    }
    return result(builder, cfg, q, nu, wrench, overrides)[0]

def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("--config", type=Path, required=True); ap.add_argument("--output-dir", type=Path, required=True); ap.add_argument("--case-id"); args = ap.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {output}")
    run_cfg, run_inputs = load_config(args.config.resolve()); cfg, local_inputs = load_config(ROOT / run_cfg["source_local_profile"])
    hard, hard_inputs = load_config(ROOT / cfg["source_hard_profile"]); model, model_inputs = load_config(ROOT / hard["model_profile"]); contact, contact_inputs = load_config(ROOT / hard["contact_profile"])
    _, continuous_inputs = load_config(ROOT / contact["continuous_contact_config"]); eqpath = ROOT / model["equilibrium"]; equilibrium = json.loads(eqpath.read_text())
    builder = HardQpBuilder(hard, model, contact, equilibrium); plant = Plant(model, equilibrium); qeq = builder.oracle.sample_qpos(model["samples"][0])
    wrench = reference_wrench(builder, cfg, qeq); builder.oracle.forward(qeq)
    reference = {"position": builder.oracle.data.site_xpos[builder.oracle.base_control_site].copy(), "quaternion": qeq[3:7].copy(), "active": -qeq[builder.oracle.active_qpos].copy()}
    cases = [("tuning", x) for x in run_cfg.get("tuning_cases", run_cfg.get("cases", []))] + [("holdout", x) for x in run_cfg.get("holdout_cases", [])]
    if args.case_id: cases = [(split, case) for split, case in cases if case["id"] == args.case_id]
    if len(cases) != 1: raise RuntimeError("Select exactly one frozen case with --case-id")
    split, case = cases[0]; ticks = int(round(float(run_cfg["duration_s"]) / (plant.model.opt.timestep * int(run_cfg["physics_steps_per_control"]))))
    workspace_profile = json.loads((ROOT / hard["workspace_gate"]["runtime_profile"]).read_text())
    workspace_values = workspace_profile["closure"]["workspace_rad"]
    workspace_bounds = np.asarray([[min(workspace_values[name]), max(workspace_values[name])]
                                   for name in ("hip", "knee", "wheel", "hip", "knee", "wheel")])
    fields = ["case_id", "split", "tick", "time_s", "solver_ok", "tick_finite", "workspace_inside", "workspace_violation_rad", "hard_violation", "saturated", "x_m", "y_m", "height_m", "roll_rad", "pitch_rad", "yaw_rad", "linear_speed_m_s", "angular_speed_rad_s", "left_normal_n", "right_normal_n", "penetration_m", "rolling_slip_m_s", "lateral_slip_m_s", "closure_residual_m", "normalized_slack", "task_cost", "task_residual"] + [f"joint_delta{i}" for i in range(6)] + [f"tau{i}" for i in range(6)]
    maxima = {"abs_x_m":0., "abs_y_m":0., "height_error_m":0., "abs_roll_rad":0., "abs_pitch_rad":0., "abs_yaw_rad":0., "penetration_m":0., "rolling_slip_m_s":0., "lateral_slip_m_s":0., "closure_residual_m":0., "workspace_violation_rad":0., "hard_residual":0., "bound_violation":0., "normalized_slack":0., "task_cost":0., "task_residual":0.}; bilateral = failures = saturation = nonfinite_ticks = workspace_failures = 0; minimum_normal = float("inf")
    output.mkdir(parents=True, exist_ok=True)
    with (output / "ticks.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader(); plant.reset()
        for tick in range(ticks):
            state_q = builder.canonical_qpos(plant.data.qpos.copy())
            joint_delta = (-state_q[builder.oracle.active_qpos]) - reference["active"]
            workspace_violation = float(max(0.0, np.max(workspace_bounds[:, 0] - joint_delta),
                                            np.max(joint_delta - workspace_bounds[:, 1])))
            workspace_inside = workspace_violation == 0.0
            if workspace_inside:
                audit = solve_tick(builder, cfg, plant, reference, wrench, run_cfg["nonlinear_gains"])
            else:
                audit = {"qp_success": False, "runtime_status": "outside_workspace"}
            physical = np.asarray(audit.get("physical_z", [])); task_values = np.asarray([v for task in audit.get("tasks", {}).values() for v in (task["normalized_residual"] + [task["normalized_cost"]])]); tick_finite = bool(workspace_inside and np.all(np.isfinite(physical)) and np.all(np.isfinite(task_values)) and np.isfinite(audit.get("hard_violation", np.nan)) and np.isfinite(audit.get("qp_bound_violation", np.nan))); valid = bool(audit.get("qp_success") and tick_finite and audit.get("hard_violation", np.inf) <= run_cfg["gates"]["maximum_hard_residual"])
            workspace_failures += int(not workspace_inside); maxima["workspace_violation_rad"] = max(maxima["workspace_violation_rad"], workspace_violation)
            failures += int(not valid); torque = np.asarray(audit["physical_z"][12:18]) if valid else np.zeros(6); limits = np.asarray(hard["bounds"]["torque_nm"]); saturated = bool(np.any(np.abs(torque) >= limits - 1e-6)); saturation += int(saturated)
            plant.data.ctrl[:] = 0.;
            for actuator, value in zip(plant.actuators, torque): plant.data.ctrl[actuator] = -value
            for _ in range(int(run_cfg["physics_steps_per_control"])):
                plant.data.xfrc_applied[plant.base_body, :] = 0.
                if int(run_cfg.get("disturbance_start_tick", ticks)) <= tick < int(run_cfg.get("disturbance_start_tick", ticks)) + int(run_cfg.get("disturbance_ticks", 0)):
                    plant.data.xfrc_applied[plant.base_body, :3] = np.asarray(case.get("force_n", [0.,0.,0.]))
                    plant.data.xfrc_applied[plant.base_body, 3:] = np.asarray(case.get("moment_nm", [0.,0.,0.]))
                mujoco.mj_step(plant.model, plant.data)
            metrics = plant.metrics(); tick_finite = bool(tick_finite and np.all(np.isfinite(list(metrics.values()))) and np.all(np.isfinite(torque))); nonfinite_ticks += int(not tick_finite); bilateral += int(metrics["left_normal_n"] > 0 and metrics["right_normal_n"] > 0); minimum_normal = min(minimum_normal, metrics["left_normal_n"], metrics["right_normal_n"])
            for key in ("penetration_m", "rolling_slip_m_s", "lateral_slip_m_s", "closure_residual_m"): maxima[key] = max(maxima[key], metrics[key])
            for key, value in (("abs_x_m", abs(metrics["x_m"]-reference["position"][0])), ("abs_y_m", abs(metrics["y_m"]-reference["position"][1])), ("height_error_m", abs(metrics["height_m"]-reference["position"][2])), ("abs_roll_rad",abs(metrics["roll_rad"])), ("abs_pitch_rad",abs(metrics["pitch_rad"])), ("abs_yaw_rad",abs(metrics["yaw_rad"]))): maxima[key] = max(maxima[key], value)
            maxima["hard_residual"] = max(maxima["hard_residual"], float(audit.get("hard_violation", np.inf))); maxima["bound_violation"] = max(maxima["bound_violation"], float(audit.get("qp_bound_violation", np.inf)))
            slack = np.asarray(audit.get("physical_z", np.full(42, np.inf))[30:42]); normalized_slack = float(np.max(np.abs(slack) / np.tile(cfg["task"]["scales"]["wrench_per_side_flu"], 2))); maxima["normalized_slack"] = max(maxima["normalized_slack"], normalized_slack); task_cost = sum(x["normalized_cost"] for x in audit.get("tasks", {}).values()); task_residual = max((max(abs(v) for v in x["normalized_residual"]) for x in audit.get("tasks", {}).values()), default=float("inf")); maxima["task_cost"] = max(maxima["task_cost"], task_cost); maxima["task_residual"] = max(maxima["task_residual"], task_residual)
            writer.writerow({"case_id":case["id"],"split":split,"tick":tick,"time_s":plant.data.time,"solver_ok":valid,"tick_finite":tick_finite,"workspace_inside":workspace_inside,"workspace_violation_rad":workspace_violation,"hard_violation":audit.get("hard_violation",float("inf")),"saturated":saturated,**metrics,"normalized_slack":normalized_slack,"task_cost":task_cost,"task_residual":task_residual,**{f"joint_delta{i}":value for i,value in enumerate(joint_delta)},**{f"tau{i}":value for i,value in enumerate(torque)}})
    final = plant.metrics(); values = {**maxima, "bilateral_contact_fraction":bilateral/ticks, "minimum_normal_force_n":minimum_normal, "final_linear_speed_m_s":final["linear_speed_m_s"], "final_angular_speed_rad_s":final["angular_speed_rad_s"], "solver_failure_count":failures,"saturation_count":saturation,"nonfinite_tick_count":nonfinite_ticks,"workspace_failure_count":workspace_failures}
    g = run_cfg["gates"]; gates = {"workspace":workspace_failures==0 and values["workspace_violation_rad"]<=g["maximum_workspace_violation_rad"], "solver":failures==0, "finite":nonfinite_ticks==0, "hard":values["hard_residual"]<=g["maximum_hard_residual"] and values["bound_violation"]<=g["maximum_bound_violation"], "saturation":saturation==0, "task":values["task_residual"]<=g.get("maximum_task_residual",float("inf")) and values["task_cost"]<=g.get("maximum_task_cost",float("inf")), "slack":values["normalized_slack"]<=g.get("maximum_normalized_slack",float("inf")), "position":values["abs_x_m"]<=g["maximum_abs_x_m"] and values["abs_y_m"]<=g["maximum_abs_y_m"] and values["height_error_m"]<=g["maximum_height_error_m"], "orientation":values["abs_roll_rad"]<=g["maximum_abs_roll_rad"] and values["abs_pitch_rad"]<=g["maximum_abs_pitch_rad"] and values["abs_yaw_rad"]<=g["maximum_abs_yaw_rad"], "settling":values["final_linear_speed_m_s"]<=g["maximum_final_linear_speed_m_s"] and values["final_angular_speed_rad_s"]<=g["maximum_final_angular_speed_rad_s"], "contact":values["bilateral_contact_fraction"]>=g["minimum_bilateral_contact_fraction"] and values["minimum_normal_force_n"]>=g["minimum_normal_force_n"], "plant":values["penetration_m"]<=g["maximum_penetration_m"] and values["rolling_slip_m_s"]<=g["maximum_abs_rolling_slip_m_s"] and values["lateral_slip_m_s"]<=g["maximum_abs_lateral_slip_m_s"] and values["closure_residual_m"]<=g["maximum_closure_residual_m"]}
    summary = {"schema_version":1,"phase":21,"profile":run_cfg["profile"],"scope":"Frozen nonlinear single-case run; offline SLSQP oracle, no Core claim.","case":{"id":case["id"],"split":split},"ticks":ticks,"reference_wrench_flu":wrench.tolist(),"results":values,"gates":gates,"normalized_slack_report_only":values["normalized_slack"],"pass":all(gates.values())}
    dump(output / "summary.json", summary); script=Path(__file__).resolve(); inputs=run_inputs+local_inputs+hard_inputs+model_inputs+contact_inputs+continuous_inputs; sources=[script,ROOT/"tools/experiments/validate_weighted_wbc_tasks_42d.py",ROOT/"tools/experiments/validate_weighted_wbc_tasks.py",ROOT/"tools/experiments/validate_weighted_wbc_hard_qp_42d.py",ROOT/"tools/experiments/validate_mujoco_weighted_wbc_model.py"]
    dump(output / "manifest.json", {"schema_version":1,"created_at":datetime.now(timezone.utc).isoformat(),"command":[sys.executable,*sys.argv],"interpreter":sys.executable,"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy.__version__,"mujoco":mujoco.__version__,"inputs":{str(p.relative_to(ROOT)):sha(p) for p in inputs},"equilibrium":sha(eqpath),"sources":{str(p.relative_to(ROOT)):sha(p) for p in sources},"outputs":{n:sha(output/n) for n in ("summary.json","ticks.csv")}})
    print(json.dumps({"ticks":ticks,"gates":gates,"pass":summary["pass"]}, indent=2, default=lambda x: x.item() if isinstance(x, np.generic) else x)); return 0 if summary["pass"] else 1
if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as err: print(f"ERROR: {err}",file=sys.stderr); sys.exit(2)
