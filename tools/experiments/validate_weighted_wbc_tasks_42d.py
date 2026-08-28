#!/usr/bin/env python3
"""P21-T06 local algebraic oracle for the frozen 42D weighted-task candidate."""
from __future__ import annotations

import argparse, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_hard_qp_42d import HardQpBuilder, corpus, independent_oracle  # noqa: E402
from validate_weighted_wbc_continuous_pfaffian import A_matrix  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
NVAR = 42

def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()
def mx(x: np.ndarray) -> float: return float(np.max(np.abs(x))) if x.size else 0.0
def dump(path: Path, value: Any) -> None:
    def native(x: Any) -> Any:
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False, default=native) + "\n")

def base_rotation(builder: HardQpBuilder, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    builder.oracle.forward(q)
    return builder.oracle.data.site_xpos[builder.oracle.base_control_site].copy(), builder.oracle.data.site_xmat[builder.oracle.base_control_site].reshape(3, 3).copy()

def wrench_flu(builder: HardQpBuilder, q: np.ndarray, side: int, wc: np.ndarray) -> np.ndarray:
    patch = builder.patch.geometry(q, side); pb, rnb = base_rotation(builder, q)
    rc = np.column_stack((patch["rolling"], patch["lateral"], builder.patch.n))
    force_n, moment_n = rc @ wc[:3], rc @ wc[3:]
    return np.r_[rnb.T @ force_n, rnb.T @ (moment_n + np.cross(patch["contact_center"] - pb, force_n))]

def wrench_flu_map(builder: HardQpBuilder, q: np.ndarray, side: int) -> np.ndarray:
    return np.column_stack([wrench_flu(builder, q, side, np.eye(6)[:, i]) for i in range(6)])

def bias_contact(builder: HardQpBuilder, q: np.ndarray, nu: np.ndarray, side: int) -> np.ndarray:
    step = float(builder.oracle.config["solver"]["second_difference_step"])
    plus, minus = (builder.oracle.integrate_flow(q, nu, s * step) for s in (1., -1.))
    return (A_matrix(builder.oracle, builder.patch, plus, side) - A_matrix(builder.oracle, builder.patch, minus, side)) @ nu / (2 * step)

def add(h: np.ndarray, g: np.ndarray, a: np.ndarray, target: np.ndarray, scale: np.ndarray, weight: float) -> dict[str, Any]:
    an, tn = a / scale[:, None], target / scale
    h += weight * an.T @ an; g -= weight * an.T @ tn
    return {"A": a, "target": target, "scale": scale, "weight": weight}

def task_problem(builder: HardQpBuilder, cfg: dict[str, Any], q: np.ndarray, nu: np.ndarray, reference: np.ndarray, overrides: dict[str, np.ndarray] | None = None) -> tuple[dict[str, Any], list[tuple[str, dict[str, Any]]]]:
    hard = builder.build(q, nu); t = cfg["task"]; scales = t["scales"]; d = np.diag(builder.scale)
    h = np.eye(NVAR) * float(t["scaled_regularization"]); g = np.zeros(NVAR); specs = []
    def put(name: str, a_physical: np.ndarray, target: np.ndarray, scale: np.ndarray, weight: float) -> None:
        if overrides and name in overrides: target = overrides[name]
        specs.append((name, add(h, g, a_physical @ d, target, scale, weight)))
    contact = np.zeros((6, NVAR))
    for side in range(2): contact[3*side:3*side+3, :12] = A_matrix(builder.oracle, builder.patch, q, side)
    put("contact", contact, -np.r_[bias_contact(builder, q, nu, 0), bias_contact(builder, q, nu, 1)], np.full(6, scales["contact_linear_m_s2"]), t["weights"]["contact"])
    for name, cols, scale in (("base_x", [0], scales["base_linear_m_s2"]), ("height", [2], scales["base_linear_m_s2"]), ("orientation", [3,4,5], scales["base_angular_rad_s2"])):
        a = np.zeros((len(cols), NVAR)); a[np.arange(len(cols)), cols] = 1.; put(name, a, np.zeros(len(cols)), np.full(len(cols), scale), t["weights"][name])
    indices = np.asarray(t["leg_canonical_indices"]); leg = np.zeros((4, NVAR)); leg[np.arange(4), 6 + indices] = 1.
    put("leg", leg, np.zeros(4), np.full(4, scales["leg_rad_s2"]), t["weights"]["leg"])
    wf = np.zeros((12, NVAR))
    for side, start in enumerate((18, 24)):
        wf[6*side:6*side+6, start:start+6] = wrench_flu_map(builder, q, side)
    wf[:, 30:42] = -np.eye(12)
    put("wrench_fidelity", wf, reference, np.tile(scales["wrench_per_side_flu"], 2), t["weights"]["wrench_fidelity"])
    # Explicit independent slack cost: scaled task variables are physical, as are config scales.
    slack = np.zeros((12, NVAR)); slack[:, 30:42] = np.eye(12)
    put("slack_penalty", slack, np.zeros(12), np.tile(scales["wrench_per_side_flu"], 2), float(t["slack_penalty"]))
    return {**hard, "H": h, "g": g}, specs

def result(builder: HardQpBuilder, cfg: dict[str, Any], q: np.ndarray, nu: np.ndarray, reference: np.ndarray, overrides: dict[str, np.ndarray] | None = None) -> tuple[dict[str, Any], list[tuple[str, dict[str, Any]]], dict[str, Any]]:
    p, specs = task_problem(builder, cfg, q, nu, reference, overrides); audit = independent_oracle(p["H"], p["g"], p["A"], p["l"], p["u"], cfg["oracle"])
    if not audit.get("qp_success"): return audit, specs, p
    z = builder.transform @ audit["x"]; tasks = {}
    for name, spec in specs:
        residual = spec["A"] @ audit["x"] - spec["target"]
        normal = residual / spec["scale"]
        tasks[name] = {"normalized_residual": normal.tolist(), "normalized_cost": float(spec["weight"] * normal @ normal)}
    audit["physical_z"] = z.tolist(); audit["tasks"] = tasks; audit["hard_violation"] = float(max(0., np.max(p["l"]-p["A"]@audit["x"]), np.max(p["A"]@audit["x"]-p["u"])))
    return audit, specs, p

def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("--config", type=Path, required=True); ap.add_argument("--output-dir", type=Path, required=True); args = ap.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {output}")
    cfg, cfg_inputs = load_config(args.config.resolve()); hard_cfg, hard_inputs = load_config(ROOT / cfg["source_hard_profile"])
    model, model_inputs = load_config(ROOT / hard_cfg["model_profile"]); contact, contact_inputs = load_config(ROOT / hard_cfg["contact_profile"])
    _, continuous_inputs = load_config(ROOT / contact["continuous_contact_config"])
    eqpath = ROOT / model["equilibrium"]; builder = HardQpBuilder(hard_cfg, model, contact, json.loads(eqpath.read_text()))
    capture_path = ROOT / hard_cfg["dynamic_capture"]; capture = np.load(capture_path)
    qeq = builder.oracle.sample_qpos(model["samples"][0]); zero = np.zeros(12); base = builder.build(qeq, zero)
    static_a = np.vstack((base["A"], np.eye(12, NVAR))); static_l = np.r_[base["l"], np.zeros(12)]; static_u = np.r_[base["u"], np.zeros(12)]
    static = independent_oracle(base["H"], base["g"], static_a, static_l, static_u, cfg["oracle"])
    if not static.get("qp_success"): raise RuntimeError("Frozen zero-nudot static reference solve failed")
    zstatic = builder.transform @ static["x"]; reference = np.r_[wrench_flu(builder, qeq, 0, zstatic[18:24]), wrench_flu(builder, qeq, 1, zstatic[24:30])]
    cases = []
    for name, q, nu in corpus(builder, capture):
        audit, _, _ = result(builder, cfg, q, nu, reference); cases.append({"id": name, "audit": audit})
    equilibrium, _, ep = result(builder, cfg, qeq, zero, reference)
    direction = float(cfg["ablations"]["normalized_direction"]); ablations = []
    template, specs = task_problem(builder, cfg, qeq, zero, reference)
    nominal_output = {name: np.asarray(equilibrium["tasks"][name]["normalized_residual"]) + spec["target"] / spec["scale"] for name, spec in specs}
    for name, spec in specs:
        if name == "slack_penalty": continue
        for index in range(len(spec["target"])):
            for sign in (-1., 1.):
                target = spec["target"].copy(); target[index] += sign * direction * spec["scale"][index]
                audit, _, _ = result(builder, cfg, qeq, zero, reference, {name: target})
                achieved = np.asarray(audit["tasks"][name]["normalized_residual"]) + target / spec["scale"]
                ablations.append({"task": name, "component": index, "sign": sign, "achieved_normalized_direction": float(sign * (achieved[index] - nominal_output[name][index])), "task_costs": {k: v["normalized_cost"] for k,v in audit["tasks"].items()}})
    # Named transform/sign cases are kept independent of the QP objective assembly.
    named_wc = np.array([3., -2., 5., .7, -.4, .2]); geom = builder.patch.geometry(qeq, 0); pb, rnb = base_rotation(builder, qeq)
    rc = np.column_stack((geom["rolling"], geom["lateral"], builder.patch.n)); fn, mn = rc @ named_wc[:3], rc @ named_wc[3:]
    manual = np.r_[rnb.T @ fn, rnb.T @ (mn + np.cross(geom["contact_center"] - pb, fn))]
    feasible = np.array([2., -3., 4., .1, -.2, .3]); reference_case = np.array([.5, -1., 2., -.1, .2, -.3]); slack_case = feasible - reference_case
    transform_cases = {"named_contact_wrench_C": named_wc.tolist(), "transform_shift_error": mx(wrench_flu(builder, qeq, 0, named_wc) - manual), "named_slack_residual": mx(feasible - reference_case - slack_case), "slack_sign": "W_feasible-W_reference-slack=0"}
    thresholds = cfg["thresholds"]
    hard_audit = base["A"][:, 30:42]
    hessian = ep["H"]
    gates = {"dimensions_and_104_hard_rows": NVAR == 42 and base["A"].shape == (104,42), "slack_absent_from_hard": mx(hard_audit) == 0., "hessian_finite_symmetric_pd": bool(np.all(np.isfinite(hessian)) and mx(hessian-hessian.T) <= thresholds["hessian_symmetry"] and np.min(np.linalg.eigvalsh(hessian)) >= thresholds["hessian_minimum_eigenvalue"]), "corpus_hard_feasible": len(cases) == 32 and all(c["audit"].get("qp_success") and c["audit"].get("hard_violation", np.inf) <= thresholds["hard_violation"] for c in cases), "equilibrium": equilibrium.get("qp_success") and mx(np.asarray(equilibrium["physical_z"][:12])) <= thresholds["equilibrium_nudot"] and mx(np.asarray(equilibrium["tasks"]["wrench_fidelity"]["normalized_residual"])) <= thresholds["equilibrium_wrench_residual"] and mx(np.asarray(equilibrium["physical_z"][30:42])) <= thresholds["equilibrium_slack"], "directional_ablations": len(ablations) == 54 and all(a["achieved_normalized_direction"] >= thresholds["directional_response"] for a in ablations), "wrench_transform_and_slack_semantics": transform_cases["transform_shift_error"] <= thresholds["transform"] and transform_cases["named_slack_residual"] <= thresholds["transform"]}
    summary = {"schema_version": 1, "phase":21, "profile":cfg["profile"], "scope":"P21-T06 local algebraic oracle only; no nonlinear, controller, solver or production claim.", "variable_order":["nudot_12","tau_6","wL_C_6","wR_C_6","slackL_FLU_6","slackR_FLU_6"], "hard_rows":builder.row_map, "task_order":cfg["task"]["order"], "equilibrium_reference_generation":cfg["equilibrium_reference_generation"], "reference_wrench_flu":reference.tolist(), "static_reference":{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in static.items() if k != "x"}, "corpus":cases, "equilibrium":equilibrium, "ablations":ablations, "transform_cases":transform_cases, "gates":gates, "pass":all(gates.values()), "limitations":"Exploratory equal-weight local task oracle; it neither tunes weights nor demonstrates nonlinear stability or authorizes Core integration."}
    output.mkdir(parents=True, exist_ok=True); dump(output / "summary.json", summary)
    script = Path(__file__).resolve(); inputs = cfg_inputs + hard_inputs + model_inputs + contact_inputs + continuous_inputs
    sources = [script, ROOT / "tools/experiments/validate_weighted_wbc_hard_qp_42d.py", ROOT / "tools/experiments/validate_weighted_wbc_continuous_pfaffian.py", ROOT / "tools/experiments/validate_mujoco_weighted_wbc_model.py"]
    dump(output / "manifest.json", {"schema_version":1,"created_at":datetime.now(timezone.utc).isoformat(),"interpreter":sys.executable,"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy.__version__,"mujoco":mujoco.__version__,"inputs":{str(p.relative_to(ROOT)):sha(p) for p in inputs},"equilibrium":sha(eqpath),"capture":sha(capture_path),"sources":{str(p.relative_to(ROOT)):sha(p) for p in sources},"outputs":{n:sha(output/n) for n in ("summary.json",)}})
    print(json.dumps({"cases":len(cases),"ablations":len(ablations),"gates":gates,"pass":summary["pass"]}, indent=2)); return 0 if summary["pass"] else 1
if __name__ == "__main__":
    try: sys.exit(main())
    except Exception as err: print(f"ERROR: {err}", file=sys.stderr); sys.exit(2)
