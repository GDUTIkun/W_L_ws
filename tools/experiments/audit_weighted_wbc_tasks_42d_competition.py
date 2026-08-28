#!/usr/bin/env python3
"""Independent P21-T06 42D local task competition/accounting audit."""
from __future__ import annotations

import argparse, copy, hashlib, json, platform, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import scipy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_hard_qp_42d import HardQpBuilder, corpus  # noqa: E402
from validate_weighted_wbc_tasks_42d import NVAR, result, task_problem, wrench_flu  # noqa: E402
from validate_weighted_wbc_hard_qp_42d import independent_oracle  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
TASKS = ("contact", "base_x", "height", "orientation", "leg", "wrench_fidelity")
def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()
def dump(path: Path, value: Any) -> None:
    def native(x: Any) -> Any:
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False, default=native) + "\n")
def own_sse(audit: dict[str, Any], name: str) -> float:
    r = np.asarray(audit["tasks"][name]["normalized_residual"]); return float(r @ r)

def main() -> int:
    ap = argparse.ArgumentParser(); ap.add_argument("--config", type=Path, default=ROOT / "simulation/mujoco/config/phase21_task_prefreeze_42d.json"); ap.add_argument("--output-dir", type=Path, required=True); args = ap.parse_args(); out=args.output_dir.resolve()
    if out.exists() and any(out.iterdir()): raise RuntimeError(f"Refusing non-empty output directory: {out}")
    cfg, cfg_inputs=load_config(args.config.resolve()); hard, hard_inputs=load_config(ROOT/cfg["source_hard_profile"]); model, model_inputs=load_config(ROOT/hard["model_profile"]); contact, contact_inputs=load_config(ROOT/hard["contact_profile"]); _, continuous_inputs=load_config(ROOT/contact["continuous_contact_config"])
    eqpath=ROOT/model["equilibrium"]; builder=HardQpBuilder(hard, model, contact, json.loads(eqpath.read_text())); capture_path=ROOT/hard["dynamic_capture"]; capture=np.load(capture_path)
    qeq=builder.oracle.sample_qpos(model["samples"][0]); base=builder.build(qeq,np.zeros(12)); static=independent_oracle(base["H"],base["g"],np.vstack((base["A"],np.eye(12,NVAR))),np.r_[base["l"],np.zeros(12)],np.r_[base["u"],np.zeros(12)],cfg["oracle"])
    if not static.get("qp_success"): raise RuntimeError("Frozen static reference solve failed")
    z=builder.transform@static["x"]; reference=np.r_[wrench_flu(builder,qeq,0,z[18:24]),wrench_flu(builder,qeq,1,z[24:30])]
    variants={"baseline":cfg}
    for name in TASKS:
        variant=copy.deepcopy(cfg); variant["task"]["weights"][name]=0.; variants[f"disabled_{name}"]=variant
    records=[]; aggregate={key:{name:0. for name in TASKS} for key in variants}; max_h=max_g=max_objective=max_slack_matrix=max_slack_target=0.; all_valid=True
    for case_id,q,nu in corpus(builder,capture):
        for variant_id,variant in variants.items():
            audit,specs,problem=result(builder,variant,q,nu,reference); valid=bool(audit.get("qp_success") and np.all(np.isfinite(audit.get("x",[]))) and audit.get("hard_violation",np.inf)<=2e-7); all_valid &= valid
            row={"case":case_id,"variant":variant_id,"valid":valid,"hard_violation":audit.get("hard_violation",np.inf),"own_sse":{name:own_sse(audit,name) if valid else float("inf") for name in TASKS}}
            if variant_id=="baseline" and valid:
                h=np.eye(NVAR)*float(cfg["task"]["scaled_regularization"]); g=np.zeros(NVAR)
                for name,spec in specs:
                    an=spec["A"]/spec["scale"][:,None]; tn=spec["target"]/spec["scale"]; h+=spec["weight"]*an.T@an; g-=spec["weight"]*an.T@tn
                x=np.asarray(audit["x"]); stated=.5*float(cfg["task"]["scaled_regularization"])*float(x@x)
                for _,spec in specs:
                    residual=(spec["A"]@x-spec["target"])/spec["scale"]; target=spec["target"]/spec["scale"]; stated+=.5*spec["weight"]*(float(residual@residual)-float(target@target))
                error=abs(float(audit["objective"])-stated); max_h=max(max_h,float(np.max(np.abs(h-problem["H"])))); max_g=max(max_g,float(np.max(np.abs(g-problem["g"])))); max_objective=max(max_objective,error)
                row["objective_accounting"]={"H_error":float(np.max(np.abs(h-problem["H"]))),"g_error":float(np.max(np.abs(g-problem["g"]))),"objective_error":error,"terms":[name for name,_ in specs]}
                fidelity=dict(specs)["wrench_fidelity"]; physical_block=fidelity["A"][:,30:42] / builder.scale[30:42][None,:]
                slack_matrix_error=float(np.max(np.abs(physical_block + np.eye(12)))); slack_target_error=float(np.max(np.abs(fidelity["target"]-reference))); max_slack_matrix=max(max_slack_matrix,slack_matrix_error); max_slack_target=max(max_slack_target,slack_target_error)
                row["wrench_fidelity_slack_semantics"]={"physical_slack_matrix_error":slack_matrix_error,"target_reference_error":slack_target_error,"residual_example":float(np.max(np.abs(np.ones(12)-reference-(np.ones(12)-reference))))}
            for name,value in row["own_sse"].items(): aggregate[variant_id][name]+=value
            records.append(row)
    attribution={}
    for name in TASKS:
        baseline=aggregate["baseline"][name]; disabled=aggregate[f"disabled_{name}"][name]; attribution[name]={"baseline_own_sse":baseline,"disabled_own_sse":disabled,"absolute_improvement":disabled-baseline,"relative_improvement":(disabled-baseline)/max(1.,abs(disabled))}
    gates={"all_cases_variants_feasible_finite_hard":all_valid,"own_task_attribution":all(v["absolute_improvement"]>1e-10 for v in attribution.values()),"objective_assembly":max_h<=1e-12 and max_g<=1e-12,"objective_value":max_objective<=1e-9,"slack_absent_from_104_hard":bool(np.max(np.abs(base["A"][:,30:42]))==0.),"wrench_fidelity_slack_matrix_and_target":max_slack_matrix<=1e-12 and max_slack_target<=1e-12}
    summary={"schema_version":1,"phase":21,"profile":cfg["profile"],"scope":"32-case local weighted-objective competition/accounting audit only; no tuning/nonlinear/Core claim.","cases":32,"variants":list(variants),"reference_wrench_flu":reference.tolist(),"aggregate_own_sse":aggregate,"attribution":attribution,"maximum_H_assembly_error":max_h,"maximum_g_assembly_error":max_g,"maximum_objective_accounting_error":max_objective,"maximum_wrench_fidelity_physical_slack_matrix_error":max_slack_matrix,"maximum_wrench_fidelity_target_reference_error":max_slack_target,"gates":gates,"pass":all(gates.values())}
    out.mkdir(parents=True,exist_ok=True); dump(out/"summary.json",summary); dump(out/"details.json",records)
    script=Path(__file__).resolve(); inputs=cfg_inputs+hard_inputs+model_inputs+contact_inputs+continuous_inputs; sources=[script,ROOT/"tools/experiments/validate_weighted_wbc_tasks_42d.py",ROOT/"tools/experiments/validate_weighted_wbc_hard_qp_42d.py",ROOT/"tools/experiments/validate_mujoco_weighted_wbc_model.py"]
    dump(out/"manifest.json",{"schema_version":1,"created_at":datetime.now(timezone.utc).isoformat(),"command":[sys.executable,*sys.argv],"interpreter":sys.executable,"python":platform.python_version(),"numpy":np.__version__,"scipy":scipy.__version__,"mujoco":mujoco.__version__,"inputs":{str(p.relative_to(ROOT)):sha(p) for p in inputs},"equilibrium":sha(eqpath),"capture":sha(capture_path),"sources":{str(p.relative_to(ROOT)):sha(p) for p in sources},"outputs":{n:sha(out/n) for n in("summary.json","details.json")}})
    print(json.dumps({"gates":gates,"pass":summary["pass"]},indent=2)); return 0 if summary["pass"] else 1
if __name__=="__main__":
    try: sys.exit(main())
    except Exception as err: print(f"ERROR: {err}",file=sys.stderr); sys.exit(2)
