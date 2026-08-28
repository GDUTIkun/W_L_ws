#!/usr/bin/env python3
"""Export the frozen workspace-aware 42D task problems for C++ parity tests."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_hard_qp_42d import HardQpBuilder, corpus, independent_oracle  # noqa: E402
from validate_weighted_wbc_tasks_42d import task_problem, wrench_flu  # noqa: E402


def line(array: np.ndarray) -> str:
    return " ".join(format(float(value), ".17g") for value in array.ravel())


def robot_state(builder: HardQpBuilder, model: dict, q: np.ndarray,
                nu: np.ndarray) -> np.ndarray:
    builder.oracle.forward(q, builder.oracle.reduction(q)[0] @ nu)
    site = builder.oracle.base_control_site
    rotation = builder.oracle.data.site_xmat[site].reshape(3, 3)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, rotation.ravel())
    return np.r_[builder.oracle.data.site_xpos[site], quat, nu[:3], nu[3:6],
                 np.asarray(model["canonical_joint_offsets_rad"]) -
                 q[builder.oracle.active_qpos], nu[6:]]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite golden corpus: {output}")
    cfg, _ = load_config(ROOT / "simulation/mujoco/config/phase21_task_prefreeze_42d_runtime_v2.json")
    hard, _ = load_config(ROOT / cfg["source_hard_profile"])
    model, _ = load_config(ROOT / hard["model_profile"])
    contact, _ = load_config(ROOT / hard["contact_profile"])
    equilibrium = json.loads((ROOT / model["equilibrium"]).read_text())
    builder = HardQpBuilder(hard, model, contact, equilibrium)
    capture = np.load(ROOT / hard["dynamic_capture"])
    qeq = builder.oracle.sample_qpos(model["samples"][0])
    base = builder.build(qeq, np.zeros(12))
    static_a = np.vstack((base["A"], np.eye(12, 42)))
    static = independent_oracle(base["H"], base["g"], static_a,
                                np.r_[base["l"], np.zeros(12)],
                                np.r_[base["u"], np.zeros(12)], cfg["oracle"])
    if not static.get("qp_success"):
        raise RuntimeError("zero-acceleration static reference solve failed")
    z = builder.transform @ static["x"]
    wrench = np.r_[wrench_flu(builder, qeq, 0, z[18:24]),
                   wrench_flu(builder, qeq, 1, z[24:30])]
    reference = np.r_[np.zeros(9), wrench]
    cases = corpus(builder, capture)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        stream.write(f"WBC_PROBLEM_GOLDEN_V1 {len(cases)}\n")
        for case_id, q, nu in cases:
            problem, _ = task_problem(builder, cfg, q, nu, wrench)
            stream.write(case_id + "\n")
            stream.write(line(robot_state(builder, model, q, nu)) + "\n")
            stream.write(line(reference) + "\n")
            stream.write(line(np.r_[problem["H"].ravel(), problem["g"],
                                      problem["A"].ravel(), problem["l"],
                                      problem["u"]]) + "\n")
    print(f"wrote {output} ({len(cases)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
