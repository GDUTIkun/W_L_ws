#!/usr/bin/env python3
"""Export frozen weighted-task QPs with independent solutions for C++ solver audit."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_mujoco_weighted_wbc_model import load_config  # noqa: E402
from validate_weighted_wbc_hard_qp_42d import HardQpBuilder, corpus, independent_oracle  # noqa: E402
from validate_weighted_wbc_tasks_42d import task_problem, wrench_flu  # noqa: E402


def values(array: np.ndarray) -> str:
    return " ".join(format(float(value), ".17g") for value in array.ravel())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rho", type=float, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite solver corpus: {output}")
    cfg, _ = load_config(ROOT / "simulation/mujoco/config/phase21_task_prefreeze_42d_runtime_v2.json")
    hard, _ = load_config(ROOT / cfg["source_hard_profile"])
    model, _ = load_config(ROOT / hard["model_profile"])
    contact, _ = load_config(ROOT / hard["contact_profile"])
    equilibrium = json.loads((ROOT / model["equilibrium"]).read_text())
    builder = HardQpBuilder(hard, model, contact, equilibrium)
    capture = np.load(ROOT / hard["dynamic_capture"])
    qeq = builder.oracle.sample_qpos(model["samples"][0])
    base = builder.build(qeq, np.zeros(12))
    static = independent_oracle(
        base["H"], base["g"], np.vstack((base["A"], np.eye(12, 42))),
        np.r_[base["l"], np.zeros(12)], np.r_[base["u"], np.zeros(12)],
        cfg["oracle"])
    if not static.get("qp_success"):
        raise RuntimeError("zero-acceleration static reference solve failed")
    z = builder.transform @ static["x"]
    wrench = np.r_[wrench_flu(builder, qeq, 0, z[18:24]),
                   wrench_flu(builder, qeq, 1, z[24:30])]
    cases = []
    for case_id, q, nu in corpus(builder, capture):
        problem, _ = task_problem(builder, cfg, q, nu, wrench)
        audit = independent_oracle(problem["H"], problem["g"], problem["A"],
                                   problem["l"], problem["u"], cfg["oracle"])
        if not audit.get("qp_success"):
            raise RuntimeError(f"independent task QP failed: {case_id}")
        cases.append((problem, audit["x"]))
    output.parent.mkdir(parents=True, exist_ok=True)
    settings = hard["solver"]
    with output.open("w", encoding="utf-8") as stream:
        stream.write(f"DENSE_QP_CORPUS_V1 {len(cases)}\n")
        for problem, oracle in cases:
            stream.write(
                f"42 104 12 {args.rho:.17g} {settings['sigma']:.17g} "
                f"{settings['absolute_tolerance']:.17g} "
                f"{settings['relative_tolerance']:.17g} "
                f"{int(settings['maximum_iterations'])}\n")
            stream.write(values(np.r_[problem["H"].ravel(), problem["g"],
                                       problem["A"].ravel(), problem["l"],
                                       problem["u"]]) + "\n")
            stream.write("oracle\n")
            stream.write(values(oracle) + "\n")
    print(f"wrote {output} ({len(cases)} cases, rho={args.rho:g})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
