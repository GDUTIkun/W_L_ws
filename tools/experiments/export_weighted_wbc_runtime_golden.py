#!/usr/bin/env python3
"""Export fresh MuJoCo/Python golden cases for the Phase-21 C++ model test."""

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
from validate_weighted_wbc_hard_qp_42d import HardQpBuilder  # noqa: E402
from validate_weighted_wbc_continuous_pfaffian import A_matrix  # noqa: E402
from validate_weighted_wbc_tasks_42d import bias_contact  # noqa: E402


def values(array: np.ndarray) -> str:
    return " ".join(format(float(value), ".17g") for value in array.ravel())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"refusing to overwrite golden corpus: {output}")
    hard, _ = load_config(ROOT / "simulation/mujoco/config/phase21_hard_qp_42d_runtime_v2.json")
    model, _ = load_config(ROOT / hard["model_profile"])
    contact, _ = load_config(ROOT / hard["contact_profile"])
    equilibrium = json.loads((ROOT / model["equilibrium"]).read_text())
    builder = HardQpBuilder(hard, model, contact, equilibrium)
    capture = np.load(ROOT / hard["dynamic_capture"])
    cases: list[tuple[str, np.ndarray, np.ndarray]] = [
        ("workspace_" + sample["id"], builder.oracle.sample_qpos(sample), np.zeros(12))
        for sample in model["samples"]
    ]
    for tick in (68, 204, 259, 271):
        q = builder.canonical_qpos(capture["qpos"][tick])
        cases.append((f"dynamic_tick_{tick}", q,
                      builder.reduced_velocity(q, capture["qvel"][tick])))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        stream.write(f"WBC_MODEL_GOLDEN_V1 {len(cases)}\n")
        for case_id, q, nu in cases:
            builder.oracle.forward(q, builder.oracle.reduction(q)[0] @ nu)
            site = builder.oracle.base_control_site
            rotation = builder.oracle.data.site_xmat[site].reshape(3, 3)
            quat = np.zeros(4)
            mujoco.mju_mat2Quat(quat, rotation.ravel())
            state = np.r_[builder.oracle.data.site_xpos[site], quat,
                          nu[:3], nu[3:6],
                          np.asarray(model["canonical_joint_offsets_rad"]) -
                          q[builder.oracle.active_qpos], nu[6:]]
            mass, bias, actuation, wrench = builder.dynamics(q, nu)
            reduction, _ = builder.oracle.reduction(q)
            reduction[:6] = np.eye(6, 12)
            arrays = [q[7:17], reduction, mass, bias, actuation,
                      wrench[0], wrench[1],
                      A_matrix(builder.oracle, builder.patch, q, 0),
                      A_matrix(builder.oracle, builder.patch, q, 1),
                      bias_contact(builder, q, nu, 0),
                      bias_contact(builder, q, nu, 1)]
            stream.write(case_id + "\n")
            stream.write(values(state) + "\n")
            stream.write(values(np.concatenate([a.ravel() for a in arrays])) + "\n")
    print(f"wrote {output} ({len(cases)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
