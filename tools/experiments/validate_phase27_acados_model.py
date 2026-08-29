#!/usr/bin/env python3
"""Cross-check the generated Phase 27 discrete model against its oracle."""

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path

import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation

from generate_phase27_acados_solver import create_ocp

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase27_acados_ocp_v1.json"
DEFAULT_GOLDEN = ROOT / "docs/workflow/phases/27-theory-restored-minimal-wbc/evidence/automated/wheel-aware-model-oracle-v2/golden.txt"


def take(tokens: list[str], count: int) -> np.ndarray:
    return np.asarray([float(tokens.pop(0)) for _ in range(count)])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--golden", type=Path, default=DEFAULT_GOLDEN)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    tokens = args.golden.read_text(encoding="utf-8").split()
    if tokens.pop(0) != "PHASE27_WHEEL_AWARE_MODEL_GOLDEN_V1":
        raise ValueError("unexpected golden schema")
    sample_count = int(tokens.pop(0))
    with tempfile.TemporaryDirectory(prefix="phase27-acados-model-") as directory:
        model = create_ocp(config, Path(directory)).model
        function = ca.Function(
            "phase27_disc", [model.x, model.u, model.p],
            [model.disc_dyn_expr, ca.jacobian(model.disc_dyn_expr, model.x),
             ca.jacobian(model.disc_dyn_expr, model.u)],
        )
        maximum_next = maximum_state_jacobian = maximum_input_jacobian = 0.0
        for _ in range(sample_count):
            tokens.pop(0)
            reference_vector = take(tokens, 3)
            state = take(tokens, 16)
            control = take(tokens, 12)
            take(tokens, 16)
            expected_next = take(tokens, 16)
            take(tokens, 16 * 16)
            take(tokens, 16 * 12)
            expected_a = take(tokens, 16 * 16).reshape(16, 16)
            expected_b = take(tokens, 16 * 12).reshape(16, 12)
            reference = Rotation.from_rotvec(reference_vector).as_matrix()
            actual_next, actual_a, actual_b = function(state, control, reference.reshape(-1))
            maximum_next = max(maximum_next, float(np.max(np.abs(np.asarray(actual_next).ravel() - expected_next))))
            maximum_state_jacobian = max(maximum_state_jacobian, float(np.max(np.abs(np.asarray(actual_a) - expected_a))))
            maximum_input_jacobian = max(maximum_input_jacobian, float(np.max(np.abs(np.asarray(actual_b) - expected_b))))
    if tokens:
        raise ValueError("unconsumed golden tokens")
    result = {
        "samples": sample_count,
        "maximum_next_error": maximum_next,
        "maximum_state_jacobian_error": maximum_state_jacobian,
        "maximum_input_jacobian_error": maximum_input_jacobian,
        "pass": maximum_next <= 2.0e-8 and maximum_state_jacobian <= 1.0e-5 and maximum_input_jacobian <= 1.0e-5,
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
