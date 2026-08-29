#!/usr/bin/env python3
"""Generate the append-only Phase 29 converged-SQP diagnostic artifact."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import casadi as ca
from acados_template import AcadosOcpSolver

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.experiments.generate_phase27_acados_solver import create_ocp  # noqa: E402

DEFAULT_METHOD = ROOT / "simulation/mujoco/config/phase29_nmpc_root_cause_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", type=Path, default=DEFAULT_METHOD)
    args = parser.parse_args()
    method = json.loads(args.method.read_text(encoding="utf-8"))
    source_path = ROOT / method["source_ocp_config"]
    output = (ROOT / method["offline_sqp_generated_dir"]).resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output must be absent or empty: {output}")

    config = json.loads(source_path.read_text(encoding="utf-8"))
    oracle = method["offline_sqp"]
    config["profile"] = method["profile"] + "_offline_sqp"
    config["model_name"] = oracle["model_name"]
    config["solver"]["nlp_solver_type"] = "SQP"
    ocp = create_ocp(config, output)
    ocp.solver_options.nlp_solver_max_iter = int(oracle["nlp_solver_max_iter"])
    ocp.solver_options.nlp_solver_tol_stat = float(oracle["tol_stat"])
    ocp.solver_options.nlp_solver_tol_eq = float(oracle["tol_eq"])
    ocp.solver_options.nlp_solver_tol_ineq = float(oracle["tol_ineq"])
    ocp.solver_options.nlp_solver_tol_comp = float(oracle["tol_comp"])
    ocp.solver_options.globalization = oracle["globalization"]

    generation = config["generation"]
    acados_source = Path(os.environ.get("ACADOS_SOURCE_DIR", "")).resolve()
    if str(acados_source) != generation["acados_source_dir"]:
        raise RuntimeError(f"ACADOS_SOURCE_DIR mismatch: {acados_source}")
    commit = subprocess.check_output(
        ["git", "-C", str(acados_source), "rev-parse", "HEAD"], text=True
    ).strip()
    if commit != generation["acados_commit"]:
        raise RuntimeError(f"acados commit mismatch: {commit}")
    tera = Path(os.environ.get("TERA_PATH", "")).resolve()
    if not tera.is_file() or sha256(tera) != generation["tera_sha256"]:
        raise RuntimeError("TERA_PATH missing or hash mismatch")

    output.mkdir(parents=True, exist_ok=True)
    AcadosOcpSolver.generate(ocp)
    files = sorted(path for path in output.rglob("*") if path.is_file())
    manifest = {
        "schema_version": 1,
        "profile": config["profile"],
        "purpose": "offline-only converged SQP oracle; never linked by production",
        "method": str(args.method.resolve().relative_to(ROOT)),
        "method_sha256": sha256(args.method),
        "source_config": str(source_path.relative_to(ROOT)),
        "source_config_sha256": sha256(source_path),
        "allowed_differences": {
            "model_name": oracle["model_name"],
            "nlp_solver_type": "SQP",
            "nlp_solver_max_iter": oracle["nlp_solver_max_iter"],
            "tol_stat": oracle["tol_stat"],
            "tol_eq": oracle["tol_eq"],
            "tol_ineq": oracle["tol_ineq"],
            "tol_comp": oracle["tol_comp"],
            "globalization": oracle["globalization"],
        },
        "generator": str(Path(__file__).resolve().relative_to(ROOT)),
        "generator_sha256": sha256(Path(__file__).resolve()),
        "base_generator": "tools/experiments/generate_phase27_acados_solver.py",
        "base_generator_sha256": sha256(
            ROOT / "tools/experiments/generate_phase27_acados_solver.py"
        ),
        "acados_source_dir": str(acados_source),
        "acados_commit": commit,
        "casadi_version": ca.__version__,
        "tera_sha256": sha256(tera),
        "generated_files": [str(path.relative_to(output)) for path in files],
        "generated_sha256": {
            str(path.relative_to(output)): sha256(path) for path in files
        },
    }
    (output / "phase29_generation_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
