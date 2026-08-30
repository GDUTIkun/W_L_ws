#!/usr/bin/env python3
"""Generate the append-only Phase 34 12-state base NMPC artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase34_base_acados_ocp_v1.json"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def skew(vector: ca.SX) -> ca.SX:
    return ca.vertcat(
        ca.horzcat(0, -vector[2], vector[1]),
        ca.horzcat(vector[2], 0, -vector[0]),
        ca.horzcat(-vector[1], vector[0], 0),
    )


def rotation_matrix(vector: ca.SX) -> ca.SX:
    squared = ca.dot(vector, vector)
    angle = ca.sqrt(squared + 1.0e-24)
    hat = skew(vector)
    sine = ca.if_else(
        squared < 1.0e-12,
        1 - squared / 6 + squared * squared / 120,
        ca.sin(angle) / angle,
    )
    cosine = ca.if_else(
        squared < 1.0e-12,
        0.5 - squared / 24 + squared * squared / 720,
        (1 - ca.cos(angle)) / squared,
    )
    return ca.SX.eye(3) + sine * hat + cosine * ca.mtimes(hat, hat)


def left_jacobian_inverse(vector: ca.SX) -> ca.SX:
    squared = ca.dot(vector, vector)
    angle = ca.sqrt(squared + 1.0e-24)
    hat = skew(vector)
    coefficient = ca.if_else(
        squared < 1.0e-12,
        1.0 / 12.0 + squared / 720,
        1 / squared - (1 + ca.cos(angle)) / (2 * angle * ca.sin(angle)),
    )
    return ca.SX.eye(3) - 0.5 * hat + coefficient * ca.mtimes(hat, hat)


def discrete_expression(config: dict) -> tuple[ca.SX, ca.SX, ca.SX, ca.SX]:
    x = ca.SX.sym("x", 12)
    u = ca.SX.sym("u", 12)
    parameter = ca.SX.sym("p", 11)
    reference_rotation = ca.reshape(parameter[:9], 3, 3).T
    com_b = ca.DM(config["body_com_from_base_b_m"])
    inertia_b = ca.DM(config["body_inertia_com_b_kg_m2"])

    def flow(state: ca.SX) -> ca.SX:
        rotation_vector = state[3:6]
        angular_velocity = state[9:12]
        rotation = ca.mtimes(rotation_matrix(rotation_vector), reference_rotation)
        left_origin = ca.vertcat(parameter[9], *config["left_wheel_origin_yz_b_m"])
        right_origin = ca.vertcat(parameter[10], *config["right_wheel_origin_yz_b_m"])
        force_b = u[:3] + u[6:9]
        moment_b = (
            u[3:6]
            + ca.cross(left_origin, u[:3])
            + u[9:12]
            + ca.cross(right_origin, u[6:9])
        )
        force_n = ca.mtimes(rotation, force_b)
        com_n = ca.mtimes(rotation, com_b)
        inertia_n = ca.mtimes([rotation, inertia_b, rotation.T])
        angular = ca.solve(
            inertia_n,
            ca.mtimes(rotation, moment_b)
            - ca.cross(com_n, force_n)
            - ca.cross(angular_velocity, ca.mtimes(inertia_n, angular_velocity)),
        )
        com_acceleration = force_n / float(config["body_mass_kg"])
        com_acceleration[2] -= float(config["gravity_m_s2"])
        linear = (
            com_acceleration
            - ca.cross(angular, com_n)
            - ca.cross(angular_velocity, ca.cross(angular_velocity, com_n))
        )
        return ca.vertcat(
            state[6:9],
            ca.mtimes(left_jacobian_inverse(rotation_vector), angular_velocity),
            linear,
            angular,
        )

    step = float(config["sampling_period_s"]) / int(config["integration_substeps"])
    discrete = x
    for _ in range(int(config["integration_substeps"])):
        k1 = flow(discrete)
        k2 = flow(discrete + 0.5 * step * k1)
        k3 = flow(discrete + 0.5 * step * k2)
        k4 = flow(discrete + step * k3)
        discrete += step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return x, u, parameter, discrete


def create_ocp(config: dict, output: Path, *, sqp: bool) -> AcadosOcp:
    nx, nu = 12, 12
    x, u, parameter, discrete = discrete_expression(config)
    model = AcadosModel()
    model.name = config["model_name"] + ("_sqp" if sqp else "")
    model.x, model.u, model.p, model.disc_dyn_expr = x, u, parameter, discrete

    q = np.asarray(config["state_weight"]) / np.square(config["state_error_scale"])
    r = np.asarray(config["input_weight"]) / np.square(config["input_error_scale"])
    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = int(config["horizon_steps"])
    ocp.solver_options.tf = float(config["horizon_s"])
    ocp.parameter_values = np.r_[np.eye(3).reshape(-1), config["equilibrium_wheel_position_m"]]
    ocp.cost.cost_type = ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = np.diag(np.r_[q, r])
    ocp.cost.W_e = np.diag(float(config["terminal_weight_multiplier"]) * q)
    ocp.cost.Vx = np.zeros((nx + nu, nx))
    ocp.cost.Vx[:nx] = np.eye(nx)
    ocp.cost.Vu = np.zeros((nx + nu, nu))
    ocp.cost.Vu[nx:] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    equilibrium_state = np.asarray(config["equilibrium_state"])
    equilibrium_input = np.asarray(config["equilibrium_input"])
    ocp.cost.yref = np.r_[equilibrium_state, equilibrium_input]
    ocp.cost.yref_e = equilibrium_state
    ocp.constraints.x0 = equilibrium_state
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.lbu = np.asarray(config["input_lower"])
    ocp.constraints.ubu = np.asarray(config["input_upper"])
    envelope = np.asarray(config["state_envelope_half_width"])
    ocp.constraints.idxbx = np.arange(nx)
    ocp.constraints.lbx = equilibrium_state - envelope
    ocp.constraints.ubx = equilibrium_state + envelope
    ocp.constraints.idxbx_e = np.arange(nx)
    ocp.constraints.lbx_e = equilibrium_state - envelope
    ocp.constraints.ubx_e = equilibrium_state + envelope

    solver = config["solver"]
    ocp.solver_options.nlp_solver_type = "SQP" if sqp else solver["nlp_solver_type"]
    ocp.solver_options.qp_solver = solver["qp_solver"]
    ocp.solver_options.qp_solver_cond_N = int(solver["qp_solver_cond_N"])
    ocp.solver_options.hessian_approx = solver["hessian_approx"]
    ocp.solver_options.integrator_type = solver["integrator_type"]
    ocp.solver_options.regularize_method = solver["regularize_method"]
    ocp.solver_options.qp_solver_iter_max = int(solver["qp_solver_iter_max"])
    ocp.solver_options.qp_tol = float(solver["qp_tol"])
    ocp.solver_options.nlp_solver_warm_start_first_qp = bool(solver["warm_start_first_qp"])
    ocp.solver_options.print_level = int(solver["print_level"])
    if sqp:
        ocp.solver_options.nlp_solver_max_iter = 100
        ocp.solver_options.nlp_solver_tol_stat = 1.0e-9
        ocp.solver_options.nlp_solver_tol_eq = 1.0e-9
        ocp.solver_options.nlp_solver_tol_ineq = 1.0e-9
        ocp.solver_options.nlp_solver_tol_comp = 1.0e-9
        ocp.solver_options.globalization = "MERIT_BACKTRACKING"
    ocp.code_gen_options.code_export_directory = str(output)
    ocp.code_gen_options.json_file = str(output / "acados_ocp.json")
    return ocp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sqp", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output must be absent or empty: {output}")
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
    AcadosOcpSolver.generate(create_ocp(config, output, sqp=args.sqp))
    files = sorted(path for path in output.rglob("*") if path.is_file())
    manifest = {
        "schema_version": 1,
        "profile": config["profile"] + ("_sqp" if args.sqp else "_rti"),
        "purpose": "Phase34 opt-in diagnostic; production Phase27 remains unchanged",
        "generator": str(Path(__file__).resolve().relative_to(ROOT)),
        "generator_sha256": sha256(Path(__file__).resolve()),
        "config": str(args.config.resolve().relative_to(ROOT)),
        "config_sha256": sha256(args.config),
        "solver_difference": "SQP convergence options only" if args.sqp else "none",
        "acados_commit": commit,
        "casadi_version": ca.__version__,
        "tera_sha256": sha256(tera),
        "generated_files": [str(path.relative_to(output)) for path in files],
        "generated_sha256": {
            str(path.relative_to(output)): sha256(path) for path in files
        },
    }
    (output / "phase34_generation_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
