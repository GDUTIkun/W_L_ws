#!/usr/bin/env python3
"""Generate the frozen Phase 27 wheel-aware acados OCP artifact."""

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
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase27_acados_ocp_v1.json"
DEFAULT_OUTPUT = ROOT / "ros_ws/src/wheel_leg_core/acados_generated/phase27_wheel_aware_nmpc_v1"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def skew(vector: ca.SX) -> ca.SX:
    return ca.vertcat(
        ca.horzcat(0, -vector[2], vector[1]),
        ca.horzcat(vector[2], 0, -vector[0]),
        ca.horzcat(-vector[1], vector[0], 0),
    )


def rotation_matrix(vector: ca.SX) -> ca.SX:
    squared_angle = ca.dot(vector, vector)
    angle = ca.sqrt(squared_angle + 1.0e-24)
    hat = skew(vector)
    sine_scale = ca.if_else(
        squared_angle < 1.0e-12,
        1 - squared_angle / 6 + squared_angle * squared_angle / 120,
        ca.sin(angle) / angle,
    )
    cosine_scale = ca.if_else(
        squared_angle < 1.0e-12,
        0.5 - squared_angle / 24 + squared_angle * squared_angle / 720,
        (1 - ca.cos(angle)) / squared_angle,
    )
    return ca.SX.eye(3) + sine_scale * hat + cosine_scale * ca.mtimes(hat, hat)


def left_jacobian_inverse(vector: ca.SX) -> ca.SX:
    squared_angle = ca.dot(vector, vector)
    angle = ca.sqrt(squared_angle + 1.0e-24)
    hat = skew(vector)
    coefficient = ca.if_else(
        squared_angle < 1.0e-12,
        1.0 / 12.0 + squared_angle / 720,
        1 / squared_angle - (1 + ca.cos(angle)) / (2 * angle * ca.sin(angle)),
    )
    return ca.SX.eye(3) - 0.5 * hat + coefficient * ca.mtimes(hat, hat)


def create_ocp(config: dict, output: Path) -> AcadosOcp:
    nx, nu = 16, 12
    x = ca.SX.sym("x", nx)
    u = ca.SX.sym("u", nu)
    parameter = ca.SX.sym("p", 9)
    reference_rotation = ca.reshape(parameter, 3, 3).T
    com_b = ca.DM(config["body_com_from_base_b_m"])
    inertia_b = ca.DM(config["body_inertia_com_b_kg_m2"])
    body_mass = float(config["body_mass_kg"])
    wheel_mass = float(config["wheel_mass_kg"])
    radius = float(config["wheel_radius_m"])
    wheel_inertia = float(config["wheel_axle_inertia_kg_m2"])
    wheel_denominator = wheel_mass * radius + wheel_inertia / radius
    gravity = float(config["gravity_m_s2"])

    def flow(state: ca.SX) -> ca.SX:
        rotation_vector = state[3:6]
        velocity = state[6:9]
        angular_velocity = state[9:12]
        rotation = ca.mtimes(rotation_matrix(rotation_vector), reference_rotation)
        left_force = u[0:3]
        left_torque = u[3:6]
        right_force = u[6:9]
        right_torque = u[9:12]
        left_origin = ca.vertcat(state[12], *config["left_wheel_origin_yz_b_m"])
        right_origin = ca.vertcat(state[13], *config["right_wheel_origin_yz_b_m"])
        force_b = left_force + right_force
        moment_b = left_torque + ca.cross(left_origin, left_force) + right_torque + ca.cross(right_origin, right_force)
        force_n = ca.mtimes(rotation, force_b)
        moment_n = ca.mtimes(rotation, moment_b)
        com_offset_n = ca.mtimes(rotation, com_b)
        inertia_n = ca.mtimes([rotation, inertia_b, rotation.T])
        angular_acceleration = ca.solve(
            inertia_n,
            moment_n - ca.cross(com_offset_n, force_n)
            - ca.cross(angular_velocity, ca.mtimes(inertia_n, angular_velocity)),
        )
        com_acceleration = force_n / body_mass - ca.vertcat(0, 0, gravity)
        base_acceleration = (
            com_acceleration
            - ca.cross(angular_acceleration, com_offset_n)
            - ca.cross(angular_velocity, ca.cross(angular_velocity, com_offset_n))
        )
        base_forward_acceleration = force_b[0] / body_mass
        left_wheel_acceleration = -base_forward_acceleration - (
            radius * left_force[0] + left_torque[1]
        ) / wheel_denominator
        right_wheel_acceleration = -base_forward_acceleration - (
            radius * right_force[0] + right_torque[1]
        ) / wheel_denominator
        return ca.vertcat(
            velocity,
            ca.mtimes(left_jacobian_inverse(rotation_vector), angular_velocity),
            base_acceleration,
            angular_acceleration,
            state[14], state[15],
            left_wheel_acceleration, right_wheel_acceleration,
        )

    substep = float(config["sampling_period_s"]) / int(config["integration_substeps"])
    discrete = x
    for _ in range(int(config["integration_substeps"])):
        k1 = flow(discrete)
        k2 = flow(discrete + 0.5 * substep * k1)
        k3 = flow(discrete + 0.5 * substep * k2)
        k4 = flow(discrete + substep * k3)
        discrete = discrete + substep / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    model = AcadosModel()
    model.name = config["model_name"]
    model.x, model.u, model.p = x, u, parameter
    model.disc_dyn_expr = discrete

    state_scale = np.asarray(config["state_error_scale"], dtype=float)
    input_scale = np.asarray(config["input_error_scale"], dtype=float)
    q = np.asarray(config["state_weight"], dtype=float) / np.square(state_scale)
    r = np.asarray(config["input_weight"], dtype=float) / np.square(input_scale)
    ny = nx + nu
    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = int(config["horizon_steps"])
    ocp.solver_options.tf = float(config["horizon_s"])
    ocp.parameter_values = np.eye(3).reshape(-1)
    ocp.cost.cost_type = ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = np.diag(np.concatenate((q, r)))
    ocp.cost.W_e = np.diag(float(config["terminal_weight_multiplier"]) * q)
    ocp.cost.Vx = np.zeros((ny, nx)); ocp.cost.Vx[:nx, :] = np.eye(nx)
    ocp.cost.Vu = np.zeros((ny, nu)); ocp.cost.Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref = np.concatenate((config["equilibrium_state"], config["equilibrium_input"]))
    ocp.cost.yref_e = np.asarray(config["equilibrium_state"], dtype=float)
    ocp.constraints.x0 = np.asarray(config["equilibrium_state"], dtype=float)
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.lbu = np.asarray(config["input_lower"], dtype=float)
    ocp.constraints.ubu = np.asarray(config["input_upper"], dtype=float)
    envelope = np.asarray(config["state_envelope_half_width"], dtype=float)
    center = np.asarray(config["equilibrium_state"], dtype=float)
    ocp.constraints.idxbx = np.arange(nx)
    ocp.constraints.lbx = center - envelope
    ocp.constraints.ubx = center + envelope
    ocp.constraints.idxbx_e = np.arange(nx)
    ocp.constraints.lbx_e = center - envelope
    ocp.constraints.ubx_e = center + envelope

    solver = config["solver"]
    ocp.solver_options.nlp_solver_type = solver["nlp_solver_type"]
    ocp.solver_options.qp_solver = solver["qp_solver"]
    ocp.solver_options.qp_solver_cond_N = int(solver["qp_solver_cond_N"])
    ocp.solver_options.hessian_approx = solver["hessian_approx"]
    ocp.solver_options.integrator_type = solver["integrator_type"]
    ocp.solver_options.regularize_method = solver["regularize_method"]
    ocp.solver_options.qp_solver_iter_max = int(solver["qp_solver_iter_max"])
    ocp.solver_options.qp_tol = float(solver["qp_tol"])
    ocp.solver_options.nlp_solver_warm_start_first_qp = bool(solver["warm_start_first_qp"])
    ocp.solver_options.print_level = int(solver["print_level"])
    ocp.code_gen_options.code_export_directory = str(output)
    ocp.code_gen_options.json_file = str(output / "acados_ocp.json")
    return ocp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output must be absent or empty: {output}")
    generation = config["generation"]
    acados_source = Path(os.environ.get("ACADOS_SOURCE_DIR", "")).resolve()
    if str(acados_source) != generation["acados_source_dir"]:
        raise RuntimeError(f"ACADOS_SOURCE_DIR mismatch: {acados_source}")
    commit = subprocess.check_output(["git", "-C", str(acados_source), "rev-parse", "HEAD"], text=True).strip()
    if commit != generation["acados_commit"]:
        raise RuntimeError(f"acados commit mismatch: {commit}")
    tera = Path(os.environ.get("TERA_PATH", "")).resolve()
    if not tera.is_file() or sha256(tera) != generation["tera_sha256"]:
        raise RuntimeError("TERA_PATH missing or hash mismatch")
    output.mkdir(parents=True, exist_ok=True)
    AcadosOcpSolver.generate(create_ocp(config, output))
    generated_files = sorted(path for path in output.rglob("*") if path.is_file())
    manifest = {
        "schema_version": 1, "profile": config["profile"],
        "generator": str(Path(__file__).resolve().relative_to(ROOT)),
        "generator_sha256": sha256(Path(__file__).resolve()),
        "config": str(args.config.resolve().relative_to(ROOT)),
        "config_sha256": sha256(args.config),
        "acados_source_dir": str(acados_source), "acados_commit": commit,
        "casadi_version": ca.__version__, "tera_version": generation["tera_version"],
        "tera_sha256": sha256(tera),
        "generated_files": [str(path.relative_to(output)) for path in generated_files],
        "generated_sha256": {str(path.relative_to(output)): sha256(path) for path in generated_files},
    }
    (output / "phase27_generation_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
