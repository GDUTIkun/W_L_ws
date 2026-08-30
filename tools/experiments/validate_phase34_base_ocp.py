#!/usr/bin/env python3
"""Validate the generated Phase 34 RTI and converged-SQP OCP artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import warnings
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver

from generate_phase34_base_acados_solver import discrete_expression

ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clean(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: clean(item) for key, item in value.items()}
    if isinstance(value, list):
        return [clean(item) for item in value]
    return value


def advance(reference: np.ndarray, stage: int, step: float) -> np.ndarray:
    result = reference.copy()
    time = stage * step
    result[:3] += time * reference[6:9]
    result[3:6] += time * reference[9:12]
    return result


class Solver:
    def __init__(self, directory: Path, config: dict):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            ocp = AcadosOcp.from_json(str(directory / "acados_ocp.json"))
            self.solver = AcadosOcpSolver(ocp, generate=False, build=False)
        self.config = config
        self.n = int(config["horizon_steps"])
        self.step = float(config["sampling_period_s"])
        self.q = np.asarray(config["state_weight"]) / np.square(config["state_error_scale"])
        self.r = np.asarray(config["input_weight"]) / np.square(config["input_error_scale"])
        self.qe = float(config["terminal_weight_multiplier"]) * self.q
        self.equilibrium_input = np.asarray(config["equilibrium_input"])
        self.envelope = np.asarray(config["state_envelope_half_width"])
        x, u, p, discrete = discrete_expression(config)
        self.discrete = ca.Function(
            "phase34_audit_discrete",
            [x, u, p],
            [discrete, ca.jacobian(discrete, x), ca.jacobian(discrete, u)],
        )
        self.input_lower = np.asarray(config["input_lower"])
        self.input_upper = np.asarray(config["input_upper"])
        self.input_scale = np.asarray(config["input_error_scale"])

    def reset(self) -> None:
        self.solver.reset(reset_qp_solver_mem=True, reset_numerical_values=True)

    def solve(
        self,
        state: np.ndarray,
        reference: np.ndarray,
        rotation: np.ndarray,
        xi: np.ndarray,
        *,
        cold: bool,
    ) -> dict:
        parameter = np.r_[rotation.reshape(-1), xi]
        running_weight = np.diag(np.r_[self.q, self.r])
        terminal_weight = np.diag(self.qe)
        references = []
        for stage in range(self.n + 1):
            stage_reference = advance(reference, stage, self.step)
            references.append(stage_reference)
            self.solver.set(stage, "p", parameter)
            self.solver.cost_set(
                stage,
                "yref",
                np.r_[stage_reference, self.equilibrium_input]
                if stage < self.n
                else stage_reference,
            )
            self.solver.cost_set(stage, "W", running_weight if stage < self.n else terminal_weight)
            if stage > 0:
                self.solver.constraints_set(stage, "lbx", stage_reference - self.envelope)
                self.solver.constraints_set(stage, "ubx", stage_reference + self.envelope)
            if cold:
                self.solver.set(stage, "x", stage_reference)
                if stage < self.n:
                    self.solver.set(stage, "u", self.equilibrium_input)
        self.solver.constraints_set(0, "lbx", state)
        self.solver.constraints_set(0, "ubx", state)
        status = int(self.solver.solve())
        states = [np.asarray(self.solver.get(stage, "x")) for stage in range(self.n + 1)]
        controls = [np.asarray(self.solver.get(stage, "u")) for stage in range(self.n)]
        models = [self.discrete(states[stage], controls[stage], parameter) for stage in range(self.n)]
        defects = [
            float(np.max(np.abs(states[stage + 1] - np.asarray(models[stage][0]).reshape(-1))))
            for stage in range(self.n)
        ]
        costate = self.qe * (states[-1] - references[-1])
        projected_stationarity = 0.0
        for stage in range(self.n - 1, -1, -1):
            jacobian_x = np.asarray(models[stage][1])
            jacobian_u = np.asarray(models[stage][2])
            gradient = self.step * self.r * (controls[stage] - self.equilibrium_input) + jacobian_u.T @ costate
            projected = np.clip(
                controls[stage] - gradient / (self.step * self.r),
                self.input_lower,
                self.input_upper,
            )
            projected_stationarity = max(
                projected_stationarity,
                float(np.max(np.abs(controls[stage] - projected) / self.input_scale)),
            )
            costate = self.step * self.q * (states[stage] - references[stage]) + jacobian_x.T @ costate
        residuals = np.asarray(self.solver.get_residuals(recompute=True))
        return {
            "status": status,
            "u0": controls[0],
            "residuals": residuals,
            "maximum_defect": max(defects),
            "projected_stationarity": projected_stationarity,
            "time_tot_s": float(self.solver.get_stats("time_tot")),
            "sqp_iter": int(self.solver.get_stats("sqp_iter")),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--method", type=Path, required=True)
    parser.add_argument("--rti", type=Path, required=True)
    parser.add_argument("--sqp", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise RuntimeError(f"output already exists: {output}")
    config = json.loads(args.config.read_text(encoding="utf-8"))
    method = json.loads(args.method.read_text(encoding="utf-8"))
    gates = method["gates"]
    for directory in (args.rti, args.sqp):
        if not (directory / "acados_ocp.json").is_file() or not list(directory.glob("libacados_ocp_solver_*.so")):
            raise RuntimeError(f"unbuilt artifact: {directory}")

    rng = np.random.default_rng(3411)
    equilibrium = np.asarray(config["equilibrium_state"])
    envelope = np.asarray(config["state_envelope_half_width"])
    xi_equilibrium = np.asarray(config["equilibrium_wheel_position_m"])
    rti = Solver(args.rti, config)
    cases = []
    state = equilibrium.copy()
    for index in range(40):
        reference = equilibrium.copy()
        reference[6] = 0.1 * min(index / 20.0, 1.0)
        if index > 0:
            state += 0.002 * envelope * rng.uniform(-1, 1, 12)
            state = np.clip(state, reference - 0.1 * envelope, reference + 0.1 * envelope)
        state[3:6] = np.clip(state[3:6], -0.01, 0.01)
        xi = xi_equilibrium + rng.uniform(-0.01, 0.01, 2)
        solution = rti.solve(state, reference, np.eye(3), xi, cold=index == 0)
        cases.append(solution)

    sqp = Solver(args.sqp, config)
    converged = []
    for index in range(8):
        reference = equilibrium.copy()
        state = reference + 0.1 * envelope * rng.uniform(-1, 1, 12)
        state[3:6] *= 0.25
        result = sqp.solve(state, reference, np.eye(3), xi_equilibrium, cold=True)
        passes = 1
        while (
            (result["residuals"][0] > float(gates["converged_stationarity_max"])
             or max(result["residuals"][1:]) > float(gates["converged_feasibility_max"]))
            and passes < 5
        ):
            result = sqp.solve(state, reference, np.eye(3), xi_equilibrium, cold=False)
            passes += 1
        result["passes"] = passes
        converged.append(result)
        sqp.reset()

    rti.reset()
    deterministic_a = rti.solve(equilibrium, equilibrium, np.eye(3), xi_equilibrium, cold=True)
    rti.reset()
    deterministic_b = rti.solve(equilibrium, equilibrium, np.eye(3), xi_equilibrium, cold=True)
    maxima = {
        "rti_time_s": max(case["time_tot_s"] for case in cases),
        "rti_stationarity": max(float(case["residuals"][0]) for case in cases),
        "rti_projected_stationarity": max(case["projected_stationarity"] for case in cases),
        "rti_feasibility": max(float(max(case["residuals"][1:])) for case in cases),
        "rti_defect": max(case["maximum_defect"] for case in cases),
        "sqp_stationarity": max(float(case["residuals"][0]) for case in converged),
        "sqp_feasibility": max(float(max(case["residuals"][1:])) for case in converged),
        "deterministic_u0_error": float(np.max(np.abs(deterministic_a["u0"] - deterministic_b["u0"]))),
    }
    pass_map = {
        "rti_status": all(case["status"] == 0 for case in cases),
        "rti_deadline": maxima["rti_time_s"] <= float(gates["solver_deadline_s"]),
        "rti_projected_stationarity": maxima["rti_projected_stationarity"] <= float(gates["solver_projected_stationarity"]),
        "rti_defect": maxima["rti_defect"] <= float(gates["solver_maximum_defect"]),
        "sqp_status": all(case["status"] == 0 for case in converged),
        "sqp_stationarity": maxima["sqp_stationarity"] <= float(gates["converged_stationarity_max"]),
        "sqp_feasibility": maxima["sqp_feasibility"] <= float(gates["converged_feasibility_max"]),
        "cold_reset_determinism": maxima["deterministic_u0_error"] <= float(gates["replay_max_abs_error"]),
    }
    summary = {"pass": all(pass_map.values()), "gates": pass_map, "maxima": maxima}
    output.mkdir(parents=True)
    (output / "details.json").write_text(json.dumps(clean({"rti": cases, "sqp": converged}), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "summary.json").write_text(json.dumps(clean(summary), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema_version": 1,
        "profile": method["profile"] + "_base_ocp",
        "command": " ".join(__import__("sys").argv),
        "runner": str(Path(__file__).resolve().relative_to(ROOT)),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "config_sha256": sha256(args.config),
        "method_sha256": sha256(args.method),
        "rti_manifest_sha256": sha256(args.rti / "phase34_generation_manifest.json"),
        "sqp_manifest_sha256": sha256(args.sqp / "phase34_generation_manifest.json"),
        "outputs": {name: sha256(output / name) for name in ("details.json", "summary.json")},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
