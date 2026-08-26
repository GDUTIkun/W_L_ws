#!/usr/bin/env python3
"""Identify and validate the Phase-20 static full-3D standing controller."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from validate_mujoco_3d_standing_contract import (
    ACTIVE_JOINTS,
    PASSIVE_JOINTS,
    object_id,
    rotation_vector,
    sha256,
)

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "simulation/mujoco/config/phase20_prefreeze.json"


class Plant:
    def __init__(
        self,
        config: dict[str, Any],
        equilibrium: list[float],
        s_roll: list[float],
    ) -> None:
        self.config = config
        self.equilibrium = np.asarray(equilibrium, dtype=float)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / config["scene"]))
        self.base_weld = object_id(
            self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "base_weld"
        )
        self.base_site = object_id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "base_control_frame"
        )
        self.base_body = object_id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "base_body"
        )
        self.floor = object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.wheels = [
            object_id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            for name in ("left_wheel_collision", "right_wheel_collision")
        ]
        self.active_joint_ids = np.asarray([
            object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in ACTIVE_JOINTS
        ])
        self.active_qpos = self.model.jnt_qposadr[self.active_joint_ids]
        self.active_dofs = self.model.jnt_dofadr[self.active_joint_ids]
        self.passive_qpos = np.asarray([
            self.model.jnt_qposadr[
                object_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            ] for name in PASSIVE_JOINTS
        ])
        self.reference = np.asarray([
            equilibrium[2], equilibrium[3], 0.0,
            equilibrium[0], equilibrium[1], 0.0,
        ])
        self.passive_reference = np.asarray(equilibrium[5:9], dtype=float)
        self.support_native = np.asarray([
            equilibrium[9], equilibrium[10], 0.0,
            equilibrium[11], equilibrium[12], 0.0,
        ])
        self.torque_limit = np.asarray(config["torque_limit_nm"], dtype=float)
        self.virtual_basis = np.column_stack([
            np.asarray([0, 0, 1, 0, 0, 1], dtype=float),
            np.asarray(s_roll, dtype=float),
            np.asarray([0, 0, 1, 0, 0, -1], dtype=float),
        ])
        self.anchor = self.state(self.reset())[1]

    def reset(self) -> mujoco.MjData:
        data = mujoco.MjData(self.model)
        data.eq_active[self.base_weld] = 0
        data.qpos[:7] = (0.0, 0.0, self.equilibrium[4], 1.0, 0.0, 0.0, 0.0)
        data.qpos[self.active_qpos] = self.reference
        data.qpos[self.passive_qpos] = self.passive_reference
        data.qvel[:] = 0.0
        data.ctrl[:] = self.support_native
        mujoco.mj_forward(self.model, data)
        return data

    def reset_state(self, state: np.ndarray) -> mujoco.MjData:
        data = self.reset()
        rotation = np.asarray([state[4], state[2], state[6]])
        angle = float(np.linalg.norm(rotation))
        if angle > 0.0:
            data.qpos[3:7] = np.concatenate([[math.cos(angle / 2.0)],
                math.sin(angle / 2.0) * rotation / angle])
        mujoco.mj_forward(self.model, data)
        data.qpos[0] += self.anchor[0] + state[0] - data.site_xpos[self.base_site, 0]
        data.qvel[3:6] = (state[5], state[3], state[7])
        mujoco.mj_forward(self.model, data)
        data.qvel[0] += state[1] - self.state(data)[1][3]
        mujoco.mj_forward(self.model, data)
        return data

    def state(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
        rotation = data.site_xmat[self.base_site].reshape(3, 3)
        error = rotation_vector(rotation)
        jacobian_position = np.zeros((3, self.model.nv))
        jacobian_rotation = np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(
            self.model, data, jacobian_position, jacobian_rotation,
            self.base_site,
        )
        linear = jacobian_position @ data.qvel
        angular = jacobian_rotation @ data.qvel
        position = data.site_xpos[self.base_site].copy()
        state = np.asarray([
            position[0], linear[0], error[1], angular[1],
            error[0], angular[0], error[2], angular[2],
        ])
        return state, np.concatenate([position, linear, error, angular])

    def contact_bits(self, data: mujoco.MjData) -> tuple[int, int]:
        result = [0, 0]
        for contact in data.contact[: data.ncon]:
            geoms = {int(contact.geom1), int(contact.geom2)}
            for side, wheel in enumerate(self.wheels):
                if geoms == {wheel, self.floor}:
                    result[side] = 1
        return result[0], result[1]

    def control(self, data: mujoco.MjData, virtual: np.ndarray) -> tuple[np.ndarray, bool]:
        native = self.support_native.copy()
        leg = np.asarray([0, 1, 3, 4])
        native[leg] += (
            float(self.config["leg_kp_nm_per_rad"])
            * (self.reference[leg] - data.qpos[self.active_qpos[leg]])
            - float(self.config["leg_kd_nm_s_per_rad"])
            * data.qvel[self.active_dofs[leg]]
        )
        # Adapter maps canonical torque to native with a minus sign.
        native -= self.virtual_basis @ virtual
        clipped = np.clip(native, -self.torque_limit, self.torque_limit)
        return clipped, bool(np.any(clipped != native))

    def tick(
        self,
        data: mujoco.MjData,
        virtual: np.ndarray,
        force: np.ndarray | None = None,
        moment: np.ndarray | None = None,
    ) -> bool:
        data.ctrl[:], saturated = self.control(data, virtual)
        for _ in range(int(self.config["physics_steps_per_control"])):
            data.xfrc_applied[:] = 0.0
            if force is not None:
                data.xfrc_applied[self.base_body, :3] = force
            if moment is not None:
                data.xfrc_applied[self.base_body, 3:] = moment
            mujoco.mj_step(self.model, data)
        return saturated


def collect(
    plant: Plant,
    seed: int,
    episodes: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    settings = plant.config["identification"]
    rng = np.random.default_rng(seed)
    amplitude = np.asarray(settings["input_amplitude_nm"], dtype=float)
    before, inputs, after = [], [], []
    bilateral = 0
    total = 0
    for _ in range(episodes):
        data = plant.reset()
        virtual = np.zeros(3)
        for tick in range(int(settings["ticks_per_episode"])):
            if tick < int(settings["excitation_ticks"]):
                if tick % int(settings["hold_ticks"]) == 0:
                    virtual = amplitude * rng.choice((-1.0, 1.0), size=3)
            else:
                virtual = np.zeros(3)
            state, _ = plant.state(data)
            state[0] -= plant.anchor[0]
            plant.tick(data, virtual)
            next_state, _ = plant.state(data)
            next_state[0] -= plant.anchor[0]
            before.append(state)
            inputs.append(virtual.copy())
            after.append(next_state)
            bilateral += int(all(plant.contact_bits(data)))
            total += 1
    return (
        np.asarray(before), np.asarray(inputs), np.asarray(after),
        bilateral / total,
    )


def linearize(
    plant: Plant,
    state_scale: np.ndarray,
    input_scale: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    fraction = float(plant.config["design"]["linearization_fraction_of_scale"])

    def transition(state: np.ndarray, virtual: np.ndarray) -> np.ndarray:
        data = plant.reset_state(state)
        plant.tick(data, virtual)
        result, _ = plant.state(data)
        result[0] -= plant.anchor[0]
        return result

    zero_state = np.zeros(8)
    zero_input = np.zeros(3)
    state_steps = fraction * state_scale
    input_steps = fraction * input_scale
    a = np.column_stack([
        (transition(np.eye(8)[index] * state_steps[index], zero_input)
         - transition(-np.eye(8)[index] * state_steps[index], zero_input))
        / (2.0 * state_steps[index])
        for index in range(8)
    ])
    b = np.column_stack([
        (transition(zero_state, np.eye(3)[index] * input_steps[index])
         - transition(zero_state, -np.eye(3)[index] * input_steps[index]))
        / (2.0 * input_steps[index])
        for index in range(3)
    ])
    affine = transition(zero_state, zero_input)
    scaled_a = a * state_scale[None, :] / state_scale[:, None]
    scaled_b = b * input_scale[None, :] / state_scale[:, None]
    return a, b, affine, scaled_a, scaled_b


def normalized_rms(
    a: np.ndarray,
    b: np.ndarray,
    affine: np.ndarray,
    states: np.ndarray,
    inputs: np.ndarray,
    next_states: np.ndarray,
    scale: np.ndarray,
) -> float:
    predicted = states @ a.T + inputs @ b.T + affine
    return float(np.sqrt(np.mean(((predicted - next_states) / scale) ** 2)))


def lqr(
    config: dict[str, Any],
    scaled_a: np.ndarray,
    scaled_b: np.ndarray,
) -> tuple[np.ndarray, int]:
    design = config["design"]
    state_scale = np.asarray(design["state_scale"], dtype=float)
    input_scale = np.asarray(design["input_scale_nm"], dtype=float)
    q = np.eye(8)
    r = float(design["control_penalty"]) * np.eye(3)
    p = q.copy()
    for iteration in range(int(design["riccati_maximum_iterations"])):
        gain = np.linalg.solve(
            r + scaled_b.T @ p @ scaled_b,
            scaled_b.T @ p @ scaled_a,
        )
        updated = (
            q + scaled_a.T @ p @ scaled_a
            - scaled_a.T @ p @ scaled_b @ gain
        )
        relative_change = np.max(np.abs(updated - p)) / max(
            1.0, float(np.max(np.abs(updated)))
        )
        if relative_change <= float(design["riccati_tolerance"]):
            physical_gain = input_scale[:, None] * gain / state_scale[None, :]
            return physical_gain, iteration + 1
        p = updated
    raise RuntimeError("Discrete Riccati iteration did not converge")


def run_case(
    plant: Plant,
    gain: np.ndarray,
    case: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    data = plant.reset()
    rows = []
    saturated_count = 0
    ticks = int(round(float(plant.config["duration_s"]) / float(plant.config["control_period_s"])))
    start = int(plant.config["disturbance_start_tick"])
    stop = start + int(plant.config["disturbance_ticks"])
    force = np.asarray(case.get("force_n", [0, 0, 0]), dtype=float)
    moment = np.asarray(case.get("moment_nm", [0, 0, 0]), dtype=float)
    for tick in range(ticks):
        state, diagnostics = plant.state(data)
        state[0] -= plant.anchor[0]
        virtual = -gain @ state
        active = start <= tick < stop
        saturated_count += int(plant.tick(
            data, virtual,
            force if active else None,
            moment if active else None,
        ))
        state_after, diagnostics_after = plant.state(data)
        state_after[0] -= plant.anchor[0]
        left, right = plant.contact_bits(data)
        rows.append({
            "case": case["id"], "tick": tick, "time_s": float(data.time),
            "x_m": float(state_after[0]), "vx_m_s": float(state_after[1]),
            "pitch_rad": float(state_after[2]), "omega_y_rad_s": float(state_after[3]),
            "roll_rad": float(state_after[4]), "omega_x_rad_s": float(state_after[5]),
            "yaw_rad": float(state_after[6]), "omega_z_rad_s": float(state_after[7]),
            "y_m": float(diagnostics_after[1] - plant.anchor[1]),
            "z_error_m": float(diagnostics_after[2] - plant.anchor[2]),
            "vy_m_s": float(diagnostics_after[4]), "vz_m_s": float(diagnostics_after[5]),
            "u_common_nm": float(virtual[0]), "u_roll_nm": float(virtual[1]),
            "u_yaw_nm": float(virtual[2]),
            "left_contact": left, "right_contact": right,
        })
    final = rows[-1]
    metrics = {
        "case": case["id"],
        "finite": all(
            math.isfinite(float(value))
            for row in rows for value in row.values()
            if not isinstance(value, str)
        ),
        "maximum_abs_x_m": max(abs(row["x_m"]) for row in rows),
        "maximum_abs_y_m": max(abs(row["y_m"]) for row in rows),
        "maximum_height_error_m": max(abs(row["z_error_m"]) for row in rows),
        "maximum_abs_pitch_rad": max(abs(row["pitch_rad"]) for row in rows),
        "maximum_abs_roll_rad": max(abs(row["roll_rad"]) for row in rows),
        "maximum_abs_yaw_rad": max(abs(row["yaw_rad"]) for row in rows),
        "final_linear_speed_m_s": float(np.linalg.norm([
            final["vx_m_s"], final["vy_m_s"], final["vz_m_s"]
        ])),
        "final_angular_speed_rad_s": float(np.linalg.norm([
            final["omega_x_rad_s"], final["omega_y_rad_s"], final["omega_z_rad_s"]
        ])),
        "bilateral_contact_fraction": sum(
            row["left_contact"] and row["right_contact"] for row in rows
        ) / len(rows),
        "saturation_count": saturated_count,
    }
    gates = plant.config["gates"]
    metrics["pass"] = bool(
        metrics["finite"]
        and metrics["maximum_abs_x_m"] <= gates["maximum_abs_x_m"]
        and metrics["maximum_abs_y_m"] <= gates["maximum_abs_y_m"]
        and metrics["maximum_height_error_m"] <= gates["maximum_height_error_m"]
        and metrics["maximum_abs_pitch_rad"] <= gates["maximum_abs_pitch_rad"]
        and metrics["maximum_abs_roll_rad"] <= gates["maximum_abs_roll_rad"]
        and metrics["maximum_abs_yaw_rad"] <= gates["maximum_abs_yaw_rad"]
        and metrics["final_linear_speed_m_s"] <= gates["maximum_final_linear_speed_m_s"]
        and metrics["final_angular_speed_rad_s"] <= gates["maximum_final_angular_speed_rad_s"]
        and metrics["bilateral_contact_fraction"] >= gates["minimum_bilateral_contact_fraction"]
        and metrics["saturation_count"] <= gates["maximum_saturation_count"]
    )
    return rows, metrics


def controllability_rank(a: np.ndarray, b: np.ndarray) -> int:
    return int(np.linalg.matrix_rank(np.column_stack([
        np.linalg.matrix_power(a, power) @ b for power in range(a.shape[0])
    ])))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--equilibrium", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"Refusing non-empty output directory: {output}")
    output.mkdir(parents=True, exist_ok=True)

    config_path = args.config.resolve()
    equilibrium_path = args.equilibrium.resolve()
    contract_path = args.contract.resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    equilibrium = json.loads(equilibrium_path.read_text(encoding="utf-8"))["candidate"]
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    plant = Plant(config, equilibrium, contract["s_roll_canonical_joint_order"])
    identification = config["identification"]
    training = collect(
        plant, int(identification["seed"]), int(identification["training_episodes"])
    )
    validation = collect(
        plant, int(identification["seed"]) + 1,
        int(identification["validation_episodes"]),
    )
    state_scale = np.asarray(config["design"]["state_scale"], dtype=float)
    input_scale = np.asarray(config["design"]["input_scale_nm"], dtype=float)
    a, b, affine, scaled_a, scaled_b = linearize(
        plant, state_scale, input_scale
    )
    training_error = normalized_rms(a, b, affine, *training[:3], state_scale)
    validation_error = normalized_rms(a, b, affine, *validation[:3], state_scale)
    nominal_gain, riccati_iterations = lqr(config, scaled_a, scaled_b)

    chosen_gain = None
    chosen_scale = None
    tuning_evidence = []
    all_rows: list[dict[str, Any]] = []
    for scale in config["design"]["gain_scales"]:
        gain = float(scale) * nominal_gain
        cases = []
        scale_rows = []
        for case in config["tuning_cases"]:
            rows, metrics = run_case(plant, gain, case)
            scale_rows.extend(rows)
            cases.append(metrics)
        tuning_evidence.append({"gain_scale": scale, "cases": cases})
        if chosen_gain is None and all(case["pass"] for case in cases):
            chosen_gain, chosen_scale, all_rows = gain, float(scale), scale_rows
    if chosen_gain is None:
        chosen_gain = nominal_gain
        chosen_scale = 1.0

    holdouts = []
    for case in config["holdout_cases"]:
        rows, metrics = run_case(plant, chosen_gain, case)
        all_rows.extend(rows)
        holdouts.append(metrics)

    poles = np.linalg.eigvals(a - b @ chosen_gain)
    model = {
        "state_order": contract["state_order"],
        "virtual_input_order": contract["virtual_input_order"],
        "A": a.tolist(), "B": b.tolist(), "affine": affine.tolist(),
        "controllability_rank": controllability_rank(a, b),
        "training_normalized_rms": training_error,
        "validation_normalized_rms": validation_error,
        "training_bilateral_contact_fraction": training[3],
        "validation_bilateral_contact_fraction": validation[3],
        "riccati_iterations": riccati_iterations,
        "nominal_lqr_gain": nominal_gain.tolist(),
        "chosen_gain_scale": chosen_scale,
        "K_canonical_u_equals_minus_Kx": chosen_gain.tolist(),
        "closed_loop_poles": [[float(value.real), float(value.imag)] for value in poles],
        "closed_loop_spectral_radius": float(np.max(np.abs(poles))),
    }
    gates = config["gates"]
    model["pass"] = bool(
        model["controllability_rank"] == 8
        and max(training_error, validation_error)
        <= gates["maximum_identification_normalized_rms"]
        and model["closed_loop_spectral_radius"]
        <= gates["maximum_closed_loop_spectral_radius"]
        and training[3] == 1.0 and validation[3] == 1.0
    )
    summary = {
        "schema_version": 1,
        "phase": 20,
        "evidence_class": "exploratory pre-freeze decision gate",
        "model": model,
        "s_roll_canonical_joint_order": contract["s_roll_canonical_joint_order"],
        "tuning": tuning_evidence,
        "holdouts": holdouts,
        "overall_pass": bool(
            model["pass"]
            and any(all(case["pass"] for case in item["cases"]) for item in tuning_evidence)
            and all(case["pass"] for case in holdouts)
        ),
    }
    summary["decision"] = "IMPLEMENT_CORE" if summary["overall_pass"] else "REWORK"
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if all_rows:
        with (output / "timeseries.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(all_rows[0]))
            writer.writeheader()
            writer.writerows(all_rows)
    manifest = {
        "config": str(config_path.relative_to(ROOT)), "config_sha256": sha256(config_path),
        "equilibrium": str(equilibrium_path.relative_to(ROOT)), "equilibrium_sha256": sha256(equilibrium_path),
        "contract": str(contract_path.relative_to(ROOT)), "contract_sha256": sha256(contract_path),
        "script_sha256": sha256(Path(__file__).resolve()),
        "scene_sha256": sha256(ROOT / config["scene"]),
        "summary_sha256": sha256(output / "summary.json"),
        "hardware_data": False,
    }
    if (output / "timeseries.csv").exists():
        manifest["timeseries_sha256"] = sha256(output / "timeseries.csv")
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["overall_pass"] else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
