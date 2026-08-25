# Phase 03 Interface Grounding

## Evidence basis

- CBM project `W_L_ws`, generation `2026-08-25T03:18:30Z`, mode `full`; all cited live-code paths reported `no_recorded_issue`. This is best-effort coverage, so the exact source snippets below remain authoritative.
- Phase 02 contract freezes FLU `{N}`, torso-COM `base_control_frame`, active `q_N_from_B=[w,x,y,z]`, and joint order `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`.
- Graphify links the Phase 03 PLAN to `RobotState → Controller Core → TorqueCommand` and to both Adapter boundaries. Historical field sketches were treated as candidates only.

## Actual Simulink consumers

| Source | Actual input/output | Phase 03 consequence |
| --- | --- | --- |
| `full_base_nmpc_state_signal.m:1-64` | Reconstructs time, base position, orientation/Euler rates, base linear velocity, six relative joint `q/dq`, and wheel longitudinal geometry into legacy `[t; state(16); wheelHeight]` | Public state must carry base pose/twist and six joint `q/dq`. Euler rates and wheel geometry are derived compatibility quantities, not public canonical fields. |
| `full_base_nmpc_reference.m:1-73` | Consumes `[t; planner(4); previousWrench(12)]` and emits NMPC horizon references | Planner/reference input is separate from physical `RobotState`; Phase 03 does not freeze it. |
| `controller_qp_core.m:1-161` | Legacy single-leg WBC consumes time, planar base state, joint `q/dq`, upper wrench and motion terms; emits three joint torques | Upper-layer wrench/reference values are internal Controller data, not Adapter state. |
| `spatial_two_leg_qp_core.m:1-718` | Current two-leg WBC consumes `[fullNmpcState(18); qL/dqL; qR/dqR; upperWrench(12); wheelReference(4)]` and emits six torques | The external boundary needs canonical base pose/twist and joint `q/dq`; legacy state, upper wrench and wheel references require explicitly named internal packers after algorithm migration. |

The baseline currently assumes both rolling contacts inside its model and does not accept a trustworthy hardware contact observation. Phase 03 therefore exposes only a left/right tri-state contact observation (`unknown/no-contact/contact`); `unknown` is valid and prevents simulation ground truth from becoming a hardware requirement.

## Canonical field decisions

| Public field | Evidence/need | Native-to-canonical responsibility |
| --- | --- | --- |
| `sample_time_ns` | Controller consumers use sample time; Core needs ordering and age checks | Adapter maps acquisition time into Controller-host monotonic nanoseconds. ROS wall/header time is not used. |
| torso-COM position and `q_N_from_B` | Legacy NMPC/WBC consume base position/orientation | Adapter applies Phase 02 origin and frame transforms. |
| torso-COM linear velocity and base angular velocity, both expressed in `{N}` | Legacy state consumes base velocity and Euler rates | Adapter supplies canonical twist; legacy packer derives Euler rates from quaternion + angular velocity. |
| six joint `q/dq` | Both WBC paths consume them | Adapter applies Phase 02/04 sign and offset mapping. |
| left/right contact tri-state | Required for later physical contact-aware control without requiring simulator truth | Adapter may publish `unknown` when no validated estimator exists. |
| six joint output-axis torques | Current two-leg WBC output and target Adapter boundary | Controller emits canonical order; Adapter applies native torque sign. |

Not included: raw IMU, acceleration, measured effort, driver diagnostics, transport sequence, enable/e-stop, planner references, NMPC wrench, wheel-height/`xi` compatibility fields. Each is raw/diagnostic, transport/safety, internal Controller data, or derivable from the frozen state plus model.

## Current ROS assets

CBM found one current ROS entry point: `wheel_leg_stm32_bridge/src/stm32_bridge_node.cpp:main`. Its `NormalState`/`NormalCommand` and protocol structs mix raw IMU, feedback effort, enable/e-stop and transport diagnostics. They remain Phase 05 experimental interfaces and are not reused as the canonical schema. The `ros_ws/README.md` reference to missing `wheel_leg_bridge` was stale and is corrected by Phase 03.

## Compatibility boundary

Legacy packers, when added with the migrated algorithm, must be explicitly named and tested:

```text
native sample -> Adapter transform -> RobotState
RobotState + model -> legacy 16-state / WBC input pack
legacy six-torque result -> TorqueCommand -> Adapter transform
```

The legacy `[forward,right,up]` field order is never treated as a 3-D frame. Planner/reference packs stay inside Controller Core and are not fields of `RobotState`.
