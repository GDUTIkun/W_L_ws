# RobotState / TorqueCommand Contract

Status: `frozen — Phase 03`

## Boundary

The shared physical boundary is:

```text
MuJoCo Adapter ─┐
                ├─ RobotState → Controller Core → TorqueCommand ─┬─ MuJoCo Adapter
Hardware Adapter┘                                                 └─ Hardware Adapter
```

All vectors use SI units and Phase 02 canonical world `{N}`: X forward, Y left, Z up. `B` is the torso `base_control_frame`, parallel to the CAD body frame with origin at the torso rigid-body COM.

## RobotState

| C++ field | Type | Unit / order | Exact semantic |
| --- | --- | --- | --- |
| `sample_time_ns` | `uint64` | ns | Acquisition time mapped into the Controller host's monotonic clock domain. It is not Unix/ROS wall time and must strictly increase between accepted samples. |
| `base_position_n_m` | `double[3]` | m, `[Nx,Ny,Nz]` | Position of the origin of `B` relative to the origin of `{N}`, expressed in `{N}`. |
| `q_n_from_b` | `double[4]` | `[w,x,y,z]` | Active unit quaternion rotating vectors expressed in `B` into `{N}`. `q` and `-q` are equivalent. |
| `base_linear_velocity_n_m_s` | `double[3]` | m/s, `[Nx,Ny,Nz]` | Velocity of the origin of `B` relative to `{N}`, expressed in `{N}`. |
| `base_angular_velocity_n_rad_s` | `double[3]` | rad/s, `[Nx,Ny,Nz]` | Angular velocity of `B` relative to `{N}`, expressed in `{N}`; it is not Euler-rate order or raw gyro axes. |
| `joint_position_rad` | `double[6]` | rad | Canonical output-joint coordinates in the fixed joint order below. |
| `joint_velocity_rad_s` | `double[6]` | rad/s | Time derivative of the same canonical joint coordinates. |
| `contact_state` | `uint8[2]` | `[left,right]` | Tri-state observation/estimate: `0 unknown`, `1 no-contact`, `2 contact`. Simulator ground truth is allowed only at its Adapter; hardware may validly publish `unknown`. |

Fixed joint order:

```text
[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]
```

Every numeric field must be finite. Quaternion norm error must not exceed the configured tolerance (default `1e-6`). Contact values outside `0..2` are invalid. There is no public sequence number: sequencing is transport metadata, while time ordering is a physical sample property.

## TorqueCommand

| C++ field | Type | Unit / order | Exact semantic |
| --- | --- | --- | --- |
| `source_sample_time_ns` | `uint64` | ns | `RobotState.sample_time_ns` from which this command was computed. |
| `joint_torque_nm` | `double[6]` | N·m | Desired canonical output-axis torque in the fixed joint order. All values must be finite. |

`TorqueCommand` contains no enable, e-stop, watchdog, transport sequence, current or driver diagnostics. Those safety/transport functions remain mandatory at the Adapter and actuator boundary but are not physical Controller output coordinates.

## ROS representation

`wheel_leg_msgs/msg/RobotState` maps one-to-one except that `geometry_msgs/Quaternion` stores components as fields `x,y,z,w`. Only `wheel_leg_ros::toRos/fromRos` may reorder `[w,x,y,z] ↔ [x,y,z,w]`. The messages intentionally contain no `std_msgs/Header`; `sample_time_ns` keeps the monotonic clock contract explicit.

## Controller lifecycle

```text
configure(config) → reset() → step(state, now_ns) ...
```

- `configure` validates `max_state_age_ns` and quaternion tolerance and resets history.
- `reset` clears only time/history state; it produces no command.
- `step` validates fields, rejects samples from the future, samples older than `max_state_age_ns`, and timestamps not strictly newer than the last accepted sample.
- The first accepted sample has `dt=0`; later calls derive `dt` from consecutive accepted sample times. A rejected sample never advances history.
- `step` is deterministic for the same configuration, reset state and input sequence. Errors are returned as `StepStatus`; validation does not throw.
- Until a separately validated algorithm is migrated, every accepted call returns exactly six finite zero torques. Rejected calls also carry a value-initialized zero command, but the ROS wrapper does not publish it.

This contract leaves Planner/NMPC/WBC scheduling inside the Core. Later multi-rate modules may use the accepted sample time and derived `dt` without changing the Adapter boundary.

## Package and dependency direction

```text
wheel_leg_msgs ───────────────┐
                              v
wheel_leg_core ───────→ wheel_leg_ros ───────→ rclcpp
  (C++17 only)           conversions + wrapper
```

- `wheel_leg_core`: ordinary C++ types, validation and safe Core; no ROS, MuJoCo, serial or CAN dependency.
- `wheel_leg_msgs`: aggregate ROS interface definitions only.
- `wheel_leg_ros`: explicit conversions and minimal node; depends on both packages.
- Adapters depend on the public types/messages; the Core never depends back on an Adapter.

Phase 05 `wheel_leg_stm32_bridge` remains isolated and is not a compatibility base. `canonicalFluToLegacyForwardRightUp` and its inverse are restricted to legacy position/linear-velocity fields and regression-tested. Full legacy Simulink state arrays still require named model-aware packers in the later algorithm migration phase; they are not aliases for this schema.
