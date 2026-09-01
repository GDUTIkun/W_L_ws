# Phase 47 Record

Status: `complete`

The repository now has one current route: `Controller Core → ROS2 → MuJoCo`, launched by
`wheel_leg_mujoco current_weighted_wbc.launch.py`. STM32 firmware, serial bridge and real-robot route
were removed; Simulink and historical experiment assets remain non-current references/oracles.

Public authority remains RobotState/TorqueCommand. W_ref, reconstructed W_WBC, signed interaction
slack and W_MJ diagnostics are frozen internal interfaces. The nominal WBC controller configuration
and H0 initialization each have one shared current definition.

Verification: 4-package build PASS, 37 tests PASS, ROS-vs-direct Core torque parity PASS, five-second
current launch smoke PASS, and Phase 46 pre/post authoritative replay PASS with no behavior change.
See [cleanup regression](CLEANUP_REGRESSION.md), [inventory](LEGACY_INTERFACE_INVENTORY.md),
[current path](../../../mujoco/CURRENT_CONTROL_PATH.md) and
[interface contract](../../../interfaces/robot_state_torque_command.md).

Historical Phase 46 stays `review/REWORK`. The next planned work is Phase 48 Weighted-WBC/QP
realization closure, followed by Phase 49 12X/16X NMPC candidate comparison.

## Commit boundaries

- `502ffc6`: Phase 46 pre-cleanup authority and evidence.
- `5b0d8ff`: current ROS Weighted-WBC path and legacy build isolation.
- `3e1d8e1`: retired hardware runtime deletion.
- `48934de`: route/workflow documentation, archive redirects and regression evidence.
