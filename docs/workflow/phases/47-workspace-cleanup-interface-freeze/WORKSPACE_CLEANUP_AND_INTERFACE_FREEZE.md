# Workspace Cleanup and Interface Freeze

## Scope and authority

- Route changed to MuJoCo-only: `Controller Core → ROS2 → MuJoCo`.
- Phase 46 baseline commit: `502ffc6`.
- Controller/QP/task/primitive-contact numerical behavior changes: **NONE**.
- Phase 46 stays `review/REWORK`; no Phase 46 evidence was overwritten.

## Changes

- Removed `firmware/` and `wheel_leg_stm32_bridge` (72 tracked files in the deletion commit).
- Archived four hardware-route documents under `docs/legacy/hardware/`; old paths are redirects.
- Added ROS Weighted-WBC mode, a single compiled nominal controller profile, frozen H0 initialization,
  reset/startup handshake and one current launch/config.
- Kept the direct `weighted_wbc_loop` oracle. Phase34–46 specialized runners are isolated behind the
  default-OFF `WHEEL_LEG_BUILD_LEGACY_RUNNERS` option.
- Rewrote active README/architecture/workflow/interface documents for MuJoCo-only authority.

## Frozen runtime interfaces

- Public: RobotState and TorqueCommand only.
- Internal: W_ref (`WbcReference`), reconstructed W_WBC, signed interaction slack, solver/rank/residual
  diagnostics and W_MJ oracle semantics.
- Full definitions: `docs/interfaces/robot_state_torque_command.md`.

## Commits

- `502ffc6`: pre-cleanup Phase 46 authority and evidence.
- `5b0d8ff`: current ROS Weighted-WBC runtime and legacy build isolation.
- `3e1d8e1`: retired hardware runtime deletion.
- `48934de`: MuJoCo-only route documentation, archive redirects and pre/post regression evidence.
