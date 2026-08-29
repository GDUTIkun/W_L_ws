# Phase 24: MuJoCo interactive NMPC viewer — RECORD

Status: `complete`

## Outcome

当前 nominal acados NMPC+WBC 闭环已有一个 opt-in、实时 pacing 的 MuJoCo C++ viewer；正式 headless benchmark/formal 路径保持原样。

## Delivered

- [`weighted_wbc_loop.cpp`](../../../../ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp)：`--viewer true`、MuJoCo/GLFW renderer、overlay、Esc/关闭退出。
- [`CMakeLists.txt`](../../../../ros_ws/src/wheel_leg_mujoco/CMakeLists.txt)：使用 host 已安装的 GLFW 3.3。
- [`wheel_leg_mujoco README`](../../../../ros_ws/src/wheel_leg_mujoco/README.md)：可直接执行的 GUI 入口和性能边界说明。

## Verification Evidence

- Release build PASS；headless fresh CSV smoke PASS。
- `DISPLAY=:0` 下 actual GLFW/MuJoCo render smoke PASS。
- ROS component suite：26 tests, 0 errors, 0 failures, 0 skipped。

## Decisions Confirmed

- UI 位于 MuJoCo host simulation tool，不在 Adapter/通信层。
- UI 运行不写 CSV，渲染/v-sync 不进入正式性能结论。

## Deviations from PLAN

None.

## Known Limitations and Follow-ups

- 只提供观察与退出，不提供交互调参、remote telemetry 或 recording；需要这些能力时单独建立 Phase。
- 真机/STM32 通信仍冻结，Phase 05 仍 blocked。

## ROADMAP Update

- 本 Phase 对应阶段：Phase 24。
- 状态变化：`review → complete`。
- 下一建议 Phase：Phase 05，等待真机冻结解除。

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- ROADMAP：[ROADMAP](../../ROADMAP.md)
