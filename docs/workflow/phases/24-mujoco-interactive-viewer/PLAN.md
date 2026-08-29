# Phase 24: MuJoCo interactive NMPC viewer — PLAN

Status: `complete`

## Goal

在不改变控制、通信和正式性能验证路径的前提下，提供一个可交互观察当前 nominal acados NMPC+WBC 闭环的 MuJoCo 画面入口。

## Current State

- 已有：`weighted_wbc_loop` 以 2 ms plant / 10 ms Controller / 5-step ZOH 执行 Phase 23 的 nominal NMPC+WBC，并输出可重复的 headless CSV。
- 缺少：同一 C++ Controller/Adapter 闭环的窗口、现场诊断和简单启动方式。
- 证据：Phase 23 formal-v2 已验证 headless loop；GLFW 3.3.10 与 MuJoCo 3.7.0 已在 host 安装。

## Scope

- 为既有 runner 加入显式的 interactive viewer 选项；默认 headless 行为、CLI 和 CSV schema 保持不变。
- 使用 MuJoCo + 已安装 GLFW 渲染模型、允许关闭窗口退出，并实时显示 sim time、controller mode、NMPC+WBC 耗时、solver/status/latch。
- viewer 模式不创建 CSV，不作为性能或 formal evidence；按物理时间 pacing 以便人工观察。
- 更新 Phase/ROS 入口文档，并实际构建与启动验证。

## Out of Scope

- 不改变 `Adapter`、`RobotState`/`TorqueCommand`、WBC/NMPC 数学、MuJoCo scene、通信/STM32 或正式 benchmark。
- 不做通用 GUI 框架、远程遥测、交互式调参、录制回放或 performance certification。

## Frozen Decisions

- UI 只是 `wheel_leg_mujoco` 的 host-side simulation tool，不属于 Hardware Adapter/通信层。
- `--viewer` 是 opt-in；不传时仍要求两个输出 CSV 且不链接/初始化窗口运行时逻辑。
- viewer 显示的 compute time 与 real-time pacing/渲染分离；正式性能以 headless Phase 23 入口为准。

## Open Questions / Decision Gates

None.

## Interfaces and Compatibility

- 输入：既有 `weighted_wbc_loop` 选项；viewer 最少只需 `--model`、`--viewer true`。
- 输出：窗口内模型和 overlay；关闭窗口正常结束，不写 CSV。
- 必须保持：headless 参数校验、non-overwrite、2/10 ms schedule、5-step ZOH、controller/adapter safety 语义。
- 允许改变：runner 的 opt-in viewer flag、CMake GLFW link 和用户入口文档。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P24-T01 | 冻结 viewer 边界与已有 loop 接缝 | Phase 23、CBM/Graphify、host dependencies | 本 PLAN 与最小实现决定 | current source/coverage/dependency probe | done |
| P24-T02 | 实现 opt-in MuJoCo viewer | existing C++ loop、MuJoCo、GLFW | `--viewer true`、overlay、real-time pacing、headless compatibility | clean build、headless smoke、X/GLFW smoke | done |
| P24-T03 | 记录人工入口并审查 | P24-T02 | README/REVIEW/RECORD、ROADMAP 状态 | GUI launch and close, regression result | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `colcon build --packages-up-to wheel_leg_mujoco`：Release build succeeds with GLFW.
- Existing headless `weighted_wbc_loop` smoke with fresh CSV paths：exit 0 and non-empty outputs.
- `xvfb-run` viewer smoke where an X display is unavailable：window/OpenGL initialization and bounded simulation succeed.

### Manual / Evidence

- Run the documented `--viewer true` NMPC command on a desktop, observe rendering and overlay, then close the window. PASS means controller loop remains live until close and reports diagnostics without writing CSV.

## Acceptance Criteria

- [x] Viewer is opt-in and uses the existing ControllerCore + Adapter loop.
- [x] Default headless CSV/formal behavior is unchanged.
- [x] Overlay exposes simulation time, controller status/latch and NMPC+WBC time.
- [x] Build and headless regression pass; GUI initialization is actually exercised.
- [x] REVIEW is PASS before RECORD and ROADMAP completion.

## Execution Notes

按任务 ID 记录实际命令、结果、偏差和证据链接；不要建立第二份任务状态表。

- 2026-08-29 P24-T01：CBM generation `2026-08-28T13:13:14Z` 定位 `weighted_wbc_loop::run` 与 Adapter 边界；该 runner source metadata changed，已直接读取。Graphify 现有图确认既有 viewer 仅为 Phase19 Python 观察工具。host probe：MuJoCo 3.7.0、GLFW 3.3.10 可用。
- 2026-08-29 P24-T02：`weighted_wbc_loop --viewer true` 仅在 opt-in 时创建 GLFW/MuJoCo renderer；不创建 CSV，按 `data->time` pacing。overlay 显示 sim time、mode、Core/NMPC+WBC milliseconds、Core/NMPC/acados status、latch 与 age；Esc/关闭窗口退出。headless 默认路径及 Adapter/Controller 不改。
- 2026-08-29 P24-T03：Release build command `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DACADOS_ROOT=/home/t/opt/acados` PASS。fresh `/tmp` headless NMPC 20-tick CSV smoke PASS；`DISPLAY=:0 ... --viewer true ... --ticks 5` actual GLFW render smoke PASS；`colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose` = `26 tests, 0 errors, 0 failures, 0 skipped`。`git diff --check` PASS。

## Blockers

None.
