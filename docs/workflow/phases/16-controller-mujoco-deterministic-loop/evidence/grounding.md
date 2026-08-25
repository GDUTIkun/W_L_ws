# Phase 16 Grounding

## Live code baseline

Grounding 使用 CBM project `W_L_ws` generation `2026-08-25T06:16:31Z` 查询当前符号、调用关系和依赖；`adapter.hpp:28` 的 partial coverage 由直接读取源码补足。文档和实验脚本不在 CBM 主索引中，按仓库规则直接读取。Graphify 只查询现有本地图核对 Phase 14/15 历史与两轮复现路线，没有执行 extract/update，也没有修改 `graphify-out/`。

| Responsibility | Current implementation | Phase 16 treatment |
| --- | --- | --- |
| Canonical validation/control | `wheel_leg::ControllerCore::configure/reset/step` | 原样链接；有效状态仍只产生零力矩 |
| ROS conversion/wrapper | `wheel_leg_ros` conversions and controller node | 不进入正式 deterministic loop；只作兼容性回归 |
| MuJoCo state/command mapping | `wheel_leg_mujoco::Adapter::extractState/acceptCommand/writeControls/reset` | 原样复用；不复制 joint order/sign/offset/watchdog |
| Wall-paced simulation | `wheel_leg_mujoco::MujocoNode` and `zero_loop.launch.py` | 只作 topic/schema/reset smoke，不作为确定性证据 |
| Physics/control scheduler | Phase 04 没有固定 control-tick runner | 本 Phase 新增 C++ `deterministic_loop` |
| Evidence orchestration | Phase 14/15 non-overwrite runners | 复用其 profile/manifest/追加式证据模式 |

## Increment over Phase 04

Phase 04 已经证明 canonical/native 映射、one-hot 非零力矩、watchdog、reset、fixed/floating smoke。本 Phase 不重复认领这些结论，只新增：直接组合 Core/Adapter/MuJoCo 的固定步数调度、逐 control-tick 日志、独立 source/receipt clock、fault schedule、reset/fresh replay、跨进程比较和完整 hash manifest。

## Dependency and scope result

- Core 不依赖 MuJoCo；deterministic executable 依赖 Core 与 Adapter，Adapter 继续依赖 Core contract。
- 正式 scene 只新增 wrapper XML，不修改 `wheel_leg.xml` 或 Phase 14/15 evidence。
- 没有 Hardware Adapter、STM32、传感器、真机数据或真机命令路径。
- 为观察 receipt timeout，fault runner 只在标记样本处向 Adapter 注入 `1 N·m`；production Core 保持零输出，实验 torque 与 Core torque 在 CSV 中分列。

