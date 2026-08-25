# Phase 17 Grounding

Grounding 使用 CBM project `W_L_ws` generation `2026-08-25T06:16:31Z` 查询 Core、ROS wrapper、Adapter 和调用边界；Phase 16 runner 在该 generation 中尚未跟踪，因此直接读取当前源码。文档和实验脚本按仓库规则直接读取。Graphify 只查询已有本地图，没有执行 extract/update，也没有修改 `graphify-out/`。

| Responsibility | Existing authority | Phase 17 increment |
| --- | --- | --- |
| canonical state/command | `wheel_leg_core/types.hpp` | 不改 schema、顺序、单位或时间语义 |
| control computation | `ControllerCore::configure/reset/step` | opt-in Joint PD、解析重力、求和后 clamp、diagnostics 和 reference reset |
| ROS transport | `wheel_leg_ros::ControllerNode` | 静态 profile 参数；默认仍为 zero，非法 profile 启动失败 |
| native mapping/watchdog | `wheel_leg_mujoco::Adapter` | 原样复用 `q_C=-q_M+b`、`tau_M=-tau_C` 和 fail-to-zero |
| fixed-step physics | Phase 16 `deterministic_loop` | 只追加 control scenario/reference/disturbance/diagnostic 列，不复制循环 |
| closed-chain reduction | Phase 15 `S=[I;-pinv(Jp)Ja]` | 作为离线 gravity oracle；不进入 production Core |

Core 和 ROS package 均没有 MuJoCo include、link 或 runtime model pointer。真机、contact、floating-base、站立、WBC 与 NMPC 没有进入本 Phase。
