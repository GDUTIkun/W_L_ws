# W_L_ws

轮腿机器人从 Simulink 控制仿真逐步迁移到 MuJoCo 和真实机器的总仓库。

本项目不把尚未验证完成的控制器和参数一次性搬到真机，而是按照“定义、执行器、状态、机械模型、动力学、接触、闭环控制”的顺序逐层验证。每一层先在 MuJoCo 中通过，再进行对应的低风险真机验证；MuJoCo 与真机结果无法基本对应时，停留在当前层排查模型失配。

详细迁移路线见 [Simulink → MuJoCo → Real 流程](docs/mujoco/simulink%202%20mujoco%202%20real流程.md)，目标 ROS2 边界见 [ROS2 架构](docs/mujoco/ros2%20架构.md)。

## 当前状态

- 已成功复现的 Simulink 仿真基线位于 `simulation/simulink_baseline/`；生产 Controller 尚未从其中迁移。
- `simulation/` 尚无可运行的 MuJoCo 工程。
- `ros_ws/` 只有早期消息转换骨架；其依赖包、运行节点和完整接口尚未落地，当前不能视为可独立构建的 ROS2 workspace。
- `firmware/stm32/` 已包含 STM32 工程以及一套 UART2 状态/力矩通信实验实现。
- 树莓派与 STM32 的正式生产通信链路尚未决定；现有 UART2 实现是候选和实验资产，不是已冻结协议。

因此，本 README 只提供仓库导航和目标工作方式，不提供当前并不存在的“一键构建/运行”命令。可执行入口应由后续 Phase 在实现并验证后补充到相应子目录 README。

## 目标运行架构

```text
                        同一套 Controller Core（手写 C++）
                                      │
                               ROS2 Controller Node
                                      │
                         RobotState / TorqueCommand
                                      │
                    ┌─────────────────┴─────────────────┐
                    │                                   │
             主机仿真 profile                     树莓派真机 profile
                    │                                   │
             MuJoCo Adapter                      Hardware Adapter
                    │                                   │
                  MuJoCo                       STM32 → 驱动器/电机
```

- 主机运行 MuJoCo、MuJoCo Adapter 和 Controller。
- 真机运行时，树莓派运行同一 Controller 和 Hardware Adapter。
- MuJoCo 与真机最终都提供统一的聚合 `RobotState`，并接收统一的 `TorqueCommand`。
- Controller Core 与 ROS 消息、MuJoCo API、串口/CAN 等传输实现分层。
- 消息精确字段、六关节顺序、坐标系、单位和时间语义必须在专门 Phase 中冻结；本仓库当前文档只确定边界，不提前固化 schema。

## 迁移原则

1. Simulink 是算法和数值行为的对照基线，不作为生产代码生成源。
2. 控制算法手工重写为 C++，每个模块先完成 Simulink/C++ 一致性验证。
3. 同一控制核心依次连接 MuJoCo Adapter 与 Hardware Adapter，切换环境时不修改控制算法。
4. 证据依赖型结论必须读取真实仿真或实验结果；“代码写完”不等于模型、参数或控制效果获批。
5. 树莓派—STM32 传输方案、RobotState schema 等开放技术问题必须在对应 Phase 中明确决策，不能由实现过程顺带决定。

## 仓库导航

| 路径 | 职责 | 当前状态 |
| --- | --- | --- |
| [`docs/`](docs/README.md) | 技术设计、实验方法、迁移路线和人工工作流 | 已有 MuJoCo/真机路线文档 |
| [`firmware/`](firmware/README.md) | STM32 固件与板级实时逻辑 | 已有可追踪源码与 UART2 实验实现 |
| [`ros_ws/`](ros_ws/README.md) | 主机和树莓派共用的 ROS2 packages | 早期转换骨架 + Phase 05 实验 STM32 bridge |
| [`simulation/`](simulation/README.md) | 已复现的 Simulink 基线和 MuJoCo 工程 | Simulink baseline 已就位；MuJoCo 尚未落地 |
| [`tools/`](tools/README.md) | 正式实验、分析、维护和轻量试验脚本 | 已建立目录边界 |
| [`data/`](data/README.md) | 正式设计实验/仿真批次的数据包与追溯清单 | 原始与派生大数据默认不入 Git |
| [`docs/workflow/`](docs/workflow/README.md) | ROADMAP、Phase 规则、索引和模板 | 人工工作流入口 |

第三方库、生成代码和构建输出不承担仓库导航职责，不为其逐层维护 README。

## 人工 Phase 工作流

项目不使用 GSD。统一入口为 [工作流说明](docs/workflow/README.md)：

```text
ROADMAP 选定 Phase
        ↓
PLAN：冻结范围、任务、接口和验收
        ↓
实现与真实验证
        ↓
REVIEW：PASS 或 REWORK
        ↓
RECORD：仅在 PASS 后记录真实结果
        ↓
ROADMAP 标记 complete
```

Phase 状态固定为 `planned → active → review → complete`；无法继续时使用 `blocked`。

## 知识工具职责

- **CBM（Codebase Memory）**：查询当前源码中的文件、符号、调用关系、数据流和影响面。当前代码结构以 CBM 和实际源码为准。
- **Graphify**：查询设计文档、Phase 记录、实验结论和工程脚本之间的历史关系。它不替代 live code discovery。

本次 README 建设不更新 CBM 索引，也不重建 Graphify 图；索引刷新应作为独立维护动作执行。

## 文档维护原则

- README 只描述稳定职责、边界、入口和当前能力，不维护逐任务进度。
- 阶段状态只在 `docs/workflow/ROADMAP.md` 与对应 Phase 文档中维护。
- 新的运行命令必须在真实执行通过后才写入 README。
- 技术结论必须链接到代码、仿真输出、实验记录或审查证据。
- 只有会影响技术结论的正式设计实验才建立 `data/experiments/` 数据包；轻量画图、快速测试和探索性试验不强制走完整流程。
