# 项目 ROADMAP

本文件是阶段状态的唯一总台账。技术细节、任务执行和审查内容写入对应 Phase；本文件只维护阶段顺序、状态、依赖和链接。

## 状态定义

- `planned`：已列入路线，尚未开始。
- `active`：正在设计、实现或验证。
- `review`：实现已停止扩张，等待或正在审查。
- `complete`：REVIEW 为 PASS，RECORD 已完成。
- `blocked`：存在明确阻塞条件，无法继续。

## 当前总体状态

- Simulink 控制仿真：接近验证完成，尚未迁入仓库。
- MuJoCo：尚未落地可运行工程。
- ROS2：存在不完整的消息转换骨架，不能视为完整 workspace。
- STM32：已有固件和 UART2 实验通信实现；生产链路尚未冻结。
- 真机迁移：按照 MuJoCo PASS → 真机低风险验证的方式逐层推进。

## 阶段路线

| 顺序 | 阶段 | 状态 | Phase | 放行条件/证据 |
| --- | --- | --- | --- | --- |
| 01 | 迁入 Simulink 基线与验证入口 | planned | — | 基线模型、运行方式和当前验证结果可复现 |
| 02 | 坐标系、单位、关节顺序与接口语义 | planned | — | Simulink、MuJoCo、Controller、真机物理语义一致 |
| 03 | ROS2 公共类型与 Controller Core 骨架 | planned | — | 纯 C++ 核心边界和聚合消息契约通过测试 |
| 04 | MuJoCo 基础模型与 Adapter | planned | — | 状态/命令闭环通路可运行，基础语义检查通过 |
| 05 | 执行器力矩映射、摩擦与附加惯量 | active | [Phase 05](phases/05-actuator-torque-identification/PLAN.md) | 3508+C620 先跑通；各执行器真实辨识完成，MuJoCo 与实验在预先规定误差内一致 |
| 06 | RobotState 与传感器正式验证 | planned | — | 时间戳、单位、方向、滤波和延迟满足控制要求 |
| 07 | 运动学、重力、质量与 COM | planned | — | FK/Jacobian/重力矩得到模型与实验支持 |
| 08 | 完整惯量、动力学耦合与接触 | planned | — | MuJoCo 与真机关键动力学和接触趋势一致 |
| 09 | Joint PD 与重力补偿 | planned | — | 单关节、整腿及扰动恢复在两端通过 |
| 10 | Floating-base 简单站立 | planned | — | 低风险保护条件下稳定站立 |
| 11 | Weighted WBC | planned | — | 约束、任务和限幅逐层验证通过 |
| 12 | NMPC | planned | — | NMPC → WBC → torque 全链路稳定 |
| 13 | Roll/Yaw/Turning 与差分辨识 | planned | — | 工作范围与鲁棒裕量有真实证据支持 |

README 与工作流骨架属于仓库引导建设，不作为产品开发 Phase。当前优先启动 Phase 05 的拆机执行器辨识；其台架实验可独立于前序软件迁移 Phase 推进，MuJoCo 对应验证仍依赖 Phase 04。

详细技术次序以 [MuJoCo → Real 当前更新路线](../mujoco/simulink%202%20mujoco%202%20real流程.md) 为准。建立真实 Phase 后，用 Phase 链接替换表中的“—”。

## 维护规则

- 每次只修改真实发生变化的状态，不预先填写 PASS。
- `complete` 必须同时满足 REVIEW=PASS、RECORD 已写和证据链接有效。
- 阶段拆分或重排时保留原编号的历史含义，不静默复用编号。
- 发现需要改变状态、输入、模型或控制架构时，先建立技术决策任务，再继续实现。
