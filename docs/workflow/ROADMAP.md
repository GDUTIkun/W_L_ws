# 项目 ROADMAP

本文件是阶段状态的唯一总台账。技术细节、任务执行和审查内容写入对应 Phase；本文件只维护阶段顺序、状态、依赖和链接。

## 状态定义

- `planned`：已列入路线，尚未开始。
- `active`：正在设计、实现或验证。
- `review`：实现已停止扩张，等待或正在审查。
- `complete`：REVIEW 为 PASS，RECORD 已完成。
- `blocked`：存在明确阻塞条件，无法继续。

## 当前总体状态

- Simulink 控制仿真：平地验证基线已迁入并通过目标路径 smoke；terrain adaptation 仍未完成。
- MuJoCo：3.7.0 基础 Adapter 与 nominal plant 内部动力学已分别通过 Phase 04/14；参数与真机一致性、接触保真度和控制效果尚未验证。
- ROS2：canonical Core/messages/wrapper 与 MuJoCo Adapter 已通过 Jazzy build/test；Hardware Adapter 与树莓派部署 profile 尚未落地。
- STM32：已有固件和 UART2 实验通信实现；生产链路尚未冻结。
- 真机迁移：Phase 14 MuJoCo-only Gate B 已 PASS；可恢复 MuJoCo–真机辨识，但每项真机工作仍须先关闭对应计量、通信、同步与安全 gate。

## 阶段路线

| 顺序 | 阶段 | 状态 | Phase | 放行条件/证据 |
| --- | --- | --- | --- | --- |
| 01 | 迁入 Simulink 基线与验证入口 | complete | [Phase 01](phases/01-simulink-baseline-import/PLAN.md) | 基线模型、运行方式和当前验证结果可复现 |
| 02 | 坐标系、单位、关节顺序与接口语义 | complete | [Phase 02](phases/02-coordinate-interface-contract/PLAN.md) | FLU canonical、Simscape/MuJoCo 映射、COM frame 与 joint sign 契约通过审查；真机安装验证转 Phase 06 |
| 03 | 统一 Robot 接口与 Controller Core 骨架 | complete | [Phase 03](phases/03-robot-interface-controller-core/PLAN.md) | C++ Core、聚合消息、ROS2 wrapper 与 Jazzy pub/sub 测试通过 |
| 04 | MuJoCo 基础模型与 Adapter | complete | [Phase 04](phases/04-mujoco-model-adapter/PLAN.md) | MuJoCo 3.7.0 状态/零力矩闭环、fixed/floating sanity、映射与 fail-safe 通过审查 |
| 05 | MuJoCo 运动学与内部动力学验证 | complete | [Phase 14](phases/14-mujoco-internal-dynamics-validation/PLAN.md) | 不接真机；FK/Jacobian、重力、M(q)、正逆动力学、约束、耦合、能量与开环回放自洽并通过审查 |
| 06 | 执行器力矩映射、摩擦与附加惯量 | active | [Phase 05](phases/05-actuator-torque-identification/PLAN.md) | Phase 14 前置已 PASS；关闭自身 DG01–DG06 后，3508+C620/GIM6010 真实辨识与 MuJoCo 对应验证通过 |
| 07 | RobotState 与传感器正式验证 | planned | — | Phase 14 PASS 后才接真机；时间戳、单位、方向、滤波和延迟满足控制要求 |
| 08 | MuJoCo–真机运动学、重力、质量与 COM 辨识 | planned | — | 复用 Phase 14 基线，FK/Jacobian/重力矩及 mass/COM 得到模型与实验支持 |
| 09 | MuJoCo–真机完整惯量、动力学耦合与接触辨识 | planned | — | 复用 Phase 14 激励与分析，关键动力学和接触趋势在预定误差内一致 |
| 10 | Joint PD 与重力补偿 | planned | — | 单关节、整腿及扰动恢复在两端通过 |
| 11 | Floating-base 简单站立 | planned | — | 低风险保护条件下稳定站立 |
| 12 | Weighted WBC | planned | — | 约束、任务和限幅逐层验证通过 |
| 13 | NMPC | planned | — | NMPC → WBC → torque 全链路稳定 |
| 14 | Roll/Yaw/Turning 与差分辨识 | planned | — | 工作范围与鲁棒裕量有真实证据支持 |

README 与工作流骨架属于仓库引导建设，不作为产品开发 Phase。Phase 14 已完成；下一执行项恢复为 Phase 05，但 Phase 14 PASS 不替代 Phase 05 自身的通信、Load Cell、同步和安全放行条件。

详细技术次序以 [MuJoCo → Real 当前更新路线](../mujoco/simulink%202%20mujoco%202%20real流程.md) 为准。建立真实 Phase 后，用 Phase 链接替换表中的“—”。

## 维护规则

- 每次只修改真实发生变化的状态，不预先填写 PASS。
- `complete` 必须同时满足 REVIEW=PASS、RECORD 已写和证据链接有效。
- 阶段拆分或重排时保留原编号的历史含义，不静默复用编号。
- “顺序”表示当前执行次序，Phase 编号是稳定 ID；重排时两者可以不同。
- 发现需要改变状态、输入、模型或控制架构时，先建立技术决策任务，再继续实现。
