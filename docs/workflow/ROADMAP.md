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
- 真机迁移：Phase 14 MuJoCo-only Gate B 已 PASS；当前按用户决定冻结所有真机上电、板级联调、传感器采集和辨识执行，Phase 05 保留已有实现与计划但不继续执行。

## 当前路线决策：两轮复现与非覆盖

- 第一轮使用当前 nominal MuJoCo 模型，继续完成可独立验证和复用的纯仿真工作；总体次序为：完整闭链运动学/Jacobian 补强 → Controller↔MuJoCo 闭环 → Joint PD/重力补偿 → 轮地接触与 floating-base → 简单站立 → WBC → NMPC。前三项已由 Phase 15/16/17 完成，下一层是尚未制定详细 Phase 的轮地接触与 floating-base plant 验证。任何 simulation-only PASS 都不能写成真机 PASS。
- 第二轮在真机工作解冻后执行 MuJoCo–真机共同辨识；形成新的 identified plant profile 后，按第一轮相同的输入契约、runner、日志 schema、阈值口径和控制层次从头重跑。第二轮是对第一轮的复现与比较，不替换第一轮。
- 每个后续阶段都必须把模型版本、参数 profile、Controller 版本、求解器配置、seed/激励、阈值和输入文件 hash 写入 manifest；运行输出进入新的带日期/模型 ID 的目录。已完成 Phase 的 PLAN/REVIEW/RECORD 和正式 evidence 不原地覆盖，修订通过新 Phase、新 run 或带 `supersedes` 关系的记录追加。
- 当前 `wheel_leg.xml` 与 Phase 14 evidence 作为 nominal baseline 保留。后续 SolidWorks 调整髋部电机或连接件尺寸并重新导出时，必须建立新的模型 revision，保留旧导出和 hash；重新检查 joint/body/site 名称与拓扑、frame/axis/zero offset、closure、collision、mass/COM/inertia，并重跑 Adapter、运动学和内部动力学回归。接口不变时控制与验证入口应直接复用，但不能假定几何和惯量结果自动不变。

## 阶段路线

| 顺序 | 阶段 | 状态 | Phase | 放行条件/证据 |
| --- | --- | --- | --- | --- |
| 01 | 迁入 Simulink 基线与验证入口 | complete | [Phase 01](phases/01-simulink-baseline-import/PLAN.md) | 基线模型、运行方式和当前验证结果可复现 |
| 02 | 坐标系、单位、关节顺序与接口语义 | complete | [Phase 02](phases/02-coordinate-interface-contract/PLAN.md) | FLU canonical、Simscape/MuJoCo 映射、COM frame 与 joint sign 契约通过审查；真机安装验证转 Phase 06 |
| 03 | 统一 Robot 接口与 Controller Core 骨架 | complete | [Phase 03](phases/03-robot-interface-controller-core/PLAN.md) | C++ Core、聚合消息、ROS2 wrapper 与 Jazzy pub/sub 测试通过 |
| 04 | MuJoCo 基础模型与 Adapter | complete | [Phase 04](phases/04-mujoco-model-adapter/PLAN.md) | MuJoCo 3.7.0 状态/零力矩闭环、fixed/floating sanity、映射与 fail-safe 通过审查 |
| 05 | MuJoCo 运动学与内部动力学验证 | complete | [Phase 14](phases/14-mujoco-internal-dynamics-validation/PLAN.md) | 不接真机；FK/Jacobian、重力、M(q)、正逆动力学、约束、耦合、能量与开环回放自洽并通过审查 |
| 06 | 完整闭链运动学、接触点与 Jacobian 验证 | complete | [Phase 15](phases/15-mujoco-closed-chain-kinematics/PLAN.md) | 210 样本 nominal 装配分支、被动解、工作域、reduced Jacobian、有限差分、速度与虚功通过 REVIEW；入口可跨 revision 非覆盖复用 |
| 07 | Controller ↔ MuJoCo 确定性闭环运行基线 | complete | [Phase 16](phases/16-controller-mujoco-deterministic-loop/PLAN.md) | 不接真机、不新增控制算法；2 ms/10 ms/5-step fixed loop、双时钟/reset/fail-safe、逐 tick 日志、replay 与非覆盖 gate 通过 REVIEW |
| 08 | nominal Joint PD 与重力补偿 | complete | [Phase 17](phases/17-nominal-joint-pd-gravity-compensation/PLAN.md) | 不接真机；解析 reduced gravity + canonical Joint PD 已通过双 oracle、保持、正负阶跃、限幅、扰动、对称与 replay 审查 |
| 09 | nominal 轮地接触与 floating-base plant 验证 | planned | — | 不接真机；normal/rolling/lateral/friction、滑移趋势、base state 和 reset 在冻结工况内通过，不提前做站立 |
| 10 | nominal floating-base 简单站立 | planned | — | 不接真机；在已验证 contact 与 joint control 上完成 z/pitch/leg posture/wheel position 的仿真站立 |
| 11 | nominal Weighted WBC | planned | — | 不接真机；约束、任务、软接触和限幅逐层通过，保留可对 identified profile 重跑的入口 |
| 12 | nominal NMPC | planned | — | 不接真机；NMPC → WBC → torque 全链路在 nominal profile 中通过并保存非覆盖证据 |
| 13 | 执行器力矩映射、摩擦与附加惯量 | blocked | [Phase 05](phases/05-actuator-torque-identification/PLAN.md) | 当前冻结真机；解冻后仍须关闭 Phase 05 自身 DG01–DG06，才能执行真实辨识与 MuJoCo 对应验证 |
| 14 | RobotState 与传感器正式验证 | planned | — | 真机解冻后验证时间戳、单位、方向、滤波和延迟，形成 identified/real 可用状态边界 |
| 15 | MuJoCo–真机运动学、重力、质量与 COM 辨识 | planned | — | 复用 Phase 14/15 基线，FK/Jacobian/重力矩及 mass/COM 得到模型与实验支持 |
| 16 | MuJoCo–真机完整惯量、动力学耦合与接触辨识 | planned | — | 复用 Phase 14/15 激励与分析，关键动力学和接触趋势在预定误差内一致 |
| 17 | identified profile 分层复现与三方比较 | planned | — | 使用同一 runner/schema/阈值从运动学到 NMPC 追加重跑，保留 nominal ↔ identified ↔ real 对照，不覆盖第一轮 |
| 18 | Roll/Yaw/Turning 与差分辨识 | planned | — | 在前述两轮证据基础上验证工作范围与鲁棒裕量 |

README 与工作流骨架属于仓库引导建设，不作为产品开发 Phase。Phase 14/15/16/17 已完成；Phase 05 因当前真机冻结而 blocked。Phase 15/16/17 的 PASS 都只属于 simulation-only；恢复 Phase 05 时，它们不替代通信、Load Cell、同步和安全放行条件。

详细技术次序以 [MuJoCo → Real 当前更新路线](../mujoco/simulink%202%20mujoco%202%20real流程.md) 为准。建立真实 Phase 后，用 Phase 链接替换表中的“—”。

## 维护规则

- 每次只修改真实发生变化的状态，不预先填写 PASS。
- `complete` 必须同时满足 REVIEW=PASS、RECORD 已写和证据链接有效。
- 阶段拆分或重排时保留原编号的历史含义，不静默复用编号。
- “顺序”表示当前执行次序，Phase 编号是稳定 ID；重排时两者可以不同。
- 发现需要改变状态、输入、模型或控制架构时，先建立技术决策任务，再继续实现。
- 模型、参数、配置和 evidence 采用追加式版本管理；新 revision/new run 不覆盖已获批 baseline，跨 revision 比较必须能同时解析两边 manifest。
