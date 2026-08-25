# Phase 03: 统一 Robot 接口与 Controller Core 骨架 — PLAN

Status: `complete`

## Goal

冻结并实现一个不依赖 ROS2、MuJoCo 或硬件传输的 `RobotState` / `TorqueCommand` 公共边界和 Controller Core 可构建骨架，使后续 MuJoCo Adapter 与 Hardware Adapter 能以同一物理语义接入，并用真实编译、转换与契约测试证明接口一致。

## Current State

- 已有：Phase 02 已冻结 canonical world 为 FLU、base control frame 为机身 COM 候选、active quaternion 为 scalar-first `[w,x,y,z]`、公共六关节顺序为 `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`，以及 MuJoCo/Controller 的关节相对符号关系。
- 已有：Simulink baseline 中存在 legacy 16-state、控制器专用输入 pack、接触量和 WBC/NMPC 相关信号，但这些内部数组尚未整理为跨平台公共类型。
- 已有：[`ros_ws/README.md`](../../../../ros_ws/README.md) 已冻结“同一 Controller Core + MuJoCo/Hardware Adapter + 聚合边界”的目标架构。
- 开工前代码事实：`ros_ws/src/` 只有 Phase 05 的自包含实验包 `wheel_leg_stm32_bridge`；早期 `wheel_leg_bridge` 并不存在。执行后已新增 `wheel_leg_core`、`wheel_leg_msgs` 与 `wheel_leg_ros`；Adapter 仍留给后续 Phase。
- 当前工具链：执行环境为 Ubuntu 24.04、CMake 3.28.3、GNU C++ 13.3.0 与 ROS2 Jazzy；纯 C++ 和完整 ROS workspace 均已在本 Phase 真实验证。
- 历史架构证据：[`ros2 架构.md`](../../../mujoco/ros2%20架构.md) 将 `RobotState`、Controller Core、`TorqueCommand`、MuJoCo Adapter 和 Hardware Adapter 定义为同一条边界链；其字段草案只能作为输入，不能替代本 Phase 的精确契约。

## Scope

- Ground Simulink baseline 中实际被 Controller 消费的状态、参考量、接触量和输出量，形成“原生信号 → canonical 公共字段 → legacy 兼容 pack”的可追溯映射。
- 冻结 `RobotState` 与 `TorqueCommand` 的精确字段、类型、数组顺序、单位、frame/origin、时间戳、有效性和有限值要求。
- 冻结 Controller Core 的调用、复位、时间推进、无效输入和安全输出语义；建立不含控制算法的可构建骨架。
- 建立 ROS 无关的纯 C++ 公共类型与契约校验；Controller Core 不包含 ROS、MuJoCo、串口或 CAN 依赖。
- 建立对应的 ROS2 聚合消息和显式转换层；处理 C++ `[w,x,y,z]` 与 ROS quaternion `[x,y,z,w]` 的排列差异。
- 建立纯 C++ 单元测试、消息转换 round-trip 测试、编译依赖检查和最小 ROS2 pub/sub 集成测试。
- 更新 `ros_ws` 的目录说明、package 入口、构建验证条件和后续 Adapter 接入规则。

## Out of Scope

- 移植 Planner、NMPC、WBC、Joint PD、重力补偿或实现任何能宣称控制效果的算法。
- 冻结高层运动参考/遥控命令的最终 schema；本 Phase 只保证以后可在不破坏 RobotState/TorqueCommand 的前提下增加该输入。
- 实现 MuJoCo Adapter、Hardware Adapter、树莓派—STM32 生产协议或部署 profile。
- 决定生产通信中的 enable、e-stop、watchdog、重传和诊断 schema；这些属于系统安全/传输层，不混入 Controller Core 的物理力矩输出。
- 标定 MuJoCo joint zero offset、修改模型 joint axis，或执行真机关节/IMU 方向验证；MuJoCo offset 属于 Phase 04，真机验证属于 Phase 06。
- 以零力矩骨架、成功编译或消息回环证明机器人控制算法、模型或真机已经通过。

## Frozen Decisions

- 公共边界采用 Phase 02 的 canonical `{N}`：X 前、Y 左、Z 上，SI 单位；所有位置、速度、角速度、力矩字段都必须注明点、原点和表达 frame。
- 公共关节顺序固定为 `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`；闭链辅助 joint 不进入公共接口。
- 公共 C++ 姿态采用 active `q_N_from_B=[w,x,y,z]`；ROS message 的 `geometry_msgs/Quaternion` 排列为 `[x,y,z,w]`，只允许在命名转换函数中重排。
- Controller Core 与公共物理类型必须是 ROS 无关的普通 C++；ROS message、node、QoS 和 clock 类型停留在 wrapper/Adapter 边界。
- `RobotState` 是估计后的 canonical 控制状态，不是 IMU/encoder 原始帧集合；原始传感字段和驱动诊断保留在 Adapter/诊断层。
- `TorqueCommand` 表达六个 canonical 关节的期望输出轴力矩，单位 `N·m`。enable、e-stop、transport sequence 和驱动器原始电流不是 Controller Core 力矩向量的一部分。
- Controller Core 初始骨架在没有已迁移且已验证算法时只允许产生显式安全零力矩；不得用临时 PD、直通或随机输出填充“骨架”。
- Phase 05 的 `NormalState` / `NormalCommand` 是实验通信消息，不是本 Phase 公共 schema 的兼容基线。
- 两个 Adapter 必须复用同一个 C++ 类型和转换契约，不能各自复制结构体或维护不同关节顺序。

## Open Questions / Decision Gates

- **DG01 / CLOSED — RobotState 最小充分集：** torso-COM base pose/twist、六关节 `q/dq` 和左右三态 contact；raw IMU、加速度、反馈力矩和诊断不进入 Core。
- **DG02 / CLOSED — Twist 与接触语义：** base twist 是 `B` 原点相对 `{N}` 的速度，linear/angular 均表达在 `{N}`；contact 为 `[left,right]` observation/estimate，允许 `unknown`。
- **DG03 / CLOSED — 时间与有效性：** host steady-clock ns、无 sequence；finite/quaternion/contact、future/stale/non-monotonic checks 和 accepted-history rule 已冻结并测试。
- **DG04 / CLOSED — Core 生命周期：** `configure/reset/step(state,now)`；首样本 `dt=0`，后续从 accepted timestamps 得到；错误用 `StepStatus` 返回。
- **DG05 / CLOSED — 消息与 package 边界：** `wheel_leg_core`, `wheel_leg_msgs`, `wheel_leg_ros`，依赖单向指向 ROS wrapper。
- **DG06 / CLOSED — ROS2 构建证据：** ROS2 Jazzy 完整 workspace build/test 与 pub/sub 集成测试通过，见 automated evidence。
- **DG07 / CLOSED — 用户确认：** 用户明确要求执行本 PLAN；交付保持接口与安全零输出骨架，未迁移控制算法。

## Interfaces and Compatibility

- 输入：Adapter 产生的单样本 canonical `RobotState`；调用方提供明确的单调采样时间。高层运动参考接口暂不冻结。
- 输出：六关节 canonical `TorqueCommand`，顺序固定、单位 `N·m`；骨架输出必须为有限值且默认为全零。
- base 候选字段：`base_control_frame`（机身 COM 候选）的 world pose 和 twist；精确 twist 表达 frame 由 DG02 关闭后写入接口规格。
- joint 候选字段：六个 canonical joint position `[rad]` 与 velocity `[rad/s]`；Adapter 负责 Phase 02/04 定义的 native sign/offset 映射。
- contact 候选字段：left/right 两侧接触观测及未知/无效语义；不得把仿真 ground truth 无条件假设为真机可用输入。
- ROS 边界：聚合 message 与 C++ 类型一一映射；所有数组长度、quaternion 排列、frame/order 和 finite checks 必须由测试覆盖。
- legacy 兼容：Simulink 16-state 和控制器内部 pack 只能经显式命名 Adapter 转换；legacy `[前,右,上]` 不能被命名为三维 frame。
- 必须保持：Phase 02 坐标、单位、关节顺序、四元数和 joint sign 契约；Phase 05 实验 package 的现有行为不得因本 Phase 静默改变。
- 允许改变：`ros_ws` 中尚不存在的 package 布局、公共类型和测试入口；可修正 README 中指向缺失 package 的陈旧描述。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground Simulink 控制边界与当前 ROS 资产 | Phase 01/02 证据、baseline 源码、ROS package 现状 | `evidence/interface_grounding.md`：消费者、字段、frame、单位、速率、缺失资产和不可复用实验接口 | CBM/源码引用可追溯；每个公共候选字段至少对应真实消费者或明确后续依赖 | done |
| T02 | 关闭 DG01–DG03，冻结公共数据契约 | T01、坐标契约、后续 Adapter 约束 | `docs/interfaces/robot_state_torque_command.md`：字段表、不变量、时间/有效性和示例 | 逐字段审查；维度、单位、frame/origin/order、有效/未知状态无空白 | done |
| T03 | 关闭 DG04–DG05，冻结 Core 与 package 结构 | T01/T02、目标部署架构 | Core lifecycle/API 规格、依赖方向图、package 清单和兼容策略 | 无 ROS→Core 反向依赖；接口可容纳后续多率算法且无未定义 `dt` | done |
| T04 | 建立纯 C++ types 与契约校验 | T02/T03 | 公共 headers/library、finite/quaternion/order/time validation | 本机 CMake configure/build/CTest；告警作为错误；边界与非法输入测试 | done |
| T05 | 建立 Controller Core 安全骨架 | T03/T04 | 可 reset/step 的 Core 骨架和显式状态/错误结果 | 重复运行确定性；有效输入全零输出；无效/乱序输入按契约拒绝且不输出非零力矩 | done |
| T06 | 建立 ROS2 聚合消息与转换层 | T02/T04 | message package、conversion library、必要 package metadata | C++↔ROS round-trip；quaternion 重排、数组顺序、时间转换和非法值测试 | done |
| T07 | 建立最小 Controller ROS wrapper | T03/T05/T06 | 订阅 RobotState、调用同一 Core、发布 TorqueCommand 的最小 node/component | 无算法情况下只发布安全零力矩；重复/过期/无效 state 行为符合契约 | done |
| T08 | 完成跨环境自动验证 | T04–T07、ROS2 Jazzy 环境 | 纯 C++ CTest 与 ROS2 `colcon build/test` 的真实输出证据 | 全部测试通过；依赖检查确认 Core 无 ROS/MuJoCo/硬件依赖；最小 pub/sub 测试通过 | done |
| T09 | 更新入口文档并准备审查 | 全部交付物和验证输出 | `ros_ws`/package README、Execution Notes、证据索引和 Phase REVIEW 输入 | 从文档入口可复现构建/测试；不存在对缺失 package 或未验证能力的陈述 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- 纯 C++：在当前 Windows 工具链使用 repository 提供的 CMake preset/明确命令 configure、build 和 `ctest --output-on-failure`；覆盖默认值、数组顺序、有限值、quaternion norm、时间单调、reset 和零输出确定性。
- 契约性质测试：构造 canonical 状态，经 C++→ROS→C++ round-trip 后逐字段一致；姿态比较处理 `q` 与 `-q` 等价，但排列错误必须导致测试失败。
- ROS2 Jazzy：在真实环境执行 `colcon build --symlink-install`、`colcon test` 和 `colcon test-result --verbose`；精确命令在环境落地后记录，不在 PLAN 中伪造已经可用的 workspace setup。
- 最小集成：发布一个有效 RobotState，wrapper 输出六关节全零 TorqueCommand；再注入错误数组/非有限值/过期或乱序样本，验证拒绝、诊断和无非零输出。
- 依赖边界：自动检查纯 C++ package 不链接/包含 `rclcpp`、ROS messages、MuJoCo、serial/CAN；ROS conversion 和 wrapper 只能单向依赖 core/types。
- 兼容映射：使用 Phase 02 的坐标/关节顺序 fixture 验证 Simulink legacy pack 与 canonical 字段转换，不允许重复换轴或左右镜像。

### Manual / Evidence

- 在实现前由用户确认 DG07：Phase 03 是接口和安全骨架，不包含控制算法迁移；确认记录写入 Execution Notes。
- 审查 `RobotState` 字段表，逐项回答“测量/估计对象、原点、表达 frame、单位、采样时间、无效/未知如何表示”；任一项缺失则不放行实现。
- 在 ROS2 Jazzy 环境保存编译器、ROS distro、命令、摘要和失败日志位置；没有真实 `colcon` 证据时 Phase 保持 active/review REWORK。
- 后续 Phase 04 用 MuJoCo Adapter 接入相同消息，Phase 06 用 Hardware Adapter 接入相同消息；本 Phase 不预判两端物理证据 PASS。

## Acceptance Criteria

- [x] T01 grounding 覆盖 Simulink 实际控制消费者、当前 ROS 资产和历史草案差异，不以缺失代码为事实来源。
- [x] `RobotState` / `TorqueCommand` 精确 schema 已冻结，所有字段都有类型、单位、frame/origin、顺序、时间和有效性语义。
- [x] ROS 无关 C++ types 与 Controller Core 骨架已实现并通过本机 CMake/CTest；Core 没有 ROS、MuJoCo 或硬件传输依赖。
- [x] Controller Core 在未迁移控制算法时只产生显式安全零力矩；无效、过期或乱序输入不会产生非零命令。
- [x] ROS2 聚合消息、转换层和最小 wrapper 已在真实 ROS2 Jazzy 环境通过 build/test 与 pub/sub 集成测试。
- [x] quaternion 排列、canonical/legacy 映射、六关节顺序和 finite/time checks 有自动回归测试。
- [x] 文档、package metadata、源码和测试使用同一契约；README 不再声称缺失 package 可用。
- [x] DG01–DG07 全部关闭，无未解决 blocking finding；控制算法、MuJoCo Adapter 和 Hardware Adapter 没有被误标为本 Phase 已交付。

## Execution Notes

- 2026-08-25：创建 Phase 03 PLAN，状态保持 `planned`；本次只完成范围、冻结约束、decision gates、任务和验证设计，未开始源码实现。
- 2026-08-25：当前代码索引与文件系统均确认 `ros_ws/src/` 仅有 `wheel_leg_stm32_bridge`；`ros_ws/README.md` 中的 `wheel_leg_bridge` 链接是陈旧目标描述，登记到 T01/T09 处理。
- 2026-08-25：当前 Windows 环境检测到 CMake/MinGW，未检测到 ROS2、`colcon` 或 Docker；纯 C++ 与 ROS2 证据分层，DG06 在真实 Jazzy 环境验证前保持开放。
- 2026-08-25：历史图谱确认目标数据流为 `MuJoCo/Hardware Adapter → RobotState → Controller Core → TorqueCommand → Adapter`；历史字段仅用于 grounding，不直接冻结 schema。
- 2026-08-25：用户明确要求执行本 Phase，DG07 关闭；实现范围保持“接口 + 安全零输出 Core 骨架”，不迁移 Simulink 控制算法。
- 2026-08-25：T01–T03 完成。DG01–DG05 由 `evidence/interface_grounding.md` 与 `docs/interfaces/robot_state_torque_command.md` 关闭；开始 T04 实现。
- 2026-08-25：T04–T09 完成；独立 CTest 1/1，ROS2 Jazzy workspace 4 packages 构建成功，`colcon test-result` 为 11 tests、0 failures，依赖边界检查 PASS。
- 2026-08-25：REVIEW 无 blocking finding，Verdict=`PASS`；RECORD 已创建，Phase 状态进入 `complete`。

## Blockers

None. Phase 当前可进入 T01–T03；开始 T04 实现前必须先关闭 DG01–DG05 并获得 DG07 的用户确认。
