# 当前 MuJoCo 轮腿模型（Model B）

Status: `current nominal simulation model`

本文说明当前 Controller 与 MuJoCo 联调实际使用的模型。它不是
`simulation/simulink_baseline/` 中的旧 Simulink/Simscape 模型；后者只保留为算法与历史行为对照，
不定义本文的 plant 拓扑、状态维度或接触动力学。

## 1. 权威入口与适用范围

- 场景入口：`simulation/mujoco/model/scene_axisymmetric_centered_com_v1.xml`
- 机器人 MJCF：`simulation/mujoco/model/wheel_leg_axisymmetric_centered_com_v1.xml`
- Controller 动力学：`ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp`
- 下层 QP：`ros_ws/src/wheel_leg_core/src/weighted_wbc_problem.cpp`
- 下层求解与诊断：`ros_ws/src/wheel_leg_core/src/weighted_wbc_controller.cpp`
- current reference/safety：`ros_ws/src/wheel_leg_core/src/controller_core.cpp`
- Controller/plant 公共类型：`ros_ws/src/wheel_leg_core/include/wheel_leg_core/types.hpp`
- MuJoCo 边界：`ros_ws/src/wheel_leg_mujoco/src/adapter.cpp`

“Model B”指轴对称圆柱轮碰撞、轮体质心位于轮轴的 nominal 模型。MJCF 是 MuJoCo plant
的权威定义；`NominalWbcModel` 是 Controller 内部与其对应的约束降维刚体模型。二者是需要持续
交叉验证的两个实现，不应视为同一份代码自动生成的模型。

本文只描述 nominal 平地仿真模型。它不是经过真机辨识的数字孪生，也不批准电机、减速器、轮胎、
结构柔性、传感器噪声或通信延迟模型。

## 2. MuJoCo plant

### 2.1 拓扑与维度

机器人由一个 6-DoF floating base 和左右两套闭链轮腿组成。每侧包含 hip、knee、wheel 三个驱动
hinge，以及 connect1、connect2 两个被动 hinge；connect2 与 calf 通过 equality connect 闭合。

编译后的 nominal plant 契约为：

| 量 | 维度 | 含义 |
| --- | ---: | --- |
| `nq` | 17 | floating-base position/quaternion + 10 个 hinge position |
| `nv` | 16 | 6 个 base velocity + 10 个 hinge velocity |
| `nu` | 6 | 左右 hip、knee、wheel 的直接力矩输入 |

公共驱动顺序固定为
`[left_hip, left_knee, left_wheel, right_hip, right_knee, right_wheel]`。
四个 connect joint 只用于闭链实现，不进入公共 `RobotState` 或 `TorqueCommand`。

### 2.2 几何、惯量与接触

- visual geometry 保留 CAD mesh；动力学质量与惯量由 MJCF 中的 geom/inertial 定义。
- `base_control_frame` 与 CAD body frame 平行，原点位于当前 nominal 机身 COM：
  `[-0.077378152, 0.000000810, -0.032277680] m`（在 base CAD frame 表达）。
- 左右轮的碰撞体是半径 `0.05 m`、半宽 `0.02 m` 的 cylinder；轮体质心位于轮轴。
- 场景是 Z-up 平面，重力 `[0, 0, -9.81] m/s²`，步长 `0.002 s`，
  `implicitfast` 积分器，floor contact `condim=3`。

这些质量、COM 和惯量来自当前 MJCF/CAD nominal 参数，不代表已经由真机标定确认。

## 3. Canonical controller 边界

Canonical world `{N}` 是右手 FLU：X 前、Y 左、Z 上；当前 MuJoCo world 与 `{N}` 同轴。
姿态使用 active quaternion `[w,x,y,z]`。base state 取自 `base_control_frame` site，而不是 CAD 原点、
整机 subtree COM 或尚未定义的真实 IMU frame。

Adapter 输出：

- base position、world linear velocity、world angular velocity；
- 6 个 canonical 驱动关节的位置和速度；
- 左、右轮与 floor 的离散 contact state。

MuJoCo joint 与 canonical joint 的符号关系为

```text
q_C   = -q_M + b_joint
dq_C  = -dq_M
tau_M = -tau_C
```

其中 `b_joint` 是 matching-pose 冻结的 nominal offset。Adapter 还对命令 receipt age 和 source
sample age 执行 watchdog；失效或超时时向 MuJoCo 写零力矩。

## 4. Controller 内部动力学

### 4.1 12 维 reduced coordinates

WBC 不直接优化 MuJoCo 的 16 维 tree acceleration，而使用

```text
nu = [v_base_N(3), omega_base_N(3), dq_canonical(6)]
```

共 12 维。闭链的 4 个被动速度由状态相关 reduction matrix `N(q)` 重建：

```text
qdot_tree = N(q) nu
```

加速度映射不是纯线性。闭链运动学 bias 必须保留：

```text
qdd_tree = N(q) nudot + c_N(q, nu)
```

因此 reduced dynamics 为

```text
M_r nudot + h_r = S_r tau + G_L w_L + G_R w_R
M_r = N^T M_tree N
h_r = N^T (h_tree + M_tree c_N)
```

`NominalWbcModel` 先由主动关节状态重建被动闭链姿态，检查 closure residual、被动 Jacobian
最小奇异值和条件数，再生成 `M_r`、`h_r`、actuation、contact Jacobian/bias 与 wrench maps。

### 4.2 接触量的比较空间

每侧接触 wrench 顺序为 `[Fx,Fy,Fz,Tx,Ty,Tz]`，在该侧 contact frame 表达，并以接触点为原点。
Controller 与 MuJoCo 的接触作用只能在同一 reduced generalized-force space 比较：

```text
g_contact,r = N(q)^T J_tree(q)^T w
```

禁止把 12 维 reduced generalized force 任意“抬升”为 16 维 native force。若比较 native acceleration，
必须使用上一节的仿射映射并包含 `c_N`。

### 4.3 Current lower-level Weighted-WBC

当前 ROS runtime 通过 `ControllerCore::stepWeightedWbc` 调用
`WeightedWbcController`，使用 `WeightedWbcProfile::kNominal`。它是一个每 `0.010 s`
求解一次的 42 变量加权 QP；Phase 34--46 的 rolling/contact-response profiles 只是历史
regression/oracle，不是 current runtime 的下层控制器。

QP 的 physical solution 顺序为

```text
x = [nudot(12), tau(6), w_L(6), w_R(6), s_W(12)]
```

其中 `nudot` 是 12 维 reduced acceleration，`tau` 是六个 canonical 驱动关节力矩，
`w_L/w_R` 是左右接触 wrench 决策量，`s_W` 是 interaction-wrench fidelity 的 signed slack。
求解器内部对变量、动力学行和任务残差做尺度归一化，但 `physical_solution`、输出力矩和本文公式
均使用物理单位。

### 4.4 Hard constraints

current nominal QP 只有 reduced rigid-body dynamics 是等式约束：

```text
M_r nudot + h_r = S_r tau + G_L w_L + G_R w_R
```

其余 current hard bounds 为：

- 六轴力矩界：`[10,10,2,10,10,2] N·m`；
- 每侧 37 行接触 wrench-cone 不等式；
- reduced acceleration 界：base linear `10 m/s²`、base angular `20 rad/s²`、
  六个驱动关节 `50 rad/s²`。

实现为固定 `117 x 42` constraint matrix；current nominal profile 使用前 104 行（12 dynamics、
6 torque、74 wrench-cone、12 acceleration），其余 13 行保持 inactive，预留给历史诊断 profile。

需要特别区分：双轮 contact acceleration 在 current nominal profile 中是**软任务**，不是额外
hard equality。进入 QP 前，Controller Core 另有双轮必须接触、周期、位姿包络和 finite/solver/
hard-residual 检查。current 包络为相对 anchor 的 `|x|,|y| <= 0.02 m`、高度误差
`<= 0.01 m`、`|roll|,|pitch| <= 0.03 rad`、`|yaw| <= 0.05 rad`；任一失败会 latch safety，
且不会发布有效非零力矩。

### 4.5 Current 的七类软任务

所有任务都以归一化 least-squares residual 加入同一个目标，没有严格层级。若写成
`r_i = (y_i-y_i^ref)/sigma_i`，QP 最小化这些 `r_i` 的平方和，并加 `1e-6` 数值正则项。

| 任务 | 维度 | residual / reference | `sigma` |
| --- | ---: | --- | ---: |
| Contact acceleration | 6 | 左右各 3 维 `J_c nudot + Jdot_c nu -> 0` | `10 m/s²` |
| Base X hold | 1 | `nudot_x -> a_x^ref` | `10 m/s²` |
| Base height | 1 | `nudot_z -> a_z^ref` | `10 m/s²` |
| Base orientation | 3 | base angular acceleration `-> alpha^ref` | `20 rad/s²` |
| Leg posture | 4 | 左右 hip/knee acceleration `-> qdd_leg^ref` | `50 rad/s²` |
| Wrench fidelity | 12 | `W_WBC - W_ref - s_W -> 0` | force `50 N`；moment `2.5 N·m` |
| Slack penalty | 12 | `s_W -> 0` | force `50 N`；moment `2.5 N·m` |

因此“wrench fidelity”不是 hard command：当动力学、力矩界、wrench cone 或其他任务与
`W_ref` 竞争时，QP 可以选择非零 `s_W`。诊断恒按

```text
interaction_residual = W_WBC - W_ref - s_W
```

报告；`W_ref`、`W_WBC` 和 `s_W` 都是 Core 内部量，不增加 ROS public topic。

`WeightedWbcController::Task` 枚举还保留 wheel-vertical manifold、wheel-longitudinal tracking、
native wheel-rate 和 contact-consistent rolling 等历史任务，但它们在 current
`kNominal` profile 中不进入目标函数，不能计入上述七类 current tasks。current 也没有 base-Y
tracking task；Y 方向只受 dynamics/acceleration hard constraints 和上层 safety envelope 约束。

#### 与 Phase 46 historical runner 的区别

上表描述的是 Phase 47 冻结后的 **current ROS `kNominal` runtime**，不是 Phase 46 的实验
profile。Phase 46 最终 contact-response replay 使用
`WeightedWbcProfile::kPhase46MujocoContactResponse`，只激活以下五类软任务：

1. contact acceleration；
2. wheel-longitudinal (`xi`) tracking；
3. contact-consistent rolling；
4. interaction-wrench fidelity；
5. signed-slack penalty。

它不激活 current nominal 的 base-X、height、orientation、leg-posture 四类任务，也不激活
wheel-vertical manifold 或 native wheel-rate。除此之外，该 Phase 46 profile 还加入最多 12 行
MuJoCo primitive contact-response **hard equality**；这些 hard rows 是诊断 payload，不是软任务，
也没有进入 current ROS `kNominal` runtime。

### 4.6 Reference 的生成

current stand controller 在首次有效 state 上冻结 `x/y/yaw` anchor，并由 PD law 生成加速度参考：

```text
a_x_ref     = -9 (x-x_anchor) - 6 vx
a_z_ref     = -25 (z-0.3154399840) - 10 vz
alpha_ref   = -Kp_R e_R - Kd_R omega
qdd_leg_ref =  36 (q_leg_target-q_leg) - 12 dq_leg
```

其中 `Kp_R=[25,25,9]`、`Kd_R=[10,10,6]`；姿态目标是零 roll/pitch 和冻结 yaw。
`q_leg_target` 只包含左右 hip/knee：

```text
[-0.9719989158, 1.6393957459, -0.9833909356, 1.6394010277] rad
```

current fixed interaction-wrench reference 按
`[Fx,Fy,Fz,Tx,Ty,Tz]_L + [Fx,Fy,Fz,Tx,Ty,Tz]_R`、Controller body FLU、
以对应 wheel-body origin 为矩心、wheel 对 leg/base 的 follower wrench 定义：

```text
[-0.01460018, -0.00214471, 31.57222316,  6.93928728, 0.30721275,  0.00002386,
  0.01460018,  0.00214471, 31.54924084, -6.93345575, 0.39850336, -0.00002386]
```

未来若由上层 NMPC 提供 override，只替换同一 `W_ref`，不会改变 lower-WBC 的 42D 决策变量、
hard constraints 或任务语义。

### 4.7 求解、输出与失败语义

QP 使用 dense solver，absolute/relative tolerance 均为 `1e-8`，最大 `10000` iterations，并在
连续有效 tick 间 warm start。解算后重新检查 hard violation；超过 `2e-7`、非有限值、模型/QP/
solver rejection 或力矩越界都会 reset warm start 并触发上层 safety latch。成功时只取
`physical_solution[12:18]` 作为

```text
TorqueCommand.joint_torque_nm = tau
```

发送给 MuJoCo Adapter。接触 wrench、slack、task residual/cost、active-set margin 和 solver residual
只用于 Core/回归诊断，不是 plant 的额外输入。

## 5. 当前验证边界

Model B 已关闭旧 wheel mesh 的绝对角度接触伪影，并成为当前 nominal MuJoCo/WBC 对照入口；但这不等于
滚动控制已经通过。Phase 44 的 WBC-to-plant realization audit 仍为 `REWORK / P44-U`：多层诊断已经落地，
但部分耦合滚动工况的 authority/realization 判据尚未通过。因此当前可以声称的是“模型与诊断边界已定义”，
不能声称“闭环滚动稳定”“MuJoCo 等价于真机”或“接触参数已辨识”。

任何修改若涉及 wheel collision/COM、joint topology/sign/offset、闭链约束、质量惯量、接触设置或
`RobotState`/`TorqueCommand`，都必须同步检查 MJCF、Adapter、`NominalWbcModel` 和相应 Phase 验证。

## 6. 相关文档

- `docs/models/coordinate_frame_contract.md`：跨系统坐标、frame、joint sign 的完整契约。
- `docs/workflow/phases/39-idealized-nominal-wheel-architecture-revalidation/RECORD.md`：Model B 选择与放行记录。
- `docs/workflow/phases/44-wbc-to-plant-constrained-rolling-realization-audit/PLAN.md`：当前 WBC-to-plant 审核定义。
- `docs/workflow/ROADMAP.md`：当前阶段状态；状态不在本文重复维护。
