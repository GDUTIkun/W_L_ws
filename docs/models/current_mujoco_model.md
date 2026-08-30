# 当前 MuJoCo 轮腿模型（Model B）

Status: `current nominal simulation model`

本文说明当前 Controller 与 MuJoCo 联调实际使用的模型。它不是
`simulation/simulink_baseline/` 中的旧 Simulink/Simscape 模型；后者只保留为算法与历史行为对照，
不定义本文的 plant 拓扑、状态维度或接触动力学。

## 1. 权威入口与适用范围

- 场景入口：`simulation/mujoco/model/scene_axisymmetric_centered_com_v1.xml`
- 机器人 MJCF：`simulation/mujoco/model/wheel_leg_axisymmetric_centered_com_v1.xml`
- Controller 动力学：`ros_ws/src/wheel_leg_core/src/nominal_wbc_model.cpp`
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
