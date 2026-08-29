---
title: 原理论模型恢复与 Minimal WBC 验证方案
date: 2026-08-29
status: phase-plan-draft
tags:
  - wheel-legged-robot
  - NMPC
  - WBC
  - minimal-WBC
  - MuJoCo
  - architecture-validation
---

# 1. 本阶段目标

本阶段不继续沿用当前 MuJoCo 版本的 12-state locked-composite NMPC 作为最终上层结构，而是恢复原理论模型中的：

\[
\boxed{
\text{Wheel-position Planner}
+
\text{16-state NMPC}
+
\text{wheel-relative dynamics}
}
\]

同时保留当前 MuJoCo 路径中已经经过验证的 WBC / contact / solver 工程改进。

本阶段最终要回答的问题是：

> **当原理论上层结构恢复、NMPC→WBC interaction-wrench contract 被正确重建后，仅保留最小职责的 WBC，是否足以完成平地 nominal 闭环？**

本阶段只做到：

\[
\boxed{
\text{构建}
\rightarrow
\text{验证}
\rightarrow
\text{若失败则定位问题}
}
\]

如果 Minimal WBC 不能满足稳定性或性能指标：

- 本阶段不增加 height / pitch / rolling / leg posture / wheel-position 等补偿任务；
- 本阶段不通过调高额外 WBC task 权重“救稳”；
- 本阶段只确定失效发生在哪一层、表现为什么、最可能缺失什么控制能力；
- 是否增加必要低层任务留到下一 Phase 独立研究。

---

# 2. 设计原则

本阶段采用：

\[
\boxed{
\text{恢复理论上层}
+
\text{保留当前已验证底层}
+
\text{Minimal WBC}
}
\]

而不是整体退回旧 Simulink 实现。

## 2.1 恢复的部分

恢复：

1. common wheel-position planner；
2. 16-state NMPC；
3. NMPC 中的左右 wheel-position / wheel-speed state；
4. paper Eq.(12) wheel-relative-to-base dynamics；
5. 20 ms NMPC 周期；
6. 原理论中的 wheel-position reference semantics。

## 2.2 保留当前 MuJoCo 版本的部分

保留：

1. current nominal full-3D MuJoCo plant；
2. canonical `RobotState -> TorqueCommand` 边界；
3. 12-DoF reduced WBC model；
4. 当前 42D WBC decision vector；
5. 每轮 6D contact-centred wrench representation；
6. 当前 contact H-cone / friction feasibility；
7. 当前 rigid-body dynamics hard constraints；
8. torque bounds；
9. acceleration bounds / workspace fail-closed；
10. current ProxQP WBC solver；
11. fail-zero / latch / reset 安全语义；
12. MuJoCo 与 Controller Core 的 runtime separation。

明确不恢复旧 36D single-point-force WBC。

原因是旧 point-contact representation 在 MuJoCo 路径中已经暴露出接触合力矩表达不足的问题；后续 continuous surface-patch oracle 又证明多点接触的对外物理作用可以凝聚为每轮一个 6D contact-centred wrench，因此当前 42D / 6D-contact WBC 应作为已验证底层继续保留。

## 2.3 3D contact → 6D contact 不改变 NMPC 控制端口

需要严格区分两类量：

```text
NMPC：
每侧 6D interaction wrench
[Fx, Fy, Fz, Tx, Ty, Tz]

WBC 内部 contact variable：
旧版：每侧 3D point contact force
新版：每侧 6D contact-centred wrench
```

因此：

\[
\boxed{
\text{WBC contact variable: }3D\rightarrow6D
}
\]

并不意味着 NMPC 的输入必须随之改变。

恢复 Simulink 16-state NMPC 时，上层仍保留原理论中的左右 12D interaction-wrench input。本阶段真正需要重新建立的是：

\[
\boxed{
\text{NMPC interaction wrench}
\rightarrow
\text{WBC realized interaction wrench}
}
\]

这一层的物理合同，而不是让 NMPC 直接输出 WBC 的 contact-centred wrench。

---

# 3. 目标闭环架构

本阶段目标架构为：

```text
motion reference
      ↓
common wheel-position planner
      ↓
xi_c_ref
dxi_c_ref
ddxi_c_ref
      ↓
16-state NMPC
├─ 3D base dynamics
└─ wheel-relative Eq.(12) dynamics
      ↓
left/right 12D original interaction-wrench request
      ↓
NMPC→WBC Interaction-Wrench Contract
├─ freeze point / frame / sign / left-right order
├─ preserve original NMPC wrench semantics
├─ DO NOT reinterpret it as contact-centred wrench
└─ WBC reconstructs the actually realized interaction wrench
      ↓
Minimal 42D WBC
├─ full reduced rigid-body dynamics
├─ 6D contact-centred wrench / wheel
├─ hard contact / friction / torque constraints
├─ interaction-wrench realization + slack
├─ soft contact acceleration
└─ weak regularization
      ↓
six joint torques
      ↓
MuJoCo plant
```

核心职责划分为：

```text
Wheel planner：
生成共同轮位参考

NMPC：
决定 base + wheel-relative motion
并输出原理论语义下的左右 interaction wrench

NMPC→WBC interface：
冻结 point / frame / sign / order，
并定义“WBC 实际实现了多少 interaction wrench”；
它不是新的控制器，也不允许把 interaction wrench
直接改解释成 contact-centred wrench

Minimal WBC：
以 6D contact-centred wrench 作为内部接触变量，
在完整动力学、接触和执行器约束下
重构实际 interaction wrench，并尽量实现 NMPC request

MuJoCo：
提供完整 nonlinear plant response
```

---

# 4. Wheel-position Planner

恢复原理论中的共同轮位规划。

定义：

\[
\xi_c=\frac{\xi_L+\xi_R}{2}
\]

以及：

\[
\xi_\Delta=\frac{\xi_R-\xi_L}{2}.
\]

其中：

- \(\xi_L,\xi_R\) 是 wheel center 相对 base 的前向几何位置；
- 不是 wheel spin angle；
- 左右符号、frame 和差模定义必须重新通过当前 MuJoCo canonical state 验证。

planner 输出：

\[
\boxed{
\xi_c^d,\quad
\dot\xi_c^d,\quad
\ddot\xi_c^d
}
\]

左右 NMPC wheel-position reference 使用共同目标：

\[
\xi_L^d=\xi_R^d=\xi_c^d.
\]

本阶段不人为生成相反的左右 wheel-position reference 作为转向手段。

转向仍由左右 wrench 差动产生。

---

# 5. 16-State NMPC

## 5.1 状态

恢复：

\[
x=
\begin{bmatrix}
p_B^N\\
\phi\\
\theta\\
\psi\\
v_B^N\\
\dot\phi\\
\dot\theta\\
\dot\psi\\
\xi_L\\
\xi_R\\
\dot\xi_L\\
\dot\xi_R
\end{bmatrix}
\in\mathbb R^{16}.
\]

即：

```text
3  base position
3  base orientation
3  base linear velocity
3  orientation rates
2  wheel-relative position
2  wheel-relative velocity
-------------------------
16 states
```

---

## 5.2 输入

NMPC 仍使用左右 interaction wrench：

\[
u=
\begin{bmatrix}
w_L\\
w_R
\end{bmatrix}
\in\mathbb R^{12}
\]

单侧：

\[
w_i=
[F_x,F_y,F_z,T_x,T_y,T_z]^T.
\]

这里的 \(w_i\) 必须恢复为 **原 Simulink / Eq.(12) 所使用的 interaction-wrench 物理语义**，而不是直接继承 Phase 23 的 external base-control-point wrench 语义。

具体必须冻结：

- wrench 的物理对象：谁对谁的力/力矩；
- wrench 的作用点；
- body / world / controller FLU 表达 frame；
- internal / external semantics；
- sign；
- left/right block order；
- torque component order；
- 是否已经做过 point transport。

不得直接把旧 16D NMPC 方程的 \(w_i\) 写入当前 `WbcReference` 后假设两者天然一致。

---

## 5.3 NMPC → WBC Interaction-Wrench Contract

这是本次架构恢复中必须新增并独立验证的一层。

### 5.3.1 不做的事情

本阶段禁止：

```text
NMPC interaction wrench
        ↓
直接当成
WBC contact-centred wrench
```

原因是二者是不同物理量：

- NMPC interaction wrench 是上层理论模型的控制输入；
- WBC contact-centred wrench 是轮地接触的内部优化变量；
- 3D point-force → 6D contact-wrench 是 WBC contact representation 的变化，不是 NMPC 控制端口的变化。

### 5.3.2 推荐合同

推荐保留原理论中的 NMPC interaction-wrench 定义，并让 WBC 根据：

```text
q, nu
nudot
contact-centred wrench
known wheel/body inertial terms
```

重构 **实际实现的 interaction wrench**。

定义：

\[
W_{i,\mathrm{NMPC}}^I
\]

为 NMPC 对第 \(i\) 侧请求的 interaction wrench，

\[
W_{i,\mathrm{real}}^I(z;q,\nu)
\]

为 WBC 当前 decision \(z\) 所对应的实际 interaction wrench。

WBC wrench-fidelity residual 定义为：

\[
\boxed{
r_{W,i}
=
W_{i,\mathrm{real}}^I
-
W_{i,\mathrm{NMPC}}^I
-
s_i
}
\]

零 residual 时保持：

\[
\boxed{
W_{i,\mathrm{real}}^I
=
W_{i,\mathrm{NMPC}}^I+s_i
}
\]

其中 \(s_i\) 仍是 signed interaction-wrench slack。

### 5.3.3 WBC 内部仍使用 contact-centred wrench

WBC hard dynamics 不使用 NMPC interaction wrench 直接替代 contact variable。

仍保持：

\[
w_{C,i}
=
[F_r,F_l,F_n,M_r,M_l,M_n]^T
\]

作为每轮 contact-centred wrench decision。

数据流必须是：

```text
NMPC request:
W_NMPC^I
        ↓
        │ desired interaction wrench
        │
WBC decision:
nudot, tau, w_C,left, w_C,right
        ↓
wheel / body Newton-Euler mapping
        ↓
W_real^I
        ↓
compare with W_NMPC^I
        ↓
interaction-wrench slack
```

即：

\[
\boxed{
W_{\mathrm{NMPC}}^I
\neq
w_C
}
\]

### 5.3.4 Point / frame transport

如果 NMPC 与 WBC interaction-wrench fidelity 使用不同作用点或表达 frame，只允许做显式、可审计的 spatial-wrench transport。

若 wrench 从点 \(A\) 搬移到点 \(B\)，定义：

\[
r_{AB}=p_B-p_A
\]

则在同一 frame 中：

\[
F_B=F_A
\]

\[
M_B=M_A-r_{AB}\times F_A.
\]

随后再通过明确的 rotation 转到目标 frame。

实现中必须记录：

```text
source point
target point
source frame
target frame
rotation
lever arm
sign convention
left/right order
```

任何 lever arm 只能计算一次。

### 5.3.5 internal ↔ external 不能只靠坐标变换

如果原 Simulink NMPC wrench 是 wheel-to-body / body-to-wheel 的 **internal interaction wrench**，而当前 WBC reference 是 external contact wrench，则二者不能只通过：

```text
rotation + point shift
```

互相替代。

此时必须用对应刚体的 Newton-Euler balance 建立物理映射。

概念上：

\[
\boxed{
\text{interaction wrench}
=
f(
\text{rigid-body acceleration},
\text{bias/inertia},
\text{contact wrench},
\text{gravity}
)
}
\]

具体符号由冻结的“谁对谁”定义决定，不能凭旧变量名猜测。

推荐做法是：

> **保留 NMPC 原 interaction-wrench 语义，在 WBC 内部从 \(\dot\nu\) 和 6D contact-centred wrench 重构 realized interaction wrench，再与 NMPC request 做 fidelity。**

不推荐先把 NMPC request 强行转换成 external contact-wrench target，因为该转换一般依赖实际 acceleration / rigid-body balance，而这些正是 WBC decision 的一部分。

### 5.3.6 接口必须保持线性/仿射 QP 形式

在固定 \(q,\nu\) 的一个 WBC control tick 内，interaction-wrench reconstruction 必须写成对 WBC decision 的线性或仿射形式：

\[
\boxed{
W_{\mathrm{real}}^I
=
A_I(q,\nu)z+b_I(q,\nu)
}
\]

从而：

\[
r_W
=
A_I z+b_I-W_{\mathrm{NMPC}}^I-s
\]

仍然是 Weighted QP 中的线性 residual。

如果推导得到 nonlinear-in-decision 的映射，则必须先重新审查 WBC problem class，不能静默塞入当前 ProxQP QP。

### 5.3.7 Interface Oracle

接入闭环前必须至少通过：

1. equilibrium interaction-wrench parity；
2. positive / negative \(F_x\) sign；
3. positive / negative \(F_z\) sign；
4. positive / negative \(T_y\) sign；
5. left/right symmetry；
6. action-reaction sign；
7. point-transport round trip；
8. frame-rotation round trip；
9. no-double-lever-arm；
10. WBC decision → reconstructed interaction wrench 的独立 Newton-Euler / virtual-work consistency；
11. NMPC requested wrench 与 WBC realized wrench 的 component order 一致；
12. slack sign oracle：

\[
W_{\mathrm{real}}^I-W_{\mathrm{NMPC}}^I-s=0.
\]

只有该 interface oracle PASS 后，才允许把 16D NMPC 与 Minimal WBC 接成完整闭环。

---

# 6. Wheel-Relative Dynamics

恢复 paper Eq.(12) wheel-relative dynamics。

定义：

\[
D_w
=
m_w\rho+\frac{I_w}{\rho}
\]

以及：

\[
a_{Bx}
=
\frac{F_{Lx}+F_{Rx}}{m_b}.
\]

则：

\[
\ddot\xi_L
=
-a_{Bx}
-
\frac{\rho F_{Lx}+T_{Ly}}{D_w}
\]

\[
\ddot\xi_R
=
-a_{Bx}
-
\frac{\rho F_{Rx}+T_{Ry}}{D_w}.
\]

common mode：

\[
\ddot\xi_c
=
-\frac{F_{Lx}+F_{Rx}}{m_b}
-
\frac{
\rho(F_{Lx}+F_{Rx})
+
T_{Ly}+T_{Ry}
}{
2D_w
}.
\]

differential mode：

\[
\ddot\xi_\Delta
=
\frac{
\rho(F_{Lx}-F_{Rx})
+
T_{Ly}-T_{Ry}
}{
2D_w
}.
\]

本阶段优先保留 Eq.(12) 本身，但只有在其 \(F_x,T_y\) 继续使用原理论 interaction-wrench 语义时才成立。

因此本 Phase 的默认路线为：

```text
恢复原 NMPC interaction-wrench semantics
        ↓
Eq.(12) 保持原物理含义
        ↓
WBC 内部另外建立
contact-centred wrench → realized interaction wrench
映射
```

不允许为了适配当前 6D contact variable 而直接把 \(w_C\) 的 \(F_r,M_l\) 等分量代入 Eq.(12)。

若最终确认当前代码无法恢复原 interaction-wrench 端口，而必须让 NMPC 直接使用 external contact wrench，则必须重新推导 wheel-relative dynamics，并把它视为新的 NMPC model revision，而不是“接口转换”。

禁止重复计算 lever arm。

---

# 7. NMPC 时间与预测域

目标恢复：

\[
T_{\mathrm{NMPC}}=20\text{ ms}
\]

\[
N=20
\]

因此：

\[
T_h=0.4\text{ s}.
\]

NMPC 使用 acados SQP-RTI + HPIPM 路径。

现有 Phase 23 的 generated solver artifact 不能直接继续使用，因为：

- state dimension 从 12D 改回 16D；
- dynamics 改变；
- state cost / bounds / reference 改变。

因此必须重新：

```text
freeze model
→ regenerate solver
→ model parity
→ OCP validation
→ timing validation
```

---

# 8. 控制周期

目标恢复原理论周期关系：

\[
T_{\mathrm{NMPC}}=20\text{ ms}
\]

\[
T_{\mathrm{WBC}}=5\text{ ms}.
\]

由于当前 MuJoCo physics step 为 2 ms，而：

\[
5/2=2.5
\]

不能形成严格整数 ZOH。

因此本阶段优先候选为：

\[
\boxed{
T_{\mathrm{physics}}=1\text{ ms},
\quad
T_{\mathrm{WBC}}=5\text{ ms},
\quad
T_{\mathrm{NMPC}}=20\text{ ms}
}
\]

对应：

```text
physics : WBC : NMPC
1 ms    : 5 ms : 20 ms
```

即：

```text
5 physics steps / WBC tick
4 WBC ticks / NMPC update
20 physics steps / NMPC update
```

最终周期必须在 Phase 实现前冻结并重新验证：

- plant numerical stability；
- WBC deadline；
- NMPC deadline；
- ZOH；
- reset / replay determinism。

---

# 9. Minimal WBC

本阶段不恢复当前 nominal WBC 中的全部 standing tasks。

第一版直接构建 Minimal WBC。

## 9.1 决策变量

继续使用当前 42D contract：

\[
z=
[
\dot\nu_{12},
\tau_6,
w_{L,C6},
w_{R,C6},
s_{L,6}^{I},
s_{R,6}^{I}
].
\]

其中：

- \(w_{L,C6},w_{R,C6}\)：左右轮的 6D contact-centred wrench；
- \(s_L^I,s_R^I\)：**interaction-wrench slack**，对应 NMPC→WBC 的上层 wrench contract；
- 二者不能混为同一物理变量。

---

## 9.2 保留的 Hard Constraints

保留：

### Reduced rigid-body dynamics

\[
M(q)\dot\nu+h(q,\nu)
=
S\tau+J_c^T w_c.
\]

### Torque bounds

\[
\tau_{\min}\le\tau\le\tau_{\max}.
\]

### Contact wrench feasibility

保留当前每轮 6D contact-centred wrench 与 H-cone。

### Acceleration / workspace safety

保留当前已经冻结的 acceleration bounds 与 workspace fail-closed。

这些不属于 task necessity 审计对象。

---

# 10. Minimal WBC Soft Objectives

只保留：

\[
\boxed{
J_{\min}
=
J_{\mathrm{wrench}}
+
J_{\mathrm{contact}}
+
\epsilon J_{\mathrm{reg}}
}
\]

其中：

## 10.1 Interaction-Wrench Realization

NMPC 给出的不是 contact-centred wrench，而是：

\[
W_{\mathrm{NMPC}}^I
\]

即原理论语义下的左右 interaction-wrench request。

WBC 内部使用：

\[
\dot\nu,\tau,w_{C,L},w_{C,R}
\]

通过冻结的 wheel/body Newton-Euler interaction map 重构：

\[
W_{\mathrm{real}}^I
=
A_I(q,\nu)z+b_I(q,\nu).
\]

定义 residual：

\[
r_W
=
W_{\mathrm{real}}^I
-
W_{\mathrm{NMPC}}^I
-
s_I.
\]

零 residual 时：

\[
\boxed{
W_{\mathrm{real}}^I
=
W_{\mathrm{NMPC}}^I+s_I
}
\]

保持 signed-slack 语义。

对应代价至少包含：

\[
J_{\mathrm{fidelity}}
=
\|W_r r_W\|^2
\]

以及 interaction-wrench slack penalty：

\[
J_{\mathrm{slack}}
=
\|W_s s_I\|^2.
\]

本阶段可合写为：

\[
\boxed{
J_{\mathrm{wrench}}
=
J_{\mathrm{fidelity}}
+
J_{\mathrm{slack}}
}
\]

但日志中必须分开记录：

```text
requested interaction wrench
realized interaction wrench
fidelity residual
interaction-wrench slack
contact-centred wrench
```

不能把 `contact wrench error` 和 `interaction-wrench realization error` 混成一个指标。

---

## 10.2 Contact acceleration

保留当前 soft contact acceleration task，用于维持必要的 rolling / lateral / normal kinematic consistency。

该项不承担 base motion planning。

---

## 10.3 Weak regularization

保留弱：

\[
J_{\mathrm{reg}}
=
\|\dot\nu\|^2_{W_{\dot\nu}}
+
\|\tau\|^2_{W_\tau}
+
\|w_c\|^2_{W_c}.
\]

目的仅为：

- 去除冗余解；
- 防止异常大的 acceleration；
- 防止异常 torque / contact wrench；
- 改善数值条件。

如果 regularization 对闭环运动产生明显主导作用，应标记为新的诊断问题。

---

# 11. 本阶段明确关闭的 WBC Tasks

第一版 Minimal WBC 中关闭：

```text
base-X / common rolling task

base height task

base roll task

base pitch task

heading / yaw reset task

left leg posture task
right leg posture task

wheel common tracking task
wheel differential tracking task

其他额外状态反馈 / correction
```

特别注意：

即使恢复了 wheel-position planner 和 16D NMPC，本阶段也不在 WBC 中加入：

\[
\xi_c
\]

或：

\[
\xi_\Delta
\]

直接 tracking task。

本阶段要验证：

> NMPC 已经显式预测并控制 wheel-relative motion 后，WBC 是否可以只作为 wrench realization 层工作。

---

# 12. 本阶段实现顺序

不能一次修改全部结构后只看最终轨迹。

按以下 Gate 逐层验证。

---

## Gate 1：Wheel-State / Planner Contract

验证：

```text
RobotState
→ wheel-center geometry
→ xi_L / xi_R
→ xi_c / xi_delta
→ planner
→ xi_c_ref / dxi_c_ref / ddxi_c_ref
```

至少检查：

- left/right order；
- sign；
- frame；
- finite difference；
- velocity reconstruction；
- planner continuity；
- governor limits；
- reset determinism。

Gate 1 不通过，不进入 NMPC。

---

# 13. Gate 2：16D NMPC Model

验证：

```text
RobotState
→ x16
→ continuous model
→ RK4 discrete model
→ acados generated model
```

至少检查：

- equilibrium；
- one-step dynamics；
- analytic / finite-difference sensitivity；
- Eq.(12) wheel dynamics；
- common/differential mode；
- left/right symmetry；
- 原 interaction-wrench sign；
- 原 interaction-wrench point / frame / internal-external semantics；
- Eq.(12) 中 \(F_x,T_y\) 与该 contract 一致；
- bounds；
- reset；
- generated model parity。

Gate 2 不通过，不接 WBC 闭环。

---

# 14. Gate 3：NMPC OCP

独立验证：

- equilibrium hold；
- positive / negative longitudinal reference；
- acceleration；
- braking；
- return-to-zero；
- wheel-position reference tracking；
- differential wrench response；
- state bounds；
- wrench bounds；
- solver status；
- defect；
- KKT / projected stationarity；
- solve time；
- deterministic reset。

此阶段首先确认：

\[
\boxed{
\text{NMPC 本身能产生合理 wrench}
}
\]

而不是用 WBC task 帮助 NMPC 达到 reference。

---

# 15. Gate 4：Minimal WBC Algebra

在不跑完整 nonlinear 闭环前验证：

- 42D problem assembly；
- hard constraints；
- wrench mapping；
- slack sign；
- contact task；
- regularization；
- solver parity；
- equilibrium QP；
- representative dynamic states；
- torque extraction；
- deadline。

特别检查 NMPC→WBC interface：

```text
W_NMPC^I
        ↓
interaction-wrench contract
        ↓
WBC decision
(nudot, contact-centred wrench)
        ↓
reconstructed W_real^I
        ↓
fidelity residual / slack
```

必须确认：

- NMPC request 和 WBC realized wrench 是同一种 physical wrench；
- point / frame / sign / left-right / component order 完全一致；
- internal↔external 映射若存在，来自 Newton-Euler balance，不是变量改名；
- point transport 的 lever arm 只计算一次；
- contact-centred wrench 不被直接当成 NMPC interaction wrench；
- `W_real^I - W_NMPC^I - slack` 的符号 oracle 为零；
- reconstruction 对 WBC decision 保持线性/仿射，未改变 QP problem class。

---

# 16. Gate 5：Full Closed-Loop Test

通过前四个 Gate 后，才运行：

\[
\boxed{
\text{Wheel Planner}
+
16D\ \text{NMPC}
+
\text{Minimal WBC}
+
\text{MuJoCo}
}
\]

---

# 17. 本阶段测试范围

只测试平地 nominal operating envelope。

不做 terrain。

原因：

1. 当前目标是验证原理论 nominal 分层是否能成立；
2. 当前冗余 WBC task 不是因为 terrain adaptation 才被加入；
3. terrain 会额外引入 contact normal change、impact 和 working-point variation，降低基础架构失效归因的清晰度。

---

# 18. 测试工况

## T0：Static Hold

```text
flat ground
v_ref = 0
yaw_rate_ref = 0
nominal initial state
```

目标：

- equilibrium；
- height；
- pitch / roll；
- wheel-position drift；
- wrench realization；
- contact；
- solver。

---

## T1：Straight Start–Cruise–Brake

使用固定 longitudinal reference：

```text
start
→ constant speed
→ brake
→ stop
```

第一版采用代表性平地速度。

目标：

- velocity tracking；
- wheel-position tracking；
- pitch / height；
- wrench realization；
- torque / contact margin；
- braking / recovery。

---

## T2：Continuous Turning

恢复转向验证。

采用固定代表工况，包括：

```text
left turn
right turn
```

目标：

- yaw / yaw-rate tracking；
- left/right wrench distribution；
- differential wheel response；
- wheel-position drift；
- left/right symmetry；
- roll / pitch；
- wrench realization。

转向模型/reference 必须先完成独立设计与 Gate，不直接继承当前 Phase 23 straight-only claim。

---

## T3：Wheel-Differential Initial Offset

设置：

\[
\xi_\Delta(0)=+\Delta\xi
\]

和：

\[
\xi_\Delta(0)=-\Delta\xi.
\]

使用固定对称幅值，例如 baseline 中已有的 \(\pm10\) mm 量级。

目标：

- differential mode 是否恢复；
- 是否持续漂移；
- 是否引起 yaw / base motion degradation；
- NMPC 是否能够依靠 wrench 自身完成恢复。

本阶段不使用 WBC differential wheel-position task 帮助恢复。

---

# 19. 测试指标

评价采用：

\[
\boxed{
\text{Hard Gate}
+
\text{Closed-loop Performance}
+
\text{NMPC Performance}
+
\text{WBC Realization}
+
\text{Control Resource}
}
\]

---

## 19.1 Hard Gate

至少记录：

```text
simulation_completed
stable
failureReason

NMPC_status
NMPC_fault_ratio

WBC_solver_status
QP_feasible_ratio

NMPC_solve_p99 / max
WBC_solve_p99 / max
combined_control_time

dynamics_residual_max

torque_margin_min
contact_cone_margin_min
normal_load_min

workspace_status
NaN / Inf
```

Hard Gate 失败时，不用 tracking 指标宣称该架构通过。

---

# 20. Closed-Loop Performance

至少记录：

```text
vx_RMSE
vx_peak_error
vx_steady_state_error

z_RMSE
z_peak_error

roll_RMS
roll_peak

pitch_RMS
pitch_peak

yaw_rate_RMSE
yaw_error

xi_c_RMSE
xi_c_peak_error

xi_delta_peak
xi_delta_tail
xi_delta_settling_time
```

不同工况只使用有意义的指标。

---

# 21. NMPC Performance

记录：

```text
state_tracking_error

wheel_position_tracking_error
wheel_speed_tracking_error

prediction_defect

input_bound_violation
state_bound_violation

projected_stationarity / KKT metric

wrench magnitude
wrench rate / delta-wrench

NMPC solve time
```

目的是区分：

\[
\boxed{
\text{上层规划/预测失败}
}
\]

与：

\[
\boxed{
\text{下层 realization 失败}
}
\]

---

# 22. WBC Realization

重点记录：

```text
interaction_Fx_realization_error
interaction_Fz_realization_error
interaction_Ty_realization_error

interaction_wrench_requested
interaction_wrench_realized

interaction_wrench_fidelity_residual_RMS / peak
interaction_wrench_slack_RMS / peak
per-component interaction slack RMS / peak

contact_centred_wrench_RMS / peak

contact rolling residual
contact lateral residual
contact normal residual

hard residual
```

如果 wrench 采用完整 12D active reporting，则所有 12 个 component 均保留日志；性能汇总可优先显示当前主动/关键通道。

---

# 23. Control Resource

至少记录：

```text
tau_RMS
tau_peak
torque_saturation_ratio

contact wrench RMS / peak

torque margin
contact-cone margin
normal-load margin
```

用于防止某个 candidate 仅依赖长期贴边控制获得表面 tracking。

---

# 24. 通过判定原则

Minimal 架构只有同时满足以下条件，才能作为本阶段 PASS：

1. T0～T3 中预声明的工况均保持 closed-loop stable；
2. hard constraints 与 workspace contract 满足；
3. NMPC solver / WBC solver 无持续 fault；
4. tracking 指标满足冻结门槛；
5. wheel-position / wheel-speed tracking 满足冻结门槛；
6. wrench realization 不出现不可接受的持续失配；
7. 无长期 torque saturation；
8. contact / friction margin 保持有效；
9. control timing 满足冻结周期；
10. replay / reset 行为可重复。

不能用以下任一单项替代完整 PASS：

```text
QP feasible
small slack
NMPC status = 0
没有跌倒
某条轨迹看起来正常
```

---

# 25. 如果 Minimal WBC 不稳定

如果 Full Closed-Loop Test 失败：

\[
\boxed{
\text{本阶段停止在 Failure Attribution}
}
\]

不增加任何 compensatory WBC task。

---

# 26. Failure Attribution 分类

至少区分以下几类。

## A. Wheel Planner / Reference Failure

表现：

- wheel reference 本身不合理；
- discontinuity；
- saturation；
- reference 与 geometry 不一致；
- common/differential semantics 错误。

结论：

```text
failure layer = planner/reference
```

---

## B. NMPC Model Failure

表现：

- prediction 与 nonlinear plant 明显偏离；
- Eq.(12) wheel dynamics residual 异常；
- wrench point/frame/sign 不一致；
- common/differential response 不合理；
- model validity envelope 超出。

结论：

```text
failure layer = NMPC model
```

---

## C. NMPC OCP / Tuning Failure

表现：

- model 本身正确；
- 但 reference tracking 差；
- wrench command 过大 / 过快；
- state/input bounds 主导；
- horizon / weights / constraint profile 不合理；
- solver/KKT 性能不健康。

结论：

```text
failure layer = NMPC OCP
```

允许在本阶段继续调 NMPC 本身，因为 NMPC 是本 Phase 正在恢复的核心模型。

但不得用新增 WBC state task 代替修复 NMPC。

---

## D. NMPC → WBC Interface Failure

表现：

- NMPC interaction wrench 单独检查合理；
- Eq.(12) 使用的 wrench 与 WBC fidelity 使用的 wrench 不是同一种 physical quantity；
- internal interaction wrench 被误当成 external contact wrench；
- contact-centred wrench 被直接当成 NMPC request；
- 进入 WBC 后 sign / frame / moment 改变；
- point transport / lever arm 重复或遗漏；
- left/right order 或 `[Fx,Fy,Fz,Tx,Ty,Tz]` order 不一致；
- action-reaction sign 错误；
- reconstructed \(W_{\mathrm{real}}^I\) 与独立 Newton-Euler oracle 不一致；
- interface mapping 对 decision 变成非线性但仍被当作 QP；
- slack 定义或符号不一致。

结论：

```text
failure layer = NMPC→WBC interaction-wrench contract
```

该类问题属于本 Phase 必须修复的结构错误，不能通过增加 WBC task 掩盖。

---

## E. WBC Realization Failure

表现：

- NMPC wrench 合理；
- WBC 无法较好实现；
- slack 大；
- contact cone / torque / acceleration constraints 频繁 active；
- hard feasible 但 realization 明显不足。

结论：

```text
failure layer = WBC realization / feasibility
```

本阶段只记录机制。

不增加 height / pitch / rolling / posture task。

---

## F. Contact Representation / Plant Mismatch

表现：

- WBC 内部 contact model 满足；
- MuJoCo plant contact truth 出现系统性偏差；
- slip / load / moment support 与 controller model 不一致；
- realization 误差随 contact state 增长。

结论：

```text
failure layer = contact / plant mismatch
```

---

## G. Missing Fast Low-Level Stabilization

只有在满足：

```text
planner/reference 合理
NMPC model/OCP 健康
wrench interface 正确
WBC realization 较好
hard constraints 健康
```

但仍出现：

- 某自由度高频振荡；
- 低层未阻尼模态；
- 快速 joint/posture 漂移；
- 上层 20 ms 无法稳定的明显 fast mode；

才能暂时记录：

```text
suspected missing fast low-level stabilization
```

注意：

这只是下一 Phase 的研究入口。

本阶段禁止直接加 task 验证或修复。

---

# 27. 本阶段停止条件

出现以下任一情况后，本阶段不继续扩展 WBC task：

### 情况 1：Minimal WBC PASS

结论：

\[
\boxed{
\text{恢复理论上层后，Minimal WBC 足以完成当前平地 nominal 闭环}
}
\]

进入下一 Phase 做：

- robustness；
- 更完整性能；
- terrain；
- 或其他研究。

### 情况 2：Minimal WBC FAIL，但问题已定位到 planner / NMPC / interface / realization / contact

先修复属于本 Phase 范围的结构错误。

修复后重新跑 Minimal WBC。

### 情况 3：基础模型/interface均正确，但仍存在无法由当前层解释的快速稳定性缺口

本 Phase 结束并输出：

```text
Minimal WBC: FAIL

Failure type:
suspected missing fast low-level stabilization

Evidence:
<对应状态、wrench、slack、residual、frequency、constraint evidence>
```

下一 Phase 再研究：

> 是否需要低层补偿任务，以及最小需要哪一个。

---

# 28. 本阶段明确禁止

本阶段不做：

- add-back task；
- one-by-one compensatory task test；
- adaptive WBC weight；
- HQP；
- task hierarchy redesign；
- slack feedback；
- feasibility-aware NMPC；
- disturbance observer；
- parameter adaptation；
- terrain adaptation；
- learning / RL；
- 新 low-level controller。

也不因为 Minimal WBC 失败，就恢复全部旧 WBC standing tasks。

---

# 29. 本阶段输出

至少形成：

```text
1. frozen architecture / interface spec
2. wheel planner contract
3. 16D NMPC model contract
4. NMPC→WBC interaction-wrench contract
5. interaction-wrench mapping/oracle evidence
6. regenerated acados solver artifact
7. minimal WBC profile
8. timing profile
9. T0-T3 test matrix
10. per-run summary
11. failureReason
12. failure attribution report
13. final REVIEW
14. final RECORD
```

如果失败，RECORD 必须明确：

```text
Minimal WBC passed / failed

failed case

first failure time

primary failed state

NMPC status

WBC status

wrench realization

slack

active constraints

contact state

failure layer

next-phase question
```

---

# 30. 本阶段最终问题

本阶段只回答：

\[
\boxed{
\text{恢复原 16D wheel-aware NMPC 后，
仅使用 wrench realization + contact + weak regularization 的 WBC，
能否完成平地 nominal 闭环？}
}
\]

如果答案是否定的：

\[
\boxed{
\text{失败究竟发生在哪一层？}
}
\]

本阶段不回答：

\[
\boxed{
\text{应该加哪个补偿任务？}
}
\]

后者留给下一 Phase。

---

# 31. 最终流程

```text
Current MuJoCo baseline
        ↓
Freeze wheel / wrench / timing semantics
        ↓
Restore wheel-position planner
        ↓
Restore 16D NMPC + Eq.(12)
        ↓
Freeze original NMPC interaction-wrench semantics
        ↓
Build / validate NMPC→WBC interaction-wrench map
        ↓
Regenerate / validate acados solver
        ↓
Keep current 42D / 6D-contact WBC foundation
        ↓
Remove all auxiliary WBC state tasks
        ↓
Minimal WBC
        ↓
T0 static
        ↓
T1 straight
        ↓
T2 turning
        ↓
T3 differential-offset recovery
        ↓
     PASS ?
     /   \
   YES    NO
    |      |
    |      ↓
    |   Failure Attribution
    |      |
    |      ├─ planner/reference
    |      ├─ NMPC model
    |      ├─ NMPC OCP
    |      ├─ wrench interface
    |      ├─ WBC realization
    |      ├─ contact/plant
    |      └─ suspected missing fast low-level stabilization
    |      |
    |      ↓
    |   End Phase
    |
    ↓
Minimal architecture accepted
        ↓
End Phase
```

---

# 32. Phase Boundary

本 Phase 的边界是：

> **建立并验证“原理论上层 + 当前已验证底层 + Minimal WBC”这一基础架构，并在失败时完成层级归因。**

下一 Phase 才允许研究：

> **如果基础架构仍缺少低层稳定作用，究竟需要什么 task、为什么需要、应该放在哪一层、应该采用什么带宽。**
