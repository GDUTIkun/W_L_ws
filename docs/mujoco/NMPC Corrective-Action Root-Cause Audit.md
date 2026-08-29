# Phase 29：NMPC Corrective-Action Root-Cause Audit

## 1. Phase 目标

本 Phase 只回答：

$$
\boxed{
\text{为什么 16-state NMPC 在 T0/T1 首偏离状态附近产生非恢复性控制作用？}
}
$$

Phase 28 已经证明：

```text
NMPC request
    ↓
WBC realization          PASS
    ↓
reduced acceleration     PASS
    ↓
MuJoCo response          PASS
```

但：

```text
state error
    ↓
NMPC requested wrench
    ↓
net acceleration reinforces error
```

因此本 Phase 不再研究：

* WBC realization；
* contact representation；
* ProxQP；
* torque/contact margin；
* plant mismatch；
* fast low-level stabilization task。

当前 primary failure layer 冻结为：

$$
\boxed{\texttt{B\_nmpc\_corrective\_failure}}
$$

本 Phase 的目标是继续向 NMPC 内部拆解，找到产生错误 corrective action 的最早机制。

---

# 2. 当前已知事实

## T0 Static

Phase 28 frozen-state oracle 得到：

$$
\frac{\partial \dot\omega_y}{\partial\theta}
=
+118.153
$$

$$
\frac{\partial \dot\omega_y}{\partial\omega_y}
=
+18.2632
$$

即 pitch / pitch-rate perturbation 会得到强化误差方向的 angular acceleration。

因此 T0 的核心问题为：

$$
\boxed{
\text{为什么 NMPC 局部 pitch closed-loop response 呈 reinforcing behavior？}
}
$$

---

## T1 Straight

Phase 28 得到局部 perturbation derivatives：

$$
\frac{\partial a_x}{\partial e_x}
=
-0.972159
$$

$$
\frac{\partial a_x}{\partial e_v}
=
-0.491522
$$

局部导数本身具有恢复趋势。

但实际 first-divergence snapshot：

$$
e_x<0,\qquad e_v<0
$$

而：

$$
a_x^{net}
=
-0.0118472\ {\rm m/s^2}
$$

即实际净动作仍然强化 longitudinal error。

因此 T1 的问题不是简单的 feedback sign reversal，而是：

$$
\boxed{
\text{为什么 restorative local feedback 被其他 NMPC action/coupling/bias 抵消？}
}
$$

---

# 3. Phase 边界

继续冻结 Phase 27/28：

```text
current nominal MuJoCo plant

wheel-state reconstruction
wheel-position planner

16-state state definition
relative-rotation-vector attitude chart

Eq.(12)

current physical parameters

wheel-centre internal interaction-wrench semantics

20 ms NMPC
N = 20
two 10 ms RK4 substeps

SQP-RTI
partial-condensing HPIPM

42D Minimal WBC
104 hard rows
6D contact-centred wrench

2 / 10 / 20 ms schedule
```

本 Phase 禁止：

```text
增加 WBC pitch task
增加 base-X / rolling task
增加 height / leg / wheel task

修改 WBC weights
修改 WBC hard constraints

修改 NMPC weights
修改 NMPC bounds
修改 horizon
修改 solver family
修改 reference
```

在根因关闭前不得通过 tuning 改变现象。

---

# 4. 总体分析链

把 NMPC corrective action 拆成：

```text
measured state / reference
        ↓
state-error construction
        ↓
state normalization
        ↓
stage / terminal cost
        ↓
model sensitivity
        ↓
OCP optimal trade-off
        ↓
requested interaction wrench
        ↓
predicted acceleration
```

逐层判断：

$$
\boxed{
\text{错误第一次出现在哪里？}
}
$$

---

# 5. Gate 1：State / Reference / Error Semantics

首先重新审计 NMPC 看见的状态误差。

## T0

检查：

```text
orientation state r

desired orientation r_ref

pitch physical sign

pitch-rate omega_y sign

state error sign

yaw-aligned R_ref

world/body expression
```

必须建立：

$$
\theta_{\rm physical}
\leftrightarrow
r
\leftrightarrow
x-x_{\rm ref}
$$

的 golden vectors。

做：

$$
\theta=\pm\Delta\theta
$$

和：

$$
\omega_y=\pm\Delta\omega_y
$$

检查 NMPC 内部 cost error 是否随物理误差正确变号。

Gate 1 要排除：

```text
orientation chart sign error
reference sign error
body/world rate mismatch
state ordering error
```

---

## T1

检查：

```text
x
x_ref
vx
vx_ref

moving horizon anchor

stage reference advance

planner xi reference

xi / dxi reference
```

确认：

$$
e_x=x-x_{\rm ref}
$$

和：

$$
e_v=v_x-v_{\rm ref}
$$

在整个 horizon 中的定义一致。

特别检查：

> 当前时刻实际落后 reference 时，未来各 stage 的 reference 是否仍按正确方向推进。

Gate 1 PASS 后才能进入 cost/action 审计。

---

# 6. Gate 2：Cost Gradient Direction Audit

不运行完整闭环。

在冻结 snapshot 上直接计算：

$$
J(x,u)
$$

对关键状态和 input 的一阶响应。

## T0

至少审计：

$$
\frac{\partial J}{\partial\theta}
$$

$$
\frac{\partial J}{\partial\omega_y}
$$

以及通过 solver 得到的：

$$
\frac{\partial u^*}{\partial\theta}
$$

$$
\frac{\partial u^*}{\partial\omega_y}.
$$

需要区分：

```text
state cost gradient 本身方向错误

vs

cost gradient 正确，
但 dynamics/input coupling 让 optimal wrench 方向错误
```

---

## T1

至少审计：

$$
\frac{\partial J}{\partial x},
\quad
\frac{\partial J}{\partial v_x},
\quad
\frac{\partial J}{\partial \xi_L},
\quad
\frac{\partial J}{\partial \xi_R}
$$

以及对应 optimal common:

$$
F_x^{c}
=
F_{Lx}+F_{Rx}
$$

$$
T_y^{c}
=
T_{Ly}+T_{Ry}.
$$

目标回答：

$$
\boxed{
\text{cost 是否真的要求机器人恢复？}
}
$$

---

# 7. Gate 3：Model Sensitivity / Control Authority Audit

如果 cost gradient 正确，继续审计 NMPC 模型认为：

$$
u
\rightarrow
\dot x
$$

到底是什么关系。

## T0

重点计算：

$$
\frac{\partial\dot\omega_y}
{\partial F_{Lx}},
\quad
\frac{\partial\dot\omega_y}
{\partial F_{Rx}}
$$

$$
\frac{\partial\dot\omega_y}
{\partial T_{Ly}},
\quad
\frac{\partial\dot\omega_y}
{\partial T_{Ry}}.
$$

并验证：

```text
+Fx
-Fx
+Ty
-Ty
```

对 pitch acceleration 的方向。

目的是判断：

> NMPC 是因为认为某个 wrench 能恢复 pitch，所以主动输出；还是 cost 本身就在要求错误方向？

---

## T1

审计：

$$
\frac{\partial a_x}
{\partial F_{Lx}},
\quad
\frac{\partial a_x}
{\partial F_{Rx}}
$$

以及 Eq.(12) 引起的：

$$
F_x
\leftrightarrow
\xi,\dot\xi
$$

coupling。

特别检查：

$$
\boxed{
\text{base forward acceleration}
\quad\text{与}\quad
\text{wheel-position recovery}
}
$$

是否在 OCP 中形成冲突。

---

# 8. Gate 4：Optimal-Action Decomposition

这是 T1 最关键的一步。

把实际 optimal action 的形成拆解成若干贡献。

目标概念形式：

$$
a_x^{net}
=
a_x^{base}
+
a_x^{velocity}
+
a_x^{wheel}
+
a_x^{attitude}
+
a_x^{input}
+
a_x^{terminal}
+
a_x^{constraint}.
$$

不要求数学上强行做唯一线性分解，但必须通过冻结 snapshot 的 controlled counterfactual 获得因果归因。

例如保持其他条件不变分别：

```text
只消除 x error

只消除 vx error

只消除 xi error

只消除 dxi error

只消除 pitch error

只消除 pitch-rate error
```

重新求一次 OCP。

比较：

$$
\Delta W_{\rm request}
$$

和：

$$
\Delta a_x.
$$

从而判断究竟是哪一组 state/reference 让：

$$
a_x^{net}<0.
$$

---

# 9. Gate 5：T0 Positive-Feedback Source Isolation

T0 单独建立 attribution matrix。

依次做 frozen-state counterfactual：

```text
Case A:
实际 snapshot

Case B:
theta → 0

Case C:
omega_y → 0

Case D:
theta, omega_y → 0

Case E:
wheel states → reference

Case F:
base x/vx → reference
```

所有其他量保持一致。

记录：

```text
requested Fx_L/R
requested Ty_L/R

predicted angular acceleration

objective

state/input cost

bounds activity

stationarity
```

要求回答：

$$
\boxed{
+118.153,\ +18.2632
\text{ 的正反馈究竟来自哪条状态/动力学/cost 路径？}
}
$$

---

# 10. Gate 6：T1 Net-Action Source Isolation

对 T1 first-divergence snapshot 做相同 counterfactual。

至少：

```text
实际 snapshot

x = x_ref

vx = vx_ref

xi = xi_ref

dxi = dxi_ref

pitch = 0

omega_y = 0

wheel state 全部 = reference

base longitudinal state = reference
```

观察：

$$
a_x^{net}
$$

何时从：

$$
a_x<0
$$

变成：

$$
a_x>0.
$$

最早能够翻转符号的状态组，就是 primary coupling candidate。

---

# 11. Gate 7：Horizon Attribution

如果单点 cost/model 都正确，需要继续看：

$$
\boxed{
\text{有限预测域的 optimal trade-off}
}
$$

检查每个 prediction stage：

```text
x error
vx error
pitch
xi_L/R
dxi_L/R

Fx_L/R
Ty_L/R

stage cost
active bounds
```

重点找：

> 当前阶段为了减少某个未来 wheel/attitude/state cost，是否允许短期 longitudinal/pitch error 继续增大。

T1 尤其检查：

```text
wheel-position cost
vs
base x/vx tracking
```

T0 尤其检查：

```text
pitch stabilization
vs
wheel / input / other-state trade-off
```

---

# 12. Gate 8：Constraint / Bound Influence

虽然 Phase 28 已证明 WBC resource 不受限，但 NMPC 自己仍可能受 OCP bound 影响。

检查：

```text
state bounds
wheel workspace bounds
input bounds
orientation chart bounds
```

使用：

* distance-to-bound；
* active constraint；
* multiplier；
* projected gradient；

判断：

$$
\boxed{
\text{wrong corrective action 是否是 constraint-driven optimum}
}
$$

本 Phase 只分析，不放宽 bound。

---

# 13. Failure Classification

最终 T0/T1 分别只能归入以下一种 primary cause。

## A. State / Reference Semantics Error

例如：

```text
sign
frame
reference construction
state order
attitude/rate definition
```

---

## B. Cost Direction Error

状态/reference 本身正确，但：

$$
\nabla J
$$

推动错误方向。

---

## C. Dynamics / Control-Authority Sign Error

cost 想恢复，但：

$$
u\rightarrow\dot x
$$

在 NMPC model 中的控制作用方向错误。

---

## D. Cross-State / Wheel Coupling Dominance

单独的目标都正确，但：

```text
wheel-relative state
base state
attitude
```

之间的 optimal trade-off 导致净作用方向错误。

---

## E. Horizon / Reference-Propagation Effect

当前反馈方向正确，但：

```text
future reference
terminal objective
finite horizon
```

让最优控制选择短期强化当前误差。

---

## F. NMPC Constraint-Driven Action

正确恢复动作被：

```text
state/input/workspace bound
```

改变。

---

## G. Solver / RTI Approximation Artifact

只有在：

```text
model
reference
cost
constraint
```

均正确，而 SQP-RTI 实际 solution 与高精度 OCP oracle 给出的 corrective direction 不一致时才能使用。

---

# 14. Full-Solve Oracle

为防止把 SQP-RTI 的一次迭代问题误认为 OCP 本身问题，本 Phase 应增加一个 offline oracle。

在少数冻结 snapshot 上：

```text
production:
SQP_RTI

offline oracle:
多次 SQP / converged solve
```

使用完全相同：

```text
model
horizon
cost
reference
bounds
initial state
```

比较：

$$
W_{\rm RTI}
$$

与：

$$
W_{\rm converged}.
$$

如果两者 corrective direction 相同：

$$
\boxed{
\text{问题属于 OCP formulation}
}
$$

如果 converged solve 恢复，而 RTI 不恢复：

$$
\boxed{
\text{问题属于 SQP-RTI lifecycle / approximation}
}
$$

这个测试只作为 offline oracle，不改变 production solver。

---

# 15. T2 / T3

## T2

本 Phase 不继续扩大 T2。

Phase 28 已知：

```text
T2 right:
与 T1 B-path 一致

T2 left:
不一致
```

因此只在 T1 根因关闭后，对 right/left 各取少量 snapshot 检查同一 mechanism 是否出现。

不单独建立新的 turning architecture。

---

## T3

继续 out of scope。

T3 的：

```text
single-RTI stationarity robustness
```

仍留待独立 Phase。

---

# 16. 本 Phase 禁止

禁止：

```text
调 Q/R

改 pitch 权重

改 wheel-position 权重

改 input penalty

改 horizon

改 Ts

改 constraints

加 WBC task

增加 inner loop

重新设计 planner
```

本 Phase 的任务是：

$$
\boxed{\text{找原因}}
$$

不是：

$$
\boxed{\text{试着调到能跑}}
$$

---

# 17. Phase PASS 条件

T0 与 T1 分别必须得到唯一 primary root cause：

```text
A state/reference semantics
B cost direction
C dynamics/control sign
D cross-state coupling
E horizon/reference propagation
F NMPC constraint
G SQP-RTI approximation
```

并有至少三类证据闭合：

```text
frozen-state perturbation

counterfactual OCP solve

predicted acceleration/action

stage/horizon evidence

full-solve oracle
```

如果多个机制无法区分：

```text
REWORK / unresolved
```

不得选择“最可能”。

---

# 18. Phase 输出

至少形成：

```text
1. PLAN

2. state-reference contract audit

3. T0 corrective-action root-cause report

4. T1 corrective-action root-cause report

5. frozen-state counterfactual corpus

6. model/control sensitivity report

7. OCP action-decomposition report

8. horizon/stage-cost report

9. constraint-activity report

10. RTI-vs-converged oracle

11. final root-cause matrix

12. REVIEW

13. RECORD
```

---

# 19. 最终问题

本 Phase 最终只回答：

$$
\boxed{
\text{为什么恢复后的 16-state NMPC 会生成强化 T0/T1 误差的 interaction wrench？}
}
$$

Phase 结束后，如果确认是：

```text
reference / sign / model bug
```

下一 Phase 才修结构。

如果确认是：

```text
cost / coupling / horizon / constraint
```

下一 Phase 才进行 NMPC redesign / retuning。

如果确认是：

```text
SQP-RTI approximation
```

下一 Phase 才研究 solver lifecycle。

只有把 NMPC corrective action 修复并重新验证以后，才重新讨论：

$$
\boxed{
\text{是否仍然需要额外 WBC stabilization task}
}
$$
