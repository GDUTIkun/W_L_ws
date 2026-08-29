# Minimal Closed-Loop Drift / Divergence Attribution

## 1. Phase 目标

本 Phase 不增加任何新的 WBC stabilization task。

目标是回答：

$$
\boxed{
\text{为什么经过验证的 16D NMPC + Minimal WBC 闭环仍会出现漂移/发散？}
}
$$

重点研究 Phase 27 中：

* T0 static：静止状态下的姿态/基座漂移；
* T1 straight：纵向 reference tracking 丧失；
* T2 left/right：验证 T1 机制在转向条件下是否保持一致。

本 Phase 最终只需要得到：

$$
\boxed{
\text{first physical divergence mechanism}
}
$$

而不是决定：

$$
\boxed{
\text{应该增加哪个 WBC task}
}
$$

后者留到下一 Phase。

---

# 2. 本 Phase 的核心问题

将闭环链路拆为：

```text
state error
    ↓
NMPC
    ↓
requested interaction wrench
    ↓
Minimal WBC
    ↓
realized interaction wrench
    ↓
joint torque / contact wrench
    ↓
MuJoCo plant
    ↓
actual acceleration / state evolution
```

需要逐层回答：

```text
1. 状态开始偏离时，NMPC 有没有产生正确方向的恢复作用？

2. NMPC 给出的恢复 wrench，
   WBC 有没有真正实现？

3. WBC 已经实现正确 wrench 后，
   plant 是否产生了模型预测的 acceleration？

4. 如果以上都正确，
   漂移是否来自上层闭环带宽不足 / fast mode？

5. 原 safety envelope 是否只是过早终止了一个
   实际会自行恢复的 trajectory？
```

---

# 3. 冻结内容

本 Phase 不重新设计 Phase 27 的基础架构。

继续冻结：

```text
current nominal MuJoCo plant

wheel-state definition
wheel-position planner

16-state NMPC
Eq.(12)
current parameters

wheel-to-body internal interaction-wrench contract

NMPC → WBC affine wrench mapping

42D Minimal WBC
104 hard rows
6D contact-centred wrench / wheel

ProxQP

2 / 10 / 20 ms
physics / WBC / NMPC schedule

torque / contact / acceleration / workspace constraints

fault / fail-zero / reset semantics
```

特别禁止因为看到漂移就：

* 修改 NMPC/WBC 权重；
* 增加 pitch task；
* 增加 base-X task；
* 增加 rolling task；
* 增加 height / leg / orientation task；
* 改 contact cone；
* 改 torque limit；
* 改 controller timing；
* 放宽原 formal PASS threshold 后重新宣称 PASS。

---

# 4. 允许增加的东西

本 Phase 只允许增加：

```text
diagnostic logging
offline analysis
shadow calculation
finite-difference / perturbation oracle
diagnostic continuation runner
frequency / growth-rate analysis
prediction-vs-plant residual
failure attribution tooling
```

这些东西不得改变施加到 plant 的正常控制律。

---

# 5. Gate 0：Phase 27 Failure Reproduction

首先重新运行最小工况：

```text
T0 static
T1 straight
T2 left
T2 right
```

要求复现：

* first-failure time；
* first-failure state；
* requested wrench；
* realized wrench；
* NMPC status；
* WBC status；
* constraint state；
* safety-envelope trigger。

Phase 27 原始 formal threshold 保持不变。

Gate 0 的目的不是重新判断 PASS/FAIL，而是建立本 Phase 的统一 baseline。

---

# 6. Gate 1：Diagnostic Continuation

Phase 27 的原 safety envelope 继续作为：

$$
\boxed{
\text{nominal acceptance envelope}
}
$$

但新增一个只用于仿真的：

$$
\boxed{
\text{diagnostic termination envelope}
}
$$

它必须比 nominal envelope 更宽，并在运行前冻结。

目的不是让 Minimal controller 重新 PASS，而是：

> 在第一次 nominal envelope 越界后继续观察一段时间，判断状态究竟是恢复、形成有界振荡、漂到新平衡点，还是持续发散。

因此每个 case 同时记录：

```text
nominal_failure_time

diagnostic_stop_time

maximum excursion after nominal failure

recovery / bounded / drift / divergence
```

如果状态越过原 threshold 后又自然恢复，并重新进入 nominal envelope，则允许记录：

```text
possible overly-conservative nominal envelope
```

但仍不得修改 Phase 27 的历史判定。

---

# 7. Gate 2：NMPC Corrective-Action Attribution

这是第一层真正的控制归因。

## 7.1 T0：Pitch / Static Drift

重点记录：

$$
\theta,\quad
\omega_y
$$

以及 NMPC 请求：

$$
F_{Lx},F_{Rx},
T_{Ly},T_{Ry}.
$$

重点检查：

```text
pitch error 开始变负
        ↓
NMPC 是否产生恢复方向的 Fx / Ty
        ↓
预测 angular acceleration 是否指向恢复
```

需要形成：

$$
\theta
\rightarrow
W_{\mathrm{NMPC}}
\rightarrow
\dot\omega_y^{\mathrm{pred}}
$$

的时间序列。

并做冻结状态的小扰动 oracle，例如：

$$
\theta\pm\Delta\theta
$$

$$
\omega_y\pm\Delta\omega_y
$$

比较 NMPC 输出变化方向。

目标是回答：

$$
\boxed{
\text{NMPC 本身是否对 pitch error 具有正确恢复反馈？}
}
$$

---

## 7.2 T1：Longitudinal Drift

重点记录：

$$
e_x=x-x_{\mathrm{ref}}
$$

$$
e_v=v_x-v_{\mathrm{ref}}
$$

以及：

$$
F_{Lx}+F_{Rx}
$$

和必要的：

$$
T_{Ly}+T_{Ry}.
$$

检查：

```text
x 开始落后 reference
        ↓
velocity error 变大
        ↓
NMPC 是否增加正确方向的 longitudinal wrench
```

目标回答：

$$
\boxed{
\text{NMPC 是否真正试图消除 }e_x,e_v？
}
$$

如果：

$$
|e_x|\uparrow
$$

但恢复 wrench 没有相应增大，优先归因到：

```text
NMPC feedback / cost / model behavior
```

而不是 WBC。

---

# 8. Gate 3：WBC Wrench Realization Attribution

如果 Gate 2 证明 NMPC request 是正确的，继续检查：

$$
W_{\mathrm{NMPC}}^I
$$

和：

$$
W_{\mathrm{real}}^I.
$$

至少逐 tick 记录：

```text
requested Fx / Fz / Ty
realized Fx / Fz / Ty

full 12D requested wrench
full 12D realized wrench

wrench fidelity residual
signed slack

torque
torque margin

contact wrench
contact-cone margin

active constraints
```

计算：

$$
e_W=
W_{\mathrm{real}}^I-W_{\mathrm{NMPC}}^I.
$$

如果：

$$
W_{\mathrm{NMPC}}
$$

明显具有恢复作用，但：

$$
W_{\mathrm{real}}
$$

无法跟随，则归因：

```text
WBC realization / feasibility
```

进一步区分：

```text
torque limited
contact-cone limited
acceleration limited
workspace limited
soft-objective competition
solver / numerical
```

本 Phase 只归因，不修改约束。

---

# 9. Gate 4：Predicted Acceleration vs Actual Plant

如果：

```text
NMPC request 正确
WBC realization 正确
```

则继续检查：

> 为什么正确 wrench 没有产生预期 state response？

建立三层 acceleration：

```text
NMPC predicted acceleration

WBC/reduced-model nudot

MuJoCo actual acceleration
```

例如 T1：

$$
a_x^{\mathrm{NMPC}}
$$

$$
a_x^{\mathrm{WBC}}
$$

$$
a_x^{\mathrm{plant}}.
$$

T0：

$$
\dot\omega_y^{\mathrm{NMPC}}
$$

$$
\dot\omega_y^{\mathrm{WBC}}
$$

$$
\dot\omega_y^{\mathrm{plant}}.
$$

定义：

$$
r_{\mathrm{upper}}
=
a_{\mathrm{WBC}}
-
a_{\mathrm{NMPC}}
$$

以及：

$$
r_{\mathrm{plant}}
=
a_{\mathrm{plant}}
-
a_{\mathrm{WBC}}.
$$

重点检查：

```text
base dynamics mismatch

wheel-relative dynamics mismatch

interaction-wrench dynamics mismatch

contact moment/support mismatch

unmodelled closed-chain effect

fast joint / leg mode
```

如果 WBC 预测会恢复，但 MuJoCo 实际 acceleration 明显不恢复，则归因：

```text
model / plant closed-loop mismatch
```

---

# 10. Gate 5：Fast-Mode / Bandwidth Attribution

只有在满足：

```text
NMPC corrective action 正确

WBC realization 正确

predicted-vs-actual acceleration 基本一致

hard constraints 健康
```

以后，才研究：

$$
\boxed{
\text{是否存在上层 20 ms 无法抑制的 fast mode}
}
$$

分析：

```text
pitch
pitch rate
x
vx
wheel-relative states
joint states
requested wrench
torque
```

估计：

* divergence growth rate；
* dominant oscillation frequency；
* damping；
* command/state phase lag；
* NMPC update 与发散模态的时间尺度关系。

如果状态在两次甚至一次 NMPC update 之间就出现显著增长，可以记录：

```text
suspected insufficient upper-loop bandwidth
```

或：

```text
suspected missing fast low-level stabilization
```

但仍不得在本 Phase 添加 task。

---

# 11. Gate 6：T2 Symmetry Check

T2 本 Phase 不重新进行完整独立设计。

只用于回答：

$$
\boxed{
\text{T1 的纵向失效机制在 turning 时是否仍成立？}
}
$$

检查 left/right：

```text
first divergence state

longitudinal error evolution

NMPC Fx response

realized Fx response

pitch / roll response

contact margins
```

如果左右 case 与 T1 给出相同因果链，则合并归因。

如果 T2 出现额外：

```text
yaw
roll
left/right asymmetric contact
```

机制，则单独记录为 secondary finding，但不扩展 Phase 范围。

---

# 12. T3 的处理

T3 不作为本 Phase 的主要研究对象。

Phase 27 已经表明：

```text
xi_delta ±10 mm
        ↓
native acados stationarity failure
```

它属于：

$$
\boxed{
\text{NMPC SQP-RTI / OCP lifecycle robustness}
}
$$

而不是当前 T0–T2 的漂移问题。

因此：

```text
T3 = deferred
```

后续另开 Phase 研究：

* cold / warm start；
* SQP-RTI preparation-feedback lifecycle；
* differential initial-state feasibility；
* initial trajectory；
* stationarity；
* RTI robustness。

本 Phase 不让 T3 干扰 T0/T1 的稳定性归因。

---

# 13. 最终 Failure Classification

Phase 结束时，T0 和 T1 至少要进入以下一个主要类别。

### A. Threshold-only / Early-Termination Effect

```text
越过 nominal envelope
但继续运行后自行恢复或保持有界
```

说明：

```text
formal threshold 可能偏保守
```

但不修改历史 Phase 27 结果。

---

### B. NMPC Corrective-Action Failure

```text
state error 持续增大
但 NMPC 没有生成合理恢复 wrench
```

归因：

```text
NMPC model / cost / reference / feedback behavior
```

---

### C. WBC Realization Failure

```text
NMPC request 正确
但 W_real 无法跟随
```

归因：

```text
WBC realization / feasibility
```

---

### D. Model → Plant Response Mismatch

```text
request 正确
realization 正确
WBC/model predicted response 正确
但 MuJoCo actual response 明显不同
```

归因：

```text
model / contact / plant mismatch
```

---

### E. Fast Closed-Loop Stabilization Gap

```text
NMPC 正确
WBC realization 正确
plant/model 基本一致
但 fast state mode 仍持续增长
```

归因：

```text
suspected insufficient bandwidth /
missing fast low-level stabilization
```

这才允许成为下一 Phase 的 task-necessity 研究入口。

---

# 14. 本 Phase PASS 条件

本 Phase 不要求 Minimal controller 稳定。

Phase PASS 条件是：

$$
\boxed{
\text{T0 和 T1 的漂移具有可复现、唯一且证据闭合的首要物理归因}
}
$$

至少必须完成：

```text
Phase 27 failure reproduction

diagnostic continuation

NMPC corrective-action audit

requested → realized wrench audit

predicted → actual acceleration audit

constraint/resource audit

growth-rate / bandwidth analysis

T2 symmetry confirmation

final first-cause classification
```

不能以：

```text
“看起来像缺 pitch task”
```

或：

```text
“加 base-X task 应该能好”
```

作为 Phase 结论。

---

# 15. 本 Phase 输出

至少形成：

```text
1. PLAN

2. diagnostic-continuation contract

3. T0 drift-attribution report

4. T1 drift-attribution report

5. T2 symmetry report

6. NMPC corrective-action oracle

7. requested-vs-realized wrench report

8. predicted-vs-plant acceleration report

9. mode / growth-rate analysis

10. first-failure classification

11. REVIEW

12. RECORD
```

---

# 16. Phase Boundary

本 Phase 只回答：

$$
\boxed{
\text{为什么 Minimal closed loop 会漂移/发散？}
}
$$

不回答：

$$
\boxed{
\text{该增加哪个 task？}
}
$$

如果最终归因为：

```text
missing fast low-level stabilization
```

下一 Phase 才进入：

$$
\boxed{
\text{Minimal Stabilization Task Necessity Audit}
}
$$

并从已经证明的失效模态出发，只测试与该模态对应的最小候选 task。
