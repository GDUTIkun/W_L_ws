
# Phase 30：NMPC Corrective-Action Formulation Repair

## 1. Phase 目标

在保持 Phase 27～29 已验证的 plant、模型、wrench contract、WBC、solver runtime 与安全语义不变的前提下，对 Phase 29 已关闭的两个 NMPC formulation 根因实施**最小、独立、可归因的修复**：

$$
\boxed{
T0:\ P29\text{-}E\rightarrow
\text{repair terminal base-longitudinal propagation}
}
$$

$$
\boxed{
T1:\ P29\text{-}D\rightarrow
\text{repair attitude-dominant cross-state coupling}
}
$$

本 Phase 目标不是“把机器人调到能跑”，而是首先恢复正确的局部 corrective behavior，然后再验证完整闭环。

---

# 2. 已冻结的根因

## T0

Phase 29 已证明：

```text
production RTI        non-restorative
cold solve            non-restorative
repeated RTI          non-restorative
converged SQP         non-restorative
```

但：

```text
remove terminal objective
        → restorative

remove terminal base-longitudinal component
        → restorative

held-reference shadow
        → still non-restorative

bound shadow
        → still non-restorative
```

因此 T0 primary cause 冻结为：

$$
\boxed{
\texttt{P29-E\_horizon\_reference\_propagation}
}
$$

具体为：

$$
\boxed{
\text{terminal base-longitudinal objective propagation}
}
$$

---

## T1

Phase 29 已证明：

```text
terminal shadow
        → still non-restorative

reference shadow
        → still non-restorative

bound shadow
        → still non-restorative

remove attitude cost
        → restorative
```

pairwise 与 acceleration decomposition 进一步证明 attitude 为主导 coupling，wheel-rate 为 secondary interaction。

因此 T1 primary cause 冻结为：

$$
\boxed{
\texttt{P29-D\_cross\_state\_coupling}
}
$$

具体为：

$$
\boxed{
\text{attitude-dominant cross-state coupling}
}
$$

---

# 3. 本 Phase 的总体原则

本 Phase 采用：

$$
\boxed{
\text{one root cause}
\rightarrow
\text{one minimal intervention}
\rightarrow
\text{local oracle}
\rightarrow
\text{closed-loop validation}
}
$$

禁止：

```text
同时修改多个 Q

同时修改 terminal 和 running cost

修改 horizon

修改 Ts

修改 solver family

修改 WBC

增加 WBC task

修改 contact / torque / workspace constraint

修改 safety threshold

为了 PASS 而反复手调参数
```

所有 candidate 必须在 primary closed-loop run 前预冻结。

---

# 4. Frozen Production Baseline

继续冻结：

```text
current nominal MuJoCo plant

16-state wheel-aware NMPC

Eq.(12)

current physical parameters

wheel-centre internal interaction wrench

20 ms NMPC
N = 20
0.4 s horizon
two 10 ms RK4 substeps

production SQP-RTI
partial-condensing HPIPM

42D Minimal WBC
104 hard rows
6D contact-centred wrench

ProxQP

2 / 10 / 20 ms schedule

fault / fail-zero / reset

Phase 27 safety envelopes
```

Phase 30 允许变化的 production OCP 项只有本 Phase 明确批准的：

```text
T0 branch:
terminal base-longitudinal cost

T1 branch:
attitude cost scaling / structure
```

其他全部保持不变。

---

# 5. Repair Branch A：T0 Terminal Repair

## 5.1 第一候选

第一候选采用最小修改：

$$
\boxed{
Q_{e,x}^{new}=0
}
$$

即移除 terminal stage 的 base longitudinal absolute-position cost。

保留：

```text
running x cost

running vx cost

terminal vx cost

attitude cost

wheel-state cost

input cost

所有 bounds
```

目的不是删除整个 terminal objective，而是只删除 Phase 29 已被证明具有因果作用的 component。

---

## 5.2 T0 Local Corrective Gate

在 Phase 28/29 的 frozen T0 snapshot 上重新执行：

$$
\theta\pm\Delta\theta
$$

$$
\omega_y\pm\Delta\omega_y
$$

要求新的局部 response 满足：

$$
\boxed{
\frac{\partial\dot\omega_y}
{\partial\theta}<0
}
$$

以及：

$$
\boxed{
\frac{\partial\dot\omega_y}
{\partial\omega_y}<0
}
$$

原 Phase 28 数值：

$$
+118.153,\qquad +18.2632
$$

必须由 reinforcing 方向翻转为 restorative 方向。

如果仍为正：

```text
T0 candidate FAIL
```

不得继续 closed-loop。

---

# 6. T0 Minimality Sweep

如果：

$$
Q_{e,x}=0
$$

通过 local gate，则建立有限、预冻结的恢复 sweep：

$$
Q_{e,x}^{new}
=
\alpha Q_{e,x}^{old}
$$

其中：

$$
\alpha\in
\{0,\alpha_1,\alpha_2,\ldots,1\}
$$

具体 grid 在执行前冻结。

目标找：

$$
\boxed{
\alpha_{\max}
}
$$

使得：

```text
pitch corrective derivative restorative

T0 frozen snapshot net action restorative

OCP stationarity / bounds PASS
```

优先选择：

$$
\boxed{
\text{最大的仍然安全 restorative 的 }\alpha
}
$$

而不是无条件使用 0。

这样最大限度保留原 terminal longitudinal design。

---

# 7. T0 Closed-Loop Gate

candidate 通过 local gate 后才运行 T0 static。

至少检查：

```text
pitch

pitch rate

x

vx

requested Fx / Ty

predicted angular acceleration

realized wrench

WBC residual/slack

NMPC stationarity

contact / torque margin
```

要求：

1. 不再复现 Phase 27/28 的单调 pitch drift；
2. 原 0.58 s pitch failure 消失；
3. diagnostic continuation 内不出现更晚同机制发散；
4. x/vx 不因 terminal-x 修复出现明显新的不稳定；
5. WBC realization 和 plant-match 继续 PASS。

---

# 8. Repair Branch B：T1 Attitude-Coupling Repair

T1 不修改 terminal formulation。

首先只修改：

$$
\boxed{
Q_{\mathrm{att}}^{new}
=
\beta Q_{\mathrm{att}}^{old}
}
$$

其中 attitude group 使用 Phase 29 中对应的原 production attitude cost group。

不得同时提高：

$$
Q_x,\ Q_v
$$

否则无法证明修复来自 attenuation of attitude dominance。

---

# 9. T1 Local Net-Action Gate

在 Phase 29 T1 authority snapshot 上，已知：

$$
e_x<0,\qquad e_v<0
$$

原：

$$
a_x^{net}=-0.0118472\ {\rm m/s^2}
$$

candidate 必须满足：

$$
\boxed{
e_x<0,\ e_v<0
\Rightarrow
a_x^{net}>0
}
$$

即 longitudinal corrective direction 翻转。

同时检查 attitude 本身不能变为非恢复：

$$
\boxed{
\theta>0
\Rightarrow
\dot\omega_y<0
}
$$

以及对应负扰动的对称恢复方向。

---

# 10. T1 Attitude Scaling Sweep

预冻结：

$$
\beta\in
\{\beta_1,\beta_2,\ldots,1\}
$$

从原值 1 向下降低。

每个 \(\beta\) 记录：

```text
net longitudinal acceleration

pitch corrective acceleration

roll corrective acceleration

requested Fx common

requested Ty common

objective decomposition

attitude contribution

longitudinal contribution

wheel-rate contribution

stationarity

bounds / active set
```

目标寻找：

$$
\boxed{
\beta_{\max}
}
$$

满足同时：

$$
a_x^{net}>0
$$

和：

$$
\text{attitude corrective direction remains restorative}
$$

选择最大可行 \(\beta\)，以尽量保持原 attitude authority。

---

# 11. Wheel-Rate Secondary Interaction Gate

只有当 attitude scaling 后：

```text
longitudinal direction 已恢复
但 corrective magnitude 仍显著不足
```

才允许进入 wheel-rate secondary gate。

测试：

$$
Q_{\dot\xi}^{new}
=
\gamma Q_{\dot\xi}^{old}
$$

但必须单独开 candidate branch。

禁止：

```text
同时修改 beta 和 gamma 后直接判 PASS
```

流程必须是：

```text
attitude-only candidate
        ↓
freeze
        ↓
wheel-rate secondary candidate
```

只有 Phase 29 已证明的 secondary interaction 仍在限制闭环时才允许保留 wheel-rate 修改。

---

# 12. Candidate Selection Rule

候选选择不是“哪个跑得最好”，而是：

$$
\boxed{
\text{smallest causal formulation change}
}
$$

优先级：

## T0

```text
1. terminal x weight reduction only
2. only if needed: terminal longitudinal group restructuring
3. only if still unresolved: terminal/running responsibility redesign
```

## T1

```text
1. attitude scaling only
2. only if needed: wheel-rate secondary scaling
3. only if still unresolved: structured cross-state cost redesign
```

---

# 13. Combined Candidate

T0 与 T1 各自独立关闭后，才生成 combined production candidate：

```text
T0-approved terminal repair
+
T1-approved attitude repair
```

不能在两支独立验证前直接组合。

组合后重新验证 frozen snapshots：

```text
T0 authority snapshot

T1 authority snapshot
```

确保：

```text
T0 repair 未被 T1 修改破坏

T1 repair 未被 T0 修改破坏
```

---

# 14. Full Closed-Loop Validation

组合 candidate 再运行：

```text
T0 static

T1 straight start-cruise-brake

T2 left

T2 right
```

T3 暂不作为本 Phase blocking gate。

---

# 15. T0 Acceptance

T0 至少要求：

```text
original 0.58 s failure removed

pitch remains bounded

pitch rate decays / remains bounded

x tracking remains acceptable

NMPC corrective oracle restorative

WBC realization healthy

plant acceleration agreement healthy
```

并且不得出现：

```text
同一 pitch mechanism 只是延迟出现
```

---

# 16. T1 Acceptance

T1 至少要求：

```text
original 0.45 s x-envelope failure removed

x / vx corrective action has correct sign

base starts following advancing reference

attitude remains bounded

wheel common state remains acceptable

NMPC stationarity / bounds healthy
```

---

# 17. T2 Validation

T2 只在 T0/T1 组合修复后检查。

由于 Phase 29：

```text
T2 right = same as T1

T2 left = not_same
```

因此：

### T2 right

要求 T1 attitude-dominant mechanism 不再出现。

### T2 left

不得假设 T1 修复一定有益。

检查：

```text
longitudinal action

yaw

roll

left/right wrench distribution

xi_delta

contact symmetry
```

如果 left 出现新失效：

```text
T2-left = new attribution required
```

不得继续调 T1 参数去兼容它。

---

# 18. Regression Against Root-Cause Evidence

修复成功不仅要求 closed-loop PASS，还要求：

$$
\boxed{
\text{原 Phase 29 root-cause oracle 按预测发生变化}
}
$$

例如：

## T0

原：

```text
terminal longitudinal present
→ non-restorative
```

新：

```text
reduced terminal longitudinal
→ restorative
```

## T1

原：

```text
full attitude cost
→ non-restorative
```

新：

```text
reduced attitude dominance
→ restorative
```

如果 closed-loop 变好，但 root-cause oracle 没按预测改变：

```text
REWORK
```

因为这说明可能是其他未归因因素偶然补偿。

---

# 19. Solver / Component Regression

每个 candidate 必须继续满足：

```text
generated model parity

objective reconstruction

full-horizon defect

projected stationarity

input/state bounds

cold reset determinism

production SQP-RTI deadline

WBC 42D/104 hard-row parity

fault / exact-zero / latch / reset

non-overwrite

Phase21～29 default regression
```

---

# 20. Failure Classification

如果 candidate 失败，只允许以下结论。

## R30-A

```text
T0 terminal repair insufficient
```

说明需要进一步 terminal structure redesign。

---

## R30-B

```text
T1 attitude scaling insufficient
```

说明需要进一步 structured cross-state cost redesign。

---

## R30-C

```text
local corrective behavior repaired
but closed loop still unstable
```

此时才重新打开：

```text
bandwidth / fast low-level stabilization
```

作为后续 Phase 候选。

---

## R30-D

```text
T0/T1 repaired
but T2 introduces new mechanism
```

另开 turning-specific Phase。

---

## R30-E

```text
repair breaks OCP feasibility / stationarity / resource
```

说明 formulation trade-off 需要重新设计，不能靠继续降权解决。

---

# 21. Phase PASS 条件

Phase 30 PASS 要求：

```text
T0 local corrective gate PASS

T1 local corrective gate PASS

T0 closed-loop PASS

T1 closed-loop PASS

combined candidate preserves both

T2 right no longer reproduces T1 mechanism

T2 left no unclassified blocking regression

NMPC component gates PASS

WBC / plant gates remain PASS

fault / replay / regression PASS

no safety threshold relaxation

no hidden task added
```

---

# 22. Phase 输出

至少形成：

```text
1. PLAN

2. frozen repair-method config

3. T0 terminal-repair sweep report

4. T0 corrective-oracle report

5. T0 closed-loop report

6. T1 attitude-scaling sweep report

7. T1 corrective-oracle report

8. wheel-rate secondary interaction report
   （only if needed）

9. T1 closed-loop report

10. combined candidate report

11. T2 left/right validation

12. solver/component regression

13. repair-causality matrix

14. REVIEW

15. RECORD
```

---

# 23. Phase Boundary

本 Phase 允许：

$$
\boxed{
\text{修复 Phase 29 已证明的 NMPC formulation root cause}
}
$$

本 Phase 不允许：

$$
\boxed{
\text{为了 closed-loop PASS 引入新的未归因结构}
}
$$

如果 T0/T1 formulation 修复后仍然存在闭环不稳定，下一 Phase 才重新评估：

$$
\boxed{
\text{是否存在真正的 fast low-level stabilization gap}
}
$$

在此之前不得增加 WBC pitch/base-X/rolling task。
