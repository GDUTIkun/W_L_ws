
---
title: Minimal WBC 任务必要性审计与性能测试框架
date: 2026-08-29
status: frozen-test-framework
tags:

* minimal-WBC
* WBC
* NMPC
* task-ablation
* performance-test
---
# 1. 研究目的

当前 NMPC–WBC 分层控制中，NMPC 已负责：

* 基座速度；
* 基座位置 / 高度；
* roll / pitch / yaw；
* 左右轮位；
* 左右轮位速度；
* interaction wrench 规划。

而 WBC 中又包含：

* rolling / speed task；
* wheel-position common task；
* wheel-position differential task；
* height task；
* pitch task；
* common-\(F_x\) correction；
* contact task；
* wrench realization；
* regularization。

因此本阶段的核心问题为：

> **在上层 NMPC 已经负责状态控制与运动规划的前提下，下层 WBC 为维持平地正常运动究竟最少需要哪些独立控制任务？**

目标不是机械追求任务数量最少，而是建立：

$$
\boxed{
\text{Minimal Stable WBC}
}
$$

并区分：

1. 必须保留的任务；
2. 有作用但可以被剩余架构替代的任务；
3. 基本冗余的任务；
4. 可能与上层或其他 WBC task 产生竞争的任务。

---

# 2. 本阶段测试边界

本阶段只研究这些 WBC 辅助任务在**进入地形测试之前的平地工作域**中的必要性。

因此测试范围限定为：

$$
\boxed{
\text{Flat-ground nominal operating-envelope audit}
}
$$

包含：

* 平地静止；
* 平地直线运动；
* 平地持续转向；
* 平地左右腿非对称恢复。

本阶段明确不包含：

* 坡地；
* 台阶；
* 连续不平地形；
* 地形诱导接触法向变化；
* 单轮支撑；
* 腾空；
* 大冲击；
* terrain contact transition。

原因是当前被审查的冗余任务均在进入地形测试前已经加入。

因此当前首先回答：

> **在原本平地控制问题中，这些任务是否必要？**

地形环境下是否需要额外低层任务属于后续独立问题。

---

# 3. Minimal WBC 定义

Minimal WBC 保留所有物理与安全硬约束。

## 3.1 必须保留的硬约束

包括：

* 完整整身动力学；
* 执行器力矩限制；
* 法向接触力非负；
* 摩擦锥 / 摩擦棱锥；
* 膝关节及其他机构安全约束。

即：

$$
M(q)\ddot q+h(q,\dot q)
=
S\tau+J_c^T\lambda
$$

以及：

$$
\tau_{\min}\le\tau\le\tau_{\max}
$$

$$
\lambda_n\ge0
$$

$$
\lambda\in\mathcal C_\mu
$$

这些不属于 task ablation 对象。

## 3.2 Minimal WBC 保留的软目标

Minimal WBC 定义为：

$$
\boxed{
J_{\min}
=
J_{\mathrm{wrench}}
+
J_{\mathrm{contact}}
+
\epsilon J_{\mathrm{reg}}
}
$$

其中：

### Wrench realization

尽量实现 NMPC 输出的 interaction wrench：

$$
w_{\mathrm{feas}}
=
w_{\mathrm{des}}+s_w
$$

并最小化：

$$
J_{\mathrm{wrench}}
=
\|W_s s_w\|^2
$$

### Contact task

维持必要的轮地接触运动学一致性：

$$
a_c
=
J_c\ddot q+\dot J_c\dot q
$$

### Weak regularization

仅用于：

* 去除冗余解；
* 限制异常大的 \(\ddot q\)；
* 限制异常大的 \(\tau\)；
* 限制异常大的 \(\lambda\)；
* 改善数值条件。

正则化不得承担主要状态稳定职责。

---

# 4. 待审计辅助任务

当前重点审查：

```text
R   = common rolling / speed task
WC  = wheel-position common task
WD  = wheel-position differential task
H   = base height task
P   = base pitch task
FX  = common-Fx feedback / correction
```

这些任务不能预先定义为“冗余”或“必要”。

最终必须由实验确定。

---

# 5. 总体实验原则

采用：

$$
\boxed{
\text{Baseline}
\rightarrow
\text{Frozen-parameter screening}
\rightarrow
\text{Minimal WBC}
\rightarrow
\text{Retuning}
\rightarrow
\text{必要时 Add-back}
}
$$

不采用所有任务的完全组合搜索。

不进行 \(2^N\) 全因子排列。

只在已有实验出现明确交互迹象时增加组合实验。

---

# 6. 调参原则

任务删除与重新调参必须分开。

## 6.1 第一轮：Frozen-parameter test

删除某个任务后：

* NMPC 参数不变；
* 其他 WBC task 权重不变；
* PD 增益不变；
* solver 设置不变；
* reference 不变；
* plant 参数不变。

只允许：

* 删除任务本身；
* 删除已经失去意义的对应参数；
* 必要的纯数值修正。

目的：

> 判断该任务在当前已调好架构中承担了什么作用。

## 6.2 第二轮：Retuned test

只有重要候选架构才重新调参。

重点重新调参：

1. Minimal WBC；
2. 删除后明显失败的关键任务；
3. 删除后明显改善、怀疑存在任务冲突的架构。

目的：

> 判断该任务是否真正不可替代，而不是仅仅因为原参数针对旧架构调过。

因此必须区分：

```text
Frozen result
```

和：

```text
Retuned result
```

---

# 7. 测试工况

本阶段固定四个基础平地工况。

---

## T0：平地静止平衡

### 输入

```text
v_ref = 0
yaw_rate_ref = 0
正常初始构型
```

### 目的

检查：

* 基本平衡；
* 高度保持；
* pitch 保持；
* wheel common 漂移；
* wheel differential 漂移；
* wrench realization。

主要激励：

```text
height
pitch
equilibrium
wrench realization
```

---

# 8. T1：平地直线运动

采用：

```text
v_ref = 0.20 m/s
```

包含：

```text
启动
→ 匀速
→ 制动
```

所有架构使用完全相同的 reference profile。

### 目的

主要检查：

* 前向速度；
* common wheel position；
* pitch；
* height；
* common-\(F_x\)；
* rolling；
* wrench realization。

主要用于审查：

```text
R
WC
H
P
FX
```

---

# 9. T2：平地持续转向

采用固定代表性工况：

```text
v_ref        = 0.20 m/s
yaw_rate_ref = ±0.08 rad/s
```

分别执行：

```text
左转 360°
右转 360°
```

不在当前阶段重新扫描完整：

```text
速度 × yaw rate × 圈数
```

矩阵。

### 目的

检查：

* yaw tracking；
* 左右 wheel differential；
* 左右执行器 / contact 分配；
* 转向时 pitch / roll；
* 删除任务后的方向偏置；
* wrench realization。

特别用于审查：

```text
WD
R
FX
```

---

# 10. T3：左右腿非对称恢复

人为设置：

$$
\xi_\Delta(0)=+10\text{ mm}
$$

以及：

$$
\xi_\Delta(0)=-10\text{ mm}
$$

其余条件保持一致。

### 目的

主动激励 differential mode，检查：

* 左右轮位差是否恢复；
* 是否持续漂移；
* 删除 WD 后是否出现明显失效；
* 恢复过程是否损害 yaw / speed / pitch；
* 是否依靠执行器饱和维持。

主要用于审查：

```text
WD
anti-split / differential stabilization mechanism
```

---

# 11. 本阶段暂不进行外扰边界测试

外扰恢复属于重要整机性能，但不是第一轮 task necessity audit 的主要归因工况。

因此：

```text
T0 静止
T1 直线
T2 转向
T3 左右不对称
```

完成架构筛选以后，仅对最终候选 Minimal Stable WBC 再增加外扰恢复测试。

---

# 12. 第一类指标：Validity / Hard Gate

每次实验首先判断该结果是否具有比较资格。

至少记录：

```text
stable
simulation_completed
failureReason

NMPC_status
NMPC_fault_ratio

QP_feasible_ratio

NMPC_solve_p99
QP_solve_p99

dynamics_residual_max

torque_margin_min
friction_margin_min
normal_force_min

NaN / Inf
```

硬门槛原则：

* simulation 必须正常完成；
* 不允许 NaN / Inf；
* NMPC 不允许持续 fault；
* QP 必须保持基本可行；
* 动力学硬约束 residual 应保持数值精度；
* torque / friction / normal-force 约束不得持续违规。

注意：

$$
w_{\mathrm{feas}}-w_{\mathrm{des}}
$$

不是动力学硬约束残差。

由于 WBC 明确允许 wrench slack，因此 wrench realization error 不要求接近 \(10^{-6}\)，而作为独立性能指标评价。

---

# 13. 第二类指标：Closed-loop Performance

统一保留：

## 前向速度

```text
vx_RMSE
vx_peak_error
vx_steady_state_error
```

## 高度

```text
z_RMSE
z_peak_error
```

## Pitch

```text
pitch_RMS
pitch_peak
```

## Common wheel position

```text
xi_c_RMSE
xi_c_peak_error
```

## Differential wheel position

```text
xi_delta_peak
xi_delta_tail
```

T3 额外记录：

```text
xi_delta_settling_time
```

T2 转向额外记录：

```text
yaw_rate_RMSE
yaw_final_error
left_right_symmetry
```

---

# 14. 第三类指标：WBC Realization

这是本阶段区别于普通整机性能测试的重点。

NMPC 当前主要主动 wrench 通道为：

$$
F_x,\qquad F_z,\qquad T_y
$$

分别记录：

```text
Fx_wrench_RMSE
Fz_wrench_RMSE
Ty_wrench_RMSE

Fx_slack_RMS
Fz_slack_RMS
Ty_slack_RMS

Fx_slack_peak
Fz_slack_peak
Ty_slack_peak
```

必要时增加归一化 realization error：

$$
e_w^{norm}
=
\frac{
\operatorname{RMS}(w_{\mathrm{feas}}-w_{\mathrm{des}})
}{
\operatorname{RMS}(w_{\mathrm{des}})+\epsilon
}
$$

用于不同架构和不同工况之间比较。

---

# 15. Task Residual

所有仍启用的主要 soft task 应记录 residual。

包括：

```text
rolling_residual_RMS

contact_rolling_residual_RMS
contact_lateral_residual_RMS
contact_normal_residual_RMS

wheel_common_residual_RMS
wheel_diff_residual_RMS

height_residual_RMS
pitch_residual_RMS
```

关闭的任务标记：

```text
OFF
```

而不是记为零 residual。

Task residual 的主要目的不是设置统一通过门槛，而是用于判断：

$$
\boxed{
\text{task competition}
}
$$

例如：

```text
删除 rolling 后：

vx tracking ≈ 不变
Fx wrench realization 明显改善
```

则需要重点检查：

```text
rolling task
↔
Fx wrench realization
```

是否存在竞争。

---

# 16. 第四类指标：Control Resource

统一记录：

```text
tau_RMS
tau_peak
torque_saturation_ratio

torque_margin_min

friction_margin_min
normal_force_min
```

目的：

> 防止某个简化架构表面 tracking 更好，但实际上依靠更大的控制资源或长期接近约束边界维持。

---

# 17. 相对性能评价

本阶段不为所有性能指标重新制定大量绝对通过门槛。

采用：

$$
\boxed{
\text{Hard absolute gate}
+
\text{relative comparison with Current WBC}
}
$$

Current WBC 作为性能基准。

定义：

$$
\Delta M
=
\frac{
M_{\mathrm{candidate}}
-
M_{\mathrm{current}}
}{
M_{\mathrm{current}}
}
$$

对于误差类指标，可采用第一版工程判断带：

```text
变化 ≤ ±10%
→ 基本等效

改善 > 10%
→ 明显改善

恶化 10%～25%
→ 轻度退化

恶化 > 25%
→ 明显退化

失稳 / 硬约束失败
→ Fail
```

该比例仅作为固定筛选标准，不作为理论阈值。

对于 baseline 接近 0 的指标，不使用百分比评价，应使用绝对 tolerance。

---

# 18. 实验阶段

## Stage 0：Current WBC Baseline

首先冻结当前完整 WBC。

执行：

```text
Current × T0
Current × T1
Current × T2
Current × T3
```

形成统一 baseline。

后续所有架构均与该 baseline 比较。

---

# 19. Stage 1：Frozen-parameter Screening

第一轮优先进行高信息量消融。

重点：

```text
Current - R
Current - FX
Current - H
```

其他低优先级任务可以先进行组消融：

```text
Current - {WC, WD, P}
```

如果组删除没有明显影响，则暂不立即增加大量单项实验。

如果组删除产生明显失效，再进一步拆分：

```text
Current - WC
Current - WD
Current - P
```

---

# 20. Stage 2：Minimal WBC

构建：

```text
Minimal =
Hard Constraints
+ Wrench Realization
+ Contact
+ Weak Regularization
```

首先使用 frozen 参数运行。

然后针对 Minimal 架构重新调参。

因此至少得到：

```text
Minimal-Frozen
Minimal-Retuned
```

重点比较：

```text
Current
vs
Minimal-Retuned
```

而不是只拿 Current 的旧参数直接判断 Minimal 是否有效。

---

# 21. Stage 3：定向 Add-back

只有 Minimal WBC 无法稳定，或者存在明显性能缺失时进行。

从：

```text
Minimal
```

出发，优先根据前面实验结果增加最可能必要的任务。

例如：

```text
Minimal + R
Minimal + FX
Minimal + H
Minimal + WD
```

不要求所有任务全部做一次 add-back。

哪个能力缺失，就优先添加最可能承担该职责的 task。

---

# 22. Stage 4：Interaction Test

只有出现以下情况才增加组合实验：

```text
Minimal + A    无法恢复
Minimal + B    无法恢复
```

但已有证据怀疑：

```text
Minimal + A + B
```

可能恢复。

此时才测试任务交互。

不预先进行完整两两组合搜索。

---

# 23. Early-stop 规则

为了减少实验工作量，各架构不一定必须无条件完成 T0～T3。

建议顺序：

```text
T0
↓
T1
↓
T2
↓
T3
```

如果某个架构在前一工况已经出现：

* 明确失稳；
* 持续约束违反；
* 可重复 QP / NMPC failure；
* 明显不可接受的状态发散；

则停止后续更复杂工况。

该架构标记：

```text
Fail at T?
```

并记录：

```text
failureReason
failureTime
primaryFailedState
constraintStatus
wrenchRealization
taskResiduals
```

无需继续浪费完整测试时间。

---

# 24. Task 与重点工况映射

为了进一步降低工作量：

## Rolling / speed task

重点：

```text
T1
T2
```

## Wheel common task

重点：

```text
T1
```

## Wheel differential task

重点：

```text
T2
T3
```

## Height task

重点：

```text
T0
T1
```

## Pitch task

重点：

```text
T0
T1
T2
```

## Common-Fx correction

重点：

```text
T1
T2
```

非重点工况只承担 smoke / stability check。

---

# 25. 最终结果分类

每个 task 最终只归入以下几类。

## A. 明显冗余

表现：

```text
删除后性能基本等效
+
重新调参后仍等效
```

则认为：

> 当前没有证据表明该 task 承担不可替代职责。

---

## B. 有贡献，但可以替代

表现：

```text
Frozen 删除后性能下降
```

但：

```text
Retuned 后恢复
```

说明：

> 该 task 在原架构中承担了一部分控制作用，但不是结构上不可替代。

---

## C. 当前架构必要

表现：

```text
删除后产生明确、可重复失效
```

并且：

```text
合理 Retuning 后仍无法恢复
```

而：

```text
Minimal + task
```

能够针对性恢复对应能力。

说明：

> 该 task 在当前 NMPC–WBC 架构下具有关键必要性。

但不能直接推广为：

> 所有 NMPC–WBC 都理论上必须包含该 task。

---

## D. 疑似任务冲突

表现：

```text
删除 task 后
tracking 不恶化
甚至改善
```

同时：

```text
wrench realization 改善
task residual / torque / constraint margin 改善
```

则重点考虑：

$$
\boxed{
\text{duplicated feedback / cross-layer competition}
}
$$

---

# 26. 最终实验输出

每次运行至少保存：

```text
config snapshot
summary.csv
failureReason
representative time series
```

最终形成统一比较表，例如：

| Metric              | Current | -R | -FX | -H | Minimal | Minimal Retuned |
| ------------------- | ------: | -: | --: | -: | ------: | --------------: |
| stable              |         |    |     |    |         |                 |
| vx RMSE             |         |    |     |    |         |                 |
| z RMSE              |         |    |     |    |         |                 |
| pitch RMS           |         |    |     |    |         |                 |
| xi_c RMSE           |         |    |     |    |         |                 |
| xi_delta peak       |         |    |     |    |         |                 |
| Fx realization RMSE |         |    |     |    |         |                 |
| Fz realization RMSE |         |    |     |    |         |                 |
| Ty realization RMSE |         |    |     |    |         |                 |
| torque RMS          |         |    |     |    |         |                 |
| torque margin min   |         |    |     |    |         |                 |
| friction margin min |         |    |     |    |         |                 |
| QP feasible ratio   |         |    |     |    |         |                 |

不同 T0～T3 工况分别输出，不使用单一综合分数把不同失效模式合并。

---

# 27. 本阶段最终流程

```text
Current WBC
    │
    ▼
Baseline：T0 / T1 / T2 / T3
    │
    ▼
Frozen task screening
    │
    ├─ Current - R
    ├─ Current - FX
    ├─ Current - H
    └─ Current - {WC, WD, P}
    │
    ▼
Minimal WBC
    │
    ├─ Frozen
    └─ Retuned
    │
    ▼
是否稳定且性能可接受？
    │
 ┌──┴──┐
 Yes    No
 │       │
 ▼       ▼
候选       定向 Add-back
Minimal    找缺失职责
Stable     │
WBC        ▼
 │      必要时任务组合
 └───────┬───────
         ▼
Minimal Stable WBC
         │
         ▼
后续完整性能验证
```

---

# 28. 本阶段最终要回答的问题

最终不是简单回答：

> 哪个 task 加上以后比较稳？

而是回答：

1. Current WBC 中哪些 task 可以删除而基本不损失性能？
2. 哪些 task 删除以后只需要重新调参即可恢复？
3. 哪些 task 在当前架构下真正不可替代？
4. 哪些 task 与 NMPC wrench realization 或其他 WBC task 存在明显竞争？
5. Minimal WBC 是否能够独立实现平地静止、直线、转向和左右不对称恢复？
6. 如果不能，最小缺失控制职责是什么？
7. 最终平地工作域下的 Minimal Stable WBC 应由哪些任务组成？

最终目标是得到清晰的职责边界：

```text
NMPC：
决定机器人应该如何运动，以及需要什么 interaction wrench

WBC：
在完整动力学、接触和执行器约束下实现该 wrench

必要低层内环：
只有被实验明确证明不可替代时才保留
```
