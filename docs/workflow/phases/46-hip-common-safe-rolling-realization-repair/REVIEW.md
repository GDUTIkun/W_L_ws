# Phase 46 Review

结论：`REWORK`
日期：2026-08-31  
classification：`P46-E — multiple remaining mechanisms`

## Finding

**BLOCKING — DG46-EQ FAIL。** 唯一 repair 已按冻结定义实现为
`P_safe=I-hh^T`、`h=(e6+e9)/sqrt(2)`，只投影 slip task realization row；component test
确认 columns 6/9 之外逐元素不变、hip-common 分量为零、projection idempotent，且 xi task、task
scale、bias/target、contact、weight 与 solver 未改。

但在 Phase45 compatible-H0 的同一 state/wrench 上，tick0 actual
`ddxi_L=+0.0785440164 m/s2`、`ddxi_R=+0.1603087132 m/s2`，分别超过冻结
`0.05 m/s2` 上限。Phase45 compatible-H0 原先两侧 residual 约为零，因此静态删除该 direction
并不保持原 equilibrium：bilateral hip-common freedom 不只承载 harmful directional response，也参与
零输入下 rolling-bias、xi 与全身动力学的共同 realization。

这不是数值闭合或接触失效：material tangent acceleration 为
`[-0.00116445,-0.000555747] m/s2`，双侧 load 为 `[30.9303,31.4501] N` 且 rolling active；
hard=`3.46e-10`、slack=`3.84e-4`、minimum torque margin=`1.9973 Nm`，whole dynamics residual
`7.11e-15`、contact reconstruction residual `0`。fresh replay semantic error=`0`。

## Stop Order

DG46-EQ 是首个 mandatory gate，故 AUTH、REAL、SHORT、10 s 与 post-repair authority reaudit
全部未进入；正式 evidence 中没有 `+/-` 或 trajectory probe。调试期间用于校验 runner 的预探针不纳入
Phase46 authority，也不用于放行或 classification。

## Verification

- `.venv`：MuJoCo 3.7.0、NumPy 2.2.6、SciPy 1.15.3；`py_compile` PASS；
- targeted `colcon build` PASS；core 17/17、adapter 6/6，aggregate 35 tests、0 failure；
- formal-v1/replay-v1 各6 CSV + 8 JSON，numeric non-finite=0；
- formal与replay的native tick0记录均只有`pre_command/post_command`，无trajectory integration；
- `git diff --check` PASS。

## Verdict

静态删除 bilateral hip-common realization freedom **不足以**让 repair 在 actual plant 上成立；它在
authority audit 之前已破坏 frozen compatible equilibrium。该可靠失败既不是单纯的 authority loss
或 mode migration，也不能归为 evidence unreliable；在冻结 taxonomy 中归入
`P46-E — multiple remaining mechanisms`。不创建 RECORD，不调参，不尝试 soft penalty、coupled task、
precompensation 或 dynamic projection。

Machine-readable authority：
[formal-v1](evidence/automated/hip-common-safe-formal-v1/summary.json) 与
[fresh replay-v1](evidence/automated/hip-common-safe-replay-v1/summary.json)。

## REWORK — Frozen Nominal, Limited Increment

结论：`REWORK`
classification：`B-EQUILIBRIUM_PRESERVED_BUT_COUPLING_REMAINS`

唯一新增的约束只在 external `slip-common` delta 非零时生效：将 QP 的 bilateral
hip-common acceleration 固定为 frozen compatible-H0 nominal 值
`-0.009961062735978504 rad/s2`；zero delta 完全保留原 Phase45 rolling realization。
它不是 static deletion、target/bias offset、gain/weight/wrench tuning 或 cross-gain
precompensation。

**DG46I-EQ PASS。** zero delta 下 `ddxi=[4.06e-14,2.87e-14] m/s2`，material tangent
`[1.04e-16,-6.94e-17] m/s2`，loads `[30.970,31.499] N`，hard `4.68e-11`，slack
`0.001344`，torque margin `1.998 Nm`，whole-dynamics/contact residual
`2.13e-14/0`。因此 nominal equilibrium 没有被改变。

**DG46I-AUTH FAIL。** hard equality 在 QP 端确实清除了 hip-common increment：QP hip
common gain `2.97e-11 rad/s2/(m/s2)`，对应 `ddxi_c` contribution `-8.25e-12`。但相同
fixed-state command 经 MuJoCo realization 后，hip-common gain 回到 `+14.4813`，并贡献
`-4.027036` 至 `ddxi_c`。故 actual cross 为 `-4.2950569`，相对 frozen
`-4.2950932` 的降低只有 `8.45e-6`（约 `0.000845%`），远未满足收敛门。

slip self authority 仍为正 `+0.0308533`，没有被投影摧毁。knee-common 是次级负贡献
`-0.267966`；base `5.18e-17`、native wheel `0`、`(Jdot,v)` `4.51e-15`、hip differential
`+0.000310`，均没有取代 hip-common 为主导路径。`+/-` branches 与 `1/0.5/0.25` scales
一致；leg DOF/mode decomposition closure `<=8.88e-16`，whole dynamics/contact closure
`<=1e-8`。fresh replay semantic max error 为 `0`，且所有记录为 tick0 command snapshots，未做
trajectory/REAL/SHORT/10 s。

这说明 nominal hip-common 可以保留，但静态地限制其 **QP increment** 不能限制其 **actual plant
increment**；failure 仍是 QP-to-MuJoCo realization，而不是 authority loss。Phase46 保持
`review/REWORK`；不创建 RECORD，不尝试下一 repair。

Machine-readable evidence：
[incremental formal-v1](evidence/automated/incremental-hip-common-formal-v1/summary.json) 与
[fresh replay-v1](evidence/automated/incremental-hip-common-replay-v1/summary.json)。
