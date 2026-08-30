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

