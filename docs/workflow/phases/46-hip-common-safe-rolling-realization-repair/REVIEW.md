# Phase 46 Review

结论：`REWORK`
日期：2026-08-31  
classification：`B-HARMFUL-CROSS-REMAINS`

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

## REWORK — Frozen-state directional realization attribution

结论：`B-CONTACT_REALIZATION_DOMINANT`。本节只扩展既有 P46-R03 的 compatible-H0、tick0、
fixed-state `slip-common only` probes；不改变 gain、weight、wrench、state、Model B、WBC、contact、
friction 或 solver，且不运行 trajectory。

在同一个 frozen-state realized constrained-dynamics balance 中，以 bilateral hip-common selector
乘 `M^-1` 映射每个 generalized-force channel，并把 MuJoCo realized contact 与
`other_constraint` 显式保留在 RHS。central directional gain 为：QP hip-common acceleration
`+2.97e-11`、MuJoCo actual hip-common acceleration `+14.4813154` rad/s2/(m/s2)；actuator
contribution `+14.5417603`，QP-predicted contact `+34.2295966`，MuJoCo actual contact
`-0.1303392`，contact-realization difference `-34.3599358`，remaining/passive
`+0.0698943`。因此 QP contact prediction 与 MuJoCo realized contact 的 mapping difference
远大于 actuator 与 remaining，并是 QP hip-common cancellation 没有在 plant 中实现的主 gap。

`actuator + actual contact + remaining = +14.4813154096434`，对 actual hip-common
`+14.4813154096469` 的 contribution closure 为 `3.44e-12`；force-balance closure
`1.07e-12`，whole-dynamics/contact closure `3.55e-14`。`+/-` split 为 `2.67e-5`，
`1/0.5/0.25` scale convergence 为 `4.00e-5`，最大 QP hip-common residual
`7.31e-10`。fresh replay semantic error 为 `0`。故当前 evidence 已否定把单纯 QP-space
projection 或 penalty 当作主要 repair 方向；该层只说明 gap 位于 contact realization，未授权修复。

Machine-readable evidence：
[attribution formal-v5](evidence/automated/incremental-hip-common-attribution-formal-v5/incremental-authority/constrained-hip-common-attribution.json) 与
[fresh replay-v5](evidence/automated/incremental-hip-common-attribution-replay-v5/incremental-authority/constrained-hip-common-attribution.json)。

## REWORK — Contact mapping / wrench realization parity

结论：`B-WRENCH_REALIZATION_DOMINANT`。本节 supersede 上一节把 QP reduced reduction
直接用于 MuJoCo free-joint Jacobian 所产生的 mapping 混叠；范围仍严格保持 compatible-H0、
tick0、fixed-state `slip-common only`，没有修改 controller、contact model、gain、weight、task、
wrench、solver 或 plant，也没有运行 trajectory。

geometry/frame parity 已由冻结 Model B oracle 独立重建：QP contact map 与 controller 日志最大误差
`4.44e-15`，闭链残差 `1.48e-13 m`；QP 与 MuJoCo 的 rolling/lateral/normal frame 完全一致，
两侧 analytic contact point 差异仅约 `0.06--0.21 mm`，统一到相同 base-control/canonical
generalized-coordinate ordering 后，wrench-map 最大逐元素差 `1.70e-4`。

同一 frozen actual constrained-dynamics mapping 下，固定 QP directional wrench 后，QP 与 MuJoCo
same-wrench hip-common contribution 分别为 `-14.5790947` 与 `-14.5791298`，所以
`(J_MJ^T-J_QP^T)dw_QP` 仅贡献 `-3.51e-5`。MuJoCo actual contact contribution 为
`-0.1002359`，而 `J_MJ^T(dw_MJ-dw_QP)` 贡献 `+14.4788939`；两者闭合总 contact gap
`+14.4788588`，mapping fraction `2.43e-6`、wrench-realization fraction `1.0000024`。

wrench-realization hip-common contribution 以 right wheel 为主：right `+10.4274367`，left
`+4.0514572`。分量上 right `Fr=+9.1501332`、`Fn=+1.2764542`，left
`Fr=+3.5206261`、`Fn=+0.5269844`；其余 `Fl/Mr/Ml/Mn` 合计仅为小量。所有 probes
均保持每轮两个 3D contact，normal-frame error 为 `0`，最小 friction-margin diagnostic
为 `15.199 N`，没有 contact-count、contact-dimension、normal-frame 或 friction-bound switch。
因此现有证据不要求重推 geometry/contact kinematic model；下一层若继续调查，应建模 QP aggregate
6D wrench 到 MuJoCo two-point compliant solver reaction 的 realization relation。当前证据尚不能把
该 relation 唯一归因到 friction、compliance 或某个 solver 子机制，所以不分类为
`D-CONSTRAINT_REALIZATION_SPECIFIC`，且本 Phase 不实施 repair。

`+/-` split `6.94e-5`，`1/0.5/0.25` scale convergence `1.04e-4`，contact decomposition
closure `3.55e-13`；fresh replay semantic error `0`。证据见
[contact parity formal-v7](evidence/automated/incremental-contact-parity-formal-v7/incremental-authority/contact-mapping-wrench-parity.json) 与
[fresh replay-v1](evidence/automated/incremental-contact-parity-replay-v1/incremental-authority/contact-mapping-wrench-parity.json)。

## REWORK — Local 4D QP-solution to plant contact sensitivity

结论：`B-STABLE_BUT_STRONGLY_COUPLED`。本节仅在 compatible-H0、tick0、fixed-state
下，以 command-space calibration 合成四个 QP `Fr/Fn` target directions；每个 direction 均执行
`+/-` 与 `1/0.5/0.25` probes。MuJoCo 每个 probe 只接收一次完整 QP solution 的 actuator torque，
actual contact 始终由原 solver 自洽生成；没有直接施加 QP wrench、没有重算 hypothetical reaction、
没有修改 controller/contact/friction/solver，也没有运行 trajectory。

以 `u=y=[Fr_L,Fn_L,Fr_R,Fn_R]` ordering，central local map 为：

```text
Rc = [[-0.105826, -5.795091, -0.154472, -7.386599],
      [ 0.116083,  1.541001,  0.017805,  0.762607],
      [-0.152731, -7.516716, -0.105773, -5.925631],
      [ 0.023618,  0.892615,  0.114871,  1.602804]]
```

branch split `5.48e-8`、scale convergence `7.76e-9`，所有 probes 的 contact/solver
regime signature 相同；point-to-aggregate wrench closure `3.90e-17`、whole-dynamics/contact
closure `3.55e-14`，fresh replay semantic error `0`。因此该 fixed-state local map 稳定。

但它不是 pure contact-physics law。command-to-u condition number 为 `4493.62`；target 4D u
purity 很高（最大 off-target ratio `7.46e-9`），同时完整 QP solution 明显联动：lateral-force
ratio 最高 `1.208`、moment-equivalent ratio 最高 `15.443`、torque-equivalent ratio 最高
`272.964`，`Rc` off-diagonal Frobenius ratio 为 `0.9866`。结构上，Fr self realization
约 `-0.106` 且反号；Fn self realization 为 `1.54--1.60`；Fn input 到 bilateral Fr 的 cross
terms `-5.80--7.52` 为最大耦合，左右 Fn cross realization 也达到 `0.76--0.89`。

原 slip-common direction 的 aggregate `y-u` 为
`[-2.13166,+0.24378,-5.58777,+0.58867]`，right-wheel Fr 仍为最大 mismatch。每轮两个
point force 严格求和到 aggregate resultant；point redistribution mode 确实显著，norm ratio
约 `1.018`，但 actual aggregate Fr/Fn 本身已发生明显变化，因此不是
point-redistribution-only。当前只具备稳定的 local QP-solution-to-plant sensitivity，尚不足以授权
realization-aware correction 或 inverse map；应继续查 compliant multi-contact / solver-reaction
mechanism。本 Phase 到此停止，不实施 repair。

Machine-readable evidence：
[formal-v4](evidence/automated/contact-realization-sensitivity-formal-v4/contact-realization-sensitivity.json) 与
[fresh replay-v1](evidence/automated/contact-realization-sensitivity-replay-v1/contact-realization-sensitivity.json)。

## REWORK — Torque replay and free-contact-acceleration attribution

结论：`A-FREE_ACCELERATION_DRIVEN`。本节直接复用上一节保存的四个 `Fr/Fn` QP directions，
不重新求 QP。对每个 full-scale branch 记录的 `Delta tau`，在同一 compatible-H0、tick0、
frozen state 下直接施加 `tau0+s Delta tau`，`s={1,0.5,0.25}`，由原 MuJoCo solver 自洽生成
contact reaction；没有修改 controller、contact、friction 或 solver，也没有运行 trajectory。

Stage 0 torque replay PASS：baseline aggregate wrench、point forces 与 hip-common acceleration
逐值相等；所有 replay 对原 QP probes 的 bilateral 6D wrench、Fr/Fn、point forces 与 hip-common
最大相对误差 `3.10e-9`，contact/solver regime signature 全部一致。因此冻结结论：QP contact
wrench 不是 plant direct input；当前 actual reaction 可由 QP solution 的 actuator torque increment
单独重现。

Stage 1 以同一 frozen `M` 和四个 actual contact-point Jacobian 计算
`M^-1 B Delta tau`。`Fn_L` direction 的 torque gain
`[LH,LK,LW,RH,RK,RW]=[6.352,3.609,0.128,9.952,5.816,0.128] Nm/N`，已在
left/right contact 制造 `+41.50/+53.58` rolling free acceleration；contact reaction contribution
为 `-40.54/-52.43`，other-constraint contribution 仅约 `-0.52/-0.50`，最后 actual aggregate
Fr response 为 `-5.795/-7.517 N/N`。`Fn_R` direction 的 torque gain
`[9.594,5.571,0.130,6.588,3.772,0.130] Nm/N`，制造 `+52.50/+42.55` bilateral free rolling
acceleration；contact reaction 为 `-51.43/-41.53`，actual Fr 为 `-7.387/-5.926 N/N`。

四个 wheel/direction 的 solver reaction 均与 free rolling tendency 反号，并抵消
`98.79--98.93%`。更关键的是，`Fn_L` 的 free cross ratio `1.291` 与 actual Fr cross ratio
`1.297` 匹配，`Fn_R` 分别为 `1.234` 与 `1.247`：左右 cross 在 solver 之前已由 bilateral
torque/free motion 形成，solver 主要按约束要求抵消该 tendency，而不是独立生成新的 cross mode。
因此当前 `Fn->Fr` 是正常 constrained-dynamics reaction：QP 的 Fn-labelled solution direction
携带强 bilateral hip/knee torque，先造成 rolling free motion，再由 contact solver 产生相反 Fr。

`+/-` branch split `5.18e-9`、scale convergence `2.67e-9`，point-to-aggregate wrench closure
`3.90e-17`、whole-dynamics/contact closure `3.55e-14`、free/reaction qacc balance normalized
closure `6.48e-8`；fresh replay semantic error `0`。本 Phase 到此停止，不实施 repair。

Machine-readable evidence：
[formal-v1](evidence/automated/torque-free-contact-attribution-formal-v1/torque-free-contact-attribution.json) 与
[fresh replay-v1](evidence/automated/torque-free-contact-attribution-replay-v1/torque-free-contact-attribution.json)。

## REWORK — Fn→Fr root-cause closure

结论保持 `REWORK`；本节只做 compatible-H0、tick0、fixed-state attribution，没有实施 repair。

**Torque-generation mechanism：`T2-ACCELERATION_TASK_COUPLING_DOMINANT`。** 项目真实 QP
increment identity `M Δnudot = B Δtau + Jw^T Δlambda` 的最大 residual 为 `3.392e-9`，
additive torque decomposition closure 为 `0`。Fn_L/Fn_R acceleration-component torque norm
share 为 `94.9461% / 93.7565%`，contact component 为 `7.0881% / 9.8319%`；other 仅为
`2.63e-10` 数值量级。fixed-active-set KKT 对 observed solution 的 relative error
`<=9.16e-9`，xi+rolling excitation torque closure `<=9.27e-11`。因此 bilateral hip/knee
torque 主要由 xi/rolling acceleration objectives 经 KKT dynamics coupling 生成，不是 contact
balancing torque 本身；这解释 generation mechanism，但不单独构成 R3 verdict。

**First material mismatch：`R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH`。** 左右 actual
two-point force map 均为 `rank 5`，condition(nonzero) 约 `50`，缺失方向几乎纯 `Ml`。
compatible nominal wrench 的不可实现 fraction 仅 `8.73e-5 / 1.48e-4`，但 Fn_L 左右
increments 为 `0.35798 / 0.33664`，Fn_R 为 `0.79030 / 0.04910`；该正交分量对 QP
rolling cancellation 的 contribution fraction 为 `0.9995--1.0249`。minimum-norm projected
point forces 在 nominal load 上仍 normal-positive、friction margin positive，故不是 friction、
unilateral 或 regime switch。

Stage C 在 actual frozen points 的 QP contact-space balance closure 为 `1.136e-9`。QP 确实把
large torque-induced rolling motion 与 aggregate contact reaction 放在同一 balance 中，但承担
material cancellation 的 `Ml` 不在 actual point-force image 内。MuJoCo 只接收 torque，随后在
相同 regime 下正常抵消 `98.79--98.93%` free-motion tendency；actual Fr cross ratio 在 solver
之前已经由 bilateral free acceleration 基本形成。因此 solver 不是 first mismatch，也不能描述为
把 normal force 自行转换为 rolling force。

唯一下一 repair layer 冻结为 **actual point-contact-realizable force/wrench parameterization**。
本轮不批准 hip task redesign、hip-common projection/penalty、inverse `R_c`、precompensation 或
contact/friction/solver tuning。

Formal authority 为
[root-cause formal-v3](evidence/automated/root-cause-closure-formal-v3/root-cause-decision.json)；
[fresh replay-v3](evidence/automated/root-cause-closure-replay-v3/summary.json) semantic error=`0`。
`formal-v1` 因 closure metric 与 R3/R4 interpretation 错误已标 rejected；`replay-v1` 因 Markdown
比较器错误中断已标 incomplete；v2 为通过但被 v3 的新增 share 字段 supersede。

Verification：`.venv` MuJoCo `3.7.0`、NumPy `2.2.6`、SciPy `1.15.3`；`py_compile` 与
diagnostic-only C++ operator dump (`-Wall -Wextra -Wpedantic -Werror`) PASS；targeted colcon build
PASS；workspace `35 tests, 0 errors, 0 failures`；formal/replay 共 `5057` numeric fields 无
non-finite；`git diff --check` PASS。Phase46 保持 `review/REWORK`，不创建 RECORD。

## REWORK — point-contact-realizable repair execution

### Implemented candidate

新增独立 `kPhase46PointRealizableRolling` profile。每轮以 contact-frame 轮轴单位向量 `a` 构造
`P_w=diag(I3,I3-a*a^T)`；不改变 42D QP，且 projector 一致进入 dynamics、37-row wrench cone、
minimal interaction-wrench realization 与对外 physical solution。未叠加旧 hip-common projection、
increment hard equality、gain/weight/wrench tuning，也未修改 MuJoCo plant、friction 或 solver。

`DG46P-COMP` PASS：左右 projector 均 rank 5，symmetry/idempotence/null residual 分别不超过
`1.02e-31`，所有 formal probes 的 axial moment 最大 `2.42e-19 Nm`；目标组件测试和 EOM
closure PASS，历史 weighted-WBC golden tests 保持通过。

### Mandatory stop

`DG46P-EQ` **FAIL**。QP 预测的 left/right ddxi 为
`-5.76e-6 / -8.21e-6 m/s2`，但 frozen MuJoCo actual 为
`+0.0377309 / -0.0753842 m/s2`；右侧超过 `abs(ddxi)<=0.05`。其余本次 EQ 指标可信且通过：
material tangent acceleration `+0.000971 / +0.004510 m/s2`、bilateral two contacts、hard
`4.48e-11`、slack `0.0016795`、minimum torque margin `1.99595 Nm`、whole dynamics
`2.13e-14`、contact apply-ft closure `0`。

因此按冻结顺序停在 EQ，不进入 AUTH、REAL、SHORT、10 s 或 RECORD。此前在完成 EQ 判定前误先
生成的一次 full-direction probe 只保留为 diagnostic-only：harmful cross 从 `-4.29509` 降到
`-0.107631`（97.49%），但 actual slip self 为 `-0.140562`、发生反号；该结果不能授权越过
EQ，也不能作为调参理由。

正式 COMP/EQ evidence 为
[formal-v1](evidence/automated/point-realizable-repair-equilibrium-formal-v1/equilibrium-decision.json)；
[fresh replay-v1](evidence/automated/point-realizable-repair-equilibrium-replay-v1/equilibrium-decision.json)
semantic max error `0`。diagnostic-only decision 为
[repair-formal-v4](evidence/automated/point-realizable-repair-formal-v4/repair-decision.json)。依赖探针为
MuJoCo `3.7.0`、NumPy `2.2.6`、SciPy `1.15.3`；workspace `35 tests, 0 errors, 0 failures`。

Verdict：point-realizable layer 已正确实施并关闭 axial unrealizability，但最小 subspace candidate
未通过 actual equilibrium。Phase46 继续 `review/REWORK`，不创建 RECORD；下一步必须先归因
post-R1 equilibrium response mismatch，禁止 friction/solref/solimp/solver 或 task weight tuning。

### Post-R1 equilibrium attribution

在 mandatory EQ stop 后，只对相同 frozen tick0 做 Phase45 compatible-H0 与 point-realizable
candidate 的 before/after causal comparison，不运行原 AUTH/REAL。state、mass、xi map delta 均为
`0`，两者均保持每侧 two-point contact，排除 state/regime switch。

Phase45 actual ddxi 为数值零；point-realizable actual 为
`+0.0377309 / -0.0753842 m/s2`，而 QP ddxi before/after 仅变化
`+1.06e-7 / +5.32e-6 m/s2`。以 frozen plant mass 映射 generalized-force delta：

- actuator-free contribution：`+0.0377318 / -0.0433649`；
- QP contact prediction：`-0.154932 / +0.165665`；
- actual contact response：`+0.0001186 / -0.0328795`；
- actual-minus-QP contact-response gap：`+0.155050 / -0.198545`；
- remaining：`-0.0001195 / +0.0008602`。

actual/causal ddxi closure 均 `<=3.12e-14`，generalized-force closure `2.44e-15`；response-gap
norm 是 observed actual delta 的 `2.9883` 倍。由此冻结新的 first mismatch 为
`R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1`。它说明 rank-5 realizability 修复后，QP contact
prediction 仍不能代表 actual compliant constrained response；不等于授权 inverse map 或经验补偿。

证据为
[post-R1 formal-v1](evidence/automated/post-r1-equilibrium-attribution-formal-v1/post-r1-equilibrium-attribution.json)
与 [fresh replay-v1](evidence/automated/post-r1-equilibrium-attribution-replay-v1/summary.json)，semantic
error `0`。当前没有已冻结的下一 repair law；Phase46 保持 `review/REWORK`。

## REWORK — point-subspace equivalence audit

结论：`C-REFERENCE_POINT_MISMATCH`。本节 supersede 上一节把 current Ml-deletion candidate 当作
exact point-force-image repair 的前提；上一节数值仍可作为 candidate-specific diagnostic，但
`R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1` 不再 authoritative。

actual two-point map 与既有 rank/SVD oracle parity 闭合至 `2.22e-16`。然而 current `P_w` 删除
pure `Ml`，actual missing direction 分别含 `Fr=-2.15265e-4 / -1.52042e-4`；左右
`||P_w-P_G||_2=2.15265e-4 / 1.52042e-4`，最大 principal angle 同量级，mutual containment 与
双向 reconstruction 均 material FAIL。因此 `Range(P_w) != Range(G_p)`。

标准 wrench transport 表明该差异来自 production reference 相对 contact midpoint 的 normal offset；
移到 midpoint 后 missing direction 才严格为 pure `Ml`，transport parity `<=3.47e-18`、往返 closure
`0`。pure-`Ml` 是 reference-point-specific fact，不是 current production reference 下的 exact
geometry fact。

因此 current `DG46P-EQ FAIL` 只能证明 **approximate Ml-deletion candidate fails equilibrium**，
不能证明 exact R1 repair fails equilibrium。Phase46 保持 `REWORK`；本轮不修改 production
reference/projector/controller，也不定义新 repair。详见
[audit](POINT_SUBSPACE_EQUIVALENCE.md) 与
[machine-readable formal](evidence/automated/point-subspace-equivalence-formal-v1/point-subspace-equivalence.json)。

## REWORK — exact R1 point-force-image repair

结论：`EXACT-R1-COMP PASS / EXACT-R1-EQ FAIL`。

唯一 candidate 已从 approximate pure-`Ml` deletion 替换为 frozen actual two-point
`P_G=G_pG_p^dagger`。同一 projector 一致进入 dynamics、wrench cone、interaction realization 与
controller physical output。左右 production/actual projector parity `<=1.55e-15`，mutual
containment `<=9.16e-16`，最大 principal angle `<=1.10e-15 rad`，missing-direction annihilation
`<=4.89e-16`，physical wrench reconstruction `<=1.60e-14`；rank 5、symmetry/idempotence、
EOM/contact algebra与 historical tests 均 PASS。因此 `Range(decision)=Range(G_p)`，R1 已 exact
closed；旧 `DG46P-EQ FAIL` 被 supersede。

COMP 后进入同一 compatible-H0/tick0 EQ。actual `ddxi_L/R` 为
`+0.0379952/-0.0752634 m/s2`，right 超过 `0.05`，故 mandatory EQ FAIL。tangent、bilateral
two-point contact、hard/slack、torque margin 与 whole-dynamics/contact closure 均通过。按 stop rule
未运行 AUTH、REAL、SHORT、10 s 或 trajectory。

before/after causal evidence closure `<=4.85e-14`，但只记录
`EXACT_R1_EQ_FAIL_CAUSAL_EVIDENCE`；没有设计 R2 或第二 repair，原
`R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1` **仍未重新授权**。详见
[exact repair](EXACT_R1_REPAIR.md) 与
[formal-v3](evidence/automated/exact-r1-equilibrium-formal-v3/equilibrium-decision.json)。

## REWORK — post-exact-R1 first-mismatch attribution

结论：`C-MAPPING-OR-REFERENCE-REGRESSION`。exact R1 closure 与 state/contact regime parity 均
PASS，exact post-R1 wrench 的 `G_p` reconstruction residual `<=1.42e-14`。但是同一 wrench 经
production aggregate-wrench map 与 reconstructed frozen point-contact map 后，reduced generalized
force 最大差为 left `2.40757`、right `2.48120`，相对差 `7.511% / 7.503%`。因此按 mandatory
ordering 在 same-wrench mapping gate 停止，不能把后续 response gap 直接冻结为 R2。

frozen causal decomposition 仍可信：actuator/free `+0.0378838/-0.0432577`，QP contact
`-0.0362691/+0.0447233`，actual contact `+0.000232924/-0.0328646`，actual-QP gap
`+0.0365020/-0.0775879 m/s2`，gap/observed ratio `1.01702`；closures `<=4.85e-14`。contact
signature stable，actual solver reaction 与 free-motion tendency 反向，故 solver 不是 first
mismatch。

旧 `R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1` 保留为 historical / approximate-candidate /
non-authoritative；`R2-CONTACT_RESPONSE_MISMATCH_AFTER_EXACT_R1` 不授权。详见
[attribution](POST_EXACT_R1_ATTRIBUTION.md)、[formal-v4](evidence/automated/post-exact-r1-attribution-formal-v4/post-exact-r1-attribution.json)
与 [replay-v4](evidence/automated/post-exact-r1-attribution-replay-v4/summary.json)，semantic error `0`。
本轮未实施 repair，Phase46 保持 `review/REWORK`。

## REWORK — production-reference point-force image

结论：`A-PRODUCTION-REFERENCE-IMAGE-CLOSED`。以 frozen actual two-point map 为起点，使用已
验证的 dual wrench/twist transport 唯一得到 `Gp_prod=Tw Gp_point`。左右 rank 均为 5；transported
full/reduced operator residual 分别 `<=1.67e-16 / <=4.44e-15`，deterministic virtual work PASS。

`Pg_prod=Gp_prod pinv(Gp_prod)` 的 symmetry exact，idempotence `<=1.11e-15`、range containment
`<=1.13e-15`、point-force reconstruction `<=1.37e-14`。missing direction 仍以 `Ml` 为主，但同时
包含 production-reference-specific `Fr/Fn` 分量。

current projector 与 true `Pg_prod` 的 spectral difference 为 left `1.21140e-4`、right
`1.19751e-4`，max element `1.16173e-4 / 1.03252e-4`，dominant column 均为 `Ml`。因此 true
production-reference R1 image 已知，但 current controller 中 R1 尚未 exact closed。本轮未修改
controller/projector，下一允许动作仅为另行实现一个 corrected exact-R1 candidate。详见
[audit](PRODUCTION_REFERENCE_IMAGE_AUDIT.md)、
[formal-v1](evidence/automated/production-reference-image-audit-formal-v1/production-reference-image-audit.json)
与 [replay-v1](evidence/automated/production-reference-image-audit-replay-v1/summary.json)，semantic
error `0`。Phase46 保持 `review/REWORK`。

## REWORK — wrench/generalized-force operator identity

结论：`C-REFERENCE-POINT-MISMATCH`。独立构造 actual `Gp/Jp` 与 production `Aw` 后，raw
`Aw Gp-Jp^T` 在 full coordinates 的 spectral norm 为 left `4.18341e-4`、right
`3.38708e-4`，max element `1.69721e-4 / 1.03252e-4`；reduced 层同样 FAIL。dominant basis
为 left `Fn`，dominant DOF block 为 left base rotation。

actual 与 production reference 在 contact coordinates 的 offset 为 left
`[-1.16173e-4,-1.69721e-4,-3.43335e-5] m`、right
`[+6.06580e-5,0,+1.03252e-4] m`。应用标准 wrench/twist dual transport 后，full residual
`<=1.67e-16`、reduced residual `<=4.44e-15`，deterministic virtual work PASS。因此 first
mismatch 唯一定位为 reference transport，不是 frame/order/sign 或 reduction 本身。

旧 same-wrench audit 的 `-3.51221e-5` hip-common scalar 与 `2.42575e-6` fraction 在其窄范围内
仍有效：它比较同一 numeric wrench 经各模型自己的 verified reference/map 后的单一 selector，未检验
完整六列 operator identity。上一节 7.5% 则由未 transport reference 并混用 production/plant
reduction semantics 产生，正式 supersede；真实 raw relative operator gap 仅约
`1.4e-4--1.9e-4`，且 transport 后消失。

authority consequence：existing exact-R1 projector 相对正确 production-reference point-image
projector 的 max difference 为 left `1.16173e-4`、right `1.03252e-4`，故 exact R1 不再可称为
在 production wrench reference 下 closed；R2 不授权。详见
[operator audit](WRENCH_GENERALIZED_FORCE_OPERATOR_AUDIT.md)、
[formal-v2](evidence/automated/wrench-generalized-force-operator-audit-formal-v2/wrench-generalized-force-operator-audit.json)
与 [replay-v2](evidence/automated/wrench-generalized-force-operator-audit-replay-v2/summary.json)，semantic
error `0`。本轮未实施 repair，Phase46 保持 `review/REWORK`。

## REWORK — corrected production-reference exact-R1 repair

结论：`CORRECTED-R1-COMP PASS / CORRECTED_EXACT_R1_EQ_PASS`。

唯一 `kPhase46PointRealizableRolling` candidate 现使用 frozen compatible-H0 actual two-point
contact line 相对 production aggregate-wrench reference 的完整 contact-frame offset；未改 task、
gain/weight、friction、solver、contact parameter 或 wrench reference，也未叠加 hip-common
projection/hard equality、inverse map 或 precompensation。同一 `Pg_prod` 继续进入 dynamics、37-row
wrench cone、minimal interaction-wrench realization 与 controller physical output。

DG46PR-COMP PASS：controller 与 audited `Pg_prod` 的 max difference 为 left
`8.88e-16`、right `2.11e-15`（spectral `1.18e-15 / 2.27e-15`）；左右 rank 5，symmetry exact，
idempotence `<=1.55e-15`，mutual containment `<=1.55e-15`，missing-direction annihilation
`<=3.94e-16`，point-force reconstruction `<=1.42e-14`。full/reduced operator parity 分别
`1.67e-16 / 4.44e-15`。QP constraint、physical interaction task 与 controller diagnostic/output
都仅使用 physical range；冻结 `1e-6` solver regularization 仍作用于 latent null direction，但从
physical task Hessian 扣除该明确 numerical term 后 residual `<=8.98e-17`，没有改变 solver 或
weight。core `17/17`、adapter `6/6` 与 historical profile tests 全部 PASS。

COMP PASS 后合法进入 DG46PR-EQ。同一 Phase45 compatible-H0/tick0、Model B、state、contact、
friction、solver、gain/weight 与 torque limits 下，actual `ddxi_L/R` 为
`-0.0193390931/-0.0491110277 m/s2`，均满足 `abs(ddxi)<=0.05`。material tangent acceleration
为 `+0.000770289/+0.002162943 m/s2`，bilateral two-point contact 保持，normal loads
`9.92264/9.61238 N`，hard `4.48e-11`、slack `0.00152222`、minimum torque margin
`1.99684 Nm`、whole-dynamics/contact closure `2.13e-14/0`，故 equilibrium PASS。

按 stop rule，本轮没有运行 AUTH、REAL、SHORT、10 s 或 trajectory；`R2` 不授权。下一允许动作
仅为独立 fixed-state authority audit。formal-v1 因把冻结 numerical regularization 误算成 physical
interaction task 而得到 false COMP FAIL，已 rejected；它遵守 stop rule 且未运行 EQ。authoritative
evidence 为 [formal-v2](evidence/automated/corrected-exact-r1-formal-v2/corrected-exact-r1-comp.json)
与 [fresh replay-v1](evidence/automated/corrected-exact-r1-replay-v1/summary.json)，semantic error `0`。
旧 pure-`Ml` 与 previous exact-R1 EQ 结果均为 superseded candidate evidence，不评价本 corrected
candidate。Phase46 保持 `review/REWORK`，不创建 RECORD。

## REWORK — corrected production-reference exact-R1 fixed-state AUTH

结论：`B-HARMFUL-CROSS-REMAINS`，DG46PR-AUTH FAIL。全部 24 个 directional probe 都使用
`(probe_output-baseline_output)/signed_delta`，没有直接以 probe output 除输入。common actual
`slip -> ddxi` 为 `-0.1180399992`：相对 Phase45 `-4.2950931926` 降低 `97.2517%`，通过 90%
reduction gate，但仍超过 absolute `0.1` gate。actual xi self 为 `+0.9873663720`；actual slip
self 为 `-0.1401691458`，反号且未保留正 authority。QP common matrix 近单位阵，故 slip self
还出现 QP/MuJoCo 符号不一致。

differential actual matrix的 xi/slip self 为 `+0.9381805015/-0.0027965976`，slip self 同样反号；
common-slip 输入到 differential 输出的最大 actual contamination 为 `0.1735731065`。这作为并存的
self-authority loss/mode contamination 记录，但因 harmful common cross 本身仍超 mandatory absolute
gate，主分类按冻结优先语义为 `B-HARMFUL-CROSS-REMAINS`。

所有 branch split `<=1.19e-11`，scale convergence `<=2.78e-11`。每个 probe 的 corrected R1
projector/range/point-force/full+reduced operator closure 均 PASS，最大 residual `2.75e-14`；双侧
各两个 3D contacts、normal frame、active constraints 和 solver/contact signature 均稳定，最小
friction margin `15.2007 N`。fresh replay semantic error 为 `0`。

按 stop rule 未进入 REAL、SHORT、10 s、trajectory 或任何 repair/R2。下一允许动作仅为
`post-corrected-R1 authority attribution`。证据见 [AUTH report](CORRECTED_EXACT_R1_AUTH.md)、
[formal-v1](evidence/automated/corrected-exact-r1-auth-formal-v1/corrected-exact-r1-auth.json) 与
[fresh replay-v1](evidence/automated/corrected-exact-r1-auth-replay-v1/summary.json)。Phase46 保持
`review/REWORK`，不创建 RECORD。

## REWORK — post-corrected-R1 fixed-state authority attribution

结论：`E-MULTIPLE-REMAINING-MECHANISMS`。全部 parity、R1、regime 与 causal closure gates PASS，
fresh replay error `0`。slip-common 四维 discrepancy 中 contact gap 的 alignment 为
`0.687409`、residual `0.321378`；other constraint/passive gap alignment 为 `0.312591`。两者均
material 且同向破坏 slip self，所以 contact response 虽是最大贡献者，仍不能按冻结 dominance
规则单独分类为 A。slip-differential 则由 contact gap 支配（alignment `1.005667`、residual
`0.006796`）。

point-force gap 在 production reference 下几乎全是 aggregate-changing，left/right norm
`1.48333/0.687654`；null redistribution 仅约 `6e-14`，wrench closure `<=1.77e-13`。actual
reaction 正常反向抵消 large negative free slip，没有 solver bug evidence。xi-common self 仍健康，
因为 contact/other gaps 主要互相抵消；slip-common 两者则同向累积。详见
[attribution report](POST_CORRECTED_R1_AUTHORITY_ATTRIBUTION.md)、
[formal-v3](evidence/automated/post-corrected-r1-authority-attribution-formal-v3/post-corrected-r1-authority-attribution.json)
与 [fresh replay-v3](evidence/automated/post-corrected-r1-authority-attribution-replay-v3/summary.json)。

本轮未进入 KKT、未修改 controller 或参数、未实施 repair。`R2` 不授权；下一允许动作仅为继续
fixed-state attribution，把 material `other` 拆成 other-constraint 与 passive/applied 后再决定
repair layer。Phase46 保持 `review/REWORK`，不创建 RECORD。

## REWORK — post-corrected-R1 other-gap closure

结论：`D-NONCONTACT-CONSTRAINT-GAP`。slip-common previous other gap 的 slip_c
`-0.371684465` 由 bilateral `left_leg_closure/right_leg_closure` equality-response gap 以 fraction
`1.00000000000046` 重建。QP 已通过 frozen plant-constrained reduction 建模该项；MuJoCo 侧由
equality `efc_J.T@efc_force` row-wise 重建，所以结论是同一物理项的 QP-vs-MJ response gap，
不是把 plant equality contribution误称 omission。

passive、applied、external/body-applied、bias delta、limit、friction-loss 与 other constraint 均为
zero；numerical remainder `<=5.88e-13`。constraint total closure `<=1.42e-14`，contact-row vs
point-contact closure `<=2.84e-12`，other-gap closure `<=7.54e-14`，fresh replay error `0`。
因此 other gap 独立于 contact，且没有 contact bookkeeping overlap。

slip-differential 继续由 contact gap支配；xi-common 中 contact/equality gaps 反向抵消，而
slip-common 中两者同向破坏 authority。上一轮 overall
`E-MULTIPLE-REMAINING-MECHANISMS` 因此继续 authoritative，不标 superseded；contact response
不是 unique first mismatch，R2 不是 re-authorization candidate 且不授权。详见
[other-gap report](POST_CORRECTED_R1_OTHER_GAP_ATTRIBUTION.md)、
[formal-v2](evidence/automated/post-corrected-r1-other-gap-attribution-formal-v2/post-corrected-r1-other-gap-attribution.json)
与 [fresh replay-v2](evidence/automated/post-corrected-r1-other-gap-attribution-replay-v2/summary.json)。
本轮未进入 KKT、未实施 repair；Phase46 保持 `review/REWORK`。

## REWORK — bilateral leg-closure equality-response operator audit

结论：`D-QP-CONSTRAINED-REDUCTION/REACTION-MISMATCH`。QP 与 MuJoCo 使用完全相同的 bilateral
site-pair Jacobian：raw/spectral difference `0`，rank均为 `6`，mutual containment
`1.44e-15`，nullspace projector difference `0`；q/qdot frozen且 `Jdotv=0`。geometry/row-space
不是 first mismatch。

MuJoCo 因约 `1e-4 m` closure position residual产生最高 `efc_aref=0.409092 m/s2`，而 QP rigid
target为 zero closure acceleration；但 full coupled target-gap传播到 slip_c 仅 `-0.00491981`，
占 equality gap `1.32365%`，不足以解释 AUTH failure。继续合法进入 coupled rigid diagnostic后，
QP reconstructed reaction 的 `99.9233%` 不在 `Range(J_eq^T)` 内，QP-vs-rigid relative difference
`0.999967`；MuJoCo-vs-rigid 为 `0.111808`。因此 first material mismatch 位于 QP constrained
reduction/reaction reconstruction，而不是 geometry、target或已证明的 solver bug。

全 probe rigid KKT residual `<=2.54e-14`，branch split `1.80e-10`、scale convergence
`4.16e-10`，fresh replay error `0`。详见
[operator audit](LEG_CLOSURE_EQUALITY_OPERATOR_AUDIT.md)、
[formal-v4](evidence/automated/leg-closure-equality-operator-audit-formal-v4/leg-closure-equality-operator-audit.json)
与 [fresh replay-v4](evidence/automated/leg-closure-equality-operator-audit-replay-v4/summary.json)。
本轮未修改 equality/reduction/solver，未实施 repair；R2 继续不授权，Phase46 保持
`review/REWORK`。

## REWORK — Constraint-consistent reaction implementation audit

结论：`D-REACTION-SEMANTICS-IMPLEMENTATION-FAIL`。

新 profile 的现有 C++ 改动只复用 corrected-R1 QP/profile 与 point-force projection，没有产生、
保存或输出 coupled `[J_contact; J_eq]` KKT 恢复所得的 runtime `lambda_eq`、`Q_eq` 和 contact
companion reaction。原 component runner 则直接把历史 `rigid.equality_generalized_force` 赋给
`new_QP_equality_generalized_force`，随后执行 `relative(qeq, qeq)` 并把 contact parity 固定为
`0.0`；因此 formal-v1 的 COMP-A/B PASS 不能作为实现证据。

runner 已改为 fail-closed：输入没有 `runtime_qp_reaction_probes` 时直接输出 COMP-A FAIL，且
COMP-B/EQ/AUTH 均为 `NOT_RUN`。fresh replay 的 decision JSON 逐值相同，non-finite audit 无命中。
targeted build 通过；`wheel_leg_core` 与 `wheel_leg_mujoco` 共 35 tests 全部通过，但构建/回归通过
不改变 reaction semantics gate 的失败。下一允许动作仅为 implementation fix；R2 仍未授权。

Evidence：
[formal-v2](evidence/automated/constraint-consistent-leg-closure-reaction-formal-v2/constraint-consistent-leg-closure-reaction-repair.json) 与
[fresh replay-v1](evidence/automated/constraint-consistent-leg-closure-reaction-replay-v1/constraint-consistent-leg-closure-reaction-repair.json)。

## REWORK — Runtime implementation-status audit

结论：`C-EXPLICIT-REACTION-NOT-ACTUALLY-IN-QP`。

`R46E-*` runtime command 能选择 `kPhase46ConstraintConsistentLegClosureReaction`，并进入
`WeightedWbcController::step` 的通用 42D solve。candidate-specific profile checks也会执行，但只让
该 profile复用 corrected-R1 的 minimal interaction wrench、point-realizable contact projector 和
rolling task。actual QP construction、decision layout与solution extraction中不存在 `J_eq`、
`lambda_eq`、explicit equality rows、coupled KKT/Schur recovery或 runtime equality-reaction output。

因此 profile reachability 不能证明 frozen reaction formulation 已实现。旧 non-physical equality
reaction也从未作为 term进入 real QP；它只存在于历史 post-hoc diagnostic reconstruction。按本轮
Case B stop rule，IMPLEMENTATION-STATUS 是首个 mandatory FAIL，故没有补 instrumentation、没有
执行 runtime solve，RUNTIME-PROVENANCE 与 COMP-A 均未运行；COMP-B/EQ/AUTH/R2 同样未进入。
formal-v2 的失败仍只表示 runtime evidence contract拒绝 oracle 冒充，并不拒绝 physical repair
hypothesis。下一允许动作仅为 implementation fix。

Evidence：
[formal-v3](evidence/automated/constraint-consistent-leg-closure-reaction-formal-v3/constraint-consistent-leg-closure-reaction-repair.json)。

## REWORK — Reduced-QP/full-constrained-dynamics equivalence

结论：`B-REDUCED-QP-VALID-DIAGNOSTIC-RECONSTRUCTION-INVALID`。

actual `R46E-H0` runtime 以 status 0 解出 unchanged 42D production QP。只读 instrumentation
记录 production 自身的 `N/c_N/J_eq/JdotV` 与 full-tree dynamics；旧 CSV 的 776 个共同字段逐值
不变，semantic max error 为 `0`。production closure operator 的 rank 是 `4`，不是此前用作
plant/equality geometry审计的 MuJoCo 6-row operator；`rank(N)=12=16-rank(J_eq)`，
`||J_eq N||_2=2.47e-13`，affine residual `0`，primal/dual projector difference分别
`1.75e-15/1.74e-15`，故 primal与dual subspace equivalence均 PASS。

runtime lifted full EOM 的 projected residual为 `4.48e-9`。required equality reaction 的
range-orthogonal fraction为 `3.44e-8`，但 absolute residual同为 `4.48e-9`，与 solver-feasibility
误差一致；legal `J_eq^T lambda` recovery max residual `4.48e-9`、virtual work residual
`1.06e-16`，algebraic consistency PASS。exact affine pullback full oracle 保留所有 production
contact/objective/constraint semantics，reduced/full qacc、tau、physical contact、slack、task、active
set与objective difference均为 `0`；latent optimum nonunique but physically equivalent。

因此 historical `0.999233` 只证明旧 post-hoc reaction reconstruction无效，不能证明 production
reduced QP formulation错误。corrected-R1保持 CLOSED，本 audit未造成 regression。explicit-lambda
controller repair与R2均不授权；下一允许动作仅为修复 diagnostic/reaction reporting。

Evidence：
[audit report](REDUCED_QP_FULL_DYNAMICS_EQUIVALENCE_AUDIT.md)、
[formal-v4](evidence/automated/reduced-qp-full-dynamics-equivalence-formal-v4/reduced-qp-full-dynamics-equivalence-audit.json) 与
[fresh replay-v2](evidence/automated/reduced-qp-full-dynamics-equivalence-replay-v2/summary.json)。

## REWORK — Legal equality reaction re-attribution

结论：`E-MIXED-REMAINING-MECHANISMS`。历史 QP equality reaction 与 equality gap均
**SUPERSEDED**。production rank-4合法 recovery在全部 probe通过；MuJoCo raw equality rank为6，
operational common/prod-only/MJ-only dimensions为 `4/0/2`。

slip-common new legal QP/MJ-common equality contributions分别 `-0.067334201/-0.063724030`，
gap仅 `+0.003610170`；旧 `-0.371684465` equality gap约 `99.03%` 被移除。contact gap仍为
material `-0.749895431`，但只占 total discrepancy norm `0.678965`，FREE response仍有独立
material gap，故 contact不是 unique remaining mismatch。

range/reconstruction residual norms `<=6.97e-17/1.08e-16`，source closure `0`，branch/scale
errors `5.83e-11/1.33e-10`，fresh replay `0`。production numerics未改变，corrected-R1保持
CLOSED；R2与explicit-lambda repair均不授权。详见
[report](LEGAL_EQUALITY_REACTION_REATTRIBUTION.md)、
[formal-v4](evidence/automated/legal-equality-reaction-reattribution-formal-v4/legal-equality-reaction-reattribution.json) 与
[fresh replay-v1](evidence/automated/legal-equality-reaction-reattribution-replay-v1/summary.json)。

## REWORK — Smooth/pre-contact first-mismatch attribution

结论：`C1-RAW-MASS-INERTIA-RESPONSE-MISMATCH`。

strict bookkeeping重现 target remainder `-0.388661935`，residual `<=5.56e-13`。torque
application、generalized actuator mapping和other smooth-force gates依次通过（errors
`0/0/1.78e-13`），state provenance通过。first material mismatch首次出现在full-tree mass
operator：matrix max difference `0.497927324`、relative Frobenius `0.096988856`；同一smooth
force的raw qacc gap norm `1.016570084`，slip-common projection `-0.388661935`，解释目标
`99.999983%`。

dominant qacc DOF为base-z translation；RH/LH actuator contributions为
`-0.653811267/+0.292795751`，hip family signed share `92.884%`。slip-differential self gap仅
`-0.000129902`，故common-mode specific。按stop rule未进入closure-conditioned与observable
层。branch/scale errors `2.64e-10/1.23e-9`，fresh replay `0`。contact仍material但不是unique；
legal equality不material；R2不授权。详见
[report](PRECONTACT_FREE_RESPONSE_ATTRIBUTION.md)、
[formal-v3](evidence/automated/precontact-free-response-attribution-formal-v3/precontact-free-response-attribution.json) 与
[replay-v1](evidence/automated/precontact-free-response-attribution-replay-v1/summary.json)。

## REWORK — Closure-conditioned effective-inertia / precontact response attribution

结论：`D-MIXED-EFFECTIVE-INERTIA-AND-CLOSURE`。上一轮 C1 只冻结为 raw-tree first mismatch，
不再提前称 physical inertial-parameter root cause。

common4 没有抽选四条 MuJoCo rows，而是用 verified production rank-4 closure row space在共享
full-tree ordering 中建立唯一正交 operator与 tangent basis。ranks `4/4/6`；production/common
principal angles `<=4.87e-16`、projector difference `0`、mutual containment `6.97e-16`。
production conditioned K与matched reduced response spectral gap `6.36e-11`，全部 closure residual
`<=1.42e-14`，故 operator semantics可信。

同一 smooth force下，raw slip-c gap `-0.3886619350`；common4 gap
`-0.3883828695`、qacc norm `1.012755439`，解释 raw target `99.9281984%`。MuJoCo native6相对
common4的 slip-c contribution为 `-0.04428122835`、qacc norm `3.709800806`，占 raw target
`11.3932506%`。两者均 material，因此不能分类 A 或 C，也不能把 contact写成 unique mismatch。

matched tangent mass relative gap为 `0.0970278065`，representative tangent kinetic-energy parity
FAIL（max relative `0.187622671`）。Delta-K dominant input/output均为 base-z/base-ry/hip combination。
slip-differential与xi-common conditioned self gaps仅 `-0.000132496/-0.000867364`，common-mode
specific。observable map parity PASS。

force provenance `<=1.78e-13`，branch/scale errors `5.52e-11/2.23e-10`，全部 R1/regime PASS，fresh
replay error `0`。本轮没有修改 controller、QP、plant、mass/inertia、closure/contact，也没有实施
body-level counterfactual；source保持 NOT ATTRIBUTED。下一允许动作仅为 additional inertial-source
attribution，closure-model attribution candidate同时保留；inertial modification、R2均不授权。
详见 [audit](CLOSURE_CONDITIONED_EFFECTIVE_INERTIA_AUDIT.md)、
[formal-v2](evidence/automated/closure-conditioned-effective-inertia-formal-v2/closure-conditioned-effective-inertia-audit.json)
与 [fresh replay-v1](evidence/automated/closure-conditioned-effective-inertia-replay-v1/summary.json)。

## REWORK — Common-tangent inertial / kinematic-assembly source attribution

结论：`H-KINEMATIC-INERTIA-ASSEMBLY-MISMATCH`。

11-body mapping、kinematic/inertial provenance均PASS。双方mass与principal inertia逐body一致，
armature全零；centered-wheel COM存在微小source差异。使用各自真实body/COM/angular Jacobian与
normalized inertia独立重建runtime M，production/MJ max errors分别 `<=2.22e-16/1.11e-16`。

fixed-common4 factorial精确重现 target `-0.3883828695`。inertial main effect为
`-0.0086175696`（signed `2.2188%`，nonmaterial），kinematic assembly为
`-0.3797747947`（signed `97.7836%`，material），interaction仅 `+9.49e-6`
（signed `-0.0024%`）。按stop rule未进入inertial group/body attribution。

dominant source定位为production `base_control_frame` point velocity与MuJoCo `base_body` origin
free-joint translational velocity的reference/Jacobian assembly语义。真实source-level Jacobian
transport关闭 `97.7812%` slip-c target，remaining `-0.0086174558`；tangent mass/operator gaps降至
`1.54e-5/5.74e-4`，dominant Delta-K input/output alignment均 `0.999810`，dominant tangent
kinetic-energy gap从 `-0.154433` 降至 `-1.02e-6`。

fresh replay error `0`。本轮不修改任何source/model/controller/contact/closure；允许下一轮定义一个
base generalized-velocity reference semantic parity candidate，但kinematic/inertial modification与R2
仍不授权。详见 [report](COMMON_TANGENT_INERTIAL_KINEMATIC_SOURCE_ATTRIBUTION.md)、
[formal-v3](evidence/automated/common-tangent-source-attribution-formal-v3/common-tangent-inertial-kinematic-source-attribution.json)
与 [fresh replay-v1](evidence/automated/common-tangent-source-attribution-replay-v1/summary.json)。

## REWORK — Base reference semantic canonicalization candidate

结论：`A-EXACT-BASE-REFERENCE-CANONICALIZATION-CANDIDATE`。

真实`base_control_frame` site geometry导出configuration mapping与其微分，不使用拟合offset。
body pose/rotation parity为 `3.93e-17/2.76e-16`，configuration/acceleration FD errors最大
`4.34e-9/4.48e-10`；X inverse closure为zero，twist/virtual-power residuals
`<=2.35e-16/4.44e-16`。H0 `Xdot*nu=0`经实际zero velocity证明。

same-production-model covariance全部machine-scale PASS：mass relative `3.08e-17`，energy
`3.55e-15`，bias/full-EOM `1.78e-15`，Jacobian `2.35e-16`，reduction `2.83e-16`。
因此production control-point dynamics内部自洽；first wrong consumer是Phase46 cross-model audit直接
比较不同base reference coordinates，最小插入点是diagnostic comparison boundary，不是production
controller或`NominalWbcModel`。

candidate移除common4 slip-c `-0.379765414`（signed `97.7812%`），remaining
`-0.008617456`与冻结secondary inertial effect一致；mass/operator/kinetic residual分别
`1.54e-5/5.74e-4/-1.02e-6`。MJ-only closure与contact机制未混入。

fresh replay error `0`。下一轮只授权实现一个diagnostic-boundary base reference canonicalization；
不授权production kinematic/inertial modification或R2。详见
[report](BASE_REFERENCE_SEMANTIC_CANONICALIZATION_CANDIDATE.md)、
[formal-v2](evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json)
与 [fresh replay-v1](evidence/automated/base-reference-candidate-replay-v1/summary.json)。

## REWORK — Diagnostic-boundary base reference canonicalization implementation

结论：`A-DIAGNOSTIC-BASE-REFERENCE-CANONICALIZATION-IMPLEMENTED`。

已授权candidate仅实现在Phase46 cross-model diagnostic comparison boundary，位于
`M/h/Q/J/N/qacc/observable`比较之前。`DG46RC-COMP`全部PASS，最大协变残差`1.78e-15`，observable
invariance PASS。production controller、reduced QP、state semantics、模型参数、contact与equality均未修改；
fresh controller CSV数值差`0`，R1继续closed。

common4 slip-c gap从`-0.388382869511`降至`-0.008598876446`，移除`97.7859795%`。reference
semantic mismatch已关闭，旧precontact physical-mismatch interpretation被取代，secondary inertial
family residual为nonmaterial。physical-channel re-decomposition无double count：legal equality
nonmaterial；contact response `-0.748977633253`与独立MJ-only closure mechanism均material。

因此contact不是unique material remaining mismatch，R2 candidate NO、R2 unauthorized。fresh replay
error `0`；按stop rule下一允许动作仅为closure-model attribution。详见
[report](BASE_REFERENCE_CANONICALIZATION_IMPLEMENTATION.md)、
[formal-v3](evidence/automated/base-reference-canonicalization-implementation-formal-v3/base-reference-canonicalization-implementation.json)
与 [fresh replay-v1](evidence/automated/base-reference-canonicalization-implementation-replay-v1/summary.json)。
## REWORK addendum — MuJoCo-only closure-model attribution

P46-R99～R103 已完成。exact site-pair geometry证明 native6 中两个弱方向分别是左右 connect
的 Cartesian-y row；其 norm 为 `1.83e-4/1.81e-4`，完全由有限 x/z closure residual 对 base
rotation 的叉乘产生，exact manifold 上同时消失并恢复 rank4。旧 `MJ6-common4`
slip-common `-0.044281228354` 的数值复现闭合，但 hard conditioned inverse 会归一化任意非零
row，因而该 material counterfactual 在零 row 极限不连续，不能解释为独立 physical closure。
weak-row `efc_pos/aref` 均为 roundoff，stabilization 也没有提供独立 target。

结论为 `BOOKKEEPING-HARD-RANK-ARTIFACT-NOT-INDEPENDENT`：MJ-only closure从 remaining physical
mechanisms 中移除，contact response成为 unique material remaining mismatch；R2 candidate可在下一轮
重新授权，但本轮 `R2 AUTHORIZED=NO`。formal-v1 PASS，fresh replay-v1 error `0`。Phase46仍为
`REWORK`，不创建 RECORD。详见 [closure-model attribution](CLOSURE_MODEL_ATTRIBUTION.md)。
## REWORK addendum — R2 contact-response re-authorization

P46-R104～R112 已完成。fresh contact slip-common gap为 `-0.753272490427`。current QP 已 hard
couple `nudot/tau/contact wrench`，但缺少将 chosen wrench绑定到 same-tau plant reaction 的
constitutive/complementarity law。native runtime oracle以
`f=D(aref-Jqacc)` 与 coupled dynamics重建 qacc/row force/point force/generalized force/observable，
最大 material reconstruction error `4.34e-14`，Stage S PASS。

coupled A 与 Schur B 物理等价；但 Stage R diagnostic integration在 H0产生 `0.0368512` constraint
violation，H0 equilibrium、branch和scale gates均 FAIL。故不得从 source-oracle PASS 推断 repair
authorization。最终为 `E-R2-SOURCE-CLOSED-BUT-LAW-NOT-TRUSTED`，R2 authorized/implemented均 NO，
production numerics unchanged，严格停止。formal-v1 PASS，fresh replay error `0`。详见
[R2 re-authorization](R2_CONTACT_RESPONSE_REAUTHORIZATION.md)。

## REWORK addendum — R2 reduced-integration first-mismatch attribution

P46-R113～R116 completed without production repair. Historical Stage R reproduced fresh with H0
violation `0.0368511841794`, `ddxi_c/slip_c=-6.86299498911/-0.783202206426`, branch
`3.12034206829`, and scale error `8.29578558891`; the trusted plant oracle remained machine-level closed.

After retaining the frozen nonmaterial legal-equality conditioning, the actual H0 witness closes full and
reduced dynamics at `2.84e-14/2.13e-14`, R1 at `2.73e-14`, and constitutive response at `1.33e-15`.
The first material hard mismatch is the Stage-R mixed-level relation
`Aw_prod W_actual = Qc0_prod + Qct_prod tau_current`, residual `4.83664403838`. Row-force to
point-force generalized-force parity is exact, while point/aggregate mapping has the same residual and a
one-dimensional redistribution nullity per side. Classification is therefore
`B-CONTACT-REACTION-REPRESENTATION-MISMATCH`; closure double-count is rejected and optimization audit is
not entered because the witness is already hard-infeasible. Formal-v2 PASS, fresh replay error `0`, no
nonfinite values, production numerics unchanged. Phase46 remains `REWORK`; R2 candidate, authorization,
implementation authorization, and implementation remain `NO`. See
[reduced-integration attribution](R2_CONTACT_LAW_REDUCED_INTEGRATION_ATTRIBUTION.md).

## REWORK addendum — R2 contact-reaction commuting-diagram attribution

P46-R117～R121 completed without repair. Fresh maps re-establish `Aw*Gp=Jp^T` at
`1.67e-16` full/reduced and virtual work at `5.77e-16`. H0 row→point E1 is `4.88e-15` and
point→aggregate E2 is `8.88e-16`; after putting both sides in production `P` generalized-force
coordinates, aggregate→Stage-R E3 is `1.25e-13`.

The historical `4.83664403838` is reproduced only when `Qagg_M` is compared directly with
`QStageR_P`. It decomposes into `1.52219637586` offset-frame and `3.31444766252` slope-frame
contributions. Therefore the prior `B-CONTACT-REACTION-REPRESENTATION-MISMATCH` remains a valid broad
localization but the stronger aggregate-incompatibility interpretation is superseded. Primary is now
`A-AGGREGATE-DYNAMICS-SUFFICIENT-STAGER-AFFINE-MAP-MISMATCH`: the missing relation is force-dual
base-reference canonicalization at the Stage-R affine-map boundary, not point-force information loss.

Both `Gp` maps have rank/nullity `5/1`, `Jp^T n_p` is numerical, and H0 eta amplitudes are below
`8e-16`; directional slip-common, xi-common, and slip-differential representation replays all close at
machine scale. No representation change is required or proposed. Formal-v2 PASS, fresh replay error `0`,
operator replay error `0`, production numerics unchanged. Phase46 remains `REWORK`; R2 authorization and
implementation remain `NO`. See
[commuting-diagram attribution](R2_CONTACT_REACTION_COMMUTING_DIAGRAM_ATTRIBUTION.md).

## REWORK addendum — Stage-R affine reaction-map provenance/reference attribution

P46-R122～R126 completed without production repair. `Qc0/Qct` are built and consumed only by the
Phase46 diagnostic Stage-R script; CBM/source audit confirms production `WeightedWbcProblem` consumes
independent aggregate wrench through reduced dynamics and tasks, not an equivalent constitutive affine law.

The affine origin is `tau=0`. Producer outputs are `Qc0_M/Qct_M`; the historical producer transformed
them to P, while the extra-equality consumer left `Aw*W` in M. Thus the first wrong edge is exactly
`Aw_M W == Qc0_P + Qct_P tau`. `X_MP^T` covariance passes per actuator column, and the historical
residual decomposes as `1.52219637586 + 3.31444766252 = 4.83664403838` in the dominant component.

Applying `Aw_P=X_MP^T Aw_M` only inside the diagnostic equality closes the map at `1.98e-14`.
Corrected H0 has violation `0`, `ddxi_c=0.000401232234`, `slip_c=0.027082170421`, KKT
`2.90e-14` and R1 `2.67e-14`; branch/scale are `1.16e-10/9.14e-11`, with xi-common and
slip-differential controls PASS. Classification is `A-DIAGNOSTIC-STAGER-REFERENCE-MIX-CLOSED`.
Because production does not consume the affine law, R2 candidate/authorization/implementation remain NO.
Phase46 stays `REWORK`; next is production contact-response integration attribution. See
[Stage-R affine reference attribution](R2_STAGER_AFFINE_REFERENCE_ATTRIBUTION.md).

## REWORK addendum — production contact-response integration attribution

P46-R127～R130 completed without production repair. The 42D production QP already couples
`nudot/tau/aggregate wrench` through 12 hard reduced-dynamics rows, cones and corrected R1, but has no
constitutive/complementarity relation binding the selected wrench to the same-state, same-tau plant
reaction. That is the first missing production relation.

The corrected diagnostic `Qc=Qc0+Qct*tau` is an `R1` closed-response operator obtained by eliminating
`qacc` and constraint reactions from MuJoCo full dynamics plus `f=D(aref-Jqacc)`. Under identical
primitives it is algebraically equivalent to the coupled equations and does not literally count contact
force twice. Its coefficients nevertheless require `efc_J/D/aref`, active solver rows, pyramidal friction
regime and solver/contact parameters. It is therefore `P2`, local to a fixed state/active set, and not a
production-computable `P0/P1` contact law.

The strict P2 stop was applied before rank augmentation, physical-H0 witness, shadow-QP, independent
prediction or contact-response residual claims; all are `NOT ENTERED`. Corrected diagnostic Stage-R
H0/branch/scale/controls remain frozen PASS but do not authorize production insertion. Preferred form is
`I4`; classification is `B-DIAGNOSTIC-LAW-SIMULATOR-SPECIFIC`. Aggregate representation remains
sufficient; R2 candidate/authorization/implementation authorization/implementation remain `NO`.
Next allowed action is only to derive a controller-side physical contact-response model. See
[integration attribution](R2_PRODUCTION_CONTACT_RESPONSE_INTEGRATION_ATTRIBUTION.md).

## REWORK addendum — MuJoCo-dependent simulation-only hard R2

> **WARNING — MUJOCO-DEPENDENT SIMULATION-ONLY R2.** This implementation uses current-state
> MuJoCo internal constraint-response quantities only to close the simulation loop. It must not be deployed
> to the real robot unchanged; replace it with a hardware-valid realization model first.

P46-R131～R136 completed to the first mandatory failure. Same pre-command snapshot provenance passes,
post-response leakage is absent, qacc oracle error is `1.48e-12`, contact/equality partition error is
`2.36e-16`, and M→P virtual-power error is `3.55e-15`. Full 16D insertion is illegal; the legal reduced
form has decision-row rank 7 and condition `121.623`, increases hard rank `12→19`, and has corrected-R1
image residual `3.55e-15`. Pre-solve active-set consistency passes with minimum predicted row force
`1.15928`.

The only new profile, `kPhase46MujocoContactResponse`, rebuilds this payload from every pre-command
MuJoCo snapshot and adds seven independent hard rows. Historical/default profiles leave the extra capacity
unbounded and pass golden/controller regressions. At compatible H0, however, ProxQP returns
`PrimalInfeasible` with primal residual `0.149974`; therefore `COMP=FAIL` and classification is
`H-MUJOCO-R2-HARD-INTEGRATION-OVERCONSTRAINED`. EQ, AUTH, REAL, SHORT and 10 s were not entered.
No soft fallback, tuning or active-set iteration was attempted. Formal and fresh replay agree exactly.
See [simulation R2 report](R2_MUJOCO_DEPENDENT_SIMULATION_REPAIR.md).

## REWORK addendum — primitive contact-law pre-assembly result

P46-R137 remains `REWORK`. The implementation no longer assembles the rejected closed tau-response:
the simulation adapter constructs primitive contact rows from the legal production acceleration lift and
the core receives only compressed `[nudot,W_L,W_R]` rows. Core and adapter builds pass; all 35 selected
tests pass.

The fresh compatible-H0 pre-assembly audit stopped before the 42D witness and COMP. W1 acceleration
lift, W2 primitive row law, W3 row-to-point virtual-work decode, W4 per-wheel rank-5 aggregation and W6
rank independence pass (`row rank=10`, hard-rank increment `=10`). W5 generalized commuting fails with
maximum affine operator residual `7.65679` (`offset residual=7.20092`). The diagnostic projection of raw
MuJoCo `qacc` also predicts minimum row force `-3.73503`; it is diagnostic only and was not used as a
legal witness. Therefore 42D witness, COMP, EQ, AUTH, REAL, SHORT and 10 s are `NOT ENTERED`; no formal
output directory was created. The next allowed action is to resolve the production-reference/operator
commuting mismatch, not to relax the law, add soft fallback, or revive `Qc(tau)`.
