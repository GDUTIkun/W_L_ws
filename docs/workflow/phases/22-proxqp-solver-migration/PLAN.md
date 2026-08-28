# Phase 22: ProxQP solver migration — PLAN

Status: `complete`

## Goal

在不改变 Phase 21 Weighted WBC 数学问题、控制接口或 plant 的前提下，将 production `DenseQpSolver` 的 project-owned Eigen ADMM 后端替换为 ProxSuite ProxQP v0.7.3 dense backend，并以独立 QP corpus、完整 WBC formal、fresh replay 和历史回归证明数值、安全、确定性与 current nominal simulation deadline 仍满足冻结门槛。

## Current State

- 已有：[Phase 21](../21-nominal-weighted-wbc/RECORD.md) 已冻结 current nominal 12-DoF/42-variable Weighted WBC、104 hard rows、`RobotState -> TorqueCommand`、fault latch/reset、2 ms physics、10 ms control 和 5-step ZOH；其最终 authority 为 formal-v1，不得原地覆盖。
- 已有：production `wheel_leg::DenseQpSolver` 对外提供固定 42 变量/128-row capacity、bound-form `lower <= A*x <= upper`、cold/warm `setup/solve/reset`、状态、迭代数、残差和候选解；内部当前是 Eigen-only ADMM，Weighted WBC 直接依赖该接口。
- 已有：Phase 21 的32-case workspace/dynamic corpus、独立 SLSQP/HiGHS oracle、golden problem、1000-run cold/repeated-warm/cycling-warm benchmark、19个10 s normal/perturbation和6个fault case可作为迁移回归输入。旧solver结果只作为对照，不自动授权新solver。
- 依赖事实：`/home/t/opt/proxsuite` 为干净的 ProxSuite `v0.7.3`、commit `b93d7778ffc3299d84b5cb0851022a29bf24a596`；Release、Python OFF、tests OFF、OpenMP OFF、vectorization ON 构建已安装到`/usr/local`。当前主机存在`/usr/local/include/proxsuite`、`/usr/local/lib/cmake/proxsuite/proxsuiteConfig.cmake`、ROS package metadata和`proxsuite::proxsuite` target，installed header与本地源码/build生成头hash一致。标准CMake probe返回`proxsuite found`；但仅source Jazzy后`ros2 pkg prefix proxsuite`仍返回`Package not found`，因此ROS resource-index可见性不能作为当前已满足事实。
- 缺少：仓库尚未声明 ProxSuite 依赖、尚无 bound-form 到 ProxQP equality/inequality contract、状态/残差映射、warm compatibility规则、ProxQP solver profile、迁移后 corpus/formal evidence，也未证明 clean ROS build 能消费当前安装。
- Grounding：CBM project `W_L_ws`、generation `2026-08-28T02:59:23Z`、full/ready；`dense_qp_solver.hpp/.cpp`、Core CMake/package无 recorded coverage issue，`test_dense_qp_solver.cpp:29`为parse partial且已直接读取。`docs/`和`tools/`不在CBM主索引，历史关系由现有Graphify图与真实Phase 21文档复核。

## Scope

- 冻结并记录 ProxSuite v0.7.3 的可消费安装、CMake/ament dependency、source commit、installed config/header hash、编译器、Eigen及实际solver settings；项目CMake不得硬编码`/home/t/opt/proxsuite`或`/usr/local`。
- 仅以`proxsuite::proxqp::dense::QP<double>`替换`DenseQpSolver`内部算法，保留42变量、128-row输入capacity和现有bound-form公共入口；按冻结规则将行拆分为ProxQP equality `A*x=b`与inequality `l<=C*x<=u`。
- 冻结 ProxQP status、cold/warm/reset、输入拒绝、候选清零、残差和异常边界；任何未求解、infeasible、dual infeasible、iteration limit、non-finite或adapter exception都不得输出非零candidate。
- 在production替换前，用Phase 21相同32-case corpus和失败corpus完成ProxQP component profile、backend/settings、oracle parity、cold/warm determinism、setup+solve timing及allocation行为审计。
- 更新`wheel_leg_core`构建依赖、solver实现和必要测试/benchmark；只允许对Weighted WBC caller做solver-setting和diagnostic适配，不改变WBC model、problem、task、weight、scale、reference、torque extraction或hard acceptance。
- 建立独立Phase 22 validation method、versioned solver/formal config、manifest和新evidence目录；复用Phase 21的case/fault矩阵与plant gates，执行fresh replay、non-overwrite及Phase 14/15/18/20/21兼容性回归。
- 将nominal NMPC顺延到独立Phase 23；只有Phase 22 REVIEW=`PASS`后，NMPC才可把ProxQP-backed Weighted WBC当作已验证下游层。

## Out of Scope

- NMPC/OCP、reference producer替换、trajectory planning、terrain、单轮支撑、控制权重重调或任何高于Phase 21的控制复杂度。
- 改变42D decision order、104-row hard problem、objective、constraint、scale、contact/wrench/slack语义、12-DoF reduced model或Phase 21冻结profile。
- 修改canonical FLU、joint order/sign/unit、`RobotState/TorqueCommand`、ROS message schema、Adapter、MuJoCo model/contact参数、控制周期或fault/reset策略。
- 真机、STM32/树莓派、Hardware Adapter、identified/new CAD profile及目标硬件实时性结论。
- vendoring ProxSuite、`FetchContent`联网拉取、复制其headers到仓库、链接`proxsuite::proxsuite-vectorized`、修改`/home/t/opt/proxsuite`或`/usr/local`安装内容。
- 删除旧Phase 21证据、改写其manifest/config/result，或仅凭“ProxQP返回SOLVED”、编译成功、轨迹可运行推断Phase PASS。

## Frozen Decisions

- **Phase ordering：** Phase 22只做solver migration；nominal NMPC改为Phase 23。solver替换是已冻结WBC层的基础设施变更，不与新控制层合并验证。
- **Dependency identity：** production要求`find_package(proxsuite 0.7.3 EXACT CONFIG REQUIRED)`并链接非vectorized `proxsuite::proxsuite` target；`package.xml`声明对应dependency。实际prefix只可由标准CMake package search或明确的构建环境解析，源码目录仅用于provenance，不进入编译路径；ROS CLI能否枚举该非运行时CMake包不替代clean configure/build gate。
- **Backend family：** 只使用ProxQP dense backend，初始candidate固定`DenseBackend::PrimalDualLDLT`；不得在同一production profile中自动切换dense backend、sparse backend或旧ADMM fallback。若exact corpus不能关闭DG22-02，必须REWORK PLAN后再改变candidate。
- **Public QP contract：** 保持`0.5*x'H*x + g'x`和`lower <= A*x <= upper`、42 variables、最多128 rows、输入finite/symmetric/PSD/bounds validation、`setup/solve/reset`及成功时返回scaled-domain `x`。不把ProxQP类型暴露到canonical robot I/O。
- **Row split：** 逐行仅当`lower[i] == upper[i]`精确成立时分类为equality，`b=lower[i]`；其余为inequality，保持各分类内原始行序。禁止用epsilon把窄区间静默改成等式。通用solver仍接受0～128行的合法组合；Phase 21 authoritative WBC路径必须得到12 equality和92 inequality，否则由WBC integration gate拒绝。
- **Warm compatibility：** cold setup/solve使用`NO_INITIAL_GUESS`并清除旧candidate；warm只在前一次成功且equality mask、`n_eq/n_in`和row order兼容时通过`update`与`WARM_START_WITH_PREVIOUS_RESULT`复用，否则自动重建并cold。`reset()`使下一次求解必为cold；warm只能改变数值路径，不能改变接受解、reset determinism或安全语义。
- **Settings boundary：** 删除`rho/sigma/relaxation`等旧ADMM-only production配置，不做伪映射。初始ProxQP candidate使用`eps_abs=1e-8`、`eps_rel=1e-8`、`max_iter=10000`、closest-primal-feasible disabled、verbose OFF；其余v0.7.3参数、preconditioner/update策略和最终值必须在DG22-02基于exact corpus冻结并完整写入versioned config/manifest，不能依赖未记录默认值。
- **Status mapping：** `PROXQP_SOLVED -> kConverged`，maximum iteration保持`kMaximumIterations`；primal infeasible与`SOLVED_CLOSEST_PRIMAL_FEASIBLE`映射为additive `kPrimalInfeasible`并拒绝candidate，dual infeasible映射为additive `kDualInfeasible`，`NOT_RUN`/内部异常映射为现有`kFactorizationFailure`。现有enum值不重排；所有失败Result的`x`严格为零。
- **Residual authority：** hard violation继续由caller从原始`A/l/u/x`独立重算；stationarity由adapter以ProxQP返回的equality/inequality dual重算`||Hx+g+Aeq^T y+C^T z||`。`primal_residual/dual_residual`记录ProxQP reported KKT residual，并与独立primal/stationarity audit交叉检查；不得把旧ADMM的algorithmic dual-step residual冒充同一物理量。Phase 22 manifest必须标注residual definition/version。
- **Acceptance safety：** solver status只是候选条件；finite、hard/equality/stationarity、objective/physical-torque equivalence、deadline和Core torque validation仍由项目侧独立执行。ProxQP infeasibility recovery或closest feasible模式不进入production。
- **Evidence non-overwrite：** 新config、方法、运行结果和hash进入Phase 22 namespace；Phase 21 formal-v1/replay、solver audit和历史config保持字节不变。对照以旧manifest/hash与新run并列进行。
- **Claim boundary：** PASS最多证明current nominal simulation host上，冻结Phase 21 WBC在ProxQP v0.7.3下等价且满足既有门槛；不推断NMPC、真机、identified profile、target CPU determinism或硬实时性。

## Open Questions / Decision Gates

- **DG22-01 / CLOSED / ENV+BUILD — dependency readiness：** clean Release `--packages-up-to wheel_leg_mujoco`通过exact 0.7.3 CMake config与`proxsuite::proxsuite`；隔离缺依赖configure按REQUIRED语义exit 1，无silent fallback。
- **DG22-02 / CLOSED / CODEX+EVIDENCE — ProxQP profile：** 32-case exact corpus已冻结backend/settings、preconditioner update、cold/warm、status、residual及versioned profile。
- **DG22-03 / CLOSED / CODE+EVIDENCE — component parity：** component/failure tests与三种1000-run production benchmark满足数学、physical torque/objective、determinism及10 ms reference-host gate。
- **DG22-04 / CLOSED / EVIDENCE — integrated WBC：** authoritative formal-v2与fresh replay-v2均19/19 normal、6/6 fault PASS；non-overwrite、solver/task/plant、hash及Phase14/15/18/20回归全部PASS。
- **DG22-05 / CLOSED / REVIEW — claims：** REVIEW blocking findings为零；Phase 21数学/public I/O与旧证据未变，结论保持current nominal simulation-only。

## Interfaces and Compatibility

- 输入：Phase 21 scaled 42D Hessian/gradient、最多128行bound-form constraints、cold/warm意图；ProxSuite exact-version system package；versioned Phase 22 solver/formal config。
- 内部：`DenseQpSolver`适配层完成validation、row split、ProxQP init/update/solve、status/dual/residual映射和candidate清零；`WeightedWbcController`仍只看到project-owned solver contract。
- 输出：现有`DenseQpSolver::Result`与additive infeasibility statuses、现有WBC/Core diagnostics、canonical六路`TorqueCommand`；新benchmark/summary/manifest/evidence。
- 必须保持：42D/104-row authoritative WBC、decision/sign/order/scale、task/weight/reference、hard gate、fault latch/reset、old controller modes、2/10 ms timing、5-step ZOH、Adapter/plant、public message和Phase 21 evidence。
- 允许改变：`dense_qp_solver.hpp/.cpp`内部与algorithm-specific settings/status；`weighted_wbc_controller.cpp`中的solver profile；`wheel_leg_core` CMake/package dependency；相关component/controller tests、benchmark/export/evaluator、Phase 22 config和文档。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P22-T01 | 固化grounding与dependency probe | Phase 21 RECORD/solver audit、CBM/Graphify、`/home/t/opt/proxsuite`与`/usr/local` | dependency/provenance记录、exact CMake/ament probe、影响面与禁止修改清单 | DG22-01；version/commit/config/target/header hash一致，缺失依赖在configure阶段明确失败 | done |
| P22-T02 | 冻结adapter数学与ProxQP profile | 42D/104-row contract、ProxQP v0.7.3 API、32-case和failure corpus | row split、status、residual、warm/reset、settings/profile spec及versioned config | DG22-02；12/92 split、cold/warm和独立KKT审计通过；任何profile变更先更新PLAN | done |
| P22-T03 | 实现build dependency与solver adapter | P22-T01/T02冻结spec、现有`DenseQpSolver` | CMake/package依赖、ProxQP-backed solver、caller setting适配；移除production ADMM实现 | clean `colcon build`；无绝对依赖路径、无旧ADMM fallback、warnings-as-errors通过 | done |
| P22-T04 | 扩充component与failure tests | P22-T03、现有solver/controller tests | unconstrained/SPD/equality/bounds/mixed、42nd dimension、invalid/nonfinite/nonconvex/infeasible/dual-infeasible/limit、mask-change、cold/warm/reset tests | DG22-03 component部分；失败candidate严格zero，cold/reset deterministic，warm等价 | done |
| P22-T05 | 执行oracle parity与1000-run benchmark | Phase 21 exact 32-case corpus、独立SLSQP/HiGHS、P22-T03 | 新Phase 22 corpus/manifest、cold/repeated-warm/cycling-warm结果、allocation/timing对照 | hard/equality/stationarity≤`2e-7`、physical torque差≤`5e-4 N·m`、objective gap≤`2e-6`、cold/dynamic setup+solve≤`10 ms` | done |
| P22-T06 | 建立集成方法、config与evaluator | P22-T05、Phase 21 formal method/config/runner | `docs/experiments/` Phase 22方法、独立formal config、solver identity/residual schema、必要日志/evaluator更新 | formal前freeze；非空输出拒绝；solver version/settings/source/config/output hash完整 | done |
| P22-T07 | 执行full formal、fresh replay与历史回归 | P22-T06冻结输入、current nominal plant | 新`evidence/automated/<run-id>/`、summary/manifest/replay/regression audit | DG22-04；19/19 normal、6/6 fault、solver/task/plant、replay、non-overwrite及Phase14/15/18/20/21兼容性全PASS | done |
| P22-T08 | REVIEW | 全部任务、源码和真实evidence | `REVIEW.md`；仅PASS后创建`RECORD.md` | DG22-05关闭、blocking findings=0后才更新ROADMAP complete并放行Phase 23 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Dependency and Build

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
cmake --find-package -DNAME=proxsuite -DCOMPILER_ID=GNU \
  -DLANGUAGE=CXX -DMODE=EXIST
colcon build --symlink-install --packages-up-to wheel_leg_mujoco \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

- configure必须解析exact ProxSuite 0.7.3和`proxsuite::proxsuite`，不得从源码/build目录偶然取include；同时记录`ros2 pkg prefix proxsuite`在实际环境中的结果，但不以ROS CLI枚举代替CMake/colcon消费证据。
- Core/ROS/MuJoCo现有tests与新增solver/controller tests全部0 failures；旧controller modes结果不变。

### Solver Component and Oracle

- 使用Phase 21相同32个workspace/dynamic QP和独立oracle，另加unconstrained、mixed equality/inequality、classification-mask变化、primal/dual infeasible、maximum-iteration、non-finite和exception边界。
- 对每个accepted candidate独立重算objective、hard/bound/equality/stationarity和physical torque；ProxQP vendor residual只作交叉检查，不单独作为PASS依据。
- cold、同题warm、cycling dynamic warm各1000次；记录setup、solve、total、iterations、status、allocation、compiler、host和完整solver settings。必须满足既有`10 ms`component gate，并与Phase 21 frozen benchmark并列报告；不从reference-host结果推断目标硬件。
- 重复cold与reset后的第一解在冻结数值容差内一致；warm与cold在objective、physical torque和hard gates上等价。

### Formal MuJoCo

在创建稳定输出目录前，使用仓库冻结解释器完成依赖探针与语法检查：

```bash
cd /home/t/W_L_ws
./.venv/bin/python -c "import mujoco, numpy, scipy; print(mujoco.__version__, numpy.__version__, scipy.__version__)"
./.venv/bin/python -m py_compile <phase22-wrapper-and-evaluator-files>
./.venv/bin/python <phase22-formal-wrapper> \
  --output-dir docs/workflow/phases/22-proxqp-solver-migration/evidence/automated/<new-run-id>
```

- 输出目录必须不存在或为空；失败后新建run并记录`supersedes`，不得覆盖。
- 复用Phase 21的19个normal/perturbation与6个fault case、state/contact/slip/closure/task/slack/hard/deadline gates、双episode reset、fresh replay和manifest/hash语义。
- primary/replay除允许的墙钟字段外确定性一致；fault从注入tick起六路zero并锁存，reset恢复cold且exact replay。
- Phase 14/15/18/20正式回归重新执行；Phase 21兼容性指同一WBC数学/case矩阵在新solver下PASS以及旧evidence/hash保持不变，不要求在production保留可切换旧ADMM。

## Acceptance Criteria

- [x] DG22-01关闭：clean ROS configure/build只通过标准package discovery解析exact ProxSuite v0.7.3，provenance和依赖失败语义可复现。
- [x] DG22-02关闭：row split、backend/settings、status、residual、warm/reset和allocation contract已冻结并写入versioned config/manifest。
- [x] production `DenseQpSolver`已使用ProxQP dense backend，旧ADMM迭代实现和algorithm-specific production settings已移除，且没有silent fallback。
- [x] QP problem math、42D order/scale、104 hard rows、WBC task/weight/reference、canonical I/O和plant/timing保持Phase 21冻结值。
- [x] component/failure tests全PASS；所有非成功状态返回严格zero candidate并保持Core fail-zero/latch/reset语义。
- [x] 32-case oracle parity和1000-run cold/warm/dynamic benchmark满足`2e-7` residual、`5e-4 N·m`torque、`2e-6`objective及`10 ms`component门槛，实际对照结果已记录。
- [x] 19/19 normal/perturbation、6/6 fault、fresh replay、non-overwrite、solver/task/plant gates和Phase14/15/18/20/21兼容性回归全部PASS。
- [x] Phase 21正式config、manifest和evidence未被修改；Phase 22所有结论引用新namespace和真实hash。
- [x] REVIEW blocking findings为零且Verdict=`PASS`后才创建RECORD、把ROADMAP标记complete并开始Phase 23 NMPC。

## Execution Notes

按任务 ID 在本文件记录实际命令、结果、偏差和证据链接；不要建立第二份任务状态表。P22-T02完成前不得修改production solver；formal失败不得通过放宽Phase 21 gate、改变WBC数学或调权来修复。任何必须改变backend family、依赖版本、QP contract、public I/O或plant的发现都先将Phase置为REWORK/blocked并修订PLAN。

- 2026-08-28：P22-T01完成source/install/CMake identity与Core clean configure/build正向探针；完整packages-up-to build和缺依赖失败探针留在adapter集成后关闭DG22-01，证据见[evidence/dependency_and_solver_profile.md](evidence/dependency_and_solver_profile.md)。
- 2026-08-28：P22-T02在未调用production solver的profile工具上执行Phase 21原始32-case corpus。Release下1000次cold最大0.832765 ms、cycling dynamic warm最大0.256366 ms；独立stationarity最大2.9134814e-9、bound/equality最大9.7655862e-9、physical torque oracle差2.8314025e-5 N m、objective gap2.4415758e-9，全部PASS。冻结`phase22_proxqp_solver_v1.json`；DG22-02关闭，允许开始P22-T03/T04。
- 2026-08-28：P22-T03/T04完成production ProxQP adapter、exact dependency、12/92 WBC守卫与failure/warm/reset tests。clean Release packages-up-to build通过；缺依赖隔离configure exit 1；ROS汇总`24 tests, 0 errors, 0 failures`。CBM刷新至generation `2026-08-28T07:49:08Z`，live `setup/solve -> WeightedWbcController::step -> weighted_wbc_loop`调用链与源码一致；test第29行parse partial和按策略排除的`tools/`已直接读取。
- 2026-08-28：P22-T05 production adapter在原32-case corpus上cold/repeated-warm/cycling-warm各1000次PASS。cold/dynamic最大`0.929801/0.689053 ms`，独立stationarity`2.9134814e-9`，bound/equality`9.7655862e-9`，物理力矩差`2.8314025e-5 N m`，objective gap`2.4415758e-9`；见[evidence/automated/2026-08-28-solver-benchmark-v1](evidence/automated/2026-08-28-solver-benchmark-v1/README.md)。
- 2026-08-28：P22-T06/T07以MuJoCo 3.7.0、NumPy 2.2.6、SciPy 1.15.3和`py_compile`通过后执行formal。v1因overlay深合并继承Phase 21的`rho/over_relaxation_alpha` manifest元数据而保留并supersede；修正为solver整块替换后，authoritative [formal-v2](evidence/automated/2026-08-28-formal-v2/README.md)与[fresh replay-v2](evidence/automated/2026-08-28-formal-v2-replay/README.md)均19/19 normal、6/6 fault PASS。25个plant CSV字节一致，control仅22,379个`core_step_ns`单元不同，去墙钟summary相等；两套各71项hash无漂移；non-overwrite exit 2且最终53文件不变；Phase14/15/18/20 fresh regressions与coordinate contract PASS。
- 2026-08-28：P22-T08 REVIEW=`PASS`，blocking findings=0；DG22-01至DG22-05关闭，创建RECORD并将Phase置为complete。结论仅限current nominal simulation host，不外推NMPC、identified profile、真机或目标硬件实时性。

## Blockers

None.
