# Phase 23: nominal acados NMPC — PLAN

Status: `complete`

## Goal

在不连接真机、不改变 Phase 21/22 Weighted WBC 数学与 canonical robot I/O 的前提下，为 current nominal full-3D MuJoCo profile 建立基于 acados 的 nominal NMPC：从 canonical `RobotState` 和内部确定性 reference 生成左右 12D interaction-wrench command，经冻结的 ProxQP-backed WBC 输出六路力矩，并以可复现 code generation、独立模型/OCP/KKT 审计、组合时延、完整 MuJoCo formal、fault/reset、fresh replay 和非覆盖证据证明该新增上层在声明范围内可用。

## Current State

- 已有：[Phase 21](../21-nominal-weighted-wbc/RECORD.md) 已冻结 current nominal 12-DoF、42-variable/104-row Weighted WBC、内部 `WbcReference.interaction_wrench_flu` 12D 接口、10 ms Core 周期、fail-zero/latch/reset 及 19 normal/perturbation + 6 fault formal；[Phase 22](../22-proxqp-solver-migration/RECORD.md) 已在不改变上述数学的前提下把 **WBC** QP backend 迁移为 ProxQP v0.7.3，并完成 component/oracle/deadline/formal-v2/fresh replay。
- 已完成：live `ControllerCore` 提供 opt-in `kNominalNmpcWbc`，只替换 12D wrench producer，保留其他 WBC reference、solver、torque extraction 和安全边界；2:1 update/ZOH、age、fault、reset 和 diagnostics 已由 component/formal 验证。
- 已有：P23-T02/T03 与 DG23-01 已冻结并验证 12D locked-composite base state、12D external base-FLU wrench、continuous/RK4 model、analytic sensitivity、equilibrium、canonical mapping 和有效域；这些 PASS 不证明 OCP、acados solver、Core 或 formal。
- 已完成：`NominalNmpcModel`、append-only v2 generated solver、private C++ wrapper、explicit acados loader contract 和 Core integration 已进入 `wheel_leg_core`，并通过 golden/component/integrated formal。
- acados 安装事实：`/home/t/opt/acados` 是 clean git tree，HEAD `21376cb1af6b7dd45f675367272d3ba8100b26c0`（`v0.6.0-2-g21376cb1a`），Release/shared build，包含 `libacados.so`、`libhpipm.so`、`libblasfeo.so`、C headers 和 CMake package；构建配置为 HPIPM/BLASFEO、OpenMP OFF，未启用 QPOASES 等候选 backend。
- 环境已关闭：仓库 `.venv` 可导入 CasADi 3.7.2/acados_template，固定 renderer SHA 已记录；最终 binary 通过 RPATH 从 `/home/t/opt/acados/lib` 解析 acados/HPIPM/BLASFEO。普通构建不运行 generator。
- Grounding：最终复核使用 CBM project `W_L_ws`、generation `2026-08-28T13:13:14Z`（full/ready）；changed metadata 和按策略排除的 `tools/docs` 均直接读取，coverage 仅作 best-effort 信号。

## Scope

- 保持 DG23-01 已批准的 12D physical state、12D contact-wrench input、reference frame/order/sign/unit、20 ms phase、状态有效域与 stale/reset 语义；16D 历史候选的 `xi/dxi` 仅保留为 workspace 诊断，不进入 physical model。
- 用 project-owned generator 从冻结的 current nominal 12D RK4 model 和 OCP config 生成 acados solver；首个待证候选为 `Ts=0.020 s`、`N=20`、0.40 s、multiple shooting、single SQP-RTI、Gauss-Newton、HPIPM QP。具体 condensing、regularization、termination、warm-start、cost/constraint transcription 和任何 optimizer-only augmented state 必须由 DG23-02/03 显式冻结。
- 把 generated solver source/config/ABI/provenance 作为受控、可哈希、不可手工编辑的 project-owned artifact；普通 clean build 使用已冻结 artifact，不在 CMake/colcon 中隐式运行 Python code generation。
- 生成期使用仓库 `./.venv`、`ACADOS_SOURCE_DIR=/home/t/opt/acados`、固定 CasADi/acados_template/acados commit；运行期只允许 generated solver、acados/HPIPM/BLASFEO 和既有 C++ Core 依赖，不加载 Python、CasADi、MuJoCo、MATLAB 或 Simulink。
- 建立 versioned project-owned `NmpcProblem/NmpcResult` 与 acados wrapper；acados C ABI、capsule、memory/workspace、set/get、status 和 reset 不泄漏到 Controller public interface。
- 只让 accepted NMPC first control 写入 `WbcReference.interaction_wrench_flu`；base/height/orientation/leg acceleration reference、42D/104-row WBC、dense ProxQP adapter、hard acceptance、torque extraction 和 canonical `RobotState -> TorqueCommand` 保持冻结。
- 使用内部 versioned deterministic reference profile验证 equilibrium hold、正/负小幅直线 longitudinal reference、回零和扰动恢复；不新增公共 ROS command/schema，不把 trajectory planning、转弯或大姿态纳入本 Phase。
- 增加 opt-in NMPC+WBC Core mode、additive diagnostics、component/oracle/benchmark、独立 MuJoCo loop 日志和 fault injection；复用 Phase 22 runner/formal/evaluator/manifest 结构。
- 在正式输入冻结后执行 Phase 22 的 19+6 最低回归矩阵，并增加 4 个 NMPC-specific normal/reference case 与 4 个 NMPC-specific solver/late/stale/non-finite fault case；执行 fresh replay、non-overwrite、hash 审计和 Phase 14/15/18/20/21/22 compatibility regression。

## Out of Scope

- 真机、STM32/树莓派、Hardware Adapter、正式通信协议、传感器/执行器/contact 辨识、目标硬件时延和任何 real/identified-profile 结论。
- trajectory/foothold planner、terrain、斜坡/台阶、单轮支撑、跳跃、跌倒恢复、roll/yaw recovery、yaw-rate/turning、continuous turning 或 large-yaw；这些保留给后续 Phase。
- 改变 Phase 21 的 12-DoF model、42D decision order、104 hard rows、task/weight/scale、wrench/slack sign、WBC 10 ms timing、dense ProxQP adapter、torque limit 或 fail-zero/latch/reset。
- 修改 canonical FLU、quaternion/world twist、joint order/sign/unit、公共 `RobotState/TorqueCommand`、ROS messages、Adapter、MuJoCo plant/contact/timestep 或 Phase 20 equilibrium。
- 把 Simulink 16D/Euler OCP、历史 acados S-function/generated code、历史 weights/bounds、wheel-to-body internal wrench 或 last-valid-hold 直接复制为 current production authority。
- 在 runtime 或普通 colcon build 中依赖 Python、CasADi、acados_template、MATLAB、Simulink 或 MuJoCo；在 CMake configure/build 时联网、自动下载 renderer/dependency、修改 `/home/t/opt/acados` 或静默使用另一个 acados 安装。
- 把 NMPC 的 acados/HPIPM 与 WBC 的 ProxQP 混称或互作 fallback；无旧 NMPC、ProxQP NMPC、dense/sparse、nominal-wrench、standing-mode 或 last-valid 自动 fallback。
- 在线辨识、自动调权、observer、积分器、gain scheduling、异步线程/队列或新的 supervisor 架构；若同步 candidate 无法关闭 deadline gate，先 REWORK 本 PLAN。
- 覆盖 Phase 21/22 config、manifest、formal 或 evidence；仅凭 code generation、编译、vendor status、短时轨迹或 WBC slack 中任一单项宣称 NMPC PASS。

## Frozen Decisions

- **Phase/claim authority：** Phase 23 是 Phase 22 之后唯一新增的 nominal upper layer；结论只限 current nominal full-3D simulation reference host和 manifest 指定的 acados build/profile。Phase 21/22 的 WBC 数学和 solver evidence 是冻结下游基线，不自动证明 NMPC。
- **Solver ownership：** NMPC 使用 acados generated OCP solver；首个 candidate 固定为 single `SQP_RTI` + `PARTIAL_CONDENSING_HPIPM` + Gauss-Newton。ProxQP 只继续服务冻结的下游 WBC，不再作为 NMPC solver candidate。若 DG23-02 失败，必须保留证据并修订 PLAN，禁止 silent solver/fallback 切换。
- **Generation/runtime boundary：** OCP/model/config 与 generator script由本项目拥有；generated source、solver JSON/ABI manifest 和 hashes 进入版本控制且不得手工编辑。code generation是显式维护步骤，普通 CMake/colcon build只编译冻结 artifact。runtime 明确依赖 acados/HPIPM/BLASFEO shared libraries，但不依赖 CasADi/acados_template Python。
- **Dependency identity：** 当前 candidate 只认 `/home/t/opt/acados`、commit `21376cb1af6b7dd45f675367272d3ba8100b26c0`、Release/shared、HPIPM/BLASFEO profile。构建必须显式传入并验证 acados prefix/commit/library hashes；loader机制由 DG23-02 以 clean build/run证据冻结，不依赖未记录的个人 shell 状态。
- **Physical state contract：** physical state 固定为 12D `x=[p_B^N(3), r_B^N(3), v_B^N(3), omega_B^N(3)]`。`r_B^N` 是 current reset/reference chart 内、与 Phase 21 shortest-arc orientation error 同号同轴的 world-axis rotation vector；姿态传播用 quaternion/Exp-Log 后投回冻结的小姿态 chart。任何为表达 delta-wrench 等引入的 optimizer-only memory/augmented state不得改变 canonical mapping、physical model或公共接口，必须在 DG23-02 明列 order、initialization和reset。
- **Input contract：** `u=[W_left_FLU(6), W_right_FLU(6)]`，每侧顺序 `[Fx,Fy,Fz,Tx,Ty,Tz]`、单位 N/N·m，与 `WbcReference.interaction_wrench_flu` 完全一致。NMPC output只进入该字段；不得通过 ROS、MuJoCo truth 或旁路直接输出 torque。
- **Model authority：** acados symbolic/discrete model必须与已通过 DG23-01 的 current nominal locked-composite continuous/RK4 model逐项一致；优先以 acados `DISCRETE` dynamics承载冻结的20 ms RK4 map，禁止让未记录的内部 integrator设置改变已批准离散模型。历史 `full_base_body_dynamics` 只作拒绝项对照。
- **OCP candidate：** `Ts=0.020 s`、`N=20`、horizon `0.40 s`、multiple shooting、single SQP-RTI、Gauss-Newton；reference/cost/input-state bounds、delta-wrench transcription、condensing horizon、HPIPM/regularization/tolerance/warm-start与 RTI preparation/feedback顺序由 DG23-02/03 corpus一次性冻结。production不得运行时切换 profile。
- **Constraint ownership：** NMPC input/contact-wrench constraints从 Phase 21 validated wrench/contact-frame contract映射，state/workspace bounds不得弱于 Phase 15/21 safety envelope；WBC dynamics、torque、H-cone、acceleration和candidate audit仍是最终 actuator-level authority。NMPC约束不可替代 WBC hard gate。
- **Reference boundary：** Phase 23只使用内部、versioned、确定性的 equilibrium与小幅直线 longitudinal reference；Y、roll/pitch/yaw nominal reference保持 reset equilibrium，禁止 turning/large-yaw。reference amplitude、rate、tuning/holdout split和tracking gate必须在 DG23-03 holdout前一次性冻结。
- **Schedule：** physics `0.002 s`、WBC `0.010 s` 保持不变；NMPC每两个WBC tick同步更新一次并在中间tick做两拍ZOH。更新tick的 `acados solve + wrapper audit + WBC step` 在reference host上必须 `<10 ms`。同步solve若超时只能在返回后判定，该run因此是deadline FAIL并zero/latch；不得用20 ms supervisor period或fault injection掩盖真实阻塞。
- **Safety/staleness：** 只有本拍 acados solve成功、finite、通过project-owned OCP/constraint/KKT audit且age在两拍ZOH合同内的wrench可交给WBC。nonzero/unknown solver status、late、non-finite、KKT/hard gate失败、sequence/timestamp错误或stale均使本拍六路torque严格为零并锁存到reset；不无限保持last-valid。
- **Lifecycle/reset：** wrapper独占一个预分配 solver capsule/workspace；control tick禁止 allocation、code generation、文件I/O或动态装载。reset清除accepted wrench、schedule、diagnostics与warm state，并按冻结的 previous-applied input语义执行首个cold solve；不得复用故障前 iterate。
- **Compatibility：** `kZero`、Phase17/19/20 modes和`kWeightedWbc`行为保持不变；NMPC以新opt-in mode/additive diagnostics进入。public Adapter/watchdog/ROS conversions不变，WBC non-NMPC component corpus逐项回归。
- **Evidence authority：** Phase23 config/method/result使用新namespace、source/generator/generated/library/config/output hash与不存在或空的run目录。primary/replay除明确允许的wall-clock字段外确定性一致；失败后新建run并记录`supersedes`，不得覆盖旧evidence。

## Open Questions / Decision Gates

- **DG23-00 / CLOSED / CODEX — route revision：** 按用户指令解除冻结，保留12-state locked-composite/12-wrench/WBC接缝，NMPC solver路线从project-owned sparse ProxQP改为acados generated SQP-RTI + HPIPM；不合并trajectory/turning/terrain/real work。
- **DG23-01 / CLOSED / CODEX+EVIDENCE — state/model：** `state-oracle-v2`与`model-oracle-v5`已关闭canonical mapping、12D model、20 ms RK4、analytic sensitivity、equilibrium和有效域；solver路线变化不重开这些数学结论，但generated model必须重新做parity。
- **DG23-02 / CLOSED / CODEX+EVIDENCE — acados toolchain/OCP/solver/timing：** `phase23-acados-t04-v1`已冻结`.venv` generation依赖、acados prefix/commit/library hashes、固定renderer、generated ABI/artifact layout、RPATH loader、SQP-RTI/HPIPM profile、warm/reset/status schema、normalized clean regeneration、generated-model parity、project-owned full-horizon objective/dynamics/bound/projected-KKT audit、3×1000-run determinism和更新tick组合`<10 ms`；独立projected-stationarity门槛为`0.05`，corpus最大`0.0428125`。该关闭不替代DG23-03的cost/reference/constraint与holdout批准，也不授权现有formal为production authority。
- **DG23-03 / CLOSED / CODEX+EVIDENCE — reference/cost/constraints：** `phase23_acados_t05_profile_v1`在holdout前冻结现有normalized Q/R、terminal multiplier `10`、per-side input bounds、相对state envelope、5 mm/0.5 s/1.5 s longitudinal reference与tracking/recovery gates。四个隔离generated solver的真实重求解证明longitudinal/terminal/selective-wrench cost归因；baseline normalized delta-wrench最大`0.0238423 < 0.05`，因此冻结为无optimizer-only augmentation。4个10 s tuning与6个预声明未见nonlinear组合holdout均PASS，holdout最大组合耗时/独立defect/projected-KKT为`3.558 ms / 2.80e-6 / 0.03215`。T06随后已把冻结state envelope转写到stages 1..N并完成generated/C++ parity；pre-freeze formal仍不作为integrated authority。
- **DG23-04 / CLOSED / CODE+TEST — runtime contract：** v2 wrapper lifecycle、opt-in mode、2:1 schedule、ZOH、timestamp/age、NMPC→WBC order、四类NMPC failure、zero/latch/reset和旧WBC mode均通过component与runner验证；loader失败保持启动环境失败语义。
- **DG23-05 / CLOSED / EVIDENCE — integrated formal：** authority v2与fresh replay均23/23 normal/reference、10/10 fault PASS；generated/binary provenance、model/OCP/WBC/plant/deadline、non-overwrite、hash和Phase14/15/18/20/21/22兼容性全部通过。
- **DG23-06 / CLOSED / REVIEW — claims：** REVIEW blocking findings为零且Verdict=`PASS`；随后创建RECORD并将ROADMAP标记complete。结论保持current nominal simulation-host范围。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；内部 versioned `NmpcReference`；Phase21 current nominal model/equilibrium/contact-wrench profile；20 ms NMPC phase与10 ms WBC control tick。
- 内部：`RobotState -> NmpcState12 -> NmpcProblem -> AcadosNmpcSolver -> NmpcResult{wrench12,status,KKT,age}`；accepted wrench只写入现有`WbcReference.interaction_wrench_flu`，随后走冻结`WeightedWbcController`。
- generated boundary：project-owned generator/config → immutable generated C/JSON/ABI manifest → private C++ wrapper；Controller headers和public ROS API不暴露acados types。
- 输出：canonical six-channel `TorqueCommand`；additive NMPC diagnostics；generation/model/OCP/solver benchmark、control/plant CSV、summary、manifest和append-only evidence。
- 必须保持：Phase15 coordinate/workspace、Phase21 WBC model/QP/task/reference其余字段、Phase22 dense ProxQP solver、FLU/quaternion/world twist、joint order/sign/unit、2/10 ms timing、5-step torque ZOH、fault latch/reset、public messages、Adapter/plant和既有evidence。
- 允许改变：`wheel_leg_core`新增acados generated artifact、private solver wrapper、config/result/diagnostics/tests和opt-in NMPC+WBC mode；`wheel_leg_mujoco`新增或最小扩展独立loop target；`tools/experiments`新增generator/oracle/benchmark/wrapper/evaluator；新增Phase23 config/method/evidence。任何超出此列表的改动先修订PLAN。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P23-T01 | 固化grounding、provenance与禁止复制清单 | Phase15/20/21/22、live Core/WBC、Simulink NMPC、CBM/Graphify | source/graph/coverage记录、current-vs-history差异、最小影响面与复用入口 | 历史candidate不被当作production authority；Phase22 baseline 24/24 PASS | done |
| P23-T02 | 冻结12D state/input/reference/time contract并审计16D历史candidate | P23-T01、RobotState、Phase15 xi、WbcReference | exact order/frame/sign/unit/chart、mapping、validity/stale/reset spec、golden vectors与model-closure decision | DG23-01 mapping部分；finite-difference/边界/fault tests PASS，internal/external wrench冲突关闭 | done |
| P23-T03 | 建立independent model与sensitivity oracle | P23-T02、current nominal profile、历史body dynamics | continuous/RK4 model、analytic Jacobian、equilibrium/energy/virtual-work/sign oracle、versioned corpus | DG23-01关闭；model/sensitivity误差和有效域满足冻结门槛 | done |
| P23-T04 | 冻结acados toolchain、OCP/codegen与solver profile | P23-T03、`/home/t/opt/acados`、SQP-RTI/HPIPM candidate | dependency/loader audit、exact OCP/RTI/HPIPM spec、generated ABI/layout/provenance、prototype、golden/failure corpus与1000-run benchmark | DG23-02；clean generation/build/run、parity、KKT、warm/reset、determinism、组合deadline PASS | done |
| P23-T05 | 冻结reference/cost/constraint profile | P23-T04、Phase22 envelope、预声明tuning/holdout cases | versioned weights/bounds/reference、delta-wrench transcription、ablation/attribution、tracking/recovery/fault thresholds | DG23-03；tuning后冻结，未见holdout与nonlinear pre-freeze全部PASS | done |
| P23-T06 | 实现受控generation pipeline与acados C++组件 | P23-T04/T05冻结spec | generator+lock/probe、immutable generated artifact+manifest、CMake/link/loader contract、private wrapper、warm/reset/result diagnostics | 与golden逐项一致；双clean regeneration；ordinary build无Python generator依赖；component/failure tests PASS | done |
| P23-T07 | 集成additive NMPC+WBC Core mode | P23-T06、现有WbcReference/Core safety | opt-in mode、2:1 update/ZOH、wrench injection、status/age/KKT diagnostics、zero/latch/reset | DG23-04 Core部分；NMPC→WBC顺序、stale/late/failure和旧mode回归PASS | done |
| P23-T08 | 建立/扩展full-3D NMPC loop与日志 | P23-T07、Phase22 runner/Adapter | 最小loop target、deterministic references/faults、逐tickacados/NMPC/WBC/control/plant日志 | DG23-04关闭；2/10/20 ms相位、5-step torque ZOH、双时钟、truth隔离和replayPASS | done |
| P23-T09 | 建立正式方法、profiles与evaluator | P23-T05/T08、Phase22 formal schema | `docs/experiments/`方法、versioned generation/model/solver/reference/formal config、wrapper/evaluator/manifest schema | formal前freeze；依赖探针/py_compile/non-empty拒绝/hash/schema/case/threshold完整 | done |
| P23-T10 | 执行full formal、fresh replay与历史回归 | P23-T09 frozen inputs、current nominal plant | 新`evidence/automated/<run-id>/`、summary/manifest/replay/non-overwrite/regression audit | DG23-05；23 normal/reference、10 fault、generated/model/OCP/WBC/plant/replay/history全部PASS | done |
| P23-T11 | REVIEW | 全部任务、live source和真实evidence | `REVIEW.md`；仅PASS后创建`RECORD.md` | DG23-06关闭、blocking findings=0后才更新ROADMAP complete | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Toolchain, Code Generation and OCP Pre-freeze

- 生成前先用仓库解释器探针实际需要的NumPy/SciPy/CasADi/acados_template，并记录Python、包版本、`ACADOS_SOURCE_DIR`、acados git commit/build cache、renderer来源/版本/hash、compiler/CMake、HPIPM/BLASFEO和shared-library SHA-256。当前CasADi和本地renderer缺失是待解决的environment gate，不是model/control FAIL；显式generation也不得临时联网下载未固定renderer。
- generator只写入新的临时目录；依赖探针、generator `py_compile`和输入schema通过后才生成。用相同冻结输入执行两次clean generation，比较generated C/H、solver JSON、ABI manifest和允许字段归一化后的hash；差异必须解释并冻结，不能把旧目录当成功缓存。
- ordinary clean build不得调用generator、访问网络或依赖CasADi；删除临时generation环境后，受控artifact仍须clean compile/link/run。generated source不得手工修补；任何变更回到generator/config并生成新version/hash。
- generated dynamics在equilibrium、symmetry、正负wrench、workspace和dynamic corpus逐项对DG23-01 C++/Python RK4 oracle；最大误差门槛沿用model authority，不能以闭环轨迹相似替代。
- 独立重算每个accepted iterate的objective、dynamics defect、bound violation、stationarity/complementarity；acados/HPIPM status和reported residual只作输入，不单独授权输出。exact数值门槛由P23-T04 corpus在production前冻结，不能沿用未经证实的ProxQP阈值。
- cold、repeated-warm、cycling dynamic warm各1000次；记录preparation、feedback/solve、wrapper audit、WBC和组合total、iteration、allocation、host/compiler和完整settings。更新tick `acados solve + audit + WBC`须`<10 ms`，reset后首解与cold deterministic。
- CMake/loader probe必须从显式acados prefix clean configure，验证headers、`acados`/HPIPM/BLASFEO链接、final binary `ldd/readelf`和无个人shell缓存的运行。选择的RPATH或显式environment hook必须写入DG23-02与manifest；未解析`.so`时不得创建formal输出。

### ROS Build and Component Tests

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco \
  --cmake-clean-cache \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  -DACADOS_ROOT=/home/t/opt/acados
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

- `ACADOS_ROOT`及最终loader机制必须由P23-T04冻结并由wrapper/manifest显式记录；上述candidate命令不能在环境gate关闭前当作approved build contract。
- 所有既有Core/ROS/MuJoCo tests和新增generation/model/OCP/wrapper/Core/runner tests为0 failures；warnings-as-errors保持通过。
- component覆盖capsule create/free、cold/warm、reset、non-finite/invalid state、unknown/nonzero status、iteration limit、late、KKT audit fail、stale和重复sequence。solver create/loader失败在启动probe阻止运行；已进入control tick的任何非成功路径从故障拍起六路严格zero并锁存。
- `kWeightedWbc`对冻结Phase22 component corpus逐项不变；新mode reset恢复cold、previous-applied input和相位零点的冻结语义。

### Formal MuJoCo

在创建稳定输出目录前使用仓库解释器完成实际依赖、binary loader和语法检查：

```bash
cd /home/t/W_L_ws
ACADOS_SOURCE_DIR=/home/t/opt/acados ./.venv/bin/python -c \
  "import mujoco, numpy, scipy, casadi, acados_template; print(mujoco.__version__, numpy.__version__, scipy.__version__, casadi.__version__, acados_template.__file__)"
./.venv/bin/python -m py_compile <phase23-generator-oracle-wrapper-evaluator-files>
<phase23-acados-loader-probe>
./.venv/bin/python <phase23-formal-wrapper> \
  --output-dir docs/workflow/phases/23-nominal-nmpc/evidence/automated/<new-run-id>
```

- 正式矩阵最低为继承Phase22的19 normal/perturbation + 6 fault，并追加4个预冻结NMPC straight-reference/return/recovery normal cases和4个solver-error/late/stale/non-finite fault cases；所有case使用同一frozen generated solver/model/reference profile。
- Phase22 plant/contact/slip/closure/WBC hard/task/torque/deadline gate不得变弱；追加generation/model/OCP/KKT、reference tracking/recovery、NMPC update/ZOH phase和age gate。tracking具体幅值与阈值由P23-T05在holdout前冻结。
- 输出目录必须不存在或为空；primary/fresh replay除明确列出的wall-clock字段外逐字段一致，plant CSV字节一致；manifest记录解释器/依赖、acados git/build/library及renderer hashes、generator input/output、generated ABI/source、model/solver/reference/controller/runner/scene/config/output hashes。
- fresh执行Phase14/15/18/20回归，重跑Phase21/22 frozen WBC component/formal兼容性入口；旧config/manifest/evidence hash保持不变。

## Acceptance Criteria

- [x] DG23-01关闭：12D base state/chart、12D external contact wrench、canonical mapping、locked-composite continuous/RK4 model、analytic sensitivity、equilibrium和有效域通过独立oracle；历史Euler/internal-wrench/16D candidate差异已解释。
- [x] DG23-02关闭：指定acados commit/build、generation依赖、generated ABI/artifact、loader、SQP-RTI/HPIPM OCP profile全部冻结；clean regeneration/build/run、model/OCP/KKT、warm/reset、1000-run determinism和更新tick组合`<10 ms`全部PASS，无silent fallback。
- [x] DG23-03关闭：cost/scale/constraints/reference、必要的optimizer-only augmentation、tuning-holdout split与正式tracking/recovery thresholds在holdout前冻结，ablation、attribution和未见nonlinear holdout全PASS。
- [x] production runtime只依赖受控generated solver、acados/HPIPM/BLASFEO和既有Core库，不依赖MuJoCo/MATLAB/Simulink/Python/CasADi/acados_template；NMPC只写现有12D wrench boundary，WBC/torque/public I/O保持冻结。
- [x] generated artifact可由冻结generator/env双clean复现且hash/provenance完整；ordinary clean build不运行generator或联网；final binary loader依赖从显式contract解析。
- [x] component/build tests全PASS；2:1 schedule、两拍wrench ZOH、RTI phase、timestamp/age、warm/cold/reset和所有control failure路径保持deterministic fail-zero/latch。
- [x] formal完成23 normal/reference + 10 fault，generation/model/OCP/WBC/plant/deadline gates全PASS；fresh replay、non-overwrite/hash和Phase14/15/18/20/21/22兼容性回归全PASS。
- [x] Phase21/22 source-of-truth config、manifest和evidence未被覆盖；Phase23所有结论引用新namespace和真实hash。
- [x] REVIEW blocking findings为零且Verdict=`PASS`后才创建RECORD、把ROADMAP标记complete；后续roll/yaw/turning仍须独立Phase。

## Execution Notes

按任务ID在本文件记录实际命令、结果、偏差和证据链接；不要建立第二份任务状态表。P23-T04/T05和DG23-02/03关闭前不得实现production generated solver/Core。任何需要改变physical state/input、public I/O、WBC数学、acados solver family、20 ms schedule、同步执行或plant的发现都先保留失败证据并将本Phase置为REWORK/blocked后修订PLAN；formal失败不得通过放宽Phase22 plant/WBC gate、隐藏fallback或覆盖旧run修复。

- 2026-08-28：P23-T01完成live CBM Verify、Graphify历史查询、20个精确路径与两个production scope coverage、三个parse-partial range源码fallback及clean Release基线。确认production无NMPC，最小接缝仅为现有12D `WbcReference.interaction_wrench_flu`；历史Euler/acados/last-valid与current contract冲突，不获继承。四packages构建通过，ROS汇总`24 tests, 0 errors, 0 failures`。详见[evidence/grounding.md](evidence/grounding.md)。
- 2026-08-28：P23-T02冻结`base_control_frame`原点、spatial shortest-arc rotation-vector、world twist及含旋转项的轮心相对坐标；12D input明确为已运输到同一base-control点的FLU wrench，禁止二次加入接触lever arm。依赖探针为MuJoCo 3.7.0/NumPy 2.2.6/SciPy 1.15.3。append-only `state-oracle-v1`因evaluator把静态xi误计为speed而保留为superseded FAIL；修正后的`state-oracle-v2`九门全PASS，最大映射FD误差`1.38e-10`、rotation-rate误差`8.01e-11`、determinism为0。详见[evidence/state-contract.md](evidence/state-contract.md)。
- 2026-08-28：P23-T03 pre-freeze closure审计拒绝历史16D production candidate：历史输入是wheel-to-body内部wrench，current WBC reference是已运输到base-control点的external contact wrench；复制历史xï与moment方程会混淆物理端口并二次计算lever arm。按DG23-01修订为12D locked-composite base candidate，xi/dxi仅保留workspace诊断；公共12D wrench/WBC边界、20 ms/N=20候选和安全语义不变。详见[evidence/model-closure-decision.md](evidence/model-closure-decision.md)。
- 2026-08-28：P23-T03 authority `model-oracle-v5`十门全PASS：current reduced dynamics误差`1.18e-11`、equilibrium导数`3.42e-15`、RK4-vs-DOP853 `1.06e-9`，并新增非零yaw-anchor验证`R=Exp(r)R_ref`。C++ AutoDiff golden test的continuous/RK4误差`3.11e-15/5.56e-17`、continuous/discrete sensitivity误差`1.34e-9/4.13e-11`；invalid/chart fail-closed，Release Core 7/7且repo summary 25/25 tests。DG23-01关闭，详见[evidence/model-oracle.md](evidence/model-oracle.md)。
- 2026-08-28：按用户指令第一次冻结Phase23。未编译验证的sparse-ProxQP OCP prototype及其CMake target已撤回；P23-T01～T03和DG23-01的真实PASS证据保留，未进入REVIEW、未创建RECORD、未宣称NMPC可用。
- 2026-08-28：用户解除冻结并指定NMPC改用`/home/t/opt/acados`。Codex重新打开route decision，核对live Core/CMake与acados安装：commit `21376cb1...`的Release/shared libraries存在，但默认loader未解析HPIPM/BLASFEO，`./.venv`缺CasADi。原“project-owned sparse ProxQP NMPC/no generated code”路线被本PLAN显式取代；Phase状态改为`active`，P23-T04=`doing`。这些是规划与环境事实，不是acados集成或NMPC PASS。
- 2026-08-28：P23-T04继续验证后，冻结解释器依赖探针、`py_compile`、clean Release build和loader/RPATH均通过；显式固定`ACADOS_SOURCE_DIR=/home/t/opt/acados`与`TERA_PATH=/home/t/W_L_ws/.cache/acados/t_renderer`后，双clean generation及checked-in artifact在仅忽略绝对输出路径及其派生hash时一致。generated model parity PASS；加入不读取acados KKT/multiplier的project-owned full-horizon objective、20段RK4 defect、全段bound与costate projected-gradient KKT audit后，cold/repeated-warm/dynamic-warm各1000次仍PASS，多次运行的保守p99/max为`2.247/2.900 ms`、`1.430/2.480 ms`、dynamic max `2.719 ms`；independent defect/KKT最大`3.38e-6/0.0428125`，门槛`1e-3/0.05`，wrapper已按两门fail-closed，单case evaluator smoke全门PASS。v7 normal integrated max `1.920 ms < 10 ms`，v7/v8 replay在四个声明wall-clock字段外逐字段一致且plant CSV字节一致。详见[`phase23-acados-t04-v1`](evidence/automated/2026-08-28-phase23-acados-t04-v1/summary.json)，DG23-02关闭、P23-T04=`done`、P23-T05=`doing`。v1～v8 formal仍不得倒推production authority，且其manifest缺少显式`supersedes/replay_of`关系，后续authority run必须补齐。
- 2026-08-28：P23-T05先写入hash固定的profile与4 tuning/6 holdout split，再执行真实重求解ablation和integrated tuning，最后在不改profile/gate下运行未见holdout。四solver×四case reduced corpus全解成功、input/state envelope violation为0；去掉longitudinal cost使step tracking消失，去掉terminal cost使return error放大`15228x`，uniform wrench cost使受保护轴变化放大`75.16x`。baseline normalized delta-wrench最大`0.0238423 < 0.05`，故冻结无optimizer-only memory。4 tuning和6 nonlinear组合holdout全PASS；holdout最坏组合耗时`3.558 ms`、独立defect`2.80e-6`、projected stationarity `0.03215`，所有tracking/recovery、WBC与plant gate通过。prefreeze v2显式supersede缺validator hash的v1且结果相同。详见[`t05-prefreeze.md`](evidence/t05-prefreeze.md)、[`phase23-t05-prefreeze-v2`](evidence/automated/2026-08-28-phase23-t05-prefreeze-v2/summary.json)、[`phase23-t05-tuning-v1`](evidence/automated/2026-08-28-phase23-t05-tuning-v1/summary.json)和[`phase23-t05-holdout-v1`](evidence/automated/2026-08-28-phase23-t05-holdout-v1/summary.json)。DG23-03关闭、P23-T05=`done`、P23-T06=`doing`。
- 2026-08-29：P23-T06将冻结相对state envelope转写到v2 generated artifact的stages 1..N，stage 0保持exact measured state；依赖探针、golden parity、双clean normalized generation、ordinary build无codegen、RPATH/loader、cold/warm/reset及3×1000 component均PASS。ROS汇总`26 tests, 0 errors, 0 failures`。详见[`phase23-t06-v1`](evidence/automated/2026-08-29-phase23-t06-v1/summary.json)。
- 2026-08-29：P23-T07/T08完成opt-in Core与runner闭环；component直接覆盖NMPC→WBC、2:1/ZOH、reset、solver-failure/late/stale-age=2/non-finite strict zero+latch。逐tick日志新增state-bound audit，5-step torque ZOH和双时钟由formal逐case验证；DG23-04关闭。
- 2026-08-29：P23-T09冻结[实验方法](../../../experiments/phase23_nominal_acados_nmpc.md)、append-only formal v2/replay config和generated provenance选择；manifest新增完整依赖、v2 generated hashes、`supersedes/replay_of`，非空目录拒绝保持exit 2。
- 2026-08-29：P23-T10 authority [`formal-v2`](evidence/automated/2026-08-29-phase23-acados-formal-v2/README.md)与[fresh replay](evidence/automated/2026-08-29-phase23-acados-formal-v2-replay/README.md)均23/23 normal/reference、10/10 fault PASS。normal NMPC+WBC最大`3.641244 ms`、defect`2.26e-6`、projected-KKT`0.02507`、input/state bound violation均0；33 plant CSV字节一致，control仅四个声明wall-clock字段不同，双run各99项hash无漂移。coordinate和fresh Phase14/15/18/20回归PASS，Phase21/22 component/formal契约与旧source-of-truth未修改；DG23-05关闭。
- 2026-08-29：P23-T11 REVIEW=`PASS`、blocking findings=0，DG23-06关闭；随后创建RECORD并将ROADMAP Phase23标记complete。结论仅限current nominal MuJoCo simulation host。

## Blockers

None. Phase 23已完成；真机仍冻结，后续roll/yaw/turning必须建立独立Phase并重新冻结其模型、reference和验证门槛。
