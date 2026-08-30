# Phase 44: WBC-to-Plant Constrained Rolling Realization Audit — PLAN

Status: `review`  
Date: 2026-08-30

## Goal

在不创建 repair candidate、不调 gain/weight、不改变 plant/contact 的前提下，闭合
`a_des -> (nudot_QP, lambda_QP, tau_QP) -> MuJoCo contact/acceleration -> physical rolling motion`，
并以可重复的分层证据解释 Phase43 的 xi 改善与 native rolling mode 失稳为何并存。

## Current State

- Phase42 已冻结 `P42-E_multiple_coupled_causes`：tick0 非 rolling equilibrium、初始左右接触不对称，
  以及后续 wheel-rate-sensitive amplification 共同导致 tick111 首次右轮失触。
- Phase43 已冻结 `P43-U`/REWORK：A/B/C/D 均未通过全部 mandatory gates；B/C/D 在 tick0
  将 common `ddxi` 从约 `-0.120206` 改为约 `-0.0313 m/s^2`，但右轮 native qdd 仍约
  `-3.09 rad/s^2`，所有 nominal case 均未达到 10 s。
- live controller 链为 `ControllerCore::stepPhase27MinimalNmpcWbc -> stepWeightedWbc ->
  WeightedWbcController::step -> WeightedWbcProblem::assemble/NominalWbcModel::evaluate`；QP 的前
  12 个 physical variables 是 reduced generalized acceleration，18:30 是左右 contact wrench。
- Phase43 formal-v3 保存 baseline 及 B/C/D own-trajectory control rows，包含 base、native active/passive
  joint position/velocity、task reference、QP solution摘要和 contact load，可恢复 exact native snapshot。
- 缺少逐 task QP attribution、正确的 reduced-to-native affine acceleration oracle、共同 generalized-force
  space 的 QP/MuJoCo contact comparison、rolling material-point kinematics 与 local authority matrices。
- CBM project `W_L_ws`，generation `2026-08-29T06:47:42Z`。核心 WBC 源文件 metadata 已变化，
  因此图查询仅用于定位，技术事实以直读源码为准；`docs/`、`tools/`、evidence 未进主索引。
- Graphify 现有图覆盖 Phase21 的 reduced/contact 历史设计，但未包含 Phase42/43 最新结论；本 Phase
  不因该历史图缺口重建图，Phase42/43 文档和真实 formal evidence 是当前 authority。

## Scope

- 复现 Phase43 baseline tick111，并验证 Phase42/43 authoritative hashes 与 no-repair flags。
- 使用两组 snapshot：Phase42 common-state ticks `0/46/74/101/110`；Phase43 B/C/D 各自 formal-v3
  trajectory 自动选取的关键 ticks（包含 tick0、增长/残差/xi/base/slack/contact事件和 failure 前5/1 tick）。
- 对 B/C/D 已有 wheel task 记录 desired、QP realized、raw/normalized/weighted residual/cost，并记录
  dynamics/contact hard residual、slack、torque margin与可可靠取得的 active inequalities。
- 在 exact native state 上应用 frozen WBC torque，仅调用 `mj_forward`，比较 QP prediction 与 MuJoCo
  actual acceleration/contact realization。
- 计算 wheel-center、xi、native wheel rate、instantaneous wheel-surface material-point slip velocity/
  tangential acceleration和normal/tangential load。
- 对已存在的 B/C/D task reference 做冻结的中心差分，形成 `G_QP`、`G_MJ`、`G_mis`，并在
  left/right 与 common/differential basis报告 self/cross gain、sign、condition与near-null directions。
- 定量分解 C 的 `ddxi` 为 base、leg、wheel与`Jdot*v` contribution，形成正式分类
  `P44-A/B/C/E/U` 及 mechanism tag。

## Out of Scope

- gain/weight/task scale tuning，新增 task、candidate、controller structure或 repair rollout。
- R43-A+B、planner、Phase34 step/ramp、12D NMPC、16D Eq.(12) repair。
- Model B、contact/friction、torque limit、interaction wrench、initial state、solver、control/plant timing修改。
- 新 10 s repair rollout、hardware/real validation或 Phase45 implementation。

## Frozen Decisions

1. Controller reduced coordinate为
   `nu=[v_base_world(3), omega_base_world(3), canonical active qdot(6)]`，wheel rows为8/11；
   MuJoCo native tree velocity为16D。
2. QP-to-native acceleration不是纯线性映射。每个 state 使用
   `qacc_native_pred = N(q) * nudot_QP + c_N(q,qvel)`；`c_N` 是闭链 passive acceleration bias，
   active/base rows为零。`N*nudot` 单独只能作为 linear contribution，不得命名为完整 prediction。
3. acceleration 主 residual 为 `e_acc_native=qacc_MJ-(N*nudot_QP+c_N)`；同时用
   `N^T M N` 加权 least-squares projection得到 reduced actual acceleration，作为可比较的12D辅助判据。
4. contact 主判据只在共同 reduced generalized-force space：
   `Qcontact_QP=sum(W_i*lambda_i)`，`Qcontact_MJ=N^T*qfrc_constraint_MJ`。
   禁止任意把12D reduced generalized force抬升到16D native force；raw lambda 只作 representation摘要。
5. WBC QP contact wrench是每侧6D contact-centred wrench，表达在 controller contact frame；MuJoCo
   contact representation可能多点/不同basis，故不以 raw lambda difference判 mismatch。
6. plant torque使用与 Phase42/43 相同 canonical-to-native sign；snapshot后只 `mj_forward`，不得 integration。
7. material-point tangential acceleration使用 rigid-body instantaneous formula；contact centroid migration的
   finite difference不得称为 material-point acceleration。
8. finite-difference delta在看结果前冻结：xi acceleration `0.01 m/s^2`，native wheel acceleration
   `0.2 rad/s^2`；同时用半幅检查局部线性/对称性，不做 sweep。
9. `G_QP` 与 `G_MJ` 对同一 exact state、同一 profile和同一 reference求导；candidate warm-start history
   只用于恢复 production-equivalent solve ordering，中心差分各自 cold/reset以避免路径污染。
10. active-set仅报告距 bound `<=1e-7` 的 assembled inequalities及torque/contact类别；solver若不提供
    multiplier/basic-set authority则明确标记 unavailable，不从 residual猜 multiplier activity。

## Open Questions / Decision Gates

- DG44-01：wheel task target是否在QP内实现；若否，哪类 objective/constraint contribution material？
- DG44-02/03：QP predicted acceleration/contact generalized force是否由 MuJoCo actual plant实现？
- DG44-04/06：已有 wheel task authority在plant中进入 wheel、base、leg、contact/load/slip的比例为何？
- DG44-05/07：xi成功是否主要由wheel、base、leg或`Jdot*v`实现，是否仍有未约束rolling direction？
- DG44-08：证据支持 P44-A/B/C/E 中哪类；任一关键oracle不可信则必须为P44-U。

## Interfaces and Compatibility

- 输入：Phase42 native snapshots、Phase43 formal-v3 control CSV、frozen phase43 config、Model B scene；
  state单位和DOF/sign顺序继承 Phase21/42/43。
- 输出：append-only JSON/CSV formal bundle与审计Markdown；诊断入口不得进入 runtime command path。
- 必须保持：42-variable/104-row QP、12 equality rows、solver settings、production controller semantics、
  RobotState/TorqueCommand boundary及已有 Phase43 artifacts不覆盖。
- 允许改变：新增只读诊断字段/审计 executable、runner、Phase44 config与文档。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P44-T01 | provenance与snapshot authority | Phase42/43 manifest/formal-v3 | manifest、snapshot table、DG44-00 | tick111、hash、own-trajectory恢复、no-repair flags | done |
| P44-T02 | task/QP attribution | WBC model/problem/solution | task-reference/QP table、`task-realization-audit.md` | raw/normalized/weighted算术自洽，hard residual闭合 | done |
| P44-T03 | affine acceleration oracle | `N`、passive bias、MuJoCo `qacc` | QP-vs-MJ qacc table、`qp-to-plant-acceleration-audit.md` | closure、sign/DOF mapping、whole-vector residual | done |
| P44-T04 | reduced contact oracle | QP wrench、`qfrc_constraint` | contact table、`contact-realization-audit.md` | controller/MJ reduced generalized-force构造闭合 | done |
| P44-T05 | rolling/material-point oracle | exact snapshot/contact geometry | rolling table、`rolling-coordinate-audit.md` | slip velocity与instantaneous acceleration definitions检查 | done |
| P44-T06 | local authority matrices | frozen +/-delta probes | G matrices、`authority-matrix-audit.md` | symmetry、half-delta independence、basis conversion | done |
| P44-T07 | C paradox decomposition | C own-trajectory key snapshots | decomposition JSON、`xi-realization-decomposition.md` | components sum to realized ddxi within tolerance | done |
| P44-T08 | formal/replay/regression | T01-T07实现 | append-only formal与fresh replay | dependency probe、py_compile、colcon build/test、parse/nonfinite/diff | done |
| P44-T09 | classification与review | 全部证据 | `phase44-decision.md`、`REVIEW.md` | 五个问题逐项回答、DG44-00..09审查 | done |
| P44-T10 | record/roadmap | REVIEW PASS | `RECORD.md`、ROADMAP complete | 仅PASS后执行 | blocked |

## Validation Plan

### Automated

- `./.venv/bin/python -c 'import mujoco,numpy,scipy; ...'`：记录实际解释器与版本。
- `./.venv/bin/python -m py_compile tools/experiments/run_phase44_realization_audit.py`。
- 从 `ros_ws/` 运行 targeted `colcon build` 与 `colcon test`，审计 executable和core/adapter tests通过。
- formal与fresh replay写入两个新目录；忽略时间戳/运行时字段后 machine-readable semantic error在冻结容差内。
- CSV/JSON parse、全数值 non-finite scan、dynamics/contact closure、delta symmetry/half-delta、xi decomposition
  closure、authority matrix reproducibility与`git diff --check`通过。

### Manual / Evidence

- 审查所有关键 snapshot 的 task、acceleration、contact、rolling与authority链；不得从build或runner exit推断分类。
- 对 P44-A/B/C/E 的每个 material layer给出独立数值；oracle缺失或不可信则P44-U/REWORK。
- 显式记录 `gain_tuning=false`、`weight_tuning=false`、`new_task=false`、
  `new_repair_candidate=false`、`phase34_run=false`、`nmpc_12d_run=false`、
  `repair_16d=false`、`plant_modification=false`、`contact_modification=false`。

## Acceptance Criteria

- [ ] DG44-00 baseline/provenance/no-repair PASS。
- [ ] DG44-01 task reference到QP realization完全可观察。
- [ ] DG44-02 affine QP-to-MuJoCo mapping验证通过。
- [ ] DG44-03 reduced contact generalized-force reconstruction闭合。
- [ ] DG44-04 contact authority-transfer audit有效。
- [ ] DG44-05 rolling/slip coordinate definitions有效。
- [ ] DG44-06 `G_QP/G_MJ/G_mis`可重复且中心差分有效。
- [ ] DG44-07 C paradox由定量分解解释。
- [ ] DG44-08 分类由显式分层证据支持，或诚实判P44-U。
- [ ] DG44-09 formal/fresh replay、targeted regression、parse/nonfinite/diff PASS。
- [ ] REVIEW 无 blocking finding；仅此时创建RECORD并把ROADMAP标记complete。

## Execution Notes

- P44-T01 grounding：CBM定位到live controller/plant边界；coverage显示WBC核心文件相对索引metadata
  已变化，已转为直读源码。Graphify仅确认Phase21历史关系，Phase42/43缺失作为限制记录，不触发无关全图重建。
- 草案审核修正：原 `T_QP->MJ*nudot` 改为state-dependent affine mapping；原native contact-force
  comparison改为共同reduced generalized-force主判据。这两项是执行前数学放行条件。
- P44-T01：Phase43 control-site state通过MuJoCo site Jacobian反演native free-joint state；在Phase42
  双authority ticks上验证 `qpos=2.22e-16`、`qvel=4.44e-16` max error，无需新10 s rollout。
- P44-T02..07：formal-v1完成；DG44-00..05/07 PASS，DG44-06因late-snapshot odd symmetry
  `0.61634`、half-delta差`0.51337` FAIL；按冻结规则分类P44-U，provisional layered finding=P44-E。
- P44-T08：targeted build PASS；core 17/17、adapter 6/6；formal/fresh replay semantic error=0；
  12 CSV/5 JSON each可解析、non-finite=0、`git diff --check` PASS。

## Blockers

DG44-06 authority matrix validity在late snapshots失败；REVIEW=REWORK，P44-T10不得执行。
