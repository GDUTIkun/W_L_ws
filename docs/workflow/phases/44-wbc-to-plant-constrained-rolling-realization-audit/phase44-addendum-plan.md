# Phase 44 Addendum: Regime-Aware Directional Rolling Authority Audit — PLAN

Status: `complete`
Date: 2026-08-30

## Goal

只修复 `DG44-06`：在 Phase44 已冻结 snapshot 上，以 regime signature 和单边有限差分判断
rolling authority 是否可解释。原 Phase44 evidence 保持 append-only；不进入 Phase45 repair。

## Scope

- 复用 Phase42 common ticks `0/46/74/101/110` 和 Phase43 B/C/D own-trajectory key ticks。
- 对每个 baseline、`+/- {1, 0.5, 0.25} delta` 冻结 contact/load、QP inequality、solver/task signature。
- 仅在 perturbation signature 与 baseline 相同的方向计算 `G_QP`、`G_MJ`、`G_mis`；用下一小档
  检查 directional convergence。
- 按 output family 分类 `R44-S/P/O+/O-/B`，并对 trusted direction 审计 contact authority transfer。
- 形成 late-state chronology association，重新审查 `P44-A/B/C/E/U`。

## Out of Scope

gain/weight 调整、新 task/candidate/repair、Model B/contact/friction/torque limit/interaction wrench修改、
Phase34/12D NMPC/16D repair、planner、10 s repair rollout、Phase45 implementation。

## Frozen Decisions

1. 基础 delta：xi `0.01 m/s^2`，native wheel acceleration `0.2 rad/s^2`；只允许 scale
   `1.0/0.5/0.25`。主 derivative 使用仍在 baseline regime 内且有下一小档可检查的最大 scale。
2. equality tolerance 不比较连续 contact position/load 值。离散 signature 比较：
   - 每侧 contact existence/count、排序后的 `(geom1, geom2, dim)`；原始 order 仅记录；
   - normal-load state、friction-utilization band、penetration state、tangential slip sign；
   - torque/contact-friction/acceleration inequality逐 row lower/upper near-active code及分类统计，阈值
     `distance<=1e-7`；
   - torque-bound state、slack state、enabled task rows、solver success/status、固定 QP `42x104`、
     candidate/profile。solver multiplier active-set 继续为 `unavailable`。
3. contact/load thresholds 在看 addendum 结果前冻结：positive load `1e-6 N`，friction near-active
   utilization `0.95`，penetration deadband `1e-8 m`，slip deadband `1e-5 m/s`，slack inactive
   `1e-7`、material `0.02`。
4. plus/minus regime-valid 只表示 signature 与 baseline 相同；trusted 还必须有下一小档同 regime，且
   directional gain relative difference `<=0.05`。没有下一小档时为 untrusted。
5. `R44-S` 要求两边 regime-valid 且该 output-family 的 `G+/-` relative difference `<=0.05`；超过为
   `R44-P`，不得平均。仅一边 valid 为 `R44-O+/-`，两边均 invalid 为 `R44-B`。
6. condition number 只对完整且 trusted 的 channel matrix计算；one-sided/incomplete result不强算。
7. QP/plant/contact mapping、snapshot selection、no-repair contract继承原 Phase44，不重新设计。

## Tasks

| ID | Task | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- |
| P44-A-T01 | 冻结 signature/delta/分类规则 | 本 PLAN、`regime-signature.md`、addendum config | DG44-R1/R2/R3 | done |
| P44-A-T02 | 实现 directional oracle | runner、JSON/CSV outputs | DG44-R4/R5/R6 | done |
| P44-A-T03 | contact transfer与transition审计 | transfer CSV、events JSON、audit docs | DG44-R7/R8 | done |
| P44-A-T04 | formal/fresh replay/regression | formal/replay bundles | DG44-R10 | done |
| P44-A-T05 | classification/review | decision、`REVIEW-addendum.md` | DG44-R9 | done |
| P44-A-T06 | Phase44 RECORD/ROADMAP | `RECORD.md`、ROADMAP | 仅 addendum REVIEW PASS 后 | done |

## Acceptance Gates

- DG44-R1 signature 在 rerun 前冻结；R2 snapshot provenance不变；R3 no-repair contract保持。
- DG44-R4 每个 audited family 分类；R5 trusted derivative通过有限 delta convergence。
- DG44-R6 只将 trusted direction用于正式 `G_QP/G_MJ/G_mis` 解释。
- DG44-R7 trusted contact transfer balance闭合；R8 late nonsmooth/boundary显式识别。
- DG44-R9 使用 repaired oracle重新审查 Phase44 classification。
- DG44-R10 dependency probe、py_compile、targeted colcon build/test、parse/nonfinite、snapshot/dynamics/
  contact closure、fresh replay、`git diff --check` 全部通过。

## Execution Note

CBM `W_L_ws` generation `2026-08-29T06:47:42Z` 用于 live core 定位；`docs/`、`tools/` 被索引排除，
故 addendum runner、config、文档以直接源码为 authority。Graphify 仅用于 Phase42–44 历史关系核对。
