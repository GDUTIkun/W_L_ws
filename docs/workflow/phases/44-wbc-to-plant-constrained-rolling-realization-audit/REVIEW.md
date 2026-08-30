# Phase 44: WBC-to-Plant Constrained Rolling Realization Audit — REVIEW

Status: `PASS`（initial REWORK 由 append-only addendum 关闭）
Reviewer/date: Codex, 2026-08-30

## Review Scope

- PLAN: [`PLAN.md`](PLAN.md)
- Implementation: WBC affine/diagnostic fields, Phase43 single-snapshot diagnostic output, Phase44 runner/config.
- Evidence: [`realization-audit-formal-v1`](evidence/automated/realization-audit-formal-v1),
  [`realization-audit-replay-v1`](evidence/automated/realization-audit-replay-v1).

## Implementation Check

| PLAN Task | Delivered | Result |
| --- | --- | --- |
| P44-T01 | Phase42/43 provenance、common/own snapshot恢复与selection | PASS |
| P44-T02 | desired/QP raw-normalized-weighted attribution与active diagnostics | PASS |
| P44-T03 | `N*nudot+c_N` native oracle、M-weighted reduced辅助比较 | PASS |
| P44-T04 | reduced QP/MuJoCo contact force与wheel-row authority balance | PASS |
| P44-T05 | material-point slip/acceleration、load/penetration | PASS |
| P44-T06 | G_QP/G_MJ/G_mis及半幅/对称检查 | FAIL（数据完成，validity gate失败） |
| P44-T07 | C native xi decomposition | PASS |
| P44-T08 | formal/replay/build/test/parse/nonfinite/diff | PASS |
| P44-T09 | 五问题与分类审查 | REWORK/P44-U |
| P44-T10 | RECORD/complete | NOT AUTHORIZED |

## Validation Results

| Validation | Actual Result |
| --- | --- |
| dependency probe | `.venv`: MuJoCo 3.7.0, NumPy 2.2.6, SciPy 1.15.3 |
| `py_compile` | PASS |
| targeted `colcon build` | wheel_leg_core + wheel_leg_mujoco PASS |
| targeted `colcon test` | core 17/17、adapter 6/6；aggregate 35 tests, 0 failures |
| native snapshot reconstruction | qpos `2.22e-16`、qvel `4.44e-16` max error，PASS |
| whole-vector/contact closure | `<=1e-8`，PASS |
| CSV/JSON/non-finite | 每个run 12 CSV + 5 JSON可解析；non-finite=0 |
| fresh replay | machine-readable semantic max error `0`，PASS |
| `git diff --check` | PASS |
| DG44-06 | odd symmetry `0.61634`、half-delta `0.51337` > `0.05`，FAIL |

## Findings

### Blocking

1. **DG44-06 authority validity FAIL.** late Phase43 states对冻结±delta出现active-set/contact-regime
   nonsmooth response；虽然formal/replay完全一致，但单一中心Jacobian在required snapshot set上不可信。
2. **DG44-08 classification blocked.** 冻结规则规定authority matrix不可信时必须P44-U；因此不能把
   provisional `P44-E`写成正式结论，也不能创建Phase45 repair。

### Non-blocking

- tick0 wheel task在QP内几乎精确实现，plant native wheel acceleration仍显著偏离；这一 finding不依赖
  late-snapshot local linearity。
- C的physical xi acceleration由leg/wheel-center motion主导，native wheel-spin contribution为0；分解
  oracle closure PASS。
- reduced contact difference与authority cancellation支持provisional `B-contact` mechanism，但仍受
  DG44-06对完整local-authority结论的限制。

## Decision and Evidence Review

- 冻结决策保持：是。gain/weight/task/repair/Phase34/12D NMPC/plant/contact均未修改或运行。
- 证据足以支持正式P44-A/B/C/E：否；足以支持P44-U与明确rework原因：是。
- 下一步：留在Phase44，以addendum冻结regime-aware directional derivative/unchanged-active-set oracle；
  不创建Phase45，不实施repair。

## Verdict

`REWORK`（initial review）

当时不创建 `RECORD.md`，ROADMAP保持 `review`。

## Addendum Supersession

`REVIEW-addendum.md` 使用 regime-aware directional oracle关闭 DG44-06，DG44-R1..R10 全部PASS，
最终分类 `P44-E`。该 addendum 不覆盖上述 initial REWORK 事实；它追加授权 `RECORD.md` 与 ROADMAP
complete。Phase45 未在本 Phase 创建或执行。
