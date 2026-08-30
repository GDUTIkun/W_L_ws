# Phase 41: Workspace Contract Correction + H0 Production Revalidation — PLAN

状态：`complete`  
日期：2026-08-30

## 审核结论

用户方案获批并按以下边界执行：

1. Phase40 已冻结 R3，本 Phase 不再审计 representation，也不重新设计 controller。
2. production correction 必须删除 diagnostic policy 分叉：默认且唯一的 WBC workspace contract
   为 leg bounds enforced、wheel q finite-only。
3. `inspectWorkspace` 可保留 historical wheel ±1 margins 作为 attribution telemetry，但 wheel
   entries 不得再决定 `inside()`、`first_failed_index` 或 enforced minimum margin。
4. Phase35 历史 target/证据保持 append-only；Phase41 用同一源代码的独立 production
   revalidation target延长 H0，并以正式默认 `evaluate/step` API运行。
5. 若 H0 首个独立 failure 不是 Phase40 shadow 的 tick111 right contact loss，判为 production ↔
   diagnostic semantic mismatch并停止；无论结果如何都不在本 Phase 修 contact loss或运行Phase34。

## 唯一目标

把正式 WBC workspace contract 从 wheel `|q-qeq|<=1 rad` 改为 wheel q finite-only，同时保持
leg、contact、hard/slack、torque、solver、base envelope和dq validity不变；随后以production
semantics重跑 frozen H0并确定第一个独立 failure。

## Grounding

- Phase40 RECORD：R3、`P40-A+F+G`，next authorized work即本 Phase。
- CBM project `W_L_ws` generation `2026-08-29T06:47:42Z`；相关路径 metadata changed，已直接
  读取当前源码。Phase40 policy、WBC model/controller和Phase35/40 loop均在当前脏工作树内。
- Graphify 本地图确认 workspace inspector、NominalWbcModel、Phase35 loop与历史 contract链；
  当前源码与 Phase40 stable evidence仍为技术 authority。

## Frozen decisions

### Production R3 contract

```text
hip/knee workspace: unchanged and enforced
wheel q: finite-only; absolute magnitude never causes kOutsideWorkspace
wheel dq: unchanged RobotState finite validation and existing rate/safety consumers
physical model/controller: unchanged
```

删除 `WheelWorkspacePolicy` 和所有调用参数；不得留下另一个非默认 bypass API。

### H0 authority

Model B、Phase27 Minimal、fixed equilibrium interaction wrench、no xi task、no target、0 gain，
physics 2 ms/control 10 ms。最长2000 ticks/20 s或3 revolutions；任一 contact、model/solver、
hard>1e-7、slack>0.05、torque margin<-1e-10、base position change>0.1 m、rotation>0.35 rad、
linear speed>2 m/s、angular speed>5 rad/s即停。

冻结预期仅作 parity gate：old-bound crossing tick96；first independent failure tick111、right
contact loss；formal/replay semantic error<=1e-12。

## Strictly forbidden

- 不修改 WBC task/gain、wheel-rate damping、xi、fixed wrench、planner、NMPC；
- 不修改 contact/friction、torque limits、model geometry/mass/inertia；
- 不修复或继续越过 first independent failure；
- 不运行 Phase34，不宣布 tracking PASS；
- 不修改真机 protocol 或推断真机无 mechanical limit。

## Gates and tasks

| ID | Gate/task | PASS condition | Status |
| --- | --- | --- | --- |
| P41-T01 / DG41-00 | contract-only diff | only wheel magnitude rejection removed; policy branch deleted | done / PASS |
| P41-T02 / DG41-01 | workspace regression | hip/knee, NaN/Inf still reject; wheel 2π/10π accepted | done / PASS |
| P41-T03 | unchanged-gate regression | contact/hard/slack/torque/solver/base/dq test authorities PASS | done / PASS |
| P41-T04 | production H0 target/runner | default API, frozen source/config, append-only outputs | done |
| P41-T05 / DG41-02 | formal + replay | old gate crossed without rejection; first independent failure identified; replay parity | done / PASS |
| P41-T06 | REVIEW | answer contract correction, semantic parity and next-Phase eligibility | done / PASS |
| P41-T07 | RECORD only after PASS | stable decision and next branch | done |

## Classification

- `P41-A_workspace_contract_corrected_contact_loss_reproduced`；
- `P41-B_production_shadow_semantic_mismatch`；
- `P41-U_regression_or_evidence_invalid`。

## Required deliverables

`PLAN.md`、`contract-diff-audit.md`、`regression-audit.md`、
`production-h0-revalidation.md`、`REVIEW.md`；PASS后创建`RECORD.md`。

## Verification

- `./.venv/bin/python` dependency probe + `py_compile` before stable output；
- 从 `ros_ws/` targeted colcon build/test；
- formal/replay独立稳定目录，decision artifacts semantic-equal；
- manifest hashes executable/config/model/runner/Phase40 authority；
- JSON/XML parse、`git diff --check`；
- Phase34 run=false，contact-loss repair=false。
