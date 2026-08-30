# Phase 39: Idealized Nominal Wheel Model + Architecture Revalidation — PLAN

状态：`complete`  
日期：2026-08-30

## 审核结论

用户提供的 Phase 39 方向获批，但按以下约束收紧后执行：

1. centered-COM model 只是一套用于架构辨识的 ideal nominal plant，不是 CAD/真机质量属性修正；
2. Phase 32 必须完整重放 C1、C2、C3 和 wheel-angle hybrid 三个正式入口，不能只运行
   `run_phase32_markov_closure.py`；
3. wheel absolute angle/mesh phase 与 wheel spin rate 是不同 hidden variables：前者应在圆柱碰撞
   和 centered COM 下消失，后者仍可能通过 rolling/slip 动力学产生真实差异；
4. `same x16 + same requested wrench` 是原 closure 命题；realized wrench 必须单独报告，只有其
   parity 通过时才能作 physical-interaction-wrench closure 表述；
5. Phase 32 的 C1/C2 构造允许 soft-contact height/normal-motion 改变，不能在本 Phase 事后拆成
   新的“zeta/vertical/contact-coordinate”独立因果族；需要新 pair 才能细分，留作后续 Phase；
6. Phase 34 tracking 不在本 Phase 执行。Phase 39 只完成 nominal plant、Phase 37、Phase 32、
   Phase 35 H0 和 workspace evidence 更新。

本 PLAN 冻结范围、决策、任务和 gate；实现与验证按冻结顺序执行。

## 唯一目标

在同时移除已知的 rotating collision-mesh phase artifact 和 radial-COM phase artifact 后，回答：

```text
same x16 + same requested interaction wrench
+ different admissible hidden/full-body state
→ physical ddxi 是否仍超过 Phase 32 frozen closure gate？
```

并更新以下架构证据，而不直接批准任何生产控制架构：

```text
16D wheel-aware NMPC Eq.(12)
vs
12D base NMPC + full-body WBC wheel realization
```

## Grounding 与继承 authority

- CBM project `W_L_ws`，generation `2026-08-29T06:47:42Z`，9580 nodes / 17210 edges，
  `ready`。`docs/`、`tools/` 被索引策略排除，Phase 35 loop 和 Phase 37/38 新模型未被当前
  generation 跟踪，因此相关事实已直接读取源码、配置、manifest 和 evidence。
- Graphify 现有图确认 Phase 35 runner/loop、WBC model 和历史 Phase 关系；图仅用于历史关系，
  不覆盖当前源码和 formal evidence。
- Phase 36 `P36-D`、Phase 37 `P37-D`、Phase 38 `P38-A` 保持冻结，不重新归因。
- Phase 32 的最终 authority 是：
  - C1/C2：`leg-nullspace-v5`，method `phase32_markov_closure_v2.json`；
  - C3 wheel rate：`markov-closure-v4`，同一 method；
  - wheel absolute angle / discrete patch：`wheel-angle-hybrid-v4`，method
    `phase32_wheel_angle_hybrid_v1.json`；
  - frozen closure difference `0.05 m/s²`、normalization `0.5 m/s²`、normalized gate `0.1`；
  - requested wrench parity `1e-12`，realized-wrench relative parity `2%`。
- Phase 35 H0 authority 是 `workspace-attribution-formal-v2/H0-a.csv`；现有 C++ loop 接受显式
  model path，因此原则上复用现有 executable，不修改 controller/WBC。

## 两套模型的冻结语义

### Model A — CAD-like mismatch / robustness reference

保持 Phase 37 模型不变：

```text
scene_axisymmetric_collision_v1.xml
wheel_leg_axisymmetric_collision_v1.xml
```

- axisymmetric cylinder collision；
- 原 mesh-derived mass、COM、COM-centered inertia 和 inertial orientation；
- radial COM offset 约 `0.12 mm`；
- 用于后续 mismatch/robustness replay，不作为本 Phase nominal architecture verdict 的 plant。

### Model B — ideal nominal control-validation plant

追加创建：

```text
scene_axisymmetric_centered_com_v1.xml
wheel_leg_axisymmetric_centered_com_v1.xml
```

Model B 从 fresh-compiled Model A descriptor 生成显式 wheel inertial authority。唯一允许的
compiled rigid-body parameter change是两轮 `body_ipos` 的 body-X/Y 归零：

```text
left/right body_ipos[0:2] = 0
```

必须保持：

- wheel mass；
- axial COM component `body_ipos[2]`；
- COM-centered principal moments；
- principal-frame orientation（允许 quaternion `q/-q` 等价，比较 rotation/tensor）；
- visual mesh、hinge pose/axis、joint/actuator、cylinder geometry；
- friction、contact、solver、base/leg parameter 和所有非 wheel-body parameter。

显式 inertial serialization 只为冻结 compiled parity；不得解释为真实装配轮质量属性。Model A
不得覆盖、删除或改写。

## 严格禁止

Phase 39 不允许：

- 修改 Eq.(12)、x12/x16 state/equations 或 full-to-reduced projection；
- 修改 NMPC Q/R/Qe、horizon、solver、planner、lifecycle 或 reference；
- 修改 WBC task、wheel tracking gain、fixed equilibrium wrench 或 torque limit；
- 恢复 zeta/base-height/pitch/posture task，或增加第四组 gain；
- 修改 friction/contact coefficients、wheel radius/width/mass/inertia tensor、actuator；
- 改动 Phase 32/35/36/37/38 原配置、证据、阈值或 conclusion；
- 绕过、放宽或删除 `±1 rad` live workspace gate；
- 运行 Phase 34 tracking，或用本 Phase 结果直接宣布 12D/16D 最终正确；
- 将 Model B 宣称为 CAD truth、real truth 或 production plant。

## Authority matrix

| Question | Frozen input | Phase 39 action |
| --- | --- | --- |
| Model B 是否只改 radial COM | Phase 37 compiled model + Phase 38 V1 descriptor | append-only compiled parity |
| absolute wheel phase 是否 clean | Phase 37 corpus/config/threshold | Model B formal + fresh replay |
| C1 leg configuration | Phase 32 `leg-nullspace-v5` | same pairs/method, new scene only |
| C2 leg velocity | Phase 32 `leg-nullspace-v5` | same pairs/method, new scene only |
| C3 wheel spin rate | Phase 32 `markov-closure-v4` | same pairs/method, new scene only |
| wheel absolute angle | Phase 32 `wheel-angle-hybrid-v4` | same pairs/delta, new scene only |
| pre-target drift | Phase 35 H0 formal-v2 | same 150 ticks/fixed wrench/live gate, new scene only |

## Gates

### DG39-00 — Nominal-model parity

在 stable evidence 目录创建前完成 dependency probe、model compile 和 parity dry run。PASS 要求：

- Model A 与 Model B 拓扑、names、dimensions、options 和非 wheel parameters 完全一致；
- 两个 wheel body 之外不存在 compiled parameter difference；
- wheel raw compiled parameter difference 仅为 `body_ipos` X/Y；
- centered radial COM `<=1e-15 m`；
- axial COM、mass、principal moments、body-frame inertia tensor/orientation 的 max absolute
  difference `<=1e-12`；
- cylinder/visual geom、joint、actuator、contact parameter difference `<=1e-12`；
- descriptor、source/config/script hashes 和 Model A↔B diff 写入 manifest。

任何额外差异：`P39-A_nominal_model_parity_failure`，停止。

### DG39-01 — Reclose Phase 37 phase isolation

在 Model B 上重放 Phase 37 frozen periodic/contact ON/OFF corpus，不改
`phase37_axisymmetric_collision_v1.json` 的 threshold 语义。必须同时验证：

- contact centroid/normal/depth/topology phase invariance；
- `q` 与 `q+2π` periodicity；
- mass/bias、qacc、physical `ddxi`、normal/tangential load phase modulation；
- frozen isolation rule：contact-on effect
  `<= max(0.001 m/s², 10 × contact-off effect)`；
- formal 与 fresh replay semantic summary 完全相等。

FAIL：`P39-A_nominal_phase_isolation_not_closed`，停止，不运行 Phase 32/H0。

### DG39-02 — Complete Phase 32 closure replay

DG39-01 PASS 后，使用只替换 `scene` 的 append-only Phase 39 method config，完整运行：

1. `run_phase32_leg_nullspace.py`：C1 configuration、C2 velocity；
2. `run_phase32_markov_closure.py`：C3 common/differential wheel rate；
3. `run_phase32_wheel_angle_hybrid.py`：common wheel absolute-angle pair。

冻结不变：authority cases/ticks、pair scale/sign、full/half construction、state projection、requested
wrench、WBC sweep、fixed-baseline-torque branch、oracle epsilon、contact/finite requirements和全部
Phase 32 thresholds。旧 hybrid runner 的 process exit 以“discrete failure expected”为成功条件；
Phase 39 evaluator 必须按 frozen numeric gate 重新裁决，不能把 artifact 消失导致的非零 exit 当作
环境失败，也不能修改旧 runner/evidence。

每个 pair/family 必须报告：

- exact pair ID 与 baseline authority hash；
- same-x16 projection residual；
- requested wrench difference；
- realized wrench relative difference；
- composed-WBC 和 fixed-baseline-torque `ddxi` difference；
- contact IDs/dimensions、penetration、load、constraint velocity；
- formal/replay equality；
- 与 Phase 32 authority 的同 ID 数值差异及 pass/fail transition。

Smooth C1/C2/C3 只有 projection、oracle、full/half、bilateral contact、finite gates 全部有效后，
才按 max symmetric `ddxi <=0.05 m/s²` 裁决 closure。Wheel-angle family 不再使用 hybrid
derivative consistency；全部 authority pair 的 max symmetric difference 都必须 `<=0.05 m/s²`
才算该 family restored。

#### DG39-02 classification

- `P39-B_x16_closure_restored`：C1、C2、C3、wheel-angle 四族全部有效且全部通过 frozen
  closure gate。
- `P39-C_x16_closure_improved_but_not_restored`：wheel-angle family 通过，但 C1/C2/C3 中
  至少一族通过、至少一族仍失败。已知 artifact 改变了原结论强度，但 x16 仍不 closed。
- `P39-D_x16_nonclosure_structurally_persists`：wheel-angle family 通过，但 C1、C2、C3
  三族仍全部失败。已知 artifact 被移除，非闭合仍跨 configuration/velocity/rate families 保持。
- `P39-U_closure_replay_inconclusive`：任一 authority validity/replay gate 失败、wheel-angle
  family 仍失败，或结果不属于上述互斥分支。不得强行映射为 B/C/D。

注意：C3 是 wheel **rate** / rolling-slip family，不得因 wheel **angle** phase artifact 消失而
自动判定其应通过。只有 realized-wrench parity `<=2%` 时才允许追加 physical-wrench closure
表述；requested-wrench closure 结论仍须同时参考 fixed-torque branch。

### DG39-03 — Phase 35 H0 Minimal Hold replay

仅在 DG39-02 完整结束（无论 B/C/D）后运行，不因架构分类提前跳过。使用 Model B 和现有
Phase 35 C++ loop，保持：

```text
Phase27 Minimal profile
+ fixed equilibrium interaction wrench
+ no xi task / no target motion
+ 150 ticks / 0.01 s control / 0.002 s physics
+ live ±1 rad workspace gate enabled
```

运行 formal 与 fresh replay，记录原 Phase 35 全量字段。分类冻结为：

- `P39-E_H0_spin_drift_removed`：无 workspace rejection，且 150 ticks 内左右 wheel canonical
  delta 的最大绝对变化均 `<=0.1 rad`；
- `P39-F_H0_spin_drift_persists`：任一 wheel drift `>0.1 rad` 或出现 workspace rejection；
- `P39-U_H0_inconclusive`：contact、solver、finite、hard/slack/torque 或 replay validity gate
  先失败。

不得绕过 live gate 延长 H0；若需要 gate-free/long-horizon rollout，必须建立独立 Phase。

### DG39-04 — Workspace-contract reassessment

只解释证据，不修改生产 gate：

- Phase 36/37/39 periodic evidence 可否定 `±1 rad` 作为 wheel absolute-angle
  model-validity singularity 的依据；
- 不能由 periodicity 自动否定 controller-domain、state-estimation 或 safety envelope；
- 报告 H0 rejection 是否仍由 right-wheel delta 首先触发，以及 gate 前是否已有 contact、WBC、
  torque/slack 或 base-state failure；
- 只有独立长时域安全/可观测性/数值表示验证完成后，才可在后续 Phase 修改 workspace contract。

## Architecture evidence update

`architecture-evidence-update.md` 必须分为：

### Evidence still valid

- Phase 31 measurement/kinematics contracts；
- Phase 32 pair construction、projection 和 full-body oracle 中在 Model B 仍通过的部分；
- Phase 36 rotating mesh artifact、Phase 38 radial-COM numerical causality；
- Phase 35 对原 Model A/mesh plant 的历史观测，作为 mismatch evidence 保留。

### Superseded for causal interpretation

- rotating mesh angle引起的 closure difference；
- eccentric COM 引起的 absolute-phase difference；
- 上述两者不得继续单独作为 x16 intrinsic non-closure 或 12D architecture 的证据。

### Revalidated evidence

- Model B 上 C1/C2/C3/wheel-angle family 的逐项 verdict；
- Model B 上 H0 drift 和 workspace chronology；
- composed-WBC 与 fixed-torque、requested 与 realized wrench 的分离解释。

架构表述边界：

- P39-B 只授权后续公平重建/比较 16D 与 12D candidates，不恢复旧 Eq.(12) production；
- P39-C/P39-D 加强 12D responsibility-split candidate，但不证明其 tracking/robustness PASS；
- P39-U 不得用于选择架构。

## Tasks

| ID | Task | Deliverable | Status |
| --- | --- | --- | --- |
| P39-T01 | 冻结 Model A/B 语义和 source hashes | PLAN/config contract | done |
| P39-T02 | 创建 append-only centered-COM model/scene | model revision | done |
| P39-T03 | compiled parity audit | `nominal-wheel-model-parity.md` | done |
| P39-T04 | Phase 37 corpus formal + fresh replay | `phase-isolation-revalidation.md` | done |
| P39-T05 | C1/C2 nominal replay | Phase 32 evidence | done |
| P39-T06 | C3 wheel-rate nominal replay | Phase 32 evidence | done |
| P39-T07 | wheel-angle nominal replay | Phase 32 evidence | done |
| P39-T08 | B/C/D/U aggregate classification | `phase32-closure-revalidation.md` | done |
| P39-T09 | Phase 35 H0 formal + fresh replay | `phase35-h0-revalidation.md` | done |
| P39-T10 | reassess finite wheel-angle gate evidence | `workspace-contract-reassessment.md` | done |
| P39-T11 | update architecture evidence | `architecture-evidence-update.md` | done |
| P39-T12 | verification and REVIEW | `REVIEW.md` | done |
| P39-T13 | create RECORD only after REVIEW PASS | `RECORD.md` | done |

任务状态仅使用 `todo / doing / done / blocked`。

## Required deliverables

执行后至少包含：

```text
PLAN.md
nominal-wheel-model-parity.md
phase-isolation-revalidation.md
phase32-closure-revalidation.md
phase35-h0-revalidation.md
workspace-contract-reassessment.md
architecture-evidence-update.md
REVIEW.md
```

只有 `REVIEW=PASS` 后创建 `RECORD.md` 并将 ROADMAP 改为 `complete`。所有模型、配置和
evidence append-only，不覆盖 Phase 32–38 authority。

## Verification protocol

- repository Python 固定使用 `./.venv/bin/python`；
- stable output 前探针并记录 MuJoCo/NumPy/SciPy 实际版本，再执行所有新/改 Python 的
  `py_compile`；
- 每个 formal 后运行 fresh replay，manifest 记录 command、interpreter、dependencies、
  model/config/source/authority hashes；
- 新目录拒绝覆盖，formal/replay 不共享 mutable output；
- 若不修改 ROS/C++，复用现有 Phase 31/35 executables，不进行无意义 rebuild；若 executable
  不存在或必须改 C++，只能从 `ros_ws/` 执行 `colcon build` 和相关 tests；
- `git diff --check`、JSON parse、XML compile/parity 和 evidence schema check 是 REVIEW 前置项；
- build/import/environment failure 不得记录为模型或架构 evidence FAIL。

## Stop conditions

- DG39-00 FAIL：停止，不运行任何 dynamics replay；
- DG39-01 FAIL：`P39-A`，停止，不运行 Phase 32/H0；
- Phase 32 任一 validity/replay authority 不成立：`P39-U`，停止架构裁决；
- H0 validity 失败：保留 Phase 32 结论，但 H0/workspace 问题为 `P39-U`；
- 任何结果需要新 state、pair、task、gain、gate bypass 或 controller change 才能解释时，建立
  新 Phase，不扩张 Phase 39；
- Phase 34 servo tracking 只允许由 Phase 39 REVIEW 推荐的下一独立 Phase 重开。

## REVIEW 必答问题

1. Model B 是否只改变两轮 radial COM？
2. material absolute wheel-phase sensitivity 是否关闭？
3. C1、C2、C3 rate、wheel-angle 四族分别是 PASS/FAIL/invalid 哪一种？
4. x16 closure 属于 P39-B、P39-C、P39-D 还是 P39-U？
5. wheel absolute angle 与 wheel spin rate 是否得到正确分离？
6. requested/realized wrench 与 composed/fixed-torque evidence 是否支持相应表述？
7. Model B 上 Phase 35 H0 drift 是否仍存在？
8. `±1 rad` 失去了哪些 model-validity 依据，又保留哪些 safety/domain 未决问题？
9. 当前 evidence 允许重新考虑 16D、继续 12D candidate，还是无法裁决？
10. 基于实际 branch，下一阶段唯一推荐实验是什么？
