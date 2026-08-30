# Phase 42: Wheel-Spin Drift / Contact-Loss Causal Attribution — PLAN

状态：`complete`  
日期：2026-08-30

## 审核结论

用户提出的 attribution Phase 获批，但须按以下证据边界执行：

1. Phase41 只证明 production H0 越过旧 tick96 gate 后，于 tick111 首次发生 right-wheel
   contact loss；它没有证明 wheel spin 是 contact loss 的原因。
2. 时间先后只建立候选因果顺序。最终分类还必须有 wheel-row 瞬时动力学闭合和 fixed-state
   counterfactual 支持；不得把相关性写成 first mover。
3. Phase41 的 10 ms CSV 不含 native full state、`qacc`、逐接触 wrench 或 generalized-force
   分解，不能作为精确 snapshot authority。Phase42 必须追加同步的 2 ms plant chronology。
4. `qfrc_constraint` 聚合 contact、equality 和 limit constraint；不得直接解释为某侧 wheel
   contact contribution。逐接触贡献必须从 contact wrench/Jacobian 独立重建并验证总闭合。
5. 本 Phase 只归因，不修改 controller、fixed wrench、task、gain、planner、contact 或 plant。
   rollout ablation 不预授权；只有基础审计无法唯一分类时，才允许先写冻结 addendum 再运行。

## 唯一目标

在 Phase41 frozen production H0 上建立从 tick0 到 tick111 的可复现证据链，回答 wheel-rate
drift 的 first mover、common/differential evolution、base/leg/load/contact redistribution 与
right contact loss 的因果关系，并唯一分类为 P42-A/B/C/D，或用定量证据分类为 P42-E/P42-U。

本 Phase 不选择或实施 repair，也不重新运行 Phase34 tracking。

## Grounding

- Phase41 RECORD/H0 authority：Model B、Phase27 Minimal、fixed equilibrium interaction wrench、
  no xi task、no target、zero gains；tick96 model OK，tick111 first independent failure为 right
  contact loss，tick110以前 frozen gates全 PASS，formal/replay parity exact。
- Phase39 closure authority：wheel absolute angle 已降为非 material；wheel spin rate仍是 material
  hidden variable，C3 symmetric physical `ddxi=1.65810 m/s²`，但这不等同于 H0 contact-loss cause。
- CBM project `W_L_ws`，generation `2026-08-29T06:47:42Z`。相关 loop/WBC 路径存在
  metadata-change、not-tracked 或 tools-excluded coverage，故当前源码是实现 authority。
- 现有 Graphify 图只用于继承 Phase31/32/39 的设计与实验关系，不替代 live source 或新 formal。
- 可复用实现包括现有 weighted/standing loop 的 `mj_contactForce`、geom-order sign、world-frame
  force、penetration/slip/closure，Phase36 Audit 的 `mj_fullM`/contact Jacobian/`qacc`/xi acceleration，
  Phase35 runner 的 sustained-trend helper，以及 Phase28 的同步 control/plant CSV 结构。

## Frozen hypotheses

- **H42-A — request/realization is not rolling equilibrium**：在 frozen state，当前 fixed request 经
  production WBC realization 后已给 wheel DOF material nonzero instantaneous acceleration。
- **H42-B — unstabilized rolling mode**：tick0 mean drive可近零，但 wheel-rate common/differential
  mode缺少 restoring/dissipative mechanism，初值或微扰被保持或放大。
- **H42-C — asymmetric coupling drives differential drift**：左右 contact/load/geometry/dynamics
  的 material asymmetry 先于并驱动 differential wheel-rate 与 right unload。
- **H42-D — base/leg/contact drift is primary**：base/leg/contact quantity先成为 material，normal
  load redistribution驱动失触；wheel spin主要是伴随或次级状态。

H42-A～D 可相互耦合；只有无法合理分离且各自都有独立 mechanism evidence 时才判 P42-E。

## Frozen experiment contract

### Baseline

完全继承 Phase41 H0：同 executable semantics、Model B、initial state、fixed request、controller、
solver、2 ms physics / 10 ms control、stop gates和 tick111 stop-on-first-failure。不得延长越过
first contact loss。formal 与 fresh replay 写入独立 append-only 目录。

### Two synchronized tables

`control.csv` 每个 10 ms control tick 记录 production model/controller result、requested/realized
interaction wrench、QP residual、slack、torque和所有 frozen gate margin。

`plant.csv` 每个 2 ms physics substep记录 `control_tick`、`physics_substep`，至少包含：

- native `qpos[17]`、`qvel[16]`、`ctrl[6]`、`qacc[16]`，及按名称解析的 wheel DOF address；
- wheel q/dq/ddq及 common/differential mode，physical xi/dxi/ddxi；
- base full pose/twist及 x/z/pitch、vx/vz/pitch-rate，所有 hip/knee q/dq；
- wheel-origin pose/velocity；
- 每个 wheel-floor contact 的 geom pair、position、frame、distance、dimension、local wrench、
  signed world wrench、COP/penetration、rolling/lateral slip、friction coefficient、tangential norm
  和 friction margin；多 contact 不得提前聚合后丢失拓扑；
- `qfrc_bias/passive/actuator/applied/constraint`、full mass matrix与逐接触 generalized contribution
  所需的 Jacobian/wrench；
- per-side actual contact wrench、normal/tangential load，与 WBC requested/realized interaction
  wrench分别命名，不得混写。

每个 control boundary 必须分别记录：

1. **pre-command**：held previous `ctrl` 下的当前 native `qacc`；
2. **post-command instantaneous**：写入本 tick production torque 后，仅 `mj_forward`、不积分所得
   `qacc`。

H42-A 只以 post-command instantaneous 审计为 authority；连续轨迹以正常 stepped plant row为准。

### Dynamics convention and closure

实现前通过 MuJoCo 3.7.0 oracle 冻结并验证：

```text
M(q) qacc + qfrc_bias
  = qfrc_actuator + qfrc_passive + qfrc_applied + qfrc_constraint
```

只有 full-vector residual 及逐接触 reconstruction 通过预冻结 tolerance 后，才可解释 wheel rows。
逐接触 force 使用 `mj_contactForce`、contact frame、wheel geom顺序修正和 contact Jacobian/
`mj_applyFT` 重建；必须明确 equality/limit 与 wheel-floor contact 的剩余项。pyramidal contact 的
solver row数不得以 `contact.dim` 简单切片。friction ratio仅作 diagnostic，不冒充精确 cone margin。

### Event detection and key snapshots

正式轨迹生成前，在 versioned config 中冻结每个 signal family 的单位、方向、absolute detection
floor、persistent window、minimum same-direction steps和 sensitivity bands；config进入 manifest。
默认复用 Phase35 的五样本 sustained-trend 规则。所有 onset 同时报 numeric、material 和至少一档
sensitivity；若 signal-family ordering随 sensitivity反转，不得宣称严格先后。

快照选择规则在看新结果前固定为：tick0；每个 signal-family 的 first persistent material onset
及其前/后一个 control tick；以及 `loss-10`、`loss-5`、`loss-2`、`loss-1`。去重、排序并限制在
bilateral-contact区间；tick111只作 terminal event，不用于双侧 equilibrium inference。

## Counterfactual hierarchy

### CF42-1 — fixed-state instantaneous rolling-equilibrium audit

从 plant authority 精确恢复 key snapshot 的 native `qpos/qvel/ctrl/time`，复算本状态的 production
WBC command和 post-command `mj_forward`。逐 wheel row报告 `M qacc`、bias、actuator、passive、
applied、left/right wheel-floor contact、other constraints及closure residual，回答当前 request/
realization在 tick0及后续状态是否是 rolling equilibrium。

### CF42-2 — zero-wheel-rate fixed-state audit

在同一 key snapshot 只令 native left/right wheel `qvel=0`，其余 `qpos/qvel`、request、controller
和 contact semantics不变；重新求 production command和 instantaneous dynamics。报告 wheel ddq、
common/differential ddq、actual contact wrench/load及 physical ddxi 的 absolute/relative delta。
该反事实只说明 local hidden-rate sensitivity，不代表一条 dynamically reachable trajectory。

必要时可在同一 snapshot 用预冻结中心差分估计 wheel-rate到 wheel acceleration 的 2x2 local
Jacobian并变换到 common/differential basis；它是 attribution oracle，不是新增 damping。

### Conditional rollout ablation

仅当 DG42-04/05 后仍有两个以上机制无法区分，先在 PLAN addendum 中写明唯一待区分命题、
单一 intervention、可达性/约束语义、阈值及停止条件，再允许一个最小 rollout。不得临时运行
“remove differential generalized drive”或“symmetrize contact”——两者会改变 force channel或
constraint manifold，除非先证明可实现且不会把 repair混入 attribution。

## Gates and tasks

| ID | Gate/task | PASS condition | Status |
| --- | --- | --- | --- |
| P42-T01 / DG42-00 | provenance + no-repair contract | Phase41 H0/hash/semantics frozen；diff只含Phase42 instrumentation/runner/config/docs | done |
| P42-T02 / DG42-01 | schema, threshold and oracle freeze | 两表schema、pre/post-command语义、onset rules、key-tick rule、closure tolerances在formal前冻结 | done |
| P42-T03 / DG42-02 | instrumented baseline + replay | 首失效仍为tick111 right contact loss；所有既有control/plant字段与Phase41 semantic-equal；native snapshot/replay可精确恢复 | done |
| P42-T04 / DG42-03 | time-causal audit | 自动给出各family numeric/material onset、trend、crossing、peak derivative和sensitivity-stable partial order | done |
| P42-T05 / DG42-04 | rolling-row balance audit | 所有key snapshot full-vector/逐接触closure PASS；tick0至loss前wheel-row net drive有可审计分解 | done |
| P42-T06 / DG42-05 | zero-rate counterfactual | 仅wheel dq改变被证明；ddq/contact/load/ddxi local effect定量且fresh replay一致 | done |
| P42-T07 / DG42-06 | attribution decision | chronology + mechanism + counterfactual支持唯一P42-A/B/C/D或定量P42-E；否则P42-U | done |
| P42-T08 | REVIEW | 审查因果强度、替代解释、证据限制和下一Phase branch；不得包含repair | done |
| P42-T09 | RECORD only after PASS | 仅REVIEW=PASS后写稳定分类与获批下一步 | done |

任一 gate FAIL 停在当前层；不得为了得到分类跳过 closure、replay 或 counterfactual validity。

## Classification contract

- `P42-A_fixed_request_realization_not_rolling_equilibrium`：tick0 post-command已有 material wheel-row
  net acceleration，force balance指出来源，且其符号/演化与后续 drift一致。
- `P42-B_unstabilized_wheel_rate_mode_primary`：tick0 rolling drive不 material；rate perturbation/local
  Jacobian显示非恢复/非耗散，rate evolution先于load/contact material drift并能解释其传播。
- `P42-C_asymmetric_coupling_drives_differential_instability`：material left/right drive/load/contact
  asymmetry先于 differential drift，wheel-row balance和counterfactual均支持该通道。
- `P42-D_base_leg_contact_drift_primary_wheel_secondary`：base/leg/contact first mover先于wheel material
  drift；zero-rate intervention对卸载链影响不 material，机械分解支持load/contact主通道。
- `P42-E_multiple_coupled_causes`：两个或以上机制均有独立 material contribution，无法在不改变
  frozen system的前提下约化为单一 first mover；必须量化各贡献，不能用“复杂耦合”代替分析。
- `P42-U_unresolved`：replay、closure、event ordering或反事实 validity不足以支持以上分类。

`P42-U`、DG FAIL 或 classification依赖未批准ablation时 REVIEW=`REWORK`，不得写 RECORD。

## Strictly forbidden

- 不加 wheel-rate damping、rolling-speed/xi task、gain、planner/NMPC或新 equilibrium wrench；
- 不修改 WBC objective/constraint、torque/contact/friction、model geometry/mass/inertia或initial state；
- 不放宽 Phase41 frozen gates，不越过 tick111继续跑；
- 不把 requested/realized WBC wrench 当 actual MuJoCo contact wrench；
- 不以单一 plot、单一阈值顺序或相关性宣布 cause；
- 不运行 Phase34，不宣布 tracking、真机或一般稳定性 PASS。

## Required deliverables

- `PLAN.md`；
- versioned Phase42 config、dedicated instrumentation target和runner；
- `chronology-audit.md`与machine-readable event table；
- `rolling-equilibrium-audit.md`及wheel-row balance table；
- `zero-wheel-rate-counterfactual.md`；
- `attribution-decision.md`、`REVIEW.md`；仅PASS后创建`RECORD.md`。

## Verification

- 先用`./.venv/bin/python`做实际依赖 probe并记录MuJoCo/NumPy/SciPy版本，再`py_compile`；
- 从`ros_ws/`执行targeted colcon build/test；
- instrumented baseline对Phase41 authority逐字段semantic parity，formal/fresh replay写新目录；
- snapshot restoration、pre/post-command语义、contact sign/order和whole-vector balance均有独立unit/oracle；
- manifest包含 executable/config/model/runner/Phase41 authority hashes；
- JSON/XML/CSV completeness parse、non-finite scan、`git diff --check`；
- controller/plant/fixed-wrench diff=false，Phase34 run=false，repair=false。
