# Phase 46: Hip-Common-Safe Rolling Realization Repair — PLAN

状态：`review`  
日期：2026-08-31

## Goal

只验证一个 repair hypothesis：在 Phase45 compatible-H0 上，仅从 slip task 的 realization row
静态删除 bilateral hip-common direction，判断它是否足以关闭 actual plant 的 harmful
`slip-common -> ddxi-common` cross-coupling，同时保留有效 rolling/slip authority。

## Frozen Decisions

- 继承 Phase45 Model B、compatible equilibrium wrench、unified rolling semantics、state、contact、
  friction、torque limits、solver、gain、weight 与 10 ms/2 ms timing。
- 12D reduced acceleration 顺序为 base linear `0:2`、base angular `3:5`、left
  hip/knee/wheel `6:8`、right hip/knee/wheel `9:11`。
- 唯一 projection 为
  `h=(e6+e9)/sqrt(2)`、`P_safe=I-h*h^T`、`J_slip_new=J_slip*P_safe`。
  等价地只从 slip row 的 columns 6/9 同时减去 `(row[6]+row[9])/2`。它保留 hip
  differential 与其余所有方向，且不投影 slip observable、bias 或 target。
- xi task、contact model、interaction wrench、task scale、所有 penalty/weight 均不改变。
- 禁止第二 candidate、gain/weight/wrench tuning、新 task、planner/tracking/12D NMPC、
  coupled xi-slip redesign、precompensation、state-dependent/dynamic projection。

Phase45 frozen root cause 为 `G_MJ[ddxi_c,slip_c]=-4.2950931926`，其中 actual
hip-common contribution `-4.1083402117`（95.65%）；classification 为
`B-QP_CANCELLATION_BROKEN_IN_PLANT`。

## Gate Contract

严格执行 `EQ -> AUTH -> REAL -> SHORT -> 10 s -> post-repair authority reaudit`；任一 mandatory
gate FAIL 后不进入后续 gate。fixed-state AUTH 使用 xi-only/slip-only、common/differential、正负 branch
与 scales `1/0.5/0.25`，delta 仍为 `0.01 m/s2`。

- EQ：Phase45 阈值不变：`abs(ddxi_L/R)<=0.05 m/s2`、
  `abs(material tangent acceleration)<=0.01 m/s2`，并要求 bilateral contact/active、hard
  `<=1e-7`、slack `<=0.05`、torque margin `>=-1e-10 Nm`、whole dynamics/contact closure
  `<=1e-8`。
- AUTH：common actual cross `abs(G_MJ[ddxi_c,slip_c])<=0.1` 且相对 Phase45 至少降低
  90%；common actual slip self gain 必须为正并保留至少 Phase45 `0.0308422887` 的 50%；
  common actual xi self gain必须为正且 `>=0.05`。QP/MuJoCo common unified projected authority
  必须同号且 actual magnitude `>=0.05`；所有 branch/scale convergence relative `<=0.05`。
  同时完整报告 QP/MuJoCo 2x2 common transfer matrix、differential authority 与全部 cross gains。
- REAL：对 slip-common-only 的 QP 与 actual 使用 Phase45 相同 base、leg/non-wheel、native wheel、
  `(Jdot,v)`、逐 leg DOF/common-differential mode decomposition；闭合 `<=1e-10`。报告 actuator、
  contact、remaining/lhs generalized force 与 hip/knee common projection；whole dynamics/contact
  closure保持 `<=1e-8`。不得出现 knee-common/base/native-wheel/differential-leg 的 material migration，
  定义为任一替代 mode 对 `ddxi_c` 的绝对贡献 `>0.1`；native wheel qacc gain不得超过 Phase45
  absolute baseline，且 slip self不得低于 AUTH 门。
- SHORT/10 s：沿用 Phase45 223/1000 tick 与全部 contact、xi、wheel-rate、slip、base/full-body、
  WBC、wrench realization 阈值，不放宽。
- reaudit：仅在10 s全部 mandatory gate PASS 后，于 ticks `0/50/500/999` 对相同 fixed-state
  authority 和 regime signature 复审；判据与 AUTH 相同。
- fresh replay：对实际进入的 gates 做 append-only fresh replay，semantic max error `<=1e-11`；
  replay不授权进入已经被 mandatory FAIL 阻止的后续 gate。

## Tasks

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-T01 | 冻结 projection 与 Phase 边界 | PLAN、ROADMAP、source/CBM grounding | done |
| P46-T02 | 实现唯一 Phase46 profile | static rank-one slip-row projection；QP diagnostic 同 row；xi/contact/weights unchanged | done |
| P46-T03 | component verification | targeted build/test；projection exactness、idempotence、only-columns-6/9、QP algebra | done |
| P46-T04 / DG46-EQ | compatible-H0 equilibrium | tick0 closure完成；frozen EQ limits FAIL | done |
| P46-T05 / DG46-AUTH | full directional transfer audit | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T06 / DG46-REAL | physical realization audit | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T07 / DG46-SHORT | 223-tick rollout | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T08 / DG46-ROLL | 1000-tick rollout | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T09 / DG46-REAUDIT | post-repair authority | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T10 | formal/replay/review | dependency probe、py_compile、fresh replay、classification、REVIEW；PASS only creates RECORD | done |

### REWORK tasks — frozen nominal, limited increment

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R01 | 实现唯一 incremental restriction | zero delta 维持原 rolling QP；仅 external slip-common delta 启用 frozen nominal hip-common hard equality | done |
| P46-R02 / DG46I-EQ | 重新验证 compatible-H0 equilibrium | 原 ddxi/tangent/contact/load/hard/slack/torque/dynamics/contact gates 全部 PASS | done |
| P46-R03 / DG46I-AUTH | frozen directional incremental audit | `+/-`、`1/0.5/0.25`、QP/MuJoCo decomposition 与 closure 完成；actual cross 未下降，FAIL | done |
| P46-R04 | formal、fresh replay 与 REVIEW | dependency probe、py_compile、build/test、non-finite audit、replay 完成；保持 REWORK | done |

### REWORK tasks — root-cause closure

本轮继续 Phase46，且只做 compatible-H0、tick0、fixed-state attribution；不实施 repair。
前序 `contact-realization-sensitivity-formal-v1..v3` 仅为定义演进，正式 local 4D authority
从 v4 起；旧目录保留并在本轮 decision evidence 中标明 superseded/rejected reason。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R05 | 审计既有 attribution 与真实 QP 方程入口 | authoritative/superseded 关系、CBM/source coverage、QP operator dump contract | done |
| P46-R06 | QP torque-source 与 contact-space closure | `qp-torque-source`、`contact-space-balance`；真实 sign/order 下 EOM closure | done |
| P46-R07 | actual point-contact realizability gate | 左右 `G_p` rank/SVD、投影残差、nominal+increment feasibility、harmful contribution | done |
| P46-R08 | conditional KKT / actual response / Fn→Fr closure | 按触发条件执行 KKT；复用 frozen torque replay 闭合 solver response 与 Fn_L/Fn_R chain | done |
| P46-R09 | root-cause decision 与正式验证 | 四个主文档、machine-readable decision、fresh replay、build/tests/non-finite/diff-check | done |

P46-R05～R09 的停止条件与用户冻结顺序一致：任一 EOM、mapping、replay 或 regime closure
不可信即 `U-UNTRUSTED`；否则持续到唯一 first material mismatch 与唯一 repair layer 已闭合。

### REWORK tasks — point-contact-realizable repair

本轮继续 Phase46，不开启新 Phase。冻结唯一 repair candidate 为 actual two-point force image
parameterization：对每轮 contact-frame 6D wrench 使用
`P_w=diag(I3, I3-a*a^T)`，其中 `a` 是轮轴在 contact frame 中的单位向量。`P_w` 必须一致进入
dynamics、wrench cone、interaction-wrench realization 与对外 physical solution；QP 仍为 42D，旧
profile、task/gain/weight、friction、solver、plant/contact 参数均不变。新增独立
`kPhase46PointRealizableRolling` profile，禁止叠加此前 hip-common projection 或 hard equality。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R10 | 冻结 realizable parameterization 与影响面 | rank-5 projector；四处一致应用；历史 profile bitwise/algebra unchanged | done |
| P46-R11 | 实现独立 repair profile | model axis contract、QP projection、controller output、MuJoCo repair executable/config | done |
| P46-R12 / DG46P-COMP | component verification | `P=P^T=P^2`、rank 5、轴向 moment `<=1e-10`、EOM/constraint/task closure | done |
| P46-R13 / DG46P-EQ | compatible-H0 equilibrium | component PASS；actual right `ddxi=-0.07538` 超 `0.05`，mandatory FAIL | done |
| P46-R14 / DG46P-AUTH | fixed-state authority | DG46P-EQ FAIL，未授权进入；已误先生成的 full-direction result 仅 diagnostic-only | blocked |
| P46-R15 / DG46P-REAL | realizability closure | DG46P-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-R16 | formal、fresh replay 与 REVIEW | dependency probe、py_compile、build/tests、non-finite、EQ fresh replay、REVIEW 完成；保持 REWORK | done |

执行顺序为 `COMP -> EQ -> AUTH -> REAL -> fresh replay`；mandatory gate FAIL 时停止后续运行并保持
`review/REWORK`。只有全部 PASS 才能创建 RECORD，并将 Phase46/ROADMAP 标记 complete。

### REWORK tasks — post-R1 equilibrium attribution

`DG46P-EQ` FAIL 后不进入原 AUTH/REAL。只在相同 frozen tick0 比较 Phase45 compatible-H0 与
point-realizable candidate，按 `torque/free -> QP contact cancellation -> actual contact response ->
remaining` 闭合 actual ddxi 变化；本节不实现第二 candidate。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R17 | frozen before/after parity | state/mass/xi-map delta `0`；old/new 均 bilateral two-point contact | done |
| P46-R18 | post-R1 causal balance | actuator-free、QP contact、actual contact、remaining 对 left/right ddxi closure `3.11e-14` | done |
| P46-R19 | next mismatch decision | `R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1`；fresh replay `0`；未授权下一 candidate | done |

### REWORK tasks — point-subspace equivalence audit

只审计 current `P_w` 与 actual `G_p` 的 range；不修改 projector/controller，不定义下一 repair，
不运行 trajectory。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R20 | actual two-point image vs current projector | 独立 `G_p` rebuild、`P_G`/`P_w` mutual containment、principal angles、双向 reconstruction、reference transport、formal/replay | done |

### REWORK tasks — exact R1 point-force-image repair

继续使用 `kPhase46PointRealizableRolling` 作为唯一 candidate，但以 frozen actual two-point
`P_G=G_pG_p^dagger` supersede approximate pure-`Ml` projector。只执行 `COMP -> EQ`；EQ 后停止。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R21 | exact projector implementation | actual two-point `G_p/P_G` 一致进入 dynamics/cone/interaction/output | done |
| P46-R22 / EXACT-R1-COMP | exact image component gate | rank/SVD、symmetry/idempotence、containment、principal angle、reconstruction、missing direction、historical tests | done |
| P46-R23 / EXACT-R1-EQ | compatible-H0 tick0 equilibrium | COMP PASS 后进入；actual right `ddxi=-0.0752634`，mandatory FAIL | done |
| P46-R24 | causal evidence/formal replay/review | before/after closure、fresh replay；不分类 R2，不授权第二 repair | done |

### REWORK tasks — post-exact-R1 first-mismatch attribution

只比较 Phase45 compatible-H0 与 exact-R1 candidate 的 frozen tick0；不实施 repair，不运行
AUTH/REAL/trajectory。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R25 | strict state/regime parity | q/qdot、M/bias、reduction、J/Jdot、xi、contact topology/signature frozen-compatible | done |
| P46-R26 | QP/plant causal decomposition | actuator/free、QP contact、actual contact、remaining 与 per-actuator closure | done |
| P46-R27 | same-wrench and point-force gates | exact `G_p` reconstruction PASS；same-wrench reduced-force parity material FAIL | done |
| P46-R28 | classification/formal replay | `C-MAPPING-OR-REFERENCE-REGRESSION`；replay `0`；R2 not authorized | done |

### REWORK tasks — wrench/generalized-force operator identity

只做 compatible-H0/tick0 frozen algebra；不修改 projector、reference、controller 或参数。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R29 | independently rebuild `Gp/Jp/Aw` | actual point Jacobians、production map、full/reduced operators、frame/order/sign contract | done |
| P46-R30 | basis/DOF/virtual-work audit | six projected wrench columns、DOF blocks、deterministic virtual work | done |
| P46-R31 | reference transport and old-audit reconciliation | raw parity FAIL；transported parity machine-level PASS；old audit narrow-scope | done |
| P46-R32 | authority/formal replay | `C-REFERENCE-POINT-MISMATCH`；exact R1 no longer closed at production reference；replay `0` | done |

### REWORK tasks — production-reference point-force image

仅将 frozen actual two-point image transport 到 production wrench reference；不实施 repair。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R33 | construct `Gp_prod = Tw Gp_point` | frame/order/sign、reference offset、wrench/twist dual transport | done |
| P46-R34 | close transported operators | full/reduced identity 与 deterministic virtual work machine-level PASS | done |
| P46-R35 | construct and compare `Pg_prod` | rank/missing direction/projector/reconstruction；current projector comparison | done |
| P46-R36 | formal replay and authority | `A-PRODUCTION-REFERENCE-IMAGE-CLOSED`；replay `0`；不实施 candidate | done |

### REWORK tasks — corrected production-reference exact-R1 repair

继续使用唯一 `kPhase46PointRealizableRolling` profile，只将 frozen compatible-H0 的 actual
two-point image transport 到 production aggregate-wrench reference，并以
`Pg_prod=Gp_prod Gp_prod^dagger` supersede 旧 projector。执行顺序严格为 `IMPLEMENT -> COMP -> EQ
-> STOP`；不运行 AUTH、REAL、SHORT、10 s、trajectory 或 R2 classification。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R37 | corrected projector implementation | 完整 frozen contact-line offset；同一 projector 进入 dynamics/cone/interaction/output | done |
| P46-R38 / DG46PR-COMP | production-reference component gate | controller identity、rank-5 image、full/reduced operator、point-force、semantics 与 regression PASS | done |
| P46-R39 / DG46PR-EQ | compatible-H0 tick0 equilibrium | COMP PASS 后进入；actual `ddxi=[-0.0193391,-0.0491110]`，全部 frozen EQ gates PASS；随后停止 | done |
| P46-R40 | formal replay and review | formal-v2 authoritative、fresh replay error `0`；formal-v1 harness false failure rejected；R2 not authorized | done |

### REWORK tasks — corrected production-reference exact-R1 AUTH

只执行 compatible-H0、tick0 的 fixed-state directional AUTH。全部导数严格使用
`(probe - baseline) / signed_delta`；不修改 controller/projector/task/参数，不进入 REAL、SHORT、
10 s、trajectory 或 R2。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R41 | common/differential directional probes | xi/slip 独立输入，`+/-`、scales `1/0.5/0.25`，baseline-subtracted QP/MJ 2x2 transfer | done |
| P46-R42 | per-probe R1/regime closure | projector/range/point-force/full+reduced operator closure；contact/frame/friction/active-set/solver stability | done |
| P46-R43 / DG46PR-AUTH | frozen authority gates | `B-HARMFUL-CROSS-REMAINS`：cross `-0.118040` 超 absolute gate；slip self 同时反号 | done |
| P46-R44 | formal replay and review | 24 probes + baseline，fresh replay error `0`；严格停在 AUTH | done |

### REWORK tasks — post-corrected-R1 fixed-state authority attribution

只分解 corrected-R1 AUTH 的 slip-common、slip-differential 与最小 xi-common control；不修改
controller/parameter，不实施 repair，不运行 trajectory 层验证。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R45 | common four-output causal balance | free/QP-contact/MJ-contact/other 与 actuator、contact-point closure | done |
| P46-R46 | differential/contamination/healthy-control attribution | common↔differential mechanism 与 xi-common 对照 | done |
| P46-R47 | point-force aggregate/null split and dominance | production-reference closure、alignment/residual/norm metrics、solver interpretation | done |
| P46-R48 | classification and fresh replay | `E-MULTIPLE-REMAINING-MECHANISMS`；replay `0`；R2 not authorized | done |

### REWORK tasks — post-corrected-R1 other-gap closure

只关闭 slip-common `other gap` 的 generalized-force source，不进入 KKT、repair 或 trajectory。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R49 | smooth/constraint split | smooth-minus-actuator numerical zero；material gap 位于 constraint side | done |
| P46-R50 | row-wise constraint reconstruction | equality/contact/limit/friction-loss/other 由 `efc_J.T@efc_force` 闭合 | done |
| P46-R51 | QP-vs-MJ channel semantics | QP reduction equality reaction对比 MuJoCo bilateral leg-closure rows；passive/applied/external/bias zero | done |
| P46-R52 | independence/classification/replay | `D-NONCONTACT-CONSTRAINT-GAP`；independent YES；replay `0`；R2 not authorized | done |

### REWORK tasks — bilateral leg-closure equality-response operator audit

只审计 equality geometry/J/RHS/coupled reaction operator；不修改 equality、reduction 或 solver。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R53 | closure geometry and row-space parity | bilateral site/IDs/rows、raw J exact、rank/containment/nullspace PASS | done |
| P46-R54 | Jdotv and acceleration-target audit | Jdotv exact；MJ stabilization target nonzero但 slip_c influence仅 `1.32%` | done |
| P46-R55 | coupled rigid reaction counterfactual | all equality+contact rows；KKT residual `<=2.54e-14`；QP reaction range/rigid mismatch | done |
| P46-R56 | classification and replay | `D-QP-CONSTRAINED-REDUCTION/REACTION-MISMATCH`；replay `0`；R2 not authorized | done |

### REWORK tasks — constraint-consistent reaction implementation audit

只审计附件冻结的 bilateral leg-closure reaction candidate。门序为
`COMP-A -> COMP-B -> EQ -> AUTH`；首个 mandatory FAIL 后停止。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R57 | runtime reaction evidence contract | component runner 必须读取真实 QP runtime reaction probes，禁止以 rigid oracle 自比较替代 | done |
| P46-R58 / DG46ER-COMP-A | reaction legality gate | runtime probes 缺失，`D-REACTION-SEMANTICS-IMPLEMENTATION-FAIL`；立即停止 | done |
| P46-R59 / DG46ER-COMP-B | coupled-rigid parity | DG46ER-COMP-A FAIL，未运行 | blocked |
| P46-R60 / DG46ER-EQ | compatible-H0 equilibrium | DG46ER-COMP-A FAIL，未运行 | blocked |
| P46-R61 / DG46ER-AUTH | fixed-state authority | DG46ER-COMP-A FAIL，未运行 | blocked |

### REWORK tasks — runtime implementation-status audit

只执行 `IMPLEMENTATION-STATUS -> RUNTIME-PROVENANCE -> COMP-A -> STOP`。Case B 在首关
触发时立即停止，不补 instrumentation，不修改 QP formulation。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R62 | actual runtime branch audit | `R46E-*` profile进入通用 42D solve；candidate-specific profile branches可达 | done |
| P46-R63 / DG46ER-IMPLEMENTATION-STATUS | reaction formulation presence | actual QP 无 `J_eq/lambda_eq/coupled KKT/Schur recovery`；Case B FAIL | done |
| P46-R64 / DG46ER-RUNTIME-PROVENANCE | runtime reaction provenance | implementation-status FAIL，未运行 | blocked |
| P46-R65 / DG46ER-COMP-A | runtime reaction legality | implementation-status FAIL，未运行 | blocked |

### REWORK tasks — reduced-QP/full-constrained-dynamics equivalence audit

本轮只读审计 production reduced QP；不修改 formulation，不实现 `lambda_eq`，不运行
EQ/AUTH/REAL/trajectory/R2。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R66 | runtime/full-tree provenance | actual `R46E-H0` 42D solve；只读记录 `N/c_N/J_eq/JdotV/M_full/h_full/B_full/Aw_full` | done |
| P46-R67 | kinematic and dual equivalence | `rank(N)=12`、`rank(J_eq)=4`；primal/dual projector differences `<=1.75e-15` | done |
| P46-R68 | legal full-dynamics lift | projected residual `4.48e-9`；legal reaction recovery、range与virtual work PASS | done |
| P46-R69 | QP-contact full oracle | exact affine pullback；primal/contact/slack/objective differences `0`；nonunique but equivalent | done |
| P46-R70 | reconciliation/formal replay | classification B；formal-v4、fresh replay `0`、35 tests、non-finite/diff checks | done |

### REWORK tasks — legal equality reaction re-attribution

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R71 | legal production reaction recovery | `Qeq=Pprod*rfull`；range `6.97e-17`、reconstruction `1.08e-16` | done |
| P46-R72 | production/MuJoCo force-space audit | ranks `4/6`；common/prod-only/MJ-only dimensions `4/0/2` | done |
| P46-R73 | corrected-R1 re-decomposition | baseline + 32 probes；all source closures PASS | done |
| P46-R74 | classification and replay | `E-MIXED-REMAINING-MECHANISMS`；replay `0`；R2 NO | done |

### REWORK tasks — smooth/pre-contact first-mismatch attribution

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R75 | remainder bookkeeping | reproduce slip-common `-0.388661935`；closure residual `5.56e-13` | done |
| P46-R76 | torque/actuation/smooth-force gates | torque and `B*tau` gaps `0`；other smooth `<=1.78e-13` | done |
| P46-R77 | raw mass first-mismatch closure | mass relative difference `0.09699`；raw output gap explains target | done |
| P46-R78 | mode attribution and replay | hip signed share `92.884%`；classification C1；replay `0` | done |

### REWORK tasks — closure-conditioned effective-inertia / precontact response attribution

只在同一个合法 rank-4 closure tangent space 比较 production/MuJoCo，并独立量化 MuJoCo native
rank-6 的额外两模态；不实施 repair，不修改模型、controller、closure 或 contact。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R79 | common closure subspace gate | shared orthonormal common4；ranks `4/4/6`、angles/projectors/containment/tangent PASS | done |
| P46-R80 | conditioned/reduced response gate | `K_prod4/K_MJ4/K_MJ6`；production KKT-vs-reduced spectral gap `6.36e-11` | done |
| P46-R81 | three-way response attribution | common4 slip-c保留 raw target `99.928%`；MJ-only贡献 `11.393%` | done |
| P46-R82 | tangent mass/energy/operator audit | tangent mass gap `0.09703`、kinetic energy FAIL、Delta-K SVD、probe controls | done |
| P46-R83 | classification and fresh replay | Primary D；replay `0`；source NOT ATTRIBUTED；R2 NO | done |

### REWORK tasks — common-tangent inertial / kinematic-assembly source attribution

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R84 | body mapping and provenance | 11-body ID/name/parent mapping；normalized inertials与armature provenance | done |
| P46-R85 | independent runtime-M rebuild | production/MJ max errors `<=2.22e-16/1.11e-16` | done |
| P46-R86 | four-combination factorial | target closure `1.1e-14`；I/K/interaction=`2.219%/97.784%/-0.0024%` | done |
| P46-R87 | dominant assembly closure | base control-point vs body-origin Jacobian reference；97.781% closure、energy/Delta-K validation | done |
| P46-R88 | classification and replay | H；source-specific candidate YES；fresh replay `0`；no repair/R2 | done |

### REWORK tasks — base reference semantic canonicalization candidate

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R89 | configuration/frame mapping | real site offset、orientation/joint map、pose + configuration FD parity | done |
| P46-R90 | twist/force/acceleration transform | `X/X^-1/Xdot`、point twist、virtual power、acceleration FD | done |
| P46-R91 | same-model covariance | M/energy/h/full-EOM/J/reduction machine-scale PASS | done |
| P46-R92 | first-consumer and candidate oracle | diagnostic comparison boundary；common4 source closure `97.781%` | done |
| P46-R93 | classification and replay | A；candidate authorized next round；fresh replay `0`；no implementation/R2 | done |

### REWORK tasks — diagnostic-boundary base reference canonicalization implementation

只实施已授权的diagnostic candidate；不修改production controller、QP、state semantics、模型参数、
contact或equality。执行顺序固定为IMPLEMENT → DG46RC-COMP → controller regression → common4
re-attribution → physical-channel re-decomposition → reclassification → STOP。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R94 | diagnostic canonicalization utility | 在cross-model comparison boundary统一configuration/twist reference | done |
| P46-R95 / DG46RC-COMP | covariance and invariance | M/h/Q/J/N/qacc协变与observable不变；最大残差`1.78e-15` | done |
| P46-R96 | controller regression | controller CSV数值差`0`；R1与production reduced QP保持有效 | done |
| P46-R97 | common4 and physical re-attribution | reference gap关闭`97.786%`；physical channels无double count | done |
| P46-R98 | classification and replay | A implemented；replay`0`；contact不unique；R2 NO；STOP | done |

### REWORK tasks — MuJoCo-only closure-model attribution

只解释此前 `native6-common4` 的两个方向及其 material observable counterfactual；不修改 XML、
controller、QP、contact、`solref/solimp` 或 solver。必须区分 raw row magnitude、exact-manifold
rank 与把任意非零 row 正交归一化后的 hard-constraint rank，禁止用 row count 直接宣称 physical
DOF。执行顺序为 `GEOMETRY -> SUBSPACE -> RESPONSE-SEMANTICS -> CLASSIFY -> STOP`。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R99 | exact site-pair and row geometry | 两个 connect、6 raw rows、site residual、row norms 与 singular spectrum | done |
| P46-R100 | common4 / native-only 2D decomposition | native-only basis、Cartesian row coefficients、generalized-force/relative-motion interpretation | done |
| P46-R101 | hard-rank response semantics | weak-row scaling/null limit、conditioned-operator discontinuity、observable contribution closure | done |
| P46-R102 | stabilization/compliance independence | `efc_pos/aref/D/R` 与 weak-row origin；判断是否 independent physical mechanism | done |
| P46-R103 | classification and fresh replay | formal-v1 + replay-v1 `0`；contact unique；R2 candidate YES / authorized NO；STOP | done |

### REWORK tasks — R2 contact-response repair re-authorization

本轮只做 source freeze、native runtime oracle、missing-relation attribution 与 diagnostic controller
counterfactual；不修改 production controller/model/contact/solver。最多授权一个 physical law 的下一轮
implementation，严格停在 authorization decision。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R104 | authoritative source freeze and contact-law inventory | current equations/variables/tasks/R1；fresh contact gap | done |
| P46-R105 | native runtime contact-response oracle | qacc/row force/point force/wrench/observable/generalized-force reconstruction | done |
| P46-R106 | first missing relation attribution | RHS/operator/compliance/friction/geometry/active-set split；no-repackaging gate | done |
| P46-R107 | physical-law candidates and A/B equivalence | coupled primal vs Schur elimination；C/D rejection reasons | done |
| P46-R108 | Stage-S same-tau source validation | force and observable errors；explained contact-gap fraction | done |
| P46-R109 | Stage-R closed-loop diagnostic counterfactual | attempted；H0/feasibility/branch/scale FAIL，结果不可信 | done |
| P46-R110 | R1/H0/healthy-control/computational gates | R1 PASS；H0/feasibility/branch/scale FAIL；无 authorization | done |
| P46-R111 | classification and authorization decision | E；R2 authorized NO；production unchanged；STOP | done |
| P46-R112 | formal, fresh replay and regressions | formal-v1 PASS、replay-v1 `0`、targeted/regression/diff checks | done |

### REWORK tasks — R2 reduced-integration first-mismatch attribution

本轮仅审计 valid plant contact law 到 production 12D reduced WBC 的 first wrong integration
relation；不实施 production repair，不调权重、增益、contact 或 solver。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R113 | exact Stage-R reconstruction | fresh H0/failure/branch/scale reproduction | done |
| P46-R114 | complete H0 physical witness and ordered gates | canonical lift、rank4 conditioning、full/reduced dynamics、R1、row/point/wrench parity | done |
| P46-R115 | H1/H2/H3 first-mismatch attribution | H2 confirmed；H1 rejected；optimizer audit not entered | done |
| P46-R116 | evidence, replay, unchanged-production gate | formal-v2 PASS；replay-v2 `0`；production unchanged | done |

### REWORK tasks — R2 contact-reaction commuting-diagram attribution

本轮只定位 row reaction → point force → production aggregate wrench → generalized force 与
Stage-R affine reaction map 的第一条不交换 edge；不预设 point-force variable，不实施 repair。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R117 | fresh map/provenance reconstruction | contact points/frames/references/Tλp/Gp/Jp/Aw/Jefc | done |
| P46-R118 | corrected-R1 operator and virtual-work gate | full/reduced `AwGp=JpT` machine-level PASS | done |
| P46-R119 | E1/E2/E3 commuting attribution | E1/E2 numerical；historical 4.836644 located at mixed M/P aggregate→Stage-R edge | done |
| P46-R120 | affine offset/slope and directional replay | offset 1.522196；slope 3.314448；corrected replay machine-level PASS | done |
| P46-R121 | sufficiency/null regression and evidence | aggregate dynamics sufficient；eta nonmaterial；formal/replay PASS | done |

### REWORK tasks — Stage-R affine reaction-map provenance/reference attribution

本轮只审计 diagnostic `Qc0/Qct` producer/consumer、affine origin 与 M/P force-dual boundary；
不修改 production QP，不实施 contact law。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R122 | consumer and production provenance | diagnostic-only producer/consumer；production equivalent law absent | done |
| P46-R123 | affine origin and force-dual covariance | tau=0 origin；Qc0/Qct column covariance PASS | done |
| P46-R124 | historical mixed-reference reconstruction | offset 1.522196 + slope 3.314448 → 4.836644 | done |
| P46-R125 | diagnostic-only corrected H0 replay | map 1.98e-14；violation 0；H0 PASS | done |
| P46-R126 | minimal directional trust and evidence | branch/scale machine-level PASS；formal/replay PASS | done |

### REWORK tasks — production contact-response integration attribution

本轮只审计 production 42D QP 的首个缺失 contact-response relation，以及 diagnostic
`Qc=Qc0+Qct*tau` 的 R0/R1/R2/R3、P0/P1/P2/P3 与合法插入形式；修正后的 diagnostic Stage-R
PASS 保持冻结，不实施 production R2。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R127 | production equation inventory | 42D variables/hard dynamics/cones/soft tasks/R1；first missing relation | done |
| P46-R128 | diagnostic affine-law provenance | R1 Schur closed response；MuJoCo internal dependencies and exactness domain | done |
| P46-R129 | double-counting, causality and insertion audit | identical-primitives algebraic equivalence；P2 stop；I4 | done |
| P46-R130 | classification and append-only evidence | B；shadow/prediction/residual gates NOT ENTERED；R2 remains NO | done |

### REWORK tasks — MuJoCo-dependent simulation-only hard R2

> **WARNING — MUJOCO-DEPENDENT SIMULATION-ONLY R2.** This profile uses current-state MuJoCo
> internals only for simulation closure. It is not hardware-ready and must be replaced before real-robot
> deployment.

本轮允许 one hard simulation-only profile；strict order 在首个 mandatory FAIL 停止，不允许 soft
fallback、tuning、future-response feedback 或 active-set inner iteration。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R131 | same-snapshot oracle and partition | pre-command provenance PASS；future leakage NO；partition/oracle machine-level PASS | done |
| P46-R132 | full/reduced legality and rank | full illegal；reduced legal；rank 12→19，incremental independent rank 7 | done |
| P46-R133 | image and active-set gates | R1 image residual `3.55e-15`；pre-solve row-force sign/regime PASS | done |
| P46-R134 | one hard simulation profile | per-tick builder + `kPhase46MujocoContactResponse`；default profile unchanged | done |
| P46-R135 | COMP and strict stop | H0 `PrimalInfeasible`；COMP FAIL；EQ/AUTH/REAL/SHORT/10 s NOT ENTERED | done |
| P46-R136 | evidence, replay and regressions | formal/replay equal；targeted tests PASS；one unrelated timing test passed on rerun | done |

## Classification

最终只能选择：

- `P46-A — static hip-common projection sufficient`
- `P46-B — cross-coupling reduced but insufficient`
- `P46-C — harmful mode migrated to another DOF/mode`
- `P46-D — slip authority destroyed by projection`
- `P46-E — multiple remaining mechanisms`
- `P46-U — evidence unreliable`

若 mandatory gate FAIL，保持 Phase46 `review/REWORK`，不创建 RECORD，也不自动进入 soft penalty、
coupled task、precompensation 或 dynamic projection。

## REWORK addendum — primitive contact-law integration audit

P46-R137 replaces the rejected closed `Qc0+Qct*tau` hard payload with contact rows only:
`fc=Dc(aref_c-Jc(N*nudot+cN))`, followed by row reaction → Cartesian point force → per-wheel
production-reference aggregate wrench and the frozen two rank-5 image bases. The core payload is a
generic hard row over `[nudot,W_L,W_R]`; `tau` has no direct coefficient. Entry order remains
`W1→W2→W3→W4→W5→W6→42D witness→COMP` and stops at the first failure.

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R137 | primitive-law hard-row implementation | per-tick contact rows；rank-10 compressed hard payload | done |
| P46-R138 | W5 three-route closure | `K_A/K_B/K_C` static parity；historical `7.65679/7.20092` reproduced；fixed at machine scale | done |
| P46-R139 | 42D witness and COMP | solver/hard/cone/torque/R1/primitive-law witness PASS；COMP PASS | done |
| P46-R140 | EQ and ordered stop | normalized slack `0.0585037 > 0.05`；EQ FAIL；SHORT/10 s NOT ENTERED | done |
| P46-R141 | formal replay and regressions | fresh replay equal；targeted tests、canonicalization self-check、diff check PASS | done |

### REWORK tasks — primitive-R2 wrench-slack closure

本轮不调 weight/gain/threshold/contact/solver；先执行 frozen-H0 request-feasibility mandatory gate，
若 infeasible 则在 soft/KKT/ablation 前停止。

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R142 | fresh slack reproduce and semantic decode | `0.0585037086778` reproduced；12D order/frame/origin/sign/scale and exact reconstruction | done |
| P46-R143 | baseline/R2 component decomposition | point-realizable baseline `0.00152222039539`；dominant right `Tx` | done |
| P46-R144 | direct fixed-H0 request feasibility | 22 hard equalities；full rank-12 wrench equality infeasible；minimum deviation `0.0783204306734` | done |
| P46-R145 | mandatory classification and stop | `A-WRENCH-REFERENCE-NOT-PRIMITIVE-FEASIBLE`；soft/KKT/ablation/repair NOT ENTERED | done |
| P46-R146 | append-only formal/replay evidence | fresh formal/replay decision byte-identical | done |

## REWORK — Frozen Nominal, Limited Increment

本次 REWORK 不新开 Phase。Phase46 static row projection 已可靠地在 EQ FAIL，故唯一允许的替代是：
zero slip perturbation 继续走未投影的 Phase45 rolling QP；仅在 frozen tick0 `slip-common` external
task delta 非零时，追加一个无权重的 hard equality
`0.5*(nudot_left_hip+nudot_right_hip)= -0.009961062735978504 rad/s2`。
该右端是已冻结 compatible-H0 nominal QP hip-common 值，不是 xi/slip target offset、gain/weight
调整或 cross-gain precompensation；它只令 requested slip perturbation 的 QP hip-common increment 为零。
不运行 REAL/SHORT/10 s。本轮 gate 仅为 `EQ -> Incremental AUTH -> fresh replay`。
