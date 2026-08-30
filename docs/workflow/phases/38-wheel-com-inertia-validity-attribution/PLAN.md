# Phase 38 PLAN — Wheel COM / Inertia Validity Attribution

状态：`complete`  
日期：2026-08-30

## 审核结论与唯一目标

附件方案获批，但收紧 counterfactual 数学定义后执行。本 Phase 只归因 Phase37 cylinder
plant 剩余的 within-cycle phase sensitivity，不以 controller 结果优化模型，不运行
Phase32、Phase35 H0 或 Phase34 tracking。

## Frozen semantics and decisions

- MuJoCo compiled `body_ipos` 是 body-frame 表达的 COM；`body_inertia` 是关于 COM 的 principal
  moments；`body_iquat` 将 principal inertial frame 定向到 body frame。完整 body-frame tensor
  为 `R diag(Iprincipal) Rᵀ`。hinge point 是 body origin，axis 是 body local Z。
- V0 是 Phase37 authority。V1 只令 COM body-X/Y 为零，保持 axial COM、mass、COM-centered
  principal inertia 与 inertia orientation 不变。V2 保持 COM/mass，将 body/axle-frame tensor
  改为 `diag((Ixx+Iyy)/2,(Ixx+Iyy)/2,Izz)`。V3 同时应用 V1/V2。
- V1–V3 只在 fresh compiled `MjModel` 内存副本上应用，并调用 `mj_setConst`；config 中规则和
  evidence manifest 是 append-only model-variant authority。不会写回任何 nominal/Phase37 XML。
- COM 显著阈值 `0.05 mm`；transverse anisotropy/products 显著阈值 `1e-4`；primary causal
  reduction 要求 candidate/baseline `≤0.2`，且至少在 rigid-body quantity 与 response 两类量一致。
- V4 只有 L/R unexplained mismatch `>5%` 才可生成；否则禁止 opportunistic homogenization。
- STL 是现有唯一 wheel geometry source。没有 SolidWorks/STEP mass-property report 时，只能判
  numerical causality/plausibility，不能宣称 counterfactual 是真实物理修正。

## Tasks

| ID | Task | Acceptance |
| --- | --- | --- |
| P38-T01 | source/frame/COM/inertia semantics audit | complete |
| P38-T02 | axle COM/tensor and analytic plausibility | complete |
| P38-T03 | materialize V0–V3 diagnostic variants | complete |
| P38-T04 | contact-off phase isolation | complete |
| P38-T05 | contact-on amplification | complete |
| P38-T06 | classify physical vs modeling validity | complete |
| P38-T07 | fresh replay, REVIEW and conditional RECORD | complete |

## Verification

Use `./.venv/bin/python`; dependency probe and `py_compile` precede stable evidence. One runner reuses
the Phase36/37 state/contact machinery and records every variant's compiled inertial arrays and hashes.

## Stop conditions

Unknown transform semantics, nonpositive counterfactual inertia, contact-geometry regression, replay
mismatch, or failure to obtain a multi-observable attribution yields REWORK/P38-U. A numerically clean
V1/V2/V3 does not authorize production adoption without physical source evidence.
