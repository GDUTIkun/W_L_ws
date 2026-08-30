# Phase 37 PLAN — Axisymmetric Wheel Collision Correction + Causal Revalidation

状态：`review`  
日期：2026-08-30

## 目标与范围

用最小 collision-only revision 将两轮的接触 geom 从 rotating CAD mesh 改为与 hinge axis
同轴的 cylinder，验证 P36-D 是否消失，并按 gate 顺序重放 Phase36、Phase32、Phase35 H0。
不修改原 nominal model、wheel rigid-body parameters、contact coefficients、controller、NMPC、WBC、
gain 或 `±1 rad` live gate。Phase34 tracking 只有得到独立 workspace-contract correction 后才可重开。

## Frozen decisions

- 新 revision：`wheel_leg_axisymmetric_collision_v1.xml`；原 mesh 保留为 visual + mass/inertia
  authority，`contype=0, conaffinity=0`；新增 cylinder 是唯一 wheel-ground collision geom。
- cylinder body-frame axis 为 local Z，与 wheel hinge `axis="0 0 1"` 一致；radius `0.05 m`，
  half-width `0.02 m`，轴向中心取原 compiled mesh bbox 中点 `left +0.0075/right -0.0075 m`。
- cylinder `mass=0`，原 visual mesh 保留 `mass=0.3431`，从而 compiled body mass/COM/inertia
  不发生变化；friction/solref/solimp 继承同一 default。
- frozen corpus、thresholds 和 source authorities 位于
  `phase37_axisymmetric_collision_v1.json`；Phase32 原 gates 不变。
- DG37-00~03 任一失败即停止，不运行 Phase32/H0；Phase32 完成后才能运行 H0。
- H0 只记录 gate crossing 与漂移，不绕过 gate。Phase34 tracking 本 Phase 不因 cylinder 修改自动授权。

## Tasks and gates

| ID | Task | Acceptance |
| --- | --- | --- |
| P37-T01 | 创建 collision-only model revision | complete |
| P37-T02 | compiled model parity | complete — DG37-00 PASS |
| P37-T03 | static/periodic/contact ON-OFF audit | complete — DG37-01/02 PASS, DG37-03 FAIL |
| P37-T04 | replay Phase32 x16 oracle | blocked by frozen DG37-03 prerequisite |
| P37-T05 | replay Phase35 H0 twice | blocked by frozen DG37-03 prerequisite |
| P37-T06 | workspace reassessment and REVIEW | complete — REVIEW=REWORK |

## Validation

Use `./.venv/bin/python`. Probe MuJoCo/NumPy/SciPy, `py_compile`, then run the Phase37 causal
runner into a new append-only directory and fresh replay. ROS source is unchanged; the existing
Phase35 executable accepts a model path and is reused. If rebuild is needed it must run from `ros_ws/`.

## Stop conditions

Parity failure → P37-E. Persistent primitive phase sensitivity → P37-D. Missing/invalid Phase32 or H0
authority → P37-U. No downstream gate may be skipped to obtain a controller result.
