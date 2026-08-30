# Phase 35 state-envelope contract

Frozen method: `simulation/mujoco/config/phase35_workspace_attribution_v1.json`.

Each 10 ms sample records the live six-joint inspector, canonical and raw native state, raw wheel
mesh phase, independent MuJoCo wheel-origin xyz/velocity, xi/zeta channels, base quaternion and
rotation vector, requested/realized wrench and slack, torque margins, contact/load and QP metrics.
The rejecting sample is written before stopping. Undefined post-rejection model/QP values remain
NaN; raw MuJoCo and live-inspector fields remain finite.

formal-v1 established the attribution but lacked gate-independent rejecting-tick wheel geometry.
It is retained. formal-v2 adds those fields, records `supersedes=workspace-attribution-formal-v1`,
and is authoritative. All rejecting envelopes and raw geometries are complete, and live status ↔
inspector parity passes for every row.

Trend semantics are the predeclared five-tick, four-decrease, `1e-3` normalized-loss rule. Near
boundary is normalized margin `<=0.05`; precedence requires two ticks. DG35-01: **PASS**.
