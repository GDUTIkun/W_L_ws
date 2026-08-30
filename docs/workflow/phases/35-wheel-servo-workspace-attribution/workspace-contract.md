# Phase 35 workspace contract audit

## Live authority

An exhaustive literal search found one production `kOutsideWorkspace` assignment: the initial
canonical-joint loop in `NominalWbcModel::evaluate`. The loop runs before passive reconstruction and
now consumes `inspectWorkspace(state)`, which evaluates the same six strict comparisons and preserves
equality as admissible. The inspector test covers every canonical upper equality and `+1e-9 rad`
rejection, plus first-loop-index precedence.

Canonical order is `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`. For joint `j`,
`q_eq=kCanonicalOffset-kEquilibriumActiveNative`, `delta=q-q_eq`, and the hip/knee/wheel intervals are
`[-0.65,0.65]`, `[-0.75,0.75]`, and `[-1,1] rad`. The bounds are emitted by
`tools/experiments/export_weighted_wbc_runtime_profile.py` from the Phase21/Phase15 validation
profile; they are a validated runtime-model domain, not mechanical limits.

CBM Verify-tier discovery used project `W_L_ws`, generation `2026-08-29T06:47:42Z`, then the changed
header/source/test and both Phase34/35 runners were read directly. Coverage reported no recorded
issue on the exact WBC paths but stale metadata after this change; unrelated generated acados and
adapter parse gaps do not intersect this gate.

DG35-00: **PASS**.
