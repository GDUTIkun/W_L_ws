# P21-T07 Runtime Workspace Gate Repair

Date: 2026-08-28
Verdict: **PASS — Phase 15 workspace unchanged; nominal and rejection authority are now consistent**

## Repair decision

The earlier runtime audit found that `dynamic_tick_271` was simultaneously treated as a
nominal 42D parity case and required to fail closed by the Phase 15 reconstruction
workspace. The repair does not expand that workspace. It supersedes the original corpus
with `phase21_hard_qp_42d_runtime_v2` and classifies plant states before consulting QP
feasibility or objective values.

The frozen selection range is capture ticks 1–271. Exactly 259 ticks, 1–259, satisfy the
componentwise canonical joint-delta bounds. The 28 dynamic nominal cases are selected by
nearest-index, half-up rounding over that complete eligible list:

`[1,11,20,30,39,49,58,68,77,87,97,106,116,125,135,144,154,163,173,183,192,202,211,221,230,240,249,259]`.

Ticks 260 and 271 are frozen rejection cases and both produce `outside_workspace`. The four
declared model workspace samples remain nominal, giving the same 32-case count without
using solver outcomes to choose states.

## Re-executed gates

- `2026-08-28-phase21-hard-qp-42d-runtime-v2`: all eleven mathematics, selection and
  rejection gates PASS; 42 variables, 104 rows and 32 nominal cases.
- `2026-08-28-phase21-hard-layers-42d-runtime-v2`: all four cumulative hard layers and the
  116-row zero-acceleration equilibrium PASS. Minimum cone margin is `0.3101016421`; minimum
  torque margin is `1.998539395 N·m`.
- `2026-08-28-phase21-tasks-42d-runtime-v2` and
  `2026-08-28-phase21-task-competition-42d-runtime-v2`: local task algebra, hard-row
  separation, directional attribution, objective accounting and wrench/slack sign all PASS.
- All four tuning and all nine predeclared holdout runs were rerun with a pre-QP workspace
  check. Every one of 13 cases PASS with zero workspace failures and zero workspace
  violation. Worst normalized slack/task residual/task cost remain
  `0.003535101/0.005452469/0.0001501772`, below the unchanged frozen gates.

The original v1 corpus and its documents remain historical evidence but no longer authorize
runtime nominal parity. In particular, the former statement that tick 271 was a nominal
active-cone case is superseded.
