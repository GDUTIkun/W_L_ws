# Phase 40 REVIEW

Verdict: **PASS**  
Date: 2026-08-30

All blocking gates pass. Formal/replay authority is `angle-domain-formal-v2` and
`angle-domain-replay-v2`; their summary SHA-256 is identical:
`2d2adba4341172fcca9106263f85397665a9235ae01cdc9eea8209400b06ed35`.

## Findings

1. **Is wheel absolute angle physically bounded?** Model B/current nominal software: no; it is a
   cyclic hinge coordinate. Experimental C620 supports multi-turn. Real assembled hardware: unknown,
   because cable/stop/encoder authority is not yet established.
2. **Does collision/rigid-body behavior require |q|<1?** No. No transition exists at ±1; engineering
   periodic sweep passes.
3. **Does large unwrapped q materially degrade numerics?** Not through ±1e6 revolutions. Maximum
   normalized error is `5.4883e-10`. First material diagnostic rotation error occurs at `5e7`
   revolutions, far outside the engineering horizon.
4. **Is q/q+2πk equivalence preserved?** Yes through the complete mandatory/engineering corpus for
   left, right and bilateral shifts.
5. **Is raw wrapping safe for every consumer?** No. Physical evaluation is periodic, but raw
   workspace and generic position residuals jump by `6.283183307 rad` at the π seam.
6. **Is recentering needed?** No for the frozen engineering horizon. R2 parity passes and remains a
   future mitigation if a named lifetime/precision requirement reaches tens of millions of revs.
7. **Does a real mechanical/sensor finite limit exist?** Not established. Repository absence cannot
   prove real hardware absence, so P40-E is not claimed.
8. **Is the existing [-1,+1] gate justified?** No for nominal model/software: `P40-F`.
9. **Replacement contract?** R3: finite raw unwrapped q, independent finite dq, periodic physical
   validation; retain leg/contact/model/solver/safety gates and separate accumulated count only if
   required.
10. **What happens after the H0 historical crossing?** The diagnostic path continues from tick 96
    through tick 110 without another gate failure.
11. **Does an independent failure appear?** Yes: right-wheel contact loss at tick 111, causing the
    frozen stop (`P40-G`). It is later than and independent of the historical angle gate.
12. **Is Phase34 authorized next?** Not directly. The next Phase must first enact the minimal
    workspace-contract correction and rerun H0. Only its passing gate may reopen frozen Phase34.

## Verification

- dependency probe: MuJoCo 3.7.0, NumPy 2.2.6, SciPy 1.15.3 under `./.venv/bin/python`;
- Python `py_compile`: PASS;
- `colcon build --packages-select wheel_leg_core wheel_leg_mujoco`: PASS from `ros_ws/`;
- targeted ROS tests: 35 tests, 0 errors/failures/skips;
- default gate regression: unchanged tick 96, model status outside-workspace, failed canonical index
  5, right delta `-1.0367881053654626 rad`;
- diagnostic regression: wheel q+2π accepted only with explicit diagnostic policy; default rejects;
  leg violation remains rejected under diagnostic policy;
- formal/replay JSON artifacts and semantic CSVs: identical; shadow replay max error 0;
- JSON parse, Model B XML parse and `git diff --check`: PASS;
- Phase34 tracking run: false; production gate modification: false.

## Classification

`P40-A_absolute_angle_is_valid_unbounded_coordinate +
P40-F_current_plus_minus_1_rad_bound_is_unsupported_contract +
P40-G_post_bound_rollout_reveals_independent_real_failure`.

There are no blocking findings. RECORD is authorized.
