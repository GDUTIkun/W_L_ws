# Phase46 corrected production-reference exact-R1 repair

## Verdict

`CORRECTED-R1-COMP PASS / CORRECTED_EXACT_R1_EQ_PASS`.

The sole Phase46 repair profile now uses
`Pg_prod = (Tw Gp_point)(Tw Gp_point)^dagger` at the frozen compatible-H0 production aggregate-wrench
reference.  The same physical wrench enters dynamics, the wrench cone, interaction-wrench
realization, controller output, and diagnostics.  No task, gain/weight, friction, solver, contact
parameter, reference, hip-common restriction, inverse map, or precompensation changed.

Controller-projector differences from the audited `Pg_prod` are `8.88e-16` left and `2.11e-15`
right.  Both images are rank 5; symmetry, idempotence, mutual containment, missing-direction
annihilation, point-force reconstruction, and full/reduced generalized-force operators close at
machine precision.  Component and historical regression tests pass.

After COMP passed, the frozen tick0 equilibrium produced actual
`ddxi_L/R = -0.0193390931/-0.0491110277 m/s2`.  Both satisfy the unchanged `0.05 m/s2` gate, while
material tangent acceleration, bilateral contact/load, hard residual, slack, torque margin,
whole-dynamics closure, and contact reconstruction also pass.

The run stops here.  R1 exact closure is verified only at frozen H0; runtime `Gp_prod(q)` is not
claimed.  AUTH, REAL, SHORT, 10 s, trajectory, and R2 were not run or authorized.  The next allowed
action is a separate fixed-state authority audit.

Authoritative evidence:
[formal-v2](evidence/automated/corrected-exact-r1-formal-v2/corrected-exact-r1-comp.json) and
[fresh replay-v1](evidence/automated/corrected-exact-r1-replay-v1/summary.json), replay error `0`.
Formal-v1 is rejected because its harness included the unchanged numerical regularizer in the
physical interaction-task null check; it stopped before EQ and is retained append-only.
