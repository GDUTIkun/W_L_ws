# Task Reference to QP Realization Audit

Evidence: [`task-reference-qp.csv`](evidence/automated/realization-audit-formal-v1/task-reference-qp.csv)

At the shared tick0 state, all enabled B/C/D wheel rows are realized essentially exactly inside the QP:

- B native rows: maximum normalized residual `3.70e-9`.
- C xi rows: maximum normalized residual `2.46e-10`.
- D xi/native rows: maximum normalized residual `3.96e-10` / `1.65e-10`.

Therefore Phase43's tick0 native-wheel acceleration failure is not an optimization failure. On later common/own
snapshots, task competition/limits become material: the maximum enabled normalized residual reaches `18.7863`
(native scale `20 rad/s^2`) and the xi raw/normalized residual reaches `2.5 m/s^2`. QP loss is therefore a
later-trajectory layer, not the tick0 root layer.

The evidence table also records per-row raw/normalized/squared cost, gradient norm, contact/wrench/slack task
costs, torque/contact/acceleration active-inequality counts and margins. Minimal profiles have no base task;
base-task attribution is explicitly `unavailable_in_minimal_profile`. Solver multipliers/basic active-set data are
unavailable and were not inferred.
