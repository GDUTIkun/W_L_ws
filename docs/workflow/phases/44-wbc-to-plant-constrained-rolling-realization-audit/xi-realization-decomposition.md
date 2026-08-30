# R43-C Xi Realization Decomposition

Evidence: [`xi-decomposition.csv`](evidence/automated/realization-audit-formal-v1/xi-decomposition.csv)

QP rows use the controller affine map. MuJoCo rows independently identify the native 2x16 xi velocity Jacobian at
the exact state and decompose physical `ddxi` into base 6D, all active/passive leg DOFs, native wheel DOFs and
`Jdot*v`. Every row closes within `1e-10` (DG44-07 PASS).

For C own-trajectory key snapshots, mean absolute contributions are:

- base: `3.73e-10 m/s^2`;
- leg: `2.1131 m/s^2`;
- native wheel: `0`;
- `Jdot*v`: `0.03854 m/s^2`.

At tick0 the leg term is already about `0.03186 m/s^2`; by tick106 it is about `3.9763 m/s^2`, versus
`0.21869 m/s^2` from `Jdot*v`. C is therefore Case C2: its xi improvement is realized through leg/wheel-center
reconfiguration, not wheel spin. The simultaneous base-rotation failure is a full-body/contact consequence of this
coordinate choice, not evidence that native wheel qdot is irrelevant.
