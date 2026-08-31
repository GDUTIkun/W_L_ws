# Root-cause closure

Torque mechanism: `T2-ACCELERATION_TASK_COUPLING_DOMINANT`.

First mismatch verdict: `R4-MULTIPLE_MATERIAL_MISMATCH`.

Ordered causality: (1) `R3-QP_TASK_FORMULATION_MISMATCH`—xi/rolling objectives plus soft contact structure generate the bilateral acceleration/torque mode; (2) `R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH`—the resulting aggregate wrench also contains a material rank-deficient point-unrealizable moment.

Actual `Fr` is the normal constrained reaction to the already-present rolling free acceleration, not a solver-created Fn→Fr conversion.
