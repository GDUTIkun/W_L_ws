# Root-cause closure

Torque mechanism: `T2-ACCELERATION_TASK_COUPLING_DOMINANT`.

First mismatch verdict: `R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH`.

The xi/rolling objectives are the true KKT excitation and explain why torque is large, but that is a generation mechanism rather than the first proved mismatch. The first proved mismatch is `R1`: the QP uses a material lateral-axis moment outside the actual rank-5 point-force image, and this unrealizable component supplies approximately all predicted rolling cancellation.

Actual `Fr` is the normal constrained reaction to the already-present rolling free acceleration, not a solver-created Fn→Fr conversion.
