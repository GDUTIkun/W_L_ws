# 12D Base Model and OCP

DG34-01 and DG34-02: **PASS for the offline candidate**.

The append-only model is the first twelve Phase27 equations, with the wheel-origin longitudinal
coordinates supplied only as two ZOH geometry parameters. It contains neither the four wheel
derivative rows nor Eq.(12). Independent model formal-v1 passed with maximum next-state error
`6.0694e-9`, state/input Jacobian errors `5.5777e-9 / 5.9128e-9`, and xi-parameter Jacobian error
`1.9125e-7`.

The generated RTI and converged-SQP artifacts passed offline formal-v3. RTI maximum time was
`1.592 ms`, defect/feasibility `4.2817e-4`, and independently projected stationarity `0.03014`.
Converged SQP feasibility/stationarity were `2.4911e-10 / 9.6195e-10`; cold reset was deterministic.

Formal-v1 mixed unrelated randomized problems into a warm lifecycle. Formal-v2 added a non-frozen
native-stationarity gate. Both classifications are invalidated; their raw outputs remain append-only.
Formal-v3 applies the T02-frozen projected-stationarity gate and is authoritative.

No Phase34 runtime C++ solver selection or ControllerCore integration was added because DG34-04
later failed. Passing these offline gates therefore does not approve a production solver path.
