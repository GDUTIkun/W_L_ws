# QP-to-Plant Acceleration Audit

Evidence: [`qp-vs-mj-qacc.csv`](evidence/automated/realization-audit-formal-v1/qp-vs-mj-qacc.csv)

The frozen prediction is affine:

`qacc_native_pred = N(q) * nudot_QP + c_N(q,qvel)`.

The closed-chain passive bias `c_N` is exported directly by `NominalWbcModel`; no pure `N*nudot` claim is used.
MuJoCo actual reduced acceleration is an `M`-weighted projection used only as a 12D comparison aid, never as a
replacement for the native 16D residual.

At shared tick0, B/C/D QP wheel tasks are nearly exact, yet native errors are consistently about
`-0.095 rad/s^2` left and `-3.087 rad/s^2` right. The right xi acceleration error is about
`-0.0534 m/s^2`. Later snapshots reach much larger mismatch (native wheel maximum `376.585 rad/s^2`, xi
maximum `7.4195 m/s^2`) near the independently stopped Phase43 trajectories.

Whole-vector MuJoCo dynamics closure remains within `1e-8`, so these differences are not an equation-balance or
coordinate-order arithmetic failure. The evidence supports a material controller-to-plant constrained-realization
layer, especially after the trajectories approach their failure gates.
