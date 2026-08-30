# Phase 35 reproduced-baseline task comparison

The reproduced baseline defines xi as wheel-center position relative to the base geometry, explicitly
not wheel spin (`docs/models/simulink_mpc_wm_wbc_baseline.md`, state reconstruction section). Its
`wheel_position_lqr_reference.m` produces an Eq.(21) desired xi position, and
`spatial_two_leg_qp_core.m` forms Eq.(12) xi-acceleration feedforward plus common/differential xi PD.
The baseline also contains a common rolling-speed task, which regulates velocity rather than an
absolute periodic wheel angle.

Phase35's exact live trigger is canonical wheel joint angle / rotating-mesh phase. No inspected paper
or reproduced-baseline task states that absolute wheel spin angle is regulated or bounded at
`±1 rad`. Therefore there is **no justified paper-task mapping** for the limiting coordinate. The xi
task may influence the trajectory but is neither the live condition nor the necessary activation
branch. DG35-07 comparison: **PASS (no supported mapping)**.
