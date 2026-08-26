# Phase 19 exact-planar state/sign validation

Result: `PASS`

The `base_control_frame` state evaluator was checked against MuJoCo site Jacobians and centered finite differences at the compliant equilibrium. Maximum site-X Jacobian error was `1.0249023851827133e-12`; pitch Jacobian error was `0`. Positive base-X and `+Y` pitch derivatives were both `+1`, and velocity oracles returned the injected `+0.37 m/s` and `-0.29 rad/s` exactly.

For both wheels, positive native rotation produced no-slip displacement `[+0.05, 0, 0] m/rad` within `1e-17`. Adapter relations remain `canonical joint=-native+offset` and `native actuator torque=-canonical torque`; consequently canonical positive wheel rotation/torque has the opposite rolling sign. The existing repository coordinate-contract regression also passed.

Primary and replay `summary.json` hashes were exact: `22c52deb911d5c706543a9bac471296e320bd1e00e0fcf36c95f67cd7dd82c94`.
