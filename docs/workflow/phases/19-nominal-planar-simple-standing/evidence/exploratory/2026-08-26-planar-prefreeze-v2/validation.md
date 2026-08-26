# Phase 19 v2 full-plant pre-freeze validation

Result: `REWORK`

The reset-local four-state model passed its own limited gates: rank `4`, affine drift below `4e-13`, LQR closed-loop spectral radius `0.9847891283`. That result did **not** carry over to the complete sampled plant.

The same equilibrium, support+PD law, common-wheel gain, `2 ms` physics, `10 ms` control and 5-step ZOH were linearized over all `13 qpos + 13 qvel` states without resetting hidden states. The full closed-loop spectral radius was `1.7671456993`, with three poles outside the unit circle. This blocks DG19-05.

All five nonlinear holdouts failed the frozen gate, including nominal. The `±1e-5 rad` pitch cases reached more than `0.5 rad`; bilateral wheel contact fraction fell below `0.53`. This is evidence that the candidate/controller abstraction is invalid for the current full planar contact/equality dynamics, not proof that every possible controller is impossible.

Primary and fresh-process replay `summary.json` hashes were exact: `98c3acc82073c280931f09d75e12d2ffb61add768d37da374090509fadca6a8b`. Both runs intentionally exited `1`. No Core code or hardware was touched.
