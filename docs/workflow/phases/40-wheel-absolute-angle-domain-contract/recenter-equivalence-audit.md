# Recenter equivalence audit

For every frozen corpus state, Phase40 applied the exact inverse integer shift `q_r=q_u-2πk` while
leaving qvel, base, legs, torque and contact setup unchanged. It then compared geometry/contact,
M/bias/J, closure, qacc, ddxi, wheel qacc and load with the modulo authority.

All mandatory and engineering cases pass the `1e-8` physical gate, preserve exact contact topology,
remain finite, and leave dq unchanged. The stored `raw_reconstruction_error_rad` reports only the
floating-point subtraction remainder; there is no physical impulse because no velocity or dynamic
state is changed.

R2 is therefore physically valid provided the integer revolution count/wrap epoch is updated
atomically and transported separately. It is not selected now: the current schema has no such
field, raw unwrapped R3 passes the engineering horizon, and unnecessary recenter events would add
state-management failure modes.
