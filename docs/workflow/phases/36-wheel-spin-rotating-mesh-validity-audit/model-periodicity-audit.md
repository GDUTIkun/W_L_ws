# Model periodicity audit

For 15 pre-frozen pairs (three modes × five base phases), the following maximum `q` versus
`q+2π` error is `3.469446951953614e-18`:

- wheel center, xi, zeta;
- wheel-origin Jacobian and reduced `A_xi`;
- full/reduced mass matrix;
- full/reduced bias;
- closure Jacobian.

This is far below the frozen `1e-8` absolute/relative gate. It establishes exact numerical
periodicity of the rigid-body/core model beyond `±1 rad`; there is no full-body kinematic,
mass, bias, closure, or acceleration-map validity loss at one radian.

Raw collision contacts do not share that robust equivalence: some `q/q+2π` pairs select a different
contact topology, and the corresponding instantaneous response changes. That is not attributed to
the WBC analytic model because (1) its core quantities remain periodic and (2) disabling only
MuJoCo contact nearly removes the phase response. QP solution, realized wrench and QP residual are
recorded as unavailable outside the gate; the live validator was not bypassed.

DG36-02: **PASS** for core-model periodicity. Raw mesh-contact periodic response is explicitly FAIL
and carried into P36-D rather than hidden inside the core result.
