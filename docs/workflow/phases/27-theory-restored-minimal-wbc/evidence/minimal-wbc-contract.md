# Phase 27 Minimal WBC contract

Date: 2026-08-29

Decision: `T07 PASS`

The opt-in `kPhase27Minimal` profile retains the existing 42-variable order,
104 hard rows, 12 dynamics equalities, torque/acceleration/contact bounds and
ProxQP backend. Its soft objective contains exactly:

1. two-side soft contact acceleration;
2. full 12D realized internal interaction-wrench fidelity with signed slack,
   plus the separate slack penalty;
3. `1e-6` normalized regularization on `nudot`, torque and contact wrench.

The slack block is not separately regularized because its explicit penalty is
already positive definite. Base-X, height, orientation and leg tasks are not
assembled. A test changes all four disabled references by large values and
confirms the Minimal Hessian and gradient remain bit-identical.

For side `i`, the implemented residual is

```text
r_i = A_nudot_i nudot + A_contact_i w_C_i + b_i
      - W_request_i - s_i.
```

Thus zero residual has the approved sign
`W_real_i = W_request_i + s_i`. The controller reports requested-independent
realized wrench, signed slack and their residual separately. A second assembly
in the component test reconstructs the normalized contact, affine-wrench,
slack and regularization blocks and matches the production Hessian/gradient
within `2e-12`; hard matrices and bounds match the nominal profile exactly.

The equilibrium plus four independent dynamic model states pass the solver,
hard and finite gates. Torque extraction matches decision indices 12–17.
Three hundred changing interaction requests give `p99=0.749 ms` and
`max=0.939 ms` under the frozen 10 ms combined deadline. The full Release
Core/ROS/MuJoCo regression is 31 tests with zero errors or failures. These are
component claims only; runtime scheduling and plant behavior remain T08/T10.
