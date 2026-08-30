# Wheel absolute-angle representation contract

## Verdict

Primary recommendation: **R3 — raw unwrapped plant q plus periodic physical validation**.

```text
q_wheel: finite, continuous accumulated cyclic hinge coordinate
dq_wheel: independent finite wheel-rate measurement/state
physical orientation/model: periodic modulo 2π
absolute |q|: not a nominal model-validity condition
accumulated travel: separate revolution count only if a consumer actually needs it
xi: wheel-origin relative translation; never inferred from q_wheel alone
```

R0 is physically valid and is the storage half of R3. R1 is rejected as a shared raw state because
ordinary subtraction jumps at ±π. R2 is physically valid and available as a future numeric
mitigation, but the engineering horizon does not require it and today’s state/protocol has no
revolution-count field.

Classifications:

- `P40-A_absolute_angle_is_valid_unbounded_coordinate`
- `P40-F_current_plus_minus_1_rad_bound_is_unsupported_contract`
- `P40-G_post_bound_rollout_reveals_independent_real_failure`

The P40-G failure is contact loss in the uncontrolled shadow H0, not an angle-representation
failure and not evidence that wheel-spin drift is solved. Real hardware limit authority remains
unknown; `P40-E` is not established.
