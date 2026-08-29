# Phase 31 dynamics attribution and candidate decision

## Valid acceleration and Eq.(12) audit

The valid acceleration oracle is `wheel-acceleration-v4`. It differentiates analytic geometric
`dxi` at five 2 ms substeps with a fourth-order centered stencil. Its second/fourth-order maximum
disagreement is `0.1752 m/s² < 0.5 m/s²`.

At the six T0/T1 authority states, requested and realized Eq.(12) predictions differ by at most
`0.00185 m/s²`, but plant-minus-Eq.(12) reaches `1.903 m/s²`. WBC wrench realization is therefore
not the closed-loop residual source.

## Controlled input response

Authority is `wheel-input-response-v9`, replayed byte-for-byte as `v10`. Every perturbation replays
the complete production WBC prefix, then changes only the target tick. Baseline joint torque is
reproduced exactly; full/half step consistency is within `1.283%`.

- WBC realized-wrench primary sensitivity error: `<=0.398%`;
- WBC realized-wrench cross sensitivity: `<=0.00179%`;
- plant response signs: PASS;
- plant cross coupling: `<=7.365%`, PASS;
- plant primary gain error against Eq.(12): up to `93.40%`, FAIL.

The exact same-state kinematic decomposition closes within `1.57e-9`. Wheel-origin translation is
the dominant term; base translation and base angular acceleration are nonzero, mode-dependent
contributions. This is a full articulated/contact response, not the isolated wheel denominator in
Eq.(12).

## Scalar-inertia rejection and root class

`effective-inertia-audit-v3`, replayed as `v4`, gives:

| Channel-derived denominator | Mean | Within-channel CV |
| --- | ---: | ---: |
| common `Ty` | `0.04730` | `3.06%` |
| differential `Fx` | `0.10531` | `4.22%` |
| differential `Ty` | `0.05931` | `3.13%` |

The current `D_w` is `0.02563`. The inferred channel means differ by a factor of `2.226`; common
`Fx` admits no positive scalar denominator at any authority state. Configuration variation within a
channel is small compared with the contradiction between channels. Consequently:

```text
M3 constant/equivalent inertia: rejected
P31-C configuration-dependent scalar inertia: not supported
M1 measurement: rejected
M2 sign/frame: rejected
M4 missing articulated/contact coupling: supported
M5 unsuitable coordinate: not proven
```

Frozen classification: `P31-E_missing_wheel_kinematic_dynamic_coupling` (`M4`). Six-point
correlations are retained as descriptive evidence only; the causal decision comes from same-state
perturbations and exact acceleration decomposition.

## Candidate gate

Two physically motivated diagnostic candidates were tried and rejected without switching
production:

1. project the Minimal WBC soft-contact generalized acceleration to `ddxi` (`v7`): maximum
   sensitivity error `23.99`;
2. feed the WBC joint torque through a bilateral hard-contact full-model KKT (`v8`): maximum
   sensitivity error `7.91`; common `Fx` predicted about `-2.28` while MuJoCo measured about
   `-0.16`.

Both diagnostic source changes were removed after the failed gates. Production remains the Phase27
Eq.(12) artifact.

The next admissible candidate is a newly derived, contact-regime-aware closed-chain response model
whose independent oracle predicts MuJoCo `ddxi` from the frozen 16-state variables and interaction
wrench. It must derive the rolling/friction constraint and articulated leg response explicitly; it
may not be a fitted `D_eff`, correction factor, cost change, or WBC task add-back. If that response
cannot be a function of the existing 16 states, M5 must be reopened and the state definition revised.
No such model is yet frozen, so one-step, 20–400 ms rollout, T0/T1 return, and production integration
remain blocked.

