# Phase 32 closure decision and augmentation

## Decision

`P32-C_16state_markov_closure_failure / M5` is proven. The causal hidden families are all present:

| Family | Same-x16 maximum symmetric `ddxi` difference | Consistency |
| --- | ---: | ---: |
| C1 leg configuration / soft penetration (`P32-D`) | `0.10833 m/s²` | `0.00414` |
| C2 leg velocity / normal-motion (`P32-E`) | `2.10066 m/s²` | `3.79e-6` |
| C3 wheel spin / rolling slip (`P32-F`) | `1.67536 m/s²` | `8.19e-7` |
| wheel mesh angle / discrete patch (`P32-F`) | min `1.14150`, max `1.32688 m/s²` | hybrid replay gate |

All smooth-pair projection, oracle, bilateral-contact, finite and full/half gates pass. Requested
wrench is identical and WBC algebraic realized-wrench relative differences remain below `2.7e-6`;
fixed-torque responses retain the mechanisms. A direct physical joint-wrench parity claim is not
made. Formal/replay semantic files are byte-identical.

## Consequences

Phase31's M4-only conclusion is superseded. No function
`ddxi=f(x16,W_requested)` exists for the current floating-base mesh-contact plant. A scalar inertia,
16-state constrained response, cost change or solver change cannot repair this.

The minimum evidence-backed observable candidate is x24 from
`full-to-reduced-state-contract.md`. It is not yet proven sufficient because the current mesh contact
plant is hybrid and Phase21 already found mesh-vertex support selection non-differentiable. The
historical analytic continuous six-point representation is differentiable, but its static reduced
dynamics previously passed only `122/173` states; that old failure cannot be promoted to authority.

## Next admissible direction

Before a new NMPC artifact, freeze one contact authority:

1. **Recommended:** validate an analytic round-wheel collision/contact plant against the mesh plant
   and real wheel geometry, separating mesh tessellation phase from physical tire dynamics. If
   approved, re-run closure; wheel angle may drop and x22 becomes the first candidate.
2. If the mesh plant must remain exact authority, retain x24 and derive a hybrid/contact-regime model
   with explicit patch phase. It is unlikely to be a smooth RTI-friendly production model and must
   pass a separate differentiability/deadline gate.

Until that decision is validated, candidate sensitivity, rollout, T0/T1 corrective return and
production integration remain blocked. Production remains the Phase27 artifact.
