# Phase 38 REVIEW

状态：`PASS`  
日期：2026-08-30  
分类：`P38-A_COM_eccentricity_is_primary_phase_source`

## Gate results

| Gate | Result | Finding |
| --- | --- | --- |
| DG38-00 semantics | PASS | mesh-derived COM-centered principal inertia and transforms established |
| DG38-01 COM | PASS | radial COM `0.12114/0.11975 mm`, both significant |
| DG38-02 tensor | PASS | nearly axle-axisymmetric; no frame/LR error |
| DG38-03 plausibility | PASS | inertia is 94.5–98.8% of solid-cylinder scale |
| DG38-04 rigid-body isolation | PASS | V1 removes >99.9% across mass/bias/response; V2 does not |
| DG38-05 contact amplification | PASS | V1 removes load/ddxi modulation; geometry remains invariant |
| DG38-06 attribution | PASS | numerical source uniquely P38-A; physical correctness unresolved |

Formal-v1 and fresh replay-v1 summaries are exactly equal. Production XML/controller/workspace are
unchanged; Phase32/H0 were not run.

## Required answers

1. **Is COM on the hinge axis?** No. Radial offsets are `0.12114 mm` left and `0.11975 mm` right.
2. **Is inertia axle-axisymmetric?** Approximately yes; transverse anisotropy is only
   `6.70e-5/8.04e-5`, below the frozen significance gate.
3. **Are tensor and COM framed correctly?** Yes within MuJoCo semantics: tensor is about COM,
   `body_iquat` maps the principal frame, axle is principal, and no parallel-axis error is evidenced.
4. **Are left/right inertially consistent?** Yes by frozen gates; V4 was not authorized.
5. **What explains residual phase dependence?** Numerically, radial COM eccentricity. V1 passes
   multi-observable causality; V2 is effectively neutral.
6. **Why does contact amplify it?** Constraints convert the phase-varying eccentric-COM gravity and
   coupling response from mostly internal generalized motion into contact loads and wheel-origin/base
   acceleration.
7. **Is correction physically justified?** No. Centering is numerically causal but repository source
   lacks real/CAD mass-property provenance needed to call the existing `~0.12 mm` offset erroneous.
8. **Single next experiment:** obtain an independent assembled-wheel mass-property measurement/report
   in an axle-fixed frame, resolving radial COM to better than `0.05 mm`. Only that result may authorize
   a centered-COM diagnostic model for Phase37→32→35 revalidation.

PASS means attribution is complete, not that V1 is approved as production truth.
