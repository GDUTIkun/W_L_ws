# Phase 37 REVIEW

状态：`REWORK`  
日期：2026-08-30  
Collision verdict：`P37-D_axisymmetric_collision_still_phase_sensitive`  
Closure verdict：`P37-U_not_reached_after_DG37_03`  
H0 verdict：`P37-U_not_reached_after_DG37_03`

## Gate results

| Gate | Result | Finding |
| --- | --- | --- |
| DG37-00 parity | PASS | all non-collision compiled parameters exactly equal |
| DG37-01 static contact | PASS | centroid/normal/depth/count invariant to numerical precision |
| DG37-02 periodic replay | PASS | contact/dynamic `q/q+2π` error `2.30e-13` |
| DG37-03 ON/OFF isolation | **FAIL** | `0.01332` on vs `0.0001338 m/s²` off; ratio `99.54` |
| DG37-04 Phase32 | NOT RUN | stopped at DG37-03 as frozen |
| DG37-05 H0 | NOT RUN | stopped at DG37-03 as frozen |
| DG37-06 workspace | HOLD | still no natural ±1 basis, but no contract edit authorized |
| DG37-07 Phase34 | NOT AUTHORIZED | prerequisites incomplete and live gate unchanged |

Formal-v1/replay-v1 are retained as pre-manifest runner iterations. Formal-v2 and replay-v2 include
complete manifests and reproduce identical summaries; they are the current authority.

## Required answers

1. **Did the cylinder eliminate nonphysical contact-geometry phase sensitivity?** Yes: contact
   centroid, normal, depth and topology are invariant to `≤2.78e-17` scale.
2. **How much did physical-ddxi sensitivity fall?** From `1.5300` to `0.0133225 m/s²`, about
   `114.84×`; nevertheless the stricter ON/OFF gate fails by roughly `9.95×` relative to its ratio limit.
3. **Phase32 x16 result after correction?** Unresolved/not run because DG37-03 failed.
4. **Does Phase35 H0 drift persist?** Unresolved/not run for the same causal reason.
5. **Does ±1 rad retain a validity basis?** No natural transition or periodicity basis was found,
   but production removal remains unauthorized until the residual is closed.
6. **Next phase?** Continue remaining full-body/contact mismatch attribution, specifically validate
   the CAD-derived wheel COM/inertia about its axle. Do not choose 12D/16D, restore Eq.(12), change
   workspace, or reopen servo tracking yet.

No RECORD is created because REVIEW is REWORK. The original nominal model and production controller
were not modified.
