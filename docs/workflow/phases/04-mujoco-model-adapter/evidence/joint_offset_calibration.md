# Phase 04 Joint Zero Offset Calibration

Date: 2026-08-25

## Method

Phase 02 froze `q_C = -q_M + b_joint`. At MuJoCo `q_M=0`, the imported thigh and shank centerlines were measured in world FLU from the compiled joint anchors and wheel body origin. Therefore each canonical hip angle and relative knee angle directly gives `b_joint`.

The reference angle for a segment from `a` to `b` in the sagittal x-z plane is:

`angle = atan2(b_x-a_x, -(b_z-a_z))`.

This uses geometry, not MJCF joint ordering or raw qpos publication. Wheel rotation has no unique absolute zero for the rotationally symmetric imported wheel; its gauge is frozen to zero. Both sides use the same Phase 02 sign relationship and no mirror-specific sign.

## Frozen values

| Canonical joint | `b_joint` rad |
| --- | ---: |
| left hip | -1.3267204093873923 |
| left knee | 2.2088002548867229 |
| left wheel | 0.0 |
| right hip | -1.3267204093873923 |
| right knee | 2.2088002548867229 |
| right wheel | 0.0 |

At `q_M=0`, the left thigh vector is `[-0.17467, 0.12525, -0.04350] m` and the left shank vector is `[0.17368, 0.04280, -0.14297] m`. The right vectors mirror only the lateral component. Their sagittal angles produce the values above.

Phase 14 replaced the truncated imported Euler constants (`1.5708`, `3.14159`) with exact `pi/2` and `pi` values after the truncation was shown to create a spurious closed-chain constraint rank. The offsets above were then recomputed from the corrected compiled geometry; the mapping equation and calibration method did not change.

## Independent pose regression

The adapter test applies the nonzero MuJoCo perturbation

`[+0.10, -0.07, +0.13, -0.08, +0.11, -0.17] rad`

in canonical left-hip/left-knee/left-wheel/right-hip/right-knee/right-wheel order. It verifies both:

- every published joint satisfies `q_C=b_joint-delta` to `1e-12 rad`; and
- hip and relative knee values reconstructed independently from the transformed FLU segment geometry match the published values within `2e-6 rad` on both sides.

The Simulink baseline nominal reference was also evaluated as `q0=[-0.7023717590, 1.1553364373, 0] rad`, wheel point `[-0.0729388591, -0.5818630029] m`; a second reference was `[-0.6223717590, 1.0453364373, 0.17] rad`, wheel point `[-0.06037436, -0.60353095] m`. These values confirm the canonical sagittal convention, but absolute wheel-point fitting is intentionally not used: the simplified Simulink leg lengths and imported CAD geometry differ. Treating endpoint mismatch as an offset would hide a model-geometry error that belongs to Phase 07/08.

DG03 is closed for the Phase 04 interface mapping. These offsets are model-coordinate offsets, not real encoder zero calibration; hardware zero remains Phase 06 work.
