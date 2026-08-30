# Wheel COM-to-axle audit

For hinge origin `o=0` and axle `a=[0,0,1]`, the compiled COM values are:

| side | COM in body/axle frame (m) | radial distance |
| --- | --- | ---: |
| left | `[-5.8294e-5,-1.06191e-4,+7.41514e-3]` | `0.121140 mm` |
| right | `[+1.14221e-4,+3.59707e-5,-7.41514e-3]` | `0.119751 mm` |

Both exceed the pre-frozen `0.05 mm` numerical-significance threshold. The axial components do not
produce wheel-spin phase modulation; the radial components rotate about the axle and do.

Radial-magnitude left/right mismatch is only `1.146%`, below the frozen `5%` V4 threshold. Their
directions are not a simple mirror, but the available independent STL exports provide no assembly
datum with which to label those directions correct or erroneous.

DG38-01: **PASS**. Current COM is not on the hinge axis; V4 is not authorized.
