# Wheel-phase semantics

## Live mapping

Canonical order remains `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`.
The MuJoCo Adapter maps wheel hinge state with zero offset and negative sign:
`q_canonical=-q_native`; reset/equilibrium wheel phase is zero. Both wheel joints are scalar,
unlimited hinges with local axis `0 0 1`; MuJoCo retains the unbounded scalar coordinate and does
not wrap it.

In `wheel_leg.xml`, each named wheel collision geom is `type="mesh"`, belongs to its wheel body,
and has collision enabled. The same body is rigidly rotated by its wheel hinge. Thus the collision
mesh—not merely a visual mesh—rotates with native wheel phase. The compiled mesh has 875 vertices
per side. The pre-frozen finite-symmetry search found no exact order 2–24 symmetry at `1e-6 m`;
the smallest tested mismatch remains millimetric.

## Distinct quantities

- **joint coordinate / wheel spin**: the unbounded hinge scalar;
- **mesh phase**: that scalar modulo `2π` as seen by the rigid collision mesh;
- **wheel origin**: wheel-body origin, fixed by the leg pose and unaffected by its own hinge angle;
- **xi**: base-frame longitudinal coordinate of the wheel origin, not rolling arc length;
- **rolling displacement**: a path integral requiring contact/no-slip semantics; it is not the
  absolute hinge coordinate;
- **zeta**: base-frame vertical wheel-origin coordinate.

## Model dependency

MuJoCo rigid transforms use the hinge rotation, so wheel inertial and collision frames are periodic.
The raw contact manifold explicitly depends on rotated mesh vertices. `NominalWbcModel` uses
wheel angle in forward kinematics, but its analytic cylinder-like contact construction uses the
wheel axis/radius rather than MuJoCo's discrete mesh contact points. Its live workspace validator
rejects canonical wheel delta outside `[-1,+1] rad` before a valid live QP result can be claimed.

DG36-00: **PASS**. These semantics are sufficient for the isolated audit and do not reinterpret
wheel spin as xi, zeta, reach, or a mechanical travel limit.
