# Collision model parity

Authority: `evidence/automated/causal-revalidation-formal-v2/collision_parity.json`.

The new revision keeps the original CAD wheel mesh as a non-colliding visual/inertial geom and adds
one massless cylinder per wheel as the only wheel-ground collision geom. Cylinder radius is `0.05 m`,
half-width is `0.02 m`, and its local Z axis is the wheel hinge axis.

Compiled old/new comparison gives exactly zero difference for body pose, wheel mass, COM, principal
inertia and inertia frame, joint origin/axis/range, damping/frictionloss, actuator gear/control range,
equality data, gravity, timestep, integrator, ground friction and solver contact parameters. Dimensions
remain `(nq,nv,nu,nbody)=(17,16,6,12)`; `ngeom` changes from 74 to 76 only because each former wheel
geom is represented by one visual mesh plus one collision cylinder. Visual masks are `(0,0)` and
cylinder masks are `(0,1)` against the floor `(1,0)`.

The nominal baseline file remains unchanged with SHA-256
`d7d46cd47d92097fb21d0d5923c83dfac07a5f1c48e2be65deb8e98c4a67f0d9`.

DG37-00: **PASS**.
