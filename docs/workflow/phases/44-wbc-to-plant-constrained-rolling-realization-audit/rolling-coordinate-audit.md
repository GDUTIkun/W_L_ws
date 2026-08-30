# Rolling Coordinate Audit

Evidence: [`rolling-kinematics.csv`](evidence/automated/realization-audit-formal-v1/rolling-kinematics.csv),
[`mujoco-contact-details.csv`](evidence/automated/realization-audit-formal-v1/mujoco-contact-details.csv)

Each snapshot records native wheel rate, direction-normalized rim rate, wheel-center relative rate `dxi`,
wheel-surface material-point tangential/lateral/normal velocity, penetration and load. Tangential acceleration is
computed at the frozen current material point using
`a_P=a_C+alpha x r_CP+omega x (omega x r_CP)`; contact-centroid migration is not differentiated.

At tick0 all rates/slips are zero, yet the post-command material-point tangent acceleration is already nonzero and
left/right asymmetric. Across Phase43 own trajectories, `dxi`, native rim rate and material slip are not one
coordinate: xi depends on wheel-center/base/leg kinematics, native spin changes surface velocity, and contact
reaction determines whether either lies on the rolling manifold.

Consequently neither xi alone nor native qdot alone is a sufficient rolling truth. The minimum future regulated
state must be contact-consistent and contain at least wheel-center xi, native wheel spin and tangential slip/contact
load information. Phase44 does not implement that repair.
