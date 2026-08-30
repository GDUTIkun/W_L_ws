# Wheel inertia semantics

The current Phase37 wheel body has its hinge point at body origin and hinge axis along body local Z.
The cylinder collision axis is the same local Z. The visual/inertial STL geom rotates rigidly with
the wheel body.

The source MJCF does not contain an explicit `<inertial>` element. Instead, each wheel STL geom has
assigned mass `0.3431 kg`; MuJoCo compiles its mesh-derived mass properties into:

- `body_ipos`: COM expressed in the wheel body frame;
- `body_inertia`: principal moments about the COM;
- `body_iquat`: principal inertial-frame orientation relative to the body;
- `I_body = R(body_iquat) diag(body_inertia) Rᵀ`.

MuJoCo subsequently rotates this COM-centered inertial frame with the wheel body and applies the
required body-origin/parallel-axis terms internally. The compiled tensors are positive definite,
the largest principal moment aligns with the axle, and no axis swap or double parallel-axis
application is evidenced.

Left/right use separately named STL exports. Their axial COM components are mirrored, while radial
magnitudes and full tensors agree closely. No STEP/SolidWorks assembly or mass-property report is
present, so STL-derived geometry plus assigned total mass is the end of available provenance.

DG38-00: **PASS for model semantics; physical CAD truth remains unavailable**.
