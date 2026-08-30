# Phase 38 RECORD

状态：`complete`  
日期：2026-08-30  
分类：`P38-A_COM_eccentricity_is_primary_phase_source`

The Phase37 residual is numerically caused by the compiled wheel mesh COM being about `0.12 mm`
off the hinge axis and amplified by contact constraints. Centering COM alone reduces contact-off
ddxi modulation to `7.13e-4×`, contact-on modulation to `2.15e-6×`, mass modulation to
`4.71e-4×`, and bias modulation to zero. Transverse-inertia symmetrization alone leaves the response
unchanged. Cylinder contact geometry remains phase invariant.

The tensor is positive, axle-aligned, nearly axisymmetric and analytically plausible; L/R parameters
are consistent. No frame, parallel-axis, or mirror implementation error is evidenced. Because only
STL geometry and assigned total mass are available, centered COM is not approved as physical truth.
The next authority must be an independent assembled-wheel mass-property result in the axle frame.

Authority:

- config hash: `12eb6909f1a5d6735e51a2e13076d67cc45febbae44e42c216abf02f73c91ccf`;
- formal: `evidence/automated/inertia-attribution-formal-v1`;
- fresh replay: `evidence/automated/inertia-attribution-replay-v1`;
- MuJoCo 3.7.0 / NumPy 2.2.6 / SciPy 1.15.3 via repository `.venv`.

No Phase32/H0/controller/workspace change was made.
