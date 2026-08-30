# CAD / analytic plausibility

For `m=0.3431 kg`, `r=0.05 m`, and width `L=0.04 m`, a non-authoritative solid cylinder gives:

- axial inertia `4.28875e-4 kg·m²`;
- transverse inertia `2.6018417e-4 kg·m²`.

The compiled wheel is `98.80%` of the axial reference and `94.50–94.51%` of the transverse
reference. It is positive definite and contains no order-of-magnitude anomaly or suspicious axis
swap. This comparison is scale-only: the STL may represent hub/spokes/flanges, while its single
assigned mass does not encode material or assembly density provenance.

No SolidWorks/STEP mass-property report, motor/hub/fastener decomposition, or measured COM record
was found. Consequently the `~0.12 mm` radial offset is plausible as either real small imbalance or
mesh/uniform-density export detail; repository evidence cannot choose between them.

DG38-03: **PASS for plausibility, not physical validation**.
