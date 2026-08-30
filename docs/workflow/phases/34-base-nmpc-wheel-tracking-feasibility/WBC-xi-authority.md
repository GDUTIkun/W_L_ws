# Gain-Free Longitudinal WBC Authority

DG34-03: **PASS**.

The Phase34 diagnostic profile adds only two longitudinal wheel-origin acceleration objective rows
to Phase27 Minimal. The QP remains 42 variables and 104 hard rows; A/lower/upper are unchanged.
The analytic affine map matched an independent central derivative to `1.8367e-12 m/s^2`.

At the authoritative Phase28 T0/T1 states, centered common/differential requests were replayed
through MuJoCo physical acceleration:

- minimum physical self gain: `0.7009409313` (gate `>=0.2`);
- maximum cross/self ratio: `0.0130391903` (gate `<=0.5`);
- maximum 2x2 condition number: `1.350769849` (gate `<=10`);
- maximum realized-wrench relative change: `0.0017213554` (gate `<=0.02`);
- maximum hard violation: `1.3678237e-8` (gate `<=1e-7`).

This establishes local gain-free authority and isolation. It does not establish stable position
tracking or closure of the full plant; those remained separate DG34-04 requirements.
