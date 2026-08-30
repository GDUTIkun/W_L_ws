# Contact-constrained amplification audit

V0 physical-ddxi modulation is `1.3384e-4 m/s²` contact-off and `0.0133225 m/s²` contact-on, an
amplification of `99.54×`. The same phase-dependent COM produces gravity/bias and coupling terms;
without contact most response remains internal wheel/generalized acceleration, while contact
constraints redirect it into wheel-origin/base acceleration and contact force.

V1 removes that mechanism consistently:

- contact-on ddxi modulation becomes `2.8606e-8 m/s²` (`2.15e-6×` V0);
- normal-load modulation becomes `1.32e-9 N`;
- tangential-load modulation becomes `2.54e-8 N`;
- contact-off ddxi is `9.54e-8 m/s²` and contact-on/off ratio becomes `0.300`.

V2 retains V0 amplification and response. V3 is numerical zero; its formal ratio is not physically
meaningful because the denominator is zero-scale. Across all variants, contact centroid, normal,
depth and count remain invariant, excluding a reintroduced geometry artifact.

DG38-05: **PASS — contact amplifies the small eccentric-COM rigid-body effect rather than creating a
new geometry-phase state**.
