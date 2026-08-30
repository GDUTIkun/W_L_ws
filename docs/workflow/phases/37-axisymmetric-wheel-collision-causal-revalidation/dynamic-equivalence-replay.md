# Dynamic equivalence and ON/OFF replay

Phase36 rotating-mesh maximum physical-ddxi phase effect was `1.5300095 m/s²`. With the cylinder it
falls to `0.0133225 m/s²`, an improvement ratio of `0.00870748` (about `114.84×` lower), satisfying
the frozen `≤0.01` improvement target.

However contact-off phase effect is only `0.000133841 m/s²`. Contact-on remains about `99.54×`
contact-off, failing the frozen `≤10×` or `≤0.001 m/s²` isolation rule. The retained, deliberately
unchanged wheel inertial model is itself phase-sensitive away from a full revolution:

- mass/reduced-mass phase variation: `7.97e-5`;
- bias/reduced-bias phase variation: `7.82e-4`.

Contact geometry is invariant, so the remaining response is not the old mesh-facet/contact-topology
artifact. Evidence is consistent with the preserved off-axis COM/non-axisymmetric inertia being
amplified by contact constraints, but Phase37 forbids changing those quantities and cannot close that
attribution experimentally.

DG37-03: **FAIL — P37-D_axisymmetric_collision_still_phase_sensitive** under the frozen Phase37
isolation definition. Formal-v2 and fresh replay-v2 reproduce the same summary exactly.
