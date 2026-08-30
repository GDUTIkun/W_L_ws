# Long-horizon periodicity audit

Authority: Phase39 Model B, Phase32 T0_static tick 54 plant state, fixed Phase31 baseline torque.
Formal authority is `evidence/automated/angle-domain-formal-v2/periodicity.json`; replay-v2 is
decision-identical. formal-v1/replay-v1 are retained and superseded because the final classification
name and independent-stop reporting were completed in v2.

The frozen corpus covers left-only, right-only and bilateral shifts:

- mandatory: k = 0, ±1, ±5, ±25, ±50 revolutions;
- engineering: k = ±500 through ±1,000,000 by the PLAN sequence;
- diagnostic: k = ±5,000,000, ±50,000,000, ±500,000,000.

At every engineering state, body/geom/site positions and rotations, contact point/frame/depth/load
and topology, M, bias, all-body Jacobians, closure, qacc, physical ddxi and wheel qacc are finite and
within gates. Results:

| Metric | Engineering maximum | Gate |
| --- | ---: | ---: |
| normalized physical error | 5.488302590173079e-10 | 1e-8 |
| dynamic raw error | 3.940181514394681e-13 m/s² | 1e-4 m/s² |
| rotation orthogonality error | 6.883382752675971e-15 | 1e-10 |
| contact topology mismatch | none | exact |

DG40-02: **PASS**. q and q+2πk physical equivalence is preserved through the pre-frozen engineering
horizon of one million revolutions.
