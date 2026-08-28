# Phase 23 locked-composite model oracle

Authority: P23-T03 / DG23-01. The authoritative append-only run is
`automated/model-oracle-v5`; v1 retained a real equilibrium FAIL caused by the
decimal-frozen WBC reference, while v2/v3/v4 are superseded model/output-schema
steps. v5 adds the non-zero yaw-anchor check required by the relative chart.

The selected 12D model uses the current nominal locked composite at the
canonical base-control point:

- mass `6.4344 kg`;
- COM offset in B `[-0.01118028740, 0.00009238574, -0.07308450551] m`;
- full non-diagonal inertia about COM recorded in the v5 summary;
- two external base-FLU contact wrenches, each already about the base-control
  point; Newton-Euler moment transport is therefore only from that point to COM.

The frozen Phase 21 wrench required a maximum `1.17e-7` N/N·m symmetric
projection to make the rigid-composite equilibrium exact. This projection is
smaller than the downstream WBC audit tolerances and is recorded explicitly;
the source wrench remains in the manifest and was not overwritten.

Independent Python results:

- all ten v5 gates PASS;
- spatial-inertia reconstruction error exactly zero;
- continuous acceleration versus current 12-DoF reduced mass/bias maximum
  `1.18e-11`;
- equilibrium derivative `3.42e-15`;
- 20 ms RK4 versus DOP853 maximum `1.06e-9` and RK4 step-doubling maximum
  `9.94e-10`;
- continuous/discrete finite-difference reference stability `3.22e-9` and
  `6.94e-11`; repeated evaluation error exactly zero. A non-zero `0.22 rad`
  yaw anchor verifies `R_N_from_B=Exp(r)R_ref` rather than treating the relative
  chart as an absolute orientation.

The C++17 `NominalNmpcModel` uses fixed-size Eigen and forward automatic
differentiation, not runtime finite differences. Its five-sample golden test
reports continuous/RK4 value errors `3.11e-15`/`5.56e-17` and
continuous/discrete sensitivity errors `1.34e-9`/`4.13e-11`. Non-finite input
and a rotation chart norm above `0.35 rad` are rejected with zero-initialized
results. The Release Core suite passed 7/7 tests; the repository-wide test
summary remained `25 tests, 0 errors, 0 failures`.

This closes DG23-01. It does not authorize the OCP profile, tracking weights,
solver, Core integration or MuJoCo claims, which begin at DG23-02/03.
