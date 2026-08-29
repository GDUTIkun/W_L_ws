# Phase 23 T05 reference/cost/constraint pre-freeze

Date: 2026-08-28

Freeze point: the profile, tuning/holdout split and all gates below were fixed before any
T05 holdout case was executed. This record authorizes only the already-declared holdout
run; it is not integrated formal or Phase authority.

- frozen candidate: `simulation/mujoco/config/phase23_acados_t05_profile_v1.json`
  (`sha256 4f11075cb8a112a516caad06d7d34fce4cb0a76bbe9b94cebbabfe96bbc96d15`)
- tuning config: `simulation/mujoco/config/phase23_acados_t05_tuning_v1.json`
  (`sha256 04ff9eb53fdf325152a2bb12a5f53c6388cd199ef609a2721836848bf2acc091`)
- unseen holdout config: `simulation/mujoco/config/phase23_acados_t05_holdout_v1.json`
  (`sha256 d099d49641f579e3b3f83ec717f12dd33ab207257ccef387e7db3763bd3f6e13`)

The candidate keeps the normalized Q/R and terminal multiplier from the T04 prototype,
freezes the existing per-side wrench bounds, and freezes a relative state envelope from
the Phase 21 plant safety gates plus the Phase 23 model-validity scales. T06 must
transcribe that envelope at stages 1..N relative to the reset anchor; this does not change
the public 12D state. Delta-wrench optimizer memory is deliberately absent: the true
solver re-run must keep maximum componentwise `delta_u/input_scale <= 0.05`, otherwise
the no-augmentation decision fails rather than being silently changed.

The authoritative reduced-model pre-freeze run
`automated/2026-08-28-phase23-t05-prefreeze-v2/` passed all gates. It explicitly
supersedes v1, which had the same passing results but omitted the validator's own hash
from its manifest. Across four isolated,
newly generated acados variants and four 10 s cases, every solve returned zero and input
and state-envelope violations were zero. The baseline maximum normalized delta wrench was
`0.0238423`. Removing longitudinal state cost eliminated step tracking; removing terminal
cost increased return error from `3.37e-7 m` to `5.13e-3 m` (ratio `15228`); replacing the
selective wrench cost with a uniform cost increased protected-axis motion by a ratio
`75.16`. These are re-solved ablations, not accounting on a fixed trajectory.

The integrated tuning run `automated/2026-08-28-phase23-t05-tuning-v1/` then passed all
four declared 10 s cases (`hold`, positive, negative, return) and the solver-failure safety
probe without changing the candidate. All inherited plant/WBC, independent dynamics/KKT,
deadline, tracking and recovery checks passed. The largest observed Core step was
`3.168 ms`; the candidate tracking gates remain `1.5 mm` step, `0.8 mm` return excursion
and `0.5 mm` final return error.

The next permitted action is to run exactly the six holdout cases already present in the
frozen holdout config. Profile, split, reference amplitude/timing and gates must not change
after observing those results.

## Frozen holdout result

The append-only run `automated/2026-08-28-phase23-t05-holdout-v1/` used the frozen
holdout config hash above without changing the profile or any gate. All six 10 s nonlinear
combination cases and the solver-failure safety probe passed. Step final displacement was
`1.677..1.883 mm` with the required sign; return excursions were `1.232/1.439 mm` and
final errors `0.216/0.110 mm`.

Across holdout, the maximum combined NMPC-audit-WBC time was `3.558 ms`, independent
dynamics defect `2.80e-6`, projected stationarity `0.03215`, hard violation `1.63e-8`,
task residual `0.005325`, normalized slack `0.003975`, and closure residual `0.18349 mm`.
Minimum wheel normal load was `30.949 N`; no saturation, solver failure, non-finite tick or
contact loss occurred. DG23-03 is therefore closed. T06 must implement the frozen state
envelope and prove generated/C++ parity before this profile can become production input.
