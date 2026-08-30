# Shadow H0 beyond the historical workspace gate

This is diagnostic-only Model B, Phase27 Minimal, fixed equilibrium interaction wrench, no xi task,
no target and no gain change. The diagnostic policy ignores only wheel absolute-magnitude workspace
rejection. Leg workspace, finite/model/solver checks, contact, hard/slack/torque and frozen base
envelopes remain active. The production/default policy remains `kEnforce`.

| Event | Result |
| --- | --- |
| historical right-wheel ±1 crossing | tick 96 |
| continued past historical gate | yes |
| first independent stop | tick 111, right wheel contact loss |
| max accumulated rotation at stop | 2.848143533837517 rad = 0.4532961220 rev |
| all independent gates through tick 110 | PASS |
| formal/replay semantic max error | 0 |
| 3-revolution stop reached | no; earlier independent gate correctly stopped run |

This establishes `P40-G_post_bound_rollout_reveals_independent_real_failure`. It does not establish
production safety beyond ±1, does not solve wheel-spin drift, and does not show an absolute-angle
representation failure. It shows the historical gate was masking a later contact failure in this
uncontrolled H0 trajectory.
