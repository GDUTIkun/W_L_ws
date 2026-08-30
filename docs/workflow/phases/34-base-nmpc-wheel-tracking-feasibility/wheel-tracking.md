# Wheel Tracking Gain Screen

DG34-04: **FAIL — `P34-E_wheel_tracking_failure`**.

Exactly three critically damped gain sets were frozen before results: 2.5, 3.5 and 5 Hz. Each ran
the existing planner through one `5 mm` common step and one `0.02 m/s`, 0.25 s common ramp while
holding the reset-time differential coordinate. No fourth gain or changed threshold was tried.

All six MuJoCo runs were rejected by the unchanged `NominalWbcModel::kOutsideWorkspace` contract at
ticks 90--92, about 0.39--0.42 s after target onset. The last available common error was still
`4.195--9.580 mm`, so no run met `<=1 mm` final error or `<=0.2 s` settling. Before rejection, all
runs retained bilateral contact and passed the hard, WBC deadline, normalized-slack, differential-
drift and torque-limit gates.

Raw formal-v1 stopped on the first process rejection. Formal-v2 completed all commands but did not
persist stderr semantics. Formal-v3 is authoritative and records all six exit codes and
`status=2`; evaluation-v2 corrects only v1's negative-valued torque-margin presentation and preserves
the same failed classification.

Per the frozen stop rule, Phase34 did not run the x12 T0/T1 corrective oracle, did not integrate a
ControllerCore Phase34 mode, and did not run static/straight/turning closed-loop cases. The result
does not authorize gain retuning, workspace feedback, state augmentation, Q/Qe changes or Eq.(12).
