# Proposed minimal workspace contract

For the next Phase, propose replacing only the wheel absolute-magnitude part of the WBC workspace
validator:

```text
leg coordinates: retain current validated finite workspace bounds
wheel q: require finite; do not reject by |q-qeq|
wheel dq: retain finite and independently justified rate/safety checks
physical model: require periodic equivalence, finite outputs, closure/conditioning validity
contact/controller: retain contact, solver, hard/slack, torque and base-envelope gates
hardware: do not remove a real limit until cable/stop/encoder authority is explicitly established
```

Keep raw unwrapped R3 at the shared RobotState boundary. Do not globally wrap q. Do not add a
revolution-count field unless a named odometry/maintenance consumer needs it. Keep xi independent
of q. The minimal correction Phase must remove the diagnostic bypass from experimental authority,
make the intended wheel policy explicit in the production contract, rerun default regressions and
H0, then decide whether the frozen Phase34 xi tracking gate can reopen.

Phase40 does not enact this proposal. The only C++ addition is a non-default diagnostic policy used
by the Phase40 target; tests prove the default still rejects the same wheel and preserves all leg
bounds.
