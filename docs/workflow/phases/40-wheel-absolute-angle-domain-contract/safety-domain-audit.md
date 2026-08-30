# Safety and controller-domain audit

Model validity and safety domain are separate.

For Model B, no collision, rigid-body, closure, conditioning or finite failure is tied to ±1 rad.
The live value originates in the Phase21 near-equilibrium workspace table and is applied by raw
subtraction; Phase36/37/39 and Phase40 periodic evidence show no physical boundary there.

Repository inspection found no authoritative continuously rotating wheel cable-twist, hard-stop,
wire-routing, gearbox travel, encoder absolute range or mechanical-interference limit. This is
**unknown/not established**, not proof that the assembled robot has none. The only located firmware
limit flag concerns a knee, while the C620 wheel implementation reconstructs multi-turn rotation.

Controller-domain findings:

- Phase27 Minimal and the current WBC physical model do not require wheel q near zero.
- Wheel dq, contact, solver residual, torque/slack, finite state, base envelope and conditioning
  remain meaningful independent gates.
- Other generic joint-position/standing profiles use raw position subtraction across all joints;
  they require a local/periodic reference audit before any global wrapped-state migration.
- No evidence authorizes removal of the gate on real hardware. It authorizes a next Phase to make
  the minimal software workspace-contract correction and then revalidate H0.

Verdict: no established nominal model/software safety authority supports ±1 rad; real mechanical
and sensor authority is unresolved, so `P40-E` is not claimed.
