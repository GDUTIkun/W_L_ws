# Dynamic equivalence audit

Each pair uses identical base pose/twist, leg/passive coordinates, wheel origins, actuator torque
(Phase35 H0 tick 0), controller-independent state, and nominal ground. Only wheel hinge phase changes.

Results:

- maximum contact-on physical-ddxi change over the sweep: `1.5300095116621495 m/s²`;
- maximum contact-off physical-ddxi change: `1.3384056580889592e-4 m/s²`;
- ratio contact-off/contact-on: `8.74769501684314e-5`;
- worst `q/q+2π` generalized-acceleration difference: `18.760394235403602` in native
  generalized-acceleration units;
- some periodic pairs retain identical contact topology and response, while a pair at the
  collision selection boundary changes topology and response.

Therefore absolute mesh phase genuinely enters the current plant's instantaneous dynamics through
raw rotating-mesh contact. The nearly four-order isolation when contact is disabled rules out the
rigid-body mass/bias/WBC map as the primary source. The failure of robust `q/q+2π` response
equivalence despite machine-precision core periodicity is characteristic of discrete collision
manifold selection, not an unbounded hinge or a natural one-radian domain.

No live QP was requested outside its admitted domain. Consequently this audit makes no unsupported
claim about phase-separated QP torque parity; it compares the same frozen applied torque at the
plant boundary.

DG36-03: **PASS** (oracle executed and source isolated); physical-equivalent contact response parity
itself is **FAIL**, which is the positive evidence for P36-D.
