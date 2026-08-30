# Phase 44 Addendum — Decision

## Q1 — Why did the central Jacobian fail?

It is a real, deterministic piecewise directional response, not numerical replay noise. The frozen
observable regime and exact assembled near-active row signature remain unchanged at all three delta
scales, yet 84/480 output-family rows are R44-P. Fresh replay semantic error is `0`.

## Q2 — Does tick0 QP +1 to negative/attenuated wheel authority remain?

Yes. Trusted B/native-common one-sided results give QP wheel authority about `+1` and MuJoCo
authority about `-0.509/-0.508`. C/xi common remains about `+1` QP to `+0.948` MuJoCo, while its
native wheel response changes sign from about `-2.56` QP to `+1.33` MuJoCo.

## Q3 — Is contact cancellation stable in same-regime probes?

Yes in the explicitly bounded D/native-common mechanism scope: 27/27 trusted directions oppose
actuator authority, with cancellation ratio `0.761..0.902`. This is not generalized to every input
channel.

## Q4 — When do late trajectories become directional/piecewise?

B has first persistent R44-P at tick98; D at tick110; C has none in its audited own snapshots. B's
onset is one tick after common-rate growth and coincides with task-residual onset. D's onset follows
rate growth by seven ticks and coincides with xi-deviation onset. These are associations only.

## Q5 — Final classification

`P44-E — Multiple realization layers`:

1. initial controller-to-plant contact/constrained realization mismatch is material and confirmed by
   trusted tick0 directional authority;
2. late B/D piecewise directional response and Phase44's already-approved late task competition are
   material contributing layers;
3. C's xi improvement remains leg/wheel-center reconfiguration rather than native wheel spin, so xi
   is insufficient as the complete rolling-manifold coordinate.

Mechanism tags: initial `B-contact`; late `optimization + constrained directional branch`;
coordinate `C-insufficiency`. This is not a repair selection and does not authorize a particular
new task representation.

