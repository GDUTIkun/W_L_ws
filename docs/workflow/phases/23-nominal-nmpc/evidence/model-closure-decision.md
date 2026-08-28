# Phase 23 model-closure decision

Verdict: the historical 16-state candidate is rejected as a production model;
Phase 23 proceeds with a 12-state locked-composite base model while preserving
the same 12D current contact-wrench output boundary.

## Evidence

The apparent 12D input match is only dimensional:

- Historical `full_base_body_dynamics.m` explicitly defines its inputs as
  wheel-to-body interaction wrenches. Its wheel equation applies their force
  and axle torque with the opposite sign to each wheel and uses the historical
  rolling denominator. Its base moment is reconstructed from `halfTrack`,
  height and `xi`.
- Current `WeightedWbcProblem::assemble` applies
  `NominalWbcModel::wrench_flu_map` to the solved contact-centred wrench before
  comparing it with `WbcReference.interaction_wrench_flu`.
- Current `NominalWbcModel::evaluate` constructs that map from the contact frame
  and `skew(contact_center - base_control_position)`, so the reference is the
  external ground-contact resultant already transported to the canonical base
  control point.

Consequently, substituting the current input into the historical equations
would both treat an external wrench as an internal one and add the contact
lever arm a second time. The historical `xi` acceleration also needs internal
wheel/body forces or an inner-actuation policy that the current external
contact-wrench port does not contain. No unique 16D `f(x,u)` with those copied
equations exists at the approved boundary.

The current external wrench does close whole-system base dynamics. Under the
Phase 23 nominal assumptions that the Phase 21 leg task locks internal shape
near equilibrium, the exact reduced model's base 6x6 block is a rigid composite
spatial inertia. This authorizes a smaller candidate state
`[p(3), r(3), v(3), omega(3)]`; Phase 15 joint/workspace and the independently
validated `xi` map remain input validity diagnostics at every tick.

This is a pre-production REWORK decision under DG23-01, not evidence that the
12-state model passes. The replacement continuous model, RK4 and sensitivities
must still pass the current reduced-model oracle before any solver/Core code is
allowed.
