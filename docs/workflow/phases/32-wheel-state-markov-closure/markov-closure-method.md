# Phase 32 Markov-closure method

Every pair replays the complete Minimal-WBC prefix, changes only the target state, preserves the
requested internal wrench, and separately records WBC torque/realized wrench and a fixed-baseline-
torque plant response. Direct base components must match within `1e-9`, reconstructed `xi/dxi`
within `1e-7`, and requested wrench within `1e-12`. Smooth pairs use full/half perturbations with a
`10%` sensitivity-consistency gate and a `0.05 m/s²` closure threshold.

## Pair families

- **C1 configuration:** per side perturb one hip coordinate, solve the other three leg coordinates
  to preserve closed-chain equality X/Z residual and `xi`, then minimally correct leg velocity to
  preserve equality velocity and `dxi`. The authoritative `0.00035 rad` scale remains inside the
  bilateral soft-contact regime. The earlier `0.001 rad` run is retained as inconclusive.
- **C2 velocity:** at fixed configuration, take the numerical null vector of
  `[closure-velocity X/Z; dxi]`, scaled to `0.5 rad/s` maximum active-joint change per side.
- **C3 wheel spin:** at fixed q and all other v, add common/differential `1 rad/s` canonical wheel
  spin. This is an exact x16 projection null direction but has contact-constraint Jacobian norm at
  least `0.09969`.
- **Hybrid wheel angle:** at fixed remaining q/v use common `±0.02 rad`. The collision wheel is a
  faceted mesh, so angle changes discrete contact-patch membership. Smooth derivative scaling is
  inapplicable; the frozen hybrid gate instead requires bilateral contact, independent oracle
  convergence, `>0.05 m/s²` in every authority pair, and byte-identical fresh replay.

The rank audit shows `closure+xi` has one leg-coordinate null direction per side, while additionally
fixing wheel height raises the four-coordinate matrix to rank four (minimum singular value
`0.03319`). Hence C1/C2 are real only because x16 omits soft-contact height/normal motion; no pair was
fabricated under fixed normal geometry.
