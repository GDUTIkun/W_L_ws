# Phase 23 canonical NMPC state contract

Authority: P23-T02 mapping evidence. The original 16D mapping below remains a
valid diagnostic map, but its `xi/dxi` pair was rejected from the production
OCP after the input-port closure audit in
[model-closure-decision.md](model-closure-decision.md). The selected physical
NMPC state is the first 12 entries; MuJoCo is used only by the external oracle
that generated the golden vectors.

## State and input

The selected physical state is

```text
x = [p_B^N(3), r_B^N(3), v_B^N(3), omega_B^N(3)]
```

The oracle additionally audits the rejected historical auxiliary pair through

```text
x = [p_B^N(3), r_B^N(3), v_B^N(3), omega_B^N(3), xi_L, xi_R, dxi_L, dxi_R]
```

- `B` is the canonical `base_control_frame` site used by Phase 21, not the
  model root body or center of mass. `N` is the canonical world frame.
- `p_B^N` is the position of `B`, in metres.
- `r_B^N = Log(R_N_from_B R_N_from_B_ref^T)` is the shortest-arc spatial
  rotation vector, in radians. It has the same sign and axis as the existing
  Phase 21 `orientationError`; it is not Euler angle. The frozen chart requires
  `||r|| <= 0.35 rad` and a normalized finite input quaternion.
- `v_B^N` and `omega_B^N` are the spatial linear and angular velocity of `B`,
  expressed in `N`, matching canonical `RobotState`.
- For side `s`, `xi_s = e_x^T R_N_from_B^T (p_wheel_s^N - p_B^N)`. Its time
  derivative includes the rotating-frame term:
  `dxi_s = e_x^T[R^T(v_wheel_s^N-v_B^N) - (R^T omega_B^N) x
  (R^T(p_wheel_s^N-p_B^N))]`.
- Active joint positions must remain inside the Phase 15/21 workspace before
  this map is accepted. The corresponding enumerated wheel-coordinate envelope
  is `[-0.3303432354, 0.1678677251] m` left and
  `[-0.3321211483, 0.1659029424] m` right; these are diagnostic consequences,
  not a substitute for the joint/workspace gate.

The NMPC input is the existing WBC reference quantity:

```text
u = [Fx_L,Fy_L,Fz_L,Tx_L,Ty_L,Tz_L,
     Fx_R,Fy_R,Fz_R,Tx_R,Ty_R,Tz_R]
```

Forces and moments are expressed in canonical base FLU and each moment is about
`base_control_frame`. Units are N and N·m. The Phase 21
`wrench_flu_map` already transports contact-centred wrench to this point; an
upper model must not add the contact lever arm a second time.

## Reference and timing

- On reset, `p_ref.x/y` and `R_ref` are captured from the first valid state;
  height, leg equilibrium and equilibrium wrench remain the frozen Phase 21
  nominal values. Phase 23 may alter only the internal small longitudinal
  `p_ref.x` profile.
- The WBC tick remains 10 ms. NMPC runs synchronously at schedule phases 0, 2,
  4, ... (20 ms); the accepted wrench is used on its solve tick and exactly one
  following WBC tick. Its valid ages are therefore 0 and 1 WBC ticks.
- A repeated, decreasing or non-finite source timestamp, a source interval
  outside the existing 10 ms tolerance, an invalid contact/workspace/chart, or
  an age beyond one tick is rejected. There is no last-valid or nominal-wrench
  fallback.
- Reset clears the fault latch, sparse-solver state, previous-applied input,
  accepted wrench, timestamp and schedule phase. The first valid post-reset
  tick is a cold NMPC update at phase zero.
- Any mapping, solver, audit, timing, stale or WBC failure produces six exact
  zero torques on that tick and latches until reset.

## Oracle evidence

The repository interpreter dependency probe recorded MuJoCo 3.7.0, NumPy
2.2.6 and SciPy 1.15.3. The source and config are
`tools/experiments/validate_nominal_nmpc_state.py` and
`simulation/mujoco/config/phase23_nmpc_state_oracle_v1.json`.

The first append-only run, `automated/state-oracle-v1`, is superseded because
its evaluator accidentally included the two static `xi` positions in an
`equilibrium_speed` slice. It is retained as failed evaluator evidence. The
corrected authority is `automated/state-oracle-v2`:

- all nine gates PASS;
- maximum base-twist error `5.56e-17`;
- maximum position/xi-rate finite-difference error `1.38e-10`;
- maximum rotation-chart rate error `8.01e-11`;
- quaternion-sign and repeated-run error exactly zero;
- equilibrium `xi=[-0.00957364950, -0.01274069584] m` and all physical speeds
  exactly zero.

These results close only the 12D base mapping portion of DG23-01. The continuous model,
RK4 and sensitivity remain P23-T03 and are not inferred from this PASS.
