# Cleanup Regression

## Environment and commands

- Interpreter: `/home/t/W_L_ws/.venv/bin/python`
- MuJoCo `3.7.0`, NumPy `2.2.6`, SciPy `1.15.3`
- Pre-cleanup build: 5 packages, 35 tests PASS (included retired STM32 bridge).
- Post-cleanup build: 4 packages, 37 tests PASS; no STM32 package or dependency remains.
- Historical replay build: `wheel_leg_mujoco` with
  `-DWHEEL_LEG_BUILD_LEGACY_RUNNERS=ON`; final current build returns the option to OFF.

## Pre vs post authoritative snapshot

| Gate/metric | Pre | Post | Result |
| --- | ---: | ---: | --- |
| W1–W6 | PASS | PASS | exact |
| 42D H0 witness | PASS | PASS | exact |
| COMP / EQ | PASS / FAIL | PASS / FAIL | exact |
| solver | SOLVED | SOLVED | exact |
| active-set signature | `598c64fd2b1b1793` | same | exact |
| QP operator dump SHA-256 | `836ed659...c86b8` | same | byte-identical |
| baseline normalized slack | `0.001522220395389018` | same | exact |
| primitive R2 normalized slack | `0.05850370867784012` | same | exact |
| dominant | RIGHT Tx, `-0.05850370867784012` normalized | same | exact |
| W_ref primitive feasible | NO | NO | exact |
| minimum unavoidable normalized L∞ deviation | `0.07832043067340007` | same | exact |
| W5 operator / offset | `1.0658141036401503e-14` / `2.6645352591003757e-15` | same | exact |
| historical operator / offset | `7.656794677961396` / `7.2009163679271335` | same | exact |
| R1 residual | `1.2258392916278746e-14` | same | exact |
| primitive residual | `2.7419271386719402e-7` | same | exact |
| hard residual | `3.623378202098832e-9` | same | exact |
| minimum inequality / torque margin | `0.2197161714633595` / `1.9990801609079853` | same | exact |

The primitive decision JSON, slack-closure decision JSON and QP dump are byte-identical. The raw
one-row CSV differs only in measured `wbc_time_s` (`0.002038869` vs `0.001041431`); every state,
W_ref, W_WBC, tau, slack, solver, rank, residual, margin and active-set field is text-identical.

## ROS current-path regression

- H0 initialization equals the frozen golden RobotState and recreates bilateral contact after reset.
- `controller.mode=weighted_wbc` produces TorqueCommand values exactly equal to direct Core at H0.
- Five-second launch smoke produced finite nonzero torque with no Controller rejection/safety-latch log.
- Existing invalid, nonmonotonic, timeout and stale-command fail-safe tests remain PASS.

## Decision

```text
BEHAVIOR CHANGE: NO
WORKSPACE CLEANUP: PASS
```

Evidence is append-only under this Phase's `evidence/automated/{pre,post}-cleanup-*` directories.
