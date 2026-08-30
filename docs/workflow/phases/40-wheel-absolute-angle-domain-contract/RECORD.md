# Phase 40 RECORD

Status: **complete / REVIEW PASS**  
Date: 2026-08-30

## Decision

Freeze R3 for the current nominal architecture: wheel absolute q is a finite raw unwrapped cyclic
hinge coordinate; physical model validity is periodic modulo 2π; dq is an independent state. The
absolute `[-1,+1] rad` wheel workspace bound has no supported nominal model/software basis.

Do not globally wrap RobotState q. Do not add recenter/revolution state until a named engineering
lifetime or odometry consumer requires it. Do not infer real-hardware safety from repository
silence.

## Evidence retained

- Model B periodicity through ±1e6 revolutions: PASS;
- maximum engineering normalized error `5.488302590173079e-10`;
- first diagnostic material angle-reduction error at `5e7` revolutions;
- R1 physical parity PASS but raw residual discontinuity established;
- R2 physical/recenter parity PASS, not required now;
- shadow H0 crosses the historical gate at tick 96 and stops on right contact loss at tick 111;
- formal-v2/replay-v2 summary hash identical.

## Change boundary

Production/default wheel workspace enforcement remains unchanged. Phase40 added only a non-default
diagnostic policy, a separately compiled Phase40 target, regression coverage, config/runner and
evidence. Phase34 was not run.

## Next authorized work

Create a minimal workspace-contract correction Phase. It may replace the wheel magnitude rejection
with the R3 contract, rerun H0 under production semantics, and only then decide whether Phase34 xi
tracking is eligible to reopen. The tick-111 contact loss and unresolved real hardware limit
authority remain explicit inputs; wheel-spin drift remains a separate control/model issue.
