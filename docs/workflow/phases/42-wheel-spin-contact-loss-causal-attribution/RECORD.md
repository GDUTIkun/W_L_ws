# Phase 42 Record

Status: **complete / REVIEW PASS**  
Date: 2026-08-30

## Decision

The Phase41 H0 wheel-spin/contact-loss chain is classified as
`P42-E_multiple_coupled_causes`. A non-equilibrium fixed request, pre-existing left/right contact
asymmetry, and a later material wheel-rate-sensitive amplification all contribute; the evidence
does not support collapsing the chain to P42-A, B, C or D alone.

## Stable evidence

Formal and fresh replay reproduce the first right contact loss at tick111 with zero Phase41 control
semantic error. Whole-vector dynamics closes to `1.279e-13`, contact reconstruction to zero, and
actual snapshot WBC torque replay to zero. The zero-rate fixed-state effect grows to
`14.07469 m/s²` in wheel-origin acceleration and `1.41770 N` in normal load before loss.

## Scope retained

No controller, WBC task/objective/constraint, request, gain, planner, model, contact, initial state
or safety gate changed. No Phase34 run, reachable ablation, tracking PASS, hardware claim or repair
is included.

## Next authorized work

A new Phase must choose and separately validate the minimum repair architecture against all three
material channels. That technical decision remains open; Phase34 stays frozen until the selected
repair passes its own equilibrium, rate-mode, asymmetry, H0 and regression gates.

