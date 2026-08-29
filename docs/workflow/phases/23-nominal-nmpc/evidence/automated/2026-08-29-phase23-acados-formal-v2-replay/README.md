# Phase 23 authoritative acados formal v2 fresh replay

Result: `PASS` — 23/23 normal/reference and 10/10 fault cases.

The manifest declares `replay_of=2026-08-29-phase23-acados-formal-v2`.
All 33 plant CSVs are byte-identical to the primary run; all control fields are
identical after removing the four declared wall-clock fields. See the primary
run README for the frozen method, maxima, regressions and scope limit.
