# Phase 22 authoritative formal v2 fresh replay

Result: `PASS` — 19/19 normal/perturbation and 6/6 fault cases.

The replay uses the same frozen config chain, ProxQP solver profile, runner,
scene and sources as `2026-08-28-formal-v2`. All 25 plant CSVs are byte-exact;
control output differs only in the declared `core_step_ns` wall-clock column,
and the summary is identical after removing `maximum_core_step_ms`.

The replay manifest has 71 independently rechecked hash entries and an
unambiguous ProxQP-only solver identity. It is the replay authority paired with
`2026-08-28-formal-v2`.
