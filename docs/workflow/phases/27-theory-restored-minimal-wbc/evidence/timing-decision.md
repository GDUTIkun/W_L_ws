# Phase 27 timing decision

Date: 2026-08-29

Decision: `DG27-04 PASS — retain 2/10/20 ms`

## Frozen schedule

Phase 27 retains the approved current-nominal integer schedule:

```text
physics: 2 ms
WBC:    10 ms = 5 physics steps
NMPC:   20 ms = 2 WBC ticks = 10 physics steps
```

NMPC runs synchronously on WBC phases `0,2,4,...`. A successful result has
age zero on its solve tick and age one on the following WBC tick. Any missing,
failed, non-finite, late or age-greater-than-one result produces exact zero
torques and latches until reset. Reset clears solver warm state, accepted
wrench, age and phase; the first valid tick is phase-zero/cold.

The command accepted on a WBC tick is held exactly over the following five
MuJoCo steps. Plant disturbance selection occurs before each physics step and
is keyed to the enclosing WBC tick. Source, receipt and command timestamps use
the same integer phase lattice; replay ignores only measured wall-clock
`core_step_ns`.

The WBC and synchronous NMPC+WBC deadline is `10 ms`. The 20 ms NMPC period is
not permission to exceed the 10 ms actuation deadline. T06 must audit the new
16-state generated solver against this frozen deadline; failure blocks that
artifact and does not silently switch the plant schedule.

## Numerical comparison

The Release runner gained an opt-in comparison-only timing profile. It first
validates the original 2 ms XML through Adapter, then may set the local model
copy to 1 ms; Adapter invariants and the production default were not relaxed.
The `1/5/20` profile is refused for the existing nominal-NMPC mode because its
legacy `%2` schedule would mean 10 ms, not 20 ms.

`timing-comparison-v1` is retained as FAIL. Its ad-hoc `10 N + 1 Nm / 0.2 s`
disturbance caused bilateral contact loss in both profiles, and its first
evaluator wrote an invalid positive decision sentence despite `pass=false`.
It is not decision evidence.

Superseding `timing-comparison-v2` uses the already frozen Phase 21
`combined_positive` disturbance for the same physical `0.1 s`, plus hold and
fresh replay. Both profiles pass the identical Phase 21 plant/control gates:

| Metric | 2/10/20 | 1/5/20 |
| --- | ---: | ---: |
| max WBC Core step | `0.9561 ms` | `0.9631 ms` |
| p99 WBC Core step | `0.4243 ms` | `0.4645 ms` |
| deadline miss ratio | `0` | `0` |
| min normal load | `31.3676 N` | `31.3767 N` |
| max penetration | `0.5147 mm` | `0.5143 mm` |
| max rolling slip | `0.00743 m/s` | `0.00517 m/s` |
| max lateral slip | `0.001544 m/s` | `0.001538 m/s` |
| max closure residual | `0.183315 mm` | `0.183244 mm` |

Control replay is exact after removing only `core_step_ns`; plant CSV is byte-
value exact for both profiles. The candidate's small contact-metric change does
not close a failed gate or establish a control benefit, while it doubles both
physics and WBC execution load. Therefore the minimal approved choice is to
retain `2/10/20 ms`. The current Phase 23 component run in the same Release
build reported a maximum dynamic NMPC path of `2.4213 ms`, already below the
same 10 ms combined deadline; the new artifact remains independently gated in
T06.
