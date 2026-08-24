# Stage 1 flat-ground performance study

This study adds repeatable full-plant Simscape runners for Stage 1 flat-ground
tracking, anti-split recovery, and physical body-force disturbances.

## Entry points

- `run_stage1_straight_cases([0.10 0.20], 16)` runs start/hold/brake cases.
- `run_stage1_split_cases([-0.010 0.010], 8)` runs anti-split OFF/ON pairs.
- `run_stage1_disturbance_cases(filter, ratios, 0.5, 12)` runs World-frame
  body-force pulses. Physical X is forward, Y is vertical, and Z is lateral.
- Turning tests are paused by user instruction. Do not launch continuous-turn
  cases unless the user explicitly re-enables them.

The functions write their summary and representative time-series CSV files
to the MATLAB current directory. Run them from a dedicated results folder;
do not run them from this source folder if the outputs should remain ignored.

## Current evidence run

The 2026-08-22 evidence is under:

`calibration/results/2026_08_stage1_performance/`

The engineering verdict and tuning record are maintained in the Research
vault at `projects/proformace_test/analysis/阶段一性能测试与调参记录.md`.

## Closeout status

The 2026-08-22 model is closed as **Stage 1 not accepted** before switching
to a new plant model:

- accepted evidence: straight 0.10/0.20 m/s and +/-10 mm anti-split OFF/ON;
- turning: historical 90-degree cases passed, long-turn acceptance failed,
  and all further turning runs are paused;
- disturbance: `final_disturbance_stand_x` completed but is non-monotonic,
  so it is not a publishable stability boundary;
- `final_disturbance_stand_z` was interrupted after partial raw output;
- `final_disturbance_straight_x` and `_z` were not run.

Do not merge partial disturbance directories into a summary or infer missing
cases. The new model must re-establish its own coordinate contract, baseline,
and stability envelope.

## Interpretation notes

- The split runner uses the canonical sign
  `xiDelta = (xiRight - xiLeft)/2`.
- Simscape assembly and contact constraints alter raw joint assembly targets.
  The runner therefore records the realized initial split and applies a
  symmetric compensation identified from a pilot run.
- `forceAppliedReplay` is a deterministic replay of the plant-side limiter
  from logged canonical split states. The current model logging adapter does
  not expose the internal plant-side anti-split diagnostics directly.
- The test copy of `source.slx` contains a torso `External Force and Torque`
  chain driven by workspace variable `simin`; `startup.m` supplies a default
  zero-force command for legacy runs.
- Stability-boundary sweeps must use isolated initialization and repeated
  cases. A smaller failed pulse followed by a larger passing pulse is a
  repeatability warning, not evidence of a non-conservative envelope.
