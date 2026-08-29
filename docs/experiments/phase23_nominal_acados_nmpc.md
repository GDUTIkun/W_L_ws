# Phase 23 nominal acados NMPC validation

## Scope

This method validates the current nominal MuJoCo-only controller stack:
12-state locked-composite NMPC, generated acados SQP-RTI/HPIPM, the existing
12-wrench NMPC-to-WBC boundary, ProxQP Weighted WBC, and the frozen nominal
plant. It does not claim identified-plant, real-machine, turning, terrain, or
target-hardware real-time validity.

## Frozen inputs

- OCP: `simulation/mujoco/config/phase23_acados_ocp_v2.json`
- reference/cost/constraints: `phase23_acados_t05_profile_v1.json`
- formal matrix: `phase23_acados_formal_v2.json` (23 normal/reference and 10
  fault cases)
- generated artifact:
  `ros_ws/src/wheel_leg_core/acados_generated/phase23_nominal_nmpc_v2/`
- acados: `/home/t/opt/acados`, commit
  `21376cb1af6b7dd45f675367272d3ba8100b26c0`

The v2 artifact adds the frozen relative state envelope at stages 1 through N;
stage 0 remains the exact measured state. Ordinary builds compile checked-in C
and do not invoke Python, CasADi, `acados_template`, the renderer, or a network.

## Required preflight

Use `./.venv/bin/python` to import MuJoCo, NumPy, SciPy, CasADi and
`acados_template`, then `py_compile` the generator, model validator and formal
evaluator. Verify `ldd` resolves `libacados`, `libhpipm` and `libblasfeo` from
the frozen prefix. Generate only into a nonexistent or empty directory and
compare two clean generations after normalizing the absolute code-export path
and its derived acados JSON hash.

Build from `ros_ws` with Release, tests enabled and
`-DACADOS_ROOT=/home/t/opt/acados`. The component gates cover cold/warm/reset,
3x1000 solves, invalid and infeasible OCP inputs, 2:1 scheduling, wrench ZOH,
NMPC-before-WBC injection, and solver/late/stale/non-finite fail-zero/latch.

## Formal and replay

Run `tools/experiments/run_mujoco_weighted_wbc_formal.py` once with the v2
config and once with the append-only replay config, each into a new directory.
All inherited WBC/plant gates remain active. Additional gates require acados
success, even-tick update/odd-tick age-one ZOH, combined update time below
10 ms, residual/defect/projected-stationarity limits, zero input/state bound
violation, tracking/recovery, and four NMPC fault paths.

Primary and replay plant CSVs must be byte-identical. Control CSVs may differ
only in `core_step_ns`, `nmpc_preparation_s`, `nmpc_feedback_s`, and
`nmpc_wbc_total_s`; summaries may additionally differ in profile identity and
the derived `maximum_core_step_ms`. A run pointed at a non-empty directory must
fail before simulation.

Finally run fresh Phase 14/15/18/20 entries, the coordinate contract, current
Core/ROS/Adapter tests, and verify Phase 21/22 configs and evidence remain
unmodified.
