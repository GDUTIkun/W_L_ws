# Phase 42 Wheel-Spin / Contact-Loss Causal Attribution

## Purpose

On the frozen Phase 41 production H0, attribute the wheel-rate drift and tick111 right-wheel
contact loss without changing the controller, request, gains, model, contact, initial state or stop
gates.

## Frozen entry

```bash
./.venv/bin/python tools/experiments/run_phase42_causal_attribution.py \
  --output OUTPUT
```

The method is `simulation/mujoco/config/phase42_causal_attribution_v1.json`. Run only after the
same interpreter imports MuJoCo 3.7.0, NumPy and SciPy and after the dedicated ROS target builds.

## Outputs and gates

- `control-{a,b}.csv`: unchanged Phase41 control schema and two-run semantic replay.
- `control-{a,b}_native.csv`: pre-command, post-command-instantaneous and five stepped native
  snapshots per control interval.
- `plant.csv`: restored MuJoCo state, full mass/force balance, wheel coordinates and per-side
  aggregate contact truth.
- `contacts.csv`: one row per wheel-floor contact with topology, frame, signed wrench, slip and
  independently reconstructed generalized force.
- `events.json`, `wheel-row-balance.csv`, `zero-rate-counterfactual.json`: chronology, mechanism
  and fixed-state intervention evidence.

PASS requires tick111 Phase41 parity; whole-vector dynamics/contact/oracle closure; exact WBC
snapshot replay; a valid wheel-rate-only intervention; and a resolved P42 classification. The run
stops at first contact loss and does not authorize a repair or Phase34.

