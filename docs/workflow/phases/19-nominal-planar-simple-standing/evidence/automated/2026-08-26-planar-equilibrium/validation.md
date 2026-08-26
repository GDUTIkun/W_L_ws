# Phase 19 exact-planar equilibrium validation

Result: `PASS`

Commands:

```bash
./.venv/bin/python -m py_compile tools/experiments/solve_mujoco_planar_equilibrium.py
./.venv/bin/python tools/experiments/solve_mujoco_planar_equilibrium.py
./.venv/bin/python tools/experiments/solve_mujoco_planar_equilibrium.py --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-equilibrium-replay
```

The deterministic compliant-contact/equality solve converged in four iterations. Both wheel actuator commands were exactly zero. Maximum acceleration was `9.814824688027055e-11`, maximum generalized-force residual was `2.2681945210933918e-10`, and one-step position/velocity drift was `3.885780586188048e-16 / 1.9631156670847962e-13`.

The six actual-wheel mesh contacts carried positive left/right normal loads of `22.357688848898487 N / 40.763775151328346 N`. Maximum closed-chain equality displacement was `0.00015865773519990745 m`, below the frozen `0.0002 m` compliant-equilibrium limit. Maximum support torque was `4.428369830750493 Nm`.

The current nominal CAD model is not perfectly left/right symmetric. Exact zero-acceleration equilibrium therefore uses side-specific active references: native hip difference `0.019079212258660716 rad` and knee difference `0.001357335037021401 rad`, both within the frozen `0.03 rad` limit. Enforcing identical native references leaves a nonzero acceleration residual and is not used as evidence.

Primary and fresh-process replay hashes were exact:

- `equilibrium.json`: `6d30377af2f817ce26cfc2f59d2daad23cc51275e188102790ea81ca2abca9cf`
- `solver_trace.json`: `701d668dd559598f0724d15a1a2304d56023e779a9ca42f890ad6b8d9936d97b`
- `summary.json`: `249879aba746449929b1d2e7a1667af7327f8440dcb1d7b5bbfb4be2ad00c212`

A non-empty primary directory was rejected with exit code `2`. No hardware data was used.
