# Phase 19 exact-planar model validation

Result: `PASS`

Commands:

```bash
./.venv/bin/python -m py_compile tools/experiments/build_mujoco_planar_model.py tools/experiments/test_build_mujoco_planar_model.py
./.venv/bin/python tools/experiments/test_build_mujoco_planar_model.py
./.venv/bin/python tools/experiments/build_mujoco_planar_model.py --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-model
./.venv/bin/python tools/experiments/build_mujoco_planar_model.py --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-planar-model-replay
```

The unique direct attribute-free `base_body/freejoint` was replaced by world `X`, world `Z`, and `+Y` pitch joints. Full topology was `nq=17, nv=16, njnt=11`; derived topology is `nq=13, nv=13, njnt=13`. XML transform audit passed, every preserved compiled field had maximum difference `0`, initial body poses had maximum difference `0`, and solver/options had maximum difference `0`.

Primary and replay hashes were exact: derived model `339166fb0eeefade6ffa99bb31a5b48817d40f1588d352ea50e9a3ee01e9945c`, scene `5943c17337c4ed15654ed06f75178c00896844b3a023d47173f3ddc25b1002e9`, and audit `89383d24baac9a480bc82294e85c9aba42a084e58b99abd1435c0128a044d964`. A non-empty output directory was rejected before writing.

No hardware data was used. This proves only the declared base-topology derivation from current nominal MuJoCo physics.
