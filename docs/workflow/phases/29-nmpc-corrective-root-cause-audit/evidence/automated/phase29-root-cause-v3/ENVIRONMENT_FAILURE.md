# Phase 29 replay-v3 environment failure

Status: `ENVIRONMENT_GATE_FAIL`

The evaluator created this new non-overwrite output root, then stopped before
producing semantic output because `libhpipm.so` was not available on the
invoking shell's dynamic-library path. This is not model, solver, controller or
classification evidence.

The retry used the frozen `/home/t/opt/acados` source/library environment and
wrote to the new `phase29-root-cause-v4` directory. No v1, v2 or v3 file was
overwritten.
