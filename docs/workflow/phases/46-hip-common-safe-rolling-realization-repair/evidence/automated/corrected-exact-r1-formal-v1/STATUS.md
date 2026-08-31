# Rejected harness result

This run obeyed the COMP stop rule and did not enter EQ, but its COMP verdict is rejected: the
harness counted the unchanged `1e-6` numerical QP regularizer as part of the physical
interaction-wrench task.  The projector, image, full/reduced operators, and point-force
reconstruction all passed in this run.  Formal-v2 corrects only that verification decomposition;
it does not change the candidate, solver, or weights.
