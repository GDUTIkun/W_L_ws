# Phase 35 hold attribution

Both fresh H0 replays are numerically identical apart from measured solve time and reject at tick 88
(`0.88 s`). Both H1 replays are likewise identical and reject at tick 89. In every run canonical
index 5 (`right_wheel`) crosses the lower `-1 rad` delta bound first.

H0 therefore closes the earliest branch before any xi target or feedback exists:
`P35-A_pre_target_minimal_wbc_workspace_drift`. H1 cannot supersede H0 and excludes the zero-ddxi row
as a necessary cause. DG35-02: **PASS / P35-A**.
