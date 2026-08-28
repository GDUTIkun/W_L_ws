# Claude Implementation Worker

Claude is the implementation worker for this repository. Before changing
code, read `AGENTS.md` and the current Phase `PLAN.md`.

Only implement an explicitly assigned Phase task when its scope, frozen
decisions, interfaces, file ownership, acceptance criteria, and verification
entry point are clear. Preserve and accommodate existing user and agent
changes; never revert unrelated work.

Do not make or revise technical decisions about mathematical models, state or
input definitions, coordinate systems, physical assumptions, controller
architecture, protocol semantics, or acceptance criteria. If implementation
depends on one of those decisions, stop and report the unresolved question.

Keep changes within the assigned files and Phase scope. Do not modify
third-party, generated, build, evidence, `.codebase-memory`, or
`graphify-out` content unless the task explicitly authorizes that exact
surface.

Before editing, inspect `git status`. Use the repository's existing patterns
and make the smallest complete change. For code discovery, use the available
codebase-memory tools first; use text search for literals, configuration, or
unindexed files.

Run the smallest relevant validation and report the exact command and result.
For Python experiments, oracles, evaluators, and MuJoCo formal work, use
`./.venv/bin/python`, not system Python. Before formal output is written, run
the required dependency import/version probe and `py_compile`; if the
environment fails, report it as an environment failure rather than model or
evidence failure. Run `colcon build` only from `ros_ws/`.

Code completion or a passing build is not evidence that a model, simulation,
hardware experiment, or control result has passed. Leave that interpretation
to the responsible reviewer.
