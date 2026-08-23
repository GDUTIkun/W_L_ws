Read the Phase design document and create a codebase-grounded implementation specification.

# Grounding Task

## 1. Role and Scope

- Ground the approved design into the current codebase.
- Do not redesign the solution.
- Do not generate PLAN.md.
- Do not implement code.
- Produce only the grounding artifact; do not generate an intermediate plan-input.md.

## 2. Codebase Exploration

Use Code Base Memory for targeted lookup only.

For relevant implementation areas, identify:

- file/path
- symbol/function/subsystem
- current responsibility
- relevant callers/callees
- reusable infrastructure
- required change
- interfaces/behavior to preserve

Exploration rules:

- Start from files/symbols mentioned in the design.
- Prefer <=10 targeted CBM queries.
- Expand only when a required implementation path is unresolved.
- Do not perform repository-wide exploration.
- Stop once entry point, core implementation, dependency path, modification surface, and validation path are clear.

## 3. Historical Knowledge Grounding

- Use Graphify selectively when the Phase depends on previous engineering work, modeling decisions, experimental conclusions, rejected approaches, or design rationale.
- Use Graphify to answer:
  - why a previous design decision was made;
  - what prior experiments already established;
  - what historical failure modes constrain the current Phase.
- Prefer targeted retrieval only; do not broadly explore the knowledge graph.
- Use roughly 3–6 targeted Graphify queries when historical context is materially relevant.
- Preserve only historical findings that affect current decisions, constraints, validation requirements, known failure modes, or execution routing.
- Do not let historical notes silently override an explicitly approved current design. If they reveal a genuine contradiction, preserve it as a technical decision gate.

Tool roles:

- Graphify = historical engineering knowledge / design rationale.
- Code Base Memory = current code structure / symbol relationships / impact surface.

## 4. Question Handling

- If a question can be answered from code, resolve it.
- If it requires simulation/data, do not guess. Convert it into a validation task or decision gate.

## 5. Execution Routing

Classify remaining work by whether the technical specification is frozen.

### CODEX

Use CODEX when the task requires deciding, revising, or interpreting:

- mathematical model structure
- state or input definition
- coordinate / sign convention
- physical assumptions
- control architecture
- identification or algorithm structure
- experimental evidence that may change the technical design
- unresolved technical decision gates

CODEX owns technical decisions, not routine production implementation.

CODEX may inspect code, run bounded diagnostic probes, derive or audit models, interpret evidence, and freeze the technical specification.

Routine MATLAB/Simulink implementation, adapters, runners, pipelines, and regression tests belong to CC once the specification is frozen.

If executable work is necessary only to resolve a technical decision, CODEX may use a minimal bounded decision-support probe, but should not turn it into the production implementation.

### CC

Use CC only when the technical specification is frozen and the remaining work is implementation or bounded execution.

Typical CC work:

- MATLAB implementation
- modification of existing code
- interface adaptation
- logging and test implementation
- data processing
- bounded smoke / short simulation execution
- preparation of manual full-batch runners
- regression fixes
- implementation-level debugging
- Use `flash-high` by default.
- Use `flash-max` when the design is frozen but implementation is materially more difficult, such as:

  - complex multi-file changes
  - difficult MATLAB debugging
  - complex validation pipelines
  - Simulink / Simscape integration
  - substantial regression work
- Do not assign `pro-max` automatically.
- If a CC task requires changing a frozen technical decision, stop and escalate instead of redesigning it.

### Execution Knowledge Access

- CODEX decision/reasoning tasks may use both Graphify and Code Base Memory as needed.
- CC implementation tasks should not use Graphify by default.
- CC may use Code Base Memory for implementation-level inspection.
- If CC discovers that implementation requires changing a frozen technical decision, it must stop and escalate the issue instead of using historical notes to redesign it.

## 6. Simulink / Simscape Rules

For Simulink/Simscape internals:

- Do not invent block names.
- Mark grounding confidence as HIGH / MEDIUM / LOW.

## 7. Output Artifact

Output to:

./planning_input/grounding/<Phase Name></phase>--grounding.md

### Required Structure

```markdown
# Phase: <Phase Name>

## 1. Phase Goal

## 2. Approved Design Decisions

## 3. Codebase Grounding

For each implementation area:

- Confidence:
- Files:
- Symbols:
- Current behavior:
- Relevant dependency path:
- Reusable infrastructure:
- Required change:
- Must preserve:

## 4. Required Implementation Changes

## 5. Interfaces / Constraints

## 6. Resolved Codebase Questions

## 7. Evidence-Dependent Questions / Decision Gates

## 8. Validation

### Agent-Automated

### Manual Full-Batch Gate

## 9. Acceptance Criteria

### Implementation Completion

### Evidence / Model Approval

## 10. Documentation Handoff

## 11. Execution Groups

## 12. GSD Planning Rules
```

### Execution Group Schema

For `## 11. Execution Groups`, use the following schema for every execution group:

GROUP_ID: G<N></n>

EXECUTION_OWNER: CODEX | CC

For CC only:

EXECUTION_TIER: flash-high | flash-max

Depends on:

- <GROUP_ID or none>

Frozen decisions:

- <technical decisions this group must not change>

Grounded implementation surface:

- Files:
  - <path>
- Symbols:
  - <symbol>

Tasks:

- <task 1>
- <task 2>

Deliverable:

- <deliverable>

Escalate if:

- <condition>

## 8. GSD Planning Rules

### General Planning Rules

- Technical design is already approved.
- Do not redo research or architecture design.
- Do not expand scope.
- Use the grounded files/symbols above.
- Preserve evidence-dependent questions as validation gates.
- Split PLANs at execution-routing boundaries.
- Prefer one PLAN only when the grouped work shares the same `EXECUTION_OWNER` and compatible `EXECUTION_TIER`.
- Separate `flash-high` and `flash-max` work when their implementation complexity materially justifies separate execution.
- Prefer the minimum number of PLANs needed to preserve routing. Typically this is 1–3 PLANs, but do not merge incompatible routing groups merely to satisfy a PLAN-count target.
- Preserve `GROUP_ID`, `EXECUTION_OWNER`, dependencies and `EXECUTION_TIER`.
- Keep tasks coarse-grained.
- Full expensive simulations are manual checkpoints.
- Reuse the existing `XX-CONTEXT.md` when it remains consistent with the supplied grounding. Do not regenerate CONTEXT merely because PLANs are being replanned.
- `files_modified` must list only files the plan actually intends to modify. Files that are read, inspected, referenced, or used as dependencies belong in `<context>`, not in `files_modified`.
- Do not add STRIDE/threat-model sections for ordinary local MATLAB/modeling plans unless the phase has a real security, trust-boundary, external-input, or deployment concern.

### CODEX Plan Rules

- Create a normal local-executor PLAN.
- Do not set `cross_ai: true`.
- Preserve `EXECUTION_OWNER: CODEX` in the PLAN objective.
- End CODEX PLANs with a frozen decision/specification for downstream CC.
- Do not assign routine production implementation to CODEX.

### CC / Cross-AI Plan Rules

- Create a separate Cross-AI PLAN.
- Set PLAN frontmatter:
  `cross_ai: true`
- Inside `<objective>`, include the literal routing tags:
  - `EXECUTION_OWNER: CC`
  - `EXECUTION_TIER: flash-high` or `EXECUTION_TIER: flash-max`
- Do not rely on frontmatter alone.
- Do not merge CC implementation with CODEX decision work.
- Do not invent or upgrade execution tiers during GSD planning.

### Cross-AI Plan Sizing

Cross-AI plans must be independently completable within one bounded executor run.

Split a CC execution group when any of these are true:

- it contains more than 2 substantial implementation tasks;
- it modifies more than ~4 production files across different responsibilities;
- it combines Simulink/model wiring with analysis-pipeline implementation;
- it combines implementation with expensive smoke/validation;
- one task cannot reach a testable atomic commit before the next major responsibility begins.

Prefer:

- implementation → commit
- validation/smoke → separate PLAN
- human expensive batch → separate checkpoint PLAN

Do not increase PLAN size merely to minimize PLAN count.
Execution reliability takes priority over minimum PLAN count.

- Size Cross-AI plans to complete comfortably within the configured wall-clock timeout,
  including bounded tests and smoke execution.
- If MATLAB/Simulink runtime is expected to consume a substantial portion of that timeout,
  split implementation and smoke/validation into separate PLANs.
- Do not rely on increasing `max-turns` when wall-clock runtime is the actual limiting factor.

### Cross-AI Commit Discipline

For CC plans:

- Each substantial task must end in a verified atomic commit before starting the next task.
- Do not defer all commits or SUMMARY work until the end of the executor run.
- After a task passes its bounded verification:
  1. inspect scoped diff;
  2. commit that task;
  3. continue to the next task.

If max-turns is reached later, already verified work must remain recoverable from commits.

## 9. Output Size

Target:

- Prefer 150-250 lines.
- Preserve hard constraints and acceptance criteria.
- Remove historical reasoning, failed-design narratives, duplicated verification, and unrelated dependency details.

## 10. Completion Report

When complete, report:

Grounding complete.
Output:
./planning_input/grounding/<Phase Name></phase>--grounding.md
