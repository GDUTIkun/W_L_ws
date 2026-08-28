# P21-T07 Runtime Workspace Gate Audit

Resolution: **superseded by the PASS repair in
[runtime_workspace_gate_repair.md](runtime_workspace_gate_repair.md)**. This file preserves
the original blocking observation and must not be read as the current Phase state.

Date: 2026-08-28  
Status: **blocking contract conflict; P21-T07 remains blocked**

## Trigger

The first runtime-independent C++ model golden replay applied the frozen Phase-15
workspace gate before model evaluation. Six fresh MuJoCo/Python golden cases were
accepted and matched the analytic C++ reconstruction/model. The frozen
`dynamic_tick_271` stress case was rejected before reconstruction because its active
coordinates are outside the Phase-15 componentwise workspace.

This is not a compiler, dependency, solver, or MuJoCo environment failure. It exposes
an inconsistency between already-frozen Phase-21 contracts.

## Reproduction and exact values

The authoritative equilibrium canonical active position is:

```text
[-0.97199892, 1.63939575, 0,
 -0.98339094, 1.63940103, 0]
```

Reconstructing the P21-T04/T05 capture at tick 271 gives:

```text
canonical active:
[ 0.24481393, 1.09066968, -0.23665864,
  0.26519486, 1.06668879, -0.26346735]

delta from equilibrium:
[ 1.21681285, -0.54872606, -0.23665864,
  1.24858580, -0.57271223, -0.26346735]
```

The Phase-15 frozen componentwise active workspace is hip `[-0.65, 0.65]`,
knee `[-0.75, 0.75]`, wheel `[-1.0, 1.0]`. Both hip deltas therefore exceed
the permitted envelope. `NominalWbcModel` correctly returns
`kOutsideWorkspace` for this case.

## Conflicting frozen authorities

- `evidence/reduced_model_contract.md` requires fail-closed rejection of any state
  outside the Phase-15 frozen workspace.
- `evidence/hard_feasibility_42d.md` says those workspace checks remain applicable,
  but also calls the unchanged 4-workspace + 28-dynamic corpus `32/32` feasible.
- `dynamic_tick_271` is explicitly treated as a nominal case and is the tightest
  torque/cone/acceleration-margin case.
- `evidence/p21_t07_implementation_handoff.md` requires exact runtime parity on the
  entire frozen 32-case corpus.

The offline P21-T04/T05 builder reconstructed tick 271 without enforcing the
Phase-15 workspace gate. QP feasibility at that state therefore cannot establish
that the production runtime is authorized to evaluate it.

## Work completed before the gate

- Deterministic runtime profile generation passes two fresh byte-identical runs;
  profile SHA-256 is
  `280e84918438922d55b515f94cfe31bd89ec4e401626db6489e8baccde17c78c`.
- Dependency probe used `./.venv/bin/python` with MuJoCo `3.7.0`, NumPy `2.2.6`,
  SciPy `1.15.3`; exporter and golden exporter pass `py_compile`.
- All 11 compiled body inertials match the Phase-14 parameter manifest at maximum
  absolute error `0`; the changed current `wheel_leg.xml` hash is recorded rather
  than hidden.
- Analytic C++ results for four workspace cases and dynamic ticks 68 and 204 match
  fresh MuJoCo/Python golden data. Maximum observed errors before the blocking case
  are: native reconstruction `6.72e-10`, reduction `1.77e-10`, mass `3.34e-12`,
  bias `2.04e-11`, wrench/contact Jacobian `4.44e-15`, contact bias `8.39e-13`.
- The test explicitly expects fail-closed rejection for tick 271. This is a fault
  contract check, not acceptance of missing 32-case parity.

## Required decision gate

P21-T07 cannot be closed until Codex re-audits the full 32-case corpus and freezes
one consistent route:

1. classify every out-of-workspace captured state as rejection/fault coverage and
   replace the production parity corpus with a predeclared in-workspace corpus; or
2. expand the reconstruction workspace using fresh branch/closure/conditioning,
   model/contact, hard-QP, weighted-task and nonlinear evidence.

Route 2 is not authorized by the existing evidence. Silently bypassing the runtime
workspace gate, relabeling tick 271 after seeing the result, or claiming exact
32-case parity while rejecting it are all forbidden.
