# Reduced-QP / Full-Constrained-Dynamics Equivalence Audit

## Decision

Classification: `B-REDUCED-QP-VALID-DIAGNOSTIC-RECONSTRUCTION-INVALID`.

The actual `R46E-H0` runtime solved the unchanged 42-variable production QP.
Read-only fields exposed the exact affine lift and full-tree dynamics used by
`NominalWbcModel`; all pre-existing controller CSV fields remained bitwise
unchanged.

## Reduction and dual spaces

The production lift is `qdd_tree = N * nudot + c_N`, with full dimension 16
and reduced dimension 12. At compatible-H0, `rank(N)=12`, while the production
closure Jacobian has `rank(J_eq)=4`, so `dim Null(J_eq)=12`.

- `||J_eq N||_2 = 2.4685242385838613e-13`
- affine closure max residual: `0`
- `Range(N) -> Null(J_eq)` containment: `9.234010097690393e-16`
- reverse containment: `1.5223876254179994e-15`
- projector difference: `1.7515402541706546e-15`
- dual projector difference: `1.7399534719088365e-15`

Thus `Range(N)=Null(J_eq)` and `Null(N^T)=Range(J_eq^T)` to numerical
precision. The earlier six-row MuJoCo site Jacobian must not be substituted for
the rank-four production closure operator when judging the production reduction.

## Full dynamics lift

Using the production sign convention
`M_full qdd + h_full - B_full tau - Q_contact - Q_eq = 0`, the runtime lifted
solution has projected-dynamics max residual `4.482018312046421e-9`. The
unbalanced full residual norm is `0.13067173520843547`; its equality-range
orthogonal fraction is `3.438791046560477e-8`, with absolute reconstruction
residual `4.482018312046421e-9`, the same solver-feasibility scale as the
projected residual. Legal reaction recovery and algebraic consistency pass;
nullspace virtual-work residual is `1.0560634270546873e-16`.

## Optimization equivalence

The diagnostic full oracle uses the exact affine bijection
`nudot=N^+(qdd-c_N)` under the production closure constraint, leaving every
other production variable, constraint, scaling, task, slack, and objective term
unchanged. Reduced/full qacc, torque, physical contact wrench, slack, task
residual, active set, total reaction, and objective differences are zero.
The latent optimum is non-unique but physically equivalent; KKT comparison is
not required after the exact affine identity and primal/objective closure.

## Historical reaction reconstruction reconciliation

The historical diagnostic range residual `0.999233` remains valid evidence that
that post-hoc reconstructed reaction was non-physical. It does not invalidate
the production reduced QP. The historical diagnostic reaction is superseded as
an interpretation of the controller formulation; corrected-R1 remains closed,
and this read-only audit caused no R1 regression.

Explicit-lambda controller repair and R2 remain unauthorized. The next allowed
action is diagnostic/reaction-reporting repair only.
