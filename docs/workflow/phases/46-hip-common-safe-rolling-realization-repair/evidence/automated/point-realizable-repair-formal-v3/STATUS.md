# SUPERSEDED

The numerical evidence is valid, but the decision layer reported AUTH as the
mandatory stop even though the frozen equilibrium had already failed
`abs(ddxi_right) <= 0.05`. The corrected decision stops at DG46P-EQ and treats
the already-generated authority result as diagnostic-only. Superseded by v4.
