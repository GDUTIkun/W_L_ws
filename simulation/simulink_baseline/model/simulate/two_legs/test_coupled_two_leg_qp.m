function test_coupled_two_leg_qp()
%TEST_COUPLED_TWO_LEG_QP Phase-08 model-side QP contract regression.

modelDir = fileparts(mfilename("fullpath"));
run(fullfile(modelDir, "startup.m"));
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");
wheelLqr = evalin("base", "wheelLqr");
fullBaseNmpc = evalin("base", "fullBaseNmpc");

frozenWheelWeight = [5; 500];
assert(isequal(ctrl.spatialQpWheelConfigurationWeight, frozenWheelWeight));
assert(~ctrl.differentialDriftStabilizer.enabled);
assert(~evalin("base", "differentialIdentification.enabled"));

wheelReference = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
fullState = [0; fullBaseNmpc.model.xEq; -traj.zO0];
rawWrench = fullBaseNmpc.model.uEq;
x = [fullState; leg.q0; leg.dq0; leg.q0; leg.dq0; ...
    rawWrench; wheelReference];

% --- Test project_uDiffRealizable helper ---
testProjectionHelper();

clear coupled_two_leg_qp_core spatial_two_leg_qp_core
clear differential_drift_stabilizer
[tauBaseline, baseline] = coupled_two_leg_qp_core(x);
assert(numel(tauBaseline) == 6 && all(isfinite(tauBaseline)));
assert(baseline.qpFeasible);
assert(isequal(baseline.wrenchCommand, rawWrench));
assert(baseline.uDiffCorrectionRequested == 0);
assert(baseline.uDiffCorrectionApplied == 0);
assert(baseline.uDiffFinal == baseline.uDiffNominal);
assert(baseline.driftReset && ~baseline.driftFailSafe);

% --- Verify observer-only diagnostics are present ---
assert(isfield(baseline, "uDiffScalarRequested"));
assert(isfield(baseline, "uDiffScalarApplied"));
assert(isfield(baseline, "uDiffScalarFinal"));
assert(isfield(baseline, "uDiffProjectedRequested"));
assert(isfield(baseline, "uDiffProjectedApplied"));
assert(isfield(baseline, "uDiffProjectedFinal"));
assert(isfield(baseline, "uDiffResidualRmsRequested"));
assert(isfield(baseline, "uDiffResidualRmsApplied"));
assert(isfield(baseline, "uDiffResidualRmsFinal"));
assert(isfield(baseline, "qpFeasibleControllerSide"));
assert(isfield(baseline, "plantWrenchUnavailable"));
assert(isfield(baseline, "wbcTaskSensitivity"));
assert(isfield(baseline, "wbcTaskCost"));
assert(isfield(baseline, "wbcTaskResidual"));
assert(isfield(baseline, "wbcTaskAttributionValid"));
assert(isfield(baseline, "wbcTaskGradientClosure"));
assert(isfield(baseline, "wbcRollingFxHierarchyRequested"));
assert(isfield(baseline, "wbcRollingFxHierarchyApplied"));
assert(isfield(baseline, "wbcRollingFxHierarchyStage1Feasible"));
assert(isfield(baseline, ...
    "wbcRollingFxHierarchyStage1RollingAcceleration"));
assert(isfield(baseline, "wbcRollingFxHierarchyLockResidual"));
assert(baseline.qpFeasibleControllerSide == baseline.qpFeasible);
assert(baseline.plantWrenchUnavailable == true);
assert(~baseline.wbcTaskAttributionValid);
assert(isequal(baseline.wbcTaskSensitivity, zeros(16, 1)));
assert(all(isfinite(baseline.wbcTaskCost)) ...
    && all(baseline.wbcTaskCost >= 0));
assert(all(isfinite(baseline.wbcTaskResidual)) ...
    && all(baseline.wbcTaskResidual >= 0));
assert(baseline.wbcTaskGradientClosure < 1e-6);
assert(~baseline.wbcRollingFxHierarchyRequested);
assert(~baseline.wbcRollingFxHierarchyApplied);
assert(~baseline.wbcRollingFxHierarchyStage1Feasible);
assert(baseline.wbcRollingFxHierarchyStage1RollingAcceleration == 0);
assert(baseline.wbcRollingFxHierarchyLockResidual == 0);

% --- Verify signal contract ---
signalContract = coupled_two_leg_qp_signal_contract();
assert(signalContract.legacyWidth == 85);
assert(signalContract.width == 198);
assert(numel(signalContract.appendedNames) == 24);
assert(isequal(signalContract.appendedIndices, (86:198).'));
assert(numel(signalContract.attributionNames) == 16);
baselineSignal = coupled_two_leg_qp_signal(x);
assert(numel(baselineSignal) == signalContract.width);
expectedLegacy = legacyDiagnostics(tauBaseline, baseline);
% Solve time is the sole wall-clock diagnostic and is expected to differ
% across two evaluations; every other legacy value and position is exact.
solveTimeIndex = 54;
legacyComparable = setdiff(1:74, solveTimeIndex);
assert(isequal(baselineSignal(legacyComparable), ...
    expectedLegacy(legacyComparable)), ...
    "Legacy diagnostic columns 1:74 changed ordering or semantics.");
assert(isfinite(baselineSignal(solveTimeIndex)) ...
    && baselineSignal(solveTimeIndex) >= 0);
phase07Expected = [baseline.xiDeltaCanonical; baseline.dxiDeltaCanonical; ...
    baseline.uDiffCorrectionRequested; baseline.uDiffCorrectionApplied; ...
    baseline.uDiffNominal; baseline.uDiffFinal; baseline.uDiffRealized; ...
    double(baseline.driftAmplitudeSaturated); double(baseline.driftRateLimited); ...
    double(baseline.driftFailSafe); double(baseline.driftReset)];
assert(isequal(baselineSignal(75:85), phase07Expected), ...
    "Legacy diagnostic columns 75:85 changed ordering or semantics.");

% --- Verify new audit fields at zero input (baseline) ---
% Indices: 86:89 scalar requested/applied/final/QP-feasible; four 12D
% projected vectors follow; 138:141 residual RMS; 142:143 status flags.
assert(abs(baselineSignal(86)) < 1e-10, ...
    "Scalar requested projection should be near zero at equilibrium.");
assert(abs(baselineSignal(87)) < 1e-10, ...
    "Scalar applied projection should be near zero at equilibrium.");
assert(abs(baselineSignal(88)) < 1e-10, ...
    "Scalar final projection should be near zero at equilibrium.");
assert(baselineSignal(142) == double(baseline.qpFeasible), ...
    "qpFeasibleControllerSide must retain the controller-side flag.");
assert(baselineSignal(143) == 1, ...
    "plantWrenchUnavailable should be 1 (true).");
assert(isequal(baselineSignal(144:159), zeros(16, 1)));
assert(isequal(baselineSignal(160:175), baseline.wbcTaskCost));
assert(isequal(baselineSignal(176:191), baseline.wbcTaskResidual));
assert(baselineSignal(192) == 0);
assert(baselineSignal(193) == baseline.wbcTaskGradientClosure);
assert(isequal(baselineSignal(194:198), zeros(5, 1)));

% The observer-only KKT audit must become finite without changing the QP
% solution when enabled.
attributionCtrl = ctrl;
attributionCtrl.wbcTaskAttributionEnabled = true;
assignin("base", "ctrl", attributionCtrl);
clear coupled_two_leg_qp_core spatial_two_leg_qp_core
[tauAttributed, attributed] = coupled_two_leg_qp_core(x);
assert(norm(tauAttributed - tauBaseline, inf) < 1e-8);
assert(attributed.wbcTaskAttributionValid);
assert(all(isfinite(attributed.wbcTaskSensitivity)));
assert(attributed.wbcTaskGradientClosure < 1e-6);
assignin("base", "ctrl", ctrl);
clear coupled_two_leg_qp_core spatial_two_leg_qp_core

% The pairwise hierarchy is default-off. When enabled it must solve Stage 1,
% apply the rolling lock in Stage 2, and retain the original hard QP
% feasibility without changing any controller setting other than the flag.
hierarchyCtrl = ctrl;
hierarchyCtrl.wbcRollingFxHierarchyEnabled = true;
assignin("base", "ctrl", hierarchyCtrl);
clear coupled_two_leg_qp_core spatial_two_leg_qp_core
[~, hierarchy] = coupled_two_leg_qp_core(x);
assert(hierarchy.wbcRollingFxHierarchyRequested);
assert(hierarchy.wbcRollingFxHierarchyApplied);
assert(hierarchy.wbcRollingFxHierarchyStage1Feasible);
assert(hierarchy.qpFeasible);
assert(abs(hierarchy.wbcRollingFxHierarchyLockResidual) < 1e-8);
hierarchySignal = coupled_two_leg_qp_signal(x);
assert(isequal(hierarchySignal(194:196), ones(3, 1)));
assert(abs(hierarchySignal(198)) < 1e-8);
assignin("base", "ctrl", ctrl);
clear coupled_two_leg_qp_core spatial_two_leg_qp_core

% --- Verify numerical equality of projected quantities ---
% projected = basis * scalar; for equilibrium, scalar~0 => projected~0
contract = contact_consistent_differential_contract();
basis = contract.inputContract.basis(:);
scalarReq = baseline.uDiffScalarRequested;
projReq = baseline.uDiffProjectedRequested;
assert(norm(projReq - scalarReq*basis, inf) < 1e-12, ...
    "Projected requested wrench must equal scalar * basis.");
resReq = baseline.wrenchCommandBeforeIdentification - projReq;
assert(norm(resReq - baseline.uDiffResidualRequested, inf) < 1e-12, ...
    "Residual must equal raw minus projected.");

% --- Enabled drift stabilizer test ---
enabledCtrl = ctrl;
enabledCtrl.differentialDriftStabilizer.enabled = true;
enabledCtrl.differentialDriftStabilizer.Kxi = 10;
enabledCtrl.differentialDriftStabilizer.Kd = 0;
enabledCtrl.differentialDriftStabilizer.rateLimit = 1e6;
assignin("base", "ctrl", enabledCtrl);

xEnabled = x;
xEnabled(1) = 0.1;
xEnabled(14:15) = xEnabled(14:15) + [-0.01; 0.01];
clear coupled_two_leg_qp_core spatial_two_leg_qp_core
clear differential_drift_stabilizer
[~, enabled] = coupled_two_leg_qp_core(xEnabled);
assert(abs(enabled.xiDeltaCanonical - 0.01) < 1e-12);
assert(enabled.dxiDeltaCanonical == 0);
assert(abs(enabled.uDiffCorrectionRequested - 0.1) < 1e-12);
assert(abs(enabled.uDiffCorrectionApplied - 0.1) < 1e-12);
assert(norm(enabled.wrenchCommand - rawWrench ...
    - enabled.uDiffCorrectionApplied*basis, inf) < 1e-12, ...
    "The stabilizer may alter only the frozen rank-1 wrench basis.");
assert(abs(enabled.uDiffFinal - enabled.uDiffNominal ...
    - enabled.uDiffCorrectionApplied) < 1e-12);
assert(enabled.uDiffRealized == ...
    0.5*(enabled.wrenchFeasible(7) - enabled.wrenchFeasible(1)));
assert(all(enabled.wrenchCommand >= fullBaseNmpc.uMin - 1e-12));
assert(all(enabled.wrenchCommand <= fullBaseNmpc.uMax + 1e-12));

% Verify enabled observer diagnostics
assert(abs(enabled.uDiffScalarRequested) < 1e-10, ...
    "Raw controller request must remain unchanged by the stabilizer.");
assert(abs(enabled.uDiffScalarApplied) < 1e-10, ...
    "Pre-stabilizer applied wrench must remain unchanged by the stabilizer.");
assert(abs(enabled.uDiffScalarFinal) > 1e-10, ...
    "Final scalar should be nonzero when drift correction is active.");
assert(norm(enabled.uDiffProjectedFinal, inf) > 1e-10, ...
    "Projected final wrench should be nonzero when drift correction is active.");
assert(enabled.uDiffResidualRmsRequested >= 0);
assert(enabled.uDiffResidualRmsApplied >= 0);
assert(enabled.uDiffResidualRmsFinal >= 0);
assert(enabled.qpFeasibleControllerSide == enabled.qpFeasible);
assert(enabled.plantWrenchUnavailable == true);

% Verify signal adapter with enabled drift
enabledSignal = coupled_two_leg_qp_signal(xEnabled);
assert(numel(enabledSignal) == signalContract.width);
assert(abs(enabledSignal(88)) > 1e-10, ...
    "Signal scalar final should be nonzero when drift correction is active.");
assert(norm(enabledSignal(114:125), inf) > 1e-10, ...
    "Signal projected final should be nonzero when drift correction is active.");

assignin("base", "ctrl", ctrl);
xDisabled = xEnabled;
xDisabled(1) = 0.2;
[~, disabled] = coupled_two_leg_qp_core(xDisabled);
assert(disabled.uDiffCorrectionApplied == 0 && disabled.driftReset);
assert(isequal(disabled.wrenchCommand, rawWrench), ...
    "Disabled configuration must follow the pre-existing command path.");
assert(isequal(ctrl.spatialQpWheelConfigurationWeight, frozenWheelWeight));

% Verify disabled observer diagnostics (should match baseline)
assert(abs(disabled.uDiffScalarRequested) < 1e-10, ...
    "Scalar requested should be near zero when drift correction is disabled.");
assert(abs(disabled.uDiffScalarApplied) < 1e-10, ...
    "Scalar applied should be near zero when drift correction is disabled.");

fprintf("Coupled two-leg Phase-08 QP checks passed.\n");
end

function testProjectionHelper()
%TESTPROJECTIONHELPER Verify project_uDiffRealizable per 08-01-G1 contract.
contract = contact_consistent_differential_contract();
basis = contract.inputContract.basis(:);

% Test 1: Zero input returns zero scalar, zero projected, zero residual
[u, wProj, wRes, rmsRes] = project_uDiffRealizable(zeros(12,1), contract);
assert(u == 0, "Zero input must return zero scalar.");
assert(norm(wProj, inf) == 0, "Zero input must return zero projected.");
assert(norm(wRes, inf) == 0, "Zero input must return zero residual.");
assert(rmsRes == 0, "Zero input must return zero RMS residual.");

% Test 2: Exact basis input returns scalar=1, full projection, zero residual
[u, wProj, wRes, rmsRes] = project_uDiffRealizable(basis, contract);
assert(abs(u - 1) < 1e-12, "Basis input must return scalar=1.");
assert(norm(wProj - basis, inf) < 1e-12, "Basis input must project to itself.");
assert(norm(wRes, inf) < 1e-12, "Basis input must have zero residual.");
assert(rmsRes < 1e-12, "Basis input must have zero RMS residual.");

% Test 3: Scaled basis input returns scalar=scale
[u, wProj, wRes, rmsRes] = project_uDiffRealizable(3*basis, contract);
assert(abs(u - 3) < 1e-12, "Scaled basis must return scalar=scale.");
assert(norm(wProj - 3*basis, inf) < 1e-12);
assert(norm(wRes, inf) < 1e-12);

% Test 4: Orthogonal input returns zero scalar, zero projection, full residual
ortho = zeros(12,1); ortho(2) = 1;  % orthogonal to rank-1 basis
[u, wProj, wRes, rmsRes] = project_uDiffRealizable(ortho, contract);
assert(abs(u) < 1e-12, "Orthogonal input must return zero scalar.");
assert(norm(wProj, inf) < 1e-12, "Orthogonal input must return zero projected.");
assert(norm(wRes - ortho, inf) < 1e-12, "Orthogonal residual must equal input.");
assert(abs(rmsRes - 1/sqrt(12)) < 1e-12, ...
    "Orthogonal unit input must have RMS residual = 1/sqrt(12).");

% Test 5: Sign convention: positive scalar for positive (right-left)/2
wSign = zeros(12,1); wSign(7) = 1; wSign(1) = -1;
[u, ~, ~, ~] = project_uDiffRealizable(wSign, contract);
assert(u > 0, "Positive (right-left)/2 must yield positive scalar.");

% Test 6: Shape rejection — non-12D input
try
    project_uDiffRealizable(ones(11,1), contract);
    error("Should have thrown for 11D input.");
catch e
    assert(contains(e.message, "12-by-1"));
end

% Test 7: NaN rejection
try
    project_uDiffRealizable(nan(12,1), contract);
    error("Should have thrown for NaN input.");
catch e
    assert(contains(e.message, "finite"));
end

% Test 8: Inf rejection
try
    project_uDiffRealizable(inf(12,1), contract);
    error("Should have thrown for Inf input.");
catch e
    assert(contains(e.message, "finite"));
end

% Test 9: Complex input rejection
try
    project_uDiffRealizable(ones(12,1) + 1i, contract);
    error("Should have thrown for complex input.");
catch e
    assert(contains(e.message, "real finite"));
end

% Test 10: Projection normalization — projected component norm <= input norm
rng(42);
wRand = randn(12,1);
[u, wProj, wRes, rmsRes] = project_uDiffRealizable(wRand, contract);
assert(norm(wProj) <= norm(wRand) + 1e-12, ...
    "Projected component must not exceed input norm.");
assert(abs(norm(wRes)^2 + norm(wProj)^2 - norm(wRand)^2) < 1e-10, ...
    "Pythagorean decomposition must hold: ||w||^2 = ||proj||^2 + ||res||^2.");
expectedRms = norm(wRes, 2) / sqrt(12);
assert(abs(rmsRes - expectedRms) < 1e-12, ...
    "RMS residual must equal ||wRes||/sqrt(12).");

% Test 11: Contract equivalence — two calls with same explicit contract
% must return identical results regardless of persistent state.
[u2, wProj2, wRes2, rmsRes2] = project_uDiffRealizable(wRand, contract);
assert(abs(u - u2) < 1e-12, "Same contract must return same scalar.");
assert(norm(wProj - wProj2, inf) < 1e-12, ...
    "Same contract must return same projected.");
assert(abs(rmsRes - rmsRes2) < 1e-12, ...
    "Same contract must return same RMS residual.");

fprintf("project_uDiffRealizable checks passed.\n");
end

function y = legacyDiagnostics(tau, debug)
y = [tau; debug.wrenchSlack; debug.wrenchFeasible; ...
    debug.wrenchSlackNorm; double(debug.qpFeasible); ...
    debug.FcLeft; debug.FcRight; debug.exitflag; ...
    norm(debug.dynamicsResidual, inf); ...
    norm(debug.contactAcceleration, 2); ...
    norm(debug.wrenchResidual, inf); debug.qddBase; ...
    debug.contactResidualDirection; debug.xiCommonError; ...
    debug.dxiCommonError; debug.qpSolveTime; debug.tauDifferential; ...
    debug.xiCommonAcceleration; debug.xiCommonCommand; ...
    abs(debug.xiCommonAcceleration - debug.xiCommonCommand); ...
    debug.frictionMargin; debug.torqueMargin; ...
    debug.xiDifferential; debug.dxiDifferential; ...
    debug.xiDifferentialAcceleration; debug.xiDifferentialCommand];
end
