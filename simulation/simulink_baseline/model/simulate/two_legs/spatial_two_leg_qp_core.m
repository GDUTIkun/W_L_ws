function [tau, debug] = spatial_two_leg_qp_core(x)
%SPATIAL_TWO_LEG_QP_CORE 12-DoF inverse-dynamics QP for both wheel legs.
%
% Input layout:
%   [fullNmpcState(18); qL(3); dqL(3); qR(3); dqR(3); ...
%    upperWrench(12); wheelReference(4)]
% fullNmpcState = [t; pB(3); roll; pitch; yaw; vB(3); omegaB(3); ...
%                  xiL; xiR; dxiL; dxiR; wheelHeight], where the angular
%                  velocity states are Euler rates [rollDot;pitchDot;yawDot].
%
% Generalized coordinates are [pB(3); roll; pitch; yaw; qL(3); qR(3)].
% The QP retains all six floating-base equations and three contact-force
% components per wheel.  Joint axes remain sagittal; underactuation is
% represented by the actuator selection matrix rather than deleting the
% roll/yaw/lateral equations.

persistent qpOptions zWarm realizableInputContract projectionContract

if (ischar(x) || (isstring(x) && isscalar(x))) ...
        && strcmpi(string(x), "contact-audit")
    tau = zeros(6, 1);
    debug = contactAuditSnapshot();
    return;
end

x = double(x(:));
if numel(x) ~= 46
    error("spatial_two_leg_qp_core:InvalidInput", ...
        "Expected the 46D spatial two-leg controller input.");
end

leg = evalin("base", "leg");
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");
fullBaseNmpc = evalin("base", "fullBaseNmpc");

t = x(1);
state = x(2:17);
qLeft = x(19:21);
dqLeft = x(22:24);
qRight = x(25:27);
dqRight = x(28:30);
rawWrenchCommand = x(31:42);
[wrenchCommand, identificationExcitation] = ...
    differential_identification_wrench(t, rawWrenchCommand);
wrenchCommand_before_correction = wrenchCommand;
wheelReference = x(43:46);
nq = 12;
[rollingSpeed, rollingSpeedReference, rollingAccelerationCommand, ...
    rollingTaskWeight, JrollingCommon, rollingForceCorrection, ...
    rollingForceOverrideEnabled] = ...
    commonRollingSpeedTask(t, state, ctrl, nq, fullBaseNmpc.model.m);
smallSignal = sagittal_small_signal_excitation(t, ctrl);
rollingAccelerationCommand = rollingAccelerationCommand ...
    + smallSignal.rollingTaskAcceleration;
xiCommon = 0.5*(state(13) + state(14));
dxiCommon = 0.5*(state(15) + state(16));
xiDifferential = 0.5*(state(13) - state(14));
dxiDifferential = 0.5*(state(15) - state(16));
% The legacy WBC state uses (left-right)/2. The Phase-07 controller contract
% is explicitly the opposite, (right-left)/2, using the direct rate states.
xiDeltaCanonical = -xiDifferential;
dxiDeltaCanonical = -dxiDifferential;
driftConfig = getField(ctrl, "differentialDriftStabilizer", ...
    disabledDriftConfig(ctrl.Ts));
[uDiffStab, driftDiagnostics] = differential_drift_stabilizer( ...
    t, xiDeltaCanonical, dxiDeltaCanonical, driftConfig);
if isempty(realizableInputContract)
    frozenContract = contact_consistent_differential_contract();
    realizableInputContract = frozenContract.inputContract;
end
stabilizedWrenchCommand = wrenchCommand ...
    + uDiffStab*realizableInputContract.basis(:);
if rollingForceOverrideEnabled
    rollingForceCorrection = rollingForceCorrection ...
        - sum(stabilizedWrenchCommand([1, 7]));
end
stabilizedWrenchCommand([1, 7]) = ...
    stabilizedWrenchCommand([1, 7]) + 0.5*rollingForceCorrection;
stabilizedWrenchCommand([1, 7]) = ...
    stabilizedWrenchCommand([1, 7]) + 0.5*smallSignal.commonRollingForce;
% Retain the existing final per-channel NMPC command bounds after the sole
% rank-1 correction. No independent force or axle-moment path is added.
wrenchCommand = min(max(stabilizedWrenchCommand, ...
    fullBaseNmpc.uMin(:)), fullBaseNmpc.uMax(:));
uDiffNominal = 0.5*(rawWrenchCommand(7) - rawWrenchCommand(1));
uDiffFinal = 0.5*(wrenchCommand(7) - wrenchCommand(1));
upperDerivative = full_base_body_dynamics( ...
    state, wrenchCommand, fullBaseNmpc.model, leg);
if getField(ctrl, "paperWheelAccelerationFeedforwardEnabled", false)
    % The upper NMPC predicts each wheel-relative acceleration from the
    % paper's wheel mass/inertia and no-slip dynamics. The lower WBC must
    % realize that same motion instead of fighting the selected wrench.
    xiCommonFeedforward = 0.5*(upperDerivative(15) ...
        + upperDerivative(16));
    xiDifferentialFeedforward = 0.5*(upperDerivative(15) ...
        - upperDerivative(16));
else
    xiCommonFeedforward = wheelReference(3);
    xiDifferentialFeedforward = 0;
end
xiCommonFeedforward = xiCommonFeedforward ...
    + smallSignal.wheelRelativeAcceleration;
xiCommonError = xiCommon - wheelReference(1);
dxiCommonError = dxiCommon - wheelReference(2);
xiDifferentialError = xiDifferential;
dxiDifferentialError = dxiDifferential;
xiCommonCommand = xiCommonFeedforward ...
    - getField(ctrl, "commonWheelPositionKp", 0)*xiCommonError ...
    - getField(ctrl, "commonWheelPositionKd", 0)*dxiCommonError;
xiDifferentialCommand = xiDifferentialFeedforward ...
    - getField(ctrl, ...
    "differentialWheelPositionKp", 0)*xiDifferentialError ...
    - getField(ctrl, "differentialWheelPositionKd", 0)*dxiDifferentialError;

angles = state(4:6);
eulerRates = state(10:12);
q = [state([1, 3, 2]); angles; qLeft; qRight];
dq = [state([7, 9, 8]); eulerRates; dqLeft; dqRight];

[M, h, Jc, contactBias, modelData] = ...
    spatialDynamics(q, dq, base, leg, ctrl);
h(7:9) = h(7:9) + ctrl.commonModeJointDamping(:).*dqLeft;
h(10:12) = h(10:12) + ctrl.commonModeJointDamping(:).*dqRight;

baseQddController = upperDerivative(7:12);
baseQddCommand = baseQddController([1, 3, 2, 4, 5, 6]);

planarState = [state(1); state(3); state(5); ...
    state(7); state(9); state(11)];
planarWrench = [wrenchCommand(1) + wrenchCommand(7); ...
    wrenchCommand(3) + wrenchCommand(9); ...
    wrenchCommand(5) + wrenchCommand(11)];
rH = rotatePitch2D(base.rHBody(:), planarState(3));
planarBaseQdd = [
    planarWrench(1)/fullBaseNmpc.model.m;
    planarWrench(2)/fullBaseNmpc.model.m - base.g;
    (rH(1)*planarWrench(2) - rH(2)*planarWrench(1) ...
        + planarWrench(3))/base.Iyy
];
drdtheta = [-rH(2); rH(1)];
aHCommand = planarBaseQdd(1:2) + planarBaseQdd(3)*drdtheta ...
    - planarState(6)^2*rH;
perLegForce = -base.symmetricLoadShare*planarWrench(1:2);
[qd, dqd, ddqd] = floating_base_leg_reference(t, planarState, ...
    traj, leg, base, aHCommand, perLegForce, true, wheelReference);

commonCtrl = ctrl;
commonCtrl.Kp = getField(ctrl, "commonModeKp", ctrl.Kp);
commonCtrl.Kd = getField(ctrl, "commonModeKd", ctrl.Kd);
qCommon = 0.5*(qLeft + qRight);
dqCommon = 0.5*(dqLeft + dqRight);
qDifferential = 0.5*(qLeft - qRight);
dqDifferential = 0.5*(dqLeft - dqRight);
qddCommonCommand = relativeLegAccelerationCommand(qCommon, dqCommon, ...
    qd, dqd, ddqd, planarState, planarBaseQdd, commonCtrl);
differentialKp = getField(ctrl, "differentialModeKp", commonCtrl.Kp);
differentialKd = getField(ctrl, "differentialModeKd", commonCtrl.Kd);
qddDifferentialCommand = -differentialKp*qDifferential ...
    - differentialKd*dqDifferential;
qddLeftCommand = qddCommonCommand + qddDifferentialCommand;
qddRightCommand = qddCommonCommand - qddDifferentialCommand;

kneeMinLeft = kneeAccelerationLowerBound(qLeft, dqLeft, ctrl);
kneeMinRight = kneeAccelerationLowerBound(qRight, dqRight, ctrl);
qddLeftCommand(2) = max(qddLeftCommand(2), kneeMinLeft);
qddRightCommand(2) = max(qddRightCommand(2), kneeMinRight);

% z = [qdd(12); tauL(3); tauR(3); lambdaL(3); lambdaR(3); slackW(12)].
ntau = 6;
nlambda = 6;
nwrench = 12;
idxQdd = 1:nq;
idxTau = nq + (1:ntau);
idxLambda = nq + ntau + (1:nlambda);
idxSlack = nq + ntau + nlambda + (1:nwrench);
nz = idxSlack(end);

% Weighted WBC levels use dimensionless priorities on normalized residuals.
% Level 0 remains in Aeq/Aineq/lb/ub. Levels 1--4 are assembled below.
wBase = normalizedWeight(ctrl, ...
    "spatialQpBaseAccelRegularizationWeight", 0.1*ones(6, 1), ...
    "spatialQpBaseAccelScale", [10; 10; 10; 20; 20; 20], 6);
wCommon = normalizedWeight(ctrl, ...
    "spatialQpCommonLegTaskWeight", [5; 5; 0.1], ...
    "spatialQpLegAccelScale", [20; 20; 50], 3);
wDifferential = normalizedWeight(ctrl, ...
    "spatialQpDifferentialLegTaskWeight", [1; 1; 0], ...
    "spatialQpLegAccelScale", [20; 20; 50], 3);
wTauPerLeg = normalizedWeight(ctrl, ...
    "spatialQpTorqueRegularizationWeight", 0.1*ones(3, 1), ...
    "spatialQpTorqueScale", ctrl.tauMax(:), 3);
wTau = repmat(wTauPerLeg, 2, 1);
contactPriority = getField(ctrl, "spatialQpContactAccelWeight", ...
    [50; 20; 50]);
contactPriority = contactPriority(:);
if isscalar(contactPriority)
    contactPriority = repmat(contactPriority, 3, 1);
elseif numel(contactPriority) ~= 3
    error("spatial_two_leg_qp_core:InvalidContactAccelWeight", ...
        "spatialQpContactAccelWeight must be a scalar or three elements.");
end
contactScale = getVector(ctrl, "spatialQpContactAccelScale", ...
    [5; 2; 5], 3);
if any(~isfinite(contactPriority) | contactPriority < 0) ...
        || any(~isfinite(contactScale) | contactScale <= 0)
    error("spatial_two_leg_qp_core:InvalidContactAccelWeight", ...
        "Contact priorities must be nonnegative and scales positive.");
end
wContactDirection = contactPriority./contactScale.^2;
wContact = repmat(wContactDirection, 2, 1);
commonWrenchScale = getVector(ctrl, "spatialQpCommonWrenchScale", ...
    [140; 100; 140; 100; 160; 100], 6);
differentialWrenchScale = getVector(ctrl, ...
    "spatialQpDifferentialWrenchScale", commonWrenchScale, 6);
if any(~isfinite(commonWrenchScale) | commonWrenchScale <= 0) ...
        || any(~isfinite(differentialWrenchScale) ...
        | differentialWrenchScale <= 0)
    error("spatial_two_leg_qp_core:InvalidWrenchScale", ...
        "Wrench residual scales must be finite and positive.");
end
wrenchPenalty = getField(ctrl, "spatialQpWrenchPenalty", 1e5);
if ~isscalar(wrenchPenalty) || ~isfinite(wrenchPenalty) ...
        || wrenchPenalty < 0
    error("spatial_two_leg_qp_core:InvalidWrenchWeight", ...
        "spatialQpWrenchPenalty must be finite and nonnegative.");
end
wrenchScale = repmat(commonWrenchScale, 2, 1);
hierarchyRequested = logical(getField(ctrl, ...
    "wbcRollingFxHierarchyEnabled", false));
if ~isscalar(hierarchyRequested)
    error("spatial_two_leg_qp_core:InvalidRollingFxHierarchy", ...
        "wbcRollingFxHierarchyEnabled must be scalar.");
end

weights = [wBase; zeros(6, 1); wTau; zeros(6, 1); zeros(12, 1)];
H = diag(weights) + 1e-9*eye(nz);
f = zeros(nz, 1);
% Level 4 base acceleration is a zero-centered regularizer only. It does
% not track an NMPC-derived base acceleration or close a second base loop.
commonBlock = diag(0.5*(wCommon + wDifferential));
crossBlock = diag(0.5*(wCommon - wDifferential));
H(7:12, 7:12) = [commonBlock, crossBlock; crossBlock, commonBlock] ...
    + 1e-9*eye(6);
f(7:9) = -wCommon.*qddCommonCommand ...
    - wDifferential.*qddDifferentialCommand;
f(10:12) = -wCommon.*qddCommonCommand ...
    + wDifferential.*qddDifferentialCommand;
commonContactWeight = normalizedWeight(ctrl, ...
    "spatialQpCommonContactForceRegularizationWeight", ...
    0.1*ones(3, 1), "spatialQpContactForceScale", ...
    [140; 140; 160], 3);
differentialContactWeight = normalizedWeight(ctrl, ...
    "spatialQpDifferentialContactForceRegularizationWeight", ...
    ones(3, 1), "spatialQpContactForceScale", ...
    [140; 140; 160], 3);
commonContactBlock = diag(0.5*(commonContactWeight ...
    + differentialContactWeight));
crossContactBlock = diag(0.5*(commonContactWeight ...
    - differentialContactWeight));
H(idxLambda, idxLambda) = [commonContactBlock, crossContactBlock; ...
    crossContactBlock, commonContactBlock] + 1e-9*eye(6);
commonFxPenaltyMatrix = zeros(nz);
for channel = 1:6
    pair = idxSlack([channel, channel + 6]);
    commonWeight = wrenchPenalty/commonWrenchScale(channel)^2;
    differentialWeight = wrenchPenalty ...
        / differentialWrenchScale(channel)^2;
    H(pair, pair) = H(pair, pair) ...
        + commonWeight*[1, 1; 1, 1] ...
        + 0.25*differentialWeight*[1, -1; -1, 1];
    if channel == 1
        commonFxPenaltyMatrix(pair, pair) = ...
            commonWeight*[1, 1; 1, 1];
    end
end

S = zeros(nq, ntau);
S(7:9, 1:3) = diag(ctrl.tauSign(:));
S(10:12, 4:6) = diag(ctrl.tauSign(:));
[Dw, Dlambda, wrenchOffset] = interactionWrenchMap( ...
    modelData, base, leg);
[wheelPositionJacobian, wheelPositionBias] = ...
    wheelPositionTaskKinematics(q, dq, base, leg, ctrl);
JxiCommon = wheelPositionJacobian.common;
JxiDifferential = wheelPositionJacobian.differential;
xiCommonBias = wheelPositionBias.common;
xiDifferentialBias = wheelPositionBias.differential;
wheelTaskWeight = normalizedWeight(ctrl, ...
    "spatialQpWheelConfigurationWeight", [5; 20], ...
    "spatialQpWheelAccelScale", [5; 5], 2);
xiCommonTarget = xiCommonCommand - xiCommonBias;
xiDifferentialTarget = xiDifferentialCommand - xiDifferentialBias;
H(idxQdd, idxQdd) = H(idxQdd, idxQdd) ...
    + wheelTaskWeight(1)*(JxiCommon'*JxiCommon) ...
    + wheelTaskWeight(2)*(JxiDifferential'*JxiDifferential);
f(idxQdd) = f(idxQdd) ...
    - wheelTaskWeight(1)*JxiCommon'*xiCommonTarget ...
    - wheelTaskWeight(2)*JxiDifferential'*xiDifferentialTarget;
H(idxQdd, idxQdd) = H(idxQdd, idxQdd) ...
    + rollingTaskWeight*(JrollingCommon'*JrollingCommon);
f(idxQdd) = f(idxQdd) ...
    - rollingTaskWeight*JrollingCommon'*rollingAccelerationCommand;
[baseDriftAccelerationCommand, baseDriftTaskWeight, JbaseDrift] = ...
    baseHeightPitchTask(state, ctrl, nq);
H(idxQdd, idxQdd) = H(idxQdd, idxQdd) ...
    + JbaseDrift'*diag(baseDriftTaskWeight)*JbaseDrift;
f(idxQdd) = f(idxQdd) ...
    - JbaseDrift'*(baseDriftTaskWeight.*baseDriftAccelerationCommand);
H(idxQdd, idxQdd) = H(idxQdd, idxQdd) ...
    + Jc'*diag(wContact)*Jc;
[contactAccelerationTarget, normalContactDiagnostics] = ...
    normalContactAccelerationTarget(modelData, dq, base, leg, ctrl);
f(idxQdd) = f(idxQdd) + Jc'*(wContact.*( ...
    contactBias - contactAccelerationTarget));
H = 0.5*(H + H');
wrenchRhs = wrenchCommand - wrenchOffset;
Aeq = [
    M, -S, -Jc', zeros(nq, nwrench);
    Dw, zeros(nwrench, ntau), Dlambda, -eye(nwrench)
];
beq = [-h; wrenchRhs];

[Aineq, bineq] = spatialFrictionConstraints(getField(ctrl, "mu", 0.8), nz, idxLambda);
if isfinite(kneeMinLeft)
    row = zeros(1, nz);
    row(8) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMinLeft];
end
if isfinite(kneeMinRight)
    row = zeros(1, nz);
    row(11) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMinRight];
end

tauMax = repmat(ctrl.tauMax(:), 2, 1);
lb = -inf(nz, 1);
ub = inf(nz, 1);
lb(idxTau) = -tauMax;
ub(idxTau) = tauMax;
lb(idxLambda([3, 6])) = 0;

robotMass = base.body.mass + 2*(leg.m1 + leg.m2 + leg.mw);
z0 = zeros(nz, 1);
z0(idxQdd) = [baseQddCommand; qddLeftCommand; qddRightCommand];
z0(idxLambda([3, 6])) = robotMass*base.g/2;
if t <= 0 || isempty(zWarm) || numel(zWarm) ~= nz || any(~isfinite(zWarm))
    zWarm = z0;
elseif getField(ctrl, "qpWarmStart", true)
    z0 = zWarm;
end

hierarchyApplied = false;
hierarchyStage1Feasible = false;
hierarchyStage1RollingAcceleration = 0;
hierarchyRollingLockValue = 0;
AeqSolve = Aeq;
beqSolve = beq;
if hierarchyRequested
    % Pairwise Stage 1 retains every baseline objective except the
    % common-Fx common-mode slack penalty. The achieved rolling acceleration
    % therefore differs from baseline only through removal of that competing
    % objective; all other task weights and targets remain untouched.
    Hstage1 = 0.5*(H - commonFxPenaltyMatrix ...
        + (H - commonFxPenaltyMatrix)');
    [zStage1, ~] = solveEqualityQp( ...
        Hstage1, f, Aeq, beq);
    hierarchyStage1Feasible = isUsableQpCandidate( ...
        zStage1, Aeq, beq, Aineq, bineq, lb, ub, 1e-4);
    try
        if ~hierarchyStage1Feasible && isempty(qpOptions)
            qpOptions = optimoptions("quadprog", "Display", "off", ...
                "Algorithm", "interior-point-convex");
        end
        if ~hierarchyStage1Feasible
            [zStage1, ~, stage1ExitFlag] = quadprog( ...
                Hstage1, f, Aineq, bineq, Aeq, beq, ...
                lb, ub, z0, qpOptions);
            hierarchyStage1Feasible = isUsableQpCandidate( ...
                zStage1, Aeq, beq, Aineq, bineq, lb, ub, 1e-4) ...
                && stage1ExitFlag > 0;
        end
    catch
        hierarchyStage1Feasible = false;
    end
    if hierarchyStage1Feasible
        hierarchyStage1RollingAcceleration = ...
            JrollingCommon*zStage1(idxQdd);
        hierarchyRollingLockValue = ...
            hierarchyStage1RollingAcceleration;
        rollingLockRow = zeros(1, nz);
        rollingLockRow(idxQdd) = JrollingCommon;
        AeqSolve = [Aeq; rollingLockRow];
        beqSolve = [beq; hierarchyRollingLockValue];
        hierarchyApplied = true;
    end
end

exitflag = -999;
solveStart = tic;
% With inactive friction/torque bounds, the equality-constrained KKT point
% is already the exact QP optimum and is more reliable than an iterative
% solve of this strongly weighted hierarchical objective.
[z, kktFlag] = solveEqualityQp(H, f, AeqSolve, beqSolve);
candidateUsable = isUsableQpCandidate( ...
    z, AeqSolve, beqSolve, Aineq, bineq, lb, ub, 1e-4);
if candidateUsable
    exitflag = kktFlag;
end
try
    if ~candidateUsable && isempty(qpOptions)
        qpOptions = optimoptions("quadprog", "Display", "off", ...
            "Algorithm", "interior-point-convex");
    end
    if ~candidateUsable
        [z, ~, exitflag] = quadprog( ...
            H, f, Aineq, bineq, AeqSolve, beqSolve, ...
            lb, ub, z0, qpOptions);
    end
catch
    z = [];
end
candidateUsable = isUsableQpCandidate( ...
    z, AeqSolve, beqSolve, Aineq, bineq, lb, ub, 1e-4);
if ~candidateUsable
    [z, fallbackFlag] = solveEqualityQp( ...
        H, f, AeqSolve, beqSolve);
    if isempty(z)
        z = z0;
    end
    z(idxTau) = min(max(z(idxTau), -tauMax), tauMax);
    z(idxLambda([3, 6])) = max(z(idxLambda([3, 6])), 0);
    exitflag = min(-1, fallbackFlag);
else
    zWarm = z;
    exitflag = max(1, exitflag);
end
qpSolveTime = toc(solveStart);

qddSolution = z(idxQdd);
tauQp = z(idxTau);
tau = tauQp;
lambda = z(idxLambda);
wrenchSlack = z(idxSlack);
wrenchFeasible = Dw*qddSolution + Dlambda*lambda + wrenchOffset;
eqResidual = Aeq*z - beq;
ineqResidual = Aineq*z - bineq;
if hierarchyApplied
    hierarchyRollingLockResidual = ...
        JrollingCommon*qddSolution - hierarchyRollingLockValue;
else
    hierarchyRollingLockResidual = 0;
end
[taskSensitivity, taskCost, taskResidual, taskAttributionValid, ...
    taskGradientClosure] = wbcSoftTaskAttribution( ...
    logical(getField(ctrl, "wbcTaskAttributionEnabled", false)), ...
    z, H, f, Aeq, Aineq, bineq, lb, ub, idxQdd, idxTau, ...
    idxLambda, idxSlack, wBase, wCommon, wDifferential, ...
    qddCommonCommand, qddDifferentialCommand, wTau, ...
    commonContactWeight, differentialContactWeight, wrenchPenalty, ...
    commonWrenchScale, differentialWrenchScale, JxiCommon, ...
    xiCommonTarget, wheelTaskWeight(1), JxiDifferential, ...
    xiDifferentialTarget, wheelTaskWeight(2), JrollingCommon, ...
    rollingAccelerationCommand, rollingTaskWeight, JbaseDrift, ...
    baseDriftAccelerationCommand, baseDriftTaskWeight, Jc, ...
    contactBias, contactAccelerationTarget, wContactDirection);
physicalToController = [1, 0, 0; 0, 0, 1; 0, 1, 0];
contactForceLeft = physicalToController ...
    * modelData.contactBasis(:, :, 1)*lambda(1:3);
contactForceRight = physicalToController ...
    * modelData.contactBasis(:, :, 2)*lambda(4:6);
muPyramid = getField(ctrl, "mu", 0.8)/sqrt(2);
frictionMargin = [
    muPyramid*lambda(3) - abs(lambda(1));
    muPyramid*lambda(3) - abs(lambda(2));
    muPyramid*lambda(6) - abs(lambda(4));
    muPyramid*lambda(6) - abs(lambda(5))
];

% Optional legacy anti-split action through the physical hip/knee actuators.
% It remains available for comparison, but the paper-equation configuration
% disables it because the WBC now realizes the NMPC wheel accelerations.
legForceConfig = getField(ctrl, ...
    "differentialLegForceStabilizer", ...
    disabledLegForceConfig(ctrl.Ts));
[legForceCorrection, legForceDiagnostics] = ...
    differential_leg_force_stabilizer(t, xiDeltaCanonical, ...
    dxiDeltaCanonical, legForceConfig);
actuatedDifferentialGradient = JxiDifferential(7:12).';
actuatedDifferentialGradient([3, 6]) = 0;
tau = tau + repmat(ctrl.tauSign(:), 2, 1) ...
    .*(actuatedDifferentialGradient ...
    * legForceCorrection);
tau = min(max(tau, -tauMax), tauMax);

debug = struct();
debug.qdd = qddSolution;
debug.qddBase = qddSolution([1, 3, 2, 4, 5, 6]);
debug.qddDifferential = 0.5*(qddSolution(7:9) - qddSolution(10:12));
debug.qddDifferentialCommand = qddDifferentialCommand;
debug.tauDifferential = 0.5*(tau(1:3) - tau(4:6));
debug.contactForceDifferential = 0.5*(contactForceLeft - contactForceRight);
debug.FcLeft = contactForceLeft;
debug.FcRight = contactForceRight;
debug.lambdaLeft = lambda(1:3);
debug.lambdaRight = lambda(4:6);
debug.wrenchCommand = wrenchCommand;
debug.wrenchCommandBeforeIdentification = rawWrenchCommand;
debug.identificationExcitation = identificationExcitation;
debug.wrenchSlack = wrenchSlack;
debug.wrenchFeasible = wrenchFeasible;
debug.xiDeltaCanonical = xiDeltaCanonical;
debug.dxiDeltaCanonical = dxiDeltaCanonical;
debug.uDiffNominal = uDiffNominal;
debug.uDiffCorrectionRequested = driftDiagnostics.requested;
debug.uDiffCorrectionApplied = uDiffStab;
debug.uDiffFinal = uDiffFinal;
debug.uDiffRealized = 0.5*(wrenchFeasible(7) - wrenchFeasible(1));
debug.driftAmplitudeSaturated = driftDiagnostics.amplitudeSaturated;
debug.driftRateLimited = driftDiagnostics.rateLimited;
debug.driftFailSafe = driftDiagnostics.failSafe;
debug.driftReset = driftDiagnostics.reset;
debug.wrenchSlackNorm = norm(wrenchSlack./wrenchScale);
debug.wrenchResidual = wrenchFeasible - wrenchCommand - wrenchSlack;
debug.exitflag = exitflag;
debug.dynamicsResidual = M*qddSolution + h - S*tauQp - Jc'*lambda;
debug.contactAcceleration = Jc*qddSolution + contactBias;
debug.contactResidual = debug.contactAcceleration ...
    - contactAccelerationTarget;
debug.contactAccelerationTarget = contactAccelerationTarget;
debug.normalContactCompliance = normalContactDiagnostics;
contactResidualMatrix = reshape(debug.contactResidual, 3, 2);
debug.contactResidualDirection = sqrt(sum(contactResidualMatrix.^2, 2));
debug.qpFeasible = exitflag > 0 && norm(eqResidual, inf) < 1e-4 ...
    && (isempty(ineqResidual) || max(ineqResidual) < 1e-6);
debug.frictionMargin = frictionMargin;
debug.torqueMargin = tauMax - abs(tau);
debug.xiDifferential = xiDifferential;
debug.dxiDifferential = dxiDifferential;
debug.xiCommon = xiCommon;
debug.dxiCommon = dxiCommon;
debug.xiCommonError = xiCommonError;
debug.dxiCommonError = dxiCommonError;
debug.xiDifferentialError = xiDifferentialError;
debug.dxiDifferentialError = dxiDifferentialError;
debug.xiCommonAcceleration = JxiCommon*qddSolution + xiCommonBias;
debug.xiDifferentialAcceleration = JxiDifferential*qddSolution ...
    + xiDifferentialBias;
debug.xiCommonCommand = xiCommonCommand;
debug.xiDifferentialCommand = xiDifferentialCommand;
debug.xiCommonFeedforward = xiCommonFeedforward;
debug.xiDifferentialFeedforward = xiDifferentialFeedforward;
debug.wheelTaskWeight = wheelTaskWeight;
debug.contactTaskWeight = wContactDirection;
debug.contactPitchEstimate = modelData.contactPitchEstimate;
debug.commonRollingSpeed = rollingSpeed;
debug.commonRollingSpeedReference = rollingSpeedReference;
debug.commonRollingAccelerationCommand = rollingAccelerationCommand;
debug.commonRollingTaskWeight = rollingTaskWeight;
debug.commonRollingForceCorrection = rollingForceCorrection;
debug.commonRollingForceOverrideEnabled = rollingForceOverrideEnabled;
debug.baseDriftAccelerationCommand = baseDriftAccelerationCommand;
debug.baseDriftTaskWeight = baseDriftTaskWeight;
debug.qConfigurationDifferential = qDifferential;
debug.dqConfigurationDifferential = dqDifferential;
debug.qpSolveTime = qpSolveTime;
debug.legForceCorrectionRequested = legForceDiagnostics.requested;
debug.legForceCorrectionApplied = legForceCorrection;
debug.legForceCorrectionAmplitudeSaturated = ...
    legForceDiagnostics.amplitudeSaturated;
debug.legForceCorrectionRateLimited = legForceDiagnostics.rateLimited;
debug.legForceCorrectionFailSafe = legForceDiagnostics.failSafe;
debug.massMatrix = M;
debug.spatialQp = true;
debug.wrenchCommandBeforeDriftCorrection = wrenchCommand_before_correction;
debug.wbcTaskSensitivity = taskSensitivity;
debug.wbcTaskCost = taskCost;
debug.wbcTaskResidual = taskResidual;
debug.wbcTaskAttributionValid = taskAttributionValid;
debug.wbcTaskGradientClosure = taskGradientClosure;
debug.wbcRollingFxHierarchyRequested = hierarchyRequested;
debug.wbcRollingFxHierarchyApplied = hierarchyApplied;
debug.wbcRollingFxHierarchyStage1Feasible = ...
    hierarchyStage1Feasible;
debug.wbcRollingFxHierarchyStage1RollingAcceleration = ...
    hierarchyStage1RollingAcceleration;
debug.wbcRollingFxHierarchyLockResidual = ...
    hierarchyRollingLockResidual;

% --- Observer-only uDiffRealizable projection diagnostics (08-02-G2) ---
if isempty(projectionContract)
    projectionContract = contact_consistent_differential_contract();
end

function [speed, speedReference, accelerationCommand, taskWeight, J, ...
        forceCorrection, forceOverrideEnabled] = commonRollingSpeedTask( ...
        t, state, ctrl, nq, vehicleMass)
config = getField(ctrl, "commonRollingSpeedTracker", struct());
enabled = logical(getField(config, "enabled", false));
taskEnabled = logical(getField(config, "taskEnabled", enabled));
forceFeedbackEnabled = logical(getField(config, ...
    "forceFeedbackEnabled", false));
forceOverrideEnabled = logical(getField(config, ...
    "forceOverrideEnabled", false));
Kp = getField(config, "Kp", 0);
accelerationMax = getField(config, "accelerationMax", 0);
priority = getField(config, "taskWeight", 0);
scale = getField(config, "accelerationScale", 1);
parameters = [Kp; accelerationMax; priority; scale];
if ~isscalar(enabled) || ~isscalar(taskEnabled) ...
        || ~isscalar(forceFeedbackEnabled) ...
        || ~isscalar(forceOverrideEnabled) ...
        || ~isscalar(vehicleMass) || ~isfinite(vehicleMass) ...
        || vehicleMass <= 0 || any(~isfinite(parameters)) || Kp < 0 ...
        || accelerationMax < 0 || priority < 0 || scale <= 0
    error("spatial_two_leg_qp_core:InvalidRollingSpeedTracker", ...
        "Common rolling-speed tracker parameters are invalid.");
end
heading = [cos(state(6)), -sin(state(6))];
lateral = [sin(state(6)), cos(state(6))];
speed = heading*state(7:8);
lateralSpeed = lateral*state(7:8);
speedReference = speed;
accelerationCommand = 0;
taskWeight = 0;
forceCorrection = 0;
J = zeros(1, nq);
% This is a base forward-acceleration/speed task, not a wheel-ground
% Pfaffian row and not the wheel-relative acceleration from paper Eq. (12).
% Together with the wheel-relative task it determines the absolute wheel
% acceleration; the two requested accelerations must not be equated.
J(1) = heading(1);
J(3) = heading(2);
if ~enabled
    return;
end
baseLqr = evalin("base", "baseLqr");
[reference, referenceAcceleration] = floating_base_reference(t, baseLqr);
speedReference = reference(4);
forwardAccelerationCommand = referenceAcceleration(1) ...
    + Kp*(speedReference - speed);
forwardAccelerationCommand = min(max(forwardAccelerationCommand, ...
    -accelerationMax), accelerationMax);
% d(v_forward)/dt = heading*a_world - yawRate*v_lateral.
accelerationCommand = forwardAccelerationCommand ...
    + state(12)*lateralSpeed;
if taskEnabled
    taskWeight = priority/scale^2;
end
if forceOverrideEnabled
    forceCorrection = vehicleMass*forwardAccelerationCommand;
elseif forceFeedbackEnabled
    feedbackAcceleration = Kp*(speedReference - speed);
    feedbackAcceleration = min(max(feedbackAcceleration, ...
        -accelerationMax), accelerationMax);
    forceCorrection = vehicleMass*feedbackAcceleration;
end
end

function [command, taskWeight, J] = baseHeightPitchTask(state, ctrl, nq)
config = getField(ctrl, "baseHeightPitchTracker", struct());
enabled = logical(getField(config, "enabled", false));
heightKp = getField(config, "heightKp", 0);
heightKd = getField(config, "heightKd", 0);
heightAccelerationMax = getField(config, "heightAccelerationMax", 0);
pitchKp = getField(config, "pitchKp", 0);
pitchKd = getField(config, "pitchKd", 0);
pitchAccelerationMax = getField(config, "pitchAccelerationMax", 0);
priority = getVector(config, "taskWeight", zeros(2, 1), 2);
scale = getVector(config, "accelerationScale", ones(2, 1), 2);
parameters = [heightKp; heightKd; heightAccelerationMax; ...
    pitchKp; pitchKd; pitchAccelerationMax; priority; scale];
if ~isscalar(enabled) || any(~isfinite(parameters)) ...
        || any(parameters(1:6) < 0) || any(priority < 0) ...
        || any(scale <= 0)
    error("spatial_two_leg_qp_core:InvalidBaseHeightPitchTracker", ...
        "Base height-pitch tracker parameters are invalid.");
end
J = zeros(2, nq);
J(1, 2) = 1;
J(2, 5) = 1;
command = zeros(2, 1);
taskWeight = zeros(2, 1);
if ~enabled
    return;
end
command(1) = -heightKp*state(3) - heightKd*state(9);
command(2) = -pitchKp*state(5) - pitchKd*state(11);
command(1) = min(max(command(1), -heightAccelerationMax), ...
    heightAccelerationMax);
command(2) = min(max(command(2), -pitchAccelerationMax), ...
    pitchAccelerationMax);
taskWeight = priority./scale.^2;
end
auditWrenches = struct( ...
    "requested", rawWrenchCommand, ...
    "applied", wrenchCommand_before_correction, ...
    "final", wrenchCommand, ...
    "qpFeasible", wrenchFeasible);
[debug.uDiffScalarRequested, debug.uDiffProjectedRequested, ...
    debug.uDiffResidualRequested, debug.uDiffResidualRmsRequested] = ...
    project_uDiffRealizable(auditWrenches.requested, projectionContract);
[debug.uDiffScalarApplied, debug.uDiffProjectedApplied, ...
    debug.uDiffResidualApplied, debug.uDiffResidualRmsApplied] = ...
    project_uDiffRealizable(auditWrenches.applied, projectionContract);
[debug.uDiffScalarFinal, debug.uDiffProjectedFinal, ...
    debug.uDiffResidualFinal, debug.uDiffResidualRmsFinal] = ...
    project_uDiffRealizable(auditWrenches.final, projectionContract);
[debug.uDiffScalarQpFeasible, debug.uDiffProjectedQpFeasible, ...
    debug.uDiffResidualQpFeasible, debug.uDiffResidualRmsQpFeasible] = ...
    project_uDiffRealizable(auditWrenches.qpFeasible, projectionContract);
debug.uDiffAuditRawWrenches = auditWrenches;
debug.qpFeasibleControllerSide = debug.qpFeasible;
debug.plantWrenchUnavailable = true;
end

function [sensitivity, cost, residualMetric, valid, gradientClosure] = ...
        wbcSoftTaskAttribution(enabled, z, H, f, Aeq, Aineq, bineq, ...
        lb, ub, idxQdd, idxTau, idxLambda, idxSlack, wBase, ...
        wCommon, wDifferential, qddCommonCommand, ...
        qddDifferentialCommand, wTau, commonContactWeight, ...
        differentialContactWeight, wrenchPenalty, commonWrenchScale, ...
        differentialWrenchScale, JxiCommon, xiCommonTarget, ...
        wheelCommonWeight, JxiDifferential, xiDifferentialTarget, ...
        wheelDifferentialWeight, JrollingCommon, rollingTarget, ...
        rollingWeight, JbaseDrift, baseDriftTarget, baseDriftWeight, ...
        Jc, contactBias, contactTarget, contactDirectionWeight)
% Observer-only task decomposition. Task order is frozen by
% coupled_two_leg_qp_signal_contract().attributionNames.
nTask = 16;
nz = numel(z);
gradient = zeros(nz, nTask);
cost = zeros(nTask, 1);
residualMetric = zeros(nTask, 1);
qdd = z(idxQdd);
tauQp = z(idxTau);
lambda = z(idxLambda);
slack = z(idxSlack);

% 1: base acceleration regularization.
gradient(idxQdd(1:6), 1) = wBase.*qdd(1:6);
cost(1) = 0.5*sum(wBase.*qdd(1:6).^2);
residualMetric(1) = sqrt(mean(qdd(1:6).^2));

% 2-3: common/differential leg acceleration tasks.
qddLeft = qdd(7:9);
qddRight = qdd(10:12);
commonResidual = 0.5*(qddLeft + qddRight) - qddCommonCommand;
differentialResidual = 0.5*(qddLeft - qddRight) ...
    - qddDifferentialCommand;
gradient(idxQdd(7:9), 2) = wCommon.*commonResidual;
gradient(idxQdd(10:12), 2) = wCommon.*commonResidual;
gradient(idxQdd(7:9), 3) = wDifferential.*differentialResidual;
gradient(idxQdd(10:12), 3) = -wDifferential.*differentialResidual;
cost(2) = sum(wCommon.*commonResidual.^2);
cost(3) = sum(wDifferential.*differentialResidual.^2);
residualMetric(2) = sqrt(mean(commonResidual.^2));
residualMetric(3) = sqrt(mean(differentialResidual.^2));

% 4: actuator torque regularization.
gradient(idxTau, 4) = wTau.*tauQp;
cost(4) = 0.5*sum(wTau.*tauQp.^2);
residualMetric(4) = sqrt(mean(tauQp.^2));

% 5-6: common/differential analytic contact-force regularization.
lambdaCommon = 0.5*(lambda(1:3) + lambda(4:6));
lambdaDifferential = 0.5*(lambda(1:3) - lambda(4:6));
gradient(idxLambda(1:3), 5) = commonContactWeight.*lambdaCommon;
gradient(idxLambda(4:6), 5) = commonContactWeight.*lambdaCommon;
gradient(idxLambda(1:3), 6) = ...
    differentialContactWeight.*lambdaDifferential;
gradient(idxLambda(4:6), 6) = ...
    -differentialContactWeight.*lambdaDifferential;
cost(5) = sum(commonContactWeight.*lambdaCommon.^2);
cost(6) = sum(differentialContactWeight.*lambdaDifferential.^2);
residualMetric(5) = sqrt(mean(lambdaCommon.^2));
residualMetric(6) = sqrt(mean(lambdaDifferential.^2));

% 7: common-Fx slack regularization. 8: every other wrench-slack mode.
otherWrenchResidual = zeros(11, 1);
otherIndex = 0;
for channel = 1:6
    pair = idxSlack([channel, channel + 6]);
    pairSlack = slack([channel, channel + 6]);
    commonWeight = wrenchPenalty/commonWrenchScale(channel)^2;
    differentialWeight = wrenchPenalty ...
        /differentialWrenchScale(channel)^2;
    commonValue = sum(pairSlack);
    differentialValue = pairSlack(1) - pairSlack(2);
    commonGradient = commonWeight*commonValue*[1; 1];
    differentialGradient = 0.25*differentialWeight ...
        *differentialValue*[1; -1];
    if channel == 1
        gradient(pair, 7) = commonGradient;
        cost(7) = 0.5*commonWeight*commonValue^2;
        residualMetric(7) = abs(commonValue);
        gradient(pair, 8) = gradient(pair, 8) + differentialGradient;
        cost(8) = cost(8) + 0.125*differentialWeight ...
            *differentialValue^2;
        otherIndex = otherIndex + 1;
        otherWrenchResidual(otherIndex) = ...
            differentialValue/differentialWrenchScale(channel);
    else
        gradient(pair, 8) = gradient(pair, 8) ...
            + commonGradient + differentialGradient;
        cost(8) = cost(8) + 0.5*commonWeight*commonValue^2 ...
            + 0.125*differentialWeight*differentialValue^2;
        otherIndex = otherIndex + 1;
        otherWrenchResidual(otherIndex) = ...
            commonValue/commonWrenchScale(channel);
        otherIndex = otherIndex + 1;
        otherWrenchResidual(otherIndex) = ...
            differentialValue/differentialWrenchScale(channel);
    end
end
residualMetric(8) = sqrt(mean(otherWrenchResidual.^2));

% 9-11: wheel common, wheel differential, and common rolling acceleration.
wheelCommonResidual = JxiCommon*qdd - xiCommonTarget;
wheelDifferentialResidual = JxiDifferential*qdd - xiDifferentialTarget;
rollingResidual = JrollingCommon*qdd - rollingTarget;
gradient(idxQdd, 9) = wheelCommonWeight*JxiCommon'*wheelCommonResidual;
gradient(idxQdd, 10) = wheelDifferentialWeight ...
    *JxiDifferential'*wheelDifferentialResidual;
gradient(idxQdd, 11) = rollingWeight*JrollingCommon'*rollingResidual;
cost(9) = 0.5*wheelCommonWeight*wheelCommonResidual^2;
cost(10) = 0.5*wheelDifferentialWeight*wheelDifferentialResidual^2;
cost(11) = 0.5*rollingWeight*rollingResidual^2;
residualMetric(9) = abs(wheelCommonResidual);
residualMetric(10) = abs(wheelDifferentialResidual);
residualMetric(11) = abs(rollingResidual);

% 12-13: base height and pitch tracking.
baseResidual = JbaseDrift*qdd - baseDriftTarget;
for row = 1:2
    taskIndex = 11 + row;
    gradient(idxQdd, taskIndex) = baseDriftWeight(row) ...
        *JbaseDrift(row, :)'*baseResidual(row);
    cost(taskIndex) = 0.5*baseDriftWeight(row)*baseResidual(row)^2;
    residualMetric(taskIndex) = abs(baseResidual(row));
end

% 14-16: rolling/lateral/normal contact-acceleration tasks, both wheels.
contactResidual = Jc*qdd + contactBias - contactTarget;
for direction = 1:3
    taskIndex = 13 + direction;
    rows = [direction, direction + 3];
    directionWeight = contactDirectionWeight(direction);
    gradient(idxQdd, taskIndex) = directionWeight ...
        *Jc(rows, :)'*contactResidual(rows);
    cost(taskIndex) = 0.5*directionWeight ...
        *sum(contactResidual(rows).^2);
    residualMetric(taskIndex) = sqrt(mean(contactResidual(rows).^2));
end

objectiveGradient = H*z + f;
gradientClosure = norm(sum(gradient, 2) - objectiveGradient, inf) ...
    /max(1, norm(objectiveGradient, inf));
sensitivity = zeros(nTask, 1);
valid = false;
if ~enabled
    return
end

% Include the locally active inequality/bound rows in the adjoint KKT
% system. This makes the projection a constrained sensitivity rather than a
% meaningless direct dot product with the slack coordinate.
activeTolerance = 1e-6;
activeRows = Aineq*z >= bineq - activeTolerance;
activeMatrix = Aineq(activeRows, :);
for index = 1:nz
    if isfinite(lb(index)) && z(index) <= lb(index) + activeTolerance
        row = zeros(1, nz);
        row(index) = 1;
        activeMatrix = [activeMatrix; row]; %#ok<AGROW>
    elseif isfinite(ub(index)) && z(index) >= ub(index) - activeTolerance
        row = zeros(1, nz);
        row(index) = 1;
        activeMatrix = [activeMatrix; row]; %#ok<AGROW>
    end
end
constraintMatrix = [Aeq; activeMatrix];
rowScale = max(vecnorm(constraintMatrix, 2, 2), 1e-9);
scaledConstraint = constraintMatrix./rowScale;
regularization = 1e-9;
KKT = [H + regularization*eye(nz), scaledConstraint'; ...
    scaledConstraint, zeros(size(scaledConstraint, 1))];
selector = zeros(nz, 1);
selector(idxSlack([1, 7])) = 1;
rhs = [selector; zeros(size(scaledConstraint, 1), 1)];
if rcond(KKT) < 1e-14
    adjoint = pinv(KKT')*rhs;
else
    adjoint = KKT'\rhs;
end
if all(isfinite(adjoint))
    sensitivity = -(adjoint(1:nz)'*gradient).';
    valid = all(isfinite(sensitivity));
end
end

function config = disabledDriftConfig(Ts)
config = struct( ...
    "enabled", false, ...
    "Kxi", 0, ...
    "Kd", 0, ...
    "polarity", 1, ...
    "amplitudeLimit", 0, ...
    "rateLimit", 0, ...
    "Ts", Ts);
end

function config = disabledLegForceConfig(Ts)
config = struct( ...
    "enabled", false, ...
    "Kxi", 0, ...
    "Kd", 0, ...
    "polarity", 1, ...
    "amplitudeLimit", 0, ...
    "rateLimit", 0, ...
    "Ts", Ts);
end

function audit = contactAuditSnapshot()
%CONTACTAUDITSNAPSHOT Read-only exposure of the current WBC/contact maps.
requiredVariables = ["base", "leg", "ctrl", "fullBaseNmpc"];
for name = requiredVariables
    if evalin("base", "exist('" + name + "', 'var')") ~= 1
        error("spatial_two_leg_qp_core:MissingAuditData", ...
            "Required base-workspace variable '%s' is unavailable.", name);
    end
end
base = evalin("base", "base");
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
fullBaseNmpc = evalin("base", "fullBaseNmpc");
if ~isfield(leg, "q0") || ~isfield(leg, "dq0") ...
        || numel(leg.q0) ~= 3 || numel(leg.dq0) ~= 3
    error("spatial_two_leg_qp_core:MissingAuditData", ...
        "leg.q0 and leg.dq0 must expose the three-joint nominal state.");
end
if ~isfield(ctrl, "tauMax") || ~isfield(ctrl, "mu") ...
        || ~isfield(fullBaseNmpc, "uMin") ...
        || ~isfield(fullBaseNmpc, "uMax")
    error("spatial_two_leg_qp_core:MissingAuditData", ...
        "Torque, friction, and NMPC bounds are required for the audit.");
end

qLeg = double(leg.q0(:));
dqLeg = double(leg.dq0(:));
q = getField(ctrl, "contactAuditQ", [zeros(6, 1); qLeg; qLeg]);
dq = getField(ctrl, "contactAuditDq", [zeros(6, 1); dqLeg; dqLeg]);
q = double(q(:));
dq = double(dq(:));
if numel(q) ~= 12 || numel(dq) ~= 12 || any(~isfinite(q)) ...
        || any(~isfinite(dq))
    error("spatial_two_leg_qp_core:InvalidContactAuditState", ...
        "ctrl.contactAuditQ and contactAuditDq must be finite 12-vectors.");
end
[~, ~, Jc, contactBias, modelData] = ...
    spatialDynamics(q, dq, base, leg, ctrl);
[normalContactTarget, normalContactDiagnostics] = ...
    normalContactAccelerationTarget(modelData, dq, base, leg, ctrl);
[legacyJc, materialPointJc] = contactJacobianPair(modelData, leg);
legacyBias = contactJacobianDirectionalBias( ...
    q, dq, base, leg, modelData.contactPitchEstimate, false);
materialPointBias = contactJacobianDirectionalBias( ...
    q, dq, base, leg, modelData.contactPitchEstimate, true);
[Dw, Dlambda, wrenchOffset] = interactionWrenchMap( ...
    modelData, base, leg);
[wheelPositionJacobian, ~] = ...
    wheelPositionTaskKinematics(q, dq, base, leg, ctrl);
if ~isequal(size(Jc), [6, 12]) ...
        || ~isequal(size(modelData.contactBasis), [3, 3, 2]) ...
        || ~isequal(size(Dlambda), [12, 6])
    error("spatial_two_leg_qp_core:InvalidAuditMap", ...
        "Unexpected contact Jacobian, basis, or interaction-wrench map size.");
end

audit = struct();
audit.nominalQ = q;
audit.nominalDq = dq;
audit.contactJacobian = Jc;
audit.contactBias = contactBias;
audit.contactAccelerationTarget = normalContactTarget;
audit.normalContactCompliance = normalContactDiagnostics;
audit.contactBasis = modelData.contactBasis;
audit.rollingDirection = modelData.contactBasis(:, 1, 1);
audit.lateralDirection = modelData.contactBasis(:, 2, 1);
audit.normalDirection = modelData.contactBasis(:, 3, 1);
audit.axleDirection = modelData.axle;
audit.wheelCenterJacobian = modelData.Jv(:, :, modelData.wheelBody);
audit.wheelAngularJacobian = modelData.Jw(:, :, modelData.wheelBody);
audit.contactPointOffset = -double(leg.r) ...
    *reshape(modelData.contactBasis(:, 3, :), 3, 2);
audit.pfaffian = struct( ...
    "legacy", legacyJc, ...
    "materialPoint", materialPointJc, ...
    "legacyBias", legacyBias, ...
    "materialPointBias", materialPointBias, ...
    "selectedUsesMaterialPoint", logical(getField(ctrl, ...
        "materialPointContactKinematicsEnabled", false)));
audit.interactionWrenchMap = struct( ...
    "acceleration", Dw, "contact", Dlambda, "offset", wrenchOffset);
audit.wheelPositionJacobian = wheelPositionJacobian;
audit.contactPitchEstimate = modelData.contactPitchEstimate;
audit.torqueLimit = double(ctrl.tauMax(:));
audit.frictionCoefficient = double(ctrl.mu);
audit.nmpcBounds = struct("min", double(fullBaseNmpc.uMin(:)), ...
    "max", double(fullBaseNmpc.uMax(:)));
audit.sampleTimes = struct("wbc", double(ctrl.Ts), ...
    "nmpc", double(fullBaseNmpc.Ts));
audit.robotMass = base.body.mass + 2*(leg.m1 + leg.m2 + leg.mw);
audit.gravity = double(base.g);
audit.wheelRadius = double(leg.r);
audit.frames = struct( ...
    "contact", "world physical [rolling tangent,lateral,normal]", ...
    "wrench", "per-side body/controller [force(3);moment(3)]", ...
    "targetPair", "controller components [1,5]: forward force, pitch moment");
end

function [jacobian, bias] = ...
        wheelPositionTaskKinematics(q, dq, base, leg, ctrl)
% Differentiate the exact xi definition in full_base_nmpc_state_signal.
% Contact directions may follow terrain, while xi depends only on base pitch
% and the corresponding sagittal leg coordinates.
pitch = q(5);
pitchRate = dq(5);
pitchSign = getField(ctrl, "basePitchToAbsHipSign", 1);
rH = rotatePitch2D(base.rHBody(:), pitch);
Jside = cell(2, 1);
sideBias = zeros(2, 1);
for side = 1:2
    if side == 1
        first = 7;
    else
        first = 10;
    end
    qAbsolute = q(first:first + 2);
    dqAbsolute = dq(first:first + 2);
    qAbsolute(1) = qAbsolute(1) + pitchSign*pitch;
    dqAbsolute(1) = dqAbsolute(1) + pitchSign*pitchRate;
    kin = wheel_leg_kinematics(qAbsolute, dqAbsolute, [], leg);

    Jside{side} = zeros(1, 12);
    Jside{side}(5) = -rH(2) + pitchSign*kin.JO(1, 1);
    Jside{side}(first:first + 2) = kin.JO(1, :);
    sideBias(side) = -rH(1)*pitchRate^2 ...
        + kin.dJO(1, :)*dqAbsolute;
end
Jleft = Jside{1};
Jright = Jside{2};
leftBias = sideBias(1);
rightBias = sideBias(2);

jacobian = struct( ...
    "left", Jleft, ...
    "right", Jright, ...
    "common", 0.5*(Jleft + Jright), ...
    "differential", 0.5*(Jleft - Jright));
bias = struct( ...
    "left", leftBias, ...
    "right", rightBias, ...
    "common", 0.5*(leftBias + rightBias), ...
    "differential", 0.5*(leftBias - rightBias));
end

function [M, h, Jc, contactBias, data] = ...
        spatialDynamics(q, dq, base, leg, ctrl)
[M, data] = spatialMassMatrix(q, dq, base, leg);
[data, contactPitch] = configuredContactFrame(data, ctrl);
n = numel(q);
dM = zeros(n, n, n);
step = 1e-6;
for k = 4:n
    delta = zeros(n, 1);
    delta(k) = step;
    Mplus = spatialMassMatrix(q + delta, zeros(n, 1), base, leg);
    Mminus = spatialMassMatrix(q - delta, zeros(n, 1), base, leg);
    dM(:, :, k) = (Mplus - Mminus)/(2*step);
end
Mdot = zeros(n);
kineticGradient = zeros(n, 1);
for k = 1:n
    Mdot = Mdot + dM(:, :, k)*dq(k);
    kineticGradient(k) = dq'*dM(:, :, k)*dq;
end
coriolis = Mdot*dq - 0.5*kineticGradient;
gravity = zeros(n, 1);
for body = 1:numel(data.mass)
    gravity = gravity + data.Jv(:, :, body)' ...
        *(data.mass(body)*[0; base.g; 0]);
end
h = coriolis + gravity;

materialPointEnabled = logical(getField(ctrl, ...
    "materialPointContactKinematicsEnabled", false));
if ~isscalar(materialPointEnabled)
    error("spatial_two_leg_qp_core:InvalidContactKinematicsMode", ...
        "materialPointContactKinematicsEnabled must be scalar logical.");
end
Jc = contactJacobian(data, leg, materialPointEnabled);
if norm(dq, inf) == 0
    contactBias = zeros(6, 1);
    data.biasV = zeros(3, numel(data.mass));
    data.biasW = zeros(3, numel(data.mass));
else
    dt = 1e-6/max(1, norm(dq, inf));
    [~, plus] = spatialMassMatrix(q + dt*dq, dq, base, leg);
    [~, minus] = spatialMassMatrix(q - dt*dq, dq, base, leg);
    plus = setContactPitch(plus, contactPitch);
    minus = setContactPitch(minus, contactPitch);
    Jplus = contactJacobian(plus, leg, materialPointEnabled);
    Jminus = contactJacobian(minus, leg, materialPointEnabled);
    contactBias = ((Jplus - Jminus)/(2*dt))*dq;
    bodyCount = numel(data.mass);
    data.biasV = zeros(3, bodyCount);
    data.biasW = zeros(3, bodyCount);
    for body = 1:bodyCount
        data.biasV(:, body) = ((plus.Jv(:, :, body) ...
            - minus.Jv(:, :, body))/(2*dt))*dq;
        data.biasW(:, body) = ((plus.Jw(:, :, body) ...
            - minus.Jw(:, :, body))/(2*dt))*dq;
    end
end
end

function [data, pitchEstimate] = configuredContactFrame(data, ctrl)
% Rotate the analytic contact basis when the active terrain study supplies
% its known piecewise slope geometry. Normal operation remains flat-map.
map = getField(ctrl, "terrainContactMap", struct());
mapEnabled = logical(getField(map, "enabled", false));
if mapEnabled
    wheelX = mean(data.position(1, data.wheelBody));
    pitchEstimate = mappedTerrainPitch(map, wheelX);
    data = setContactPitch(data, pitchEstimate);
    data.contactPitchEstimate = pitchEstimate;
    return;
end
pitchEstimate = 0;
data = setContactPitch(data, pitchEstimate);
data.contactPitchEstimate = pitchEstimate;
end

function [target, diagnostics] = normalContactAccelerationTarget( ...
        data, dq, base, leg, ctrl)
% Diagnostic compliant-normal target using known terrain geometry only.
% Penetration delta is positive overlap. Since deltaDot = -v_n, the target
% a_n = Kp*(delta-deltaRef) + Kd*deltaDot yields the stable error equation
% deltaDDot + Kd*deltaDot + Kp*(delta-deltaRef) = 0.
config = getField(ctrl, "normalContactCompliance", struct());
enabled = logical(getField(config, "enabled", false));
if ~isscalar(enabled)
    error("spatial_two_leg_qp_core:InvalidNormalContactCompliance", ...
        "normalContactCompliance.enabled must be scalar logical.");
end
frequencyHz = getField(config, "frequencyHz", 1.72648796001821);
dampingRatio = getField(config, "dampingRatio", 1.0);
penetrationReference = getField(config, "penetrationReference", ...
    ctrl.commonModeContactPreload);
values = [frequencyHz, dampingRatio, penetrationReference];
if any(~isfinite(values)) || frequencyHz <= 0 || dampingRatio < 0 ...
        || penetrationReference < 0
    error("spatial_two_leg_qp_core:InvalidNormalContactCompliance", ...
        "Normal compliance frequency, damping, and penetration reference are invalid.");
end

omega = 2*pi*frequencyHz;
Kp = omega^2;
Kd = 2*dampingRatio*omega;
target = zeros(6, 1);
penetration = zeros(2, 1);
penetrationRate = zeros(2, 1);
normalVelocity = zeros(2, 1);
physicalOffset = [0; base.simscapeWorldYOffset ...
    - ctrl.commonModeContactPreload; 0];
for side = 1:2
    body = data.wheelBody(side);
    wheelPosition = data.position(:, body) + physicalOffset;
    wheelVelocity = data.Jv(:, :, body)*dq;
    normal = data.contactBasis(:, 3, side);
    planePoint = terrainPlanePoint( ...
        ctrl, base, wheelPosition(1));
    normalDistance = normal'*(wheelPosition - planePoint);
    penetration(side) = leg.r - normalDistance;
    normalVelocity(side) = normal'*wheelVelocity;
    penetrationRate(side) = -normalVelocity(side);
    if enabled
        row = 3*side;
        target(row) = Kp*(penetration(side) - penetrationReference) ...
            + Kd*penetrationRate(side);
    end
end
diagnostics = struct( ...
    "enabled", enabled, ...
    "penetration", penetration, ...
    "penetrationRate", penetrationRate, ...
    "normalVelocity", normalVelocity, ...
    "penetrationReference", penetrationReference, ...
    "frequencyHz", frequencyHz, ...
    "dampingRatio", dampingRatio, ...
    "Kp", Kp, "Kd", Kd, ...
    "target", target([3, 6]));
end

function point = terrainPlanePoint(ctrl, base, wheelX)
map = getField(ctrl, "terrainContactMap", struct());
enabled = logical(getField(map, "enabled", false));
groundTop = getField(map, "groundTopY", base.simscapeGroundTopY);
point = [wheelX; groundTop; 0];
if ~enabled
    return;
end
required = ["leadingEdgeX", "upEndX", ...
    "platformEndX", "trailingEdgeX", "slopeAngle"];
for name = required
    if ~isfield(map, name) || ~isscalar(map.(name)) ...
            || ~isfinite(map.(name))
        error("spatial_two_leg_qp_core:InvalidNormalContactTerrain", ...
            "Normal compliance terrain requires finite scalar field '%s'.", ...
            name);
    end
end
riseHeight = getField(map, "riseHeight", ...
    (map.upEndX - map.leadingEdgeX)*tan(map.slopeAngle));
if wheelX >= map.leadingEdgeX && wheelX < map.upEndX
    point = [map.leadingEdgeX; groundTop; 0];
elseif wheelX >= map.upEndX && wheelX < map.platformEndX
    point = [wheelX; groundTop + riseHeight; 0];
elseif wheelX >= map.platformEndX && wheelX < map.trailingEdgeX
    point = [map.platformEndX; groundTop + riseHeight; 0];
end
end

function pitch = mappedTerrainPitch(map, wheelX)
required = ["slopeAngle", "leadingEdgeX", "upEndX", ...
    "platformEndX", "trailingEdgeX"];
for name = required
    if ~isfield(map, name) || ~isscalar(map.(name)) ...
            || ~isfinite(map.(name))
        error("spatial_two_leg_qp_core:InvalidTerrainContactMap", ...
            "Mapped contact frame requires finite scalar field '%s'.", name);
    end
end
pitch = 0;
transitionLength = getField(map, "transitionLength", 0);
if ~isscalar(transitionLength) || ~isfinite(transitionLength) ...
        || transitionLength < 0
    error("spatial_two_leg_qp_core:InvalidTerrainContactMap", ...
        "Mapped contact transition length must be finite and nonnegative.");
end
if transitionLength == 0
    if wheelX >= map.leadingEdgeX && wheelX < map.upEndX
        pitch = map.slopeAngle;
    elseif wheelX >= map.platformEndX && wheelX < map.trailingEdgeX
        pitch = -map.slopeAngle;
    end
else
    pitch = map.slopeAngle*( ...
        smoothStep(wheelX, map.leadingEdgeX, transitionLength) ...
        - smoothStep(wheelX, map.upEndX, transitionLength) ...
        - smoothStep(wheelX, map.platformEndX, transitionLength) ...
        + smoothStep(wheelX, map.trailingEdgeX, transitionLength));
end
end

function value = smoothStep(x, center, width)
s = min(max((x - center)/width + 0.5, 0), 1);
value = s^2*(3 - 2*s);
end

function data = setContactPitch(data, pitch)
horizontalForward = data.contactBasis(:, 1, 1);
horizontalForward(2) = 0;
if norm(horizontalForward) < 1e-9
    horizontalForward = [1; 0; 0];
else
    horizontalForward = horizontalForward/norm(horizontalForward);
end
up = [0; 1; 0];
tangent = cos(pitch)*horizontalForward + sin(pitch)*up;
lateral = cross(horizontalForward, up);
lateral = lateral/norm(lateral);
normal = cross(lateral, tangent);
for side = 1:2
    data.contactBasis(:, :, side) = [tangent, lateral, normal];
end
end

function [M, data] = spatialMassMatrix(q, dq, base, leg)
n = 12;
bodyCount = 7;
data.mass = zeros(bodyCount, 1);
data.position = zeros(3, bodyCount);
data.Jv = zeros(3, n, bodyCount);
data.Jw = zeros(3, n, bodyCount);
data.inertia = zeros(3, 3, bodyCount);
data.omega = zeros(3, bodyCount);
data.wheelBody = [4, 7];

p = q(1:3);
angles = q(4:6);
[R, dR, E] = rotationData(angles);
data.baseRotation = R;
baseInertia = diag(base.body.mass/12 * [
    base.body.widthY^2 + base.body.heightZ^2;
    base.body.lengthX^2 + base.body.widthY^2;
    base.body.lengthX^2 + base.body.heightZ^2]);
data.mass(1) = base.body.mass;
data.position(:, 1) = p;
data.Jv(:, 1:3, 1) = eye(3);
data.Jw(:, 4:6, 1) = E;
data.inertia(:, :, 1) = R*baseInertia*R';
data.omega(:, 1) = data.Jw(:, :, 1)*dq;

link1Inertia = linkInertia(leg.m1, leg.L1, leg);
link2Inertia = linkInertia(leg.m2, leg.L2, leg);
wheelTransverse = leg.mw*(3*leg.r^2 + leg.width^2)/12;
wheelInertia = diag([wheelTransverse; wheelTransverse; leg.Iw]);
for side = 1:2
    if side == 1
        first = 7;
        hipBody = base.body.hipPositionBodyLeft3D(:);
        bodyFirst = 2;
    else
        first = 10;
        hipBody = base.body.hipPositionBodyRight3D(:);
        bodyFirst = 5;
    end
    qh = q(first);
    qhk = q(first) + q(first + 1);
    qWheel = qhk + q(first + 2);
    localPoints = [
        leg.c1*sin(qh), leg.L1*sin(qh) + leg.c2*sin(qhk), ...
            leg.L1*sin(qh) + leg.L2*sin(qhk);
        -leg.c1*cos(qh), -leg.L1*cos(qh) - leg.c2*cos(qhk), ...
            -leg.L1*cos(qh) - leg.L2*cos(qhk);
        0, 0, 0
    ];
    localDerivatives = zeros(3, 3, 3);
    localDerivatives(:, 1, 1) = [leg.c1*cos(qh); leg.c1*sin(qh); 0];
    localDerivatives(:, 1, 2) = [leg.L1*cos(qh) + leg.c2*cos(qhk); ...
        leg.L1*sin(qh) + leg.c2*sin(qhk); 0];
    localDerivatives(:, 2, 2) = [leg.c2*cos(qhk); leg.c2*sin(qhk); 0];
    localDerivatives(:, 1, 3) = [leg.L1*cos(qh) + leg.L2*cos(qhk); ...
        leg.L1*sin(qh) + leg.L2*sin(qhk); 0];
    localDerivatives(:, 2, 3) = [leg.L2*cos(qhk); leg.L2*sin(qhk); 0];
    bodyAngles = [qh, qhk, qWheel];
    bodyMasses = [leg.m1, leg.m2, leg.mw];
    bodyInertias = cat(3, link1Inertia, link2Inertia, wheelInertia);
    for localBody = 1:3
        body = bodyFirst + localBody - 1;
        rBody = hipBody + localPoints(:, localBody);
        data.mass(body) = bodyMasses(localBody);
        data.position(:, body) = p + R*rBody;
        data.Jv(:, 1:3, body) = eye(3);
        for k = 1:3
            data.Jv(:, 3 + k, body) = dR(:, :, k)*rBody;
        end
        data.Jv(:, first, body) = R*localDerivatives(:, 1, localBody);
        if localBody >= 2
            data.Jv(:, first + 1, body) = R*localDerivatives(:, 2, localBody);
        end
        data.Jw(:, 4:6, body) = E;
        axisWorld = R*[0; 0; 1];
        data.Jw(:, first, body) = axisWorld;
        if localBody >= 2
            data.Jw(:, first + 1, body) = axisWorld;
        end
        if localBody == 3
            data.Jw(:, first + 2, body) = axisWorld;
        end
        bodyRotation = R*rotationZ(bodyAngles(localBody));
        data.inertia(:, :, body) = bodyRotation ...
            * bodyInertias(:, :, localBody)*bodyRotation';
        data.omega(:, body) = data.Jw(:, :, body)*dq;
    end
end

M = zeros(n);
for body = 1:bodyCount
    M = M + data.mass(body)*(data.Jv(:, :, body)'*data.Jv(:, :, body)) ...
        + data.Jw(:, :, body)'*data.inertia(:, :, body)*data.Jw(:, :, body);
end
M = 0.5*(M + M');

forward = R*[1; 0; 0];
tangent = [forward(1); 0; forward(3)];
if norm(tangent) < 1e-9
    tangent = [1; 0; 0];
else
    tangent = tangent/norm(tangent);
end
lateral = cross(tangent, [0; 1; 0]);
normal = [0; 1; 0];
data.contactBasis = zeros(3, 3, 2);
data.axle = zeros(3, 2);
for side = 1:2
    data.contactBasis(:, :, side) = [tangent, lateral, normal];
    data.axle(:, side) = R*[0; 0; 1];
end
end

function [legacyJc, materialPointJc] = contactJacobianPair(data, leg)
legacyJc = contactJacobian(data, leg, false);
materialPointJc = contactJacobian(data, leg, true);
end

function bias = contactJacobianDirectionalBias( ...
        q, dq, base, leg, contactPitch, materialPointEnabled)
if norm(dq, inf) == 0
    bias = zeros(6, 1);
    return;
end
dt = 1e-6/max(1, norm(dq, inf));
[~, plus] = spatialMassMatrix(q + dt*dq, dq, base, leg);
[~, minus] = spatialMassMatrix(q - dt*dq, dq, base, leg);
plus = setContactPitch(plus, contactPitch);
minus = setContactPitch(minus, contactPitch);
Jplus = contactJacobian(plus, leg, materialPointEnabled);
Jminus = contactJacobian(minus, leg, materialPointEnabled);
bias = ((Jplus - Jminus)/(2*dt))*dq;
end

function Jc = contactJacobian(data, leg, materialPointEnabled)
Jc = zeros(6, 12);
for side = 1:2
    body = data.wheelBody(side);
    basis = data.contactBasis(:, :, side);
    axle = data.axle(:, side);
    rows = 3*side - 2:3*side;
    if materialPointEnabled
        % Eq. (4) applied to the full spatial wheel velocity at the
        % terrain-normal nominal contact point Upsilon = -r*n:
        % v_C = v_O + omega x Upsilon = (Jv + r*[n]x*Jw)*dq.
        normal = basis(:, 3);
        pointJacobian = data.Jv(:, :, body) ...
            + leg.r*skewMatrix(normal)*data.Jw(:, :, body);
        Jc(rows, :) = basis'*pointJacobian;
    else
        % Paper-reduced legacy rows. These equal Eq. (4) only when the
        % wheel axle is the contact lateral and angular velocity is spin.
        Jc(rows(1), :) = basis(:, 1)'*data.Jv(:, :, body) ...
            + leg.r*axle'*data.Jw(:, :, body);
        Jc(rows(2), :) = basis(:, 2)'*data.Jv(:, :, body);
        Jc(rows(3), :) = basis(:, 3)'*data.Jv(:, :, body);
    end
end
end

function matrix = skewMatrix(vector)
vector = vector(:);
matrix = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function [Dw, Dlambda, offset] = interactionWrenchMap(data, base, leg)
Dw = zeros(12, 12);
Dlambda = zeros(12, 6);
offset = zeros(12, 1);
for side = 1:2
    if side == 1
        wheelBody = 4;
    else
        wheelBody = 7;
    end
    rows = 6*side - 5:6*side;
    mass = data.mass(wheelBody);
    Jv = data.Jv(:, :, wheelBody);
    Jw = data.Jw(:, :, wheelBody);
    inertia = data.inertia(:, :, wheelBody);
    gravityForce = [0; -mass*base.g; 0];
    % The upper model lumps the torso and both links into its floating body.
    % Its per-side interface is therefore the wheel-to-rest-of-robot wrench
    % at the wheel centre, not the complete leg-on-torso hip wrench.
    physicalDw = [-mass*Jv; -inertia*Jw];
    physicalOffset = [gravityForce - mass*data.biasV(:, wheelBody); ...
        -inertia*data.biasW(:, wheelBody) ...
        - cross(data.omega(:, wheelBody), inertia*data.omega(:, wheelBody))];
    columns = 3*side - 2:3*side;
    basis = data.contactBasis(:, :, side);
    physicalDlambda = [basis; zeros(3, 3)];
    physicalDlambda(4:6, 1) = leg.r*data.axle(:, side);
    physicalToController = [1, 0, 0; 0, 0, 1; 0, 1, 0];
    physicalToBodyController = physicalToController*data.baseRotation';
    wrenchTransform = blkdiag( ...
        physicalToBodyController, physicalToBodyController);
    Dw(rows, :) = wrenchTransform*physicalDw;
    Dlambda(rows, columns) = wrenchTransform*physicalDlambda;
    offset(rows) = wrenchTransform*physicalOffset;
end
end

function [A, b] = spatialFrictionConstraints(mu, nz, idxLambda)
muPyramid = mu/sqrt(2);
A = zeros(8, nz);
b = zeros(8, 1);
for side = 1:2
    columns = idxLambda(3*side - 2:3*side);
    row = 4*side - 3;
    A(row, columns) = [1, 0, -muPyramid];
    A(row + 1, columns) = [-1, 0, -muPyramid];
    A(row + 2, columns) = [0, 1, -muPyramid];
    A(row + 3, columns) = [0, -1, -muPyramid];
end
end

function qddCommand = relativeLegAccelerationCommand(q, dq, qd, dqd, ...
        ddqd, baseState, baseQddCommand, ctrl)
qdRelative = [qd(1) - baseState(3); qd(2:3)];
dqdRelative = [dqd(1) - baseState(6); dqd(2:3)];
ddqdRelative = [ddqd(1) - baseQddCommand(3); ddqd(2:3)];
qddCommand = ddqdRelative + ctrl.Kd*(dqdRelative - dq) ...
    + ctrl.Kp*(qdRelative - q);
end

function lowerBound = kneeAccelerationLowerBound(q, dq, ctrl)
if ~getField(ctrl, "kneeGuardEnabled", false) || q(2) >= ctrl.kneeGuardMin
    lowerBound = -inf;
    return;
end
wn = 2*pi*ctrl.kneeGuardFrequencyHz;
lowerBound = -2*ctrl.kneeGuardDamping*wn*dq(2) ...
    - wn^2*(q(2) - ctrl.kneeGuardMin);
end

function [z, exitflag] = solveEqualityQp(H, f, Aeq, beq)
regularization = 1e-9;
rowScale = max(vecnorm(Aeq, 2, 2), 1e-9);
scaledAeq = Aeq./rowScale;
scaledBeq = beq./rowScale;
KKT = [H + regularization*eye(size(H)), scaledAeq'; ...
    scaledAeq, zeros(size(scaledAeq, 1))];
rhs = [-f; scaledBeq];
if rcond(KKT) < 1e-14
    solution = pinv(KKT)*rhs;
else
    solution = KKT\rhs;
end
z = solution(1:size(H, 1));
exitflag = double(all(isfinite(z)));
if exitflag == 0
    z = [];
end
end

function usable = isUsableQpCandidate( ...
        z, Aeq, beq, Aineq, bineq, lb, ub, tolerance)
usable = ~isempty(z) && all(isfinite(z));
if ~usable
    return;
end
usable = norm(Aeq*z - beq, inf) <= tolerance ...
    && (isempty(Aineq) || max(Aineq*z - bineq) <= tolerance) ...
    && max(lb - z) <= tolerance ...
    && max(z - ub) <= tolerance;
end

function inertia = linkInertia(mass, lengthValue, leg)
inertia = diag(mass/12 * [
    lengthValue^2 + leg.depth^2;
    leg.width^2 + leg.depth^2;
    lengthValue^2 + leg.width^2]);
end

function [R, dR, E] = rotationData(angles)
[R, dR, E] = controller_attitude_kinematics(angles);
end

function R = rotationZ(angle)
R = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; 0, 0, 1];
end

function rWorld = rotatePitch2D(rBody, theta)
rWorld = [cos(theta)*rBody(1) - sin(theta)*rBody(2); ...
    sin(theta)*rBody(1) + cos(theta)*rBody(2)];
end

function value = getField(s, name, fallback)
if isfield(s, name)
    value = s.(name);
else
    value = fallback;
end
end

function value = getVector(s, name, fallback, count)
value = getField(s, name, fallback);
value = value(:);
if numel(value) ~= count
    error("spatial_two_leg_qp_core:InvalidWeight", ...
        "%s must contain %d elements.", name, count);
end
end

function weight = normalizedWeight(s, weightName, weightDefault, ...
        scaleName, scaleDefault, count)
priority = getVector(s, weightName, weightDefault, count);
scale = getVector(s, scaleName, scaleDefault, count);
if any(~isfinite(priority) | priority < 0) ...
        || any(~isfinite(scale) | scale <= 0)
    error("spatial_two_leg_qp_core:InvalidNormalizedWeight", ...
        "%s must be nonnegative and %s must be positive.", ...
        weightName, scaleName);
end
weight = priority./scale.^2;
end
