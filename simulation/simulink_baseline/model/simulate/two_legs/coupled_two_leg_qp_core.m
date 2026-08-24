function [tau, debug] = coupled_two_leg_qp_core(x, mode)
%COUPLED_TWO_LEG_QP_CORE Floating-base QP shared by both wheel legs.
%
% x = [t; xB; zB; thetaB; dxB; dzB; dthetaB;
%      qL(3); dqL(3); qR(3); dqR(3); upperCommand(3 or 12); wheelRef(4)]
% The 12D full-model command is [FL(3); TL(3); FR(3); TR(3)].

persistent qpOptions zWarm zWarmCommon

if nargin < 2 || isempty(mode)
    mode = "full";
end
mode = string(mode);
if mode ~= "full" && mode ~= "common"
    error("coupled_two_leg_qp_core:InvalidMode", ...
        "Mode must be 'full' or 'common'.");
end

x = double(x(:));
if numel(x) == 46
    if mode ~= "full"
        error("coupled_two_leg_qp_core:InvalidMode", ...
            "The 46D spatial input is only valid in full mode.");
    end
    [tau, debug] = spatial_two_leg_qp_core(x);
    return;
end
if numel(x) ~= 26 && numel(x) ~= 35
    error("coupled_two_leg_qp_core:InvalidInput", ...
        "Expected the 26D common or 35D full 8-DoF controller input.");
end

leg = evalin("base", "leg");
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

if mode == "common"
    if isfield(ctrl, "commonModeKp")
        ctrl.Kp = ctrl.commonModeKp;
    end
    if isfield(ctrl, "commonModeKd")
        ctrl.Kd = ctrl.commonModeKd;
    end
end

t = x(1);
baseState = x(2:7);
qLeft = x(8:10);
dqLeft = x(11:13);
qRight = x(14:16);
dqRight = x(17:19);
upperCommand = x(20:end-4);
wheelReference = x(end-3:end);
if numel(upperCommand) == 12
    wrenchLeftCommand = upperCommand([1, 3, 5]);
    wrenchRightCommand = upperCommand([7, 9, 11]);
    wrenchCommand = wrenchLeftCommand + wrenchRightCommand;
else
    wrenchLeftCommand = zeros(3, 1);
    wrenchRightCommand = zeros(3, 1);
    wrenchCommand = [-upperCommand(1:2); upperCommand(3)];
end

if mode == "common"
    qCommon = 0.5 * (qLeft + qRight);
    dqCommon = 0.5 * (dqLeft + dqRight);
    qLeft = qCommon;
    qRight = qCommon;
    dqLeft = dqCommon;
    dqRight = dqCommon;
end

qFull = [baseState(1:3); qLeft; qRight];
dqFull = [baseState(4:6); dqLeft; dqRight];
[M, h, Jc, dJcDq, wheelData] = floatingBaseDynamics( ...
    qFull, dqFull, base, leg);
[xiDifferential, dxiDifferential] = differentialWheelState( ...
    qLeft, dqLeft, qRight, dqRight, baseState, leg, ctrl);
if isfield(ctrl, "commonModeJointDamping")
    jointDamping = ctrl.commonModeJointDamping(:);
    h(4:6) = h(4:6) + jointDamping .* dqLeft;
    h(7:9) = h(7:9) + jointDamping .* dqRight;
end

commandModel = base;
if evalin("base", "exist('baseNmpc', 'var')") == 1
    baseNmpc = evalin("base", "baseNmpc");
    commandModel.m = baseNmpc.model.m;
    commandModel.Iyy = baseNmpc.model.Iyy;
end
[baseQddCmd, aHCmd] = desiredBaseAcceleration(baseState, ...
    wrenchCommand, commandModel);
perLegForce = -base.symmetricLoadShare*wrenchCommand(1:2);
[qd, dqd, ddqd] = floating_base_leg_reference(t, ...
    baseState, traj, leg, base, aHCmd, perLegForce, true, ...
    wheelReference);
if mode == "common"
    qddLeftCmd = relativeLegAccelerationCommand(qLeft, dqLeft, ...
        qd, dqd, ddqd, baseState, baseQddCmd, ctrl);
    qddRightCmd = relativeLegAccelerationCommand(qRight, dqRight, ...
        qd, dqd, ddqd, baseState, baseQddCmd, ctrl);
else
    commonCtrl = ctrl;
    commonCtrl.Kp = getCtrlField(ctrl, "commonModeKp", ctrl.Kp);
    commonCtrl.Kd = getCtrlField(ctrl, "commonModeKd", ctrl.Kd);
    qCommon = 0.5 * (qLeft + qRight);
    dqCommon = 0.5 * (dqLeft + dqRight);
    qDifferential = 0.5 * (qLeft - qRight);
    dqDifferential = 0.5 * (dqLeft - dqRight);
    qddCommonCmd = relativeLegAccelerationCommand(qCommon, dqCommon, ...
        qd, dqd, ddqd, baseState, baseQddCmd, commonCtrl);
    differentialKp = getCtrlField(ctrl, ...
        "differentialModeKp", commonCtrl.Kp);
    differentialKd = getCtrlField(ctrl, ...
        "differentialModeKd", commonCtrl.Kd);
    qddDifferentialCmd = -differentialKp*qDifferential ...
        - differentialKd*dqDifferential;
    qddLeftCmd = qddCommonCmd + qddDifferentialCmd;
    qddRightCmd = qddCommonCmd - qddDifferentialCmd;
end
kneeMinLeft = kneeAccelerationLowerBound(qLeft, dqLeft, ctrl);
qddLeftCmd(2) = max(qddLeftCmd(2), kneeMinLeft);
kneeMinRight = kneeAccelerationLowerBound(qRight, dqRight, ctrl);
qddRightCmd(2) = max(qddRightCmd(2), kneeMinRight);

% Coupled z = [qddBase(3); qddLeft(3); qddRight(3); ...
%              tauLeft(3); tauRight(3); FcLeft(2); FcRight(2); sW(3)].
% The base stays planar (x, z, pitch), while independent leg variables let
% the QP actively reject the otherwise-uncontrolled left/right difference.
wBaseQdd = getCtrlVec(ctrl, "qpWbaseQdd", 1e-3 * ones(3, 1));
wLegQdd = getCtrlVec(ctrl, "qpWqdd", ones(3, 1));
% Preserve the validated common-mode priority and give the independent
% left/right mode its own explicit soft-task weight.
wCommonQdd = getCtrlVec(ctrl, "commonModeQpWqdd", wLegQdd);
wDifferentialQdd = getCtrlVec(ctrl, ...
    "differentialModeQpWqdd", wCommonQdd);
if mode == "common"
    wLegQdd = wCommonQdd;
end
wTau = getCtrlVec(ctrl, "qpWtau", 1e-5 * ones(3, 1));
wFc = getCtrlVec(ctrl, "qpWFc", 1e-5 * ones(2, 1));
wDifferentialFc = getCtrlVec(ctrl, "differentialModeQpWFc", wFc);
slackScale = getCtrlVec(ctrl, "qpSlackScale", [140; 140; 160]);
wSlack = getCtrlVec(ctrl, "qpWslack", 1e6 * ones(3, 1));
qddTarget = [baseQddCmd; qddLeftCmd; qddRightCmd];
qddCommonCmd = 0.5 * (qddLeftCmd + qddRightCmd);
qddDifferentialCmd = 0.5 * (qddLeftCmd - qddRightCmd);
weights = [wBaseQdd; zeros(6, 1); wTau; wTau; zeros(4, 1); ...
    wSlack ./ (slackScale.^2)];
H = diag(weights) + 1e-9 * eye(22);
commonBlock = diag(0.5 * (wCommonQdd + wDifferentialQdd));
crossBlock = diag(0.5 * (wCommonQdd - wDifferentialQdd));
H(4:9, 4:9) = [commonBlock, crossBlock; crossBlock, commonBlock] ...
    + 1e-9 * eye(6);
commonForceBlock = diag(0.5 * (wFc + wDifferentialFc));
crossForceBlock = diag(0.5 * (wFc - wDifferentialFc));
H(16:19, 16:19) = [commonForceBlock, crossForceBlock; ...
    crossForceBlock, commonForceBlock] + 1e-9 * eye(4);
f = zeros(22, 1);
f(1:3) = -wBaseQdd .* baseQddCmd;
f(4:6) = -wCommonQdd .* qddCommonCmd ...
    - wDifferentialQdd .* qddDifferentialCmd;
f(7:9) = -wCommonQdd .* qddCommonCmd ...
    + wDifferentialQdd .* qddDifferentialCmd;
contactForceDifferentialCommand = zeros(2, 1);
rollYawMomentCommand = zeros(2, 1);
if mode == "full" && numel(upperCommand) == 12
    contactForceDifferentialCommand = 0.5*( ...
        wrenchLeftCommand(1:2) - wrenchRightCommand(1:2));
    lateralHalfSpacing = abs(base.body.hipPositionBodyLeft3D(3));
    if lateralHalfSpacing <= 0
        error("coupled_two_leg_qp_core:InvalidHipSpacing", ...
            "The lateral hip half-spacing must be positive.");
    end
    % Controller-frame moments produced by the left/right force split.
    rollYawMomentCommand = 2*lateralHalfSpacing ...
        * [contactForceDifferentialCommand(2); ...
        -contactForceDifferentialCommand(1)];
    f(16:17) = -wDifferentialFc ...
        .* contactForceDifferentialCommand;
    f(18:19) = wDifferentialFc ...
        .* contactForceDifferentialCommand;
end
xiDifferentialCmd = -getCtrlField(ctrl, ...
    "differentialWheelPositionKp", 0) * xiDifferential ...
    - getCtrlField(ctrl, "differentialWheelPositionKd", 0) ...
    * dxiDifferential;
JxiDifferential = 0.5 * (wheelData.J(1, :, 1) ...
    - wheelData.J(1, :, 2));
xiDifferentialBias = 0.5 * (wheelData.bias(1, 1) ...
    - wheelData.bias(1, 2));
if mode == "full"
    wXiDifferential = getCtrlField(ctrl, ...
        "differentialWheelPositionQpWeight", 0);
    xiTarget = xiDifferentialCmd - xiDifferentialBias;
    H(1:9, 1:9) = H(1:9, 1:9) ...
        + wXiDifferential * (JxiDifferential' * JxiDifferential);
    f(1:9) = f(1:9) ...
        - wXiDifferential * JxiDifferential' * xiTarget;
end
tauSign = getCtrlVec(ctrl, "tauSign", ones(3, 1));
S = zeros(9, 6);
S(4:6, 1:3) = diag(tauSign);
S(7:9, 4:6) = diag(tauSign);

Kc = getCtrlField(ctrl, "constraintVelocityGain", 0);
contactRhs = -dJcDq - Kc * (Jc * dqFull);

if mode == "common"
    [tau, debug, zWarmCommon, qpOptions] = solveCommonModeQp( ...
        t, baseQddCmd, qddLeftCmd, qddRightCmd, ...
        kneeMinLeft, kneeMinRight, M, h, Jc, contactRhs, ...
        S, wheelData, wrenchCommand, ...
        wBaseQdd, wLegQdd, wTau, wFc, wSlack, slackScale, ...
        base, leg, ctrl, zWarmCommon, qpOptions);
    return;
end

[Dw, Dforce, wrenchOffset] = fullInteractionWrenchMap(wheelData, leg);
wrenchRhs = wrenchCommand - wrenchOffset;

Aeq = [
    M, -S, -Jc', zeros(9, 3);
    Jc, zeros(4, 13);
    Dw, zeros(3, 6), Dforce, -eye(3)
];
beq = [-h; contactRhs; wrenchRhs];

[Aineq, bineq] = frictionConstraints(getCtrlField(ctrl, "mu", 0.8));
if isfinite(kneeMinLeft)
    row = zeros(1, 22);
    row(5) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMinLeft];
end
if isfinite(kneeMinRight)
    row = zeros(1, 22);
    row(8) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMinRight];
end

tauMax = ctrl.tauMax(:);
lb = [-inf(9, 1); -tauMax; -tauMax; -inf; 0; -inf; 0; -inf(3, 1)];
ub = [ inf(9, 1);  tauMax;  tauMax;  inf; inf;  inf; inf;  inf(3, 1)];

robotMass = base.m + 2 * (leg.m1 + leg.m2 + leg.mw);
z0 = [qddTarget; zeros(6, 1); 0; robotMass*base.g/2; ...
    0; robotMass*base.g/2; zeros(3, 1)];
if t <= 0 || isempty(zWarm) || numel(zWarm) ~= 22 ...
        || any(~isfinite(zWarm))
    zWarm = z0;
elseif getCtrlField(ctrl, "qpWarmStart", true)
    z0 = zWarm;
end

exitflag = -999;
if string(getCtrlField(ctrl, "qpSolver", "quadprog")) == "equality"
    [z, exitflag] = solveEqualityQp(H, f, Aeq, beq);
else
    try
        if isempty(qpOptions)
            qpOptions = optimoptions("quadprog", "Display", "off", ...
                "Algorithm", "interior-point-convex");
        end
        [z, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, ...
            lb, ub, z0, qpOptions);
    catch
        z = [];
    end
end

if isempty(z) || exitflag <= 0 || any(~isfinite(z))
    [z, fallbackFlag] = solveEqualityQp(H, f, Aeq, beq);
    if isempty(z)
        z = z0;
    end
    z(10:12) = min(max(z(10:12), -tauMax), tauMax);
    z(13:15) = min(max(z(13:15), -tauMax), tauMax);
    z(17) = max(z(17), 0);
    z(19) = max(z(19), 0);
    exitflag = min(-1, fallbackFlag);
else
    zWarm = z;
end

tau = z(10:15);
qddSolution = z(1:9);
contactForce = z(16:19);
wrenchSlack = z(20:22);
wrenchFeasible = Dw*qddSolution + Dforce*contactForce + wrenchOffset;

eqResidual = Aeq * z - beq;
ineqResidual = Aineq * z - bineq;
debug = struct();
debug.qdd = qddSolution;
debug.contactForce = contactForce;
debug.FcLeft = contactForce(1:2);
debug.FcRight = contactForce(3:4);
debug.exitflag = exitflag;
debug.qpFeasible = exitflag > 0 && norm(eqResidual, inf) < 1e-6 ...
    && (isempty(ineqResidual) || max(ineqResidual) < 1e-6);
debug.wrenchCommand = wrenchCommand;
debug.wrenchSlack = wrenchSlack;
debug.wrenchFeasible = wrenchFeasible;
debug.wrenchSlackNorm = norm(wrenchSlack ./ slackScale);
debug.dynamicsResidual = M*qddSolution + h ...
    - S*tau - Jc'*contactForce;
debug.symmetryQddError = qddSolution(4:6) - qddSolution(7:9);
debug.qddCommon = 0.5 * (qddSolution(4:6) + qddSolution(7:9));
debug.qddDifferential = 0.5 * debug.symmetryQddError;
debug.qddDifferentialCommand = qddDifferentialCmd;
debug.tauCommon = 0.5 * (tau(1:3) + tau(4:6));
debug.tauDifferential = 0.5 * (tau(1:3) - tau(4:6));
debug.contactForceCommon = 0.5 * (debug.FcLeft + debug.FcRight);
debug.contactForceDifferential = 0.5 * (debug.FcLeft - debug.FcRight);
debug.rollYawMomentCommand = rollYawMomentCommand;
debug.contactForceDifferentialCommand = contactForceDifferentialCommand;
debug.xiDifferential = xiDifferential;
debug.dxiDifferential = dxiDifferential;
debug.xiDifferentialCommand = xiDifferentialCmd;
debug.xiDifferentialAcceleration = JxiDifferential*qddSolution ...
    + xiDifferentialBias;
mu = getCtrlField(ctrl, "mu", 0.8);
debug.frictionMargin = [
    mu*debug.FcLeft(2) - abs(debug.FcLeft(1));
    mu*debug.FcRight(2) - abs(debug.FcRight(1))
];
debug.torqueMargin = [tauMax; tauMax] - abs(tau);
debug.massMatrix = M;
debug.commonMode = false;
end

function [tau, debug, zWarm, qpOptions] = solveCommonModeQp( ...
        t, baseQddCmd, qddLeftCmd, qddRightCmd, ...
        kneeMinLeft, kneeMinRight, M, h, Jc, contactRhs, ...
        S, wheelData, wrenchCommand, ...
        wBaseQdd, wLegQdd, wTau, wFc, wSlack, slackScale, ...
        base, leg, ctrl, zWarm, qpOptions)
% Restrict the full two-leg dynamics to qL=qR without approximating the
% doubled mass, actuator, or contact contributions.
Tq = [eye(3), zeros(3); zeros(3), eye(3); zeros(3), eye(3)];
Ttau = [eye(3); eye(3)];
Tlambda = [eye(2); eye(2)];

Mr = Tq' * M * Tq;
hr = Tq' * h;
Sr = Tq' * S * Ttau;
Jforce = Tq' * Jc' * Tlambda;
Jrolling = Jc(1:2, :) * Tq;
[DwcQdd, DwcForce, wrenchOffset] = commonInteractionWrenchMap( ...
    wheelData, Tq, leg);
wrenchRhs = wrenchCommand - wrenchOffset;

qddLegCmd = 0.5 * (qddLeftCmd + qddRightCmd);
qddTarget = [baseQddCmd; qddLegCmd];
weights = [wBaseQdd; 2*wLegQdd; 2*wTau; 2*wFc; ...
    wSlack ./ (slackScale.^2)];
H = diag(weights) + 1e-9 * eye(14);
f = [-weights(1:6) .* qddTarget; zeros(8, 1)];

Aeq = [
    Mr, -Sr, -Jforce, zeros(6, 3);
    Jrolling, zeros(2, 8);
    DwcQdd, zeros(3, 3), DwcForce, -eye(3)
];
beq = [-hr; contactRhs(1:2); wrenchRhs];

mu = getCtrlField(ctrl, "mu", 0.8);
Aineq = zeros(3, 14);
Aineq(1, 10:11) = [1, -mu];
Aineq(2, 10:11) = [-1, -mu];
Aineq(3, 11) = -1;
bineq = zeros(3, 1);
kneeMin = max(kneeMinLeft, kneeMinRight);
if isfinite(kneeMin)
    row = zeros(1, 14);
    row(5) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMin];
end

tauMax = ctrl.tauMax(:);
lb = [-inf(6, 1); -tauMax; -inf; 0; -inf(3, 1)];
ub = [ inf(6, 1);  tauMax;  inf; inf;  inf(3, 1)];
% In strict common mode there is no differential posture that can recover a
% freely relaxed body moment. Keep Fx/Fz soft and bound the moment error.
momentSlackMax = getCtrlField(ctrl, "commonModeMomentSlackMax", 0.5);
lb(14) = -momentSlackMax;
ub(14) = momentSlackMax;
robotMass = base.m + 2 * (leg.m1 + leg.m2 + leg.mw);
z0 = [qddTarget; zeros(3, 1); 0; robotMass*base.g/2; zeros(3, 1)];
if t <= 0 || isempty(zWarm) || numel(zWarm) ~= 14 ...
        || any(~isfinite(zWarm))
    zWarm = z0;
elseif getCtrlField(ctrl, "qpWarmStart", true)
    z0 = zWarm;
end

exitflag = -999;
if string(getCtrlField(ctrl, "qpSolver", "quadprog")) == "equality"
    [z, exitflag] = solveEqualityQp(H, f, Aeq, beq);
else
    try
        if isempty(qpOptions)
            qpOptions = optimoptions("quadprog", "Display", "off", ...
                "Algorithm", "interior-point-convex");
        end
        [z, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, ...
            lb, ub, z0, qpOptions);
    catch
        z = [];
    end
end

if isempty(z) || exitflag <= 0 || any(~isfinite(z))
    [z, fallbackFlag] = solveEqualityQp(H, f, Aeq, beq);
    if isempty(z)
        z = z0;
    end
    z(7:9) = min(max(z(7:9), -tauMax), tauMax);
    z(11) = max(z(11), 0);
    exitflag = min(-1, fallbackFlag);
else
    zWarm = z;
end

qddReduced = z(1:6);
qddSolution = Tq * qddReduced;
tauPerLeg = z(7:9);
tau = Ttau * tauPerLeg;
contactPerLeg = z(10:11);
contactForce = Tlambda * contactPerLeg;
wrenchSlack = z(12:14);
wrenchFeasible = DwcQdd*qddReduced ...
    + DwcForce*contactPerLeg + wrenchOffset;

eqResidual = Aeq * z - beq;
ineqResidual = Aineq * z - bineq;
debug = struct();
debug.qdd = qddSolution;
debug.contactForce = contactForce;
debug.FcLeft = contactPerLeg;
debug.FcRight = contactPerLeg;
debug.exitflag = exitflag;
debug.qpFeasible = exitflag > 0 && norm(eqResidual, inf) < 1e-6 ...
    && (isempty(ineqResidual) || max(ineqResidual) < 1e-6);
debug.wrenchCommand = wrenchRhs + wrenchOffset;
debug.wrenchSlack = wrenchSlack;
debug.wrenchFeasible = wrenchFeasible;
debug.wrenchSlackNorm = norm(wrenchSlack ./ slackScale);
debug.dynamicsResidual = M*qddSolution + h ...
    - S*tau - Jc'*contactForce;
debug.symmetryQddError = zeros(3, 1);
debug.massMatrix = Mr;
debug.commonMode = true;
debug.tauTotal = 2 * tauPerLeg;
debug.contactForceTotal = 2 * contactPerLeg;
end

function qddCmd = relativeLegAccelerationCommand(q, dq, qd, dqd, ...
        ddqd, baseState, baseQddCmd, ctrl)
qdRelative = [qd(1) - baseState(3); qd(2:3)];
dqdRelative = [dqd(1) - baseState(6); dqd(2:3)];
ddqdRelative = [ddqd(1) - baseQddCmd(3); ddqd(2:3)];
qddCmd = ddqdRelative + ctrl.Kd * (dqdRelative - dq) ...
    + ctrl.Kp * (qdRelative - q);
end

function [baseQdd, aH] = desiredBaseAcceleration(baseState, wrench, base)
theta = baseState(3);
dtheta = baseState(6);
rH = rotatePitch2D(base.rHBody(:), theta);
drdtheta = [-rH(2); rH(1)];
d2rdtheta2 = -rH;
baseQdd = [
    wrench(1) / base.m;
    wrench(2) / base.m - base.g;
    (rH(1)*wrench(2) - rH(2)*wrench(1) + wrench(3)) / base.Iyy
];
aH = baseQdd(1:2) + baseQdd(3)*drdtheta ...
    + dtheta^2*d2rdtheta2;
end

function [M, h, Jc, dJcDq, wheelData] = floatingBaseDynamics(q, dq, base, leg)
n = 9;
M = zeros(n);
h = zeros(n, 1);
M(1, 1) = base.m;
M(2, 2) = base.m;
M(3, 3) = base.Iyy;
h(2) = base.m * base.g;
Jc = zeros(4, n);
dJcDq = zeros(4, 1);
wheelData.J = zeros(2, n, 2);
wheelData.bias = zeros(2, 2);
wheelData.W = zeros(1, n, 2);

theta = q(3);
dtheta = dq(3);
rH = rotatePitch2D(base.rHBody(:), theta);
JHip = zeros(2, n);
JHip(:, 1:2) = eye(2);
JHip(:, 3) = [-rH(2); rH(1)];
hipBias = -rH * dtheta^2;

for side = 1:2
    first = 4 + 3*(side - 1);
    qh = theta + q(first);
    q2 = qh + q(first + 1);
    dqh = dtheta + dq(first);
    dq2 = dqh + dq(first + 1);

    [J1, b1] = pointKinematics(JHip, hipBias, first, ...
        leg.c1, 0, qh, q2, dqh, dq2);
    [J2, b2] = pointKinematics(JHip, hipBias, first, ...
        leg.L1, leg.c2, qh, q2, dqh, dq2);
    [Jw, bw] = pointKinematics(JHip, hipBias, first, ...
        leg.L1, leg.L2, qh, q2, dqh, dq2);
    W1 = angularJacobian(n, first, 0);
    W2 = angularJacobian(n, first, 1);
    Ww = angularJacobian(n, first, 2);
    wheelData.J(:, :, side) = Jw;
    wheelData.bias(:, side) = bw;
    wheelData.W(:, :, side) = Ww;

    M = M + leg.m1*(J1'*J1) + leg.I1*(W1'*W1) ...
        + leg.m2*(J2'*J2) + leg.I2*(W2'*W2) ...
        + leg.mw*(Jw'*Jw) + leg.Iw*(Ww'*Ww);
    gravity = [0; base.g];
    h = h + J1'*(leg.m1*(b1 + gravity)) ...
        + J2'*(leg.m2*(b2 + gravity)) ...
        + Jw'*(leg.mw*(bw + gravity));

    rows = 2*side - 1:2*side;
    Jc(rows(1), :) = Jw(1, :) + leg.r * Ww;
    Jc(rows(2), :) = Jw(2, :);
    dJcDq(rows) = bw;
end
M = (M + M') / 2;
end

function [xiDifferential, dxiDifferential] = differentialWheelState( ...
        qLeft, dqLeft, qRight, dqRight, baseState, leg, ctrl)
pitchSign = getCtrlField(ctrl, "basePitchToAbsHipSign", 1);
qLeftAbsolute = qLeft;
qRightAbsolute = qRight;
dqLeftAbsolute = dqLeft;
dqRightAbsolute = dqRight;
qLeftAbsolute(1) = qLeftAbsolute(1) + pitchSign*baseState(3);
qRightAbsolute(1) = qRightAbsolute(1) + pitchSign*baseState(3);
dqLeftAbsolute(1) = dqLeftAbsolute(1) + pitchSign*baseState(6);
dqRightAbsolute(1) = dqRightAbsolute(1) + pitchSign*baseState(6);
left = wheel_leg_kinematics(qLeftAbsolute, dqLeftAbsolute, [], leg);
right = wheel_leg_kinematics(qRightAbsolute, dqRightAbsolute, [], leg);
xiDifferential = 0.5 * (left.pO(1) - right.pO(1));
dxiDifferential = 0.5 * (left.vO(1) - right.vO(1));
end

function [Dqdd, Dforce, offset] = fullInteractionWrenchMap(wheelData, leg)
% Independent-side form of the validated strict common-mode wrench map.
Dqdd = zeros(3, 9);
Dforce = zeros(3, 4);
offset = zeros(3, 1);
B = [1, 0; 0, 1; leg.r, 0];
for side = 1:2
    J = wheelData.J(:, :, side);
    W = wheelData.W(:, :, side);
    bias = wheelData.bias(:, side);
    Dqdd = Dqdd + [-leg.mw*J(1, :); -leg.mw*J(2, :); -leg.Iw*W];
    Dforce(:, 2*side-1:2*side) = B;
    offset = offset + [-leg.mw*bias(1); ...
        -leg.mw*bias(2) - leg.mw*leg.g; 0];
end
end

function [Dqdd, Dforce, offset] = commonInteractionWrenchMap( ...
        wheelData, Tq, leg)
% Total wheel-to-body interaction wrench from the two wheel rigid bodies.
Dqdd = zeros(3, 6);
Dforce = zeros(3, 2);
offset = zeros(3, 1);
for side = 1:2
    J = wheelData.J(:, :, side) * Tq;
    W = wheelData.W(:, :, side) * Tq;
    bias = wheelData.bias(:, side);
    Dqdd = Dqdd + [-leg.mw*J(1, :); -leg.mw*J(2, :); -leg.Iw*W];
    Dforce = Dforce + [1, 0; 0, 1; leg.r, 0];
    offset = offset + [-leg.mw*bias(1); ...
        -leg.mw*bias(2) - leg.mw*leg.g; 0];
end
end

function [J, bias] = pointKinematics(JHip, hipBias, first, ...
        a1, a2, qh, q2, dqh, dq2)
J = JHip;
jh = [a1*cos(qh) + a2*cos(q2); ...
      a1*sin(qh) + a2*sin(q2)];
jk = [a2*cos(q2); a2*sin(q2)];
J(:, 3) = J(:, 3) + jh;
J(:, first) = jh;
J(:, first + 1) = jk;
bias = hipBias + [
    -a1*sin(qh)*dqh^2 - a2*sin(q2)*dq2^2;
     a1*cos(qh)*dqh^2 + a2*cos(q2)*dq2^2
];
end

function W = angularJacobian(n, first, relativeJointCount)
W = zeros(1, n);
W(3) = 1;
W(first) = 1;
if relativeJointCount >= 1
    W(first + 1) = 1;
end
if relativeJointCount >= 2
    W(first + 2) = 1;
end
end

function [A, b] = frictionConstraints(mu)
A = zeros(6, 22);
A(1, 16:17) = [1, -mu];
A(2, 16:17) = [-1, -mu];
A(3, 17) = -1;
A(4, 18:19) = [1, -mu];
A(5, 18:19) = [-1, -mu];
A(6, 19) = -1;
b = zeros(6, 1);
end

function qddMin = kneeAccelerationLowerBound(q, dq, ctrl)
if ~getCtrlField(ctrl, "kneeGuardEnabled", false)
    qddMin = -inf;
    return;
end
qMin = getCtrlField(ctrl, "kneeGuardMin", 0);
frequencyHz = max(0, getCtrlField(ctrl, "kneeGuardFrequencyHz", 3));
zeta = max(0, getCtrlField(ctrl, "kneeGuardDamping", 1));
omega = 2 * pi * frequencyHz;
qddMin = -2*zeta*omega*dq(2) - omega^2*(q(2) - qMin);
end

function [z, exitflag] = solveEqualityQp(H, f, Aeq, beq)
n = size(H, 1);
p = size(Aeq, 1);
KKT = [H, Aeq'; Aeq, zeros(p, p)];
rhs = [-f; beq];
if rcond(KKT) < 1e-12
    KKT = KKT + 1e-9 * eye(size(KKT));
end
solution = KKT \ rhs;
z = solution(1:n);
if all(isfinite(z))
    exitflag = 1;
else
    z = [];
    exitflag = -1;
end
end

function rWorld = rotatePitch2D(rBody, theta)
rWorld = [
    cos(theta)*rBody(1) - sin(theta)*rBody(2);
    sin(theta)*rBody(1) + cos(theta)*rBody(2)
];
end

function value = getCtrlField(ctrl, fieldName, defaultValue)
if isfield(ctrl, fieldName)
    value = ctrl.(fieldName);
else
    value = defaultValue;
end
end

function value = getCtrlVec(ctrl, fieldName, defaultValue)
value = getCtrlField(ctrl, fieldName, defaultValue);
value = value(:);
end
