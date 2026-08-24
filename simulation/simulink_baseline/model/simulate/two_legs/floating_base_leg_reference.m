function [qd, dqd, ddqd, debug] = floating_base_leg_reference(t, baseState, traj, leg, base, aH, FH_ext, updatePlanner, wheelReference)
%FLOATING_BASE_LEG_REFERENCE Floating-base-consistent wheel-leg reference.
%
% Scheme 1 converts the final upper-layer body force into a bounded wheel
% equilibrium. A second-order governor supplies consistent position,
% velocity, and acceleration references before IK.
%
% The two-link IK receives pO_ref - pH, so qh/qk stay geometrically
% consistent with the current floating-base pose instead of fighting it.

if nargin < 3 || isempty(traj)
    traj = evalin("base", "traj");
end
if nargin < 4 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 5 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 6 || isempty(aH)
    aH = zeros(2, 1);
else
    aH = double(aH(:));
end
if numel(aH) ~= 2
    error("floating_base_leg_reference:InvalidHipAcceleration", ...
        "aH must be a 2-element vector.");
end
if nargin < 7 || isempty(FH_ext)
    FH_ext = zeros(2, 1);
else
    FH_ext = double(FH_ext(:));
end
if numel(FH_ext) ~= 2
    error("floating_base_leg_reference:InvalidHipForce", ...
        "FH_ext must be a 2-element vector.");
end
if nargin < 8 || isempty(updatePlanner)
    updatePlanner = true;
else
    updatePlanner = logical(updatePlanner);
end
if nargin < 9
    wheelReference = [];
end
if ~isscalar(t) || ~isfinite(t)
    error("floating_base_leg_reference:InvalidTime", ...
        "t must be a finite scalar.");
end

baseState = double(baseState(:));
if numel(baseState) ~= 6
    error("floating_base_leg_reference:InvalidInput", ...
        "baseState must be [xB; zB; thetaB; dxB; dzB; dthetaB].");
end

theta = baseState(3);
dtheta = baseState(6);

rH = rotatePitch2D(base.rHBody(:), theta);
drdtheta = [-rH(2); rH(1)];

pH = baseState(1:2) + rH;
vH = baseState(4:5) + dtheta * drdtheta;

groundTop = getFieldOrDefault(base, "simscapeGroundTopY", 0);
% baseState is measured relative to the Planar Joint standard offset, while
% groundTop is a world coordinate. Express the wheel-center height in the
% same relative frame before subtracting the relative hip position.
worldYOffset = getFieldOrDefault(base, "simscapeWorldYOffset", 0);
wheelCenterZ = groundTop + leg.r - worldYOffset;

plannerMode = lower(string(getFieldOrDefault(traj, ...
    "wheelPositionPlanner", "qp_force")));
if plannerMode == "lqr"
    wheelReference = double(wheelReference(:));
    if numel(wheelReference) < 3 || any(~isfinite(wheelReference(1:3)))
        error("floating_base_leg_reference:MissingLqrReference", ...
            "The LQR planner requires [xiRef; dxiRef; ddxiRef].");
    end
    rXDes = wheelReference(1);
    drXDes = wheelReference(2);
    ddrXDes = wheelReference(3);
    loadShare = getFieldOrDefault(base, "symmetricLoadShare", 1);
    FBody = -FH_ext(:) / loadShare;
    aB = [FBody(1) / base.m; FBody(2) / base.m - base.g];
    plan = struct("plannerMode", plannerMode, "rXDes", rXDes, ...
        "drXDes", drXDes, "ddrXDes", ddrXDes, ...
        "FBody", FBody, "aB", aB);
elseif plannerMode == "qp_force"
    [rXDes, drXDes, ddrXDes, aB, plan] = plannedWheelOffset(t, ...
        baseState, rH, wheelCenterZ, FH_ext, traj, leg, base, updatePlanner);
    plan.plannerMode = plannerMode;
else
    error("floating_base_leg_reference:InvalidPlanner", ...
        "wheelPositionPlanner must be 'lqr' or 'qp_force'.");
end
pO = [rXDes - rH(1); wheelCenterZ - pH(2)];
pO = projectToReachableAnnulus(pO, leg);
vO = [baseState(4) + drXDes - vH(1); -vH(2)];
aO = [aB(1) + ddrXDes - aH(1); -aH(2)];

[qJoint, dqJoint, ddqJoint] = wheel_leg_inverse_kinematics(pO, vO, aO, leg);

wheelX = baseState(1) + rXDes;
wheelDx = baseState(4) + drXDes;
[qw, dqw, ddqw] = wheelSpinReference(wheelX, wheelDx, qJoint, ...
    dqJoint, ddqJoint, aB(1) + ddrXDes, traj, leg, base);

qd = [qJoint; qw];
dqd = [dqJoint; dqw];
ddqd = [ddqJoint; ddqw];
debug = plan;
debug.pO = pO;
debug.qJoint = qJoint;
end

function [rXDes, drXDes, ddrXDes, aB, plan] = plannedWheelOffset(t, ...
        baseState, rH, wheelCenterZ, FH_ext, traj, leg, base, updatePlanner)
FBody = -FH_ext(:);
aB = [FBody(1) / base.m; FBody(2) / base.m - base.g];

thetaEq = getFieldOrDefault(base, "thetaEq", 0);
rHEq = rotatePitch2D(base.rHBody(:), thetaEq);
rXEq = rHEq(1) + getFieldOrDefault(traj, "xO0", 0);

qkMin = getFieldOrDefault(traj, "wheelPositionKneeMin", 0);
qkMin = min(max(qkMin, 0), pi);
maxReach = sqrt(leg.L1^2 + leg.L2^2 + ...
    2 * leg.L1 * leg.L2 * cos(qkMin));
zOH = wheelCenterZ - (baseState(2) + rH(2));
geometryFeasible = abs(zOH) <= maxReach;
xOHMax = sqrt(max(0, maxReach^2 - zOH^2));
rXLower = rH(1) - xOHMax;
rXUpper = rH(1) + xOHMax;
rXNeutral = clamp(rXEq, rXLower, rXUpper);
deltaMax = min(rXNeutral - rXLower, rXUpper - rXNeutral);

height = max(baseState(2) - wheelCenterZ, 1e-3);
forceScale = getFieldOrDefault(traj, "wheelPositionForceScale", 1);
forceSource = lower(string(getFieldOrDefault(traj, ...
    "wheelPositionForceSource", "total_lqr_force")));
switch forceSource
    case "reference_acceleration"
        [~, baseAccelerationReference] = floating_base_reference(t);
        forcePlanningX = base.m * baseAccelerationReference(1);
    case "total_lqr_force"
        forcePlanningX = FBody(1);
    otherwise
        error("floating_base_leg_reference:InvalidForceSource", ...
            "Unsupported wheelPositionForceSource: %s", forceSource);
end
if deltaMax > eps
    force0 = base.m * base.g * deltaMax / height;
    rXEquilibrium = rXNeutral - deltaMax * tanh( ...
        forceScale * forcePlanningX / force0);
else
    force0 = 0;
    rXEquilibrium = rXNeutral;
end

planningEnabled = getFieldOrDefault(traj, "wheelPositionPlanning", false);
if planningEnabled
    [rXDes, drXDes, ddrXDes] = wheelPositionGovernor(t, ...
        rXEquilibrium, rXNeutral, rXLower, rXUpper, traj, base, ...
        updatePlanner);
else
    rXDes = rXNeutral;
    drXDes = 0;
    ddrXDes = 0;
end

plan = struct("rXEquilibrium", rXEquilibrium, ...
    "rXNeutral", rXNeutral, "rXDes", rXDes, ...
    "drXDes", drXDes, "ddrXDes", ddrXDes, ...
    "rXLower", rXLower, "rXUpper", rXUpper, ...
    "force0", force0, "geometryFeasible", geometryFeasible, ...
    "forceSource", forceSource, "forcePlanningX", forcePlanningX, ...
    "zOH", zOH, "maxReach", maxReach, "FBody", FBody, "aB", aB);
end

function [rDes, vDes, aDes] = wheelPositionGovernor(t, rEq, rNeutral, ...
        rLower, rUpper, traj, base, updateState)
persistent state

if ~updateState
    if isempty(state) || t <= 0
        rDes = rNeutral;
        vDes = 0;
        aDes = 0;
    else
        rDes = clamp(state.r, rLower, rUpper);
        vDes = state.v;
        aDes = state.a;
    end
    return;
end

if isempty(state) || t <= 0 || t < state.t
    state = struct("t", t, "r", rNeutral, "v", 0, "a", 0);
    rDes = state.r;
    vDes = state.v;
    aDes = state.a;
    return;
end

dt = t - state.t;
if dt <= max(1e-12, eps(max(1, abs(t))))
    rDes = clamp(state.r, rLower, rUpper);
    vDes = state.v;
    aDes = state.a;
    return;
end

frequencyHz = max(0, getFieldOrDefault(traj, ...
    "wheelPositionFrequencyHz", 0.8));
zeta = max(0, getFieldOrDefault(traj, "wheelPositionDamping", 1));
vMax = max(0, getFieldOrDefault(traj, ...
    "wheelPositionVelocityMax", inf));
aMax = max(0, getFieldOrDefault(traj, ...
    "wheelPositionAccelerationMax", inf));
omega = 2 * pi * frequencyHz;
nominalTs = max(1e-6, getFieldOrDefault(base, "Ts", dt));
nSteps = max(1, ceil(dt / nominalTs - 1e-9));
h = dt / nSteps;

r = clamp(state.r, rLower, rUpper);
v = clamp(state.v, -vMax, vMax);
a = 0;
for idx = 1:nSteps
    vPrevious = v;
    a = clamp(omega^2 * (rEq - r) - 2 * zeta * omega * v, ...
        -aMax, aMax);
    v = clamp(v + h * a, -vMax, vMax);
    rCandidate = r + h * v;
    r = clamp(rCandidate, rLower, rUpper);
    if r ~= rCandidate
        v = 0;
    end
    a = (v - vPrevious) / h;
end

state = struct("t", t, "r", r, "v", v, "a", a);
rDes = r;
vDes = v;
aDes = a;
end

function [qw, dqw, ddqw] = wheelSpinReference(wheelX, wheelDx, qJoint, ...
    dqJoint, ddqJoint, wheelDdx, traj, leg, base)
qw0 = getFieldOrDefault(traj, "qw0", 0);
thetaWheelBase0 = getFieldOrDefault(traj, "thetaWheelBase0", sum(qJoint));

thetaEq = getFieldOrDefault(base, "thetaEq", 0);
rHEq = rotatePitch2D(base.rHBody(:), thetaEq);
xEq = getFieldOrDefault(base, "xEq", zeros(6, 1));
wheelX0 = xEq(1) + rHEq(1) + getFieldOrDefault(traj, "xO0", 0);

qw = qw0 - (wheelX - wheelX0) / leg.r ...
    - (sum(qJoint) - thetaWheelBase0);
dqw = -wheelDx / leg.r - sum(dqJoint);
ddqw = -wheelDdx / leg.r - sum(ddqJoint);
end

function p = projectToReachableAnnulus(p, leg)
reach = norm(p);
if reach < eps
    p = [0; -(abs(leg.L1 - leg.L2) + 1e-3)];
    return;
end

margin = 1e-3;
minReach = abs(leg.L1 - leg.L2) + margin;
maxReach = leg.L1 + leg.L2 - margin;
targetReach = min(max(reach, minReach), maxReach);
p = p * (targetReach / reach);
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 - sin(theta)*rz0;
    sin(theta)*rx0 + cos(theta)*rz0
];
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end

function value = clamp(value, lower, upper)
value = min(max(value, lower), upper);
end
