function [xRef, aRef] = floating_base_reference(t, baseLqr)
%FLOATING_BASE_REFERENCE Configurable round-trip planar base reference.
%
% xRef = [x; z; theta; dx; dz; dtheta]
% aRef = [ddx; ddz; ddtheta]

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

xRef = baseLqr.xRef(:);
aRef = zeros(3, 1);
trajectory = getFieldOrDefault(baseLqr, "trajectory", struct());
if ~getFieldOrDefault(trajectory, "enabled", false) || ~isfinite(t)
    return;
end

mode = lower(string(getFieldOrDefault(trajectory, "mode", "stand")));
switch mode
    case "stand"
        return;
    case {"velocity", "velocity_round_trip"}
        [xOffset, dxRef, ddxRef] = velocityRoundTrip(t, trajectory);
        xRef(1) = xRef(1) + xOffset;
        xRef(4) = xRef(4) + dxRef;
        aRef(1) = ddxRef;
    case "z"
        % z uses the same smooth down/hold/recover profile previously named
        % crouch, without adding horizontal motion.
    otherwise
        error("floating_base_reference:InvalidMode", ...
            "Trajectory mode must be 'stand', 'z', or 'velocity'.");
end

if mode == "z" || mode == "velocity_round_trip"
    [zOffset, dzRef, ddzRef] = crouchProfile(t, trajectory);
    xRef(2) = xRef(2) + zOffset;
    xRef(5) = xRef(5) + dzRef;
    aRef(2) = ddzRef;
end
end

function [x, dx, ddx] = velocityRoundTrip(t, trajectory)
settleTime = getFieldOrDefault(trajectory, "settleTime", 0);
speed = getFieldOrDefault(trajectory, "cruiseVelocity", 0);
accelDuration = getFieldOrDefault(trajectory, "accelDuration", 0);
cruiseDuration = getFieldOrDefault(trajectory, "cruiseDuration", 0);
decelDuration = getFieldOrDefault(trajectory, "decelDuration", accelDuration);
turnHoldDuration = getFieldOrDefault(trajectory, "turnHoldDuration", 0);

segmentDuration = accelDuration + cruiseDuration + decelDuration;
forwardDistance = speed * (cruiseDuration ...
    + 0.5 * (accelDuration + decelDuration));
tForward = t - settleTime;

if tForward <= segmentDuration + turnHoldDuration
    [x, dx, ddx] = velocitySegment(tForward, speed, ...
        accelDuration, cruiseDuration, decelDuration);
else
    [xBack, dx, ddx] = velocitySegment( ...
        tForward - segmentDuration - turnHoldDuration, -speed, ...
        accelDuration, cruiseDuration, decelDuration);
    x = forwardDistance + xBack;
end
end

function [z, dz, ddz] = crouchProfile(t, trajectory)
depth = max(0, getFieldOrDefault(trajectory, "crouchDepth", 0));
if depth == 0
    z = 0;
    dz = 0;
    ddz = 0;
    return;
end

downDuration = getFieldOrDefault(trajectory, ...
    "crouchDownDuration", getFieldOrDefault(trajectory, "settleTime", 1));
recoverStart = getFieldOrDefault(trajectory, "crouchRecoverStart", inf);
recoverDuration = getFieldOrDefault(trajectory, ...
    "crouchRecoverDuration", downDuration);

if t < downDuration
    [alpha, dalpha, ddalpha] = smoothStep(t, downDuration);
    z = -depth * alpha;
    dz = -depth * dalpha;
    ddz = -depth * ddalpha;
elseif t < recoverStart
    z = -depth;
    dz = 0;
    ddz = 0;
elseif t < recoverStart + recoverDuration
    [alpha, dalpha, ddalpha] = smoothStep( ...
        t - recoverStart, recoverDuration);
    z = -depth * (1 - alpha);
    dz = depth * dalpha;
    ddz = depth * ddalpha;
else
    z = 0;
    dz = 0;
    ddz = 0;
end
end

function [x, dx, ddx] = velocitySegment(t, speed, accelDuration, ...
        cruiseDuration, decelDuration)
if t <= 0
    x = 0;
    dx = 0;
    ddx = 0;
elseif t < accelDuration
    [alpha, dalpha] = smoothStep(t, accelDuration);
    s = t / accelDuration;
    x = speed * accelDuration * smoothStepIntegral(s);
    dx = speed * alpha;
    ddx = speed * dalpha;
elseif t < accelDuration + cruiseDuration
    x = speed * (0.5 * accelDuration + t - accelDuration);
    dx = speed;
    ddx = 0;
elseif t < accelDuration + cruiseDuration + decelDuration
    tDecel = t - accelDuration - cruiseDuration;
    [beta, dbeta] = smoothStep(tDecel, decelDuration);
    s = tDecel / decelDuration;
    x = speed * (0.5 * accelDuration + cruiseDuration ...
        + tDecel - decelDuration * smoothStepIntegral(s));
    dx = speed * (1 - beta);
    ddx = -speed * dbeta;
else
    x = speed * (cruiseDuration ...
        + 0.5 * (accelDuration + decelDuration));
    dx = 0;
    ddx = 0;
end
end

function value = smoothStepIntegral(s)
value = 2.5*s^4 - 3*s^5 + s^6;
end

function [alpha, dalpha, ddalpha] = smoothStep(t, duration)
if duration <= 0
    alpha = 1;
    dalpha = 0;
    ddalpha = 0;
    return;
end

s = min(max(t / duration, 0), 1);
alpha = 10*s^3 - 15*s^4 + 6*s^5;
dalpha = (30*s^2 - 60*s^3 + 30*s^4) / duration;
ddalpha = (60*s - 180*s^2 + 120*s^3) / duration^2;
end

function value = getFieldOrDefault(s, name, defaultValue)
if isstruct(s) && isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
