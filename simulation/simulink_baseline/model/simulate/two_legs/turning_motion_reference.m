function reference = turning_motion_reference(t, vxRef, trajectory, halfTrack)
%TURNING_MOTION_REFERENCE Smooth yaw-rate and differential rolling plan.
%
% Output: [yaw; yawRate; yawAcceleration; curvature; vLeft; vRight].
% Positive yaw uses the current NMPC convention: the right wheel rolls
% faster than the left wheel and d*(FRx-FLx) produces positive yaw moment.

if nargin < 3 || isempty(trajectory)
    trajectory = struct();
end
if nargin < 4 || isempty(halfTrack)
    halfTrack = 0;
end
turning = getFieldOrDefault(trajectory, "turning", struct());
yaw0 = getFieldOrDefault(turning, "yaw0", 0);
yaw = yaw0;
yawRate = 0;
yawAcceleration = 0;

enabled = getFieldOrDefault(turning, "enabled", false);
speedCommand = abs(getFieldOrDefault(trajectory, "cruiseVelocity", vxRef));
minimumSpeed = getFieldOrDefault(turning, "minimumSpeed", 0.02);
if enabled && speedCommand >= minimumSpeed
    startTime = getFieldOrDefault(turning, "startTime", 2);
    rampDuration = getFieldOrDefault(turning, "rampDuration", 0.5);
    holdDuration = getFieldOrDefault(turning, "holdDuration", 1);
    zeroHoldDuration = getFieldOrDefault(turning, "zeroHoldDuration", 0.5);
    yawRateCommand = getFieldOrDefault(turning, "yawRate", 0);
    mode = lower(string(getFieldOrDefault(turning, "mode", "single")));
    if ~isfinite(rampDuration) || rampDuration <= 0
        error("turning_motion_reference:InvalidRampDuration", ...
            "turning.rampDuration must be finite and positive.");
    end
    if ~isfinite(holdDuration) || holdDuration < 0 ...
            || ~isfinite(zeroHoldDuration) || zeroHoldDuration < 0
        error("turning_motion_reference:InvalidHoldDuration", ...
            "Turning hold durations must be finite and nonnegative.");
    end

    switch mode
        case "single"
            transitionTimes = [startTime, ...
                startTime + rampDuration + holdDuration];
            transitionDeltas = [yawRateCommand, -yawRateCommand];
        case "s"
            downTime = startTime + rampDuration + holdDuration;
            negativeTime = downTime + rampDuration + zeroHoldDuration;
            upTime = negativeTime + rampDuration + holdDuration;
            transitionTimes = [startTime, downTime, negativeTime, upTime];
            transitionDeltas = [yawRateCommand, -yawRateCommand, ...
                -yawRateCommand, yawRateCommand];
        otherwise
            error("turning_motion_reference:InvalidMode", ...
                "turning.mode must be 'single' or 's'.");
    end

    for k = 1:numel(transitionTimes)
        tau = t - transitionTimes(k);
        [shape, shapeDerivative, shapeIntegral] = ...
            smoothTransition(tau, rampDuration);
        delta = transitionDeltas(k);
        yawRate = yawRate + delta*shape;
        yawAcceleration = yawAcceleration + delta*shapeDerivative;
        yaw = yaw + delta*shapeIntegral;
    end
end

curvature = 0;
if abs(vxRef) >= minimumSpeed
    curvature = yawRate/vxRef;
end
vLeft = vxRef - halfTrack*yawRate;
vRight = vxRef + halfTrack*yawRate;
reference = [yaw; yawRate; yawAcceleration; curvature; vLeft; vRight];
end

function [value, derivative, integral] = smoothTransition(t, duration)
if t <= 0
    value = 0;
    derivative = 0;
    integral = 0;
elseif t < duration
    s = t/duration;
    value = 3*s^2 - 2*s^3;
    derivative = 6*s*(1 - s)/duration;
    integral = duration*(s^3 - 0.5*s^4);
else
    value = 1;
    derivative = 0;
    integral = t - 0.5*duration;
end
end

function value = getFieldOrDefault(data, fieldName, defaultValue)
if isstruct(data) && isfield(data, fieldName)
    value = data.(fieldName);
else
    value = defaultValue;
end
end
