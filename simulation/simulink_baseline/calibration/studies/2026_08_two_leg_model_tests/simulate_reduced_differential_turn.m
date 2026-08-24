function result = simulate_reduced_differential_turn( ...
        targetYawDeg, speedReference, yawRateReference, config)
%SIMULATE_REDUCED_DIFFERENTIAL_TURN Fast endurance model for anti-split.
%
% The reduced plant deliberately assumes a separately stabilized common
% roll/pitch mode. It retains continuous yaw, first-order speed/yaw-rate
% dynamics, a disturbed second-order differential leg coordinate, and the
% deployed force controller's amplitude/rate limits.

if nargin < 4 || isempty(config)
    config = struct("enabled", true, "Kxi", 1000, "Kd", 100, ...
        "polarity", 1, "amplitudeLimit", 50, "rateLimit", 1000, ...
        "Ts", 0.005);
end
validateattributes(targetYawDeg, {'numeric'}, ...
    {'scalar','real','finite','nonzero'});
validateattributes(speedReference, {'numeric'}, ...
    {'scalar','real','finite','positive'});
validateattributes(yawRateReference, {'numeric'}, ...
    {'scalar','real','finite','nonzero'});
if sign(targetYawDeg) ~= sign(yawRateReference)
    error("simulate_reduced_differential_turn:Direction", ...
        "target yaw and yaw-rate reference must have the same sign.");
end

Ts = config.Ts;
turnStart = 2.0;
rampDuration = 0.5;
targetTime = turnStart + abs(deg2rad(targetYawDeg) ...
    / yawRateReference) + 0.5*rampDuration;
stopTime = targetTime + 5.0;
time = (0:Ts:stopTime).';
n = numel(time);
speed = zeros(n, 1);
yawRate = zeros(n, 1);
yaw = zeros(n, 1);
xi = zeros(n, 1);
dxi = zeros(n, 1);
force = zeros(n, 1);
disturbance = zeros(n, 1);
xi(1) = 0.010; % deliberate 10 mm initial split

clear differential_leg_force_stabilizer
for k = 1:n-1
    t = time(k);
    speedCommand = speedReference*smoothStep(t - 0.5, 0.5);
    yawCommand = yawRateReference*smoothStep(t - turnStart, rampDuration);
    speed(k+1) = speed(k) + Ts*(speedCommand - speed(k))/0.35;
    yawRate(k+1) = yawRate(k) + Ts*(yawCommand - yawRate(k))/0.20;
    yaw(k+1) = yaw(k) + Ts*yawRate(k+1);
    [force(k), ~] = differential_leg_force_stabilizer( ...
        t, xi(k), dxi(k), config);
    disturbance(k) = 0.4*sin(0.7*t) + 0.2*sin(2.3*t) ...
        + 10*speed(k)*yawRate(k);
    % Effective differential leg mode: positive controller force restores
    % positive canonical split through the Jacobian-transpose mapping.
    ddxi = (disturbance(k) - force(k) - 8*dxi(k))/4;
    dxi(k+1) = dxi(k) + Ts*ddxi;
    xi(k+1) = xi(k) + Ts*dxi(k+1);
end
force(end) = force(end-1);
disturbance(end) = disturbance(end-1);

[~, targetIndex] = min(abs(time - targetTime));
steady = time >= turnStart + 5;
tail = time >= stopTime - 5;
settled = abs(xi) <= 1e-3 & abs(dxi) <= 2e-3;
recoveryTime = inf;
holdSamples = round(1/Ts);
for k = 1:max(1, n-holdSamples)
    if all(settled(k:k+holdSamples))
        recoveryTime = time(k);
        break;
    end
end

result = struct();
result.targetYawDeg = targetYawDeg;
result.speedReference = speedReference;
result.yawRateReference = yawRateReference;
result.actualYawAtTargetDeg = rad2deg(yaw(targetIndex));
result.yawErrorAtTargetDeg = result.actualYawAtTargetDeg-targetYawDeg;
result.finalSpeed = speed(end);
result.finalYawRate = yawRate(end);
result.maxAbsXiAfter5Mm = 1e3*max(abs(xi(steady)));
result.tailMaxAbsXiMm = 1e3*max(abs(xi(tail)));
result.tailRmsXiMm = 1e3*rms(xi(tail));
result.recoveryTime = recoveryTime;
result.maxAbsForce = max(abs(force));
result.completed = all(isfinite([speed; yawRate; yaw; xi; dxi; force]));
result.pass = result.completed ...
    && abs(result.yawErrorAtTargetDeg) <= 2 ...
    && abs(result.finalSpeed-speedReference) <= 0.02 ...
    && abs(result.finalYawRate-yawRateReference) <= 0.005 ...
    && result.maxAbsXiAfter5Mm <= 2 ...
    && result.tailMaxAbsXiMm <= 1 ...
    && result.recoveryTime <= 5;
result.time = time;
result.speed = speed;
result.yawRate = yawRate;
result.yaw = yaw;
result.xi = xi;
result.dxi = dxi;
result.force = force;
result.disturbance = disturbance;
end

function value = smoothStep(t, duration)
if t <= 0
    value = 0;
elseif t >= duration
    value = 1;
else
    s = t/duration;
    value = 3*s^2 - 2*s^3;
end
end
