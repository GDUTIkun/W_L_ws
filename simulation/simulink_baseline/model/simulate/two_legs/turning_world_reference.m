function reference = turning_world_reference(t, baseLqr, halfTrack)
%TURNING_WORLD_REFERENCE World trajectory from body-forward speed and yaw.
%
% Output: [pX; pY; yaw; vX; vY; yawRate; aX; aY; yawAcceleration;
%          curvature; vLeft; vRight]. Positive yaw turns toward negative Y
% in the existing Simscape/controller world convention.

[baseReference, aReference] = floating_base_reference(t, baseLqr);
trajectory = baseLqr.trajectory;
turning = turning_motion_reference(t, baseReference(4), ...
    trajectory, halfTrack);
yaw = turning(1);
yawRate = turning(2);
forwardVelocity = baseReference(4);
forwardAcceleration = aReference(1);
heading = [cos(yaw); -sin(yaw)];
lateral = [sin(yaw); cos(yaw)];
worldVelocity = forwardVelocity*heading;
worldAcceleration = forwardAcceleration*heading ...
    - forwardVelocity*yawRate*lateral;

settings = trajectory.turning;
enabled = settings.enabled && abs(settings.yawRate) > 0 ...
    && abs(trajectory.cruiseVelocity) >= settings.minimumSpeed;
[initialReference, ~] = floating_base_reference(0, baseLqr);
if ~enabled
    worldPosition = [baseReference(1); 0];
else
    startTime = settings.startTime;
    [startReference, ~] = floating_base_reference(startTime, baseLqr);
    yaw0 = settings.yaw0;
    initialPosition = [initialReference(1); 0];
    startPosition = initialPosition ...
        + (startReference(1) - initialReference(1)) ...
        * [cos(yaw0); -sin(yaw0)];
    if t <= startTime
        worldPosition = initialPosition ...
            + (baseReference(1) - initialReference(1))*heading;
    else
        endTime = profileEndTime(settings);
        integrationEnd = min(t, endTime);
        worldPosition = startPosition + integrateTurn( ...
            startTime, integrationEnd, trajectory.cruiseVelocity, ...
            trajectory, halfTrack);
        if t > endTime
            [endReference, ~] = floating_base_reference(endTime, baseLqr);
            endTurning = turning_motion_reference(endTime, ...
                endReference(4), trajectory, halfTrack);
            endHeading = [cos(endTurning(1)); -sin(endTurning(1))];
            worldPosition = worldPosition ...
                + (baseReference(1) - endReference(1))*endHeading;
        end
    end
end

reference = [worldPosition; yaw; worldVelocity; yawRate; ...
    worldAcceleration; turning(3:6)];
end

function displacement = integrateTurn(t0, t1, speed, trajectory, halfTrack)
settings = trajectory.turning;
ramp = settings.rampDuration;
hold = settings.holdDuration;
if lower(string(settings.mode)) == "single"
    points = [t0, t0 + ramp, t0 + ramp + hold, ...
        t0 + 2*ramp + hold];
else
    zeroHold = settings.zeroHoldDuration;
    down = t0 + ramp + hold;
    negative = down + ramp + zeroHold;
    up = negative + ramp + hold;
    points = [t0, t0 + ramp, down, down + ramp, negative, ...
        negative + ramp, up, up + ramp];
end
displacement = zeros(2, 1);
for k = 1:numel(points) - 1
    a = points(k);
    b = min(t1, points(k + 1));
    if b > a
        displacement = displacement + integrateInterval( ...
            a, b, speed, trajectory, halfTrack);
    end
    if t1 <= points(k + 1)
        return;
    end
end
end

function displacement = integrateInterval(a, b, speed, trajectory, halfTrack)
mid = 0.5*(a + b);
midReference = turning_motion_reference(mid, speed, trajectory, halfTrack);
if abs(midReference(3)) < 1e-12
    aReference = turning_motion_reference(a, speed, trajectory, halfTrack);
    yawA = aReference(1);
    omega = midReference(2);
    duration = b - a;
    if abs(omega) < 1e-12
        displacement = speed*duration*[cos(yawA); -sin(yawA)];
    else
        yawB = yawA + omega*duration;
        displacement = speed/omega ...
            * [sin(yawB) - sin(yawA); cos(yawB) - cos(yawA)];
    end
    return;
end
nodes = [-0.960289856497536, -0.796666477413627, ...
    -0.525532409916329, -0.183434642495650, ...
    0.183434642495650, 0.525532409916329, ...
    0.796666477413627, 0.960289856497536];
weights = [0.101228536290376, 0.222381034453374, ...
    0.313706645877887, 0.362683783378362, ...
    0.362683783378362, 0.313706645877887, ...
    0.222381034453374, 0.101228536290376];
displacement = zeros(2, 1);
for k = 1:numel(nodes)
    sample = mid + 0.5*(b - a)*nodes(k);
    sampleReference = turning_motion_reference( ...
        sample, speed, trajectory, halfTrack);
    displacement = displacement + weights(k) ...
        * [cos(sampleReference(1)); -sin(sampleReference(1))];
end
displacement = speed*0.5*(b - a)*displacement;
end

function endTime = profileEndTime(settings)
ramp = settings.rampDuration;
hold = settings.holdDuration;
if lower(string(settings.mode)) == "single"
    endTime = settings.startTime + 2*ramp + hold;
else
    endTime = settings.startTime + 4*ramp + 2*hold ...
        + settings.zeroHoldDuration;
end
end
