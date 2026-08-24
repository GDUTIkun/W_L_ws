function yaw = continuous_heading(t, yawWrapped)
%CONTINUOUS_HEADING Unwrap a sequential heading measurement across +/-pi.

persistent previousTime previousWrapped previousYaw
if isempty(previousTime) || t <= 0 || t < previousTime
    previousTime = t;
    previousWrapped = yawWrapped;
    previousYaw = yawWrapped;
else
    delta = atan2(sin(yawWrapped - previousWrapped), ...
        cos(yawWrapped - previousWrapped));
    previousYaw = previousYaw + delta;
    previousWrapped = yawWrapped;
    previousTime = t;
end
yaw = previousYaw;
end
