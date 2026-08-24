function configure_turning_case(vxRef, yawRateRef, profileMode, modelName)
%CONFIGURE_TURNING_CASE Configure the minimal vx/yaw-rate steering input.

if nargin < 3 || isempty(profileMode)
    profileMode = "single";
end
if nargin < 4 || isempty(modelName)
    modelName = "source";
end
validateattributes(vxRef, {'numeric'}, {'scalar', 'real', 'finite'});
validateattributes(yawRateRef, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
profileMode = lower(string(profileMode));
if ~ismember(profileMode, ["single", "s"])
    error("configure_turning_case:InvalidMode", ...
        "profileMode must be 'single' or 's'.");
end

base = evalin("base", "base");
minimumSpeed = base.trajectory.turning.minimumSpeed;
if abs(yawRateRef) > 0 && abs(vxRef) < minimumSpeed
    error("configure_turning_case:InPlaceTurnUnsupported", ...
        "This version requires |vxRef| >= %.3g m/s for nonzero yaw rate.", ...
        minimumSpeed);
end

configure_base_tracking_case("velocity", "lqr", modelName);
base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
base.trajectory.cruiseVelocity = vxRef;
base.trajectory.turning.enabled = yawRateRef ~= 0;
base.trajectory.turning.mode = profileMode;
base.trajectory.turning.yawRate = yawRateRef;
baseLqr.trajectory = base.trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
fprintf("Configured steering: vxRef = %.4g m/s, yawRateRef = %.4g rad/s, %s profile.\n", ...
    vxRef, yawRateRef, profileMode);
end
