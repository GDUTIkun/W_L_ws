function configure_base_tracking_case(caseMode, plannerMode, modelName)
%CONFIGURE_BASE_TRACKING_CASE Configure stand, z, or velocity tracking.

if nargin < 1 || isempty(caseMode)
    caseMode = "velocity";
end
if nargin < 2 || isempty(plannerMode)
    plannerMode = "lqr";
end
if nargin < 3 || isempty(modelName)
    modelName = "source_common";
end
caseMode = lower(string(caseMode));
plannerMode = lower(string(plannerMode));
model = string(modelName);
assert(any(model == ["source", "source_common"]), ...
    "modelName must be 'source' or 'source_common'.");
if ~ismember(caseMode, ["stand", "z", "velocity"])
    error("configure_base_tracking_case:InvalidMode", ...
        "caseMode must be 'stand', 'z', or 'velocity'.");
end
if ~ismember(plannerMode, ["lqr", "qp_force"])
    error("configure_base_tracking_case:InvalidPlanner", ...
        "plannerMode must be 'lqr' or 'qp_force'.");
end

load_system(model);

base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
baseNmpc = evalin("base", "baseNmpc");
traj = evalin("base", "traj");
trajectory = base.trajectory;
trajectory.enabled = true;
trajectory.mode = caseMode;
trajectory.crouchDepth = 0;
if model == "source_common" && caseMode == "velocity"
    % The strict common-mode plant has no differential posture reserve.
    % Keep the 0.5 m/s command, but halve its acceleration to 0.5 m/s^2.
    trajectory.accelDuration = 1.0;
    trajectory.decelDuration = 1.0;
elseif model == "source" && caseMode == "velocity"
    % First validated working point of the full spatial QP.  The strict
    % common-mode model retains its established 0.5 m/s command.
    trajectory.cruiseVelocity = 0.1;
end
stopTime = 10;
if caseMode == "stand"
    stopTime = 5;
elseif caseMode == "z"
    trajectory.crouchDepth = 0.025;
end
traj.wheelPositionPlanner = plannerMode;
baseNmpc.enabled = true;
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
assignin("base", "baseNmpc", baseNmpc);
assignin("base", "traj", traj);

pulseBlocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "DiscretePulseGenerator");
for idx = 1:numel(pulseBlocks)
    set_param(pulseBlocks{idx}, "Amplitude", "0");
end

set_param(model, "StopTime", string(stopTime));
fprintf("Configured %s: %g s %s case with %s wheel planning; direct NMPC enabled = %d.\n", ...
    model, stopTime, caseMode, plannerMode, baseNmpc.enabled);
end
