function reference = full_base_nmpc_reference(x, baseLqr, config, wheelLqr)
%FULL_BASE_NMPC_REFERENCE Build 16-state/12-input horizon references.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end
if nargin < 3 || isempty(config)
    config = evalin("base", "fullBaseNmpc");
end
if nargin < 4 || isempty(wheelLqr)
    wheelLqr = evalin("base", "wheelLqr");
end
x = double(x(:));
if numel(x) ~= 17
    error("full_base_nmpc_reference:InvalidInput", ...
        "Expected [t; planner(4); previousWrench(12)].");
end
t = x(1);
planner = x(2:5);
previousWrench = x(6:17);
exactDeltaInput = isfield(config, "incrementCostMode") ...
    && string(config.incrementCostMode) == "state_memory";
N = config.N;
stageSize = 40;
pathReference = zeros(stageSize, max(N - 1, 0));
xiRef = planner(1);
dxiRef = planner(2);
xiRaw = planner(4);

for k = 0:N
    stageTime = t + k*config.Ts;
    [baseReference, aRef] = floating_base_reference(stageTime, baseLqr);
    turningReference = turning_world_reference( ...
        stageTime, baseLqr, config.model.halfTrack);
    angles = [0; baseReference(3); turningReference(3)];
    eulerRates = [0; baseReference(6); turningReference(6)];
    stateReference = [
        turningReference(1); turningReference(2); baseReference(2);
        angles;
        turningReference(4); turningReference(5); baseReference(5);
        eulerRates;
        xiRef; xiRef; dxiRef; dxiRef
    ];
    uRef = feedforwardWrench(angles, ...
        [turningReference(7:8); aRef(2)], ...
        [0; aRef(3); turningReference(9)], xiRef, config.model);
    uRef = min(max(uRef, config.uMin(:)), config.uMax(:));
    if exactDeltaInput
        incrementReference = zeros(12, 1);
    else
        incrementReference = previousWrench;
    end
    stageReference = [stateReference; uRef; incrementReference];
    if k == 0
        initialReference = stageReference;
    elseif k < N
        pathReference(:, k) = stageReference;
    else
        terminalReference = stateReference;
    end
    if k < N
        if isfield(wheelLqr, "governorEnabled") ...
                && ~logical(wheelLqr.governorEnabled)
            xiRef = xiRaw;
            dxiRef = 0;
        else
            [xiRef, dxiRef] = wheel_position_governor_step( ...
                xiRef, dxiRef, xiRaw, config.Ts, wheelLqr);
        end
    end
end
reference = [initialReference; pathReference(:); terminalReference];
end

function u = feedforwardWrench(angles, worldAcceleration, ...
        eulerAcceleration, xiRef, model)
P = [1, 0, 0; 0, 0, 1; 0, 1, 0];
[R, ~, ~] = controller_attitude_kinematics(angles);
worldAccelerationPhysical = P'*worldAcceleration;
forceBody = P*R'*(model.m*(worldAccelerationPhysical + [0; model.g; 0]));
d = model.halfTrack;
h = model.rWzEq;
rollForceDifference = h*forceBody(2)/(2*d);
yawMoment = model.inertia(3)*eulerAcceleration(3);
yawForceDifference = (yawMoment - xiRef*forceBody(2))/(2*d);
pitchMoment = model.inertia(2)*eulerAcceleration(2) ...
    - (xiRef - model.xiEq)*forceBody(3) + h*forceBody(1);
leftForce = [forceBody(1)/2 - yawForceDifference; forceBody(2)/2; ...
    forceBody(3)/2 + rollForceDifference];
rightForce = [forceBody(1)/2 + yawForceDifference; forceBody(2)/2; ...
    forceBody(3)/2 - rollForceDifference];
u = [leftForce; 0; pitchMoment/2; 0; ...
    rightForce; 0; pitchMoment/2; 0];
end
