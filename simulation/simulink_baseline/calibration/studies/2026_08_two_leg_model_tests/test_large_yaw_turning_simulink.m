function summary = test_large_yaw_turning_simulink( ...
        caseFilter, controllerOverride, diagnosticStopTimeCap, ...
        fullBaseNmpcOverride, showAnimation)
%TEST_LARGE_YAW_TURNING_SIMULINK Validate 30--720 deg continuous turns.
% Results are merged into large_yaw_turning_regression.csv so long cases
% can be run independently. The all mode gates A6 on a physically stable A5.
% Set showAnimation=true for an interactive Mechanics Explorer run.

if nargin < 1 || isempty(caseFilter)
    caseFilter = "all";
end
if nargin < 2 || isempty(controllerOverride)
    controllerOverride = struct();
end
if nargin < 3 || isempty(diagnosticStopTimeCap)
    diagnosticStopTimeCap = inf;
end
if nargin < 4 || isempty(fullBaseNmpcOverride)
    fullBaseNmpcOverride = struct();
end
if nargin < 5 || isempty(showAnimation)
    showAnimation = false;
end
if ~isstruct(controllerOverride) || ~isscalar(controllerOverride)
    error("test_large_yaw_turning_simulink:InvalidControllerOverride", ...
        "controllerOverride must be a scalar struct.");
end
validateattributes(diagnosticStopTimeCap, {'numeric'}, ...
    {'scalar', 'real', 'positive'});
if ~isstruct(fullBaseNmpcOverride) || ~isscalar(fullBaseNmpcOverride)
    error("test_large_yaw_turning_simulink:InvalidNmpcOverride", ...
        "fullBaseNmpcOverride must be a scalar struct.");
end
validateattributes(showAnimation, {'logical', 'numeric'}, ...
    {'scalar', 'real', 'finite'});
showAnimation = logical(showAnimation);
caseFilter = string(caseFilter);
studyDir = fileparts(mfilename("fullpath"));
codeRoot = fileparts(fileparts(fileparts(studyDir)));
modelDir = fullfile(codeRoot, "model", "simulate", "two_legs");
addpath(modelDir);
model = "source";
evalin("base", "startup");
ctrl = evalin("base", "ctrl");
overrideNames = fieldnames(controllerOverride);
for overrideIndex = 1:numel(overrideNames)
    name = overrideNames{overrideIndex};
    ctrl.(name) = controllerOverride.(name);
end
assignin("base", "ctrl", ctrl);
fullBaseNmpc = evalin("base", "fullBaseNmpc");
nmpcOverrideNames = fieldnames(fullBaseNmpcOverride);
for overrideIndex = 1:numel(nmpcOverrideNames)
    name = nmpcOverrideNames{overrideIndex};
    fullBaseNmpc.(name) = fullBaseNmpcOverride.(name);
end
assignin("base", "fullBaseNmpc", fullBaseNmpc);
% A tuning study may supply a separately generated solver with the same
% S-Function name. Put that build first on the MATLAB path after startup
% has added the frozen baseline build, otherwise the override silently uses
% the old binary while reporting the new configuration.
if isfield(fullBaseNmpc, "generatedDir") ...
        && isfolder(fullBaseNmpc.generatedDir)
    addpath(fullBaseNmpc.generatedDir, "-begin");
end
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
originalOpenMechanicsExplorer = get_param(model, ...
    "SimMechanicsOpenEditorOnUpdate");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty, ...
    originalOpenMechanicsExplorer));
set_param(model, "InitFcn", "");
if showAnimation
    set_param(model, "SimMechanicsOpenEditorOnUpdate", "on");
else
    set_param(model, "SimMechanicsOpenEditorOnUpdate", "off");
end

speed = 0.10;
yawRate = 0.08;
cases = [
    turnCase("A1_30_left", 30, speed, yawRate)
    turnCase("A2_60_left", 60, speed, yawRate)
    turnCase("A3_90_left", 90, speed, yawRate)
    turnCase("A3_90_right", -90, speed, -yawRate)
    turnCase("A4_180_left", 180, speed, yawRate)
    turnCase("A5_360_left", 360, speed, yawRate)
    turnCase("A6_720_left", 720, speed, yawRate)
    turnCase("B1_30_left_v010_yaw003", 30, 0.10, 0.03)
    turnCase("B2_90_left_v010_yaw003", 90, 0.10, 0.03)
    turnCase("B3_360_left_v010_yaw003", 360, 0.10, 0.03)
    turnCase("B4_720_left_v010_yaw003", 720, 0.10, 0.03)
    turnCase("H1_360_left_v020_yaw003", 360, 0.20, 0.03)
    turnCase("H2_720_left_v020_yaw003", 720, 0.20, 0.03)
    continuousTurnCase("C0_90_left_continuous", 90, 0.10, 0.08)
    continuousTurnCase("C0_90_right_continuous", -90, 0.10, -0.08)
    continuousTurnCase("C1_360_left_continuous", 360, 0.10, 0.08)
    continuousTurnCase("C1_360_right_continuous", -360, 0.10, -0.08)
    continuousTurnCase("C2_720_left_continuous", 720, 0.10, 0.08)
    continuousTurnCase("C2_720_right_continuous", -720, 0.10, -0.08)
    continuousTurnCase("C3_1800_left_continuous", 1800, 0.10, 0.08)
    continuousTurnCase("HC1_360_left_v020_continuous", 360, 0.20, 0.08)
    continuousTurnCase("HC2_720_left_v020_continuous", 720, 0.20, 0.08)
    continuousTurnCase("HS1_90_left_v100_yaw008", 90, 1.00, 0.08)
    continuousTurnCase("HS2_90_left_v100_yaw020", 90, 1.00, 0.20)
    continuousTurnCase("HS2_90_right_v100_yaw020", -90, 1.00, -0.20)
    continuousTurnCase("HS3_360_left_v100_yaw020", 360, 1.00, 0.20)
];
if caseFilter ~= "all"
    caseNames = string({cases.name});
    selected = strcmpi(caseNames, caseFilter);
    if ~any(selected)
        selected = contains(caseNames, caseFilter, "IgnoreCase", true);
    end
    cases = cases(selected);
    if isempty(cases)
        error("test_large_yaw_turning_simulink:UnknownFilter", ...
            "No large-yaw case matches '%s'.", caseFilter);
    end
end

records = repmat(emptyRecord(), 0, 1);
for k = 1:numel(cases)
    if caseFilter == "all" && cases(k).name == "A6_720_left"
        a5 = records(string({records.name}) == "A5_360_left");
        if isempty(a5) || ~a5.controlStable
            fprintf("A6 skipped because A5 did not pass control/physics.\n");
            continue;
        end
    end
    record = runCase(cases(k), model, diagnosticStopTimeCap);
    records(end + 1, 1) = record; %#ok<AGROW>
    mergeSummary(struct2table(record));
end
summary = struct2table(records);

if any(ismember(string({records.name}), ...
        ["A3_90_left", "A3_90_right"]))
    addMirrorMetrics();
end
clear cleanup
end

function record = runCase(testCase, model, diagnosticStopTimeCap)
rampDuration = 0.5;
if abs(testCase.speed) >= 0.8
    velocityRampDuration = 2.0;
    startTime = 3.5;
else
    velocityRampDuration = NaN;
    startTime = 2.2;
end
targetDuration = abs(deg2rad(testCase.targetYawDeg)/testCase.yawRate);
if testCase.continuous
    enduranceTail = 2.0;
    % One ramp contributes half a ramp of yaw. Keep the command in its
    % constant-rate hold beyond the target so 360/720/... are true
    % continuous turns rather than turn-exit tests.
    holdDuration = targetDuration - 0.5*rampDuration ...
        + enduranceTail + 1.0;
    turnEndTime = startTime + targetDuration + 0.5*rampDuration;
    stopTime = turnEndTime + enduranceTail;
else
    holdDuration = targetDuration - rampDuration;
    turnEndTime = startTime + 2*rampDuration + holdDuration;
    stopTime = turnEndTime + 3.0;
end
assert(holdDuration >= 0, "Turn target is shorter than its ramps.");
stopTime = min(stopTime, diagnosticStopTimeCap);

set_initial_base_state(zeros(6, 1), zeros(4, 1));
configure_turning_case(testCase.speed, testCase.yawRate, "single", model);
base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
trajectory = base.trajectory;
if ~isnan(velocityRampDuration)
    trajectory.accelDuration = velocityRampDuration;
end
trajectory.cruiseDuration = turnEndTime + 2.0;
trajectory.turning.startTime = startTime;
trajectory.turning.rampDuration = rampDuration;
trajectory.turning.holdDuration = holdDuration;
trajectory.turning.zeroHoldDuration = 0.5;
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);

clear spatial_two_leg_qp_core coupled_two_leg_qp_core ...
    full_base_nmpc_command wheel_position_governor_step continuous_heading
fprintf("%s: simulating %.2f s for target %.0f deg.\n", ...
    testCase.name, stopTime, testCase.targetYawDeg);
simulationStart = tic;
simulationInput = Simulink.SimulationInput(model);
simulationInput = simulationInput.setBlockParameter( ...
    model + "/PD_only/Brick Solid3", ...
    "BrickDimensions", "[100 0.05 100]");
simulationInput = simulationInput.setModelParameter( ...
    "StopTime", num2str(stopTime, "%.9g"), ...
    "SimulationMode", "accelerator", ...
    "ReturnWorkspaceOutputs", "on", ...
    "SignalLogging", "on", ...
    "SaveOutput", "off", ...
    "SaveState", "off", ...
    "SaveFinalState", "off", ...
    "SaveTime", "off", ...
    "CaptureErrors", "on");
out = sim(simulationInput);
simulationWallTime = toc(simulationStart);
simulationErrorText = string(out.ErrorMessage);
simulationCompleted = strlength(simulationErrorText) == 0;
simulationError = simulationErrorCode(simulationErrorText);
logs = out.logsout;

[time, state] = namedSignal(logs, "baseNmpcState");
[symmetryTime, symmetry] = namedSignal(logs, "symmetryLegState");
[symmetryTime, symmetry] = thinLoggedSignal( ...
    symmetryTime, symmetry, stopTime, 0.005);
[qpTime, qp] = namedSignal(logs, "coupledQpSignal");
[wrenchTime, bodyWrench] = namedSignal(logs, "nmpcBodyWrench");
[statusTime, nmpcStatus] = namedSignal(logs, "nmpcStatus");
[cpuTime, nmpcCpuTime] = namedSignal(logs, "nmpcCpuTime");
[faultTime, nmpcFault] = namedSignal(logs, "nmpcFault");
save("large_yaw_last_raw.mat", "testCase", "simulationCompleted", ...
    "simulationError", "simulationErrorText", "simulationWallTime", ...
    "time", "state", ...
    "symmetryTime", "symmetry", "qpTime", "qp", "wrenchTime", ...
    "bodyWrench", "statusTime", "nmpcStatus", "cpuTime", ...
    "nmpcCpuTime", "faultTime", "nmpcFault", "baseLqr", ...
    "turnEndTime", "startTime");

if isempty(time) || isempty(state) || isempty(qp)
    record = emptyRecord();
    record.name = string(testCase.name);
    record.continuousTurn = testCase.continuous;
    record.targetYawDeg = testCase.targetYawDeg;
    record.speedRef = testCase.speed;
    record.yawRateRef = testCase.yawRate;
    record.turnEndTime = turnEndTime;
    record.simulationWallTime = simulationWallTime;
    record.simulationCompleted = false;
    record.simulationError = simulationError;
    record.failureReason = "simulation integration: " ...
        + extractBefore(simulationErrorText + newline, newline);
    fprintf("%s: simulation produced no logged samples: %s\n", ...
        testCase.name, simulationErrorText);
    return;
end

halfTrack = evalin("base", "fullBaseNmpc.model.halfTrack");
reference = zeros(numel(time), 12);
for i = 1:numel(time)
    reference(i, :) = turning_world_reference( ...
        time(i), baseLqr, halfTrack).';
end
bodyForward = cos(state(:, 6)).*state(:, 7) ...
    - sin(state(:, 6)).*state(:, 8);
bodyLateral = sin(state(:, 6)).*state(:, 7) ...
    + cos(state(:, 6)).*state(:, 8);
active = abs(reference(:, 6)) >= 0.8*abs(testCase.yawRate);
exitWindow = time >= turnEndTime + 0.5;
turnStartIndex = nearestIndex(time, startTime);
turnEndIndex = nearestIndex(time, turnEndTime);

yawError = state(:, 6) - reference(:, 3);
yawRateError = state(:, 12) - reference(:, 6);
wrappedYaw = atan2(sin(state(:, 6)), cos(state(:, 6)));
wrapCrossing = find(abs(diff(wrappedYaw)) > pi);
maxYawStep = max(abs(diff(state(:, 6))));
maxYawRateStep = max(abs(diff(state(:, 12))));
pathError = hypot(state(:, 1) - reference(:, 1), ...
    state(:, 2) - reference(:, 2));

[actualRadius, circleFitRmse] = fitCircle( ...
    state(active, 1), state(active, 2));
referenceRadius = abs(testCase.speed/testCase.yawRate);
actualCurvature = mean(state(active, 12)) ...
    / mean(bodyForward(active));
referenceCurvature = testCase.yawRate/testCase.speed;
closureError = norm(state(turnEndIndex, 1:2) ...
    - state(turnStartIndex, 1:2));
referenceClosureError = norm(reference(turnEndIndex, 1:2) ...
    - reference(turnStartIndex, 1:2));

leg = evalin("base", "leg");
wheelSpeed = -leg.r*[sum(symmetry(:, 4:6), 2), ...
    sum(symmetry(:, 10:12), 2)];
wheelReference = zeros(numel(symmetryTime), 2);
for i = 1:numel(symmetryTime)
    turning = turning_world_reference(symmetryTime(i), baseLqr, halfTrack);
    wheelReference(i, :) = turning(11:12).';
end
wheelReferenceDifference = wheelReference(:, 2) - wheelReference(:, 1);
wheelDifference = wheelSpeed(:, 2) - wheelSpeed(:, 1);
wheelActive = abs(wheelReferenceDifference) ...
    >= 0.8*max(abs(wheelReferenceDifference));

xiDelta = 0.5*(state(:, 13) - state(:, 14));
xiCommon = 0.5*(state(:, 13) + state(:, 14));
contactResidual = abs(qp(:, 49:51));
frictionMargin = min(qp(:, 61:64), [], 2);
torqueMargin = min(qp(:, 65:70), [], 2);
crossingWrenchJump = wrenchJumpAtCrossings( ...
    time, wrapCrossing, wrenchTime, bodyWrench);

record = emptyRecord();
record.name = string(testCase.name);
record.continuousTurn = testCase.continuous;
record.targetYawDeg = testCase.targetYawDeg;
record.speedRef = testCase.speed;
record.yawRateRef = testCase.yawRate;
record.turnEndTime = turnEndTime;
record.actualYawDeg = rad2deg(state(turnEndIndex, 6) ...
    - state(turnStartIndex, 6));
record.finalYawErrorDeg = rad2deg(yawError(turnEndIndex));
record.yawRmseDeg = rad2deg(rms(yawError(active)));
record.yawRateRmse = rms(yawRateError(active));
record.forwardVelocityRmse = rms(bodyForward(active) - testCase.speed);
record.lateralVelocityRms = rms(bodyLateral(active));
record.maxAbsLateralVelocity = max(abs(bodyLateral));
record.pathRmse = rms(pathError(active));
record.actualRadius = actualRadius;
record.referenceRadius = referenceRadius;
record.radiusErrorPercent = 100*(actualRadius/referenceRadius - 1);
record.circleFitRmse = circleFitRmse;
record.actualCurvature = actualCurvature;
record.referenceCurvature = referenceCurvature;
record.curvatureErrorPercent = 100*(actualCurvature/referenceCurvature - 1);
record.closureError = closureError;
record.referenceClosureError = referenceClosureError;
record.wrapCrossingCount = numel(wrapCrossing);
record.maxUnwrappedYawStep = maxYawStep;
record.maxYawRateStep = maxYawRateStep;
record.maxWrenchJumpAtWrap = crossingWrenchJump;
record.forwardDirectionCorrectRatio = mean(bodyForward(active) > 0);
record.wheelDirectionCorrectRatio = mean( ...
    wheelDifference(wheelActive).*wheelReferenceDifference(wheelActive) > 0);
record.maxAbsRollDeg = rad2deg(max(abs(state(:, 4))));
record.maxAbsPitchDeg = rad2deg(max(abs(state(:, 5))));
record.maxAbsXiCommon = max(abs(xiCommon));
record.maxAbsXiDelta = max(abs(xiDelta));
record.finalXiDelta = xiDelta(end);
if any(exitWindow)
    record.finalAbsYawRate = max(abs(state(exitWindow, 12)));
else
    record.finalAbsYawRate = abs(state(end, 12));
end
record.maxWrenchSlackNorm = max(qp(:, 31));
record.maxRollingResidual = max(contactResidual(:, 1));
record.maxLateralResidual = max(contactResidual(:, 2));
record.maxNormalResidual = max(contactResidual(:, 3));
record.qpFeasibleRatio = mean(qp(:, 32) > 0.5);
record.maxDynamicsResidual = max(qp(:, 40));
record.maxWrenchResidual = max(qp(:, 42));
record.minFrictionMargin = min(frictionMargin);
record.minTorqueMargin = min(torqueMargin);
record.nmpcStatusMax = max(abs(nmpcStatus));
record.nmpcFaultRatio = mean(nmpcFault ~= 0);
record.maxNmpcSolveTimeMs = 1e3*max(nmpcCpuTime);
record.p99NmpcSolveTimeMs = 1e3*prctile(nmpcCpuTime, 99);
record.nmpcDeadlineMissRatio = mean(nmpcCpuTime > 0.020);
record.maxQpSolveTimeMs = 1e3*max(qp(:, 54));
record.simulationWallTime = simulationWallTime;
record.simulationCompleted = simulationCompleted;
record.simulationError = simulationError;
[record.controlStable, record.realtimePass, record.failureReason] = ...
    classify(record);

writeTimeSeries(testCase.name, time, state, reference, bodyForward, ...
    bodyLateral, pathError, wrappedYaw, wrenchTime, bodyWrench, ...
    qpTime, qp, statusTime, nmpcStatus, cpuTime, nmpcCpuTime, ...
    faultTime, nmpcFault);
fprintf("%s: control=%d realtime=%d yaw %.2f/%.0f deg, " + ...
    "R %.3f/%.3f m (%+.1f%%), closure %.3f m, " + ...
    "roll/pitch %.2f/%.2f deg, status %g, faults %.3g.\n", ...
    record.name, record.controlStable, record.realtimePass, ...
    record.actualYawDeg, record.targetYawDeg, record.actualRadius, ...
    record.referenceRadius, record.radiusErrorPercent, ...
    record.closureError, record.maxAbsRollDeg, record.maxAbsPitchDeg, ...
    record.nmpcStatusMax, record.nmpcFaultRatio);
end

function data = turnCase(name, targetYawDeg, speed, yawRate)
data = struct("name", name, "targetYawDeg", targetYawDeg, ...
    "speed", speed, "yawRate", yawRate, "continuous", false);
end

function data = continuousTurnCase(name, targetYawDeg, speed, yawRate)
data = struct("name", name, "targetYawDeg", targetYawDeg, ...
    "speed", speed, "yawRate", yawRate, "continuous", true);
end

function record = emptyRecord()
names = ["targetYawDeg", "speedRef", "yawRateRef", "turnEndTime", ...
    "actualYawDeg", "finalYawErrorDeg", "yawRmseDeg", "yawRateRmse", ...
    "forwardVelocityRmse", "lateralVelocityRms", ...
    "maxAbsLateralVelocity", "pathRmse", "actualRadius", ...
    "referenceRadius", "radiusErrorPercent", "circleFitRmse", ...
    "actualCurvature", "referenceCurvature", "curvatureErrorPercent", ...
    "closureError", "referenceClosureError", "wrapCrossingCount", ...
    "maxUnwrappedYawStep", "maxYawRateStep", "maxWrenchJumpAtWrap", ...
    "forwardDirectionCorrectRatio", "wheelDirectionCorrectRatio", ...
    "maxAbsRollDeg", "maxAbsPitchDeg", "maxAbsXiCommon", ...
    "maxAbsXiDelta", "finalXiDelta", "finalAbsYawRate", ...
    "maxWrenchSlackNorm", "maxRollingResidual", "maxLateralResidual", ...
    "maxNormalResidual", "qpFeasibleRatio", "maxDynamicsResidual", ...
    "maxWrenchResidual", "minFrictionMargin", "minTorqueMargin", ...
    "nmpcStatusMax", "nmpcFaultRatio", "maxNmpcSolveTimeMs", ...
    "p99NmpcSolveTimeMs", "nmpcDeadlineMissRatio", ...
    "maxQpSolveTimeMs", "simulationWallTime"];
record = struct("name", "");
for k = 1:numel(names)
    record.(names(k)) = 0;
end
record.controlStable = false;
record.realtimePass = false;
record.continuousTurn = false;
record.failureReason = "";
record.simulationCompleted = false;
record.simulationError = "";
record.mirrorYawRmseDifferenceDeg = nan;
record.mirrorRadiusMagnitudeDifference = nan;
record.mirrorXiDeltaPeakDifference = nan;
end

function [controlStable, realtimePass, reason] = classify(record)
failures = strings(0, 1);
if ~record.simulationCompleted
    failures(end + 1) = "simulation integration";
end
if record.qpFeasibleRatio < 0.99
    failures(end + 1) = "QP feasibility";
end
if record.nmpcStatusMax ~= 0 || record.nmpcFaultRatio ~= 0
    failures(end + 1) = "NMPC status/fault";
end
if record.maxDynamicsResidual >= 1e-6 || record.maxWrenchResidual >= 1e-6
    failures(end + 1) = "dynamics/wrench residual";
end
if record.minFrictionMargin < -1e-6 || record.minTorqueMargin < -1e-6
    failures(end + 1) = "friction/torque margin";
end
if record.yawRateRmse > 0.015 || abs(record.finalYawErrorDeg) > 5
    failures(end + 1) = "yaw tracking";
end
if record.forwardVelocityRmse > 0.03 || record.lateralVelocityRms > 0.03
    failures(end + 1) = "body velocity tracking";
end
if record.forwardDirectionCorrectRatio < 0.99 ...
        || record.wheelDirectionCorrectRatio < 0.95
    failures(end + 1) = "forward/wheel direction";
end
if record.maxAbsRollDeg > 2 || record.maxAbsPitchDeg > 3
    failures(end + 1) = "attitude bound";
end
if record.maxAbsXiDelta > 0.08 || abs(record.finalXiDelta) > 0.03
    failures(end + 1) = "xi_delta bound/recovery";
end
if record.continuousTurn
    if abs(record.finalAbsYawRate - abs(record.yawRateRef)) > 0.015
        failures(end + 1) = "continuous yaw rate";
    end
elseif record.finalAbsYawRate > 0.02
    failures(end + 1) = "turn exit";
end
if record.maxUnwrappedYawStep > 0.02 || record.maxYawRateStep > 0.05
    failures(end + 1) = "yaw continuity";
end
if record.maxWrenchSlackNorm > 0.05 ...
        || max([record.maxRollingResidual, record.maxLateralResidual, ...
        record.maxNormalResidual]) > 5
    failures(end + 1) = "wrench/contact residual";
end
controlStable = isempty(failures);
realtimePass = record.nmpcDeadlineMissRatio == 0;
if controlStable
    reason = "ok";
else
    reason = strjoin(failures, "; ");
end
end

function [radius, rmseValue] = fitCircle(x, y)
matrix = [2*x, 2*y, ones(size(x))];
solution = matrix\(x.^2 + y.^2);
center = solution(1:2).';
radialDistance = hypot(x - center(1), y - center(2));
radius = mean(radialDistance);
rmseValue = rms(radialDistance - radius);
end

function jump = wrenchJumpAtCrossings(time, crossings, wrenchTime, wrench)
if isempty(crossings)
    jump = 0;
    return;
end
sampled = interp1(wrenchTime, wrench, time, "linear", "extrap");
jump = max(vecnorm(sampled(crossings + 1, :) ...
    - sampled(crossings, :), 2, 2));
end

function index = nearestIndex(time, target)
[~, index] = min(abs(time - target));
end

function writeTimeSeries(name, time, state, reference, forwardVelocity, ...
        lateralVelocity, pathError, wrappedYaw, wrenchTime, bodyWrench, ...
        qpTime, qp, statusTime, status, cpuTime, cpu, faultTime, fault)
sample = [true; diff(floor(time/0.1)) > 0];
t = time(sample);
wrench = interp1(wrenchTime, bodyWrench, t, "linear", "extrap");
qpSample = interp1(qpTime, qp(:, [31, 32, 40, 42, 49:51, 61:70]), ...
    t, "previous", "extrap");
statusSample = interp1(statusTime, status, t, "previous", "extrap");
cpuSample = interp1(cpuTime, cpu, t, "previous", "extrap");
faultSample = interp1(faultTime, fault, t, "previous", "extrap");
matrix = [t, state(sample, 1), state(sample, 2), ...
    state(sample, 4), state(sample, 5), state(sample, 6), ...
    wrappedYaw(sample), state(sample, 12), reference(sample, 1), ...
    reference(sample, 2), reference(sample, 3), reference(sample, 6), ...
    state(sample, 7), state(sample, 8), forwardVelocity(sample), ...
    lateralVelocity(sample), pathError(sample), state(sample, 13), ...
    state(sample, 14), wrench(:, 1), wrench(:, 2), wrench(:, 3), ...
    wrench(:, 7), wrench(:, 8), wrench(:, 9), qpSample(:, 1), ...
    qpSample(:, 3), qpSample(:, 4), qpSample(:, 5), qpSample(:, 6), ...
    qpSample(:, 7), min(qpSample(:, 8:11), [], 2), ...
    min(qpSample(:, 12:17), [], 2), statusSample, cpuSample, ...
    faultSample];
variableNames = ["time", "worldX", "worldY", ...
    "roll", "pitch", "yawUnwrapped", "yawWrapped", "yawRate", ...
    "referenceWorldX", "referenceWorldY", "referenceYaw", ...
    "referenceYawRate", "worldVx", "worldVy", "bodyForwardVelocity", ...
    "bodyLateralVelocity", "pathError", "xiLeft", "xiRight", ...
    "leftBodyFx", "leftBodyFy", "leftBodyFz", "rightBodyFx", ...
    "rightBodyFy", "rightBodyFz", "wrenchSlackNorm", ...
    "dynamicsResidual", "wrenchResidual", "rollingResidual", ...
    "lateralResidual", "normalResidual", "minFrictionMargin", ...
    "minTorqueMargin", "nmpcStatus", "nmpcCpuTime", "nmpcFault"];
tableData = array2table(matrix, ...
    "VariableNames", cellstr(variableNames));
folder = "large_yaw_timeseries";
if ~isfolder(folder)
    mkdir(folder);
end
writetable(tableData, fullfile(folder, name + ".csv"));
end

function mergeSummary(newRows)
file = "large_yaw_turning_regression.csv";
if isfile(file)
    try
        oldRows = readtable(file, "TextType", "string");
        if ismember("name", string(oldRows.Properties.VariableNames))
            oldRows(ismember(string(oldRows.name), string(newRows.name)), :) = [];
            newRows = [oldRows; newRows];
        end
    catch
        % Replace an interrupted or malformed previous summary.
    end
end
[~, order] = sort(string(newRows.name));
writetable(newRows(order, :), file);
end

function code = simulationErrorCode(message)
if strlength(message) == 0
    code = "";
elseif contains(message, "Nonlinear iteration") ...
        && contains(message, "hmin")
    code = "simscape_nonlinear_hmin";
elseif contains(message, "ACADOS", "IgnoreCase", true)
    code = "acados_error";
else
    code = "simulation_error";
end
end

function addMirrorMetrics()
file = "large_yaw_turning_regression.csv";
if ~isfile(file)
    return;
end
data = readtable(file, "TextType", "string");
left = data(string(data.name) == "A3_90_left", :);
right = data(string(data.name) == "A3_90_right", :);
if isempty(left) || isempty(right)
    return;
end
rows = ismember(string(data.name), ["A3_90_left", "A3_90_right"]);
data.mirrorYawRmseDifferenceDeg(rows) = ...
    abs(left.yawRmseDeg - right.yawRmseDeg);
data.mirrorRadiusMagnitudeDifference(rows) = ...
    abs(abs(left.actualRadius) - abs(right.actualRadius));
data.mirrorXiDeltaPeakDifference(rows) = ...
    abs(left.maxAbsXiDelta - right.maxAbsXiDelta);
writetable(data, file);
end

function [time, data] = namedSignal(logs, name)
element = logs.get(name);
assert(~isempty(element), "Missing logged signal %s.", name);
time = element.Values.Time;
data = squeeze(element.Values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(time)
    data = data.';
end
end

function [time, data] = thinLoggedSignal(time, data, stopTime, sampleTime)
% Keep analysis logs bounded without changing the simulation or source log.
maxSamples = max(2, ceil(stopTime/sampleTime) + 1);
if numel(time) <= maxSamples
    return;
end
indices = unique(round(linspace(1, numel(time), maxSamples)));
time = time(indices);
data = data(indices, :);
end

function restoreModel(model, initFcn, wasDirty, openMechanicsExplorer)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn);
    set_param(model, "SimMechanicsOpenEditorOnUpdate", ...
        openMechanicsExplorer);
    if wasDirty == "off"
        set_param(model, "Dirty", "off");
    end
end
end
