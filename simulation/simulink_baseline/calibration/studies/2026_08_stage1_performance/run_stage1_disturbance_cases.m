function summary = run_stage1_disturbance_cases( ...
        caseFilter, forceRatios, pulseWidth, stopTime)
%RUN_STAGE1_DISTURBANCE_CASES Physical body-force recovery sweep.
% Force ratios are normalized by total robot weight.  The force is resolved
% in the World frame by the External Force and Torque block in source.slx.

if nargin < 1 || isempty(caseFilter)
    caseFilter = "all";
end
if nargin < 2 || isempty(forceRatios)
    forceRatios = [0.10, 0.25, 0.50, 0.75];
end
if nargin < 3 || isempty(pulseWidth)
    pulseWidth = 0.50;
end
if nargin < 4 || isempty(stopTime)
    stopTime = 12.0;
end
validateattributes(forceRatios, {'numeric'}, ...
    {'vector', 'real', 'finite', 'nonnegative'});
validateattributes(pulseWidth, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(stopTime, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>', 7});

[model, cleanup] = prepareModel(); %#ok<ASGLU>
cases = buildCases(forceRatios(:), pulseWidth, stopTime);
caseFilter = string(caseFilter);
if ~strcmpi(caseFilter, "all")
    names = string({cases.name});
    selected = contains(names, caseFilter, "IgnoreCase", true);
    cases = cases(selected);
    if isempty(cases)
        error("run_stage1_disturbance_cases:UnknownFilter", ...
            "No disturbance case matches '%s'.", caseFilter);
    end
end

records = repmat(emptyRecord(), numel(cases), 1);
for k = 1:numel(cases)
    records(k) = runCase(model, cases(k));
end
summary = struct2table(records);
writetable(summary, "stage1_disturbance_summary.csv");
clear cleanup
end

function cases = buildCases(forceRatios, pulseWidth, stopTime)
scenarios = [
    struct("name", "stand", "speed", 0, "yawRate", 0)
    struct("name", "straight", "speed", 0.10, "yawRate", 0)
    struct("name", "turn", "speed", 0.10, "yawRate", 0.08)
];
axes = [
    struct("name", "world_x", "vector", [1, 0, 0])
    % Simscape Multibody uses X forward, Y vertical, Z lateral in this
    % plant.  The controller state later reorders Z into its lateral y.
    struct("name", "world_z", "vector", [0, 0, 1])
];
template = struct("name", "", "scenario", "", "speed", 0, ...
    "yawRate", 0, "axis", "", "direction", zeros(1, 3), ...
    "forceRatio", 0, "pulseStart", 5.0, ...
    "pulseWidth", pulseWidth, "stopTime", stopTime);
cases = repmat(template, numel(scenarios)*numel(axes)*numel(forceRatios), 1);
row = 0;
for s = 1:numel(scenarios)
    for a = 1:numel(axes)
        for r = 1:numel(forceRatios)
            row = row + 1;
            ratioToken = replace(sprintf("%.3f", forceRatios(r)), ".", "p");
            cases(row) = template;
            cases(row).name = scenarios(s).name + "_" + axes(a).name ...
                + "_r" + ratioToken;
            cases(row).scenario = scenarios(s).name;
            cases(row).speed = scenarios(s).speed;
            cases(row).yawRate = scenarios(s).yawRate;
            cases(row).axis = axes(a).name;
            cases(row).direction = axes(a).vector;
            cases(row).forceRatio = forceRatios(r);
        end
    end
end
end

function record = runCase(model, testCase)
evalin("base", "startup");
set_initial_base_state(zeros(6, 1), zeros(4, 1));
configure_turning_case(testCase.speed, testCase.yawRate, "single", model);
base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
trajectory = base.trajectory;
trajectory.cruiseDuration = testCase.stopTime + 2.0;
trajectory.decelDuration = 1.0;
trajectory.turnHoldDuration = 0;
if testCase.yawRate == 0
    trajectory.turning.enabled = false;
else
    trajectory.turning.enabled = true;
    trajectory.turning.startTime = 2.2;
    trajectory.turning.rampDuration = 0.5;
    trajectory.turning.holdDuration = testCase.stopTime;
    trajectory.turning.zeroHoldDuration = 0.5;
end
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);

robotMass = evalin("base", ...
    "baseBodyMass + 2*(leg.m1 + leg.m2 + leg.mw)");
forceN = testCase.forceRatio*robotMass*9.81;
simin = buildForceCommand(testCase, forceN);

clear spatial_two_leg_qp_core coupled_two_leg_qp_core ...
    full_base_nmpc_command wheel_position_governor_step ...
    continuous_heading differential_leg_force_stabilizer
fprintf("%s: %.2f N (%.2f mg), %.2f s pulse.\n", ...
    testCase.name, forceN, testCase.forceRatio, testCase.pulseWidth);
in = Simulink.SimulationInput(model);
in = in.setVariable("simin", simin);
in = in.setModelParameter( ...
    "StopTime", num2str(testCase.stopTime, "%.9g"), ...
    "SimulationMode", "accelerator", ...
    "ReturnWorkspaceOutputs", "on", ...
    "SignalLogging", "on", ...
    "SaveOutput", "off", "SaveState", "off", ...
    "SaveFinalState", "off", "SaveTime", "off", ...
    "CaptureErrors", "on");
wallClock = tic;
out = sim(in);
wallTime = toc(wallClock);
errorText = string(out.ErrorMessage);
logs = out.logsout;
[time, state] = namedSignal(logs, "baseNmpcState");
[qpTime, qp] = namedSignal(logs, "coupledQpSignal");
[statusTime, status] = namedSignal(logs, "nmpcStatus");
[cpuTime, cpu] = namedSignal(logs, "nmpcCpuTime");
[faultTime, fault] = namedSignal(logs, "nmpcFault");

bodyForward = cos(state(:, 6)).*state(:, 7) ...
    - sin(state(:, 6)).*state(:, 8);
bodyLateral = sin(state(:, 6)).*state(:, 7) ...
    + cos(state(:, 6)).*state(:, 8);
speedError = bodyForward - testCase.speed;
xiDelta = 0.5*(state(:, 13) - state(:, 14));
contactResidual = abs(qp(:, 49:51));
frictionMargin = min(qp(:, 61:64), [], 2);
torqueMargin = min(qp(:, 65:70), [], 2);
pulseEnd = testCase.pulseStart + testCase.pulseWidth;
postPulse = time >= pulseEnd;
tail = time >= max(time(1), time(end) - 1.0);
recoveryTime = findRecoveryTime(time, pulseEnd, speedError, ...
    bodyLateral, state(:, 4), state(:, 5), xiDelta, testCase.speed);

record = emptyRecord();
record.name = string(testCase.name);
record.scenario = string(testCase.scenario);
record.axis = string(testCase.axis);
record.speedRef = testCase.speed;
record.yawRateRef = testCase.yawRate;
record.forceRatioMg = testCase.forceRatio;
record.forceN = forceN;
record.pulseWidth = testCase.pulseWidth;
record.simulationCompleted = strlength(errorText) == 0;
record.simulationError = simulationErrorCode(errorText);
record.recoveryTime = recoveryTime;
record.maxAbsForwardSpeedErrorAfterPulse = safeMaxAbs(speedError(postPulse));
record.tailAbsForwardSpeedError = safeMaxAbs(speedError(tail));
record.maxAbsLateralVelocityAfterPulse = safeMaxAbs(bodyLateral(postPulse));
record.tailAbsLateralVelocity = safeMaxAbs(bodyLateral(tail));
record.maxAbsRollDeg = rad2deg(max(abs(state(:, 4))));
record.maxAbsPitchDeg = rad2deg(max(abs(state(:, 5))));
record.tailAbsRollDeg = rad2deg(max(abs(state(tail, 4))));
record.tailAbsPitchDeg = rad2deg(max(abs(state(tail, 5))));
record.maxAbsXiDeltaMm = 1e3*max(abs(xiDelta));
record.tailAbsXiDeltaMm = 1e3*max(abs(xiDelta(tail)));
record.maxWrenchSlackNorm = max(qp(:, 31));
record.maxNormalResidual = max(contactResidual(:, 3));
record.qpFeasibleRatio = mean(qp(:, 32) > 0.5);
record.maxDynamicsResidual = max(qp(:, 40));
record.maxWrenchResidual = max(qp(:, 42));
record.minFrictionMargin = min(frictionMargin);
record.minTorqueMargin = min(torqueMargin);
record.nmpcStatusMax = max(abs(status));
record.nmpcFaultRatio = mean(fault ~= 0);
record.p99NmpcSolveTimeMs = 1e3*prctile(cpu, 99);
record.nmpcDeadlineMissRatio = mean(cpu > 0.020);
record.maxQpSolveTimeMs = 1e3*max(qp(:, 54));
record.simulationWallTime = wallTime;
[record.stable, record.realtimePass, record.failureReason] = ...
    classify(record, testCase.speed);
writeTimeseries(testCase, forceN, time, state, bodyForward, ...
    bodyLateral, speedError, xiDelta, qpTime, qp, statusTime, status, ...
    cpuTime, cpu, faultTime, fault, errorText);
fprintf("%s: stable=%d, recovery=%.3g s, roll/pitch=%.2f/%.2f deg.\n", ...
    record.name, record.stable, record.recoveryTime, ...
    record.maxAbsRollDeg, record.maxAbsPitchDeg);
end

function simin = buildForceCommand(testCase, forceN)
dt = 0.005;
t = (0:dt:testCase.stopTime).';
edgeTime = min(0.05, 0.2*testCase.pulseWidth);
pulseStart = testCase.pulseStart;
pulseEnd = pulseStart + testCase.pulseWidth;
shape = zeros(size(t));
rise = t >= pulseStart & t < pulseStart + edgeTime;
shape(rise) = 0.5*(1 - cos(pi*(t(rise) - pulseStart)/edgeTime));
hold = t >= pulseStart + edgeTime & t <= pulseEnd - edgeTime;
shape(hold) = 1;
fall = t > pulseEnd - edgeTime & t <= pulseEnd;
shape(fall) = 0.5*(1 + cos(pi*(t(fall) ...
    - (pulseEnd - edgeTime))/edgeTime));
force = forceN*shape.*testCase.direction;
simin = [t, force];
end

function recoveryTime = findRecoveryTime(time, pulseEnd, speedError, ...
        bodyLateral, roll, pitch, xiDelta, speedRef)
if speedRef == 0
    speedLimit = 0.02;
else
    speedLimit = 0.03;
end
inside = abs(speedError) <= speedLimit ...
    & abs(bodyLateral) <= 0.02 ...
    & abs(rad2deg(roll)) <= 2 ...
    & abs(rad2deg(pitch)) <= 3 ...
    & abs(xiDelta) <= 1e-3;
recoveryTime = inf;
candidate = find(time >= pulseEnd & inside);
for k = 1:numel(candidate)
    startIndex = candidate(k);
    endTime = time(startIndex) + 0.5;
    endIndex = find(time >= endTime, 1);
    if ~isempty(endIndex) && all(inside(startIndex:endIndex))
        recoveryTime = time(startIndex) - pulseEnd;
        return
    end
end
end

function [stable, realtimePass, reason] = classify(record, speedRef)
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
if record.maxAbsRollDeg > 2 || record.maxAbsPitchDeg > 3
    failures(end + 1) = "attitude";
end
if record.maxDynamicsResidual >= 1e-6 ...
        || record.maxWrenchResidual >= 1e-6
    failures(end + 1) = "dynamics/wrench residual";
end
% The QP solution is accepted to roughly 1e-9 numerical precision.  Do not
% classify roundoff-level negative margins as physical constraint failure.
if record.minFrictionMargin < -1e-6 || record.minTorqueMargin < -1e-6
    failures(end + 1) = "constraint margin";
end
if ~isfinite(record.recoveryTime)
    failures(end + 1) = "not recovered";
end
speedLimit = 0.03;
if speedRef == 0
    speedLimit = 0.02;
end
if record.tailAbsForwardSpeedError > speedLimit ...
        || record.tailAbsLateralVelocity > 0.02 ...
        || record.tailAbsXiDeltaMm > 1
    failures(end + 1) = "tail recovery";
end
stable = isempty(failures);
realtimePass = record.p99NmpcSolveTimeMs < 20 ...
    && record.nmpcDeadlineMissRatio == 0 ...
    && record.maxQpSolveTimeMs < 5;
reason = "ok";
if ~stable
    reason = strjoin(failures, "; ");
end
end

function record = emptyRecord()
record = struct("name", "", "scenario", "", "axis", "", ...
    "speedRef", NaN, "yawRateRef", NaN, "forceRatioMg", NaN, ...
    "forceN", NaN, "pulseWidth", NaN, ...
    "simulationCompleted", false, "simulationError", "", ...
    "recoveryTime", NaN, ...
    "maxAbsForwardSpeedErrorAfterPulse", NaN, ...
    "tailAbsForwardSpeedError", NaN, ...
    "maxAbsLateralVelocityAfterPulse", NaN, ...
    "tailAbsLateralVelocity", NaN, ...
    "maxAbsRollDeg", NaN, "maxAbsPitchDeg", NaN, ...
    "tailAbsRollDeg", NaN, "tailAbsPitchDeg", NaN, ...
    "maxAbsXiDeltaMm", NaN, "tailAbsXiDeltaMm", NaN, ...
    "maxWrenchSlackNorm", NaN, "maxNormalResidual", NaN, ...
    "qpFeasibleRatio", NaN, "maxDynamicsResidual", NaN, ...
    "maxWrenchResidual", NaN, "minFrictionMargin", NaN, ...
    "minTorqueMargin", NaN, "nmpcStatusMax", NaN, ...
    "nmpcFaultRatio", NaN, "p99NmpcSolveTimeMs", NaN, ...
    "nmpcDeadlineMissRatio", NaN, "maxQpSolveTimeMs", NaN, ...
    "simulationWallTime", NaN, "stable", false, ...
    "realtimePass", false, "failureReason", "not run");
end

function writeTimeseries(testCase, forceN, time, state, bodyForward, ...
        bodyLateral, speedError, xiDelta, qpTime, qp, statusTime, status, ...
        cpuTime, cpu, faultTime, fault, errorText)
forceCommand = buildForceCommand(testCase, forceN);
force = interp1(forceCommand(:, 1), forceCommand(:, 2:4), ...
    time, "linear", 0);
stateTable = table(time, force(:, 1), force(:, 2), force(:, 3), ...
    state(:, 1), state(:, 2), state(:, 3), state(:, 4), state(:, 5), ...
    state(:, 6), bodyForward, bodyLateral, speedError, xiDelta, ...
    'VariableNames', {'time','forceWorldX','forceWorldY','forceWorldZ', ...
    'x','y','z','roll','pitch','yaw','bodyForward','bodyLateral', ...
    'speedError','xiDelta'});
writetable(stateTable, testCase.name + "_timeseries.csv");
save(testCase.name + "_raw.mat", "testCase", "forceN", "errorText", "time", ...
    "state", "bodyForward", "bodyLateral", "speedError", "xiDelta", ...
    "qpTime", "qp", "statusTime", "status", "cpuTime", "cpu", ...
    "faultTime", "fault");
end

function value = safeMaxAbs(data)
if isempty(data)
    value = NaN;
else
    value = max(abs(data));
end
end

function [time, data] = namedSignal(logs, name)
element = logs.get(name);
time = double(element.Values.Time(:));
data = squeeze(double(element.Values.Data));
if isvector(data)
    data = data(:);
end
end

function code = simulationErrorCode(message)
code = "";
if strlength(message) == 0
    return
end
token = regexp(message, "([A-Za-z]\w*:[A-Za-z]\w*)", ...
    "tokens", "once");
if isempty(token)
    code = extractBefore(message + newline, newline);
else
    code = string(token{1});
end
end

function [model, cleanup] = prepareModel()
studyDir = fileparts(mfilename("fullpath"));
root = fileparts(fileparts(fileparts(studyDir)));
modelDir = fullfile(root, "model", "simulate", "two_legs");
codeDir = fullfile(root, "model", "code");
addpath(modelDir, codeDir, "-begin");
model = "source";
evalin("base", "startup");
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
openMechanicsExplorer = get_param(model, ...
    "SimMechanicsOpenEditorOnUpdate");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty, ...
    openMechanicsExplorer));
set_param(model, "InitFcn", "");
set_param(model, "SimMechanicsOpenEditorOnUpdate", "off");
end

function restoreModel(model, initFcn, wasDirty, openMechanicsExplorer)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn);
    set_param(model, "SimMechanicsOpenEditorOnUpdate", ...
        openMechanicsExplorer);
    if strcmpi(wasDirty, "off")
        set_param(model, "Dirty", "off");
    end
    close_system(model, 0);
end
end
