function summary = run_stage1_straight_cases( ...
        speeds, stopTime, transitionDuration)
%RUN_STAGE1_STRAIGHT_CASES Full-plant start/hold/brake tests.

if nargin < 1 || isempty(speeds)
    speeds = [0.10, 0.20];
end
if nargin < 2 || isempty(stopTime)
    stopTime = 16;
end
if nargin < 3 || isempty(transitionDuration)
    transitionDuration = NaN;
end
speeds = double(speeds(:));
validateattributes(stopTime, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(transitionDuration, {'numeric'}, ...
    {'scalar', 'real'});
if ~isnan(transitionDuration)
    validateattributes(transitionDuration, {'numeric'}, ...
        {'finite', 'positive'});
end

[model, cleanup] = prepareModel(); %#ok<ASGLU>
records = repmat(emptyRecord(), numel(speeds), 1);
for k = 1:numel(speeds)
    records(k) = runCase( ...
        model, speeds(k), stopTime, transitionDuration);
end
summary = struct2table(records);
writetable(summary, "stage1_straight_summary.csv");
clear cleanup
end

function record = runCase(model, speed, stopTime, transitionDuration)
evalin("base", "startup");
set_initial_base_state(zeros(6, 1), zeros(4, 1));
configure_turning_case(speed, 0, "single", model);
base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
trajectory = base.trajectory;
trajectory.turning.enabled = false;
if isnan(transitionDuration)
    trajectory.decelDuration = 2.0;
else
    trajectory.accelDuration = transitionDuration;
    trajectory.decelDuration = transitionDuration;
end
trajectory.turnHoldDuration = 2.0;
trajectory.cruiseDuration = stopTime - trajectory.settleTime ...
    - trajectory.accelDuration - trajectory.decelDuration ...
    - trajectory.turnHoldDuration;
assert(trajectory.cruiseDuration > 1, ...
    "Stop time is too short for a start/hold/brake case.");
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);

clear spatial_two_leg_qp_core coupled_two_leg_qp_core ...
    full_base_nmpc_command wheel_position_governor_step ...
    continuous_heading differential_leg_force_stabilizer
input = Simulink.SimulationInput(model);
input = input.setBlockParameter( ...
    model + "/PD_only/Brick Solid3", ...
    "BrickDimensions", "[100 0.05 100]");
input = input.setModelParameter( ...
    "StopTime", num2str(stopTime, "%.9g"), ...
    "SimulationMode", "accelerator", ...
    "ReturnWorkspaceOutputs", "on", ...
    "SignalLogging", "on", ...
    "SaveOutput", "off", "SaveState", "off", ...
    "SaveFinalState", "off", "SaveTime", "off", ...
    "CaptureErrors", "on");
wallClock = tic;
out = sim(input);
wallTime = toc(wallClock);
errorText = string(out.ErrorMessage);
logs = out.logsout;
[time, state] = namedSignal(logs, "baseNmpcState");
[~, qp] = namedSignal(logs, "coupledQpSignal");
[~, status] = namedSignal(logs, "nmpcStatus");
[~, cpu] = namedSignal(logs, "nmpcCpuTime");
[~, fault] = namedSignal(logs, "nmpcFault");

referenceSpeed = zeros(size(time));
for i = 1:numel(time)
    reference = floating_base_reference(time(i), baseLqr);
    referenceSpeed(i) = reference(4);
end
bodyForward = cos(state(:, 6)).*state(:, 7) ...
    - sin(state(:, 6)).*state(:, 8);
active = abs(referenceSpeed) >= 0.8*abs(speed);
settled = time >= max(0, stopTime - 0.5);
xiDelta = 0.5*(state(:, 13) - state(:, 14));
contactResidual = abs(qp(:, 49:51));

record = emptyRecord();
record.name = "straight_v" + replace(sprintf("%.2f", speed), ".", "p");
record.speedRef = speed;
record.stopTime = stopTime;
record.simulationCompleted = strlength(errorText) == 0;
record.simulationError = simulationErrorCode(errorText);
record.forwardVelocityRmse = rms(bodyForward(active) - referenceSpeed(active));
record.maxForwardVelocityError = max(abs( ...
    bodyForward(active) - referenceSpeed(active)));
record.finalAbsForwardVelocity = max(abs(bodyForward(settled)));
record.maxAbsRollDeg = rad2deg(max(abs(state(:, 4))));
record.maxAbsPitchDeg = rad2deg(max(abs(state(:, 5))));
record.maxAbsXiDelta = max(abs(xiDelta));
record.finalAbsXiDelta = abs(xiDelta(end));
record.qpFeasibleRatio = mean(qp(:, 32) > 0.5);
record.maxDynamicsResidual = max(qp(:, 40));
record.maxWrenchResidual = max(qp(:, 42));
record.maxContactResidual = max(contactResidual, [], "all");
record.nmpcStatusMax = max(abs(status));
record.nmpcFaultRatio = mean(fault ~= 0);
record.p99NmpcSolveTimeMs = 1e3*prctile(cpu, 99);
record.nmpcDeadlineMissRatio = mean(cpu > 0.020);
record.maxQpSolveTimeMs = 1e3*max(qp(:, 54));
record.simulationWallTime = wallTime;
[record.pass, record.failureReason] = classify(record);
writeTimeseries(record.name, time, referenceSpeed, bodyForward, state);
fprintf("%s: pass=%d, vRMSE=%.4f m/s, stop=%.4f m/s, xi=%.4f m.\n", ...
    record.name, record.pass, record.forwardVelocityRmse, ...
    record.finalAbsForwardVelocity, record.maxAbsXiDelta);
end

function [pass, reason] = classify(record)
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
if record.forwardVelocityRmse > 0.03
    failures(end + 1) = "velocity RMSE";
end
if record.finalAbsForwardVelocity > 0.02
    failures(end + 1) = "braking";
end
if record.maxAbsRollDeg > 2 || record.maxAbsPitchDeg > 3
    failures(end + 1) = "attitude";
end
if record.maxDynamicsResidual >= 1e-6 ...
        || record.maxWrenchResidual >= 1e-6
    failures(end + 1) = "dynamics/wrench residual";
end
pass = isempty(failures);
reason = "ok";
if ~pass
    reason = strjoin(failures, "; ");
end
end

function record = emptyRecord()
numericNames = ["speedRef", "stopTime", "forwardVelocityRmse", ...
    "maxForwardVelocityError", "finalAbsForwardVelocity", ...
    "maxAbsRollDeg", "maxAbsPitchDeg", "maxAbsXiDelta", ...
    "finalAbsXiDelta", "qpFeasibleRatio", "maxDynamicsResidual", ...
    "maxWrenchResidual", "maxContactResidual", "nmpcStatusMax", ...
    "nmpcFaultRatio", "p99NmpcSolveTimeMs", ...
    "nmpcDeadlineMissRatio", "maxQpSolveTimeMs", ...
    "simulationWallTime"];
record = struct("name", "");
for name = numericNames
    record.(name) = nan;
end
record.simulationCompleted = false;
record.simulationError = "";
record.pass = false;
record.failureReason = "";
end

function writeTimeseries(name, time, referenceSpeed, bodyForward, state)
sample = [true; diff(floor(time/0.1)) > 0];
data = table(time(sample), referenceSpeed(sample), bodyForward(sample), ...
    state(sample, 4), state(sample, 5), ...
    0.5*(state(sample, 13) - state(sample, 14)), ...
    'VariableNames', {'time', 'referenceSpeed', 'bodyForwardVelocity', ...
    'roll', 'pitch', 'xiDelta'});
folder = "stage1_straight_timeseries";
if ~isfolder(folder)
    mkdir(folder);
end
writetable(data, fullfile(folder, name + ".csv"));
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
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty));
set_param(model, "InitFcn", "");
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

function code = simulationErrorCode(message)
if strlength(message) == 0
    code = "";
elseif contains(message, "Nonlinear iteration") && contains(message, "hmin")
    code = "simscape_nonlinear_hmin";
else
    code = "simulation_error";
end
end

function restoreModel(model, initFcn, wasDirty)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn);
    if wasDirty == "off"
        set_param(model, "Dirty", "off");
    end
end
end
