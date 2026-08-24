function summary = run_stage1_split_cases(initialDeltas, stopTime)
%RUN_STAGE1_SPLIT_CASES Full-plant +/-10 mm anti-split ON/OFF tests.
% initialDeltas use the canonical convention (right-left)/2.

if nargin < 1 || isempty(initialDeltas)
    initialDeltas = [-0.010, 0.010];
end
if nargin < 2 || isempty(stopTime)
    stopTime = 8;
end
initialDeltas = double(initialDeltas(:));
[model, cleanup] = prepareModel(); %#ok<ASGLU>
records = repmat(emptyRecord(), 2*numel(initialDeltas), 1);
row = 0;
for enabled = [false, true]
    for k = 1:numel(initialDeltas)
        row = row + 1;
        records(row) = runCase(model, initialDeltas(k), enabled, stopTime);
    end
end
summary = struct2table(records);
writetable(summary, "stage1_split_summary.csv");
clear cleanup
end

function record = runCase(model, initialDelta, enabled, stopTime)
evalin("base", "startup");
set_initial_base_state(zeros(6, 1), zeros(4, 1));
configure_base_tracking_case("stand", "lqr", model);
ctrl = evalin("base", "ctrl");
ctrl.differentialLegForceStabilizer.enabled = enabled;
assignin("base", "ctrl", ctrl);
leg = evalin("base", "leg");
kin0 = wheel_leg_kinematics(leg.q0, zeros(3, 1), zeros(3, 1), leg);
% The high-priority Simscape assembly targets share the floating base and
% contact constraints, so the realized t=0 split is smaller than the raw IK
% offset. This factor was identified from the symmetric +/-10 mm pilot:
% 10/5.9031 = 1.6940. The realized initial value remains recorded below.
assemblyCompensation = 1.6940;
leftPosition = kin0.pO + [-assemblyCompensation*initialDelta; 0];
rightPosition = kin0.pO + [assemblyCompensation*initialDelta; 0];
qLeftLeg = wheel_leg_inverse_kinematics(leftPosition, [], [], leg);
qRightLeg = wheel_leg_inverse_kinematics(rightPosition, [], [], leg);
qLeft = [qLeftLeg; leg.q0(3)];
qRight = [qRightLeg; leg.q0(3)];

clear spatial_two_leg_qp_core coupled_two_leg_qp_core ...
    full_base_nmpc_command wheel_position_governor_step ...
    continuous_heading differential_leg_force_stabilizer
input = Simulink.SimulationInput(model);
input = setInitialLegState(input, qLeft, qRight);
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

xiCanonical = 0.5*(state(:, 14) - state(:, 13));
dxiCanonical = 0.5*(state(:, 16) - state(:, 15));
[requestedForce, appliedForce, amplitudeSaturated, rateLimited] = ...
    replayForce(time, xiCanonical, dxiCanonical, ...
    ctrl.differentialLegForceStabilizer);
recoveryTime = heldRecoveryTime(time, abs(xiCanonical), 1e-3, 1.0);
lastSecond = time >= max(0, time(end) - 1);

record = emptyRecord();
enabledLabel = "off";
if enabled
    enabledLabel = "on";
end
record.name = sprintf("split_%+dmm_%s", round(1e3*initialDelta), ...
    enabledLabel);
record.antiSplitEnabled = enabled;
record.requestedInitialXiDelta = initialDelta;
record.actualInitialXiDelta = xiCanonical(1);
record.simulationCompleted = strlength(errorText) == 0;
record.simulationError = simulationErrorCode(errorText);
record.recoveryTimeTo1mm = recoveryTime;
record.maxAbsXiDelta = max(abs(xiCanonical));
record.finalAbsXiDelta = abs(xiCanonical(end));
record.lastSecondMaxAbsXiDelta = max(abs(xiCanonical(lastSecond)));
record.xiDeltaIae = trapz(time, abs(xiCanonical));
record.maxAbsRequestedForce = max(abs(requestedForce));
record.maxAbsAppliedForce = max(abs(appliedForce));
record.amplitudeSaturationRatio = mean(amplitudeSaturated);
record.rateLimitRatio = mean(rateLimited);
record.maxAbsRollDeg = rad2deg(max(abs(state(:, 4))));
record.maxAbsPitchDeg = rad2deg(max(abs(state(:, 5))));
record.maxAbsBodySpeed = max(hypot(state(:, 7), state(:, 8)));
record.qpFeasibleRatio = mean(qp(:, 32) > 0.5);
record.maxDynamicsResidual = max(qp(:, 40));
record.maxWrenchResidual = max(qp(:, 42));
record.nmpcStatusMax = max(abs(status));
record.nmpcFaultRatio = mean(fault ~= 0);
record.p99NmpcSolveTimeMs = 1e3*prctile(cpu, 99);
record.nmpcDeadlineMissRatio = mean(cpu > 0.020);
record.maxQpSolveTimeMs = 1e3*max(qp(:, 54));
record.simulationWallTime = wallTime;
[record.pass, record.failureReason] = classify(record);
writeTimeseries(record.name, time, xiCanonical, dxiCanonical, ...
    requestedForce, appliedForce, state);
fprintf("%s: pass=%d, xi0=%.4f m, final=%.4f m, recover=%.3g s.\n", ...
    record.name, record.pass, record.actualInitialXiDelta, ...
    record.finalAbsXiDelta, record.recoveryTimeTo1mm);
end

function input = setInitialLegState(input, qLeft, qRight)
values = [qLeft(1)-pi/2, qLeft(2), qLeft(3), ...
    qRight(1)-pi/2, qRight(2), qRight(3)];
blocks = ["source/PD_only/Left Revolute Joint3", ...
    "source/PD_only/Revolute Joint1", ...
    "source/PD_only/Revolute Joint2", ...
    "source/PD_only/Right Revolute Joint", ...
    "source/PD_only/Revolute Joint4", ...
    "source/PD_only/Revolute Joint5"];
for k = 1:numel(blocks)
    input = input.setBlockParameter(blocks(k), ...
        "PositionTargetValue", sprintf("%.17g", values(k)));
end
end

function [requested, applied, saturated, rateLimited] = ...
        replayForce(time, xi, dxi, config)
clear differential_leg_force_stabilizer
requested = zeros(size(time));
applied = zeros(size(time));
saturated = false(size(time));
rateLimited = false(size(time));
for k = 1:numel(time)
    [applied(k), diagnostics] = differential_leg_force_stabilizer( ...
        time(k), xi(k), dxi(k), config);
    requested(k) = diagnostics.requested;
    saturated(k) = diagnostics.amplitudeSaturated;
    rateLimited(k) = diagnostics.rateLimited;
end
clear differential_leg_force_stabilizer
end

function recoveryTime = heldRecoveryTime(time, error, threshold, holdTime)
recoveryTime = nan;
for k = 1:numel(time)
    finish = find(time >= time(k) + holdTime, 1, "first");
    if ~isempty(finish) && all(error(k:finish) <= threshold)
        recoveryTime = time(k);
        return;
    end
end
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
if record.antiSplitEnabled && (isnan(record.recoveryTimeTo1mm) ...
        || record.recoveryTimeTo1mm > 5 ...
        || record.lastSecondMaxAbsXiDelta > 1e-3)
    failures(end + 1) = "anti-split recovery";
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
numericNames = ["requestedInitialXiDelta", "actualInitialXiDelta", ...
    "recoveryTimeTo1mm", "maxAbsXiDelta", "finalAbsXiDelta", ...
    "lastSecondMaxAbsXiDelta", "xiDeltaIae", ...
    "maxAbsRequestedForce", "maxAbsAppliedForce", ...
    "amplitudeSaturationRatio", "rateLimitRatio", ...
    "maxAbsRollDeg", "maxAbsPitchDeg", "maxAbsBodySpeed", ...
    "qpFeasibleRatio", "maxDynamicsResidual", "maxWrenchResidual", ...
    "nmpcStatusMax", "nmpcFaultRatio", "p99NmpcSolveTimeMs", ...
    "nmpcDeadlineMissRatio", "maxQpSolveTimeMs", ...
    "simulationWallTime"];
record = struct("name", "");
for name = numericNames
    record.(name) = nan;
end
record.antiSplitEnabled = false;
record.simulationCompleted = false;
record.simulationError = "";
record.pass = false;
record.failureReason = "";
end

function writeTimeseries(name, time, xi, dxi, requested, applied, state)
sample = [true; diff(floor(time/0.05)) > 0];
data = table(time(sample), xi(sample), dxi(sample), ...
    requested(sample), applied(sample), state(sample, 4), ...
    state(sample, 5), hypot(state(sample, 7), state(sample, 8)), ...
    'VariableNames', {'time', 'xiDeltaCanonical', ...
    'dxiDeltaCanonical', 'forceRequested', 'forceAppliedReplay', ...
    'roll', 'pitch', 'bodySpeed'});
folder = "stage1_split_timeseries";
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
