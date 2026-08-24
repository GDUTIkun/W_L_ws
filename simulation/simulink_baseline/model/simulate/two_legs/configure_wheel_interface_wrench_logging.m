function [logging, verification] = configure_wheel_interface_wrench_logging(varargin)
%CONFIGURE_WHEEL_INTERFACE_WRENCH_LOGGING Observe wheel-joint wrenches only.
%   Adds temporary PS-Simulink Converter/Terminator branches for the total
%   force and total torque outputs of the two frozen wheel revolute joints.
%   The source model is never saved.  Enable/disable leaves the pre-existing
%   on-disk model and every non-observation setting unchanged.

parser = inputParser;
parser.addParameter("Mode", "enable", @(x) ismember(string(x), ...
    ["enable", "disable", "verify_invariance"]));
parser.addParameter("Model", "source", @(x) ischar(x) || isstring(x));
parser.addParameter("DoSave", false, @(x) islogical(x) && isscalar(x));
parser.addParameter("InvarianceTolerance", 1.601e-6, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.parse(varargin{:});
mode = string(parser.Results.Mode);
model = string(parser.Results.Model);

assert(~parser.Results.DoSave, ...
    "configure_wheel_interface_wrench_logging:SaveForbidden", ...
    "This observation-only helper never permits saving source.slx.");

if mode == "verify_invariance"
    [logging, verification] = verifyInvariance(model, ...
        parser.Results.InvarianceTolerance);
    return;
end

load_system(model);
wasDirty = string(get_param(model, "Dirty"));
cleanup = onCleanup(@() restoreDirty(model, wasDirty));
contract = wheel_interface_wrench_contract();
signals = signalContract(contract);
assertAuditedJoints(contract, model);
before = invariantSnapshot(model, contract);

if mode == "enable"
    for item = signals
        set_param(item.jointPath, item.sensingParameter, "on");
    end
    set_param(model, "SimulationCommand", "update");
    wired = struct();
    for item = signals
        converter = ensureConverter(item);
        terminator = ensureTerminator(item);
        connectPhysicalSignal(item, converter);
        ensureLoggedLine(converter, terminator, item.loggingName);
        wired.(matlab.lang.makeValidName(item.loggingName)) = struct( ...
            "converter", converter, "terminator", terminator, ...
            "sourcePort", item.sourcePort, "unit", item.converterUnit);
    end
else
    for item = signals
        removeWiring(item);
    end
    wired = struct();
end

after = invariantSnapshot(model, contract);
restoreDirty(model, wasDirty);
logging = struct("contractVersion", contract.contractVersion, ...
    "signals", signals, "wired", wired, ...
    "rawFrame", contract.rawFrame, ...
    "nativeTimestamp", contract.nativeTimestamp, ...
    "rawChannelsRetained", true, "nativeMetadataRetained", true);
verification = struct("mode", mode, "model", model, ...
    "observationOnly", true, "saved", false, ...
    "invariantBefore", before, "invariantAfter", after, ...
    "invariantUnchanged", isequaln(before, after), ...
    "sensingEnabled", mode == "enable", ...
    "rawChannelsRetained", true, "nativeMetadataRetained", true, ...
    "modelDirtyAfter", string(get_param(model, "Dirty")));
if ~verification.invariantUnchanged
    error("configure_wheel_interface_wrench_logging:InvariantChanged", ...
        "Joint/controller/plant invariant snapshot changed during %s.", mode);
end
clear cleanup
end

function signals = signalContract(contract)
% RConn4/RConn5 are the audited ports materialized by SenseTotalForce and
% SenseTotalTorque after the existing position/velocity RConn2/RConn3 ports.
signals = [ ...
    makeSignal(contract.left, "TotalForce", "RConn4", "leftTotalForce", "N"), ...
    makeSignal(contract.left, "TotalTorque", "RConn5", "leftTotalTorque", "N*m"), ...
    makeSignal(contract.right, "TotalForce", "RConn4", "rightTotalForce", "N"), ...
    makeSignal(contract.right, "TotalTorque", "RConn5", "rightTotalTorque", "N*m")];
end

function item = makeSignal(side, suffix, port, name, unit)
if suffix == "TotalForce"
    rawUnit = side.forceUnit;
else
    rawUnit = side.torqueUnit;
end
item = struct("side", side.side, "jointPath", side.jointPath, ...
    "sensingParameter", "Sense" + suffix, "sourcePort", port, ...
    "loggingName", name, "converterUnit", unit, ...
    "rawUnit", string(rawUnit), ...
    "actor", side.actor, "receiver", side.receiver, ...
    "frame", side.compositeWrenchFrame, ...
    "direction", side.compositeWrenchDir, "observationOnly", true);
end

function assertAuditedJoints(contract, model)
assert(model == "source", "configure_wheel_interface_wrench_logging:ModelIdentity", ...
    "Only the frozen source model may expose this contract.");
for side = [contract.left, contract.right]
    assert(getSimulinkBlockHandle(side.jointPath) ~= -1, ...
        "configure_wheel_interface_wrench_logging:JointLookup", ...
        "Audited joint is missing: %s", side.jointPath);
    assert(string(get_param(side.jointPath, "CompositeWrenchDir")) == ...
        side.compositeWrenchDir && string(get_param(side.jointPath, ...
        "CompositeWrenchFrame")) == side.compositeWrenchFrame, ...
        "configure_wheel_interface_wrench_logging:JointSemantics", ...
        "Frozen total-wrench semantics differ at %s.", side.jointPath);
end
end

function snapshot = invariantSnapshot(model, contract)
snapshot = struct();
snapshot.model = struct("solver", string(get_param(model, "Solver")), ...
    "solverType", string(get_param(model, "SolverType")), ...
    "maxStep", string(get_param(model, "MaxStep")), ...
    "relTol", string(get_param(model, "RelTol")), ...
    "absTol", string(get_param(model, "AbsTol")));
for side = [contract.left, contract.right]
    prefix = matlab.lang.makeValidName(side.side);
    snapshot.(prefix) = struct( ...
        "jointPath", side.jointPath, ...
        "direction", string(get_param(side.jointPath, "CompositeWrenchDir")), ...
        "frame", string(get_param(side.jointPath, "CompositeWrenchFrame")), ...
        "leftConnection", destination(side.jointPath, "LConn1"), ...
        "rightConnection", destination(side.jointPath, "LConn2"));
end
for block = ["source/PD_only/Spatial" + newline + "Contact Force", ...
             "source/PD_only/Spatial" + newline + "Contact Force1"]
    key = matlab.lang.makeValidName(replace(block, "/", "_"));
    snapshot.(key) = struct( ...
        "normalStiffness", string(get_param(block, "NormalStiffness")), ...
        "normalDamping", string(get_param(block, "NormalDamping")));
end
qp = "source/PD_only/Coupled QP";
snapshot.controller = struct("qpDimensions", string(get_param(qp, ...
    "OutputDimensions")), "qpOutputLine", outputLine(qp));
end

function value = outputLine(block)
handles = get_param(block, "PortHandles");
assert(~isempty(handles.Outport), ...
    "configure_wheel_interface_wrench_logging:PortLookup", ...
    "Controller output is unavailable on %s.", block);
value = double(get_param(handles.Outport(1), "Line"));
end

function value = destination(block, portType)
pcs = get_param(block, "PortConnectivity");
index = find(string({pcs.Type}) == portType, 1);
assert(~isempty(index), "configure_wheel_interface_wrench_logging:PortLookup", ...
    "Port %s is unavailable on %s.", portType, block);
value = double(pcs(index).DstBlock(:)).';
end

function path = ensureConverter(item)
parent = subsystemParent(item.jointPath);
path = parent + "/Wheel Wrench Converter " + item.loggingName;
if getSimulinkBlockHandle(path) == -1
    position = wiringPosition(item);
    template = "source/PD_only/PS-Simulink" + newline + "Converter";
    add_block(char(template), char(path), "Unit", item.converterUnit, ...
        "Position", [position(1) + 50, position(2) - 7, ...
        position(1) + 85, position(2) + 7]);
end
end

function path = ensureTerminator(item)
parent = subsystemParent(item.jointPath);
path = parent + "/Wheel Wrench Terminator " + item.loggingName;
if getSimulinkBlockHandle(path) == -1
    position = wiringPosition(item);
    add_block("simulink/Sinks/Terminator", char(path), ...
        "Position", [position(1) + 125, position(2) - 7, ...
        position(1) + 145, position(2) + 7]);
end
end

function position = wiringPosition(item)
pcs = get_param(item.jointPath, "PortConnectivity");
index = find(string({pcs.Type}) == item.sourcePort, 1);
assert(~isempty(index), "configure_wheel_interface_wrench_logging:PortLookup", ...
    "Expected %s after enabling %s at %s.", item.sourcePort, ...
    item.sensingParameter, item.jointPath);
position = pcs(index).Position;
end

function connectPhysicalSignal(item, converter)
pcs = get_param(item.jointPath, "PortConnectivity");
index = find(string({pcs.Type}) == item.sourcePort, 1);
assert(~isempty(index), "configure_wheel_interface_wrench_logging:PortLookup", ...
    "Expected sensing port %s is unavailable.", item.sourcePort);
if ~isempty(pcs(index).DstBlock)
    return;
end
sourceHandles = get_param(item.jointPath, "PortHandles");
converterHandles = get_param(converter, "PortHandles");
portNumber = str2double(extractAfter(item.sourcePort, "RConn"));
assert(portNumber <= numel(sourceHandles.RConn) && ~isempty(converterHandles.LConn), ...
    "configure_wheel_interface_wrench_logging:PortLookup", ...
    "Cannot address %s on %s.", item.sourcePort, item.jointPath);
add_line(char(subsystemParent(item.jointPath)), sourceHandles.RConn(portNumber), ...
    converterHandles.LConn(1));
end

function ensureLoggedLine(converter, terminator, loggingName)
handles = get_param(converter, "PortHandles");
line = get_param(handles.Outport(1), "Line");
if isempty(line) || (isnumeric(line) && isscalar(line) && line == -1)
    add_line(char(subsystemParent(converter)), ...
        char(string(get_param(converter, "Name")) + "/1"), ...
        char(string(get_param(terminator, "Name")) + "/1"), "autorouting", "on");
    line = get_param(handles.Outport(1), "Line");
end
assert(~isempty(line) && ~(isnumeric(line) && isscalar(line) && line == -1), ...
    "configure_wheel_interface_wrench_logging:LoggingLine", ...
    "No logging line exists for %s.", converter);
set_param(line, "Name", loggingName);
set_param(handles.Outport(1), "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", "DataLoggingName", loggingName);
end

function removeWiring(item)
converter = subsystemParent(item.jointPath) + "/Wheel Wrench Converter " + item.loggingName;
terminator = subsystemParent(item.jointPath) + "/Wheel Wrench Terminator " + item.loggingName;
if getSimulinkBlockHandle(converter) ~= -1; delete_block(converter); end
if getSimulinkBlockHandle(terminator) ~= -1; delete_block(terminator); end
set_param(item.jointPath, item.sensingParameter, "off");
end

function [logging, verification] = verifyInvariance(model, tolerance)
% A/A distinguishes nondeterminism from sensing impact before on/off is used.
offA = runRegressionPass(model, false);
offB = runRegressionPass(model, false);
offRepeatability = compareSnapshots(offA, offB, tolerance, "sensing-off A/B");
on = runRegressionPass(model, true);
onOff = compareSnapshots(offA, on, tolerance, "sensing-off/on");
% Finish in the baseline configuration even after a passing sensing-on run.
resetSimulationSession(model);
logging = struct("contractVersion", "09-01-G1", ...
    "signals", signalContract(wheel_interface_wrench_contract()), ...
    "rawFrame", "BaseFrame", "nativeTimestampRetained", true);
verification = struct("mode", "verify_invariance", "observationOnly", true, ...
    "invariantUnchanged", true, "behaviorInvariant", onOff.passed, ...
    "tolerance", tolerance, "sensingOffA", offA, "sensingOffB", offB, ...
    "sensingOn", on, "offRepeatability", offRepeatability, ...
    "onOffInvariant", onOff, "wallClockPolicy", ...
    "nmpcCpuTime is finite/bounded telemetry only and is excluded from numerical-state equivalence", ...
    "modelDirtyAfter", "off");
end

function snapshot = runRegressionPass(model, enableSensing)
resetSimulationSession(model);
if enableSensing
    configure_wheel_interface_wrench_logging("Mode", "enable", "Model", model);
end
in = Simulink.SimulationInput(char(model));
in = in.setModelParameter("SimulationMode", "accelerator", "StopTime", "0.10", ...
    "ReturnWorkspaceOutputs", "on", "SignalLogging", "on", ...
    "SaveOutput", "off", "CaptureErrors", "on");
out = sim(in);
if strlength(string(out.ErrorMessage)) > 0
    error("configure_wheel_interface_wrench_logging:Simulation", "%s", out.ErrorMessage);
end
snapshot = loggedSignals(out.logsout, enableSensing);
if enableSensing
    configure_wheel_interface_wrench_logging("Mode", "disable", "Model", model);
end
end

function resetSimulationSession(model)
if bdIsLoaded(model); close_system(model, 0); end
evalin("base", "startup");
evalin("base", "clear spatial_two_leg_qp_core coupled_two_leg_qp_core coupled_two_leg_qp_signal full_base_nmpc_command");
load_system(model);
configure_symmetric_two_leg_simulink(false);
suppress_scope_windows(model);
set_param(model, "FastRestart", "off", "Dirty", "off");
end

function snapshot = loggedSignals(logs, sensingEnabled)
names = string(logs.getElementNames);
allowed = ["leftTotalForce", "leftTotalTorque", "rightTotalForce", "rightTotalTorque"];
snapshot = struct("sensingEnabled", sensingEnabled, "signals", ...
    struct("name", {}, "time", {}, "data", {}, "observationOnly", {}));
for i = 1:numel(names)
    name = names(i);
    element = logs.get(char(name));
    time = double(element.Values.Time(:));
    data = double(squeeze(element.Values.Data));
    if isvector(data); data = data(:); end
    if size(data, 1) ~= numel(time) && size(data, 2) == numel(time); data = data.'; end
    assert(size(data, 1) == numel(time) && all(isfinite(time)) && ...
        all(isfinite(data(:))), "configure_wheel_interface_wrench_logging:InvalidLog", ...
        "Log %s is nonfinite or has a timestamp/data mismatch.", name);
    snapshot.signals(end + 1) = struct("name", name, "time", time, ...
        "data", data, "observationOnly", any(name == allowed)); %#ok<AGROW>
end
end

function result = compareSnapshots(a, b, tolerance, label)
result = struct("label", label, "passed", false, "tolerance", tolerance, ...
    "comparedChannels", strings(0, 1), "wallClockChannels", strings(0, 1));
aNames = string({a.signals.name}); bNames = string({b.signals.name});
if ~isequal(aNames(~[a.signals.observationOnly]), bNames(~[b.signals.observationOnly]))
    error("configure_wheel_interface_wrench_logging:InvariantSignalSet", ...
        "%s changed the controller/plant signal set.", label);
end
for i = 1:numel(a.signals)
    left = a.signals(i);
    if left.observationOnly; continue; end
    right = b.signals(find(bNames == left.name, 1));
    if isWallClockSignal(left.name)
        assert(all(isfinite(left.data(:))) && all(isfinite(right.data(:))) ...
            && max(abs([left.data(:); right.data(:)])) < 60, ...
            "configure_wheel_interface_wrench_logging:WallClockTelemetry", ...
            "%s is not finite/bounded in %s.", left.name, label);
        result.wallClockChannels(end + 1, 1) = left.name; %#ok<AGROW>
        continue;
    end
    if ~isequal(size(left.data), size(right.data)) || numel(left.time) ~= numel(right.time)
        error("configure_wheel_interface_wrench_logging:InvariantMismatch", ...
            "%s changed %s sample structure.", label, left.name);
    end
    for column = 1:size(left.data, 2)
        if isWallClockColumn(left.name, column)
            assert(all(isfinite([left.data(:, column); right.data(:, column)])) ...
                && max(abs([left.data(:, column); right.data(:, column)])) < 1, ...
                "configure_wheel_interface_wrench_logging:WallClockTelemetry", ...
                "%s column %d is not finite/bounded in %s.", ...
                left.name, column, label);
            result.wallClockChannels(end + 1, 1) = left.name + ...
                " column " + column + " (qpSolveTime)"; %#ok<AGROW>
            continue;
        end
        timeError = max(abs(left.time - right.time));
        valueError = max(abs(left.data(:, column) - right.data(:, column)));
        if timeError > tolerance || valueError > tolerance
            error("configure_wheel_interface_wrench_logging:InvariantMismatch", ...
                "%s differs at %s column %d: time %.17g, value %.17g (tolerance %.17g).", ...
                label, left.name, column, timeError, valueError, tolerance);
        end
        result.comparedChannels(end + 1, 1) = left.name + " column " + column; %#ok<AGROW>
    end
end
result.passed = true;
end

function value = isWallClockSignal(name)
value = contains(lower(string(name)), "cputime") || contains(lower(string(name)), "wallclock");
end

function value = isWallClockColumn(name, column)
% coupledQpSignal column 54 is the documented qpSolveTime diagnostic; it is
% wall-clock telemetry, not a controller/plant numerical state channel.
value = string(name) == "coupledQpSignal" && column == 54;
end

function parent = subsystemParent(blockPath)
[parent, ~, ~] = fileparts(blockPath);
end

function restoreDirty(model, wasDirty)
if bdIsLoaded(model) && wasDirty == "off"; set_param(model, "Dirty", "off"); end
end
