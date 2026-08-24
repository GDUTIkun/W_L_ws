function [wired, verification] = configure_contact_ground_truth_logging( ...
        manifest, varargin)
%CONFIGURE_CONTACT_GROUND_TRUTH_LOGGING Observation-only contact logging wiring.
% Applies the frozen G1 manifest as pure sensing/logging wiring on the two
% Spatial Contact Force blocks: each selected Sense* parameter is enabled and
% its physical-signal port is routed through a PS-Simulink Converter to a
% logging-only terminator line named by manifest.loggingName.
%
% The wiring is observation-only and in-memory: force-law parameters
% (manifest.loggingContract.parameterInvariant), geometry connections,
% solver, sample timing, and controller wiring are untouched, and the model
% is never saved (DoSave defaults to false). G1 audited that toggling these
% Sense* parameters only adds physical-signal output ports.
%
% Usage:
%   [wired, verification] = configure_contact_ground_truth_logging(manifest);
%   [wired, verification] = configure_contact_ground_truth_logging(manifest, ...
%       "Mode", "disable");        % restore baseline sensing-off state
%   Options: Mode ("enable"|"disable"), Model ("source"), DoSave (false).
% Returns:
%   wired        - struct mapping loggingName -> converter/terminator block paths
%   verification - struct with invariant parameter snapshots and wiring proof
%
% R2024b install notes (audited while wiring):
%  - simscape/Utilities ships empty here, so the PS-Simulink Converter is
%    copied from the converter blocks already embedded in source/PD_only.
%  - add_line cannot address the frozen manifest block names because they
%    embed char(10) and the name parser rejects it; each source block is
%    therefore temporarily renamed to a plain name for the add_line call and
%    restored immediately afterwards (lines are handle-based and survive).
%  - Simscape sense ports are addressed by their PortConnectivity type
%    (RConn2..RConn7), not by numeric index or the manifest port label.

parser = inputParser;
parser.addParameter("Mode", "enable", @(x) ismember(string(x), ["enable", "disable"]));
parser.addParameter("Model", "source", @(x) ischar(x) || isstring(x));
parser.addParameter("DoSave", false, @(x) islogical(x) && isscalar(x));
parser.parse(varargin{:});
mode = string(parser.Results.Mode);
model = string(parser.Results.Model);
doSave = parser.Results.DoSave;

assert(isstruct(manifest) && isfield(manifest, "selectedSignals") ...
    && isfield(manifest, "loggingContract"), ...
    "configure_contact_ground_truth_logging:ManifestSchemaMismatch", ...
    "A frozen contact_ground_truth_manifest artifact is required.");
if string(manifest.loggingContract.mode) ~= "observation_only"
    error("configure_contact_ground_truth_logging:ManifestSchemaMismatch", ...
        "Manifest logging mode is %s; expected observation_only.", ...
        manifest.loggingContract.mode);
end

load_system(model);
wasDirty = string(get_param(model, "Dirty"));
cleanup = onCleanup(@() restoreDirty(model, wasDirty));

blocks = unique(string({manifest.selectedSignals.sourceBlock}));
assert(numel(blocks) >= 2, ...
    "configure_contact_ground_truth_logging:BlockLookup", ...
    "Expected at least two Spatial Contact Force blocks, found %d.", numel(blocks));
for blockPath = blocks
    assert(getSimulinkBlockHandle(blockPath) ~= -1, ...
        "configure_contact_ground_truth_logging:BlockLookup", ...
        "Audited source block is missing: %s", blockPath);
end

snapshot = snapshotInvariants(manifest, blocks);
wired = struct();
if mode == "enable"
    % Optional sense ports are materialized by a diagram update after their
    % Sense* parameters are enabled.  The runner configures the model before
    % calling this function, so this update resolves the existing workspace
    % parameters without changing any force-law or controller setting.
    for item = manifest.selectedSignals
        blockPath = string(item.sourceBlock);
        sensing = string(get_param(blockPath, item.sensingParameter));
        if sensing ~= "on"
            set_param(blockPath, item.sensingParameter, "on");
        end
    end
    set_param(model, "SimulationCommand", "update");
    for item = manifest.selectedSignals
        blockPath = string(item.sourceBlock);
        converterPath = ensureConverter(item);
        terminatorPath = ensureTerminator(item);
        connectPhysicalSignal(blockPath, item, converterPath);
        ensureLoggedLine(converterPath, terminatorPath, item.loggingName);
        wired.(item.loggingName) = struct("converter", converterPath, ...
            "terminator", terminatorPath);
    end
else
    for item = manifest.selectedSignals
        removeWiring(item);
    end
end

after = snapshotInvariants(manifest, blocks);
verification = struct("mode", mode, "model", model, ...
    "observationOnly", true, "saved", doSave, ...
    "invariantBefore", snapshot, "invariantAfter", after, ...
    "invariantUnchanged", isequaln(snapshot, after), ...
    "wiredSignalCount", numel(fieldnames(wired)), ...
    "scopeFences", string(manifest.scopeFences));
if ~verification.invariantUnchanged
    error("configure_contact_ground_truth_logging:InvariantChanged", ...
        "Contact force-law parameters changed during observation-only wiring.");
end
if mode == "enable"
    fprintf("Enabled observation-only contact logging for %d manifest signals.\n", ...
        numel(manifest.selectedSignals));
else
    fprintf("Disabled observation-only contact logging; baseline sensing restored.\n");
end
clear cleanup
end

function snapshot = snapshotInvariants(manifest, blocks)
invariant = string(manifest.loggingContract.parameterInvariant);
snapshot = struct();
for blockPath = blocks
    for parameter = invariant
        snapshot.(matlab.lang.makeValidName( ...
            string(replace(blockPath, "/", "_")) + "_" + parameter)) = ...
            string(get_param(blockPath, parameter));
    end
end
end

function path = ensureConverter(item)
parent = subsystemParent(item.sourceBlock);
path = parent + "/Contact GT Converter " + item.loggingName;
if getSimulinkBlockHandle(path) ~= -1
    return;
end
unit = string(item.unit);
position = wiringPosition(item.sourceBlock, string(item.sourcePort));
template = "source/PD_only/PS-Simulink" + newline + "Converter";
add_block(char(template), char(path), ...
    "Unit", unit, ...
    "Position", [position(1) + 50, position(2) - 7, ...
    position(1) + 84, position(2) + 7]);
end

function path = ensureTerminator(item)
parent = subsystemParent(item.sourceBlock);
path = parent + "/Contact GT Terminator " + item.loggingName;
if getSimulinkBlockHandle(path) ~= -1
    return;
end
position = wiringPosition(item.sourceBlock, string(item.sourcePort));
add_block("simulink/Sinks/Terminator", char(path), ...
    "Position", [position(1) + 120, position(2) - 7, ...
    position(1) + 140, position(2) + 7]);
end

function position = wiringPosition(sourceBlock, portLabel)
blockPosition = get_param(sourceBlock, "Position");
port = physicalPortIndex(sourceBlock, portLabel);
pcs = get_param(sourceBlock, "PortConnectivity");
position = pcs(port).Position;
if isempty(position) || ~all(isfinite(position))
    position = [blockPosition(3) + 20, blockPosition(2)];
end
end

function connectPhysicalSignal(sourceBlock, item, converterPath)
% Physical signal ports are exposed through PortConnectivity as RConn2..RConn7
% entries (there is no Line field on this R2024b install, so connection state
% is checked through DstBlock).  Connect by physical-port handles: this avoids
% parsing the newline embedded in the audited Spatial Contact Force names.
index = physicalPortIndex(sourceBlock, string(item.sourcePort));
pcs = get_param(sourceBlock, "PortConnectivity");
if ~isempty(pcs(index).DstBlock)
    return; % already connected
end
sourceHandles = get_param(sourceBlock, "PortHandles");
converterHandles = get_param(converterPath, "PortHandles");
portType = string(pcs(index).Type);
portNumber = str2double(extractAfter(portType, "RConn"));
assert(isfinite(portNumber) && portNumber <= numel(sourceHandles.RConn), ...
    "configure_contact_ground_truth_logging:SourcePortLookup", ...
    "Source port %s is unavailable on %s.", item.sourcePort, sourceBlock);
assert(~isempty(converterHandles.LConn), ...
    "configure_contact_ground_truth_logging:ConverterLookup", ...
    "Converter %s has no physical input port.", converterPath);
add_line(char(subsystemParent(sourceBlock)), ...
    sourceHandles.RConn(portNumber), converterHandles.LConn(1));
connected = get_param(sourceBlock, "PortConnectivity");
assert(~isempty(connected(index).DstBlock), ...
    "configure_contact_ground_truth_logging:LineFailed", ...
    "Failed to connect %s/%s to %s.", sourceBlock, item.sourcePort, converterPath);
end

function index = physicalPortIndex(sourceBlock, portLabel)
pcs = get_param(sourceBlock, "PortConnectivity");
% RConn1 is the wheel conserving port; RConn2..RConn7 are the six sense
% ports, declared in the manifest order con, pen, fnm, ffrm, vn, vt.
physical = find(arrayfun(@(p) startsWith(string(p.Type), "RConn") ...
    && string(p.Type) ~= "RConn1", pcs));
if isempty(physical)
    error("configure_contact_ground_truth_logging:NoPhysicalPorts", ...
        "Block %s exposes no physical-signal output ports.", sourceBlock);
end
labels = ["con", "pen", "fnm", "ffrm", "vn", "vt"];
position = find(strcmp(labels, portLabel));
assert(~isempty(position), ...
    "configure_contact_ground_truth_logging:NoPhysicalPorts", ...
    "Unknown manifest source port label %s.", portLabel);
assert(numel(physical) == 6, ...
    "configure_contact_ground_truth_logging:NoPhysicalPorts", ...
    "Block %s exposes %d physical ports; cannot map manifest labels.", ...
    sourceBlock, numel(physical));
index = physical(position);
end

function ensureLoggedLine(converterPath, terminatorPath, loggingName)
handles = get_param(converterPath, "PortHandles");
line = get_param(handles.Outport(1), "Line");
if isempty(line) || (isnumeric(line) && isscalar(line) && line == -1)
    % Converter output -> terminator input; both are plain Simulink ports.
    add_line(char(subsystemParent(converterPath)), ...
        char(string(get_param(converterPath, "Name")) + "/1"), ...
        char(string(get_param(terminatorPath, "Name")) + "/1"), ...
        "autorouting", "on");
    line = get_param(handles.Outport(1), "Line");
end
if isempty(line) || (isnumeric(line) && isscalar(line) && line == -1)
    error("configure_contact_ground_truth_logging:NoLine", ...
        "Converter %s output is not connected to its terminator.", converterPath);
end
set_param(line, "Name", loggingName);
set_param(handles.Outport(1), "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", "DataLoggingName", loggingName);
end

function removeWiring(item)
converterPath = subsystemParent(item.sourceBlock) ...
    + "/Contact GT Converter " + item.loggingName;
if getSimulinkBlockHandle(converterPath) ~= -1
    delete_block(converterPath);
end
terminatorPath = subsystemParent(item.sourceBlock) ...
    + "/Contact GT Terminator " + item.loggingName;
if getSimulinkBlockHandle(terminatorPath) ~= -1
    delete_block(terminatorPath);
end
set_param(item.sourceBlock, item.sensingParameter, "off");
end

function parent = subsystemParent(blockPath)
[parent, ~, ~] = fileparts(blockPath);
end

function restoreDirty(model, wasDirty)
if bdIsLoaded(model) && wasDirty == "off"
    set_param(model, "Dirty", "off");
end
end
