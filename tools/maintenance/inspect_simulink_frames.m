function manifest = inspect_simulink_frames(outputFile)
%INSPECT_SIMULINK_FRAMES Export a read-only Simulink/Simscape frame audit.
%
% This utility loads the frozen Phase-01 source model through its isolated
% entry point, records frame-related blocks, parameters, physical ports and
% immediate connections, then closes the model without saving it.
%
% Example:
%   matlab -batch "addpath('tools/maintenance'); ...
%       inspect_simulink_frames"

arguments
    outputFile (1, 1) string = ""
end

toolDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(toolDir));
baselineRoot = fullfile(repoRoot, "simulation", "simulink_baseline");
modelFile = fullfile(baselineRoot, "model", "simulate", ...
    "two_legs", "source.slx");
if outputFile == ""
    outputFile = fullfile(repoRoot, "docs", "workflow", "phases", ...
        "02-coordinate-interface-contract", "evidence", ...
        "simulink_frame_manifest.json");
end

addpath(baselineRoot, "-begin");
context = open_proformance_test(false);
model = string(context.model);
cleanup = onCleanup(@() closeWithoutSaving(model));

dirtyBefore = string(get_param(model, "Dirty"));
blocks = find_system(model, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "Type", "Block");

entries = repmat(emptyEntry(), 0, 1);
for index = 1:numel(blocks)
    blockPath = string(blocks{index});
    entry = inspectBlock(blockPath);
    if entry.include
        entries(end + 1, 1) = entry; %#ok<AGROW>
    end
end

dirtyAfter = string(get_param(model, "Dirty"));
if dirtyAfter ~= dirtyBefore
    error("inspect_simulink_frames:ModelModified", ...
        "Read-only inspection changed model Dirty from %s to %s.", ...
        dirtyBefore, dirtyAfter);
end

manifest = struct();
manifest.schemaVersion = "simulink-frame-manifest/1.0.0";
manifest.generatedUtc = string(datetime("now", "TimeZone", "UTC", ...
    "Format", "yyyy-MM-dd'T'HH:mm:ss'Z'"));
manifest.matlabRelease = string(version("-release"));
manifest.matlabVersion = string(version);
manifest.repoRoot = string(repoRoot);
manifest.modelFile = string(modelFile);
manifest.modelName = model;
manifest.modelDirtyBefore = dirtyBefore;
manifest.modelDirtyAfter = dirtyAfter;
manifest.totalBlocks = numel(blocks);
manifest.includedBlocks = numel(entries);
manifest.selectionPolicy = [ ...
    "blocks with Simscape physical ports", ...
    "blocks whose type/mask/reference/path names frame, transform, sensor, joint, pose, quaternion, contact, or mechanism semantics"];
manifest.blocks = entries;

outputDir = fileparts(outputFile);
if ~isfolder(outputDir)
    mkdir(outputDir);
end
encoded = jsonencode(manifest, PrettyPrint=true);
fileId = fopen(outputFile, "w", "n", "UTF-8");
if fileId < 0
    error("inspect_simulink_frames:CannotOpenOutput", ...
        "Cannot write %s.", outputFile);
end
fileCleanup = onCleanup(@() fclose(fileId));
fprintf(fileId, "%s\n", encoded);
clear fileCleanup

fprintf("Frame manifest: %s\n", outputFile);
fprintf("Model blocks: %d total, %d selected; Dirty %s -> %s\n", ...
    manifest.totalBlocks, manifest.includedBlocks, ...
    dirtyBefore, dirtyAfter);
clear cleanup
end

function entry = inspectBlock(blockPath)
entry = emptyEntry();
entry.path = blockPath;
entry.sid = safeString(@() Simulink.ID.getSID(blockPath));
entry.blockType = safeString(@() get_param(blockPath, "BlockType"));
entry.maskType = safeString(@() get_param(blockPath, "MaskType"));
entry.referenceBlock = safeString(@() get_param(blockPath, "ReferenceBlock"));
entry.linkStatus = safeString(@() get_param(blockPath, "LinkStatus"));
entry.connectivity = inspectConnectivity(blockPath);

portHandles = get_param(blockPath, "PortHandles");
entry.ports = inspectPorts(portHandles);
hasPhysicalPorts = any([entry.ports.physical]);

searchText = lower(strjoin([entry.path, entry.blockType, ...
    entry.maskType, entry.referenceBlock], " "));
keywords = ["frame", "transform", "sensor", "joint", "pose", ...
    "quaternion", "rotation", "translation", "mechanism", ...
    "six-dof", "contact force", "simscape", "multibody"];
entry.include = hasPhysicalPorts || any(contains(searchText, keywords));
if ~entry.include
    return;
end

if hasPhysicalPorts
    entry.selectionReasons(end + 1, 1) = "physical-port";
end
matchedKeywords = keywords(contains(searchText, keywords));
entry.selectionReasons = [entry.selectionReasons; ...
    "keyword:" + matchedKeywords(:)];
entry.parameters = inspectDialogParameters(blockPath);
end

function connectivity = inspectConnectivity(blockPath)
connectivity = repmat(struct( ...
    "type", "", ...
    "position", "", ...
    "neighbors", strings(0, 1)), 0, 1);
raw = get_param(blockPath, "PortConnectivity");
for index = 1:numel(raw)
    handles = [raw(index).SrcBlock(:); raw(index).DstBlock(:)];
    handles = unique(handles(handles > 0));
    neighbors = strings(0, 1);
    for handleIndex = 1:numel(handles)
        neighbors(end + 1, 1) = safeString( ... %#ok<AGROW>
            @() getfullname(handles(handleIndex)));
    end
    connectivity(end + 1, 1) = struct( ... %#ok<AGROW>
        "type", string(raw(index).Type), ...
        "position", string(mat2str(raw(index).Position)), ...
        "neighbors", neighbors);
end
end

function parameters = inspectDialogParameters(blockPath)
parameters = repmat(struct("name", "", "value", ""), 0, 1);
dialogParameters = get_param(blockPath, "DialogParameters");
if isempty(dialogParameters)
    return;
end
names = fieldnames(dialogParameters);
for index = 1:numel(names)
    name = string(names{index});
    value = safeString(@() get_param(blockPath, name));
    parameters(end + 1, 1) = struct( ... %#ok<AGROW>
        "name", name, "value", value);
end
end

function ports = inspectPorts(portHandles)
ports = repmat(struct( ...
    "category", "", ...
    "index", 0, ...
    "name", "", ...
    "portType", "", ...
    "physical", false, ...
    "connected", false, ...
    "peers", strings(0, 1)), 0, 1);
categories = fieldnames(portHandles);
for categoryIndex = 1:numel(categories)
    category = string(categories{categoryIndex});
    handles = portHandles.(categories{categoryIndex});
    handles = handles(:);
    for portIndex = 1:numel(handles)
        handle = handles(portIndex);
        if handle <= 0
            continue;
        end
        peers = connectedPeers(handle);
        ports(end + 1, 1) = struct( ... %#ok<AGROW>
            "category", category, ...
            "index", portIndex, ...
            "name", safeString(@() get_param(handle, "Name")), ...
            "portType", safeString(@() get_param(handle, "PortType")), ...
            "physical", any(category == ["LConn", "RConn"]), ...
            "connected", ~isempty(peers), ...
            "peers", peers);
    end
end
end

function peers = connectedPeers(portHandle)
peers = strings(0, 1);
lineHandle = get_param(portHandle, "Line");
if isempty(lineHandle) || all(lineHandle < 0)
    return;
end
lineHandle = lineHandle(1);
candidateHandles = [];
for parameter = ["SrcPortHandle", "DstPortHandle"]
    try
        values = get_param(lineHandle, parameter);
        candidateHandles = [candidateHandles; values(:)]; %#ok<AGROW>
    catch
        % Some physical connection types do not expose directed endpoints.
    end
end
candidateHandles = unique(candidateHandles(candidateHandles > 0));
candidateHandles(candidateHandles == portHandle) = [];
for index = 1:numel(candidateHandles)
    handle = candidateHandles(index);
    parent = safeString(@() get_param(handle, "Parent"));
    portNumber = safeString(@() get_param(handle, "PortNumber"));
    portType = safeString(@() get_param(handle, "PortType"));
    peers(end + 1, 1) = parent + " [" + portType + ... %#ok<AGROW>
        " " + portNumber + "]";
end
end

function value = safeString(getter)
try
    raw = getter();
    value = textValue(raw);
catch exception
    value = "<unavailable: " + string(exception.identifier) + ">";
end
end

function value = textValue(raw)
if isstring(raw)
    value = join(raw(:).', " | ");
elseif ischar(raw)
    value = string(raw);
elseif isnumeric(raw) || islogical(raw)
    value = string(mat2str(raw));
elseif iscell(raw)
    converted = strings(size(raw));
    for index = 1:numel(raw)
        converted(index) = textValue(raw{index});
    end
    value = join(converted(:).', " | ");
else
    value = "<" + string(class(raw)) + ">";
end
end

function entry = emptyEntry()
entry = struct( ...
    "include", false, ...
    "selectionReasons", strings(0, 1), ...
    "path", "", ...
    "sid", "", ...
    "blockType", "", ...
    "maskType", "", ...
    "referenceBlock", "", ...
    "linkStatus", "", ...
    "connectivity", repmat(struct("type", "", "position", "", ...
        "neighbors", strings(0, 1)), 0, 1), ...
    "parameters", repmat(struct("name", "", "value", ""), 0, 1), ...
    "ports", repmat(struct("category", "", "index", 0, ...
        "name", "", "portType", "", ...
        "physical", false, "connected", false, ...
        "peers", strings(0, 1)), 0, 1));
end

function closeWithoutSaving(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end
