function configure_base_nmpc_simulink(doSave, modelName)
%CONFIGURE_BASE_NMPC_SIMULINK Wire direct NMPC into the coupled lower QP.

if nargin < 1 || isempty(doSave)
    doSave = true;
end
if nargin < 2 || isempty(modelName)
    modelName = "source_common";
end

simulateDir = fileparts(mfilename("fullpath"));
if evalin("base", "exist('baseNmpc', 'var')") ~= 1
    evalin("base", "run(" + quoted(fullfile(simulateDir, "startup.m")) + ");");
end
model = string(modelName);
isFull = model == "source";
if isFull
    baseNmpc = evalin("base", "fullBaseNmpc");
    configVariable = "fullBaseNmpc";
else
    baseNmpc = evalin("base", "baseNmpc");
    configVariable = "baseNmpc";
end
blockFile = fullfile(baseNmpc.generatedDir, ...
    baseNmpc.solverName + "_ocp_solver_simulink_block.slx");
if ~isfile(blockFile)
    error("configure_base_nmpc_simulink:MissingGeneratedBlock", ...
        "Build the solver first: build_base_nmpc_solver(true).");
end

subsystem = model + "/PD_only";
load_system(model);
load_system(blockFile);
[~, generatedModel] = fileparts(blockFile);
cleanupGeneratedModel = onCleanup(@() close_system(generatedModel, 0));

wheelStateBlock = subsystem + "/Common Wheel State";
wheelPlanner = subsystem + "/Common Wheel Position LQR";
coupledInput = subsystem + "/Coupled QP Input";
templateBlock = wheelStateBlock;

generatedBlocks = find_system(generatedModel, "SearchDepth", 1, ...
    "BlockType", "S-Function");
if numel(generatedBlocks) ~= 1
    error("configure_base_nmpc_simulink:GeneratedBlock", ...
        "Expected exactly one generated S-Function block.");
end

names = [
    "NMPC State Split"
    "NMPC Reference Input"
    "NMPC Reference"
    "NMPC Reference Split"
    "NMPC Solver"
    "NMPC OCP State"
    "NMPC Previous Input"
    "NMPC Command Mux"
    "NMPC Command Guard"
    "NMPC Guard Split"
    "NMPC Fallback Terminator"
    "NMPC Fault Terminator"
    "NMPC Planar State Split"
    "NMPC Planar State Terminator"
];
for i = 1:numel(names)
    deleteBlockIfPresent(subsystem + "/" + names(i));
end
disconnectInput(coupledInput, 14);

stateSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC State Split", ...
    "Outputs", sprintf("[1 %d 1]", size(baseNmpc.model.A, 1)), ...
    "Position", [1235, 535, 1240, 635]);
if isFull
    nmpcStateSource = subsystem + "/Full NMPC State";
    planarStateSplit = add_block("simulink/Signal Routing/Demux", ...
        subsystem + "/NMPC Planar State Split", "Outputs", "[1 8 1]", ...
        "Position", [1175, 650, 1180, 735]);
    planarStateTerminator = add_block("simulink/Sinks/Terminator", ...
        subsystem + "/NMPC Planar State Terminator", ...
        "Position", [1220, 690, 1240, 710]);
else
    nmpcStateSource = wheelStateBlock;
end
referenceInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Reference Input", ...
    "Inputs", "3", "Position", [1285, 500, 1290, 620]);
referenceBlock = add_block(templateBlock, subsystem + "/NMPC Reference", ...
    "Position", [1340, 535, 1495, 575]);
referenceFunction = "base_nmpc_reference";
if isFull
    referenceFunction = "full_base_nmpc_reference";
end
set_param(referenceBlock, "MATLABFcn", referenceFunction, ...
    "OutputDimensions", string(baseNmpc.referenceSize), ...
    "SampleTime", configVariable + ".Ts");
referenceSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Reference Split", ...
    "Outputs", sprintf("[%d %d %d]", ...
        size(baseNmpc.model.A, 1) + 2*size(baseNmpc.model.B, 2), ...
        (size(baseNmpc.model.A, 1) + 2*size(baseNmpc.model.B, 2)) ...
        *(baseNmpc.N - 1), size(baseNmpc.model.A, 1)), ...
    "Position", [1545, 505, 1550, 595]);
solverBlock = add_block(generatedBlocks{1}, subsystem + "/NMPC Solver", ...
    "Position", [1605, 480, 1825, 670]);
previousInput = add_block("simulink/Discrete/Unit Delay", ...
    subsystem + "/NMPC Previous Input", ...
    "InitialCondition", configVariable + ".model.uEq", ...
    "SampleTime", configVariable + ".Ts", ...
    "Position", [1855, 650, 1905, 690]);
exactDeltaInput = isfield(baseNmpc, "incrementCostMode") ...
    && string(baseNmpc.incrementCostMode) == "state_memory";
if exactDeltaInput
    ocpState = add_block("simulink/Signal Routing/Mux", ...
        subsystem + "/NMPC OCP State", ...
        "Inputs", "2", "Position", [1545, 625, 1550, 700]);
end
commandMux = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Command Mux", ...
    "Inputs", "4", "Position", [1875, 495, 1880, 625]);
guardBlock = add_block(templateBlock, subsystem + "/NMPC Command Guard", ...
    "Position", [1930, 535, 2090, 575]);
guardFunction = "base_nmpc_command";
if isFull
    guardFunction = "full_base_nmpc_command";
end
set_param(guardBlock, "MATLABFcn", guardFunction, ...
    "OutputDimensions", string(size(baseNmpc.model.B, 2) + 1), ...
    "SampleTime", configVariable + ".Ts");
guardSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Guard Split", ...
    "Outputs", sprintf("[%d 1]", size(baseNmpc.model.B, 2)), ...
    "Position", [2140, 520, 2145, 595]);
faultTerminator = add_block("simulink/Sinks/Terminator", ...
    subsystem + "/NMPC Fault Terminator", ...
    "Position", [2210, 585, 2230, 605]);

connect(subsystem, nmpcStateSource, 1, stateSplit, 1);
connect(subsystem, stateSplit, 1, referenceInput, 1);
connect(subsystem, wheelPlanner, 1, referenceInput, 2);
connect(subsystem, solverBlock, 1, previousInput, 1);
connect(subsystem, previousInput, 1, referenceInput, 3);
connect(subsystem, referenceInput, 1, referenceBlock, 1);
if exactDeltaInput
    connect(subsystem, stateSplit, 2, ocpState, 1);
    connect(subsystem, previousInput, 1, ocpState, 2);
    connect(subsystem, ocpState, 1, solverBlock, 1);
    connect(subsystem, ocpState, 1, solverBlock, 2);
else
    connect(subsystem, stateSplit, 2, solverBlock, 1);
    connect(subsystem, stateSplit, 2, solverBlock, 2);
end
connect(subsystem, referenceBlock, 1, referenceSplit, 1);
connect(subsystem, referenceSplit, 1, solverBlock, 3);
connect(subsystem, referenceSplit, 2, solverBlock, 4);
connect(subsystem, referenceSplit, 3, solverBlock, 5);
connect(subsystem, stateSplit, 1, commandMux, 1);
connect(subsystem, solverBlock, 1, commandMux, 2);
connect(subsystem, solverBlock, 2, commandMux, 3);
connect(subsystem, solverBlock, 3, commandMux, 4);
connect(subsystem, commandMux, 1, guardBlock, 1);
connect(subsystem, guardBlock, 1, guardSplit, 1);
connect(subsystem, guardSplit, 1, coupledInput, 14);
connect(subsystem, guardSplit, 2, faultTerminator, 1);

% Remove the six-state base LQR from the executable diagram. The wheel LQR
% remains because it is the online dynamic planner used by the paper.
lqrBlocks = find_system(subsystem, "SearchDepth", 1, ...
    "BlockType", "MATLABFcn", "MATLABFcn", "floating_base_lqr_command");
for i = 1:numel(lqrBlocks)
    delete_block(lqrBlocks{i});
end

logSignal(solverBlock, 1, "nmpcBodyWrench");
logSignal(solverBlock, 2, "nmpcStatus");
logSignal(solverBlock, 3, "nmpcCpuTime");
logSignal(guardSplit, 1, "totalUpperCommand");
logSignal(guardSplit, 2, "nmpcFault");
if isFull
    connect(subsystem, wheelStateBlock, 1, planarStateSplit, 1);
    connect(subsystem, planarStateSplit, 2, planarStateTerminator, 1);
    logSignal(planarStateSplit, 2, "baseWheelState");
    logSignal(stateSplit, 2, "baseNmpcState");
else
    logSignal(stateSplit, 2, "baseWheelState");
end
logSignal(wheelPlanner, 1, "wheelPositionLqrReference");

set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured direct upper-layer NMPC in %s.\n", model);
end

function value = quoted(pathValue)
value = "'" + replace(string(pathValue), "'", "''") + "'";
end

function deleteBlockIfPresent(block)
if getSimulinkBlockHandle(block) ~= -1
    delete_block(block);
end
end

function disconnectInput(block, port)
handles = get_param(block, "PortHandles");
line = get_param(handles.Inport(port), "Line");
if line ~= -1
    delete_line(line);
end
end

function connect(parent, srcBlock, srcPort, dstBlock, dstPort)
srcHandles = get_param(srcBlock, "PortHandles");
dstHandles = get_param(dstBlock, "PortHandles");
disconnectPort(dstHandles.Inport(dstPort));
add_line(parent, srcHandles.Outport(srcPort), dstHandles.Inport(dstPort), ...
    "autorouting", "on");
end

function disconnectPort(portHandle)
line = get_param(portHandle, "Line");
if line ~= -1
    delete_line(line);
end
end

function logSignal(block, port, name)
handles = get_param(block, "PortHandles");
outport = handles.Outport(port);
line = get_param(outport, "Line");
if line == -1
    error("configure_base_nmpc_simulink:MissingLine", ...
        "Cannot log unconnected signal %s.", name);
end
set_param(line, "Name", name);
set_param(outport, "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", "DataLoggingName", name);
end
