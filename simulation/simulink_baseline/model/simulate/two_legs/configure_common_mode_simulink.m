function configure_common_mode_simulink(doSave)
%CONFIGURE_COMMON_MODE_SIMULINK Build the strict 6-DoF common-mode model.
%
% source.slx remains the full two-leg plant. source_common.slx keeps one
% center-plane equivalent leg whose mass, contact, and applied torque equal
% the sum of the two physical legs on the symmetric manifold.

if nargin < 1 || isempty(doSave)
    doSave = true;
end

sourceModel = "source";
model = "source_common";
modelDir = fileparts(mfilename("fullpath"));
targetFile = fullfile(modelDir, model + ".slx");

if bdIsLoaded(model)
    close_system(model, 0);
end
load_system(sourceModel);
configure_symmetric_two_leg_simulink(false);
save_system(sourceModel, targetFile, "OverwriteIfChangedOnDisk", true);
close_system(sourceModel, 0);
load_system(model);

subsystem = model + "/PD_only";
leftQ = [
    subsystem + "/Fcn1"
    named(subsystem, "PS-Simulink", "Converter13")
    named(subsystem, "PS-Simulink", "Converter16")
];
leftDq = [
    named(subsystem, "PS-Simulink", "Converter17")
    named(subsystem, "PS-Simulink", "Converter14")
    named(subsystem, "PS-Simulink", "Converter15")
];

replaceVectorInputs(subsystem, leftQ, ...
    subsystem + "/Common Wheel State Input", 8);
replaceVectorInputs(subsystem, leftDq, ...
    subsystem + "/Common Wheel State Input", 11);
replaceVectorInputs(subsystem, leftQ, subsystem + "/Coupled QP Input", 8);
replaceVectorInputs(subsystem, leftDq, subsystem + "/Coupled QP Input", 11);
replaceVectorInputs(subsystem, leftQ, ...
    subsystem + "/Symmetry Diagnostics", 7);
replaceVectorInputs(subsystem, leftDq, ...
    subsystem + "/Symmetry Diagnostics", 10);

deleteBlocks([
    named(subsystem, "Rigid", "Transform5")
    subsystem + "/Right Revolute Joint"
    named(subsystem, "Rigid", "Transform3")
    subsystem + "/Brick Solid1"
    named(subsystem, "Rigid", "Transform1")
    subsystem + "/Revolute Joint1"
    named(subsystem, "Rigid", "Transform2")
    subsystem + "/Brick Solid2"
    named(subsystem, "Rigid", "Transform4")
    subsystem + "/Revolute Joint2"
    subsystem + "/Cylindrical Solid"
    named(subsystem, "Spatial", "Contact Force")
    named(subsystem, "Simulink-PS", "Converter")
    named(subsystem, "Simulink-PS", "Converter1")
    named(subsystem, "Simulink-PS", "Converter4")
    named(subsystem, "PS-Simulink", "Converter")
    named(subsystem, "PS-Simulink", "Converter1")
    named(subsystem, "PS-Simulink", "Converter2")
    named(subsystem, "PS-Simulink", "Converter9")
    named(subsystem, "PS-Simulink", "Converter10")
    named(subsystem, "PS-Simulink", "Converter11")
    subsystem + "/Fcn"
    subsystem + "/Right Torque Split"
]);

set_param(named(subsystem, "Rigid", "Transform10"), ...
    "TranslationStandardOffset", "0");
set_param(named(subsystem, "Rigid", "Transform"), ...
    "TranslationStandardOffset", ...
    "base.simscapeWorldYOffset - ctrl.commonModeContactPreload");
set_param(subsystem + "/Brick Solid4", "Mass", "2.40");
set_param(subsystem + "/Brick Solid5", "Mass", "1.60");
set_param(subsystem + "/Cylindrical Solid1", "Mass", "0.70");
set_param(named(subsystem, "Spatial", "Contact Force1"), ...
    "NormalStiffness", "ctrl.commonModeContactStiffness", ...
    "NormalDamping", "ctrl.commonModeContactDamping");
commonJoints = [
    subsystem + "/Left Revolute Joint3"
    subsystem + "/Revolute Joint4"
    subsystem + "/Revolute Joint5"
];
for idx = 1:numel(commonJoints)
    set_param(commonJoints(idx), ...
        "DampingCoefficient", ...
        "2 * ctrl.commonModeJointDamping(" + idx + ")", ...
        "DampingCoefficientUnits", "N*m/(rad/s)");
end
set_param(subsystem + "/Coupled QP", ...
    "MATLABFcn", "common_mode_qp_signal", "OutputDimensions", "18");

sumBlock = subsystem + "/Common Total Torque";
deleteBlockIfPresent(sumBlock);
sumBlock = add_block("simulink/Math Operations/Add", sumBlock, ...
    "Inputs", "++", "Position", [1615, 500, 1650, 555]);
split = subsystem + "/Coupled QP Split";
leftSplit = subsystem + "/Left Torque Split";
disconnectInput(leftSplit, 1);
connect(subsystem, split, 1, sumBlock, 1);
connect(subsystem, split, 2, sumBlock, 2);
connect(subsystem, sumBlock, 1, leftSplit, 1);

setZeroPulseAmplitude(model);
configure_base_nmpc_simulink(false, model);
set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured strict common-mode NMPC-QP control in %s.\n", model);
end

function value = named(parent, prefix, suffix)
value = parent + "/" + prefix + newline + suffix;
end

function deleteBlocks(paths)
for i = 1:numel(paths)
    deleteBlockIfPresent(paths(i));
end
end

function deleteBlockIfPresent(path)
if getSimulinkBlockHandle(path) ~= -1
    delete_block(path);
end
end

function setZeroPulseAmplitude(model)
blocks = find_system(model, "LookUnderMasks", "all", "FollowLinks", "on", ...
    "BlockType", "DiscretePulseGenerator");
for i = 1:numel(blocks)
    set_param(blocks{i}, "Amplitude", "0");
end
end

function replaceVectorInputs(system, sources, destination, firstPort)
for i = 1:numel(sources)
    disconnectInput(destination, firstPort + i - 1);
    connect(system, sources(i), 1, destination, firstPort + i - 1);
end
end

function disconnectInput(blockPath, port)
handles = get_param(blockPath, "PortHandles");
line = get_param(handles.Inport(port), "Line");
if line ~= -1
    delete_line(line);
end
end

function connect(system, source, sourcePort, destination, destinationPort)
sourceHandles = get_param(source, "PortHandles");
destinationHandles = get_param(destination, "PortHandles");
add_line(system, sourceHandles.Outport(sourcePort), ...
    destinationHandles.Inport(destinationPort), "autorouting", "on");
end
