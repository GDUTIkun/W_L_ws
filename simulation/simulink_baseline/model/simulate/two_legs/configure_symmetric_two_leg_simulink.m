function configure_symmetric_two_leg_simulink(doSave)
%CONFIGURE_SYMMETRIC_TWO_LEG_SIMULINK Wire one floating-base QP to both legs.
%
% The upper controller produces one total body wrench. Both leg states,
% both rolling contacts, and both actuator vectors enter one coupled QP.

if nargin < 1 || isempty(doSave)
    doSave = true;
end

model = "source";
subsystem = model + "/PD_only";
load_system(model);

if evalin("base", "exist('base', 'var')") ~= 1
    startupFile = fullfile(fileparts(mfilename("fullpath")), "startup.m");
    evalin("base", "run('" + replace(startupFile, "'", "''") + "')");
end

% The deployed model already owns the NMPC and plant topology. For an
% existing coupled-QP interface, update only the append-only diagnostic
% width and new sink wiring; do not rebuild unrelated controller blocks.
if getSimulinkBlockHandle(subsystem + "/Coupled QP") ~= -1 ...
        && getSimulinkBlockHandle(subsystem + "/Coupled QP Split") ~= -1
    modelLocation = get_param(model, "Location");
    subsystemLocation = get_param(subsystem, "Location");
    updateCoupledDiagnosticsInterface(subsystem);
    set_param(model, "SimulationCommand", "update");
    if doSave
        % Preserve the model's saved open-view state; load_system alone would
        % otherwise create unrelated UI metadata diffs in the SLX archive.
        open_system(model);
        open_system(subsystem);
        set_param(model, "Location", modelLocation);
        set_param(subsystem, "Location", subsystemLocation);
        save_system(model, [], "OverwriteIfChangedOnDisk", true);
    end
    fprintf("Updated coupled two-leg QP diagnostics in %s.\n", model);
    return;
end

baseStateZoh = subsystem + "/Zero-Order" + newline + "Hold";
upperControllers = find_system(subsystem, "SearchDepth", 1, ...
    "BlockType", "MATLABFcn", "MATLABFcn", "floating_base_lqr_command");
if isempty(upperControllers)
    template = subsystem + "/Common Wheel State";
    if getSimulinkBlockHandle(template) == -1
        error("configure_symmetric_two_leg_simulink:BlockLookup", ...
            "Cannot rebuild the floating-base LQR block without a MATLABFcn template.");
    end
    upperController = add_block(template, subsystem + "/Base LQR Command", ...
        "MATLABFcn", "floating_base_lqr_command", ...
        "OutputDimensions", "3", "SampleTime", "base.Ts", ...
        "Position", [550, 400, 690, 440]);
elseif numel(upperControllers) == 1
    upperController = string(upperControllers{1});
else
    error("configure_symmetric_two_leg_simulink:BlockLookup", ...
        "Expected at most one floating-base LQR block, found %d.", ...
        numel(upperControllers));
end

names = [
    "Per-Leg Wrench"
    "Per-Leg Wrench Split"
    "Common Wheel State Input"
    "Common Wheel State"
    "Common Wheel Position LQR"
    "Left QP Input"
    "Right QP Input"
    "Left QP Input ZOH"
    "Right QP Input ZOH"
    "Left QP"
    "Right QP"
    "Left QP Split"
    "Right QP Split"
    "Left Torque Split"
    "Right Torque Split"
    "Left Slack Terminator"
    "Left Feasible Terminator"
    "Left Slack Norm Terminator"
    "Left QP Status Terminator"
    "Left Contact Force Terminator"
    "Right Slack Terminator"
    "Right Feasible Terminator"
    "Right Slack Norm Terminator"
    "Right QP Status Terminator"
    "Right Contact Force Terminator"
    "Coupled QP Input"
    "Coupled QP Input ZOH"
    "Coupled QP"
    "Coupled QP Split"
    "Coupled Slack Terminator"
    "Coupled Feasible Terminator"
    "Coupled Slack Norm Terminator"
    "Coupled QP Status Terminator"
    "Coupled Left Contact Force Terminator"
    "Coupled Right Contact Force Terminator"
    "Coupled Exit Flag Terminator"
    "Coupled Dynamics Residual Terminator"
    "Coupled Contact Residual Terminator"
    "Coupled Wrench Residual Terminator"
    "Coupled Base Qdd Terminator"
    "Coupled Differential Qdd Terminator"
    "Coupled Differential Qdd Command Terminator"
    "Coupled Differential Torque Terminator"
    "Coupled Differential Contact Force Terminator"
    "Coupled Friction Margin Terminator"
    "Coupled Torque Margin Terminator"
    "Coupled Differential Wheel Position Terminator"
    "Coupled Differential Wheel Velocity Terminator"
    "Coupled Differential Wheel Acceleration Terminator"
    "Coupled Differential Wheel Command Terminator"
    "Coupled Canonical Drift Terminator"
    "Coupled Canonical Drift Rate Terminator"
    "Coupled Drift Correction Requested Terminator"
    "Coupled Drift Correction Applied Terminator"
    "Coupled Differential Input Nominal Terminator"
    "Coupled Differential Input Final Terminator"
    "Coupled Differential Input Realized Terminator"
    "Coupled Drift Amplitude Saturated Terminator"
    "Coupled Drift Rate Limited Terminator"
    "Coupled Drift Fail Safe Terminator"
    "Coupled Drift Reset Terminator"
    "Coupled uDiff Scalar Requested Terminator"
    "Coupled uDiff Scalar Applied Terminator"
    "Coupled uDiff Scalar Final Terminator"
    "Coupled uDiff Scalar QP Feasible Terminator"
    "Coupled uDiff Projected Requested Terminator"
    "Coupled uDiff Projected Applied Terminator"
    "Coupled uDiff Projected Final Terminator"
    "Coupled uDiff Projected QP Feasible Terminator"
    "Coupled uDiff Residual RMS Requested Terminator"
    "Coupled uDiff Residual RMS Applied Terminator"
    "Coupled uDiff Residual RMS Final Terminator"
    "Coupled uDiff Residual RMS QP Feasible Terminator"
    "Coupled QP Feasible Controller Side Terminator"
    "Coupled Plant Wrench Unavailable Terminator"
    "Symmetry Diagnostics"
    "Symmetry Diagnostics Terminator"
    "NMPC State Split"
    "NMPC Reference Input"
    "NMPC Reference"
    "NMPC Reference Split"
    "NMPC Solver"
    "NMPC Command Mux"
    "NMPC Command Guard"
    "NMPC Guard Split"
    "NMPC Fallback Terminator"
    "NMPC Previous Input"
    "NMPC Fault Terminator"
    "Full NMPC State Input"
    "Full NMPC State"
    "NMPC Planar State Split"
    "Base Pitch from Quaternion"
    "Base Roll from Quaternion"
    "Base Yaw from Quaternion"
    "Base Roll Yaw State"
    "Base Angular Velocity Split"
    "Base Roll Rate Terminator"
    "Base Yaw Rate Terminator"
    "Base Lateral State"
    "Base Lateral State Terminator"
];
for i = 1:numel(names)
    deleteBlockIfPresent(subsystem + "/" + names(i));
end

set_param(baseStateZoh, "SampleTime", "base.Ts");
set_param(upperController, "OutputDimensions", "3", ...
    "SampleTime", "base.Ts");

[rollYawState, lateralState] = wireSixDofPlanarState(subsystem);
set_param(block(subsystem, "Rigid", "Transform"), ...
    "TranslationStandardOffset", ...
    "base.simscapeWorldYOffset - ctrl.commonModeContactPreload");
set_param(block(subsystem, "Spatial", "Contact Force"), ...
    "NormalStiffness", "ctrl.commonModeContactStiffness/2", ...
    "NormalDamping", "ctrl.commonModeContactDamping/2");
set_param(block(subsystem, "Spatial", "Contact Force1"), ...
    "NormalStiffness", "ctrl.commonModeContactStiffness/2", ...
    "NormalDamping", "ctrl.commonModeContactDamping/2");

% Keep the Simscape plant and the damping term used by the coupled QP
% consistent.  The strict common-mode plant uses twice this value on its
% single equivalent leg; the full plant applies one share to each leg.
physicalJoints = [
    subsystem + "/Left Revolute Joint3"
    subsystem + "/Revolute Joint4"
    subsystem + "/Revolute Joint5"
    subsystem + "/Right Revolute Joint"
    subsystem + "/Revolute Joint1"
    subsystem + "/Revolute Joint2"
];
for idx = 1:numel(physicalJoints)
    dampingIndex = mod(idx - 1, 3) + 1;
    set_param(physicalJoints(idx), ...
        "DampingCoefficient", ...
        "ctrl.commonModeJointDamping(" + dampingIndex + ")", ...
        "DampingCoefficientUnits", "N*m/(rad/s)");
end

wheelStateInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Common Wheel State Input", ...
    "Inputs", "13", "Position", [720, 485, 725, 705]);
wheelState = add_block(upperController, subsystem + "/Common Wheel State", ...
    "MATLABFcn", "wheel_position_state_signal", ...
    "OutputDimensions", "10", "SampleTime", "base.Ts", ...
    "Position", [780, 565, 955, 605]);
wheelPlanner = add_block(upperController, ...
    subsystem + "/Common Wheel Position LQR", ...
    "MATLABFcn", "wheel_position_lqr_reference", ...
    "OutputDimensions", "4", "SampleTime", "base.Ts", ...
    "Position", [1010, 565, 1195, 605]);
fullStateInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Full NMPC State Input", "Inputs", "15", ...
    "Position", [1010, 710, 1015, 930]);
fullState = add_block(upperController, subsystem + "/Full NMPC State", ...
    "MATLABFcn", "full_base_nmpc_state_signal", ...
    "OutputDimensions", "18", "SampleTime", "base.Ts", ...
    "Position", [1070, 800, 1220, 840]);

coupledInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Coupled QP Input", "Inputs", "15", ...
    "Position", [1240, 310, 1245, 880]);
coupledZoh = add_block("simulink/Discrete/Zero-Order Hold", ...
    subsystem + "/Coupled QP Input ZOH", "SampleTime", "base.Ts", ...
    "Position", [1295, 575, 1345, 615]);
coupledQp = add_block(upperController, subsystem + "/Coupled QP", ...
    "MATLABFcn", "coupled_two_leg_qp_signal", ...
    "OutputDimensions", ...
    string(coupled_two_leg_qp_signal_contract().width), ...
    "SampleTime", "base.Ts", "Position", [1395, 575, 1545, 615]);
coupledSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Coupled QP Split", ...
    "Outputs", coupledDiagnosticDemuxOutputs(), ...
    "Position", [1585, 500, 1590, 690]);
leftTorqueSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Left Torque Split", "Outputs", "3", ...
    "Position", [1640, 475, 1645, 555]);
rightTorqueSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Right Torque Split", "Outputs", "3", ...
    "Position", [1640, 595, 1645, 675]);

coupledTerminators = addCoupledTerminators(subsystem, 1700, 700);

leftQ = [
    subsystem + "/Fcn1"
    block(subsystem, "PS-Simulink", "Converter13")
    block(subsystem, "PS-Simulink", "Converter16")
];
leftDq = [
    block(subsystem, "PS-Simulink", "Converter17")
    block(subsystem, "PS-Simulink", "Converter14")
    block(subsystem, "PS-Simulink", "Converter15")
];
rightQ = [
    subsystem + "/Fcn"
    block(subsystem, "PS-Simulink", "Converter1")
    block(subsystem, "PS-Simulink", "Converter2")
];
rightDq = [
    block(subsystem, "PS-Simulink", "Converter9")
    block(subsystem, "PS-Simulink", "Converter10")
    block(subsystem, "PS-Simulink", "Converter11")
];

symmetryDiagnostics = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Symmetry Diagnostics", "Inputs", "12", ...
    "Position", [1010, 930, 1015, 1110]);
symmetryTerminator = add_block("simulink/Sinks/Terminator", ...
    subsystem + "/Symmetry Diagnostics Terminator", ...
    "Position", [1190, 1005, 1210, 1025]);
connectVectorSources(subsystem, leftQ, symmetryDiagnostics, 1);
connectVectorSources(subsystem, leftDq, symmetryDiagnostics, 4);
connectVectorSources(subsystem, rightQ, symmetryDiagnostics, 7);
connectVectorSources(subsystem, rightDq, symmetryDiagnostics, 10);
connect(subsystem, symmetryDiagnostics, 1, symmetryTerminator, 1);

% Fcn/Fcn1 both convert the Simscape hip angle by pi/2.
connect(subsystem, baseStateZoh, 1, wheelStateInput, 1);
connectVectorSources(subsystem, leftQ, wheelStateInput, 2);
connectVectorSources(subsystem, leftDq, wheelStateInput, 5);
connectVectorSources(subsystem, rightQ, wheelStateInput, 8);
connectVectorSources(subsystem, rightDq, wheelStateInput, 11);
connect(subsystem, wheelStateInput, 1, wheelState, 1);
connect(subsystem, wheelState, 1, wheelPlanner, 1);
connect(subsystem, baseStateZoh, 1, fullStateInput, 1);
connect(subsystem, rollYawState, 1, fullStateInput, 2);
connect(subsystem, lateralState, 1, fullStateInput, 3);
connectVectorSources(subsystem, leftQ, fullStateInput, 4);
connectVectorSources(subsystem, leftDq, fullStateInput, 7);
connectVectorSources(subsystem, rightQ, fullStateInput, 10);
connectVectorSources(subsystem, rightDq, fullStateInput, 13);
connect(subsystem, fullStateInput, 1, fullState, 1);

wireCoupledInput(subsystem, coupledInput, fullState, leftQ, leftDq, ...
    rightQ, rightDq, upperController, wheelPlanner);
connect(subsystem, coupledInput, 1, coupledZoh, 1);
connect(subsystem, coupledZoh, 1, coupledQp, 1);
connect(subsystem, coupledQp, 1, coupledSplit, 1);
connect(subsystem, coupledSplit, 1, leftTorqueSplit, 1);
connect(subsystem, coupledSplit, 2, rightTorqueSplit, 1);
for i = 1:numel(coupledTerminators)
    connect(subsystem, coupledSplit, i + 2, coupledTerminators(i), 1);
end

leftActuation = [
    block(subsystem, "Simulink-PS", "Converter5")
    block(subsystem, "Simulink-PS", "Converter6")
    block(subsystem, "Simulink-PS", "Converter7")
];
rightActuation = [
    block(subsystem, "Simulink-PS", "Converter")
    block(subsystem, "Simulink-PS", "Converter1")
    block(subsystem, "Simulink-PS", "Converter4")
];
for i = 1:3
    connect(subsystem, leftTorqueSplit, i, leftActuation(i), 1);
    connect(subsystem, rightTorqueSplit, i, rightActuation(i), 1);
end

setZeroPulseAmplitude(model);
logOutput(coupledQp, "coupledQpSignal");
logOutput(wheelState, "commonWheelStateSignal");
logOutput(wheelPlanner, "commonWheelReference");
logOutput(fullState, "fullBaseNmpcStateSignal");
logOutput(upperController, "totalUpperCommand");
logOutput(symmetryDiagnostics, "symmetryLegState");

set_param(model, "SolverType", "Variable-step", "Solver", "ode15s", ...
    "MaxStep", "base.Ts", "RelTol", "1e-3", "AbsTol", "1e-4");
set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured coupled two-leg floating-base LQR-QP control in %s.\n", model);
end

function updateCoupledDiagnosticsInterface(parent)
coupledQp = parent + "/Coupled QP";
coupledSplit = parent + "/Coupled QP Split";
splitPosition = get_param(coupledSplit, "Position");
set_param(coupledQp, "OutputDimensions", ...
    string(coupled_two_leg_qp_signal_contract().width));
set_param(coupledSplit, "Outputs", coupledDiagnosticDemuxOutputs());
set_param(coupledSplit, "Position", splitPosition);

suffixes = allDiagnosticSuffixes();
for idx = 1:numel(suffixes)
    terminator = parent + "/Coupled " + suffixes(idx) + " Terminator";
    if getSimulinkBlockHandle(terminator) == -1
        add_block("simulink/Sinks/Terminator", terminator, ...
            "Position", [1700, 1435 + 35*(idx - 1), ...
            1720, 1455 + 35*(idx - 1)]);
    end
    connect(parent, coupledSplit, 23 + idx, terminator, 1);
end
end

function suffixes = allDiagnosticSuffixes()
suffixes = [ ...
    "Canonical Drift", "Canonical Drift Rate", ...
    "Drift Correction Requested", "Drift Correction Applied", ...
    "Differential Input Nominal", "Differential Input Final", ...
    "Differential Input Realized", "Drift Amplitude Saturated", ...
    "Drift Rate Limited", "Drift Fail Safe", "Drift Reset", ...
    "uDiff Scalar Requested", "uDiff Scalar Applied", ...
    "uDiff Scalar Final", "uDiff Scalar QP Feasible", ...
    "uDiff Projected Requested", "uDiff Projected Applied", ...
    "uDiff Projected Final", "uDiff Projected QP Feasible", ...
    "uDiff Residual RMS Requested", ...
    "uDiff Residual RMS Applied", "uDiff Residual RMS Final", ...
    "uDiff Residual RMS QP Feasible", ...
    "QP Feasible Controller Side", "Plant Wrench Unavailable", ...
    "WBC Task Sensitivity", "WBC Task Cost", "WBC Task Residual", ...
    "WBC Task Attribution Valid", "WBC Task Gradient Closure", ...
    "Rolling Fx Hierarchy Requested", ...
    "Rolling Fx Hierarchy Applied", ...
    "Rolling Fx Hierarchy Stage1 Feasible", ...
    "Rolling Fx Hierarchy Stage1 Acceleration", ...
    "Rolling Fx Hierarchy Lock Residual"];
end

function outputs = coupledDiagnosticDemuxOutputs()
[outputs, ~] = coupled_two_leg_qp_demux_outputs();
end

function [rollYawState, lateralState] = wireSixDofPlanarState(parent)
% Keep the existing planar controller state [x z pitch dx dz dpitch]
% while the Simscape plant uses a free 6-DoF base joint.
joint = parent + "/6-DOF Joint";
set_param(joint, ...
    "PxPositionTargetValue", "base.x0(1)", ...
    "PyPositionTargetValue", "base.x0(2)", ...
    "PzPositionTargetValue", "0", ...
    "PxVelocityTargetSpecify", "on", ...
    "PxVelocityTargetValue", "base.x0(4)", ...
    "PyVelocityTargetSpecify", "on", ...
    "PyVelocityTargetValue", "base.x0(5)", ...
    "PzVelocityTargetSpecify", "on", ...
    "PzVelocityTargetValue", "0", ...
    "SphPositionTargetRotationMethod", "Quaternion", ...
    "SphPositionTargetQuaternion", "base.initialQuaternion", ...
    "SphVelocityTargetSpecify", "on", ...
    "SphVelocityTargetValue", "base.initialAngularVelocity", ...
    "SphVelocityTargetInFollowerFrame", "off");
baseState = parent + "/Mux";
px = block(parent, "PS-Simulink", "Converter4");
vx = block(parent, "PS-Simulink", "Converter3");
py = block(parent, "PS-Simulink", "Converter6");
vy = block(parent, "PS-Simulink", "Converter7");
pz = block(parent, "PS-Simulink", "Converter8");
vz = block(parent, "PS-Simulink", "Converter5");
quaternion = block(parent, "PS-Simulink", "Converter18");
angularVelocity = block(parent, "PS-Simulink", "Converter19");

pitch = add_block("simulink/User-Defined Functions/Fcn", ...
    parent + "/Base Pitch from Quaternion", ...
    "Expr", ["atan2(2*(u(2)*u(3)+u(1)*u(4))," + ...
        "sqrt((1-2*(u(3)^2+u(4)^2))^2+" + ...
        "(2*(u(2)*u(4)-u(1)*u(3)))^2))"], ...
    "Position", [-245, 45, -85, 75]);
roll = add_block("simulink/User-Defined Functions/Fcn", ...
    parent + "/Base Roll from Quaternion", ...
    "Expr", "atan2(2*(u(1)*u(2)-u(3)*u(4)),1-2*(u(2)^2+u(4)^2))", ...
    "Position", [-245, 155, -85, 185]);
yaw = add_block("simulink/User-Defined Functions/Fcn", ...
    parent + "/Base Yaw from Quaternion", ...
    "Expr", "atan2(2*(u(1)*u(3)-u(4)*u(2)),1-2*(u(3)^2+u(4)^2))", ...
    "Position", [-245, 195, -85, 225]);
angularVelocitySplit = add_block("simulink/Signal Routing/Demux", ...
    parent + "/Base Angular Velocity Split", "Outputs", "3", ...
    "Position", [-245, 80, -240, 145]);
rollYawState = add_block("simulink/Signal Routing/Mux", ...
    parent + "/Base Roll Yaw State", "Inputs", "4", ...
    "Position", [-25, 155, -20, 245]);
lateralState = add_block("simulink/Signal Routing/Mux", ...
    parent + "/Base Lateral State", "Inputs", "2", ...
    "Position", [-245, -10, -240, 30]);
lateralStateTerminator = add_block("simulink/Sinks/Terminator", ...
    parent + "/Base Lateral State Terminator", ...
    "Position", [-190, 0, -170, 20]);

connect(parent, px, 1, baseState, 2);
connect(parent, py, 1, baseState, 3);
connect(parent, quaternion, 1, pitch, 1);
connect(parent, pitch, 1, baseState, 4);
connect(parent, vx, 1, baseState, 5);
connect(parent, vy, 1, baseState, 6);
connect(parent, angularVelocity, 1, angularVelocitySplit, 1);
connect(parent, angularVelocitySplit, 3, baseState, 7);
connect(parent, quaternion, 1, roll, 1);
connect(parent, quaternion, 1, yaw, 1);
connect(parent, roll, 1, rollYawState, 1);
connect(parent, yaw, 1, rollYawState, 2);
connect(parent, angularVelocitySplit, 1, rollYawState, 3);
connect(parent, angularVelocitySplit, 2, rollYawState, 4);
connect(parent, pz, 1, lateralState, 1);
connect(parent, vz, 1, lateralState, 2);
connect(parent, lateralState, 1, lateralStateTerminator, 1);

logOutput(quaternion, "baseQuaternion");
logOutput(angularVelocity, "baseAngularVelocity");
logOutput(lateralState, "baseLateralState");
logOutput(rollYawState, "baseRollYawState");
end

function wireCoupledInput(parent, inputMux, fullState, qLeft, dqLeft, ...
        qRight, dqRight, wrench, wheelReference)
connect(parent, fullState, 1, inputMux, 1);
connectVectorSources(parent, qLeft, inputMux, 2);
connectVectorSources(parent, dqLeft, inputMux, 5);
connectVectorSources(parent, qRight, inputMux, 8);
connectVectorSources(parent, dqRight, inputMux, 11);
connect(parent, wrench, 1, inputMux, 14);
connect(parent, wheelReference, 1, inputMux, 15);
end

function connectVectorSources(parent, sources, destination, firstPort)
for i = 1:numel(sources)
    connect(parent, sources(i), 1, destination, firstPort + i - 1);
end
end

function paths = addCoupledTerminators(parent, x, y)
suffixes = ["Slack", "Feasible", "Slack Norm", "QP Status", ...
    "Left Contact Force", "Right Contact Force", "Exit Flag", ...
    "Dynamics Residual", "Contact Residual", "Wrench Residual", ...
    "Base Qdd", "Differential Qdd", ...
    "Differential Qdd Command", "Differential Torque", ...
    "Differential Contact Force", "Friction Margin", "Torque Margin"];
suffixes = [suffixes, "Differential Wheel Position", ...
    "Differential Wheel Velocity", "Differential Wheel Acceleration", ...
    "Differential Wheel Command", "Canonical Drift", ...
    "Canonical Drift Rate", "Drift Correction Requested", ...
    "Drift Correction Applied", "Differential Input Nominal", ...
    "Differential Input Final", "Differential Input Realized", ...
    "Drift Amplitude Saturated", "Drift Rate Limited", ...
    "Drift Fail Safe", "Drift Reset"];
suffixes = [suffixes, "uDiff Scalar Requested", ...
    "uDiff Scalar Applied", "uDiff Scalar Final", ...
    "uDiff Scalar QP Feasible", ...
    "uDiff Projected Requested", "uDiff Projected Applied", ...
    "uDiff Projected Final", "uDiff Projected QP Feasible", ...
    "uDiff Residual RMS Requested", ...
    "uDiff Residual RMS Applied", "uDiff Residual RMS Final", ...
    "uDiff Residual RMS QP Feasible", ...
    "QP Feasible Controller Side", "Plant Wrench Unavailable"];
paths = strings(numel(suffixes), 1);
for i = 1:numel(suffixes)
    paths(i) = parent + "/Coupled " + suffixes(i) + " Terminator";
    add_block("simulink/Sinks/Terminator", paths(i), ...
        "Position", [x, y + 35*(i - 1), x + 20, y + 20 + 35*(i - 1)]);
end
end

function value = block(parent, prefix, suffix)
value = parent + "/" + prefix + newline + suffix;
end

function blockPath = findOne(parent, varargin)
matches = find_system(parent, "SearchDepth", 1, varargin{:});
matches = setdiff(string(matches), string(parent), "stable");
if numel(matches) ~= 1
    error("configure_symmetric_two_leg_simulink:BlockLookup", ...
        "Expected one matching block under %s, found %d.", parent, numel(matches));
end
blockPath = matches(1);
end

function deleteBlockIfPresent(blockPath)
if getSimulinkBlockHandle(blockPath) ~= -1
    delete_block(blockPath);
end
end

function connect(parent, source, sourcePort, destination, destinationPort)
sourceHandles = get_param(source, "PortHandles");
destinationHandles = get_param(destination, "PortHandles");
disconnectPort(destinationHandles.Inport(destinationPort));
add_line(parent, sourceHandles.Outport(sourcePort), ...
    destinationHandles.Inport(destinationPort), "autorouting", "on");
end

function disconnectPort(portHandle)
line = get_param(portHandle, "Line");
if isnumeric(line) && isscalar(line) && line ~= -1 && line ~= 0
    delete_line(line);
end
end

function setZeroPulseAmplitude(model)
blocks = find_system(model, "LookUnderMasks", "all", "FollowLinks", "on", ...
    "BlockType", "DiscretePulseGenerator");
for i = 1:numel(blocks)
    set_param(blocks{i}, "Amplitude", "0");
end
end

function logOutput(blockPath, name)
handles = get_param(blockPath, "PortHandles");
line = get_param(handles.Outport(1), "Line");
if line ~= -1
    set_param(line, "Name", name);
end
set_param(handles.Outport(1), "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", "DataLoggingName", name);
end
