function blocks = configure_step_terrain_simulink(model, doSave)
%CONFIGURE_STEP_TERRAIN_SIMULINK Add a parameterized step in memory.
% The accepted flat-ground model remains the source of truth. This helper
% adds one fixed Brick Solid plus one Spatial Contact Force per wheel. The
% added parameters reference the base-workspace struct `terrain`, allowing
% SimulationInput.setVariable to configure each case without saving source.

if nargin < 1 || isempty(model)
    model = "source";
end
if nargin < 2 || isempty(doSave)
    doSave = false;
end
model = string(model);
validateattributes(doSave, {'logical'}, {'scalar'});

if ~bdIsLoaded(model)
    load_system(model);
end
if evalin("base", "exist('terrain', 'var')") == 0
    assignin("base", "terrain", step_terrain_parameters(0));
end

subsystem = model + "/PD_only";
ground = subsystem + "/Brick Solid3";
leftWheel = subsystem + "/Cylindrical Solid1";
rightWheel = subsystem + "/Cylindrical Solid";
leftGroundContact = subsystem + "/Spatial" + newline + "Contact Force1";
rightGroundContact = subsystem + "/Spatial" + newline + "Contact Force";
world = subsystem + "/World Frame1";

blocks = struct();
blocks.solid = subsystem + "/TerrainStepSolid";
blocks.pose = subsystem + "/TerrainStepPose";
blocks.leftContact = subsystem + "/TerrainContactLeft";
blocks.rightContact = subsystem + "/TerrainContactRight";

if getSimulinkBlockHandle(blocks.solid) == -1
    add_block(ground, blocks.solid, ...
        "Position", [2460, 80, 2540, 140]);
end
if getSimulinkBlockHandle(blocks.pose) == -1
    transformTemplate = subsystem + "/Rigid" + newline + "Transform";
    add_block(transformTemplate, blocks.pose, ...
        "Position", [2290, 170, 2370, 230]);
end
if getSimulinkBlockHandle(blocks.leftContact) == -1
    add_block(leftGroundContact, blocks.leftContact, ...
        "Position", [2290, 20, 2380, 70]);
end
if getSimulinkBlockHandle(blocks.rightContact) == -1
    add_block(rightGroundContact, blocks.rightContact, ...
        "Position", [2290, 260, 2380, 310]);
end

set_param(blocks.solid, ...
    "BrickDimensions", "terrain.brickDimensions");
set_param(blocks.pose, ...
    "RotationMethod", "None", ...
    "TranslationMethod", "Cartesian", ...
    "TranslationCartesianOffset", "terrain.translation", ...
    "TranslationCartesianOffsetUnits", "m");

connectPhysical(world, "RConn", 1, blocks.pose, "LConn", 1);
connectPhysical(blocks.pose, "RConn", 1, blocks.solid, "RConn", 1);
connectPhysical(leftWheel, "LConn", 1, ...
    blocks.leftContact, "LConn", 1);
connectPhysical(blocks.leftContact, "RConn", 1, ...
    blocks.solid, "LConn", 1);
connectPhysical(rightWheel, "LConn", 1, ...
    blocks.rightContact, "LConn", 1);
connectPhysical(blocks.rightContact, "RConn", 1, ...
    blocks.solid, "LConn", 1);

set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
end

function connectPhysical(source, sourceType, sourceIndex, ...
        target, targetType, targetIndex)
sourceHandles = get_param(source, "PortHandles");
targetHandles = get_param(target, "PortHandles");
sourcePort = sourceHandles.(sourceType)(sourceIndex);
targetPort = targetHandles.(targetType)(targetIndex);

sourceConnectivity = get_param(source, "PortConnectivity");
targetHandle = get_param(target, "Handle");
alreadyConnected = false;
for k = 1:numel(sourceConnectivity)
    if string(sourceConnectivity(k).Type) ~= sourceType + sourceIndex
        continue
    end
    alreadyConnected = any(sourceConnectivity(k).DstBlock == targetHandle);
    break
end
if ~alreadyConnected
    add_line(char(subsystemParent(source)), sourcePort, targetPort);
end
end

function parent = subsystemParent(block)
parent = extractBefore(string(block), "/" + ...
    string(get_param(block, "Name")));
end
