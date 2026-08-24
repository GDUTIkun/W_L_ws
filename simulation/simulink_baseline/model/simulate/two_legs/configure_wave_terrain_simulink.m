function blocks = configure_wave_terrain_simulink(model, doSave)
%CONFIGURE_WAVE_TERRAIN_SIMULINK Add segmented wave terrain in memory.
% Each chord of the raised-cosine profile is represented by one rotated
% Brick Solid and one Spatial Contact Force per wheel. source.slx is saved
% only when doSave is explicitly true.

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
if evalin("base", "exist('waveTerrain', 'var')") == 0
    assignin("base", "waveTerrain", wave_terrain_parameters(0, 0.5));
end
waveTerrain = evalin("base", "waveTerrain");
validateattributes(waveTerrain.segmentCount, {'numeric'}, ...
    {'scalar', 'integer', 'positive'});

subsystem = model + "/PD_only";
solidTemplate = subsystem + "/Brick Solid3";
transformTemplate = subsystem + "/Rigid" + newline + "Transform";
leftContactTemplate = subsystem + "/Spatial" + newline + "Contact Force1";
rightContactTemplate = subsystem + "/Spatial" + newline + "Contact Force";
leftWheel = subsystem + "/Cylindrical Solid1";
rightWheel = subsystem + "/Cylindrical Solid";
world = subsystem + "/World Frame1";

blocks = repmat(struct("solid", "", "pose", "", ...
    "leftContact", "", "rightContact", ""), ...
    waveTerrain.segmentCount, 1);
for index = 1:waveTerrain.segmentCount
    suffix = sprintf("%03d", index);
    yOffset = 120*mod(index - 1, 10);
    xOffset = 360*floor((index - 1)/10);
    blocks(index).solid = subsystem + "/TerrainWaveSolid" + suffix;
    blocks(index).pose = subsystem + "/TerrainWavePose" + suffix;
    blocks(index).leftContact = ...
        subsystem + "/TerrainWaveContactLeft" + suffix;
    blocks(index).rightContact = ...
        subsystem + "/TerrainWaveContactRight" + suffix;

    addCloneIfMissing(solidTemplate, blocks(index).solid, ...
        [2660 + xOffset, 80 + yOffset, 2740 + xOffset, 140 + yOffset]);
    addCloneIfMissing(transformTemplate, blocks(index).pose, ...
        [2480 + xOffset, 80 + yOffset, 2560 + xOffset, 140 + yOffset]);
    addCloneIfMissing(leftContactTemplate, blocks(index).leftContact, ...
        [2300 + xOffset, 30 + yOffset, 2390 + xOffset, 80 + yOffset]);
    addCloneIfMissing(rightContactTemplate, blocks(index).rightContact, ...
        [2300 + xOffset, 90 + yOffset, 2390 + xOffset, 140 + yOffset]);

    set_param(blocks(index).solid, "BrickDimensions", ...
        "waveTerrain.brickDimensions(" + index + ",:)");
    set_param(blocks(index).pose, ...
        "RotationMethod", "StandardAxis", ...
        "RotationStandardAxis", "+Z", ...
        "RotationAngle", "waveTerrain.rotationAngles(" + index + ")", ...
        "RotationAngleUnits", "rad", ...
        "TranslationMethod", "Cartesian", ...
        "TranslationCartesianOffset", ...
        "waveTerrain.translations(" + index + ",:)", ...
        "TranslationCartesianOffsetUnits", "m");

    connectPhysical(world, "RConn", 1, blocks(index).pose, "LConn", 1);
    connectPhysical(blocks(index).pose, "RConn", 1, ...
        blocks(index).solid, "RConn", 1);
    connectPhysical(leftWheel, "LConn", 1, ...
        blocks(index).leftContact, "LConn", 1);
    connectPhysical(blocks(index).leftContact, "RConn", 1, ...
        blocks(index).solid, "LConn", 1);
    connectPhysical(rightWheel, "LConn", 1, ...
        blocks(index).rightContact, "LConn", 1);
    connectPhysical(blocks(index).rightContact, "RConn", 1, ...
        blocks(index).solid, "LConn", 1);
end

set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
end

function addCloneIfMissing(template, target, position)
if getSimulinkBlockHandle(target) == -1
    add_block(template, target, "Position", position);
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
for index = 1:numel(sourceConnectivity)
    if string(sourceConnectivity(index).Type) ~= sourceType + sourceIndex
        continue
    end
    alreadyConnected = any(sourceConnectivity(index).DstBlock == targetHandle);
    break
end
if ~alreadyConnected
    add_line(char(subsystemParent(source)), sourcePort, targetPort);
end
end

function parent = subsystemParent(block)
parent = extractBefore(string(block), "/" + string(get_param(block, "Name")));
end
