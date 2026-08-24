function blocks = configure_slope_terrain_simulink(model, doSave)
%CONFIGURE_SLOPE_TERRAIN_SIMULINK Add a continuous slope in memory.
% The flat-ground source model is not modified unless doSave is explicitly
% true. Three Brick Solids form an up-ramp, platform, and down-ramp. Each
% wheel receives one Spatial Contact Force per terrain segment.

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
if evalin("base", "exist('slopeTerrain', 'var')") == 0
    assignin("base", "slopeTerrain", slope_terrain_parameters(0));
end

subsystem = model + "/PD_only";
solidTemplate = subsystem + "/Brick Solid3";
transformTemplate = subsystem + "/Rigid" + newline + "Transform";
leftContactTemplate = subsystem + "/Spatial" + newline + "Contact Force1";
rightContactTemplate = subsystem + "/Spatial" + newline + "Contact Force";
leftWheel = subsystem + "/Cylindrical Solid1";
rightWheel = subsystem + "/Cylindrical Solid";
world = subsystem + "/World Frame1";

segmentNames = ["Up", "Platform", "Down"];
solidExpressions = ["slopeTerrain.rampBrickDimensions", ...
    "slopeTerrain.platformBrickDimensions", ...
    "slopeTerrain.rampBrickDimensions"];
translationExpressions = ["slopeTerrain.upTranslation", ...
    "slopeTerrain.platformTranslation", ...
    "slopeTerrain.downTranslation"];
angleExpressions = ["slopeTerrain.slopeAngle", "0", ...
    "slopeTerrain.downSlopeAngle"];

blocks = struct();
for k = 1:numel(segmentNames)
    key = lower(segmentNames(k));
    blocks.(key + "Solid") = subsystem + "/TerrainSlope" ...
        + segmentNames(k) + "Solid";
    blocks.(key + "Pose") = subsystem + "/TerrainSlope" ...
        + segmentNames(k) + "Pose";
    blocks.(key + "LeftContact") = subsystem + "/TerrainSlope" ...
        + segmentNames(k) + "ContactLeft";
    blocks.(key + "RightContact") = subsystem + "/TerrainSlope" ...
        + segmentNames(k) + "ContactRight";

    addCloneIfMissing(solidTemplate, blocks.(key + "Solid"), ...
        [2660, 80 + 180*(k - 1), 2740, 140 + 180*(k - 1)]);
    addCloneIfMissing(transformTemplate, blocks.(key + "Pose"), ...
        [2480, 80 + 180*(k - 1), 2560, 140 + 180*(k - 1)]);
    addCloneIfMissing(leftContactTemplate, ...
        blocks.(key + "LeftContact"), ...
        [2300, 30 + 180*(k - 1), 2390, 80 + 180*(k - 1)]);
    addCloneIfMissing(rightContactTemplate, ...
        blocks.(key + "RightContact"), ...
        [2300, 100 + 180*(k - 1), 2390, 150 + 180*(k - 1)]);

    set_param(blocks.(key + "Solid"), ...
        "BrickDimensions", solidExpressions(k));
    if segmentNames(k) == "Platform"
        set_param(blocks.(key + "Pose"), "RotationMethod", "None");
    else
        set_param(blocks.(key + "Pose"), ...
            "RotationMethod", "StandardAxis", ...
            "RotationStandardAxis", "+Z", ...
            "RotationAngle", angleExpressions(k), ...
            "RotationAngleUnits", "rad");
    end
    set_param(blocks.(key + "Pose"), ...
        "TranslationMethod", "Cartesian", ...
        "TranslationCartesianOffset", translationExpressions(k), ...
        "TranslationCartesianOffsetUnits", "m");

    connectPhysical(world, "RConn", 1, ...
        blocks.(key + "Pose"), "LConn", 1);
    connectPhysical(blocks.(key + "Pose"), "RConn", 1, ...
        blocks.(key + "Solid"), "RConn", 1);
    connectPhysical(leftWheel, "LConn", 1, ...
        blocks.(key + "LeftContact"), "LConn", 1);
    connectPhysical(blocks.(key + "LeftContact"), "RConn", 1, ...
        blocks.(key + "Solid"), "LConn", 1);
    connectPhysical(rightWheel, "LConn", 1, ...
        blocks.(key + "RightContact"), "LConn", 1);
    connectPhysical(blocks.(key + "RightContact"), "RConn", 1, ...
        blocks.(key + "Solid"), "LConn", 1);
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
