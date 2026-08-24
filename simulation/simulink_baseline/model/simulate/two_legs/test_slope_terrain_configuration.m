function test_slope_terrain_configuration()
%TEST_SLOPE_TERRAIN_CONFIGURATION Compile-check in-memory slope terrain.

modelDir = fileparts(mfilename("fullpath"));
startupFile = replace(fullfile(modelDir, "startup.m"), "'", "''");
evalin("base", "run('" + startupFile + "')");
model = "source";
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(model);
end
cleanup = onCleanup(@() closeIfOpened(model, wasLoaded));

slopeTerrain = slope_terrain_parameters(0.10);
assignin("base", "slopeTerrain", slopeTerrain);
blocks = configure_slope_terrain_simulink(model, false);

names = fieldnames(blocks);
for k = 1:numel(names)
    assert(getSimulinkBlockHandle(blocks.(names{k})) ~= -1, ...
        "Missing slope terrain block %s.", blocks.(names{k}));
end
assert(string(get_param(blocks.upSolid, "BrickDimensions")) ...
    == "slopeTerrain.rampBrickDimensions");
assert(string(get_param(blocks.platformSolid, "BrickDimensions")) ...
    == "slopeTerrain.platformBrickDimensions");
assert(string(get_param(blocks.upPose, "RotationMethod")) ...
    == "StandardAxis");
assert(string(get_param(blocks.upPose, "RotationAngle")) ...
    == "slopeTerrain.slopeAngle");
assert(string(get_param(blocks.downPose, "RotationAngle")) ...
    == "slopeTerrain.downSlopeAngle");
assert(abs(slopeTerrain.riseHeight - 0.5*tan(0.10)) < 1e-12);
assert(abs(slopeTerrain.trailingEdgeX - 2.0) < 1e-12);

% Verify that the nominal top corners meet the flat ground and platform.
upLowY = slopeTerrain.upTranslation(2) ...
    - 0.5*slopeTerrain.rampLength*sin(slopeTerrain.slopeAngle) ...
    + 0.5*slopeTerrain.thickness*cos(slopeTerrain.slopeAngle);
upHighY = slopeTerrain.upTranslation(2) ...
    + 0.5*slopeTerrain.rampLength*sin(slopeTerrain.slopeAngle) ...
    + 0.5*slopeTerrain.thickness*cos(slopeTerrain.slopeAngle);
assert(abs(upLowY - slopeTerrain.groundTopY) < 1e-12);
assert(abs(upHighY - slopeTerrain.groundTopY ...
    - slopeTerrain.riseHeight) < 1e-12);

set_param(model, "SimulationCommand", "update");
fprintf("Slope terrain configuration checks passed.\n");
clear cleanup
end

function closeIfOpened(model, wasLoaded)
if bdIsLoaded(model) && ~wasLoaded
    close_system(model, 0);
end
end
