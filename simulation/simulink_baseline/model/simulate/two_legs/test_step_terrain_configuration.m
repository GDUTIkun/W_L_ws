function test_step_terrain_configuration()
%TEST_STEP_TERRAIN_CONFIGURATION Compile-check the in-memory terrain plant.

modelDir = fileparts(mfilename("fullpath"));
startupFile = replace(fullfile(modelDir, "startup.m"), "'", "''");
evalin("base", "run('" + startupFile + "')");
model = "source";
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(model);
end
cleanup = onCleanup(@() closeIfOpened(model, wasLoaded));

terrain = step_terrain_parameters(0.03);
assignin("base", "terrain", terrain);
blocks = configure_step_terrain_simulink(model, false);

names = fieldnames(blocks);
for k = 1:numel(names)
    assert(getSimulinkBlockHandle(blocks.(names{k})) ~= -1, ...
        "Missing terrain block %s.", blocks.(names{k}));
end
assert(string(get_param(blocks.solid, "BrickDimensions")) ...
    == "terrain.brickDimensions");
assert(string(get_param(blocks.pose, "TranslationCartesianOffset")) ...
    == "terrain.translation");
assert(all(abs(terrain.translation - [0.9, 0.04, 0]) < 1e-12));
assert(all(abs(terrain.brickDimensions - [0.6, 0.03, 2]) < 1e-12));

set_param(model, "SimulationCommand", "update");
fprintf("Step terrain configuration checks passed.\n");
clear cleanup
end

function closeIfOpened(model, wasLoaded)
if ~wasLoaded && bdIsLoaded(model)
    close_system(model, 0);
end
end
