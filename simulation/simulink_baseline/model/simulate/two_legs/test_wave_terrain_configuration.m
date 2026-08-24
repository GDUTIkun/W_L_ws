function tests = test_wave_terrain_configuration
tests = functiontests(localfunctions);
end

function testGeometryContract(testCase)
terrain = wave_terrain_parameters(0.02, 0.5, ...
    "WavelengthCount", 2, "SegmentsPerWavelength", 10);
verifyEqual(testCase, terrain.segmentCount, 20);
verifyEqual(testCase, terrain.trailingEdgeX - terrain.leadingEdgeX, 1.0, ...
    "AbsTol", 1e-12);
verifyEqual(testCase, min(terrain.heightEdges), 0, "AbsTol", 1e-12);
verifyEqual(testCase, max(terrain.heightEdges), 0.02, "AbsTol", 1e-12);
verifySize(testCase, terrain.brickDimensions, [20, 3]);
verifySize(testCase, terrain.translations, [20, 3]);
verifyTrue(testCase, all(terrain.brickDimensions(:) > 0));
verifyEqual(testCase, terrain.contactFrameMode, "horizontal-baseline");
end

function testDisabledTerrainIsParked(testCase)
terrain = wave_terrain_parameters(0, 0.5, "Enabled", false);
verifyFalse(testCase, terrain.enabled);
verifyTrue(testCase, all(terrain.translations(:, 2) == -10));
end

function testInMemoryConfiguration(testCase)
modelDir = fileparts(mfilename("fullpath"));
startupFile = replace(fullfile(modelDir, "startup.m"), "'", "''");
evalin("base", "run('" + startupFile + "')");
model = "source";
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(model);
end
wasDirty = string(get_param(model, "Dirty"));
waveWasPresent = getSimulinkBlockHandle( ...
    model + "/PD_only/TerrainWaveSolid001") ~= -1;
cleanup = onCleanup(@() restoreModel(model, wasLoaded, wasDirty, ...
    waveWasPresent));

terrain = wave_terrain_parameters(0.02, 0.5, ...
    "WavelengthCount", 1, "SegmentsPerWavelength", 4);
assignin("base", "waveTerrain", terrain);
contract = coupled_two_leg_qp_signal_contract();
[demuxOutputs, ~] = coupled_two_leg_qp_demux_outputs();
set_param(model + "/PD_only/Coupled QP", ...
    "OutputDimensions", string(contract.width));
set_param(model + "/PD_only/Coupled QP Split", "Outputs", demuxOutputs);
blocks = configure_wave_terrain_simulink(model, false);
verifyNumElements(testCase, blocks, terrain.segmentCount);
for index = 1:numel(blocks)
    verifyNotEqual(testCase, getSimulinkBlockHandle(blocks(index).solid), -1);
    verifyNotEqual(testCase, getSimulinkBlockHandle(blocks(index).pose), -1);
    verifyEqual(testCase, string(get_param(blocks(index).solid, ...
        "BrickDimensions")), ...
        "waveTerrain.brickDimensions(" + index + ",:)");
end
verifyEqual(testCase, string(get_param(model, "Dirty")), "on");
clear cleanup
end

function restoreModel(model, wasLoaded, wasDirty, waveWasPresent)
if ~bdIsLoaded(model)
    return
end
if ~waveWasPresent
    subsystem = model + "/PD_only/";
    prefixes = ["TerrainWaveContactLeft", "TerrainWaveContactRight", ...
        "TerrainWaveSolid", "TerrainWavePose"];
    for prefix = prefixes
        for index = 1:999
            path = subsystem + prefix + sprintf("%03d", index);
            if getSimulinkBlockHandle(path) == -1
                if index == 1
                    continue
                end
                break
            end
            delete_block(path);
        end
    end
end
if ~wasLoaded
    close_system(model, 0);
elseif wasDirty == "off"
    set_param(model, "Dirty", "off");
end
end
