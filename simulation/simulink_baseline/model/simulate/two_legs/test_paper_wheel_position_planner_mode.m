function test_paper_wheel_position_planner_mode()
%TEST_PAPER_WHEEL_POSITION_PLANNER_MODE Verify raw Eq. (21) A/B semantics.

modelDir = fileparts(mfilename("fullpath"));
run(fullfile(modelDir, "startup.m"));
wheelLqr = evalin("base", "wheelLqr");
baseLqr = evalin("base", "baseLqr");
fullBaseNmpc = evalin("base", "fullBaseNmpc");

rawConfig = wheelLqr;
rawConfig.governorEnabled = false;
baseState = baseLqr.model.xEq;
input = [0; baseState; rawConfig.neutral; 0; ...
    mean(rawConfig.heightGrid)];
clear wheel_position_lqr_reference
planner = wheel_position_lqr_reference(input, rawConfig, baseLqr);
assert(planner(1) == planner(4));
assert(planner(2) == 0 && planner(3) == 0);

previousWrench = fullBaseNmpc.model.uEq;
reference = full_base_nmpc_reference( ...
    [0; planner; previousWrench], baseLqr, fullBaseNmpc, rawConfig);
stageCount = fullBaseNmpc.N;
path = reshape(reference(1:40*stageCount), 40, stageCount);
assert(all(path(13:14, :) == planner(4), "all"));
assert(all(path(15:16, :) == 0, "all"));
assert(all(path(29:40, :) == previousWrench, "all"));

exactDeltaConfig = fullBaseNmpc;
exactDeltaConfig.incrementCostMode = "state_memory";
exactReference = full_base_nmpc_reference( ...
    [0; planner; previousWrench], baseLqr, exactDeltaConfig, rawConfig);
exactPath = reshape(exactReference(1:40*stageCount), 40, stageCount);
assert(all(exactPath(29:40, :) == 0, "all"), ...
    "Exact Eq. (23) mode must use zero as the delta-u residual target.");

fprintf("Paper raw Eq. (21) planner-mode checks passed.\n");
end
