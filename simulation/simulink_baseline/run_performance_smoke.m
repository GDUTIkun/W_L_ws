function summary = run_performance_smoke(stopTime)
%RUN_PERFORMANCE_SMOKE Run a short Accelerator test in the snapshot.

arguments
    stopTime (1, 1) double {mustBeFinite, mustBePositive} = 5
end

context = open_proformance_test(false);
previousDirectory = pwd;
cleanup = onCleanup(@() cd(previousDirectory));
cd(context.modelDir);
summary = test_large_yaw_turning_simulink( ...
    "C0_90_left_continuous", struct(), stopTime, struct());
assert(all(summary.simulationCompleted), ...
    "Performance snapshot smoke simulation did not complete.");
disp(summary(:, ["name", "simulationCompleted", "controlStable", ...
    "maxAbsXiDelta", "finalXiDelta", "simulationWallTime"]));
clear cleanup
end
