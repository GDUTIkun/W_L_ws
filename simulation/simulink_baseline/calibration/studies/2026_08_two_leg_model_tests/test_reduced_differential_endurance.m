function summary = test_reduced_differential_endurance()
%TEST_REDUCED_DIFFERENTIAL_ENDURANCE 360/720/multi-turn acceptance suite.

verifyControllerGuards();
cases = [
    struct("name","C1_360_v010", "yaw",360, "speed",0.10, "rate",0.08)
    struct("name","C2_720_v010", "yaw",720, "speed",0.10, "rate",0.08)
    struct("name","C3_1800_v010", "yaw",1800, "speed",0.10, "rate",0.08)
    struct("name","C1R_360_v010", "yaw",-360, "speed",0.10, "rate",-0.08)
    struct("name","C2R_720_v010", "yaw",-720, "speed",0.10, "rate",-0.08)
    struct("name","HC1_360_v020", "yaw",360, "speed",0.20, "rate",0.08)
    struct("name","HC2_720_v020", "yaw",720, "speed",0.20, "rate",0.08)
    struct("name","HC1R_360_v020", "yaw",-360, "speed",0.20, "rate",-0.08)
];
records = repmat(struct(), numel(cases), 1);
for k = 1:numel(cases)
    result = simulate_reduced_differential_turn( ...
        cases(k).yaw, cases(k).speed, cases(k).rate, []);
    records(k).name = cases(k).name;
    records(k).targetYawDeg = result.targetYawDeg;
    records(k).speedReference = result.speedReference;
    records(k).actualYawAtTargetDeg = result.actualYawAtTargetDeg;
    records(k).yawErrorAtTargetDeg = result.yawErrorAtTargetDeg;
    records(k).finalSpeed = result.finalSpeed;
    records(k).finalYawRate = result.finalYawRate;
    records(k).maxAbsXiAfter5Mm = result.maxAbsXiAfter5Mm;
    records(k).tailMaxAbsXiMm = result.tailMaxAbsXiMm;
    records(k).tailRmsXiMm = result.tailRmsXiMm;
    records(k).recoveryTime = result.recoveryTime;
    records(k).maxAbsForce = result.maxAbsForce;
    records(k).pass = result.pass;
end
summary = struct2table(records);
studyDir = fileparts(mfilename("fullpath"));
writetable(summary, fullfile(studyDir, ...
    "reduced_differential_endurance.csv"));
disp(summary);
assert(all(summary.pass), ...
    "Reduced differential endurance acceptance failed.");
end

function verifyControllerGuards()
config = struct("enabled", true, "Kxi", 1000, "Kd", 100, ...
    "polarity", 1, "amplitudeLimit", 50, "rateLimit", 1000, ...
    "Ts", 0.005);
clear differential_leg_force_stabilizer
[force, diagnostics] = differential_leg_force_stabilizer( ...
    0, 0.1, 0, config);
assert(force == 5 && diagnostics.amplitudeSaturated ...
    && diagnostics.rateLimited, ...
    "Controller amplitude/rate guards did not engage.");
[force, diagnostics] = differential_leg_force_stabilizer( ...
    0.005, NaN, 0, config);
assert(force == 0 && diagnostics.failSafe, ...
    "Controller nonfinite-input fail-safe did not engage.");
config.enabled = false;
[force, ~] = differential_leg_force_stabilizer(0.010, 0.1, 0, config);
assert(force == 0, "Disabled controller must apply zero force.");
end
