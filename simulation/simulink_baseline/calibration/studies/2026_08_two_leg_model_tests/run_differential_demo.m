function output = run_differential_demo(mode, detailedStopTime)
%RUN_DIFFERENTIAL_DEMO Visual entry point for the anti-split controller.
%
%   run_differential_demo("quick")
%       Compare controller enabled/disabled during a continuous 720 deg,
%       0.20 m/s reduced-model turn. This is the recommended first run.
%
%   run_differential_demo("detailed", 10)
%       Run the source.slx chain in Accelerator mode for 10 seconds and
%       plot the logged yaw, attitude, and differential leg coordinate.
%
%   run_differential_demo("all", 10)
%       Run both demonstrations.

arguments
    mode (1, 1) string {mustBeMember(mode, ["quick", "detailed", "all"])} = "quick"
    detailedStopTime (1, 1) double {mustBeFinite, mustBePositive} = 10
end

studyDir = fileparts(mfilename("fullpath"));
codeRoot = fileparts(fileparts(fileparts(studyDir)));
addpath(fullfile(codeRoot, "model", "code"));
addpath(studyDir);

output = struct();
if mode == "quick" || mode == "all"
    output.quick = runQuickDemo();
end
if mode == "detailed" || mode == "all"
    output.detailed = runDetailedDemo( ...
        codeRoot, detailedStopTime);
end
end

function output = runQuickDemo()
targetYawDeg = 720;
speedReference = 0.20;
yawRateReference = 0.08;
config = struct("enabled", true, "Kxi", 1000, "Kd", 100, ...
    "polarity", 1, "amplitudeLimit", 50, "rateLimit", 1000, ...
    "Ts", 0.005);

enabled = simulate_reduced_differential_turn( ...
    targetYawDeg, speedReference, yawRateReference, config);
config.enabled = false;
disabled = simulate_reduced_differential_turn( ...
    targetYawDeg, speedReference, yawRateReference, config);

summary = table( ...
    ["enabled"; "disabled"], ...
    [enabled.pass; disabled.pass], ...
    [enabled.maxAbsXiAfter5Mm; disabled.maxAbsXiAfter5Mm], ...
    [enabled.tailMaxAbsXiMm; disabled.tailMaxAbsXiMm], ...
    [enabled.recoveryTime; disabled.recoveryTime], ...
    [enabled.maxAbsForce; disabled.maxAbsForce], ...
    'VariableNames', {'controller', 'pass', 'steadyPeakXiMm', ...
    'tailPeakXiMm', 'recoveryTimeS', 'peakForceN'});
disp(summary);

targetTime = 2 + abs(deg2rad(targetYawDeg)/yawRateReference) + 0.25;
figure("Name", "Differential anti-split: quick comparison", ...
    "Color", "w");
layout = tiledlayout(2, 2, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "720 deg continuous turn at 0.20 m/s");

nexttile;
plot(enabled.time, rad2deg(enabled.yaw), "LineWidth", 1.4);
yline(targetYawDeg, "--", "Target 720 deg");
xline(targetTime, ":", "Target time");
grid on;
xlabel("Time (s)");
ylabel("Unwrapped yaw (deg)");
title("Continuous yaw");

nexttile;
semilogy(enabled.time, max(1e3*abs(enabled.xi), 1e-3), ...
    "LineWidth", 1.5);
hold on;
semilogy(disabled.time, max(1e3*abs(disabled.xi), 1e-3), ...
    "LineWidth", 1.2);
yline(1, "--", "1 mm acceptance");
grid on;
xlabel("Time (s)");
ylabel("|xiDelta| (mm, log scale)");
legend("Controller enabled", "Controller disabled", ...
    "Location", "best");
title("Leg split comparison");

nexttile;
plot(enabled.time, enabled.force, "LineWidth", 1.3);
hold on;
plot(enabled.time, enabled.disturbance, "LineWidth", 1.0);
yline(50, "--", "Force limit");
yline(-50, "--", "HandleVisibility", "off");
grid on;
xlabel("Time (s)");
ylabel("Force (N)");
legend("Anti-split force", "Disturbance", "Location", "best");
title("Bounded controller action");

nexttile;
yyaxis left;
plot(enabled.time, enabled.speed, "LineWidth", 1.3);
ylabel("Forward speed (m/s)");
yyaxis right;
plot(enabled.time, enabled.yawRate, "LineWidth", 1.3);
ylabel("Yaw rate (rad/s)");
grid on;
xlabel("Time (s)");
title("Command tracking");

output = struct("enabled", enabled, "disabled", disabled, ...
    "summary", summary);
end

function output = runDetailedDemo(codeRoot, stopTime)
modelDir = fullfile(codeRoot, "model", "simulate", "two_legs");
previousDirectory = pwd;
cleanup = onCleanup(@() cd(previousDirectory));
cd(modelDir);

caseName = "C0_90_left_continuous";
summary = test_large_yaw_turning_simulink( ...
    caseName, struct(), stopTime, struct());
csvPath = fullfile(modelDir, "large_yaw_timeseries", ...
    caseName + ".csv");
if ~isfile(csvPath)
    error("run_differential_demo:MissingDetailedLog", ...
        "Detailed simulation did not produce %s.", csvPath);
end
data = readtable(csvPath);
xiDeltaMm = 500*(data.xiRight - data.xiLeft);

figure("Name", "Differential anti-split: detailed Accelerator run", ...
    "Color", "w");
layout = tiledlayout(3, 1, "TileSpacing", "compact", ...
    "Padding", "compact");
title(layout, "source.slx detailed-chain observation");

nexttile;
plot(data.time, rad2deg(data.yawUnwrapped), "LineWidth", 1.3);
hold on;
plot(data.time, rad2deg(data.referenceYaw), "--", "LineWidth", 1.1);
grid on;
ylabel("Yaw (deg)");
legend("Measured", "Reference", "Location", "best");

nexttile;
plot(data.time, xiDeltaMm, "LineWidth", 1.3);
hold on;
yline(1, "--", "+1 mm");
yline(-1, "--", "-1 mm");
grid on;
ylabel("xiDelta (mm)");
title("Canonical leg split: (right-left)/2");

nexttile;
plot(data.time, rad2deg(data.roll), "LineWidth", 1.2);
hold on;
plot(data.time, rad2deg(data.pitch), "LineWidth", 1.2);
grid on;
xlabel("Time (s)");
ylabel("Attitude (deg)");
legend("Roll", "Pitch", "Location", "best");

output = struct("summary", summary, "data", data, ...
    "logPath", csvPath);
clear cleanup
end
