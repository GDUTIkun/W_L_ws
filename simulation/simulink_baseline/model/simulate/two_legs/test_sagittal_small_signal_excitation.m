function result = test_sagittal_small_signal_excitation()
%TEST_SAGITTAL_SMALL_SIGNAL_EXCITATION Verify default-off and node mapping.

ctrl = struct();
zero = sagittal_small_signal_excitation(1, ctrl);
assert(all(struct2array(zero) == 0), ...
    "Missing audit configuration must preserve nominal behavior.");

frequencies = [1.25; 1.75];
amplitudes = [0.03; 0.02];
phases = [0.2; -0.4];
time = 1.3;
expected = sum(amplitudes.*sin(2*pi*frequencies*time + phases));
channels = ["rolling_task", "wheel_feedforward", "common_wrench"];
fields = ["rollingTaskAcceleration", "wheelRelativeAcceleration", ...
    "commonRollingForce"];
for index = 1:numel(channels)
    ctrl.sagittalSmallSignalAudit = struct( ...
        "enabled", true, "channel", channels(index), ...
        "frequenciesHz", frequencies, "amplitudes", amplitudes, ...
        "phasesRad", phases, "startTime", 1, "stopTime", 2);
    actual = sagittal_small_signal_excitation(time, ctrl);
    values = struct2array(actual);
    assert(abs(actual.(fields(index)) - expected) < 1e-12);
    values(index) = 0;
    assert(all(abs(values) < 1e-12), ...
        "Excitation leaked into another summing node.");
    outside = sagittal_small_signal_excitation(0.5, ctrl);
    assert(all(struct2array(outside) == 0));
end

result = struct("passed", true, "expectedValue", expected);
disp(result);
end
