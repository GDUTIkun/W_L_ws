function excitation = sagittal_small_signal_excitation(t, ctrl)
%SAGITTAL_SMALL_SIGNAL_EXCITATION Default-off diagnostic multisine.
% Values map to existing controller summing nodes; this helper does not
% alter nominal behavior when ctrl.sagittalSmallSignalAudit is absent or
% disabled.

excitation = struct( ...
    "rollingTaskAcceleration", 0, ...
    "wheelRelativeAcceleration", 0, ...
    "commonRollingForce", 0);
if ~isfield(ctrl, "sagittalSmallSignalAudit")
    return;
end
config = ctrl.sagittalSmallSignalAudit;
if ~logical(fieldOr(config, "enabled", false))
    return;
end

channel = lower(string(fieldOr(config, "channel", "")));
frequencies = double(fieldOr(config, "frequenciesHz", []));
amplitudes = double(fieldOr(config, "amplitudes", []));
phases = double(fieldOr(config, "phasesRad", zeros(size(frequencies))));
startTime = double(fieldOr(config, "startTime", 0));
stopTime = double(fieldOr(config, "stopTime", inf));
frequencies = frequencies(:);
amplitudes = amplitudes(:);
phases = phases(:);
if ~ismember(channel, ["rolling_task", "wheel_feedforward", "common_wrench"]) ...
        || isempty(frequencies) || numel(amplitudes) ~= numel(frequencies) ...
        || numel(phases) ~= numel(frequencies) ...
        || any(~isfinite(frequencies) | frequencies <= 0) ...
        || any(~isfinite(amplitudes)) || any(~isfinite(phases)) ...
        || ~isscalar(startTime) || ~isfinite(startTime) ...
        || ~isscalar(stopTime) || isnan(stopTime) || stopTime <= startTime
    error("sagittal_small_signal_excitation:InvalidConfiguration", ...
        "Sagittal small-signal audit configuration is invalid.");
end
if t < startTime || t > stopTime
    return;
end

value = sum(amplitudes.*sin(2*pi*frequencies*t + phases));
switch channel
    case "rolling_task"
        excitation.rollingTaskAcceleration = value;
    case "wheel_feedforward"
        excitation.wheelRelativeAcceleration = value;
    case "common_wrench"
        excitation.commonRollingForce = value;
end
end

function value = fieldOr(input, name, fallback)
if isfield(input, name)
    value = input.(name);
else
    value = fallback;
end
end
