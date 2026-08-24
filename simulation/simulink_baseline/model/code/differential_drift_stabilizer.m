function [uApplied, diagnostics] = differential_drift_stabilizer( ...
        t, xiDeltaCanonical, dxiDeltaCanonical, config)
%DIFFERENTIAL_DRIFT_STABILIZER Reset-safe scalar canonical-drift feedback.
% The requested correction is
%   polarity*(Kxi*xiDeltaCanonical + Kd*dxiDeltaCanonical).
% It is amplitude limited first and then slew limited from the previously
% applied value. Disabled, zero-gain, reset, rollback, and invalid calls are
% strict identity and cannot retain a previous correction.

persistent previousApplied previousTime

diagnostics = struct( ...
    "requested", 0, ...
    "amplitudeSaturated", false, ...
    "rateLimited", false, ...
    "failSafe", false, ...
    "reset", false);
uApplied = 0;

if ~isValidConfig(config) || ~isFiniteScalar(t) ...
        || ~isFiniteScalar(xiDeltaCanonical) ...
        || ~isFiniteScalar(dxiDeltaCanonical)
    diagnostics.failSafe = true;
    diagnostics.reset = true;
    previousApplied = [];
    previousTime = [];
    return;
end

enabled = logical(config.enabled);
zeroGain = config.Kxi == 0 && config.Kd == 0;
timeReset = t <= 0;
timeRollback = ~isempty(previousTime) && t < previousTime;
if ~enabled || zeroGain || timeReset || timeRollback
    diagnostics.reset = true;
    previousApplied = [];
    previousTime = [];
    return;
end

requested = config.polarity*(config.Kxi*xiDeltaCanonical ...
    + config.Kd*dxiDeltaCanonical);
if ~isfinite(requested)
    diagnostics.failSafe = true;
    diagnostics.reset = true;
    previousApplied = [];
    previousTime = [];
    return;
end
diagnostics.requested = requested;

amplitudeLimited = min(max(requested, -config.amplitudeLimit), ...
    config.amplitudeLimit);
diagnostics.amplitudeSaturated = amplitudeLimited ~= requested;

if isempty(previousApplied) || isempty(previousTime)
    previousApplied = 0;
end
if ~isempty(previousTime) && t == previousTime
    uApplied = previousApplied;
    diagnostics.rateLimited = uApplied ~= amplitudeLimited;
    return;
end

maximumStep = config.rateLimit*config.Ts;
delta = amplitudeLimited - previousApplied;
limitedDelta = min(max(delta, -maximumStep), maximumStep);
uApplied = previousApplied + limitedDelta;
diagnostics.rateLimited = limitedDelta ~= delta;

previousApplied = uApplied;
previousTime = t;
end

function valid = isValidConfig(config)
required = ["enabled", "Kxi", "Kd", "polarity", ...
    "amplitudeLimit", "rateLimit", "Ts"];
valid = isstruct(config) && isscalar(config);
if ~valid
    return;
end
for idx = 1:numel(required)
    fieldName = char(required(idx));
    if ~isfield(config, fieldName)
        valid = false;
        return;
    end
    value = config.(fieldName);
    if ~(isnumeric(value) || islogical(value)) || ~isreal(value) ...
            || ~isscalar(value) || ~isfinite(value)
        valid = false;
        return;
    end
end
values = [double(config.enabled), config.Kxi, config.Kd, ...
    config.polarity, config.amplitudeLimit, config.rateLimit, config.Ts];
valid = all(isfinite(values)) && (config.enabled == 0 || config.enabled == 1) ...
    && config.Kxi >= 0 && config.Kd >= 0 ...
    && abs(config.polarity) == 1 ...
    && config.amplitudeLimit >= 0 && config.rateLimit >= 0 ...
    && config.Ts > 0;
end

function valid = isFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) ...
    && isfinite(value);
end
