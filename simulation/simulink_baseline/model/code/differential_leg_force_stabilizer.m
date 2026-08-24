function [forceApplied, diagnostics] = differential_leg_force_stabilizer( ...
        t, xiDeltaCanonical, dxiDeltaCanonical, config)
%DIFFERENTIAL_LEG_FORCE_STABILIZER Bounded plant-side anti-split PD force.
% The caller maps forceApplied through the hip/knee part of the differential
% wheel-center Jacobian. The wheel-joint columns are deliberately excluded.

persistent previousApplied previousTime

diagnostics = struct( ...
    "requested", 0, ...
    "amplitudeSaturated", false, ...
    "rateLimited", false, ...
    "failSafe", false, ...
    "reset", false);

if t <= 0 || isempty(previousApplied) || isempty(previousTime) ...
        || t < previousTime
    previousApplied = 0;
    previousTime = t;
    diagnostics.reset = true;
end

required = [xiDeltaCanonical; dxiDeltaCanonical; config.Kxi; config.Kd; ...
    config.polarity; config.amplitudeLimit; config.rateLimit; config.Ts];
if ~config.enabled
    forceApplied = 0;
    previousApplied = 0;
    previousTime = t;
    return;
end
if any(~isfinite(required)) || config.amplitudeLimit < 0 ...
        || config.rateLimit < 0 || config.Ts <= 0
    forceApplied = 0;
    previousApplied = 0;
    previousTime = t;
    diagnostics.failSafe = true;
    return;
end

diagnostics.requested = config.polarity*(config.Kxi*xiDeltaCanonical ...
    + config.Kd*dxiDeltaCanonical);
amplitudeLimited = min(max(diagnostics.requested, ...
    -config.amplitudeLimit), config.amplitudeLimit);
diagnostics.amplitudeSaturated = amplitudeLimited ~= diagnostics.requested;

dt = max(config.Ts, t - previousTime);
maximumStep = config.rateLimit*dt;
forceApplied = min(max(amplitudeLimited, previousApplied - maximumStep), ...
    previousApplied + maximumStep);
diagnostics.rateLimited = forceApplied ~= amplitudeLimited;
previousApplied = forceApplied;
previousTime = t;
end
