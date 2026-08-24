function y = floating_base_lqr_command(x, baseLqr)
%FLOATING_BASE_LQR_COMMAND Simulink upper-layer command for base-state LQR.
%
% Input:
%   x = [t; xB; zB; thetaB; dxB; dzB; dthetaB]
% or:
%   x = [xB; zB; thetaB; dxB; dzB; dthetaB]
%
% Output:
%   y = [FHx_ext; FHz_ext; MBy_des]
%
% The LQR state uses the floating-base CoM state. If base.controllerType is
% "discrete", baseLqr.K is designed from the ZOH-discretized base model and
% this function should be driven by sampled states in Simulink.
%
% The LQR wrench
% [FHx_des; FHz_des; MBy_des] is the wrench applied by the leg to the body.
% The leg QP uses the external force applied by the body to the leg, so this
% function flips only the force components:
%   FH_ext = -[FHx_des; FHz_des]
%   MBy_des is kept as the body-side desired pure pitch moment.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

uBody = floating_base_lqr_wrench(x, baseLqr);
FH_ext = -uBody(1:2);
MBy_des = uBody(3);

y = [FH_ext; MBy_des];
y = applyCommandShaping(y, x, baseLqr);
end

function y = applyCommandShaping(yRaw, x, baseLqr)
persistent yPrev lastT lastSignature

shaping = getFieldOrDefault(baseLqr, "commandShaping", struct());
if isempty(shaping) || ~getFieldOrDefault(shaping, "enabled", false)
    y = yRaw(:);
    return;
end

x = double(x(:));
if numel(x) == 7
    t = x(1);
else
    t = NaN;
end

signature = shapingSignature(shaping);
reset = isempty(yPrev) || numel(yPrev) ~= numel(yRaw) ...
    || isempty(lastSignature) || lastSignature ~= signature ...
    || (~isnan(t) && (isempty(lastT) || t <= 0 || t < lastT));

if reset
    yPrev = yRaw(:);
    lastT = t;
    lastSignature = signature;
    y = yRaw(:);
    return;
end

dt = getSampleTime(t, lastT, baseLqr);
y = yRaw(:);

channels = getFieldOrDefault(shaping, "channels", [true; false; false]);
channels = logical(channels(:));
if isscalar(channels)
    channels = repmat(channels, numel(y), 1);
end
channels = channels(1:min(numel(channels), numel(y)));

filterTau = getFieldOrDefault(shaping, "filterTau", 0);
if filterTau > 0
    alpha = dt / (filterTau + dt);
    idx = find(channels);
    y(idx) = yPrev(idx) + alpha * (y(idx) - yPrev(idx));
end

rateLimit = getFieldOrDefault(shaping, "rateLimit", inf);
if isfinite(rateLimit) && rateLimit > 0
    if isscalar(rateLimit)
        rateLimit = repmat(rateLimit, numel(y), 1);
    else
        rateLimit = rateLimit(:);
    end
    idx = find(channels);
    for k = idx(:).'
        maxStep = rateLimit(min(k, numel(rateLimit))) * dt;
        y(k) = yPrev(k) + min(max(y(k) - yPrev(k), -maxStep), maxStep);
    end
end

yPrev = y(:);
lastT = t;
lastSignature = signature;
end

function dt = getSampleTime(t, lastT, baseLqr)
if ~isnan(t) && ~isempty(lastT) && t > lastT
    dt = t - lastT;
elseif ~isnan(t)
    dt = 0;
else
    dt = getFieldOrDefault(baseLqr, "Ts", 0.005);
    if ~isfinite(dt) || dt <= 0
        dt = 0.005;
    end
end
end

function signature = shapingSignature(shaping)
channels = getFieldOrDefault(shaping, "channels", [true; false; false]);
channels = logical(channels(:));
filterTau = getFieldOrDefault(shaping, "filterTau", 0);
rateLimit = getFieldOrDefault(shaping, "rateLimit", inf);
signature = string(sprintf("%s|%.9g|%s", mat2str(channels(:).'), ...
    double(filterTau), mat2str(double(rateLimit(:).'))));
end

function value = getFieldOrDefault(s, name, defaultValue)
if isstruct(s) && isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
