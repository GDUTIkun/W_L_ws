function y = floating_base_lqr_wrench(x, baseLqr)
%FLOATING_BASE_LQR_WRENCH Return full upper-layer LQR body wrench.
%
% Output:
%   y = [FHx_des; FHz_des; MBy_des]
%
% The returned wrench is applied by the leg to the body. The leg QP should
% receive FH_ext = -[FHx_des; FHz_des] and optional MBy_des.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end
persistent thetaIntegral lastT

x = double(x(:));
if numel(x) == 7
    t = x(1);
    X = x(2:7);
elseif numel(x) == 6
    t = NaN;
    X = x(:);
else
    error("floating_base_lqr_wrench:InvalidInput", ...
        "Expected x = [t;x;z;theta;dx;dz;dtheta] or 6-state vector.");
end

[xRef, aRef] = floating_base_reference(t, baseLqr);
uBody = feedforwardWrench(aRef, baseLqr.model) ...
    - baseLqr.K * (X - xRef);
thetaKi = getFieldOrDefault(baseLqr, "thetaIntegralGain", 0);
if thetaKi ~= 0 && numel(x) == 7
    if isempty(thetaIntegral) || isempty(lastT) || t <= 0 || t < lastT
        thetaIntegral = 0;
        lastT = t;
    else
        dt = max(0, t - lastT);
        thetaIntegral = thetaIntegral + (X(3) - xRef(3)) * dt;
        thetaLimit = getFieldOrDefault(baseLqr, "thetaIntegralLimit", inf);
        thetaIntegral = min(max(thetaIntegral, -thetaLimit), thetaLimit);
        lastT = t;
    end
    uBody(3) = uBody(3) - thetaKi * thetaIntegral;
end

forceMax = baseLqr.forceMax(:);
if isscalar(forceMax)
    forceMax = repmat(forceMax, 2, 1);
end
uBody(1:2) = min(max(uBody(1:2), -forceMax), forceMax);
uBody(3) = min(max(uBody(3), -baseLqr.momentMax), baseLqr.momentMax);

y = uBody(:);
end

function u = feedforwardWrench(aRef, model)
FHx = model.m * aRef(1);
FHz = model.m * (model.g + aRef(2));
MBy = model.Iyy * aRef(3) ...
    - model.rHEq(1) * FHz + model.rHEq(2) * FHx;
u = [FHx; FHz; MBy];
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
