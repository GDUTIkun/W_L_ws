function y = wheel_position_lqr_reference(x, wheelLqr, baseLqr)
%WHEEL_POSITION_LQR_REFERENCE Standalone scheduled WIPM-LQR wheel planner.
%
% Input:  [t; baseState(6); xi; dxi; height]
% Output: [xiRef; dxiRef; ddxiRef; xiRaw]

if nargin < 2 || isempty(wheelLqr)
    wheelLqr = evalin("base", "wheelLqr");
end
if nargin < 3 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

persistent state
x = double(x(:));
if numel(x) ~= 10
    error("wheel_position_lqr_reference:InvalidInput", ...
        "Expected [t; baseState(6); xi; dxi; height].");
end

t = x(1);
baseState = x(2:7);
xi = x(8);
dxi = x(9);
height = min(max(x(10), wheelLqr.heightGrid(1)), ...
    wheelLqr.heightGrid(end));
[xRef, aRef] = floating_base_reference(t, baseLqr);
K = interp1(wheelLqr.heightGrid, wheelLqr.K, height, "linear");
xiFeedforward = -height / baseLqr.model.g * aRef(1);
errorState = [baseState(1) - xRef(1); baseState(4) - xRef(4)];
xiRaw = clamp(wheelLqr.neutral + xiFeedforward - K * errorState, ...
    wheelLqr.positionMin, wheelLqr.positionMax);

if isfield(wheelLqr, "governorEnabled") ...
        && ~logical(wheelLqr.governorEnabled)
    % Paper Eq. (21) supplies a desired position.  Eq. (38) then uses zero
    % desired wheel velocity, so no synthetic raw-reference derivative is
    % introduced here.
    state = [];
    y = [xiRaw; 0; 0; xiRaw];
    return
end

if isempty(state) || t <= 0 || t < state.t
    state = struct("t", t, ...
        "position", clamp(xi, wheelLqr.positionMin, wheelLqr.positionMax), ...
        "velocity", clamp(dxi, -wheelLqr.velocityMax, wheelLqr.velocityMax), ...
        "acceleration", 0);
else
    dt = t - state.t;
    if dt > max(1e-12, eps(max(1, abs(t))))
        steps = max(1, ceil(dt / wheelLqr.Ts - 1e-9));
        h = dt / steps;
        for idx = 1:steps
            [state.position, state.velocity, state.acceleration] = ...
                wheel_position_governor_step(state.position, state.velocity, ...
                xiRaw, h, wheelLqr);
        end
        state.t = t;
    end
end

y = [state.position; state.velocity; state.acceleration; xiRaw];
end

function y = clamp(x, lowerBound, upperBound)
y = min(max(x, lowerBound), upperBound);
end
