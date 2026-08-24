function reference = base_nmpc_reference(x, baseLqr, baseNmpc, wheelLqr)
%BASE_NMPC_REFERENCE Return flattened S-Function references over the horizon.
%
% Layout:
%   [y_ref_0(14); y_ref(stages 1:N-1, 14 each); y_ref_e(8)]
% Simulink input: [t; xiRef; dxiRef; ddxiRef; xiRaw; previousWrench(3)].

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end
if nargin < 3 || isempty(baseNmpc)
    baseNmpc = evalin("base", "baseNmpc");
end
if nargin < 4 || isempty(wheelLqr)
    wheelLqr = evalin("base", "wheelLqr");
end

x = double(x(:));
if isscalar(x)
    t = x;
    planner = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
    previousWrench = baseNmpc.model.uEq(:);
elseif numel(x) == 8
    t = x(1);
    planner = x(2:5);
    previousWrench = x(6:8);
else
    error("base_nmpc_reference:InvalidInput", ...
        "Expected t or [t; planner(4); previousWrench(3)].");
end
N = baseNmpc.N;
pathReference = zeros(14, max(N - 1, 0));
xiRef = planner(1);
dxiRef = planner(2);
xiRaw = planner(4);

for k = 0:N
    [baseReference, aRef] = floating_base_reference( ...
        t + k*baseNmpc.Ts, baseLqr);
    stateReference = [baseReference; xiRef; dxiRef];
    uRef = feedforwardWrench(aRef, xiRef, baseNmpc.model);
    uRef = min(max(uRef, baseNmpc.uMin(:)), baseNmpc.uMax(:));
    stageReference = [stateReference; uRef; previousWrench];

    if k == 0
        initialReference = stageReference;
    elseif k < N
        pathReference(:, k) = stageReference;
    else
        terminalReference = stateReference;
    end

    if k < N
        [xiRef, dxiRef] = wheel_position_governor_step( ...
            xiRef, dxiRef, xiRaw, baseNmpc.Ts, wheelLqr);
    end
end

reference = [initialReference; pathReference(:); terminalReference];
end

function u = feedforwardWrench(aRef, xiRef, model)
FHx = model.m * aRef(1);
FHz = model.m * (model.g + aRef(2));
MBy = model.Iyy * aRef(3) ...
    - (xiRef - model.xiEq) * FHz + model.rWzEq * FHx;
u = [FHx; FHz; MBy];
end
