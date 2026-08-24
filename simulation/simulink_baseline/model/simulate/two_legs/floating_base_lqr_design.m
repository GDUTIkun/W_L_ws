function baseLqr = floating_base_lqr_design(base, Q, R)
%FLOATING_BASE_LQR_DESIGN Build A/B matrices and LQR gain.
%
% Returns a struct containing:
%   model: floating-base state-space data
%   K:     u = uEq - K*(x - xRef)
%   Q,R:   weights used for LQR

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end

model = floating_base_state_space(base);

if nargin < 2 || isempty(Q)
    Q = getFieldOrDefault(base, "Q", diag([25, 80, 120, 8, 16, 10]));
end
if nargin < 3 || isempty(R)
    R = getFieldOrDefault(base, "R", diag([1/80^2, 1/140^2, 1/60^2]));
end

Q = double(Q);
R = double(R);

controllerType = getFieldOrDefault(base, "controllerType", "continuous");
controllerType = lower(string(controllerType));
if isfield(base, "Ts") && ~isempty(base.Ts)
    Ts = double(base.Ts);
else
    Ts = [];
end

if controllerType == "discrete"
    if isempty(Ts) || ~isscalar(Ts) || Ts <= 0
        error("floating_base_lqr_design:InvalidSampleTime", ...
            "Discrete LQR requires a positive scalar base.Ts.");
    end
    [K, S, poles, discreteModel] = localDlqr(model.A, model.B, Q, R, Ts);
else
    [K, S, poles] = localLqr(model.A, model.B, Q, R);
    discreteModel = [];
    Ts = NaN;
end

baseLqr = struct();
baseLqr.model = model;
baseLqr.K = K;
baseLqr.S = S;
baseLqr.poles = poles;
baseLqr.controllerType = controllerType;
baseLqr.Ts = Ts;
baseLqr.discreteModel = discreteModel;
baseLqr.Q = Q;
baseLqr.R = R;
baseLqr.forceMax = getFieldOrDefault(base, "forceMax", [inf; inf]);
baseLqr.momentMax = getFieldOrDefault(base, "momentMax", inf);
baseLqr.thetaIntegralGain = getFieldOrDefault(base, "thetaIntegralGain", 0);
baseLqr.thetaIntegralLimit = getFieldOrDefault(base, "thetaIntegralLimit", inf);
baseLqr.commandShaping = getFieldOrDefault(base, "commandShaping", struct());
baseLqr.trajectory = getFieldOrDefault(base, "trajectory", struct());
baseLqr.xRef = getFieldOrDefault(base, "xRef", model.xEq);
baseLqr.xRef = baseLqr.xRef(:);
end

function [K, S, poles, discreteModel] = localDlqr(A, B, Q, R, Ts)
if exist("ss", "file") == 2 && exist("c2d", "file") == 2
    sysc = ss(A, B, eye(size(A, 1)), zeros(size(A, 1), size(B, 2)));
    sysd = c2d(sysc, Ts, "zoh");
    Ad = sysd.A;
    Bd = sysd.B;
else
    [Ad, Bd] = zohDiscretize(A, B, Ts);
end

if exist("dlqr", "file") == 2
    [K, S, poles] = dlqr(Ad, Bd, Q, R);
elseif exist("dare", "file") == 2
    [S, ~, poles] = dare(Ad, Bd, Q, R);
    K = (R + Bd' * S * Bd) \ (Bd' * S * Ad);
else
    error("floating_base_lqr_design:MissingDiscreteLqr", ...
        "Discrete LQR requires dlqr or dare.");
end

discreteModel = struct();
discreteModel.A = Ad;
discreteModel.B = Bd;
discreteModel.Ts = Ts;
discreteModel.method = "zoh";
end

function [Ad, Bd] = zohDiscretize(A, B, Ts)
n = size(A, 1);
m = size(B, 2);
M = expm([A, B; zeros(m, n + m)] * Ts);
Ad = M(1:n, 1:n);
Bd = M(1:n, n+1:n+m);
end

function [K, S, poles] = localLqr(A, B, Q, R)
if exist("lqr", "file") == 2
    [K, S, poles] = lqr(A, B, Q, R);
    return;
end

if exist("care", "file") == 2
    [S, ~, poles] = care(A, B, Q, R);
    K = R \ (B' * S);
    return;
end

if exist("icare", "file") == 2
    [S, ~, poles] = icare(A, B, Q, R);
    K = R \ (B' * S);
    return;
end

% Last-resort continuous-time algebraic Riccati solve through the
% Hamiltonian invariant subspace. This keeps the demo runnable on MATLAB
% installations without Control System Toolbox.
n = size(A, 1);
G = B * (R \ B');
H = [A, -G; -Q, -A'];
[V, D] = eig(H);
stableIdx = find(real(diag(D)) < 0);
if numel(stableIdx) ~= n
    error("floating_base_lqr_design:MissingStableSubspace", ...
        "Could not isolate the stable Hamiltonian subspace.");
end
Vstable = V(:, stableIdx);
V1 = Vstable(1:n, :);
V2 = Vstable(n+1:end, :);
S = real(V2 / V1);
S = 0.5 * (S + S');
K = R \ (B' * S);
poles = eig(A - B*K);
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
