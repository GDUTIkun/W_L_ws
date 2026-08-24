function model = base_wheel_state_space(base, leg, traj)
%BASE_WHEEL_STATE_SPACE Linear planar base-plus-common-wheel-position model.
%
% State: [xB; zB; thetaB; dxB; dzB; dthetaB; xi; dxi]
% Input: [FHx; FHz; MBy], the total two-leg wrench applied to the base.

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 2 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 3 || isempty(traj)
    traj = evalin("base", "traj");
end

A = zeros(8, 8);
B = zeros(8, 3);
bodyMass = base.m + 2 * (leg.m1 + leg.m2);
A(1, 4) = 1;
A(2, 5) = 1;
A(3, 6) = 1;
A(7, 8) = 1;

rollingDenominator = leg.mw * leg.r + leg.Iw / leg.r;
B(4, 1) = 1 / bodyMass;
B(5, 2) = 1 / bodyMass;
B(8, 1) = -1 / bodyMass - leg.r / (2 * rollingDenominator);
B(8, 3) = -1 / (2 * rollingDenominator);

rHEq = rotatePitch2D(base.rHBody(:), base.thetaEq);
xiEq = rHEq(1) + traj.xO0;
rWzEq = rHEq(2) + traj.zO0;
uEq = [0; bodyMass * base.g; 0];

% Paper-style interaction wrench moment about the torso CoM:
%   Iyy*ddtheta = (xi-xiEq)*Fz - rWz*Fx + My.
% Centering xi at the full-plant static posture absorbs the leg-mass CoM
% offset omitted by the paper's reduced torso-plus-wheel approximation.
% Linearize it at the upright equilibrium for diagnostics and tests.
A(6, 7) = uEq(2) / base.Iyy;
B(6, 1) = -rWzEq / base.Iyy;
B(6, 3) = 1 / base.Iyy;

model = struct();
model.A = A;
model.B = B;
model.C = eye(8);
model.D = zeros(8, 3);
model.xEq = [base.xEq(:); xiEq; 0];
model.uEq = uEq;
model.stateNames = ["xB"; "zB"; "thetaB"; "dxB"; "dzB"; ...
    "dthetaB"; "xi"; "dxi"];
model.inputNames = ["FHx"; "FHz"; "MBy"];
model.rollingDenominator = rollingDenominator;
model.wheelCount = 2;
model.rWzEq = rWzEq;
model.xiEq = xiEq;
model.m = bodyMass;
model.Iyy = base.Iyy;
model.g = base.g;
end

function y = rotatePitch2D(v, theta)
y = [cos(theta), -sin(theta); sin(theta), cos(theta)] * v(:);
end
