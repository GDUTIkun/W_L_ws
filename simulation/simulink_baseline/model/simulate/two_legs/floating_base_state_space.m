function model = floating_base_state_space(base)
%FLOATING_BASE_STATE_SPACE Linearize a 2D floating-base wrench model.
%
% State:
%   X = [x; z; theta; dx; dz; dtheta]
%
% Input:
%   U = [FHx; FHz; MBy], the total symmetric two-leg body wrench
%
% FH is the summed force that both wheel legs apply to the body. MBy is the
% summed pure pitch moment. Positive theta and MBy follow the 3D pitch-axis convention used
% in the notes:
%   tau = r_x*FHz - r_z*FHx + MBy.

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end

m = getBaseMass(base);
Iyy = getBaseIyy(base);
g = getFieldOrDefault(base, "g", 9.81);
rHBody = getBaseHipPosition(base);
thetaEq = getFieldOrDefault(base, "thetaEq", 0);

rH = rotatePitch2D(rHBody(:), thetaEq);
rx = rH(1);
rz = rH(2);

uEq = [0; m*g; -rx*m*g];
xEq = getFieldOrDefault(base, "xEq", [0; 0; thetaEq; 0; 0; 0]);
xEq = xEq(:);
if numel(xEq) ~= 6
    error("floating_base_state_space:InvalidEquilibrium", ...
        "base.xEq must be a 6-element vector.");
end

A = zeros(6, 6);
A(1, 4) = 1;
A(2, 5) = 1;
A(3, 6) = 1;

% d/dtheta of tau = r_x(theta)*FHz - r_z(theta)*FHx + MBy.
% With r_x' = -r_z and r_z' = r_x:
%   ktheta = -r_z*FHz_eq - r_x*FHx_eq.
ktheta = -rz*uEq(2) - rx*uEq(1);
A(6, 3) = ktheta / Iyy;

B = zeros(6, 3);
B(4, 1) = 1 / m;
B(5, 2) = 1 / m;
B(6, 1) = -rz / Iyy;
B(6, 2) = rx / Iyy;
B(6, 3) = 1 / Iyy;

model = struct();
model.A = A;
model.B = B;
model.C = eye(6);
model.D = zeros(6, 3);
model.xEq = xEq;
model.uEq = uEq;
model.rHEq = rH;
model.ktheta = ktheta;
model.m = m;
model.Iyy = Iyy;
model.g = g;
model.body = getFieldOrDefault(base, "body", struct());
model.stateNames = ["x"; "z"; "theta"; "dx"; "dz"; "dtheta"];
model.inputNames = ["FHx"; "FHz"; "MBy"];
end

function m = getBaseMass(base)
if isfield(base, "body") && isfield(base.body, "mass")
    m = base.body.mass;
else
    m = getField(base, "m");
end
end

function Iyy = getBaseIyy(base)
if isfield(base, "body") && isfield(base.body, "inertiaIyy")
    Iyy = base.body.inertiaIyy;
elseif isfield(base, "body") && all(isfield(base.body, ...
        ["mass", "lengthX", "heightZ"]))
    Iyy = base.body.mass * ...
        (base.body.lengthX^2 + base.body.heightZ^2) / 12;
else
    Iyy = getField(base, "Iyy");
end
end

function rHBody = getBaseHipPosition(base)
if isfield(base, "body") && isfield(base.body, "hipPositionBody")
    com = [0; 0];
    if isfield(base.body, "comPositionBody")
        com = base.body.comPositionBody(:);
    end
    rHBody = base.body.hipPositionBody(:) - com;
else
    rHBody = getField(base, "rHBody");
end
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 - sin(theta)*rz0;
    sin(theta)*rx0 + cos(theta)*rz0
];
end

function value = getField(s, name)
if ~isfield(s, name)
    error("floating_base_state_space:MissingField", ...
        "Missing base.%s.", name);
end
value = s.(name);
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
