function dX = floating_base_nonlinear_state(~, X, u, base)
%FLOATING_BASE_NONLINEAR_STATE Nonlinear 2D floating-base state equation.
%
% X = [x; z; theta; dx; dz; dtheta]
% u = [FHx; FHz; MBy], total force/moment applied by both legs to the body.

if nargin < 4 || isempty(base)
    base = evalin("base", "base");
end

X = double(X(:));
u = double(u(:));
if numel(X) ~= 6 || numel(u) ~= 3
    error("floating_base_nonlinear_state:InvalidInput", ...
        "Expected X to have 6 elements and u to have 3 elements.");
end

m = getBaseMass(base);
Iyy = getBaseIyy(base);
g = base.g;
rH = rotatePitch2D(getBaseHipPosition(base), X(3));

FHx = u(1);
FHz = u(2);
MBy = u(3);

dX = zeros(6, 1);
dX(1:3) = X(4:6);
dX(4) = FHx / m;
dX(5) = FHz / m - g;
dX(6) = (rH(1)*FHz - rH(2)*FHx + MBy) / Iyy;
end

function m = getBaseMass(base)
if isfield(base, "body") && isfield(base.body, "mass")
    m = base.body.mass;
else
    m = base.m;
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
    Iyy = base.Iyy;
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
    rHBody = base.rHBody(:);
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
