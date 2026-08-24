function varargout = wheel_leg_dynamics(q, dq, leg, term)
%WHEEL_LEG_DYNAMICS Fixed-hip planar two-link wheel-leg dynamics.
%
% Returns M(q), C(q,dq), and G(q) for:
%   M(q)*ddq + C(q,dq) + G(q) = tau + J_H(q)'*F_H + J_c(q)'*F_c
%
% q = [qh; qk; qw]. The wheel absolute angular velocity is:
%   dqh + dqk + dqw

if nargin < 3 || isempty(leg)
    leg = evalin("base", "leg");
end

if nargin < 4
    term = "all";
end

q = double(q(:));
dq = double(dq(:));

if numel(q) ~= 3 || numel(dq) ~= 3
    error("wheel_leg_dynamics:InvalidState", ...
        "Expected q and dq to be 3-element vectors.");
end

qh = q(1);
qk = q(2);
dqh = dq(1);
dqk = dq(2);

L1 = leg.L1;
L2 = leg.L2;
c1 = leg.c1;
c2 = leg.c2;
m1 = leg.m1;
m2 = leg.m2;
mw = leg.mw;
I1 = leg.I1;
I2 = leg.I2;
Iw = leg.Iw;
g = leg.g;

cos_qk = cos(qk);
sin_qk = sin(qk);

M11 = I1 + I2 + Iw + m1*c1^2 ...
    + m2*(L1^2 + c2^2 + 2*L1*c2*cos_qk) ...
    + mw*(L1^2 + L2^2 + 2*L1*L2*cos_qk);
M12 = I2 + Iw + m2*(c2^2 + L1*c2*cos_qk) ...
    + mw*(L2^2 + L1*L2*cos_qk);
M22 = I2 + Iw + m2*c2^2 + mw*L2^2;
M13 = Iw;
M23 = Iw;
M33 = Iw;
M = [
    M11, M12, M13;
    M12, M22, M23;
    M13, M23, M33
];

h = (m2*L1*c2 + mw*L1*L2) * sin_qk;
C = [
    -h * (2*dqh*dqk + dqk^2);
     h * dqh^2;
     0
];

G = [
    g*((m1*c1 + m2*L1 + mw*L1)*sin(qh) ...
        + (m2*c2 + mw*L2)*sin(qh + qk));
    g*(m2*c2 + mw*L2)*sin(qh + qk);
    0
];

switch string(term)
    case "M"
        varargout = {M};
    case "C"
        varargout = {C};
    case "G"
        varargout = {G};
    otherwise
        varargout = {M, C, G};
end
end
