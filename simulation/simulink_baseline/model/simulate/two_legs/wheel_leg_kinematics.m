function kin = wheel_leg_kinematics(q, dq, ddq, leg)
%WHEEL_LEG_KINEMATICS Kinematics and contact Jacobians for one wheel-leg.
%
% q = [qh; qk; qw]. The contact point velocity convention is:
%   vcx = xdot_O + r*(dqh + dqk + dqw)
%   vcz = zdot_O

if nargin < 4 || isempty(leg)
    leg = evalin("base", "leg");
end

q = double(q(:));
if nargin < 2 || isempty(dq)
    dq = zeros(3, 1);
else
    dq = double(dq(:));
end
if nargin < 3 || isempty(ddq)
    ddq = zeros(3, 1);
else
    ddq = double(ddq(:));
end

if numel(q) ~= 3 || numel(dq) ~= 3 || numel(ddq) ~= 3
    error("wheel_leg_kinematics:InvalidState", ...
        "Expected q, dq, and ddq to be 3-element vectors.");
end

qh = q(1);
qk = q(2);
dqh = dq(1);
dqk = dq(2);

L1 = leg.L1;
L2 = leg.L2;
r = leg.r;

qhk = qh + qk;
dqhk = dqh + dqk;

pO = [
    L1*sin(qh) + L2*sin(qhk);
   -L1*cos(qh) - L2*cos(qhk)
];

JO = [
    L1*cos(qh) + L2*cos(qhk), L2*cos(qhk), 0;
    L1*sin(qh) + L2*sin(qhk), L2*sin(qhk), 0
];

dJO = [
    -L1*sin(qh)*dqh - L2*sin(qhk)*dqhk, -L2*sin(qhk)*dqhk, 0;
     L1*cos(qh)*dqh + L2*cos(qhk)*dqhk,  L2*cos(qhk)*dqhk, 0
];

vO = JO * dq;
aO = JO * ddq + dJO * dq;

JH = -JO;

Jc = [
    JO(1, 1) + r, JO(1, 2) + r, r;
    JO(2, 1),     JO(2, 2),     0
];

dJc = [
    dJO(1, 1), dJO(1, 2), 0;
    dJO(2, 1), dJO(2, 2), 0
];

kin = struct();
kin.pO = pO;
kin.vO = vO;
kin.aO = aO;
kin.JO = JO;
kin.dJO = dJO;
kin.JH = JH;
kin.Jc = Jc;
kin.dJc = dJc;
kin.vc = Jc * dq;
kin.ac = Jc * ddq + dJc * dq;
kin.wheelAbsoluteAngle = qh + qk + q(3);
kin.wheelAbsoluteRate = sum(dq);
end
