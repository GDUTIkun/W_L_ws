function y = wheel_position_state_signal(x, base, leg, ctrl)
%WHEEL_POSITION_STATE_SIGNAL Build the measured common-mode upper state.
%
% Input:  [t; baseState(6); qLeft(3); dqLeft(3); qRight(3); dqRight(3)]
% Output: [t; baseState(6); xi; dxi; height]

if nargin < 2 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 3 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 4 || isempty(ctrl)
    ctrl = evalin("base", "ctrl");
end

x = double(x(:));
if numel(x) ~= 19
    error("wheel_position_state_signal:InvalidInput", ...
        "Expected [t; baseState(6); qLeft(3); dqLeft(3); qRight(3); dqRight(3)].");
end

t = x(1);
baseState = x(2:7);
qRelative = [x(8:10), x(14:16)];
dqRelative = [x(11:13), x(17:19)];
pitchSign = ctrl.basePitchToAbsHipSign;
q = qRelative;
dq = dqRelative;
q(1, :) = q(1, :) + pitchSign * baseState(3);
dq(1, :) = dq(1, :) + pitchSign * baseState(6);
kinLeft = wheel_leg_kinematics(q(:, 1), dq(:, 1), [], leg);
kinRight = wheel_leg_kinematics(q(:, 2), dq(:, 2), [], leg);

rH = rotatePitch2D(base.rHBody(:), baseState(3));
drH = baseState(6) * [-rH(2); rH(1)];
rWheel = [rH + kinLeft.pO, rH + kinRight.pO];
vWheelRelative = [drH + kinLeft.vO, drH + kinRight.vO];
commonWheel = mean(rWheel, 2);
commonVelocity = mean(vWheelRelative, 2);
height = max(1e-3, -commonWheel(2));
y = [t; baseState; commonWheel(1); commonVelocity(1); height];
end

function y = rotatePitch2D(v, theta)
y = [cos(theta), -sin(theta); sin(theta), cos(theta)] * v(:);
end
