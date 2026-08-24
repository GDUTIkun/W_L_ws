function [R, dR, E, Edot] = controller_attitude_kinematics(angles, angleRates)
%CONTROLLER_ATTITUDE_KINEMATICS Large-yaw, local-roll/pitch frame map.
%
% Physical Simscape axes are X forward, Y vertical, Z lateral.  The
% orientation is R = Ry(yaw)*Rz(pitch)*Rx(roll), so yaw may accumulate
% freely while roll and pitch remain local small-angle coordinates.

roll = angles(1);
pitch = angles(2);
yaw = angles(3);
Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
Ry = [cos(yaw), 0, sin(yaw); 0, 1, 0; -sin(yaw), 0, cos(yaw)];
Rz = [cos(pitch), -sin(pitch), 0; ...
    sin(pitch), cos(pitch), 0; 0, 0, 1];
dRx = [0, 0, 0; 0, -sin(roll), -cos(roll); ...
    0, cos(roll), -sin(roll)];
dRy = [-sin(yaw), 0, cos(yaw); 0, 0, 0; ...
    -cos(yaw), 0, -sin(yaw)];
dRz = [-sin(pitch), -cos(pitch), 0; ...
    cos(pitch), -sin(pitch), 0; 0, 0, 0];
R = Ry*Rz*Rx;
dR = cat(3, Ry*Rz*dRx, Ry*dRz*Rx, dRy*Rz*Rx);
E = [Ry*Rz*[1; 0; 0], Ry*[0; 0; 1], [0; 1; 0]];
if nargin < 2 || isempty(angleRates)
    Edot = 0*E;
else
    angleRates = angleRates(:);
    zeroColumn = 0*angles;
    dE_dPitch = [Ry*dRz*[1; 0; 0], zeroColumn, zeroColumn];
    dE_dYaw = [dRy*Rz*[1; 0; 0], dRy*[0; 0; 1], zeroColumn];
    Edot = angleRates(2)*dE_dPitch + angleRates(3)*dE_dYaw;
end
end
