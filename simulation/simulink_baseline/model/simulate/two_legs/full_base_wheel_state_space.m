function model = full_base_wheel_state_space(base, leg, traj)
%FULL_BASE_WHEEL_STATE_SPACE 16-state, 12-input 8-DoF upper model.
%
% State: [pB(3); roll; pitch; yaw; vB(3); EulerRate(3); xiL; xiR; dxiL; dxiR]
% Input: [FL(3); TL(3); FR(3); TR(3)] in the current body-aligned
%        controller frame, where X is forward, Y is lateral, and Z is
%        vertical. Position and translational velocity remain world-frame
%        measurements; attitude rates use local Euler coordinates so their
%        meaning and feedback weights remain invariant under large yaw.

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end
if nargin < 2 || isempty(leg)
    leg = evalin("base", "leg");
end
if nargin < 3 || isempty(traj)
    traj = evalin("base", "traj");
end

planar = base_wheel_state_space(base, leg, traj);
mass = planar.m;
halfTrack = abs(base.body.hipPositionBodyLeft3D(3));
inertia = [base.Iroll; base.Iyy; base.Iyaw];
xiEq = planar.xiEq;
rVertical = planar.rWzEq;
wheelDenominator = leg.mw*leg.r + leg.Iw/leg.r;

A = zeros(16, 16);
B = zeros(16, 12);
A(1:6, 7:12) = eye(6);
A(13, 15) = 1;
A(14, 16) = 1;

forceLeft = 1:3;
torqueLeft = 4:6;
forceRight = 7:9;
torqueRight = 10:12;
B(7:9, forceLeft) = eye(3)/mass;
B(7:9, forceRight) = eye(3)/mass;

% Roll and yaw use the physical left/right lever arms. Pitch preserves the
% sign convention of the already-validated planar NMPC model.
B(10, forceLeft) = [0, -rVertical, halfTrack]/inertia(1);
B(10, torqueLeft(1)) = 1/inertia(1);
B(10, forceRight) = [0, -rVertical, -halfTrack]/inertia(1);
B(10, torqueRight(1)) = 1/inertia(1);
A(11, 13:14) = planar.uEq(2)/(2*inertia(2));
B(11, forceLeft) = [-rVertical, 0, 0]/inertia(2);
B(11, torqueLeft(2)) = 1/inertia(2);
B(11, forceRight) = [-rVertical, 0, 0]/inertia(2);
B(11, torqueRight(2)) = 1/inertia(2);
B(12, forceLeft) = [-halfTrack, xiEq, 0]/inertia(3);
B(12, torqueLeft(3)) = 1/inertia(3);
B(12, forceRight) = [halfTrack, xiEq, 0]/inertia(3);
B(12, torqueRight(3)) = 1/inertia(3);

% Paper Eq. (12): each relative wheel position contains the common torso
% acceleration and its own rolling-constrained wheel acceleration. This is
% the full left/right model; no empirical differential gain is applied.
baseAcceleration = -1/mass;
wheelForceGain = -leg.r/wheelDenominator;
wheelTorqueGain = -1/wheelDenominator;
B(15, forceLeft(1)) = baseAcceleration + wheelForceGain;
B(15, forceRight(1)) = baseAcceleration;
B(16, forceLeft(1)) = baseAcceleration;
B(16, forceRight(1)) = baseAcceleration + wheelForceGain;
B(15, torqueLeft(2)) = wheelTorqueGain;
B(16, torqueRight(2)) = wheelTorqueGain;

xEq = zeros(16, 1);
xEq([1, 3, 5, 7, 9, 11]) = planar.xEq(1:6);
xEq(13:14) = xiEq;
uEq = zeros(12, 1);
uEq([3, 9]) = mass*base.g/2;
gravity = zeros(16, 1);
gravity(9) = -base.g;
gravity(11) = -planar.uEq(2)*xiEq/inertia(2);

model = struct("A", A, "B", B, "C", eye(16), ...
    "D", zeros(16, 12), "xEq", xEq, "uEq", uEq, ...
    "gravity", gravity, "m", mass, "inertia", inertia, ...
    "g", base.g, ...
    "halfTrack", halfTrack, "rWzEq", rVertical, "xiEq", xiEq, ...
    "rollingDenominator", wheelDenominator, ...
    "wheelMass", leg.mw, "wheelRadius", leg.r, ...
    "wheelAxleInertia", leg.Iw, ...
    "wheelRelativeDynamics", "paper_eq12", ...
    "dynamicsVersion", 7, ...
    "inputFrame", "body_aligned", "stateFrame", "world_euler_rate");
model.stateNames = ["X"; "Y"; "Z"; "roll"; "pitch"; "yaw"; ...
    "vX"; "vY"; "vZ"; "rollRate"; "pitchRate"; "yawRate"; ...
    "xiL"; "xiR"; "dxiL"; "dxiR"];
model.inputNames = ["FLx"; "FLy"; "FLz"; "TLx"; "TLy"; "TLz"; ...
    "FRx"; "FRy"; "FRz"; "TRx"; "TRy"; "TRz"];
end
