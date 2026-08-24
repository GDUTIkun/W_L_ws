function design = wheel_position_lqr_design(base, leg, traj)
%WHEEL_POSITION_LQR_DESIGN Height-scheduled WIPM-LQR wheel planner.

rHEq = rotatePitch2D(base.rHBody(:), base.thetaEq);
neutral = rHEq(1) + traj.xO0;
heightNominal = max(0.1, -(rHEq(2) + traj.zO0));

qkMin = min(max(traj.wheelPositionKneeMin, 0), pi);
maxReach = sqrt(leg.L1^2 + leg.L2^2 ...
    + 2 * leg.L1 * leg.L2 * cos(qkMin));
xOHMax = sqrt(max(0, maxReach^2 - traj.zO0^2));

design = struct();
design.Q = diag(traj.wheelLqrQ(:));
design.R = traj.wheelLqrR;
design.heightGrid = linspace(max(0.1, heightNominal - 0.15), ...
    heightNominal + 0.15, 9);
design.K = zeros(numel(design.heightGrid), 2);
for idx = 1:numel(design.heightGrid)
    B = [0; -base.g / design.heightGrid(idx)];
    design.K(idx, :) = lqr([0, 1; 0, 0], B, design.Q, design.R);
end
design.heightNominal = heightNominal;
design.positionMin = rHEq(1) - xOHMax;
design.positionMax = rHEq(1) + xOHMax;
design.neutral = min(max(neutral, design.positionMin), design.positionMax);
design.frequencyHz = traj.wheelPositionFrequencyHz;
design.damping = traj.wheelPositionDamping;
design.velocityMax = traj.wheelPositionVelocityMax;
design.accelerationMax = traj.wheelPositionAccelerationMax;
design.Ts = base.Ts;
end

function y = rotatePitch2D(v, theta)
y = [cos(theta), -sin(theta); sin(theta), cos(theta)] * v(:);
end
