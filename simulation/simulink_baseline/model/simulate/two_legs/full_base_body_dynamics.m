function xdot = full_base_body_dynamics(x, u, model, leg)
%FULL_BASE_BODY_DYNAMICS World-state dynamics driven by body-frame wrench.

P = [1, 0, 0; 0, 0, 1; 0, 1, 0];
angles = x(4:6);
eulerRates = x(10:12);
[R, ~, E, Edot] = controller_attitude_kinematics(angles, eulerRates);
FL = u(1:3);
TL = u(4:6);
FR = u(7:9);
TR = u(10:12);
forceBody = FL + FR;
d = model.halfTrack;
h = model.rWzEq;
xiEq = model.xiEq;
momentBody = [
    d*(FL(3) - FR(3)) - h*(FL(2) + FR(2)) + TL(1) + TR(1);
    (x(13) - xiEq)*FL(3) + (x(14) - xiEq)*FR(3) ...
        - h*(FL(1) + FR(1)) + TL(2) + TR(2);
    -d*FL(1) + d*FR(1) + x(13)*FL(2) + x(14)*FR(2) ...
        + TL(3) + TR(3)
];

forceWorldPhysical = R*(P'*forceBody);
accelerationWorld = P*(forceWorldPhysical/model.m + [0; -model.g; 0]);
omegaPhysical = E*eulerRates;
inertiaBodyPhysical = P'*diag(model.inertia)*P;
inertiaWorld = R*inertiaBodyPhysical*R';
momentWorldPhysical = R*(P'*momentBody);
angularMomentum = inertiaWorld*omegaPhysical;
gyroscopic = [
    omegaPhysical(2)*angularMomentum(3) ...
        - omegaPhysical(3)*angularMomentum(2);
    omegaPhysical(3)*angularMomentum(1) ...
        - omegaPhysical(1)*angularMomentum(3);
    omegaPhysical(1)*angularMomentum(2) ...
        - omegaPhysical(2)*angularMomentum(1)
];
omegaDerivativePhysical = inertiaWorld\(momentWorldPhysical - gyroscopic);
eulerAcceleration = E\(omegaDerivativePhysical - Edot*eulerRates);

% Paper Eq. (12): integrated torso-wheel dynamics in the yaw-aligned
% controller frame. The inputs are the wheel-to-body interaction wrenches,
% so the same forces accelerate the torso and act with opposite sign on the
% wheels. Pure rolling eliminates the ground force and contributes the
% wheel mass/inertia denominator m_w*r + I_w/r. Because FL/FR and TL/TR are
% already expressed in the controller frame, their first force and second
% torque components are the required forward projections.
wheelDenominator = model.rollingDenominator;
baseForwardAcceleration = forceBody(1)/model.m;
xiAccelerationLeft = -baseForwardAcceleration ...
    - (leg.r*FL(1) + TL(2))/wheelDenominator;
xiAccelerationRight = -baseForwardAcceleration ...
    - (leg.r*FR(1) + TR(2))/wheelDenominator;
xdot = [x(7:9); eulerRates; accelerationWorld; eulerAcceleration; ...
    x(15:16); xiAccelerationLeft; xiAccelerationRight];
end
