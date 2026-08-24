function test_paper_wheel_relative_dynamics()
%TEST_PAPER_WHEEL_RELATIVE_DYNAMICS Verify the paper Eq. (12) contract.

startup;
base = evalin("base", "base");
leg = evalin("base", "leg");
model = full_base_wheel_state_space(base, leg, evalin("base", "traj"));

assert(model.dynamicsVersion == 7);
assert(model.wheelRelativeDynamics == "paper_eq12");
assert(~isfield(model, "differentialRollingGain"));
assert(abs(model.rollingDenominator ...
    - (leg.mw*leg.r + leg.Iw/leg.r)) < 1e-12);

x = model.xEq;
u = zeros(12, 1);
u(1) = 7.5;
u(5) = -0.8;
u(7) = -3.0;
u(11) = 0.35;
xdot = full_base_body_dynamics(x, u, model, leg);

baseForwardAcceleration = (u(1) + u(7))/model.m;
expectedLeft = -baseForwardAcceleration ...
    - (leg.r*u(1) + u(5))/model.rollingDenominator;
expectedRight = -baseForwardAcceleration ...
    - (leg.r*u(7) + u(11))/model.rollingDenominator;
assert(abs(xdot(15) - expectedLeft) < 1e-12);
assert(abs(xdot(16) - expectedRight) < 1e-12);
assert(norm(model.B(15:16, :)*u - xdot(15:16), inf) < 1e-12);

expectedDifferential = ( ...
    leg.r*(u(1) - u(7)) + u(5) - u(11)) ...
    /(2*model.rollingDenominator);
actualDifferential = 0.5*(xdot(16) - xdot(15));
assert(abs(actualDifferential - expectedDifferential) < 1e-12);

uCommon = zeros(12, 1);
uCommon([1, 7]) = 4;
uCommon([5, 11]) = -0.2;
xdotCommon = full_base_body_dynamics(x, uCommon, model, leg);
assert(abs(xdotCommon(16) - xdotCommon(15)) < 1e-12);

fprintf("Paper Eq. (12) wheel-relative dynamics checks passed.\n");
end
