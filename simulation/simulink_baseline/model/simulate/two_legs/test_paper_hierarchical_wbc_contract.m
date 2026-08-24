function test_paper_hierarchical_wbc_contract()
%TEST_PAPER_HIERARCHICAL_WBC_CONTRACT Check paper Eq. (26), (29), (40).

modelDir = fileparts(mfilename("fullpath"));
run(fullfile(modelDir, "startup.m"));
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");
wheelLqr = evalin("base", "wheelLqr");
fullBaseNmpc = evalin("base", "fullBaseNmpc");

% Eq. (26): unavailable lateral-force and roll/yaw-torque inputs are fixed.
inactive = [2, 4, 6, 8, 10, 12];
assert(all(fullBaseNmpc.uMin(inactive) == 0));
assert(all(fullBaseNmpc.uMax(inactive) == 0));

% Eq. (40): after dimensional normalization, physical wheel-torque cost is
% already larger than the hip/knee cost because its allowable torque is lower.
physicalTorqueWeight = ctrl.spatialQpTorqueRegularizationWeight(:) ...
    ./ctrl.spatialQpTorqueScale(:).^2;
expectedRatio = (ctrl.tauMax(1)/ctrl.tauMax(3))^2;
assert(abs(physicalTorqueWeight(3)/physicalTorqueWeight(1) ...
    - expectedRatio) < 1e-12);
assert(physicalTorqueWeight(3) > physicalTorqueWeight(1));

wheelReference = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
fullState = [0; fullBaseNmpc.model.xEq; -traj.zO0];
input = [fullState; leg.q0; leg.dq0; leg.q0; leg.dq0; ...
    fullBaseNmpc.model.uEq; wheelReference];

clear spatial_two_leg_qp_core
[~, baseline] = spatial_two_leg_qp_core(input);
assert(norm(baseline.wrenchFeasible - baseline.wrenchCommand ...
    - baseline.wrenchSlack, inf) < 1e-8, ...
    "WBC slack must satisfy paper Eq. (29) with the implemented sign.");

assignin("base", "ctrl", ctrl);
clear spatial_two_leg_qp_core
fprintf("Paper hierarchical WBC contract checks passed.\n");
end
