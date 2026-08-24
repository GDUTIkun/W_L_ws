function test_wheel_position_coordinate_contract()
%TEST_WHEEL_POSITION_COORDINATE_CONTRACT Verify every measured xi Jacobian.

% The upper state defines xi from the base-relative horizontal wheel
% position. Terrain mapping may rotate the contact basis, but it must not
% change the Jacobian of left, right, common, or differential xi.

modelDir = fileparts(mfilename("fullpath"));
run(fullfile(modelDir, "startup.m"));
base = evalin("base", "base");
leg = evalin("base", "leg");
originalCtrl = evalin("base", "ctrl");
cleanup = onCleanup(@() assignin("base", "ctrl", originalCtrl));

flat = contractSnapshot(base, leg, originalCtrl);
assertContract(flat, "flat");
assert(abs(flat.rollingDirection(2)) < 1e-12, ...
    "Flat contact rolling direction unexpectedly has a vertical component.");

slopeCtrl = originalCtrl;
slopeCtrl.terrainContactMap = struct( ...
    "enabled", true, "slopeAngle", 0.10, ...
    "leadingEdgeX", -10, "upEndX", 10, ...
    "platformEndX", 20, "trailingEdgeX", 30, ...
    "transitionLength", 0);
slope = contractSnapshot(base, leg, slopeCtrl);
assertContract(slope, "mapped_slope_010");
assert(abs(slope.contactPitch - 0.10) < 1e-12, ...
    "The mapped contact frame did not retain its 0.10 rad terrain pitch.");
assert(abs(slope.rollingDirection(2) - sin(0.10)) < 1e-12, ...
    "The mapped rolling basis was changed while repairing wheel-position xi.");

names = ["left", "right", "common", "differential"];
fprintf("Wheel-position coordinate contract (inf-norm error):\n");
for name = names
    fprintf("  %-12s flat %.3e, mapped slope %.3e\n", name, ...
        flat.errorInf.(name), slope.errorInf.(name));
end
clear cleanup
end

function assertContract(snapshot, label)
names = ["left", "right", "common", "differential"];
for name = names
    errorInf = snapshot.errorInf.(name);
    assert(errorInf < 1e-6, ...
        sprintf("%s measured-xi and WBC-Jxi %s contract disagrees: %.6g.", ...
        label, name, errorInf));
end
end

function snapshot = contractSnapshot(base, leg, ctrl)
assignin("base", "ctrl", ctrl);
clear spatial_two_leg_qp_core
[~, audit] = spatial_two_leg_qp_core("contact-audit");
measured = measuredXiGeneralizedJacobians(base, leg, ctrl);
names = ["left", "right", "common", "differential"];
wbc = struct();
errorInf = struct();
for name = names
    wbc.(name) = audit.wheelPositionJacobian.(name);
    errorInf.(name) = norm(measured.(name) - wbc.(name), inf);
end
snapshot = struct( ...
    "measured", measured, "wbc", wbc, "errorInf", errorInf, ...
    "contactPitch", audit.contactPitchEstimate, ...
    "rollingDirection", audit.rollingDirection);
end

function jacobian = measuredXiGeneralizedJacobians(base, leg, ctrl)
step = 1e-7;
left = zeros(1, 12);
right = zeros(1, 12);
qNominal = [zeros(6, 1); leg.q0(:); leg.q0(:)];
for coordinate = 1:12
    qPlus = qNominal;
    qMinus = qNominal;
    qPlus(coordinate) = qPlus(coordinate) + step;
    qMinus(coordinate) = qMinus(coordinate) - step;
    xiPlus = measuredXi(base, leg, ctrl, qPlus);
    xiMinus = measuredXi(base, leg, ctrl, qMinus);
    gradient = (xiPlus - xiMinus)/(2*step);
    left(coordinate) = gradient(1);
    right(coordinate) = gradient(2);
end
jacobian = struct( ...
    "left", left, ...
    "right", right, ...
    "common", 0.5*(left + right), ...
    "differential", 0.5*(left - right));
end

function xi = measuredXi(base, leg, ctrl, q)
planarBase = [q(1); q(2); q(5); zeros(3, 1)];
rollYaw = [q(4); q(6); 0; 0];
lateral = [q(3); 0];
input = [0; planarBase; rollYaw; lateral; ...
    q(7:9); zeros(3, 1); q(10:12); zeros(3, 1)];
output = full_base_nmpc_state_signal(input, base, leg, ctrl);
xi = output(14:15);
end
