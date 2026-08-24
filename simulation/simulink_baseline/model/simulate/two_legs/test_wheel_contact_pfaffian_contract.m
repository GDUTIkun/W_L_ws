function results = test_wheel_contact_pfaffian_contract()
%TEST_WHEEL_CONTACT_PFAFFIAN_CONTRACT Audit all three wheel contact rows.
%
% The paper-reduced implementation is compared with Eq. (4) evaluated at
% the terrain-normal nominal contact point Upsilon = -r*n.  The latter is
% the full 12-DoF material-point distribution
%   g = [t l n]'*(Jv + r*[n]x*Jw).

modelDir = fileparts(mfilename("fullpath"));
run(fullfile(modelDir, "startup.m"));
leg = evalin("base", "leg");
originalCtrl = evalin("base", "ctrl");
cleanup = onCleanup(@() restoreCtrl(originalCtrl));

q0 = [zeros(6, 1); leg.q0(:); leg.q0(:)];
cases = struct("name", {}, "q", {}, "dq", {}, "ctrl", {});

dq = zeros(12, 1);
dq(9) = 2.5;
cases(end + 1) = makeCase("wheel_spin_only", q0, dq, originalCtrl);

dq = zeros(12, 1);
dq(1:3) = [0.4; -0.1; 0.2];
cases(end + 1) = makeCase("base_translation_only", q0, dq, originalCtrl);

dq = zeros(12, 1);
dq(1) = 0.35;
dq([9, 12]) = -0.35/leg.r;
cases(end + 1) = makeCase("pure_rolling", q0, dq, originalCtrl);

dq = zeros(12, 1);
dq(3) = 0.3;
cases(end + 1) = makeCase("lateral_motion", q0, dq, originalCtrl);

dq = zeros(12, 1);
dq(2) = 0.25;
cases(end + 1) = makeCase("normal_motion", q0, dq, originalCtrl);

qAttitude = q0;
qAttitude(4:6) = [0.12; -0.08; 0.16];
dq = zeros(12, 1);
dq(4:6) = [0.2; -0.3; 0.15];
cases(end + 1) = makeCase("yaw_pitch_roll", ...
    qAttitude, dq, originalCtrl);

slopeCtrl = uniformSlopeCtrl(originalCtrl, 0.10);
qSlope = q0;
qSlope(4:6) = [0.12; 0.08; -0.10];
dq = [0.15; -0.04; 0.07; 0.18; -0.22; 0.11; ...
    0.10; -0.16; 0.35; -0.09; 0.14; -0.28];
cases(end + 1) = makeCase("oracle_normal_010", ...
    qSlope, dq, slopeCtrl);

rowCount = numel(cases);
name = strings(rowCount, 1);
materialError = zeros(rowCount, 1);
legacyMismatch = zeros(rowCount, 1);
rollingMismatch = zeros(rowCount, 1);
lateralMismatch = zeros(rowCount, 1);
normalMismatch = zeros(rowCount, 1);
snapshots = cell(rowCount, 1);
for index = 1:rowCount
    item = cases(index);
    audit = snapshot(item.ctrl, item.q, item.dq);
    snapshots{index} = audit;
    [materialError(index), componentMismatch] = ...
        materialPointError(audit, item.dq);
    delta = (audit.pfaffian.legacy ...
        - audit.pfaffian.materialPoint)*item.dq;
    legacyMismatch(index) = norm(delta, inf);
    delta = reshape(delta, 3, 2);
    rollingMismatch(index) = max(abs(delta(1, :)));
    lateralMismatch(index) = max(abs(delta(2, :)));
    normalMismatch(index) = max(abs(delta(3, :)));
    assert(materialError(index) < 1e-10, ...
        "%s material-point row mismatch: %.6g", ...
        item.name, materialError(index));
    assert(max(componentMismatch, [], "all") < 1e-10, ...
        "%s per-direction material-point mismatch: %.6g", ...
        item.name, max(componentMismatch));
    name(index) = item.name;
end

spin = snapshots{1};
spinVelocity = spin.pfaffian.materialPoint(1:3, :)*cases(1).dq;
assert(abs(spinVelocity(1) - leg.r*cases(1).dq(9)) < 1e-12, ...
    "Wheel spin is not present in the rolling material-point row.");
assert(norm(spinVelocity(2:3), inf) < 1e-12, ...
    "Nominal wheel spin leaked into lateral or normal velocity.");

pureRollingVelocity = snapshots{3}.pfaffian.materialPoint*cases(3).dq;
assert(norm(pureRollingVelocity, inf) < 1e-10, ...
    "Flat nominal pure rolling did not reduce to zero contact velocity.");
assert(legacyMismatch(1) < 1e-12 && legacyMismatch(3) < 1e-12, ...
    "Legacy rows do not reproduce the paper's nominal spin-only limit.");

lateralVelocity = reshape( ...
    snapshots{4}.pfaffian.materialPoint*cases(4).dq, 3, 2);
normalVelocity = reshape( ...
    snapshots{5}.pfaffian.materialPoint*cases(5).dq, 3, 2);
assert(norm(lateralVelocity - repmat([0; 0.3; 0], 1, 2), inf) < 1e-12, ...
    "Lateral translation did not project exclusively into g_l.");
assert(norm(normalVelocity - repmat([0; 0; 0.25], 1, 2), inf) < 1e-12, ...
    "Normal translation did not project exclusively into g_n.");

assert(legacyMismatch(6) > 1e-4, ...
    "Attitude-rate test did not expose the reduced-row mismatch.");
assert(legacyMismatch(7) > 1e-4, ...
    "0.10 rad oracle-normal test did not expose the reduced-row mismatch.");

[legacyBiasError, materialBiasError] = directionalDerivativeCheck( ...
    slopeCtrl, qSlope, dq);
assert(legacyBiasError < 1e-7, ...
    "Legacy dot(g)*dq directional derivative mismatch: %.6g", ...
    legacyBiasError);
assert(materialBiasError < 1e-7, ...
    "Material-point dot(g)*dq directional derivative mismatch: %.6g", ...
    materialBiasError);

candidateCtrl = slopeCtrl;
candidateCtrl.materialPointContactKinematicsEnabled = true;
candidate = snapshot(candidateCtrl, qSlope, dq);
assert(candidate.pfaffian.selectedUsesMaterialPoint, ...
    "Candidate mode was not selected by the production contact path.");
assert(norm(candidate.contactJacobian ...
    - candidate.pfaffian.materialPoint, inf) < 1e-12, ...
    "Candidate production path does not use the audited material-point rows.");

results = table(name, materialError, legacyMismatch, ...
    rollingMismatch, lateralMismatch, normalMismatch);
results.legacyBiasError(:) = legacyBiasError;
results.materialPointBiasError(:) = materialBiasError;
disp(results);
fprintf("dot(g)*dq errors: legacy %.3e, material point %.3e\n", ...
    legacyBiasError, materialBiasError);
clear cleanup
end

function item = makeCase(name, q, dq, ctrl)
item = struct("name", string(name), "q", q, "dq", dq, "ctrl", ctrl);
end

function ctrl = uniformSlopeCtrl(ctrl, angle)
ctrl.terrainContactMap = struct( ...
    "enabled", true, "slopeAngle", angle, ...
    "leadingEdgeX", -10, "upEndX", 10, ...
    "platformEndX", 20, "trailingEdgeX", 30, ...
    "transitionLength", 0);
end

function audit = snapshot(ctrl, q, dq)
ctrl.contactAuditQ = q;
ctrl.contactAuditDq = dq;
assignin("base", "ctrl", ctrl);
clear spatial_two_leg_qp_core
[~, audit] = spatial_two_leg_qp_core("contact-audit");
end

function [errorInf, componentError] = materialPointError(audit, dq)
componentError = zeros(3, 2);
for side = 1:2
    rows = 3*side - 2:3*side;
    centerVelocity = audit.wheelCenterJacobian(:, :, side)*dq;
    angularVelocity = audit.wheelAngularJacobian(:, :, side)*dq;
    pointVelocity = centerVelocity ...
        + cross(angularVelocity, audit.contactPointOffset(:, side));
    expected = audit.contactBasis(:, :, side)'*pointVelocity;
    actual = audit.pfaffian.materialPoint(rows, :)*dq;
    componentError(:, side) = abs(actual - expected);
end
errorInf = max(componentError, [], "all");
end

function [legacyError, materialPointError] = ...
        directionalDerivativeCheck(ctrl, q, dq)
center = snapshot(ctrl, q, dq);
epsilon = 1e-6/max(1, norm(dq, inf));
plus = snapshot(ctrl, q + epsilon*dq, dq);
minus = snapshot(ctrl, q - epsilon*dq, dq);
legacyExpected = ((plus.pfaffian.legacy ...
    - minus.pfaffian.legacy)/(2*epsilon))*dq;
legacyIndependent = ((reconstructPfaffian(plus, false) ...
    - reconstructPfaffian(minus, false))/(2*epsilon))*dq;
materialExpected = ((reconstructPfaffian(plus, true) ...
    - reconstructPfaffian(minus, true))/(2*epsilon))*dq;
assert(norm(legacyExpected - legacyIndependent, inf) < 1e-10, ...
    "Independent legacy distribution reconstruction disagrees.");
legacyError = norm(center.pfaffian.legacyBias - legacyExpected, inf);
materialPointError = norm( ...
    center.pfaffian.materialPointBias - materialExpected, inf);
end

function matrix = reconstructPfaffian(audit, materialPointEnabled)
matrix = zeros(6, 12);
for side = 1:2
    rows = 3*side - 2:3*side;
    basis = audit.contactBasis(:, :, side);
    Jv = audit.wheelCenterJacobian(:, :, side);
    Jw = audit.wheelAngularJacobian(:, :, side);
    if materialPointEnabled
        rho = audit.contactPointOffset(:, side);
        pointJacobian = Jv - skew(rho)*Jw;
        matrix(rows, :) = basis'*pointJacobian;
    else
        matrix(rows(1), :) = basis(:, 1)'*Jv ...
            + audit.wheelRadius*audit.axleDirection(:, side)'*Jw;
        matrix(rows(2), :) = basis(:, 2)'*Jv;
        matrix(rows(3), :) = basis(:, 3)'*Jv;
    end
end
end

function matrix = skew(vector)
matrix = [0, -vector(3), vector(2); ...
    vector(3), 0, -vector(1); ...
    -vector(2), vector(1), 0];
end

function restoreCtrl(originalCtrl)
assignin("base", "ctrl", originalCtrl);
clear spatial_two_leg_qp_core
end
