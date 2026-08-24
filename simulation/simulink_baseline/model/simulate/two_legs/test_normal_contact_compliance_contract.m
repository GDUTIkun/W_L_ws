function test_normal_contact_compliance_contract()
%TEST_NORMAL_CONTACT_COMPLIANCE_CONTRACT Verify oracle penetration signs.

modelDir = fileparts(mfilename("fullpath"));
run(fullfile(modelDir, "startup.m"));
leg = evalin("base", "leg");
originalCtrl = evalin("base", "ctrl");
cleanup = onCleanup(@() restoreCtrl(originalCtrl));

ctrl = originalCtrl;
ctrl.normalContactCompliance = struct( ...
    "enabled", true, ...
    "frequencyHz", 1.72648796001821, ...
    "dampingRatio", 1.0, ...
    "penetrationReference", ctrl.commonModeContactPreload);
q0 = [zeros(6, 1); leg.q0(:); leg.q0(:)];
dq0 = zeros(12, 1);
nominal = snapshot(ctrl, q0, dq0);
assert(max(abs(nominal.normalContactCompliance.penetration ...
    - ctrl.commonModeContactPreload)) < 1e-10, ...
    "Nominal oracle penetration does not equal the configured preload.");
assert(norm(nominal.contactAccelerationTarget, inf) < 1e-9, ...
    "Nominal penetration produced a nonzero compliant-normal target.");

down = q0;
down(2) = down(2) - 1e-3;
downAudit = snapshot(ctrl, down, dq0);
expectedPositionTarget = ...
    downAudit.normalContactCompliance.Kp*1e-3;
assert(max(abs(downAudit.contactAccelerationTarget([3, 6]) ...
    - expectedPositionTarget)) < 1e-9, ...
    "Positive penetration error did not command positive normal acceleration.");

dqDown = dq0;
dqDown(2) = -0.02;
rateAudit = snapshot(ctrl, q0, dqDown);
expectedRateTarget = rateAudit.normalContactCompliance.Kd*0.02;
assert(max(abs(rateAudit.contactAccelerationTarget([3, 6]) ...
    - expectedRateTarget)) < 1e-9, ...
    "Increasing penetration did not command positive normal acceleration.");

disabled = ctrl;
disabled.normalContactCompliance.enabled = false;
disabledAudit = snapshot(disabled, down, dqDown);
assert(norm(disabledAudit.contactAccelerationTarget, inf) == 0, ...
    "Disabled compliant-normal mode changed the N0 target.");

fprintf("Normal compliance contract passed: delta>0 and deltaDot>0 " + ...
    "command positive separating acceleration; nominal preload is zero error.\n");
clear cleanup
end

function audit = snapshot(ctrl, q, dq)
ctrl.contactAuditQ = q;
ctrl.contactAuditDq = dq;
assignin("base", "ctrl", ctrl);
clear spatial_two_leg_qp_core
[~, audit] = spatial_two_leg_qp_core("contact-audit");
end

function restoreCtrl(ctrl)
assignin("base", "ctrl", ctrl);
clear spatial_two_leg_qp_core
end
