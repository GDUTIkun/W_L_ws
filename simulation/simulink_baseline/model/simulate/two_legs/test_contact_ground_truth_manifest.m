function test_contact_ground_truth_manifest()
%TEST_CONTACT_GROUND_TRUTH_MANIFEST Phase-6 G1 audit-contract checks.

testManifestShape();
testSelectedSignalProvenance();
testCurrentModelAudit();
testProxyExclusion();
testCanonicalAndLegacySignContract();
testDownstreamLoggingContract();
fprintf("Contact ground-truth manifest checks passed.\n");
end

function testManifestShape()
m = contact_ground_truth_manifest();
required = ["schemaVersion", "canonicalConvention", "legacyMapping", ...
    "selectedSignals", "unavailableSignals", "proxySignals", ...
    "sourceClasses", "loggingContract", "scopeFences", "escalation"];
assert(all(isfield(m, required)));
assert(m.status == "ready_for_g2");
classes = string({m.sourceClasses.name});
assert(all(ismember(["plant_ground_truth", "controller_proxy", ...
    "derived_kinematic_proxy"], classes)));
assert(m.escalation.status == "ready_for_g2");
assert(isempty(m.escalation.blockers));
end

function testSelectedSignalProvenance()
m = contact_ground_truth_manifest();
required = ["sourceBlock", "sourcePort", "sensingParameter", ...
    "physicalMeaning", "unit", "frame", "sign", "sampleTime", ...
    "sideOwnership", "sourceClass", "observationOnly"];
assert(numel(m.selectedSignals) == 12);
for item = m.selectedSignals
    assert(all(isfield(item, required)));
    for field = required(1:end - 1)
        assert(strlength(string(item.(field))) > 0, ...
            "Selected signal %s lacks %s.", item.name, field);
    end
    assert(item.sourceClass == "plant_ground_truth");
    assert(item.observationOnly);
end
assert(nnz(string({m.selectedSignals.side}) == "left") == 6);
assert(nnz(string({m.selectedSignals.side}) == "right") == 6);
end

function testCurrentModelAudit()
m = contact_ground_truth_manifest();
model = "source";
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(model);
end
cleanup = onCleanup(@() closeIfOpened(model, wasLoaded));
for item = m.selectedSignals
    assert(getSimulinkBlockHandle(item.sourceBlock) ~= -1, ...
        "Audited source block is missing: %s", item.sourceBlock);
    parameters = get_param(item.sourceBlock, "DialogParameters");
    assert(isfield(parameters, item.sensingParameter), ...
        "Sensing parameter %s is unavailable.", item.sensingParameter);
    assert(string(get_param(item.sourceBlock, item.sensingParameter)) == "off", ...
        "G1 expects baseline sensing to remain off.");
end
blocks = string({m.inspection.blocks.path});
assert(numel(unique(blocks)) == 2);
assert(all(get_param(model, "Dirty") == "off"));
clear cleanup
end

function testProxyExclusion()
m = contact_ground_truth_manifest();
requiredNames = ["normalLoadDifference", "contactForceLeftRight", ...
    "rollingResidual", "wheelAngularQuantities", ...
    "wheelPeripheralQuantities", "wheelCenterTangentQuantities"];
names = string({m.proxySignals.name});
assert(all(ismember(requiredNames, names)));
assert(all(~[m.proxySignals.mayServeAsContactPhysicalState]));
assert(all(string({m.proxySignals.allowedRole}) == "diagnostic_only"));
assert(all(string({m.proxySignals.claimedPhysicalQuantity}) == "none"));
assert(~m.loggingContract.proxySubstitutionAllowed);
end

function testCanonicalAndLegacySignContract()
m = contact_ground_truth_manifest();
assert(m.canonicalConvention.differential == "(right-left)/2");
assert(m.canonicalConvention.commonMode == "(right+left)/2");
assert(m.legacyMapping.legacyConvention == "(left-right)/2");
assert(m.legacyMapping.canonicalConvention == "(right-left)/2");
assert(m.legacyMapping.scale == -1);
assert(m.legacyMapping.provenanceRequired);
assert(contains(m.legacyMapping.rewritePolicy, "never overwrite"));
left = [2; -4];
right = [6; 8];
legacy = 0.5*(left - right);
canonical = 0.5*(right - left);
commonMode = 0.5*(right + left);
assert(isequal(canonical, m.legacyMapping.scale*legacy));
assert(isequal(commonMode, [4; 2]));
end

function testDownstreamLoggingContract()
m = contact_ground_truth_manifest();
c = m.loggingContract;
assert(c.mode == "observation_only");
assert(c.missingSignalPolicy == "fail_closed");
assert(c.failClosed.invalidStatus == "invalid_for_identification");
assert(~c.failClosed.allowProxyFallback);
assert(numel(c.requiredRawFields) == numel(m.selectedSignals));
assert(all(ismember(string({m.selectedSignals.loggingName}), ...
    c.requiredRawFields)));
requiredProcessed = ["xiDeltaLegacy", "dxiDeltaLegacy", ...
    "xiDeltaCanonical", "dxiDeltaCanonical", "vRollDelta", ...
    "uDiffRealizable", "contactPenetrationDelta", ...
    "contactNormalVelocityDelta"];
assert(all(ismember(requiredProcessed, c.requiredProcessedFields)));
assert(all(ismember(["schemaVersion", "selectedSignals", ...
    "canonicalConvention", "legacyMapping", "inspection"], ...
    c.requiredManifestSnapshotFields)));
assert(numel(c.smoke.caseNames) == 3);
assert(all(c.smoke.yawRates == [0, 0.02, -0.02]));
assert(c.manualFullBatch.caseCount == 25);
assert(c.manualFullBatch.owner == "human-manual-gate");
assert(~c.manualFullBatch.agentMayRun);
assert(contains(c.manualFullBatch.command, ...
    "run_contact_state_discovery_batch"));
end

function closeIfOpened(model, wasLoaded)
if ~wasLoaded && bdIsLoaded(model)
    close_system(model, 0);
end
end
