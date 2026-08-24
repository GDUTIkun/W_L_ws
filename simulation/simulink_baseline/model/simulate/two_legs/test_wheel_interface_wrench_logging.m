function test_wheel_interface_wrench_logging(mode)
%TEST_WHEEL_INTERFACE_WRENCH_LOGGING Regress the frozen wheel-wrench seam.

if nargin < 1
    mode = "contract_and_reconstruction";
end

switch string(mode)
    case "contract_and_reconstruction"
        testContractAndReconstruction();
    case "in_memory_model"
        testInMemoryModel();
    case "sensing_invariance"
        testSensingInvariance();
    otherwise
        error("test_wheel_interface_wrench_logging:UnknownMode", ...
            "Unsupported test mode '%s'.", string(mode));
end

function testInMemoryModel()
model = "source";
evalin("base", "startup");
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(model);
end
wasDirty = string(get_param(model, "Dirty"));
cleanup = onCleanup(@() closeIfOpened(model, wasLoaded));

[logging, verification] = configure_wheel_interface_wrench_logging( ...
    "Mode", "enable", "Model", model);
expectedNames = ["leftTotalForce", "leftTotalTorque", ...
    "rightTotalForce", "rightTotalTorque"];
assert(isequal(string({logging.signals.loggingName}), expectedNames));
assert(all(isfield(logging.wired, matlab.lang.makeValidName(expectedNames))));
assert(verification.observationOnly);
assert(~verification.saved);
assert(verification.invariantUnchanged);
assert(verification.sensingEnabled);
assert(verification.rawChannelsRetained);
assert(verification.nativeMetadataRetained);
assert(verification.modelDirtyAfter == wasDirty);

[disabled, disabledVerification] = configure_wheel_interface_wrench_logging( ...
    "Mode", "disable", "Model", model);
assert(isempty(fieldnames(disabled.wired)));
assert(disabledVerification.invariantUnchanged);
assert(~disabledVerification.sensingEnabled);
assert(disabledVerification.modelDirtyAfter == wasDirty);
assert(string(get_param(model, "Dirty")) == wasDirty);
clear cleanup
end

function testSensingInvariance()
model = "source";
evalin("base", "startup");
wasLoaded = bdIsLoaded(model);
if ~wasLoaded
    load_system(model);
end
wasDirty = string(get_param(model, "Dirty"));
cleanup = onCleanup(@() closeIfOpened(model, wasLoaded));

[~, verification] = configure_wheel_interface_wrench_logging( ...
    "Mode", "verify_invariance", "Model", model);
assert(verification.observationOnly);
assert(verification.invariantUnchanged);
assert(verification.behaviorInvariant);
assert(verification.offRepeatability.passed);
assert(verification.onOffInvariant.passed);
assert(verification.tolerance == 1.601e-6);
assert(contains(verification.wallClockPolicy, "excluded"));
assert(verification.modelDirtyAfter == wasDirty);
assert(string(get_param(model, "Dirty")) == wasDirty);
clear cleanup
end
end

function testContractAndReconstruction()
contract = wheel_interface_wrench_contract();
assert(contract.contractVersion == "09-01-G1");
assert(contract.canonicalDifferential == "(right-left)/2");
assert(contract.scalarPolarity == 1);
assert(contract.left.jointPath == "source/PD_only/Revolute Joint5");
assert(contract.right.jointPath == "source/PD_only/Revolute Joint2");
assert(contract.left.compositeWrenchDir == "FollowerOnBase");
assert(contract.left.compositeWrenchFrame == "BaseFrame");
assert(contract.controller.componentOrder == ...
    "[left Fx;Fy;Fz;Mx;My;Mz;right Fx;Fy;Fz;Mx;My;Mz]");

P = [1, 0, 0; 0, 0, 1; 0, 1, 0];
Rwb = controller_attitude_kinematics([0.11; -0.23; 0.37]);
RwjLeft = Rwb*rotationZ(0.31);
RwjRight = Rwb*rotationZ(-0.19);
sample = validSample(Rwb, RwjLeft, RwjRight);
projectionContract = struct("inputContract", ...
    struct("basis", [1; -2; 0.5; 0; 0; 3; -1; 2; -0.5; 0; 0; -3]));

result = reconstruct_direct_plant_wrench(sample, contract, projectionContract);
Rleft = P*Rwb'*RwjLeft;
Rright = P*Rwb'*RwjRight;
expected = [Rleft*sample.left.force; Rleft*sample.left.torque; ...
    Rright*sample.right.force; Rright*sample.right.torque];
assert(norm(result.directPlantWrench - expected, inf) < 1e-12);
assert(norm(result.raw.left.force - sample.left.force, inf) < 1e-12);
assert(norm(result.raw.right.torque - sample.right.torque, inf) < 1e-12);
assert(norm(result.rotation.left - Rleft, inf) < 1e-12);
assert(norm(result.rotation.right - Rright, inf) < 1e-12);

[u, projected, residual, residualRms] = ...
    project_uDiffRealizable(expected, projectionContract);
assert(abs(result.uDiffPlantDirect - u) < 1e-12);
assert(norm(result.projectedWrench - projected, inf) < 1e-12);
assert(norm(result.residualWrench - residual, inf) < 1e-12);
assert(abs(result.projectionResidualRms - residualRms) < 1e-12);

assertFails(@() reconstruct_direct_plant_wrench( ...
    withTime(sample, [2; 1]), contract, projectionContract), ...
    "reconstruct_direct_plant_wrench:NonmonotoneTimestamp");
assertFails(@() reconstruct_direct_plant_wrench( ...
    withLeftForce(sample, [NaN; 2; 3]), contract, projectionContract), ...
    "reconstruct_direct_plant_wrench:InvalidSample");
badSemantics = sample;
badSemantics.left.actor = "leg acting on wheel";
assertFails(@() reconstruct_direct_plant_wrench( ...
    badSemantics, contract, projectionContract), ...
    "reconstruct_direct_plant_wrench:SemanticMismatch");
badOrigin = sample;
badOrigin.coincidentWheelCenterOrigin = false;
assertFails(@() reconstruct_direct_plant_wrench( ...
    badOrigin, contract, projectionContract), ...
    "reconstruct_direct_plant_wrench:OriginNotProven");
end

function sample = validSample(Rwb, RwjLeft, RwjRight)
sample = struct();
sample.timestamp = 1.25;
sample.baseRotation = Rwb;
sample.coincidentWheelCenterOrigin = true;
sample.left = sideSample("left", "source/PD_only/Revolute Joint5", ...
    [1; 2; 3], [4; 5; 6], RwjLeft);
sample.right = sideSample("right", "source/PD_only/Revolute Joint2", ...
    [-2; 1; 4], [-1; 3; 2], RwjRight);
end

function side = sideSample(name, path, force, torque, rotation)
side = struct("side", name, "jointPath", path, ...
    "actor", "wheel follower", "receiver", "leg/base", ...
    "compositeWrenchDir", "FollowerOnBase", ...
    "compositeWrenchFrame", "BaseFrame", "force", force, ...
    "torque", torque, "jointRotation", rotation, ...
    "forceUnit", "N", "torqueUnit", "Nm");
end

function changed = withTime(sample, timestamp)
changed = sample;
changed.timestamp = timestamp;
changed.baseRotation = repmat(sample.baseRotation, 1, 1, numel(timestamp));
changed.left.force = repmat(sample.left.force, 1, numel(timestamp));
changed.left.torque = repmat(sample.left.torque, 1, numel(timestamp));
changed.left.jointRotation = repmat(sample.left.jointRotation, 1, 1, numel(timestamp));
changed.right.force = repmat(sample.right.force, 1, numel(timestamp));
changed.right.torque = repmat(sample.right.torque, 1, numel(timestamp));
changed.right.jointRotation = repmat(sample.right.jointRotation, 1, 1, numel(timestamp));
end

function changed = withLeftForce(sample, force)
changed = sample;
changed.left.force = force;
end

function closeIfOpened(model, wasLoaded)
if ~wasLoaded && bdIsLoaded(model)
    close_system(model, 0);
end
end

function assertFails(action, expectedIdentifier)
try
    action();
catch exception
    assert(string(exception.identifier) == string(expectedIdentifier), ...
        "Expected %s, received %s.", expectedIdentifier, exception.identifier);
    return;
end
error("test_wheel_interface_wrench_logging:ExpectedFailure", ...
    "Expected error %s was not thrown.", expectedIdentifier);
end

function R = rotationZ(angle)
R = [cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0, 0, 1];
end
