function audit = audit_differential_realizable_input_basis()
%AUDIT_DIFFERENTIAL_REALIZABLE_INPUT_BASIS Freeze the signed WBC input basis.
% The scalar candidate input follows (right-left)/2 and excites the contact
% rolling direction. Its upper-wrench image necessarily couples forward force
% and axle moment; those channels are therefore not treated as independent.

thisDir = fileparts(mfilename("fullpath"));
requiredBaseVariables = ["leg", "base", "ctrl", "fullBaseNmpc"];
baseReady = true;
for idx = 1:numel(requiredBaseVariables)
    baseReady = baseReady && evalin("base", ...
        "exist('" + requiredBaseVariables(idx) + "', 'var') == 1");
end
if ~baseReady
    run(fullfile(thisDir, "startup.m"));
    % startup.m intentionally clears its caller workspace before repopulating
    % the base workspace, so reconstruct this path after it returns.
    thisDir = fileparts(mfilename("fullpath"));
end
[~, snapshot] = spatial_two_leg_qp_core("contact-audit");
validateSnapshot(snapshot);

rightMinusLeft = 0.5*[-eye(6), eye(6)];
antisymmetricContact = [-eye(3); eye(3)];
contactMap = snapshot.interactionWrenchMap.contact;
fullWrenchImage = contactMap*antisymmetricContact;
coupledWrenchImage = rightMinusLeft*fullWrenchImage;

targetPairSelector = zeros(2, 6);
targetPairSelector(1, 1) = 1;
targetPairSelector(2, 5) = 1;
targetPairImage = targetPairSelector*coupledWrenchImage;
rollingPair = targetPairImage(:, 1);
if rank(rollingPair) ~= 1 || abs(rollingPair(1)) < 1e-12
    error("audit_differential_realizable_input_basis:NoRollingBasis", ...
        "The inspected contact map has no finite forward rolling-wrench image.");
end

forceScale = rollingPair(1);
signedBasis = fullWrenchImage(:, 1)/forceScale;
signedPairBasis = rollingPair/forceScale;
achievableRank = rank(signedPairBasis);
requestedDimension = size(targetPairSelector, 1);
nullspaceBasis = null(signedPairBasis');
taskProjection = signedPairBasis*pinv(signedPairBasis);
[nmpcMin, nmpcMax] = scalarBounds( ...
    signedBasis, snapshot.nmpcBounds.min, snapshot.nmpcBounds.max);
normalReference = snapshot.robotMass*snapshot.gravity/2;
frictionPyramid = snapshot.frictionCoefficient/sqrt(2);
contactCoefficientPerUnit = 1/abs(forceScale);
frictionMagnitude = frictionPyramid*normalReference ...
    / contactCoefficientPerUnit;
estimatedMin = max(nmpcMin, -frictionMagnitude);
estimatedMax = min(nmpcMax, frictionMagnitude);

audit = struct();
audit.name = "uDiffRealizable";
audit.rank = achievableRank;
audit.achievableDimension = achievableRank;
audit.fullDifferentialRank = rank(coupledWrenchImage);
audit.requestedPairDimension = requestedDimension;
audit.nullspaceDimension = requestedDimension - achievableRank;
audit.nullspaceBasis = nullspaceBasis;
audit.basis = signedBasis;
audit.targetPairBasis = signedPairBasis;
audit.contactCoefficientBasis = antisymmetricContact;
audit.coupledWrenchImage = coupledWrenchImage;
audit.targetPairImage = targetPairImage;
audit.taskProjection = taskProjection;
audit.limits = struct( ...
    "nmpcScalarMin", nmpcMin, ...
    "nmpcScalarMax", nmpcMax, ...
    "frictionScalarMagnitudeEstimate", frictionMagnitude, ...
    "estimatedScalarMin", estimatedMin, ...
    "estimatedScalarMax", estimatedMax, ...
    "torquePerJointAbs", snapshot.torqueLimit, ...
    "frictionCoefficient", snapshot.frictionCoefficient, ...
    "frictionPyramidCoefficient", frictionPyramid, ...
    "normalForceReferencePerWheel", normalReference, ...
    "nmpcPerWrenchMin", snapshot.nmpcBounds.min, ...
    "nmpcPerWrenchMax", snapshot.nmpcBounds.max);
audit.frames = snapshot.frames;
audit.frames.sign = "positive scalar gives positive right-minus-left forward force";
audit.convention = "(right-left)/2";
audit.inputUnit = "N-equivalent rolling contact effort";
audit.sampleTimes = snapshot.sampleTimes;
audit.source = struct( ...
    "controller", "spatial_two_leg_qp_core/contact-audit", ...
    "contactMap", "interactionWrenchMap contact coefficient map", ...
    "contactBasis", "spatialDynamics contactBasis", ...
    "rollingDirection", snapshot.rollingDirection, ...
    "wheelRadius", snapshot.wheelRadius);

artifactDir = fullfile(thisDir, "generated", "audits");
if ~isfolder(artifactDir)
    mkdir(artifactDir);
end
audit.artifactPath = fullfile( ...
    artifactDir, "differential_realizable_input_basis.mat");
save(audit.artifactPath, "audit");
end

function validateSnapshot(snapshot)
required = ["contactJacobian", "contactBasis", "rollingDirection", ...
    "interactionWrenchMap", "torqueLimit", "frictionCoefficient", ...
    "nmpcBounds", "frames", "sampleTimes", "robotMass", "gravity"];
for name = required
    if ~isfield(snapshot, name)
        error("audit_differential_realizable_input_basis:MissingWbcField", ...
            "Required WBC/contact field '%s' is unavailable.", name);
    end
end
if ~isfield(snapshot.interactionWrenchMap, "contact") ...
        || ~isequal(size(snapshot.interactionWrenchMap.contact), [12, 6]) ...
        || ~isequal(size(snapshot.contactJacobian), [6, 12]) ...
        || ~isequal(size(snapshot.contactBasis), [3, 3, 2])
    error("audit_differential_realizable_input_basis:InvalidWbcField", ...
        "WBC/contact maps have unexpected dimensions.");
end
end

function [minimum, maximum] = scalarBounds(basis, lower, upper)
minimum = -inf;
maximum = inf;
for index = 1:numel(basis)
    coefficient = basis(index);
    if abs(coefficient) < 1e-12
        if lower(index) > 0 || upper(index) < 0
            error("audit_differential_realizable_input_basis:InfeasibleBounds", ...
                "A zero basis component violates the inspected NMPC bounds.");
        end
        continue;
    end
    interval = sort([lower(index)/coefficient, upper(index)/coefficient]);
    minimum = max(minimum, interval(1));
    maximum = min(maximum, interval(2));
end
if minimum > maximum
    error("audit_differential_realizable_input_basis:InfeasibleBounds", ...
        "The signed realizable basis has no scalar interval inside NMPC bounds.");
end
end
