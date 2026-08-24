function result = reconstruct_direct_plant_wrench(sample, contract, projectionContract)
%RECONSTRUCT_DIRECT_PLANT_WRENCH Convert raw joint-base wrench to audit frame.
% Retains raw total-wrench samples and native timestamps, applies the frozen
% R_CJ=P*R_WB'*R_WJ rotation to force and torque, and delegates the only
% scalar coordinate to project_uDiffRealizable.  No origin shift is applied.

if nargin < 2 || isempty(contract)
    contract = wheel_interface_wrench_contract();
end
if nargin < 3 || isempty(projectionContract)
    error("reconstruct_direct_plant_wrench:MissingProjectionContract", ...
        "An explicit frozen uDiffRealizable projection contract is required.");
end
validateContract(contract);
validateSample(sample, contract);

timestamp = double(sample.timestamp(:));
count = numel(timestamp);
P = double(contract.controller.physicalToController);
directPlantWrench = zeros(12, count);
rotation = struct("left", zeros(3, 3, count), ...
    "right", zeros(3, 3, count));
for index = 1:count
    Rwb = sliceRotation(sample.baseRotation, index, count, "baseRotation");
    RwjLeft = sliceRotation(sample.left.jointRotation, index, count, ...
        "left.jointRotation");
    RwjRight = sliceRotation(sample.right.jointRotation, index, count, ...
        "right.jointRotation");
    rotation.left(:, :, index) = P*Rwb'*RwjLeft;
    rotation.right(:, :, index) = P*Rwb'*RwjRight;
    directPlantWrench(:, index) = [ ...
        rotation.left(:, :, index)*sample.left.force(:, index); ...
        rotation.left(:, :, index)*sample.left.torque(:, index); ...
        rotation.right(:, :, index)*sample.right.force(:, index); ...
        rotation.right(:, :, index)*sample.right.torque(:, index)];
end

u = zeros(count, 1);
projected = zeros(12, count);
residual = zeros(12, count);
residualRms = zeros(count, 1);
for index = 1:count
    [u(index), projected(:, index), residual(:, index), residualRms(index)] = ...
        project_uDiffRealizable(directPlantWrench(:, index), projectionContract);
end

result = struct();
result.contractVersion = contract.contractVersion;
result.timestamp = sample.timestamp;
result.raw = struct("left", rawSide(sample.left), "right", rawSide(sample.right));
result.rotation = collapseRotation(rotation, count);
result.directPlantWrench = collapseColumns(directPlantWrench, count);
result.uDiffPlantDirect = collapseColumns(u, count);
result.projectedWrench = collapseColumns(projected, count);
result.residualWrench = collapseColumns(residual, count);
result.projectionResidualRms = collapseColumns(residualRms, count);
result.provenance = struct( ...
    "rawFrame", contract.rawFrame, ...
    "transformation", contract.controller.rotation, ...
    "momentOrigin", contract.controller.momentOrigin, ...
    "projection", contract.projection.helper, ...
    "nativeTimestampRetained", true);
end

function validateContract(contract)
required = ["contractVersion", "left", "right", "controller", "projection"];
for name = required
    if ~isfield(contract, name)
        error("reconstruct_direct_plant_wrench:InvalidContract", ...
            "Contract is missing %s.", name);
    end
end
if contract.contractVersion ~= "09-01-G1" ...
        || contract.canonicalDifferential ~= "(right-left)/2" ...
        || contract.scalarPolarity ~= 1 ...
        || contract.controller.originShiftAllowed
    error("reconstruct_direct_plant_wrench:InvalidContract", ...
        "Contract must retain the frozen Phase-09 differential and origin rules.");
end
end

function validateSample(sample, contract)
required = ["timestamp", "baseRotation", "left", "right", ...
    "coincidentWheelCenterOrigin"];
if ~isstruct(sample) || ~all(isfield(sample, required))
    error("reconstruct_direct_plant_wrench:InvalidSample", ...
        "Sample must provide timestamps, orientations, both sides, and origin proof.");
end
if ~islogical(sample.coincidentWheelCenterOrigin) ...
        || ~isscalar(sample.coincidentWheelCenterOrigin) ...
        || ~sample.coincidentWheelCenterOrigin
    error("reconstruct_direct_plant_wrench:OriginNotProven", ...
        "The coincident wheel-center moment origin must be explicitly proven.");
end
timestamp = double(sample.timestamp(:));
if isempty(timestamp) || ~isreal(timestamp) || any(~isfinite(timestamp))
    error("reconstruct_direct_plant_wrench:InvalidSample", ...
        "Native timestamps must be real and finite.");
end
if numel(timestamp) > 1 && any(diff(timestamp) <= 0)
    error("reconstruct_direct_plant_wrench:NonmonotoneTimestamp", ...
        "Native timestamps must be strictly increasing.");
end
validateSide(sample.left, contract.left, numel(timestamp));
validateSide(sample.right, contract.right, numel(timestamp));
validateRotationSeries(sample.baseRotation, numel(timestamp), "baseRotation");
end

function validateSide(side, expected, count)
semanticFields = ["side", "jointPath", "actor", "receiver", ...
    "compositeWrenchDir", "compositeWrenchFrame", "forceUnit", "torqueUnit"];
dataFields = ["force", "torque", "jointRotation"];
if ~isstruct(side) || ~all(isfield(side, [semanticFields, dataFields]))
    error("reconstruct_direct_plant_wrench:InvalidSample", ...
        "Each side must provide semantics, vectors, and joint orientation.");
end
for field = semanticFields
    if string(side.(field)) ~= string(expected.(field))
        error("reconstruct_direct_plant_wrench:SemanticMismatch", ...
            "Joint-side semantic field %s does not match the frozen contract.", field);
    end
end
validateVectorSeries(side.force, count, "force");
validateVectorSeries(side.torque, count, "torque");
validateRotationSeries(side.jointRotation, count, "jointRotation");
end

function validateVectorSeries(value, count, name)
value = double(value);
if ~isreal(value) || ~isequal(size(value), [3, count]) || any(~isfinite(value(:)))
    error("reconstruct_direct_plant_wrench:InvalidSample", ...
        "%s must be a real finite 3-by-%d series.", name, count);
end
end

function validateRotationSeries(value, count, name)
if ~isreal(value) || any(~isfinite(value(:))) ...
        || ~(isequal(size(value), [3, 3]) && count == 1) ...
        && ~isequal(size(value), [3, 3, count])
    error("reconstruct_direct_plant_wrench:InvalidSample", ...
        "%s must be a finite 3-by-3-by-N rotation series.", name);
end
for index = 1:count
    rotation = sliceRotation(value, index, count, name);
    if norm(rotation'*rotation - eye(3), inf) > 1e-10 || det(rotation) <= 0
        error("reconstruct_direct_plant_wrench:InvalidSample", ...
            "%s sample %d is not a proper rotation.", name, index);
    end
end
end

function rotation = sliceRotation(series, index, count, name)
if count == 1 && isequal(size(series), [3, 3])
    rotation = double(series);
elseif isequal(size(series), [3, 3, count])
    rotation = double(series(:, :, index));
else
    error("reconstruct_direct_plant_wrench:InvalidSample", ...
        "%s does not match the timestamp count.", name);
end
end

function value = rawSide(side)
value = struct("force", side.force, "torque", side.torque, ...
    "jointPath", side.jointPath, "actor", side.actor, ...
    "receiver", side.receiver, "forceUnit", side.forceUnit, ...
    "torqueUnit", side.torqueUnit);
end

function value = collapseColumns(value, count)
if count == 1
    value = value(:, 1);
end
end

function value = collapseRotation(value, count)
if count == 1
    value.left = value.left(:, :, 1);
    value.right = value.right(:, :, 1);
end
end
