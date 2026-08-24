function [uScalar, wProjected, wResidual, projectionResidualRms] = ...
    project_uDiffRealizable(wrench, contract)
%PROJECT_UDIFFREALIZABLE Frozen rank-1 projection for the Phase-8 audit.
%
% Projects a 12-by-1 controller-frame per-side wrench onto the frozen
% uDiffRealizable basis.  The scalar coordinate, projected wrench,
% residual, and projection-residual RMS are returned.  Invalid inputs
% produce an error; the function never imputes, clips, or independently
% projects force and torque channels.
%
% Contract 08-01-G1:
%   u = (b'*w) / (b'*b)
%   wProjected = b * u
%   wResidual = w - wProjected
%   projectionResidualRms = norm(wResidual, 2) / sqrt(12)
%   minimumDenominator = 1e-12

persistent frozenContract

if nargin < 2 || isempty(contract)
    if isempty(frozenContract)
        frozenContract = contact_consistent_differential_contract();
    end
    contract = frozenContract;
end

% --- input validation ---------------------------------------------------
wrench = double(wrench(:));
if ~isreal(wrench) || any(~isfinite(wrench)) || numel(wrench) ~= 12
    error("project_uDiffRealizable:InvalidInput", ...
        "project_uDiffRealizable requires a real finite 12-by-1 wrench.");
end

% --- frozen basis extraction --------------------------------------------
basis = double(contract.inputContract.basis(:));
if ~isreal(basis) || any(~isfinite(basis)) || numel(basis) ~= 12
    error("project_uDiffRealizable:InvalidBasis", ...
        "The frozen contract basis must have 12 elements.");
end

% --- rank-1 projection --------------------------------------------------
bb = basis.' * basis;
if abs(bb) < 1e-12
    error("project_uDiffRealizable:DegenerateBasis", ...
        "The frozen basis denominator is below the 1e-12 threshold.");
end

uScalar = (basis.' * wrench) / bb;
wProjected = basis * uScalar;
wResidual = wrench - wProjected;
projectionResidualRms = norm(wResidual, 2) / sqrt(12);
end
