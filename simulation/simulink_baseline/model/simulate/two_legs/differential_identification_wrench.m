function [combinedWrench, excitation, value] = ...
        differential_identification_wrench(t, requestedWrench, config, limits)
%DIFFERENTIAL_IDENTIFICATION_WRENCH Add a bounded differential WBC test input.
% Commands are expressed in the frozen one-dimensional uDiffRealizable basis
% (contact_consistent_differential_contract, D-05): the scalar input drives
% the coupled wrench image of the rolling contact direction, so FDelta and
% TDelta are not independent production inputs. A "force" channel interprets
% amplitude as N-equivalent rolling effort; a "torque" channel rescales so
% the realized (right-left)/2 pitch moment equals the requested amplitude.
% The reported value is the realized scalar input 0.5*(excitation(7)-...).
% Positive values follow (right - left)/2.

if nargin < 3 || isempty(config)
    if evalin("base", "exist('differentialIdentification', 'var')") == 1
        config = evalin("base", "differentialIdentification");
    else
        config = struct("enabled", false);
    end
end
if nargin < 4 || isempty(limits)
    if evalin("base", "exist('fullBaseNmpc', 'var')") == 1
        nmpc = evalin("base", "fullBaseNmpc");
        limits = struct("min", nmpc.uMin(:), "max", nmpc.uMax(:));
    else
        limits = struct("min", -inf(12, 1), "max", inf(12, 1));
    end
end

requestedWrench = double(requestedWrench(:));
if numel(requestedWrench) ~= 12 || any(~isfinite(requestedWrench))
    error("differential_identification_wrench:InvalidWrench", ...
        "requestedWrench must contain 12 finite values.");
end
combinedWrench = requestedWrench;
excitation = zeros(12, 1);
value = 0;
if ~logical(fieldOr(config, "enabled", false))
    return;
end

startTime = scalarField(config, "startTime", 0);
duration = scalarField(config, "duration", inf);
if duration <= 0 || t < startTime || t >= startTime + duration
    return;
end
amplitude = scalarField(config, "amplitude", 0);
localTime = double(t) - startTime;
excitationType = lower(string(fieldOr(config, "type", "step")));
switch excitationType
    case "step"
        value = amplitude;
    case "chirp"
        frequency = vectorField(config, "frequencyHz", [0.2; 3]);
        if numel(frequency) ~= 2 || any(frequency <= 0)
            error("differential_identification_wrench:InvalidFrequency", ...
                "A chirp requires two positive frequencyHz values.");
        end
        sweepRate = (frequency(2) - frequency(1))/duration;
        phase = 2*pi*(frequency(1)*localTime ...
            + 0.5*sweepRate*localTime^2);
        value = amplitude*sin(phase);
    case "multisine"
        frequency = vectorField(config, "frequencyHz", [0.2; 0.5; 1; 2]);
        if any(frequency <= 0)
            error("differential_identification_wrench:InvalidFrequency", ...
                "Multisine frequencies must be positive.");
        end
        phase = vectorField(config, "phaseRad", zeros(size(frequency)));
        weight = vectorField(config, "weight", ones(size(frequency)));
        if numel(phase) ~= numel(frequency) || numel(weight) ~= numel(frequency)
            error("differential_identification_wrench:InvalidMultisine", ...
                "frequencyHz, phaseRad, and weight must have equal lengths.");
        end
        scale = sum(abs(weight));
        if scale == 0
            value = 0;
        else
            value = amplitude*sum(weight.*sin(2*pi*frequency*localTime + phase))/scale;
        end
    case "prbs"
        bitPeriod = scalarField(config, "bitPeriod", 0.1);
        seed = scalarField(config, "seed", 1);
        if bitPeriod <= 0 || seed < 0
            error("differential_identification_wrench:InvalidPrbs", ...
                "bitPeriod must be positive and seed nonnegative.");
        end
        index = floor(localTime/bitPeriod);
        word = uint32(mod(index + floor(seed), double(intmax("uint32") - 1)) + 1);
        word = bitxor(word, bitshift(word, 13));
        word = bitxor(word, bitshift(word, -17));
        word = bitxor(word, bitshift(word, 5));
        value = amplitude*(2*double(bitand(word, uint32(1))) - 1);
    otherwise
        error("differential_identification_wrench:InvalidType", ...
            "Unknown excitation type '%s'.", excitationType);
end

channel = lower(string(fieldOr(config, "channel", "force")));
basis = realizableBasis(config);
switch channel
    case "force"
        scalar = value;
    case "torque"
        coupling = 0.5*(basis(11) - basis(5));
        if abs(coupling) < 1e-12
            error("differential_identification_wrench:NoTorqueCoupling", ...
                "The frozen uDiffRealizable basis has no pitch-moment coupling.");
        end
        scalar = value/coupling;
    otherwise
        error("differential_identification_wrench:InvalidChannel", ...
            "channel must be 'force' or 'torque'.");
end
excitation = scalar*basis;

candidate = requestedWrench + excitation;
if logical(fieldOr(config, "clipToNmpcBounds", true))
    lowerBound = double(limits.min(:));
    upperBound = double(limits.max(:));
    if numel(lowerBound) ~= 12 || numel(upperBound) ~= 12
        error("differential_identification_wrench:InvalidLimits", ...
            "limits.min and limits.max must contain 12 values.");
    end
    combinedWrench = min(max(candidate, lowerBound), upperBound);
else
    combinedWrench = candidate;
end
excitation = combinedWrench - requestedWrench;
value = 0.5*(excitation(7) - excitation(1));
end

function basis = realizableBasis(config)
basis = fieldOr(config, "uDiffRealizableBasis", []);
if isempty(basis)
    persistent contract
    if isempty(contract)
        contract = contact_consistent_differential_contract();
    end
    basis = contract.inputContract.basis;
end
basis = double(basis(:));
if numel(basis) ~= 12 || any(~isfinite(basis))
    error("differential_identification_wrench:InvalidBasis", ...
        "The frozen uDiffRealizable basis must span the 12 wrench channels.");
end
end

function value = fieldOr(config, name, defaultValue)
if isfield(config, name)
    value = config.(name);
else
    value = defaultValue;
end
end

function value = scalarField(config, name, defaultValue)
value = double(fieldOr(config, name, defaultValue));
if ~isscalar(value) || ~isreal(value) || isnan(value)
    error("differential_identification_wrench:InvalidConfig", ...
        "%s must be a real scalar.", name);
end
end

function value = vectorField(config, name, defaultValue)
value = double(fieldOr(config, name, defaultValue));
value = value(:);
if ~isreal(value) || any(~isfinite(value))
    error("differential_identification_wrench:InvalidConfig", ...
        "%s must contain finite real values.", name);
end
end
