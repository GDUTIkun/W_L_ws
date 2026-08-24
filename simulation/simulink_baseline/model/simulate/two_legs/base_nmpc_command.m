function y = base_nmpc_command(x, baseNmpc)
%BASE_NMPC_COMMAND Validate the direct NMPC output for the lower QP.
%
% x = [time; NMPC wheel-to-body interaction wrench(3); status; CPU time]
% y = [selected QP command(3); solver fault active]

if nargin < 2 || isempty(baseNmpc)
    baseNmpc = evalin("base", "baseNmpc");
end

x = double(x(:));
if numel(x) ~= 6
    error("base_nmpc_command:InvalidInput", "Expected a 6-element input.");
end

time = x(1);
uBody = x(2:4);
status = x(5);
cpuTime = x(6);
valid = baseNmpc.enabled && status == 0 ...
    && all(isfinite(uBody)) && isfinite(cpuTime) ...
    && cpuTime <= baseNmpc.maxSolveTime;

persistent lastCommand
equilibriumCommand = [-baseNmpc.model.uEq(1:2); baseNmpc.model.uEq(3)];
if time <= 0 || isempty(lastCommand) || any(~isfinite(lastCommand))
    lastCommand = equilibriumCommand;
end

if valid
    lastCommand = [-uBody(1:2); uBody(3)];
end

% At model initialization the equilibrium command is intentional.  The
% solver output may not yet have a meaningful timing sample, so t = 0 is
% not a runtime solver fault.
faultActive = time > 0 && ~valid;
y = [lastCommand; faultActive];
end
