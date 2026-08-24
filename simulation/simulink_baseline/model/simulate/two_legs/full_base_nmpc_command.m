function y = full_base_nmpc_command(x, config)
%FULL_BASE_NMPC_COMMAND Guard the 12D NMPC interaction-wrench output.

if nargin < 2 || isempty(config)
    config = evalin("base", "fullBaseNmpc");
end
x = double(x(:));
if numel(x) ~= 15
    error("full_base_nmpc_command:InvalidInput", ...
        "Expected [time; wrench(12); status; CPU time].");
end
time = x(1);
wrench = x(2:13);
status = x(14);
cpuTime = x(15);
valid = config.enabled && status == 0 && all(isfinite(wrench)) ...
    && isfinite(cpuTime) && cpuTime <= config.maxSolveTime;

persistent lastCommand
if time <= 0 || isempty(lastCommand) || numel(lastCommand) ~= 12 ...
        || any(~isfinite(lastCommand))
    lastCommand = config.model.uEq(:);
end
if valid
    lastCommand = wrench;
end
faultActive = time > 0 && ~valid;
y = [lastCommand; faultActive];
end
