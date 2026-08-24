function suppress_scope_windows(model)
%SUPPRESS_SCOPE_WINDOWS Keep Scope blocks closed during scripted runs.

if nargin < 1 || isempty(model)
    model = "source";
end

if ~bdIsLoaded(model)
    load_system(model);
end

scopes = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "Scope");
for i = 1:numel(scopes)
    try
        set_param(scopes{i}, "OpenAtSimulationStart", "off");
        set_param(scopes{i}, "Open", "off");
    catch
    end
end
end
