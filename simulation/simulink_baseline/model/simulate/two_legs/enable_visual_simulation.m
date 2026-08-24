function enable_visual_simulation(doSave)
%ENABLE_VISUAL_SIMULATION Restore visual windows for manual Simulink runs.

if nargin < 1 || isempty(doSave)
    doSave = false;
end

model = "source";
load_system(model);
open_system(model);

try
    set(0, "DefaultFigureVisible", "on");
catch
end

try
    set_param(model, "SimMechanicsOpenEditorOnUpdate", "on");
catch
end

try
    set_param(model, "SimscapeLogType", "all");
catch
end

try
    set_param(model, "SimulationCommand", "update");
catch
end

if doSave
    save_system(model);
end

fprintf("Enabled visual simulation for %s.\n", model);
end
