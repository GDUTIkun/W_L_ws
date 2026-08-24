function context = open_proformance_test(openModel)
%OPEN_PROFORMANCE_TEST Activate the isolated two-leg performance snapshot.

arguments
    openModel (1, 1) logical = true
end

root = fileparts(mfilename("fullpath"));
modelDir = fullfile(root, "model", "simulate", "two_legs");
codeDir = fullfile(root, "model", "code");
studyDir = fullfile(root, "calibration", "studies", ...
    "2026_08_two_leg_model_tests");
workDir = fullfile(root, "work");

requiredFolders = [string(modelDir), string(codeDir), string(studyDir)];
for folder = requiredFolders
    if ~isfolder(folder)
        error("open_proformance_test:MissingFolder", ...
            "Snapshot folder is missing: %s", folder);
    end
    addpath(folder, "-begin");
end
if ~isfolder(workDir)
    mkdir(workDir);
end
Simulink.fileGenControl("set", ...
    "CacheFolder", workDir, ...
    "CodeGenFolder", workDir, ...
    "createDir", true);

clear spatial_two_leg_qp_core coupled_two_leg_qp_core ...
    differential_leg_force_stabilizer differential_drift_stabilizer
rehash path;

startupFile = fullfile(modelDir, "startup.m");
evalin("base", "run(" + quoted(startupFile) + ")");

resolvedController = string(which("differential_leg_force_stabilizer"));
if ~startsWith(resolvedController, string(root), ...
        "IgnoreCase", ispc)
    error("open_proformance_test:PathLeak", ...
        "Controller resolved outside the snapshot: %s", ...
        resolvedController);
end

modelFile = fullfile(modelDir, "source.slx");
load_system(modelFile);
if openModel
    open_system("source");
end

context = struct( ...
    "root", root, ...
    "modelDir", modelDir, ...
    "codeDir", codeDir, ...
    "studyDir", studyDir, ...
    "workDir", workDir, ...
    "model", "source", ...
    "controller", resolvedController);
fprintf("Performance snapshot ready: %s\n", root);
fprintf("Resolved anti-split controller: %s\n", resolvedController);
end

function value = quoted(pathValue)
value = "'" + replace(string(pathValue), "'", "''") + "'";
end
