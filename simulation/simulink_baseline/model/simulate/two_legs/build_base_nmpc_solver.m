function info = build_base_nmpc_solver(forceRebuild, updateSimulinkModel, baseNmpcOverride)
%BUILD_BASE_NMPC_SOLVER Build the solver and update its Simulink block.
%
% Recommended non-interactive use:
%   matlab -batch "cd('<this folder>'); build_base_nmpc_solver(true)"

if nargin < 1 || isempty(forceRebuild)
    forceRebuild = false;
end
if nargin < 2 || isempty(updateSimulinkModel)
    updateSimulinkModel = true;
end
if nargin < 3
    baseNmpcOverride = [];
end
if ~isempty(baseNmpcOverride) ...
        && ~(isstruct(baseNmpcOverride) && isscalar(baseNmpcOverride))
    error("build_base_nmpc_solver:InvalidOverride", ...
        "baseNmpcOverride must be an empty value or a scalar struct.");
end

simulateDir = fileparts(mfilename("fullpath"));
repoRoot = findRepoRoot(simulateDir);
acadosDir = fullfile(repoRoot, "tools", "acados");
casadiDir = fullfile(repoRoot, "tools", "casadi");
addpath(casadiDir);
addpath(fullfile(acadosDir, "interfaces", "acados_matlab_octave"));
acados_env_variables_windows;

startupFile = fullfile(simulateDir, "startup.m");
evalin("base", "run(" + quoted(startupFile) + ");");
base = evalin("base", "base");
leg = evalin("base", "leg");
baseNmpc = evalin("base", "baseNmpc");
if ~isempty(baseNmpcOverride)
    baseNmpc = baseNmpcOverride;
end
isFull = isfield(baseNmpc, "variant") ...
    && string(baseNmpc.variant) == "full8dof";
if isFull
    assignin("base", "fullBaseNmpc", baseNmpc);
else
    assignin("base", "baseNmpc", baseNmpc);
end
generatedRoot = fullfile(simulateDir, "generated");
generatedDir = char(baseNmpc.generatedDir);

if ~isPathInside(generatedDir, generatedRoot)
    error("build_base_nmpc_solver:UnsafeGeneratedDir", ...
        "Refusing to operate outside %s.", generatedRoot);
end

sfunFile = fullfile(generatedDir, char(baseNmpc.sfunName) + "." + mexext);
jsonFile = fullfile(generatedDir, char(baseNmpc.solverName) + ".json");
manifestFile = fullfile(generatedDir, "base_nmpc_build_config.mat");
signature = buildSignature(base, leg, baseNmpc);
reuse = ~forceRebuild && isfile(sfunFile) && isfile(jsonFile) ...
    && manifestMatches(manifestFile, signature);

if ~reuse
    closeLoadedModelSafely("source");
    closeLoadedModelSafely(baseNmpc.solverName + "_ocp_solver_simulink_block");
    clearSolverFunctions(baseNmpc.solverName, baseNmpc.sfunName);
    removeGeneratedPaths(generatedDir);
    if isfolder(generatedDir)
        rmdir(generatedDir, "s");
    end
end

if ~isfolder(generatedDir)
    mkdir(generatedDir);
end

generated = ~reuse;
if generated
    if isFull
        ocp = full_base_nmpc_ocp(base, leg, baseNmpc);
    else
        ocp = base_nmpc_ocp(base, leg, baseNmpc);
    end
    creationOpts = struct( ...
        "generate", true, ...
        "build", true, ...
        "check_reuse_possible", false, ...
        "compile_mex_wrapper", true, ...
        "compile_interface", [], ...
        "output_dir", fullfile(generatedDir, "build"));
    solver = AcadosOcpSolver(ocp, creationOpts);
    delete(solver);
    clear solver;

    oldDir = pwd;
    cleanup = onCleanup(@() cd(oldDir));
    cd(generatedDir);
    make_sfun;
    clear cleanup
    save(manifestFile, "signature");
else
    fprintf("Reusing NMPC S-Function for Ts = %.6g s, N = %d.\n", ...
        baseNmpc.Ts, baseNmpc.N);
end

if ~isfile(sfunFile)
    error("build_base_nmpc_solver:MissingSFunction", ...
        "Expected S-Function was not created: %s", sfunFile);
end

addpath(generatedDir);
modelConfigured = false;
if updateSimulinkModel
    if isFull
        ensureModelIsClean("source");
        configure_symmetric_two_leg_simulink(true);
        configure_base_nmpc_simulink(true, "source");
    else
        ensureModelIsClean("source_common");
        configure_common_mode_simulink(true);
    end
    modelConfigured = true;
end
info = struct( ...
    "generated", logical(generated), ...
    "generatedDir", string(generatedDir), ...
    "sfunFile", string(sfunFile), ...
    "modelConfigured", modelConfigured);
fprintf("NMPC S-Function ready: %s\n", sfunFile);
end

function repoRoot = findRepoRoot(startDir)
% Locate the shared third-party tool boundary from nested model snapshots.
repoRoot = char(startDir);
while true
    acadosDir = fullfile(repoRoot, "tools", "acados");
    casadiDir = fullfile(repoRoot, "tools", "casadi");
    if isfolder(acadosDir) && isfolder(casadiDir)
        return;
    end
    parent = fileparts(repoRoot);
    if isempty(parent) || strcmp(parent, repoRoot)
        error("build_base_nmpc_solver:MissingToolRoot", ...
            "Could not locate tools/acados and tools/casadi above %s.", ...
            startDir);
    end
    repoRoot = parent;
end
end

function signature = buildSignature(base, leg, baseNmpc)
signature = struct( ...
    "schemaVersion", 8, ...
    "solverName", string(baseNmpc.solverName), ...
    "baseMass", double(base.m), ...
    "baseIyy", double(base.Iyy), ...
    "modelStateSize", size(baseNmpc.model.A, 1), ...
    "modelInputSize", size(baseNmpc.model.B, 2), ...
    "ocpStateSize", getModelField(baseNmpc, "ocpStateSize", ...
        size(baseNmpc.model.A, 1)), ...
    "incrementCostMode", string(getModelField(baseNmpc, ...
        "incrementCostMode", "previous_applied_anchor")), ...
    "modelInertia", getModelField(baseNmpc.model, "inertia", zeros(3, 1)), ...
    "modelMass", getModelField(baseNmpc.model, "m", base.m), ...
    "modelHalfTrack", getModelField(baseNmpc.model, "halfTrack", 0), ...
    "modelXiEq", getModelField(baseNmpc.model, "xiEq", 0), ...
    "modelDynamicsVersion", getModelField( ...
        baseNmpc.model, "dynamicsVersion", 1), ...
    "wheelMomentArmZ", double(baseNmpc.model.rWzEq), ...
    "wheelRadius", double(leg.r), ...
    "wheelMass", double(leg.mw), ...
    "wheelInertia", double(leg.Iw), ...
    "Ts", double(baseNmpc.Ts), ...
    "N", double(baseNmpc.N), ...
    "Q", double(baseNmpc.Q), ...
    "R1", double(baseNmpc.R1), ...
    "R2", double(baseNmpc.R2), ...
    "W_e", double(baseNmpc.W_e), ...
    "uMin", double(baseNmpc.uMin(:)), ...
    "uMax", double(baseNmpc.uMax(:)), ...
    "driveCoefficient", double(baseNmpc.driveCoefficient));
end

function value = getModelField(model, name, defaultValue)
if isfield(model, name)
    value = double(model.(name));
else
    value = defaultValue;
end
end

function tf = manifestMatches(manifestFile, signature)
tf = false;
if ~isfile(manifestFile)
    return;
end
stored = load(manifestFile, "signature");
if isfield(stored, "signature")
    tf = isequaln(stored.signature, signature);
end
end

function value = quoted(pathValue)
value = "'" + replace(string(pathValue), "'", "''") + "'";
end

function tf = isPathInside(pathValue, rootValue)
pathValue = char(java.io.File(pathValue).getCanonicalPath());
rootValue = char(java.io.File(rootValue).getCanonicalPath());
tf = startsWith(pathValue, [rootValue, filesep], "IgnoreCase", true);
end

function closeLoadedModelSafely(model)
if ~bdIsLoaded(model)
    return;
end
if strcmp(get_param(model, "Dirty"), "on")
    error("build_base_nmpc_solver:DirtyModel", ...
        "Save or discard unsaved changes in %s before rebuilding.", model);
end
close_system(model, 0);
end

function ensureModelIsClean(model)
if bdIsLoaded(model) && strcmp(get_param(model, "Dirty"), "on")
    error("build_base_nmpc_solver:DirtyModel", ...
        "Save or discard unsaved changes in %s before updating its NMPC block.", ...
        model);
end
end

function clearSolverFunctions(solverName, sfunName)
names = [
    string(sfunName)
    string(solverName) + "_mex_solver"
    "acados_mex_create_" + string(solverName)
    "acados_mex_free_" + string(solverName)
    "acados_mex_set_" + string(solverName)
    "acados_mex_solve_" + string(solverName)
];
for i = 1:numel(names)
    eval("clear " + names(i));
end
% A compiled S-Function can keep its dependent solver DLL loaded after the
% named entry point is cleared. MATLAB only exposes clear mex for releasing
% that dependency lock reliably before an in-place rebuild.
evalin("base", "clear mex");
rehash;
end

function removeGeneratedPaths(generatedDir)
entries = string(strsplit(path, pathsep));
generatedDir = string(java.io.File(generatedDir).getCanonicalPath());
for i = 1:numel(entries)
    if entries(i) == ""
        continue;
    end
    entry = string(java.io.File(entries(i)).getCanonicalPath());
    if strcmpi(entry, generatedDir) || startsWith(entry, generatedDir + filesep, ...
            "IgnoreCase", true)
        rmpath(entries(i));
    end
end
end
