function harnessFile = createAerodynamicsSinglePlateHarness()
%CREATEAERODYNAMICSSINGLEPLATEHARNESS Create the external Sentman harness.

projectRoot = fileparts(fileparts(mfilename("fullpath")));
addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "simulink"));
addpath(fullfile(projectRoot, "src", "environment"));

modelFile = fullfile(projectRoot, "models", "aocs_plant.slx");
modelName = "aocs_plant";
owner = modelName + ...
    "/Orbit & Environment/Disturbance Torques/Sentman Multispecies Panels";
harnessName = "AerodynamicsSinglePlateHarness";
harnessFile = fullfile(projectRoot, "tests", "harnesses", harnessName + ".slx");

AOCS = setupAocsSimulation(fullfile(projectRoot, "config", ...
    "AocsSimulationConfig.json"));
load_system(modelFile);
applyAocsSimulationSettings(modelName, AOCS);
cleanup = onCleanup(@() closeLoadedModels(modelName, harnessName));

existing = sltest.harness.find(owner, "Name", harnessName);
if ~isempty(existing)
    sltest.harness.delete(owner, harnessName);
end

sltest.harness.create(owner, ...
    "Name", harnessName, ...
    "Source", "Inport", ...
    "Sink", "Outport", ...
    "SaveExternally", true, ...
    "HarnessPath", harnessFile, ...
    "RebuildOnOpen", false, ...
    "SynchronizationMode", "SyncOnOpenAndClose");

sltest.harness.open(owner, harnessName);
set_param(harnessName, ...
    "SolverType", "Fixed-step", ...
    "Solver", "FixedStepDiscrete", ...
    "FixedStep", "1", ...
    "SaveOutput", "on", ...
    "OutputSaveName", "yout", ...
    "SaveFormat", "Dataset");
save_system(harnessName);
sltest.harness.close(owner, harnessName);
save_system(modelName, modelFile);

fprintf("Created %s for %s.\n", harnessFile, owner);
end

function closeLoadedModels(modelName, harnessName)
if bdIsLoaded(harnessName)
    close_system(harnessName, 0);
end
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end
