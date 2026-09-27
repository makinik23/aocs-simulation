function [out, AOCS, runDirectory] = run_aocs_simulation(configFile)
%RUN_AOCS_SIMULATION Run a scoped experiment and keep its immutable evidence.
% The configured results file remains a convenience copy for plotting.
% results/runs/<unique-id>/ contains the authoritative result and provenance.
root = setupAocsPaths();
configureAocsFileGeneration();
if nargin < 1
    configFile = "";
end
requireDtm2020Native();
[simIn, AOCS] = createAocsSimulationInput(configFile);
if strcmp(get_param(AOCS.Model.Name, 'Dirty'), 'on')
    error("AOCS:Simulation:UnsavedModel", ...
        "Save or discard model edits before an archived run; source hashes describe files on disk.");
end
runs = fullfile(AOCS.Results.Directory, 'runs');
if ~isfolder(runs)
    mkdir(runs);
end
runDirectory = string(tempname(runs));
mkdir(runDirectory);
metadata = aocsRunProvenance(root);
metadata.ModelSHA256 = aocsFileHash(AOCS.Model.File);
metadata.Status = "running";
writeAocsJson(fullfile(runDirectory, 'configuration.json'), AOCS);
writeAocsJson(fullfile(runDirectory, 'metadata.json'), metadata);
try
    out = sim(simIn);
    metadata.Status = "completed";
    metadata.CompletedUTC = string(datetime('now', TimeZone='UTC'));
    save(fullfile(runDirectory, 'simulation.mat'), 'out', 'AOCS', 'metadata');
    save(AOCS.Results.File, 'out', 'AOCS', 'metadata');
catch problem
    metadata.Status = "failed";
    metadata.ErrorIdentifier = string(problem.identifier);
    metadata.ErrorMessage = string(problem.message);
    writeAocsJson(fullfile(runDirectory, 'metadata.json'), metadata);
    rethrow(problem);
end
writeAocsJson(fullfile(runDirectory, 'metadata.json'), metadata);
fprintf('Simulation evidence: %s\n', runDirectory);
end
