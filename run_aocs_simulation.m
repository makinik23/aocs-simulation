function out = run_aocs_simulation(configFile)
% Description:
%   Loads the configuration, creates the AOCS bus objects, configures the
%   Aerospace Blockset 6DOF block, runs the Simulink plant, and saves the
%   latest results to the configured results file.
%
% Arguments:
%   configFile - Optional path to an AocsSimulationConfig JSON file.
%
% Outputs:
%   out - Simulink.SimulationOutput returned by the plant simulation.

projectRoot = fileparts(mfilename("fullpath"));
addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "simulink"));
addpath(fullfile(projectRoot, "src", "environment"));

if nargin < 1 || strlength(string(configFile)) == 0
    configFile = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
end

AOCS = setupAocsSimulation(configFile);

if ~isfile(AOCS.Model.File)
    error("Model not found: %s", char(AOCS.Model.File));
end

load_system(AOCS.Model.File);
applyAocsSimulationSettings(AOCS.Model.Name, AOCS);

out = sim(AOCS.Model.Name);

if ~isfolder(AOCS.Results.Directory)
    mkdir(AOCS.Results.Directory);
end

save(AOCS.Results.File, "out", "AOCS");

disp("Simulation finished.");
disp("Results saved to " + AOCS.Results.File);
end
