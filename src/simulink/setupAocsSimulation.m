function AOCS = setupAocsSimulation(configFile)
% Description:
%   Creates AOCS_ConfigBus and AOCS_StateBus in the base workspace, wraps the
%   numeric plant config in a Simulink.Parameter, and assigns AOCS plus
%   AOCS_Config for model evaluation.
%
% Arguments:
%   configFile - Optional path to an AocsSimulationConfig JSON file.
%
% Outputs:
%   AOCS - Validated configuration struct returned by loadAocsSimulationConfig.

projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
if nargin < 1 || strlength(string(configFile)) == 0
    configFile = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
end

addpath(fullfile(projectRoot, "src", "config"));
addpath(fullfile(projectRoot, "src", "simulink"));

AOCS = loadAocsSimulationConfig(configFile, projectRoot);
createAocsConfigBus("base");
createAocsStateBus("base");

AOCS_Config = Simulink.Parameter(AOCS.Config);
AOCS_Config.DataType = "Bus: AOCS_ConfigBus";
AOCS_Config.CoderInfo.StorageClass = "Auto";
AOCS_Config.Description = "AOCS plant configuration loaded from JSON";

assignin("base", "AOCS", AOCS);
assignin("base", "AOCS_Config", AOCS_Config);
end
