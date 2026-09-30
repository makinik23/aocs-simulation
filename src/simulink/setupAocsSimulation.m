function [AOCS, variables] = setupAocsSimulation(configFile, publishConfiguration)
% Description:
%   Creates bus objects in the base workspace, wraps numeric config
%   payloads in Simulink.Parameters, and assigns AOCS plus config parameters
%   for model evaluation.
%
% Arguments:
%   configFile - Optional path to an AocsSimulationConfig JSON file.
%
% Outputs:
%   AOCS - Validated configuration struct returned by loadAocsSimulationConfig.

if nargin < 2
    publishConfiguration = true;
end

projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
if nargin < 1 || strlength(string(configFile)) == 0
    configFile = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
end

addpath(projectRoot);
setupAocsPaths(projectRoot);

nativeBuildDirectory = fullfile(projectRoot, "build", "native", ...
    "dtm2020", computer("arch"));
% Initialization never downloads dependencies or invokes compilers.
if isfolder(nativeBuildDirectory)
    addpath(nativeBuildDirectory);
end
AOCS_DTM2020_CoefficientFile = char(fullfile(projectRoot, "third_party", ...
    "dtm2020", "upstream", "data", "DTM_2020_F107_Kp.dat"));

AOCS = loadAocsSimulationConfig(configFile, projectRoot);
createAocsBuses();

AOCS_Config = Simulink.Parameter(AOCS.Config);
AOCS_Config.DataType = "Bus: ConfigBus";
AOCS_Config.CoderInfo.StorageClass = "Auto";
AOCS_Config.Description = "AOCS plant configuration loaded from JSON";

AOCS_OrbitConfig = Simulink.Parameter(AOCS.OrbitConfig);
AOCS_OrbitConfig.DataType = "Bus: OrbitConfigBus";
AOCS_OrbitConfig.CoderInfo.StorageClass = "Auto";
AOCS_OrbitConfig.Description = "AOCS orbit configuration loaded from JSON";

AOCS_EnvironmentConfig = Simulink.Parameter(AOCS.EnvironmentConfig);
AOCS_EnvironmentConfig.DataType = "Bus: EnvironmentConfigBus";
AOCS_EnvironmentConfig.CoderInfo.StorageClass = "Auto";
AOCS_EnvironmentConfig.Description = "AOCS environment configuration loaded from JSON";

AOCS_SensorConfig = Simulink.Parameter(AOCS.SensorConfig);
AOCS_SensorConfig.DataType = "Bus: SensorConfigBus";
AOCS_SensorConfig.CoderInfo.StorageClass = "Auto";
AOCS_SensorConfig.Description = "AOCS sensor configuration loaded from JSON";

AOCS_GNCConfig = Simulink.Parameter(AOCS.GNCConfig);
AOCS_GNCConfig.DataType = "Bus: GNCConfigBus";
AOCS_GNCConfig.CoderInfo.StorageClass = "Auto";
AOCS_GNCConfig.Description = "AOCS onboard GNC configuration loaded from JSON";

variables = struct("AOCS", AOCS, "AOCS_Config", AOCS_Config, ...
    "AOCS_OrbitConfig", AOCS_OrbitConfig, ...
    "AOCS_EnvironmentConfig", AOCS_EnvironmentConfig, ...
    "AOCS_SensorConfig", AOCS_SensorConfig, ...
    "AOCS_GNCConfig", AOCS_GNCConfig, ...
    "AOCS_DTM2020_CoefficientFile", AOCS_DTM2020_CoefficientFile);
if publishConfiguration
    names = fieldnames(variables);
    for k = 1:numel(names)
        assignin("base", names{k}, variables.(names{k}));
    end
end
end
