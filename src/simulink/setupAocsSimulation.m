function AOCS = setupAocsSimulation(configFile)
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

projectRoot = fileparts(fileparts(fileparts(mfilename("fullpath"))));
if nargin < 1 || strlength(string(configFile)) == 0
    configFile = fullfile(projectRoot, "config", "AocsSimulationConfig.json");
end

addpath(projectRoot);
setupAocsPaths(projectRoot);

nativeBuildDirectory = fullfile(projectRoot, "build", "native", ...
    "dtm2020", computer("arch"));
nativeSFunction = fullfile(nativeBuildDirectory, "dtm2020_sfun." + mexext);
if ~isfile(nativeSFunction)
    addpath(fullfile(projectRoot, "tools"));
    nativeArtifacts = buildDtm2020Native();
    nativeBuildDirectory = nativeArtifacts.BuildDirectory;
end
addpath(nativeBuildDirectory);
AOCS_DTM2020_CoefficientFile = char(fullfile(projectRoot, "third_party", ...
    "dtm2020", "upstream", "data", "DTM_2020_F107_Kp.dat"));

AOCS = loadAocsSimulationConfig(configFile, projectRoot);
createConfigBus("base");
createOrbitConfigBus("base");
createEnvironmentConfigBus("base");
createAttitudeStateBus("base");
createOrbitStateBus("base");
createEnvironmentContextBus("base");
createAtmosphereBus("base");
createMagneticFieldBus("base");
createSunBus("base");
createIlluminationBus("base");
createSrpBus("base");
createDisturbanceBus("base");
createEnvironmentBus("base");
createPlantStateBus("base");
createGyroConfigBus("base");
createMagnetometerConfigBus("base");
createSensorConfigBus("base");
createGyroMeasurementBus("base");
createMagnetometerMeasurementBus("base");
createSensorMeasurementBus("base");

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

assignin("base", "AOCS", AOCS);
assignin("base", "AOCS_Config", AOCS_Config);
assignin("base", "AOCS_OrbitConfig", AOCS_OrbitConfig);
assignin("base", "AOCS_EnvironmentConfig", AOCS_EnvironmentConfig);
assignin("base", "AOCS_SensorConfig", AOCS_SensorConfig);
assignin("base", "AOCS_DTM2020_CoefficientFile", AOCS_DTM2020_CoefficientFile);
end
