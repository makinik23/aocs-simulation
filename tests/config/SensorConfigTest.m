classdef SensorConfigTest < matlab.unittest.TestCase

    properties
        ProjectRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            % Description:
            %   Locates the project root and adds source/test helper paths.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            testCase.ProjectRoot = projectRoot();
            setupAocsPaths(testCase.ProjectRoot, true);
        end
    end

    methods (Test)
        function defaultConfigLoadsGyroSettings(testCase)
            % Description:
            %   Verifies that config/sensors.json reaches the validated AOCS
            %   struct and numeric Simulink sensor config payload.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            AOCS = loadAocsSimulationConfig(fullfile(testCase.ProjectRoot, ...
                "config", "AocsSimulationConfig.json"), testCase.ProjectRoot);
            gyro = AOCS.Sensors.Gyro;
            gyroBusConfig = AOCS.SensorConfig.Gyro;

            testCase.verifyTrue(gyro.Enabled);
            testCase.verifyEqual(gyro.Mode, "nominal");
            testCase.verifyEqual(gyro.SampleTime_s, 0.1);
            testCase.verifyEqual(gyroBusConfig.mode_id, 1.0);
            testCase.verifyEqual(gyroBusConfig.gyro_sample_time_s, gyro.SampleTime_s);
            testCase.verifyEqual(gyroBusConfig.noise_std_rad_s, ...
                gyro.NoiseDensity_rad_s_sqrt_Hz / sqrt(gyro.SampleTime_s), ...
                "RelTol", 1.0e-15);
            testCase.verifyEqual(gyroBusConfig.bias_random_walk_step_std_rad_s, ...
                gyro.BiasRandomWalkStd_rad_s_sqrt_s * sqrt(gyro.SampleTime_s), ...
                "RelTol", 1.0e-15);
            testCase.verifyEqual(gyroBusConfig.misalignment_matrix, eye(3));
            testCase.verifyEqual(gyroBusConfig.failure_valid, 0.0);
        end

        function sensorConfigBusesExposeGyroConfig(testCase)
            % Description:
            %   Checks that sensor config buses publish the expected gyro
            %   configuration contract for Simulink subsystems.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            createGyroConfigBus("base");
            sensorBus = createSensorConfigBus();
            gyroBus = createGyroConfigBus();

            testCase.verifyEqual(string({sensorBus.Elements.Name}), "Gyro");
            testCase.verifyEqual(string(sensorBus.Elements(1).DataType), ...
                "Bus: GyroConfigBus");
            testCase.verifyEqual(string({gyroBus.Elements.Name}), [ ...
                "enabled", ...
                "mode_id", ...
                "gyro_sample_time_s", ...
                "noise_density_rad_s_sqrt_Hz", ...
                "noise_std_rad_s", ...
                "bias_initial_rad_s", ...
                "bias_random_walk_std_rad_s_sqrt_s", ...
                "bias_random_walk_step_std_rad_s", ...
                "scale_factor", ...
                "misalignment_matrix", ...
                "range_rad_s", ...
                "resolution_rad_s", ...
                "noise_seed", ...
                "bias_seed", ...
                "failure_output_rad_s", ...
                "failure_valid"]);
        end

        function sensorMeasurementBusesExposeGyroMeasurement(testCase)
            % Description:
            %   Checks that sensor measurement buses publish the expected gyro
            %   measurement contract for GNC consumers.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            createGyroMeasurementBus("base");
            sensorBus = createSensorMeasurementBus();
            gyroBus = createGyroMeasurementBus();

            testCase.verifyEqual(string({sensorBus.Elements.Name}), "Gyro");
            testCase.verifyEqual(string(sensorBus.Elements(1).DataType), ...
                "Bus: GyroMeasurementBus");
            testCase.verifyEqual(string({gyroBus.Elements.Name}), ...
                ["omega_rad_s", "valid"]);
            testCase.verifyEqual(gyroBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(gyroBus.Elements(2).Dimensions, 1.0);
        end

        function setupAssignsOnlyTopLevelSensorConfigParameter(testCase)
            % Description:
            %   Verifies that setupAocsSimulation assigns only the top-level
            %   sensor config Simulink.Parameter into the MATLAB base workspace.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            setupAocsSimulation(fullfile(testCase.ProjectRoot, ...
                "config", "AocsSimulationConfig.json"));

            sensorConfig = evalin("base", "AOCS_SensorConfig");

            testCase.verifyEqual(string(sensorConfig.DataType), ...
                "Bus: SensorConfigBus");
            testCase.verifyEqual(sensorConfig.Value.Gyro.gyro_sample_time_s, ...
                0.1);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_GyroConfig', 'var')"), 0.0);
        end

        function failureModeMapsToNumericTestMode(testCase)
            % Description:
            %   Verifies that an override config can place the gyro in failure
            %   mode for deterministic sensor tests.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = writeFailureGyroConfig(testCase.ProjectRoot);
            cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);

            testCase.verifyEqual(AOCS.Sensors.Gyro.Mode, "failure");
            testCase.verifyEqual(AOCS.SensorConfig.Gyro.mode_id, 2.0);
            testCase.verifyEqual(AOCS.SensorConfig.Gyro.failure_output_rad_s, ...
                [0.01; -0.02; 0.03]);
            testCase.verifyEqual(AOCS.SensorConfig.Gyro.failure_valid, 1.0);
        end
    end
end

function configFile = writeFailureGyroConfig(projectRoot)
% Description:
%   Writes a temporary config overriding only gyro failure-mode settings.
%
% Arguments:
%   projectRoot - Project root used to resolve the base AOCS config.
%
% Outputs:
%   configFile - Path to the temporary config file.

payload = struct();
payload.extends = char(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));
payload.sensors.gyro.mode = "failure";
payload.sensors.gyro.failure.output_rad_s = [0.01; -0.02; 0.03];
payload.sensors.gyro.failure.valid = true;

configFile = string(tempname) + ".json";
fid = fopen(configFile, "w");
cleanupFile = onCleanup(@() fclose(fid));
fprintf(fid, "%s", jsonencode(payload, "PrettyPrint", true));
delete(cleanupFile);
end

function deleteIfFileExists(fileName)
% Description:
%   Deletes a temporary file when it exists.
%
% Arguments:
%   fileName - File path to delete.
%
% Outputs:
%   None.

if isfile(fileName)
    delete(fileName);
end
end
