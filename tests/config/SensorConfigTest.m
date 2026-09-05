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
        function defaultConfigLoadsSensorSettings(testCase)
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
            magnetometer = AOCS.Sensors.Magnetometer;
            magnetometerBusConfig = AOCS.SensorConfig.Magnetometer;

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

            testCase.verifyTrue(magnetometer.Enabled);
            testCase.verifyEqual(magnetometer.Mode, "nominal");
            testCase.verifyEqual(magnetometer.SampleTime_s, 0.1);
            testCase.verifyEqual(magnetometerBusConfig.mode_id, 1.0);
            testCase.verifyEqual(magnetometerBusConfig.sample_time_s, ...
                magnetometer.SampleTime_s);
            testCase.verifyEqual(magnetometerBusConfig.noise_std_T, ...
                magnetometer.NoiseDensity_T_sqrt_Hz / sqrt(magnetometer.SampleTime_s), ...
                "RelTol", 1.0e-15);
            testCase.verifyEqual(magnetometerBusConfig.bias_random_walk_step_std_T, ...
                magnetometer.BiasRandomWalkStd_T_sqrt_s * sqrt(magnetometer.SampleTime_s), ...
                "RelTol", 1.0e-15);
            testCase.verifyEqual(magnetometerBusConfig.misalignment_matrix, eye(3));
            testCase.verifyEqual(magnetometerBusConfig.failure_valid, 0.0);
        end

        function sensorConfigBusesExposeSensorConfigs(testCase)
            % Description:
            %   Checks that sensor config buses publish the expected sensor
            %   configuration contracts for Simulink subsystems.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            createGyroConfigBus("base");
            createMagnetometerConfigBus("base");
            sensorBus = createSensorConfigBus();
            gyroBus = createGyroConfigBus();
            magnetometerBus = createMagnetometerConfigBus();

            testCase.verifyEqual(string({sensorBus.Elements.Name}), ...
                ["Gyro", "Magnetometer"]);
            testCase.verifyEqual(string(sensorBus.Elements(1).DataType), ...
                "Bus: GyroConfigBus");
            testCase.verifyEqual(string(sensorBus.Elements(2).DataType), ...
                "Bus: MagnetometerConfigBus");
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
            testCase.verifyEqual(string({magnetometerBus.Elements.Name}), [ ...
                "enabled", ...
                "mode_id", ...
                "sample_time_s", ...
                "noise_density_T_sqrt_Hz", ...
                "noise_std_T", ...
                "bias_initial_T", ...
                "bias_random_walk_std_T_sqrt_s", ...
                "bias_random_walk_step_std_T", ...
                "scale_factor", ...
                "misalignment_matrix", ...
                "range_T", ...
                "resolution_T", ...
                "noise_seed", ...
                "bias_seed", ...
                "failure_output_T", ...
                "failure_valid"]);
        end

        function sensorMeasurementBusesExposeSensorMeasurements(testCase)
            % Description:
            %   Checks that sensor measurement buses publish the expected sensor
            %   measurement contracts for GNC consumers.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            createGyroMeasurementBus("base");
            createMagnetometerMeasurementBus("base");
            sensorBus = createSensorMeasurementBus();
            gyroBus = createGyroMeasurementBus();
            magnetometerBus = createMagnetometerMeasurementBus();

            testCase.verifyEqual(string({sensorBus.Elements.Name}), ...
                ["Gyro", "Magnetometer"]);
            testCase.verifyEqual(string(sensorBus.Elements(1).DataType), ...
                "Bus: GyroMeasurementBus");
            testCase.verifyEqual(string(sensorBus.Elements(2).DataType), ...
                "Bus: MagnetometerMeasurementBus");
            testCase.verifyEqual(string({gyroBus.Elements.Name}), ...
                ["omega_rad_s", "valid"]);
            testCase.verifyEqual(gyroBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(gyroBus.Elements(2).Dimensions, 1.0);
            testCase.verifyEqual(string({magnetometerBus.Elements.Name}), ...
                ["B_B_T", "valid"]);
            testCase.verifyEqual(magnetometerBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(magnetometerBus.Elements(2).Dimensions, 1.0);
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
            testCase.verifyEqual(sensorConfig.Value.Magnetometer.sample_time_s, ...
                0.1);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_GyroConfig', 'var')"), 0.0);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_MagnetometerConfig', 'var')"), 0.0);
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

        function magnetometerFailureModeMapsToNumericTestMode(testCase)
            % Description:
            %   Verifies that an override config can place the magnetometer in
            %   failure mode for deterministic sensor tests.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = writeFailureMagnetometerConfig(testCase.ProjectRoot);
            cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);

            testCase.verifyEqual(AOCS.Sensors.Magnetometer.Mode, "failure");
            testCase.verifyEqual(AOCS.SensorConfig.Magnetometer.mode_id, 2.0);
            testCase.verifyEqual(AOCS.SensorConfig.Magnetometer.failure_output_T, ...
                [1.0e-6; -2.0e-6; 3.0e-6]);
            testCase.verifyEqual(AOCS.SensorConfig.Magnetometer.failure_valid, 1.0);
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

function configFile = writeFailureMagnetometerConfig(projectRoot)
% Description:
%   Writes a temporary config overriding only magnetometer failure-mode settings.
%
% Arguments:
%   projectRoot - Project root used to resolve the base AOCS config.
%
% Outputs:
%   configFile - Path to the temporary config file.

payload = struct();
payload.extends = char(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));
payload.sensors.magnetometer.mode = "failure";
payload.sensors.magnetometer.failure.output_T = [1.0e-6; -2.0e-6; 3.0e-6];
payload.sensors.magnetometer.failure.valid = true;

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
