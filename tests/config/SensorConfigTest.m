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
            css = AOCS.Sensors.CoarseSunSensors;
            cssBusConfig = AOCS.SensorConfig.CoarseSunSensors;
            gnss = AOCS.Sensors.GNSS;
            gnssBusConfig = AOCS.SensorConfig.GNSS;

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

            testCase.verifyTrue(css.Enabled);
            testCase.verifyEqual(css.Mode, "nominal");
            testCase.verifyEqual(css.SampleTime_s, 0.1);
            testCase.verifyEqual(cssBusConfig.mode_id, 1.0);
            testCase.verifyEqual(cssBusConfig.sample_time_s, css.SampleTime_s);
            testCase.verifyEqual(cssBusConfig.panel_normals_B, ...
                [1 -1 0 0 0 0; 0 0 1 -1 0 0; 0 0 0 0 1 -1]);
            testCase.verifyEqual(cssBusConfig.fov_cos, ...
                cos(css.FovHalfAngle_rad), "AbsTol", 1.0e-15);
            testCase.verifyEqual(cssBusConfig.noise_std_W_m2, ...
                css.NoiseDensity_W_m2_sqrt_Hz / sqrt(css.SampleTime_s), ...
                "RelTol", 1.0e-15);
            testCase.verifyEqual(cssBusConfig.failure_valid, 0.0);

            testCase.verifyTrue(gnss.Enabled);
            testCase.verifyEqual(gnss.Mode, "nominal");
            testCase.verifyEqual(gnss.SampleTime_s, 1.0);
            testCase.verifyEqual(gnssBusConfig.mode_id, 1.0);
            testCase.verifyEqual(gnssBusConfig.sample_time_s, ...
                gnss.SampleTime_s);
            testCase.verifyEqual(gnss.AcquisitionTime_s, 30.0);
            testCase.verifyEqual(gnss.Ionosphere.VerticalTec_tecu, 15.0);
            testCase.verifyEqual(gnssBusConfig.radial_ionosphere_bias_m, 9.6, ...
                "AbsTol", 1.0e-14);
            testCase.verifyEqual( ...
                gnssBusConfig.once_per_orbit_angular_rate_rad_s, ...
                2.0 * pi / 5760.0, "RelTol", 1.0e-15);
            testCase.verifyEqual( ...
                gnssBusConfig.position_periodic_amplitude_RTN_m, ...
                [0.0; 0.0; 2.0]);
            testCase.verifyEqual( ...
                gnssBusConfig.velocity_periodic_amplitude_RTN_m_s, ...
                [0.0; 0.0; 0.05]);
            expectedAlpha = exp(-gnss.SampleTime_s / ...
                gnss.GaussMarkov.CorrelationTime_s);
            expectedScale = sqrt(1.0 - expectedAlpha ^ 2);
            testCase.verifyEqual(gnssBusConfig.gauss_markov_alpha, ...
                expectedAlpha, "RelTol", 1.0e-15);
            testCase.verifyEqual( ...
                gnssBusConfig.position_gauss_markov_step_std_RTN_m, ...
                expectedScale * [10.0; 5.0; 3.5], "RelTol", 1.0e-15);
            testCase.verifyEqual( ...
                gnssBusConfig.position_white_noise_std_RTN_m, ...
                [0.5; 0.5; 0.5]);
            testCase.verifyEqual(gnssBusConfig.failure_valid, 0.0);
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
            createCoarseSunSensorConfigBus("base");
            createGnssConfigBus("base");
            sensorBus = createSensorConfigBus();
            gyroBus = createGyroConfigBus();
            magnetometerBus = createMagnetometerConfigBus();
            cssBus = createCoarseSunSensorConfigBus();
            gnssBus = createGnssConfigBus();

            testCase.verifyEqual(string({sensorBus.Elements.Name}), ...
                ["Gyro", "Magnetometer", "CoarseSunSensors", "GNSS"]);
            testCase.verifyEqual(string(sensorBus.Elements(1).DataType), ...
                "Bus: GyroConfigBus");
            testCase.verifyEqual(string(sensorBus.Elements(2).DataType), ...
                "Bus: MagnetometerConfigBus");
            testCase.verifyEqual(string(sensorBus.Elements(3).DataType), ...
                "Bus: CoarseSunSensorConfigBus");
            testCase.verifyEqual(string(sensorBus.Elements(4).DataType), ...
                "Bus: GnssConfigBus");
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
            testCase.verifyEqual(string({cssBus.Elements.Name}), [ ...
                "enabled", ...
                "mode_id", ...
                "sample_time_s", ...
                "panel_normals_B", ...
                "fov_cos", ...
                "min_valid_irradiance_W_m2", ...
                "noise_density_W_m2_sqrt_Hz", ...
                "noise_std_W_m2", ...
                "bias_W_m2", ...
                "scale_factor", ...
                "range_W_m2", ...
                "resolution_W_m2", ...
                "noise_seed", ...
                "failure_panel_signals_W_m2", ...
                "failure_sun_B_unit", ...
                "failure_irradiance_W_m2", ...
                "failure_valid"]);
            testCase.verifyEqual(string({gnssBus.Elements.Name}), [ ...
                "enabled", ...
                "mode_id", ...
                "sample_time_s", ...
                "acquisition_time_s", ...
                "dropout_probability_per_sample", ...
                "radial_ionosphere_bias_m", ...
                "once_per_orbit_angular_rate_rad_s", ...
                "once_per_orbit_phase_rad", ...
                "position_periodic_amplitude_RTN_m", ...
                "velocity_periodic_amplitude_RTN_m_s", ...
                "gauss_markov_alpha", ...
                "position_gauss_markov_step_std_RTN_m", ...
                "velocity_gauss_markov_step_std_RTN_m_s", ...
                "position_white_noise_std_RTN_m", ...
                "velocity_white_noise_std_RTN_m_s", ...
                "position_resolution_m", ...
                "velocity_resolution_m_s", ...
                "position_gauss_markov_seed", ...
                "velocity_gauss_markov_seed", ...
                "position_white_noise_seed", ...
                "velocity_white_noise_seed", ...
                "dropout_seed", ...
                "failure_r_I_m", ...
                "failure_v_I_m_s", ...
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
            createCoarseSunSensorMeasurementBus("base");
            createGnssMeasurementBus("base");
            sensorBus = createSensorMeasurementBus();
            gyroBus = createGyroMeasurementBus();
            magnetometerBus = createMagnetometerMeasurementBus();
            cssBus = createCoarseSunSensorMeasurementBus();
            gnssBus = createGnssMeasurementBus();

            testCase.verifyEqual(string({sensorBus.Elements.Name}), ...
                ["Gyro", "Magnetometer", "CoarseSunSensors", "GNSS"]);
            testCase.verifyEqual(string(sensorBus.Elements(1).DataType), ...
                "Bus: GyroMeasurementBus");
            testCase.verifyEqual(string(sensorBus.Elements(2).DataType), ...
                "Bus: MagnetometerMeasurementBus");
            testCase.verifyEqual(string(sensorBus.Elements(3).DataType), ...
                "Bus: CoarseSunSensorMeasurementBus");
            testCase.verifyEqual(string(sensorBus.Elements(4).DataType), ...
                "Bus: GnssMeasurementBus");
            testCase.verifyEqual(string({gyroBus.Elements.Name}), ...
                ["omega_rad_s", "valid"]);
            testCase.verifyEqual(gyroBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(gyroBus.Elements(2).Dimensions, 1.0);
            testCase.verifyEqual(string({magnetometerBus.Elements.Name}), ...
                ["B_B_T", "valid"]);
            testCase.verifyEqual(magnetometerBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(magnetometerBus.Elements(2).Dimensions, 1.0);
            testCase.verifyEqual(string({cssBus.Elements.Name}), ...
                ["sun_B_unit", "irradiance_W_m2", ...
                "panel_signals_W_m2", "valid"]);
            testCase.verifyEqual(cssBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(cssBus.Elements(3).Dimensions, [6 1]);
            testCase.verifyEqual(cssBus.Elements(4).Dimensions, 1.0);
            testCase.verifyEqual(string({gnssBus.Elements.Name}), ...
                ["r_I_m", "v_I_m_s", "valid"]);
            testCase.verifyEqual(gnssBus.Elements(1).Dimensions, [3 1]);
            testCase.verifyEqual(gnssBus.Elements(2).Dimensions, [3 1]);
            testCase.verifyEqual(gnssBus.Elements(3).Dimensions, 1.0);
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
            testCase.verifyEqual(sensorConfig.Value.CoarseSunSensors.sample_time_s, ...
                0.1);
            testCase.verifyEqual(sensorConfig.Value.GNSS.sample_time_s, 1.0);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_GyroConfig', 'var')"), 0.0);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_MagnetometerConfig', 'var')"), 0.0);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_CoarseSunSensorConfig', 'var')"), 0.0);
            testCase.verifyEqual(evalin("base", ...
                "exist('AOCS_GnssConfig', 'var')"), 0.0);
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

        function coarseSunSensorFailureModeMapsToNumericTestMode(testCase)
            % Description:
            %   Verifies that an override config can place the coarse sun
            %   sensors in failure mode for deterministic sensor tests.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            configFile = writeFailureCoarseSunSensorsConfig(testCase.ProjectRoot);
            cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);

            testCase.verifyEqual(AOCS.Sensors.CoarseSunSensors.Mode, "failure");
            testCase.verifyEqual(AOCS.SensorConfig.CoarseSunSensors.mode_id, 2.0);
            testCase.verifyEqual( ...
                AOCS.SensorConfig.CoarseSunSensors.failure_panel_signals_W_m2, ...
                [100.0; 0.0; 20.0; 0.0; 0.0; 0.0]);
            testCase.verifyEqual( ...
                AOCS.SensorConfig.CoarseSunSensors.failure_sun_B_unit, ...
                [1.0; 0.0; 0.0]);
            testCase.verifyEqual( ...
                AOCS.SensorConfig.CoarseSunSensors.failure_irradiance_W_m2, ...
                100.0);
            testCase.verifyEqual(AOCS.SensorConfig.CoarseSunSensors.failure_valid, ...
                1.0);
        end

        function gnssModesMapToNumericTestModes(testCase)
            % Description:
            %   Verifies GNSS dropout and failure modes plus deterministic
            %   failure outputs used by sensor and estimator tests.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            dropoutConfig = writeGnssModeConfig(testCase.ProjectRoot, "dropout");
            cleanupDropout = onCleanup(@() deleteIfFileExists(dropoutConfig));
            failureConfig = writeGnssModeConfig(testCase.ProjectRoot, "failure");
            cleanupFailure = onCleanup(@() deleteIfFileExists(failureConfig));

            dropout = loadAocsSimulationConfig(dropoutConfig, testCase.ProjectRoot);
            failure = loadAocsSimulationConfig(failureConfig, testCase.ProjectRoot);

            testCase.verifyEqual(dropout.Sensors.GNSS.Mode, "dropout");
            testCase.verifyEqual(dropout.SensorConfig.GNSS.mode_id, 2.0);
            testCase.verifyEqual(failure.Sensors.GNSS.Mode, "failure");
            testCase.verifyEqual(failure.SensorConfig.GNSS.mode_id, 3.0);
            testCase.verifyEqual(failure.SensorConfig.GNSS.failure_r_I_m, ...
                [7.0e6; 1.0; -2.0]);
            testCase.verifyEqual(failure.SensorConfig.GNSS.failure_v_I_m_s, ...
                [0.0; 7.5e3; 3.0]);
            testCase.verifyEqual(failure.SensorConfig.GNSS.failure_valid, 1.0);

            clear cleanupDropout cleanupFailure
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

function configFile = writeFailureCoarseSunSensorsConfig(projectRoot)
% Description:
%   Writes a temporary config overriding only CSS failure-mode settings.
%
% Arguments:
%   projectRoot - Project root used to resolve the base AOCS config.
%
% Outputs:
%   configFile - Path to the temporary config file.

payload = struct();
payload.extends = char(fullfile(projectRoot, "config", "AocsSimulationConfig.json"));
payload.sensors.coarse_sun_sensors.mode = "failure";
payload.sensors.coarse_sun_sensors.failure.panel_signals_W_m2 = ...
    [100.0; 0.0; 20.0; 0.0; 0.0; 0.0];
payload.sensors.coarse_sun_sensors.failure.sun_B_unit = [1.0; 0.0; 0.0];
payload.sensors.coarse_sun_sensors.failure.irradiance_W_m2 = 100.0;
payload.sensors.coarse_sun_sensors.failure.valid = true;

configFile = string(tempname) + ".json";
fid = fopen(configFile, "w");
cleanupFile = onCleanup(@() fclose(fid));
fprintf(fid, "%s", jsonencode(payload, "PrettyPrint", true));
delete(cleanupFile);
end

function configFile = writeGnssModeConfig(projectRoot, mode)
% Description:
%   Writes a temporary config overriding GNSS mode and failure outputs.
%
% Arguments:
%   projectRoot - Project root used to resolve the base AOCS config.
%   mode - GNSS mode string to write.
%
% Outputs:
%   configFile - Path to the temporary config file.

payload = struct();
payload.extends = char(fullfile(projectRoot, "config", ...
    "AocsSimulationConfig.json"));
payload.sensors.gnss.mode = char(mode);
payload.sensors.gnss.failure.r_I_m = [7.0e6; 1.0; -2.0];
payload.sensors.gnss.failure.v_I_m_s = [0.0; 7.5e3; 3.0];
payload.sensors.gnss.failure.valid = true;

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
