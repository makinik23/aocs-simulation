classdef SensorsTest < matlab.unittest.TestCase
    %SENSORSTEST Single suite for sensor configuration, models and acquisition.
    % Read the sections in pipeline order; tests do not depend on that order.
    % Run all: runtests("tests/sensors/SensorsTest.m")
    % Run a section: runtests("tests/sensors/SensorsTest.m", "Tag", "Acquisition")
    % Harness builders and fixture helpers are local functions below the class.

    properties (SetAccess = private)
        ProjectRoot
        ModelName = "aocs_plant"
    end

    methods (TestClassSetup)
        function prepareSensorTestEnvironment(testCase)
            testCase.ProjectRoot = projectRoot();
            setupAocsPaths(testCase.ProjectRoot, true);
            setupAocsSimulation(fullfile(testCase.ProjectRoot, ...
                "config", "AocsSimulationConfig.json"));
        end
    end

    methods (TestMethodTeardown)
        function closePlantModel(testCase)
            closeLoadedSystem(testCase.ModelName);
        end
    end

    %% Configuration loading and sensor operating modes
    methods (Test, TestTags = "Configuration")
        function configurationLoadsDefaultSensorSettings(testCase)
            % Verifies that config/sensors.json reaches the validated AOCS
            % struct and numeric Simulink sensor config payload.

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

        function configurationPublishesTopLevelSensorParameter(testCase)
            % Verifies that setupAocsSimulation assigns only the top-level
            % sensor config Simulink.Parameter into the MATLAB base workspace.

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

        function configurationMapsGyroFailureMode(testCase)
            % Verifies that an override config can place the gyro in failure
            % mode for deterministic sensor tests.

            configFile = writeFailureGyroConfig(testCase.ProjectRoot);
            cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);

            testCase.verifyEqual(AOCS.Sensors.Gyro.Mode, "failure");
            testCase.verifyEqual(AOCS.SensorConfig.Gyro.mode_id, 2.0);
            testCase.verifyEqual(AOCS.SensorConfig.Gyro.failure_output_rad_s, ...
                [0.01; -0.02; 0.03]);
            testCase.verifyEqual(AOCS.SensorConfig.Gyro.failure_valid, 1.0);
        end

        function configurationMapsMagnetometerFailureMode(testCase)
            % Verifies that an override config can place the magnetometer in
            % failure mode for deterministic sensor tests.

            configFile = writeFailureMagnetometerConfig(testCase.ProjectRoot);
            cleanupConfig = onCleanup(@() deleteIfFileExists(configFile));

            AOCS = loadAocsSimulationConfig(configFile, testCase.ProjectRoot);

            testCase.verifyEqual(AOCS.Sensors.Magnetometer.Mode, "failure");
            testCase.verifyEqual(AOCS.SensorConfig.Magnetometer.mode_id, 2.0);
            testCase.verifyEqual(AOCS.SensorConfig.Magnetometer.failure_output_T, ...
                [1.0e-6; -2.0e-6; 3.0e-6]);
            testCase.verifyEqual(AOCS.SensorConfig.Magnetometer.failure_valid, 1.0);
        end

        function configurationMapsCssFailureMode(testCase)
            % Verifies that an override config can place the coarse sun
            % sensors in failure mode for deterministic sensor tests.

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

        function configurationMapsGnssOperatingModes(testCase)
            % Verifies GNSS dropout and failure modes plus deterministic
            % failure outputs used by sensor and estimator tests.

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

    %% Raw measurement and onboard report bus contracts
    methods (Test, TestTags = "Contracts")
        function contractsDefineSensorConfigurationBuses(testCase)
            % Checks that sensor config buses publish the expected sensor
            % configuration contracts for Simulink subsystems.

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

        function contractsDefineRawMeasurementBuses(testCase)
            % Checks that sensor measurement buses publish the expected sensor
            % measurement contracts for GNC consumers.

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

        function contractsSeparateDriverMetadataFromMeasurements(testCase)
            reportBus = createSensorReportBus("base");
            specs = sensorDriverSpecs();
            testCase.verifyEqual(string({reportBus.Elements.Name}), [specs.Key]);
            for index = 1:numel(specs)
                raw = evalin('base', specs(index).RawBus);
                report = evalin('base', specs(index).ReportBus);
                rawNames = string({raw.Elements.Name});
                testCase.verifyFalse(any(ismember(rawNames, ...
                    ["sample_time_s", "receive_time_s", "sequence_id"])));
                testCase.verifyEqual(string({report.Elements.Name}), ...
                    [rawNames, "receive_time_s", "sequence_id"]);
                testCase.verifyEqual(string(report.Elements(end-1).Unit), "s");
                testCase.verifyEqual(string(report.Elements(end).DataType), "uint32");
                testCase.verifyEqual(report.Elements(end).Dimensions, 1);
            end
        end

    end

    %% Production model connections and sample periods
    methods (Test, TestTags = "Wiring")
        function wiringConnectsMagnetometerMeasurementBus(testCase)
            % Verifies that the magnetometer subsystem consumes its sensor
            % config and publishes a MagnetometerMeasurementBus.

            load_system(fullfile(testCase.ProjectRoot, "models", ...
                testCase.ModelName + ".slx"));

            magnetometer = testCase.ModelName + "/Sensors/Magnetometer";
            sampleHold = find_system(magnetometer, "SearchDepth", 1, ...
                "BlockType", "ZeroOrderHold");
            busAssembly = magnetometer + "/Magnetometer Measurement Bus Assembly";

            testCase.verifyEqual(string(get_param(magnetometer + ...
                "/MagnetometerConfig", "Value")), ...
                "AOCS_SensorConfig.Magnetometer");
            testCase.verifyEqual(string(get_param(magnetometer + ...
                "/MagnetometerConfig", "OutDataTypeStr")), ...
                "Bus: MagnetometerConfigBus");
            testCase.verifyEqual(string(get_param(sampleHold{1}, ...
                "SampleTime")), ...
                "AOCS_SensorConfig.Magnetometer.sample_time_s");
            testCase.verifyEqual(string(get_param(busAssembly, ...
                "OutDataTypeStr")), ...
                "Bus: MagnetometerMeasurementBus");
            testCase.verifyBusInputName(busAssembly, 1, "B_B_T");
            testCase.verifyBusInputName(busAssembly, 2, "valid");
        end

        function wiringConnectsCssMeasurementBus(testCase)
            % Verifies that the CSS subsystem consumes its sensor config and
            % publishes a CoarseSunSensorMeasurementBus.

            load_system(fullfile(testCase.ProjectRoot, "models", ...
                testCase.ModelName + ".slx"));

            css = testCase.ModelName + "/Sensors/Coarse Sun Sensors";
            busAssembly = css + "/CSS Measurement Bus Assembly";

            testCase.verifyInputSource(css, 1, ...
                testCase.ModelName + "/Sensors/Bus" + newline + "Selector");
            testCase.verifyEqual(string(get_param(css + "/CssConfig", "Value")), ...
                "AOCS_SensorConfig.CoarseSunSensors");
            testCase.verifyEqual(string(get_param(css + "/CssConfig", ...
                "OutDataTypeStr")), "Bus: CoarseSunSensorConfigBus");
            testCase.verifyEqual(string(get_param(css + "/W", "SampleTime")), ...
                "AOCS_SensorConfig.CoarseSunSensors.sample_time_s");
            testCase.verifyEqual(string(get_param(busAssembly, "OutDataTypeStr")), ...
                "Bus: CoarseSunSensorMeasurementBus");
            testCase.verifyBusInputName(busAssembly, 1, "sun_B_unit");
            testCase.verifyBusInputName(busAssembly, 2, "irradiance_W_m2");
            testCase.verifyBusInputName(busAssembly, 3, "panel_signals_W_m2");
            testCase.verifyBusInputName(busAssembly, 4, "valid");
        end

        function wiringConnectsGnssMeasurementBus(testCase)
            % Verifies that GNSS consumes OrbitState, applies its configured
            % sample rate and RTN PVT error model, and publishes a
            % GnssMeasurementBus.

            load_system(fullfile(testCase.ProjectRoot, "models", ...
                testCase.ModelName + ".slx"));

            gnss = testCase.ModelName + "/Sensors/GNSS";
            busAssembly = gnss + "/GNSS Measurement Bus Assembly";
            errorModel = gnss + "/PVT Error Model";
            availability = gnss + "/Fix Availability";
            outputSelection = gnss + "/Output Selection";
            sampleHolds = find_system(gnss, "SearchDepth", 1, ...
                "BlockType", "ZeroOrderHold");
            noiseSources = [
                errorModel + "/Position Error RTN/Gauss-Markov/Innovation W"
                errorModel + "/Velocity Error RTN/Gauss-Markov/Innovation W"
                errorModel + "/Position Error RTN/White Noise W"
                errorModel + "/Velocity Error RTN/White Noise W"
            ];
            stateDelays = [
                errorModel + "/Position Error RTN/Gauss-Markov/State Delay"
                errorModel + "/Velocity Error RTN/Gauss-Markov/State Delay"
            ];

            testCase.verifyInputSource(gnss, 1, ...
                testCase.ModelName + "/Sensors/Bus" + newline + "Selector");
            testCase.verifyEqual(string(get_param(gnss + "/GnssConfig", ...
                "Value")), "AOCS_SensorConfig.GNSS");
            testCase.verifyEqual(string(get_param(gnss + "/GnssConfig", ...
                "OutDataTypeStr")), "Bus: GnssConfigBus");
            testCase.verifyNumElements(sampleHolds, 3);
            testCase.verifyGreaterThan(getSimulinkBlockHandle(errorModel), 0);
            testCase.verifyGreaterThan(getSimulinkBlockHandle(availability), 0);
            testCase.verifyGreaterThan(getSimulinkBlockHandle( ...
                outputSelection + "/Position Output"), 0);
            testCase.verifyGreaterThan(getSimulinkBlockHandle( ...
                outputSelection + "/Velocity Output"), 0);
            testCase.verifyGreaterThan(getSimulinkBlockHandle( ...
                outputSelection + "/Validity Output"), 0);
            testCase.verifyNumElements(find_system(outputSelection, ...
                "SearchDepth", 1, "BlockType", "Goto"), 2);
            testCase.verifyNumElements(find_system(outputSelection, ...
                "SearchDepth", 1, "BlockType", "From"), 6);
            for index = 1:numel(sampleHolds)
                testCase.verifyEqual(string(get_param(sampleHolds{index}, ...
                    "SampleTime")), "AOCS_SensorConfig.GNSS.sample_time_s");
            end
            for index = 1:numel(noiseSources)
                testCase.verifyGreaterThan(getSimulinkBlockHandle(noiseSources(index)), 0);
                testCase.verifyEqual(string(get_param(noiseSources(index), ...
                    "SampleTime")), "AOCS_SensorConfig.GNSS.sample_time_s");
            end
            for index = 1:numel(stateDelays)
                testCase.verifyGreaterThan(getSimulinkBlockHandle(stateDelays(index)), 0);
                testCase.verifyEqual(string(get_param(stateDelays(index), ...
                    "SampleTime")), "AOCS_SensorConfig.GNSS.sample_time_s");
            end
            testCase.verifyEqual(string(get_param(errorModel + ...
                "/Position Error RTN/Orbital Phase Sine", "Frequency")), ...
                "AOCS_SensorConfig.GNSS.once_per_orbit_angular_rate_rad_s");
            testCase.verifyEqual(string(get_param(availability + ...
                "/Dropout Uniform", "Seed")), ...
                "AOCS_SensorConfig.GNSS.dropout_seed");
            testCase.verifyEqual(string(get_param(busAssembly, ...
                "OutDataTypeStr")), "Bus: GnssMeasurementBus");
            testCase.verifyBusInputName(busAssembly, 1, "r_I_m");
            testCase.verifyBusInputName(busAssembly, 2, "v_I_m_s");
            testCase.verifyBusInputName(busAssembly, 3, "valid");
            testCase.verifyNoDanglingLines(gnss);
        end

        function wiringRoutesSensorMeasurementsThroughDriversToGnc(testCase)
            % Verifies that the Sensors subsystem assembles gyro,
            % magnetometer, coarse sun sensor, and GNSS measurements into
            % SensorMeasurementBus and feeds GNC through the driver layer.

            load_system(fullfile(testCase.ProjectRoot, "models", ...
                testCase.ModelName + ".slx"));

            busAssembly = testCase.ModelName + ...
                "/Sensors/Sensor Measurement Bus Assembly";

            testCase.verifyEqual(string(get_param(busAssembly, "OutDataTypeStr")), ...
                "Bus: SensorMeasurementBus");
            testCase.verifyBusInputName(busAssembly, 1, "Gyro");
            testCase.verifyBusInputName(busAssembly, 2, "Magnetometer");
            testCase.verifyBusInputName(busAssembly, 3, "CoarseSunSensors");
            testCase.verifyBusInputName(busAssembly, 4, "GNSS");
            testCase.verifyInputSource(testCase.ModelName + "/GNC", 1, ...
                testCase.ModelName + "/Drivers");
            testCase.verifyInputSource(testCase.ModelName + "/Drivers", 1, ...
                testCase.ModelName + "/Sensors");
        end

    end

    %% GNSS RTN error mapping independent of Simulink
    methods (Test, TestTags = "MeasurementModels")
        function measurementGnssMapsAlignedRtnErrors(testCase)
            % Verifies RTN-to-inertial mapping for an orbit aligned with
            % the inertial coordinate axes.

            rTruth_I_m = [7.0e6; 0.0; 0.0];
            vTruth_I_m_s = [0.0; 7.5e3; 0.0];
            positionError_RTN_m = [10.0; 20.0; 30.0];
            velocityError_RTN_m_s = [0.01; 0.02; 0.03];

            [rMeasured, vMeasured, C_I_RTN] = applyGnssRtnErrors( ...
                rTruth_I_m, vTruth_I_m_s, positionError_RTN_m, ...
                velocityError_RTN_m_s);

            testCase.verifyEqual(C_I_RTN, eye(3), "AbsTol", 1.0e-15);
            testCase.verifyEqual(rMeasured, ...
                rTruth_I_m + positionError_RTN_m, "AbsTol", 1.0e-12);
            testCase.verifyEqual(vMeasured, ...
                vTruth_I_m_s + velocityError_RTN_m_s, "AbsTol", 1.0e-12);
        end

        function measurementGnssUsesRightHandedRtnFrame(testCase)
            % Verifies radial, transverse, and normal axes for a rotated
            % prograde equatorial orbit.

            rTruth_I_m = [0.0; 7.0e6; 0.0];
            vTruth_I_m_s = [-7.5e3; 0.0; 0.0];
            expectedC = [0.0 -1.0 0.0; 1.0 0.0 0.0; 0.0 0.0 1.0];

            [rMeasured, ~, C_I_RTN] = applyGnssRtnErrors( ...
                rTruth_I_m, vTruth_I_m_s, [1.0; 2.0; 3.0], zeros(3, 1));

            testCase.verifyEqual(C_I_RTN, expectedC, "AbsTol", 1.0e-15);
            testCase.verifyEqual(rMeasured - rTruth_I_m, ...
                expectedC * [1.0; 2.0; 3.0], "AbsTol", 1.0e-12);
            testCase.verifyEqual(C_I_RTN' * C_I_RTN, eye(3), ...
                "AbsTol", 1.0e-15);
            testCase.verifyEqual(det(C_I_RTN), 1.0, "AbsTol", 1.0e-15);
        end

        function measurementGnssHandlesDegenerateTruthState(testCase)
            % Verifies deterministic finite behavior for an invalid zero
            % truth state instead of producing NaN measurements.

            [rMeasured, vMeasured, C_I_RTN] = applyGnssRtnErrors( ...
                zeros(3, 1), zeros(3, 1), [1.0; 2.0; 3.0], ...
                [0.1; 0.2; 0.3]);

            testCase.verifyEqual(C_I_RTN, eye(3));
            testCase.verifyEqual(rMeasured, [1.0; 2.0; 3.0]);
            testCase.verifyEqual(vMeasured, [0.1; 0.2; 0.3]);
        end

    end

    %% Focused gyro and event-driven receiver behavior
    methods (Test, TestTags = "Acquisition")
        function acquisitionGyroCapturesAndHoldsSamplesAtConfiguredRates(testCase)
            for period = [0.1, 0.2]
                out = simulateGyroAcquisition(period, 1);
                verifyGyroReportTiming(testCase, out, period);
                % Input rate is [t; 2*t; 3*t] rad/s. With errors disabled,
                % both the rate and its timestamp must describe the same instant.
                expectedRate = out.receiveTime.Data(:) * [1 2 3];
                testCase.verifyEqual(reshape(out.rate.Data, 3, []).', expectedRate, ...
                    'AbsTol', 1e-8);
                testCase.verifyEqual(out.validity.Data, ...
                    ones(size(out.validity.Data)));
            end
        end

        function acquisitionGyroCountsInvalidReports(testCase)
            out = simulateGyroAcquisition(0.1, 2);
            verifyGyroReportTiming(testCase, out, 0.1);
            testCase.verifyEqual(out.validity.Data, ...
                zeros(size(out.validity.Data)));
            testCase.verifyEqual(out.rate.Data, zeros(size(out.rate.Data)));
        end

        function acquisitionDriverWaitsForEventsAndCountsIdenticalPayloads(testCase)
            out = simulateDriverReceipts(false);
            t = out.receipt.Time(:);
            received = t >= 0.1-1e-10;
            expectedTime = zeros(size(t));
            expectedTime(received) = 0.1 + floor((t(received)-0.1+1e-10)/0.2)*0.2;
            expectedSeq = zeros(size(t), 'uint32');
            expectedSeq(received) = uint32(1+floor((t(received)-0.1+1e-10)/0.2));
            testCase.verifyEqual(out.receipt.Data(:), expectedTime, 'AbsTol', 1e-12);
            testCase.verifyEqual(out.sequence.Data(:), expectedSeq);
            testCase.verifyEqual(out.validity.Data(:), double(received));
            testCase.verifyEqual(t, (0:0.025:0.45)', 'AbsTol', 1e-12);
        end

        function acquisitionDriverWrapsSequenceCounter(testCase)
            out = simulateDriverReceipts(true);
            t = out.sequence.Time(:);
            expected = zeros(size(t), 'uint32');
            expected(t >= 0.1-1e-10 & t < 0.3-1e-10) = intmax('uint32');
            testCase.verifyEqual(out.sequence.Data(:), expected);
            testCase.verifyEqual(out.validity.Data(end), 1);
            testCase.verifyEqual(out.receipt.Data(end), 0.3, 'AbsTol', 1e-12);
        end

    end

    %% Complete Sensors to Drivers chain, independent rates and outages
    methods (Test, TestTags = "Integration")
        function integrationSensorsCaptureAndHoldReportsAtIndependentRates(testCase)
            periods = [0.1 0.2 0.25 0.5];
            out = simulateSensorDriverChain(periods, 'nominal', 1.45);
            verifySensorReportTimingAndHold(testCase, out, periods);
            gyroTime = out.Gyro_receive_time_s.Data(:);
            testCase.verifyEqual(signalRows(out.Gyro_omega_rad_s, 3), ...
                gyroTime * [0.1 0.2 0.3], 'AbsTol', 1e-8);
            magTime = out.Magnetometer_receive_time_s.Data(:);
            testCase.verifyEqual(signalRows(out.Magnetometer_B_B_T, 3), ...
                magTime * [1 2 3]*1e-6, 'AbsTol', 1e-12);
            cssTime = out.CoarseSunSensors_receive_time_s.Data(:);
            lit = cssTime < 0.15 | cssTime >= 1.15;
            testCase.verifyEqual(out.CoarseSunSensors_valid.Data(:), double(lit));
            sun = [ones(size(cssTime)), cssTime, zeros(size(cssTime))];
            sun = sun ./ vecnorm(sun, 2, 2);
            actual = signalRows(out.CoarseSunSensors_sun_B_unit, 3);
            testCase.verifyEqual(actual(lit,:), sun(lit,:), 'AbsTol', 1e-8);
            gnssTime = out.GNSS_receive_time_s.Data(:);
            testCase.verifyEqual(signalRows(out.GNSS_r_I_m, 3), ...
                [7e6+zeros(size(gnssTime)), 7500*gnssTime, zeros(size(gnssTime))], ...
                'AbsTol', 0.011);
            testCase.verifyEqual(signalRows(out.GNSS_v_I_m_s, 3), ...
                repmat([0 7500 0], numel(gnssTime), 1), 'AbsTol', 0.0011);
            for key = ["Gyro", "Magnetometer", "GNSS"]
                validity = out.get(key + "_valid");
                testCase.verifyEqual(validity.Data, ones(size(validity.Data)));
            end
        end

        function integrationSensorsPublishFailureReports(testCase)
            periods = [0.1 0.2 0.25 0.5];
            out = simulateSensorDriverChain(periods, 'failure', 1.45);
            verifySensorReportTimingAndHold(testCase, out, periods);
            for key = ["Gyro", "Magnetometer", "CoarseSunSensors", "GNSS"]
                validity = out.get(key + "_valid");
                testCase.verifyEqual(validity.Data, zeros(size(validity.Data)));
            end
        end

        function integrationGnssReportsAcquisitionAndDropout(testCase)
            periods = [0.1 0.1 0.1 1];
            out = simulateSensorDriverChain(periods, 'acquisition', 31.1);
            verifySensorReportTimingAndHold(testCase, out, periods);
            receipt = out.GNSS_receive_time_s.Data(:);
            testCase.verifyEqual(out.GNSS_valid.Data(:), double(receipt >= 30));
            dropped = simulateSensorDriverChain(periods, 'dropout', 2.1);
            verifySensorReportTimingAndHold(testCase, dropped, periods);
            testCase.verifyEqual(dropped.GNSS_valid.Data, zeros(size(dropped.GNSS_valid.Data)));
            testCase.verifyEqual(dropped.GNSS_r_I_m.Data, zeros(size(dropped.GNSS_r_I_m.Data)));
            testCase.verifyEqual(dropped.GNSS_v_I_m_s.Data, zeros(size(dropped.GNSS_v_I_m_s.Data)));
        end

    end

    methods (Access = private)
        function verifyBusInputName(testCase, block, inputIndex, expectedName)
            % Verifies the signal name attached to a bus-creator input.

            ports = get_param(block, "PortHandles");
            line = get_param(ports.Inport(inputIndex), "Line");

            testCase.verifyGreaterThan(line, 0);
            testCase.verifyEqual(string(get_param(line, "Name")), ...
                string(expectedName));
        end

        function verifyInputSource(testCase, block, inputIndex, expectedSource)
            % Verifies the source block connected to a subsystem input port.

            ports = get_param(block, "PortHandles");
            line = get_param(ports.Inport(inputIndex), "Line");
            sourcePort = get_param(line, "SrcPortHandle");
            source = getfullname(get_param(sourcePort, "ParentHandle"));

            testCase.verifyEqual(string(source), string(expectedSource));
        end

        function verifyNoDanglingLines(testCase, subsystem)
            % Verifies that every line in a subsystem hierarchy has a
            % valid source and at least one valid destination.

            lines = find_system(subsystem, "FindAll", "on", "Type", "line");
            for index = 1:numel(lines)
                source = get_param(lines(index), "SrcPortHandle");
                destinations = get_param(lines(index), "DstPortHandle");

                testCase.verifyNotEqual(source, -1, ...
                    "Found a line without a source in " + subsystem + ".");
                testCase.verifyFalse(any(destinations == -1), ...
                    "Found a line without a destination in " + subsystem + ".");
            end
        end

    end
end

%% Temporary configuration fixtures
function configFile = writeFailureGyroConfig(projectRoot)
% Writes a temporary config overriding only gyro failure-mode settings.

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
% Writes a temporary config overriding only magnetometer failure-mode settings.

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
% Writes a temporary config overriding only CSS failure-mode settings.

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
% Writes a temporary config overriding GNSS mode and failure outputs.

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
% Deletes a temporary file when it exists.

if isfile(fileName)
    delete(fileName);
end
end

%% Gyro acquisition harness
function verifyGyroReportTiming(testCase, out, period)
% Log at 40 Hz to observe both acquisition instants and intervening holds.
t = out.receiveTime.Time(:);
expected = floor((t + 1e-10) / period) * period;
testCase.verifyEqual(out.receiveTime.Data(:), expected, 'AbsTol', 1e-12);
testCase.verifyEqual(out.receiveTime.Data(1), 0);
testCase.verifyEqual(out.sequence.Data(:), uint32(round(expected / period) + 1));
testCase.verifyTrue(any(diff(out.receiveTime.Data(:)) == 0));
testCase.verifyTrue(any(diff(out.receiveTime.Data(:)) > 0));
testCase.verifyEqual(t, (0:0.025:0.45)', 'AbsTol', 1e-12);
end

function out = simulateGyroAcquisition(period, mode)
% Copy the production gyro into a transient model; do not modify the plant.
setupAocsSimulation;
load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
model = 'SensorsTestGyroHarness';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
plantCleanup = onCleanup(@() close_system('aocs_plant', 0));
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.025', 'StopTime', '0.45', ...
    'ReturnWorkspaceOutputs', 'on');
add_block('aocs_plant/Sensors/Gyro', [model '/Gyro']);
add_block('simulink/Sources/Clock', [model '/Clock']);
add_block('simulink/Math Operations/Gain', [model '/Rate'], ...
    'Gain', '[1;2;3]', 'Multiplication', 'Matrix(K*u)');
add_line(model, 'Clock/1', 'Rate/1');
add_block('simulink/Signal Routing/Bus Creator', [model '/Attitude'], ...
    'Inputs', '4', 'OutDataTypeStr', 'Bus: AttitudeStateBus');
values = {'zeros(3,1)', '[1;0;0;0]', 'eye(3)'};
names = {'euler_rad', 'q_be', 'DCM_be'};
for index = 1:3
    blockName = ['State' num2str(index)];
    add_block('simulink/Sources/Constant', [model '/' blockName], ...
        'Value', values{index});
    line = add_line(model, [blockName '/1'], ['Attitude/' num2str(index)]);
    set_param(line, 'Name', names{index});
end
line = add_line(model, 'Rate/1', 'Attitude/4');
set_param(line, 'Name', 'omega_b');
add_line(model, 'Attitude/1', 'Gyro/1');
add_block('simulink/Signal Routing/Bus Selector', [model '/Select'], ...
    'OutputSignals', 'receive_time_s,omega_rad_s,valid,sequence_id');
add_block('aocs_plant/Drivers/Gyro Driver', [model '/Driver']);
add_line(model, 'Gyro/1', 'Driver/1');
add_line(model, 'Gyro/2', 'Driver/2');
add_line(model, 'Driver/1', 'Select/1');
outputs = {'receiveTime', 'rate', 'validity', 'sequence'};
for index = 1:4
    % Explicit fast observer: To Workspace alone logs the source sample hits.
    observer = ['Observe' num2str(index)];
    add_block('simulink/Discrete/Zero-Order Hold', [model '/' observer], ...
        'SampleTime', '0.025');
    add_block('simulink/Sinks/To Workspace', [model '/' outputs{index}], ...
        'VariableName', outputs{index}, 'SaveFormat', 'Timeseries', ...
        'SampleTime', '0.025');
    add_line(model, ['Select/' num2str(index)], [observer '/1']);
    add_line(model, [observer '/1'], [outputs{index} '/1']);
end

config = evalin('base', 'AOCS_SensorConfig');
config.Value.Gyro.gyro_sample_time_s = period;
config.Value.Gyro.mode_id = mode;
config.Value.Gyro.enabled = 1;
config.Value.Gyro.bias_initial_rad_s = zeros(3,1);
config.Value.Gyro.bias_random_walk_step_std_rad_s = 0;
config.Value.Gyro.noise_std_rad_s = 0;
config.Value.Gyro.resolution_rad_s = 1e-9;
config.Value.Gyro.failure_valid = 0;
config.Value.Gyro.failure_output_rad_s = zeros(3,1);
simInput = Simulink.SimulationInput(model);
simInput = simInput.setVariable('AOCS_SensorConfig', config);
out = sim(simInput);
end

function specs = sensorDriverSpecs()
names = ["Gyro", "Magnetometer", "Coarse Sun Sensors", "GNSS"];
keys = ["Gyro", "Magnetometer", "CoarseSunSensors", "GNSS"];
rawBuses = ["GyroMeasurementBus", "MagnetometerMeasurementBus", ...
    "CoarseSunSensorMeasurementBus", "GnssMeasurementBus"];
reportBuses = ["GyroReportBus", "MagnetometerReportBus", ...
    "CoarseSunSensorReportBus", "GnssReportBus"];

specs = repmat(struct( ...
    "Name", "", ...
    "Key", "", ...
    "RawBus", "", ...
    "ReportBus", "", ...
    "PeriodField", "", ...
    "Period", ""), 1, numel(names));

for index = 1:numel(specs)
    specs(index).Name = names(index);
    specs(index).Key = keys(index);
    specs(index).RawBus = rawBuses(index);
    specs(index).ReportBus = reportBuses(index);
    if names(index) == "Gyro"
        specs(index).PeriodField = "gyro_sample_time_s";
    else
        specs(index).PeriodField = "sample_time_s";
    end
    specs(index).Period = "AOCS_SensorConfig." + keys(index) + "." + ...
        specs(index).PeriodField;
end
end

%% Driver receipt harness
function out = simulateDriverReceipts(wrap)
setupAocsSimulation;
load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
model = 'SensorsTestDriverHarness';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
plantCleanup = onCleanup(@() close_system('aocs_plant', 0));
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.025', 'StopTime', '0.45', 'ReturnWorkspaceOutputs', 'on');
add_block('aocs_plant/Drivers/Gyro Driver', [model '/Driver']);
if wrap
    set_param([model '/Driver/Receive Sample/Last Sequence'], ...
        'InitialCondition', 'uint32(4294967294)');
end
add_block('simulink/Sources/Constant', [model '/Measurement'], ...
    'Value', 'struct(''omega_rad_s'',[1;2;3],''valid'',1)', ...
    'OutDataTypeStr', 'Bus: GyroMeasurementBus');
add_block('simulink/Ports & Subsystems/Function-Call Generator', [model '/Receipt Events'], ...
    'sample_time', '[0.2 0.1]');
add_line(model, 'Measurement/1', 'Driver/1');
add_line(model, 'Receipt Events/1', 'Driver/2');
add_block('simulink/Signal Routing/Bus Selector', [model '/Select'], ...
    'OutputSignals', 'receive_time_s,sequence_id,valid');
add_line(model, 'Driver/1', 'Select/1');
names = {'receipt', 'sequence', 'validity'};
for index = 1:3
    observer = ['Observe' num2str(index)];
    add_block('simulink/Discrete/Zero-Order Hold', [model '/' observer], 'SampleTime', '0.025');
    add_block('simulink/Sinks/To Workspace', [model '/' names{index}], ...
        'VariableName', names{index}, 'SaveFormat', 'Timeseries');
    add_line(model, ['Select/' num2str(index)], [observer '/1']);
    add_line(model, [observer '/1'], [names{index} '/1']);
end
out = sim(model);
end

%% Integrated sensor chain harness
function verifySensorReportTimingAndHold(testCase, out, periods)
specs = sensorDriverSpecs();
for index = 1:4
    key = specs(index).Key;
    receipt = out.get(key + "_receive_time_s");
    t = receipt.Time(:);
    expected = floor((t+1e-10)/periods(index))*periods(index);
    testCase.verifyEqual(receipt.Data(:), expected, 'AbsTol', 1e-12);
    sequence = out.get(key + "_sequence_id");
    testCase.verifyEqual(sequence.Data(:), uint32(round(expected/periods(index))+1));
    % Every payload field must remain unchanged while the report ID is held.
    held = diff(sequence.Data(:)) == 0;
    reportBus = evalin('base', specs(index).ReportBus);
    for field = string({reportBus.Elements.Name})
        value = out.get(key + "_" + field);
        data = reshape(value.Data, [], numel(t)).';
        if isvector(value.Data)
            data = value.Data(:);
        end
        delta = diff(double(data), 1, 1);
        testCase.verifyEqual(delta(held,:), zeros(sum(held), size(delta,2)), ...
            'AbsTol', 1e-12, 'Payload changed without a receipt: ' + key + '.' + field);
    end
end
end

function data = signalRows(series, width)
data = reshape(series.Data, width, []).';
end

function out = simulateSensorDriverChain(periods, mode, stopTime)
setupAocsSimulation;
load_system(fullfile(projectRoot(), 'models', 'aocs_plant.slx'));
model = 'SensorsTestChainHarness';
new_system(model);
cleanup = onCleanup(@() close_system(model, 0));
plantCleanup = onCleanup(@() close_system('aocs_plant', 0));
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.025', 'StopTime', num2str(stopTime), 'ReturnWorkspaceOutputs', 'on');
add_block('aocs_plant/Sensors', [model '/Sensors']);
add_block('aocs_plant/Drivers', [model '/Drivers']);
for port = 1:5
    add_line(model, ['Sensors/' num2str(port)], ['Drivers/' num2str(port)]);
end
state = Simulink.Bus.createMATLABStruct('PlantStateBus');
state.AttitudeState.q_be = [1;0;0;0];
state.AttitudeState.DCM_be = eye(3);
state.OrbitState.r_I_m = [7e6;0;0];
state.OrbitState.v_I_m_s = [0;7500;0];
state.Environment.sun_B_unit = [1;0;0];
state.Environment.sun_visibility = 1;
state.Environment.solar_flux_shadowed_W_m2 = 1000;
workspace = get_param(model, 'ModelWorkspace');
assignin(workspace, 'fixtureState', state);
add_block('simulink/Sources/Constant', [model '/Truth'], ...
    'Value', 'fixtureState', 'OutDataTypeStr', 'Bus: PlantStateBus');
add_block('simulink/Sources/Clock', [model '/Clock']);
add_block('simulink/Signal Routing/Bus Assignment', [model '/Changing Truth'], ...
    'AssignedSignals', 'AttitudeState.omega_b,Environment.B_B_T,Environment.sun_B_unit,OrbitState.r_I_m,Environment.sun_visibility,Environment.solar_flux_shadowed_W_m2');
add_line(model, 'Truth/1', 'Changing Truth/1');
gains = {'[0.1;0.2;0.3]', '[1;2;3]*1e-6', '[0;1;0]', '[0;7500;0]'};
offsets = {'[0;0;0]', '[0;0;0]', '[1;0;0]', '[7e6;0;0]'};
for index = 1:4
    name = ['Ramp' num2str(index)];
    add_block('simulink/Math Operations/Gain', [model '/' name], ...
        'Gain', gains{index}, 'Multiplication', 'Matrix(K*u)');
    add_block('simulink/Sources/Constant', [model '/' name ' Offset'], 'Value', offsets{index});
    add_block('simulink/Math Operations/Add', [model '/' name ' Sum'], 'Inputs', '++');
    add_line(model, 'Clock/1', [name '/1']);
    add_line(model, [name '/1'], [name ' Sum/1']);
    add_line(model, [name ' Offset/1'], [name ' Sum/2']);
    add_line(model, [name ' Sum/1'], ['Changing Truth/' num2str(index+1)]);
end
add_block('simulink/Sources/Step', [model '/Eclipse Entry'], ...
    'Time', '0.15', 'Before', '1', 'After', '0', 'SampleTime', '0');
add_block('simulink/Sources/Step', [model '/Eclipse Exit'], ...
    'Time', '1.15', 'Before', '0', 'After', '1', 'SampleTime', '0');
add_block('simulink/Math Operations/Add', [model '/Visibility'], 'Inputs', '++');
add_line(model, 'Eclipse Entry/1', 'Visibility/1');
add_line(model, 'Eclipse Exit/1', 'Visibility/2');
add_line(model, 'Visibility/1', 'Changing Truth/6');
add_block('simulink/Math Operations/Gain', [model '/Flux'], 'Gain', '1000');
add_line(model, 'Visibility/1', 'Flux/1');
add_line(model, 'Flux/1', 'Changing Truth/7');
add_line(model, 'Changing Truth/1', 'Sensors/1');
specs = sensorDriverSpecs();
fields = strings(0);
for index = 1:4
    reportBus = evalin('base', specs(index).ReportBus);
    fields = [fields, specs(index).Key + '.' + string({reportBus.Elements.Name})]; %#ok<AGROW>
end
add_block('simulink/Signal Routing/Bus Selector', [model '/Select'], ...
    'OutputSignals', strjoin(fields, ','));
add_line(model, 'Drivers/1', 'Select/1');
for index = 1:numel(fields)
    name = replace(fields(index), '.', '_');
    observer = "Observer" + index;
    add_block('simulink/Discrete/Zero-Order Hold', model + "/" + observer, 'SampleTime', '0.025');
    add_block('simulink/Sinks/To Workspace', model + "/" + name, ...
        'VariableName', name, 'SaveFormat', 'Timeseries');
    add_line(model, "Select/"+index, observer+'/1');
    add_line(model, observer+'/1', name+'/1');
end
config = evalin('base', 'AOCS_SensorConfig');
for index = 1:4
    key = specs(index).Key;
    config.Value.(key).(specs(index).PeriodField) = periods(index);
    config.Value.(key).mode_id = 1;
    config.Value.(key).failure_valid = 0;
end
config.Value.Gyro.noise_std_rad_s = 0;
config.Value.Gyro.bias_initial_rad_s = zeros(3,1);
config.Value.Gyro.bias_random_walk_step_std_rad_s = 0;
config.Value.Gyro.resolution_rad_s = 1e-9;
config.Value.Magnetometer.noise_std_T = 0;
config.Value.Magnetometer.bias_initial_T = zeros(3,1);
config.Value.Magnetometer.bias_random_walk_step_std_T = 0;
config.Value.Magnetometer.resolution_T = 1e-12;
config.Value.CoarseSunSensors.noise_std_W_m2 = 0;
config.Value.CoarseSunSensors.resolution_W_m2 = 0;
config.Value.GNSS.acquisition_time_s = 0;
config.Value.GNSS.radial_ionosphere_bias_m = 0;
for field = ["position_periodic_amplitude_RTN_m", "velocity_periodic_amplitude_RTN_m_s", ...
        "position_gauss_markov_step_std_RTN_m", "velocity_gauss_markov_step_std_RTN_m_s", ...
        "position_white_noise_std_RTN_m", "velocity_white_noise_std_RTN_m_s"]
    config.Value.GNSS.(field) = zeros(3,1);
end
switch mode
    case 'failure'
        for index = 1:3
            config.Value.(specs(index).Key).mode_id = 2;
        end
        config.Value.GNSS.mode_id = 3;
    case 'acquisition'
        config.Value.GNSS.acquisition_time_s = 30;
    case 'dropout'
        config.Value.GNSS.mode_id = 2;
end
simInput = Simulink.SimulationInput(model);
simInput = simInput.setVariable('AOCS_SensorConfig', config);
out = sim(simInput);
end
