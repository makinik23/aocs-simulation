classdef MagnetometerModelTest < matlab.unittest.TestCase

    properties
        ProjectRoot
        ModelName = "aocs_plant"
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
            setupAocsSimulation(fullfile(testCase.ProjectRoot, ...
                "config", "AocsSimulationConfig.json"));
        end
    end

    methods (TestMethodTeardown)
        function closeModel(testCase)
            % Description:
            %   Closes the AOCS plant model after each test without saving.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

            closeLoadedSystem(testCase.ModelName);
        end
    end

    methods (Test)
        function magnetometerPublishesMeasurementBus(testCase)
            % Description:
            %   Verifies that the magnetometer subsystem consumes its sensor
            %   config and publishes a MagnetometerMeasurementBus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

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

        function sensorsPublishMeasurementBusToGnc(testCase)
            % Description:
            %   Verifies that the Sensors subsystem assembles gyro,
            %   magnetometer, coarse sun sensor, and GNSS measurements into
            %   SensorMeasurementBus and feeds the GNC subsystem.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

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
                testCase.ModelName + "/Sensors");
        end

        function coarseSunSensorsPublishMeasurementBus(testCase)
            % Description:
            %   Verifies that the CSS subsystem consumes its sensor config and
            %   publishes a CoarseSunSensorMeasurementBus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

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

        function gnssPublishesMeasurementBus(testCase)
            % Description:
            %   Verifies that GNSS consumes OrbitState, applies its configured
            %   sample rate and RTN PVT error model, and publishes a
            %   GnssMeasurementBus.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %
            % Outputs:
            %   None.

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
            testCase.verifyNumElements(sampleHolds, 2);
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
    end

    methods
        function verifyBusInputName(testCase, block, inputIndex, expectedName)
            % Description:
            %   Verifies the signal name attached to a bus-creator input.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   block - Bus Creator block path.
            %   inputIndex - Input port index to inspect.
            %   expectedName - Expected signal name.
            %
            % Outputs:
            %   None.

            ports = get_param(block, "PortHandles");
            line = get_param(ports.Inport(inputIndex), "Line");

            testCase.verifyGreaterThan(line, 0);
            testCase.verifyEqual(string(get_param(line, "Name")), ...
                string(expectedName));
        end

        function verifyInputSource(testCase, block, inputIndex, expectedSource)
            % Description:
            %   Verifies the source block connected to a subsystem input port.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   block - Destination block path.
            %   inputIndex - Input port index to inspect.
            %   expectedSource - Expected upstream block path.
            %
            % Outputs:
            %   None.

            ports = get_param(block, "PortHandles");
            line = get_param(ports.Inport(inputIndex), "Line");
            sourcePort = get_param(line, "SrcPortHandle");
            source = getfullname(get_param(sourcePort, "ParentHandle"));

            testCase.verifyEqual(string(source), string(expectedSource));
        end

        function verifyNoDanglingLines(testCase, subsystem)
            % Description:
            %   Verifies that every line in a subsystem hierarchy has a
            %   valid source and at least one valid destination.
            %
            % Arguments:
            %   testCase - matlab.unittest.TestCase instance.
            %   subsystem - Root subsystem path to inspect recursively.
            %
            % Outputs:
            %   None.

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
